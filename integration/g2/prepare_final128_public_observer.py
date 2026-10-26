"""Independent public-interface observer; prepare/check only until profile approved.

No extra py_poll or py_get_commit. No DLL is loaded by prepare/check. The future
entry executes the unchanged original setup/main with read-only getter samples.
"""
from pathlib import Path
from datetime import datetime
from functools import wraps
from types import SimpleNamespace
import argparse
import ast
import csv
import ctypes
import json
import math
import os
import runpy
import shutil
import sys

SOURCE = Path('F:/nav_integration_g2_20261023/nav3x5_N1c_final128_outer_height_stage1b_20261026')
FIELDS = ['applied', 'dx', 'dy', 'reserved0', 'dyaw', 'n_valid', 'cost0', 'cost_best', 'degen_x', 'degen_y', 'degen_yaw', 'n_applied']


def read(p):
    return json.loads(p.read_text(encoding='utf8'))


def save(p, v):
    p.write_text(json.dumps(v, ensure_ascii=False, indent=2) + '\n', encoding='utf8')


def meta(p):
    s = p.stat()
    return dict(path=str(p.resolve()), bytes=s.st_size, mtime_ns=s.st_mtime_ns)


class CallProxy:
    """Keep ctypes argtypes/restype assignments on the original C function."""
    def __init__(self, function, observer, name):
        object.__setattr__(self, '_function', function)
        object.__setattr__(self, '_observer', observer)
        object.__setattr__(self, '_name', name)

    def __getattr__(self, name):
        return getattr(self._function, name)

    def __setattr__(self, name, value):
        setattr(self._function, name, value)

    def __call__(self, *args, **kwargs):
        result = self._function(*args, **kwargs)
        if self._name == 'py_set_time':
            self._observer.set_time(args[0])
        elif self._name == 'py_poll':
            self._observer.after_poll()
        return result


class DLLProxy:
    def __init__(self, dll, observer):
        self._dll, self._observer, self._calls = dll, observer, {}

    def __getattr__(self, name):
        if name in ('py_set_time', 'py_poll'):
            if name not in self._calls:
                self._calls[name] = CallProxy(getattr(self._dll, name), self._observer, name)
            return self._calls[name]
        # Every other method, including consuming py_get_commit, is untouched.
        return getattr(self._dll, name)


class Observer:
    def __init__(self, out):
        out.mkdir(parents=True, exist_ok=False)
        self.out, self.now, self.clock_seq, self.poll_seq = out, None, 0, 0
        self.dispatch_seq, self.update_seq, self.last_counter = 0, 0, None
        self.pending_dispatches, self.dlls, self.restores = [], [], []
        self.events = (out / 'events.jsonl').open('w', encoding='utf8')
        self.polls = (out / 'poll_match.csv').open('w', encoding='utf8', newline='')
        self.writer = csv.writer(self.polls)
        self.writer.writerow(['poll', 'clock_sequence', 'C_now_us_uint32'] + FIELDS)
        self.changes = 0

    def emit(self, event, **kw):
        self.events.write(json.dumps(dict(event=event, C_now_us_uint32=self.now, clock_sequence=self.clock_seq,
                                         original_polls_returned=self.poll_seq, **kw), ensure_ascii=False) + '\n')

    def set_time(self, value):
        previous = self.now
        self.now = int(getattr(value, 'value', value)) & 0xffffffff
        self.clock_seq += 1
        self.emit('original_py_set_time_returned', previous_C_now_us=previous,
                  unsigned_delta_us=(self.now - previous) & 0xffffffff if previous is not None else None)

    def match_state(self):
        if not self.dlls:
            return None
        values = (ctypes.c_int32 * 12)()
        self.dlls[-1].py_get_match(values)  # A read-only getter, never polling/commit.
        row = list(values)
        row[11] &= 0xffffffff
        return row

    def after_poll(self):
        self.poll_seq += 1
        values = self.match_state()
        self.writer.writerow([self.poll_seq, self.clock_seq, self.now] + values)
        count = values[11]
        if self.last_counter is None:
            self.emit('initial_public_match_sample', match=dict(zip(FIELDS, values)))
        elif count != self.last_counter:
            self.changes += 1
            self.emit('accepted_solve_counter_changed', previous_n_applied=self.last_counter,
                      unsigned_increment=(count - self.last_counter) & 0xffffffff,
                      match=dict(zip(FIELDS, values)),
                      meaning='Counter transition observed after this original poll, not proof of full-rank XY or a nonzero correction')
        self.last_counter = count

    def install(self, R):
        factory = R.Firmware
        @wraps(factory)
        def firmware(*a, **k):
            value = factory(*a, **k)
            original = value.d
            self.dlls.append(original)
            value.d = DLLProxy(original, self)
            self.restores.append(lambda: setattr(value, 'd', original))
            self.emit('firmware_python_handle_observed', initialization_py_init_already_returned=True)
            return value
        R.Firmware = firmware
        self.restores.append(lambda: setattr(R, 'Firmware', factory))
        model = R.fpga_model.FpgaModel
        evaluate, update = model.match_eval, model.match_update
        @wraps(evaluate)
        def match_eval(instance, *a, **k):
            self.dispatch_seq += 1
            sequence = self.dispatch_seq
            self.pending_dispatches.append(sequence)
            smap = getattr(instance, 'smap', None)
            lf = getattr(smap, 'lf', None)
            self.emit('scoring_dispatch_callback_begin', scoring_sequence=sequence,
                      candidate_count=len(a[2]), candidate_rotation_center_mm=[int(a[0]), int(a[1])],
                      point_buffer_sample_count=len(instance.spts), ZREL=[int(instance.zlo), int(instance.zhi)],
                      prior=bool(getattr(smap, 'prior', False)), LF_origin=[int(v) for v in lf[:2]] if lf is not None else None,
                      n_struct=int(getattr(smap, 'n_struct', 0)))
            value = evaluate(instance, *a, **k)
            scores = [[int(n) for n in row] for row in value]
            self.emit('scoring_dispatch_callback_returned', scoring_sequence=sequence, scores=scores,
                      fields=['n_valid', 'cost', 'n_in', 'n_pts'],
                      meaning='HAL match_start calls this synchronously; C hal_match_result/solve occurs later')
            return value
        @wraps(update)
        def match_update(instance, *a, **k):
            self.update_seq += 1
            self.emit('HAL_match_update_callback_begin', update_sequence=self.update_seq,
                      preceding_dispatch_sequences=list(self.pending_dispatches), arguments=[int(n) for n in a],
                      public_match=self.match_state(),
                      meaning='May follow solve, alignment, stale/cancel or a no-start path; not an unconditional solver-invocation counter')
            value = update(instance, *a, **k)
            self.emit('HAL_match_update_callback_returned', update_sequence=self.update_seq)
            self.pending_dispatches.clear()
            return value
        model.match_eval, model.match_update = match_eval, match_update
        self.restores.append(lambda: setattr(model, 'match_eval', evaluate))
        self.restores.append(lambda: setattr(model, 'match_update', update))

    def close(self, original_output=None):
        self.emit('observer_closed', dispatch_callbacks=self.dispatch_seq, update_callbacks=self.update_seq,
                  poll_samples=self.poll_seq, accepted_counter_change_events=self.changes)
        self.events.close()
        self.polls.close()
        for restore in reversed(self.restores):
            restore()
        error_rows = 0
        if original_output is not None and (original_output / 'timeseries.csv').exists():
            with (original_output / 'timeseries.csv').open(encoding='utf8', newline='') as f:
                rows = list(csv.DictReader(f))
            with (self.out / 'error_curve.csv').open('w', encoding='utf8', newline='') as f:
                w = csv.writer(f)
                w.writerow(['original_t_s', 'truth_x_m', 'truth_y_m', 'estimated_x_m', 'estimated_y_m', 'XY_error_mm', 'original_runner_loc_valid'])
                for r in rows:
                    w.writerow([r['t'], r['x'], r['y'], r['ex'], r['ey'],
                                1000 * math.hypot(float(r['ex']) - float(r['x']), float(r['ey']) - float(r['y'])), r['loc_valid']])
            error_rows = len(rows)
        save(self.out / 'observation_summary.json', dict(original_poll_samples=self.poll_seq,
             dispatch_callbacks=self.dispatch_seq, update_callbacks=self.update_seq, error_curve_rows=error_rows,
             extra_py_poll=0, extra_py_get_commit=0, DLL_rebuilt=False, no_map_loss_verdict=None,
             confirmed=['Exact original simulated C time at each intercepted boundary', 'Every returned original poll has a read-only match sample',
                        'Scoring starts, all candidate score/valid counts, update callbacks, n_applied changes and last degen/delta',
                        'Error curve copied from the original timestamped truth/estimate trace'],
             not_confirmed=['Exact internal pm.n_runs or every failed solve', 'Internal stale/skip/at_edge/n_in_best reason',
                            'An update callback is not necessarily a solve', 'Full XY observability or no-map loss-free operation',
                            'C-instruction event times finer than the original1ms simulated poll clock'],
             timing='C_now_us is original uint32 simulated time. Raw values/clock sequence are logged; no invented age/loss threshold. Original timeseries retains its own t_s.'))


def pure_check(out):
    """Exercise only Python stand-ins, no DLL/model imports or numeric model."""
    out.mkdir(parents=True, exist_ok=False)
    class Function:
        def __init__(self, fn):
            self.fn, self.calls, self.argtypes, self.restype = fn, 0, None, None
        def __call__(self, *a, **k):
            self.calls += 1
            return self.fn(*a, **k)
    class FW:
        def __init__(self):
            self.state = [0] * 12
            def get(v):
                for i, x in enumerate(self.state): v[i] = x
            self.d = SimpleNamespace(py_set_time=Function(lambda t: ('time', t)), py_poll=Function(lambda: 'poll_return'),
                                     py_get_match=Function(get), py_get_commit=Function(lambda: 'commit_return'))
        def __getattr__(self, n): return getattr(self.d, n)
        def direct_handle_calls(self):
            self.d.py_set_time(ctypes.c_uint32(42))
            return self.d.py_poll()
    class Model:
        def __init__(self):
            self.spts, self.zlo, self.zhi = [1, 2], 60, 350
            self.smap = SimpleNamespace(prior=False, lf=None, n_struct=0)
            self.scores, self.updates = [[60, 10, 50, 2]], []
        def match_eval(self, *a): return self.scores
        def match_update(self, *a): self.updates.append(a); return 'update_return'
    R = SimpleNamespace(Firmware=FW, fpga_model=SimpleNamespace(FpgaModel=Model))
    observer = Observer(out / 'capture')
    observer.install(R)
    fw, model = R.Firmware(), R.fpga_model.FpgaModel()
    raw = observer.dlls[0]
    assert fw.py_set_time(1000) == ('time', 1000)
    assert fw.py_poll() == 'poll_return'
    assert fw.direct_handle_calls() == 'poll_return'
    fw.state[:] = [1, 0, 0, 0, 0, 60, 10, 8, 1, 1, 1, 1]
    assert fw.py_poll() == 'poll_return'
    cand = [[0, 0, 0]]
    assert model.match_eval(0, 0, cand) is model.scores
    assert model.match_update(0, 0, 0, 0, 0) == 'update_return'
    assert model.updates == [(0, 0, 0, 0, 0)]
    fw.d.py_set_time.argtypes = [ctypes.c_uint32]
    assert raw.py_set_time.argtypes == [ctypes.c_uint32]
    fw.d.py_poll.restype = None
    assert raw.py_poll.restype is None
    assert raw.py_poll.calls == observer.poll_seq == 3 and raw.py_set_time.calls == 2
    assert raw.py_get_commit.calls == 0 and fw.py_get_commit() == 'commit_return' and raw.py_get_commit.calls == 1
    original = out / 'original_output'
    original.mkdir()
    (original / 'timeseries.csv').write_text('t,x,y,ex,ey,loc_valid\n0.1,0,0,0.003,0.004,1\n', encoding='utf8')
    observer.close(original)
    assert R.Firmware is FW and isinstance(fw.d, SimpleNamespace)
    with (out / 'capture/poll_match.csv').open(encoding='utf8') as f: rows = list(csv.DictReader(f))
    assert [r['C_now_us_uint32'] for r in rows] == ['1000', '42', '42']
    events = [json.loads(s) for s in (out / 'capture/events.jsonl').read_text(encoding='utf8').splitlines()]
    assert len([e for e in events if e['event'] == 'accepted_solve_counter_changed']) == 1
    with (out / 'capture/error_curve.csv').open(encoding='utf8') as f: error = list(csv.DictReader(f))
    assert float(error[0]['XY_error_mm']) == 5.0
    result = dict(passed=True, real_DLL_loaded=False, model_or_closed_loop_started=False,
                  original_polls=3, observer_post_poll_samples=3, original_time_calls=2, extra_commit_reads=0,
                  return_values_and_score_object_identity_preserved=True, direct_d_handle_calls_observed=True,
                  ctypes_argtypes_restype_forwarded=True, original_update_arguments_preserved=True,
                  one_counter_change_with_zero_delta_and_degeneracy_logged=True, synthetic_error_curve_mm=5,
                  loss_threshold_defined=False)
    save(out / 'result.json', result)
    return result


def prepare(out, profile_path=None):
    assert not out.exists()
    out.mkdir(parents=True)
    profile = read(profile_path) if profile_path else dict(root_approved=False, source_entry=str(SOURCE / 'execution_tree/sim/closed_loop/stage1b_case.py'),
              argv=None, environment=None, selected_forward_mm=None, selected_ZHI=None, original_output=None)
    entry = Path(profile['source_entry']).resolve()
    source_root = entry.parents[3]
    assert entry.name in ('stage1b_case.py', 'final_forward_case.py')
    sources = list((source_root / 'execution_tree').rglob('*.py'))
    sources += [p for p in (source_root / 'firmware_source').rglob('*') if p.suffix in ('.c', '.h')]
    sources += [source_root / n for n in ('nav_fw.dll', 'outer_scene.json', 'scene.json', 'cases.json', 'manifest.json', 'bound_manifest.json')]
    sources = [p for p in sources if p.exists() and '__pycache__' not in p.parts]
    for p in sources:
        if p.suffix == '.py': ast.parse(p.read_text(encoding='utf8'), filename=str(p))
    # Source inspection only: no original entry imports/atexit/DLL load here.
    src = (entry.parent / 'run_cl.py').read_text(encoding='utf8')
    assert 'fw.py_poll()' in src and 'fw.py_get_commit(commit_v)' in src
    shutil.copy2(Path(__file__).resolve(), out / 'observer_entry.py')
    save(out / 'profile.json', profile)
    save(out / 'preparation.json', dict(source_entry=str(entry), bindings=[meta(p) for p in sources],
         entry=meta(out / 'observer_entry.py'), profile=meta(out / 'profile.json'),
         original_DLL_not_loaded=True, root_profile_pending=not profile.get('root_approved'),
         current_inputs_modified=False, no_extra_poll_or_commit=True))
    checked = pure_check(out / 'pure_wrapper_check')
    save(out / 'result.json', dict(prepared=True, pure_wrapper_check=checked,
         closed_loop_started=False, selected_forward_mm=profile.get('selected_forward_mm'), selected_ZHI=profile.get('selected_ZHI'),
         actual_observations_available=False, future_start_blocked=not profile.get('root_approved')))


def start(out):
    cfg, profile = read(out / 'preparation.json'), read(out / 'profile.json')
    assert meta(out / 'profile.json') == cfg['profile'] and meta(out / 'observer_entry.py') == cfg['entry']
    assert profile.get('root_approved') is True, 'Prepare a new frozen profile after root selection; do not edit this pending profile'
    assert profile['selected_forward_mm'] in (1000, 1250, 1500) and profile['selected_ZHI'] in (350, 1500)
    for row in cfg['bindings']: assert meta(Path(row['path'])) == row
    argv, env = profile['argv'], profile['environment']
    assert argv and argv[0] == profile['source_entry'] and '--no-build' in argv and '--firmware-dll' in argv
    entry = Path(profile['source_entry']).resolve()
    source_root = entry.parents[3]
    assert Path(argv[argv.index('--firmware-dll') + 1]).resolve() == (source_root / 'nav_fw.dll').resolve()
    if entry.name == 'stage1b_case.py':
        bound = source_root / 'bound_manifest.json'
        assert any(Path(row['path']).resolve() == bound.resolve() for row in cfg['bindings']), 'Selected-distance binding must exist when the approved profile freezes'
        assert read(bound)['selected_offset_mm'] == profile['selected_forward_mm']
    assert int(env['SLF_SN_LOG2']) == 7 and int(env['SLF_LF_N']) == 128
    applied_forward = 0 if '--prior' in argv else profile['selected_forward_mm']
    assert int(env['SLF_FORWARD_MM']) == applied_forward and int(env['SLF_ZHI']) == profile['selected_ZHI']
    target = Path(profile['original_output'])
    assert argv[argv.index('--out') + 1] == str(target) and not target.exists()
    assert not (out / 'run_claim.json').exists()
    save(out / 'run_claim.json', dict(pid=os.getpid(), started_at=datetime.now().astimezone().isoformat()))
    os.environ.update(env)
    sys.argv = argv
    sys.path.insert(0, str(Path(argv[0]).parent))
    globals_ = runpy.run_path(argv[0], run_name='observed_original_configuration')
    observer = Observer(out / 'capture')
    observer.install(globals_['R'])
    try:
        return globals_['R'].main()
    finally:
        observer.close(target)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--prepare', action='store_true')
    p.add_argument('--profile', type=Path)
    p.add_argument('--start', action='store_true')
    a = p.parse_args()
    assert a.prepare != a.start
    if a.prepare:
        prepare(a.out, a.profile)
        print(json.dumps(read(a.out / 'result.json'), ensure_ascii=False))
    else:
        raise SystemExit(start(a.out))
