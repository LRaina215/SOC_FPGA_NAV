"""Freeze16 observed stage1b jobs; start is separate and root-controlled.

No model/DLL/closed-loop execution in prepare. One serial child after both
original330s patrols finish and2/4GiB plus PNR growth are available.
"""
from pathlib import Path
from datetime import datetime
import argparse
import importlib.util
import json
import math
import shutil
import subprocess
import sys
import time
import traceback
import psutil

BASE = Path('F:/nav_integration_g2_20261023')
SOURCE = BASE / 'nav3x5_N1c_final128_outer_height_stage1b_20261026'
PATROL = BASE / 'nav3x5_N1c_final128_long_patrol_20261026'
FIRST = BASE / 'stage1b_new_object_first_hit_probe1/result.json'
PYTHON = Path('D:/Anaconda/python.exe')


def read(p): return json.loads(p.read_text(encoding='utf8'))


def save(p, v):
    tmp = p.with_suffix(p.suffix + '.tmp')
    tmp.write_text(json.dumps(v, ensure_ascii=False, indent=2) + '\n', encoding='utf8')
    tmp.replace(p)


def meta(p):
    s = p.stat()
    return dict(path=str(p.resolve()), bytes=s.st_size, mtime_ns=s.st_mtime_ns)


def module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def prepare(out):
    assert not out.exists()
    bound = read(SOURCE / 'bound_manifest.json')
    assert bound['selected_offset_mm'] == 1500 and len(bound['jobs']) == 16
    first = read(FIRST)
    assert first['passed'] and first['poses'] == 11
    out.mkdir(parents=True)
    source = out / 'source'
    source.mkdir()
    copied = []
    for folder in ('execution_tree', 'firmware_source'):
        shutil.copytree(SOURCE / folder, source / folder, ignore=shutil.ignore_patterns('__pycache__'))
    for name in ('nav_fw.dll', 'outer_scene.json', 'scene.json', 'cases.json', 'manifest.json', 'bound_manifest.json', 'selected_best_distance_frozen.json', 'final_task.txt'):
        if (SOURCE / name).exists(): shutil.copy2(SOURCE / name, source / name)
    for p in source.rglob('*'):
        if p.is_file():
            original = SOURCE / p.relative_to(source)
            assert p.read_bytes() == original.read_bytes()
            copied.append(dict(original=meta(original), frozen=meta(p), bytes_equal=True))
    generator = Path(__file__).parent / 'prepare_final128_public_observer_v2.py'
    shutil.copy2(generator, out / 'observer_generator_v2.py')
    shutil.copy2(Path(__file__).parent / 'wait_stage1b_first_hit_probe.py', out / 'memory_guard.py')
    shutil.copy2(Path(__file__).resolve(), out / 'controller.py')
    observer = module(out / 'observer_generator_v2.py', 'frozen_public_observer_v2')
    pure = observer.pure_check(out / 'pure_wrapper_check')
    jobs, validation = [], []
    policy = dict(P95_50mm_hard_gate=False, lower_accuracy_accepted=True,
                  actual_error_still_reported=True, collisions_waived=False, localization_failure_waived=False,
                  long_drift_waived=False, stop_contract_waived=False, no_new_loss_threshold=True)
    for j in bound['jobs']:
        directory = out / 'profiles' / j['name']
        original_out = out / 'results' / j['name']
        directory.parent.mkdir(exist_ok=True)
        argv = list(j['command'][j['command'].index('--scene') - 1:])
        argv[0] = str(source / 'execution_tree/sim/closed_loop/stage1b_case.py')
        argv[argv.index('--firmware-dll') + 1] = str(source / 'nav_fw.dll')
        argv[argv.index('--out') + 1] = str(original_out)
        env = dict(j['environment'])
        sn, prior, upper = j['SLF_SN'], j['prior'], j['ZHI']
        expected_d = 1500 if sn == 128 and not prior else 0
        assert int(env['SLF_FORWARD_MM']) == expected_d
        assert int(env['SLF_ZHI']) == upper and int(env['SLF_SN_LOG2']) == sn.bit_length() - 1
        assert env['SLF_WINDOW_MODE'] == ('forward' if sn == 128 and not prior else 'rolling')
        profile = dict(root_approved=True, profile_requested_by_root_but_not_started=True,
                       source_entry=argv[0], argv=argv, environment=env,
                       selected_forward_mm=1500, applied_forward_mm=expected_d, selected_ZHI=upper,
                       prior=prior, reference_only=sn == 256, production128_fixed=True,
                       original_output=str(original_out), latest_user_policy=policy)
        profile_input = out / f'profile_input_{j["name"]}.json'
        save(profile_input, profile)
        observer.prepare(directory, profile_input, run_check=False)
        observer.validate_profile(directory)  # Only metadata/arguments; no imports of the model or DLL.
        duration = float(argv[argv.index('--time') + 1])
        command = [str(PYTHON), '-B', '-X', 'utf8', str(directory / 'observer_entry.py'), '--start', '--out', str(directory)]
        row = dict(name=j['name'], case=j['case'], prior=prior, SLF_SN=sn, ZHI=upper,
                   selected_best_d=1500, applied_forward_mm=expected_d,
                   command=command, argv=argv, environment=env, profile=str(directory / 'profile.json'),
                   preparation=str(directory / 'preparation.json'), original_out=str(original_out),
                   capture=str(directory / 'capture'), maximum_source_time_s=duration + 2.1,
                   maximum_original_poll_samples_by_source=2 * math.ceil((duration + 2.1) * 1000) + 1)
        jobs.append(row)
        validation.append(dict(name=j['name'], passed=True, no_model_or_DLL_imported=True, reference_only=sn == 256,
                               prior_fixed=prior, applied_d=expected_d, ZHI=upper))
    zero = [r['pose']['name'] for r in first['rows'] if all(b['selected_before_PB_cap'] == 0 for b in r['bands'])]
    visibility = dict(source=meta(FIRST), actual_first_hit_totals=first['actual_first_hit_totals_by_outer_object'],
                      zero_hit_objects=first['outer_objects_with_zero_hits'], zero_structure_poses_both_bands=zero,
                      all_objects_visible=False, no_new_probe_run=True,
                      limits='11 static zero-noise poses, warm gate3-repeat fixture only; no real route/CPU/localization proof')
    save(out / 'first_hit_readback.json', visibility)
    save(out / 'profile_validation.json', dict(passed=True, checked=16, rows=validation, pure_wrapper_check=pure,
                                             static_only=True, real_DLL_loaded=False, closed_loop_started=False))
    callbacks = dict(time='Original py_set_time return, exactuint32 and sequence',
                     poll='Read-only py_get_match after every returned original py_poll',
                     score='Original FpgaModel.match_eval begin/result, all candidate scores/valid/point counts',
                     update='Original FpgaModel.match_update begin/return, not asserted to be a C solver call',
                     error='Original timeseries truth/estimate/timestamps copied into error_curve.csv',
                     no_extra_poll=True, no_extra_consuming_commit=True, numeric_array_identity_preserved=True)
    cfg = dict(prepared_at=datetime.now().astimezone().isoformat(), source=str(SOURCE), source_copy=str(source),
               selected_d=1500, planned=16, one_worker=True, automatic_EDA=False, jobs=jobs,
               copied_inputs=copied, callback_contract=callbacks, user_policy=policy,
               PNR_peak_guard='base2/4GiB + sum remaining3.5GiB peak growth for53/54, including a pending unallocated tool',
               bounded_observation='Stream each poll row and at most one <=256-candidate score array; no whole-run event list in memory. Count bound derives from original1ms loop plus2s terminal observation/100ms sample margin; no new solver-loss threshold or forced algorithm timeout.',
               first_hit_review_required_before_start=True, patrol_root=str(PATROL),
               first_hit_binding=meta(FIRST), controller_binding=meta(out / 'controller.py'),
               generator_binding=meta(out / 'observer_generator_v2.py'), memory_guard_binding=meta(out / 'memory_guard.py'),
               original_bound_manifest=meta(SOURCE / 'bound_manifest.json'),
               cannot_confirm=['Internal n_runs/all rejects/at_edge/n_in_best', 'Every update callback is a solve',
                               'No-map lock from constant loc_valid', 'No-map no-loss or sustained-drift acceptance without defined numeric rules'])
    save(out / 'preparation.json', cfg)
    save(out / 'job.json', dict(phase='prepared_not_started', planned=16, completed=0, one_worker=True,
                               root_start_requires_first_hit_review=True, actual_child_started=False))
    return cfg


def patrol_ready():
    state = read(PATROL / 'job.json')
    if state.get('phase') == 'failed': raise RuntimeError('Original patrol failed; no new observed batch starts')
    if state.get('phase') != 'complete' or state.get('completed') != 2: return False, state
    summary = read(PATROL / 'summary.json')
    assert summary['completed'] == 2 and len(summary['results']) == 2
    evidence = []
    for r in summary['results']:
        assert r['actual_simulated_duration_s'] >= 300
        runtime_path = PATROL / 'results' / r['name'] / 'runtime.json'
        runtime = read(runtime_path)
        assert runtime['exit'] == 0
        try:
            p = psutil.Process(runtime['child_pid'])
            if abs(p.create_time() - runtime['child_creation']) < .01: return False, state
        except psutil.NoSuchProcess: pass
        evidence.append(dict(name=r['name'], runtime=meta(runtime_path), duration_s=r['actual_simulated_duration_s']))
    return True, evidence


def start(out, first_hit_reviewed):
    assert first_hit_reviewed, 'Root must review actual first-hit zeros/limits before explicit start'
    cfg = read(out / 'preparation.json')
    for key in ('first_hit_binding', 'controller_binding', 'generator_binding', 'memory_guard_binding', 'original_bound_manifest'):
        assert meta(Path(cfg[key]['path'])) == cfg[key]
    for row in cfg['copied_inputs']: assert meta(Path(row['frozen']['path'])) == row['frozen']
    guard = module(out / 'memory_guard.py', 'frozen_stage1b_memory_guard')
    me = psutil.Process()
    me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    me.cpu_affinity([i for i in (18, 19) if i in me.cpu_affinity()])
    identity = dict(controller_pid=me.pid, controller_created_epoch=me.create_time(), parent_pid=me.ppid())
    with (out / 'launch_claim.json').open('x', encoding='utf8') as f: json.dump(identity, f)
    completed = []
    def status(phase, **kw):
        save(out / 'job.json', dict(identity, phase=phase, updated_at=datetime.now().astimezone().isoformat(), planned=16,
                                   completed=len(completed), one_worker=True, P95_50mm_hard_gate=False, **kw))
    try:
        while True:
            ready, evidence = patrol_ready()
            if ready: break
            status('waiting_original_two_patrols_natural_completion', patrol=evidence)
            time.sleep(17)
        save(out / 'original_patrols_finished.json', dict(passed=True, evidence=evidence))
        for job in cfg['jobs']:
            profile_dir = Path(job['profile']).parent
            assert not (profile_dir / 'run_claim.json').exists() and not Path(job['original_out']).exists(), 'No retries/overwrite'
            while True:
                mem = guard.memory()
                pg, cg, pnrs = guard.pnr_growth()
                required_p, required_c = 2 * guard.GIB + pg, 4 * guard.GIB + cg
                status('waiting_memory_and_PNR_growth', next=job['name'], memory=mem, PNRs=pnrs,
                       required_physical=required_p, required_commit=required_c)
                if mem['physical'] >= required_p and mem['commit'] >= required_c: break
                time.sleep(17)
            with (profile_dir / 'stdout.log').open('w', encoding='utf8') as log:
                child = subprocess.Popen(job['command'], cwd=out, stdout=log, stderr=subprocess.STDOUT,
                                         creationflags=subprocess.CREATE_NO_WINDOW)
                p = psutil.Process(child.pid)
                created = p.create_time()
                peak = 0
                low_since = None
                status('one_observed_case_running', current=job['name'], child_pid=child.pid, child_created_epoch=created,
                       command=job['command'], environment=job['environment'], admission=mem)
                while child.poll() is None:
                    try: peak = max(peak, p.memory_info().rss)
                    except psutil.NoSuchProcess: pass
                    mem_now = guard.memory()
                    low = mem_now['physical'] < 256 * 1024**2 and mem_now['commit'] < guard.GIB
                    low_since = time.monotonic() if low and low_since is None else low_since if low else None
                    if low_since is not None and time.monotonic() - low_since >= 30:
                        child.terminate(); child.wait()
                        raise RuntimeError('Only this observed worker stopped by sustained low memory; no retry')
                    time.sleep(5)
            save(profile_dir / 'runtime.json', dict(exit=child.returncode, child_pid=child.pid, created_epoch=created,
                                                   peak_RSS=peak, no_retry=True))
            assert child.returncode == 0
            metric = read(Path(job['original_out']) / 'metrics.json')
            observed = read(profile_dir / 'capture/observation_summary.json')
            row = dict(name=job['name'], metric=metric, observed=observed, P95_50mm_hard_gate=False,
                       no_map_loss_proved=None if not job['prior'] else 'Use original prior loc metrics; not generic no-loss acceptance',
                       automatic_acceptance=False)
            completed.append(row)
            save(out / 'summary.json', dict(completed=len(completed), planned=16, rows=completed, user_policy=cfg['user_policy'],
                                          automatic_safety_or_deployment_acceptance=False))
        status('all16_observed_runs_finished_requires_review', result=str(out / 'summary.json'), automatic_acceptance=False)
    except Exception:
        status('failed_preserved_no_retry', error=traceback.format_exc())
        raise


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--prepare', action='store_true')
    p.add_argument('--start', action='store_true')
    p.add_argument('--first-hit-reviewed', action='store_true')
    a = p.parse_args()
    assert a.prepare != a.start
    if a.prepare:
        r = prepare(a.out)
        print(json.dumps(dict(prepared=str(a.out), jobs=len(r['jobs']), actual_children_started=False)))
    else:
        start(a.out, a.first_hit_reviewed)
