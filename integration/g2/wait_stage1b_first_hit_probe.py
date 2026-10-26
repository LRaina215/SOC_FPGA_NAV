"""Persistent memory queue for one frozen first-hit probe, no PNR operations.

The original --start is a one-shot admission check. Only a rejection before
run_claim may return to waiting; an actual start/failure is never retried.
"""
from pathlib import Path
from datetime import datetime
import argparse
import ctypes
import json
import shutil
import subprocess
import sys
import time
import traceback
import psutil

BASE = Path('F:/nav_integration_g2_20261023')
PROBE = BASE / 'stage1b_new_object_first_hit_probe1'
PYTHON = Path('D:/Anaconda/python.exe')
GIB = 1024 ** 3
PEAK = int(3.5 * GIB)


def read(p):
    return json.loads(p.read_text(encoding='utf8'))


def save(p, value):
    temp = p.with_suffix(p.suffix + '.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf8')
    temp.replace(p)


def meta(p):
    s = p.stat()
    return dict(path=str(p.resolve()), bytes=s.st_size, mtime_ns=s.st_mtime_ns)


def memory():
    class M(ctypes.Structure):
        _fields_ = [('length', ctypes.c_uint32), ('load', ctypes.c_uint32)] + [(n, ctypes.c_uint64) for n in
                     ('total_phys', 'avail_phys', 'total_page', 'avail_page', 'total_virtual', 'avail_virtual', 'avail_extended')]
    m = M()
    m.length = ctypes.sizeof(m)
    assert ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(m))
    return dict(physical=m.avail_phys, commit=m.avail_page)


def alive(pid, created):
    try:
        return abs(psutil.Process(pid).create_time() - created) < .01
    except (psutil.NoSuchProcess, psutil.AccessDenied):
        return False


def pnr_growth():
    """Same per-tool3.5GiB residual as other_peak_growth, summed for53/54.

    A live waiting controller has not allocated its tool yet: reserve its full
    peak as well, avoiding an admission race with that independent controller.
    This never waits on a particular PID, pauses or terminates another job.
    """
    builds = {str((BASE / f'SLF128_storage_place{place}_pnr{52+place}').resolve()).lower(): place for place in (1, 2)}
    processes = {place: [] for place in (1, 2)}
    for p in psutil.process_iter(['pid', 'name']):
        try:
            if (p.info['name'] or '').lower() != 'gw_sh.exe':
                continue
            place = builds.get(str(Path(p.cwd()).resolve()).lower())
            if place is None:
                continue
            m = p.memory_info()
            processes[place].append(dict(pid=p.pid, created_epoch=p.create_time(), rss=m.rss,
                                         private=getattr(m, 'private', m.vms)))
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    rows = []
    for place in (1, 2):
        build = BASE / f'SLF128_storage_place{place}_pnr{52+place}'
        job_path = BASE / f'{build.name}_one_shot/job.json'
        job = read(job_path) if job_path.exists() else {}
        tools = processes[place]
        pending = (not tools and not (build / 'tool_runtime.json').exists()
                   and job.get('phase') in ('waiting_for_actual_memory_admission', 'starting_PNR_without_deadline')
                   and alive(job.get('controller_pid', -1), job.get('controller_created_epoch', -1)))
        pg = sum(max(0, PEAK - r['rss']) for r in tools) + (PEAK if pending else 0)
        cg = sum(max(0, PEAK - r['private']) for r in tools) + (PEAK if pending else 0)
        rows.append(dict(place=place, build=str(build), phase=job.get('phase'), tools=tools,
                         pending_unallocated_peak_reserved=bool(pending), physical_growth=pg, commit_growth=cg))
    return sum(r['physical_growth'] for r in rows), sum(r['commit_growth'] for r in rows), rows


def prepare(out):
    assert not out.exists(), 'No duplicate waiting controller directory'
    assert (PROBE / 'preparation.json').exists() and not (PROBE / 'run_claim.json').exists()
    cfg = read(PROBE / 'preparation.json')
    bindings = [meta(PROBE / n) for n in ('run_probe.py', 'poses.json', 'preparation.json')]
    bindings += [b['frozen'] for b in cfg['bindings']]
    out.mkdir(parents=True)
    shutil.copy2(Path(__file__).resolve(), out / 'controller.py')
    save(out / 'preparation.json', dict(probe=str(PROBE), bindings=bindings,
         controller=meta(out / 'controller.py'), poll_seconds=17,
         command=[str(PYTHON), '-B', '-X', 'utf8', str(PROBE / 'run_probe.py'), '--start', '--out', str(PROBE)],
         base_minimum_physical=2 * GIB, base_minimum_commit=4 * GIB, per_PNR_peak_estimate=PEAK,
         growth_reference='run_slf128_storage_place_pair.other_peak_growth',
         reserve_live_pending_PNR_full_peak=True, no_dependency_on_PNR_exit=True,
         actual_probe_max_starts=1, retry_only_explicit_no_claim_memory_rejection=True))
    save(out / 'job.json', dict(phase='prepared_controller_not_started', probe=str(PROBE)))


def start(out):
    cfg = read(out / 'preparation.json')
    assert meta(out / 'controller.py') == cfg['controller']
    for row in cfg['bindings']:
        assert meta(Path(row['path'])) == row
    me = psutil.Process()
    me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    if 18 in me.cpu_affinity():
        me.cpu_affinity([18])
    identity = dict(controller_pid=me.pid, controller_created_epoch=me.create_time(), parent_pid=me.ppid(),
                    executable=me.exe(), launch='Hidden WMI service-parent persistent controller', probe=str(PROBE))
    with (out / 'launch_claim.json').open('x', encoding='utf8') as f:
        json.dump(identity, f, indent=2)
    attempts = []
    def status(phase, **kw):
        save(out / 'job.json', dict(identity, phase=phase, updated_at=datetime.now().astimezone().isoformat(),
                                   command=cfg['command'], attempts=attempts, actual_probe_max_starts=1, **kw))
    try:
        while True:
            if (PROBE / 'run_claim.json').exists():
                status('existing_actual_probe_claim_no_duplicate_launch', probe_claim=read(PROBE / 'run_claim.json'))
                return 0
            for row in cfg['bindings']:
                assert meta(Path(row['path'])) == row
            mem = memory()
            pg, cg, pnrs = pnr_growth()
            required_p, required_c = 2 * GIB + pg, 4 * GIB + cg
            status('waiting_for_memory_and_both_PNR_peak_growth', memory=mem, PNRs=pnrs,
                   required_physical=required_p, required_commit=required_c)
            if mem['physical'] < required_p or mem['commit'] < required_c:
                time.sleep(cfg['poll_seconds'])
                continue
            attempt = out / f'attempt{len(attempts)+1:03d}'
            attempt.mkdir()
            assert not (PROBE / 'run_claim.json').exists()
            with (attempt / 'stdout.log').open('w', encoding='utf8') as log:
                p = subprocess.Popen(cfg['command'], cwd=str(PROBE), stdout=log, stderr=subprocess.STDOUT,
                                     creationflags=subprocess.CREATE_NO_WINDOW)
                try:
                    c = psutil.Process(p.pid)
                    created = c.create_time()
                except psutil.NoSuchProcess:
                    created = None
                row = dict(attempt=len(attempts)+1, pid=p.pid, created_epoch=created, admission=mem,
                           required_physical=required_p, required_commit=required_c, PNRs=pnrs,
                           stdout=str(attempt / 'stdout.log'))
                save(attempt / 'identity.json', row)
                attempts.append(row)
                status('original_probe_entry_admitted', child=row)
                while p.poll() is None:
                    claimed = (PROBE / 'run_claim.json').exists()
                    status('actual_probe_running_no_retry' if claimed else 'probe_admission_check_running',
                           child=row, actual_probe_claim=read(PROBE / 'run_claim.json') if claimed else None)
                    time.sleep(1)
                row['exit'] = p.returncode
                row['original_identity_exited'] = not alive(p.pid, created) if created is not None else p.poll() is not None
                save(attempt / 'runtime.json', row)
            if (PROBE / 'run_claim.json').exists():
                result = read(PROBE / 'result.json') if (PROBE / 'result.json').exists() else None
                passed = bool(p.returncode == 0 and result and result.get('passed'))
                status('finite_probe_complete_no_retry' if passed else 'actual_probe_failed_preserved_no_retry',
                       child=row, probe_claim=read(PROBE / 'run_claim.json'), result=str(PROBE / 'result.json'),
                       finite_probe_passed=passed, probe_job=read(PROBE / 'job.json'))
                return 0 if passed else 1
            probe_job = read(PROBE / 'job.json')
            assert p.returncode == 0 and probe_job.get('phase') == 'waiting_memory_not_started', (
                'No actual claim, but not a plain memory refusal; preserve failure without retry', row, probe_job)
            row['no_actual_claim_memory_rejection'] = True
            save(attempt / 'runtime.json', row)
            status('memory_race_rejected_before_actual_start_waiting_again', child=row, probe_job=probe_job)
            time.sleep(cfg['poll_seconds'])
    except Exception:
        status('controller_failed_preserved_no_retry', error=traceback.format_exc())
        raise


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--prepare', action='store_true')
    p.add_argument('--start', action='store_true')
    a = p.parse_args()
    assert a.prepare != a.start
    if a.prepare:
        prepare(a.out)
        print(json.dumps(dict(prepared=str(a.out))))
    else:
        raise SystemExit(start(a.out))
