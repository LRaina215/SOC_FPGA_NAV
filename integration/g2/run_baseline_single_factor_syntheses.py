"""Monitor the two frozen timing SYNs; never run implementation or firmware.

The first unique retry has already been launched and is adopted by exact
creation-time identity.  The independent POSE retry waits only for memory,
not for any PNR or the fusion synthesis to finish.  Both use the existing
40-minute retry and its original host-memory protection without new caps.
"""
import argparse
import copy
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback

import psutil

from retry_frozen_synthesis import memory_status

ROOT = Path(__file__).resolve().parents[2]
BASE = Path('F:/nav_integration_g2_20261023')
ORIGINAL = Path('F:/nav_integration_g2_20261022/directed_static_coverage_effective50')
STATE = BASE/'baseline_single_factor_parallel_syn1'
REVIEWER = ROOT/'integration/g2/review_baseline_single_factor_syn.py'
GIB = 1024**3
CASES = {
    'fusion': dict(prepared=BASE/'baseline_fusion_only_prepared1',
                   output=BASE/'baseline_fusion_only_syn23',
                   changed='control/imu_fusion.v', cpu=16),
    'pose': dict(prepared=BASE/'baseline_pose_only_prepared1',
                 output=BASE/'baseline_pose_only_syn24',
                 changed='nav/rtl/top/nav_subsystem.v', cpu=18),
}
_LAST_GOOD_JSON = {}
_JSON_READ_WARNINGS = []


def load(path):
    """The existing retry writes heartbeat JSON in place, so tolerate that edge."""
    path = Path(path)
    error = None
    for _ in range(6):
        try:
            value = json.loads(path.read_text(encoding='utf-8-sig'))
            _LAST_GOOD_JSON[str(path)] = copy.deepcopy(value)
            return value
        except (json.JSONDecodeError, OSError) as caught:
            error = caught
            time.sleep(.05)
    if str(path) in _LAST_GOOD_JSON:
        _JSON_READ_WARNINGS.append(dict(path=str(path), recorded_at=now(),
            finite_retries=6, used_last_known_good=True, error=str(error)))
        return copy.deepcopy(_LAST_GOOD_JSON[str(path)])
    raise error


def save(path, value):
    temporary = path.with_name(path.name+'.'+str(os.getpid())+'.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    temporary.replace(path)


def now():
    return datetime.now().astimezone().isoformat()


def identity(p):
    return dict(pid=p.pid, created_epoch=p.create_time(), command=p.cmdline())


def process(record):
    try:
        p = psutil.Process(record['pid'])
        return p if abs(p.create_time()-record['created_epoch']) < .1 else None
    except psutil.NoSuchProcess:
        return None


def identical(a, b):
    if a.stat().st_size != b.stat().st_size:
        return False
    with a.open('rb') as left, b.open('rb') as right:
        while True:
            x, y = left.read(65536), right.read(65536)
            if x != y:
                return False
            if not x:
                return True


def audit_prepared(key):
    case = CASES[key]
    prepared = case['prepared']
    review = load(prepared/'prepared_source_review.json')
    assert review['passed'] and review['modified_rtl_inputs'] == [case['changed']]
    assert not (prepared/'impl').exists(), 'frozen prepared directory must remain source-only'
    original = load(ORIGINAL/'manifest.json')
    manifest = load(prepared/'manifest.json')
    before = {Path(p).relative_to(ORIGINAL).as_posix() for p in original['source_files']}
    after = {Path(p).relative_to(prepared).as_posix() for p in manifest['source_files']}
    assert len(before) == 63 and before == after
    changed = sorted(p for p in before if not identical(ORIGINAL/p, prepared/p))
    assert changed == [case['changed']]
    suffixes = {'.hex', '.mem', '.bin', '.mif', '.vh', '.h'}
    inputs = {p.relative_to(ORIGINAL).as_posix(): p for p in ORIGINAL.rglob('*')
              if p.is_file() and 'impl' not in p.relative_to(ORIGINAL).parts and p.suffix.lower() in suffixes}
    assert sum(p.suffix.lower() in {'.vh', '.h'} for p in inputs.values()) == 31
    assert sum(p.suffix.lower() not in {'.vh', '.h'} for p in inputs.values()) == 69
    assert all(identical(p, prepared/relative) for relative, p in inputs.items())
    firmware = {}
    for directory in ('fw', 'firmware'):
        files = {p.relative_to(ORIGINAL).as_posix(): p for p in (ORIGINAL/directory).rglob('*') if p.is_file()}
        candidate = {p.relative_to(prepared).as_posix() for p in (prepared/directory).rglob('*') if p.is_file()}
        assert set(files) == candidate
        assert all(identical(p, prepared/relative) for relative, p in files.items())
        firmware[directory] = len(files)
    assert all(identical(ORIGINAL/p, prepared/p) for p in ('g2.cst', 'g2.sdc'))
    source_tcl = (ORIGINAL/'run.tcl').read_text(encoding='utf-8')
    candidate_tcl = (prepared/'run.tcl').read_text(encoding='utf-8')
    inverse = candidate_tcl.replace(prepared.as_posix(), ORIGINAL.as_posix()).replace(str(prepared), str(ORIGINAL))
    assert source_tcl == inverse
    assert [x.strip() for x in candidate_tcl.splitlines() if x.strip().startswith('run ')] == ['run syn']
    return dict(passed=True, recorded_at=now(), source=str(ORIGINAL), prepared=str(prepared),
                modified_hardware=changed, hardware_inputs=63, unchanged_hardware=62,
                identical_assets=69, identical_headers=31, firmware_files=firmware,
                IO_and_two_clock_SDC_byte_identical=True, firmware_build_not_run=True,
                prepared_review=str(prepared/'prepared_source_review.json'),
                source_directory_implementation_absent=True)


def existing_review(source):
    for p in psutil.process_iter(['pid', 'create_time', 'cmdline']):
        try:
            command = p.info['cmdline'] or []
            if not any(Path(x).name == REVIEWER.name for x in command) or '--source' not in command:
                continue
            actual = Path(command[command.index('--source')+1]).resolve()
            if actual == source.resolve():
                return identity(p)
        except (psutil.NoSuchProcess, psutil.AccessDenied, ValueError, IndexError):
            pass
    return None


def reusable_pass(path, case):
    if not path.exists():
        return False
    result = load(path)
    if not (result.get('passed') and result.get('actual_RAM_INIT_clock_26CDC_preflight_passed') and
            Path(result['source']).resolve() == case['output'].resolve() and
            Path(result['prepared']).resolve() == case['prepared'].resolve()):
        return False
    for record in result.get('reviewed_input_metadata', []):
        p = Path(record['path'])
        if not p.is_file() or (p.stat().st_size, p.stat().st_mtime_ns) != (record['bytes'], record['mtime_ns']):
            return False
    return bool(result.get('reviewed_input_metadata'))


def review_only():
    """An explicit recovery never launches retry, SYN, PNR or firmware builds."""
    claimfile = STATE/'coordinator_claim.json'
    assert not claimfile.exists() or process(load(claimfile)) is None, (
        'original unique coordinator is still alive; refuse parallel review recovery')
    folder = STATE/('review_only_recovery_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(exist_ok=False)
    state = dict(controller=identity(psutil.Process()), phase='review_only_completed_SYN_recovery',
        no_SYN_or_PNR_launch=True, programmed=False, deployment_accepted=False, cases={})
    for key, case in CASES.items():
        entry = state['cases'][key] = dict(source=str(case['output']), prepared=str(case['prepared']))
        jobfile = case['output']/'job.json'
        job = load(jobfile) if jobfile.exists() else {}
        if job.get('phase') != 'synthesis_complete' or job.get('tool_exit') != 0:
            entry['phase'] = 'not_complete_not_consumed_no_build'
            save(folder/'job.json', state)
            continue
        previous = STATE/(key+'_actual_netlist_review.json')
        if reusable_pass(previous, case):
            entry.update(phase='existing_actual_review_pass_reused', passed=True, review_result=str(previous))
            save(folder/'job.json', state)
            continue
        active = existing_review(case['output'])
        if active:
            entry.update(phase='existing_actual_review_active_not_duplicated', active_review=active)
            save(folder/'job.json', state)
            continue
        assert REVIEWER.is_file(), 'independent actual-netlist reviewer not ready'
        result = folder/(key+'_actual_netlist_review.json')
        command = [sys.executable, '-X', 'utf8', str(REVIEWER), '--source', str(case['output']),
                   '--prepared', str(case['prepared']), '--out', str(result)]
        entry.update(phase='reviewing_existing_completed_VG_only', command=command)
        save(folder/'job.json', state)
        with (folder/(key+'_actual_netlist_review.log')).open('x', encoding='utf-8') as log:
            reviewed = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                creationflags=subprocess.CREATE_NO_WINDOW | subprocess.IDLE_PRIORITY_CLASS)
        passed = reviewed.returncode == 0 and reusable_pass(result, case)
        entry.update(phase='actual_netlist_review_passed' if passed else 'actual_netlist_review_failed',
                     passed=passed, review_result=str(result), review_exit=reviewed.returncode)
        save(folder/'job.json', state)
    state.update(phase='review_only_finished', updated_at=now(), json_read_warnings=_JSON_READ_WARNINGS[-100:])
    save(folder/'job.json', state)
    print(str(folder/'job.json'), flush=True)
    return int(not all(x.get('passed') for x in state['cases'].values()))


def run():
    STATE.mkdir(exist_ok=True)
    claim = identity(psutil.Process())
    with (STATE/'coordinator_claim.json').open('x', encoding='utf-8') as f:
        json.dump(claim, f, indent=2)
    state = dict(controller=claim, started_at=now(), phase='binding_prepared_sources',
                 automatic_PNR=False, programmed=False, deployment_accepted=False,
                 PNR31_untouched=True, cases={}, memory_wait_dependency='host memory only; no PNR completion dependency')

    def update(phase=None):
        if phase:
            state['phase'] = phase
        state['updated_at'] = now()
        state['json_read_warnings'] = _JSON_READ_WARNINGS[-100:]
        save(STATE/'job.json', state)

    try:
        for key, case in CASES.items():
            evidence = audit_prepared(key)
            save(STATE/(key+'_prepared_binding.json'), evidence)
            state['cases'][key] = dict(prepared=str(case['prepared']), output=str(case['output']),
                                      prepared_binding=str(STATE/(key+'_prepared_binding.json')))
        first = load(STATE/'fusion_launch.json')
        assert Path(first['output']) == CASES['fusion']['output']
        state['cases']['fusion'].update(phase='adopted_existing_unique_retry', control=first['control'])
        assert process(first['control']) is not None or load(CASES['fusion']['output']/'job.json')['phase'] == 'synthesis_complete'
        assert not CASES['pose']['output'].exists(), 'refuse duplicate POSE synthesis'
        stable = 0
        update('pose_independent_memory_admission')
        while True:
            memory = memory_status()
            enough = memory.avail_phys >= 2*GIB and memory.avail_page >= 4*GIB
            stable = stable+1 if enough else 0
            state['cases']['pose'].update(phase='waiting_for_own_memory_admission',
                available_physical_bytes=memory.avail_phys, available_commit_bytes=memory.avail_page,
                minimum_physical_bytes=2*GIB, minimum_commit_bytes=4*GIB,
                consecutive_qualifying_samples=stable, required_samples=2)
            update()
            if stable >= 2:
                break
            time.sleep(3)
        case = CASES['pose']
        command = [sys.executable, '-X', 'utf8', str(ROOT/'integration/g2/retry_frozen_synthesis.py'),
                   str(case['prepared']), '--out', str(case['output']), '--start-now',
                   '--tool-priority', 'idle', '--tool-cpu-core', str(case['cpu'])]
        with (STATE/'pose_retry.stdout.log').open('x', encoding='utf-8') as log:
            child = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                creationflags=subprocess.CREATE_NO_WINDOW | subprocess.IDLE_PRIORITY_CLASS)
        state['cases']['pose'].update(phase='unique_retry_started', control=identity(psutil.Process(child.pid)),
                                     command=command, admitted_at=now())
        save(STATE/'pose_launch.json', state['cases']['pose'])
        update('independent_syntheses_in_progress')
        completed = set()
        while len(completed) < len(CASES):
            for key, case in CASES.items():
                if key in completed:
                    continue
                record = state['cases'][key]
                wrapper = process(record['control'])
                jobfile = case['output']/'job.json'
                actual = load(jobfile) if jobfile.exists() else None
                record['actual_synthesis_job'] = actual
                if wrapper is not None:
                    observed = {str(x['pid']): x for x in record.get('observed_owned_tree', [])}
                    for owned in [wrapper]+wrapper.children(recursive=True):
                        try:
                            observed[str(owned.pid)] = identity(owned)
                        except psutil.NoSuchProcess:
                            pass
                    record['observed_owned_tree'] = list(observed.values())
                    continue
                live = [x for x in record.get('observed_owned_tree', []) if process(x) is not None]
                if live:
                    record.update(phase='wrapper_exited_owned_descendant_still_active', live_owned_descendants=live)
                    continue
                if not actual or actual.get('phase') != 'synthesis_complete' or actual.get('tool_exit') != 0:
                    record.update(phase='synthesis_did_not_complete_no_automatic_retry', ended_at=now())
                    completed.add(key)
                    continue
                if not REVIEWER.exists():
                    record.update(phase='synthesis_complete_waiting_for_independent_actual_netlist_reviewer')
                    continue
                result = STATE/(key+'_actual_netlist_review.json')
                command = [sys.executable, '-X', 'utf8', str(REVIEWER), '--source', str(case['output']),
                           '--prepared', str(case['prepared']), '--out', str(result)]
                record.update(phase='reviewing_completed_actual_netlist', review_command=command)
                update()
                with (STATE/(key+'_actual_netlist_review.log')).open('x', encoding='utf-8') as log:
                    reviewed = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                              creationflags=subprocess.CREATE_NO_WINDOW | subprocess.IDLE_PRIORITY_CLASS)
                record.update(phase='actual_netlist_review_passed' if reviewed.returncode == 0 and result.exists() and load(result).get('passed') else 'actual_netlist_review_failed',
                              review_result=str(result), review_exit=reviewed.returncode, ended_at=now())
                completed.add(key)
            update()
            if len(completed) < len(CASES):
                time.sleep(5)
        update('both_unique_SYN_attempts_and_available_actual_reviews_finished')
        return int(not all(x.get('phase') == 'actual_netlist_review_passed' for x in state['cases'].values()))
    except Exception:
        state['error_traceback'] = traceback.format_exc()
        update('controller_failed_requires_review_no_automatic_retry')
        raise


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--review-only', action='store_true',
        help='recover only the existing completed SYN23/24 netlist reviews; never start a synthesis')
    args = parser.parse_args()
    return review_only() if args.review_only else run()


if __name__ == '__main__':
    raise SystemExit(main())
