"""Publish strict exploratory-review caches for preserved report-stage VGs.

Only the independent joint reviewer runs, as a low-priority read-only child.
Original failed jobs and artifacts stay intact. This driver has no synthesis,
PNR or hardware command. --reserve-only exposes the pending shared-review gate.
"""
import argparse
from datetime import datetime
import inspect
import json
import os
from pathlib import Path
import shutil
import sys
import time
import traceback

import psutil

from parallel_trim_synthesis import (alive, file_metadata, frozen_input_metadata,
    identity, load, memory_status, run_owned, validate_ready_cache, write)
from recover_report_stage_netlist import validate_recovery_evidence

ROOT = Path(__file__).resolve().parents[2]
GIB = 1024**3


def now():
    return datetime.now().astimezone().isoformat()


def produce(profile, folder, cpu_core, reserve_only):
    folder.mkdir(parents=True, exist_ok=True)
    path = folder/'job.json'
    old = load(path) if path.exists() else {}
    source, prepared, baseline = [Path(profile[key]).resolve()
        for key in ('synthesis', 'prepared', 'baseline')]
    recovery_path = source/'netlist_recovery_evidence.json'
    if old:
        assert old['profile_id'] == profile['id']
        assert Path(old['synthesis']).resolve() == source and Path(old['prepared']).resolve() == prepared
        if old['phase'] == 'ready_for_PNR':
            return validate_ready_cache(old, profile)
    assert Path(profile['parallel_synthesis_cache_job']).resolve() == path.resolve(), 'registry does not point to this new recovery cache'
    state = dict(schema_version=1, profile_id=profile['id'], prepared=str(prepared),
        synthesis=str(source), baseline=str(baseline), artifact_mode='recovered_emitted_netlist',
        netlist_recovery_evidence=str(recovery_path), source_frozen=True,
        source_job_not_promoted=True, vendor_synthesis_completed=False,
        controller_pid=psutil.Process().pid, controller_created_epoch=psutil.Process().create_time(),
        programmed=False, deployment_accepted=False, PNR_started=False,
        review_attempt=old.get('review_attempt', 0),
        scope='independent actual source/RAM/INIT/clock/CDC/profile review; original report-stage interruption preserved')

    def save(phase=None, **extra):
        if phase:
            state['phase'] = phase
        state.update(updated_at=now(), **extra)
        write(path, state)

    try:
        save('validating_preserved_recovery_evidence')
        evidence = validate_recovery_evidence(recovery_path, source=source)
        assert Path(evidence['prepared']).resolve() == prepared
        evidence_metadata = file_metadata(recovery_path)
        inputs = dict(prepared=frozen_input_metadata(prepared), synthesis=frozen_input_metadata(source))
        save('waiting_for_shared_reviewer_proof_and_release',
            netlist_recovery_evidence_validated=True,
            original_vendor_job_phase=evidence['original_job_phase'],
            recovered_resources=evidence['resources'],
            pending_reason='Shared console/SoC next-state proof and reviewer release are being completed independently; no tool or SYN started.')
        if reserve_only:
            return state
        import review_delivery_trim_joint as joint
        assert 'report_stage_recovery' in inspect.signature(joint.review).parameters
        while True:
            memory = memory_status()
            if memory.avail_phys >= 2*GIB and memory.avail_page >= 4*GIB:
                break
            save('waiting_for_read_only_review_memory', available_physical_bytes=memory.avail_phys,
                available_commit_bytes=memory.avail_page)
            time.sleep(15)
        attempt = int(state['review_attempt'])+1
        attempt_dir = folder/f'review_attempt{attempt}'
        attempt_dir.mkdir(exist_ok=False)
        reviewer = ROOT/'integration/g2/review_delivery_trim_joint.py'
        snapshot = attempt_dir/'reviewer_snapshot.py'
        shutil.copyfile(reviewer, snapshot)
        shutil.copyfile(ROOT/'integration/g2/recover_report_stage_netlist.py', attempt_dir/'recovery_auditor_snapshot.py')
        result_path = attempt_dir/'pre_pnr_checks.json'
        save('reviewing_actual_preserved_netlist', review_attempt=attempt,
            pre_pnr_checks=str(result_path), reviewer_snapshot=str(snapshot), pending_reason=None)
        command = [sys.executable, '-X', 'utf8', str(reviewer), '--source', str(source),
            '--baseline', str(baseline), '--report-stage-recovery', str(recovery_path), '--out', str(result_path)]
        runtime = run_owned(command, attempt_dir, 'strict_recovery_review', cpu_core, save)
        if runtime['tool_exit'] or runtime['cancelled'] or not result_path.exists():
            result = load(result_path) if result_path.exists() else None
            save('pre_pnr_review_failed', review_runtime=runtime, review_error=result,
                pending_reason='Targeted read-only repair required; no resynthesis or PNR was started.')
            return state
        result = load(result_path)
        assert result['passed'] and result['artifact_mode'] == 'recovered_emitted_netlist'
        assert result['independent_actual_netlist_precheck_passed'] and not result['vendor_synthesis_completed']
        assert Path(result['netlist_recovery_evidence']).resolve() == recovery_path
        assert snapshot.read_bytes() == reviewer.read_bytes(), 'shared reviewer changed during this attempt'
        assert inputs['prepared'] == frozen_input_metadata(prepared), 'prepared inputs changed during strict review'
        assert inputs['synthesis'] == frozen_input_metadata(source), 'preserved netlist/provenance changed during strict review'
        assert evidence_metadata == file_metadata(recovery_path), 'recovery proof changed during review'
        validate_recovery_evidence(recovery_path, source=source)
        verified = dict(**inputs, pre_pnr_checks=file_metadata(result_path),
            reviewer_snapshot=file_metadata(snapshot), netlist_recovery_evidence=evidence_metadata)
        state.update(phase='ready_for_PNR', verified_input_metadata=verified,
            review_runtime=runtime, review_error=None, pending_reason=None,
            independent_actual_netlist_precheck_passed=True, updated_at=now())
        # Validate the complete consumer contract before the atomic ready write.
        validate_ready_cache(state, profile)
        write(path, state)
        return state
    except Exception as error:
        (folder/f"review_failure_attempt{state['review_attempt']}_{time.time_ns()}.log").write_text(traceback.format_exc(), encoding='utf-8')
        save('pre_pnr_review_failed', review_error=str(error), review_error_type=type(error).__name__,
            pending_reason='Targeted read-only repair required; source job and prior cache are preserved.')
        return state


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--registry', type=Path, required=True)
    parser.add_argument('--state-dir', type=Path, required=True)
    parser.add_argument('--profile', action='append', required=True)
    parser.add_argument('--cpu-core', type=int, default=16)
    parser.add_argument('--reserve-only', action='store_true')
    args = parser.parse_args()
    assert len(args.profile) == len(set(args.profile)), 'duplicate selected profile'
    profiles = [p for p in load(args.registry)['profiles'] if p['id'] in args.profile]
    assert {p['id'] for p in profiles} == set(args.profile)
    assert all(p['kind'] == 'rtl_source' for p in profiles)
    me = psutil.Process()
    assert args.cpu_core in me.cpu_affinity()
    me.nice(psutil.IDLE_PRIORITY_CLASS)
    me.cpu_affinity([args.cpu_core])
    args.state_dir.mkdir(parents=True, exist_ok=True)
    lock = args.state_dir/'reviewer_driver.lock.json'
    if lock.exists():
        assert not alive(load(lock)), 'another recovery-review driver owns this cache'
        lock.rename(args.state_dir/f'stale_reviewer_lock_{time.time_ns()}.json')
    descriptor = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
    with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
        json.dump(identity(me), stream, indent=2)
    try:
        failed = False
        for profile in profiles:
            result = produce(profile, args.state_dir/profile['id'], args.cpu_core, args.reserve_only)
            print(json.dumps(dict(profile_id=profile['id'], phase=result['phase'],
                cache_job=str((args.state_dir/profile['id']/'job.json').resolve()),
                vendor_synthesis_completed=False, SYN_or_PNR_started=False), ensure_ascii=False), flush=True)
            failed |= result['phase'] == 'pre_pnr_review_failed'
        return int(failed)
    finally:
        if lock.exists() and load(lock)['pid'] == me.pid:
            lock.unlink()


if __name__ == '__main__':
    raise SystemExit(main())
