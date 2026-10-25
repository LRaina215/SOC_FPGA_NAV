"""Run distinct, prepared delivery tradeoffs serially after an owned PNR.

This controller never programs hardware. Removing optional functions and
reducing navigation performance are allowed only as explicitly recorded in
each profile. Core control, communication and stop checks remain mandatory.
No PNR receives a total or silent deadline. Existing builds are only observed.
"""
import argparse
from datetime import datetime
import importlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback

import psutil

from collect_routing_diagnostic import collect
from measure_row_read_context import reference_snapshot
from retry_frozen_synthesis import memory_status

ROOT = Path(__file__).resolve().parents[2]
GIB = 1024**3


def load(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def write(path, data):
    path = Path(path)
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    temporary.replace(path)


def alive(identity):
    try:
        process = psutil.Process(identity['pid'])
        return abs(process.create_time()-identity['created_epoch']) < .1
    except psutil.NoSuchProcess:
        return False


def classify(build):
    actual = collect(build)
    runtime = actual.get('runtime') or {}
    gate = load(Path(build)/'g2_gate.json') if (Path(build)/'g2_gate.json').exists() else {}
    if runtime.get('cancelled'):
        return 'review_cancelled_result', actual, gate
    if any('PR0004' not in error for error in gate.get('errors', [])):
        return 'specific_error_or_missing_report_requires_review', actual, gate
    if actual['reported_unrouted_nets'] is not None and actual['reported_unrouted_nets'] > 0:
        return 'natural_unrouted_continue_distinct_profile', actual, gate
    if gate.get('routing_passed'):
        if gate.get('setup_violations') == gate.get('hold_violations') == 0:
            return 'routing_and_reported_timing_clean_full_signoff_required', actual, gate
        setup, hold = gate.get('setup_violations'), gate.get('hold_violations')
        if isinstance(setup, int) and isinstance(hold, int) and setup+hold > 0:
            return 'routing_complete_reported_timing_failed_continue_distinct_profile', actual, gate
        return 'routing_complete_timing_needs_targeted_review', actual, gate
    return 'specific_error_or_missing_report_requires_review', actual, gate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--registry', type=Path, required=True)
    parser.add_argument('--prior-pnr', type=Path, required=True)
    parser.add_argument('--prior-state', type=Path, required=True)
    parser.add_argument('--state-dir', type=Path, required=True)
    parser.add_argument('--validate-registry-only', action='store_true')
    parser.add_argument('--resume-after-result', help='continue a stopped queue after this preserved profile has a real collected PNR result')
    args = parser.parse_args()
    registry, prior, prior_state, folder = [p.resolve() for p in
        (args.registry, args.prior_pnr, args.prior_state, args.state_dir)]
    specification = load(registry)
    identifiers = [p['id'] for p in specification['profiles']]
    assert len(identifiers) == len(set(identifiers)) and len(identifiers) >= 4
    assert specification['PNR_total_seconds'] == specification['PNR_silent_seconds'] == 0
    assert specification['hardware_programming_automatic'] is False
    for profile in specification['profiles']:
        assert profile['kind'] in ('frozen_netlist', 'rtl_source')
        assert profile['reviewer'] in ('review_delivery_trim_joint', 'review_lf_line_joint')
        assert profile['tradeoffs'] and profile['PNR']
    if args.validate_registry_only:
        print(json.dumps(dict(passed=True, profiles=identifiers, starts_builds=False)))
        return 0
    me = psutil.Process()
    me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    resume = None
    if args.resume_after_result:
        state = load(folder/'job.json')
        assert not alive(dict(pid=state['controller_pid'], created_epoch=state['controller_created_epoch'])), 'original queue still alive'
        assert state['registry'] == str(registry) and state['prior_PNR'] == str(prior) and state['prior_state'] == str(prior_state)
        assert state['phase'] == 'specific_error_or_missing_report_requires_review', 'queue did not stop for review'
        assert state['completed_order'][-1] == args.resume_after_result == state['active_profile']
        assert state['completed_order'] == identifiers[:identifiers.index(args.resume_after_result)+1], 'completed profiles are not the registry prefix'
        assert state['PNR_total_seconds'] == state['PNR_silent_seconds'] == 0
        assert not state.get('active_child')
        # Preflight is deliberately outside the mutating queue try/except.
        # An early invocation must leave the stopped job available for a later
        # real result, rather than changing its phase into another failure.
        profile_id = args.resume_after_result
        record = state['profiles'][profile_id]
        profile = next(p for p in specification['profiles'] if p['id'] == profile_id)
        assert profile == load(folder/profile_id/'profile_snapshot.json'), 'profile changed after dispatch'
        result_folder = Path(profile['PNR']).resolve()
        runtime = load(result_folder/'tool_runtime.json')
        assert runtime.get('ended_at') and runtime.get('tool_exit') is not None, 'real PNR has not finished'
        assert runtime['max_runtime_s'] == runtime['max_no_observable_progress_s'] == 0
        process = psutil.Process(runtime['pid']) if psutil.pid_exists(runtime['pid']) else None
        assert process is None or process.name().lower() != 'gw_sh.exe' or Path(process.cwd()).resolve() != result_folder, 'PNR tool still alive'
        assert (result_folder/'impl/pnr/g2_top.rpt.txt').is_file() and (result_folder/'g2_gate.json').is_file(), 'final report collection is incomplete'
        action, actual, gate = classify(result_folder)
        assert actual['pnr_report_available'] and Path(gate['build']).resolve() == result_folder
        assert gate.get('unrouted_nets') == actual['reported_unrouted_nets'], 'final report and collected gate disagree'
        saved = folder/('before_result_resume_'+profile_id+'.json')
        # Exclusive creation is the continuation claim. Two controllers must
        # never both pass an exists-then-write window and overwrite job.json.
        with saved.open('x', encoding='utf-8') as stream:
            json.dump(state, stream, ensure_ascii=False, indent=2)
            stream.write('\n')
        state.update(previous_controller=dict(pid=state['controller_pid'], created_epoch=state['controller_created_epoch']),
            controller_pid=me.pid, controller_created_epoch=me.create_time(),
            phase='resuming_preserved_profile_after_actual_PNR_result',
            resume_registered_profile_order=identifiers,
            updated_at=datetime.now().astimezone().isoformat())
        write(folder/'job.json', state)
        resume = dict(profile_id=profile_id, record=record, result_folder=result_folder,
                      action=action, actual=actual, gate=gate)
    else:
        folder.mkdir(exist_ok=False)
        state = dict(controller_pid=me.pid, controller_created_epoch=me.create_time(),
            registry=str(registry), prior_PNR=str(prior), prior_state=str(prior_state),
            profiles={}, completed_order=[], PNR_total_seconds=0, PNR_silent_seconds=0,
            maximum_simultaneous_PNR_runs=1, optional_parallel_SYN_runs=1,
            programmed=False, deployment_accepted=False)

    def save(phase, **extra):
        state.update(phase=phase, updated_at=datetime.now().astimezone().isoformat(), **extra)
        write(folder/'job.json', state)

    def memory_wait(phase):
        save(phase)
        while True:
            memory = memory_status()
            if memory.avail_phys >= 2*GIB and memory.avail_page >= 4*GIB:
                return memory
            time.sleep(15)

    def command_run(command, log, phase):
        with log.open('w', encoding='utf-8') as stream:
            child = subprocess.Popen(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT,
                creationflags=subprocess.CREATE_NO_WINDOW)
            identity = dict(pid=child.pid, created_epoch=psutil.Process(child.pid).create_time(), command=command)
            save(phase, active_child=identity, active_log=str(log))
            child.wait()
        state.pop('active_child', None)
        state['last_finished_child'] = identity
        return child.returncode

    try:
        if args.resume_after_result:
            profile_id, record, result_folder, action, actual, gate = (
                resume[k] for k in ('profile_id', 'record', 'result_folder', 'action', 'actual', 'gate'))
            write(folder/profile_id/'routing_result_after_settings_recovery.json', dict(action=action, actual=actual, gate=gate))
            record['prior_settings_timeout_result'] = dict(record)
            record.update(phase='collected_after_settings_only_recovery', action=action,
                reported_unrouted=actual['reported_unrouted_nets'], timing_report_available=actual['timing_report_available'],
                ended_at=datetime.now().astimezone().isoformat())
            save('preserved_profile_actual_result_collected', last_result=record)
            supported = {'natural_unrouted_continue_distinct_profile',
                         'routing_complete_reported_timing_failed_continue_distinct_profile'}
            if action not in supported:
                save(action, result_directory=str(result_folder))
                return 0
        else:
            snapshot = reference_snapshot(prior)
            current = load(prior_state/'job.json')
            expected = [(snapshot['pid'], 'gw_sh.exe', snapshot.get('created_epoch')),
                (current['pnr_wrapper_pid'], 'pnr_from_netlist.py', None),
                (current['controller_pid'], 'continue_row_read_joint_once.py', current['controller_created_epoch'])]
            identities = []
            for pid, needle, creation in expected:
                try:
                    process = psutil.Process(pid)
                    assert any(needle in item for item in process.cmdline()), ('owned process command changed', pid)
                    if creation is not None:
                        assert abs(process.create_time()-creation) < .1
                    if pid == snapshot['pid']:
                        assert process.ppid() == current['pnr_wrapper_pid']
                    identities.append(dict(pid=pid, created_epoch=process.create_time(), command=process.cmdline()))
                except psutil.NoSuchProcess:
                    pass
            save('waiting_for_PNR20_natural_exit_and_collection', protected_inputs=snapshot['input_metadata'],
                protected_processes=identities)
            while any(alive(identity) for identity in identities):
                time.sleep(15)
            assert reference_snapshot(prior)['input_metadata'] == snapshot['input_metadata'], 'prior inputs changed'
            action, actual, gate = classify(prior)
            write(folder/'prior_result.json', dict(action=action, actual=actual, gate=gate))
            supported = {'natural_unrouted_continue_distinct_profile',
                         'routing_complete_reported_timing_failed_continue_distinct_profile'}
            if action not in supported:
                save(action, result_directory=str(prior))
                return 0

        while True:
            specification = load(registry)
            entries = specification['profiles']
            assert [p['id'] for p in entries] == identifiers, 'profile identities/order cannot change after dispatch'
            remaining = [p for p in entries if p['id'] not in state['completed_order']]
            if not remaining:
                save('all_registered_distinct_profiles_collected_full_acceptance_not_claimed')
                return 0
            profile = remaining[0]
            pid = profile['id']
            if not profile.get('ready'):
                save('waiting_for_prepared_profile_and_targeted_proof', active_profile=pid,
                    pending_reason=profile.get('pending_reason', 'preparation incomplete'))
                time.sleep(15)
                continue
            # Ready entries become immutable before tools start; subsequent
            # registry edits cannot change a running profile's frozen inputs.
            item = folder/pid
            item.mkdir(exist_ok=False)
            write(item/'profile_snapshot.json', profile)
            prepared = Path(profile['prepared']).resolve()
            baseline = Path(profile['baseline']).resolve()
            preparation = load(prepared/'prepared_source_review.json')
            assert preparation['passed'], ('preparation failed', pid)
            source = prepared
            cache = None
            record = dict(profile=profile, phase='prepared_proof_check', started_at=datetime.now().astimezone().isoformat())
            state['profiles'][pid] = record
            save('profile_prepared_for_serial_execution', active_profile=pid)
            if profile['kind'] == 'rtl_source':
                source = Path(profile['synthesis']).resolve()
                cache_path = profile.get('parallel_synthesis_cache_job')
                if cache_path:
                    # The producer owns the only SYN. Do not compete with it,
                    # restart it, or discard a completed source after review.
                    cache_path = Path(cache_path)
                    save('waiting_for_parallel_SYN_and_actual_netlist_review', parallel_synthesis_cache_job=str(cache_path))
                    while True:
                        cache = load(cache_path) if cache_path.exists() else None
                        if cache and cache.get('phase') == 'ready_for_PNR':
                            from parallel_trim_synthesis import validate_ready_cache
                            cache = validate_ready_cache(cache_path, profile)
                            assert Path(cache['synthesis']).resolve() == source
                            assert Path(cache['prepared']).resolve() == prepared
                            record['parallel_completed_SYN_reused'] = True
                            record['parallel_synthesis_cache_job'] = str(cache_path)
                            break
                        if cache and cache.get('phase') == 'synthesis_failed':
                            record.update(phase='synthesis_failed_no_retry', parallel_synthesis_cache_job=str(cache_path))
                            state['completed_order'].append(pid)
                            save('independent_profile_SYN_failed_other_prepared_profiles_remain', last_result=record)
                            break
                        if cache and cache.get('phase') == 'pre_pnr_review_failed':
                            # A targeted review-only repair may publish ready
                            # later, using exactly this completed synthesis.
                            save('waiting_for_targeted_review_only_repair_no_resynthesis',
                                 parallel_synthesis_cache_job=str(cache_path), review_error=cache.get('review_error'))
                        time.sleep(15)
                    if record.get('phase') == 'synthesis_failed_no_retry':
                        continue
                else:
                    assert not source.exists(), ('refuse duplicate synthesis', source)
                    memory = memory_wait('waiting_for_serial_SYN_memory')
                    command = [sys.executable, '-X', 'utf8', str(ROOT/'integration/g2/retry_frozen_synthesis.py'),
                        str(prepared), '--out', str(source), '--start-now']
                    code = command_run(command, item/'synthesis.log', 'one_profile_joint_SYN_running')
                    if code:
                        job = load(source/'job.json') if (source/'job.json').exists() else {}
                        text = (item/'synthesis.log').read_text(encoding='utf-8')
                        preflight = (source/'preflight.log').read_text(encoding='utf-8') if (source/'preflight.log').exists() else ''
                        settings_only = (job.get('phase') == 'validating_frozen_settings' and 'tool_pid' not in job
                            and 'TimeoutExpired' in text and 'ERROR' not in preflight
                            and (source/'effective_options.tcl').exists() and not (source/'impl').exists()
                            and not (source/'gw_sh.log').exists())
                        if settings_only:
                            code = command_run(command+['--resume-settings-only'], item/'settings_only_retry.log',
                                'one_preserved_settings_only_retry_no_SYN_repetition')
                    assert code == 0 and load(source/'job.json')['phase'] == 'synthesis_complete', ('SYN failed', pid)
                    record['synthesis_start_free_bytes'] = memory.avail_phys
                    record['synthesis_start_commit_reserve_bytes'] = memory.avail_page

            # Re-import on each serial profile so completed, reviewed checker
            # fixes can be used before a later profile begins, never mid-check.
            if cache:
                result = load(cache['pre_pnr_checks'])
                assert result['passed'], ('parallel review did not pass', pid)
                shutil.copyfile(cache['pre_pnr_checks'], item/'pre_pnr_checks.json')
                shutil.copyfile(cache['reviewer_snapshot'], item/'reviewer_snapshot.py')
                (item/'validation.log').write_text('Reused completed source/RAM/clock/CDC review after exact file metadata revalidation; no SYN repeated.\n', encoding='utf-8')
                save('completed_parallel_SYN_and_actual_review_reused')
            else:
                reviewer = importlib.import_module(profile['reviewer'])
                reviewer = importlib.reload(reviewer)
                shutil.copyfile(Path(reviewer.__file__), item/'reviewer_snapshot.py')
                save('reviewing_profile_source_actual_RAM_clocks_CDC_and_tradeoffs')
                with (item/'validation.log').open('w', encoding='utf-8') as log:
                    try:
                        result = reviewer.review(source, baseline)
                        write(item/'pre_pnr_checks.json', result)
                        assert result['passed'], ('new netlist prechecks failed', pid)
                        log.write('Pre-PNR source, resource, driver and CDC checks passed.\n')
                    except Exception:
                        log.write(traceback.format_exc())
                        raise
            inputs = item/'pnr_inputs'
            (inputs/'impl/gwsynthesis').mkdir(parents=True)
            (inputs/'ip').mkdir()
            for relative in ('impl/gwsynthesis/g2_top.vg', 'impl/gwsynthesis/g2_top.prj',
                             'effective_options.tcl', 'manifest.json'):
                shutil.copyfile(source/relative, inputs/relative)
            constraints = Path(profile.get('constraints', str(baseline))).resolve()
            assert constraints == baseline or result.get('constraint_source') == str(constraints), 'unreviewed constraint source'
            for relative in ('g2.cst', 'g2.sdc', 'ip/serdes.csr', 'cdc_reference_audit.json', 'sfp_cdc_reference.json'):
                # SYN11 retained an older twelve-request CST. Use the reviewed
                # physical reference's fourteen-request plan, as the preceding
                # LF controller did; never silently inherit a stale SYN CST.
                shutil.copyfile(constraints/relative, inputs/relative)
            out = Path(profile['PNR']).resolve()
            assert not out.exists(), ('refuse duplicate PNR', out)
            memory_wait('waiting_for_serial_PNR_memory')
            command = [sys.executable, '-X', 'utf8', str(ROOT/'integration/g2/pnr_from_netlist.py'),
                str(inputs), '--out', str(out), '--wide-split', '1', '--replicate-resources', '1',
                '--place-option', '1', '--route-option', '2', '--timing-driven', '1',
                '--max-runtime-minutes', '0', '--max-no-progress-minutes', '0',
                '--routing-evidence', '--capture-system-telemetry']
            code = command_run(command, item/'pnr.log', 'one_profile_joint_PNR_running_no_deadline')
            if code and cache and not (out/'tool_runtime.json').exists():
                text = (item/'pnr.log').read_text(encoding='utf-8', errors='replace')
                settings_only = ("'preflight.tcl']' timed out after 60.0 seconds" in text
                    and (out/'effective_options.tcl').is_file()
                    and not any((out/name).exists() for name in ('run.tcl', 'manifest.json', 'impl')))
                if settings_only:
                    # This exact startup failure has no real PNR to repeat.
                    # The separate helper proves all saved options, immutable
                    # inputs and reviewed-cache provenance before continuing.
                    previous = state['last_finished_child']
                    helper = ROOT/'integration/g2/resume_pnr_settings_only.py'
                    plan_dir = item/'settings_only_recovery_plan'
                    recovery = [sys.executable, '-X', 'utf8', str(helper), '--resume-settings-only',
                        '--source', str(inputs), '--out', str(out), '--wrapper-log', str(item/'pnr.log'),
                        '--cache', str(profile['parallel_synthesis_cache_job']), '--registry', str(registry),
                        '--profile', pid, '--previous-wrapper-pid', str(previous['pid']),
                        '--previous-wrapper-created-epoch', str(previous['created_epoch']),
                        '--plan-dir', str(plan_dir)]
                    review_code = command_run(recovery, item/'settings_recovery_review.log', 'reviewing_saved_settings_before_first_PNR')
                    assert review_code == 0, 'saved-settings recovery review failed'
                    code = command_run([sys.executable, '-X', 'utf8', str(helper), '--resume-settings-only',
                        '--execute-plan', str(plan_dir/'resume_plan.json')], item/'settings_recovery_execute.log',
                        'first_actual_PNR_after_verified_settings_recovery_no_deadline')
            action, actual, gate = classify(out)
            write(item/'routing_result.json', dict(action=action, actual=actual, gate=gate))
            record.update(phase='collected', wrapper_exit=code, action=action,
                reported_unrouted=actual['reported_unrouted_nets'], timing_report_available=actual['timing_report_available'],
                ended_at=datetime.now().astimezone().isoformat())
            state['completed_order'].append(pid)
            save('profile_collected', last_result=record)
            if action not in supported:
                save(action, result_directory=str(out))
                return 0
    except Exception as error:
        (folder/'failure.log').write_text(traceback.format_exc(), encoding='utf-8')
        save('targeted_review_required_no_duplicate_or_blind_retry', error=str(error), error_type=type(error).__name__)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
