"""One diagnostic pure-RTL PNR from a completed, independently reviewed SYN.

There is no SYN, programming or PNR31 dependency in this command. Each candidate
owns a permanent controller claim and its own PNR directory. All timing input
bytes/options come from PNR27, including its original two clocks and no added
CDC exceptions. The 26 known connections are collected separately.
"""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import shutil
import time

import psutil

from bounded_tool import run_tool
from collect_pnr_result import finalize_gate
from retry_frozen_synthesis import memory_status


ROOT = Path('F:/nav_integration_g2_20261023')
BASELINE = ROOT/'baseline_rep1_replay_pnr27'
ORIGIN = Path('F:/nav_integration_g2_20261022/directed_static_coverage_effective50')
TOOL = Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe')
GIB = 1024**3
ASSETS = {'.hex', '.mem', '.bin', '.mif'}
PROFILES = {
    'fusion': dict(source=ROOT/'baseline_fusion_only_syn23',
                   prepared=ROOT/'baseline_fusion_only_prepared1',
                   output=ROOT/'baseline_fusion_only_pnr32',
                   changed='control/imu_fusion.v', offset=0, interval=13),
    'pose': dict(source=ROOT/'baseline_pose_only_syn24',
                 prepared=ROOT/'baseline_pose_only_prepared1',
                 output=ROOT/'baseline_pose_only_pnr33',
                 changed='nav/rtl/top/nav_subsystem.v', offset=7, interval=17),
}


def load(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def load_synthesis_job(source):
    """The existing SYN wrapper writes job.json in place; tolerate only a
    short incomplete JSON write, never a semantic failure or endless retry.
    """
    for attempt in range(8):
        try:
            return load(source/'job.json')
        except json.JSONDecodeError:
            if attempt == 7:
                raise
            time.sleep(0.1)


def write(path, value):
    path = Path(path)
    temporary = path.with_name(path.name+'.'+str(os.getpid())+'.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    temporary.replace(path)


def metadata(path):
    path = Path(path).resolve()
    stat = path.stat()
    return dict(path=str(path), bytes=stat.st_size, mtime_ns=stat.st_mtime_ns)


def equal_bytes(first, second):
    first, second = Path(first), Path(second)
    if first.stat().st_size != second.stat().st_size:
        return False
    with first.open('rb') as a, second.open('rb') as b:
        while True:
            x, y = a.read(65536), b.read(65536)
            if x != y:
                return False
            if not x:
                return True


def hardware(root):
    root = Path(root).resolve()
    paths = [Path(p).resolve() for p in load(root/'manifest.json')['source_files']]
    result = {p.relative_to(root).as_posix(): p for p in paths}
    assert len(paths) == len(result) == 63
    return result


def inputs(root, suffixes):
    return {p.relative_to(root).as_posix(): p for p in root.rglob('*')
            if p.is_file() and 'impl' not in p.relative_to(root).parts
            and '__pycache__' not in p.parts and p.suffix.lower() in suffixes}


def check_review(profile, review_file):
    """Read actual acceptance, not a prepared-source result or a zero exit code."""
    source, prepared = profile['source'], profile['prepared']
    review = load(review_file)
    assert review['passed'] is True, 'actual SYN review failed; preserve it without a PNR'
    for key, expected in [('source', source), ('prepared', prepared), ('baseline', BASELINE)]:
        assert Path(review[key]).resolve() == expected.resolve(), ('wrong review binding', key)
    vg = source/'impl/gwsynthesis/g2_top.vg'
    assert Path(review['actual_vg']).resolve() == vg.resolve()
    assert review['actual_RAM_INIT_clock_26CDC_preflight_passed'] is True
    assert review['deployment_accepted'] is False and review.get('scope')
    assert [review[k] for k in ('source_BSRAM', 'wide_leaves', 'CPU_SP_count',
                               'true_clocks', 'known_CDC_paths')] == [314, 18, 64, 2, 26]
    records = review['reviewed_input_metadata']
    indexed = {Path(r['path']).resolve(): r for r in records}
    assert len(indexed) == len(records), 'duplicate metadata binding'
    for path, record in indexed.items():
        assert record == metadata(path), ('review metadata changed', str(path))
    bindings = [(vg, Path(review['reviewed_netlist_copy'])),
                (source/'g2.cst', Path(review['reviewed_CST_copy'])),
                (BASELINE/'g2.cst', Path(review['reviewed_CST_copy'])),
                (source/'g2.sdc', Path(review['reviewed_SDC_copy'])),
                (BASELINE/'g2.sdc', Path(review['reviewed_SDC_copy']))]
    for original, snapshot in bindings:
        assert original.resolve() != snapshot.resolve(), 'review copy is the input itself'
        assert equal_bytes(original, snapshot), ('reviewed bytes changed', str(original))
        for path in (original, snapshot):
            assert indexed[path.resolve()] == metadata(path), ('review metadata changed', str(path))
    rows = review['paths']
    assert len(rows) == 26 and all(r['passed'] is True for r in rows)
    assert len({(r['source_register'], r['first_register']) for r in rows}) == 26
    return review


def check_source(profile):
    source, prepared = profile['source'], profile['prepared']
    job, manifest = load_synthesis_job(source), load(source/'manifest.json')
    assert job['phase'] == 'synthesis_complete' and job['tool_exit'] == 0
    assert Path(job['source']).resolve() == prepared.resolve()
    assert Path(manifest['frozen_source_build']).resolve() == prepared.resolve()
    log = (source/'gw_sh.log').read_text(encoding='utf-8', errors='replace')
    assert 'GowinSynthesis finish' in log and '[100%] Generate report file' in log
    assert not any(line.startswith('ERROR') for line in log.splitlines())
    assert (source/'impl/gwsynthesis/g2_top.prj').is_file()
    assert (source/'impl/gwsynthesis/g2_top_syn_rsc.xml').is_file()
    prep_review = load(prepared/'prepared_source_review.json')
    assert prep_review['passed'] is True
    assert prep_review['modified_rtl_inputs'] == [profile['changed']]
    assert Path(prep_review['source']).resolve() == ORIGIN.resolve()
    assert [prep_review[k] for k in ('unchanged_hardware_inputs', 'unchanged_assets',
                                    'unchanged_headers')] == [62, 69, 31]
    original, frozen, actual = hardware(ORIGIN), hardware(prepared), hardware(source)
    assert list(original) == list(frozen) == list(actual)
    assert [k for k in original if not equal_bytes(original[k], frozen[k])] == [profile['changed']]
    assert all(equal_bytes(frozen[k], actual[k]) for k in frozen)
    for suffixes, count in [(ASSETS, 69), ({'.h', '.vh'}, 31)]:
        old, prepared_inputs, actual_inputs = [inputs(root, suffixes)
                                               for root in (ORIGIN, prepared, source)]
        assert old.keys() == prepared_inputs.keys() == actual_inputs.keys()
        assert len(old) == count
        assert all(equal_bytes(old[k], prepared_inputs[k]) and
                   equal_bytes(old[k], actual_inputs[k]) for k in old)
    for name in ('g2.cst', 'g2.sdc'):
        assert all(equal_bytes(BASELINE/name, root/name) for root in (ORIGIN, prepared, source))
    assert (BASELINE/'g2.sdc').read_text().splitlines() == [
        'create_clock -name clk50 -period 20 [get_ports {clk50}]',
        'create_clock -name rgmii_rxc -period 10 [get_ports {rgmii_rxc}]']
    gate = load(BASELINE/'g2_gate.json')
    assert gate['routing_passed'] is True
    completed_after_checks = load_synthesis_job(source)
    for key in ('phase', 'tool_exit', 'source', 'tool_pid', 'tool_created_epoch', 'synthesis_started_at'):
        assert completed_after_checks.get(key) == job.get(key), ('SYN job changed during source check', key)
    return manifest


def reference_tcl(output):
    original = (BASELINE/'run.tcl').read_bytes()
    assert [x.strip() for x in original.decode().splitlines() if x.strip().startswith('run ')] == ['run pnr']
    assert b'set_option -route_option 0' in original
    relocated = original.replace(BASELINE.as_posix().encode(), output.as_posix().encode())
    relocated = relocated.replace(str(BASELINE).encode(), str(output).encode())
    assert relocated.replace(output.as_posix().encode(), BASELINE.as_posix().encode()).replace(
        str(output).encode(), str(BASELINE).encode()) == original
    return relocated


def prepare(profile, review_file):
    review, source_manifest = check_review(profile, review_file), check_source(profile)
    output = profile['output']
    if output.exists():
        validate_output(profile, review_file)
        return output
    output.mkdir(exist_ok=False)
    for name, origin in [('g2_top.vg', profile['source']/'impl/gwsynthesis/g2_top.vg'),
                         ('g2.cst', BASELINE/'g2.cst'), ('g2.sdc', BASELINE/'g2.sdc')]:
        shutil.copyfile(origin, output/name)
    (output/'run.tcl').write_bytes(reference_tcl(output))
    shutil.copyfile(review_file, output/'actual_SYN_preflight_review.json')
    write(output/'pure_CDC_connection_reference.json', dict(
        source_review=str(review_file), synthesis_connection_preflight_passed=True,
        paths=review['paths'], added_timing_exceptions=0,
        scope='26 connection reference only. Original two-clock SDC is deliberately unchanged.'))
    write(output/'manifest.json', dict(source=str(profile['source']), prepared=str(profile['prepared']),
        source_manifest=source_manifest, frozen_netlist=True, reused_completed_synthesis=True,
        pure_RTL_single_factor=profile['changed'], option_reference=str(BASELINE),
        actual_SYN_review=str(review_file), original_SDC_bytes_preserved=True,
        added_timing_exceptions=0, max_runtime_minutes=0, max_no_progress_minutes=0,
        reference_input_metadata=[metadata(BASELINE/name) for name in
                                  ('run.tcl', 'g2.cst', 'g2.sdc', 'impl/pnr/cmd.do')],
        diagnostic_only=True, automatic_programming=False, programmed=False,
        deployment_accepted=False,
        scope='Exploratory pure-RTL PNR; no physical/function/stop or deployment acceptance.'))
    validate_output(profile, review_file)
    return output


def validate_output(profile, review_file):
    review = check_review(profile, review_file)
    output = profile['output']
    assert load(output/'actual_SYN_preflight_review.json') == review
    for name, origin in [('g2_top.vg', profile['source']/'impl/gwsynthesis/g2_top.vg'),
                         ('g2.cst', BASELINE/'g2.cst'), ('g2.sdc', BASELINE/'g2.sdc')]:
        assert equal_bytes(output/name, origin), ('PNR input changed', name)
    assert (output/'run.tcl').read_bytes() == reference_tcl(output)
    manifest = load(output/'manifest.json')
    assert manifest['diagnostic_only'] is True and manifest['deployment_accepted'] is False
    assert manifest['added_timing_exceptions'] == 0
    assert manifest['reference_input_metadata'] == [metadata(BASELINE/name) for name in
                                                   ('run.tcl', 'g2.cst', 'g2.sdc', 'impl/pnr/cmd.do')]
    assert not (output/'cdc_reference_audit.json').exists(), 'do not require 52 intentionally absent exceptions'
    assert not (output/'tool_runtime.json').exists(), 'refuse repeated PNR'
    assert not (output/'impl').exists(), 'refuse reused PNR directory'


def physical_connections(profile, save):
    output = profile['output']; netlist = output/'impl/pnr/g2_top.vo'
    target = output/'pure_CDC_physical_connection_review.json'
    if not netlist.exists():
        write(target, dict(passed=False, reason='physical simulation netlist unavailable',
                           added_timing_exceptions=0, full_CDC_timing_signoff=False))
        return False
    wait_memory(profile, save, 'waiting_for_physical_connection_audit_memory')
    from audit_cdc_first_stages import audit
    actual = audit(netlist)
    expected = load(output/'pure_CDC_connection_reference.json')['paths']
    keys = ('group', 'source_register', 'first_register', 'second_register', 'source_clock', 'target_clock')
    signature = lambda rows: sorted(tuple(row[key] for key in keys) for row in rows)
    same = actual['passed'] and signature(actual['paths']) == signature(expected)
    actual.update(passed=actual['passed'] and same,
                  actual_source_first_second_identity_matches_reviewed_SYN=same,
                  added_timing_exceptions=0, timing_constraints_deliberately_unchanged=True,
                  full_CDC_timing_signoff=False)
    write(target, actual)
    return actual['passed']


def wait_memory(profile, save, phase):
    # Independent, offset sampling. This is admission observation, not a memory
    # reservation and never a deadline or a reason to touch an existing PNR.
    while True:
        memory = memory_status()
        save(phase, available_physical_bytes=memory.avail_phys,
             available_commit_bytes=memory.avail_page,
             minimum_physical_bytes=2*GIB, minimum_commit_bytes=4*GIB)
        if memory.avail_phys >= 2*GIB and memory.avail_page >= 4*GIB:
            return
        time.sleep(profile['interval'])


def controller_alive(record):
    try:
        process = psutil.Process(record['controller_pid'])
        return abs(process.create_time()-record['controller_created_epoch']) < 0.01
    except psutil.NoSuchProcess:
        return False


def claim_controller(candidate, review_file, continuation_attempt):
    """Continue only a failed *input review*, with an append-only claim chain.

    A physical directory containing runtime or implementation artifacts is never
    eligible. Every failed controller/claim remains in its original directory.
    Each parent has one exclusive child claim, so distinct attempt numbers
    cannot fork two controllers for the same unstarted physical output.
    """
    profile = PROFILES[candidate]
    base = ROOT/'baseline_single_factor_pnr_controllers1'/candidate
    previous = None
    if continuation_attempt is None:
        control = base
        control.mkdir(parents=True, exist_ok=True)
    else:
        assert continuation_attempt >= 1, 'continuation attempt must be positive'
        parent = base if continuation_attempt == 1 else base/f'continuation_attempt_{continuation_attempt-1}'
        old_claim = load(parent/'launch_claim.json')
        previous = load(parent/'job.json')
        assert previous['phase'] == 'failed_preserved_without_retry', 'not an unstarted failed-review controller'
        for record in (old_claim, previous):
            assert record['candidate'] == candidate
            for field in ('source', 'prepared', 'output'):
                assert Path(record[field]).resolve() == profile[field].resolve(), ('wrong continuation binding', field)
        for field in ('controller_pid', 'controller_created_epoch'):
            assert old_claim[field] == previous[field]
        assert not controller_alive(previous), 'prior controller has not exited'
        assert Path(previous['source_review']).resolve() != review_file.resolve(), 'preserve the old failed review; supply a new review path'
        output = profile['output']
        assert not (output/'impl').exists() and not (output/'tool_runtime.json').exists(), 'physical attempt already exists; no continuation permitted'
        # No waiting/claim is created against a failed or incomplete new proof.
        check_review(profile, review_file)
        check_source(profile)
        control = base/f'continuation_attempt_{continuation_attempt}'
        control.mkdir(exist_ok=False)
    me = psutil.Process()
    state = dict(candidate=candidate, controller_pid=me.pid, controller_created_epoch=me.create_time(),
                 source=str(profile['source']), prepared=str(profile['prepared']),
                 output=str(profile['output']), source_review=str(review_file),
                 total_limit_s=0, silent_limit_s=0, depends_on_PNR31=False,
                 diagnostic_only=True, programmed=False, deployment_accepted=False)
    if previous is not None:
        state.update(continuation_attempt=continuation_attempt, previous_failed_controller_directory=str(parent),
                     previous_failed_controller_job=previous, resumes_unstarted_input_review_only=True)
        with (parent/'continuation_child_claim.json').open('x', encoding='utf-8') as claim:
            json.dump(dict(state, new_controller_directory=str(control)), claim, ensure_ascii=False, indent=2)
    with (control/'launch_claim.json').open('x', encoding='utf-8') as claim:
        json.dump(state, claim, ensure_ascii=False, indent=2)
    return control, state


def run(candidate, review_file, continuation_attempt=None):
    profile = PROFILES[candidate]
    control, state = claim_controller(candidate, review_file, continuation_attempt)

    def save(phase, **values):
        state.update(phase=phase, updated_at=datetime.now().astimezone().isoformat(), **values)
        write(control/'job.json', state)
    try:
        save('waiting_for_actual_completed_SYN_review')
        while not review_file.exists():
            time.sleep(profile['interval'])
        check_review(profile, review_file)
        check_source(profile)
        save('actual_SYN_review_passed_preparing_unique_PNR')
        prepare(profile, review_file)
        if profile['offset']:
            save('staggering_memory_observation', stagger_seconds=profile['offset'])
            time.sleep(profile['offset'])
        wait_memory(profile, save, 'waiting_for_memory_admission')
        check_source(profile)
        validate_output(profile, review_file)
        # Recheck immediately after input I/O; do not use an older admission sample.
        wait_memory(profile, save, 'rechecking_memory_immediately_before_launch')
        save('starting_unique_pure_RTL_PNR')
        runtime = run_tool([str(TOOL), 'run.tcl'], profile['output'], max_runtime_s=0,
                           max_no_progress_s=0, capture_system_telemetry=True)
        save('tool_exited_collecting_actual_result', runtime=runtime)
        command = profile['output']/'impl/pnr/cmd.do'
        if not command.exists():
            save('failed_before_PNR_command_report', actual_PNR_options_verified=False)
            return 1
        normalize = lambda text, root: text.replace(str(root), '<BUILD>').replace(root.as_posix(), '<BUILD>').split()
        same_options = normalize(command.read_text(), profile['output']) == normalize(
            (BASELINE/'impl/pnr/cmd.do').read_text(), BASELINE)
        finalize_gate(profile['output'])
        if not (profile['output']/'impl/pnr/g2_top.rpt.txt').is_file():
            save('failed_required_PNR_report_unavailable',
                 actual_PNR_options_equal_to_PNR27=same_options,
                 report_collected=(profile['output']/'g2_gate.json').exists())
            return 1
        cdc = physical_connections(profile, save)
        save('completed_diagnostic_requires_actual_timing_review',
             runtime=runtime, actual_PNR_options_equal_to_PNR27=same_options,
             pure_26_physical_connections_passed=cdc,
             report_collected=(profile['output']/'g2_gate.json').exists())
        return int(not same_options or not cdc or runtime['cancelled'] or runtime['tool_exit'] != 0)
    except Exception as error:
        save('failed_preserved_without_retry', error_type=type(error).__name__, error=str(error))
        return 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate', required=True, choices=tuple(PROFILES))
    parser.add_argument('--review', required=True, type=Path)
    parser.add_argument('--prepare-only', action='store_true')
    parser.add_argument('--continuation-attempt', type=int,
                        help='explicit append-only continuation of an exited failed input-review controller; never repeats physical work')
    args = parser.parse_args()
    review = args.review.resolve()
    if args.prepare_only:
        assert args.continuation_attempt is None, '--continuation-attempt belongs to a controller run, not preparation'
        print(prepare(PROFILES[args.candidate], review))
        return 0
    return run(args.candidate, review, args.continuation_attempt)


if __name__ == '__main__':
    raise SystemExit(main())
