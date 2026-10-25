"""Explicitly resume a PNR wrapper which stopped after saved settings only.

Default operation prepares an external, reviewable plan. It does not invoke
Gowin. --execute-plan is a separate, explicit action which repeats every check
before starting the first real PNR, with both time limits zero. Original
TimeoutExpired logs and input files remain intact. This is not a retry path for
an actual synthesis, placement or routing failure.
"""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import re
import sys
import time
import xml.etree.ElementTree as ET

import psutil

from parallel_trim_synthesis import (file_metadata, load, memory_status,
                                     validate_ready_cache, write)

ROOT = Path(__file__).resolve().parents[2]
TOOL = Path(r'D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe')
DEVICE = 'GW5AT-LV138PG484AC1/I0'
REPORTS = ('report_route_congestion -max_grids 200',
           'report_high_fanout_nets -max_nets 100 -min_fanout 100')
OPTIONS = dict(top_module='g2_top', output_base_name='g2_top', global_freq='50.0',
    replicate_resources='1', convert_sdp32_36_to_sdp16_18='1',
    gen_verilog_sim_netlist='1', place_option='1', route_option='2',
    timing_driven='1', **{'use_'+x+'_as_gpio': '1' for x in
        ('sspi', 'mspi', 'ready', 'done', 'i2c', 'cpu')},
    show_all_warn='1', gen_posp='1')
ALLOWED_FILES = {'g2_top.vg', 'g2.cst', 'g2.sdc', 'ip/serdes.csr',
    'cdc_reference_audit.json', 'sfp_cdc_reference.json', 'preflight.log',
    'preflight.tcl', 'effective_options.tcl'}


def same_bytes(a, b):
    """Bounded-memory byte comparison; deliberately no digest/hashing."""
    assert a.is_file() and b.is_file() and a.stat().st_size == b.stat().st_size, (a, b)
    with a.open('rb') as x, b.open('rb') as y:
        while True:
            left, right = x.read(1024*1024), y.read(1024*1024)
            assert left == right, ('frozen bytes differ', str(a), str(b))
            if not left:
                return


def expected_settings(out):
    lines = ['set_device -device_version C '+DEVICE,
        'set_csr {'+(out/'ip/serdes.csr').as_posix()+'}']
    lines += ['add_file -type '+kind+' {'+(out/name).as_posix()+'}'
              for kind, name in (('netlist', 'g2_top.vg'), ('cst', 'g2.cst'),
                                  ('sdc', 'g2.sdc'))]
    lines += ['set_option -'+name+' '+value for name, value in OPTIONS.items()]
    return lines


def verify_no_owned_process(out, wrapper):
    observed = []
    try:
        p = psutil.Process(wrapper['pid'])
        assert abs(p.create_time()-wrapper['created_epoch']) >= .1, (
            'original PNR wrapper is still alive', wrapper['pid'])
    except psutil.NoSuchProcess:
        pass
    out_created = out.stat().st_ctime
    for p in psutil.process_iter(['pid', 'name', 'cmdline', 'create_time']):
        try:
            command = p.info['cmdline'] or []
            if any('pnr_from_netlist.py' in part for part in command):
                assert not any(str(out).lower() == part.lower() for part in command), (
                    'PNR wrapper still targets this directory', p.pid)
            if (p.info['name'] or '').lower() != 'gw_sh.exe':
                continue
            if p.info['create_time'] < out_created-2:
                # An older, unrelated settings session is not ours.
                continue
            directory = Path(p.cwd()).resolve()
            observed.append(dict(pid=p.pid, created_epoch=p.create_time(),
                                 directory=str(directory), command=command))
            assert directory != out, ('settings/build process still owns target', p.pid)
        except psutil.NoSuchProcess:
            pass
    return observed


def validate(context):
    source, out, log, cache, registry = [Path(context[k]).resolve()
        for k in ('source', 'out', 'wrapper_log', 'cache', 'registry')]
    profile = next(p for p in load(registry)['profiles'] if p['id'] == context['profile'])
    assert Path(profile['parallel_synthesis_cache_job']).resolve() == cache
    assert Path(profile['PNR']).resolve() == out
    verified = validate_ready_cache(cache, profile)
    result = load(verified['pre_pnr_checks'])
    assert result['passed'] and result['strict_targets'] == 14 and result['true_clocks'] == 4
    assert result['known_constraints'] == 154 and result['hold_only'] == result['max_delay'] == 77
    assert result['unproved_aliases'] == 0 and not result['deployment_accepted']
    actual_source = Path(verified['synthesis']).resolve()
    same_bytes(source/'impl/gwsynthesis/g2_top.vg', actual_source/'impl/gwsynthesis/g2_top.vg')
    assert out.is_dir() and source.is_dir() and source != out
    actual_files = {p.relative_to(out).as_posix() for p in out.rglob('*') if p.is_file()}
    assert actual_files == ALLOWED_FILES, ('target is not a settings-only site',
        sorted(actual_files-ALLOWED_FILES), sorted(ALLOWED_FILES-actual_files))
    assert not any((out/name).exists() for name in
        ('run.tcl', 'manifest.json', 'tool_runtime.json', 'impl', 'gw_sh.log'))
    others = verify_no_owned_process(out, context['previous_wrapper'])
    raw_log = log.read_text(encoding='utf-8', errors='replace')
    assert 'preflight.py' in raw_log and 'subprocess.TimeoutExpired:' in raw_log
    assert "'preflight.tcl']' timed out after 60.0 seconds" in raw_log
    preflight = (out/'preflight.log').read_text(encoding='utf-8', errors='replace')
    assert not re.search(r'\b(?:ERROR|FATAL)\b', preflight)
    assert 'GW5AT-138C' in preflight and DEVICE in preflight
    same_bytes(source/'impl/gwsynthesis/g2_top.vg', out/'g2_top.vg')
    for relative in ('g2.cst', 'ip/serdes.csr', 'cdc_reference_audit.json', 'sfp_cdc_reference.json'):
        same_bytes(source/relative, out/relative)
    sdc = (source/'g2.sdc').read_text(encoding='utf-8')
    assert all(sdc.splitlines().count(c) <= 1 for c in REPORTS)
    missing = [c for c in REPORTS if c not in sdc.splitlines()]
    if missing:
        sdc += '\n# Detailed routing evidence; constraints unchanged.\n'+'\n'.join(missing)+'\n'
    assert (out/'g2.sdc').read_bytes() == sdc.encode('utf-8'), 'SDC differs beyond original report requests'
    project = ET.parse(source/'impl/gwsynthesis/g2_top.prj').getroot()
    original = {x.get('type'): x.get('value') for x in project.findall('./OptionList/Option')}
    assert float(original['global_freq']) == 50
    same_bytes(source/'effective_options.tcl', actual_source/'effective_options.tcl')
    # The original wrapper explicitly requested replicate=1 and wide-split=1
    # for PNR. SYN's saved values are 0; never conflate the two stages. The
    # exact preflight command below binds the requested PNR overrides.
    settings = expected_settings(out)
    expected_preflight = '\n'.join(settings+[
        'saveto -all_options {'+(out/'effective_options.tcl').as_posix()+'}', 'exit'])+'\n'
    assert (out/'preflight.tcl').read_bytes() == expected_preflight.encode('utf-8'), 'original settings command changed'
    dump = (out/'effective_options.tcl').read_text(encoding='utf-8')
    pairs = re.findall(r'^set_option\s+-(\S+)\s+(.+?)\s*$', dump, re.M)
    assert len(pairs) == len(dict(pairs)), 'duplicate saved option'
    effective = dict(pairs)
    for key, expected in OPTIONS.items():
        assert key in effective, ('saved option missing', key)
        if key == 'global_freq':
            assert float(effective[key]) == float(expected)
        else:
            assert effective[key] == expected, ('saved option differs', key, effective[key], expected)
    # This Gowin saveto output has no trailing newline. Its last complete
    # option is checked explicitly, rather than rejecting a valid saved file.
    assert dump.splitlines()[-1] == 'set_option -co-place_io_registers 0'
    assert 'set_device '+DEVICE+' -device_version C' in dump.splitlines()
    assert 'set_csr '+(out/'ip/serdes.csr').as_posix()+'\n' in dump
    for kind, name in (('netlist', 'g2_top.vg'), ('cst', 'g2.cst'), ('sdc', 'g2.sdc')):
        assert 'add_file -type '+kind+' "'+(out/name).as_posix()+'"\n' in dump
    bindings = [source/'manifest.json', source/'impl/gwsynthesis/g2_top.prj',
        source/'effective_options.tcl', source/'impl/gwsynthesis/g2_top.vg', log,
        cache, Path(verified['pre_pnr_checks']), Path(verified['reviewer_snapshot'])]
    bindings += [out/name for name in sorted(ALLOWED_FILES)]
    metadata = [file_metadata(p) for p in bindings]
    return dict(passed=True, context=context, input_metadata=metadata,
        actual_saved_requested_options={k: effective[k] for k in OPTIONS},
        original_timeout_preserved=True, actual_PNR_previously_started=False,
        no_settings_or_wrapper_process_remains=True, other_current_Gowin_processes=others,
        ready_cache_artifact_mode=verified.get('artifact_mode', 'completed_synthesis'),
        precheck_source=str(actual_source), strict_targets=14, true_clocks=4,
        known_constraints=154, unproved_aliases=0,
        run_tcl='\n'.join(settings+['run pnr', 'exit'])+'\n',
        PNR_total_seconds=0, PNR_silent_seconds=0,
        minimum_start_physical_bytes=2*1024**3,
        minimum_start_commit_bytes=4*1024**3,
        programmed=False, deployment_accepted=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--resume-settings-only', action='store_true', required=True)
    parser.add_argument('--execute-plan', type=Path)
    parser.add_argument('--source', type=Path)
    parser.add_argument('--out', type=Path)
    parser.add_argument('--wrapper-log', type=Path)
    parser.add_argument('--cache', type=Path)
    parser.add_argument('--registry', type=Path)
    parser.add_argument('--profile')
    parser.add_argument('--previous-wrapper-pid', type=int)
    parser.add_argument('--previous-wrapper-created-epoch', type=float)
    parser.add_argument('--plan-dir', type=Path)
    args = parser.parse_args()
    if args.execute_plan:
        plan_path = args.execute_plan.resolve()
        plan = load(plan_path)
        assert plan['passed'] and plan['actual_PNR_previously_started'] is False
        current = validate(plan['context'])
        assert current == plan['validation'], 'saved plan or bound inputs changed'
        folder = plan_path.parent
        lock = folder/'execution_started.lock'
        fd = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
        os.write(fd, str(os.getpid()).encode('ascii')); os.close(fd)
        status = dict(phase='waiting_for_first_PNR_memory', pid=os.getpid(),
            created_epoch=psutil.Process().create_time(), PNR_total_seconds=0,
            PNR_silent_seconds=0, programmed=False)
        while True:
            memory = memory_status()
            if memory.avail_phys >= 2*1024**3 and memory.avail_page >= 4*1024**3:
                break
            status.update(available_physical_bytes=memory.avail_phys,
                available_commit_bytes=memory.avail_page)
            write(folder/'execution_status.json', status); time.sleep(15)
        assert validate(plan['context']) == current, 'inputs changed while waiting'
        out = Path(plan['context']['out'])
        (out/'run.tcl').write_text(current['run_tcl'], encoding='utf-8', newline='\n')
        write(out/'settings_only_recovery.json', dict(plan=str(plan_path),
            validation=current, original_timeout_preserved=True))
        source = Path(plan['context']['source'])
        write(out/'manifest.json', dict(source=str(source), frozen_netlist=True,
            wide_split=1, replicate_resources=1, place_option=1, route_option=2,
            timing_driven=1, max_runtime_minutes=0, max_no_progress_minutes=0,
            effective_options=dict(global_freq_mhz=50, wide_split=1,
                effective_options=str(out/'effective_options.tcl'),
                extra_options={k:int(v) for k,v in OPTIONS.items() if v.isdigit()}),
            source_manifest=load(source/'manifest.json'),
            cdc_reference_audit=str(out/'cdc_reference_audit.json'),
            sfp_cdc_reference=str(out/'sfp_cdc_reference.json'),
            capture_system_telemetry=True, routing_evidence=True,
            routing_report_commands=list(REPORTS), diagnostic_only=False,
            clock_resource_plan=None, original_pnr_command=None,
            settings_only_recovery_plan=str(plan_path), programmed=False))
        status['phase'] = 'first_actual_PNR_running_no_deadline'
        write(folder/'execution_status.json', status)
        from bounded_tool import run_tool
        runtime = run_tool([str(TOOL), 'run.tcl'], out, max_runtime_s=0,
            max_no_progress_s=0, capture_system_telemetry=True)
        from collect_pnr_result import finalize_gate
        gate = finalize_gate(out)
        write(folder/'execution_status.json', dict(phase='first_actual_PNR_collected',
            runtime=runtime, gate=gate, programmed=False, deployment_accepted=False))
        return int(bool(runtime['tool_exit'] or runtime['cancelled'] or not gate))
    required = ('source', 'out', 'wrapper_log', 'cache', 'registry', 'profile',
                'previous_wrapper_pid', 'previous_wrapper_created_epoch', 'plan_dir')
    assert all(getattr(args, name) is not None for name in required), 'all plan inputs are required'
    context = {k: str(getattr(args,k).resolve()) for k in
               ('source','out','wrapper_log','cache','registry')}
    context.update(profile=args.profile, previous_wrapper=dict(pid=args.previous_wrapper_pid,
        created_epoch=args.previous_wrapper_created_epoch))
    result = validate(context)
    folder = args.plan_dir.resolve(); folder.mkdir(parents=True, exist_ok=False)
    plan_path = folder/'resume_plan.json'
    command = [sys.executable, '-X', 'utf8', str(Path(__file__).resolve()),
        '--resume-settings-only', '--execute-plan', str(plan_path)]
    write(plan_path, dict(passed=True, prepared_at=datetime.now().astimezone().isoformat(),
        context=context, validation=result, actual_PNR_previously_started=False,
        PNR_started_by_preparation=False, root_execution_command=command,
        programmed=False, deployment_accepted=False))
    (folder/'proposed_run.tcl').write_text(result['run_tcl'], encoding='utf-8', newline='\n')
    print(json.dumps(dict(passed=True, plan=str(plan_path), starts_tool=False,
        root_execution_command=command, PNR_total_seconds=0, PNR_silent_seconds=0),
        ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
