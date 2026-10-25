"""Reuse the reviewed fusion/pose SYN for one unbounded implementation.

Preparation copies frozen inputs only. A run requires all source, functional
and CDC reviews to pass, and preserves PNR27's options and 20/10 ns clocks.
Neither implementation exit status nor this runner authorizes programming.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import time

import psutil

from bounded_tool import run_tool
from audit_cdc_first_stages import sdc_fragment
from collect_pnr_result import finalize_gate
from retry_frozen_synthesis import memory_status


ROOT = Path('F:/nav_integration_g2_20261023')
SOURCE = Path('F:/nav_integration_g2_20261022/directed_fusion_pose_unsigned_coverage_effective50')
REFERENCE = ROOT/'baseline_rep1_replay_pnr27'
OUTPUT = ROOT/'baseline_fusion_pose_timing_pnr29'
SOURCE_REVIEW = ROOT/'baseline_existing_fusion_pose_review_20261025/reuse_review.json'
CDC_REVIEW = ROOT/'fusion_pose_unsigned_pre_pnr_review1/review_result.json'
SDC = ROOT/'fusion_pose_unsigned_pre_pnr_review1/g2_lidar26_candidate.sdc'
TOOL = Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe')


def load(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def write(path, value):
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n',
                         encoding='utf-8', newline='\n')
    temporary.replace(path)


def collector_reference():
    # The generic audit() reader accepts flat physical VO syntax. The reviewed
    # source is synthesis VG; reuse the actual VG reader's checked records.
    reviewed = load(CDC_REVIEW)
    paths = reviewed['paths']
    assert reviewed['passed'] and len(paths) == 26 and all(row['passed'] for row in paths)
    reference = dict(netlist=str(OUTPUT/'g2_top.vg'), passed=True, errors=[], paths=paths,
                     expected_paths=26, source_review=str(CDC_REVIEW),
                     scope='Reviewed actual synthesis endpoints; physical constraints and connections remain to be checked.')
    commands = lambda value: {line.strip() for line in value.splitlines()
                              if line.lstrip().startswith('set_')}
    assert commands(sdc_fragment(reference)) == commands(SDC.read_text(encoding='utf-8'))
    return reference


def reviewed_inputs():
    source_review, cdc_review = load(SOURCE_REVIEW), load(CDC_REVIEW)
    assert source_review['passed'] is True, 'source/proof review has not passed'
    assert cdc_review['passed'] is True, 'netlist/CDC review has not passed'
    assert Path(cdc_review['source_frozen_netlist']).resolve() == (SOURCE/'impl/gwsynthesis/g2_top.vg').resolve()
    assert cdc_review['hold_only_false_paths'] == cdc_review['explicit_max_delays'] == 26
    assert Path(cdc_review['candidate_full_sdc']).resolve() == SDC.resolve()
    vg = SOURCE/'impl/gwsynthesis/g2_top.vg'
    assert vg.read_bytes() == Path(cdc_review['reviewed_netlist_copy']).read_bytes()
    assert SDC.read_bytes() == Path(cdc_review['reviewed_sdc_copy']).read_bytes()
    assert (SOURCE/'g2.cst').read_bytes() == (REFERENCE/'g2.cst').read_bytes()
    assert (SOURCE/'g2.sdc').read_bytes() == (REFERENCE/'g2.sdc').read_bytes()
    sdc = SDC.read_text(encoding='utf-8')
    clocks = re.findall(r'(?m)^create_clock -name (\S+) -period (\S+) ', sdc)
    assert clocks == [('clk50', '20'), ('rgmii_rxc', '10')], clocks
    assert len(re.findall(r'(?m)^set_false_path -from .+ -hold$', sdc)) == 26
    assert len(re.findall(r'(?m)^set_max_delay ', sdc)) == 26
    active = [line.strip() for line in sdc.splitlines()
              if line.strip() and not line.lstrip().startswith('#')]
    assert len(active) == 54, 'unexpected additional timing commands'
    assert all(line.startswith(('create_clock ', 'set_false_path -from ', 'set_max_delay '))
               for line in active)
    expected = [line.strip() for line in (REFERENCE/'g2.sdc').read_text(encoding='utf-8').splitlines()
                if line.strip() and not line.lstrip().startswith('#')]
    pairs = cdc_review['audited_constraints']
    assert len(pairs) == 26
    assert len({(row['from_pin'], row['to_pin']) for row in pairs}) == 26
    for row in pairs:
        endpoints = '-from [get_pins {'+row['from_pin']+'}] -to [get_pins {'+row['to_pin']+'}]'
        assert float(row['max_delay_ns']) in (8.0, 18.0)
        expected.extend(['set_false_path '+endpoints+' -hold',
                         'set_max_delay '+endpoints+' '+format(float(row['max_delay_ns']), '.3f')])
    assert active == expected, 'SDC endpoints or budgets differ from the actual netlist audit'
    original = (REFERENCE/'run.tcl').read_text(encoding='utf-8')
    tcl = original.replace(REFERENCE.as_posix(), OUTPUT.as_posix())
    assert REFERENCE.as_posix() not in tcl
    assert tcl.count('run pnr') == 1 and 'run syn' not in tcl
    return {'g2_top.vg': vg.read_bytes(),
            'g2.cst': (REFERENCE/'g2.cst').read_bytes(),
            'g2.sdc': SDC.read_bytes()}, tcl


def prepare():
    inputs, tcl = reviewed_inputs()
    OUTPUT.mkdir(exist_ok=False)
    for name, data in inputs.items():
        (OUTPUT/name).write_bytes(data)
    (OUTPUT/'run.tcl').write_text(tcl, encoding='utf-8', newline='\n')
    for name, path in [('source_review.json', SOURCE_REVIEW), ('netlist_cdc_review.json', CDC_REVIEW)]:
        shutil.copy2(path, OUTPUT/name)
    write(OUTPUT/'cdc_reference_audit.json', collector_reference())
    write(OUTPUT/'manifest.json', dict(
        source=str(SOURCE), source_manifest=load(SOURCE/'manifest.json'),
        source_review=str(SOURCE_REVIEW), netlist_cdc_review=str(CDC_REVIEW),
        frozen_netlist=True, reused_completed_synthesis=True,
        option_reference=str(REFERENCE), original_core_clock_mhz=50,
        original_receive_clock_mhz=100, control_125Hz_changed=False,
        max_runtime_minutes=0, max_no_progress_minutes=0,
        diagnostic_only=True, programmed=False, deployment_accepted=False,
        scope='Measured timing candidate; full physical and functional acceptance still required.'))
    return OUTPUT


def validate():
    inputs, tcl = reviewed_inputs()
    for name, data in inputs.items():
        assert (OUTPUT/name).read_bytes() == data, ('prepared input changed', name)
    assert (OUTPUT/'run.tcl').read_text(encoding='utf-8') == tcl
    assert load(OUTPUT/'source_review.json') == load(SOURCE_REVIEW)
    assert load(OUTPUT/'netlist_cdc_review.json') == load(CDC_REVIEW)
    assert load(OUTPUT/'cdc_reference_audit.json') == collector_reference()
    assert not (OUTPUT/'tool_runtime.json').exists(), 'refuse repeated implementation'
    assert not (OUTPUT/'impl').exists(), 'refuse reused implementation directory'


def run(wait_for_pnr28=False):
    validate()
    me = psutil.Process()
    state = dict(controller_pid=me.pid, controller_created_epoch=me.create_time(),
                 source=str(SOURCE), output=str(OUTPUT), total_limit_s=0,
                 silent_limit_s=0, programmed=False, deployment_accepted=False)
    # An exclusive, permanent claim also prevents two memory-waiting runners
    # from launching the same PNR before tool_runtime.json has been written.
    with (OUTPUT/'launch_claim.json').open('x', encoding='utf-8') as claim:
        json.dump(state, claim, ensure_ascii=False, indent=2)

    def save(phase, **values):
        state.update(phase=phase, updated_at=datetime.now().astimezone().isoformat(), **values)
        write(OUTPUT/'timing_job.json', state)

    if wait_for_pnr28:
        dependencies = {54744: 1792934258.851413, 44808: 1792934258.5659103}
        while True:
            alive = []
            for pid, created in dependencies.items():
                try:
                    process = psutil.Process(pid)
                    if abs(process.create_time()-created) < 0.01:
                        alive.append(pid)
                except psutil.NoSuchProcess:
                    pass
            save('waiting_for_owned_PNR28_natural_exit', dependency_pids=alive,
                 wait_has_no_deadline=True)
            if not alive:
                break
            time.sleep(10)
        prior_runtime = load(ROOT/'uart_host_no_sfp_clk25_diag_pnr28/tool_runtime.json')
        assert prior_runtime['pid'] == 54744 and prior_runtime.get('ended_at')
        assert prior_runtime['cancelled'] is False, 'review an interrupted diagnostic before continuing'
        assert prior_runtime['max_runtime_s'] == prior_runtime['max_no_observable_progress_s'] == 0
        state['prior_PNR28_final_runtime'] = prior_runtime

    while True:
        memory = memory_status()
        save('waiting_for_memory_admission', available_physical_bytes=memory.avail_phys,
             available_commit_bytes=memory.avail_page,
             minimum_physical_bytes=2*1024**3, minimum_commit_bytes=4*1024**3)
        if memory.avail_phys >= 2*1024**3 and memory.avail_page >= 4*1024**3:
            break
        time.sleep(10)
    validate()
    save('starting_reviewed_timing_implementation')
    runtime = run_tool([str(TOOL), 'run.tcl'], OUTPUT, max_runtime_s=0,
                       max_no_progress_s=0, capture_system_telemetry=True)
    save('tool_exited_collecting_actual_result', runtime=runtime)
    command = OUTPUT/'impl/pnr/cmd.do'
    if not command.exists():
        save('failed_before_PNR_command_report', actual_PNR_options_verified=False,
             report_collected=False)
        return 1
    actual = command.read_text(encoding='utf-8')
    original = (REFERENCE/'impl/pnr/cmd.do').read_text(encoding='utf-8')
    def normalized(text, root):
        return text.replace(str(root), '<BUILD>').replace(root.as_posix(), '<BUILD>').split()
    same_options = normalized(actual, OUTPUT) == normalized(original, REFERENCE)
    if (OUTPUT/'impl/pnr/g2_top.rpt.txt').exists():
        finalize_gate(OUTPUT)
    save('completed_requires_actual_timing_review', runtime=runtime,
         actual_PNR_options_equal_to_routed_reference=same_options,
         report_collected=(OUTPUT/'g2_gate.json').exists())
    return int(not same_options)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--prepare-only', action='store_true')
    parser.add_argument('--wait-for-pnr28', action='store_true',
                        help='wait for the identified diagnostic and its collector; no time limit')
    args = parser.parse_args()
    if args.prepare_only:
        print(prepare())
        return 0
    return run(args.wait_for_pnr28)


if __name__ == '__main__':
    raise SystemExit(main())
