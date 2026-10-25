"""Run one explicitly requested frozen routing diagnostic, without a deadline.

The baseline case changes output paths only. The clk25 case changes exactly
one SDC period, keeping the actual circuit and global-frequency hint intact.
Neither case can authorize deployment or modify the source build.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import sys
import time

import psutil

from bounded_tool import run_tool
from collect_pnr_result import finalize_gate
from retry_frozen_synthesis import memory_status

BASE = Path('F:/nav_integration_g2_20261023')
CASES = {
    'baseline': (Path('F:/nav_integration_g2_20261022/directed_static_coverage_rep1_effective50'),
                 BASE/'baseline_rep1_replay_pnr27'),
    'clk25': (BASE/'uart_host_no_sfp_pnr26', BASE/'uart_host_no_sfp_clk25_diag_pnr28'),
    'baseline_clk25': (BASE/'baseline_rep1_replay_pnr27', BASE/'baseline27_clk25_diag_pnr30'),
    'baseline_cdc': (BASE/'baseline_rep1_replay_pnr27', BASE/'baseline27_cdc_only_pnr31'),
}
TOOL = Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe')
CROSS_REVIEW = BASE/'pnr27_period40_and_cdc52_independent_review1/review_result.json'


def write(path, value):
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    temporary.replace(path)


def inputs(case):
    source, out = CASES[case]
    result = {}
    for name in ('g2_top.vg', 'g2.cst', 'g2.sdc'):
        result[name] = (source/name).read_bytes()
    if case in ('clk25', 'baseline_clk25'):
        text = result['g2.sdc'].decode('utf-8')
        pattern = r'(?m)^(create_clock -name clk50 -period )(\S+)( .*)$'
        matches = list(re.finditer(pattern, text))
        assert len(matches) == 1 and float(matches[0][2]) == 20
        text = re.sub(pattern, lambda m: m[1]+'40'+m[3], text)
        result['g2.sdc'] = text.encode('utf-8')
    if case in ('baseline_clk25', 'baseline_cdc'):
        review = json.loads(CROSS_REVIEW.read_text(encoding='utf-8'))
        assert review['passed'] and review['branches_independent']
        assert Path(review['source_netlist']).resolve() == (source/'g2_top.vg').resolve()
        assert result['g2_top.vg'] == Path(review['reviewed_netlist_copy']).read_bytes()
        assert result['g2.cst'] == Path(review['reviewed_cst_copy']).read_bytes()
        if case == 'baseline_clk25':
            assert result['g2.sdc'] == Path(review['period40_only_sdc']).read_bytes()
        else:
            result['g2.sdc'] = Path(review['reviewed_sdc_copy']).read_bytes()
            expected = [line.strip() for line in (source/'g2.sdc').read_text(encoding='utf-8').splitlines()
                        if line.strip() and not line.lstrip().startswith('#')]
            assert len(review['audited_constraints']) == 26
            for row in review['audited_constraints']:
                endpoints = '-from [get_pins {'+row['from_pin']+'}] -to [get_pins {'+row['to_pin']+'}]'
                assert float(row['max_delay_ns']) in (8.0,18.0)
                expected.extend(['set_false_path '+endpoints+' -hold',
                                 'set_max_delay '+endpoints+' '+format(float(row['max_delay_ns']),'.3f')])
            active = [line.strip() for line in result['g2.sdc'].decode('utf-8').splitlines()
                      if line.strip() and not line.lstrip().startswith('#')]
            assert active == expected and len(active) == 54
    original = (source/'run.tcl').read_text(encoding='utf-8')
    tcl = original.replace(source.as_posix(), out.as_posix())
    assert tcl != original and source.as_posix() not in tcl
    assert not re.search(r'(?m)^set_csr ', tcl)
    assert len(re.findall(r'(?m)^run pnr\s*$', tcl)) == 1
    return source, out, result, tcl


def prepare(case):
    source, out, expected, tcl = inputs(case)
    out.mkdir(parents=True, exist_ok=False)
    for name, data in expected.items():
        (out/name).write_bytes(data)
    (out/'run.tcl').write_text(tcl, encoding='utf-8', newline='\n')
    for name in ('cdc_reference_audit.json', 'sfp_cdc_reference.json'):
        if (source/name).exists():
            shutil.copy2(source/name, out/name)
    original_manifest = json.loads((source/'manifest.json').read_text(encoding='utf-8'))
    manifest = dict(source=str(source), source_manifest=original_manifest,
                    frozen_netlist=True, diagnostic_only=True, programmed=False,
                    deployment_accepted=False, requested_case=case,
                    max_runtime_minutes=0, max_no_progress_minutes=0,
                    only_change=('output paths' if case == 'baseline' else
                                 'output paths and52 audited first-stage commands; clocks20/10ns unchanged'
                                 if case == 'baseline_cdc' else
                                 'output paths and clk50 SDC period20ns to40ns'),
                    physical_clock_changed=False, control_125Hz_changed=False)
    write(out/'manifest.json', manifest)
    if case in ('baseline_clk25', 'baseline_cdc'):
        shutil.copy2(CROSS_REVIEW, out/'independent_input_review.json')
    if case == 'baseline_cdc':
        from audit_cdc_first_stages import sdc_fragment
        prior = json.loads((BASE/'pnr27_lidar_cdc_hold_candidate1/review_result.json').read_text(encoding='utf-8'))
        assert prior['passed'] and len(prior['paths']) == 26
        reference = dict(netlist=str(out/'g2_top.vg'),passed=True,paths=prior['paths'],expected_paths=26,
                         errors=[],scope='Previously audited exact PNR27 source/physical endpoints; final physical check required')
        commands = lambda value:{line.strip() for line in value.splitlines() if line.lstrip().startswith('set_')}
        assert commands(sdc_fragment(reference)) == commands(expected['g2.sdc'].decode('utf-8'))
        write(out/'cdc_reference_audit.json',reference)
    return out


def validate(case):
    source, out, expected, tcl = inputs(case)
    for name, data in expected.items():
        assert (out/name).read_bytes() == data, ('input differs', name)
    assert (out/'run.tcl').read_text(encoding='utf-8') == tcl
    if case in ('baseline_clk25', 'baseline_cdc'):
        assert (out/'independent_input_review.json').read_bytes() == CROSS_REVIEW.read_bytes()
    assert not (out/'tool_runtime.json').exists(), 'refuse repeated diagnostic'
    assert not (out/'impl').exists(), 'refuse reuse of an implementation directory'
    return source, out


def run(case):
    source, out = validate(case)
    me = psutil.Process()
    state = dict(case=case, source=str(source), output=str(out), controller_pid=me.pid,
                 controller_created_epoch=me.create_time(), total_limit_s=0,
                 silent_limit_s=0, diagnostic_only=True, programmed=False)
    with (out/'launch_claim.json').open('x',encoding='utf-8') as claim:
        json.dump(state,claim,ensure_ascii=False,indent=2)

    def save(phase, **values):
        state.update(phase=phase, updated_at=datetime.now().astimezone().isoformat(), **values)
        write(out/'replay_job.json', state)

    while True:
        memory = memory_status()
        save('waiting_for_memory_admission', available_physical_bytes=memory.avail_phys,
             available_commit_bytes=memory.avail_page,
             minimum_physical_bytes=2*1024**3, minimum_commit_bytes=4*1024**3)
        if memory.avail_phys >= 2*1024**3 and memory.avail_page >= 4*1024**3:
            break
        time.sleep(10)
    validate(case)
    save('starting_requested_frozen_diagnostic')
    runtime = run_tool([str(TOOL), 'run.tcl'], out, max_runtime_s=0,
                       max_no_progress_s=0, capture_system_telemetry=True)
    save('tool_exited_collecting_actual_result',runtime=runtime)
    if runtime.get('cancelled'):
        save('cancelled_requires_review',runtime=runtime,deployment_accepted=False)
        return 1
    if not (out/'impl/pnr/cmd.do').exists():
        save('failed_before_actual_PNR_command',runtime=runtime,gate_collected=False,
             actual_PNR_command_matches_reference=False,deployment_accepted=False)
        return 1
    actual = (out/'impl/pnr/cmd.do').read_text(encoding='utf-8')
    original = (source/'impl/pnr/cmd.do').read_text(encoding='utf-8')
    normalized = lambda text, root: text.replace(str(root), '<BUILD>').replace(root.as_posix(), '<BUILD>').split()
    command_matches = normalized(actual, out) == normalized(original, source)
    gate = finalize_gate(out) if (out/'impl/pnr/g2_top.rpt.txt').exists() else False
    save('completed_requires_model_review', runtime=runtime,
         actual_PNR_command_matches_reference=command_matches,
         gate_collected=(out/'g2_gate.json').exists(),
         acceptance_gate_pass=bool(gate), deployment_accepted=False)
    return int(not command_matches)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('case', choices=list(CASES)+['baseline_pair'])
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args()
    if args.case == 'baseline_pair':
        assert not args.prepare_only, 'prepare the two independent cases explicitly first'
        pair=BASE/'baseline27_single_factor_pair1'
        pair.mkdir(exist_ok=False)
        process=psutil.Process()
        state=dict(controller_pid=process.pid,controller_created_epoch=process.create_time(),
                   cases=['baseline_clk25','baseline_cdc'],mode='serial_memory_admission_each_case',
                   total_limit_s=0,silent_limit_s=0,diagnostic_only=True,programmed=False)
        for case in state['cases']:
            state.update(phase='running_or_waiting_for_memory',current_case=case,
                         updated_at=datetime.now().astimezone().isoformat())
            write(pair/'job.json',state)
            result=run(case)
            state.setdefault('completed_cases',[]).append(case)
            if result:
                state.update(phase='stopped_for_command_or_startup_failure',exit_code=result)
                write(pair/'job.json',state)
                return result
        state.update(phase='completed_both_independent_diagnostics',current_case=None,
                     updated_at=datetime.now().astimezone().isoformat())
        write(pair/'job.json',state)
        return 0
    if args.prepare_only:
        print(prepare(args.case))
        return 0
    return run(args.case)


if __name__ == '__main__':
    raise SystemExit(main())
