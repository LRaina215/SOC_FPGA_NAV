"""Two distinct frozen-netlist native-width comparisons, admitted independently.

PNR45 changes only the documented SDP32/36 splitting option versus PNR44.
PNR46 applies that same storage representation change to routed PNR35. The
same SDC also requests expanded reports. Neither performs synthesis or edits
RTL, INIT, capacity, clocks, global requests, or the existing CDC contract.
"""
import argparse
from datetime import datetime
import os
from pathlib import Path
import shutil
import time

import psutil
from audit_netlist_memories import inventory
from bounded_tool import run_tool
from check_setup_endpoint_coverage import scan_report
from collect_pnr_result import finalize_gate
from retry_frozen_synthesis import memory_status
from review_baseline_fusion_action_width_syn import load, metadata, same, save

BASE = Path('F:/nav_integration_g2_20261023')
REFERENCE = BASE/'baseline_fusion32_CDC52_pnr35'
TOOL = Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe')
GIB = 1024**3
OLD = 'set_option -convert_sdp32_36_to_sdp16_18 1'
NEW = 'set_option -convert_sdp32_36_to_sdp16_18 0'
REPORT = 'report_timing -setup -max_paths 5000 -max_common_paths 1'


def prepare(profile, output):
    bindings = []
    if profile == 'DSP_bias':
        import run_registered_dsp_round_bias_pnr as qualified
        run = qualified.run
        review, sdc, tcl = run.source_check()
        assert review['passed'] and len(review['paths']) == 26
        source = BASE/'baseline_fusion_DSP_round_bias_syn32'
        comparison = BASE/'baseline_fusion_DSP_round_bias_pnr44'
        vg = source/'impl/gwsynthesis/g2_top.vg'
        assert same(vg, comparison/'g2_top.vg')
        assert load(comparison/'g2_gate.json')['unrouted_nets'] == 1426
        bindings += [run.REVIEW, run.MAPPING]
        old_output = run.OUTPUT
    else:
        source = BASE/'baseline_fusion_only_syn23'
        comparison = REFERENCE
        vg = source/'impl/gwsynthesis/g2_top.vg'
        actual = load(BASE/'fusion32_CDC52_final_review1/review_result.json')
        assert actual['passed'] and actual['routing_passed'] and actual['hold_violations'] == 0
        assert actual['hold_only_false_paths_active'] == actual['max_delay_paths_active'] == 26
        for row in actual['evidence_metadata']:
            assert metadata(row['path']) == row
        assert same(vg, comparison/'g2_top.vg')
        assert load(source/'job.json')['phase'] == 'synthesis_complete'
        sdc = (BASE/'baseline_fusion_action_preview_pnr39/g2.sdc').read_bytes()
        assert sdc == ((REFERENCE/'g2.sdc').read_bytes()+
                       b'\n# Additional setup report only.\n'+REPORT.encode()+b'\n')
        tcl = (REFERENCE/'run.tcl').read_bytes()
        bindings += [BASE/'fusion32_CDC52_final_review1/review_result.json']
        old_output = REFERENCE
    # Inventory the actual completed source, including the read-only wide ROM.
    ram = inventory(source)
    assert ram['expanded_bsram_count'] == 314 and ram['wide32_36_count'] == 18
    assert ram['predicted_split_bsram'] == 332
    assert same(source/'g2.cst', REFERENCE/'g2.cst')
    tcl = tcl.replace(str(old_output).encode(), str(output).encode()).replace(
        old_output.as_posix().encode(), output.as_posix().encode())
    assert tcl.count(OLD.encode()) == 1
    tcl = tcl.replace(OLD.encode(), NEW.encode(), 1)
    assert tcl.count(NEW.encode()) == 1
    assert len([x for x in tcl.decode().splitlines() if 'add_file' in x and '.sdc' in x]) == 1
    assert not output.exists()
    output.mkdir()
    shutil.copyfile(vg, output/'g2_top.vg')
    shutil.copyfile(REFERENCE/'g2.cst', output/'g2.cst')
    (output/'g2.sdc').write_bytes(sdc)
    (output/'run.tcl').write_bytes(tcl)
    cdc = load(REFERENCE/'cdc_reference_audit.json')
    assert cdc['passed'] and len(cdc['paths']) == 26
    cdc.update(netlist=str(output/'g2_top.vg'),
               scope='Same frozen netlist CDC contract; re-audit physical connectivity and all effective exceptions after native-width mapping.')
    save(output/'cdc_reference_audit.json', cdc)
    contract = Path(__file__).resolve().parents[2]/'docs/4_design/evidence/g2_20261022/native_bsram_contract.json'
    finite = load(contract)
    assert finite['pass_'] and finite['cycles'] == 8192
    assert finite['same_address_collision_excluded']
    save(output/'manifest.json', dict(
        source=str(source), source_manifest=load(source/'manifest.json'), comparison=str(comparison),
        reference=str(REFERENCE), profile=profile, frozen_netlist=True, reused_completed_synthesis=True,
        only_implementation_change='native32/36 instead of automatic split into16/18',
        wide_split=0, source_BSRAM=314, source_wide_leaves=18,
        predicted_physical_BSRAM=314, predicted_saving_vs_split=18,
        physical_RAM_prediction_not_measurement=True,
        no_RTL_INIT_capacity_clock_global_request_or_CDC_exception_changes=True,
        same_single_SDC_as_PNR44=True, max_runtime_minutes=0, max_no_progress_minutes=0,
        diagnostic_only=True, programmed=False, deployment_accepted=False))
    proof = dict(passed=True, source=str(source), comparison=str(comparison),
                 no_new_synthesis=True, no_RTL_or_netlist_delta=True,
                 predicted_RAM314_vs332=True, resource_saving_not_routing_proof=True,
                 prior_different_netlist_split0_314_failed502_split1_332_failed411=True,
                 prior_failure_does_not_prove_current_outcome=True,
                 existing_functional_model_proof_scope=finite,
                 native_collision_and_physical_timing_acceptance_not_claimed=True,
                 source_inventory=ram, new_global_requests=0, added_CDC_exceptions=0,
                 bindings=[metadata(p) for p in bindings+[vg, REFERENCE/'g2.cst',
                    REFERENCE/'g2.sdc', REFERENCE/'run.tcl', contract,
                    Path(__file__)]] ,
                 prepared_bindings=[metadata(output/n) for n in
                    ['g2_top.vg','g2.cst','g2.sdc','run.tcl','cdc_reference_audit.json']])
    save(output/'preflight.json', proof)
    return proof


def main(profile):
    label, stem = (('PNR45', 'baseline_fusion_DSP_bias_native') if profile == 'DSP_bias'
                   else ('PNR46', 'baseline_fusion_anchor_native'))
    output = BASE/(stem+'_pnr'+label[3:])
    control = BASE/(stem+'_one_shot')
    control.mkdir(exist_ok=False)
    me = psutil.Process(); me.nice(psutil.NORMAL_PRIORITY_CLASS)
    me.cpu_affinity(list(range(psutil.cpu_count())))
    state = dict(controller_pid=me.pid, controller_created_epoch=me.create_time(),
                 output=str(output), label=label, profile=profile,
                 total_limit_s=0, silent_limit_s=0, programmed=False, deployment_accepted=False)
    def status(phase, **fields):
        state.update(phase=phase, updated_at=datetime.now().astimezone().isoformat(), **fields)
        save(control/'job.json', state)
    with (control/'launch_claim.json').open('x') as file:
        import json
        json.dump(state, file)
    try:
        status('preparing_unique_native_width_comparison')
        proof = prepare(profile, output)
        def wait_memory():
            while True:
                m = memory_status()
                status('waiting_for_actual_memory_admission', available_physical_bytes=m.avail_phys,
                       available_commit_bytes=m.avail_page, minimum_physical_bytes=2*GIB,
                       minimum_commit_bytes=4*GIB)
                if m.avail_phys >= 2*GIB and m.avail_page >= 4*GIB:
                    return
                time.sleep(13)
        wait_memory()
        for row in proof['bindings']+proof['prepared_bindings']:
            assert metadata(row['path']) == row
        assert not (output/'tool_runtime.json').exists() and not (output/'impl').exists()
        status('starting_unique_'+label)
        rt = run_tool([str(TOOL), 'run.tcl'], output, max_runtime_s=0,
                      max_no_progress_s=0, capture_system_telemetry=True)
        status('tool_exited_collecting', runtime=rt)
        wait_memory(); finalize_gate(output)
        gate = load(output/'g2_gate.json')
        normalize = lambda p, r: p.read_text().replace(str(r), '<BUILD>').replace(r.as_posix(), '<BUILD>').split()
        expected = normalize(REFERENCE/'impl/pnr/cmd.do', REFERENCE)
        expected.remove('-convert_sdp32_36_to_sdp16_18')
        actual = normalize(output/'impl/pnr/cmd.do', output)
        checks = dict(only_actual_command_delta_is_removal_of_width_split=(actual == expected),
                      device_cfg_identical=same(output/'impl/pnr/device.cfg', REFERENCE/'impl/pnr/device.cfg'))
        tr = output/'impl/pnr/g2_top.tr'
        if tr.exists(): save(output/'actual_setup_endpoint_coverage.json', scan_report(tr))
        status('routed_requires_full_independent_review' if gate['routing_passed'] else
               'physical_failure_preserved_no_retry', runtime=rt,
               routing_passed=gate['routing_passed'], unrouted=gate['unrouted_nets'],
               actual_options_checks=checks, TR_available=tr.exists(), gate=str(output/'g2_gate.json'))
        return int(rt['tool_exit'] != 0 or not gate['routing_passed'] or not all(checks.values()))
    except Exception as error:
        import traceback
        status('failed_preserved_no_retry', error=str(error), traceback=traceback.format_exc())
        return 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--profile', choices=['DSP_bias', 'routed_anchor'], required=True)
    raise SystemExit(main(parser.parse_args().profile))
