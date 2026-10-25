"""Compose three frozen, separately proved changes; never launch a build.

The result remains a source candidate: component proofs do not establish joint
timing, aggregate resource use, real serial behavior or vehicle acceptance.
"""
import argparse
import copy
from datetime import datetime
import difflib
import json
from pathlib import Path
import shutil

from prepare_baseline_action_csa import bind_contract
from prepare_baseline_fusion_retime import hardware_inputs, metadata, relocate, same_bytes, save
from prepare_baseline_fusion_uart_loss import decode_ram, read_elf, inventory
from prepare_fusion_square_pipeline import generate

BASE = Path('F:/nav_integration_g2_20261023')
SOURCE = BASE/'baseline_fusion_only_syn23'
ROUTED = BASE/'baseline_fusion_only_pnr32'
FUSION = BASE/'baseline_fusion_square2_prepared1'
ACTION = BASE/'baseline_fusion_action_CSA_prepared1'
UART = BASE/'baseline_fusion_uart_loss_prepared2'
OUT = BASE/'baseline_fusion_square2_action_CSA_uart_prepared1'
RTL = ['control/imu_fusion.v', 'control/policy_action_map.v']
EXISTING_FW = ['fw/app/nav_app.c', 'fw/drv/hal_fpga.c']
ADDED_FW = ['fw/nav/host_link_guard.c', 'fw/nav/host_link_guard.h']


def load(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def verify_bindings(records):
    for item in records:
        assert metadata(Path(item['path'])) == item, ('component proof input changed', item['path'])


def prepare(out=OUT):
    out = Path(out).resolve()
    assert not out.exists()
    assert load(SOURCE/'job.json')['phase'] == 'synthesis_complete'
    assert load(ROUTED/'g2_gate.json')['routing_passed']
    manifest = load(SOURCE/'manifest.json')
    assert manifest['stage'] == 'syn' and manifest['fusion_norm_pipeline']
    assert not any(manifest.get(key) for key in ('pose_predecode', 'action_map_exact_widths',
                   'action_map_constant_CSA', 'fusion_square_pipeline2', 'control_weights_changed', 'programmed'))
    assert manifest['clock_core_mhz'] == 50 and manifest['wide_split'] == 1
    hw = hardware_inputs(SOURCE, manifest)
    assert len(hw) == 63
    components = {name: load(path/'prepared_source_review.json') for name,path in
                  [('fusion',FUSION), ('action_CSA',ACTION), ('UART_loss',UART)]}
    for name, proof in components.items():
        assert proof['passed'] and Path(proof['source']).resolve() == SOURCE.resolve()
        assert Path(proof['routed_reference']).resolve() == ROUTED.resolve()
        verify_bindings(proof['bindings'])
    fp, ap, up = [components[name] for name in ('fusion', 'action_CSA', 'UART_loss')]
    assert fp['modified_rtl_inputs'] == [RTL[0]] and ap['modified_rtl_inputs'] == [RTL[1]]
    assert up['modified_rtl_inputs'] == [] and up['ready_for_integrated_synthesis']
    assert not up['deployment_accepted'] and not up['new_SYN_or_PNR_started']
    assert fp['identical_assets'] == ap['identical_assets'] == 69
    assert fp['identical_headers'] == ap['identical_headers'] == 31
    finite = load(Path(fp['proof']))
    math = load(Path(fp['math_proof']))
    boundary = load(Path(fp['independent_boundary_review']))
    assert finite['passed'] and finite['clock_cycles'] == 200000 and finite['pipeline_reset_phases'] == 4
    assert finite['external_outputs_flags_valid_and_engine_publish_cycle_exact']
    assert math['passed'] and len(math['proofs']) == 5 and all(item['status'] == 'unsat' for item in math['proofs'])
    assert math['reset_interruption_phases_checked'] == 36 and math['reset_start_control_edges_checked'] == 3060
    assert boundary['passed'] and boundary['exact_inverse_transform_recovers_source']
    assert boundary['helper_exact_inverse_transform_recovers_original_isqrt'] and boundary['actual_helper_only_one_private_fusion_call']
    verify_bindings(math['source_bindings']+[math['candidate_metadata_observed_not_RTL_proof']])
    generated = generate((SOURCE/RTL[0]).read_text(), (SOURCE/'control/imu_isqrt.v').read_text())
    assert (FUSION/RTL[0]).read_text() == generated
    bind_contract(ACTION, ACTION, ap)
    for candidate, allowed in ((FUSION,[RTL[0]]), (ACTION,[RTL[1]]), (UART,[])):
        changed = sorted(name for name in hw if not same_bytes(SOURCE/name,candidate/name))
        assert changed == allowed, (candidate,changed)
        assert all(same_bytes(SOURCE/name,candidate/name) and same_bytes(ROUTED/name,candidate/name)
                   for name in ('g2.cst','g2.sdc'))
    assert up['modified_existing_FW'] == EXISTING_FW and up['added_FW'] == ADDED_FW
    assert up['unchanged_existing_FW'] == 59 and up['hardware_assets'] == 69
    assert up['unchanged_hardware_assets'] == 58
    preprocessing = load(UART/'preprocessed_source_binding.json')
    assert preprocessing['passed'] and preprocessing['C_translation_units'] == 25
    assert preprocessing['current_source_tokens_equal_preserved_actual_compiler_inputs']
    for unit in preprocessing['units']:
        assert unit['passed'] and unit['exit_code'] == 0
        verify_bindings([unit['source'], unit['preserved_preprocessed']])
    for binding in up['frozen_FW_bindings']:
        verify_bindings([binding['source'],binding['prepared']])
        assert same_bytes(Path(binding['source']['path']),Path(binding['prepared']['path']))
    for binding in up['RK_context_bindings']+up['RK_configuration_bindings']:
        verify_bindings([binding['original'],binding['snapshot']])
        assert same_bytes(Path(binding['original']['path']),Path(binding['snapshot']['path']))
    rk = load(UART/'rk_mission_handshake_result.json')
    assert rk['passed'] and rk['checks'] == 31 and not rk['serial_opened']
    assert not rk['production_services_changed'] and not rk['full_RK_application_or_vehicle_acceptance']
    elf_image, elf = read_elf(UART/'firmware/nav_fw.elf')
    assert elf_image == (UART/'firmware/nav_fw.bin').read_bytes()
    assert len(elf_image) == 52164
    assert decode_ram(UART) == decode_ram(UART/'firmware') == elf_image+bytes(131072-52164)
    # Copy only the frozen software candidate, never working-tree RTL or firmware.
    shutil.copytree(UART,out,ignore=shutil.ignore_patterns('__pycache__'))
    proof_dir=out/'component_proofs';proof_dir.mkdir()
    for name,path in [('fusion',FUSION),('action_CSA',ACTION),('UART_loss',UART)]:
        shutil.copyfile(path/'prepared_source_review.json',proof_dir/(name+'_prepared_source_review.json'))
    shutil.copyfile(FUSION/RTL[0],out/RTL[0])
    shutil.copyfile(ACTION/RTL[1],out/RTL[1])
    changed_rtl=sorted(name for name in hw if not same_bytes(SOURCE/name,out/name))
    assert changed_rtl == RTL
    assert same_bytes(FUSION/RTL[0],out/RTL[0]) and same_bytes(ACTION/RTL[1],out/RTL[1])
    assets={name:p for name,p in inventory(SOURCE,{'.hex','.mem','.bin','.mif'}).items() if 'impl/' not in name}
    assert len(assets)==69
    changed_assets=sorted(name for name in assets if not same_bytes(SOURCE/name,out/name))
    assert changed_assets==up['modified_hardware_assets']
    assert all(same_bytes(UART/name,out/name) for name in assets)
    oldfw,newfw=inventory(SOURCE/'fw'),inventory(out/'fw')
    assert len(oldfw)==61 and len(newfw)==63
    assert ['fw/'+name for name in sorted(set(newfw)-set(oldfw))]==ADDED_FW
    assert ['fw/'+name for name in sorted(oldfw) if not same_bytes(oldfw[name],newfw[name])]==EXISTING_FW
    assert all(same_bytes(path,out/'fw'/name) for name,path in inventory(UART/'fw').items())
    assert all(same_bytes(path,out/'firmware'/name) for name,path in inventory(UART/'firmware').items())
    assert all(same_bytes(path,out/'rk'/name) for name,path in inventory(UART/'rk').items())
    assert list(inventory(out/'rk'))==['yuntuo/bridge/core.py']
    assert len(inventory(out,{'.vh','.h'}))==32
    run=(SOURCE/'run.tcl').read_text(encoding='utf-8')
    assert [line.strip() for line in run.splitlines() if line.strip().startswith('run ')]==['run syn']
    (out/'run.tcl').write_text(relocate(run,SOURCE,out),encoding='utf-8',newline='\n')
    merged=relocate(copy.deepcopy(load(UART/'manifest.json')),UART,out)
    merged.update(source_files=[str(out/name) for name in hw],frozen_source_build=str(SOURCE),
                  candidate='fusion_square2_action_constant_CSA_UART_loss_combined_source_only',
                  fusion_square_pipeline2=True,action_map_constant_CSA=True,
                  action_map_exact_widths=False,pose_predecode=False,
                  modified_rtl_inputs=changed_rtl,source_preparation_only=True,synthesis_complete=False,
                  automatic_pnr=False,programmed=False,deployment_accepted=False,
                  component_proofs_only_combined_system_acceptance_pending=True,
                  fusion_square2_proof=fp['proof'],fusion_square2_math_proof=fp['math_proof'],
                  fusion_square2_boundary_review=fp['independent_boundary_review'],
                  action_constant_CSA_proof=ap['numeric_proof'],action_constant_CSA_module_proof=ap['module_proof'],
                  UART_loss_component_review=str(UART/'prepared_source_review.json'))
    save(out/'manifest.json',merged)
    patches=[]
    for name in RTL:
        patches.extend(difflib.unified_diff((SOURCE/name).read_text().splitlines(True),(out/name).read_text().splitlines(True),
                                          fromfile='a/'+name,tofile='b/'+name))
    (out/'two_RTL_only.patch').write_text(''.join(patches),encoding='utf-8',newline='\n')
    result=dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),source=str(SOURCE),
                routed_reference=str(ROUTED),prepared=str(out),hardware_inputs=63,
                modified_rtl_inputs=changed_rtl,unchanged_hardware_inputs=61,
                modified_existing_FW=EXISTING_FW,added_FW=ADDED_FW,unchanged_existing_FW=59,
                hardware_assets=69,modified_hardware_assets=changed_assets,unchanged_hardware_assets=58,
                original_headers=31,headers_with_new_guard=32,minimum_RK_overlay=['yuntuo/bridge/core.py'],
                each_component_candidate_source_byte_identical_in_combination=True,
                all_weights_4500_CPU128KiB_map_point_capacity125Hz_and_original_stop_RTL_preserved=True,
                actual_new_CPU_image_52164_bytes_and128KiB_zero_padded_HEX_proved=True,
                source_CPU_image=elf,original_CST_20ns10ns_SDC_and_tool_options_preserved=True,
                no_POSE_failed_action_width_or_CDC52_added=True,
                reused_component_proofs=dict(fusion_cycle_proof=fp['proof'],fusion_math_proof=fp['math_proof'],
                    fusion_RTL_boundary_proof=fp['independent_boundary_review'],
                    action_cycle_and_integer_domain_proof=ap['numeric_proof'],action_module_mapping=ap['module_proof'],
                    UART_source_and_image=str(UART/'prepared_source_review.json'),
                    RK_frozen_context_31_checks=str(UART/'rk_mission_handshake_result.json')),
                no_repeated_simulation_firmware_compile_or_synthesis=True,
                new_joint_RAM_INIT_clocks26CDC_and_physical_resources_pending=True,
                combined_functional_acceptance=False,physical_timing_accepted=False,
                ready_for_joint_review_not_automatic_build=True,
                remaining_deployment_gates=['Combine-only full-control numerical/cycle/stop verification under real scheduling',
                    'New joint netlist CPU INIT/RAM geometry/weights/clocks/26 CDC and reset/enable sources',
                    'Actual routing, global allocation, all setup/hold/recovery/removal/pulse/physical CDC/IO',
                    'Actual dual UART, NV/telemetry freshness, host-loss under long planner load, explicit rearm',
                    'Controlled stop with balance maintained and separately tested genuine emergency stop',
                    'NV delayed pre-arrival USB replay/session gap, software hard WCET and actual stack watermark',
                    'Real RK service and powered/supported vehicle acceptance'],
                bindings=[metadata(p) for p in [SOURCE/'manifest.json',FUSION/RTL[0],ACTION/RTL[1],
                    FUSION/'prepared_source_review.json',ACTION/'prepared_source_review.json',UART/'prepared_source_review.json',
                    out/RTL[0],out/RTL[1],out/'firmware/nav_fw.bin',out/'firmware/nav_fw.elf',
                    out/'rk/yuntuo/bridge/core.py',out/'run.tcl',out/'g2.cst',out/'g2.sdc']],
                new_SYN_or_PNR_started=False,serial_opened=False,production_RK_modified=False,
                programmed=False,deployment_accepted=False)
    save(out/'prepared_source_review.json',result)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--out',type=Path,default=OUT);args=parser.parse_args()
    result=prepare(args.out)
    print(json.dumps({key:result[key] for key in ('passed','prepared','modified_rtl_inputs','modified_hardware_assets',
                                                'new_SYN_or_PNR_started','deployment_accepted')},ensure_ascii=False))
