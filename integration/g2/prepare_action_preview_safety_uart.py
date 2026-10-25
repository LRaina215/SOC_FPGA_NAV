"""Prepare an eventual combined source; never modify an active build.

Reuse proven preview, common-reset FIFO/MAC re-arm and UART-loss components.
Component acceptance is explicitly not combined numerical, timing or vehicle
acceptance. No new compile, simulation, synthesis or PNR is launched here.
"""
import argparse
import copy
from datetime import datetime
import difflib
import json
from pathlib import Path
import shutil
from prepare_baseline_fusion_retime import hardware_inputs,metadata,relocate,same_bytes,save
from prepare_baseline_fusion_uart_loss import decode_ram,read_elf,inventory

BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'baseline_fusion_action_preview_syn28'
ORIGINAL=BASE/'baseline_fusion_only_syn23'
UART=BASE/'baseline_fusion_uart_loss_prepared2'
MAC=BASE/'mac_rx_boundary_candidate2'
FIFO=BASE/'fifo_async_assert_candidate3'
PREVIEW=BASE/'action_preview_product_candidate1'
ADDITIONAL=['g2_top.v','nav/rtl/common/nav_async_fifo.v','nav/rtl/net/rgmii_mac100.v']
CONTROL=['control/mem_policy.v','control/policy_action_map.v','control/rl_policy_tick_v5.v']

def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))

def bindings(rows):
    for row in rows:assert metadata(Path(row['path']))==row,('proof input changed',row['path'])

def prepare(out):
    assert not out.exists()
    actual=load(BASE/'baseline_fusion_action_preview_one_shot/actual_SYN28_review.json')
    mapping=load(BASE/'baseline_fusion_action_preview_one_shot/actual_preview_product_mapping.json')
    assert actual['passed']and mapping['passed']
    bindings(actual['reviewed_input_metadata']);bindings(mapping['evidence_bindings'])
    assert load(SOURCE/'job.json')['phase']=='synthesis_complete'
    u=load(UART/'prepared_source_review.json');m=load(MAC/'result.json');f=load(FIFO/'result.json')
    assert u['passed']and u['ready_for_integrated_synthesis']and m['passed']and f['passed']
    bindings(m['source_bindings']+m['candidate_bindings']+[m['FIFO_proof_binding']])
    bindings(f['source_bindings']+f['candidate_bindings'])
    assert f['public_nav_fifo_ob_byte_identical']and f['RAM_processes_and_capacity_preserved']
    assert m['metrics']['candidate_tail_bytes']==0 and m['paired_FIFO_candidate']==str(FIFO)
    assert same_bytes(MAC/'nav_async_fifo.v',FIFO/'nav_async_fifo.v')
    pp=load(PREVIEW/'prepared_review.json');bindings(pp['source_bindings']+pp['candidate_bindings'])
    assert all(same_bytes(SOURCE/n,PREVIEW/n)for n in CONTROL)
    manifest=load(SOURCE/'manifest.json');hw=hardware_inputs(SOURCE,manifest);assert len(hw)==63
    assert sorted(n for n in hw if not same_bytes(SOURCE/n,ORIGINAL/n))==CONTROL
    assert all(same_bytes(SOURCE/n,ORIGINAL/n)and same_bytes(UART/n,ORIGINAL/n)for n in ADDITIONAL)
    assert sorted(n for n in hw if not same_bytes(UART/n,ORIGINAL/n))==[]
    # All hardware assets keep capacity. The only value changes are the11
    # previously frozen CPU image views. Do not recompile or take live RK code.
    assert u['hardware_assets']==69 and len(u['modified_hardware_assets'])==11
    image,elf=read_elf(UART/'firmware/nav_fw.elf')
    assert len(image)==52164 and image==(UART/'firmware/nav_fw.bin').read_bytes()
    assert decode_ram(UART)==decode_ram(UART/'firmware')==image+bytes(131072-len(image))
    pre=load(UART/'preprocessed_source_binding.json');assert pre['passed']and pre['C_translation_units']==25
    assert pre['current_source_tokens_equal_preserved_actual_compiler_inputs']
    for unit in pre['units']:assert unit['passed'];bindings([unit['source'],unit['preserved_preprocessed']])
    for row in u['frozen_FW_bindings']:
        bindings([row['source'],row['prepared']]);assert same_bytes(Path(row['source']['path']),Path(row['prepared']['path']))
    for row in u['RK_context_bindings']+u['RK_configuration_bindings']:
        # The original live working file is historical comparison evidence;
        # only the already tested immutable snapshot is selected here.
        bindings([row['snapshot']])
    rk=load(UART/'rk_mission_handshake_result.json');assert rk['passed']and rk['checks']==31 and not rk['serial_opened']
    shutil.copytree(UART,out,ignore=shutil.ignore_patterns('__pycache__'))
    for n in CONTROL:shutil.copyfile(SOURCE/n,out/n)
    for n,src in [('g2_top.v',MAC/'g2_top.v'),('nav/rtl/net/rgmii_mac100.v',MAC/'rgmii_mac100.v'),('nav/rtl/common/nav_async_fifo.v',FIFO/'nav_async_fifo.v')]:shutil.copyfile(src,out/n)
    changed=sorted(n for n in hw if not same_bytes(SOURCE/n,out/n));assert changed==ADDITIONAL
    total=sorted(n for n in hw if not same_bytes(ORIGINAL/n,out/n));assert total==sorted(CONTROL+ADDITIONAL)
    assets={n:p for n,p in inventory(SOURCE,{'.hex','.mem','.bin','.mif'}).items()if'impl/'not in n};assert len(assets)==69
    modified_assets=sorted(n for n in assets if not same_bytes(SOURCE/n,out/n));assert modified_assets==u['modified_hardware_assets']
    assert all(same_bytes(UART/n,out/n)for n in assets)
    for d in('fw','firmware','rk','rk_context'):
        assert all(same_bytes(p,out/d/n)for n,p in inventory(UART/d).items())
    assert len(inventory(out/'fw'))==63 and len(inventory(out,{'.h','.vh'}))==32
    assert all(same_bytes(SOURCE/n,out/n)for n in('g2.cst','g2.sdc'))
    run=(SOURCE/'run.tcl').read_text();assert [s.strip()for s in run.splitlines()if s.strip().startswith('run ')]==['run syn']
    (out/'run.tcl').write_text(relocate(run,SOURCE,out),encoding='utf-8',newline='\n')
    combined=relocate(copy.deepcopy(load(UART/'manifest.json')),UART,out)
    combined.update(source_files=[str(out/n)for n in hw],frozen_source_build=str(SOURCE),action_preview_product=True,
        async_FIFO_common_reset_repair=True,MAC_RX_rearm_on_existing_reset=True,
        action_map_constant_CSA=False,action_map_exact_widths=False,fusion_square_pipeline2=False,pose_predecode=False,
        candidate='preview_product_FIFO_MAC_UART_source_only',modified_rtl_inputs=total,
        source_preparation_only=True,synthesis_complete=False,automatic_pnr=False,programmed=False,deployment_accepted=False,
        component_proofs_only_not_combined_acceptance=True)
    save(out/'manifest.json',combined)
    patch=[]
    for n in changed:patch.extend(difflib.unified_diff((SOURCE/n).read_text().splitlines(True),(out/n).read_text().splitlines(True),fromfile='a/'+n,tofile='b/'+n))
    (out/'three_additional_RTL.patch').write_text(''.join(patch),encoding='utf-8')
    proof=dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),source=str(SOURCE),prepared=str(out),
        modified_RTL_against_SYN28=changed,modified_RTL_against_SYN23=total,hardware_inputs=63,unchanged_hardware_against_SYN28=60,
        hardware_assets=69,modified_hardware_assets=modified_assets,unchanged_assets=58,CPU128KiB_52164B_image_bound=True,CPU_image=elf,
        original_4500_all_weights125Hz_and_capacities_unchanged=True,original_control_emergency_stop_priority_RTL_unchanged=True,
        public_UART_lidar_IO_and_clock_ports_unchanged=True,existing_RX_reset_only=True,
        firmware_files=63,headers=32,minimum_RK_overlay=['yuntuo/bridge/core.py'],live_RK_context_not_adopted=True,
        reused_component_proofs=dict(preview_source=str(PREVIEW/'prepared_review.json'),preview_arithmetic=str(PREVIEW/'arithmetic_result.json'),preview_scheduler=str(PREVIEW/'policy_result.json'),actual_SYN28=str(BASE/'baseline_fusion_action_preview_one_shot/actual_SYN28_review.json'),actual_preview_mapping=str(BASE/'baseline_fusion_action_preview_one_shot/actual_preview_product_mapping.json'),FIFO=str(FIFO/'result.json'),MAC=str(MAC/'result.json'),UART=str(UART/'prepared_source_review.json'),RK31=str(UART/'rk_mission_handshake_result.json')),
        no_simulation_firmware_compile_SYN_or_PNR_started=True,PNR39_modified=False,serial_opened=False,production_RK_modified=False,
        combined_functional_acceptance=False,physical_timing_accepted=False,automatic_build=False,programmed=False,deployment_accepted=False,
        remaining=['Combined source numerical/output timing/reset/stop tests before adoption','New CPU INIT against changed52164B BIN, all other RAM geometry/INIT and actual FF resets','New26CDC reference: deliberate async-reset FF kind/control changes must be proved, not aliased to old kind','Actual routing/global allocation/setup/hold/reset/pulse/IO and all effective constraints','Actual RK dual UART freshness/loss stop/explicit rearm/feedback and independent ESTOP','Actual device/image/power/support checks and vehicle gates'],
        bindings=[metadata(p)for p in[UART/'prepared_source_review.json',FIFO/'result.json',MAC/'result.json',PREVIEW/'prepared_review.json',out/'manifest.json',out/'firmware/nav_fw.bin',out/'firmware/nav_fw.elf',out/'rk/yuntuo/bridge/core.py']+[out/n for n in total]])
    save(out/'prepared_source_review.json',proof);return proof

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);r=prepare(p.parse_args().out)
    print(json.dumps({k:r[k]for k in('passed','prepared','modified_RTL_against_SYN28','modified_hardware_assets','combined_functional_acceptance','no_simulation_firmware_compile_SYN_or_PNR_started')},ensure_ascii=False))
