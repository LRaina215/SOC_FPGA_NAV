"""Bind completed proofs and real compiled CPU images to one prepared profile."""
import argparse
import json
from pathlib import Path
import shutil

from prepare_lf_line_joint import assets,source_files
from prepare_nav_density_trim import PROFILES,MATCH,TRAJ,CONSOLE_SCOPES,transform_match,transform_traj,firmware_transform
from trim_peripheral_rtl import trim_oled,trim_tx_shadow,trim_console_storage,OLED_RELATIVE,TX_SHADOW_RELATIVE,CONSOLE_RELATIVE

def load(p):return json.loads(Path(p).read_text(encoding='utf-8'))
def write(p,value):Path(p).write_text(json.dumps(value,indent=2)+'\n',encoding='utf-8')

def finalize(source,build,hardware_proof,firmware_proof,tier):
    source,build,hardware_proof,firmware_proof=map(lambda x:Path(x).resolve(),(source,build,hardware_proof,firmware_proof))
    p=PROFILES[tier];profile=load(source/'trim_profile.json');origin=Path(profile['origin']);manifest=load(source/'manifest.json')
    hw=load(hardware_proof/'result.json');fw=load(firmware_proof);assert hw['passed'] and fw['passed']
    assert Path(hw['source']).resolve()==source and hw['tier']==tier
    stack=load(build/'stack_analysis.json');assert stack['complete'] and stack['static_bound_bytes']<=3072
    for name in ['nav_fw.bin','ram_w.hex']+[f'ram_b{i}.hex' for i in range(4)]:shutil.copyfile(build/name,source/'firmware'/name)
    for name in ['ram_w.hex']+[f'ram_b{i}.hex' for i in range(4)]:assert (source/name).read_bytes()==(build/name).read_bytes()
    old,new=assets(origin),assets(source);assert set(old)==set(new) and len(new)==104
    changed=sorted(n for n in old if old[n].read_bytes()!=new[n].read_bytes());assert len(changed)==11
    peripheral=source.parent/'delivery_trim_interface_probe3';periph=load(peripheral/'result.json')
    assert periph['passed'] and periph['live_mailbox_interface_passed'] and periph['OLED_release_source_contract_passed']
    changes={MATCH:transform_match((origin/MATCH).read_text(encoding='utf-8'),p),TRAJ:transform_traj((origin/TRAJ).read_text(encoding='utf-8'),p),OLED_RELATIVE:trim_oled((origin/OLED_RELATIVE).read_text(encoding='utf-8')),TX_SHADOW_RELATIVE:trim_tx_shadow((origin/TX_SHADOW_RELATIVE).read_text(encoding='utf-8')),CONSOLE_RELATIVE:trim_console_storage((origin/CONSOLE_RELATIVE).read_text(encoding='utf-8'))}
    hardware=source_files(source,manifest);assert sorted(n for n in hardware if (source/n).read_bytes()!=(origin/n).read_bytes())==sorted(changes)
    for n,expected in changes.items():assert (source/n).read_text(encoding='utf-8')==expected
    profile['modified_rtl_inputs']=sorted(changes)
    for key in ('allowed_changed_prefixes','allowed_memory_change_prefixes'):
        for scope in CONSOLE_SCOPES:
            if scope not in profile[key]:profile[key].append(scope)
    if not any(isinstance(t,dict) and t.get('feature')=='offline console log storage' for t in profile['tradeoffs']):
        profile['tradeoffs'].insert(2,dict(feature='offline console log storage',before='four byte-bank diagnostic log RAMs and E000-EFFF log readback',after='log writes ACK and counter retained; log readback zero; CPU HOLD/download and UART/navigation protocol retained'))
    for n in profile['firmware_modified_inputs']:
        rel=n.removeprefix('fw/');assert (source/n).read_text(encoding='utf-8')==firmware_transform(rel,(origin/n).read_text(encoding='utf-8'),p)
    # All firmware files outside the declared six source/header changes remain exact.
    fw_actual=[]
    for oldfile in (origin/'fw').rglob('*'):
        if oldfile.is_file():
            rel=oldfile.relative_to(origin).as_posix()
            if oldfile.read_bytes()!=(source/rel).read_bytes():fw_actual.append(rel)
    assert sorted(fw_actual)==sorted(profile['firmware_modified_inputs'])
    command=['D:/Anaconda/python.exe','-X','utf8',str(source/'fw/bsp/build_rv.py'),'--ram-kb','128','--output-dir',str(build),'--copy-to',str(source)]
    profile['allowed_changed_assets']=changed
    profile['firmware_provenance']=dict(source=str(source/'fw'),command=command,compiler='D:/tools/xpack-riscv-none-elf-gcc-15.2.0-1/bin/riscv-none-elf-gcc.exe',compiled_elf=str(build/'nav_fw.elf'),image_bytes=(build/'nav_fw.bin').stat().st_size,stack_analysis=str(build/'stack_analysis.json'),static_stack_bound_bytes=stack['static_bound_bytes'],RAM_bytes=131072,stack_reserve_bytes=6144,source_modified_inputs=fw_actual,output_bindings=[dict(candidate=n,compiled_output=str(build/Path(n).name)) for n in changed])
    profile['proofs']=[dict(path=str(hardware_proof/'result.json'),required_true_fields=['passed','selected_matcher_candidate_arithmetic_passed','trajectory_cap_and_exact_arithmetic_passed','invalid_result_address_returns_zero'],source_bindings=[dict(candidate=MATCH,proof_source=str(hardware_proof/'matcher/slf_match.v')),dict(candidate=TRAJ,proof_source=str(hardware_proof/'trajectory/traj_eval.v'))]),dict(path=str(peripheral/'result.json'),required_true_fields=['passed','live_mailbox_interface_passed','OLED_release_source_contract_passed'],source_bindings=[dict(candidate=OLED_RELATIVE,proof_source=str(peripheral/'control4500_core_trim.v')),dict(candidate=TX_SHADOW_RELATIVE,proof_source=str(peripheral/'host_mbx_trim.v'))]),dict(path=str(firmware_proof),required_true_fields=['passed','global_initialization_radius_and_relocation_radius_retained','generated_inverse_indices_match_literal_candidate_feeds','all_batches_within_actual_hardware_capacity','slow_forward_turn_and_reverse_categories_retained','failed_HAL_returns_zero_command'],source_bindings=fw['source_bindings'])]
    console=source.parent/'delivery_trim_console_probe2'
    profile['proofs'].append(dict(path=str(console/'result.json'),required_true_fields=['passed','console_storage_contract_passed','CPU128KiB_hold_download_and_nav_protocol_passed'],source_bindings=[dict(candidate=CONSOLE_RELATIVE,proof_source=str(console/'nav_soc_trim.v'))]))
    for evidence in profile['proofs']:
        result=load(evidence['path'])
        for flag in evidence['required_true_fields']:assert result[flag] is True
        for b in evidence['source_bindings']:assert (source/b['candidate']).read_bytes()==Path(b['proof_source']).read_bytes()
    write(source/'trim_profile.json',profile)
    state=load(source/'prepared_source_review.json');state.update(passed=True,phase='prepared_targeted_proofs_passed_waiting_for_serial_joint_queue',modified_rtl_inputs=sorted(changes),unchanged_assets=93,changed_assets=changed,assets_total=104,unchanged_hardware_inputs=63,unchanged_assets104=False,original_asset_paths104_retained=True,firmware_build=profile['firmware_provenance'],proofs=profile['proofs'],source_and_proof_byte_bindings_passed=True,original_4500_all_weights_125Hz_safety_full_telemetry_retained=True,hardware_resources_or_timing_claimed=False,navigation_tradeoff_quality_not_yet_accepted=True,console_log_storage_removed=True,console_CPU_ACK_counter_HOLD_download_retained=True)
    write(source/'prepared_source_review.json',state);write(source/'job.json',state);return state

if __name__=='__main__':
    p=argparse.ArgumentParser()
    for arg in ('source','build','hardware-proof','firmware-proof'):p.add_argument('--'+arg,required=True)
    p.add_argument('--tier',choices=PROFILES,required=True);a=p.parse_args();print(json.dumps(finalize(a.source,a.build,a.hardware_proof,a.firmware_proof,a.tier)))
