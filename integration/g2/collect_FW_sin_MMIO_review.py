"""Collect preserved one-shot source/numeric/link/stack proof. No new build."""
from pathlib import Path
from datetime import datetime
import argparse,json,re
from prepare_baseline_fusion_retime import metadata,save
from collect_P11_snapshot_FW_review import mapped
BASE=Path('F:/nav_integration_g2_20261023');REPO=Path(__file__).resolve().parents[2]

def collect(out):
    assert not out.exists();out.mkdir();ld=BASE/'FW_sin_MMIO_RV_link_probe1';cd=BASE/'FW_sin_MMIO_C_probe1'
    lr=json.loads((ld/'result.json').read_text());cr=json.loads((cd/'result.json').read_text())
    source=Path(lr['source']);parent=BASE/'P11_quiesce_shared_guard_FW_prepared1'
    m=json.loads((source/'FW_sin_MMIO_manifest.json').read_text());plan=json.loads((source/'hardware_ROM_plan.json').read_text())
    assert lr['passed'] and lr['phase']=='linked_stack_passed' and cr['passed'] and lr['frozen_inputs_unchanged']
    metrics,_=mapped(ld/'firmware/nav_fw.map');old,_=mapped(BASE/'P11_quiesce_FW_RV_link_probe1/firmware/nav_fw.map')
    stack=lr['stack_review'];assert stack['passed'] and stack['complete'] and not stack['unresolved_indirect']
    assert metrics['over_bytes']==-1924 and old['over_bytes']==180
    for x in lr['input_bindings']:assert metadata(Path(x['path']))==x,x['path']
    syms=(ld/'RV_actual_symbols.log').read_text();disasm=(ld/'RV_actual_disassembly.log').read_text()
    assert 'NAV_SIN_LUT' not in syms and '__clz_tab' in syms
    functions={}
    for name in('fx_sin','fx_cos'):
        match=re.search(r'(?m)^[0-9a-f]+ <'+name+r'>:\n(.*?)(?=\n[0-9a-f]+ <|\Z)',disasm,re.S)
        assert match,name
        body=match[0];assert '0x90000' in body and re.search(r'\blw\s',body)
        functions[name]=body
    # All owner/stop/quiesce behavior is inherited byte-for-byte from the better
    # 180B-over source, not from the later220B direct-proposal experiment.
    changed=set(m['changed_files']);assert changed=={'include\\nav_fw_sin_lookup.h','nav\\fixmath.c','nav\\traj_sw.c'}
    same=[]
    for p in sorted((parent/'fw').rglob('*')):
        if not p.is_file():continue
        rel=p.relative_to(parent/'fw')
        if str(rel)not in changed:assert p.read_bytes()==(source/'fw'/rel).read_bytes(),str(rel);same.append(str(rel))
    result=dict(at=datetime.now().astimezone().isoformat(),phase='future software numeric and original link/strict stack passed; ROM hardware unimplemented',
        source=str(source),source_parent=str(parent),manifest=metadata(source/'FW_sin_MMIO_manifest.json'),changed_FW_files=m['changed_files'],
        unchanged_parent_FW_file_count=len(same),same_guard_protocol_quiesce_motion_support_HAL=True,
        hardware_plan=metadata(source/'hardware_ROM_plan.json'),address_contract=plan['address_contract'],
        numeric=dict(result=metadata(cd/'result.json'),output=cr['output_text'],checks=641732,exact_original_words=1024,all16bit_angles=65536,
             rotation_cases=4096,trajectory_scenarios=32,modeled_MMIO_reads=201058,
             reference='preserved original FW table/source, compared to explicit CSR substitute returning the exact signed16 value; all1024 also matched existing sin_lut.hex',
             runtime=[json.loads(p.read_text())for p in sorted(cd.glob('*runtime.json'))],
             exits=[json.loads(p.read_text())for p in sorted(cd.glob('*exit.json'))],
             no_actual_BSRAM_or_CPU_bus_delay_test=True),
        actual_link=dict(result=metadata(ld/'result.json'),controller_pid=lr['controller_pid'],controller_created_epoch=lr['controller_created_epoch'],tool=lr['tool'],
             **metrics,original_RAM_bytes=131072,stack_reserved_bytes=6144,remaining_after_stack_bytes=1924,
             previous_over_bytes=180,total_measured_improvement_bytes=2104,
             previous_sections=old['sections'],rodata_reduction_bytes=2048,text_reduction_bytes=62,BSS_bytes_unchanged=70060,
             old2048B_NAV_SIN_LUT_absent_from_real_symbols=True,clz256B_table_preserved=True,
             actual_disassembly=functions,source_traj_sw_not_linked=('traj_eval_sw' not in syms),
             original_flags='RV32IMC_Zicsr/ilp32/-Os/-flto/GC sections; no extra compiler flags, RAM/stack unchanged',
             ELF=metadata(ld/'firmware/nav_fw.elf'),BIN_or_INIT_exported=False),
        strict_same_ELF_stack=dict(result=metadata(ld/'strict_stack_review/result.json'),passed=True,complete=True,
             actual_static_bound_bytes=stack['actual_static_bound_bytes'],maximum_static_gate_bytes=3072,stack_reserved_bytes=6144,
             actual_jr_count=stack['actual_jr_count'],actual_jalr_count=stack['actual_jalr_count'],unresolved_indirect=stack['unresolved_indirect'],
             input_files_changed=stack['input_files_changed'],inputs=stack['inputs'],
             method='same linked ELF, all LTO/SU, resolved bounded jump tables and complete callgraph; no source stack estimate or whitelist'),
        hardware_requirements=dict(dedicated_ROM='one1024x16 BSRAM on original CPU50 clock, no sharing or modification of matcher/traj four existing trigROM ports/INIT',
             expected_extra_BSRAM=1,expected_physical_total=281,actual_resource_prediction_not_measured=True,limit=300,
             E0='accept exact0x90000000..0x90000FFF aligned32 read, select index mem_addr[11:2] and demand ROM read',
             E1='capture signed extension of ROM Q into per_q and one-cycle per_ready; preserve pending/req mutual exclusion',
             read_hold='ROM Q holds when no demand; no runtime writes; content INIT invariant under protocol reset',
             reset='cpu_rst=rst|hold cancels pending/ACK only, not ROM content; no stale response after reset',
             unchanged='CPU128KiB RAM, console posted bytes/ACK/debug, old per/nav arbitration and all current PNR input',
             actual_RTL_generated=False,EDA_started=False),
        performance_scope=dict(source_inventory=plan['direct_source_call_inventory'],
             known_counts=['fx_sin/cos one volatile load each','fx_rot_zyx twelve source lookups per call',
                           'local_frame rotate four lookups, preserve up to eight',
                           'support_check up to8*(steps+1), steps<=64, subject to earlier rejection',
                           'motion_check loop4*(horizon+1), horizon<=64, plus4 per relevant s0 unknown cell; this cell loop is not a constant260-read bound',
                           'traj_sw two per candidate-step but absent from this actual linked ELF; actual HAL trajectory path remains hardware'],
             risks=plan['performance_risks'],actual_lookup_frequency_or_WCET_measured=False,
             stop_gates='busyCPU zero<=350ms and physicalstop<=500ms remain unverified and unchanged; C numerical equality and static stack do not satisfy them'),
        reproducible_commands=dict(prepare='D:/Anaconda/python.exe -B -X utf8 integration/g2/prepare_FW_sin_MMIO_experiment.py --out <new-source>',
             numeric='D:/Anaconda/python.exe -B -X utf8 integration/g2/validate_FW_sin_MMIO.py --source <new-source> --out <new-output>',
             original_RV_link='D:/Anaconda/python.exe -B -X utf8 integration/g2/link_P11_integrated_FW_experiment.py --source <new-source> --out <new-output>',
             collect='D:/Anaconda/python.exe -B -X utf8 integration/g2/collect_FW_sin_MMIO_review.py --out <new-review>'),
        full_RV_link_attempts=1,no_further_compile_or_BIN_export=True,selected_53572B_image_unchanged=True,PNR53_54_inputs_unchanged=True,
        remaining=['materialize and prove ROM/CPU bus handshake/reset/whole1024 read/hold',
                   'actual source/CPUINIT/trigROM/clock/stop and BSRAMmapping after final integration',
                   'actualPico busy scheduling350mszero/RK and500msphysicalstop','actual full PNR/setup/hold/CDC/reset/IO gates'],deployment_ready=False)
    save(out/'result.json',result);save(REPO/'docs/4_design/evidence/g2_20261023/FW_sin_MMIO_actual_numeric_and_budget_20261026.json',result)
    print(json.dumps(dict(output=str(out),remaining_after_stack=1924,strict_static_stack=stack['actual_static_bound_bytes'],hardware_generated=False)))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=BASE/'FW_sin_MMIO_actual_review1');collect(p.parse_args().out)
