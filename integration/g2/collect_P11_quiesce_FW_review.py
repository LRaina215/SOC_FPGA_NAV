"""Collect already completed C/link evidence; no compile, RTL, or image export."""
from pathlib import Path
from datetime import datetime
import argparse,json
from prepare_baseline_fusion_retime import metadata,save
from collect_P11_snapshot_FW_review import mapped
BASE=Path('F:/nav_integration_g2_20261023');REPO=Path(__file__).resolve().parents[2]

def collect(out):
    assert not out.exists();out.mkdir()
    cd=BASE/'P11_quiesce_FW_C_probe1';ld=BASE/'P11_quiesce_FW_RV_link_probe1'
    cr=json.loads((cd/'result.json').read_text());lr=json.loads((ld/'result.json').read_text())
    metrics,symbols=mapped(ld/'firmware/nav_fw.map')
    previous,ps=mapped(BASE/'P11_snapshot_FW_RV_link_probe3/firmware/nav_fw.map')
    baseline,bs=mapped(BASE/'stage2_SLF128_safeFW_RV_compile1/firmware/nav_fw.map')
    assert cr['passed'] and cr['proposed_CSR_substitute_only']
    assert lr['phase']=='link_failed_preserved' and lr['link_exit_code']==1 and lr['frozen_inputs_unchanged']
    assert metrics['over_bytes']==180 and previous['over_bytes']==324
    source=Path(lr['source']);manifest=json.loads((source/'quiesce_manifest.json').read_text())
    for m in lr['input_bindings']:
        assert metadata(Path(m['path']))==m,m['path']
    result=dict(at=datetime.now().astimezone().isoformat(),phase='finite proposed-CSR C passed; original 128KiB link still blocked',
        source=str(source),manifest=metadata(source/'quiesce_manifest.json'),changed_FW_files=manifest['changed_files'],
        source_parent=str(BASE/'P11_snapshot_shared_guard_FW_prepared4'),
        actual_C=dict(result=metadata(cd/'result.json'),output=cr['output_text'],checks=352,
            controller_pid=cr['controller_pid'],controller_created_epoch=cr['controller_created_epoch'],
            runtimes=[metadata(p) for p in sorted(cd.glob('*runtime.json'))],
            exits=[json.loads(p.read_text())for p in sorted(cd.glob('*exit.json'))],
            actual_app_helper_prefix_and_original_finalHAL=True,core_four_files_byte_identical_to_snapshot_proof=True,
            tests='BOOT/HOLD flush and OFF/R0 lock; pending200ms age and250ms R0; fresh zero/rearm; WIT half-state reset; bounded one READY read per pending call',
            hardware_FIFO_clear_and_READY_are_only_CSR_substitutes=True),
        actual_link=dict(result=metadata(ld/'result.json'),tool=lr['tool'],controller_pid=lr['controller_pid'],
            controller_created_epoch=lr['controller_created_epoch'],**metrics,
            RAM_bytes=131072,stack_reserved_bytes=6144,required_total_bytes=metrics['bss_end']+6144,
            previous_over_bytes=324,measured_saving_bytes=144,
            top10_new_or_increased=[dict(symbol=k,baseline_bytes=bs.get(k,0),actual_bytes=v,delta=v-bs.get(k,0)) for k,v in sorted(symbols.items(),key=lambda z:z[1]-bs.get(z[0],0),reverse=True)[:10]],
            symbol_changes_from_previous=[dict(symbol=k,previous_bytes=ps.get(k,0),actual_bytes=v,delta=v-ps.get(k,0))for k,v in symbols.items()if v!=ps.get(k,0)],
            ELF_exists=(ld/'firmware/nav_fw.elf').is_file(),new_BIN_INIT=False,strict_same_ELF_stack_gate_passed=False,
            compiler_flags='original rv32imc_zicsr/ilp32/-Os/-flto/GC sections; RAM131072 and stack6144 unchanged'),
        hardware_plan2=metadata(BASE/'P11_quiesce_flush_plan2/result.json'),
        hardware_plan_scope='explicit RX-only flush, continuous real rxs HIGH2ms, automatic ACTIVE and sticky completion READY; not implemented or tested on RTL',
        inherited_counterexamples_preserved=[str(BASE/'P11_snapshot_FW_C_probe1/old_long_session_actual_counterexample.log'),
            str(BASE/'P11_snapshot_FW_RV_link_probe1/result.json'),str(BASE/'P11_snapshot_FW_RV_link_probe2/result.json'),str(BASE/'P11_snapshot_FW_RV_link_probe3/result.json')],
        remaining=['180B original link overflow','strict same-ELF full stack gate and BIN/128KiB INIT only after legal link',
            'actual parser/CSR/real FIRST timestamp and quiesce state RTL proof','full APP busy timing and every-sending-period snapshot service',
            'real RK management,350ms zero submission and500ms physical stop'],
        no_more_micro_compile_trials_on_this_quiesce_route=True,
        selected_53572B_image_unchanged=True,PNR53_54_inputs_unchanged=True,RTL_generated_or_modified=False,EDA_started=False,deployment_ready=False)
    save(out/'result.json',result)
    save(REPO/'docs/4_design/evidence/g2_20261023/P11_quiesce_FW_actual_contract_and_budget_20261026.json',result)
    print(json.dumps(dict(output=str(out),actual_C_checks=352,original_link_over_bytes=180,measured_saving_bytes=144)))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=BASE/'P11_quiesce_FW_actual_review1');collect(p.parse_args().out)
