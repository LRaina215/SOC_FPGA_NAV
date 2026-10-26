"""Read preserved one-shot proposal experiment; never rebuild or export INIT."""
from pathlib import Path
from datetime import datetime
import argparse,json,re
from prepare_baseline_fusion_retime import metadata,save
from collect_P11_snapshot_FW_review import mapped
BASE=Path('F:/nav_integration_g2_20261023');REPO=Path(__file__).resolve().parents[2]

def collect(out):
    assert not out.exists();out.mkdir()
    cd=BASE/'P11_shared_proposal_FW_C_probe1';ld=BASE/'P11_shared_proposal_FW_RV_link_probe1'
    cr=json.loads((cd/'result.json').read_text());lr=json.loads((ld/'result.json').read_text())
    source=Path(lr['source']);prior=BASE/'P11_quiesce_shared_guard_FW_prepared1'
    nm,ns=mapped(ld/'firmware/nav_fw.map');om,os=mapped(BASE/'P11_quiesce_FW_RV_link_probe1/firmware/nav_fw.map')
    assert cr['passed'] and cr['shared_proposal_actual_source_branch']
    assert 'PASS P11_SHARED_PROPOSAL checks=648' in cr['output_text']
    assert lr['link_exit_code']==1 and lr['frozen_inputs_unchanged'] and nm['over_bytes']==220 and om['over_bytes']==180
    for m in lr['input_bindings']:assert metadata(Path(m['path']))==m,m['path']
    m=json.loads((source/'shared_proposal_manifest.json').read_text())
    assert set(m['changed_files'])=={'app\\nav_app.c','nav\\nav_task.c','nav\\nav_task.h'}
    app=(source/'fw/app/nav_app.c').read_text();task=(source/'fw/nav/nav_task.c').read_text()
    assert 'manual_owner' not in task and 'p11_note_proposal' not in app
    # Check both inherited guard math and the final hardware publication path.
    same=['nav/p11_command_guard.c','nav/p11_command_guard.h','nav/p11_snapshot_service.c','nav/p11_snapshot_service.h','drv/hal_fpga.c']
    for r in same:assert (source/'fw'/r).read_bytes()==(prior/'fw'/r).read_bytes(),r
    old_task=(prior/'fw/nav/nav_task.c').read_text()
    tail=lambda t:t[t.index('send_checked:'):t.index('void nav_task_map_work(')]
    assert tail(task)==tail(old_task)
    deltas=[dict(symbol=k,old_bytes=os.get(k,0),new_bytes=ns.get(k,0),delta=ns.get(k,0)-os.get(k,0)) for k in sorted(set(ns)|set(os)) if ns.get(k,0)!=os.get(k,0)]
    result=dict(at=datetime.now().astimezone().isoformat(),phase='finite direct-proposal C passed; one original LTO remains over budget',
        source=str(source),manifest=metadata(source/'shared_proposal_manifest.json'),contract=metadata(source/'shared_proposal_contract.json'),
        changed_files=m['changed_files'],actual_source_bindings_unchanged=True,
        byte_identical_to_parent=same,original_full_send_checked_body_identical=True,
        actual_C=dict(result=metadata(cd/'result.json'),output=cr['output_text'],checks=648,
            controller_pid=cr['controller_pid'],controller_created_epoch=cr['controller_created_epoch'],
            runtime=[json.loads(p.read_text())for p in sorted(cd.glob('*runtime.json'))],
            exits=[json.loads(p.read_text())for p in sorted(cd.glob('*exit.json'))],
            covered=['original quiesce C352-contract cases (353 counted with outer CHECK)',
                     'actual new manual branch before identical common send_checked body',
                     'motion hazard/support block/no-fresh-map reject without P11 restoring rejected motion',
                     'slow halves commands; hard ESTOP bypasses P11 provider',
                     'expiry during support invokes unchanged final HAL zero filter',
                     'raw service/provider/quarantine new lock edge publishes zero exactly once',
                     'OWNER_OFF leaves ordinary NAV tuple unmodified; next accepted snapshot is directly sampled without nav_task cache'],
            explicit_substitutes=['proposed CSR READY/flush','map/motion/support results and time advance'],
            not_complete_APP_Pico_busy_or_physical_stop=True),
        actual_link=dict(result=metadata(ld/'result.json'),controller_pid=lr['controller_pid'],controller_created_epoch=lr['controller_created_epoch'],tool=lr['tool'],
            **nm,RAM_bytes=131072,stack_reserved_bytes=6144,required_total_bytes=nm['bss_end']+6144,
            previous_over_bytes=180,total_change_bytes=40,text_change_bytes=44,BSS_change_bytes=-8,
            actual_all_LTO_symbol_size_changes=deltas,
            source_intent_not_achieved='removing8B duplicated state and old note_proposal did not lower total linked footprint; direct provider/control-flow and independent stop-edge helpers grew code',
            ELF_exists=(ld/'firmware/nav_fw.elf').is_file(),new_BIN_INIT=False,strict_same_ELF_stack_passed=False,
            compiler='unchanged original RV32IMC_Zicsr/ilp32/-Os/-flto/GC sections; no flag search, no stack or CPU reduction'),
        hardware_plan=dict(path=str(BASE/'P11_quiesce_flush_plan2/result.json'),
            selected='explicit RX-only FLUSH -> actual continuously updated rxs HIGH2ms -> automatic ACTIVE and latched cleanup-complete READY; ACTIVE normal starts do not re-isolate',
            not_yet_implemented_or_RTL_tested=True,
            WIT_warning='2ms HIGH qualifies a serial idle boundary, not a complete WIT packet; original WIT header/checksum resynchronization and a new valid packet are still required for usable attitude',
            TX_warning='cfg_div is shared by TX; explicit source change requires TX idle/empty and exclusive control. Administrative RX flush must never reset TX',
            abandoned_READY_HELD_RESUME=False),
        decision='stop this route here; retain all prior over-budget cases and deliver220B gap for root architecture decision',
        actual_original_flags_link_attempts_this_structure=1,further_compilation_started=False,
        selected_53572B_image_unchanged=True,PNR53_54_inputs_unchanged=True,RTL_generated_or_modified=False,EDA_started=False,
        remaining=['220B original RAM assertion','full strict same-ELF stack/linked indirect calls once legal ELF exists',
                   'actual hardware parser/quiesce/CSR/clock/reset proof','full APP/Pico scheduling, actual RK and350ms zero/500ms physical stop'],deployment_ready=False)
    save(out/'result.json',result)
    save(REPO/'docs/4_design/evidence/g2_20261023/P11_shared_proposal_FW_actual_contract_and_budget_20261026.json',result)
    print(json.dumps(dict(output=str(out),checks=648,over_bytes=220,actual_total_increase_bytes=40,further_trials=False)))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=BASE/'P11_shared_proposal_FW_actual_review1');collect(p.parse_args().out)
