"""Read the preserved native/LTO evidence; never rerun a tool or export INIT."""
from pathlib import Path
from datetime import datetime
import argparse,json,re
from prepare_baseline_fusion_retime import metadata,save
BASE=Path('F:/nav_integration_g2_20261023');REPO=Path(__file__).resolve().parents[2]

def mapped(path):
    t=path.read_text(encoding='utf8')
    sections={n:dict(address=int(a,16),bytes=int(s,16))for n,a,s in re.findall(r'^(\.(?:text|rodata|bss))\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)',t,re.M)}
    end=int(re.search(r'0x([0-9a-f]+)\s+_bss_end\s*=',t)[1],16)
    symbols={n:int(v,16)for n,v in re.findall(r' \.text\.([^\s]+)\s+0x[0-9a-f]+\s+0x([0-9a-f]+)',t)}
    return dict(sections=sections,bss_end=end,over_bytes=end+6144-131072),symbols

def collect(out):
    assert not out.exists();out.mkdir()
    baseline,bs=mapped(BASE/'stage2_SLF128_safeFW_RV_compile1/firmware/nav_fw.map')
    links=[]
    for n in(1,2,3):
        d=BASE/('P11_snapshot_FW_RV_link_probe'+str(n));r=json.loads((d/'result.json').read_text())
        metrics,s=mapped(d/'firmware/nav_fw.map')
        links.append(dict(output=str(d),source=r['source'],phase=r['phase'],exit_code=r['link_exit_code'],**metrics,
            runtime=json.loads((d/'RV_original_flags_LTO_link_runtime.json').read_text()),
            top10_actual_LTO_new_or_increased=[dict(symbol=k,baseline_bytes=bs.get(k,0),new_bytes=v,delta=v-bs.get(k,0))for k,v in sorted(s.items(),key=lambda z:z[1]-bs.get(z[0],0),reverse=True)[:10]],
            ELF_exists=(d/'firmware/nav_fw.elf').is_file(),inputs_unchanged=r['frozen_inputs_unchanged']))
    core=BASE/'P11_snapshot_FW_C_probe1/result.json';app=BASE/'P11_snapshot_FW_C_probe4/result.json'
    cr=json.loads(core.read_text());ar=json.loads(app.read_text());assert cr['passed']and ar['passed']
    source=Path(ar['source']);rtl_source=BASE/'SLF128_storage_safeFW_SYN43_prepared1'
    result=dict(at=datetime.now().astimezone().isoformat(),phase='finite snapshot/software contracts passed;128KiB link still blocked',
        latest_source=str(source),actual_full_link_runs=links,
        baseline=dict(image_bytes=53572,BSS_bytes=70004,stack_reserved_bytes=6144,remaining_bytes=1348),
        latest_actual=dict(text_bytes=links[-1]['sections']['.text']['bytes'],rodata_bytes=links[-1]['sections']['.rodata']['bytes'],
            BSS_bytes=links[-1]['sections']['.bss']['bytes'],code_data_end=links[-1]['sections']['.bss']['address'],
            bss_end=links[-1]['bss_end'],RAM_bytes=131072,stack_reserved_bytes=6144,over_bytes=links[-1]['over_bytes']),
        same_compiler_flags='rv32imc_zicsr/-mabi=ilp32/-Os/-flto/-ffunction-sections/-fdata-sections/--gc-sections; no parameter sweep',
        actual_components=dict(core_result=str(core),core_output=cr['output'],
            actual_app_result=str(app),actual_app_output=ar['output'],
            reused_four_core_service_C_H_files_identical=True,
            scope='proposed CSR substitute, actual guard/service C, exact app helper prefix and actual original HAL gate; motion/support invocation remains source binding, not executed fullAPP'),
        first_real_long_session_counterexample=dict(path=str(BASE/'P11_snapshot_FW_C_probe1/old_long_session_actual_counterexample.log'),
            text=cr['old_long_session_counterexample'],fix='WAIT_ZERO checks authorization halfcycle;ARMED only short fresh/order checks',
            latest_test_frames=233000,actual_elapsed64_us=4659982000),
        CPU_HOLD=dict(actual_sources=[metadata(rtl_source/'g2_top.v'),metadata(rtl_source/'nav/rtl/soc/nav_soc.v'),metadata(source/'fw/app/nav_app.c')],
            source='common rst goes to NAV andCPU; cpu_rst=rst|hold, soHOLD preserves parser/NAV CSR',
            fix='actual app init p11_guard_init; p11_set_protocol(0) writesCTRL0 and leaves parserdisabled;R0initiallocked/ownerOFF',
            finite_test='oldSEQ7/nonzero snapshot cleared;only explicit NV authorize callsenable1',
            actual_Pico_warm_boot_not_tested=True),
        WIT_handover=dict(state='OFF +wit_pending; no direct WIT byte parsing whilepending',
            order='controlled zero/explicitrelease→CTRL0→baud868→paired drain≤32byte +newquiet2ms→clearWITparserhalfstate→WITready',
            actual_native_old_queue_bytes=258,old_TS=34,drain_calls=9,max_bytes_per_call=32,quiet_restarts_on_new_byte=True,
            R0_or_NV_motion_during_pending='blocked by app gate; actual finalHAL alsozero-only',
            receiver_inflight_bound='at original217→868, residualbc<=216 plus atmost10*868 clocks <180us at50MHz;2ms post-switch quarantine covers old character, not a changed stopdeadline',
            proof_limits=['actual sourcecycle derivation/MMIObus execution not physically measured','external source must obey selected protocol after handover','CPU bounded service/busy plan timing stillpending']),
        atomicity=dict(four_read_cut_cases=16,SEQ_saturates_and_freezes_triplet=True,
            known_external_CTRL_ABA=cr['known_external_CTRL_ABA'],
            required='no IRQ, exclusiveFW CTRL writes serialized outside four reads;commonNAV reset also restartsCPU',
            optional_future_hardware_fix='monotonic generation acrossCTRL flush or explicit epoch;not implemented/authorized by this evidence'),
        structure_cleanup=dict(source=str(source),previous=str(BASE/'P11_snapshot_shared_guard_FW_prepared3'),
            only_changed_app=True,guard_and_service_bytes_unchanged=True,
            changes=['single guard_manage checker','common immediate-zero note after snapshotpoll','rawFIFO drain only WIT_DRAIN'],
            reserved_RAM_over_bytes_before=396,after=324,measured_saving_bytes=72),
        next_frontend_options_for_root_review=[
            dict(option='hardware protocol-handover quiesce/flush',
                scope='only rawbyte/TSFIFO cleanup and in-flight character boundary on explicit source switch;FW still ownsR0/feedback/TTL/owner',
                possible_target='current C WIT_DRAIN and protocol-management glue; actual saving and gates unknown',
                risks='must retain ordinary rawFIFO/WIT semantic; explicit administrative discard only, no direct motion/stop mux'),
            dict(option='fullSW shared source proposal refactor',
                scope='replace addednav_task manual fields/copiedzero construction with a single checked proposal source',
                possible_target='note_proposal120 +task_step52 +newnav_taskBSS; potentialsum is upperbound, not saving prediction',
                risks='must preserve all existing hold/ESTOP/commonmotion/support and source diagnostic behavior')],
        no_more_compile_trials_started=True,hardware_RTL_or_PNR_inputs_changed=False,
        selected_53572_image_unchanged=True,new_BIN_INIT=False,ELF_export_stack_gate_passed=False,
        remaining=['fit original128KiB +6144stack','strict sameELF fullcallgraph stack/BSS after successfullink',
                   'actual RTLsnapshot parser/CSR/timestamp mapping','every20msconsume underbusyCPU and actual350mszero/500msphysicalstop',
                   'actualRK/client ownership/messages and fullAPPmotion/support behavior'],deployment_ready=False)
    save(out/'result.json',result);save(REPO/'docs/4_design/evidence/g2_20261023/P11_snapshot_FW_actual_contract_and_budget_20261026.json',result)
    print(json.dumps(dict(output=str(out),C_passed=True,latest_over_bytes=links[-1]['over_bytes'],selected_image_unchanged=True)))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=BASE/'P11_snapshot_FW_actual_review1');collect(p.parse_args().out)
