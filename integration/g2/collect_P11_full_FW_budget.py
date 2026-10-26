"""Collect actual isolated P11 C/full-link results without rerunning tools."""
from pathlib import Path
from datetime import datetime
import argparse,json,re
from prepare_baseline_fusion_retime import metadata,save
BASE=Path('F:/nav_integration_g2_20261023')
REPO=Path(__file__).resolve().parents[2]

def collect(out):
    assert not out.exists(),out;out.mkdir()
    runs=[]
    for num in(1,2):
        d=BASE/('P11_shared_guard_FW_integrated_link_probe'+str(num))
        result=json.loads((d/'result.json').read_text());text=(d/'firmware/nav_fw.map').read_text(encoding='utf8')
        sizes={}
        for section,start,size in re.findall(r'^(\.(?:text|rodata|bss))\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)',text,re.M):
            sizes[section]=dict(address=int(start,16),bytes=int(size,16))
        end=int(re.search(r'0x([0-9a-f]+)\s+_bss_end\s*=',text)[1],16)
        sections={n:int(s,16)for n,s in re.findall(r' \.text\.([^\s]+)\s+0x[0-9a-f]+\s+0x([0-9a-f]+)',text)}
        runs.append(dict(directory=str(d),source=result['source'],runtime=json.loads((d/'RV_original_flags_LTO_link_runtime.json').read_text()),
            link_exit_code=result['link_exit_code'],phase=result['phase'],actual_sections=sizes,
            bss_end=end,stack_reserved_bytes=6144,CPU_RAM_bytes=131072,
            bytes_over_reserved_RAM=end+6144-131072,actual_linked_ELF_exists=(d/'firmware/nav_fw.elf').is_file(),
            actual_new_p11_named_sections={n:v for n,v in sections.items()if n.startswith(('p11','lock_zero','crc_byte'))},
            source_inputs_unchanged=result['frozen_inputs_unchanged']))
    component=BASE/'P11_shared_FW_C_component_probe6';cr=json.loads((component/'result.json').read_text())
    first=BASE/'P11_shared_FW_C_component_probe3'
    proof=BASE/'P11_raw_FIFO_source_contract_review1/result.json'
    r=dict(at=datetime.now().astimezone().isoformat(),stage='isolated P11 FW experiment; not selected',
        original_selected_image=dict(binary_bytes=53572,BSS_bytes=70004,stack_reserved_bytes=6144,
                                     RAM_remaining_after_reserve_bytes=1348,source=str(BASE/'stage3_SLF128_storage_RTL_prepared6')),
        actual_full_links=runs,
        latest_component=dict(result=str(component/'result.json'),passed=cr['passed'],output=cr['native_output'],
                              scope='actual guard/raw-service C + queue MMIO test double; not actual RTL bus or physical stop'),
        first_real_component_counterexample=dict(directory=str(first),reason='lock edge found inside byte helper was lost by a second poll; retained and fixed in prepared4'),
        GCC_CRC=dict(global_options_unchanged=True,new_function_only_attribute='optimize("no-optimize-crc")',
                     before_object_text_plus_rodata=2444,after_object_text_plus_rodata=1976,
                     removed_table_bytes=512,latest_rodata_bytes=0,
                     limitation='isolated RV32IM object is not the full RV32IMC LTO budget'),
        raw_source_contract=dict(result=str(proof),actual_fields=['6004 DATA valid/FIRST','6008 byte count','600C TS or zero','601C byte overflow'],
            queue_highwater=30,byte_quota=32,TS_FIFO_depth=32,TS_total_capacity_including_held=34,
            no_TS_valid_or_overflow_CSR=True,
            proof='conditional first-TS-drop with post-pair byte count <=30 UNSAT',
            assumptions=['pristine epoch','sole raw FIFO consumer','one TS read per FIRST','byte overflow quarantine',
                         'before/after count checks','no concurrent R0 debug pops'],
            remaining=['actual MMIO service ordering and quiet flush','stop-clock/restart source switching','actual bounded service time']),
        control_contract=dict(management_request='NV45 action/session; existing R0 trusted management path only',
            latest_draft_reply='NV85 result/session/state only,16 total bytes',
            owner_states=['OFF','WAIT_ZERO','ARMED','LOCKED'],NAV_remains_only_hardware_actuator_owner=True,
            P11_never_renews_R0=True,command_age_us=200000,R0_age_us=250000,feedback_age_us=50000,
            require_feedback_still_us=500000,explicit_distinct_session_and_zero_rearm=True,
            final_filter='zero-only after same common motion/support checks and R0 guard; no post-lease nonzero mux',
            original_CONTROL_V2_type_body_unchanged=True,WIT_optional_runtime_branch_retained=True),
        narrow_optimization_result='Removed only new debug counters and ACK diagnostics: actual headroom improves96B, still716B over. Further C fit has not been proved; no safety/old-function cuts authorized by this evidence.',
        next_decision='Root review: keep current image; compare a small parser/atomic mailbox offload against further justified C refactor. No RTL backup implemented here.',
        backup=dict(status='source-based design option only; no RTL/tool result',
            reuse='existing uart_speed_rx CRC/range/legacy-byte state machine; tap existing imu_uart pre-FIFO accepted byte/FIRST/start timestamp rather than create another bit receiver',
            minimal_RTL_files=['nav/rtl/io/imu_uart.v: expose accepted byte, FIRST, actual t_start for parser tap; WIT FIFOs unchanged',
                               'new parser module factored from control/uart_speed_rx.v byte state machine; no Q12 multiply/divide path',
                               'nav/rtl/top/nav_subsystem.v + register definition/generated RF: atomic command snapshot/sequence/timestamp CSRs'],
            CSR_draft=['packed signed16 forward_mmps/yaw_mradps','actual FIRST start timestamp32','monotonic valid-command sequence32/status','explicit snapshot latch so FW reads one generation'],
            needed_new_safety=['one accepted command per FIRST burst','CRC/range errors do not update sequence/time',
                'authorization snapshots sequence and requires newer post-authorization zero',
                'missed sequence or ambiguous snapshot conservatively locks','P11 source mode explicit; WIT remains selectable',
                'firmware R0/feedback/P11 TTL/common motion/support/final zero filters remain'],
            source_register_bits_before_mapping='existing parser state/pending/CRC/frame-age about109; live payload/timestamp/sequence/snapshot about192 additional bits if all duplicated; exact implementation and synthesis not done',
            BSRAM_requested=0,new_clock_domains=0,
            actual_FF_LUT_DSP_unknown=True,
            possible_FW_saving='target removes most898B raw_work parser/service and queue-integrity software; actual replacement link must measure, not assume fit'),
        deployment_ready=False,new_BIN_INIT=False,selected_53572_or_SYN43_inputs_changed=False,
        not_yet_verified=['full app/proto/MMIO integrated native fixture','full integrated linked stack (link blocked)',
                          'actual RK command/UI/ownership path','busy planner350ms command and500ms physical stop'])
    save(out/'result.json',r)
    save(REPO/'docs/4_design/evidence/g2_20261023/P11_shared_FW_actual_component_and_RAM_budget_20261026.json',r)
    print(json.dumps(dict(output=str(out),component_passed=cr['passed'],full_link_over_bytes=[x['bytes_over_reserved_RAM']for x in runs],deployment_ready=False)))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=BASE/'P11_shared_FW_actual_budget_review1');collect(p.parse_args().out)
