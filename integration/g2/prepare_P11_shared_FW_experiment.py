"""Freeze an isolated P11 software-core draft and integration contract.

This does not select or compile a new total FW, generate INIT, modify RTL, or
connect a serial device.  The two added C files deliberately remain unlinked
until the application/raw-FIFO integration and tests close their contracts.
"""
from pathlib import Path
from datetime import datetime
import argparse,json,shutil,difflib
from prepare_baseline_fusion_retime import metadata,save

BASE=Path('F:/nav_integration_g2_20261023');SOURCE=BASE/'stage3_SLF128_storage_RTL_prepared6'
REPO=Path(__file__).resolve().parents[2];TEMPLATE=Path(__file__).with_name('P11_shared_FW_candidate')

def prepare(out):
 assert not out.exists(),out
 inputs=[metadata(p)for p in sorted((SOURCE/'fw').rglob('*'))if p.is_file()]
 out.mkdir();shutil.copytree(SOURCE/'fw',out/'fw')
 added=[]
 for name in('p11_command_guard.h','p11_command_guard.c'):
  dst=out/'fw/nav'/name;assert not dst.exists();shutil.copyfile(TEMPLATE/name,dst);added.append(metadata(dst))
 assert inputs==[metadata(Path(x['path']))for x in inputs]
 unchanged=[]
 for p in sorted((SOURCE/'fw').rglob('*')):
  if p.is_file():
   dst=out/'fw'/p.relative_to(SOURCE/'fw');assert p.read_bytes()==dst.read_bytes();unchanged.append(str(p.relative_to(SOURCE/'fw')))
 patch=''.join(''.join(difflib.unified_diff([],p.read_text().splitlines(True),fromfile='/dev/null',tofile='fw/nav/'+p.name))for p in sorted((out/'fw/nav').glob('p11_command_guard.*')))
 (out/'new_component_only.patch').write_text(patch,encoding='utf8')
 contract=dict(at=datetime.now().astimezone().isoformat(),stage='isolated core draft; integration pending',source=str(SOURCE),production_BIN_53572_unchanged=True,
  original_FW_files_unchanged=len(unchanged),original_FW_inputs=inputs,added_files=added,applied_to_original=False,selected_by_production=False,compiled=False,tested=False,new_ELF_BIN_INIT=False,RTL_changed=False,new_EDA=False,serial_opened=False,
  message_draft=dict(request_type_hex='45',reply_type_hex='85',assignment='Draft only: no collision found in current frozen proto.h or selected repo/RK enum review; must be frozen into ICD/client together, not yet implemented.',
   request='NV 01 45 + u32LE action(0 query,1 authorize P11,2 release back to NAV) + u32LE new_session; fixed12B',
   reply='NV 01 85 + u32LE result + u32LE session + u32LE state + u32LE arrival_age_us + u32LE accepted + u32LE rejected; fixed28B',
   CONTROL_V2_83_and_120B_body_unchanged=True,existing_MODE_values_0_to_4_unchanged=True,untrusted_P11_bytes_never_manage_ownership=True),
  states=['OFF/WIT optional: original parser branch remains selectable','WAIT_ZERO: trusted fresh R0 IDLE + measured still500ms + drained/quiet source + new session authorized, zero only','ARMED: subsequent distinct timestamped complete CRC/range frame can propose velocity','LOCKED: R0 loss, P11 arrival age>=200ms, feedback age>=50ms or raw integrity fault; numeric zero and latched until another trusted new-session request plus fresh zero'],
  stop_contract=dict(R0_guard_us=250000,P11_arrival_age_us=200000,feedback_fresh_us=50000,rearm_static_us=500000,physical_stop_gate_us=500000,physical_stop_verified=False,
   command_publish_max_age_us_gate=350000,command_publish_max_age_us_verified=False,P11_does_not_refresh_R0_guard=True,no_auto_recover_on_packets=True,existing_SAFE_and_lease_sole_actuator_owner=True,IR_hard_EST0P_priority_unchanged=True),
  raw_UART_contract=dict(physical_input='A15 existing imu_rxd -> u_imu byte/FIRST FIFO',config_div=217,idle_us=500,byte_RAM_depth=256,byte_total_capacity=258,burst_timestamp_total_capacity=34,
   selected_sender='50Hz10-byte frames in separate bursts; >=500us idle between packets. Sender starts only after fresh management ACK and sends new zero first.',
   no_back_to_back_fresh_timestamp_claim=True,one_accepted_command_per_burst=True,arrival_stamp='Actual FIRST start time; conservative packet-start age, never software dequeue time',
   per_loop_byte_budget=32,per_search_checkpoint_byte_budget=32,raw_queue_limit=32,per_pair_elapsed_us_must_be_less_than=100,
   guard='Before and after each data/FIRST timestamp pair, verify monotone overflow baseline, count<=32 and elapsed<100us. Missing/zero stamp, misordering or backlog faults quarantine; all selected-mode old/WIT consumers disabled.',
   reason='At most32 queued byte/FIRST records, plus at most2 pipeline entries, fit34 timestamp slots; a FIRST requires>=500us idle. Measured100us pair bound avoids an unchecked producer interval. This inference still needs actual-MMIO ordering and worst-case service validation.',
   drain='In quarantine do not accept packets. Bounded drain of up to258 data and34 timestamp records, then require >=2ms with no new bytes; explicitly new trusted management request. A flood never becomes armed. Do not reset CPU/control/RAW RAM to fake an empty stream.',
   unresolved='Current HAL has no explicit timestamp-valid status and reads DATA before TS. The proposed bounds/zero rejection and quiet flush require a real MMIO miter; this draft core API alone does not establish FIFO pairing.'),
  integration_patch_plan=[
   dict(files=['fw/nav/proto.h','fw/nav/proto.c'],change='Add the exact-length explicit trusted management request/reply without overloading MODE or0x83; validate action/session before any side effect. No velocity NV message exists today.'),
   dict(files=['fw/nav/host_link_guard.c'],change='Permit management only as an accepted fresh R0 message; it may renew R0 as a management packet. P11 parser calls never call host_link_guard_observe.'),
   dict(files=['fw/app/nav_app.h','fw/app/nav_app.c'],change='Own one p11 state object and raw protocol selection. Track existing odometry samples without a second FIFO pop. On authorization clear old goal/waypoints/target/path and require original MODE IDLE. Select WIT or P11 exclusively; keep WIT source code. Poll both guards before/after bounded work and at search checkpoints; publish one immediate zero through existing HAL on any lock edge.'),
   dict(files=['fw/hal/hal.h','fw/drv/hal_fpga.c'],change='Expose existing raw divisor/idle/count/overflow/TS reads. Add measured DATA/FIRST+TS pair helper and bounded quiet flush; maintain existing final host_link_guard_filter and SAFE commit ordering. Do not override a checked zero with a nonzero UART target in HAL.'),
   dict(files=['fw/nav/nav_task.h','fw/nav/nav_task.c'],change='Add an explicit manual proposal before the shared nonzero motion_check/support_check block, with common freshness/stop conditions. Keep the original10Hz planning publication and hardware125Hz policy; received50Hz targets are staged. IDLE/ESTOP/guard lock force zero. Return to NAV clears manual state and never resumes an old autonomous task.'),
   dict(files=['selected RK bridge serial owner and client protocol definitions'],change='One full-duplex process owns P11 port; read unchanged V2 and serialize10B writes with the specified20ms packet cadence. Ownership command/ACK uses CH9102. No second process opens the same TTY; reconnect requires R0 IDLE/new session/new P11 zero.'),
   dict(files=['FW compile input list, linked stack proof and all11CPU INIT assets'],change='Only after native parser/owner/MMIO tests and busy-source proof pass, add module to actual build list, compile once and repeat strict linked stack plus BIN/128KiB lane binding. No new total FW has been produced here.')],
  required_validation=['All signed velocity ranges, 8/10-byte CRC/range/partial resync; bad traffic cannot renew age','Every owner transition at command/zero/deadline boundary; no old task or packet replay on release/rearm','R0 fresh/P11 missing; P11 fresh/R0 locked; both lost; feedback invalid; IR hard-stop precedence','Raw FIFO count/overflow/timestamp loss and queued stale packets through real DATA-pop/TS-pop MMIO semantics, with bursts at limits and uint32 time wrap','WIT optional branch and BMI088 control fusion remain unchanged; source switch flush cannot mix WIT bytes with speed bytes','Real RV stack/BSS/code size/service cycles and actual CPU busy/500ms stop before deployment'],
  deployment_ready=False)
 save(out/'contract.json',contract);save(out/'manifest.json',dict(stage=contract['stage'],build_allowed=False,added_files=added,unchanged_original_files=unchanged,source_bindings=inputs,contract=str(out/'contract.json')))
 print(json.dumps(dict(output=str(out),stage=contract['stage'],original_files=len(unchanged),added_files=len(added),compiled=False),ensure_ascii=False))

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=BASE/'P11_shared_guard_FW_experiment_prepared1');a=p.parse_args();prepare(a.out)
