"""Read-only P11-39/40 contract audit; no VG, EDA, serial or device access.

Uses frozen prepared6/SYN42 RTL and FW, the original control adapter, and the
official local pin workbook.  It reports an implementation gap, not approval
of an already functioning control receive channel.
"""
from pathlib import Path
from datetime import datetime
import argparse,json,re,zipfile,xml.etree.ElementTree as ET
from prepare_baseline_fusion_retime import metadata,save,same_bytes

BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'stage3_SLF128_storage_RTL_prepared6'
ACTUAL=BASE/'SLF128_storage_safeFW_syn42'
OFFICIAL=Path('D:/Desktop/SOC FPGA/小梅哥ACG750型国产高云GW5AT-138K FPGA开发板资料/硬件图纸')
REPO=Path(__file__).resolve().parents[2]

def excerpts(path,needles):
 lines=path.read_text(encoding='utf-8-sig').splitlines();rows=[]
 for needle in needles:
  found=[dict(line=i+1,text=x)for i,x in enumerate(lines)if needle in x]
  assert found,(path,needle);rows+=found
 return dict(file=metadata(path),lines=rows)

def workbook_rows(path):
 ns={'m':'http://schemas.openxmlformats.org/spreadsheetml/2006/main'};rows=[]
 with zipfile.ZipFile(path)as z:
  strings=[''.join(e.itertext())for e in ET.fromstring(z.read('xl/sharedStrings.xml'))]
  for name in z.namelist():
   if not(name.startswith('xl/worksheets/sheet')and name.endswith('.xml')):continue
   for row in ET.fromstring(z.read(name)).findall('.//m:row',ns):
    cells={}
    for c in row:
     v=c.find('m:v',ns)
     cells[c.attrib['r']]=strings[int(v.text)]if c.attrib.get('t')=='s'else(v.text if v is not None else ''.join(c.itertext()))
    if any(x in('GPIO0_34','GPIO0_35','UART_RXD','UART_TXD')for x in cells.values()):rows.append(dict(sheet_XML=name,row=int(row.attrib['r']),cells=cells))
 assert any(r['cells'].get('B209')=='GPIO0_34'and r['cells'].get('C209')=='A15'for r in rows)
 assert any(r['cells'].get('B210')=='GPIO0_35'and r['cells'].get('C210')=='A16'for r in rows)
 return rows

def audit(out):
 assert out.exists()and not(out/'result.json').exists(),out
 paths=[SOURCE/'g2_top.v',SOURCE/'g2.cst',SOURCE/'control4500_core.v',SOURCE/'control/uart_speed_rx.v',SOURCE/'control/uart_telemetry.v',SOURCE/'command_lease_guard.v',SOURCE/'nav/rtl/top/nav_subsystem.v',SOURCE/'fw/nav/host_link_guard.c',SOURCE/'fw/nav/host_link_guard.h',SOURCE/'fw/drv/hal_fpga.c',SOURCE/'fw/app/nav_app.c',REPO/'integration/g2/control_nav_adapter.patch',REPO/'control/tools/fpga_telemetry/protocol.py',REPO/'docs/4_design/evidence/g2_20261023/dual_uart_contract_20261025.json',REPO/'docs/1_plan/g2_single_chip_total_task_20261026.md',OFFICIAL/'ACX750-GW5A138开发板管脚信息表.xlsx',OFFICIAL/'ACX750-GW5A138-CB-250928.pdf']
 before=[metadata(p)for p in paths];top=paths[0].read_text();cst=paths[1].read_text();core=paths[2].read_text();rx=paths[3].read_text()
 assert ".UART_RXD(1'b1)"in top and'.imu_rx(imu_rxd)'in top
 assert "parameter NAV_INPUT_ENABLE=1'b1"in core and"parameter UART_SPEED_ENABLE=1'b0"in core
 locations=dict(re.findall(r'IO_LOC\s+"([^"]+)"\s+(\w+)\s*;',cst))
 assert locations['imu_rxd']=='A15'and locations['control_txd']=='A16'
 assert [k for k,v in locations.items()if v=='A15']==['imu_rxd']
 relative=['g2_top.v','g2.cst','control4500_core.v','control/uart_speed_rx.v','command_lease_guard.v']
 identical={p:same_bytes(SOURCE/p,ACTUAL/p)for p in relative};assert all(identical.values())
 original=REPO/'integration/control4500/original_src/uart_speed_rx.v';parser_byte_identical=same_bytes(paths[3],original)
 assert paths[3].read_text(encoding='utf-8-sig')==original.read_text(encoding='utf-8-sig'),'parser source has more than newline encoding differences'
 rows=workbook_rows(paths[-2]);save(out/'official_pin_workbook_rows.json',dict(passed=True,input=metadata(paths[-2]),rows=rows))
 cb=(out/'official_CB_page10.txt').read_text(encoding='utf8');assert re.search(r'B2_R20_P\s+39\s+40\s+B2_R20_N',cb)
 source_excerpts=[
  excerpts(paths[0],[".BAUD(921600)","NAV is the sole owner",'.owner_select(2\'d1)',".UART_RXD(1'b1)",'.imu_rx(imu_rxd)']),
  excerpts(paths[1],['IO_LOC "imu_rxd" A15','IO_LOC "control_txd" A16']),
  excerpts(paths[2],["parameter NAV_INPUT_ENABLE=1'b1","parameter UART_SPEED_ENABLE=1'b0",'NAV_INPUT_ENABLE ? nav_forward_q12','NAV_INPUT_ENABLE ? nav_yaw_q12','command_emergency_stop=','TELEMETRY_FRAME_HZ=100','UART_CLKS_PER_BIT=217']),
  excerpts(paths[3],['FRAME_TIMEOUT_CLKS=500000','command_valid<=0;','command_seen<=1;command_valid<=1;','values_in_range=','crc[7:0]','pending_speed>=']if 'crc[7:0]'in rx else['FRAME_TIMEOUT_CLKS=500000','command_valid<=0;','command_seen<=1;command_valid<=1;','values_in_range=']),
  excerpts(paths[5],['TIMEOUT_MS=200','STOP_DWELL_MS=500','wire changing=','wire legal=','wire stop_mode=','if(changing)','else if(estop)','else if(source_fault','zero_packet && stop_confirmed','balance_enable=1\'b1']),
  excerpts(paths[6],['.uart_rx(imu_rx)','i_IMU_RX_DATA  = iu_valid']),
  excerpts(paths[7],['g->locked=true;g->timed_out=true;','if(clear &&','cmd->vx=cmd->vy=cmd->wz=0;']),
  excerpts(paths[8],['HOST_LINK_TIMEOUT_US 250000u']),
  excerpts(paths[9],['host_link_guard_filter(&g_host_link_guard','NAV_SAFE_CMD_W']),
  excerpts(paths[10],['Publish on the lock edge','hal_send_cmd(&nt->cmd);','drain_host_messages(4u);'])]
 save(out/'source_excerpts.json',dict(passed=True,records=source_excerpts))
 evidence=dict(at=datetime.now().astimezone().isoformat(),scope='Read-only source/pin/transport contract audit; no EDA, VG full parse, serial open or device operation',audit_completed=True,complete_dual_uart_functionality=False,
  source=str(SOURCE),actual_SYN42_selected_files_identical=identical,input_bindings=before,
  official_pin_mapping=dict(workbook=str(paths[-2]),workbook_rows=str(out/'official_pin_workbook_rows.json'),base_schematic=str(paths[-1]),base_page=10,render=str(out/'official_CB_page10.png'),
   P11_39=dict(GPIO='GPIO0_34',FPGA='A15',schematic_net='B2_R20_P',direction='USB-TTL TX -> FPGA RX'),P11_40=dict(GPIO='GPIO0_35',FPGA='A16',schematic_net='B2_R20_N',direction='FPGA TX -> USB-TTL RX'),
   physical_cable_39_continuity_and_current_VCCIO2_not_measured=True,USB_end_to_RK_is_historical_user_confirmation=True,not_an_actual_image_or_electrical_acceptance=True),
  implemented_frozen6=dict(CH9102=dict(RXD='L21',TXD='M21',baud=921600,role='NAV mailbox/debug register bridge; selected FW applies host_link_guard then SAFE then command_units and command_lease'),
   P11_40=dict(signal='control_txd',FPGA='A16',role='V2 control telemetry',nominal_baud=230400,physical_baud_at_50MHz=50000000/217,format='8N1',nominal_Hz=100,payload_bytes=120,wire_bytes=126,header='A5 5A 02 78',crc='CCITT-FALSE initFFFF poly1021; version+length+payload; CRC LE',wire_utilization_at_nominal_baud=126*100*10/230400,frame_sequence_is_command_ACK=False),
   P11_39=dict(signal='imu_rxd',FPGA='A15',role='legacy external raw IMU UART -> NAV IMU registers -> WIT55 FW parser',control_RXD_constant_one=True,UART_SPEED_ENABLE=0,NAV_INPUT_ENABLE=1,control_commands_reachable=False),
   control_command_priority='IR/compiled hard emergency zero -> NAV leased targets -> fixed/UART/IR fallback only when NAV_INPUT_ENABLE=0',
   authority=dict(owner_select=1,source_id=1,session_select=1,packet_session=1,NAV_sole_owner=True),
   old_control_was_deliberately_replaced_by_NAV=True,final_task_P11_39_command_chain_requirement_unmet=True),
  existing_control_RX_protocol=dict(source=str(paths[3]),same_text_as_original_control_source=True,byte_identical=parser_byte_identical,newline_normalization_only=not parser_byte_identical,format='8N1',CLKS_PER_BIT=217,header='A5 5A 10 04',total_bytes=10,
   payload='signed16 LE forward_mmps, signed16 LE yaw_mradps',limits=dict(forward_mmps=[-600,600],yaw_mradps=[-2900,2900]),CRC='CCITT-FALSE initFFFF/poly1021 over 10 04 and 4 payload bytes; CRC LE',
   legacy8_byte_length02_kept=True,legacy_yaw_zero=True,atomically_update_both_only_after_CRC_and_ranges=True,valid_one_clock_pulse=True,seen_latched=True,valid_command_held_until_new_valid_frame_or_reset=True,
   incomplete_interbyte_parser_timeout_ms=10,command_TTL_ms=None,on_wire_sequence=None,on_wire_session=None,command_ACK=None,replay_prevention=False,parser_timeout_is_motion_watchdog=False),
  stop_gates=dict(FW_RK_guard_timeout_us=250000,FW_guard_latched=True,fresh_IDLE_required_for_rearm=True,FW_guard_final_hal_send_cmd_filter=True,FW_first_lock_zero_publication=True,
   hardware_SAFE_raw_command_watchdog_default_us=200000,hardware_lease_ms=200,feedback_fresh_ms=50,zero_and_fresh_static_feedback_dwell_ms=500,communication_stop_keeps_balance_enabled=True,IR_hard_stop_separate_and_high_priority=True,
   warning='Current FW guard protects only NAV publication. An added direct-UART mux after it would bypass that guard unless its lock/permit is also a trusted hardware interlock.'),
  preferred_shared_FW_path=dict(existing_NV_velocity_or_manual_message=False,existing_NV_modes=['IDLE','GOTO','TRACK','SEARCH','ESTOP'],
   raw_UART_reusable=True,raw_FIFO_byte_depth=256,raw_timestamp_FIFO_depth=32,raw_timestamp_unit='first byte start time per idle-delimited burst, not per complete command',register_window='0x6000 baud; 0x6004 data/FIRST; 0x6008 count; 0x600C burst timestamp; 0x6018 idle; 0x601C byte overflow',
   current_default_divider=868,current_default_baud_at_50MHz=50000000/868,new_P11_divider_required=217,
   route='A15 existing raw UART/FIFO -> bounded FW byte parser -> explicit trusted manual source selection before common motion/support checks -> same host_guard-filtered hal_send_cmd -> SAFE -> original lease with NAV sole hardware owner',
   minimum_changed_software=['new P11 speed parser/arrival-age/owner module','proto.h/proto.c for explicit trusted owner management and compatible status, not reuse0x83','nav_app.c for bounded byte/overflow handling, source switch and lock publication','nav_task.c/h to select manual proposal before shared nonzero motion/support checks','hal_fpga.c/hal.h for raw UART configuration/timestamp/overflow wrappers and unchanged final host guard','RK one serial owner for V2 reads plus serialized10-byte writes and NV ownership requests'],
   RTL_required_by_software_proposal=False,raw_IMU_branch_must_remain_selectable=True,BMI088_I2C_and_control_fusion_must_remain_unchanged=True,
   timestamp_gap='Existing only-per-burst timestamps cannot establish an independently fresh arrival for every back-to-back command. The FW experiment must specify idle-delimited command bursts or safely reject ambiguous timing; it may not stamp CPU dequeue time as wire arrival. Extra per-command hardware timestamping is a separate option, not already present.',
   P11_valid_packets_must_not_renew_R0_guard=True,source_selection_after_motion_checks_forbidden=True),
  alternative_hardware_frontend_proposal_not_selected=[
   'Reassign the existing A15 port from obsolete external IMU RX to explicit control_rxd, or retain its physical name with an explicit new role. Tie the unused NAV external IMU receiver idle; do not create two pads on A15 or feed speed bytes to WIT55.',
   'Keep core NAV_INPUT_ENABLE=1 and UART_SPEED_ENABLE=0. Put the existing CRC/range parser at the shared top-level frontend; feed completed atomic UART targets to the transport-independent lease, never directly to policy or actuators.',
   'Add trusted explicit NAV/UART ownership plus a new session on switch. An unexpected UART packet never steals NAV ownership. Switch clears the old goal/target and pending packet state, requests controlled zero, and requires a fresh zero UART frame plus existing fresh-feedback/static-dwell rearm.',
   'Preserve the 250ms RK host guard as an interlock for both owners. Publish its latched lock/permit to the shared selector (reset defaults locked), or route UART through the same guarded FW publication with a separate hardware UART arrival-age gate. Merely renewing NAV zeros must not permit serial movement after R0 loss.',
   'Only a CRC/range-valid complete UART frame advances its local receive sequence and renews the selected source lease. Bad/partial packets, parser activity, periodic telemetry and CPU re-publication cannot renew UART arrival age. Legacy 10-byte frames provide no remote replay/session proof; document that limit or add a separately versioned extended format.',
   'Retain SAFE ESTOP/raw-command watchdog and IR hard-stop priority across all owners; a communication timeout requests controlled zero with balancing enabled. CPU hang or loss of feedback must stop either owner independently.',
   'Add a single RK serial owner supporting simultaneous V2 reads and serialized command writes; current TelemetryReader is a read-only receiver. Report selected source, accepted/rejected command count, arrival age and lock state in a separate compatible status response, preserving original V2 type/120-byte body.'
  ],
  necessary_prebuild_checks=['UART pin async first-stage only constraint and second-stage timed, with actual mapped objects','CRC/range/partial/interbyte rejection; at deadline valid arrival priority; duplicate/replayed legacy limitation','NAV/UART switch while moving or while packet pending; fresh zero rearm; no stale target after reset/reconnect','P11 loss with CH9102 healthy; CH9102/RK guard loss while P11 frames continue; simultaneous loss; CPU busy/hang; feedback loss','IR hard ESTOP during either ownership; software/physical reset and stopped-clock recovery; no automatic stale restart','Unchanged V2 payload/CRC/cadence and actual RK full-duplex exclusive ownership; record physical target-zero and stationary feedback separately'],
  actual_motion_stop_verified=False,implementation_authorized_by_this_readonly_audit=False,new_EDA_started=False,VG_full_parse=False,serial_opened=False,serial_bytes_sent=0,FPGA_programmed=False,
  evidence=str(out/'source_excerpts.json'),unknown=['Current physical P11-39 return wire continuity and adapter IO voltage','Actual selected board image and full dual-UART RK mode tests','250ms guard plus UART deadline to actual <=500ms physical standstill under busy navigation','Owner/session management CSR/protocol design and selected RK writer are not implemented yet'])
 assert before==[metadata(p)for p in paths],'audited input changed'
 save(out/'result.json',evidence)
 save(REPO/'docs/4_design/evidence/g2_20261023/P11_39_40_dual_UART_gap_and_completion_plan_20261026.json',evidence)
 print(json.dumps(dict(audit_completed=True,current_P11_control_command_chain_implemented=False,A15_current_binding=locations['imu_rxd'],output=str(out/'result.json')),ensure_ascii=False))

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=BASE/'P11_39_40_dual_UART_contract_audit1');a=p.parse_args();audit(a.out)
