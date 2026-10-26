"""Source-bound conservative first-TS-drop proof for the FW reuse proposal.

This proves a queue accounting condition, not UART timing, actual CPU MMIO
ordering or a clean epoch.  No timestamp-valid/overflow CSR is fabricated.
"""
from pathlib import Path
from datetime import datetime
import argparse,json,re
from prove_soc_console_remap import z3
from prepare_baseline_fusion_retime import metadata,save,same_bytes
BASE=Path('F:/nav_integration_g2_20261023');SOURCE=BASE/'stage3_SLF128_storage_RTL_prepared6';ACTUAL=BASE/'SLF128_storage_safeFW_syn42'

def prove(out):
 assert not out.exists();out.mkdir()
 names=['nav/rtl/io/imu_uart.v','nav/rtl/common/nav_fifo.v','nav/rtl/common/nav_fifo_ob.v','nav/rtl/top/nav_subsystem.v','nav/rtl/regs/nav_regfile.v']
 texts={n:(SOURCE/n).read_text()for n in names};inputs=[metadata(SOURCE/n)for n in names]
 assert all(same_bytes(SOURCE/n,ACTUAL/n)for n in names)
 imu,fifo,ob,sub,regs=[texts[n]for n in names]
 checks={
  'actual_BYTE_RAM_AW8_TS_RAM_AW5':'.FIFO_AW(8), .TS_AW(5)'in sub,
  'common_clock_reset_for_both_fifos':imu.count('.clk(clk), .rst(rst)')==3,
  'only_default_zero_and_first_assert_assign_tpush':len(re.findall(r'\btpush\s*<=',imu))==2 and"push <= 1'b0; tpush <= 1'b0"in imu and"push <= 1'b1; push_d <= {first_pend, sh}"in imu and"if (first_pend) begin tpush <= 1'b1; tpush_d <= t_start"in imu,
  'TS_write_uses_same_registered_tpush':'.s_valid(tpush), .s_data(tpush_d), .s_ready()'in imu,
  'DATA_write_uses_registered_push':'.s_valid(push), .s_data(push_d), .s_ready(rf_ready)'in imu,
  'byte_overflow_observable_but_TS_overflow_not_observable':'if (push && !rf_ready) st_ovf <= st_ovf + 1\'b1' in imu and'assign i_IMU_STAT_OVF = iu_ovf;'in sub,
  'FIFO_ready_RAM_used_full_not_total_capacity':'wire        full = used[AW];'in fifo and'assign s_ready = ~full;'in fifo,
  'total_level_contains_RAM_and_all_held':'assign level   = {1\'b0, used} + {{AW{1\'b0}}, held};'in fifo and"assign held    = cnt + {1'b0, pend};"in ob,
  'DATA6004_valid_and_FIRST_only':'16\'h6004: rdata = i_IMU_RX_DATA;'in regs and"assign i_IMU_RX_DATA  = iu_valid ? {1'b1, 22'd0, iu_first, iu_data} : 32'd0;"in sub,
  'COUNT6008_real_total10bit':"16'h6008: rdata = i_IMU_RX_COUNT;"in regs and"assign i_IMU_RX_COUNT = {22'd0, iu_cnt};"in sub,
  'TS600C_zero_if_invalid_no_valid_bit':"16'h600C: rdata = i_IMU_RX_TS;"in regs and"assign i_IMU_RX_TS    = iu_tsv ? iu_ts : 32'd0;"in sub,
  'OVF601C_real_byte_count':"16'h601C: rdata = i_IMU_STAT_OVF;"in regs,
  'DATA_TS_pop_qualified_by_own_valid':'.rx_pop(p_IMU_RX_DATA & iu_valid)'in sub and'.ts_pop(p_IMU_RX_TS & iu_tsv)'in sub}
 assert all(checks.values()),checks
 # First-drop induction: before a pair, all retained TS records correspond to
 # FIRST-tagged DATA records because the epoch was clean, pushes are paired,
 # and the only consumer pops DATA then exactly its FIRST timestamp.  During
 # one pair, at most one DATA and one TS is popped. Producer insertions may
 # occur between any MMIO operations; there is no invented elapsed-time bound.
 B,F,T,NF,NB,dp,tp=z3.Ints('B F T incoming_first_before_drop incoming_bytes_total DATA_pops TS_pops')
 common=[B>=0,B<=30,F>=0,F<=B,T==F,NF>=0,NB>=NF+1,dp>=0,dp<=1,tp>=0,tp<=dp]
 # A TS RAM-full rejection requires RAM used32; total includes used+held and
 # is therefore at least32, regardless of the34-entry theoretical capacity.
 first_drop=T+NF-tp>=32;post_B=B+NB-dp
 q=z3.Solver();q.add(*common,first_drop,post_B<=30);answer=q.check();assert answer==z3.unsat,str(answer)
 rows=dict(first_TS_drop_with_accepted_post_pair_count_le30=str(answer),query=q.sexpr())
 assert inputs==[metadata(SOURCE/n)for n in names]
 result=dict(at=datetime.now().astimezone().isoformat(),passed=True,scope='Current RTL/MMIO source binding and conditional first-drop accounting only',input_bindings=inputs,source_same_as_completed_SYN42=True,source_checks=checks,Boolean_or_arithmetic_queries=rows,
  exactly_existing_MMIO=['6004 DATA/VALID/FIRST pop','6008 total DATA count','600C TS pop or zero if invalid','601C BYTE overflow count','SYS_TIME_US'],no_TS_valid_or_overflow_CSR_assumed=True,
  conditions=['Clean epoch established by draining both queues and a quiet-source switch, not just a software Boolean assertion','One active DATA consumer, exactly one immediate TS read for each popped FIRST; no diagnostic/R0 raw-window pop while this owner is active','All byte overflow changes quarantine; timeouts and old/zero timestamps never renew commands','Before and after every pair, actual DATA total<=30; a failure does not feed that byte to the parser','Valid production sender emits each command in its own idle-delimited burst; repeated commands in the same burst do not renew'],
  limits=['No actual FIFO/MMIO execution in this proof; native queue model is separate','Clean-epoch/quiet flush implementation and real bus interleavings still require validation','Unconditional arbitrary-host MMIO is not protected: the trusted debug bridge can independently pop6004/600C','No actual service WCET or physical350/500ms stop proof','The rare actual FIRST timestamp exactly0 is conservatively rejected because current MMIO cannot distinguish it from no timestamp'],new_EDA_started=False,physical_accepted=False)
 save(out/'result.json',result);print(json.dumps(dict(passed=True,output=str(out/'result.json'),first_TS_drop_query='UNSAT',additional_CSR_assumed=False)))

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=BASE/'P11_raw_FIFO_source_contract_review1');a=p.parse_args();prove(a.out)
