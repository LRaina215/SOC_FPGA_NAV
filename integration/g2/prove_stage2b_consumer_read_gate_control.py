"""Small Boolean next-state obligations for the frozen bus candidates.

Not an RTL run or actual mapped proof. The bounded source contracts supplying
the cut relations are recorded explicitly; complete data/ACK miters remain due.
"""
from pathlib import Path
from datetime import datetime
import argparse,json,re
from prove_soc_console_remap import z3

BASE=Path('F:/nav_integration_g2_20261023')
def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def need(text,x):assert ' '.join(x.split()) in ' '.join(text.split()),x
def unsat(name,formula):
 s=z3.Solver();s.add(formula);r=s.check();assert r==z3.unsat,(name,r,s.model()if r==z3.sat else '')
 return dict(name=name,result='UNSAT',method='Boolean next-state miter under recorded source cut relations')

def prove():
 cp=BASE/'stage2B_console_posted_prepared1';tp=BASE/'stage2B_TXshadow_RDEN_prepared2'
 c=(cp/'nav/rtl/soc/nav_soc.v').read_text();t=(tp/'nav/rtl/top/nav_subsystem.v').read_text();pico=(cp/'nav/rtl/soc/third_party/picorv32.v').read_text()
 for x in ['wire req     = mem_valid && !mem_ready;','wire        cpu_rst = rst | hold;',"reg         con_post_valid=1'b0;",'if (con_post_valid) con_post_valid <= 1\'b0;',"if (con_we && con_read_needed)","assign con_read_needed = !rst && d_con && (d_st == 2'd1);","2'd1: d_st <= 2'd3;"]:
  need(c,x)
 need(pico,'if (!resetn || trap) begin');need(pico,'if (!resetn || mem_ready)\n\t\t\t\tmem_valid <= 0;')
 block=re.search(r'    reg         con_post_valid=.*?\n    generate',c,re.S)[0]
 assert not any(x in block for x in ['or posedge','or negedge','if (rst)','if (cpu_rst)','if (hold)'])
 need(t,'(bs == B_IDLE) && bus_req &&');need(t,'if (bus_we) rf_wr <= 1\'b1; else rf_rd <= 1\'b1;')
 R,H,V,READY,SEL,LOG,PREV_H,PREV_RST,PREV_CON=z3.Bools('rst hold memvalid ready selper log prevhold prevrst prevconsoleaccept')
 req=z3.And(V,z3.Not(READY));we=z3.And(req,SEL,LOG);read=z3.And(z3.Not(R),PREV_CON)
 # A current d_st1 console request arose from a previous non-reset d_st0
 # console-read accept. That branch did not alter hold. Pico reset clears
 # mem_valid at that prior edge if hold was already active.
 relation=z3.And(z3.Implies(PREV_CON,z3.And(z3.Not(PREV_RST),H==PREV_H)),z3.Implies(PREV_H,z3.Not(V)))
 checks=[unsat('deferred_console_write_cannot_have_cpu_rst',z3.And(relation,read,we,z3.Or(R,H)))]
 deferred=z3.And(read,we)
 per_next=z3.And(z3.Not(z3.Or(R,H)),req,SEL)
 ram_next,nav_next,v_next=z3.Bools('ramnext navnext vnext')
 ready_next=z3.Or(per_next,ram_next,nav_next);req_next=z3.And(v_next,z3.Not(ready_next))
 checks.append(unsat('exact_per_ready_excludes_any_next_CPU_request',z3.And(relation,deferred,req_next)))
 next_state=z3.If(R,z3.IntVal(0),z3.If(PREV_CON,z3.IntVal(3),z3.IntVal(0)))
 next_rst,next_con=z3.Bools('nextrst nextcon');next_read=z3.And(z3.Not(next_rst),next_con,next_state==1)
 checks.append(unsat('deferred_commit_cannot_overlap_next_demanded_read',z3.And(deferred,next_read)))
 pending,wr,rd=z3.Bools('pending currentwrite currentread')
 commit=z3.Or(pending,z3.And(wr,z3.Not(rd)))
 checks.append(unsat('no_read_write_CE_overlap_under_reachable_pending_relation',z3.And(z3.Implies(pending,z3.Not(rd)),commit,rd)))
 # Clock stop stutters all pending/data/address FFs. A protocol reset is absent
 # from their transition; at the next running edge pending drives commit.
 stop,runtime_rst=z3.Bools('stop runtime_reset')
 pending_next=z3.If(stop,pending,z3.And(wr,rd))
 checks.append(unsat('stopped_clock_retains_pending_even_if_reset_changes',z3.And(stop,pending,z3.Not(pending_next))))
 checks.append(unsat('running_pending_commit_not_cancelled_by_runtime_reset',z3.And(pending,runtime_rst,z3.Not(commit))))
 bs=z3.Int('bs');rfwr,txdecode,reset,breq,bwe,qdecode=z3.Bools('rfwr txdecode reset breq bwe qdecode')
 txwe=z3.And(rfwr,txdecode);txread=z3.And(z3.Not(reset),bs==0,breq,qdecode)
 checks.append(unsat('TXshadow_physical_CE_exclusion_for_all_addresses',z3.And(z3.Implies(rfwr,bs==1),txwe,txread)))
 return dict(recorded_at=datetime.now().astimezone().isoformat(),passed=True,checks=checks,console_pending_bits=21,
  source_cut_relations=['Console d_st1 console read is reached only by a previous non-reset d_st0 read accept, which leaves hold unchanged.','PicoRV32 with resetn low at that preceding clock clears mem_valid; this is actual unchanged vendor CPU behavior, not an arbitrary master assumption.','Deferred write implies !cpu_rst and req&&sel_per; actual per_ready becomes1, so next req is0 regardless of new mem_valid/address.','Current d_st1 advances to3, so next demanded read is false; rst only further disables the read.','TX rf_wr pulse is created by B_IDLE and consumed in B_RF; demand read requires B_IDLE.'],
  limitations=['Source cut relations were checked by anchored syntax and manual reachable-state argument; not a complete automated whole-RTL induction.','Bus ACK/data/committed-word miters have not run.','Physical CEA/CEB/INIT/mode and original52 timing constraints require the one combined actual SYN/PNR review.'],
  source_modified=False,RTL_simulator_started=False,Gowin_started=False,programmed=False)

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();r=prove();a.out.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf8');print(json.dumps(dict(passed=True,checks=len(r['checks']),out=str(a.out))))
