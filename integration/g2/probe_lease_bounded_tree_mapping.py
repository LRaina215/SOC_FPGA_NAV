"""Small paired synthesis and exact checks for an explicit3level MAX tree.

No joint G2 synthesis/PnR or hardware action. Only the lease guard is mapped
with100 identical input-launch FFs and the production50MHz/200ms parameters.
"""
from pathlib import Path
from datetime import datetime
import argparse,collections,difflib,json,re,shutil,subprocess,time,traceback
import psutil
from bs4 import BeautifulSoup
from prepare_baseline_fusion_retime import metadata,save
from prepare_lease_deadline_compare import OLD
from prove_soc_console_remap import z3,Logic,literal
from synthesis_connectivity import read
from revalidate_lidar_cdc import definitions
from review_small_block_stores import parameter_reader
from audit_routing_topology import primitive_directions
from retry_frozen_synthesis import memory_status

ROOT=Path(__file__).resolve().parents[2]
BASE=Path('F:/nav_integration_g2_20261023')
ORIGINAL=BASE/'baseline_fusion_model25_NAV50_prepared4/command_lease_guard.v'
REFERENCE=BASE/'baseline_fusion_model25_deadline_compare_prepared4/command_lease_guard.v'
LIB=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
VIV=Path('E:/2026.1/Vivado/bin')
GW=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe')
TREE=""" localparam [31:0] LEASE_TIMEOUT_U32=TIMEOUT_MS;
 wire lease_age_due;
 generate if(LEASE_TIMEOUT_U32==32'd0)begin:g_deadline_zero
  assign lease_age_due=(age_ms+32'd1)>=LEASE_TIMEOUT_U32;
 end else begin:g_deadline_nonzero
  wire [7:0] max4 /* synthesis syn_keep=1 */;
  wire [1:0] max16 /* synthesis syn_keep=1 */;
  wire nonmax32 /* synthesis syn_keep=1 */;
  genvar group;
  for(group=0;group<8;group=group+1)begin:g_max4
   LUT4 u_and(.F(max4[group]),.I0(age_ms[group*4]),.I1(age_ms[group*4+1]),
              .I2(age_ms[group*4+2]),.I3(age_ms[group*4+3]));
   defparam u_and.INIT=16'h8000;
  end
  LUT4 u_low16(.F(max16[0]),.I0(max4[0]),.I1(max4[1]),.I2(max4[2]),.I3(max4[3]));
  defparam u_low16.INIT=16'h8000;
  LUT4 u_high16(.F(max16[1]),.I0(max4[4]),.I1(max4[5]),.I2(max4[6]),.I3(max4[7]));
  defparam u_high16.INIT=16'h8000;
  LUT2 u_nonmax32(.F(nonmax32),.I0(max16[0]),.I1(max16[1]));
  defparam u_nonmax32.INIT=4'h7;
  assign lease_age_due=(age_ms>=LEASE_TIMEOUT_U32-32'd1) && nonmax32;
 end endgenerate
 wire deadline=tick && have_lease && lease_age_due;"""

WRAPPER="""module g2_top(input clk,reset_n,input[99:0]raw,output[147:0]result);
 reg[99:0]launched;
 always@(posedge clk)launched<=raw;
 wire valid,fault,estop,feedback;wire[15:0]seq;
 wire signed[23:0]v,w;wire signed[15:0]mv,mw;
 assign {valid,fault,estop,seq,v,w,feedback,mv,mw}=launched;
 wire signed[23:0]ov,ow;wire armed,expired,stopped,balance;
 wire[31:0]age,accepted,rejected;
 command_lease_guard #(.CLK_HZ(50000000)) dut(clk,reset_n,2'd1,2'd1,32'd1,32'd1,
 seq,valid,fault,estop,v,w,feedback,mv,mw,ov,ow,armed,expired,stopped,age,accepted,rejected,balance);
 assign result={ov,ow,armed,expired,stopped,age,accepted,rejected,balance};
 endmodule
"""

def run(out,wait,resume=False,minimum_commit_gib=4):
 assert minimum_commit_gib in (3,4)
 if resume:
  assert out.is_dir() and (out/'first_wait_controller_identity_and_adjustment.json').exists()
  assert not list(out.rglob('*runtime.json')),'resume only before first tool'
  assert not (out/'reference').exists() and not (out/'bounded_tree').exists()
 else:
  assert not out.exists();out.mkdir(parents=True)
 me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
 state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),output=str(out),
  joint_SYN_PNR=False,programmed=False,production_probe3_modified=False,automatic_joint_build=False,
  resumed_prepared=resume,small_only_minimum_commit_gib=minimum_commit_gib,joint_build_threshold_unchanged=True)
 def status(phase,**kw):
  state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**kw);save(out/'job.json',state)
 def admission(label):
  while True:
   mem=memory_status()
   if mem.avail_phys>=2*1024**3 and mem.avail_page>=minimum_commit_gib*1024**3:return dict(physical_bytes=mem.avail_phys,available_commit_bytes=mem.avail_page)
   assert wait,('memory admission',label,mem.avail_phys,mem.avail_page)
   status('waiting_memory_admission',next_tool=label,physical_bytes=mem.avail_phys,available_commit_bytes=mem.avail_page);time.sleep(15)
 def command(cwd,label,argv):
  entry=admission(label)
  with(cwd/(label+'.log')).open('w')as log:
   proc=subprocess.Popen(argv,cwd=cwd,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW|subprocess.BELOW_NORMAL_PRIORITY_CLASS)
   process=psutil.Process(proc.pid);identity=process.create_time()
   runtime=dict(tool=label,command=argv,PID=proc.pid,created_epoch=identity,started_at=datetime.now().astimezone().isoformat(),admission=entry,PNR=False,max_runtime_seconds=2400 if label=='small_SYN' else 240)
   started=time.monotonic();low=0;last_memory_check=0
   while proc.poll()is None:
    rows=[]
    try:children=[process]+process.children(recursive=True)
    except psutil.NoSuchProcess:children=[]
    for child in children:
     try:rows.append(dict(pid=child.pid,created_epoch=child.create_time(),parent_pid=child.ppid(),name=child.name(),command=child.cmdline(),cwd=child.cwd()))
     except(psutil.NoSuchProcess,psutil.AccessDenied):pass
    now=time.monotonic()
    if now-last_memory_check>=2:
     mem=memory_status();danger=mem.avail_page<512*1024**2 or (mem.avail_phys<256*1024**2 and mem.avail_page<1024**3)
     low=low+1 if danger else 0;last_memory_check=now
     runtime.update(physical_bytes=mem.avail_phys,available_commit_bytes=mem.avail_page,consecutive_low_memory_samples=low)
    reason='sustained_host_memory' if low>=2 else ('runtime_budget' if now-started>=runtime['max_runtime_seconds'] else None)
    runtime.update(updated_at=datetime.now().astimezone().isoformat(),elapsed_seconds=now-started,processes=rows)
    save(cwd/'tool_runtime.json',runtime);status('running_'+label,tool_runtime=runtime)
    if reason:
     assert abs(process.create_time()-identity)<.01
     for child in reversed(children[1:]):
      try:child.kill()
      except psutil.NoSuchProcess:pass
     proc.kill();proc.wait();runtime.update(cancelled_reason=reason,exit_code=proc.returncode)
     save(cwd/(label+'_runtime.json'),runtime)
     raise RuntimeError(reason+' stopped only owned '+label)
    time.sleep(.5)
   runtime.update(exit_code=proc.returncode,completed_at=datetime.now().astimezone().isoformat());save(cwd/(label+'_runtime.json'),runtime)
   assert proc.returncode==0,(label,proc.returncode)
 try:
  old=ORIGINAL.read_text();candidate=old.replace(OLD,TREE);sim=out/'focused_cycle'
  if resume:
   assert old.count(OLD)==1
   assert (out/'command_lease_guard.v').read_text()==candidate
   assert (sim/'candidate.v').read_text()==candidate
   assert (sim/'original.v').read_text()==old.replace('module command_lease_guard #(','module original_guard #(',1)
   assert (sim/'tb.sv').read_bytes()==(ROOT/'integration/g2/tb_lease_deadline_compare.sv').read_bytes()
   assert (sim/'deadline_math_candidate.vh').read_text()==TREE
   assert json.loads((out/'math_proof.json').read_text())['passed'] is True
   status('resume_prepared_first_tool_no_source_or_proof_rewrite')
  else:
   status('freeze_one_expression_bounded_tree')
   old=ORIGINAL.read_text();assert old.count(OLD)==1
   candidate=old.replace(OLD,TREE);assert candidate.replace(TREE,OLD)==old
   (out/'command_lease_guard.v').write_text(candidate,encoding='utf-8',newline='\n')
   (out/'delta.patch').write_text(''.join(difflib.unified_diff(old.splitlines(True),candidate.splitlines(True),fromfile='original/command_lease_guard.v',tofile='bounded/command_lease_guard.v')),encoding='utf-8')
   # Check all32bit ages and integer parameter bit patterns using the exact
   # explicit LUT truth functions, not an assumed balanced synthesized cone.
   age,t=z3.BitVecs('age32 timeout32',32)
   groups=[z3.And(*[z3.Extract(i,i,age)==1 for i in range(g*4,g*4+4)])for g in range(8)]
   nonmax=z3.Not(z3.And(z3.And(*groups[:4]),z3.And(*groups[4:])))
   original=z3.UGE(age+z3.BitVecVal(1,32),t)
   bounded=z3.If(t==0,original,z3.And(z3.UGE(age,t-z3.BitVecVal(1,32)),nonmax))
   solver=z3.Solver();solver.set(timeout=30000);solver.add(z3.Xor(original,bounded));(out/'all_age_timeout_tree_miter.smt2').write_text(solver.to_smt2())
   assert solver.check()==z3.unsat
   save(out/'math_proof.json',dict(passed=True,status='unsat',known_age_values=2**32,known_parameter_bit_patterns=2**32,no_reachability_assumption=True,
    source_binding=metadata(out/'command_lease_guard.v'),explicit_tree_LUTs=11,expected_age_Q_to_nonmax_LUT_levels=3,
    timeout0_retains_original_arithmetic_for_four_state_behavior=True,new_mapping_unknown_until_checked=True))
   # Reuse only the focused deadline/renewal/reset fixture, with its exact math
   # block replaced by TREE and the unmodified public vendor LUT/MUX models.
   sim=out/'focused_cycle';sim.mkdir()
   (sim/'original.v').write_text(old.replace('module command_lease_guard #(','module original_guard #(',1))
   shutil.copyfile(out/'command_lease_guard.v',sim/'candidate.v')
   shutil.copyfile(ROOT/'integration/g2/tb_lease_deadline_compare.sv',sim/'tb.sv')
   (sim/'deadline_math_candidate.vh').write_text(TREE)
   lib=LIB.read_text();models=[]
   for name in('LUT4','LUT2','MUX16','MUX8','MUX4','MUX2'):
    match=re.search(r'\bmodule\s+'+name+r'\b[\s\S]*?\bendmodule',lib);assert match;models.append(match[0])
   udp=re.search(r'\bprimitive\s+mux2\b[\s\S]*?\bendprimitive',lib);assert udp;models.append(udp[0])
   (sim/'public_vendor_LUT_models.v').write_text('`timescale 1ns/1ps\n'+'\n'.join(models))
  for label,tool,args in[('xvlog_source','xvlog',['public_vendor_LUT_models.v','original.v','candidate.v']),('xvlog_TB','xvlog',['-sv','tb.sv']),
    ('xelab','xelab',['tb_lease_deadline_compare','-s','bounded_snap']),('xsim','xsim',['bounded_snap','-runall'])]:
   command(sim,label,['cmd','/c',str(VIV/(tool+'.bat')),*args])
  log=(sim/'xsim.log').read_text(errors='replace');summary=re.search(r'LEASE_DEADLINE_COMPARE_PASS[^\r\n]+',log)
  assert summary and'Fatal:'not in log,log[-2500:]
  save(out/'focused_cycle_result.json',dict(passed=True,summary=summary[0],public_vendor_models_unmodified=True,
   no_force=True,scaled_guard_clock_parameter=4000,production_model_probe3_clock_not_modified=True,
   physical_guard_timeout_or_stop_latency_not_accepted=True,full_model_probe3_untouched=True))
  # Two small guards, identical timing shell and actual50MHz constants. Never
  # add simulation-model modules to synthesis; LUT4/LUT2 are native primitives.
  for label,source in[('reference',REFERENCE.read_text()),('bounded_tree',candidate)]:
   d=out/label;d.mkdir();(d/'guard.v').write_text(source);(d/'top.v').write_text(WRAPPER)
   (d/'clock.sdc').write_text('create_clock -name clk50 -period 20 [get_ports {clk}]\n')
   (d/'run.tcl').write_text(f'''set_device -device_version C GW5AT-LV138PG484AC1/I0
add_file -type verilog {{{(d/'guard.v').as_posix()}}}
add_file -type verilog {{{(d/'top.v').as_posix()}}}
add_file -type sdc {{{(d/'clock.sdc').as_posix()}}}
set_option -top_module g2_top
set_option -output_base_name g2_top
set_option -verilog_std v2001
set_option -global_freq 50
run syn
exit
''')
   command(d,'small_SYN',[str(GW),'run.tcl'])
   assert(d/'impl/gwsynthesis/g2_top.vg').exists()
  status('small_pair_complete_actual_topology_and_equivalence_review_pending',focused_cycle_passed=True,math_passed=True,
   actual_topology_goal_accepted=False,automatic_joint_build=False)
  return 0
 except Exception as exc:
  save(out/'failed.json',dict(passed=False,reason=str(exc),traceback=traceback.format_exc()));status('failed',passed=False,error=str(exc));return 1

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--wait-for-memory',action='store_true');p.add_argument('--resume-prepared',action='store_true');p.add_argument('--minimum-commit-gib',type=int,default=4);a=p.parse_args()
 raise SystemExit(run(a.out,a.wait_for_memory,a.resume_prepared,a.minimum_commit_gib))
