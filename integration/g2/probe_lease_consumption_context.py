"""Replay a single lease guard with the actual SYN34 consumption constraints.
Only guard/top/SDC are selected. Two small SYNs at most, in sequence; the second
runs only if the first reproduces200guardFF and the12level all1 prefix.
No joint G2 SYN/PnR, production simulation mutation or board action.
"""
from pathlib import Path
from datetime import datetime
import argparse,collections,json,re,subprocess,time,traceback
import psutil
from probe_lease_bounded_tree_mapping import TREE,GW,LIB
from prepare_lease_deadline_compare import OLD
from prepare_baseline_fusion_retime import save,metadata
from retry_frozen_synthesis import memory_status
from review_lease_deadline_syn_mapping import local_guard,pure_and,hierarchy_resources,report_paths
from revalidate_lidar_cdc import definitions
from synthesis_connectivity import read

BASE=Path('F:/nav_integration_g2_20261023')
SYN34=BASE/'baseline_fusion_model25_NAV50_syn34'
WRAPPER="""module g2_top(input clk,reset_n,input[93:0]raw,output[47:0]result);
 reg[93:0]launched;
 always@(posedge clk)launched<=raw;
 wire valid,source_fault,estop,left_speed_valid,right_speed_valid,feedback_gate_bit;
 wire[15:0]seq;wire signed[19:0]v20,w20;wire signed[15:0]mv,mw;
 assign{valid,source_fault,estop,seq,v20,w20,left_speed_valid,right_speed_valid,feedback_gate_bit,mv,mw}=launched;
 wire signed[23:0]v={{4{v20[19]}},v20},w={{4{w20[19]}},w20};
 wire feedback_valid=left_speed_valid && right_speed_valid && feedback_gate_bit;
 wire signed[23:0]ov,ow;
 command_lease_guard #(.CLK_HZ(50000000)) u_command_lease(
 .clk(clk),.reset_n(reset_n),.owner_select(2'd1),.source_id(2'd1),
 .session_select(32'd1),.packet_session(32'd1),.packet_seq(seq),
 .valid(valid),.source_fault(source_fault),.estop(estop),.forward_q12(v),.yaw_q12(w),
 .feedback_valid(feedback_valid),.measured_vx_mmps(mv),.measured_wz_mradps(mw),
 .out_forward_q12(ov),.out_yaw_q12(ow),.armed(),.expired(),.stop_confirmed(),
 .age_ms(),.accepted_count(),.rejected_count(),.balance_enable());
 assign result={ov,ow};
 endmodule
"""

def profile(d,out):
 vg=d/'impl/gwsynthesis/g2_top.vg';mods=definitions(vg)
 body,cells,params,ffs=local_guard(mods,out,d.name)
 ands=[]
 for c in cells.values():
  if not re.fullmatch(r'LUT[1-6]',c['kind']):continue
  try:entry=pure_and(cells,params,c['ports']['F'][0])
  except AssertionError:continue
  if len(entry['leaves'])>=24:ands.append(entry)
 longest=max(ands,key=lambda x:x['levels'])if ands else None
 return dict(guard_FFs=len(ffs),longest_all1_age_prefix=longest,all_prefixes_support24_or_more=ands,
  resources=hierarchy_resources(d/'impl/gwsynthesis/g2_top_syn_rsc.xml'),estimated_paths=report_paths(d))

def run(out,wait):
 assert not out.exists();out.mkdir(parents=True)
 me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
 state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),output=str(out),joint_G2_SYN_PNR=False,
  production_probe3_modified=False,small_only_admission_phys_GiB=2,small_only_admission_commit_GiB=3,automatic_joint_build=False)
 def status(phase,**kw):state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**kw);save(out/'job.json',state)
 def admission():
  while True:
   mem=memory_status()
   if mem.avail_phys>=2*1024**3 and mem.avail_page>=3*1024**3:return dict(physical_bytes=mem.avail_phys,available_commit_bytes=mem.avail_page)
   assert wait,('small_memory_admission',mem.avail_phys,mem.avail_page)
   status('waiting_small_memory_admission',physical_bytes=mem.avail_phys,available_commit_bytes=mem.avail_page);time.sleep(15)
 def synth(d):
  admission_record=admission()
  with(d/'gw_sh.log').open('w')as log:
   proc=subprocess.Popen([str(GW),'run.tcl'],cwd=d,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW|subprocess.BELOW_NORMAL_PRIORITY_CLASS)
   tool=psutil.Process(proc.pid);identity=tool.create_time();start=time.monotonic();low=0
   rt=dict(PID=tool.pid,created_epoch=identity,started_at=datetime.now().astimezone().isoformat(),admission=admission_record,command=[str(GW),'run.tcl'],selected_inputs=['guard.v','top.v','clock.sdc'],maximum_minutes=40)
   while proc.poll()is None:
    mem=memory_status();danger=mem.avail_page<512*1024**2 or(mem.avail_phys<256*1024**2 and mem.avail_page<1024**3);low=low+1 if danger else 0
    try:children=[tool]+tool.children(recursive=True)
    except psutil.NoSuchProcess:children=[]
    rows=[]
    for c in children:
     try:rows.append(dict(PID=c.pid,created_epoch=c.create_time(),name=c.name(),command=c.cmdline()))
     except(psutil.NoSuchProcess,psutil.AccessDenied):pass
    rt.update(physical_bytes=mem.avail_phys,available_commit_bytes=mem.avail_page,consecutive_low_memory_samples=low,elapsed_seconds=time.monotonic()-start,
     processes=rows)
    save(d/'tool_runtime.json',rt);status('running_small_SYN_'+d.name,tool_runtime=rt)
    reason='sustained_host_memory'if low>=2 else('40minute_budget'if time.monotonic()-start>=2400 else None)
    if reason:
     assert abs(tool.create_time()-identity)<.01
     for c in reversed(children[1:]):
      try:c.kill()
      except psutil.NoSuchProcess:pass
     proc.kill();proc.wait();rt['cancelled_reason']=reason
     raise RuntimeError(reason)
    time.sleep(2)
   rt.update(exit_code=proc.returncode,completed_at=datetime.now().astimezone().isoformat());save(d/'final_runtime.json',rt)
   text=(d/'gw_sh.log').read_text(errors='replace');assert proc.returncode==0 and not re.search(r'^ERROR\b',text,re.M),text[-4000:]
   assert(d/'impl/gwsynthesis/g2_top.vg').exists()
 try:
  original=(SYN34/'command_lease_guard.v').read_text();assert original.count(OLD)==1
  candidate=original.replace(OLD,TREE)
  binding=dict(SYN34=str(SYN34),original_source=metadata(SYN34/'command_lease_guard.v'),
   actual_default_CLK_HZ=50000000,owner_select=1,source_id=1,session_select=1,packet_session=1,
   consumed_guard_outputs=['out_forward_q12[23:0]','out_yaw_q12[23:0]'],all_other_guard_outputs_unconnected=True,
   command_inputs='actual bits0..18 and23:20 independent signed bits; original command19..22 alias23',
   feedback_expression='left_speed_valid && right_speed_valid && feedback_gate_bit; matching actual shared odo_valid cone',
   external_logic_not_replayed='Actual producer UART/command-unit, sensor encoder and consumer modules are not instantiated. This run tests known constant/sign/output-use contexts, not complete wholeG2 optimization context.',
   source_bindings=[metadata(SYN34/'g2_top.v'),metadata(SYN34/'impl/gwsynthesis/g2_top.vg')])
  save(out/'wrapper_context.json',binding)
  status('prepared_actual_consumption_reference')
  for label,source in [('reference',original),('bounded_tree',candidate)]:
   d=out/label;d.mkdir()
   for name,text in [('guard.v',source),('top.v',WRAPPER),('clock.sdc','create_clock -name clk50 -period 20 [get_ports {clk}]\n')]:
    (d/name).write_text(text,encoding='utf-8',newline='\n')
   (d/'run.tcl').write_text(f"""set_device -device_version C GW5AT-LV138PG484AC1/I0
add_file -type verilog {{{(d/'guard.v').as_posix()}}}
add_file -type verilog {{{(d/'top.v').as_posix()}}}
add_file -type sdc {{{(d/'clock.sdc').as_posix()}}}
set_option -top_module g2_top
set_option -output_base_name g2_top
set_option -verilog_std v2001
set_option -global_freq 50
run syn
exit
""",encoding='utf-8',newline='\n')
   synth(d);result=profile(d,out);save(out/(label+'_profile.json'),result)
   if label=='reference':
    prefix=result['longest_all1_age_prefix'];matched=result['guard_FFs']==200 and prefix is not None and prefix['levels']==12 and len(prefix['leaves'])==32
    status('actual_reference_context_checked',reference_reproduces200FF_and12level_prefix=matched,reference_guard_FFs=result['guard_FFs'],reference_longest_prefix_levels=None if prefix is None else prefix['levels'])
    if not matched:
     save(out/'result.json',dict(passed=True,context_reproduced=False,reference=result,second_SYN_not_started=True,
      reason='Known constants/sign aliases/output-consumption constraints did not reproduce original200FF+12level prefix. Additional cross-hierarchy optimization/producer-consumer context remains; do not tune multiple wrappers or infer wholeG2 timing.',joint_SYN_PNR=False))
     status('stopped_no_context_reproduction_no_second_SYN',context_reproduced=False);return 0
  save(out/'result.json',dict(passed=True,context_reproduced=True,paired_mapping_complete=True,actual_equivalence_and_topology_review_pending=True,joint_SYN_PNR=False))
  status('matching_context_pair_complete_review_pending',context_reproduced=True);return 0
 except Exception as exc:
  save(out/'failed.json',dict(passed=False,reason=str(exc),traceback=traceback.format_exc()));status('failed',reason=str(exc));return 1

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--wait-for-memory',action='store_true');a=p.parse_args();raise SystemExit(run(a.out,a.wait_for_memory))
