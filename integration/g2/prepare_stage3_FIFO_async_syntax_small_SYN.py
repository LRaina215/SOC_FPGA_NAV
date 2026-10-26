"""Freeze one C-version FIFO-only synthesis; explicit start, no joint build/PNR."""
from pathlib import Path
from datetime import datetime
from collections import Counter
import argparse,json,re,shutil,subprocess,time,traceback
import psutil
from prepare_baseline_fusion_retime import load,save,metadata,same_bytes
from retry_frozen_synthesis import memory_status
from synthesis_connectivity import read
from revalidate_lidar_cdc import definitions
from review_small_block_stores import parameter_reader
from audit_netlist_memories import BSRAM,PARAM
from audit_routing_topology import primitive_directions

ROOT=Path('F:/nav_integration_g2_20261023')
BASE=ROOT/'stage3_SLF128_storage_RTL_prepared5'
SYN40=ROOT/'baseline_fusion50_ring_MAX_syn40'
GW=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe')
GIB=1024**3
CONFIGS={'u_rx11':dict(DW=10,AW=11,write_clock='rgmii_rxc',read_clock='clk50',write_reset='rx_rst',read_reset='core_rst',old_scope='g2_top/u_nav/u_netl/u_rxf/'),
 'u_rx12':dict(DW=10,AW=12,write_clock='clk50',read_clock='clk50',write_reset='core_rst',read_reset='core_rst',old_scope=None),
 'u_tx11':dict(DW=9,AW=11,write_clock='clk50',read_clock='clk50',write_reset='core_rst',read_reset='core_rst',old_scope='g2_top/u_nav/u_netl/u_txf/')}
WRAPPER='''module g2_top(input clk50,rgmii_rxc,core_rst,rx_rst,
 input[2:0]s_valid,m_ready,input[28:0]s_data,
 output[2:0]s_ready,m_valid,output[28:0]m_data,output[36:0]wfree);
nav_async_fifo #(.DW(10),.AW(11))u_rx11(.wclk(rgmii_rxc),.wrst(rx_rst),.s_valid(s_valid[0]),.s_data(s_data[9:0]),.s_ready(s_ready[0]),.wfree(wfree[11:0]),.rclk(clk50),.rrst(core_rst),.m_valid(m_valid[0]),.m_data(m_data[9:0]),.m_ready(m_ready[0]));
nav_async_fifo #(.DW(10),.AW(12))u_rx12(.wclk(clk50),.wrst(core_rst),.s_valid(s_valid[1]),.s_data(s_data[19:10]),.s_ready(s_ready[1]),.wfree(wfree[24:12]),.rclk(clk50),.rrst(core_rst),.m_valid(m_valid[1]),.m_data(m_data[19:10]),.m_ready(m_ready[1]));
nav_async_fifo #(.DW(9),.AW(11))u_tx11(.wclk(clk50),.wrst(core_rst),.s_valid(s_valid[2]),.s_data(s_data[28:20]),.s_ready(s_ready[2]),.wfree(wfree[36:25]),.rclk(clk50),.rrst(core_rst),.m_valid(m_valid[2]),.m_data(m_data[28:20]),.m_ready(m_ready[2]));
endmodule
'''

def stamp():return datetime.now().astimezone().isoformat()
def unchanged(rows):
 for row in rows:assert metadata(Path(row['path']))==row,('input changed',row)

def reference_RAMs():
 inventory=load(SYN40/'memory_inventory.json')
 leaves=[r for r in inventory['leaves']if any(c['old_scope']and r['path'].startswith(c['old_scope'])for c in CONFIGS.values())]
 wanted={r['module_type']for r in leaves};bodies={};kind=None;lines=[]
 # Stream only the few existing FIFO definitions; do not expand the full G2.
 with(SYN40/'impl/gwsynthesis/g2_top.vg').open(encoding='utf8')as stream:
  for line in stream:
   if line.startswith('module '):
    name=line.split()[1];kind=name if name in wanted else None;lines=[]
   if kind:lines.append(line)
   if kind and line.startswith('endmodule'):bodies[kind]=''.join(lines);kind=None
 assert set(bodies)==wanted
 result={}
 for name,cfg in CONFIGS.items():
  selected=[r for r in leaves if cfg['old_scope']and r['path'].startswith(cfg['old_scope'])]
  rows=[]
  for row in selected:
   instance=row['path'].rsplit('/',1)[1]
   params={key:value.strip()for inst,key,value in PARAM.findall(bodies[row['module_type']])if inst.strip().lstrip('\\')==instance}
   rows.append(dict(path=row['path'],kind=row['primitive'],parameters=params))
  result[name]=dict(actual_reference_available=bool(rows),RAMs=rows,
   limitation=None if rows else 'Host RX AW12 was removed in full SYN40; new small geometry will be reported without claiming an old mapped match.')
 return result

def prepare(out):
 out=out.resolve();assert not out.exists();out.mkdir(parents=True)
 (out/'top.v').write_text(WRAPPER,encoding='utf8',newline='\n')
 (out/'clock.sdc').write_text('create_clock -name clk50 -period 20 [get_ports {clk50}]\ncreate_clock -name rgmii_rxc -period 40 [get_ports {rgmii_rxc}]\n',encoding='utf8')
 (out/'run.tcl').write_text('error {SOURCE NOT BOUND: FIFO syntax candidate must be frozen before an explicit start}\n',encoding='utf8')
 save(out/'reference_RAMs.json',reference_RAMs())
 save(out/'entry.json',dict(stage='prepared_source_pending',device='GW5AT-LV138PG484AC1/I0',device_version='C',
  SYN_only=True,configs=CONFIGS,source_files=2,assets=0,CPU=False,PNR=False,automatic_start=False,
  admission_physical_bytes=2*GIB,admission_commit_bytes=4*GIB,max_runtime_seconds=2400,
  runtime_memory_guard='Two consecutive 2s samples: commit<512MiB OR physical<256MiB and commit<1GiB; kill only owned original identities.',
  public_nav_fifo_ob_source=str(BASE/'nav/rtl/common/nav_fifo_ob.v'),public_module_elaborated=False))
 save(out/'job.json',dict(phase='prepared_waiting_frozen_FIFO_source',SYN_started=False,PNR_started=False,created_at=stamp()))
 return out

def bind(out,source):
 out=out.resolve();source=source.resolve();assert (out/'entry.json').exists()and not(out/'frozen_inputs.json').exists()and not(out/'impl').exists()
 original=BASE/'nav/rtl/common/nav_async_fifo.v';old=original.read_text(encoding='utf8');expected=old
 changes=[]
 for clk,rst,reg,gray in [('wclk','wrst','rp_b_w','rp_g_w2'),('rclk','rrst','wp_b_r','wp_g_r2')]:
  before=f"always @(posedge {clk} or posedge {rst}) {reg} <= {rst} ? {{AW+1{{1'b0}}}} : g2b({gray});"
  # Whitespace is not part of the RTL contract; all non-whitespace tokens must
  # match the exact two reviewed outer-if changes and retained inner ternaries.
  after=f"always @(posedge {clk} or posedge {rst}) begin if ({rst}) {reg} <= {{AW+1{{1'b0}}}}; else {reg} <= {rst} ? {{AW+1{{1'b0}}}} : g2b({gray}); end"
  assert expected.count(before)==1;expected=expected.replace(before,after);changes.append(reg)
 assert re.sub(r'\s+','',source.read_text(encoding='utf8'))==re.sub(r'\s+','',expected),'source differs beyond exact two outer-if forms'
 assert same_bytes(BASE/'nav/rtl/common/nav_fifo_ob.v',SYN40/'nav/rtl/common/nav_fifo_ob.v')
 shutil.copyfile(source,out/'nav_async_fifo.v')
 tcl=f'''set_device -device_version C GW5AT-LV138PG484AC1/I0
add_file -type verilog {{{(out/'nav_async_fifo.v').as_posix()}}}
add_file -type verilog {{{(out/'top.v').as_posix()}}}
add_file -type sdc {{{(out/'clock.sdc').as_posix()}}}
set_option -top_module g2_top
set_option -output_base_name g2_top
set_option -verilog_std v2001
set_option -global_freq 50
run syn
exit
'''
 (out/'run.tcl').write_text(tcl,encoding='utf8',newline='\n')
 inputs=[out/x for x in ('nav_async_fifo.v','top.v','clock.sdc','run.tcl','reference_RAMs.json','entry.json')]+[source,original,BASE/'nav/rtl/common/nav_fifo_ob.v',Path(__file__).resolve()]
 save(out/'frozen_inputs.json',dict(inputs=[metadata(p)for p in inputs],candidate_source=str(source),exact_two_outer_if_changes=changes,
  RAM_write_and_read_processes_unchanged=True,public_nav_fifo_ob_byte_identical_SYN40=True,inner_reset_ternary_retained_for_XZ=True))
 save(out/'job.json',dict(phase='all_sources_frozen_not_started',SYN_started=False,PNR_started=False,bound_at=stamp()))

def canonical_RAM(rows):return Counter(json.dumps(dict(kind=r['kind'],parameters=r['parameters']),sort_keys=True)for r in rows)

def review(out):
 vg=out/'impl/gwsynthesis/g2_top.vg';cells=read(vg);modules=definitions(vg);params=parameter_reader(modules)
 directions=primitive_directions();drivers={}
 for cell in cells.values():
  for pin,nets in cell['ports'].items():
   if directions[cell['kind']][pin]=='output':
    for net in nets:drivers.setdefault(net,[]).append(cell)
 def source_of(net):
  return dict(net=net,top_input_or_constant=not drivers.get(net),drivers=[dict(cell=c,parameters=params(c))for c in drivers.get(net,[])])
 reference=load(out/'reference_RAMs.json');groups={};passed=True
 for scope,cfg in CONFIGS.items():
  subset=[c for c in cells.values()if c['name'].startswith(scope+'/')]
  memories=[dict(path=c['name'],kind=c['kind'],parameters=params(c),ports=c['ports'])for c in subset if c['kind']in BSRAM]
  ram_match=canonical_RAM(memories)==canonical_RAM(reference[scope]['RAMs'])if reference[scope]['actual_reference_available']else None
  converted=[];valid=True
  for reg,rst,clk in [('rp_b_w',cfg['write_reset'],cfg['write_clock']),('wp_b_r',cfg['read_reset'],cfg['read_clock'])]:
   for bit in range(cfg['AW']+1):
    q=f'{scope}/{reg}[{bit}]';matches=[c for c in subset if c['ports'].get('Q')==[q]]
    ok=len(matches)==1 and matches[0]['kind']=='DFFCE'and matches[0]['ports'].get('CLEAR')==[rst]and matches[0]['ports'].get('CLK')==[clk]
    converted.append(dict(signal=q,passed=ok,actual=matches));valid&=ok
  FFs=[dict(cell=c,parameters=params(c),pin_sources={pin:[source_of(net)for net in nets]for pin,nets in c['ports'].items()if pin!='Q'})for c in subset if c['kind'].startswith('DFF')]
  async_FFs=[r for r in FFs if r['cell']['kind']=='DFFCE']
  synchronous_reset_FFs=[r for r in FFs if 'RESET'in r['cell']['ports']]
  clear_ok=all(r['cell']['ports'].get('CLEAR')==[cfg['write_reset']]if r['cell']['ports'].get('CLK')==[cfg['write_clock']]and cfg['write_clock']!=cfg['read_clock']else r['cell']['ports'].get('CLEAR')==[cfg['read_reset']]for r in async_FFs)
  groups[scope]=dict(config=cfg,primitive_counts=dict(Counter(c['kind']for c in subset)),FF_counts=dict(Counter(c['kind']for c in subset if c['kind'].startswith('DFF'))),
   RAMs=memories,RAM_all_explicit_parameters_match_SYN40=ram_match,converted_pointers_all_async_clear=valid,converted_pointer_bits=converted,
   every_actual_FF_with_all_clock_reset_CE_data_sources=FFs,async_clear_FF_count=len(async_FFs),all_async_clear_sources_match_domain_reset=clear_ok,
   synchronous_reset_FFs=synchronous_reset_FFs,logical_state_alias_limit='All actual FFs are saved, including pointer/synchronizer/occupancy and payload. Optimized logical aliases are not guessed from old numbering.')
  passed&=valid and bool(memories)and ram_match is not False and clear_ok and not synchronous_reset_FFs
 logs=(out/'gw_sh.log').read_text(errors='replace')+(out/'small_SYN_stdout.log').read_text(errors='replace')
 assert 'EX3534'not in logs
 return dict(passed=passed,EX3534_absent=True,groups=groups,FF_total=sum(c['kind'].startswith('DFF')for c in cells.values()),
  BSRAM_total=sum(c['kind']in BSRAM for c in cells.values()),all_primitive_counts=dict(Counter(c['kind']for c in cells.values())),
  public_nav_fifo_ob_unchanged=same_bytes(BASE/'nav/rtl/common/nav_fifo_ob.v',SYN40/'nav/rtl/common/nav_fifo_ob.v'),
  physical_mapping_timing_or_async_recovery_removal_proved=False,full_G2_SYN_or_PNR_accepted=False)

def start(out):
 out=out.resolve();proof=load(out/'frozen_inputs.json');unchanged(proof['inputs']);assert not(out/'run_claim.json').exists()and not(out/'impl').exists()
 me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
 state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),build=str(out),PNR_started=False,programmed=False,automatic_retry=False)
 with(out/'run_claim.json').open('x',encoding='utf8')as stream:json.dump(state,stream)
 def status(phase,**kw):state.update(phase=phase,updated_at=stamp(),**kw);save(out/'job.json',state)
 try:
  while True:
   mem=memory_status();status('waiting_2GiB_4GiB_admission',physical_bytes=mem.avail_phys,commit_bytes=mem.avail_page)
   if mem.avail_phys>=2*GIB and mem.avail_page>=4*GIB:break
   time.sleep(15)
  unchanged(proof['inputs']);runtime=dict(command=[str(GW),'run.tcl'],admission_physical_bytes=mem.avail_phys,admission_commit_bytes=mem.avail_page,max_runtime_seconds=2400)
  with(out/'small_SYN_stdout.log').open('w',encoding='utf8')as log:
   child=subprocess.Popen(runtime['command'],cwd=out,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW|subprocess.BELOW_NORMAL_PRIORITY_CLASS)
   proc=psutil.Process(child.pid);runtime.update(PID=child.pid,created_epoch=proc.create_time(),started_at=stamp());seen={};began=time.monotonic();sampled=0;low=0
   while child.poll()is None:
    try:owned=[proc]+proc.children(recursive=True)
    except psutil.NoSuchProcess:owned=[]
    for p in owned:
     try:row=dict(pid=p.pid,created_epoch=p.create_time(),name=p.name(),command=p.cmdline());seen[(p.pid,row['created_epoch'])]=row
     except(psutil.NoSuchProcess,psutil.AccessDenied):pass
    now=time.monotonic()
    if now-sampled>=2:
     mem=memory_status();low=low+1 if mem.avail_page<512*1024**2 or(mem.avail_phys<256*1024**2 and mem.avail_page<GIB)else 0;sampled=now
     runtime.update(physical_bytes=mem.avail_phys,commit_bytes=mem.avail_page,consecutive_low_memory_samples=low)
    reason='sustained_host_memory'if low>=2 else 'runtime_2400s'if now-began>=2400 else None
    runtime.update(elapsed_seconds=now-began,processes=list(seen.values()));save(out/'tool_runtime.json',runtime);status('running_FIFO_only_small_SYN',runtime=runtime)
    if reason:
     for p in reversed(owned):
      try:
       row=seen[(p.pid,p.create_time())];p.kill()
      except(psutil.NoSuchProcess,psutil.AccessDenied,KeyError):pass
     child.wait();runtime['cancelled_reason']=reason;raise RuntimeError(reason)
    time.sleep(.5)
   runtime.update(exit_code=child.returncode,ended_at=stamp());save(out/'small_SYN_runtime.json',runtime);assert child.returncode==0,child.returncode
  unchanged(proof['inputs']);result=review(out);result.update(state,runtime=runtime,phase='small_SYN_complete',completed_at=stamp(),frozen_inputs_unchanged=True)
  save(out/'result.json',result);save(out/'job.json',result);return 0 if result['passed']else 1
 except Exception as error:
  result=dict(state,passed=False,phase='failed_preserved_no_retry',error=str(error),traceback=traceback.format_exc());save(out/'result.json',result);save(out/'job.json',result);return 1

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True);p.add_argument('--source',type=Path)
 m=p.add_mutually_exclusive_group(required=True);m.add_argument('--prepare',action='store_true');m.add_argument('--bind-source',action='store_true');m.add_argument('--start',action='store_true');a=p.parse_args()
 if a.prepare:print(prepare(a.out))
 elif a.bind_source:assert a.source;bind(a.out,a.source);print('all FIFO-only inputs frozen; no tool started')
 else:raise SystemExit(start(a.out))
