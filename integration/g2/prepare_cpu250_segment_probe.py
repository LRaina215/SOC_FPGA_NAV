"""Prepare isolated, instrumented CPU segment measurements from a frozen FW.

Preparation and optional cross compilation only.  This never edits the frozen
firmware or starts a simulator.  Run the generated run_xsim.py explicitly later
after resource admission.  The small SoC harness has no protected vendor models
and does not consume the occupied ModelSim licence.

Measurements include instrumentation and the synthetic register fixture.  They
are finite scenarios, not the full navigation loop or a WCET/standstill proof.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SOURCE = Path('F:/nav_integration_g2_20261023/baseline_fusion_preview_safety250_emptyIMU_prepared3')
GCC_DIR = Path('D:/tools/xpack-riscv-none-elf-gcc-15.2.0-1/bin')


def patch_once(text: str, old: str, new: str) -> str:
    if text.count(old) != 1:
        raise ValueError(f'Expected one exact instrumentation site, found {text.count(old)}: {old[:100]}')
    return text.replace(old, new, 1)


def instrument_astar(text: str) -> str:
    text = '#include "segment_probe.h"\n' + text
    text = patch_once(text, '    if(grid!=s_cost) for(int i=0;i<n*n;i++) s_cost[i]=grid[i];',
        '    SP_BEGIN(1);\n    if(grid!=s_cost) for(int i=0;i<n*n;i++) s_cost[i]=grid[i];\n    SP_END(1);')
    text = patch_once(text, '    dist_transform(cfg->block,nav_max32(cfg->clear_cells,cfg->near_cells));',
        '    SP_BEGIN(2);\n    dist_transform(cfg->block,nav_max32(cfg->clear_cells,cfg->near_cells));\n    SP_END(2);\n    SP_BEGIN(3);')
    text = patch_once(text, '    return astar_continue(out,limit);',
        '    SP_END(3);\n    return astar_continue(out,limit);')
    text = patch_once(text, '    while (s_hn > 0 && (limit<=0 || work<limit)) {',
        '    SP_BEGIN(4);\n    while (s_hn > 0 && (limit<=0 || work<limit)) {')
    text = patch_once(text, '    s_best=best;s_best_h=best_h;', '    SP_END(4);\n    s_best=best;s_best_h=best_h;')
    text = patch_once(text, '    uint16_t *tmp = (uint16_t *)out->wx;',
        '    SP_BEGIN(5);\n    uint16_t *tmp = (uint16_t *)out->wx;')
    text = patch_once(text, '    out->n = kept;', '    out->n = kept;\n    SP_END(5);')
    return text


def instrument_frontier(text: str) -> str:
    text = '#include "segment_probe.h"\n' + text
    text = patch_once(text, '    for (int i = 0; i < GM_N * GM_N; i++) s_bd[i] = 0xFFFF;',
        '    SP_BEGIN(7);\n    for (int i = 0; i < GM_N * GM_N; i++) s_bd[i] = 0xFFFF;')
    text = patch_once(text, '    while (qh < qt) {', '    SP_END(7);\n    SP_BEGIN(8);\n    while (qh < qt) {')
    text = patch_once(text, '    if (best < 0) return false;', '    SP_END(8);\n    if (best < 0) return false;')
    return text


def instrument_traj(text: str) -> str:
    text = '#include "segment_probe.h"\n' + text
    text = patch_once(text,
        '                   uint8_t cmax, hal_traj_res_t *res)\n{\n    for (int i = 0; i < k; i++) {',
        '                   uint8_t cmax, hal_traj_res_t *res)\n{\n    SP_BEGIN(9);\n    for (int i = 0; i < k; i++) {')
    text = patch_once(text, '    nav_wr(NAV_TRAJ_CMD, 1);\n    uint32_t st;',
        '    nav_wr(NAV_TRAJ_CMD, 1);\n    SP_END(9);\n    SP_BEGIN(10);\n    uint32_t st;')
    text = patch_once(text, '    do st = nav_rd(NAV_TRAJ_STAT); while (!(st & NAV_TRAJ_STAT_DONE_MASK));',
        '    do st = nav_rd(NAV_TRAJ_STAT); while (!(st & NAV_TRAJ_STAT_DONE_MASK));\n    SP_END(10);\n    SP_BEGIN(11);')
    text = patch_once(text, '    return !(st & NAV_TRAJ_STAT_STALE_MASK);',
        '    SP_END(11);\n    return !(st & NAV_TRAJ_STAT_STALE_MASK);')
    return text


HEADER = r'''#ifndef SEGMENT_PROBE_H
#define SEGMENT_PROBE_H
#include <stdint.h>
void sp_record(unsigned phase, unsigned cycles);
static inline unsigned sp_cycles(void) {
 unsigned value; __asm__ volatile("rdcycle %0":"=r"(value)::"memory"); return value;
}
#define SP_BEGIN(id) unsigned sp_t_##id=sp_cycles()
#define SP_END(id) sp_record(id,sp_cycles()-sp_t_##id)
#endif
'''


COMMON_C = r'''#include "segment_probe.h"
#include "soc.h"
#include <string.h>
typedef struct {unsigned phase,cycles;} sp_row_t;
static volatile sp_row_t sp_rows[24];
static volatile unsigned sp_count,sp_overflow;
void sp_record(unsigned phase,unsigned cycles) {
 unsigned at=sp_count;
 if(at<24) {sp_rows[at].phase=phase;sp_rows[at].cycles=cycles;sp_count=at+1;}
 else sp_overflow=1;
}
static void sp_reset(void) {sp_count=sp_overflow=0;}
static void sp_dump(unsigned kind,unsigned mode,unsigned chunk,unsigned whole) {
 con_printf("CALL kind=%u mode=%u chunk=%u cycles=%u\n",kind,mode,chunk,whole);
 for(unsigned i=0;i<sp_count;i++)
  con_printf("SEG kind=%u mode=%u chunk=%u phase=%u cycles=%u\n",kind,mode,chunk,sp_rows[i].phase,sp_rows[i].cycles);
 if(sp_overflow) {con_printf("TRACE_OVERFLOW\n");SOC_STATUS=0xbad00001;for(;;) {}}
}
'''


ASTAR_C = r'''#include "astar.h"
static uint8_t map[64*64];static astar_path_t path;
int main(void) {
 sp_reset();SP_BEGIN(0);SP_END(0);sp_dump(99,99,0,0);
 for(unsigned mode=0;mode<2;mode++) for(unsigned kind=0;kind<5;kind++) {
  astar_cfg_t cfg;astar_default_cfg(&cfg);
  if(!mode) {cfg.clear_cells=4;cfg.clear_cost=15;cfg.near_cells=3;cfg.near_cost=150;}
  memset(map,kind==1?255:0,sizeof map);
  if(kind==2) for(int y=0;y<64;y++) for(int x=0;x<64;x++)
   if(x%8==4 && y!=(x/8%2?4:59)) map[y*64+x]=254;
  if(kind==3) for(int y=0;y<64;y++) for(int x=0;x<64;x++)
   if((x+y)%3==0 && x!=2 && x!=60) map[y*64+x]=254;
  if(kind==4) {memset(map,254,sizeof map);map[32*64+2]=map[32*64+60]=0;}
  path.n=0;astar_complete_only(true);sp_reset();unsigned t=sp_cycles();
  int rc=astar_grid_bounded(&cfg,map,64,2,32,60,32,&path,128);
  unsigned elapsed=sp_cycles()-t;sp_dump(kind,mode,0,elapsed);
  unsigned chunk=0;
  while(astar_busy() && chunk<256) {
   sp_reset();t=sp_cycles();rc=astar_continue(&path,128);elapsed=sp_cycles()-t;
   sp_dump(kind,mode,++chunk,elapsed);
  }
  con_printf("FINAL kind=%u mode=%u result=%d points=%u expanded=%u dropped=%u busy=%u\n",
   kind,mode,rc,path.n,astar_last_expanded(),astar_heap_dropped(),astar_busy());
  if(astar_busy()) {SOC_STATUS=0xbad00002;for(;;) {}}
 }
 SOC_STATUS=0x600dc0de;return 0;
}
'''


FRONTIER_C = r'''#include "explore.h"
static gmap_t gm;static explore_t ex;
int main(void) {
 for(unsigned mode=0;mode<2;mode++) for(unsigned kind=0;kind<3;kind++) {
  memset(&gm,0,sizeof gm);memset(&ex,0,sizeof ex);
  gm.init=gm.ready=true;
  if(kind==1) {
   memset(gm.grid,255,sizeof gm.grid);
   for(int y=16;y<48;y++) for(int x=16;x<48;x++) gm.grid[y*64+x]=0;
  }
  if(kind==2) for(int y=0;y<64;y++) for(int x=0;x<64;x++)
   if(x%8==4 && y!=(x/8%2?4:59)) gm.grid[y*64+x]=254;
  if(mode) for(int y=0;y<64;y++) for(int x=0;x<32;x++) ex.seen_bits[(y*64+x)>>3]|=1u<<(x&7);
  explore_cfg_t cfg;explore_default_cfg(&cfg);
  hal_map_info_t mi={.seq=1,.inv_res=1311,.res_mm=50};nav_pose_t pose={.x=3000,.y=3000};
  int32_t gx=0,gy=0;sp_reset();unsigned t=sp_cycles();
  int rc=explore_frontier(&ex,&cfg,&gm,&mi,&pose,mode,&gx,&gy);
  unsigned elapsed=sp_cycles()-t;sp_dump(kind,mode,0,elapsed);
  con_printf("FRONTIER kind=%u mode=%u result=%d gx=%d gy=%d n=%u\n",kind,mode,rc,gx,gy,ex.n_frontier);
 }
 SOC_STATUS=0x600dc0de;return 0;
}
'''


TRAJ_C = r'''#include "hal.h"
static hal_cand_t cand[64];static hal_traj_res_t result[64];
int main(void) {
 for(unsigned kind=0;kind<2;kind++) {
  unsigned k=kind?64:1;sp_reset();unsigned t=sp_cycles();
  int rc=hal_traj_eval(cand,k,20,0,0,0,253,result);
  unsigned elapsed=sp_cycles()-t;sp_dump(kind,0,0,elapsed);
  con_printf("TRAJ kind=%u k=%u t=20 result=%d\n",kind,k,rc);
 }
 SOC_STATUS=0x600dc0de;return 0;
}
'''


TB = r'''`timescale 1ns/1ps
module tb_cpu250_segments;
 reg clk=0,rst=1;always #10 clk=~clk;
 wire req,we,trap;wire [15:0] addr;wire [31:0] wd;reg [31:0] rd;
 integer ticks=0,wait_count=0;reg [31:0] traj_status=0;
 localparam integer SERVICE_CYCLES=__SERVICE__;
 localparam integer WAIT_MUST_STICK=__STUCK__;
 nav_soc #(.RAM_KB(128),.INIT_B0("ram_b0.hex"),.INIT_B1("ram_b1.hex"),
 .INIT_B2("ram_b2.hex"),.INIT_B3("ram_b3.hex")) dut(
 .clk(clk),.rst(rst),.m_req(req),.m_we(we),.m_addr(addr),.m_wdata(wd),
 .m_ack(req),.m_rdata(rd),.d_req(1'b0),.d_we(1'b0),.d_addr(16'd0),.d_wdata(32'd0),
 .d_ack(),.d_rdata(),.led(),.trap(trap));
 always @* begin
  rd=0;
  if(addr==16'h4004) rd=traj_status;
 end
 always @(posedge clk) begin
  ticks<=ticks+1;
  if(rst) begin wait_count<=0;traj_status<=0;end
  else if(req && we && addr==16'h4000 && wd==1) begin
   wait_count<=SERVICE_CYCLES;traj_status<=1;
  end else if(traj_status==1 && !WAIT_MUST_STICK) begin
   if(wait_count==0) traj_status<=2;
   else wait_count<=wait_count-1;
  end
  if(dut.con_we) $write("%c",dut.mem_wdata[7:0]);
 end
 initial begin
  repeat(10) @(posedge clk);rst=0;
  fork
   begin wait(dut.status==32'h600dc0de || trap || dut.status[31:16]==16'hbad0);end
   begin repeat(__WATCHDOG__) @(posedge clk);end
  join_any
  if(WAIT_MUST_STICK && !trap && dut.status!=32'h600dc0de && traj_status==1) begin
   $display("EXPECTED_BLOCKED_WAIT_MODEL ticks=%0d; this is a fixture result, not a hardware service bound",ticks);
  end else if(dut.status==32'h600dc0de && !trap && !WAIT_MUST_STICK) $display("SEGMENT_PROBE_FINISHED");
  else $fatal(1,"Segment fixture incomplete status=%h trap=%0d",dut.status,trap);
  $finish;
 end
endmodule
'''


RUNNER = r'''"""Run the prepared CPU-only Vivado fixture after a fresh memory check."""
import argparse,ctypes,json,subprocess,time,os,psutil
from pathlib import Path

class Mem(ctypes.Structure):
 _fields_=[('length',ctypes.c_ulong),('load',ctypes.c_ulong),('total_phys',ctypes.c_ulonglong),
 ('avail_phys',ctypes.c_ulonglong),('total_page',ctypes.c_ulonglong),('avail_page',ctypes.c_ulonglong),
 ('total_virtual',ctypes.c_ulonglong),('avail_virtual',ctypes.c_ulonglong),('avail_extended',ctypes.c_ulonglong)]

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--suite',required=True,choices=['astar','frontier','traj','traj_stuck']);a=ap.parse_args()
 root=Path(__file__).resolve().parent;p=root/a.suite
 if not (p/'bench.elf').is_file():raise RuntimeError('Prepare with --compile first')
 if (p/'run_started.json').exists():raise RuntimeError('Existing run; inspect its result rather than rerunning')
 mem=Mem();mem.length=ctypes.sizeof(mem)
 if not ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(mem)):raise OSError('Memory query failed')
 if mem.avail_phys<2*1024**3 or mem.avail_page<4*1024**3:raise RuntimeError('Memory admission below2/4GiB; no simulator started')
 job={'phase':'admitted','physical_available':mem.avail_phys,'commit_available':mem.avail_page,'simulator':'Vivado CPU-only; no ModelSim',
  'controller':{'pid':os.getpid(),'created_epoch':psutil.Process().create_time(),'command':psutil.Process().cmdline(),'cwd':str(p)},'processes':[]}
 (p/'run_started.json').write_text(json.dumps(job,indent=2))
 def save():
  job['updated_epoch']=time.time();(p/'job.json').write_text(json.dumps(job,indent=2))
 save()
 viv=Path('E:/2026.1/Vivado/bin');rtl=[str(root/'rtl'/n) for n in ['nav_ram_sp.v','nav_ram_sdp.v','picorv32.v','nav_soc.v']]
 for name,args,label in [('xvlog',rtl,'rtl'),('xvlog',['-sv','tb_cpu250_segments.sv'],'tb'),('xelab',['tb_cpu250_segments','-s','snapshot'],'elab'),('xsim',['snapshot','-R'],'sim')]:
  with (p/f'{label}.log').open('w') as f:
   command=['cmd','/c',str(viv/f'{name}.bat'),*args]
   proc=subprocess.Popen(command,cwd=p,stdout=f,stderr=subprocess.STDOUT)
   job['phase']='running_'+name;job['current_pid']=proc.pid;job['current_command']=command
   seen=set();last_saved=0
   while proc.poll() is None:
    try: processes=[psutil.Process(proc.pid),*psutil.Process(proc.pid).children(recursive=True)]
    except psutil.Error: processes=[]
    for process in processes:
     try:
      identity=(process.pid,process.create_time())
      if identity not in seen:
       seen.add(identity);job['processes'].append({'phase':name,'pid':process.pid,'created_epoch':identity[1],
         'parent_pid':process.ppid(),'command':process.cmdline(),'cwd':process.cwd()})
     except psutil.Error:pass
    if time.time()-last_saved>=30:save();last_saved=time.time()
    time.sleep(1)
   rc=proc.returncode;job['last_returncode']=rc;save()
  if rc:job['phase']='failed_'+name;save();raise RuntimeError(str(p/f'{label}.log'))
 text=(p/'sim.log').read_text(errors='replace')
 if not ('SEGMENT_PROBE_FINISHED' in text or (a.suite=='traj_stuck' and 'EXPECTED_BLOCKED_WAIT_MODEL' in text)):
  raise RuntimeError('Simulation lacks actual expected completion marker')
 subprocess.run([__import__('sys').executable,str(root/'parse_segments.py'),'--suite',a.suite],check=True)
 job['phase']='complete';job['result']=str(p/'result.json');save()

if __name__=='__main__':main()
'''


PARSER = r'''import argparse,json,re
from pathlib import Path
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--suite',required=True);a=ap.parse_args();root=Path(__file__).resolve().parent;p=root/a.suite
 meta=json.loads((root/'preparation.json').read_text());text=(p/'sim.log').read_text(errors='replace')
 rows=[dict(zip(['kind','mode','chunk','phase','cycles'],map(int,row))) for row in re.findall(r'SEG kind=(\d+) mode=(\d+) chunk=(\d+) phase=(\d+) cycles=(\d+)',text)]
 calls=[dict(zip(['kind','mode','chunk','cycles'],map(int,row))) for row in re.findall(r'CALL kind=(\d+) mode=(\d+) chunk=(\d+) cycles=(\d+)',text)]
 for row in rows+calls:row['time_us_at50MHz']=row['cycles']/50
 maxima={}
 for row in rows:
  name=meta['phase_names'][str(row['phase'])]
  if name not in maxima or row['cycles']>maxima[name]['cycles']:maxima[name]=row
 report={'scope':'Finite CPU/RAM RTL segment scenarios with diagnostic instrumentation; not whole-loop WCET or physical stop proof',
 'suite':a.suite,'source':meta['source'],'actual_rows':rows,'calls':calls,'phase_maxima':maxima,
 'clock_hz':50000000,'hardware_time_accelerated':False,'deployment_firmware':False,
 'service_fixture':meta['suites'][a.suite],'instrumentation_included':True,'latency_350ms_proven':False,
 'physical_stop_500ms_proven':False,'expected_stuck_model_observed':'EXPECTED_BLOCKED_WAIT_MODEL' in text}
 (p/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report['phase_maxima'],indent=2))
if __name__=='__main__':main()
'''


def compile_suite(fw: Path, path: Path, suite: str) -> dict:
    kernels = {'astar': ['nav/astar.c', 'nav/scratch.c'],
        'frontier': ['nav/explore.c', 'nav/scratch.c', 'nav/astar.c', 'nav/gmap.c', 'nav/fixmath.c'],
        'traj': ['drv/hal_fpga.c'], 'traj_stuck': ['drv/hal_fpga.c']}[suite]
    elf = path/'bench.elf'
    cmd = [str(GCC_DIR/'riscv-none-elf-gcc.exe'), '-march=rv32imc_zicsr', '-mabi=ilp32', '-Os',
        '-flto', '-std=c99', '-ffunction-sections', '-fdata-sections', '-fno-common', '-nostartfiles',
        '-specs=nano.specs', '-DNAV_BASE=0x40000000u', '-DGM_N=64', '-DASTAR_MAXN=64', '-DGM_DS=2',
        '-Wl,--gc-sections', '-Wl,--defsym=RAM_SIZE=131072', '-Wl,--defsym=STACK_RESERVE=6144',
        '-T', str(fw/'bsp/link.ld'), '-I'+str(fw)]
    cmd += ['-I'+str(fw/d) for d in ('include', 'hal', 'nav', 'app', 'bsp')]
    cmd += [str(path/'bench.c'), *[str(fw/n) for n in kernels], str(fw/'bsp/start.S'),
        str(fw/'bsp/console.c'), '-o', str(elf), '-lgcc']
    result = subprocess.run(cmd, capture_output=True, text=True)
    (path/'compile.log').write_text(result.stdout+result.stderr)
    if result.returncode:
        raise RuntimeError(str(path/'compile.log'))
    subprocess.run([str(GCC_DIR/'riscv-none-elf-objcopy.exe'), '-O', 'binary', str(elf), str(path/'bench.bin')], check=True)
    image = (path/'bench.bin').read_bytes()
    if len(image)>131072:
        raise RuntimeError('Diagnostic image exceeds original CPU RAM')
    image = image.ljust(131072, b'\0')
    for bank in range(4):
        (path/f'ram_b{bank}.hex').write_text('\n'.join(f'{value:02x}' for value in image[bank::4])+'\n')
    size = subprocess.run([str(GCC_DIR/'riscv-none-elf-size.exe'), str(elf)], capture_output=True, text=True, check=True).stdout
    (path/'size.txt').write_text(size)
    return {'compiled':True,'binary_bytes':(path/'bench.bin').stat().st_size,'command':cmd,'size':size,
        'simulator_started':False,'measurement_available':False}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source', type=Path, default=DEFAULT_SOURCE)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--compile', action='store_true')
    args = ap.parse_args();source=args.source.resolve();out=args.out.resolve()
    if not (source/'fw/nav/host_link_guard.h').is_file():
        raise ValueError('Expected actual frozen safety firmware')
    guard = (source/'fw/nav/host_link_guard.h').read_text(encoding='utf-8-sig')
    if '#define HOST_LINK_TIMEOUT_US 250000u' not in guard:
        raise ValueError('Expected250ms safety candidate; refusing another source')
    out.mkdir(parents=True, exist_ok=False)
    fw=out/'fw';shutil.copytree(source/'fw', fw)
    files=['nav/astar.c','nav/explore.c','drv/hal_fpga.c']
    original={name:(fw/name).read_text(encoding='utf-8-sig') for name in files}
    transformations=[instrument_astar,instrument_frontier,instrument_traj]
    patches=[]
    import difflib
    for name,fn in zip(files,transformations):
        changed=fn(original[name]);(fw/name).write_text(changed,encoding='utf-8')
        patches.append(''.join(difflib.unified_diff(original[name].splitlines(True),changed.splitlines(True),
            fromfile='frozen/'+name,tofile='diagnostic/'+name)))
    (out/'instrumentation.patch').write_text('\n'.join(patches),encoding='utf-8')
    (fw/'segment_probe.h').write_text(HEADER)
    (out/'run_xsim.py').write_text(RUNNER)
    (out/'parse_segments.py').write_text(PARSER)
    (out/'rtl').mkdir()
    for relative in ['common/nav_ram_sp.v','common/nav_ram_sdp.v','soc/third_party/picorv32.v','soc/nav_soc.v']:
        shutil.copy2(source/'nav/rtl'/relative,out/'rtl'/Path(relative).name)
    phase_names={0:'empty_rdcycle_bracket',1:'astar_cost_snapshot',2:'astar_distance_transform',
        3:'astar_coordinate_and_g_parent_heap_initialization',4:'astar_up_to128_heap_pops',
        5:'astar_backtrace_and_atomic_path_publication',7:'frontier_distance_queue_initialization',
        8:'frontier_entire_BFS_and_scoring',9:'trajectory_candidate_and_command_MMIO',
        10:'trajectory_DONE_poll_wait',11:'trajectory_result_MMIO'}
    suites={
        'astar':{'bench':ASTAR_C,'service_cycles':0,'watchdog_cycles':200000000,
            'inputs':'64x64 empty, unknown, alternating-gap walls, checker obstacles, disconnected endpoints; actual coarse/fine configs; complete_only and128 heap pops; no scheduler gaps'},
        'frontier':{'bench':FRONTIER_C,'service_cycles':0,'watchdog_cycles':100000000,
            'inputs':'64x64 all-known,32x32 known island in unknown, alternating-gap walls; seen_mode false/true; actual default config and full queue'},
        'traj':{'bench':TRAJ_C,'service_cycles':6000000,'watchdog_cycles':18000000,
            'inputs':'actual HAL k1 and64,t20; synthetic immediate-ACK MMIO with120ms DONE delay, not actual trajectory RTL service proof'},
        'traj_stuck':{'bench':TRAJ_C,'service_cycles':0,'watchdog_cycles':6500000,
            'inputs':'actual HAL no DONE return;130ms finite observation, expected blocked-wait fixture, not a legal hardware worst-service assertion'},
    }
    record={'recorded_at':datetime.now().astimezone().isoformat(),'source':str(source),'out':str(out),
        'scope':'Prepared diagnostic copies only; no deployment FW or ongoing simulator inputs modified',
        'instrumented_files_only':files,'source_files_still_text_identical':{},
        'phase_names':phase_names,'suites':{},'modelsim_started':False,'xsim_started':False,
        'actual_segment_cycles_available':False,'whole_loop_WCET_proven':False,
        'observer_effect':'rdcycle and trace stores change compiler layout/register pressure; raw measurements include instrumentation; no in-timed-section console output or hook yielding',
        'budget_definition':{'host_threshold_us':250000,'target_zero_age_us':350000,
            'max_nonpreemptible_and_submit_allowance_us':100000,'physical500ms_stop_proven':False}}
    for suite,info in suites.items():
        path=out/suite;path.mkdir()
        (path/'bench.c').write_text(COMMON_C+info['bench'])
        (path/'tb_cpu250_segments.sv').write_text(TB.replace('__SERVICE__',str(info['service_cycles']))
            .replace('__STUCK__','1' if suite=='traj_stuck' else '0').replace('__WATCHDOG__',str(info['watchdog_cycles'])))
        record['suites'][suite]={k:v for k,v in info.items() if k!='bench'}
        if args.compile:
            record['suites'][suite]['build']=compile_suite(fw,path,suite)
    for name in files:
        # Compare original snapshot text with the live frozen source without hashes.
        record['source_files_still_text_identical'][name]=(source/'fw'/name).read_text(encoding='utf-8-sig')==original[name]
    if not all(record['source_files_still_text_identical'].values()):
        raise RuntimeError('Frozen source changed during diagnostic preparation')
    (out/'preparation.json').write_text(json.dumps(record,indent=2),encoding='utf-8')
    print(json.dumps({'prepared':str(out),'compiled':args.compile,'simulators_started':False,'segment_measurements_available':False}))


if __name__=='__main__':main()
