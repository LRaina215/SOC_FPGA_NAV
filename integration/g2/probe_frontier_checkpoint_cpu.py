"""Measure the real frozen CPU/checkpoint/guard/HAL under explicit mailbox inputs.

CPU/RAM RTL and FW functions are real; the register fabric supplies timer and
mailbox inputs and observes actual SAFE_CMD_W writes. No UART wire, full NAV
subsystem, control inference or physical standstill is modeled here.
"""
import argparse,json,subprocess,shutil,sys,os
from pathlib import Path
from prepare_cpu250_segment_probe import RUNNER

SOURCE_RTL=Path('F:/nav_integration_g2_20261023/baseline_fusion_preview_safety250_emptyIMU_prepared3/nav/rtl')
GCC=Path('D:/tools/xpack-riscv-none-elf-gcc-15.2.0-1/bin/riscv-none-elf-gcc.exe')

BENCH=r'''#include "nav_app.h"
#include "nav_search_checkpoint.h"
#include "host_link_guard.h"
#include "soc.h"
#include <string.h>
extern bool nav_app_search_checkpoint_impl(void);
static unsigned last_checkpoint,max_gap,checkpoint_count;
static unsigned cycles(void) {unsigned c;__asm__ volatile("rdcycle %0":"=r"(c)::"memory");return c;}
bool nav_app_search_checkpoint(void) {
 unsigned now=cycles(),gap=now-last_checkpoint;
 if(gap>max_gap)max_gap=gap;
 last_checkpoint=now;checkpoint_count++;
 return nav_app_search_checkpoint_impl();
}
static void diag(unsigned off,unsigned value) {*(volatile unsigned*)(0x40000000u+off)=value;}
static void fail(unsigned code) {con_printf("CHECKPOINT_FAIL code=%u\n",code);SOC_STATUS=0xbad00001;for(;;) {}}
int main(void) {
 for(unsigned kind=0;kind<3;kind++) {
  diag(0x7ff0,0);memset(&g_app,0,sizeof g_app);
  nav_task_t *nt=&g_app.nt;g_nav_search_interrupted=false;nt->mode=NAV_MODE_SEARCH;
  nt->gm.init=nt->gm.ready=nt->gm.has_prior=true;nt->pose.cur.x=nt->pose.cur.y=3000;
  for(unsigned y=0;y<64;y++)for(unsigned x=0;x<32;x++)nt->exp.seen_bits[(y*64+x)>>3]|=1u<<(x&7);
  nt->exp.n_seen=2048;
  if(kind) {nt->exp.bl_n=8;for(unsigned n=0;n<8;n++){nt->exp.bl_x[n]=12000+n*2000;nt->exp.bl_y[n]=-12000;}}
  explore_cfg_t cfg;explore_default_cfg(&cfg);hal_map_info_t mi={.seq=1,.inv_res=1311,.res_mm=50};
  unsigned now=hal_time_us();g_host_link_guard=(host_link_guard_t){.have_rx=true,.last_rx_us=kind==2?now-249000u:now};
  diag(0x7ff0,kind==2?2:1);diag(0x7ff4,g_host_link_guard.last_rx_us);
  nav_cmd_t moving={100,0,0,NAV_MODE_SEARCH};hal_send_cmd(&moving);
  int32_t x=99,y=99;max_gap=checkpoint_count=0;last_checkpoint=cycles();unsigned t=last_checkpoint;
  int rc=explore_frontier(&nt->exp,&cfg,&nt->gm,&mi,&nt->pose.cur,true,&x,&y);
  unsigned elapsed=cycles()-t,gap=cycles()-last_checkpoint;if(gap>max_gap)max_gap=gap;
  con_printf("CHECKPOINT_CPU kind=%u result=%u x=%d y=%d frontier=%u cycles=%u max_gap=%u checks=%u locked=%u messages=%u\n",
   kind,rc,x,y,nt->exp.n_frontier,elapsed,max_gap,checkpoint_count,g_host_link_guard.locked,g_app.n_msgs);
  if(kind<2 && (!rc||x!=4049||y!=3049||g_host_link_guard.locked||g_nav_search_interrupted))fail(1+kind);
  if(kind==2 && (rc||!g_host_link_guard.locked||!g_nav_search_interrupted||x!=99||y!=99))fail(3);
 }
 diag(0x7ff0,0);SOC_STATUS=0x600dc0de;return 0;
}
'''

TB=r'''`timescale 1ns/1ps
module tb_cpu250_segments;
 reg clk=0,rst=1;always #10 clk=~clk;
 wire req,we,trap;wire[15:0]addr;wire[31:0]wd;reg[31:0]rd;
 integer ticks=0,hb_ticks=0,profile=0,read_word=0;reg avail=0;
 reg[31:0]rx_ts=0,last_rx=0,cmd_v=0;reg moving_seen=0,zero_seen=0;
 wire[31:0]now_us=ticks/50;
 nav_soc #(.RAM_KB(128),.INIT_B0("ram_b0.hex"),.INIT_B1("ram_b1.hex"),.INIT_B2("ram_b2.hex"),.INIT_B3("ram_b3.hex")) dut(
 .clk(clk),.rst(rst),.m_req(req),.m_we(we),.m_addr(addr),.m_wdata(wd),.m_ack(req),.m_rdata(rd),
 .d_req(1'b0),.d_we(1'b0),.d_addr(16'd0),.d_wdata(32'd0),.d_ack(),.d_rdata(),.led(),.trap(trap));
 always @* begin
  rd=0;
  case(addr)
   16'h0008:rd=now_us;
   16'h1200:rd=avail?32'h80000018:0;
   16'h1204:rd=read_word==0?32'h2001564e:0;
   16'h120c:rd=rx_ts;
  endcase
 end
 always @(posedge clk) begin
  ticks<=ticks+1;
  if(profile==1) begin
   if(hb_ticks==2499999)begin avail<=1;rx_ts<=now_us;read_word<=0;hb_ticks<=0;end
   else hb_ticks<=hb_ticks+1;
  end
  if(req && !we && addr==16'h1204)read_word<=read_word+1;
  if(req && we)case(addr)
   16'h7ff0:begin profile<=wd;hb_ticks<=0;avail<=0;read_word<=0;moving_seen<=0;zero_seen<=0;end
   16'h7ff4:last_rx<=wd;
   16'h1208:begin avail<=0;read_word<=0;end
   16'h5018:cmd_v<=wd;
   16'h501c:begin
    if(cmd_v!=0||wd[15:0]!=0)moving_seen<=1;
    else if(profile==2&&moving_seen&&!zero_seen)begin
     zero_seen<=1;
     $display("CHECKPOINT_ZERO_AGE age=%0d now=%0d last_rx=%0d",now_us-last_rx,now_us,last_rx);
     if((now_us-last_rx)>350000)$fatal(1,"Zero commit exceeded350ms in this fixture");
    end
   end
  endcase
  if(dut.con_we)$write("%c",dut.mem_wdata[7:0]);
 end
 initial begin
  repeat(10)@(posedge clk);rst=0;
  fork
   begin wait(dut.status==32'h600dc0de||trap||dut.status[31:16]==16'hbad0);end
   begin repeat(150000000)@(posedge clk);end
  join_any
  if(dut.status==32'h600dc0de&&!trap)$display("SEGMENT_PROBE_FINISHED");
  else $fatal(1,"Checkpoint CPU fixture incomplete");
  $finish;
 end
endmodule
'''

PARSER=r'''import json,re
from pathlib import Path
root=Path(__file__).resolve().parent;p=root/'frontier';t=(p/'sim.log').read_text(errors='replace')
pat=r'CHECKPOINT_CPU kind=(\d+) result=(\d+) x=(-?\d+) y=(-?\d+) frontier=(\d+) cycles=(\d+) max_gap=(\d+) checks=(\d+) locked=(\d+) messages=(\d+)'
rows=[dict(zip(['kind','result','x','y','frontier','cycles','max_gap','checks','locked','messages'],map(int,v)))for v in re.findall(pat,t)]
zero=[dict(zip(['age_us','now_us','last_rx_us'],map(int,v)))for v in re.findall(r'CHECKPOINT_ZERO_AGE age=(\d+) now=(\d+) last_rx=(\d+)',t)]
assert len(rows)==3 and len(zero)==1,(rows,zero)
for r in rows:r.update(time_us=r['cycles']/50,max_checkpoint_gap_us=r['max_gap']/50)
r={'passed':True,'scope':'Actual PicoRV32/128KiB/50MHz plus actual candidate guard/checkpoint/HAL with synthetic timer/mailbox register service; not full G2/UART or physical stop',
 'rows':rows,'zero_commit':zero,'max_checkpoint_gap_us':max(x['max_checkpoint_gap_us']for x in rows),
 'kernel_observer_effect':True,'whole_loop_WCET_proven':False,'physical_stop500ms_proven':False,'uses_actual_250ms_threshold':True,
 'healthy_50ms_mailbox_heartbeat_preserved':all(x['locked']==0 and x['messages']>0 for x in rows[:2])}
(p/'result.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
'''


def prepare(candidate:Path,out:Path):
    proof_paths=[p for p in candidate.glob('host_proofs*/result.json') if json.loads(p.read_text()).get('passed')]
    assert proof_paths,'Require a completed focused host proof first'
    proof_path=max(proof_paths,key=lambda p:p.stat().st_mtime_ns)
    proof=json.loads(proof_path.read_text());assert proof['passed']
    review=json.loads((candidate/'firmware_review.json').read_text());assert review['full_fw_compile_passed']
    out.mkdir(exist_ok=False);shutil.copytree(candidate/'fw',out/'fw');fw=out/'fw';suite=out/'frontier';suite.mkdir()
    (suite/'bench.c').write_text(BENCH);(suite/'tb_cpu250_segments.sv').write_text(TB)
    (out/'run_xsim.py').write_text(RUNNER);(out/'parse_segments.py').write_text(PARSER)
    (out/'rtl').mkdir()
    for n in ['common/nav_ram_sp.v','common/nav_ram_sdp.v','soc/third_party/picorv32.v','soc/nav_soc.v']:
        shutil.copy2(SOURCE_RTL/n,out/'rtl'/Path(n).name)
    flags=['-flto','-march=rv32imc_zicsr','-mabi=ilp32','-Os','-std=c99','-ffunction-sections','-fdata-sections',
        '-fno-common','-nostartfiles','-specs=nano.specs','-DNAV_BASE=0x40000000u','-DGM_N=64','-DASTAR_MAXN=64','-DGM_DS=2']
    flags+=['-I'+str(fw/n) for n in ['include','hal','nav','app','bsp']]
    app_object=suite/'actual_app.o'
    commands=[[str(GCC),*flags,'-Dnav_app_search_checkpoint=nav_app_search_checkpoint_impl','-c',str(fw/'app/nav_app.c'),'-o',str(app_object)]]
    commands.append([str(GCC),*flags,'-Wl,--gc-sections','-Wl,--defsym=RAM_SIZE=131072','-Wl,--defsym=STACK_RESERVE=6144',
        '-T',str(fw/'bsp/link.ld'),str(suite/'bench.c'),str(app_object),*[str(p)for p in sorted((fw/'nav').glob('*.c'))],
        str(fw/'drv/hal_fpga.c'),str(fw/'bsp/console.c'),str(fw/'bsp/start.S'),'-o',str(suite/'bench.elf'),'-lgcc'])
    for i,cmd in enumerate(commands):
        r=subprocess.run(cmd,capture_output=True,text=True);(suite/f'compile{i}.log').write_text(r.stdout+'\n'+r.stderr)
        assert r.returncode==0,r.stderr
    subprocess.run([str(GCC.with_name('riscv-none-elf-objcopy.exe')),'-O','binary',str(suite/'bench.elf'),str(suite/'bench.bin')],check=True)
    image=(suite/'bench.bin').read_bytes();assert len(image)<131072;image=image.ljust(131072,b'\0')
    for n in range(4):(suite/f'ram_b{n}.hex').write_text('\n'.join(f'{v:02x}'for v in image[n::4])+'\n')
    record={'source_candidate':str(candidate),'out':str(out),'host_proof':str(proof_path),'actual_candidate_app_checkpoint_and_guard_functions':True,
        'checkpoint_instrumentation':'Function-symbol wrapper only records rdcycle intervals,then invokes unmodified candidate implementation',
        'commands':commands,'CPU_RTL4_bytes_bound_to_source':str(SOURCE_RTL),'whole_loop_WCET_proven':False,
        'model':'timer and50ms TARGET(no-op) mailbox; last loss is injected249ms old; SAFE_CMD_W observed on actual CPU master bus',
        'simulators_started_by_preparation':False,'image_bytes':(suite/'bench.bin').stat().st_size}
    (out/'preparation.json').write_text(json.dumps(record,indent=2));print(json.dumps(record,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--candidate',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();prepare(a.candidate.resolve(),a.out.resolve())
