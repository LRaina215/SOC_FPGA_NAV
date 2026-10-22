"""Measure frozen C guard cycles on the actual PicoRV32/nav_soc RTL.

This is a measured scenario envelope, not a WCET proof or whole-loop deadline.
Uses the board's 50MHz, RAM bus, RV32IMC fast multiply/divide and IRQ-off setup.
"""
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[1]


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--source',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--kernel',choices=['motion','astar','gmap','gmap_rows','support'],default='motion')
    ap.add_argument('--expand-limit',type=int,default=0)
    a=ap.parse_args();a.source=a.source.resolve();a.out=a.out.resolve()
    a.out.mkdir(parents=True,exist_ok=False)
    src=a.out/'bench.c'
    src.write_text('''#include "motion_guard.h"
#include "soc.h"
#include <string.h>
static uint8_t map[NP_MAP_N*NP_MAP_N];
static unsigned cycles(void) {unsigned c;__asm__ volatile("rdcycle %0":"=r"(c));return c;}
int main(void) {
 hal_map_info_t mi={.seq=1,.cx=0,.cy=0,.res_mm=50,.inv_res=1311};
 nav_pose_t p={0};
 lp_cfg_t cfg={0};cfg.caps.radius_mm=210;cfg.caps.vx_max=500;
 cfg.caps.wz_max=1200;cfg.dt_ms=50;cfg.safe_steps=64;
 for(int kind=0;kind<3;kind++) {
  memset(map,kind==1?255:0,sizeof map);
  if(kind==2) for(int i=0;i<128;i++) {map[58*128+i]=254;map[69*128+i]=254;}
  for(int mode=0;mode<4;mode++) {
   nav_cmd_t cmd={mode==0?2:mode==1?500:0,0,mode==2?1200:0,0};
   p.yaw=mode==3?0:16384;
   unsigned t=cycles();
   unsigned result=mode==3?motion_segment(map,&mi,&p,&cfg,800,0):motion_check(map,&mi,&p,&cfg,&cmd);
   unsigned elapsed=cycles()-t;
   con_printf("GUARD kind=%u mode=%u result=%u cycles=%u\\n",kind,mode,result,elapsed);
  }
 }
 SOC_STATUS=0x600dc0de;return 0;
}
''')
    if a.kernel=='astar':
        src.write_text('''#include "astar.h"
#include "soc.h"
#include <string.h>
static uint8_t map[64*64];static astar_path_t path;
static unsigned cycles(void) {unsigned c;__asm__ volatile("rdcycle %0":"=r"(c));return c;}
int main(void) {
 astar_cfg_t cfg;astar_default_cfg(&cfg);cfg.clear_cells=4;cfg.near_cells=3;cfg.near_cost=150;
 for(int kind=0;kind<3;kind++) {
  memset(map,kind==1?255:0,sizeof map);
  if(kind==2) for(int y=0;y<64;y++) for(int x=0;x<64;x++)
    if(x%8==4 && y!=(x/8%2?4:59)) map[y*64+x]=254;
  unsigned t=cycles();int result=astar_grid(&cfg,map,64,2,32,60,32,&path);unsigned elapsed=cycles()-t;
  con_printf("PLAN kind=%u result=%d points=%u cycles=%u\\n",kind,result,path.n,elapsed);
 }
 SOC_STATUS=0x600dc0de;return 0;
}
''')
    if a.kernel in ('gmap','gmap_rows'):
        src.write_text('''#include "gmap.h"
#include "soc.h"
#include <string.h>
static uint8_t map[NP_MAP_N*NP_MAP_N];static gmap_t gm;
static unsigned cycles(void) {unsigned c;__asm__ volatile("rdcycle %0":"=r"(c));return c;}
int main(void) {
 gmap_default_cfg(&gm.cfg);gmap_init(&gm);
 hal_map_info_t mi={.seq=1,.inv_res=1311,.res_mm=50};
 for(int kind=0;kind<3;kind++) {
  memset(map,kind==1?255:0,sizeof map);
  if(kind==2) for(int i=0;i<128*128;i++) if(i%9==0) map[i]=254;
  mi.cx=kind==2?60:0;mi.cy=0;mi.seq++;
  unsigned t=cycles();gmap_update(&gm,map,&mi,kind==2?3000:0,0);unsigned elapsed=cycles()-t;
  con_printf("GRID kind=%u updates=%u cycles=%u\\n",kind,gm.n_upd,elapsed);
 }
 SOC_STATUS=0x600dc0de;return 0;
}
''')
    if a.kernel=='gmap_rows':
        code=src.read_text()
        code=code.replace('gmap_update(&gm,map,&mi,kind==2?3000:0,0)',
                          'gmap_update_rows(&gm,map,&mi,kind==2?3000:0,0,4)')
        at='  con_printf("GRID kind=%u updates=%u cycles=%u\\n",kind,gm.n_upd,elapsed);'
        assert at in code
        code=code.replace(at,at+'''
  unsigned chunk=0;
  while(!gm.work_done) {
   t=cycles();gmap_update_rows(&gm,map,&mi,kind==2?3000:0,0,4);elapsed=cycles()-t;
   con_printf("CHUNK kind=%u chunk=%u cycles=%u\\n",kind,++chunk,elapsed);
  }
''')
        src.write_text(code)
    if a.kernel=='astar' and a.expand_limit>0:
        code=src.read_text()
        code=code.replace('astar_grid(&cfg,map,64,2,32,60,32,&path)',
                          f'astar_grid_bounded(&cfg,map,64,2,32,60,32,&path,{a.expand_limit})')
        at='  con_printf("PLAN kind=%u result=%d points=%u cycles=%u\\n",kind,result,path.n,elapsed);'
        assert at in code
        code=code.replace(at,at+f'''
  unsigned chunk=0;
  while(astar_busy()) {{
   t=cycles();result=astar_continue(&path,{a.expand_limit});elapsed=cycles()-t;
   con_printf("CHUNK kind=%u chunk=%u cycles=%u\\n",kind,++chunk,elapsed);
  }}
  con_printf("FINAL kind=%u result=%d dropped=%u\\n",kind,result,astar_heap_dropped());
''')
        src.write_text(code)
    if a.kernel=='support':
        src.write_text('''#include "support_guard.h"
#include "soc.h"
static unsigned cycles(void) {unsigned c;__asm__ volatile("rdcycle %0":"=r"(c));return c;}
int main(void) {
 hal_map_info_t mi={.seq=1,.inv_res=1311,.res_mm=50};nav_pose_t p={0};
 lp_cfg_t cfg={.safe_steps=12,.dt_ms=100};
 for(int kind=0;kind<3;kind++) {
  nav_cmd_t cmd={kind==2?0:kind==1?375:500,0,kind==2?600:kind==1?-600:0,0};
  support_reset(0);unsigned t=cycles();int result=support_check(&p,&cfg,&mi,&cmd);unsigned elapsed=cycles()-t;
  con_printf("SUPPORT kind=%u result=%u cycles=%u\\n",kind,result,elapsed);
 }
 SOC_STATUS=0x600dc0de;return 0;
}
''')
    gccdir=Path(os.environ.get('RISCV_GCC',r'D:\tools\xpack-riscv-none-elf-gcc-15.2.0-1\bin'))
    gcc=str(gccdir/'riscv-none-elf-gcc.exe')
    elf=a.out/'bench.elf'
    cmd=[gcc,'-march=rv32imc_zicsr','-mabi=ilp32','-Os','-flto','-std=c99','-ffunction-sections',
         '-fdata-sections','-nostartfiles','-specs=nano.specs','-Wl,--gc-sections',
         '-Wl,--defsym=RAM_SIZE=131072','-T',str(a.source/'bsp/link.ld')]
    cmd+=['-I'+str(a.source/d) for d in ('include','hal','nav','app','bsp')]
    kernels={'motion':('nav/motion_guard.c','nav/fixmath.c'),'astar':('nav/astar.c','nav/scratch.c'),
             'gmap':('nav/gmap.c',),'gmap_rows':('nav/gmap.c',),
             'support':('nav/support_guard.c','nav/fixmath.c','drv/hal_fpga.c')}[a.kernel]
    cmd += [str(src),*[str(a.source/n) for n in (*kernels,'bsp/start.S','bsp/console.c')],'-o',str(elf),'-lgcc']
    subprocess.run(cmd,check=True,capture_output=True)
    subprocess.run([str(gccdir/'riscv-none-elf-objcopy.exe'),'-O','binary',str(elf),str(a.out/'bench.bin')],check=True)
    img=(a.out/'bench.bin').read_bytes().ljust(131072,b'\0')
    for b in range(4):(a.out/f'ram_b{b}.hex').write_text('\n'.join(f'{x:02x}' for x in img[b::4])+'\n')
    tb=a.out/'tb_motion_cycles.sv'
    tb.write_text('''`timescale 1ns/1ps
module tb_motion_cycles;
 reg clk=0,rst=1;always #10 clk=~clk;
 wire req,we,trap;wire [15:0] addr;wire [31:0] wd;
 nav_soc #(.RAM_KB(128),.INIT_B0("ram_b0.hex"),.INIT_B1("ram_b1.hex"),
  .INIT_B2("ram_b2.hex"),.INIT_B3("ram_b3.hex")) dut(
  .clk(clk),.rst(rst),.m_req(req),.m_we(we),.m_addr(addr),.m_wdata(wd),
  .m_ack(req),.m_rdata(32'd0),.d_req(1'b0),.d_we(1'b0),.d_addr(16'd0),.d_wdata(32'd0),
  .d_ack(),.d_rdata(),.led(),.trap(trap));
 always @(posedge clk) if(dut.con_we) $write("%c",dut.mem_wdata[7:0]);
 initial begin
  repeat(10) @(posedge clk);rst=0;
  fork
   begin wait(dut.status==32'h600dc0de || trap);end
   begin repeat(100000000) @(posedge clk);end
  join_any
  if(dut.status==32'h600dc0de && !trap) $display("TEST PASSED");
  else $fatal(1,"Cycle benchmark did not finish status=%h trap=%0d",dut.status,trap);
  $finish;
 end
endmodule
''')
    if a.kernel=='support':
        code=tb.read_text().replace('.m_ack(req),.m_rdata(32\'d0)', '.m_ack(req),.m_rdata(rd)')
        at=' wire req,we,trap;wire [15:0] addr;wire [31:0] wd;'
        assert at in code
        code=code.replace(at,at+'''
 reg [31:0] rd;reg [13:0] slot=0;
 wire [7:0] tag={slot[13]?4'hf:4'h0,slot[6]?4'hf:4'h0};
 always @(posedge clk) if(req && we && addr==16'h3048) slot<=wd[13:0];
 always @* begin
  rd=0;
  case(addr)
   16'h3028:rd=32'h00010000;
   16'h3008:rd=1311;
   16'h303c:rd=1;
   16'h3054:rd=32'h80001001 | {20'd0,tag,4'd0};
  endcase
 end
''')
        tb.write_text(code)
    rtl=[]
    for n in ('rtl/common/nav_ram_sp.v','rtl/common/nav_ram_sdp.v','rtl/soc/third_party/picorv32.v','rtl/soc/nav_soc.v'):
        dst=a.out/Path(n).name;shutil.copy2(ROOT/n,dst);rtl.append(str(dst))
    viv=Path(os.environ.get('VIVADO_BIN',r'E:\2026.1\Vivado\bin'))
    for name,args in [('xvlog',rtl),('xvlog',['-sv',str(tb)]),('xelab',['tb_motion_cycles','-s','snap']),('xsim',['snap','-R'])]:
        log=a.out/f'{name}_{"sv" if "-sv" in args else "out"}.log'
        with log.open('w') as f:
            result=subprocess.run(['cmd','/c',str(viv/f'{name}.bat'),*args],cwd=a.out,stdout=f,stderr=subprocess.STDOUT)
        if result.returncode:raise RuntimeError(str(log))
    out=(a.out/'xsim_out.log').read_text(errors='replace')
    assert 'TEST PASSED' in out,out[-1500:]
    if a.kernel=='motion':
        rows=[dict(zip(('kind','mode','result','cycles'),map(int,v))) for v in re.findall(r'GUARD kind=(\d+) mode=(\d+) result=(\d+) cycles=(\d+)',out)]
        assert len(rows)==12,len(rows)
    elif a.kernel=='support':
        rows=[dict(zip(('kind','result','cycles'),map(int,v))) for v in re.findall(r'SUPPORT kind=(\d+) result=(\d+) cycles=(\d+)',out)]
        assert len(rows)==3 and all(r['result']==0 for r in rows),rows
    elif a.kernel=='astar':
        rows=[dict(zip(('kind','result','points','cycles'),map(int,v))) for v in re.findall(r'PLAN kind=(\d+) result=(-?\d+) points=(\d+) cycles=(\d+)',out)]
        assert len(rows)==3,len(rows)
    else:
        rows=[dict(zip(('kind','updates','cycles'),map(int,v))) for v in re.findall(r'GRID kind=(\d+) updates=(\d+) cycles=(\d+)',out)]
        assert len(rows)==3,len(rows)
    for r in rows:r['time_ms_50MHz']=r['cycles']/50000
    chunks=[dict(zip(('kind','chunk','cycles'),map(int,v))) for v in re.findall(r'CHUNK kind=(\d+) chunk=(\d+) cycles=(\d+)',out)]
    for r in chunks:r['time_ms_50MHz']=r['cycles']/50000
    report=dict(scope=f'{a.kernel} measured cases on board-equivalent CPU/RAM RTL; not whole-loop WCET',
        source=str(a.source),clock_hz=50000000,rows=rows,chunks=chunks,expand_limit=a.expand_limit,
        max_ms=max(v['time_ms_50MHz'] for v in rows+chunks),
        excluded=['A*/BFS','SLF wait','height read bus arbitration','planner calls and repeated guard checks','real board timing'])
    if a.kernel=='support':report['register_model']='ideal immediate-ack flat map; actual hal_fpga query sequence, excludes map arbitration/wait'
    (a.out/'result.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report))


if __name__=='__main__':main()
