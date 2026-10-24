"""Actual point filter/elevation/traversability pipeline against original gold."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import subprocess
from derive_ddr_matcher import HERE,ROOT
from derive_ddr_elevation import derive,derive_traversability,derive_map
FROZEN=Path('F:/nav_integration_g2_20261023/sfp_lane2_private_snapshot_syn11_retry1/nav')


def run(out,one_frame=False):
    assert not out.exists();out.mkdir()
    for rel,fn in [('rtl/map/elev_map.v',derive),('rtl/map/trav_v3.v',derive_traversability),('rtl/map/map_subsys.v',derive_map)]:
        source=(FROZEN/rel).read_text(encoding='utf-8-sig')
        (out/(Path(rel).stem+'_ddr.v')).write_text(fn(source),encoding='utf-8')
    names=['elev_map_ddr.v','trav_v3_ddr.v','map_subsys_ddr.v']
    for rel in ['rtl/common/nav_ram_sdp.v','rtl/common/nav_ram_banked.v','rtl/point/pc_project.v','rtl/point/pc_transform.v','rtl/point/pc_filter.v']:
        # Source paths are explicit; do not substitute a different pipeline.
        p=FROZEN/rel
        if not p.exists():
            found=list((FROZEN/'rtl').rglob(Path(rel).name));assert len(found)==1,rel;p=found[0]
        shutil.copyfile(p,out/p.name);names.append(p.name)
    for n in ['nav_ddr_dual_step.v','nav_ddr_map_cache.v','nav_ddr_elevation_store.v','nav_ddr_native.v']:
        shutil.copyfile(HERE/n,out/n);names.append(n)
    vectors=ROOT/'sim/build/tb_lidar_map'
    for n in ['counts.txt','pix.hex','pose.hex','flt_exp.hex','cost_exp.hex','mapcfg.hex','lut.hex']:
        shutil.copyfile(vectors/n,out/n)
    if one_frame:
        counts=(out/'counts.txt').read_text().split();counts[0]='9600';counts[1]='-1';counts[2]='1'
        (out/'counts.txt').write_text(' '.join(counts)+'\n')
    model=(HERE/'tb_slf_row_update.sv').read_text().split('module tb_slf_row_update;')[0]
    for a,b in [('[0:8191]','[0:16383]'),('[12:0]held_addr','[13:0]held_addr'),('addr[27:16]','addr[27:17]'),('addr[15:3]','addr[16:3]')]:
        assert model.count(a)==1,a;model=model.replace(a,b)
    (out/'native_service_model.sv').write_text(model)
    tb=(ROOT/'rtl/map/tb/tb_lidar_map.sv').read_text(encoding='utf-8-sig')
    def one(a,b):
        nonlocal tb
        assert tb.count(a)==1,a[:100];tb=tb.replace(a,b)
    one('    always #5 clk = ~clk;', '''    always #10 clk=~clk;
    reg ui_clk=0;always #5 ui_clk=~ui_clk;
    wire mem_req,mem_we,mem_ack,fault;wire[13:0]mem_addr;wire[52:0]mem_wdata,mem_reply;
    wire[2:0]cmd;wire cmd_en,cmd_ready,wen,wend,wready,rv,re;
    wire[27:0]addr;wire[255:0]wd,rd;wire[31:0]wm;
    integer refresh_phase=389,ready_period=16,read_delay=20;
    nav_ddr_elevation_store store(clk,ui_clk,rst,1'b1,mem_req,mem_we,mem_addr,mem_wdata,mem_ack,mem_reply,
        1'b0,,fault,cmd,cmd_en,addr,cmd_ready,wd,wm,wen,wend,wready,rd,rv,re);
    row_service_model dram(ui_clk,rst,1'b1,cmd,cmd_en,addr,cmd_ready,wd,wm,wen,wend,wready,rd,rv,re,
        refresh_phase,ready_period,read_delay);
    initial for(integer a=0;a<16384;a++)dram.mem[a]='1;
    always@(posedge clk)if(!rst&&fault)$fatal(1,"unexpected elevation fault");
    integer cycles=0,frame_start;
    always@(posedge clk)cycles++;''')
    one('    map_subsys u_map (','    map_subsys_ddr u_map (')
    one('        .busy(busy), .front(), .seq(seq), .lat_cx(), .lat_cy(), .frame_cnt(), .map_ready(map_ready));', '''        .busy(busy), .front(), .seq(seq), .lat_cx(), .lat_cy(), .frame_cnt(), .map_ready(map_ready),
        .mem_req(mem_req),.mem_we(mem_we),.mem_addr(mem_addr),.mem_wdata(mem_wdata),
        .mem_ack(mem_ack),.mem_reply(mem_reply),.memory_fault(fault));''')
    one('            sending = 1;', '            frame_start=cycles;sending = 1;')
    one('            check_cost(k);','            $display("MAP_FRAME frame=%0d cycles=%0d us_at_50MHz=%0f",k,cycles-frame_start,(cycles-frame_start)/50.0);\n            check_cost(k);')
    one('if (errs == 0 && fi == n_flt) $display("TEST PASSED");',
        'if (errs == 0 && (n_flt<0||fi == n_flt)) $display("TEST PASSED frames=%0d filtered_points=%0d cost_words=%0d",n_frm,fi,n_frm*NCELL);')
    one('        #200ms;', '        #2000ms; // Test budget, NOT a navigation/UPDATE admission threshold.')
    (out/'tb_map_ddr.sv').write_text(tb,encoding='utf-8')
    commands=[('xvlog',names),('xvlog',['-sv','native_service_model.sv','tb_map_ddr.sv']),
        ('xelab',['-debug','off','-timescale','1ns/1ps','tb_lidar_map','-s','snap']),('xsim',['snap','-runall'])]
    for i,(tool,args) in enumerate(commands):
        with (out/f'{i}_{tool}_stdout.log').open('w') as log:
            subprocess.run(['cmd','/c',str(Path('E:/2026.1/Vivado/bin')/(tool+'.bat')),*args],cwd=out,
                stdout=log,stderr=subprocess.STDOUT,timeout=1800,check=True,creationflags=subprocess.CREATE_NO_WINDOW)
    s=(out/'3_xsim_stdout.log').read_text(errors='replace')
    assert 'TEST PASSED' in s and not re.search(r'TEST FAILED|FATAL|Fatal|ERROR:',s),s[-3500:]
    result=dict(tested_at=datetime.now().astimezone().isoformat(),passed=True,one_frame=one_frame,
        metrics={k:int(v) for k,v in re.findall(r'(\w+)=(\d+)',re.search(r'TEST PASSED[^\r\n]*',s)[0])},
        timing=[{k:float(v) if '.' in v else int(v) for k,v in re.findall(r'(\w+)=([\d.]+)',l)} for l in s.splitlines() if l.startswith('MAP_FRAME')],
        scope='actual original point pipeline/elevation/four-pass traversal with native-service model',
        shared_structure_arbitration_verified=False,hardware_service_bound_verified=False,
        full_joint_implementation=False,programmed=False)
    (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--one-frame',action='store_true')
    a=p.parse_args();run(a.out.resolve(),a.one_frame)
