"""All accepted worst-shape UPDATE cases with dirty caches and EM takeover.

The slower native-service counterexample is deliberately retained. A model
pass is not evidence that the protected physical controller has that bound.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import subprocess
from derive_ddr_matcher import HERE,derive_store,derive_row_engine,derive_fast_cache
from derive_ddr_shared import derive_store as share_store,derive_cache


def run(out,row):
    assert not out.exists();out.mkdir()
    assert json.loads((row/'functional_result.json').read_text())['passed']
    names=['nav_ddr_native.v','nav_ddr_row_stream.v','nav_ddr_struct_cache.v']
    for n in names:shutil.copyfile(HERE/n,out/n)
    for n,src in [
        ('nav_ddr_structure_store_full.v',share_store(derive_store((HERE/'nav_ddr_structure_store.v').read_text()))),
        ('nav_slf_row_update_full.v',derive_row_engine((HERE/'nav_slf_row_update.v').read_text())),
        ('nav_ddr_struct_cache_fast.v',derive_fast_cache((HERE/'nav_ddr_struct_cache.v').read_text())),
        ('nav_ddr_em_cache.v',derive_cache((HERE/'nav_ddr_map_cache.v').read_text()))]:
        (out/n).write_text(src);names.append(n)
    shutil.copyfile(row/'slf_lf_ce.v',out/'slf_lf_ce.v');names.append('slf_lf_ce.v')
    for p in row.glob('case*.hex'):shutil.copyfile(p,out/p.name)
    shutil.copyfile(row/'cases.txt',out/'cases.txt')
    model=(HERE/'tb_slf_row_update.sv').read_text().split('module tb_slf_row_update;')[0]
    for a,b in [('[0:8191]','[0:16383]'),('[12:0]held_addr','[13:0]held_addr'),('addr[27:16]','addr[27:17]'),('addr[15:3]','addr[16:3]')]:
        assert model.count(a)==1,a;model=model.replace(a,b)
    (out/'row_service_model.sv').write_text(model)
    tb=(HERE/'tb_structure_store.sv').read_text()
    def one(a,b):
        nonlocal tb
        assert tb.count(a)==1,a[:100];tb=tb.replace(a,b)
    one('    nav_ddr_structure_store dut', '    nav_ddr_structure_store_full dut')
    one('        cmd,cmd_en,addr,cmd_ready,wd,wm,wen,wend,wready,rd,rv,re);', '''        cmd,cmd_en,addr,cmd_ready,wd,wm,wen,wend,wready,rd,rv,re,
        1'b0,1'b0,1'b0,16'd0,,,em_req,em_we,em_addr,em_wdata,em_ack,em_reply,1'b0,,);''')
    one('    nav_ddr_structure_store_full dut', '''    reg em_req=0,em_we=0,em_finished=0;
    reg[13:0]em_addr=0;reg[52:0]em_wdata=0;
    wire em_ack;wire[52:0]em_reply;
    localparam[52:0]EM_PATTERN=53'h1a123456789abc;
    integer em_checks=0;
    nav_ddr_structure_store_full dut''')
    one('    task automatic access(', '''    task automatic em_access(input bit w,input[13:0]a,input[52:0]d,input[52:0]gold);
        integer waited;
        begin
            @(negedge clk);em_req=1;em_we=w;em_addr=a;em_wdata=d;waited=0;
            do begin @(posedge clk);#1;waited++;
                if(waited>2000000)$fatal(1,"EM takeover client timeout");end while(!em_ack);
            if(!w&&em_reply!==gold)$fatal(1,"EM takeover data mismatch");
            @(negedge clk);em_req=0;
            do begin @(posedge clk);#1;end while(em_ack);
            em_checks++;
        end
    endtask
    task automatic access(''')
    one('            refresh_phase=offset;', '''            for(integer a=8192;a<16384;a++)dram.mem[a]='1;
            refresh_phase=offset;''')
    # The accepted old test used cold reset solely to invalidate a clean
    # cache. The new design additionally initializes backing DDR, so reload
    # that test case after this explicit cold initialization completes.
    one('rst=0;wait(ready);\n            end', '''rst=0;wait(ready);
                for(integer a=0;a<65536;a++)
                    dram.mem[(a/256)*19+(a%256)/14][((a%256)%14)*18+:18]=initial_words[a];
            end''')
    one('            @(negedge clk);cx=cxi;cy=cyi;started=cycles;start=1;', '''            em_access(1,0,EM_PATTERN,0);em_finished=0;
            fork begin
                em_access(0,1024,0,{53{1'b1}});
                // This new request is held throughout exclusive row DMA.
                wait(dut.state==4);em_access(0,0,0,EM_PATTERN);em_finished=1;
            end join_none
            wait(dut.u_em.state==5);
            @(negedge clk);cx=cxi;cy=cyi;started=cycles;start=1;''')
    one('            // Cold-reset only after', '''            wait(em_finished);
            if(dram.mem[8192][0+:53]!==EM_PATTERN)$fatal(1,"EM dirty write lost at handover");
            // Cold-reset only after''')
    one('        $fclose(fd);', '        $fclose(fd);$display("EM_BOUND_PROOF client_checks=%0d",em_checks);')
    (out/'tb_structure_store.sv').write_text(tb)
    commands=[('xvlog',names),('xvlog',['-sv','row_service_model.sv','tb_structure_store.sv']),
        ('xelab',['-debug','off','-timescale','1ns/1ps','tb_structure_store','-s','snap']),('xsim',['snap','-runall'])]
    for i,(tool,args) in enumerate(commands):
        with (out/f'{i}_{tool}_stdout.log').open('w') as log:
            subprocess.run(['cmd','/c',str(Path('E:/2026.1/Vivado/bin')/(tool+'.bat')),*args],cwd=out,
                stdout=log,stderr=subprocess.STDOUT,timeout=1800,check=True,creationflags=subprocess.CREATE_NO_WINDOW)
    s=(out/'3_xsim_stdout.log').read_text(errors='replace')
    assert 'TEST PASSED' in s and not re.search(r'FATAL|Fatal|ERROR:',s),s[-4000:]
    result=dict(tested_at=datetime.now().astimezone().isoformat(),passed=True,
        timing=[{k:float(v) if '.' in v else int(v) for k,v in re.findall(r'(\w+)=([\d.]+)',l)} for l in s.splitlines() if l.startswith('COHERENCE_TIMING')],
        metrics={k:int(v) for k,v in re.findall(r'(\w+)=(\d+)',re.search(r'TEST PASSED[^\r\n]*',s)[0])},
        elevation_checks=[{k:int(v) for k,v in re.findall(r'(\w+)=(\d+)',l)} for l in s.splitlines() if l.startswith('EM_BOUND_PROOF')],
        scope='all8 accepted row workloads, full dirty SM barrier, dirty EM eviction/handover and modeled native service',
        original_point_transform_tested=False,full_matcher_tested=False,
        real_worst_update_under_5ms_verified=False,physical_resources_verified=False,full_joint_implementation=False,programmed=False)
    (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--row-probe',type=Path,required=True)
    a=p.parse_args();run(a.out.resolve(),a.row_probe.resolve())
