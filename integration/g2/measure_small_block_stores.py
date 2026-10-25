"""Exhaustive address/collision/hold checks plus actual small physical mapping.

Original full-joint hierarchical utilization is used for the area baseline;
it is not resynthesized. No test replaces the later complete joint checks.
"""
import argparse
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from bounded_tool import run_tool
from measure_slf_row_banks import TOOL, LIBRARY, simulate, namespaced, inventory
from preflight import validate
from prepare_small_block_stores import transform
from retry_frozen_synthesis import memory_status

TOP = '''module g2_top(input clk,we,re,input[13:0]wa,ra,input[31:0]wd,output bq,output[31:0]mq);
nav_ram_sdp #(.DW(1),.AW(14),.INIT_VAL(1'b1)) bitmap(clk,we,wa,wd[0],re,ra,bq);
nav_ram_sdp #(.DW(32),.AW(9),.INIT_VAL(32'h12345678)) mailbox(clk,we,wa[8:0],wd,re,ra[8:0],mq);
endmodule
'''
TB = '''`timescale 1ns/1ps
module tb;
reg clk=0;always #5 clk=~clk;
reg we=0,re=0;reg[13:0]wa=0,ra=0;reg[31:0]wd=0;
wire ob,nb;wire[31:0]om,nm;
g2_old a(clk,we,re,wa,ra,wd,ob,om);
g2_new b(clk,we,re,wa,ra,wd,nb,nm);
reg bg[0:16383],be=0;reg[31:0]mg[0:511],me=0;
integer checks=0,reads=0,writes=0,collisions=0,holds=0;reg[31:0]rng=32'h32603184;
always @(posedge clk)begin
 if(re)begin be<=bg[ra];me<=mg[ra[8:0]];reads++;end else holds++;
 if(we)begin bg[wa]<=wd[0];mg[wa[8:0]]<=wd;writes++;end
 if(we&&re&&wa==ra)collisions++;
end
always @(negedge clk)begin
 checks++;
 if({ob,om}!=={be,me}||{nb,nm}!=={be,me})
  $fatal(1,"memory mismatch cycle=%0d addr=%d/%d old=%h new=%h expected=%h",checks,wa,ra,{ob,om},{nb,nm},{be,me});
end
task step(input w,r,input[13:0]x,y,input[31:0]d);
 begin @(negedge clk);#1;we=w;re=r;wa=x;ra=y;wd=d;end
endtask
initial begin
 for(integer k=0;k<16384;k++)bg[k]=1;
 for(integer k=0;k<512;k++)mg[k]=32'h12345678;
 repeat(5)step(0,0,0,0,0);
 for(integer k=0;k<16384;k++)step(0,1,0,k,0);
 for(integer k=0;k<16384;k++)begin
  step(1,1,k,k,32'h89abcdef^k);step(0,1,0,k,0);step(0,0,16383-k,16383-k,0);
 end
 for(integer k=0;k<8192;k++)begin
  rng=rng^(rng<<13);rng=rng^(rng>>17);rng=rng^(rng<<5);
  step(rng[31],rng[30],rng[13:0],rng[27:14],rng);
 end
 for(integer k=16383;k>=0;k--)step(0,1,0,k,0);
 repeat(5)step(0,0,0,0,0);
 $display("TEST PASSED checks=%0d reads=%0d writes=%0d collisions=%0d holds=%0d all16384bitmap=1 all512mailbox=1 nonzeroINIT=1 read_first=1",checks,reads,writes,collisions,holds);$finish;
end
initial begin #2000000;$fatal(1,"finite stimulus did not finish");end
endmodule
'''


def main():
    ap=argparse.ArgumentParser();ap.add_argument('source',type=Path);ap.add_argument('--out',required=True,type=Path)
    ap.add_argument('--collect-existing',action='store_true',help='reuse finished RTL/SYN/PNR/mapped evidence; never rerun the tool')
    a=ap.parse_args();source=a.source.resolve();out=a.out.resolve()
    if a.collect_existing:
        result=json.loads((out/'result.json').read_text())
        assert Path(result['source']).resolve()==source
        expected=transform((source/'nav/rtl/common/nav_ram_sdp.v').read_text(encoding='utf-8'))
        assert (out/'physical/ram.v').read_text(encoding='utf-8')==expected
        assert [c['variant'] for c in result['checks']]==['rtl','mapped']
        for row in result['checks']:
            assert row['result'] in (out/row['variant']/'xsim.log').read_text(errors='replace')
        finish(out,result);return
    out.mkdir(parents=True,exist_ok=False)
    old=(source/'nav/rtl/common/nav_ram_sdp.v').read_text(encoding='utf-8');new=transform(old)
    result=dict(source=str(source),build=str(out),checks=[],physical_run=None,programmed=False,
                read_latency_cycles=1,read_first=True,capacity_changed=False,
                scope='isolated exact memory behavior and physical mapping; no joint routability claim')
    def save():(out/'result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    for suffix,text in [('old',old),('new',new)]:
        (out/(suffix+'.v')).write_text(text.replace('module nav_ram_sdp #(','module ram_'+suffix+' #(',1),encoding='utf-8')
        (out/(suffix+'_top.v')).write_text(TOP.replace('g2_top','g2_'+suffix).replace('nav_ram_sdp','ram_'+suffix),encoding='utf-8')
    result['checks'].append(simulate(out,'rtl',[out/n for n in ('old.v','new.v','old_top.v','new_top.v')],TB));save()
    folder=out/'physical';folder.mkdir();(folder/'ram.v').write_text(new,encoding='utf-8');(folder/'top.v').write_text(TOP,encoding='utf-8')
    (folder/'clock.sdc').write_text('create_clock -name clk -period 20 [get_ports {clk}]\n',encoding='utf-8')
    settings=['set_device -device_version C GW5AT-LV138PG484AC1/I0']
    settings += [f'add_file -type verilog {{{(folder/n).as_posix()}}}' for n in ('ram.v','top.v')]
    settings += [f'add_file -type sdc {{{(folder/"clock.sdc").as_posix()}}}','set_option -top_module g2_top',
                 'set_option -output_base_name g2_top','set_option -global_freq 50','set_option -convert_sdp32_36_to_sdp16_18 1',
                 'set_option -looplimit 100000','set_option -place_option 1','set_option -route_option 2',
                 'set_option -gen_verilog_sim_netlist 1']
    effective=validate(TOOL,folder,settings,50,1)
    (folder/'run.tcl').write_text('\n'.join(settings+['run syn','run pnr','exit'])+'\n',encoding='utf-8')
    memory=memory_status()
    assert memory.avail_phys>=2*1024**3 and memory.avail_page>=4*1024**3,'host 2/4GiB reserve'
    runtime=run_tool([str(TOOL),'run.tcl'],folder,max_runtime_s=0,max_no_progress_s=0)
    assert runtime['tool_exit']==0 and not runtime['cancelled'],runtime
    log=(folder/'gw_sh.log').read_text(errors='replace');assert not re.search(r'\bERROR\s*\(',log),log[-3000:]
    syn=folder/'impl/gwsynthesis';report=(folder/'impl/pnr/g2_top.rpt.txt').read_text(errors='replace')
    phys=int(re.search(r'^\s+BSRAM\s+\|\s+(\d+)/',report,re.M)[1])
    result['physical_run']=dict(runtime=runtime,effective_options=effective,actual_bsram=phys,
       resources=ET.parse(syn/'g2_top_syn_rsc.xml').getroot().attrib,inventory=inventory(syn/'g2_top.vg'),
       startup_free_bytes=memory.avail_phys,startup_commit_reserve_bytes=memory.avail_page)
    save()
    (out/'new_vendor.v').write_text(namespaced((syn/'g2_top.vg').read_text(encoding='utf-8'),'new'),encoding='utf-8')
    result['checks'].append(simulate(out,'mapped',[LIBRARY,out/'old.v',out/'old_top.v',out/'new_vendor.v'],TB));save()
    finish(out,result)


def finish(out,result):
    folder=out/'physical';syn=folder/'impl/gwsynthesis'
    runtime=json.loads((folder/'tool_runtime.json').read_text())
    assert runtime['tool_exit']==0 and not runtime['cancelled']
    syntext=(syn/'g2_top.vg').read_text(encoding='utf-8')
    phytext=(folder/'impl/pnr/g2_top.vo').read_text(encoding='utf-8')
    widths=lambda t:sorted(int(v) for v in re.findall(r'\.BIT_WIDTH_0\s*=\s*(\d+);',t))
    assert widths(syntext)==[1,32], 'source must be one bitmap and one merged mailbox leaf'
    assert widths(phytext)==[1,16,16], 'physical mailbox split must yield exactly two narrow blocks'
    assert result['physical_run']['actual_bsram']==3 and result['physical_run']['inventory']['bsram']==2
    assert result['physical_run']['inventory']['ff']==0
    result['mapping_review']=dict(source_widths=[1,32],physical_widths=[1,16,16],
        merged_mailbox_source=True,physical_split_confirmed=True,source_ff=0,
        initial_expectation_corrected='two narrow RTL planes merged by synthesis; original source-count assertion rejected after both tests passed')
    (out/'new_physical.v').write_text(namespaced(phytext,'new'),encoding='utf-8')
    result['checks'].append(simulate(out,'physical_mapped',[LIBRARY,out/'old.v',out/'old_top.v',out/'new_physical.v'],TB))
    result.update(passed=True,bitmap_physical_bsram=1,mailbox_physical_bsram=2,
                  predicted_joint_physical_bsram=333,predicted_extra_bsram=7,
                  joint_result_still_pending=True)
    (out/'result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(passed=result['passed'],checks=result['checks'],mapping=result['mapping_review'])),flush=True)


if __name__=='__main__':main()
