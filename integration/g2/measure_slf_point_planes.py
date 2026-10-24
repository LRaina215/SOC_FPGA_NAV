"""Prove point-store cycles and measure both actual vendor implementations."""
import argparse
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from measure_slf_row_banks import TOOL, LIBRARY, simulate, namespaced, inventory
from bounded_tool import run_tool
from preflight import validate
from prepare_slf_point_planes import transform
from retry_frozen_synthesis import memory_status

TOP = '''module g2_top(input clk,we,re,input[11:0]waddr,raddr,input[35:0]wdata,output[35:0]rdata);
nav_ram_sdp #(.DW(36),.AW(12),.DEPTH(4096),.INIT_VAL(36'h123456789)) u_ram(clk,we,waddr,wdata,re,raddr,rdata);
endmodule
'''
TB = '''`timescale 1ns/1ps
module tb;
reg clk=0;always #5 clk=~clk;
reg we=0,re=0;reg[11:0]wa=0,ra=0;reg[35:0]wd=0;
wire[35:0]oldq,newq;reg[35:0]gold[0:4095],expected=0;
g2_old a(clk,we,re,wa,ra,wd,oldq);
g2_new b(clk,we,re,wa,ra,wd,newq);
integer checks=0,reads=0,writes=0,collisions=0,holds=0;
reg[31:0]rng=32'h45004096;
always@(posedge clk)begin
 if(re)begin expected<=gold[ra];reads++;end else holds++;
 if(we)begin gold[wa]<=wd;writes++;end
 if(we&&re&&wa==ra)collisions++;
end
always@(negedge clk)begin
 checks++;
 if(oldq!==expected || newq!==expected)
  $fatal(1,"point memory mismatch cycle=%0d old=%h new=%h expected=%h we=%b re=%b wa=%0d ra=%0d",checks,oldq,newq,expected,we,re,wa,ra);
end
task step(input w,r,input[11:0]x,y,input[35:0]d);
 begin @(negedge clk);#1;we=w;re=r;wa=x;ra=y;wd=d;end
endtask
initial begin
 for(integer k=0;k<4096;k++)gold[k]=36'h123456789;
 repeat(5)step(0,0,4095,0,0);
 for(integer k=0;k<4096;k++)step(0,1,0,k,0);
 // Every address collides once, then read back after the write and hold.
 for(integer k=0;k<4096;k++)begin
  step(1,1,k,k,{4'hf,32'h89abcdef}^k);
  step(0,1,4095-k,k,0);step(0,0,k,4095-k,0);
 end
 for(integer k=0;k<8192;k++)begin
  rng=rng^(rng<<13);rng=rng^(rng>>17);rng=rng^(rng<<5);
  step(rng[31],rng[30],rng[11:0],rng[23:12],{rng[3:0],rng});
 end
 for(integer k=4095;k>=0;k--)step(0,1,k,k,0);
 repeat(5)step(0,0,0,0,0);
 $display("TEST PASSED checks=%0d reads=%0d writes=%0d collisions=%0d holds=%0d all4096addresses=1 init_nonzero=1",checks,reads,writes,collisions,holds);$finish;
end
initial begin #1000000;$fatal(1,"timeout");end
endmodule
'''


def main():
    ap=argparse.ArgumentParser();ap.add_argument('source',type=Path);ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args();source=a.source.resolve();out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    old=(source/'nav/rtl/common/nav_ram_sdp.v').read_text(encoding='utf-8');new=transform(old)
    result=dict(source=str(source),build=str(out),geometry=dict(words=4096,width=36,logical_bits=147456),
                read_latency_cycles=1,read_first=True,capacity_unchanged=True,programmed=False,deployment_accepted=False,
                scope='isolated memory RTL/vendor cycles and actual module PNR; joint effects still require full SYN/PNR',runs=[],checks=[])
    def save(): (out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8',newline='\n')
    for suffix,text in [('old',old),('new',new)]:
        (out/(suffix+'.v')).write_text(text.replace('module nav_ram_sdp #(','module ram_'+suffix+' #(',1),encoding='utf-8')
        (out/(suffix+'_top.v')).write_text(TOP.replace('g2_top','g2_'+suffix).replace('nav_ram_sdp','ram_'+suffix),encoding='utf-8')
    result['checks'].append(simulate(out,'rtl',[out/x for x in ('old.v','new.v','old_top.v','new_top.v')],TB));save()
    for variant,text in [('original',old),('planes',new)]:
        folder=out/variant;folder.mkdir();(folder/'ram.v').write_text(text,encoding='utf-8');(folder/'top.v').write_text(TOP,encoding='utf-8')
        (folder/'clock.sdc').write_text('create_clock -name clk -period 20 [get_ports {clk}]\n',encoding='utf-8')
        settings=['set_device -device_version C GW5AT-LV138PG484AC1/I0']
        settings += [f'add_file -type verilog {{{(folder/f).as_posix()}}}' for f in ('ram.v','top.v')]
        settings += [f'add_file -type sdc {{{(folder/"clock.sdc").as_posix()}}}','set_option -top_module g2_top',
                     'set_option -output_base_name g2_top','set_option -global_freq 50',
                     'set_option -convert_sdp32_36_to_sdp16_18 1','set_option -looplimit 100000',
                     'set_option -place_option 1','set_option -route_option 2']
        effective=validate(TOOL,folder,settings,50,1)
        (folder/'run.tcl').write_text('\n'.join(settings+['run syn','run pnr','exit'])+'\n',encoding='utf-8')
        memory=memory_status()
        assert memory.avail_phys>=2*1024**3 and memory.avail_page>=4*1024**3, 'PNR host reserve'
        runtime=run_tool([str(TOOL),'run.tcl'],folder,max_runtime_s=0,max_no_progress_s=0)
        assert runtime['tool_exit']==0 and not runtime['cancelled'], runtime
        assert not re.search(r'\bERROR\s*\(', (folder/'gw_sh.log').read_text(errors='replace'))
        syn=folder/'impl/gwsynthesis';report=(folder/'impl/pnr/g2_top.rpt.txt').read_text(errors='replace')
        physical=int(re.search(r'^\s+BSRAM\s+\|\s+(\d+)/',report,re.M)[1])
        row=dict(variant=variant,runtime=runtime,effective_options=effective,
                 synthesis_resources=ET.parse(syn/'g2_top_syn_rsc.xml').getroot().attrib,
                 inventory=inventory(syn/'g2_top.vg'),actual_physical_bsram=physical)
        result['runs'].append(row);save();print(json.dumps(dict(variant=variant,actual_physical_bsram=physical)),flush=True)
        suffix='old' if variant=='original' else 'new'
        (out/(variant+'_vendor.v')).write_text(namespaced((syn/'g2_top.vg').read_text(encoding='utf-8'),suffix),encoding='utf-8')
    result['checks'].append(simulate(out,'vendor',[LIBRARY,out/'original_vendor.v',out/'planes_vendor.v'],TB))
    before,after=result['runs'];assert before['actual_physical_bsram']==16 and after['actual_physical_bsram']==8
    assert after['inventory']['bsram']==8
    result.update(actual_module_bsram_saved=8,all_cycles_passed=True,joint_physical_reduction_not_yet_measured=True)
    save();print(json.dumps({k:v for k,v in result.items() if k!='runs'}),flush=True)


if __name__=='__main__':main()
