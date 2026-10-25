"""Prove row-read sharing and measure one small SYN alongside the owned PNR.

The previous isolated banked SYN is reused after checking its active branch,
wrapper and settings. No full synthesis, routing or hardware operation occurs.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from measure_slf_row_banks import (TOOL,LIBRARY,WRAPPER,simulate,namespaced,
                                   inventory,run_syn)
from preflight import validate
from prepare_slf_row_banks import BRANCH
from prepare_row_read_share import transform
from retry_frozen_synthesis import memory_status

TB = '''`timescale 1ns/1ps
module tb;
reg clk=0;always #5 clk=~clk;
reg we=0,re=0;reg[7:0]wa=0,ra=0;reg[97:0]wd=0;
wire[97:0]oldq,newq;reg[97:0]gold[0:159],expected=0;
g2_old a(clk,we,re,wa,ra,wd,oldq);
g2_new b(clk,we,re,wa,ra,wd,newq);
integer checks=0,reads=0,writes=0,collisions=0,holds=0;
reg[31:0]rng=32'h45001840;
always @(posedge clk)begin
 if(re)begin expected<=gold[ra];reads++;end else holds++;
 if(we)begin gold[wa]<=wd;writes++;end
 if(we&&re&&wa==ra)collisions++;
end
always @(negedge clk)begin
 checks++;
 if(oldq!==expected || newq!==expected)
  $fatal(1,"row mismatch cycle=%0d wa=%0d ra=%0d old=%h new=%h expected=%h",checks,wa,ra,oldq,newq,expected);
end
task step(input w,r,input[7:0]x,y,input[97:0]d);
 begin @(negedge clk);#1;we=w;re=r;wa=x;ra=y;wd=d;end
endtask
initial begin
 for(integer k=0;k<160;k++)gold[k]=INIT_VALUE;
 repeat(5)step(0,0,0,0,0);
 for(integer k=0;k<160;k++)step(0,1,0,k,0);
 // All25600 pairs, including same bank/different bank and read/write collision.
 for(integer w=0;w<160;w++)for(integer r=0;r<160;r++)begin
  rng=rng^(rng<<13);rng=rng^(rng>>17);rng=rng^(rng<<5);
  step(1,1,w,r,{rng[1:0],rng,~rng,rng^32'h98abcdef});
  step(0,1,0,w,0);
  step(0,0,159-w,159-r,0);
 end
 // Continuous read/write, held output during writes, and every data bit.
 for(integer k=0;k<160;k++)for(integer b=0;b<98;b++)begin
  step(1,0,k,159-k,98'd1<<b);step(0,1,0,k,0);
 end
 for(integer k=0;k<8192;k++)begin
  rng=rng^(rng<<13);rng=rng^(rng>>17);rng=rng^(rng<<5);
  step(rng[31],rng[30],rng[15:8]%160,rng[23:16]%160,{rng[1:0],rng,~rng,rng^32'habcdef12});
 end
 for(integer k=159;k>=0;k--)step(0,1,0,k,0);
 repeat(5)step(0,0,0,0,0);
 $display("TEST PASSED checks=%0d reads=%0d writes=%0d collisions=%0d holds=%0d all25600_pairs=1 all98bits=1",checks,reads,writes,collisions,holds);$finish;
end
initial begin #1500000;$fatal(1,"finite row stimulus did not complete");end
endmodule
'''


def main():
    p=argparse.ArgumentParser();p.add_argument('source',type=Path)
    p.add_argument('--baseline-probe',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--rtl-only',action='store_true')
    p.add_argument('--resume-synthesis',action='store_true')
    a=p.parse_args();source=a.source.resolve();out=a.out.resolve();base=a.baseline_probe.resolve()
    old=(source/'nav/rtl/common/nav_ram_sdp.v').read_text(encoding='utf-8');new=transform(old)
    baseline=json.loads((base/'result.json').read_text())
    assert (base/'banked/ram.v').read_text(encoding='utf-8').count(BRANCH)==old.count(BRANCH)==1
    assert (base/'banked/top.v').read_text()==WRAPPER
    previous,=[r for r in baseline['runs'] if r['variant']=='banked']
    assert previous['runtime']['tool_exit']==0 and previous['inventory']['bsram']==0
    if a.resume_synthesis:
        result=json.loads((out/'result.json').read_text())
        assert result['source']==str(source) and result['phase']=='rtl_passed_resource_pending'
        assert (out/'candidate.v').read_text(encoding='utf-8')==new
        assert len(result['checks'])==2
    else:
        out.mkdir(parents=True,exist_ok=False)
        result=dict(source=str(source),baseline_probe=str(base),baseline=previous,
                    phase='rtl_validation',checks=[],programmed=False,new_joint_build=False,
                    previous_PNR_inputs_modified=False,capacity_bits=15680,read_latency_cycles=1,
                    scope='isolated160x98 row storage; joint resources/routing/timing not accepted')
        (out/'candidate.v').write_text(new,encoding='utf-8')
    def save():(out/'result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    if not a.resume_synthesis:
        for suffix,text in [('old',old),('new',new)]:
            (out/(suffix+'.v')).write_text(text.replace('module nav_ram_sdp #(','module ram_'+suffix+' #(',1),encoding='utf-8')
        for variant,value in [('zero',"98'd0"),('nonzero',"98'h23456789abcdef123456789ab")]:
            wrapper=WRAPPER.replace('.DEPTH(160)',f'.DEPTH(160),.INIT_VAL({value})')
            for suffix in ('old','new'):
                (out/(suffix+'_'+variant+'.v')).write_text(wrapper.replace('g2_top','g2_'+suffix).replace('nav_ram_sdp','ram_'+suffix),encoding='utf-8')
            result['checks'].append(simulate(out,'rtl_'+variant,[out/n for n in ('old.v','new.v','old_'+variant+'.v','new_'+variant+'.v')],TB.replace('INIT_VALUE',value)));save()
        result['phase']='rtl_passed_resource_pending';save()
    if a.rtl_only:return
    m=memory_status()
    assert m.avail_phys>=1024**3 and m.avail_page>=3*1024**3,'existing small-module1/3GiB reserve'
    folder=out/'candidate_syn';folder.mkdir()
    (folder/'ram.v').write_text(new,encoding='utf-8');(folder/'top.v').write_text(WRAPPER,encoding='utf-8')
    (folder/'clock.sdc').write_text('create_clock -name clk -period 20 [get_ports {clk}]\n',encoding='utf-8')
    settings=['set_device -device_version C GW5AT-LV138PG484AC1/I0']
    settings += [f'add_file -type verilog {{{(folder/n).as_posix()}}}' for n in ('ram.v','top.v')]
    settings += [f'add_file -type sdc {{{(folder/"clock.sdc").as_posix()}}}','set_option -top_module g2_top','set_option -output_base_name g2_top','set_option -global_freq 50','set_option -convert_sdp32_36_to_sdp16_18 0','set_option -looplimit 100000']
    effective=validate(TOOL,folder,settings,50,0)
    (folder/'run.tcl').write_text('\n'.join(settings+['run syn','exit'])+'\n',encoding='utf-8')
    runtime=run_syn(folder,'isolated160x98 shared read register, not a joint build')
    syn=folder/'impl/gwsynthesis';actual=inventory(syn/'g2_top.vg')
    result['candidate']=dict(runtime=runtime,effective_options=effective,
                             resources=ET.parse(syn/'g2_top_syn_rsc.xml').getroot().attrib,
                             inventory=actual,startup_free_bytes=m.avail_phys,startup_commit_reserve_bytes=m.avail_page)
    result['phase']='synthesized_vendor_validation_pending';save()
    (out/'candidate_vendor.v').write_text(namespaced((syn/'g2_top.vg').read_text(encoding='utf-8'),'new'),encoding='utf-8')
    result['checks'].append(simulate(out,'mapped',[LIBRARY,out/'old.v',out/'old_zero.v',out/'candidate_vendor.v'],TB.replace('INIT_VALUE',"98'd0")))
    before=previous['inventory'];result['area_delta']={k:actual[k]-before[k] for k in ('ff','bsram')}
    for k in ('T_Lut','T_Ssram'):
        count=lambda d:int(d.get(k,'0').split('(')[0])
        result['area_delta'][k]=count(result['candidate']['resources'])-count(previous['resources'])
    result['connection_pressure_not_increased']=actual['maximum_nonclock_load']<=before['maximum_nonclock_load']
    result['candidate_eligible_for_later_joint_review']=(actual['bsram']==0 and result['area_delta']['ff']<0 and result['area_delta']['T_Lut']<=0 and result['connection_pressure_not_increased'])
    result.update(phase='isolated_proof_complete',completed_at=datetime.now().astimezone().isoformat())
    save();print(json.dumps({k:result[k] for k in ('area_delta','connection_pressure_not_increased','candidate_eligible_for_later_joint_review','checks')}),flush=True)


if __name__=='__main__':main()
