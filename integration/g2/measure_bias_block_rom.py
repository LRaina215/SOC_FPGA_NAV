"""Measure and verify the exact enabled signed bias read in vendor primitives.

Use every original32-bit bias as oracle for its unchanged26-bit ROM encoding.
No MAC/state-machine changes, physical-routing acceptance or SHA checks.
"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from measure_slf_row_banks import TOOL,LIBRARY,run_syn,simulate,namespaced,inventory
from prepare_bias_block_rom import OLD,NEW,transform
from preflight import validate

TOP = '''module g2_top(input clk,en,input[8:0]addr,output reg signed[31:0]q);
DECLARATION
initial $readmemh("bias_narrow.mem",bias_mem);
always@(posedge clk)if(en)q<=bias_mem[addr];
endmodule
'''
TB = '''`timescale 1ns/1ps
module tb;
reg clk=0;always #5 clk=~clk;
reg en=0;reg[8:0]a=0;wire signed[31:0]oldq,newq;
reg signed[31:0]gold[0:424],expected;reg seen=0;
reg[31:0]rng=32'h4500425;integer checks=0,reads=0,holds=0;
g2_old oldrom(clk,en,a,oldq);g2_new newrom(clk,en,a,newq);
always@(posedge clk)if(en)begin expected<=gold[a];seen<=1;reads++;end else holds++;
always@(negedge clk)if(seen)begin
 checks++;
 if(oldq!==expected || newq!==expected)
  $fatal(1,"bias signed/enable/latency mismatch addr=%0d old=%h new=%h expected=%h",a,oldq,newq,expected);
end
task request(input e,input[8:0]address);
 begin @(negedge clk);#1;en=e;a=address;end
endtask
initial begin
 $readmemh("bias_original32.mem",gold);
 repeat(5)request(0,424);
 for(integer i=0;i<425;i++)begin request(1,i);if(i%7==0)request(0,424-i);end
 for(integer i=424;i>=0;i--)request(1,i);
 for(integer i=0;i<2048;i++)begin
  rng=rng^(rng<<13);rng=rng^(rng>>17);rng=rng^(rng<<5);
  request(rng[31],rng[8:0]%425);
 end
 repeat(10)request(0,0);
 $display("TEST PASSED checks=%0d reads=%0d holds=%0d all425signed_values=1",checks,reads,holds);$finish;
end
initial begin #100000;$fatal(1,"timeout");end
endmodule
'''


def main():
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();source=a.source.resolve();out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    core=(source/'fc_v4_top.v').read_text(encoding='utf-8');transform(core)
    narrow=(source/'src/mem/bias_narrow.mem').read_bytes();original=(source/'control/mem/bias.mem').read_bytes()
    def signed(data,bits):
        return [int(v,16)-(1<<bits) if int(v,16)&(1<<(bits-1)) else int(v,16) for v in data.split()]
    assert len(signed(original,32)) == 425 and signed(narrow,26) == signed(original,32)
    result=dict(source=str(source),build=str(out),geometry=dict(words=425,signed_width=26),
                all_original32bit_biases_identical=True,read_latency_cycles=1,additional_read_cycles=0,
                weights_or_algorithm_changed=False,programmed=False,deployment_accepted=False,runs=[],checks=[],
                scope='isolated bias ROM numeric, enabled-read and actual vendor INIT checks; full SYN/PNR still required')
    def save(): (out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8',newline='\n')
    for variant,decl in (('original',OLD),('block',NEW)):
        folder=out/variant;folder.mkdir();top=TOP.replace('DECLARATION',decl)
        (folder/'top.v').write_text(top,encoding='utf-8');(folder/'bias_narrow.mem').write_bytes(narrow)
        (folder/'clock.sdc').write_text('create_clock -name clk -period 20 [get_ports {clk}]\n',encoding='utf-8')
        settings=['set_device -device_version C GW5AT-LV138PG484AC1/I0',
                  f'add_file -type verilog {{{(folder/"top.v").as_posix()}}}',
                  f'add_file -type sdc {{{(folder/"clock.sdc").as_posix()}}}',
                  'set_option -top_module g2_top','set_option -output_base_name g2_top',
                  'set_option -global_freq 50','set_option -convert_sdp32_36_to_sdp16_18 0','set_option -looplimit 100000']
        effective=validate(TOOL,folder,settings,50,0);(folder/'run.tcl').write_text('\n'.join(settings+['run syn','exit'])+'\n',encoding='utf-8')
        runtime=run_syn(folder,'isolated425x26 exact signed bias ROM only');syn=folder/'impl/gwsynthesis'
        r=dict(variant=variant,effective_options=effective,runtime=runtime,
               resources=ET.parse(syn/'g2_top_syn_rsc.xml').getroot().attrib,inventory=inventory(syn/'g2_top.vg'))
        result['runs'].append(r);save();print(json.dumps(dict(variant=variant,resources=r['resources'],loads=r['inventory']['maximum_nonclock_load'])),flush=True)
        suffix='old' if variant=='original' else 'new'
        (out/(variant+'_vendor.v')).write_text(namespaced((syn/'g2_top.vg').read_text(),suffix),encoding='utf-8')
        (out/(variant+'_rtl.v')).write_text(top.replace('g2_top','g2_'+suffix),encoding='utf-8')
    # Each simulator reads the unchanged ROM files in its own working folder.
    for variant,files in [('rtl',[out/'original_rtl.v',out/'block_rtl.v']),
                          ('vendor',[LIBRARY,out/'original_vendor.v',out/'block_vendor.v'])]:
        # simulate creates its folder, so prepare by adding the file load paths
        # to the bench; no simulator-generated or source assets are overwritten.
        bench=TB.replace('"bias_original32.mem"','"'+(out/'bias_original32.mem').as_posix()+'"')
        if variant=='rtl':
            adjusted=[]
            for path in files:
                text=path.read_text().replace('"bias_narrow.mem"','"'+(out/'bias_narrow.mem').as_posix()+'"')
                target=out/(path.stem+'_absolute.v');target.write_text(text,encoding='utf-8');adjusted.append(target)
            files=adjusted
        (out/'bias_original32.mem').write_bytes(original);(out/'bias_narrow.mem').write_bytes(narrow)
        result['checks'].append(simulate(out,variant,files,bench));save()
    before,after=[x['inventory'] for x in result['runs']]
    assert before['primitives'].get('ROM16',0)==702 and after['primitives'].get('ROM16',0)==0
    assert 1 <= after['bsram'] <= 2 and before['bsram']==0
    assert after['maximum_nonclock_load'] < before['maximum_nonclock_load']
    result.update(rom16_removed=702,connection_pressure_reduced=True,physical_bsram_addition_still_pending=True)
    save();print(json.dumps({k:v for k,v in result.items() if k!='runs'}),flush=True)


if __name__=='__main__':main()
