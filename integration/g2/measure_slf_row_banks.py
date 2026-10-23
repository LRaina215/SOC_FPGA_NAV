"""Compare actual row-store connectivity and verify RTL/vendor read behavior.

One pair of small bounded SYNs. This cannot diagnose physical congestion or
grant G2 acceptance. Reject the candidate if measured connection pressure grows.
"""
import argparse
from collections import Counter, defaultdict
from datetime import datetime
import json
from pathlib import Path
import re
import subprocess
import time
import xml.etree.ElementTree as ET

import psutil
from audit_netlist_memories import BSRAM, MODULE
from audit_routing_topology import primitive_directions
from preflight import validate
from prepare_slf_row_banks import transform
from retry_frozen_synthesis import memory_status
from synthesis_connectivity import read

TOOL = Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe')
LIBRARY = Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
VIV = Path('E:/2026.1/Vivado/bin')
WRAPPER = '''module g2_top(input clk,we,re,input[7:0]waddr,raddr,input[97:0]wdata,output[97:0]rdata);
nav_ram_sdp #(.DW(98),.AW(8),.DEPTH(160)) u_ram(clk,we,waddr,wdata,re,raddr,rdata);
endmodule
'''
TB = '''`timescale 1ns/1ps
module tb;
reg clk=0;always #5 clk=~clk;
reg we=0,re=0;reg[7:0]wa=0,ra=0;reg[97:0]wd=0;
wire[97:0]oldq,newq;reg[97:0]gold[0:159],expected=0;
g2_old a(clk,we,re,wa,ra,wd,oldq);
g2_new b(clk,we,re,wa,ra,wd,newq);
integer checks=0,reads=0,writes=0,collisions=0,holds=0;
reg[31:0]rng=32'h45001024;
always@(posedge clk)begin
 if(re)begin expected<=gold[ra];reads++;end else holds++;
 if(we)begin gold[wa]<=wd;writes++;end
 if(we&&re&&wa==ra)collisions++;
end
always@(negedge clk)begin
 checks++;
 if(oldq!==expected || newq!==expected)
  $fatal(1,"row memory mismatch cycle=%0d old=%h new=%h expected=%h we=%b re=%b wa=%0d ra=%0d",checks,oldq,newq,expected,we,re,wa,ra);
end
task step(input w,r,input[7:0]x,y,input[97:0]d);
 begin @(negedge clk);#1;we=w;re=r;wa=x;ra=y;wd=d;end
endtask
initial begin
 for(integer k=0;k<160;k++)gold[k]=0;
 repeat(10)step(0,0,159,159,0);
 for(integer k=0;k<160;k++)step(0,1,k,k,0);
 // Every address, including all ten bank boundaries, is read before overwrite.
 for(integer k=0;k<160;k++)begin
  step(1,1,k,k,{2'b11,32'h01234567,32'h89abcdef,32'hfedcba98}^k);
  step(0,1,k,k,0);step(0,0,159-k,159-k,0);
 end
 for(integer k=0;k<8192;k++)begin
  rng=rng^(rng<<13);rng=rng^(rng>>17);rng=rng^(rng<<5);
  step(rng[31],rng[30],rng[15:8]%160,rng[23:16]%160,{rng[1:0],rng,~rng,rng^32'habcdef12});
 end
 for(integer k=0;k<160;k++)step(0,1,k,159-k,0);
 repeat(10)step(0,0,0,0,0);
 $display("TEST PASSED checks=%0d reads=%0d writes=%0d collisions=%0d holds=%0d",checks,reads,writes,collisions,holds);$finish;
end
initial begin #1000000;$fatal(1,"timeout");end
endmodule
'''


def run_syn(folder):
    m = memory_status()
    assert m.avail_phys >= 1024**3 and m.avail_page >= 3*1024**3, 'small SYN host reserve'
    state = dict(started_at=datetime.now().astimezone().isoformat(), max_runtime_s=180,
                 max_owned_rss=768*1024**2, scope='isolated160x98 row memory only', cancelled=False)
    started = time.monotonic()
    with (folder/'gw_sh.log').open('w', encoding='utf-8') as stream:
        p = subprocess.Popen([str(TOOL),'run.tcl'],cwd=folder,stdout=stream,stderr=subprocess.STDOUT,
                             creationflags=subprocess.CREATE_NO_WINDOW)
        state['pid'] = p.pid
        try:
            while p.poll() is None:
                used = 0
                for q in [psutil.Process(p.pid), *psutil.Process(p.pid).children(recursive=True)]:
                    try: used += q.memory_info().rss
                    except psutil.NoSuchProcess: pass
                m = memory_status()
                if time.monotonic()-started > 180 or used > 768*1024**2 or m.avail_page < 1024**3:
                    state.update(cancelled=True,reason='owned module budget or host memory reserve')
                    p.kill();p.wait();break
                time.sleep(.5)
        except psutil.NoSuchProcess:
            p.wait()
        finally:
            if p.poll() is None:p.kill();p.wait()
            state.update(tool_exit=p.returncode,elapsed_s=time.monotonic()-started)
            (folder/'module_runtime.json').write_text(json.dumps(state,indent=2),encoding='utf-8')
    assert p.returncode == 0 and not state['cancelled'], state
    assert not re.search(r'\bERROR\s*\(', (folder/'gw_sh.log').read_text(errors='replace'))
    return state


def inventory(path):
    cells = read(path);directions = primitive_directions(LIBRARY)
    loads = defaultdict(Counter)
    for c in cells.values():
        for pin,nets in c['ports'].items():
            if directions[c['kind']][pin] in ('input','inout') and pin not in ('CLK','CLKA','CLKB'):
                for n in nets:
                    if n not in ('VCC','GND'):loads[n][pin] += 1
    rows = sorted((dict(net=n,loads=sum(v.values()),pins=dict(v)) for n,v in loads.items()),
                  key=lambda r:(-r['loads'],r['net']))
    counts = Counter(c['kind'] for c in cells.values())
    return dict(primitives=dict(counts),bsram=sum(counts[t] for t in BSRAM),
                ff=sum(v for k,v in counts.items() if k.startswith('DFF')),
                maximum_nonclock_load=rows[0]['loads'],high_loads=rows[:16])


def namespaced(text, suffix):
    for name,_ in MODULE.findall(text):
        renamed = 'g2_'+suffix if name == 'g2_top' else name+'_'+suffix
        text = re.sub(r'\b'+re.escape(name)+r'\b', renamed, text)
    return text


def simulate(out, variant, files):
    folder=out/variant;folder.mkdir()
    (folder/'tb.sv').write_text(TB,encoding='utf-8')
    def run(tool,args):
        with (folder/(tool+'_stdout.log')).open('w') as stream:
            subprocess.run(['cmd','/c',str(VIV/(tool+'.bat')),*args],cwd=folder,
                           stdout=stream,stderr=subprocess.STDOUT,timeout=120,check=True)
    run('xvlog',[str(f) for f in files]);run('xvlog',['-sv','tb.sv'])
    run('xelab',['-debug','off','-timescale','1ns/1ps','tb','-s','snap'])
    run('xsim',['snap','-runall'])
    log=(folder/'xsim.log').read_text(errors='replace')
    assert 'TEST PASSED' in log and 'Fatal:' not in log,log[-2000:]
    return dict(variant=variant,result=re.search(r'TEST PASSED[^\n]*',log)[0])


def main():
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();source=a.source.resolve();out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    old=(source/'nav/rtl/common/nav_ram_sdp.v').read_text(encoding='utf-8');new=transform(old)
    (out/'old.v').write_text(old.replace('module nav_ram_sdp #(','module ram_old #(',1),encoding='utf-8')
    (out/'new.v').write_text(new.replace('module nav_ram_sdp #(','module ram_new #(',1),encoding='utf-8')
    for n in ('old','new'):
        (out/(n+'_top.v')).write_text(WRAPPER.replace('g2_top','g2_'+n).replace('nav_ram_sdp','ram_'+n),encoding='utf-8')
    result=dict(source=str(source),build=str(out),geometry=dict(DW=98,DEPTH=160,banks=10,bank_depth=16),
                logical_stored_bits=15680,latency_cycles=1,programmed=False,deployment_accepted=False,
                scope='isolated row-store connectivity and functional simulation; no physical congestion diagnosis',runs=[],checks=[])
    def save(): (out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8',newline='\n')
    result['checks'].append(simulate(out,'rtl',[out/f for f in ('old.v','new.v','old_top.v','new_top.v')]));save()
    for n,text in (('original',old),('banked',new)):
        folder=out/n;folder.mkdir();(folder/'ram.v').write_text(text,encoding='utf-8');(folder/'top.v').write_text(WRAPPER,encoding='utf-8')
        (folder/'clock.sdc').write_text('create_clock -name clk -period 20 [get_ports {clk}]\n',encoding='utf-8')
        settings=['set_device -device_version C GW5AT-LV138PG484AC1/I0']
        settings += [f'add_file -type verilog {{{(folder/f).as_posix()}}}' for f in ('ram.v','top.v')]
        settings += [f'add_file -type sdc {{{(folder/"clock.sdc").as_posix()}}}','set_option -top_module g2_top',
                     'set_option -output_base_name g2_top','set_option -global_freq 50','set_option -convert_sdp32_36_to_sdp16_18 0','set_option -looplimit 100000']
        effective=validate(TOOL,folder,settings,50,0);(folder/'run.tcl').write_text('\n'.join(settings+['run syn','exit'])+'\n',encoding='utf-8')
        runtime=run_syn(folder);syn=folder/'impl/gwsynthesis'
        row=dict(variant=n,runtime=runtime,effective_options=effective,resources=ET.parse(syn/'g2_top_syn_rsc.xml').getroot().attrib,
                 inventory=inventory(syn/'g2_top.vg'))
        result['runs'].append(row);save();print(json.dumps(dict(variant=n,inventory=row['inventory'])),flush=True)
        (out/(n+'_vendor.v')).write_text(namespaced((syn/'g2_top.vg').read_text(encoding='utf-8'),'old' if n=='original' else 'new'),encoding='utf-8')
    result['checks'].append(simulate(out,'vendor',[LIBRARY,out/'original_vendor.v',out/'banked_vendor.v']))
    before,after=[r['inventory'] for r in result['runs']]
    result['bsram_unchanged']=before['bsram']==after['bsram']==0
    result['connection_pressure_reduced']=after['maximum_nonclock_load']<before['maximum_nonclock_load']
    save();print(json.dumps({k:v for k,v in result.items() if k not in ('runs',)}),flush=True)
    return not(result['bsram_unchanged'] and result['connection_pressure_reduced'])


if __name__=='__main__':raise SystemExit(main())
