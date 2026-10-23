"""Check all trig addresses, independent enables, initial zero and actual INITs.

One small SYN verifies native storage. No SDF, joint PNR or vehicle claim.
"""
import argparse,json,re,shutil,subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
from prepare_trig_block_rom import transform
from preflight import validate

ROOT=Path(__file__).resolve().parents[2]
WRAPPER='''module trig_pair(input clk,input[1:0]en,input[9:0]a0,a1,output[15:0]q0,q1);
nav_ram_sdp #(.DW(16),.AW(10),.INIT_FILE("sin_lut.hex")) u_sin(clk,1'b0,10'd0,16'd0,en[0],a0,q0);
nav_ram_sdp #(.DW(16),.AW(10),.INIT_FILE("sin_lut.hex")) u_cos(clk,1'b0,10'd0,16'd0,en[1],a1,q1);
endmodule
'''
TB='''`timescale 1ns/1ps
module tb;
reg clk=0;always #5 clk=~clk;
reg[1:0]en=0;reg[9:0]a0=0,a1=1023;wire[15:0]q0,q1,r0,r1;
trig_pair dut(clk,en,a0,a1,q0,q1);
trig_pair_old refdut(clk,en,a0,a1,r0,r1);
reg[15:0]gold[0:1023],g0=0,g1=0;
reg[31:0]rng=32'h2304500;integer reads=0,holds=0,checks=0;
always@(posedge clk)begin
 if(en[0])g0<=gold[a0];
 if(en[1])g1<=gold[a1];
end
always@(negedge clk)begin
 checks++;
 if({q0,q1,r0,r1}!=={g0,g1,g0,g1})
  $fatal(1,"INIT/latency/independent enable mismatch check=%0d got=%h/%h ref=%h/%h expected=%h/%h",checks,q0,q1,r0,r1,g0,g1);
end
task request(input[1:0]e,input[9:0]x,y);
 begin @(negedge clk);#1;en=e;a0=x;a1=y;reads+=e[0]+int'(e[1]);holds+=2-e[0]-int'(e[1]);end
endtask
initial begin
 $readmemh("sin_lut.hex",gold);
 // Before the first enabled read the original RTL output is exactly zero.
 for(integer k=0;k<12;k++)request(0,k,~k);
 for(integer k=0;k<1024;k++)begin
  request(3,k,1023-k);
  if(k%11==0)begin request(1,~k,k);request(2,k,~k);request(0,0,1023);end
 end
 for(integer k=0;k<512;k++)begin
  rng=rng^(rng<<13);rng=rng^(rng>>17);rng=rng^(rng<<5);
  request(rng[31:30],rng[9:0],rng[19:10]);
 end
 repeat(8)request(0,0,1023);
 $display("TEST PASSED reads=%0d holds=%0d checks=%0d",reads,holds,checks);$finish;
end
initial begin #100000;$fatal(1,"timeout");end
endmodule
'''


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--source',type=Path,required=True);ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--resume-vendor-init',action='store_true',help='reuse completed RTL/SYN only when all saved inputs exactly match; never repeat them')
    a=ap.parse_args();source=a.source.resolve();out=a.out.resolve()
    if not a.resume_vendor_init:out.mkdir(parents=True,exist_ok=False)
    old=(source/'nav/rtl/common/nav_ram_sdp.v').read_text(encoding='utf-8')
    assert 'g_init_block' not in old
    inputs={'old.v':old.replace('module nav_ram_sdp #(','module nav_ram_original #(',1),
        'new.v':transform(old),'sin_lut.hex':(source/'sin_lut.hex').read_text(encoding='utf-8'),
        'wrapper.v':WRAPPER,'old_wrapper.v':WRAPPER.replace('module trig_pair(','module trig_pair_old(',1).replace('nav_ram_sdp #(','nav_ram_original #('),'tb.sv':TB}
    for name,text in inputs.items():
        if a.resume_vendor_init:assert (out/name).read_text(encoding='utf-8')==text,('saved input differs',name)
        else:(out/name).write_text(text,encoding='utf-8',newline='\n')
    assert len((out/'sin_lut.hex').read_text().split())==1024
    viv=Path('E:/2026.1/Vivado/bin');rows=[]
    def check(name,files):
        run=out/name;run.mkdir()
        shutil.copyfile(out/'sin_lut.hex',run/'sin_lut.hex')
        def call(tool,args):
            with (run/(tool+'_stdout.log')).open('w')as log:
                subprocess.run(['cmd','/c',str(viv/(tool+'.bat')),*args],cwd=run,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
        call('xvlog',[str(x)for x in files]+[str(out/'old.v'),str(out/'old_wrapper.v')])
        call('xvlog',['-sv',str(out/'tb.sv')]);call('xelab',['-debug','off','-timescale','1ns/1ps','tb','-s','snap'])
        call('xsim',['snap','-runall']);log=(run/'xsim.log').read_text(errors='replace')
        match=re.search(r'TEST PASSED reads=(\d+) holds=(\d+) checks=(\d+)',log)
        assert match and 'Fatal:' not in log,log[-2500:]
        row=dict(phase=name,passed=True,reads=int(match[1]),holds=int(match[2]),checks=int(match[3]),all_addresses_per_port=1024,initial_zero_checked=True,independent_enable_checked=True)
        rows.append(row);print(json.dumps(row),flush=True)
    if not a.resume_vendor_init:check('rtl',[out/'new.v',out/'wrapper.v'])
    else:
        previous=(out/'rtl/xsim.log').read_text(errors='replace')
        match=re.search(r'TEST PASSED reads=(\d+) holds=(\d+) checks=(\d+)',previous)
        assert match and 'Fatal:' not in previous
        rows.append(dict(phase='rtl',passed=True,reads=int(match[1]),holds=int(match[2]),checks=int(match[3]),all_addresses_per_port=1024,initial_zero_checked=True,independent_enable_checked=True))
    tool=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe')
    clock='create_clock -name clk50 -period 20 [get_ports {clk}]\n'
    if a.resume_vendor_init:assert (out/'clock.sdc').read_text()==clock
    else:(out/'clock.sdc').write_text(clock,encoding='utf-8')
    settings=['set_device -device_version C GW5AT-LV138PG484AC1/I0',
        f'add_file -type verilog {{{(out/"new.v").as_posix()}}}',f'add_file -type verilog {{{(out/"wrapper.v").as_posix()}}}',
        f'add_file -type sdc {{{(out/"clock.sdc").as_posix()}}}',
        'set_option -top_module trig_pair','set_option -output_base_name trig_pair',
        'set_option -global_freq 50','set_option -convert_sdp32_36_to_sdp16_18 1']
    script='\n'.join(settings+['run syn','exit'])+'\n'
    if not a.resume_vendor_init:
        effective=validate(tool,out,settings,50,1)
        (out/'effective.json').write_text(json.dumps(effective,indent=2),encoding='utf-8')
        (out/'run.tcl').write_text(script,encoding='utf-8')
        with (out/'gw_sh.log').open('w')as log:
            subprocess.run([str(tool),'run.tcl'],cwd=out,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=180,creationflags=subprocess.CREATE_NO_WINDOW)
    else:
        assert (out/'run.tcl').read_text()==script
        effective=dict(global_freq_mhz=50,wide_split=1,effective_options=str(out/'effective_options.tcl'))
        actual=(out/'effective_options.tcl').read_text()
        assert re.search(r'^set_option -global_freq 50\.000\s*$',actual,re.M) and re.search(r'^set_option -convert_sdp32_36_to_sdp16_18 1\s*$',actual,re.M)
    log=(out/'gw_sh.log').read_text(errors='replace');assert not re.search(r'\bERROR\s*\(',log),log[-3000:]
    syn=out/'impl/gwsynthesis';resources=ET.parse(syn/'trig_pair_syn_rsc.xml').getroot().attrib
    assert int(re.match(r'\d+',resources['T_Bsram'])[0])==2,resources
    check('vendor_INIT',[Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v'),syn/'trig_pair.vg'])
    result=dict(source=str(source),build=str(out),checks=rows,synthesis_resources=resources,effective_options=effective,
        scope='two real initialized tables, RTL and actual vendor SYN primitives; no SDF, joint PNR timing or vehicle acceptance',programmed=False)
    (out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result))


if __name__=='__main__':main()
