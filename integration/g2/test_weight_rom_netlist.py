"""Read every weight from an actual Gowin ROM netlist and primitive INITs."""
import argparse
import json
from pathlib import Path
import re
import shutil
import subprocess


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--build',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args()
    build=a.build.resolve();out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    source=build/'impl/gwsynthesis/weight_rom_timing.vg'
    shutil.copyfile(source,out/'weight_rom_timing.v')
    words=(build/'src/mem/weights.mem').read_text().split()
    assert len(words)==40704
    (out/'expected.mem').write_text('\n'.join(words+['0000']*(40960-len(words)))+'\n')
    (out/'tb.sv').write_text('''`timescale 1ns/1ps
module tb;
 reg clk=0;always #5 clk=~clk;
 reg en=0;reg[15:0]addr=0;wire signed[15:0]value;
 weight_rom_timing dut(.clk(clk),.en(en),.addr(addr),.value(value));
 reg[15:0]gold[0:40959],e0,e1,e2;
 reg en0=0,v0=0,v1=0,v2=0;
 integer reads=0,holds=0,checks=0;reg[31:0]rng=32'h45009;
 always@(posedge clk)begin
  en0<=en;
  if(en)begin e0<=gold[addr];v0<=1;end
  if(en0)begin e1<=e0;v1<=v0;end
  e2<=e1;v2<=v1;
 end
 always@(negedge clk)if(v2)begin
  checks++;
  if(value!==e2)$fatal(1,"INIT/latency/enable mismatch: got=%h expected=%h cycle=%0d",value,e2,checks);
 end
 task request(input bit active,input[15:0]index);
  begin @(negedge clk);#1;en=active;addr=index;if(active)reads++;else holds++;end
 endtask
 initial begin
  $readmemh("expected.mem",gold);
  repeat(24)@(negedge clk);
  for(integer k=0;k<40960;k++)begin
   request(1,k);
   if(k%13==0)request(0,~k);
  end
  for(integer k=0;k<4096;k++)begin
   rng=rng^(rng<<13);rng=rng^(rng>>17);rng=rng^(rng<<5);
   request(1,rng%40960);
   if(k%7==0)request(0,16'hffff);
  end
  repeat(8)request(0,16'hffff);
  $display("TEST PASSED reads=%0d holds=%0d checks=%0d",reads,holds,checks);$finish;
 end
 initial begin #1000000;$fatal(1,"timeout");end
endmodule
''')
    viv=Path('E:/2026.1/Vivado/bin')
    library=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
    def run(name,args):
        with (out/(name+'_stdout.log')).open('w') as log:
            subprocess.run(['cmd','/c',str(viv/(name+'.bat')),*args],cwd=out,stdout=log,stderr=subprocess.STDOUT,check=True)
    run('xvlog',[str(library),str(out/'weight_rom_timing.v')])
    run('xvlog',['-sv',str(out/'tb.sv')])
    run('xelab',['-debug','off','-timescale','1ns/1ps','tb','-s','snap'])
    run('xsim',['snap','-runall'])
    text=(out/'xsim.log').read_text(errors='replace')
    match=re.search(r'TEST PASSED reads=(\d+) holds=(\d+) checks=(\d+)',text)
    assert match and 'Fatal:' not in text,text[-3000:]
    result=dict(pass_=True,build=str(build),synthesized_netlist=str(source),vendor_model=str(library),
        all_addresses=40960,extra_random_reads=4096,reads=int(match[1]),holds=int(match[2]),
        checks=int(match[3]),same_enabled_read_latency=True,scope='Gowin synthesized primitive INIT and functional enable/latency; no SDF or board timing')
    (out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result))


if __name__=='__main__':main()
