"""Exact original vs storage-only RTL checks; no vehicle or timing acceptance."""
import argparse
import json
from pathlib import Path
import random
import shutil
import subprocess
from optimize_storage import ROOT,prepare


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--nav-layout',choices=['banked','ssram','hybrid','planes','directed'],default='banked')
    ap.add_argument('--packed-weight9',action='store_true');ap.add_argument('--control-only',action='store_true');a=ap.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    src=ROOT/'integration/control4500/original_src';shutil.copytree(src/'mem',out/'src/mem')
    numeric=prepare(out,directed=a.nav_layout=='directed',packed9=a.packed_weight9)
    (out/'fc_original.v').write_text((src/'fc_v4_top.v').read_text().replace('module fc_v4_top #(','module fc_original #(',1))
    (out/'nav_ram_original.v').write_text((ROOT/'rtl/common/nav_ram_sdp.v').read_text().replace('module nav_ram_sdp #(','module nav_ram_original #(',1))
    rng=random.Random(4500);vectors=[0]*168+[rng.randint(-8000,8000) for _ in range(168)]+[rng.randint(-409600,409600) for _ in range(168)]
    (out/'vectors.mem').write_text('\n'.join(f'{x&0xffffff:06x}' for x in vectors)+'\n')
    viv=Path('E:/2026.1/Vivado/bin')
    def run(name,args,log):
        with (out/log.replace('.log','_stdout.log')).open('w') as stream:
            subprocess.run(['cmd','/c',str(viv/(name+'.bat')),*args],cwd=out,stdout=stream,stderr=subprocess.STDOUT,check=True)
    files=[str(out/'fc_original.v'),str(out/'fc_v4_top.v'),str(out/'fixed4500_weight_rom.v'),str(src/'fc_v4_round_sat.v'),str(out/'nav_ram_original.v'),str(ROOT/f'integration/g2/nav_ram_sdp_{a.nav_layout}.v')]
    if a.nav_layout in ('hybrid','directed'):
        (out/'nav_ram_banked_original.v').write_text((ROOT/'rtl/common/nav_ram_banked.v').read_text().replace('module nav_ram_banked #(','module nav_ram_banked_original #(',1))
        files += [str(out/'nav_ram_banked_original.v'),str(ROOT/f'integration/g2/nav_ram_banked_{a.nav_layout}.v')]
    run('xvlog',files,'xvlog.log')
    tb=ROOT/'integration/g2/tb_storage_equivalence.sv'
    if a.nav_layout in ('hybrid','directed'):
        text=tb.read_text().replace(' reg ram_done=0;', ''' ram_pair #(.DW(1),.AW(16)) r5(clk,ram_we,ram_re,wa,ra,wd);
 banked_pair #(.DW(16),.DEPTH(19200)) r6(clk,ram_we,ram_re,wa,ra,wd);
 banked_pair #(.DW(36),.DEPTH(9600)) r7(clk,ram_we,ram_re,wa,ra,wd);
 reg ram_done=0;''')
        text+='''
module banked_pair #(parameter DW=16,DEPTH=19200,AW=$clog2(DEPTH))(
 input clk,we,re,input [15:0] wa,ra,input [63:0] wd);
 wire [DW-1:0] oldq,newq;wire [AW-1:0] w=wa%DEPTH,r=ra%DEPTH;
 nav_ram_banked_original #(.DW(DW),.DEPTH(DEPTH)) oldmem(clk,we,w,wd[DW-1:0],re,r,oldq);
 nav_ram_banked #(.DW(DW),.DEPTH(DEPTH)) newmem(clk,we,w,wd[DW-1:0],re,r,newq);
 always @(negedge clk)if($time>20 && oldq!==newq)$fatal(1,"banked mismatch DW=%0d",DW);
endmodule
'''
        tb=out/'tb_storage_equivalence.sv';tb.write_text(text)
    if a.control_only:
        text=tb.read_text();begin=text.index(' reg ram_we=0');end=text.index(' reg [23:0] vectors',begin)
        text=text[:begin]+' reg ram_done=1;\n'+text[end:]
        text=text.replace('5 RAM geometries x196608 cycles','no unchanged RAM tests')
        if a.packed_weight9:
            words=(src/'mem/weights.mem').read_text().split();words+=['0000']*(40960-len(words))
            (out/'weights_expected.mem').write_text('\n'.join(words)+'\n')
            text=text.replace(' reg ram_done=1;', ''' reg ram_done=0;
 reg scan_en=0;reg[15:0] scan_addr=0,scan_expected;wire signed[15:0] scan_value;
 reg[15:0] golden_weights[0:40959];reg scan_seen=0;integer scan_reads=0,scan_holds=0;
 fixed4500_weight_rom scan_rom(clk,scan_en,scan_addr,scan_value);
 always@(posedge clk)if(scan_en)begin scan_expected<=golden_weights[scan_addr];scan_seen<=1;end
 always@(negedge clk)if(scan_seen && scan_value!==scan_expected)$fatal(1,"weight9 word/hold mismatch addr=%0d",scan_addr);
 initial begin
  $readmemh("weights_expected.mem",golden_weights);
  repeat(5)@(negedge clk);
  for(integer k=0;k<40960;k++)begin
   #1;scan_en=1;scan_addr=k;scan_reads++;
   @(negedge clk);
   if(k%13==0)begin #1;scan_en=0;scan_addr=~k;scan_holds++;@(negedge clk);end
  end
  #1;scan_en=0;repeat(3)@(negedge clk);ram_done=1;
  $display("WEIGHT9 SCAN reads=%0d holds=%0d",scan_reads,scan_holds);
 end''')
        tb=out/'tb_storage_equivalence.sv';tb.write_text(text)
    run('xvlog',['-sv',str(tb)],'xvlog_tb.log')
    run('xelab',['-debug','off','-timescale','1ns/1ps','tb_storage_equivalence','-s','snap'],'xelab.log')
    run('xsim',['snap','-runall'],'xsim.log')
    text=(out/'xsim.log').read_text(errors='replace');assert 'TEST PASSED' in text and 'Fatal:' not in text
    if a.packed_weight9 and a.control_only:assert 'WEIGHT9 SCAN reads=40960' in text
    result=dict(numeric=numeric,nav_layout=a.nav_layout,frames=3,neuron_values=1275,cycle_exact=True,
        weight9_word_reads=40960 if a.packed_weight9 and a.control_only else None,
        RAM_geometries=0 if a.control_only else 8 if a.nav_layout in ('hybrid','directed') else 5,RAM_cycles_per_geometry=0 if a.control_only else 196608,
        scope='RTL exactness and all numeric ROM values; not real control response or PNR timing')
    (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))


if __name__=='__main__':main()
