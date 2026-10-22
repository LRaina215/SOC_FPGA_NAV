"""Exact original vs storage-only RTL checks; no vehicle or timing acceptance."""
import argparse
import json
from pathlib import Path
import random
import shutil
import subprocess
from optimize_storage import ROOT,prepare


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--nav-layout',choices=['banked','ssram','hybrid','planes','directed'],default='banked');a=ap.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    src=ROOT/'integration/control4500/original_src';shutil.copytree(src/'mem',out/'src/mem')
    numeric=prepare(out,directed=a.nav_layout=='directed')
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
    run('xvlog',['-sv',str(tb)],'xvlog_tb.log')
    run('xelab',['-debug','off','-timescale','1ns/1ps','tb_storage_equivalence','-s','snap'],'xelab.log')
    run('xsim',['snap','-runall'],'xsim.log')
    text=(out/'xsim.log').read_text(errors='replace');assert 'TEST PASSED' in text and 'Fatal:' not in text
    result=dict(numeric=numeric,nav_layout=a.nav_layout,frames=3,neuron_values=1275,cycle_exact=True,RAM_geometries=8 if a.nav_layout in ('hybrid','directed') else 5,RAM_cycles_per_geometry=196608,
        scope='RTL exactness and all numeric ROM values; not real control response or PNR timing')
    (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))


if __name__=='__main__':main()
