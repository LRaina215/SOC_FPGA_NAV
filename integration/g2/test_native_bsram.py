"""Actual vendor-model semantics for native32/36 versus split16/18 storage."""
import argparse,json,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False);viv=Path('E:/2026.1/Vivado/bin')
    def run(name,args):
        with (out/(name+'_stdout.log')).open('w') as log:
            subprocess.run(['cmd','/c',str(viv/(name+'.bat')),*args],cwd=out,stdout=log,stderr=subprocess.STDOUT,check=True)
    library=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
    run('xvlog',[str(library)])
    run('xvlog',['-sv',str(ROOT/'integration/g2/tb_native_bsram.sv')])
    run('xelab',['-debug','off','-timescale','1ns/1ps','tb_native_bsram','-s','snap'])
    run('xsim',['snap','-runall'])
    s=(out/'xsim.log').read_text();assert 'TEST PASSED' in s and 'Fatal:' not in s
    result=dict(pass_=True,vendor_model=str(library),widths=[32,36],cycles=8192,
        byte_enable=True,read_hold=True,independent_scoreboard=True,
        same_address_collision_excluded=True,scope='vendor functional models; not SDF timing or real board')
    (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))


if __name__=='__main__':main()
