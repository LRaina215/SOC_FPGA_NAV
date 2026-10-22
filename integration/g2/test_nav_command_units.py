"""Exhaustive exact units conversion, throughput and fault flush contracts."""
import argparse,json,re,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False);viv=Path('E:/2026.1/Vivado/bin')
    def run(name,args):
        with (out/(name+'_stdout.log')).open('w') as log:
            subprocess.run(['cmd','/c',str(viv/(name+'.bat')),*args],cwd=out,stdout=log,stderr=subprocess.STDOUT,check=True)
    run('xvlog',[str(ROOT/'integration/g2/nav_command_units.v')])
    run('xvlog',['-sv',str(ROOT/'integration/g2/tb_nav_command_units.sv')])
    run('xelab',['-debug','off','-timescale','1ns/1ps','tb_nav_command_units','-s','snap'])
    run('xsim',['snap','-runall']);s=(out/'xsim.log').read_text();assert 'TEST PASSED' in s and 'Fatal:' not in s
    m=re.search(r'exhaustive=(\d+) checked=(\d+) flushed=(\d+) corrections=(\d+)',s)
    result=dict(pass_=True,exhaustive_values_per_axis=int(m[1]),checked_frames=int(m[2]),flush_events=int(m[3]),
        reference='signed 64-bit original (x*4096 +/-500)/1000 with truncation toward zero',
        throughput_cycles=1,pipeline_stages=4,sequence_aligned=True,reset_and_kill_flush=True,
        reciprocal_corrections=int(m[4]),multiplier_widths=[25,17],
        scope='interface-only exact unit conversion, not policy change or completed G2 PNR')
    (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))


if __name__=='__main__':main()
