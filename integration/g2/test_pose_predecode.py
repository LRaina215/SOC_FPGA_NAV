"""Compare the new pulse to the untouched register-file pulse inside real NAV."""
import argparse,json,re,shutil,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'sim'))
from tests import ALL_RTL
from prepare_pose_predecode import generate


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    generate(out/'nav_candidate.v');shutil.copyfile(ROOT/'rtl/plan/sin_lut.hex',out/'sin_lut.hex')
    viv=Path('E:/2026.1/Vivado/bin')
    def run(name,args):
        with (out/(name+'_stdout.log')).open('w') as log:
            subprocess.run(['cmd','/c',str(viv/(name+'.bat')),*args],cwd=out,stdout=log,stderr=subprocess.STDOUT,check=True)
    sources=[str(ROOT/p) for p in ALL_RTL if p!='rtl/top/nav_subsystem.v']
    run('xvlog',['-i',str(ROOT/'rtl/regs'),*sources,str(out/'nav_candidate.v')])
    run('xvlog',['-sv',str(ROOT/'integration/g2/tb_pose_predecode.sv')])
    run('xelab',['-debug','off','-timescale','1ns/1ps','tb_pose_predecode','-s','snap'])
    run('xsim',['snap','-runall']);s=(out/'xsim.log').read_text();assert 'TEST PASSED' in s and 'Fatal:' not in s
    m=re.search(r'comparisons=(\d+) pulses=(\d+) transactions=(\d+) resets=(\d+)',s)
    r=dict(pass_=True,cycle_comparisons=int(m[1]),pose_pulses=int(m[2]),bus_transactions=int(m[3]),reset_scenarios=int(m[4]),
           reference='unmodified nav_regfile POSE_COMMIT output in actual nav_subsystem bus FSM',
           scope='finite pulse/cycle equivalence; no full-system or physical timing acceptance')
    (out/'result.json').write_text(json.dumps(r,indent=2));print(json.dumps(r))


if __name__=='__main__':main()
