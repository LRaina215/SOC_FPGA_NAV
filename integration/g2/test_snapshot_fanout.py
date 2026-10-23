"""Check cycle-exact snapshot transfer including recovery and invalid input."""
import argparse,importlib.util,json,re,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    original=(ROOT/'control/fpga/src/rl_policy_tick_v5.v').read_text(encoding='utf-8')
    (out/'original.v').write_text(original.replace('RL_POLICY_TICK_V5_V_INCLUDED','SNAPSHOT_ORIGINAL_INCLUDED').replace('module rl_policy_tick_v5 #(','module snapshot_original #('),encoding='utf-8')
    candidate=out/'candidate.v';candidate.write_text(original,encoding='utf-8')
    spec=importlib.util.spec_from_file_location('snapshot_generator',ROOT/'control/tools/g2_snapshot_fanout.py')
    generator=importlib.util.module_from_spec(spec);spec.loader.exec_module(generator);generator.generate(candidate)
    viv=Path('E:/2026.1/Vivado/bin')
    def run(name,args):
        with(out/(name+'_stdout.log')).open('w')as log:
            subprocess.run(['cmd','/c',str(viv/(name+'.bat')),*args],cwd=out,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
    run('xvlog',[str(out/'original.v'),str(candidate)])
    run('xvlog',['-sv',str(ROOT/'integration/g2/tb_snapshot_fanout.sv')])
    run('xelab',['-debug','typical','-timescale','1ns/1ps','tb_snapshot_fanout','-s','snap'])
    run('xsim',['snap','-runall'])
    log=(out/'xsim.log').read_text(errors='replace');match=re.search(r'SNAPSHOT_FANOUT_PASS cycles=(\d+) publications=(\d+) writes=(\d+) resets=(\d+)',log)
    assert match and 'Fatal:'not in log,log[-2000:]
    result=dict(passed=True,**dict(zip(('cycles','publications','writes','resets'),map(int,match.groups()))),
        cases=['changing all snapshot bits every cycle','all core input timing/data','sensor/enable interruption','core error/saturation/ID mismatch','deadline','reset while active'],
        scope='isolated policy scheduler/transfer equivalence with a core test double; no RL inference or physical routing acceptance',weights_changed=False,period_parameter_changed=False)
    (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))


if __name__=='__main__':main()
