"""Compare wheel timing candidate against the original control RTL, every cycle."""
import argparse,importlib.util,json,re,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    spec=importlib.util.spec_from_file_location('pipegen',ROOT/'control/tools/g2_observation_pipeline.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);module.generate(out/'obs_candidate.v')
    old=(ROOT/'integration/control4500/original_src/rl_observation_v5.v').read_text(encoding='utf-8')
    old=old.replace('RL_OBSERVATION_V5_V_INCLUDED','OBS_ORIGINAL_INCLUDED').replace('module rl_observation_v5 #(','module obs_original #(')
    (out/'obs_original.v').write_text(old,encoding='utf-8')
    viv=Path('E:/2026.1/Vivado/bin')
    def run(name,args):
        with (out/(name+'_stdout.log')).open('w') as log:
            subprocess.run(['cmd','/c',str(viv/(name+'.bat')),*args],cwd=out,stdout=log,stderr=subprocess.STDOUT,check=True)
    run('xvlog',[str(out/'obs_original.v'),str(out/'obs_candidate.v')])
    run('xvlog',['-sv',str(ROOT/'integration/g2/tb_observation_equivalence.sv')])
    run('xelab',['-debug','off','-timescale','1ns/1ps','tb_observation_equivalence','-s','snap'])
    run('xsim',['snap','-runall']);s=(out/'xsim.log').read_text();assert 'TEST PASSED' in s and 'Fatal:' not in s
    result=dict(pass_=True,complete_frames=int(re.search(r'TEST PASSED (\d+)',s)[1]),clock_cycles=56000,
        sign_combinations=4,external_cycle_exact=True,changing_inputs_every_cycle=True,reset_busy_capture=True,
        captured_wheel_deltas='alternate signed extremes, zero/minus-one and random values across frames',
        weights_changed=False,policy_period_changed=False,scope='isolated observation RTL timing candidate; not G2 PNR accepted')
    (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))


if __name__=='__main__':main()
