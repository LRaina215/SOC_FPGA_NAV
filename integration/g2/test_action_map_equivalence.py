"""Exercise every clipped action value and unchanged actuator timing/priority."""
import argparse,importlib.util,json,re,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    spec=importlib.util.spec_from_file_location('actiongen',ROOT/'control/tools/g2_action_map_widths.py')
    gen=importlib.util.module_from_spec(spec);spec.loader.exec_module(gen);gen.generate(out/'action_candidate.v')
    base=ROOT/'integration/control4500/original_src/policy_action_map.v'
    (out/'action_original.v').write_text(base.read_text(encoding='utf-8').replace('module policy_action_map #(','module action_map_original #(',1))
    viv=Path('E:/2026.1/Vivado/bin')
    def run(name,args):
        with (out/(name+'_stdout.log')).open('w') as log:
            subprocess.run(['cmd','/c',str(viv/(name+'.bat')),*args],cwd=out,stdout=log,stderr=subprocess.STDOUT,check=True)
    run('xvlog',[str(out/'action_original.v'),str(out/'action_candidate.v')])
    run('xvlog',['-sv',str(ROOT/'integration/g2/tb_action_map_equivalence.sv')])
    run('xelab',['-debug','off','-timescale','1ns/1ps','tb_action_map_equivalence','-s','snap'])
    run('xsim',['snap','-runall']);s=(out/'xsim.log').read_text();assert 'TEST PASSED' in s and 'Fatal:' not in s
    m=re.search(r'frames=(\d+) clip_domain_values=(\d+) invalid_cycles=(\d+) stops=(\d+) resets=(\d+) parameter_sets=(\d+)',s)
    r=dict(pass_=True,build=str(out),candidate_source=str(out/'action_candidate.v'),original_source=str(base),
        frames=int(m[1]),clip_domain_values_per_gain=int(m[2]),invalid_cycles=int(m[3]),run_stop_cycles=int(m[4]),reset_scenarios=int(m[5]),parameter_sets=int(m[6]),
        outputs_last_actions_and_validity_cycle_exact=True,scope='finite RTL equivalence plus full clip-domain enumeration; not joint PNR or vehicle acceptance')
    (out/'result.json').write_text(json.dumps(r,indent=2));print(json.dumps(r))


if __name__=='__main__':main()
