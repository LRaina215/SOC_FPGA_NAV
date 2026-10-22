"""Old vs retimed full fusion, every cycle; accelerated stimuli, no board claim."""
import argparse,importlib.util,json,re,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--gyro-sum-exact-widths',action='store_true');a=ap.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    spec=importlib.util.spec_from_file_location('fusiongen',ROOT/'control/tools/g2_fusion_pipeline.py')
    gen=importlib.util.module_from_spec(spec);spec.loader.exec_module(gen);gen.generate(out/'fusion_candidate.v')
    if a.gyro_sum_exact_widths:
        spec=importlib.util.spec_from_file_location('gyro_widths',ROOT/'control/tools/g2_gyro_sum_widths.py')
        gen=importlib.util.module_from_spec(spec);spec.loader.exec_module(gen);gen.generate(out/'fusion_candidate.v')
    base=ROOT/'integration/control4500/original_src'
    s=(base/'imu_fusion.v').read_text(encoding='utf-8').replace('IMU_FUSION_V_INCLUDED','ORIGINAL_IMU_FUSION_V_INCLUDED')
    s=re.sub(r'\bimu_fusion\b','fusion_original',s);s=re.sub(r'\bimu_div100_3\b','original_divider3',s)
    (out/'fusion_original.v').write_text(s,encoding='utf-8')
    viv=Path('E:/2026.1/Vivado/bin')
    def run(name,args):
        with (out/(name+'_stdout.log')).open('w') as log:
            subprocess.run(['cmd','/c',str(viv/(name+'.bat')),*args],cwd=out,stdout=log,stderr=subprocess.STDOUT,check=True)
    run('xvlog',[str(base/'imu_isqrt.v'),str(base/'imu_recip.v'),str(out/'fusion_original.v'),str(out/'fusion_candidate.v')])
    tb=ROOT/'integration/g2/tb_fusion_equivalence.sv';extra=[]
    if a.gyro_sum_exact_widths:
        text=tb.read_text().replace('module tb_fusion_equivalence;',
            'module tb_fusion_equivalence;\n wire gyro_widths_done;gyro_sum_math widths(gyro_widths_done);')
        text=text.replace('if(cycle==200000)begin','if(cycle==200000)begin\n   if(!gyro_widths_done)$fatal(1,"gyro sum sweep incomplete");')
        tb=out/'tb_fusion_equivalence.sv';tb.write_text(text)
        extra=[str(ROOT/'integration/g2/tb_gyro_sum_widths.sv')]
    run('xvlog',['-sv',str(tb),*extra])
    run('xelab',['-debug','off','-timescale','1ns/1ps','tb_fusion_equivalence','-s','snap'])
    run('xsim',['snap','-runall']);s=(out/'xsim.log').read_text();assert 'TEST PASSED' in s and 'Fatal:' not in s
    m=re.search(r'frames50m=(\d+) frames100k=(\d+) accepted=(\d+) rejected=(\d+) math=(\d+) busy_reset=(\d+)',s)
    result=dict(pass_=True,build=str(out),candidate_source=str(out/'fusion_candidate.v'),original_source=str(base/'imu_fusion.v'),
        gyro_sum_exact_widths=a.gyro_sum_exact_widths,
        frames_50mhz_parameter=int(m[1]),frames_100khz_parameter=int(m[2]),accepted_accel=int(m[3]),rejected_accel=int(m[4]),
        norm_extreme_random_vectors=int(m[5]),busy_resets=int(m[6]),clock_cycles=200000,
        calibration_samples_in_test=4,production_calibration_and_sample_parameters_changed=False,
        external_outputs_state_start_sqrt_input_cycle_exact=True,
        scope='finite RTL numeric/cycle equivalence at two clock parameters and injected full signed24 norm vectors; not physical IMU calibration or vehicle acceptance')
    if a.gyro_sum_exact_widths:
        assert 'GYRO WIDTHS PASSED vectors=786432 max_sum=98304' in s
        result.update(gyro_sum_vectors=786432,gyro_sum_maximum=98304,gyro_sum_bits=17,
            gyro_sum_sweep_scope='each axis all65536 values with four other-axis contexts, plus range proof; not all2^48 triples')
    (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))


if __name__=='__main__':main()
