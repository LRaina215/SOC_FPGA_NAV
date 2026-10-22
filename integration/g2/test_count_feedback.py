"""Independent integer/fraction and sampled-count oracle for isolated observer."""
import argparse,json,re,subprocess
from collections import deque
from fractions import Fraction
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    frames=[];phases=[]
    def add(name,items):
        start=len(frames);frames.extend(items);phases.append(dict(name=name,start=start,count=len(frames)-start))
    add('stationary',[(0,0,1)]*60)
    for name,numerator,sign in [('nominal_4mmps',400,1),('nominal_6mmps',600,1),('nominal_minus6mmps',600,-1)]:
        phase=0;items=[]
        for _ in range(200):
            phase+=numerator;d=phase//1547;phase%=1547;items.append((d*sign,-d*sign,1))
        add(name,items)
    add('direction_reversal_travel',[(1,-1,1),(-1,1,1)]*60)
    add('fraction_then_zero',[(1,0,1)]+[(0,0,1)]*60)
    add('half_unit_epoch_reset',[(0,0,0)])
    add('half_unit_then_zero',[(100,0,1)]+[(0,0,1)]*60)
    rng=0x450088;items=[]
    for _ in range(256):
        rng=(1664525*rng+1013904223)&0xffffffff;left=(rng%161)-80
        rng=(1664525*rng+1013904223)&0xffffffff;right=(rng%161)-80
        items.append((left,right,1))
    add('random_signed_counts',items)
    add('quality_gap',[(0,0,0)]+[(0,0,1)]*60)
    add('overflow_boundaries',[(2147483647,-2147483648,1),(-2147483648,2147483647,1),(-2147483648,-2147483648,1),(2147483647,2147483647,1)]+[(0,0,1)]*60)
    residual=0;window=deque(maxlen=50);records=[];outputs=[];overflow=0
    for left,right,quality in frames:
        n=(left-right)*1547+residual;q=(abs(n)//200)*(-1 if n<0 else 1)
        if not quality or abs(q)>32767:
            residual=0;window.clear();valid=0;v=wv=slow=travel=0
            if quality:overflow+=1
        else:
            residual=n-q*200;valid=1;v=q
            window.append(min(abs(left)+abs(right),31));travel=sum(window);wv=int(len(window)==50)
            # Nominal wheel travel plus two endpoint count uncertainties;
            # compare distance to 5mm/s * 0.5s independently of RTL threshold30.
            slow=int(wv and Fraction((travel+2)*1547,20000)<=Fraction(5,2))
        records.append(f'{left} {right} {quality} {valid} {v} {wv} {slow} {travel}')
        outputs.append((valid,v,wv,slow,travel))
    for phase in phases:
        tail=outputs[phase['start']+50:phase['start']+phase['count']]
        if phase['name']=='nominal_4mmps':assert all(x[3] for x in tail)
        if phase['name'] in ('nominal_6mmps','nominal_minus6mmps','direction_reversal_travel'):assert all(not x[3] for x in tail)
        if phase['name'] in ('fraction_then_zero','half_unit_then_zero'):assert all(x[1]==0 for x in tail)
    (out/'frames.txt').write_text('\n'.join(records)+'\n')
    viv=Path('E:/2026.1/Vivado/bin')
    def run(name,args):
        with (out/(name+'_stdout.log')).open('w') as log:
            subprocess.run(['cmd','/c',str(viv/(name+'.bat')),*args],cwd=out,stdout=log,stderr=subprocess.STDOUT,check=True)
    # Freeze measured sources beside their logs; no hashing or live source mutation.
    for name in ('nav_count_feedback.v','tb_count_feedback.sv'):(out/name).write_bytes((ROOT/'integration/g2'/name).read_bytes())
    run('xvlog',[str(out/'nav_count_feedback.v')]);run('xvlog',['-sv',str(out/'tb_count_feedback.sv')])
    run('xelab',['-debug','off','-timescale','1ns/1ps','tb_count_feedback','-s','snap'])
    run('xsim',['snap','-runall']);s=(out/'xsim.log').read_text();assert 'TEST PASSED' in s and 'Fatal:' not in s
    m=re.search(r'frames=(\d+) emissions=(\d+) explicit_fault_cases=(\d+)',s)
    r=dict(pass_=True,build=str(out),frames=int(m[1]),emissions=int(m[2]),explicit_fault_cases=int(m[3]),overflow_frames=overflow,
        phases=phases,source_clock_parameter_hz=100000,gate_cycles=1000,nominal_gate_ms=10,
        production_default_clock_hz=50000000,production_gate_cycles=500000,
        stop_signal_role='supplementary sampled-count evidence only; within-gate reversals, sub-count motion and slip remain unobserved',
        scope='isolated RTL and independent nominal count-window arithmetic oracle; not integrated G2, original4500 behavior, MuJoCo or physical stop acceptance')
    (out/'result.json').write_text(json.dumps(r,indent=2));print(json.dumps(r))


if __name__=='__main__':main()
