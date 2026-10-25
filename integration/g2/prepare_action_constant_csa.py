"""Exact constant shift/CSA action map, isolated from width-reduced candidate."""
from pathlib import Path
from datetime import datetime
import difflib,json,re,sys,subprocess,time,argparse
import psutil
sys.path.insert(0,'F:/nav_integration_g2_20261023/routing_proof_dependencies')
import z3

OUT=None
ORIGIN=Path('F:/nav_integration_g2_20261023/baseline_fusion_only_syn23/control/policy_action_map.v')
ROOT=Path(__file__).resolve().parents[2]
VIV=Path('E:/2026.1/Vivado/bin')
SCALE='''    function signed [31:0] scale;
        input signed [23:0] a;
        input integer gain;
        reg signed [63:0] product,bounded64;
        reg [63:0] p0,p1,p2,p3,save0,carry0,save1,carry1;
        begin
            // Keep the original signed64 product and exact rounding domain.
            // Only these constant call sites use a carry-save shift/add tree;
            // every other integer gain retains the original multiplication.
            bounded64=$signed(clip(a));
            if(gain==600 || gain==-600) begin
                // 600 = 512 + 64 + 16 + 8. Two CSA levels, one final adder.
                p0=bounded64<<<9;p1=bounded64<<<6;
                p2=bounded64<<<4;p3=bounded64<<<3;
                save0=p0^p1^p2;
                carry0=((p0&p1)|(p0&p2)|(p1&p2))<<1;
                save1=save0^carry0^p3;
                carry1=((save0&carry0)|(save0&p3)|(carry0&p3))<<1;
                product=$signed(save1+carry1);
                if(gain<0) product=-product;
            end else if(gain==19 || gain==-19) begin
                // 19 = 16 + 2 + 1. One CSA level, one final adder.
                p0=bounded64<<<4;p1=bounded64<<<1;p2=bounded64;
                save0=p0^p1^p2;
                carry0=((p0&p1)|(p0&p2)|(p1&p2))<<1;
                product=$signed(save0+carry0);
                if(gain<0) product=-product;
            end else begin
                product=$signed(clip(a))*gain;
            end
            scale=product<0 ? -((-product+2048)>>>12) : ((product+2048)>>>12);
        end
    endfunction'''

def meta(path):
    s=path.stat();return dict(path=str(path),bytes=s.st_size,mtime_ns=s.st_mtime_ns)

def save(path,data):path.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')

def csa(a,b,c):return a^b^c,((a&b)|(a&c)|(b&c))<<1

def product(x,gain):
    if abs(gain)==600:
        s,c=csa(x<<9,x<<6,x<<4);s,c=csa(s,c,x<<3);p=s+c
    elif abs(gain)==19:
        s,c=csa(x<<4,x<<1,x);p=s+c
    else:p=x*gain
    return -p if gain in(-600,-19) else p

def rounded(p):return -( ((-p)+2048)//4096) if p<0 else (p+2048)//4096

def main():
    global OUT,ORIGIN,VIV
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--origin',type=Path,default=ORIGIN)
    parser.add_argument('--vivado-bin',type=Path,default=VIV)
    args=parser.parse_args()
    OUT=args.out.resolve();ORIGIN=args.origin.resolve();VIV=args.vivado_bin.resolve()
    assert not OUT.exists(),'preserve existing candidate/proofs; choose a new directory'
    OUT.mkdir(parents=True)
    me=psutil.Process();me.nice(psutil.IDLE_PRIORITY_CLASS);me.cpu_affinity([16])
    text=ORIGIN.read_text(encoding='utf-8')
    pattern=re.compile(r'    function signed \[31:0\] scale;.*?    endfunction',re.S)
    assert len(pattern.findall(text))==1
    candidate=pattern.sub(lambda m:SCALE,text)
    assert pattern.sub('SCALE',candidate)==pattern.sub('SCALE',text)
    assert re.findall(r'scale\(actions\[[^\]]+\],(-?\d+)\)',candidate)==['-600','600','19','-19','19','19']
    (OUT/'policy_action_map.v').write_text(candidate,encoding='utf-8',newline='\n')
    (OUT/'action_original.v').write_text(text.replace('module policy_action_map #(','module action_map_original #(',1),encoding='utf-8',newline='\n')
    (OUT/'constant_csa.patch').write_text(''.join(difflib.unified_diff(text.splitlines(True),candidate.splitlines(True),fromfile=str(ORIGIN),tofile='policy_action_map.v')),encoding='utf-8')
    checks=[]
    # Decompose the64bit CSA theorem into its one-bit full-adder identity.
    # Weight each identity by2^i and sum all64 bits; discarding the top carry
    # gives exactly the RTL modulo2^64 result. This avoids an unnecessary
    # monolithic64bit multiply SAT instance (an initial attempt timed out,
    # without a counterexample).
    b0,b1,b2=z3.Bools('full_adder_a full_adder_b full_adder_c')
    asint=lambda b:z3.If(b,1,0)
    parity=z3.Xor(b0,z3.Xor(b1,b2))
    majority=z3.Or(z3.And(b0,b1),z3.And(b0,b2),z3.And(b1,b2))
    solver=z3.Solver();solver.add(asint(b0)+asint(b1)+asint(b2)!=asint(parity)+2*asint(majority))
    assert solver.check()==z3.unsat
    x=z3.Int('unbounded_signed_multiplicand')
    for g in(-600,600,-19,19):
        p=x*512+x*64+x*16+x*8 if abs(g)==600 else x*16+x*2+x
        if g<0:p=-p
        s=z3.Solver();s.set(timeout=1000)
        s.add(p!=x*g);t=time.perf_counter();r=s.check()
        assert r==z3.unsat,(g,str(r),str(s.model()) if r==z3.sat else s.reason_unknown())
        checks.append(dict(gain=g,integer_shift_factor_identity=str(r),CSA_full_adder_bit_identity='unsat',
                           all64bit_conclusion='Weight the proven bit identities by2^i and sum modulo2^64; apply the proven integer constant factor identity.',elapsed_s=time.perf_counter()-t))
    exhaustive=0
    for a in range(-4096,4097):
        for g in(-600,600,-19,19):
            assert product(a,g)==a*g and rounded(product(a,g))==rounded(a*g)
            exhaustive+=1
    # The fallback is literally the old expression, including signed32 gain.
    fallback_gains=[-2147483648,-601,-20,-1,0,1,20,601,2147483647]
    for a in(-8388608,-4097,-4096,-1,0,1,4096,4097,8388607):
        bounded=min(4096,max(-4096,a))
        for g in fallback_gains:assert product(bounded,g)==bounded*g
    runs=[]
    for name,args in [('xvlog',[str(OUT/'action_original.v'),str(OUT/'policy_action_map.v')]),
                      ('xvlog',['-sv',str(ROOT/'integration/g2/tb_action_map_equivalence.sv')]),
                      ('xelab',['-debug','off','-timescale','1ns/1ps','tb_action_map_equivalence','-s','snap']),
                      ('xsim',['snap','-runall'])]:
        logfile=OUT/(name+'_'+str(len(runs))+'_stdout.log')
        with logfile.open('w',encoding='utf-8') as log:
            p=subprocess.run(['cmd','/c',str(VIV/(name+'.bat')),*args],cwd=OUT,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
        runs.append(dict(tool=name,exit_code=p.returncode,log=str(logfile)))
        assert p.returncode==0,(name,str(logfile))
    log=(OUT/'xsim.log').read_text(encoding='utf-8',errors='replace')
    assert 'TEST PASSED' in log and 'Fatal:' not in log
    m=re.search(r'frames=(\d+) clip_domain_values=(\d+) invalid_cycles=(\d+) stops=(\d+) resets=(\d+) parameter_sets=(\d+)',log)
    assert m and [int(v) for v in m.groups()]==[8770,8193,64,64,1,2]
    save(OUT/'result.json',dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),
        original=str(ORIGIN),candidate=str(OUT/'policy_action_map.v'),source_bindings=[meta(ORIGIN),meta(OUT/'policy_action_map.v'),meta(ROOT/'integration/g2/tb_action_map_equivalence.sv')],
        only_scale_function_changed=True,from_failed_width_candidate=False,
        original64bit_product_and_rounding_preserved=True,all_other_integer_gains_literal_original_fallback=True,
        formal_product_checks=checks,clip_domain_values_per_gain=8193,exhaustive_scale_values_checked=exhaustive,
        gains=[-600,600,-19,19],fallback_directed_gains=fallback_gains,
        frames=8770,invalid_cycles=64,run_stop_cycles=64,reset_scenarios=1,parameter_sets=2,
        outputs_last_actions_and_validity_cycle_exact=True,unchanged_parameters_ports_valid_reset_stop=True,
        runs=runs,execution_policy=dict(priority=int(me.nice()),affinity=me.cpu_affinity()),
        mathematical_proof_scope='CSA local full-adder theorem plus weighted64bit modular sum and four unbounded integer constant-factor identities; identical clip and rounding then imply the same scale function. Zero_us/ff/pulse/output state logic is unchanged. Module simulation is finite8770frames, not all state/input sequences.',
        initial_monolithic_BV64_solver_attempt='10second timeout/unknown, not a counterexample; replaced by explicit compositional algebraic proof, no unsupported claim of monolithic formal equivalence',
        physical_timing_or_resource_passed=False,joint_synthesis_or_PNR_started=False))
    print(json.dumps(dict(passed=True,result=str(OUT/'result.json'),frames=8770,exhaustive_scale_values_checked=exhaustive)))

if __name__=='__main__':main()
