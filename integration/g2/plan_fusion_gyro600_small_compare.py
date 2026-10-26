"""Source-bound fallback plan for SYN45 gyro trust; Python strings only.

No RTL is materialized and no EDA/simulator is started. Known48-bit equivalence
and all known/unknown-bit-mask branch acceptance are lightweight Z3 proofs.
The state6 live-input sampling edge, state order and fallback parameter branch
remain exact original source. This plan has no mapped timing acceptance.
"""
import argparse
from collections import Counter
from datetime import datetime
import json
from pathlib import Path
import re

from prepare_baseline_fusion_retime import metadata
from analyze_syn43_sensor_age_snapshot_path import first_path
from synthesis_connectivity import CELL, PORT
from prove_soc_console_remap import z3

BASE = Path('F:/nav_integration_g2_20261023')
BUILD = BASE/'SLF128_measured_timing_syn45'
ANCHOR = '''    function [23:0] abs24;'''
OLD_TEST = '''&& (abs16(gyro_x) + abs16(gyro_y) + abs16(gyro_z))
                       < GYRO_MOVE_LIMIT'''
NEW_TEST = '''&& gyro_move_sum_below_limit'''

# This is a Python string, deliberately never written to a .v file here.
SMALL_COMPARE = '''    // Combinational only: consumed at the ORIGINAL state6 edge.
    // abs(raw)<1024 is necessary for sum(abs(raw))<600. The -1024 encoding
    // is explicitly rejected before the low10 two's-complement wrap to0.
    wire gyro_move_sum_below_limit;
    generate if (GYRO_MOVE_LIMIT == 600) begin:g_gyro600_small_compare
        wire [9:0] ax = gyro_x[15] ? (~gyro_x[9:0]+10'd1) : gyro_x[9:0];
        wire [9:0] ay = gyro_y[15] ? (~gyro_y[9:0]+10'd1) : gyro_y[9:0];
        wire [9:0] az = gyro_z[15] ? (~gyro_z[9:0]+10'd1) : gyro_z[9:0];
        wire x_small = gyro_x[15] ? ((&gyro_x[15:10]) && (|gyro_x[9:0])) : !(|gyro_x[15:10]);
        wire y_small = gyro_y[15] ? ((&gyro_y[15:10]) && (|gyro_y[9:0])) : !(|gyro_y[15:10]);
        wire z_small = gyro_z[15] ? ((&gyro_z[15:10]) && (|gyro_z[9:0])) : !(|gyro_z[15:10]);
        wire [11:0] a = {2'b00,ax}, b = {2'b00,ay}, c = {2'b00,az};
        wire [11:0] csa_sum = a ^ b ^ c;
        wire [11:0] csa_carry = ((a & b) | (a & c) | (b & c)) << 1;
        wire [11:0] exact_sum = csa_sum + csa_carry;
        assign gyro_move_sum_below_limit = x_small && y_small && z_small && (exact_sum < 12'd600);
    end else begin:g_original_gyro_move_compare
        assign gyro_move_sum_below_limit = (abs16(gyro_x) + abs16(gyro_y) + abs16(gyro_z)) < GYRO_MOVE_LIMIT;
    end endgenerate

'''


def save(p, v): p.write_text(json.dumps(v, ensure_ascii=False, indent=2)+'\n',encoding='utf8')


def prove(label, mismatch, timeout_ms=30000):
    solver=z3.Solver(); solver.set(timeout=timeout_ms); solver.add(z3.simplify(mismatch)); result=solver.check()
    row=dict(label=label,result=str(result).upper())
    if result!=z3.unsat:
        row['counterexample_or_unknown']=str(solver.model()) if result==z3.sat else solver.reason_unknown()
    assert result==z3.unsat,row
    return row


def known_math():
    x,y,z=z3.BitVecs('gyro_x gyro_y gyro_z',16)
    def old_abs(v):
        w=z3.SignExt(16,v)
        return z3.If(z3.Extract(15,15,v)==1,-w,w)
    def small(v):
        low=z3.Extract(9,0,v); high=z3.Extract(15,10,v); neg=z3.Extract(15,15,v)==1
        mag=z3.If(neg,-low,low)
        gate=z3.If(neg,z3.And(high==63,low!=0),high==0)
        return z3.ZeroExt(2,mag),gate
    values=[small(v)for v in(x,y,z)]; a,b,c=[v for v,_ in values]
    sum12=(a^b^c)+(((a&b)|(a&c)|(b&c))<<1)
    proposed=z3.And(*[g for _,g in values],z3.ULT(sum12,600))
    reference=z3.ULT(old_abs(x)+old_abs(y)+old_abs(z),600)
    q=[prove('all2^48 known signed raw triples: original32bit abs sum<600 == guarded low10 CSA12 sum<600',reference!=proposed)]
    q.append(prove('accepted high guard means each low10 magnitude equals complete original abs32',
        z3.Or(*[z3.And(g,z3.ZeroExt(20,v)!=old_abs(raw))for raw,(v,g)in zip((x,y,z),values)])))
    q.append(prove('unsigned12bit carry-save result equals full sum of three10bit magnitudes without truncation',
        z3.ZeroExt(20,sum12)!=z3.ZeroExt(20,a)+z3.ZeroExt(20,b)+z3.ZeroExt(20,c)))
    # Finite corner enumeration checks the signed-minimum and modulo boundary
    # assumptions independently; it is not a substitute for the symbolic proof.
    for raw in range(65536):
        s=raw if raw<32768 else raw-65536
        low=raw&1023; high=raw>>10
        gate=(high==63 and low!=0) if raw&32768 else high==0
        mag=(-low)&1023 if raw&32768 else low
        assert (abs(s)<600)==(gate and mag<600)
    corners=[(0,0,0),(599,0,0),(600,0,0),(-599,0,0),(-600,0,0),(-1024,0,0),
        (-32768,0,0),(32767,0,0),(199,200,200),(200,200,200),(1023,1023,1023),(-1023,-1023,-1023)]
    return dict(passed=True,symbolic_queries=q,all65536_single_axis_raw_encodings_checked=True,
        corner_triples=[dict(raw=t,original_sum=sum(map(abs,t)),accepted=sum(map(abs,t))<600)for t in corners],
        maximum_low10_sum=3069,exact_sum_width=12,precision_lost=0,threshold_changed=False)


def four_state():
    """Exact Verilog four-state arithmetic/bit operations as value+known mask.

    X and Z have the same unknown arithmetic/logic behavior here; no case-equal,
    tristate driver or identity folding exists in either consumed expression.
    The result compared is the procedural if known-true branch, NOT X==0.
    """
    def bit(w,i): return z3.Extract(i,i,w[0])==1,z3.Extract(i,i,w[1])==1
    def not4(a): return z3.Not(a[0]),a[1]
    def and4(a,b):
        one=z3.And(a[1],a[0],b[1],b[0])
        return one,z3.Or(z3.And(a[1],z3.Not(a[0])),z3.And(b[1],z3.Not(b[0])),one)
    def or4(a,b):
        one=z3.Or(z3.And(a[1],a[0]),z3.And(b[1],b[0]))
        return one,z3.Or(one,z3.And(a[1],z3.Not(a[0]),b[1],z3.Not(b[0])))
    def reduce4(w, op):
        v=(z3.BoolVal(op=='and'),z3.BoolVal(True))
        for i in range(w[0].size()):v=(and4 if op=='and' else or4)(v,bit(w,i))
        return v
    def bvbits(xs): return z3.Concat(*[z3.If(x,z3.BitVecVal(1,1),z3.BitVecVal(0,1))for x in reversed(xs)])
    def mux4(s,yes,no):
        vs=[];ks=[]
        for i in range(yes[0].size()):
            a,ka=bit(yes,i);b,kb=bit(no,i)
            vs.append(z3.If(s[1],z3.If(s[0],a,b),a))
            ks.append(z3.If(s[1],z3.If(s[0],ka,kb),z3.And(ka,kb,a==b)))
        return bvbits(vs),bvbits(ks)
    def muxbit4(s,a,b):
        return z3.If(s[1],z3.If(s[0],a[0],b[0]),a[0]),z3.If(s[1],z3.If(s[0],a[1],b[1]),z3.And(a[1],b[1],a[0]==b[0]))
    def neg4(w):
        n=w[0].size();full=(1<<n)-1
        return -w[0],z3.If(w[1]==full,z3.BitVecVal(full,n),z3.BitVecVal(0,n))
    def add4(a,b):
        n=a[0].size();full=(1<<n)-1
        return a[0]+b[0],z3.If(z3.And(a[1]==full,b[1]==full),z3.BitVecVal(full,n),z3.BitVecVal(0,n))
    def cmp4(w,limit):return z3.ULT(w[0],limit),w[1]==(1<<w[0].size())-1
    def zext(w,n):return z3.ZeroExt(n,w[0]),z3.Concat(z3.BitVecVal((1<<n)-1,n),w[1])
    def bitwise4(a,b,op):
        if op=='xor':return a[0]^b[0],a[1]&b[1]
        vs=[];ks=[]
        for i in range(a[0].size()):
            v,k=(and4 if op=='and' else or4)(bit(a,i),bit(b,i));vs.append(v);ks.append(k)
        return bvbits(vs),bvbits(ks)
    def left1(w):return w[0]<<1,z3.Concat(z3.Extract(w[1].size()-2,0,w[1]),z3.BitVecVal(1,1))
    raws=[(z3.BitVec('raw_value_'+n,16),z3.BitVec('raw_known_mask_'+n,16))for n in('x','y','z')]
    originals=[];mags=[];gates=[]
    for v,k in raws:
        s=bit((v,k),15); ext=z3.SignExt(16,v),z3.SignExt(16,k)
        originals.append(mux4(s,neg4(ext),ext))
        low=z3.Extract(9,0,v),z3.Extract(9,0,k)
        high=z3.Extract(15,10,v),z3.Extract(15,10,k)
        mags.append(zext(mux4(s,neg4(low),low),2))
        gates.append(muxbit4(s,and4(reduce4(high,'and'),reduce4(low,'or')),not4(reduce4(high,'or'))))
    original=cmp4(add4(add4(originals[0],originals[1]),originals[2]),600)
    a,b,c=mags
    cs=bitwise4(bitwise4(a,b,'xor'),c,'xor')
    cc=left1(bitwise4(bitwise4(bitwise4(a,b,'and'),bitwise4(a,c,'and'),'or'),bitwise4(b,c,'and'),'or'))
    candidate=cmp4(add4(cs,cc),600)
    for gate in gates:candidate=and4(candidate,gate)
    outer=z3.Bools('same_original_mag_window_value same_original_mag_window_known')
    oldbranch=and4(outer,original);newbranch=and4(outer,candidate)
    knowntrue=lambda x:z3.And(x[1],x[0])
    queries=[prove('all raw48 value+known masks and arbitrary common mag-window0/1/X/Z: original/candidate procedural known-true acceptance',knowntrue(oldbranch)!=knowntrue(newbranch)),
        prove('candidate cannot accept an unknown raw gyro bit',z3.And(knowntrue(candidate),z3.Or(*[k!=65535 for _,k in raws])))]
    return dict(passed=True,queries=queries,
        logic_value_equality_claimed=False,
        exact_scope='All16bit raw values and independent known/unknown masks for all48bits; X/Z collapsed only where Verilog arithmetic/relational/logic treats both as unknown. Common original mag-window may also be unknown. A procedural if takes its true branch only for known1.',
        permitted_difference='If another axis is definitely outside the small window while one gyro bit isX/Z, the new gate may be0 while original arithmetic comparison isX; BOTH take the original rejection/else branch. The new gate is used only by that original if, not exposed as an output.' )


def mapped_path(build):
    report=build/'impl/gwsynthesis/g2_top_syn.rpt.html'; vg=build/'impl/gwsynthesis/g2_top.vg'
    path=first_path(report)
    assert path['summary']['From']=='u_control/sensor/gyro_y_3_s0'
    assert path['summary']['To']=='u_control/model_thread/fusion/st_1_s0'
    assert path['summary']['Data Arrival Time']=='20.829'
    active=False;body=[]
    with vg.open(encoding='utf8')as f:
        for line in f:
            if re.match(r'^module imu_fusion\s*\(',line):active=True
            elif active and re.match(r'^endmodule\b',line):break
            elif active:body.append(line)
    assert active
    text=''.join(body); params={}
    for name,key,value in re.findall(r'^defparam\s+(\S+)\s*\.\s*(\w+)\s*=\s*(.*?);',text,re.M):params.setdefault(name,{})[key]=value
    cells={name.lstrip('\\'):dict(kind=kind,name=name.lstrip('\\'),ports=dict(PORT.findall(ports)),parameters=params.get(name,{}))for kind,name,ports in CELL.findall(text)}
    chain=[]
    for row in path['combinational_output_rows']:
        node=row['node'];local=node.removeprefix('u_control/model_thread/fusion/').rsplit('/',1)[0]
        assert local in cells,('reported fusion cell not found',local)
        chain.append(dict(row,cell=cells[local]))
    # Prove the actual prefix function from its LUT truth tables and wires,
    # rather than labeling automatically numbered cells as an abs circuit.
    prefix=chain[:11]
    c=prefix[0]['cell'];assert c['kind']=='LUT4' and c['parameters']['INIT']=="16'h0001"
    assert [c['ports']['I'+str(i)] for i in range(4)]==['gyro_y[3]','gyro_y[2]','gyro_y[1]','gyro_y[0]']
    prev=c['ports']['F'];c=prefix[1]['cell']
    assert c['kind']=='LUT3' and c['parameters']['INIT']=="8'h10"
    assert [c['ports']['I'+str(i)]for i in range(3)]==['gyro_y[5]','gyro_y[4]',prev]
    prev=c['ports']['F']
    for bit,row in zip(range(6,14),prefix[2:10]):
        c=row['cell'];assert c['kind']=='LUT2' and c['parameters']['INIT']=="4'h4"
        assert [c['ports']['I'+str(i)]for i in range(2)]==[f'gyro_y[{bit}]',prev]
        prev=c['ports']['F']
    c=prefix[10]['cell'];assert c['kind']=='LUT3' and c['parameters']['INIT']=="8'hB4"
    assert [c['ports']['I'+str(i)]for i in range(3)]==[prev,'gyro_y[15]','gyro_y[14]']
    alu=[i for i,r in enumerate(chain)if r['cell']['kind']=='ALU']
    assert alu==list(range(11,24))
    sections=[]
    for label,rows in [('zero_prefix_through_abs_bit14',chain[:11]),
                       ('sum_carry_ALU_outputs',chain[11:24]),('rejection_and_state_decode',chain[24:])]:
        sections.append(dict(role=label,primitive_outputs=len(rows),estimated_cell_delay_ns=round(sum(float(r['delay_ns'])for r in rows),3)))
    delay=re.search(r'Arrival Data Path Delay:</td><td> cell: ([\d.]+),.*?route: ([\d.]+),.*?tC2Q: ([\d.]+),',report.read_text(encoding='utf8'))
    assert delay
    return dict(report=metadata(report),VG=metadata(vg),summary=path['summary'],
        actual_first_path_gate_count=len(chain),actual_first_path_primitive_kinds=dict(Counter(r['cell']['kind']for r in chain)),
        actual_abs_prefix_function_proved='NOR(rawgyro_y[3:0]), then extend zero test throughbit13 in9dependent LUT steps, then LUT3INITB4 forms original sign-dependent absbit14. It is an actual11-LUT source-to-abs prefix, not an assumed32-bit adder.',
        path_sections=sections,
        actual_combinational_chain=chain,estimated_data_delay_ns=dict(cell=float(delay[1]),route=float(delay[2]),C2Q=float(delay[3])),
        path_type='Synthesis estimate, not routed physicalSTA; nominal route estimates do not identify real congestion or final delay.')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--build',type=Path,default=BUILD);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    assert not a.out.exists();a.out.mkdir(parents=True)
    source=a.build/'control/imu_fusion.v';text=source.read_text(encoding='utf8')
    assert 'parameter integer GYRO_MOVE_LIMIT = 600,'in text and text.count(OLD_TEST)==text.count(ANCHOR)==1
    assert 'begin abs16 = v[15] ? -v : v; end'in text
    owner=(a.build/'control/mem_policy.v').read_text(encoding='utf8')
    assert 'imu_fusion #(.BIAS_SAMPLES(BIAS_SAMPLES),.GYRO_MUL(32\'sd4466)) fusion('in owner
    proposed=text.replace(ANCHOR,SMALL_COMPARE+ANCHOR,1).replace(OLD_TEST,NEW_TEST,1)
    # Undo only the wire declarations and same-edge test substitution. This
    # binds every state, FF assignment, sampling qualification and stop/reset
    # branch to the frozen source, rather than a prose claim about unchangedIO.
    assert proposed.replace(SMALL_COMPARE,'',1).replace(NEW_TEST,OLD_TEST,1)==text
    known=known_math();unknown=four_state();actual=mapped_path(a.build)
    result=dict(at=datetime.now().astimezone().isoformat(),passed=True,stage='fallback_combination_plan_with_lightweight_math_only',
        source=metadata(source),owner=metadata(a.build/'control/mem_policy.v'),entry=metadata(Path(__file__)),
        actual_SYN45_path=actual,known48_math=known,four_state_branch_proof=unknown,
        candidate_python_string=SMALL_COMPARE,old_consumed_expression=OLD_TEST,new_consumed_expression=NEW_TEST,
        candidate_full_source_UTF8_bytes=len(proposed.encode('utf8')),candidate_RTL_written=False,
        sample_edge_contract='Original state6 positive edge reads the then-current raw gyro inputs and mag32 exactly as before; no sample_valid-edge precompute, no holding FF, no state/latency or stop/reset change. Inputs changing between state2 and state6 remain observed at state6.',
        source_diff_contract='Only combinational generate wires plus the state6 condition token; original abs16 stays unchanged for bias accumulation and all normalization. Other GYRO_MOVE_LIMIT values select the literal original expression.',
        analytic_structure_not_mapping=dict(old_range='abs16 returns32bits; known axis0..32768, sum0..98304',
            candidate='high6 validity in parallel; three low10 magnitudes; explicit carry-save XOR/majority followed by one12bit carry-propagate add and unsigned<600',
            added_FF=0,added_cycles=0,threshold=600,low_magnitude_bits=10,exact_sum_bits=12,
            no_promise='Actual synthesis may widen/share/flatten logic or move the critical path; source widths and CSA syntax do not prove mapped depth or physical timing gains.'),
        recommendation='Reserve only if the current two-layout physical run still fails setup in this same gyro trust cone. The actual first path remains multi-level; this is a testable combination rewrite, not a proved cure.',
        next_validation=['Materialize only by root authorization in a unique source snapshot after current physical results; keep all source assets/FW/SDC unchanged.',
            'RTL four-state comparison at original state6 acceptance, threshold599/600, signed-min,-1024, X/Z and active reset; any public state/latency difference blocks.',
            'One actual small/full-context mapping review: verify high rejection, low10/CSA12 preservation or equivalent bound; bind all state/stop/reset and public trust decision. Preserve failed mapping instead of retrying identical inputs.',
            'Only a mapped structural improvement can justify a later physical candidate; compare full routed timing endpoint coverage, not only the25 synthesis paths.'],
        EDA_started=False,simulator_started=False,RTL_modified=False,actual_new_mapping_passed=False,physical_timing_accepted=False,deployment_accepted=False)
    assert not list(a.out.rglob('*.v'))
    save(a.out/'result.json',result)
    print(json.dumps(dict(passed=True,output=str(a.out/'result.json'),symbolic_queries=len(known['symbolic_queries'])+len(unknown['queries']),new_RTL=False,EDA=False)))


if __name__=='__main__':main()
