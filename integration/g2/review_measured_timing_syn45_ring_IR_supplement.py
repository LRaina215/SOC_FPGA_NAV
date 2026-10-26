"""Only the two binding exceptions from SYN45 actual review1, read-only.

The first result and its SAT model remain unchanged. Pair source-preserved age
bits by actual Q/INIT/clock, then prove every paired transition. IR input rename
is accepted only after its external mapped Boolean functions are proved equal;
all154 actual IR state transitions and every live public output are checked.
"""
from pathlib import Path
from datetime import datetime
import argparse,gc,json,re,traceback
import review_measured_timing_syn45_storage_clock_ring as config
from review_stage3_slf128_memories import Tree,LIBRARY,meta
from synthesis_connectivity import read,DECL
from revalidate_lidar_cdc import definitions
from review_small_block_stores import parameter_reader
from review_model25_joint_syn import module_bindings
from audit_routing_topology import primitive_directions
from prepare_no_sfp_clock_plan import construct_shared_logic
from prove_soc_console_remap import Logic,z3
from retry_frozen_synthesis import memory_status

OLD,NEW=config.OLD,config.NEW
CERTIFIED_ALIASES=None
RING_OUT=None


def save(p,x):p.write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))


def bind_age(la,lb,ca,cb,pa,pb,rows):
    stem='u_control/model_thread/sensor_age'
    def index(cells):
        return {int(m[1]):c for n,c in cells.items()if c['kind'].startswith('DFF')and(m:=re.fullmatch(re.escape(stem)+r'_(\d+)_s\d+',n))}
    a,b=index(ca),index(cb);assert set(a)==set(b)==set(range(32))
    for bit in range(32):
        x,y=a[bit],b[bit]
        assert x['ports']['Q']==y['ports']['Q']==[stem+f'[{bit}]']
        assert x['kind']==y['kind']and x['ports']['CLK']==y['ports']['CLK']and pa(x)==pb(y)
        sym=z3.Bool('verified_source_preserved_sensor_age:'+str(bit))
        la.q_symbols[x['ports']['Q'][0]]=lb.q_symbols[y['ports']['Q'][0]]=sym
        rows.append(dict(role='sensor_age',bit=bit,Q_before=x['ports']['Q'][0],Q_after=y['ports']['Q'][0],before=x['name'],after=y['name'],kind=x['kind'],parameters=pa(x),binding='Exact actual semantic Q bit, kind/INIT/clock; complete next-state is proved with all paired bits.'))
    if CERTIFIED_ALIASES is not None:
        certificate=load(CERTIFIED_ALIASES)
        assert certificate['passed']and certificate['UNSAT_count']==72
        assert certificate['actual_VG']==meta(NEW/'impl/gwsynthesis/g2_top.vg')and certificate['library']==meta(LIBRARY)
        assert certificate['actual_INIT_and_reset_bases_verified']and certificate['actual_full_next_state_induction_verified']
        relations=certificate['observer_numerator_inductive_relationships']
        assert {(x['numerator'],x['bit'])for x in relations}=={(n,b)for n in range(22,28)for b in (14,15,18)}
        assert len(relations)==18 and len(certificate['last_actions_inductive_relationships'])==18
        bindings=[]
        for row in relations:
            assert row['INIT_both0']and row['reset_pins_GND_and_same_CE_verified']and row['next_equal_under_inductive_upstream_and_own_invariants']
            x,y=[cb[row[p]['name']]for p in ('left','right')]
            assert dict(**x,parameters=pb(x))==row['left']and dict(**y,parameters=pb(y))==row['right']
            q,s=row['new_Q'],row['sign_Q'];assert x['ports']['Q']==[q]and y['ports']['Q']==[s]
            assert q in lb.q_symbols and s in lb.q_symbols
            lb.q_symbols[q]=lb.q_symbols[s]
            bindings.append(dict(new_Q=q,sign_Q=s,actual_primitive_INIT_and_ports_exactly_match_certificate=True))
        lb.memo={'GND':z3.BoolVal(False),'VCC':z3.BoolVal(True)}
        save(RING_OUT/'inductive_alias_certificate_binding.json',dict(passed=True,certificate=meta(CERTIFIED_ALIASES),actual_VG=meta(NEW/'impl/gwsynthesis/g2_top.vg'),certified_two_level_induction_UNSAT_checks=72,bindings=bindings,scope='Only the18 certified new numerator highQ are replaced by their actual sign13Q; no other relation is assumed.'))


def ring(out):
    global RING_OUT
    RING_OUT=out
    import review_ring_stride32_syn44 as r
    r.OLD=OLD;r.NEW=NEW;r.PREP=config.PREP;r.CHANGED=config.CHANGED
    r.OLD_OBSERVER_ADDRESS_DSPS=0;r.EXTRA_STATE_BINDER=bind_age
    a,b=Tree(OLD/'impl/gwsynthesis/g2_top.vg'),Tree(NEW/'impl/gwsynthesis/g2_top.vg')
    result=r.ring(a,b,out)
    if CERTIFIED_ALIASES is not None:
        result.update(certified_reachable_state_aliases=18,certificate=meta(CERTIFIED_ALIASES),
                      certificate_binding=str(out/'inductive_alias_certificate_binding.json'))
    return result


def ir(out):
    dirs=primitive_directions(LIBRARY)
    scope='u_control/remote_thread';mods=[];bind=[];local=[];params=[]
    # Definitions and one module body's primitives are retained, not a second
    # complete-G2 graph. The external graph includes only native top/control.
    global_cells=[];global_params=[]
    for label,root in (('old',OLD),('new',NEW)):
        m=definitions(root/'impl/gwsynthesis/g2_top.vg')
        b=module_bindings(m,[scope])[scope]
        p=out/(label+'_IR_actual.v')
        p.write_text('module g2_top ('+m[b['module_kind']]+'endmodule\n',encoding='utf8')
        mods.append(m);bind.append(b);local.append(read(p));params.append(parameter_reader(definitions(p)))
        global_cells.append(read(root/'impl/gwsynthesis/g2_top.vg',lambda n:n in ('','u_control')or n.rpartition('/')[0]in ('','u_control')))
        global_params.append(parameter_reader(m))
    la,lb,boundary=construct_shared_logic(*global_cells,*global_params,dirs)
    checks=[]
    def equal(label,x,y):
        s=z3.Solver();s.set(timeout=20000);s.add(z3.simplify(x!=y));status=s.check()
        if status!=z3.unsat:
            save(out/'IR_counterexample.json',dict(label=label,status=str(status),model=str(s.model())if status==z3.sat else s.reason_unknown()))
        assert status==z3.unsat,(label,str(status))
        checks.append(dict(label=label,result='UNSAT'))
    def ports(m,b,kind):
        return {n.lstrip('\\'):(int(hi)if hi else None,int(lo)if lo else None)for d,hi,lo,n in DECL.findall(m[b['module_kind']])if d==kind}
    ai,bi=ports(mods[0],bind[0],'input'),ports(mods[1],bind[1],'input')
    common=ai.keys()&bi.keys();ao,bo=ports(mods[0],bind[0],'output'),ports(mods[1],bind[1],'output')
    old_only,new_only=ai.keys()-common,bi.keys()-common
    assert len(old_only)==len(new_only)==1
    old_pin,new_pin=next(iter(old_only)),next(iter(new_only))
    assert ai[old_pin]==bi[new_pin]==(None,None)
    x,y=bind[0]['ports'][old_pin][0],bind[1]['ports'][new_pin][0]
    equal('actual renamed IR input external Boolean boundary',la.expr(x),lb.expr(y))
    for p in common:
        assert ai[p]==bi[p]and bind[0]['ports'][p]==bind[1]['ports'][p],p
    assert set(bo)<=set(ao)and all(ao[p]==bo[p]for p in bo)
    # These outputs were optimized back into the source-identical IR module
    # when the external unity-gain branch no longer consumed them.
    removed=set(ao)-set(bo)
    assert removed=={'hold_count_31_11','n683_8','n681_8','ir_key_code'},removed
    af={n:c for n,c in local[0].items()if c['kind'].startswith('DFF')}
    bf={n:c for n,c in local[1].items()if c['kind'].startswith('DFF')}
    assert len(af)==len(bf)==154
    qa,qb={},{};pairs=[];used=set()
    for n,a in sorted(af.items()):
        if n in bf:b=bf[n];method='same_actual_FF_instance'
        else:
            matches=[c for c in bf.values()if c['ports']['Q']==a['ports']['Q']]
            assert len(matches)==1 and re.fullmatch(r'hold_count_0_s\d+',n)
            b=matches[0];assert re.fullmatch(r'hold_count_0_s\d+',b['name'])
            method='actual_same_hold_count_bit0_Q_and_source_role'
        assert b['name']not in used;used.add(b['name'])
        assert a['kind']==b['kind']and params[0](a)==params[1](b)and a['ports']['CLK']==b['ports']['CLK']
        sym=z3.Bool('actual_IR_state:'+n)
        qa[a['ports']['Q'][0]]=qb[b['ports']['Q'][0]]=sym
        pairs.append(dict(before=n,after=b['name'],Q_before=a['ports']['Q'][0],Q_after=b['ports']['Q'][0],kind=a['kind'],parameters=params[0](a),method=method))
    assert used==set(bf)
    a,b=Logic(local[0],params[0],dirs,qa,{}),Logic(local[1],params[1],dirs,qb,{})
    shared=z3.Bool('proved_IR_input_boundary')
    a.memo[old_pin]=b.memo[new_pin]=shared
    for row in pairs:equal(row['before']+' complete next-state',a.next_state(af[row['before']]),b.next_state(bf[row['after']]))
    outputs=[]
    for p,(hi,lo)in bo.items():
        assert bind[0]['ports'][p]==bind[1]['ports'][p],p
        bits=[p]if hi is None else[p+f'[{i}]'for i in range(hi,lo-1,-1)]
        for bit in bits:equal(bit+' live actual output',a.expr(bit),b.expr(bit));outputs.append(bit)
    assert len(outputs)==27
    save(out/'IR_actual_bindings_checks.json',dict(passed=True,bindings=bind,input_boundary=dict(before_port=old_pin,after_port=new_pin,before_net=x,after_net=y,proof='UNSAT actual enclosing native control cone'),state_pairs=pairs,checks=checks,removed_public_outputs=sorted(removed)))
    return dict(passed=True,actual_IR_FFs=154,next_state_checks=154,live_output_checks=27,external_renamed_input_boundary_checks=1,total_checks=len(checks),all_FF_kind_INIT_clock_preserved=True,all_live_stop_height_mode_outputs_equal=True,removed_unconsumed_ports=sorted(removed),scope='Actual binary IR state transitions and every live public output, including stop; removed external diagnostic/gain-consumer ports are not asserted as still public. Physical stopping/IR decoding waveform acceptance remains separate.',full_sequential_whole_G2_proof=False)


def run(out,parts):
    assert not out.exists();out.mkdir(parents=True)
    result=dict(passed=False,sections={},old=str(OLD),new=str(NEW),original_failure_preserved=str(config.DEFAULT_OUT/'result.json'),no_EDA_started=True)
    m=memory_status();assert m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3
    result['admission']=dict(physical=m.avail_phys,commit=m.avail_page)
    watched=[OLD/'impl/gwsynthesis/g2_top.vg',NEW/'impl/gwsynthesis/g2_top.vg',LIBRARY]
    if CERTIFIED_ALIASES is not None:watched.append(CERTIFIED_ALIASES)
    before=[meta(p)for p in watched]
    for name,fn in (('actual_ring',ring),('actual_IR_stop_partition',ir)):
        if name not in parts:continue
        path=out/name;path.mkdir()
        try:v=fn(path);assert v['passed']
        except Exception:v=dict(passed=False,error=traceback.format_exc())
        save(path/'result.json',v);result['sections'][name]=dict(passed=v['passed'],path=str(path/'result.json'))
        save(out/'result.json',result);gc.collect()
    assert before==[meta(p)for p in watched]
    result.update(passed=all(x['passed']for x in result['sections'].values()),input_bindings=before,completed_at=datetime.now().astimezone().isoformat(),physical_unknown=True)
    save(out/'result.json',result);print(json.dumps(result,ensure_ascii=False));return 0 if result['passed']else 1


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--parts',nargs='+',choices=['actual_ring','actual_IR_stop_partition'],default=['actual_ring','actual_IR_stop_partition'])
    p.add_argument('--certified-aliases',type=Path);a=p.parse_args();CERTIFIED_ALIASES=a.certified_aliases
    raise SystemExit(run(a.out,a.parts))
