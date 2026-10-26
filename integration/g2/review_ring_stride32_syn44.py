"""Read-only SYN43->44 source, every memory INIT, CPU and mapped ring proof.

Actual clocks/26/reset/IR/lease use the separate completed-netlist reader.
No EDA is launched. Unknown or failed sections remain explicit evidence.
"""
from pathlib import Path
from datetime import datetime
from collections import Counter
import argparse
import json
import re
import traceback

from synthesis_connectivity import read
from audit_netlist_memories import BSRAM
from audit_routing_topology import primitive_directions
from review_stage3_slf128_memories import Tree, library_defaults, canonical, cpu_image, meta, LIBRARY
from prepare_no_sfp_clock_plan import construct_shared_logic
from prove_soc_console_remap import z3, literal
from retry_frozen_synthesis import memory_status

BASE=Path('F:/nav_integration_g2_20261023')
OLD=BASE/'SLF128_storage_safeFW_syn43'
NEW=BASE/'SLF128_ring_stride32_syn44'
PREP=BASE/'SLF128_ring_stride32_prepared1'
MODEL='u_control/model_thread'
OBS=MODEL+'/observation'
POL=MODEL+'/policy'
CORE=POL+'/core'
CHANGED={'control/rl_observation_v5.v','control/rl_policy_tick_v5.v','control/mem_policy.v'}


def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def save(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
def memory_kind(k):return k in BSRAM or k.startswith(('RAM16','ROM16'))
def same(a,b):return a.read_bytes()==b.read_bytes()


def source():
    roots=(OLD,PREP,NEW);maps=[]
    for root in roots:
        ps=[Path(p).relative_to(root).as_posix()for p in load(root/'manifest.json')['source_files']]
        assert len(ps)==len(set(ps))==63
        maps.append({p:root/p for p in ps})
    a,p,b=maps;assert a.keys()==p.keys()==b.keys()
    assert {r for r in a if not same(a[r],b[r])}==CHANGED
    assert all(same(p[r],b[r])for r in b)
    assets=[]
    for root in roots:
        assets.append({p.relative_to(root).as_posix():p for p in root.rglob('*')if p.is_file()and'impl'not in p.relative_to(root).parts and p.suffix.lower()in{'.mem','.hex','.mif','.bin'}})
    assert assets[0].keys()==assets[1].keys()==assets[2].keys()and len(assets[0])==104
    assert all(same(assets[0][n],assets[1][n])and same(assets[0][n],assets[2][n])for n in assets[0])
    assert len([n for n in assets[0]if n.startswith('src/mem/weight9_')])==35
    assert (NEW/'firmware/nav_fw.bin').stat().st_size==53572
    return dict(passed=True,RTL=63,changed_RTL=sorted(CHANGED),other60_RTL_byte_identical=True,
                actual_all63_RTL_byte_identical_to_selected_candidate=True,all104_assets_byte_identical=True,
                packed_weight_assets=35,FW_BIN_bytes=53572,source_bindings=[meta(v)for v in b.values()],
                actual_VG=meta(NEW/'impl/gwsynthesis/g2_top.vg'))


def memories(a,b,out):
    defaults=library_defaults()
    am={n:dict(kind=k,parameters=a.params[n])for n,k in a.kinds.items()if memory_kind(k)}
    bm={n:dict(kind=k,parameters=b.params[n])for n,k in b.kinds.items()if memory_kind(k)}
    isring=lambda n:n.startswith(OBS+'/g_ring.frame_mem')
    aa={n:c for n,c in am.items()if not isring(n)};bb={n:c for n,c in bm.items()if not isring(n)}
    assert aa.keys()==bb.keys(),('nonring leaf set changed',sorted(aa.keys()-bb.keys()),sorted(bb.keys()-aa.keys()))
    different=[]
    for n,c in aa.items():
        d=bb[n]
        if c['kind']!=d['kind']or canonical(c['kind'],c['parameters'],defaults)!=canonical(d['kind'],d['parameters'],defaults):different.append(n)
    assert not different,('nonring geometry/INIT changed',different)
    bs={n:(c['kind'],c['parameters'])for n,c in bm.items()if c['kind']in BSRAM}
    image=cpu_image(bs,defaults);binary=(NEW/'firmware/nav_fw.bin').read_bytes()
    assert len(bs)==269 and image==binary+bytes(131072-len(binary))
    assert image==b''.join(int(x,16).to_bytes(4,'little')for x in(NEW/'ram_w.hex').read_text().split())
    for i in range(4):assert image[i::4]==bytes(int(x,16)for x in(NEW/f'ram_b{i}.hex').read_text().split())
    save(out/'all_actual_memories.json',bm)
    return dict(passed=True,all_actual_memory_counts_before=dict(Counter(c['kind']for c in am.values())),
                all_actual_memory_counts_after=dict(Counter(c['kind']for c in bm.values())),
                every_nonring_name_kind_effective_parameter_INIT_equal=True,nonring_leaves=len(aa),BSRAM=269,
                CPU=dict(passed=True,bytes=131072,bitplanes=64,BIN_bytes=len(binary),BIN_zero_padding_ram_w_four_byte_lanes_exact=True),
                whole_nonring_data_control_port_equivalence=False,port_scope='Geometry/INIT and CPU proved; all clock pins and stop partitions are separate.')


def ring(a,b,out):
    anc={'','u_control',MODEL,OBS,POL,CORE}
    keep=lambda n:n in anc or n.rpartition('/')[0]in anc or n.startswith(OBS+'/')
    ca=read(OLD/'impl/gwsynthesis/g2_top.vg',keep);cb=read(NEW/'impl/gwsynthesis/g2_top.vg',keep)
    dirs=primitive_directions(LIBRARY)
    pa=lambda c:a.params[c['name']]
    pb=lambda c:b.params[c['name']]
    la,lb,boundaries=construct_shared_logic(ca,cb,pa,pb,dirs)
    # An actual load_index[7] FF changed its generated instance suffix. Bind
    # the exact surviving semantic Q role, then prove its complete transition
    # below; do not identify generated names by suffix removal.
    semantic_state=[]
    checks=[]
    def prove(label,x,y,condition=None):
        delta=x!=y
        if condition is not None:delta=z3.And(condition,delta)
        s=z3.Solver();s.set(timeout=20000);s.add(z3.simplify(delta));answer=s.check()
        if answer!=z3.unsat:
            save(out/'ring_counterexample.json',dict(label=label,result=str(answer),model=str(s.model())if answer==z3.sat else s.reason_unknown()))
        assert answer==z3.unsat,(label,str(answer))
        checks.append(dict(label=label,result='UNSAT'))
    bv=lambda n,w:z3.BitVecVal(n,w)
    bit=lambda v,n:z3.Extract(n,n,v)==bv(1,1)
    vector=lambda xs:z3.Concat(*[z3.If(x,bv(1,1),bv(0,1))for x in xs])if len(xs)>1 else z3.If(xs[0],bv(1,1),bv(0,1))
    def regs(cells,stem,width):
        found={int(m[1]):c for n,c in cells.items()if c['kind'].startswith('DFF')and(m:=re.fullmatch(re.escape(stem)+r'_(\d+)_s\d+',n))}
        assert set(found)==set(range(width)),(stem,sorted(found))
        return [found[i]for i in reversed(range(width))]
    for i,(x,y) in zip(reversed(range(8)),zip(regs(ca,POL+'/load_index',8),regs(cb,POL+'/load_index',8))):
        assert x['kind']==y['kind']and x['ports']['CLK']==y['ports']['CLK']and pa(x)==pb(y),(i,x,y)
        sym=z3.Bool('verified_semantic_load_index:'+str(i))
        la.q_symbols[x['ports']['Q'][0]]=lb.q_symbols[y['ports']['Q'][0]]=sym
        semantic_state.append(dict(bit=i,Q_before=x['ports']['Q'][0],Q_after=y['ports']['Q'][0],before=x['name'],after=y['name'],kind=x['kind'],parameters=pa(x)))
    qvec=lambda cs:vector([lb.one(c,'Q')for c in cs])
    nxt=lambda cs:vector([lb.next_state(c)for c in cs])
    pvec=lambda c,p:vector([lb.expr(n)for n in c['ports'][p]])
    for row in semantic_state:
        prove('load_index'+str(row['bit'])+' actual complete next-state across renamed FF',la.next_state(ca[row['before']]),lb.next_state(cb[row['after']]))
    work=qvec(regs(cb,OBS+'/g_ring.work_slot',3));lane=qvec(regs(cb,OBS+'/lane',5))
    ost=qvec(regs(cb,OBS+'/state',3));pst=qvec(regs(cb,POL+'/state',3))
    valid=lb.one(cb[OBS+'/valid_s0'],'Q');init=lb.one(cb[OBS+'/initialized_s0'],'Q')
    obsreset=lb.one(regs(cb,OBS+'/g_ring.work_slot',3)[0],'CLEAR')
    snap=regs(cb,POL+'/snapshot_word',24);oldsnap=regs(ca,POL+'/snapshot_word',24)
    polreset=lb.one(snap[0],'CLEAR');oldce=la.one(oldsnap[0],'CE')
    freeze=z3.And(oldce,pst==bv(1,3));advance=z3.And(oldce,pst==bv(2,3))
    legal=z3.And(z3.Not(polreset),z3.ULE(work,bv(5,3)),z3.ULE(lane,bv(27,5)),z3.Implies(valid,init))
    segment=regs(cb,OBS+'/g_ring.read_segment',3);rlane=regs(cb,OBS+'/g_ring.read_lane',5);rslot=regs(cb,OBS+'/g_ring.read_slot',3)
    seg,ln,slot=qvec(segment),qvec(rlane),qvec(rslot)
    lives=[qvec(regs(cb,OBS+'/g_ring.live'+str(i),3))for i in range(5)]
    frozen=[qvec(regs(cb,OBS+'/g_ring.frozen'+str(i),3))for i in range(5)]
    pointer_legal=z3.And(*[z3.ULE(x,bv(5,3))for x in lives+frozen],z3.ULE(slot,bv(5,3)),z3.ULE(ln,bv(27,5)),z3.ULE(seg,bv(6,3)))
    f=z3.And(freeze,init);advance_actual=z3.And(advance,z3.ULT(seg,bv(6,3)))
    next_seg=z3.If(f,bv(0,3),z3.If(z3.And(advance_actual,ln==bv(27,5)),seg+bv(1,3),seg))
    next_ln=z3.If(f,bv(1,5),z3.If(advance_actual,z3.If(ln==bv(27,5),bv(0,5),ln+bv(1,5)),ln))
    selected=slot
    for i in reversed(range(5)):selected=z3.If(seg==bv(i,3),frozen[i],selected)
    next_slot=z3.If(f,lives[4],z3.If(z3.And(advance_actual,ln==bv(27,5)),selected,slot))
    for label,cs,want,width,reset_value in [('segment',segment,next_seg,3,6),('lane',rlane,next_ln,5,0),('slot',rslot,next_slot,3,0)]:
        prove('cursor '+label+' actual full next-state',nxt(cs),z3.If(obsreset,bv(reset_value,width),want),legal)
    cursor=[]
    for c in segment+rlane+rslot:
        pin='PRESET'if c['kind']=='DFFPE'else'CLEAR'
        assert c['kind']in('DFFCE','DFFPE')and c['ports']['CLK']==['clk50_d']
        prove(c['name']+' actual reset root',lb.one(c,pin),obsreset)
        cursor.append(dict(**c,parameters=pb(c)))
    prove('work slot actual wrap/reset next-state',nxt(regs(cb,OBS+'/g_ring.work_slot',3)),
          z3.If(obsreset,bv(0,3),z3.If(ost==bv(4,3),z3.If(work==bv(5,3),bv(0,3),work+bv(1,3)),work)),legal)
    for i in range(5):
        dest=work if i==4 else z3.If(init,lives[i+1],work)
        prove('live'+str(i)+' actual first repeat/shift',nxt(regs(cb,OBS+'/g_ring.live'+str(i),3)),z3.If(obsreset,bv(0,3),z3.If(ost==bv(4,3),dest,lives[i])),legal)
        prove('frozen'+str(i)+' actual accepted capture',nxt(regs(cb,OBS+'/g_ring.frozen'+str(i),3)),z3.If(obsreset,bv(0,3),z3.If(f,lives[i],frozen[i])),legal)
    rams={}
    for n,c in cb.items():
        if c['kind']=='RAM16SDP4'and n.startswith(OBS+'/g_ring.frame_mem'):
            m=re.fullmatch(re.escape(OBS)+r'/g_ring.frame_mem_g_ring.frame_mem_(\d+)_(\d+)_s',n);assert m,n
            rams[tuple(map(int,m.groups()))]=c
    assert set(rams)=={(bank,group)for bank in range(13)for group in range(6)}
    direct=z3.And(*[lane!=bv(i,5)for i in(0,1,2,7,10,11,13,14,18,21)])
    writing=z3.And(z3.Not(obsreset),z3.Or(z3.And(ost==bv(1,3),direct),ost==bv(3,3)))
    address=z3.Concat(work,lane)
    peek=pst==bv(1,3);ra=z3.If(peek,z3.Concat(lives[4],bv(0,5)),z3.Concat(slot,ln))
    permutation=[]
    for (bank,group),c in sorted(rams.items()):
        assert c['ports']['CLK']==['clk50_d']and pb(c)=={}
        prove(c['name']+' WAD lower4',pvec(c,'WAD'),z3.Extract(3,0,address))
        prove(c['name']+' WRE exact bank select',lb.expr(c['ports']['WRE'][0]),z3.And(writing,z3.Extract(7,4,address)==bv(bank,4)),legal)
        prove(c['name']+' RAD lower4',pvec(c,'RAD'),z3.Extract(3,0,ra),pointer_legal)
        if bank==12:prove(c['name']+' extra bank never written',lb.expr(c['ports']['WRE'][0]),z3.BoolVal(False),legal)
        # The group-to-bit assignment is tied to unchanged original actual
        # DI functions, not accepted solely from the generated suffix.
        original=ca[OBS+f'/g_ring.frame_mem_g_ring.frame_mem_0_{group}_s']
        for pos,n in enumerate(c['ports']['DI']):
            prove(c['name']+f' DI{pos} equals original arithmetic word',lb.expr(n),la.expr(original['ports']['DI'][pos]),z3.And(legal,writing))
            if bank==0:permutation.append(dict(group=group,pin=pos,logical_bit=group*4+3-pos,original_DI=original['ports']['DI'][pos],new_DI=n))
    memory=z3.Array('SYN44_arbitrary_physical_frame_memory',z3.BitVecSort(8),z3.BitVecSort(24))
    for (bank,group),c in rams.items():
        word=z3.Select(memory,z3.Concat(bv(bank,4),pvec(c,'RAD')))
        for pos,n in enumerate(c['ports']['DO']):lb.opaque_symbols[(c['name'],'DO',pos)]=bit(word,group*4+3-pos)
    lb.memo={'GND':z3.BoolVal(False),'VCC':z3.BoolVal(True)}
    raw=z3.Select(memory,ra);clip=z3.If(raw>bv(409600,24),bv(409600,24),z3.If(raw<bv(-409600,24),bv(-409600,24),raw))
    expected=z3.If(peek,clip,z3.If(z3.ULT(seg,bv(6,3)),z3.If(seg==bv(0,3),clip,raw),bv(0,24)))
    consuming=z3.And(legal,pointer_legal,z3.Or(freeze,advance))
    # D pins are the true consumers even when optimization promotes/splits
    # the original24bit observation_read_data port across module boundaries.
    actual_D=vector([lb.one(c,'D')for c in snap])
    prove('all24 actual prefetch D current clip/history raw/bank selection',actual_D,expected,consuming)
    for c in snap:
        prove(c['name']+' accepted CE identical to original',lb.one(c,'CE'),oldce,z3.Not(polreset))
    assert not any(c['kind']=='MULT12X12'and n.startswith(OBS+'/')for n,c in cb.items())
    olddsp=[dict(**c,parameters=pa(c))for n,c in ca.items()if n.startswith(OBS+'/')and c['kind']=='MULT12X12']
    assert len(olddsp)==3
    # The outer LOAD comparator was remapped into the input-valid/address
    # region.  Check the changed RAM write boundary directly rather than
    # demanding byte-identical whole-core mapped module text.
    input_mem_proofs=0
    for n,c in cb.items():
        if not n.startswith(CORE+'/input_mem_')or not c['kind'].startswith('RAM16'):continue
        x=ca[n];assert x['kind']==c['kind']and pa(x)==pb(c)and x['ports']['CLK']==c['ports']['CLK']
        for pin in('WRE','WAD','DI'):
            prove(n+'/'+pin+' actual original input write boundary',vector([la.expr(v)for v in x['ports'][pin]]),vector([lb.expr(v)for v in c['ports'][pin]]))
            input_mem_proofs+=1
    assert input_mem_proofs==132*3,input_mem_proofs
    actual_core_body_equal=b.modules[b.module_kinds[CORE]]==a.modules[a.module_kinds[CORE]]
    save(out/'actual_ring_primitives.json',dict(RAMs=[dict(**c,parameters=pb(c))for c in rams.values()],cursor=cursor,old_address_DSPs=olddsp))
    save(out/'actual_ring_checks.json',dict(passed=True,count=len(checks),checks=checks,boundaries=boundaries,verified_semantic_state_bindings=semantic_state,word_permutation=permutation))
    return dict(passed=True,actual_frame_RAM16SDP4=78,all_observer_SSRAM_are_frame_mem=True,
                observed_bank_shape='13 banks x6 four-bit leaves x16words =208x24 allocated, not192x24',
                real_declared_words=192,logical_live_words=168,allocated_words=208,allocated_bits=4992,
                unexpected_vs72_reason='Six leaves form bank12 (nominal addresses192..207); all actual WREs are false for legalwork_slot0..5/lane0..27 and arbitrary bank12 outputs cannot affect accepted prefetchD.',
                normal_banks=12,extra_unreachable_bank=12,extra_unused_words=16,explicit_INIT_parameters=False,
                source_hole_words=24,total_unconsumed_physical_words=40,old_allocated_bits=4224,allocated_delta_bits=768,
                actual_cursor_FFs=11,cursor_reset_classes=dict(Counter(c['kind']for c in cursor)),
                removed_observer_MULT12X12=3,new_observer_MULT12X12=0,checks=len(checks),
                actual_real4500_core_module_body_exact=actual_core_body_equal,
                actual_core_input_RAM_write_port_miters=input_mem_proofs,
                core_body_difference='Mapped LOAD/input_valid comparators and automatic net/instance roles changed; complete mapped core equivalence is not asserted.',
                whole_actual_core_equivalence_proved=False,
                scope='Actual binary mapped ring partition under legal pointer/caller/reset invariants, plus original real4500 source binding and 396 actual input RAM write-boundary miters; source arbitraryRAM induction and real4500 RTL pass remain separately bound. Complete mapped core equivalence is not proved.',
                limits=['Not arbitraryX/Z control-state equivalence','Nonreset next-state checks are outside asserted policyreset; actual cursor reset dominance separately included','No routed timing/resource/CDC or physical stopping acceptance'])


def run(out):
    assert not out.exists();out.mkdir(parents=True)
    result=dict(passed=False,sections={},physical_build_ready=False,automatic_PNR=False)
    def section(name,fn):
        try:r=fn();assert r.get('passed'),r
        except Exception:r=dict(passed=False,error=traceback.format_exc())
        save(out/(name+'.json'),r);result['sections'][name]=dict(passed=r['passed'],path=str(out/(name+'.json')))
        save(out/'result.json',result);return r
    m=memory_status();assert m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3
    result['admission']=dict(physical=m.avail_phys,commit=m.avail_page)
    result['inputs']=[meta(p)for p in(OLD/'impl/gwsynthesis/g2_top.vg',NEW/'impl/gwsynthesis/g2_top.vg',LIBRARY)]
    assert load(NEW/'job.json')['phase']=='synthesis_complete'and load(NEW/'job.json')['tool_exit']==0
    section('source',source)
    a=Tree(OLD/'impl/gwsynthesis/g2_top.vg');b=Tree(NEW/'impl/gwsynthesis/g2_top.vg')
    section('all_memories_CPU',lambda:memories(a,b,out))
    section('actual_ring',lambda:ring(a,b,out))
    result.update(passed=all(x['passed']for x in result['sections'].values()),completed_at=datetime.now().astimezone().isoformat(),
                  pending=['Separate actual two clocks/original26/reset/IR/lease reviewer','Physical build/timing/IO/board acceptance'],
                  no_EDA_started=True)
    assert result['inputs']==[meta(p)for p in(OLD/'impl/gwsynthesis/g2_top.vg',NEW/'impl/gwsynthesis/g2_top.vg',LIBRARY)]
    save(out/'result.json',result);print(json.dumps(result,ensure_ascii=False));return 0 if result['passed'] else 1


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True);a=p.parse_args();raise SystemExit(run(a.out))
