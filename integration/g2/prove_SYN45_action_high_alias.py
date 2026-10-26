"""Reachable SYN45 last-action/numerator high-bit aliases, actual FF semantics.

This is an INIT/reset/next-state induction on the 18 newly separate high bits,
not an assumption that independently quantified FFs start equal. No EDA.
"""
from pathlib import Path
from datetime import datetime
from collections import defaultdict
import argparse,json,re,traceback
from synthesis_connectivity import read
from revalidate_lidar_cdc import definitions
from review_small_block_stores import parameter_reader
from audit_routing_topology import primitive_directions
from prove_soc_console_remap import Logic,literal,z3
from prepare_baseline_fusion_retime import metadata,save

BASE=Path('F:/nav_integration_g2_20261023');REPO=Path(__file__).resolve().parents[2]
VG=BASE/'SLF128_measured_timing_syn45/impl/gwsynthesis/g2_top.vg'
LIB=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
P='u_control/model_thread/';MAP=P+'actuator_map/';OBS=P+'observation/';POL=P+'policy/'

def collect(out):
    r=json.loads((out/'result.json').read_text());assert r['passed']and r['UNSAT_count']==72
    assert metadata(VG)==r['actual_VG']and all(c['status']=='unsat'for c in r['checks'])
    rows=r['observer_numerator_inductive_relationships'];upstream=r['last_actions_inductive_relationships']
    assert len(rows)==len(upstream)==18
    for row in rows+upstream:
        for side in('left','right'):assert literal(row[side]['parameters']['INIT'])==0
    summary=dict(passed=True,scope='actual18reachableaction/numeratorhighbitinduction only; not arbitraryfreeFFequality orfullcoreproof',
      actual_VG=r['actual_VG'],full_result=metadata(out/'result.json'),actual_INIT_and_reset_bases_verified=True,
      full_nextstate_induction_UNSAT=72,relationships=[dict(lane=x['lane'],bit=x['bit'],new_Q=x['new_Q'],sign_Q=x['sign_Q'],
        actual_left=x['left']['name'],actual_right=x['right']['name'])for x in rows],
      upstream_actual_last_actions_relationships=[dict(lane=x['lane'],bit=x['bit'],new_Q=x['new_Q'],sign_Q=x['sign_Q'])for x in upstream],
      exact_actual_primitives_and_parameters_in_full_result=True,EDA_started=False,
      conclusion='INIT0 and actualreset establishaliases; actualwholeFFnext preserves them inductively. ChildmustverifyVG/left/rightbefore substitutingthese18Qwithnumerator13Q forDI proof.',
      physical_timing_or_wholeG2_equivalence=False)
    save(REPO/'docs/4_design/evidence/g2_20261023/SYN45_action_high_alias_induction_20261027.json',summary)
    print(json.dumps(dict(passed=True,relationships=18,UNSAT=72,evidence_saved=True)))

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--out',type=Path,default=BASE/'SYN45_action_last_high_alias_supplement1')
    ap.add_argument('--collect-only',action='store_true');a=ap.parse_args()
    if a.collect_only:collect(a.out);return
    assert not a.out.exists();a.out.mkdir();result=dict(passed=False,phase='actual_reachable_alias_induction',EDA_started=False)
    try:
        before=metadata(VG)
        def keep(n):
            return(MAP.startswith(n+'/')or OBS.startswith(n+'/')or POL.startswith(n+'/')or
              n.startswith(MAP)or n.startswith(OBS+'numerators')or
              n.startswith(POL+'actions')or n.startswith(POL+'action_valid')or
              (n.startswith(P)and'/'not in n[len(P):]))
        cells=read(VG,keep=keep);params=parameter_reader(definitions(VG));directions=primitive_directions(LIB)
        qcells={c['ports']['Q'][0]:c for c in cells.values()if c['kind'].startswith('DFF')}
        symbols={q:z3.Bool('actualQ:'+c['name'])for q,c in qcells.items()}
        logic=Logic(cells,params,directions,symbols,{})
        checks=[];relationships=[];last_relationships=[]
        def prove(label,x,y,assumptions=()):
            s=z3.Solver();s.set(timeout=20000);s.add(*assumptions,x!=y);r=s.check()
            row=dict(label=label,status=str(r));checks.append(row)
            if r!=z3.unsat:
                row['counterexample']=str(s.model())if r==z3.sat else s.reason_unknown()
                save(a.out/'first_rejection_checks.json',checks)
                raise AssertionError((label,str(r),row.get('counterexample')))
        def describe(c):return dict(c,parameters=params(c))
        def ff_for_q(q):
            assert q in qcells,('actual output not an FF Q',q)
            c=qcells[q];assert c['ports']['CLK']==['clk50_d']
            assert literal(params(c).get('INIT','0'))==0,('nonzeroINIT',c['name'],params(c))
            return c
        for lane in range(6):
            base=lane*24;sign_q=P+'model_executed_actions['+str(base+23)+']';sign=ff_for_q(sign_q)
            for bit in(14,15,18):
                q=P+'model_executed_actions['+str(base+bit)+']';c=ff_for_q(q)
                # INIT is a real primitive parameter, and every async reset
                # control on these FFs must force the same known0 base state.
                assert c['kind']==sign['kind']=='DFFCE'
                prove('last_actions lane%d bit%d asyncCLEAR matches sign'%(lane,bit),logic.one(c,'CLEAR'),logic.one(sign,'CLEAR'))
                inv=logic.expr(q)==logic.expr(sign_q)
                prove('last_actions lane%d bit%d fullnext preserves sign alias'%(lane,bit),
                  logic.next_state(c),logic.next_state(sign),(inv,))
                last_relationships.append(dict(lane=lane,bit=bit,new_Q=q,sign_Q=sign_q,
                  left=describe(c),right=describe(sign),INIT_both0=True,async_reset_same_value0=True,
                  next_equal_under_own_invariant=True))
                n=OBS+'numerators['+str(lane+22)+']_'+str(bit)+'_s0'
                sn=OBS+'numerators['+str(lane+22)+']_13_s0'
                assert n in cells and sn in cells,(n,sn)
                num=cells[n];ns=cells[sn];nq=num['ports']['Q'][0];sq=ns['ports']['Q'][0]
                assert num['kind']==ns['kind']=='DFFRE'and num['ports']['CLK']==ns['ports']['CLK']==['clk50_d']
                assert literal(params(num).get('INIT','0'))==literal(params(ns).get('INIT','0'))==0
                assert num['ports']['RESET']==ns['ports']['RESET']==['GND']
                assert num['ports']['D']==[q]and ns['ports']['D']==[sign_q]
                prove('numerator%d bit%d CE matches sign13'%(lane+22,bit),logic.one(num,'CE'),logic.one(ns,'CE'))
                ninv=logic.expr(nq)==logic.expr(sq)
                prove('numerator%d bit%d fullnext preserves sign13'%(lane+22,bit),
                  logic.next_state(num),logic.next_state(ns),(inv,ninv))
                relationships.append(dict(lane=lane,bit=bit,numerator=lane+22,new_Q=nq,sign_Q=sq,
                  left=describe(num),right=describe(ns),upstream_last_action_relation=dict(new_Q=q,sign_Q=sign_q),
                  INIT_both0=True,reset_pins_GND_and_same_CE_verified=True,next_equal_under_inductive_upstream_and_own_invariants=True))
        assert len(relationships)==len(last_relationships)==18
        assert metadata(VG)==before
        result.update(passed=True,actual_VG=before,library=metadata(LIB),small_partition_cells=len(cells),
          checks=checks,UNSAT_count=len(checks),last_actions_inductive_relationships=last_relationships,
          observer_numerator_inductive_relationships=relationships,
          actual_INIT_and_reset_bases_verified=True,actual_full_next_state_induction_verified=True,
          relation_use='Only these18 observerQ may be substituted by corresponding actual numerator13Q in the downstream DI proof, after checking thisVGmetadata and exactleft/rightprimitive+INIT',
          state_domain='INIT0 established; lastaction CLEAR1 also0. From own equality, actual wholeFF next preserves equality for arbitrary allotherstates/controls; observer relation additionally uses already-inductive upstream equality. No free-state equality was assumed as a base.',
          limitations='two-state mapped partition semantics; not wholecore/ringnumeric/physicaltiming proof; no new source or EDA')
    except Exception as e:result.update(phase='strict_alias_induction_failed_preserved',error=str(e),traceback=traceback.format_exc())
    result['ended_at']=datetime.now().astimezone().isoformat();save(a.out/'result.json',result)
    print(json.dumps(dict(passed=result['passed'],UNSAT=result.get('UNSAT_count'),relationships=len(result.get('observer_numerator_inductive_relationships',[])),error=result.get('error'))))

if __name__=='__main__':main()
