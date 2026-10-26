"""Bind actual storage ports to temporal caller controls without rerunning RTL.

All external signals, opaque memory data and FFs outside the selected caller
are independently unconstrained each edge.  Actual internal FF transitions
come from published Gowin primitives.  A k-induction PASS is a temporal proof;
a SAT induction trial is only an insufficient invariant, not a real collision.
"""
import argparse
import json
from pathlib import Path
import re
import time
import traceback
from audit_routing_topology import primitive_directions
from prove_soc_console_remap import Logic, literal, z3
from review_stage3_slf128_memories import LIBRARY, Tree, meta, library_defaults
from synthesis_connectivity import read


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--vg',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=False)
    result=dict(complete=False,all_groups_proved=False,scope=__doc__,automatic_PNR=False)
    def save(): (a.out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    try:
        tree=Tree(a.vg);cells=read(a.vg);directions=primitive_directions(LIBRARY);defaults=library_defaults()
        qcells={q:c for c in cells.values()if c['kind'].startswith('DFF')for q in c['ports']['Q']}
        qs={q:z3.Bool('actual_FF:'+q)for q in qcells}
        supported=lambda k:bool(re.fullmatch(r'LUT[1-6]',k))or k.startswith('MUX2_LUT')or k in('ALU','INV','IBUF','BUFG','VCC','GND')or k.startswith('DFF')
        opaque={(n,p,i):z3.Bool(f'opaque:{n}:{p}:{i}')for n,c in cells.items()if not supported(c['kind'])for p,ns in c['ports'].items()if directions[c['kind']].get(p)=='output'for i in range(len(ns))}
        logic=Logic(cells,lambda c:tree.params[c['name']],directions,qs,opaque)
        def active(c,pin,sel):
            p=dict(defaults[c['kind']]);p.update(tree.params[c['name']]);want=literal(p['BLK_SEL_'+sel]);ns=c['ports']['BLKSEL'+('A'if sel=='0'else'B')]
            return z3.And(logic.expr(c['ports'][pin][0]),*[logic.expr(n)==bool(want>>(len(ns)-1-i)&1)for i,n in enumerate(ns)])
        groups=[('touch','u_nav/u_match/u_touch/','u_nav/u_match/'),('slist','u_nav/u_match/u_slist/','u_nav/u_match/'),('point_planes','u_nav/u_match/u_pb/','u_nav/u_match/'),('bitmap','u_nav/u_map/u_tv/u_bm/','u_nav/u_map/u_tv/'),('TXshadow','u_nav/u_mbx/u_txs/','u_nav/'),('console','u_soc/g_con[','u_soc/')]
        reports=[];result.update(actual_VG=meta(a.vg),library=meta(LIBRARY),groups=reports)
        for label,prefix,owner in groups:
            t=time.monotonic();leaves={n:c for n,c in cells.items()if n.startswith(prefix)and c['kind']in('SDPB','SDPX9B')};assert leaves,label
            props=[];ports=[]
            for n,c in leaves.items():
                w,r=active(c,'CEA','0'),active(c,'CEB','1');props.append(z3.And(w,r))
                ports.append(dict(primitive=n,parameters={k:v for k,v in tree.params[n].items()if not k.startswith('INIT')},write_CEA=c['ports']['CEA'],read_CEB=c['ports']['CEB'],BLKSELA=c['ports']['BLKSELA'],BLKSELB=c['ports']['BLKSELB'],ADA=c['ports']['ADA'],ADB=c['ports']['ADB'],OCE=c['ports']['OCE']))
            collision=z3.simplify(z3.Or(*props))
            def vars(e):return {str(v):v for v in z3.z3util.get_vars(e)}
            def internal(q):return q.startswith(owner)and not q.startswith('u_soc/u_cpu/')
            # Start with the exact endpoint next-state relation.  Several
            # source contracts make the two registered requests mutually
            # exclusive in the same case statement, so there is no reason to
            # recursively pull unrelated matcher arithmetic into the proof.
            relevant=vars(collision);sub=[];endpoint_FFs=[]
            for name,v in relevant.items():
                if name.startswith('actual_FF:')and internal(name[len('actual_FF:'):]):
                    q=name[len('actual_FF:'):];c=qcells[q];nxt=logic.next_state(c);sub.append((v,nxt));endpoint_FFs.append(dict(Q=q,primitive=c['name'],kind=c['kind'],actual_ports=c['ports'],actual_parameters=tree.params[c['name']]))
                else:sub.append((v,z3.Bool('next_unconstrained:'+name)))
            next_collision=z3.simplify(z3.substitute(collision,*sub))
            solver=z3.Solver();solver.set(timeout=10000);solver.add(next_collision);answer=solver.check()
            report=dict(group=label,owner_scope=owner,actual_leaves=len(leaves),actual_ports=ports,actual_endpoint_FFs=endpoint_FFs,
                        universal_port_exclusion_proved=answer==z3.unsat,
                        next_actual_port_overlap_query=str(answer),
                        method='Actual endpoint next-state Boolean miter; all old FF/data/input states unconstrained',
                        raw_free_state_SAT_is_not_a_collision=True,
                        source_temporal_contract_binding_pending=answer!=z3.unsat,elapsed_s=time.monotonic()-t)
            if answer==z3.unknown:report['reason']=solver.reason_unknown()
            reports.append(report);save()
            continue
            needed=set(vars(collision));tracked={};transitions={}
            while True:
                pending={v[len('actual_FF:'):]for v in needed if v.startswith('actual_FF:')and internal(v[len('actual_FF:'):])}-set(tracked)
                if not pending:break
                for q in sorted(pending):
                    tracked[q]=qs[q];transitions[q]=logic.next_state(qcells[q]);needed.update(vars(transitions[q]))
            allvars={}
            for e in [collision,*transitions.values()]:allvars.update(vars(e))
            memo={}
            def at(e,k):
                key=(e.get_id(),k)
                if key not in memo:memo[key]=z3.substitute(e,*[(v,z3.Bool(f's{k}:'+n))for n,v in allvars.items()])
                return memo[key]
            def transition(k):return [at(qs[q],k+1)==at(e,k)for q,e in transitions.items()]
            init=[]
            for q in tracked:
                c=qcells[q];p=dict(defaults[c['kind']]);p.update(tree.params[c['name']]);init.append(at(qs[q],0)==bool(literal(p['INIT'])))
            report=dict(group=label,owner_scope=owner,actual_leaves=len(leaves),actual_ports=ports,tracked_actual_FFs=len(tracked),external_or_cut_Boolean_inputs_per_edge=len(allvars)-len(tracked),universal_port_exclusion_proved=False,trials=[])
            reports.append(report);save()
            for depth in(1,2,4,8,12,20,24):
                s=z3.Solver();s.set(timeout=10000)
                for k in range(depth):s.add(*transition(k),z3.Not(at(collision,k)))
                s.add(at(collision,depth));ans=s.check();trial=dict(k=depth,induction=str(ans));report['trials'].append(trial)
                if ans==z3.unknown:trial['reason']=s.reason_unknown()
                if ans==z3.unsat:
                    base=z3.Solver();base.set(timeout=10000);base.add(*init)
                    for k in range(depth):base.add(*transition(k))
                    base.add(z3.Or(*[at(collision,k)for k in range(depth)]));b=base.check();trial['INIT_base']=str(b)
                    if b==z3.unsat:
                        report.update(universal_port_exclusion_proved=True,method='Actual Boolean transition k-induction plus exact actual INIT base',proved_k=depth);break
                    if b==z3.unknown:trial['base_reason']=base.reason_unknown()
                save()
            report['elapsed_s']=time.monotonic()-t;save()
        result.update(complete=True,all_groups_proved=all(r['universal_port_exclusion_proved']for r in reports),physical_timing_or_board_accepted=False)
    except Exception as e:result.update(error=f'{type(e).__name__}: {e}',traceback=traceback.format_exc())
    save();print(json.dumps({k:v for k,v in result.items()if k!='groups'},ensure_ascii=False));return 0 if result['complete']else 1


if __name__=='__main__':raise SystemExit(main())
