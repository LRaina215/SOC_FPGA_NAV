"""Prove actual bitmap caller exclusion through its flush pipeline.

Only a local trav_v3 module is parsed.  Data-dependent drop/terrain decisions
are unconstrained on every edge; all phase/count/valid-token FFs use their
actual mapped next-state functions and INIT.  No RTL tool is launched.
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
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--module',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=False)
    r=dict(passed=False,scope=__doc__,trials=[],automatic_PNR=False)
    def save(): (a.out/'result.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
    try:
        raw=a.module.read_text();assert re.match(r'^module\s+trav_v3\s*\(',raw)
        v=a.out/'local_parser_view.v';v.write_text(re.sub(r'^module\s+trav_v3\s*\(','module g2_top(',raw,count=1))
        cells=read(v);tree=Tree(v);directions=primitive_directions(LIBRARY);defaults=library_defaults()
        for c in cells.values():directions.setdefault(c['kind'],{})
        qcells={q:c for c in cells.values()if c['kind'].startswith('DFF')for q in c['ports']['Q']};qs={q:z3.Bool('FF:'+q)for q in qcells}
        logic=Logic(cells,lambda c:tree.params[c['name']],directions,qs,{})
        collision=z3.And(logic.expr('bm_we'),logic.expr('bm_re'))
        allowed=re.compile(r'^(?:ph(?:_next|[1-8]|w)?|r|c|R|r1|c1|flush|v1|wval|s[2-8]|bm_we|rev)(?:\[\d+\])?$')
        def variables(e):return{str(x):x for x in z3.z3util.get_vars(e)}
        needed=set(variables(collision));tracked={};transition={}
        while True:
            pending={n[3:]for n in needed if n.startswith('FF:')and allowed.fullmatch(n[3:])}-set(tracked)
            if not pending:break
            for q in sorted(pending):tracked[q]=qs[q];transition[q]=logic.next_state(qcells[q]);needed.update(variables(transition[q]))
        allvars={}
        for e in[collision,*transition.values()]:allvars.update(variables(e))
        cuts=sorted(n for n in allvars if not(n.startswith('FF:')and n[3:]in tracked));memo={}
        def at(e,k):
            key=(e.get_id(),k)
            if key not in memo:memo[key]=z3.substitute(e,*[(v,z3.Bool(f't{k}:'+n))for n,v in allvars.items()])
            return memo[key]
        def step(k):return[at(qs[q],k+1)==at(e,k)for q,e in transition.items()]
        initial=[at(qs[q],0)==bool(literal({**defaults[qcells[q]['kind']],**tree.params[qcells[q]['name']]}['INIT']))for q in tracked]
        r.update(actual_module=meta(a.module),actual_cells=len(cells),tracked_control_FFs=len(tracked),control_FF_names=sorted(tracked),data_or_external_cuts_independent_each_edge=cuts,
                 physical_binding='Actual42 u_bm CEA=bm_we and effective CEB implies bm_re, proven separately; port exclusion avoids all physical chunks independent of address')
        save()
        for depth in(1,2,4,8,12,16,20,24):
            s=z3.Solver();s.set(timeout=15000)
            for k in range(depth):s.add(*step(k),z3.Not(at(collision,k)))
            s.add(at(collision,depth));ans=s.check();trial=dict(k=depth,induction=str(ans));r['trials'].append(trial);save()
            if ans==z3.unsat:
                b=z3.Solver();b.set(timeout=15000);b.add(*initial)
                for k in range(depth):b.add(*step(k))
                b.add(z3.Or(*[at(collision,k)for k in range(depth)]));base=b.check();trial['exact_INIT_base']=str(base)
                if base==z3.unsat:r.update(passed=True,proved_k=depth,method='Temporal k-induction and actual INIT base over real mapped control FFs; terrain/data cuts remain fully arbitrary');break
            elif ans==z3.unknown:trial['reason']=s.reason_unknown()
            save()
        r['complete']=True
    except Exception as e:r.update(error=f'{type(e).__name__}: {e}',traceback=traceback.format_exc())
    save();print(json.dumps({k:v for k,v in r.items()if k not in('control_FF_names','data_or_external_cuts_independent_each_edge')},ensure_ascii=False));return 0 if r['passed']else 1


if __name__=='__main__':raise SystemExit(main())
