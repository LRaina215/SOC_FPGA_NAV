"""Read-only reachable control trace of the actual mapped LF row-buffer ports.

This reads one verbatim extracted synthesis module, never executes EDA/RTL and
never changes its state or the frozen build.  Actual FF INIT/next-state logic
and published LUT/ALU semantics generate a finite reset/start/full-scan trace.
RAM data is not modeled: only the closed actual control-state cone is needed.
"""
import argparse
import json
from pathlib import Path
import re
import sys

from audit_netlist_memories import PARAM
from audit_routing_topology import primitive_directions
from prove_soc_console_remap import Logic, literal, z3
from review_stage3_slf128_memories import LIBRARY, Tree, meta
from synthesis_connectivity import read


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--module',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=False)
    result=dict(passed=False,scope='Finite actual-mapped Boolean control trace; no RAM data, RTL simulator, or hardware test',automatic_PNR=False)
    try:
        raw=a.module.read_text(encoding='utf8')
        assert raw.startswith('module slf_lf(')
        view=a.out/'local_parser_view.v'
        view.write_text(raw.replace('module slf_lf(','module g2_top(',1),encoding='utf8')
        tree=Tree(view);cells=read(view);directions=primitive_directions(LIBRARY)
        qcells={q:c for c in cells.values() if c['kind'].startswith('DFF') for q in c['ports']['Q']}
        qs={q:z3.Bool('actual_FF:'+q) for q in qcells}
        def supported(k):return bool(re.fullmatch(r'LUT[1-6]',k)) or k.startswith('MUX2_LUT') or k in ('ALU','INV','IBUF','BUFG','VCC','GND') or k.startswith('DFF')
        opaque={(n,p,i):z3.Bool(f'actual_opaque:{n}:{p}:{i}') for n,c in cells.items() if not supported(c['kind']) for p,ns in c['ports'].items() if directions[c['kind']].get(p)=='output' for i in range(len(ns))}
        logic=Logic(cells,lambda c:tree.params[c['name']],directions,qs,opaque)
        rows={n:c for n,c in cells.items() if c['kind']=='SDPB' and n.startswith('lb1_lb1_')}
        assert len(rows)==3
        expressions={}
        for n,c in rows.items():
            p=tree.params[n]
            assert literal(p['BIT_WIDTH_0'])==literal(p['BIT_WIDTH_1'])==32
            assert c['ports']['BLKSELA']==c['ports']['BLKSELB']==['GND']*3
            for pin in ('CEA','CEB'):
                expressions[n+':'+pin]=logic.expr(c['ports'][pin][0])
            # In 32-bit mode AD[4:1] are byte-write masks, not address bits.
            for pin in ('ADA','ADB'):
                assert len(c['ports'][pin])==14
                for i,net in enumerate(c['ports'][pin][:9]):expressions[n+':'+pin+':'+str(13-i)]=logic.expr(net)
        for name in ('run','v1','lf_busy','lf_done'):
            expressions[name]=logic.expr(name)
        for name in ('c','c1','c1_b','c1_c','r'):
            width=9 if name in ('c','c1','r') else 8
            for i in range(width):expressions[f'{name}[{i}]']=logic.expr(f'{name}[{i}]')
        def var_names(expr):return {str(v) for v in z3.z3util.get_vars(expr)}
        needed=set().union(*(var_names(e) for e in expressions.values()))
        transitions={};done=set()
        while True:
            pending={n[len('actual_FF:'):] for n in needed if n.startswith('actual_FF:')}-done
            if not pending:break
            for q in sorted(pending):
                e=logic.next_state(qcells[q]);transitions[q]=e;needed.update(var_names(e));done.add(q)
        allowed={'external:r2','external:lf_start'}
        assert needed-{f'actual_FF:{q}' for q in done} <= allowed,('Control trace is not closed',sorted(needed-{f'actual_FF:{q}' for q in done}-allowed))
        values={f'actual_FF:{q}':bool(literal(tree.params[c['name']].get('INIT',"1'b1" if c['kind'] in ('DFFSE','DFFPE') else "1'b0"))) for q,c in qcells.items() if q in done}
        # Compile each shared Z3 Boolean node once into a topologically ordered
        # elementary program; evaluate the real mapped logic, not source rules.
        nodes=[];ids={}
        def add(e):
            key=e.get_id()
            if key in ids:return ids[key]
            ch=[add(x) for x in e.children()];k=e.decl().kind()
            if z3.is_true(e):op=('const',True)
            elif z3.is_false(e):op=('const',False)
            elif not ch:op=('var',str(e))
            else:
                ops={z3.Z3_OP_NOT:'not',z3.Z3_OP_AND:'and',z3.Z3_OP_OR:'or',z3.Z3_OP_XOR:'xor',z3.Z3_OP_ITE:'if',z3.Z3_OP_EQ:'eq',z3.Z3_OP_DISTINCT:'neq'}
                assert k in ops,('Unsupported concrete Boolean node',str(e),k)
                op=(ops[k],ch)
            ids[key]=len(nodes);nodes.append(op);return ids[key]
        ei={n:add(e) for n,e in expressions.items()};ti={q:add(e) for q,e in transitions.items()}
        def evaluate(env):
            out=[]
            for op,arg in nodes:
                if op=='const':v=arg
                elif op=='var':v=env[arg]
                elif op=='not':v=not out[arg[0]]
                elif op=='and':v=all(out[j] for j in arg)
                elif op=='or':v=any(out[j] for j in arg)
                elif op=='xor':v=sum(out[j] for j in arg)%2==1
                elif op=='if':v=out[arg[1]] if out[arg[0]] else out[arg[2]]
                elif op=='eq':v=all(out[j]==out[arg[0]] for j in arg[1:])
                elif op=='neq':v=len(set(out[j] for j in arg))==len(arg)
                else:raise AssertionError(op)
                out.append(v)
            return out
        def word(out,name,width):return sum(int(out[ei[f'{name}[{i}]']])<<i for i in range(width))
        collisions=[];tail=[];run_cycles=0;writes={n:0 for n in rows};reads={n:0 for n in rows}
        for edge in range(18000):
            # Three asserted reset edges, three released edges, one start edge.
            values.update({'external:r2':edge<3,'external:lf_start':edge==6})
            out=evaluate(values)
            obs=dict(edge=edge,reset=values['external:r2'],start=values['external:lf_start'],run=out[ei['run']],v1=out[ei['v1']],busy=out[ei['lf_busy']],done=out[ei['lf_done']],c=word(out,'c',9),r=word(out,'r',9),c1=word(out,'c1',9),c1_b=word(out,'c1_b',8),c1_c=word(out,'c1_c',8))
            if obs['run']:run_cycles+=1
            for n,c in rows.items():
                we=out[ei[n+':CEA']];re_=out[ei[n+':CEB']]
                wa=sum(int(out[ei[n+':ADA:'+str(i)]])<<(i-5) for i in range(5,14))
                ra=sum(int(out[ei[n+':ADB:'+str(i)]])<<(i-5) for i in range(5,14))
                writes[n]+=we;reads[n]+=re_
                if we and re_ and wa==ra:
                    collisions.append(dict(**obs,primitive=n,write_word=wa,read_word=ra,physical_chunks=[2*wa,2*wa+1]))
            if edge>16895:tail.append(obs)
            values.update({f'actual_FF:{q}':out[i] for q,i in ti.items()})
            if edge>10 and obs['done']:break
        assert run_cycles==130*130,('Not a complete actual128 LF scan',run_cycles)
        assert all(n==130*130 for n in writes.values()),('Incomplete actual row writes',writes)
        result.update(passed=not collisions,trace_complete=True,actual_module=meta(a.module),library=meta(LIBRARY),actual_cells=len(cells),closed_control_FFs=len(done),evaluated_Boolean_nodes=len(nodes),input_sequence='r2=1 for3 edges, release3 edges, lf_start=1 for1 edge, otherwise0',edges=edge+1,scan_run_cycles=run_cycles,row_write_edges=writes,row_read_edges=reads,collision_events=collisions,last_control_edges=tail,physical_address_semantics='32-bit port covers18-bit chunks {2*AD[13:5],2*AD[13:5]+1}; AD[4:1] are write byte masks',RAM_data_modeled=False,meaning='A listed collision is an actual-control reachable array-port event; memory output usefulness is not an exception to the device collision restriction')
    except Exception as e:result.update(error=f'{type(e).__name__}: {e}')
    (a.out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps({k:v for k,v in result.items() if k!='last_control_edges'},ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
