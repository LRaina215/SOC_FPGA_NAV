"""Low-cost read-only IR/OLED source and selected-module check for SYN39.

No waits or EDA. An unfinished synthesis yields an explicit pending result.
No whole-design flattening, RAM/clock review, or automatic alias equivalence.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import re

BASE=Path('F:/nav_integration_g2_20261023')
SCOPES={'IR':'u_control/remote_thread','OLED':'u_control/display'}


def meta(p):
    s=p.stat();return dict(path=str(p),bytes=s.st_size,mtime_ns=s.st_mtime_ns)


def save(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')


def hits(path,patterns):
    return [dict(line=i,text=t.strip()) for i,t in enumerate(path.read_text(encoding='utf-8-sig').splitlines(),1)
            if any(q in t for q in patterns)]


def actual(reference,candidate):
    # Imports here avoid loading optional solver/process dependencies at all
    # when only a source/pending audit is possible.
    from audit_netlist_memories import MODULE,INST
    from review_model25_joint_syn import module_bindings
    from review_small_block_stores import parameter_reader
    from synthesis_connectivity import read,PORT,CELL
    paths=[r/'impl/gwsynthesis/g2_top.vg' for r in (reference,candidate)]
    bound=[meta(p) for p in paths]
    modules=[dict(MODULE.findall(p.read_text(encoding='utf-8'))) for p in paths]
    bindings=[module_bindings(m,list(SCOPES.values())) for m in modules]
    def closure(m,kind):
        found={}
        def visit(k):
            if k in found:return
            found[k]=m[k]
            for child,name in INST.findall(m[k]):
                if child in m:visit(child)
        visit(kind);return found
    module_checks={}
    for label,scope in SCOPES.items():
        old,new=(b[scope] for b in bindings)
        a,b=(closure(m,x['module_kind']) for m,x in zip(modules,(old,new)))
        module_checks[label]=dict(reference_binding=old,candidate_binding=new,
            exact_actual_port_binding_equal=old['ports']==new['ports'],
            exact_reachable_local_module_definitions_equal=a==b,
            local_module_kinds_reference=sorted(a),local_module_kinds_candidate=sorted(b),
            changed_or_added_local_module_definitions=sorted(k for k in a.keys()|b.keys() if a.get(k)!=b.get(k)),
            difference_policy='A changed actual spelling/definition is retained for review; no generated suffix alias is presumed equivalent.')
    ir=[]
    for path,m,b in zip(paths,modules,bindings):
        scope=SCOPES['IR'];stop=b[scope]['ports'].get('ir_emergency_stop')
        assert stop and len(stop)==1, ('actual stop port unavailable',path)
        cells=read(path,keep=lambda n:n=='u_control' or n==scope or n.startswith(scope+'/'))
        params=parameter_reader(m)
        driver=[dict(**c,parameters=params(c)) for c in cells.values() if c['ports'].get('Q')==stop]
        assert len(driver)==1 and driver[0]['kind'].startswith('DFF'), ('actual IR stop state driver',path,driver)
        ir.append(dict(stop_net=stop[0],actual_latch=driver[0],selected_IR_primitives=len(cells),
                       selected_IR_by_kind=dict(Counter(c['kind'] for c in cells.values()))))
    pads=[]
    for m in modules:
        row={}
        for kind,name,body in CELL.findall(m['g2_top']):
            if kind=='IOBUF' and name.lstrip('\\') in ('sck_iobuf','sda_iobuf'):
                row[name.lstrip('\\')]=dict(kind=kind,ports={p:v.strip() for p,v in PORT.findall(body)})
        assert set(row)=={'sck_iobuf','sda_iobuf'}
        pads.append(row)
    checks=dict(IR_latch_kind_ports_and_parameters_equal=ir[0]['actual_latch']==ir[1]['actual_latch'],
                OLED_actual_pad_bindings_equal=pads[0]==pads[1],
                selected_IR_OLED_bindings_equal=all(x['exact_actual_port_binding_equal'] for x in module_checks.values()),
                selected_IR_OLED_local_definitions_equal=all(x['exact_reachable_local_module_definitions_equal'] for x in module_checks.values()))
    return dict(status='reviewed_literal_bindings' if all(checks.values()) else 'specific_actual_differences_need_review',
                checks=checks,modules=module_checks,IR_latch=[dict(label=k,**v) for k,v in zip(('reference','candidate'),ir)],
                OLED_pad_bindings=pads,metadata_before=bound,inputs_unchanged=bound==[meta(p) for p in paths],
                whole_control_or_feedback_cone_equivalence_claimed=False)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--reference',type=Path,default=BASE/'baseline_fusion_model25_bounded_MAX_syn37')
    ap.add_argument('--candidate',type=Path,default=BASE/'baseline_fusion_model25_observation_ring_syn39')
    ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=False)
    job=json.loads((a.candidate/'job.json').read_text(encoding='utf-8-sig'));prepared=Path(job['source'])
    fixed=['g2_top.v','control4500_core.v','command_lease_guard.v','control/ir_remote_control.v',
           'control/motor520_closed_loop.v','control/motor520_hybrid_speed.v']
    fixed+=sorted(p.relative_to(a.reference).as_posix() for p in (a.reference/'control').glob('oled*.v'))
    paths=[root/rel for root in (a.reference,prepared,a.candidate) for rel in fixed]
    before=[meta(p) for p in paths]
    equality={rel:dict(prepared_equals_SYN37=(a.reference/rel).read_bytes()==(prepared/rel).read_bytes(),
                       actual_frozen_source_equals_prepared=(a.candidate/rel).read_bytes()==(prepared/rel).read_bytes()) for rel in fixed}
    patterns=['emergency_stop','external_fault','assign ready=','wire run=','stop_latched','assign active=',
              'assign status_flags=','action_hold50[335:272]','config_data;','config_bridge(',
              'assign {enable_25','assign {ready_raw','sensor_ok(sensor_ok','feedback_valid=',
              '.feedback_valid(odo_valid)','wire control_run','wire wheel_control_enable','.enable(MODEL_ENABLE && command_run)',
              '.ready(model_ready)','.ready(),.error()','HEIGHT_INITIAL_Q12','parameter IR_ENABLE']
    watch=fixed+['control/mem_policy_related25.v','control/mem_policy.v']
    evidence={rel:{key:hits(root/rel,patterns) for key,root in (('reference',a.reference),('prepared',prepared),('candidate',a.candidate))} for rel in watch}
    statements={rel:all([x['text'] for x in rows['reference']]==[x['text'] for x in rows[k]] for k in ('prepared','candidate')) for rel,rows in evidence.items()}
    source_pass=all(all(r.values()) for r in equality.values()) and all(statements.values())
    phase=job.get('phase');complete=phase=='synthesis_complete' and job.get('tool_exit')==0 and (a.candidate/'impl/gwsynthesis/g2_top.vg').exists()
    mapped=actual(a.reference,a.candidate) if complete else dict(status='pending_SYN39_completion',job_phase_observed=phase,
        pending_points=['Actual IR stop FF kind/D/Q/reset/CE and INIT parameters','IR/OLED actual module boundary bindings and reachable local module definitions','OLED two IOBUF pad bindings'],waited=False)
    result=dict(scope='Independent low-cost OLED/IR preservation check; source evidence plus selected actual bindings only, no EDA or full-netlist flattening',
                reference=str(a.reference),prepared=str(prepared),candidate=str(a.candidate),source_checks_passed=source_pass,
                unchanged_source_files=equality,selected_stop_feedback_statements_equal=statements,source_line_evidence='source_lines.json',
                actual=mapped,metadata_before=before,source_inputs_unchanged=before==[meta(p) for p in paths],
                no_EDA=True,no_SHA=True,no_full_memory_or_clock_audit=True,
                limits='Source statement identity and exact selected binding equality do not prove timing or every changed model ready/feedback cone. Any actual mapping difference remains explicit.')
    save(a.out/'source_lines.json',evidence);save(a.out/'result.json',result)
    print(json.dumps(dict(source_checks_passed=source_pass,source_files=len(fixed),actual_status=mapped['status'],out=str(a.out)),ensure_ascii=False))
    assert source_pass,'See exact source difference evidence'


if __name__=='__main__':main()
