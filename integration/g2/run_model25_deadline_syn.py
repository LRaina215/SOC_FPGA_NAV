"""Unique SYN35 for the all-state-proved one-expression deadline candidate."""
from pathlib import Path
import json,re
import run_model25_joint_syn as run

BASE=Path('F:/nav_integration_g2_20261023')
PARENT=BASE/'baseline_fusion_model25_NAV50_syn34'
PROVED=BASE/'baseline_fusion_model25_deadline_compare_prepared3'
run.PREP=BASE/'baseline_fusion_model25_deadline_compare_prepared4'
run.OUTPUT=BASE/'baseline_fusion_model25_deadline_compare_syn35'
run.CONTROL=BASE/'baseline_fusion_model25_deadline_compare_one_shot'
run.BUILD_LABEL='SYN35'

def source_check():
    prep=run.PREP
    proof=json.loads((prep/'prepared_source_review.json').read_text())
    assert proof['prepared'] and proof['only_changed_RTL']==['command_lease_guard.v']
    assert proof['selected_run_Tcl_paths_bound_to_new_candidate']
    for row in proof['bindings']:assert run.metadata(Path(row['path']))==row
    a,b=run.hardware(PARENT),run.hardware(prep)
    assert a.keys()==b.keys() and len(a)==66
    assert [n for n in a if not run.same_bytes(a[n],b[n])]==['command_lease_guard.v']
    assert run.same_bytes(prep/'command_lease_guard.v',PROVED/'command_lease_guard.v')
    math_path=BASE/'lease_deadline_compare_math3/result.json'
    cycle_path=BASE/'lease_deadline_compare_cycle2/result.json'
    math=json.loads(math_path.read_text());cycle=json.loads(cycle_path.read_text())
    assert math['passed'] and math['no_reachability_assumption']
    assert len(math['proofs'])==2 and all(p['status']=='unsat'for p in math['proofs'])
    for row in math['input_bindings']:assert run.metadata(Path(row['path']))==row
    assert cycle['passed'] and cycle['compared_cycles']==3123 and cycle['math_parameter_XZ_cases']==225
    for suffixes,count in [({'.hex','.mem','.mif','.bin'},69),({'.h','.vh'},31)]:
        old,new=run.inputs(PARENT,suffixes),run.inputs(prep,suffixes)
        assert old.keys()==new.keys() and len(new)==count
        assert all(run.same_bytes(old[n],new[n])for n in old)
    for d in ['fw','firmware']:
        old={p.relative_to(PARENT/d):p for p in(PARENT/d).rglob('*')if p.is_file()and'__pycache__'not in p.parts}
        new={p.relative_to(prep/d):p for p in(prep/d).rglob('*')if p.is_file()and'__pycache__'not in p.parts}
        assert old.keys()==new.keys()and all(run.same_bytes(old[n],new[n])for n in old)
    assert all(run.same_bytes(PARENT/n,prep/n)for n in ['g2.cst','g2.sdc'])
    tcl=(prep/'run.tcl').read_text()
    selected=re.findall(r'^add_file -type verilog \{(.+)\}$',tcl,re.M)
    assert {Path(p).resolve()for p in selected}=={p.resolve()for p in b.values()}
    assert len(selected)==66 and all(prep.resolve()in Path(p).resolve().parents for p in selected)
    assert str(PROVED)not in tcl and PROVED.as_posix()not in tcl
    return dict(reference=str(PARENT),changed_RTL=['command_lease_guard.v'],identical_selected_RTL=65,
                identical_assets=69,identical_headers=31,identical_complete_original_FW=True,
                new_candidate_RTL_byte_identical_to_proved_candidate=True,
                math_proof=str(math_path),cycle_proof=str(cycle_path),
                Tcl_input_rebinding_from_preserved_prepared3=True,
                actual_mapping_or_physical_benefit_proven=False,
                bindings=[run.metadata(p)for p in b.values()])

run.source_check=source_check
if __name__=='__main__':raise SystemExit(run.main())
