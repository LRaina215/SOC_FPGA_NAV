"""Freeze the proved row-read change as a complete, unbuilt joint candidate."""
import argparse
import copy
from datetime import datetime
import json
from pathlib import Path
import shutil

from measure_row_read_context import reference_snapshot
from prepare_row_read_share import transform

RAM='nav/rtl/common/nav_ram_sdp.v'
ASSETS={'.hex','.mem','.bin','.mif'}


def prepare(source,out,protected,proof):
    assert not out.exists() and out!=source and source not in out.parents and out not in source.parents
    manifest=json.loads((source/'manifest.json').read_text())
    source_job=json.loads((source/'job.json').read_text())
    assert source_job['phase']=='synthesis_complete' and source_job['tool_exit']==0
    assert manifest['small_block_stores_selected'] and not manifest.get('row_read_shared')
    assert not manifest['control_weights_changed'] and manifest['clock_core_mhz']==50
    local=json.loads((proof/'result.json').read_text())
    context=Path(local['prior'])
    context_state=json.loads((context/'result.json').read_text())
    assert Path(context_state['source']).resolve()==source
    assert local['phase']=='local_50MHz_passed_without_extra_extractor_globals'
    assert context_state['context_behavior'].startswith('TEST PASSED cycles=55812 valid_points=13615')
    assert all(x['internal_clock_50MHz_pass'] and x['global_assignments_match_budget'] and not x['extractor_extra_global_resources_used'] for x in local['results'].values())
    bare=source.parent/'row_read_share_probe1'
    candidate=transform((source/RAM).read_text(encoding='utf-8'))
    assert candidate==(bare/'candidate.v').read_text(encoding='utf-8')==(context/'shared/ram.v').read_text(encoding='utf-8')
    assert (context/'shared/extract.v').read_bytes()==(source/'nav/rtl/match/slf_extract.v').read_bytes()
    hardware=[Path(n).relative_to(source) for n in manifest['source_files']]
    assert len(hardware)==len(set(hardware))==68
    assets={p.relative_to(source) for p in source.rglob('*') if p.is_file() and p.suffix.lower() in ASSETS and 'impl' not in p.relative_to(source).parts}
    assert len(assets)==104
    headers={p.relative_to(source) for p in source.rglob('*') if p.is_file() and p.suffix.lower() in {'.vh','.h'} and 'impl' not in p.relative_to(source).parts}
    extra={Path('g2.cst'),Path('g2.sdc'),Path('ip/serdes.csr')}
    extra.update(p.relative_to(source) for p in (source/'ip').iterdir() if p.is_file() and p.suffix.lower() in {'.toml','.ipc','.mod','.json'})
    out.mkdir()
    for relative in set(hardware)|assets|headers|extra:
        target=out/relative;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(source/relative,target)
    (out/RAM).write_text(candidate,encoding='utf-8',newline='\n')
    changed=[p.as_posix() for p in hardware if (source/p).read_bytes()!=(out/p).read_bytes()]
    assert changed==[RAM]
    assert all((source/p).read_bytes()==(out/p).read_bytes() for p in assets)
    def rewrite(value):
        if isinstance(value,str):return value.replace(source.as_posix(),out.as_posix()).replace(str(source),str(out))
        if isinstance(value,list):return [rewrite(v) for v in value]
        if isinstance(value,dict):return {k:rewrite(v) for k,v in value.items()}
        return value
    new_manifest=rewrite(copy.deepcopy(manifest));new_manifest.pop('effective_options',None)
    new_manifest.update(row_read_shared=True,row_read_shared_base_build=str(source),
        row_read_shared_smallblock_origin=manifest['frozen_source_build'],row_read_shared_context_proof=str(proof/'result.json'),
        row_read_shared_bare_proof=str(bare/'result.json'),row_read_shared_prepared_from=str(source),
        programmed=False,complete_sfp_cdc_acceptance=False)
    (out/'manifest.json').write_text(json.dumps(new_manifest,indent=2)+'\n',encoding='utf-8')
    original=(source/'run.tcl').read_text()
    assert [line.strip() for line in original.splitlines() if line.strip().startswith('run ')]==['run syn']
    (out/'run.tcl').write_text(rewrite(original),encoding='utf-8',newline='\n')
    protected_snapshot=reference_snapshot(protected)
    state=dict(created_at=datetime.now().astimezone().isoformat(),phase='prepared_only_waiting_for_PNR19_result_review',
        source=str(source),prepared=str(out),source_files=len(hardware),unchanged_hardware_inputs=67,
        modified_rtl_inputs=changed,unchanged_assets=len(assets),copied_headers=len(headers),
        original_IO_SDC_CSR_unchanged=all((source/p).read_bytes()==(out/p).read_bytes() for p in extra),
        context_proof=str(proof/'result.json'),context_50MHz_without_extra_global_passed=True,
        predicted_physical_BSRAM=333,physical_is_prediction=True,
        original_4500_weights_125Hz_CPU_and_capacity_unchanged=True,measurement_fixture_not_in_candidate=True,
        protected_joint_build=protected_snapshot,SYN_started=False,PNR_started=False,programmed=False,
        decision_required_after_current_result=True,scope='complete frozen source candidate only; no joint netlist or timing result yet')
    (out/'prepared_source_review.json').write_text(json.dumps(state,indent=2)+'\n',encoding='utf-8')
    (out/'job.json').write_text(json.dumps(state,indent=2)+'\n',encoding='utf-8')
    return state


def main():
    p=argparse.ArgumentParser()
    for key in ('source','out','protected-pnr','proof'):p.add_argument('--'+key,type=Path,required=True)
    a=p.parse_args();r=prepare(a.source.resolve(),a.out.resolve(),a.protected_pnr.resolve(),a.proof.resolve())
    print(json.dumps({k:v for k,v in r.items() if k!='protected_joint_build'}))


if __name__=='__main__':main()
