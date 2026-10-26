"""Freeze only the reviewed LF read-enable change on final storage source6."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import shutil

from prepare_baseline_fusion_retime import relocate
from review_stage3_slf128_memories import meta


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--source',type=Path,required=True);ap.add_argument('--candidate',type=Path,required=True);ap.add_argument('--validation',type=Path,required=True);ap.add_argument('--small-review',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    proof=json.loads((a.validation/'result.json').read_text());small=json.loads((a.small_review/'result.json').read_text());assert proof['passed']and small['passed']
    assert not a.out.exists();shutil.copytree(a.source,a.out,ignore=shutil.ignore_patterns('__pycache__','impl','*.log','job.json','tool_runtime.json'))
    rel='nav/rtl/match/slf_lf.v';shutil.copyfile(a.candidate/'slf_lf.v',a.out/rel)
    changed=[];same=[]
    for p in a.source.rglob('*'):
        if not p.is_file()or '__pycache__'in p.parts:continue
        relative=p.relative_to(a.source);q=a.out/relative
        if q.exists()and p.read_bytes()!=q.read_bytes():changed.append(relative.as_posix())
        elif q.exists():same.append(relative.as_posix())
    assert changed==[rel],changed
    # Rebind only candidate-input metadata. Historical proof paths remain
    # absolute references to the untouched sources and completed regressions.
    for name in ('manifest.json','composition_review.json','firmware_binding_review.json'):
        original=json.loads((a.source/name).read_text());new=relocate(original,a.source,a.out)
        fields=('bindings','complete_frozen_input_bindings')
        for field in fields:
            if field in new:new[field]=[meta(Path(b['path']))for b in new[field]]
        new.update(candidate=str(a.out),ancestor_before_LF_read_gate=str(a.source),LF_array_read_enable_repair=True,
                   LF_repair_only_relative_to_ancestor=[rel],LF_RTL_validation=str(a.validation/'result.json'),LF_small_C_mapping_review=str(a.small_review/'result.json'),
                   automatic_SYN=False,automatic_PNR=False,programmed=False)
        if name=='manifest.json':new.update(EDA_authorized=False,stage='prepared_only',synthesis_complete=False,firmware_rebuild_pending=False)
        if name=='composition_review.json':
            new['final_for_EDA']=False
            if rel not in new['changed_RTL_relative_to128']:new['changed_RTL_relative_to128'].append(rel)
            if rel not in new['changed_RTL_relative_to_SYN40']:new['changed_RTL_relative_to_SYN40'].append(rel)
        (a.out/name).write_text(json.dumps(new,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    r=dict(prepared=True,candidate=str(a.out),ancestor=str(a.source),recorded_at=datetime.now().astimezone().isoformat(),changed_RTL=[rel],all_other_copied_inputs_byte_identical=same,
           LF_source=meta(a.out/rel),BIN=meta(a.out/'firmware/nav_fw.bin'),LF_RTL_validation=meta(a.validation/'result.json'),LF_small_C_mapping_review=meta(a.small_review/'result.json'),
           reuse_combined_scope='Unchanged CPU/FW/IMU/FIFO/storage-branch tests remain historical source-bound evidence; changed LF separately replays all five128 model frames/245 candidates/7EVAL cycles plus old/new natural-X reset miter. No unchanged-source PNR/STA result is inherited.',
           root_source_gate_pending=True,actual_SLF128_packed_weight_counterexample_pending=True,automatic_SYN=False,automatic_PNR=False,programmed=False)
    (a.out/'LF_repair_binding_review.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf8');print(json.dumps({k:v for k,v in r.items()if k!='all_other_copied_inputs_byte_identical'},ensure_ascii=False))


if __name__=='__main__':main()
