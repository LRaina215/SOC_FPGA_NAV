"""Freeze final source8 with only the proved common-enable weight change."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import shutil
from prepare_baseline_fusion_retime import relocate
from review_stage3_slf128_memories import meta


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    for n in('source','candidate','RTL_review','small_review','official_review','out'):ap.add_argument('--'+n.replace('_','-'),type=Path,required=True,dest=n)
    a=ap.parse_args();assert not a.out.exists()
    for p in(a.RTL_review,a.small_review,a.official_review):assert json.loads((p/'result.json').read_text())['passed'],p
    shutil.copytree(a.source,a.out,ignore=shutil.ignore_patterns('__pycache__','impl','*.log','job.json','tool_runtime.json'))
    shutil.copyfile(a.candidate/'fixed4500_weight_rom.v',a.out/'fixed4500_weight_rom.v')
    rewrite=['manifest.json','composition_review.json','firmware_binding_review.json']
    proofs=dict(weight_common_enable_RTL_review=meta(a.RTL_review/'result.json'),weight_common_enable_small_C_review=meta(a.small_review/'result.json'),weight_common_enable_official_primitive_review=meta(a.official_review/'result.json'))
    for name in rewrite:
        x=relocate(json.loads((a.source/name).read_text()),a.source,a.out)
        for k in('bindings','complete_frozen_input_bindings'):
            if k in x:
                assert not any(Path(v['path']).name==name for v in x[k]),'Input bindings must not self-reference their containing review JSON'
                x[k]=[meta(Path(v['path']))for v in x[k]]
        x.update(candidate=str(a.out),ancestor_before_weight_common_enable=str(a.source),weight_high_common_enable_repair=True,
                 weight_repair_only_relative_to_ancestor=['fixed4500_weight_rom.v'],weight_repair_proofs=proofs,
                 automatic_SYN=False,automatic_PNR=False,programmed=False)
        if name=='manifest.json':x.update(EDA_authorized=False,stage='prepared_only',synthesis_complete=False)
        if name=='composition_review.json':x['final_for_EDA']=False
        (a.out/name).write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    changed=[];same=[];metadata=[]
    for p in a.source.rglob('*'):
        if not p.is_file()or'__pycache__'in p.parts:continue
        rel=p.relative_to(a.source);q=a.out/rel
        if not q.exists():continue
        if rel.as_posix()in rewrite:metadata.append(rel.as_posix());continue
        if p.read_bytes()==q.read_bytes():same.append(rel.as_posix())
        else:changed.append(rel.as_posix())
    assert changed==['fixed4500_weight_rom.v'],changed
    manifest=json.loads((a.out/'manifest.json').read_text());rtl=[Path(p).relative_to(a.out).as_posix()for p in manifest['source_files']];assert len(rtl)==63
    assert all((a.source/p).read_bytes()==(a.out/p).read_bytes()for p in rtl if p!='fixed4500_weight_rom.v')
    r=dict(prepared=True,candidate=str(a.out),ancestor=str(a.source),recorded_at=datetime.now().astimezone().isoformat(),
           changed_RTL_relative_to7=changed,final_byte_identical_hardware_assets_FW_to7=same,
           rebound_review_metadata_files=metadata,byte_identical_62_of_63_RTL=True,firmware_BIN_bytes=53572,
           repaired_source=meta(a.out/'fixed4500_weight_rom.v'),BIN=meta(a.out/'firmware/nav_fw.bin'),proofs=proofs,
           LF_source_byte_identical_to7=True,LF_repair_proof_binding=meta(a.source/'LF_repair_binding_review.json'),
           old_failure_preserved=dict(source42_public_counterexample='F:/nav_integration_g2_20261023/SLF128_actual_weights_official_primitive_probe1/xsim.log',read_address=16385,expected_and_original_RTL='0176',actual42='3976'),
           exact_change='Only15high ROM read always conditions bank-qualified en become common en; no INIT/weights/ports/bitwidth/public cycle changes',
           source_all65536_fourstate_contract_passed=True,actual_small_C_35ROM_INIT_and_CE_contract_passed=True,official_mapped_all40704_plus_idle_passed=True,
           exploratory_joint_source_only=True,complete_deployment_candidate=False,root_source_gate_pending=True,automatic_SYN=False,automatic_PNR=False,programmed=False)
    (a.out/'weight_repair_binding_review.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps({k:v for k,v in r.items()if k!='final_byte_identical_hardware_assets_FW_to7'},ensure_ascii=False))


if __name__=='__main__':main()
