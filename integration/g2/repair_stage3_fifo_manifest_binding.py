"""Repair only the demonstrated manifest metadata ordering error in prepared6.

This is not a general metadata refresher.  It refuses any other stale input or
unexpected manifest semantic difference, preserves all four original records,
and never changes a Verilog/FW/asset/manifest byte.
"""
from pathlib import Path
from datetime import datetime
import argparse,json,shutil
from prepare_baseline_fusion_retime import metadata,relocate,same_bytes,save
BASE=Path('F:/nav_integration_g2_20261023');SRC=BASE/'stage3_SLF128_storage_RTL_prepared5';CAND=BASE/'stage3_SLF128_storage_RTL_prepared6';OUT=BASE/'stage3_FIFO_manifest_binding_repair1'
TABLE=[('composition_review.json','bindings'),('prepared_source_review.json','complete_frozen_input_bindings'),('firmware_binding_review.json','complete_frozen_input_bindings'),('FIFO_syntax_rebase_review.json','complete_frozen_input_bindings')]
def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def repair():
 assert not OUT.exists();OUT.mkdir();manifest=CAND/'manifest.json';actual_manifest=metadata(manifest)
 a=relocate(load(SRC/'manifest.json'),SRC,CAND);b=load(manifest)
 keys=[k for k in set(a)|set(b)if a.get(k)!=b.get(k)]
 assert set(keys)=={'modified_rtl_inputs','FIFO_reset_syntax_candidate'},keys
 assert b['modified_rtl_inputs']==load(CAND/'composition_review.json')['changed_RTL_relative_to_SYN40']
 assert Path(b['FIFO_reset_syntax_candidate'])==BASE/'stage3_FIFO_async_reset_syntax_candidate1'
 snapshots={};changes=[];preserved_inputs={}
 for filename,key in TABLE:
  p=CAND/filename;r=load(p);snapshots[filename]=r;shutil.copyfile(p,OUT/('before_'+filename))
  delta=[(i,v,metadata(Path(v['path'])))for i,v in enumerate(r[key])if metadata(Path(v['path']))!=v]
  assert len(delta)==1 and Path(delta[0][1]['path'])==manifest,(filename,delta)
  i,old,actual=delta[0];assert actual==actual_manifest and old['bytes']==8701
  changes.append(dict(file=filename,entry_index=i,old=old,actual=actual))
  for row in r[key]:
   path=Path(row['path'])
   if path!=manifest:preserved_inputs[path]=path.read_bytes()
  r[key][i]=actual_manifest
 save(OUT/'original_failure.json',dict(passed=False,error='Only manifest metadata captured before the final expected manifest update',manifest_semantic_changed_keys=keys,actual_mismatches=changes))
 # Changes to these JSON records are proof-only.  Their explicit metadata
 # references are updated in dependency order, never by a blind list refresh.
 save(CAND/'composition_review.json',snapshots['composition_review.json'])
 save(CAND/'prepared_source_review.json',snapshots['prepared_source_review.json'])
 f=snapshots['firmware_binding_review.json'];f['composition_review']=metadata(CAND/'composition_review.json');f['source_review']=metadata(CAND/'prepared_source_review.json');save(CAND/'firmware_binding_review.json',f)
 r=snapshots['FIFO_syntax_rebase_review.json'];r['composition_review']=metadata(CAND/'composition_review.json');r['source_review']=metadata(CAND/'prepared_source_review.json');r['manifest']=actual_manifest;save(CAND/'FIFO_syntax_rebase_review.json',r)
 assert metadata(manifest)==actual_manifest
 for p,data in preserved_inputs.items():assert p.read_bytes()==data,p
 for filename,key in TABLE:
  r=load(CAND/filename)
  for row in r[key]:assert metadata(Path(row['path']))==row,(filename,row)
  for ref in ['composition_review','source_review','manifest']:
   if ref in r and isinstance(r[ref],dict)and'path'in r[ref]:assert metadata(Path(r[ref]['path']))==r[ref],(filename,ref)
 result=dict(passed=True,at=datetime.now().astimezone().isoformat(),candidate=str(CAND),only_wrong_input_metadata='manifest.json',corrected_records=changes,manifest_bytes_unchanged=True,other247_input_bytes_unchanged=True,source_RTL_FW_assets_changed=False,EDA_or_tests_repeated=False,original_failure_preserved=str(OUT/'original_failure.json'),working_firmware_binding_review=metadata(CAND/'firmware_binding_review.json'),working_source_review=metadata(CAND/'prepared_source_review.json'),working_rebase_review=metadata(CAND/'FIFO_syntax_rebase_review.json'),full_input_count=len(load(CAND/'firmware_binding_review.json')['complete_frozen_input_bindings']))
 save(OUT/'result.json',result);print(json.dumps(dict(passed=True,candidate=str(CAND),input_count=result['full_input_count'],only_wrong_entry='manifest.json',report=str(OUT/'result.json')),ensure_ascii=False));return result
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--repair-known-metadata-only',action='store_true');a=p.parse_args();assert a.repair_known_metadata_only;repair()
