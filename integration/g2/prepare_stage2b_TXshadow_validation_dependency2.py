"""Add omitted frozen nav_fifo/nav_fifo_ob dependencies to a new test copy.

Hardware, wrappers and TB expressions are byte-identical; this launches no tool.
The prior xelab failure and original candidate/validation remain unchanged.
"""
from pathlib import Path
from datetime import datetime
import json,shutil
from prepare_baseline_fusion_retime import metadata,same_bytes

BASE=Path('F:/nav_integration_g2_20261023')
CAND=BASE/'stage2B_TXshadow_RDEN_prepared2'
OUT=BASE/'stage2B_TXshadow_validation_dependency2'

def prepare():
 assert not OUT.exists()
 review=json.loads((CAND/'prepared_source_review.json').read_text())
 for row in review['hardware_bindings']+review['validation_bindings']:assert metadata(Path(row['path']))==row
 failure=json.loads((BASE/'stage2B_TXshadow_bus_probe1/result.json').read_text());assert not failure['passed']
 assert 'Module <nav_fifo> not found'in(BASE/'stage2B_TXshadow_bus_probe1/1_xelab_stdout.log').read_text()
 old=CAND/'validation';shutil.copytree(old,OUT)
 entry=json.loads((old/'entry.json').read_text());extras=['nav_fifo.v','nav_fifo_ob.v']
 for name in extras:
  source=CAND/'nav/rtl/common'/name
  assert same_bytes(source,BASE/'baseline_fusion50_ring_MAX_syn40/nav/rtl/common'/name)
  shutil.copyfile(source,OUT/name)
 entry['files']=entry['files'][:-1]+extras+entry['files'][-1:]
 entry['commands'][0][1]=['-sv',*entry['files']]
 entry.update(validation_dependency_correction_only=True,prior_failure_preserved=str(BASE/'stage2B_TXshadow_bus_probe1'),new_dependencies=extras,hardware_wrappers_and_TB_expressions_unchanged=True)
 (OUT/'entry.json').write_text(json.dumps(entry,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
 same=[p.name for p in old.iterdir()if p.is_file()and p.name!='entry.json']
 assert all(same_bytes(old/n,OUT/n)for n in same)
 record=dict(prepared=True,recorded_at=datetime.now().astimezone().isoformat(),candidate=str(CAND),validation=str(OUT),prior_failure_preserved=True,added_dependency_files=extras,all_other_wrappers_TB_and_data_bytes_equal=True,unchanged_files=same,entry_changed_only_compile_file_list=True,source_RTL_or_TB_expressions_modified=False,EDA_started=False)
 (OUT/'preparation.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf8')
 bindings=[metadata(p)for p in sorted(OUT.iterdir())if p.is_file()]
 (OUT/'input_bindings.json').write_text(json.dumps(dict(inputs=bindings),indent=2)+'\n',encoding='utf8')
 print(json.dumps(record,ensure_ascii=False))

if __name__=='__main__':prepare()
