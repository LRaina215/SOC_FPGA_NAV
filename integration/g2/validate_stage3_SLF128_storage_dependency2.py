"""Correct only the simulation compilation closure; preserve probe1 and all RTL/TB."""
from pathlib import Path
import argparse,json,shutil
from prepare_baseline_fusion_retime import load,save,metadata,same_bytes
from validate_stage3_SLF128_storage_regression import run,unchanged

ROOT=Path('F:/nav_integration_g2_20261023')
OLD_INPUTS=ROOT/'stage3_SLF128_storage_regression_inputs1'
FAILED=ROOT/'stage3_SLF128_storage_regression_probe1'

def prepare(inputs):
 inputs=inputs.resolve();assert not inputs.exists()
 failed=load(FAILED/'result.json');assert not failed['passed'] and len(failed['runtimes'])==1
 text=(FAILED/'0_xvlog_stdout.log').read_text(errors='replace')
 assert "identifier 'current_value' is used before its declaration"in text and 'oled.v:281'in text
 oldrows=load(OLD_INPUTS/'frozen_bindings.json')['inputs'];unchanged(oldrows)
 shutil.copytree(OLD_INPUTS,inputs)
 entry=load(inputs/'entry.json');source63=load(Path(entry['candidate'])/'manifest.json')['source_files']
 stop=next(i for i,p in enumerate(source63)if Path(p).as_posix().endswith('/nav/rtl/top/nav_subsystem.v'))
 closure=source63[:stop+1];assert len(closure)==31
 oldargs=entry['commands'][0][1];assert oldargs[3:3+63]==source63
 entry['commands'][0][1]=oldargs[:3]+closure+oldargs[3+63:]
 entry.update(compiled_source_count=31,metadata_bound_hardware_source_count=63,
  compilation_scope='Actual nav_subsystem and standalone SLF/RAM closure. OLED, CPU, board top, MAC and control are not elaborated.',
  dependency_correction_only=True,failed_probe_preserved=str(FAILED),RTL_and_TB_unchanged=True)
 save(inputs/'entry.json',entry)
 copied=[]
 for p in OLD_INPUTS.iterdir():
  if p.is_file()and p.name not in ('entry.json','frozen_bindings.json','job.json'):
   assert same_bytes(p,inputs/p.name);copied.append(p.name)
 correction=dict(reason='Unused frozen OLED source uses late declarations rejected by Vivado SystemVerilog frontend; no xelab/xsim entered.',
  prior_output=str(FAILED),all63_hardware_inputs_unchanged=True,all_TB_and_vector_bytes_unchanged=copied,
  original_compile63=True,corrected_compile31=closure,CPU_and_other_unelaborated_source_remains_bound=True,
  physical_collision_or_full_CPU_proof=False,automatic_retry=False)
 save(inputs/'dependency_correction.json',correction)
 bound={}
 for row in oldrows:
  p=Path(row['path'])
  if p.is_relative_to(OLD_INPUTS):p=inputs/p.relative_to(OLD_INPUTS)
  bound[str(p)]=metadata(p)
 for p in (Path(__file__).resolve(),inputs/'dependency_correction.json'):bound[str(p)]=metadata(p)
 save(inputs/'frozen_bindings.json',dict(inputs=list(bound.values()),source_binding_count=248,final_new_FW_assets_bound=True))
 save(inputs/'job.json',dict(phase='dependency2_inputs_frozen_not_started',candidate=entry['candidate'],hardware_bound=63,compiled_source_count=31,RTL_started=False))
 return entry

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--inputs',type=Path,required=True);p.add_argument('--out',type=Path);p.add_argument('--prepare',action='store_true');p.add_argument('--run',action='store_true');a=p.parse_args();assert a.prepare!=a.run
 if a.prepare:
  r=prepare(a.inputs);print(json.dumps(dict(candidate=r['candidate'],bound=63,compiled=r['compiled_source_count'],TB_unchanged=True),ensure_ascii=False))
 else:
  assert a.out;code=run(a.inputs,a.out)
  result=load(a.out/'result.json');result.update(compiled_source_count=31,hardware_metadata_bound=63,dependency_correction=str(a.inputs/'dependency_correction.json'))
  save(a.out/'result.json',result);save(a.out/'job.json',result);raise SystemExit(code)
