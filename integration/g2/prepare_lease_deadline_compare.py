"""Prepare an isolated exact32bit deadline rewrite; no SYN/PnR is started."""
from pathlib import Path
from datetime import datetime
import argparse, difflib, json, shutil
from prepare_baseline_fusion_retime import metadata, save, same_bytes

BASE=Path('F:/nav_integration_g2_20261023')
DEFAULT_SOURCE=BASE/'baseline_fusion_model25_NAV50_prepared4'
OLD=" wire deadline=tick && have_lease && age_ms+1>=TIMEOUT_MS;"
# TIMEOUT_MS is an integer, age_ms is unsigned32. Preserve its exact32bit
# comparison semantics, including the all-ones increment wrapping to zero.
# Keep the original arithmetic for timeout0 so X/Z simulation behavior also
# remains unchanged; the normal production parameter is200, not zero.
NEW=""" localparam [31:0] LEASE_TIMEOUT_U32=TIMEOUT_MS;
 wire lease_age_due;
 generate if(LEASE_TIMEOUT_U32==32'd0)begin:g_deadline_zero
  assign lease_age_due=(age_ms+32'd1)>=LEASE_TIMEOUT_U32;
 end else begin:g_deadline_nonzero
  assign lease_age_due=(age_ms>=LEASE_TIMEOUT_U32-32'd1) &&
                       (age_ms!=32'hffffffff);
 end endgenerate
 wire deadline=tick && have_lease && lease_age_due;"""

def rewrite(text):
 assert text.count(OLD)==1
 result=text.replace(OLD,NEW)
 assert result.replace(NEW,OLD)==text
 return result

def prepare(source,out):
 source=source.resolve();out=out.resolve()
 assert source.is_dir()and not out.exists()
 assert out.parent==BASE.resolve()and out.name.startswith('baseline_fusion_model25_deadline_compare_')
 review=json.loads((source/'prepared_source_review.json').read_text())
 assert review['prepared']and review['out']==str(source)
 for row in review['bindings']:assert metadata(Path(row['path']))==row
 original=(source/'command_lease_guard.v').read_bytes().decode('utf-8')
 assert '\r'not in original,'Preserve source newlines explicitly rather than normalize them'
 changed=rewrite(original)
 shutil.copytree(source,out)
 (out/'command_lease_guard.v').write_text(changed,encoding='utf-8',newline='')
 # Bind selected inputs to the new candidate, rather than its unchanged parent.
 run=(out/'run.tcl').read_text(encoding='utf-8')
 run=run.replace(source.as_posix(),out.as_posix()).replace(str(source),str(out))
 (out/'run.tcl').write_text(run,encoding='utf-8',newline='\n')
 # Preserve inherited model25 review as provenance, then issue a review for
 # this new one-file source candidate. Do not present the inherited proof as
 # proof of the new RTL or physical timing.
 (out/'prepared_source_review.json').rename(out/'inherited_model25_prepared_source_review.json')
 (out/'lease_deadline_delta.patch').write_text(''.join(difflib.unified_diff(
  original.splitlines(True),changed.splitlines(True),fromfile='prepared4/command_lease_guard.v',
  tofile='deadline_compare/command_lease_guard.v')),encoding='utf-8',newline='')
 manifest=json.loads((source/'manifest.json').read_text())
 manifest['source_files']=[str(out/Path(p).relative_to(source))for p in manifest['source_files']]
 for p in manifest['source_files']:
  assert '{'+Path(p).as_posix()+'}' in run,p
 assert source.as_posix()not in run and str(source)not in run
 manifest.update(candidate='model25_NAV50_exact_deadline_compare',frozen_source_build=str(source),
  source_preparation_only=True,automatic_pnr=False,deployment_accepted=False,synthesis_complete=False,
  additional_modified_rtl_inputs=['command_lease_guard.v'])
 save(out/'manifest.json',manifest)
 # The ModGen directory also carries an unselected model25_pll_tmp.v template.
 # Hardware inputs come from the actual manifest, not an rglob of auxiliaries.
 rtl=[Path(p).relative_to(out)for p in manifest['source_files']]
 assert len(rtl)==66,len(rtl)
 changed_rtl=[str(n).replace('\\','/')for n in rtl if not same_bytes(source/n,out/n)]
 assert changed_rtl==['command_lease_guard.v'],changed_rtl
 assets=[p.relative_to(source)for p in source.rglob('*')if p.is_file()and p.suffix.lower()in{'.hex','.mem','.mif','.bin'}]
 assert len(assets)==69 and all(same_bytes(source/n,out/n)for n in assets)
 headers=[p.relative_to(source)for p in source.rglob('*')if p.is_file()and p.suffix.lower()in{'.h','.vh'}]
 assert len(headers)==31 and all(same_bytes(source/n,out/n)for n in headers)
 assert same_bytes(source/'g2.sdc',out/'g2.sdc')and same_bytes(source/'g2.cst',out/'g2.cst')
 result=dict(prepared=True,recorded_at=datetime.now().astimezone().isoformat(),source=str(source),out=str(out),
  only_changed_RTL=changed_rtl,unchanged_selected_RTL=65,selected_RTL=66,
  unselected_modgen_template_byte_identical=same_bytes(source/'model25_pll/model25_pll_tmp.v',out/'model25_pll/model25_pll_tmp.v'),
  identical_assets=69,identical_headers=31,
  original_firmware_and_all_weights_byte_identical=True,clock_CST_SDC_capacities_unchanged=True,
  selected_run_Tcl_paths_bound_to_new_candidate=True,
  original_counting_saturation_ramp_output_cycles_and_priority_unchanged=True,
  production_timeout_ms=200,age_unsigned_bits=32,timeout_integer_bits=32,
  arithmetic_domain='All2^32 known age values and all2^32 integer timeout bit patterns; normal200ms. Nonzero parameters use compare199 plus explicit MAX exclusion; timeout0 retains original32bit arithmetic to preserve X/Z behavior.',
  full_state_math_proof_pending=True,targeted_cycle_comparison_pending=True,
  new_mapping_or_timing_benefit_verified=False,balanced_reduction_not_guaranteed=True,
  new_SYN_PNR_started=False,programmed=False,deployment_accepted=False,
  bindings=[metadata(out/'command_lease_guard.v'),metadata(out/'manifest.json'),metadata(out/'g2.sdc'),metadata(out/'g2.cst'),metadata(out/'run.tcl')])
 save(out/'prepared_source_review.json',result)
 return result

if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--source',type=Path,default=DEFAULT_SOURCE);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
 print(json.dumps(prepare(a.source,a.out),indent=2))
