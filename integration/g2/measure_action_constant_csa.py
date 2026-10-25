"""One isolated module SYN after exact RTL proof; no implementation/PNR."""
from pathlib import Path
from datetime import datetime
import json,sys,time,xml.etree.ElementTree as ET,argparse
import psutil
sys.path.insert(0,str(Path(__file__).resolve().parent))
from measure_slf_row_banks import TOOL,run_syn,inventory
from preflight import validate
from retry_frozen_synthesis import memory_status

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--candidate-dir',type=Path,required=True)
parser.add_argument('--out',type=Path,required=True)
parser.add_argument('--origin-synthesis',type=Path,default=Path('F:/nav_integration_g2_20261023/baseline_fusion_only_syn23'))
parser.add_argument('--resume-settings-only',action='store_true')
parser.add_argument('--wait-for-memory',action='store_true',help='wait after imports, before the single tool launch; no SYN runtime budget starts yet')
args=parser.parse_args()
HERE=args.candidate_dir.resolve();OUT=args.out.resolve()
me=psutil.Process();me.nice(psutil.IDLE_PRIORITY_CLASS);me.cpu_affinity([16])
proof=json.loads((HERE/'result.json').read_text(encoding='utf-8'))
assert proof['passed'] and proof['frames']==8770 and proof['exhaustive_scale_values_checked']==32772
assert Path(proof['candidate']).resolve()==(HERE/'policy_action_map.v').resolve()
for binding in proof['source_bindings']:
 path=Path(binding['path']).resolve();stat=path.stat()
 assert binding==dict(path=str(path),bytes=stat.st_size,mtime_ns=stat.st_mtime_ns),('proved source changed',str(path))
source=(HERE/'policy_action_map.v').read_text(encoding='utf-8')
module_text=source.replace('module policy_action_map #(','module g2_top #(',1)
clock_text='create_clock -name clk -period 20 [get_ports {clk}]\n'
settings=['set_device -device_version C GW5AT-LV138PG484AC1/I0',
 f'add_file -type verilog {{{(OUT/"action.v").as_posix()}}}',
 f'add_file -type sdc {{{(OUT/"clock.sdc").as_posix()}}}',
 'set_option -top_module g2_top','set_option -output_base_name g2_top',
 'set_option -verilog_std v2001','set_option -global_freq 50',
 'set_option -convert_sdp32_36_to_sdp16_18 1','set_option -looplimit 100000']
run_text='\n'.join(settings+['run syn','exit'])+'\n'
if args.resume_settings_only:
 assert OUT.is_dir() and not (OUT/'impl').exists()
 assert not any((OUT/n).exists() for n in ('gw_sh.log','module_runtime.json','launch_claim.json')),'SYN already started; never repeat'
 assert (OUT/'action.v').read_text(encoding='utf-8')==module_text
 assert (OUT/'clock.sdc').read_text(encoding='utf-8')==clock_text
 assert (OUT/'run.tcl').read_text(encoding='utf-8')==run_text
 assert (OUT/'effective_options.tcl').is_file() and (OUT/'preflight.log').is_file()
 options=dict(reused_settings_only_readback=str(OUT/'effective_options.tcl'),text=(OUT/'effective_options.tcl').read_text(encoding='utf-8'))
else:
 assert not OUT.exists(),'module SYN output already exists; only explicit unstarted settings resume is permitted'
 m=memory_status()
 assert m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3,'2/4GiB real-time launch gate'
 OUT.mkdir()
 (OUT/'action.v').write_text(module_text,encoding='utf-8',newline='\n')
 (OUT/'clock.sdc').write_text(clock_text,encoding='utf-8')
 options=validate(TOOL,OUT,settings,50,1)
 (OUT/'run.tcl').write_text(run_text,encoding='utf-8')
# Recheck in this already-loaded child, so importing the audit helpers cannot
# consume an unaccounted reserve between the parent gate and actual launch.
frozen_files=[HERE/'result.json',HERE/'policy_action_map.v']+[OUT/name for name in
              ('action.v','clock.sdc','run.tcl','preflight.log','effective_options.tcl')]
def bindings():
 result=[]
 for path in frozen_files:
  stat=path.stat();result.append(dict(path=str(path),bytes=stat.st_size,mtime_ns=stat.st_mtime_ns))
 return result
frozen=bindings()
while True:
 m2=memory_status()
 if m2.avail_phys>=2*1024**3 and m2.avail_page>=4*1024**3:break
 assert args.wait_for_memory,'2/4GiB launch recheck'
 assert bindings()==frozen,'frozen input changed while waiting'
 state=dict(phase='waiting_for_2_4GiB_after_imports_before_unique_module_SYN',pid=me.pid,
  created_epoch=me.create_time(),available_physical_bytes=m2.avail_phys,available_commit_bytes=m2.avail_page,
  minimum_physical_bytes=2*1024**3,minimum_commit_bytes=4*1024**3,SYN_started=False,input_bindings=frozen)
 temporary=OUT/'admission_wait.json.tmp';temporary.write_text(json.dumps(state,indent=2),encoding='utf-8');temporary.replace(OUT/'admission_wait.json')
 time.sleep(13)
assert bindings()==frozen
with (OUT/'launch_claim.json').open('x',encoding='utf-8') as file:
 json.dump(dict(pid=me.pid,created_at=me.create_time(),source=str(HERE/'policy_action_map.v'),scope='one isolated module SYN only'),file)
runtime=run_syn(OUT,scope='isolated action constant CSA module only; not joint SYN/PNR')
metrics=ET.parse(OUT/'impl/gwsynthesis/g2_top_syn_rsc.xml').getroot().attrib
inv=inventory(OUT/'impl/gwsynthesis/g2_top.vg')
dsps={k:v for k,v in inv['primitives'].items() if any(w in k for w in ('MULT','PADD','ALU54','MAC'))}
origin=ET.parse(args.origin_synthesis/'impl/gwsynthesis/g2_top_syn_rsc.xml').getroot()
def find(el,path):
 for name in path:
  el=next(e for e in el if e.attrib.get('name')==name)
 return el
old=find(origin,['u_control','model_thread','actuator_map']).attrib
result=dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),proof=str(HERE/'result.json'),
 source=str(HERE/'policy_action_map.v'),module_source_matches_proof_except_top_name=True,
 startup_available_physical_bytes=m2.avail_phys,startup_available_commit_bytes=m2.avail_page,
 runtime=runtime,effective_options=options,resources=metrics,inventory=inv,
 DSP_primitives=dsps,target_action_DSP_removed_in_isolated_mapping=not dsps,
 original_full_joint_action_scope_resources=old,physical_timing_or_joint_mapping_proved=False,
 joint_synthesis_or_PNR_started=False,programmed=False,
 caveat='Module SYN can establish this isolated mapped arithmetic contains no DSP, not that a future full joint mapping/placement retains it or closes50MHz.')
(OUT/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k not in ('inventory','effective_options')},ensure_ascii=False,indent=2))
