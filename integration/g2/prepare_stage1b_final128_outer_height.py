"""Freeze final1b pilot; explicit best-d binding and root-controlled later launch."""
from pathlib import Path
from datetime import datetime
import argparse,difflib,inspect,json,os,shutil,sys
import stage1a_nav3x5_window_ablation as A
import stage1d_nav3x5_forward_windows as D
import prepare_stage1_final_long_patrol as L
from prepare_baseline_fusion_retime import metadata,same_bytes

OLD=A.BASE/'nav3x5_N1c_stage1c_outer_height_20261026'
OUT=A.BASE/'nav3x5_N1c_final128_outer_height_stage1b_20261026'
SELECTED=L.OUT/'selected_best_distance.json'
TASK=Path('C:/Users/Li/.codex/attachments/9211a5dd-3f64-4969-bdb7-cffbe1c53825/已粘贴的文本.txt')

ENTRY=r'''from pathlib import Path
import atexit,ctypes,json,os,sys
import stage1_case as S
R=S.R
import slf_match as M
import stage1c_support as C
from stage1b_window_math import forward_shift
root=Path(__file__).resolve().parents[3]
spec=json.loads((root/'outer_scene.json').read_text(encoding='utf8'))
ledger=C.install(R,M,spec,root)
ledger['stage']='final1b_best128_forward_height';ledger['forward_windows']=[]
assert M.LF_N==M.SN
prior='--prior'in sys.argv
assert M.LF_WINDOW_MODE==('forward'if M.SN==128 and not prior else'rolling')
bound=json.loads((root/'bound_manifest.json').read_text(encoding='utf8'))
assert int(os.environ['SLF_FORWARD_MM'])==(bound['selected_offset_mm']if M.SN==128 and not prior else 0)
ledger['selected_offset_mm']=bound['selected_offset_mm'];ledger['selection_source']=bound['selected_source']
instances=[];original_factory=R.Firmware
def firmware_factory():
 value=original_factory();instances.append(value);return value
R.Firmware=firmware_factory
original_build=M.StructMap.build_lf
def build(self,cx,cy):
 if M.LF_WINDOW_MODE=='forward':
  assert not self.prior and len(instances)==1
  state=(ctypes.c_int32*12)();instances[0].py_get_state(state)
  shift=forward_shift(int(state[3])&65535,int(os.environ['SLF_FORWARD_MM']),self.inv_res,M.nf.SIN)
  original_build(self,cx+shift['offset_cells'][0],cy+shift['offset_cells'][1])
  ledger['forward_windows'].append(dict(original_EM_center_cells=[int(cx),int(cy)],actual_LF_center_cells=[int(self.lf[0]+M.LF_N//2),int(self.lf[1]+M.LF_N//2)],
   latest_public_pose_mm=[int(state[0]),int(state[1])],shift=shift,heading_source='real C nt.pose.cur.yaw synchronously at LF rebuild after solve correction',
   original_score_rotation_pivot_preserved=True))
 else:original_build(self,cx,cy)
M.StructMap.build_lf=build
out=Path(sys.argv[sys.argv.index('--out')+1])
def finish():
 out.mkdir(parents=True,exist_ok=True)
 (out/'height_window_audit.json').write_text(json.dumps(ledger,ensure_ascii=False,indent=2,default=lambda x:int(x)if hasattr(x,'item')else str(x))+'\n',encoding='utf8')
atexit.register(finish)
if __name__=='__main__':raise SystemExit(R.main())
'''

def static_geometry(spec):
 # Pure box/ROI and original-raster embedding invariants; no lidar/closed-loop rerun.
 original=A.read(OLD/'outer_scene.json');assert spec['inner_scene']==original['inner_scene']
 assert spec['outer_primitives'][:-2]==original['outer_primitives']
 rows=[]
 for p in spec['outer_primitives']:
  x0,y0,x1,y1,h=p['args'];dx=max(0,-x1,x0-5);dy=max(0,-1.5-y1,y0-1.5);distance=(dx*dx+dy*dy)**.5
  assert (x1<=-.5 or x0>=5.5 or y1<=-2 or y0>=2)and .5<=distance<=3.01
  assert h>0 and x0>=-10 and x1<=10 and y0>=-10 and y1<=10
  maximum=max((max(0,-x,x-5)**2+max(0,-1.5-y,y-1.5)**2)**.5 for x,y in((x0,y0),(x0,y1),(x1,y0),(x1,y1)))
  rows.append(dict(name=p['name'],bounds_m=[x0,y0,x1,y1],height_m=h,nearest_distance_to_inner_m=distance,furthest_corner_distance_to_inner_m=maximum,inside_ROI_overlap=False))
 proof=dict(passed=True,scope='Static geometry/configuration only, no new ray or closed-loop simulation',inner_scene_dictionary_equal=True,
  original_scene_file_bytes_equal=True,old10_external_objects_preserved=True,new_human_prisms=2,outer_boxes=rows,
  inherited_minimal_proof=str(OLD/'minimal_proof.json'),
  inherited_actual_proof='Original7 legal ground/platform poses:2050 actual exterior valid hits;1500 selected2014 and350 selected0; true C op5 and prior ROI pass for SN128/256.',
  new_human_visibility_measured=False,new_scene_full_filtering_pass_not_claimed=True,
  analytical_bound='For original reachable foot<=84mm,350mm relative upper ends<=434mm absolute, below500mm inner boards. Original unobstructed straight ray geometry required; pose/range quantization and new object hits audited in actual next runs.',
  prior_crop_mm=[0,-1500,5000,1500],platform_provisional_mm=84,mount_height_mm=155,mount_pitch_down_deg=10,
  immutable_current_controller45668_inputs=True)
 A.write(OUT/'static_scene_contract.json',proof);return proof

def prepare():
 assert not OUT.exists(),'Do not duplicate final1b preparation'
 assert A.read(OLD/'minimal_proof.json')['passed']
 OUT.mkdir();shutil.copytree(OLD/'execution_tree',OUT/'execution_tree',ignore=shutil.ignore_patterns('__pycache__'))
 shutil.copytree(OLD/'firmware_source',OUT/'firmware_source')
 for name in('nav_fw.dll','scene.json','cases.json'):shutil.copyfile(OLD/name,OUT/name)
 shutil.copyfile(TASK,OUT/'final_task.txt')
 spec=A.read(OLD/'outer_scene.json');old_scene=json.dumps(spec,ensure_ascii=False,indent=2)+'\n'
 spec.update(id='field3x5_outer_v2_human',version=2)
 spec['outer_primitives'].extend([dict(kind='box',name='person_east',args=[6.0,1.35,6.35,1.7,1.7]),
  dict(kind='box',name='person_north',args=[3.0,2.35,3.35,2.7,1.7])])
 spec['human_model']='Solid vertical350x350mm heightfield prisms,1700mm tall; no articulated/occlusion mesh claims.'
 A.write(OUT/'outer_scene.json',spec)
 tree=OUT/'execution_tree';model=tree/'model/slf_match.py';original=model.read_text(encoding='utf8')
 derived=original.replace('assert LF_WINDOW_MODE in ("rolling", "fixed_field")','assert LF_WINDOW_MODE in ("rolling", "fixed_field", "forward")');assert derived!=original
 model.write_text(derived,encoding='utf8')
 directory=tree/'sim/closed_loop';(directory/'stage1b_case.py').write_text(ENTRY,encoding='utf8')
 (directory/'stage1b_window_math.py').write_text(inspect.getsource(D.symmetric_round)+'\n'+inspect.getsource(D.forward_shift),encoding='utf8')
 support=directory/'stage1c_support.py';old_support=support.read_text(encoding='utf8')
 new_support=old_support.replace("requested_EM_center=[int(cx),int(cy)]","requested_LF_input_center=[int(cx),int(cy)]").replace('scene_version=1,','scene_version=spec[\'version\'],')
 assert new_support!=old_support;support.write_text(new_support,encoding='utf8')
 for name,a,b in(('source',original,derived),('entry',(OLD/'execution_tree/sim/closed_loop/stage1c_case.py').read_text(encoding='utf8'),ENTRY),
  ('support',old_support,new_support),('scene',old_scene,json.dumps(spec,ensure_ascii=False,indent=2)+'\n')):
  (OUT/f'{name}_delta.patch').write_text(''.join(difflib.unified_diff(a.splitlines(True),b.splitlines(True),fromfile=f'passed_stage1c/{name}',tofile=f'final1b/{name}')),encoding='utf8')
 old=A.read(OLD/'manifest.json');jobs=[]
 for case in('goal','turn_loop'):
  for prior in(False,True):
   for hi in(1500,350):
    for sn in(256,128):
     source=next(j for j in old['jobs']if j['case']==case and j['prior']==prior and j['ZHI']==hi and j['SLF_SN']==sn)
     name=f'{case}_sn{sn}_{"prior_fixed"if prior else"forward_best"if sn==128 else"rolling_reference"}_zhi{hi}_s1'
     command=list(source['command']);command[3]=str(directory/'stage1b_case.py');command[command.index('--scene')+1]=spec['id'];command[command.index('--firmware-dll')+1]=str(OUT/'nav_fw.dll');command[command.index('--out')+1]=str(OUT/'results'/name)
     env=dict(source['environment'],SLF_WINDOW_MODE='forward'if sn==128 and not prior else'rolling',SLF_FORWARD_MM='SELECTED_BEST_MM'if sn==128 and not prior else'0')
     jobs.append(dict(name=name,case=case,prior=prior,SLF_SN=sn,ZHI=hi,seed=1,command=command,environment=env,out=str(OUT/'results'/name),
      selected_offset_required=sn==128 and not prior,cache_namespace=f'{spec["id"]}/SN{sn}/LF{sn}_{"fixed_prior"if prior else"forward_SELECTED"if sn==128 else"rolling"}/ZREL{-300 if prior else 60}_{hi}'))
 assert len(jobs)==16 and same_bytes(OLD/'nav_fw.dll',OUT/'nav_fw.dll')and same_bytes(OLD/'scene.json',OUT/'scene.json')
 preserved=[]
 for p in(OLD/'execution_tree').rglob('*'):
  relative=p.relative_to(OLD/'execution_tree').as_posix()
  if p.is_file()and'__pycache__'not in p.parts and relative not in('model/slf_match.py','sim/closed_loop/stage1c_support.py'):
   assert same_bytes(p,tree/relative);preserved.append(str(p))
 manifest=dict(recorded_at=datetime.now().astimezone().isoformat(),final_task=str(TASK),stage='final1b pilot only; prepare, do not start',production128_fixed=True,SLF256_control_only=True,no_B_fallback=True,
  scene=spec,source_passed_stage1c=str(OLD),inherited_minimal_proof=str(OLD/'minimal_proof.json'),best_distance_source=str(SELECTED),best_distance_pending=not SELECTED.exists(),
  planned=16,completed=0,jobs=jobs,DLL_rebuilt=False,production_model_or_RTL_modified=False,current_multiseed_or_patrol_inputs_modified=False,
  cold_configuration='Every16 profile receives same actual original MAP_CLEAR plus1 original py_poll at1000us before sensors/goal/lidar; LUT generation and op5 requested/applied logged.',
  actual_C_height_dispatch='Same passed ctypes64bit callback from stage1c. Intercept every real C op5 request and retain ZLO, apply explicit350/1500, assert actual model zlo/zhi.',
  prior='Same selected upper for prior structure extraction and all-points survey zcap; crop both XY arrays to[0,-1500,5000,1500] before original package build. Prior fixed LFO, no forward offset.',
  cache='Fresh namespace byscene v2/SN/LFfixed/ZLO/ZHI/innerROI/survey seed3/noise4/lidar seed11; no stage1c/prior caches copied.',
  gate='P95<=50mm/no added collision/no localization loss; record no-map observability limitation. Arrival/falsearrival reported, not d/size selection.',
  new_run_baselines=True,old_inner_only_or_old_scene_results_not_reused_as_new_environment_baselines=True,
  scope='Frozen N1c C DLL + fixedpoint perception + virtual lidar + velocity kinematics; SAFE/RL not in loop; fixed40ms matcher assumption, not RTL/board/WCET.',
  geometry_proof=str(OUT/'static_scene_contract.json'),unchanged_stage1c_files=preserved,bindings=[metadata(p)for p in tree.rglob('*')if p.is_file()and'__pycache__'not in p.parts]+[metadata(OUT/'nav_fw.dll'),metadata(OUT/'outer_scene.json'),metadata(OUT/'final_task.txt')],automatic_EDA=False)
 A.write(OUT/'manifest.json',manifest);A.write(OUT/'job.json',dict(phase='frozen_plan_waiting_real_selected_distance_and_root_start',planned=16,completed=0,actual_model_process_started=False,selected_distance_source=str(SELECTED),automatic_EDA=False))
 static_geometry(spec);return manifest

def bind():
 assert SELECTED.exists(),'Best distance not yet published; do not guess'
 assert not(OUT/'bound_manifest.json').exists(),'Bound input is immutable; do not overwrite'
 manifest=A.read(OUT/'manifest.json')
 for b in manifest['bindings']:assert metadata(Path(b['path']))==b,b['path']
 selected=A.read(SELECTED);mm=int(selected['selected_offset_mm']);assert mm in(1000,1250,1500)
 jobs=manifest['jobs']
 for j in jobs:
  if j['selected_offset_required']:j['environment']['SLF_FORWARD_MM']=str(mm);j['cache_namespace']=j['cache_namespace'].replace('forward_SELECTED',f'forward{mm}mm')
 selected_copy=OUT/'selected_best_distance_frozen.json';shutil.copyfile(SELECTED,selected_copy);assert same_bytes(SELECTED,selected_copy)
 bound=dict(recorded_at=datetime.now().astimezone().isoformat(),selected_offset_mm=mm,selected_source=str(SELECTED),selected_source_metadata=metadata(SELECTED),selection_frozen_copy=str(selected_copy),
  selection=selected,planned=16,jobs=jobs,source_manifest=str(OUT/'manifest.json'),model_bindings=manifest['bindings'],actual_model_process_started=False,
  root_must_check_current_processes_and_memory_before_start=True,automatic_EDA=False)
 A.write(OUT/'bound_manifest.json',bound);A.write(OUT/'job.json',dict(phase='distance_bound_not_started_waiting_root_process_memory_review',selected_offset_mm=mm,planned=16,completed=0,actual_model_process_started=False,automatic_EDA=False));return bound

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--prepare',action='store_true');p.add_argument('--bind',action='store_true');a=p.parse_args()
 if a.prepare:print(json.dumps(dict(root=str(OUT),planned=prepare()['planned']),ensure_ascii=False))
 if a.bind:print(json.dumps(dict(root=str(OUT),selected_offset_mm=bind()['selected_offset_mm']),ensure_ascii=False))
