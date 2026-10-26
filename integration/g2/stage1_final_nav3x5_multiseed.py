"""Final 9211 task: frozen N1c d-selection, 128 production; 256 model control only."""
from pathlib import Path
from datetime import datetime
import argparse,csv,difflib,inspect,json,math,os,shutil,subprocess,sys,time,traceback
import psutil
import stage1a_nav3x5_window_ablation as A
import stage1d_nav3x5_forward_windows as D
from prepare_baseline_fusion_retime import metadata,same_bytes

OUT=A.BASE/'nav3x5_N1c_final128_multiseed_20261026'
TASK=Path('C:/Users/Li/.codex/attachments/9211a5dd-3f64-4969-bdb7-cffbe1c53825/已粘贴的文本.txt')
CASES=('goal','ramp','step','turn_loop')

def entry():
 text=D.forward_entry()
 target='assert start==[.45,-.95,0.0]'
 assert text.count(target)==1
 return text.replace(target,'assert len(start)==3 and all(math.isfinite(v)for v in start) # Four original task starts preserved')

def geometry():
 sys.path.insert(0,str(OUT/'execution_tree/model'));import nav_fixed as nf
 rows=[]
 for mm in(1000,1250,1500):
  for yaw in(0,8192,16384,24576,32768,40960,49152,57344,65535):
   item=D.forward_shift(yaw,mm,1311,nf.SIN)
   ideal=[mm*item['cos_q14']/16384,mm*item['sin_q14']/16384]
   assert max(abs(a-b)for a,b in zip(ideal,item['actual_offset_odometry_mm']))<=65536/1311/2+1e-9
   assert D.forward_shift(yaw,0,1311,nf.SIN)['offset_cells']==[0,0]
   rows.append(item)
 for mm,n in((1000,20),(1250,25),(1500,30)):
  for yaw,want in((0,[n,0]),(16384,[0,n]),(32768,[-n,0]),(49152,[0,-n])):assert D.forward_shift(yaw,mm,1311,nf.SIN)['offset_cells']==want
 value=dict(passed=True,rows=rows,zero_shift_identity=True,scope='27 selector geometries plus cardinal and zero identities; no navigation acceptance',
  inherited_full1024_SIN_table_proof=str(D.OUT/'actual_heading_table_contract.json'),heading='real DLL py_get_state()[3] latest estimated yaw at each LF rebuild',
  center='original EM center + only LF offset; EM/score rotation pivot/UPDATE unchanged',quantization='original Q14 SIN and symmetric nearest cells, ties away zero')
 A.write(OUT/'geometry_contract.json',value);return value

def prepare():
 assert not OUT.exists(),'No duplicate final stage1 preparation'
 OUT.mkdir();shutil.copytree(A.OLD/'execution_tree',OUT/'execution_tree',ignore=shutil.ignore_patterns('__pycache__'))
 shutil.copytree(A.OLD/'firmware_source',OUT/'firmware_source')
 for name in('nav_fw.dll','scene.json','cases.json'):shutil.copyfile(A.OLD/name,OUT/name)
 shutil.copyfile(TASK,OUT/'final_task.txt')
 tree=OUT/'execution_tree';original=(A.OLD/'execution_tree/model/slf_match.py').read_text(encoding='utf8')
 derived=A.derive_model(original).replace('assert LF_WINDOW_MODE in ("rolling", "fixed_field")','assert LF_WINDOW_MODE in ("rolling", "fixed_field", "forward")')
 (tree/'model/slf_match.py').write_text(derived,encoding='utf8')
 directory=tree/'sim/closed_loop';(directory/'final_forward_case.py').write_text(entry(),encoding='utf8')
 # Keep the exact old prior algorithm and packet bytes. Only cache namespace changes.
 p=directory/'stage1_case.py';before=p.read_text(encoding='utf8')
 after=before.replace("cache=ROOT.parent/'prior_cache'/scene_name/f'sn{slf_match.SN}'","cache=ROOT.parent/'prior_cache'/scene_name/f'sn{slf_match.SN}'/'fixed_prior_ZREL-300_1500'")
 assert before!=after;p.write_text(after,encoding='utf8')
 for pkg in(A.OLD/'prior_cache').rglob('*'):
  if pkg.is_file():
   rel=pkg.relative_to(A.OLD/'prior_cache');target=OUT/'prior_cache'/rel.parent/'fixed_prior_ZREL-300_1500'/rel.name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(pkg,target);assert same_bytes(pkg,target)
 (OUT/'source_delta.patch').write_text(''.join(difflib.unified_diff(original.splitlines(True),derived.splitlines(True),fromfile='N1c/slf_match.py',tofile='final_stage1/slf_match.py')),encoding='utf8')
 (OUT/'entry_delta.patch').write_text(''.join(difflib.unified_diff((D.OUT/'execution_tree/sim/closed_loop/stage1d_case.py').read_text(encoding='utf8').splitlines(True),entry().splitlines(True),fromfile='proved_stage1d_entry',tofile='four_task_entry')),encoding='utf8')
 (OUT/'cache_delta.patch').write_text(''.join(difflib.unified_diff(before.splitlines(True),after.splitlines(True),fromfile='N1c/stage1_case.py',tofile='cache_namespace_only/stage1_case.py')),encoding='utf8')
 old=A.read(A.OLD/'manifest.json');forward=A.read(D.OUT/'manifest.json');jobs=[];reuse=[];audit=[]
 profiles=[('control256',256,False,0),('forward1000',128,False,1000),('forward1250',128,False,1250),('forward1500',128,False,1500),('prior256',256,True,0),('prior128',128,True,0)]
 for seed in(1,2,3):
  for profile,sn,prior,mm in profiles:
   for case in CASES:
    template=next(j for j in old['jobs']if j['case']==case and j['SLF_SN']==sn and j['prior']==prior)
    name=f'{case}_{profile}_s{seed}';out=OUT/'results'/name;command=list(template['command']);command[3]=str(directory/('final_forward_case.py'if mm else'stage1_case.py'))
    command[command.index('--firmware-dll')+1]=str(OUT/'nav_fw.dll');command[command.index('--seed')+1]=str(seed);command[command.index('--out')+1]=str(out)
    env=dict(SLF_SN_LOG2=str(sn.bit_length()-1),SLF_LF_N=str(sn),SLF_WINDOW_MODE='forward'if mm else'rolling',SLF_FORWARD_MM=str(mm),SLF_FIXED_CX='41',SLF_FIXED_CY='19',SLF_ZLO='60',SLF_NBR='3x3',SLF_W18='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1')
    item=dict(name=name,case=case,profile=profile,SLF_SN=sn,prior=prior,forward_mm=mm,seed=seed,ZHI=1500,command=command,environment=env,out=str(out),result_source='new_run',cache_namespace=f'field3x5_v1/SN{sn}/LF{sn}_{"fixed_prior"if prior else "forward"+str(mm)if mm else"rolling"}/ZREL{(-300 if prior else 60)}_1500')
    reused=None
    if seed==1 and not mm:reused=template
    if seed==1 and mm==1500 and case in('goal','turn_loop'):
     reused=next(j for j in forward['jobs']if j['case']==case and j['environment']['SLF_FORWARD_MM']=='1500')
    if reused:
     result=Path(reused['out']);assert(result/'metrics.json').exists()
     actual=A.read(result/'manifest.json');assert actual['arguments']['seed']==seed and actual['arguments']['scene']=='field3x5_v1'
     assert actual['arguments']['start']==template['command'][template['command'].index('--start')+1]
     assert same_bytes(Path(actual['firmware_dll']),OUT/'nav_fw.dll')
     assert same_bytes(A.OLD/'execution_tree/sim/closed_loop/run_cl.py',tree/'sim/closed_loop/run_cl.py')
     item.update(result_source='reused_same_inputs',out=str(result),reused_manifest=str(result/'manifest.json'))
     reuse.append(item);audit.append(dict(name=name,old_output=str(result),same_DLL_bytes=True,same_scene_bytes=True,same_original_runtime=True,same_CLI_except_paths=True,same_seed=True,
      forward_semantics='same Q14/estimated heading/window/score pivot; new entry only permits additional task starts'if mm else'original same profile'))
    jobs.append(item)
 assert len(jobs)==72 and len(reuse)==14
 preserved=[]
 for p in(A.OLD/'execution_tree').rglob('*'):
  if p.is_file()and'__pycache__'not in p.parts and p.relative_to(A.OLD/'execution_tree').as_posix()not in('model/slf_match.py','sim/closed_loop/stage1_case.py'):
   assert same_bytes(p,tree/p.relative_to(A.OLD/'execution_tree'));preserved.append(str(p))
 assert same_bytes(A.OLD/'scene.json',OUT/'scene.json')and same_bytes(A.OLD/'nav_fw.dll',OUT/'nav_fw.dll')
 manifest=dict(recorded_at=datetime.now().astimezone().isoformat(),final_task=str(TASK),task_frozen_copy=str(OUT/'final_task.txt'),SLF128_production_fixed=True,SLF256_control_only=True,no_B_fallback=True,
  source_N1c=str(A.OLD),scene='field3x5_v1',ZHI=1500,DLL_rebuilt=False,production_model_or_RTL_modified=False,planned=72,reused=14,new_runs=58,jobs=jobs,reused_inputs_audit=audit,
  cases=list(CASES),seeds=[1,2,3],nomap_offsets_mm=[1000,1250,1500],prior='fixed original prior window, offsets not varied',prior_cache_algorithm_and_bytes_preserved=True,
  gate=dict(valid_XY_P95_mm=50,no_added_collision=True,no_localization_loss=True,long_patrol_seconds=300,sustained_drift='not numerically specified; report absolute rolling metrics/trend, do not invent pass threshold'),
  arrival_reported_not_selection_gate=True,max_error_diagnostic_only=True,old_relative10mm_gate_superseded=True,
  scope='N1c C DLL + fixedpoint perception + virtual lidar + velocity kinematics; SAFE/RL not in loop; fixed40ms matcher latency, not RTL/WCET/board',
  unchanged_frozen_files=preserved,bindings=[metadata(p)for p in tree.rglob('*')if p.is_file()and'__pycache__'not in p.parts]+[metadata(OUT/'nav_fw.dll')],automatic_EDA=False)
 A.write(OUT/'manifest.json',manifest);A.write(OUT/'reuse_and_pending_matrix.json',dict(total=72,reused=14,pending=58,jobs=[{k:j[k]for k in('name','case','profile','seed','prior','forward_mm','SLF_SN','result_source','out')}for j in jobs]))
 A.write(OUT/'job.json',dict(phase='prepared',planned=72,reused=14,completed=14,new_completed=0,pending_new=58,automatic_EDA=False))
 geometry();return manifest

def row(j):
 p=Path(j['out']);v=A.row(p);metrics=A.read(p/'metrics.json')
 v.update(n_lost=metrics['n_lost'],t_lost_s=metrics['t_lost_s'],loc_valid_fraction=metrics['loc_valid_fraction'],loc_final=metrics['loc_final'],align_s=metrics['align_s'],n_align=metrics['n_align'],n_align_fail=metrics['n_align_fail'],simulation_end_s=metrics['simulation_end_s'])
 v['P95_le50']=v['XY_valid']['P95_mm']<=50;v['no_localization_loss']=v['n_lost']==0 and v['t_lost_s']==0
 return v

def summarize(manifest_override=None):
 if manifest_override is None:
  state=A.read(OUT/'job.json')if(OUT/'job.json').exists()else{}
  manifest_override=A.read(Path(state['active_manifest']))if state.get('active_manifest')else None
 manifest=manifest_override or A.read(OUT/'manifest.json');rows=[]
 for j in manifest['jobs']:
  if not(Path(j['out'])/'metrics.json').exists():continue
  v=row(j);control=next(b for b in manifest['jobs']if b['seed']==j['seed']and b['case']==j['case']and b['profile']==('prior256'if j['prior']else'control256'))
  pair=None
  if(Path(control['out'])/'metrics.json').exists():
   old=row(control);pair=dict(control=control['out'],control_collision=old['collisions'],no_added_collision=v['collisions']<=old['collisions'],control_n_lost=old['n_lost'],
    delta_XY_valid={k:v['XY_valid'][k]-old['XY_valid'][k]for k in('RMSE_mm','P95_mm','endpoint_mm','max_mm')})
  rows.append(dict(name=j['name'],profile=j['profile'],case=j['case'],seed=j['seed'],prior=j['prior'],SLF_SN=j['SLF_SN'],forward_mm=j['forward_mm'],result_source=j['result_source'],result=v,paired_control=pair))
 profiles={}
 for profile in('control256','forward1000','forward1250','forward1500','prior256','prior128'):
  subset=[r for r in rows if r['profile']==profile];profiles[profile]=dict(completed=len(subset),planned=12,P95_le50_k_n=[sum(r['result']['P95_le50']for r in subset),len(subset)],
   no_localization_loss_k_n=[sum(r['result']['no_localization_loss']for r in subset),len(subset)],collisions=sum(r['result']['collisions']for r in subset),
   real_arrival_k_n=[sum(r['result']['arrived']for r in subset),len(subset)],false_arrival=sum(r['result']['false_arrival']for r in subset),
   worst_valid_P95_mm=max((r['result']['XY_valid']['P95_mm']for r in subset),default=None))
 value=dict(recorded_at=datetime.now().astimezone().isoformat(),completed=len(rows),planned=72,profiles=profiles,rows=rows,gate=manifest['gate'],production128=True,
  SLF256_control_only=True,old10mm_falsearrival_selection_superseded=True,long_patrol_complete=False,full_stage1_complete=False,automatic_EDA=False)
 A.write(OUT/'summary.json',value);return value

def run():
 manifest=A.read(OUT/'manifest.json');assert A.read(OUT/'geometry_contract.json')['passed']
 for b in manifest['bindings']:assert metadata(Path(b['path']))==b,b['path']
 with(OUT/'run_claim.json').open('x')as f:json.dump(dict(pid=os.getpid(),created=psutil.Process().create_time()),f)
 pending=[j for j in manifest['jobs']if j['result_source']=='new_run'];me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS);me.cpu_affinity([18,19]);complete=0
 def status(phase,**kw):A.write(OUT/'job.json',dict(phase=phase,updated_at=datetime.now().astimezone().isoformat(),controller_pid=me.pid,controller_created_epoch=me.create_time(),
  planned=72,reused=14,new_completed=complete,completed=14+complete,pending_new=58-complete,one_worker=True,priority='BELOW_NORMAL',logical_CPUs=[18,19],minimum_start_physical_bytes=2*A.GIB,minimum_start_commit_bytes=3*A.GIB,automatic_EDA=False,**kw))
 summarize()
 try:
  for j in pending:
   assert not Path(j['out']).exists(),'Do not repeat prior attempt/output'
   while True:
    memory=A.memory_status();status('waiting_memory',next=j['name'],free_bytes=memory.avail_phys,commit_bytes=memory.avail_page)
    if memory.avail_phys>=2*A.GIB and memory.avail_page>=3*A.GIB:break
    time.sleep(10)
   path=Path(j['out']);path.mkdir(parents=True);env=os.environ.copy();env.update(j['environment'])
   with(path/'stdout.log').open('w',encoding='utf8')as f:
    child=subprocess.Popen(j['command'],env=env,stdout=f,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
    p=psutil.Process(child.pid);p.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS);p.cpu_affinity([18,19]);peak_rss=peak_private=0;low_since=None
    status('running_case',current=j['name'],child_pid=child.pid,child_created_epoch=p.create_time(),command=j['command'],environment=j['environment'],admission_physical_bytes=memory.avail_phys,admission_commit_bytes=memory.avail_page)
    while child.poll()is None:
     try:mem=p.memory_info();peak_rss=max(peak_rss,mem.rss);peak_private=max(peak_private,getattr(mem,'private',mem.vms))
     except psutil.NoSuchProcess:pass
     memory=A.memory_status();low=memory.avail_phys<256*1024**2 and memory.avail_page<A.GIB
     low_since=time.monotonic()if low and low_since is None else low_since if low else None
     if low_since is not None and time.monotonic()-low_since>=30:
      child.terminate();child.wait();raise RuntimeError('Only new worker stopped by original sustained low-memory guard')
     time.sleep(3)
   A.write(path/'runtime.json',dict(exit=child.returncode,peak_rss=peak_rss,peak_private=peak_private,child_pid=child.pid))
   assert child.returncode==0,(path/'stdout.log').read_text(encoding='utf8');assert(path/'metrics.json').exists();complete+=1;summarize();status('case_complete',last=j['name'])
  status('multiseed_complete_long_patrol_pending',summary=str(OUT/'summary.json'));return 0
 except Exception:status('failed',error=traceback.format_exc());raise

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--prepare',action='store_true');p.add_argument('--run',action='store_true');p.add_argument('--summary',action='store_true');a=p.parse_args()
 if a.prepare:print(json.dumps(dict(root=str(OUT),jobs=len(prepare()['jobs'])),ensure_ascii=False))
 if a.run:raise SystemExit(run())
 if a.summary:print(json.dumps(summarize(),ensure_ascii=False,indent=2))
