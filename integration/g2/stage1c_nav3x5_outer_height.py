"""Frozen stage1c: exterior geometry x height band; prepare first, explicit run later."""
from pathlib import Path
import argparse,difflib,json,os,shutil,subprocess,sys,time
import traceback
from datetime import datetime
import psutil
import stage1a_nav3x5_window_ablation as A
from prepare_baseline_fusion_retime import metadata,same_bytes
from retry_frozen_synthesis import memory_status

OUT=A.BASE/'nav3x5_N1c_stage1c_outer_height_20261026'
REPO=A.REPO

ENTRY=r'''from pathlib import Path
import atexit,json,os,sys
import stage1_case as S
R=S.R
import slf_match as M
import stage1c_support as C
root=Path(__file__).resolve().parents[3]
spec=json.loads((root/'outer_scene.json').read_text(encoding='utf8'))
ledger=C.install(R,M,spec,root)
out=Path(sys.argv[sys.argv.index('--out')+1])
def finish():
 out.mkdir(parents=True,exist_ok=True)
 (out/'height_window_audit.json').write_text(json.dumps(ledger,ensure_ascii=False,indent=2,default=lambda x:int(x)if hasattr(x,'item')else str(x))+'\n',encoding='utf8')
atexit.register(finish)
if __name__=='__main__':raise SystemExit(R.main())
'''

PROBE=r'''from pathlib import Path
import ctypes,json,math,os,sys
import numpy as np
import stage1_case as S
R=S.R
import slf_match as M
import stage1c_support as C
root=Path(__file__).resolve().parents[3]
spec=json.loads((root/'outer_scene.json').read_text(encoding='utf8'))
ledger=C.install(R,M,spec,root)
R.DLL=root/'nav_fw.dll'
scene=C.make_scene(R,spec);inner=C.make_scene(R,spec,False)
shift=scene.inner_embedding_shift_cells;old=scene.original_inner_world
assert np.array_equal(scene.w.h[shift:shift+old.n,shift:shift+old.n][
 int(round((-1.55-old.o)/old.res)):int(round((1.55-old.o)/old.res)),
 int(round((-.05-old.o)/old.res)):int(round((5.05-old.o)/old.res))],
 old.h[int(round((-1.55-old.o)/old.res)):int(round((1.55-old.o)/old.res)),
 int(round((-.05-old.o)/old.res)):int(round((5.05-old.o)/old.res))])
x,y=np.meshgrid(np.arange(-.05,5.051,.017),np.arange(-1.55,1.551,.019))
assert np.array_equal(scene.height(x,y),old.height(x,y));inner_samples=int(x.size)
model=R.fpga_model.FpgaModel();fw=R.Firmware()
CB=ctypes.CFUNCTYPE(None,ctypes.c_int,ctypes.c_int32,ctypes.c_int32,ctypes.c_int32)
@CB
def ctl(op,a,b,c):model.ctl(op,a,b,c)
fw.py_set_ctl_cb(ctl)
records=[];all_sp=[];all_pts=[]
poses=[(.45,-.95,0),(2.5,-.6,0),(2.5,-.6,math.pi),(4.45,.95,math.pi),
       (2.5,-.6,math.pi/2),(2.5,-.6,-math.pi/2),(1.45,.8,0)]
for x,y,yaw in poses:
 z=float(scene.height(np.array([x]),np.array([y]))[0]);assert z<=.0841, "Static sample must be on reachable ground or original84mm platform"
 Rws,tws=R.sw.compose(R.sw.rotz(yaw),np.array([x,y,z]),*R.sw.mount_pose())
 lidar=R.sw.VirtualLidar(noise_mm=0,seed=11,rng_max=8,step=.015)
 depth,_=lidar.capture(scene.w,Rws,tws)
 innerdepth,_=R.sw.VirtualLidar(noise_mm=0,seed=11,rng_max=8,step=.015).capture(inner.w,Rws,tws)
 model.frame(depth)
 rt=list(np.round(Rws*16384).astype(np.int64).reshape(-1))+list(np.round(tws*1000).astype(np.int64))
 p=R.fpga_model.fast.pc_transform(model.xyz,np.array(rt[:9]).reshape(3,3),np.array(rt[9:]))
 outside=~C.inside_xy(p,[0,-1500,5000,1500]);changed=(depth!=innerdepth).reshape(-1)
 zref=int(rt[11])-155
 m1500=M.structure_mask.__wrapped__(p,model.pv,zref,60,1500)if hasattr(M.structure_mask,'__wrapped__')else None
 # Use the frozen implementation directly for the two-band paired static sample.
 base_mask=M.structure_mask_original
 masks={hi:base_mask(p,model.pv,zref,60,hi)for hi in(350,1500)}
 actual=model.struct_points(rt)
 expected=p[M.subsample(np.nonzero(masks[M.Z_HI])[0])]
 assert np.array_equal(actual,expected)
 prior_mask=base_mask(p,model.pv,zref,M.Z_LO_PRIOR,M.Z_HI)
 all_sp.append(p[prior_mask,:2]);all_pts.append(p[model.pv&(p[:,2]<(z+M.Z_HI/1000)*1000)])
 outer_xy=np.zeros(len(p),dtype=bool)
 for item in spec["outer_primitives"]:
  a,b,c,d,h=item["args"];outer_xy|=(p[:,0]>=a*1000-30)&(p[:,0]<=c*1000+30)&(p[:,1]>=b*1000-30)&(p[:,1]<=d*1000+30)
 ext=model.pv&outside&changed&outer_xy
 records.append(dict(world_body_pose=[x,y,z,yaw],valid_points=int(model.pv.sum()),
   actual_exterior_geometry_valid_hits=int(ext.sum()),exterior_structure1500=int((ext&masks[1500]).sum()),
   exterior_structure350=int((ext&masks[350]).sum()),selected1500=int(masks[1500].sum()),selected350=int(masks[350].sum()),
   actual_online_selected_subsampled=len(actual),exterior_zrel_mm_min=int((p[ext,2]-zref).min())if ext.any()else None,
   exterior_zrel_mm_max=int((p[ext,2]-zref).max())if ext.any()else None))
(root/f"minimal_static_diagnostic_sn{M.SN}_zhi{M.Z_HI}.json").write_text(json.dumps(records,indent=2),encoding="utf8")
assert sum(v['actual_exterior_geometry_valid_hits']for v in records)>0
assert sum(v['exterior_structure1500']for v in records)>0
assert sum(v['exterior_structure350']for v in records)==0
sp,ap,crop=C.crop_prior(np.concatenate(all_sp),np.concatenate(all_pts),[0,-1500,5000,1500])
assert crop['struct_outside_removed']>0 if M.Z_HI==1500 else True
pkg,info=R.prior_map.build_package(sp,ap,bounds_mm=[0,-1500,5000,1500])
parsed=R.prior_map.parse_package(pkg)
for packet in R.rk_host.map_messages(pkg):
 buf=(ctypes.c_uint8*len(packet)).from_buffer_copy(packet);fw.py_push_host(buf,len(packet),ctypes.c_uint32(1000));fw.py_poll()
assert model.smap.prior and model.zlo==M.Z_LO_PRIOR and model.zhi==M.Z_HI
assert model.smap.lf[0]==info['ox']and model.smap.lf[1]==info['oy']
wy,wx=np.nonzero(info['occ']);gx=wx+info['ox'];gy=wy+info['oy']
assert np.all((gx>=int(R.prior_map.cell_of(0)))&(gx<=int(R.prior_map.cell_of(5000)))&
              (gy>=int(R.prior_map.cell_of(-1500)))&(gy<=int(R.prior_map.cell_of(1500))))
result=dict(passed=True,SN=M.SN,ZHI=M.Z_HI,inner_grid_equal=True,sampled_inner_height_equal=True,
 inner_height_samples=inner_samples,exterior_samples=records,
 prior_fixture=dict(crop=crop,n_struct=int(info['n_struct']),n_words=int(info['n_words']),package_bytes=len(pkg),
  actual_header_origin=[info['ox'],info['oy']],actual_loaded_origin=list(model.smap.lf[:2]),
  occupied_global_bounds=[int(gx.min()),int(gy.min()),int(gx.max()),int(gy.max())],
  source_height_band=[M.Z_LO_PRIOR,M.Z_HI],remaining_outside_struct=0,remaining_outside_all=0,
  full_survey=False,fixture_poses=len(poses),gate_halo_is_not_external_structure=True),ledger=ledger,
 DLL_rebuilt=False,scope='static virtual-lidar/fixed-point/actual C configuration and prior packet fixture; no closed-loop case started')
dest=Path(sys.argv[1]);dest.write_text(json.dumps(result,ensure_ascii=False,indent=2,default=lambda x:int(x)if isinstance(x,np.integer)else str(x))+'\n',encoding='utf8')
print(json.dumps(dict(passed=True,SN=M.SN,ZHI=M.Z_HI,exterior_valid=sum(v['actual_exterior_geometry_valid_hits']for v in records),
 exterior_structure1500=sum(v['exterior_structure1500']for v in records),exterior_structure350=sum(v['exterior_structure350']for v in records),crop=crop,n_struct=info['n_struct'],output=str(dest))))
'''

def prepare():
 assert not(OUT/'manifest.json').exists(),'Do not duplicate frozen stage1c preparation'
 OUT.mkdir(exist_ok=True)
 if not(OUT/'execution_tree').exists():shutil.copytree(A.OLD/'execution_tree',OUT/'execution_tree',ignore=shutil.ignore_patterns('__pycache__'))
 if not(OUT/'firmware_source').exists():shutil.copytree(A.OLD/'firmware_source',OUT/'firmware_source')
 for name in('nav_fw.dll','scene.json','cases.json'):shutil.copyfile(A.OLD/name,OUT/name)
 inner=A.read(A.OLD/'scene.json');spec=dict(id='field3x5_outer_v1',version=1,inner_scene=inner,expanded_world_size_m=20,
  inner_bounds_m=[0,-1.5,5,1.5],outer_geometry_distance_m=[.5,3],platform_m=.084,mount_height_m=.155,mount_pitch_down_deg=10,
  heightfield_tables_chairs_are_solid_prisms_not_overhang_mesh=True,outer_primitives=[])
 outer=[('room_west',[-2.5,-4.2,-2.35,4.2,2]),('room_east',[7.5,-4.2,7.65,4.2,2]),
  ('room_south',[-2.5,-4.2,7.65,-4.05,2]),('room_north',[-2.5,4.05,7.65,4.2,2]),
  ('table_east',[5.7,-1,6.6,-.25,.75]),('table_west',[-1.3,-.9,-.8,-.4,.75]),
  ('table_north',[1,2.2,2,2.9,.75]),('chair_east1',[5.7,.4,6.1,.8,.45]),
  ('chair_east2',[6.3,.45,6.7,.85,.45]),('chair_north',[2.3,2.2,2.7,2.6,.45])]
 for name,args in outer:spec['outer_primitives'].append(dict(kind='box',name=name,args=args))
 A.write(OUT/'outer_scene.json',spec)
 tree=OUT/'execution_tree';original=(A.OLD/'execution_tree/model/slf_match.py').read_text(encoding='utf8')
 derived=A.derive_model(original);line='Z_LO, Z_HI = int(os.environ.get("SLF_ZLO", "60")), 1500';assert line in derived
 derived=derived.replace(line,'Z_LO, Z_HI = int(os.environ.get("SLF_ZLO", "60")), int(os.environ.get("SLF_ZHI", "1500"))')
 (tree/'model/slf_match.py').write_text(derived,encoding='utf8')
 A.write(OUT/'coordinate_contract.json',A.coordinates())
 (OUT/'source_delta.patch').write_text(''.join(difflib.unified_diff(original.splitlines(True),derived.splitlines(True),fromfile='original/slf_match.py',tofile='stage1c/slf_match.py')),encoding='utf8')
 directory=tree/'sim/closed_loop';(directory/'stage1c_case.py').write_text(ENTRY,encoding='utf8');(directory/'stage1c_minimal.py').write_text(PROBE,encoding='utf8')
 support=(REPO/'integration/g2/stage1c_outer_height_support.py').read_text(encoding='utf8')
 support=support.replace('    M.structure_mask=mask','    M.structure_mask_original=original_mask\n    M.structure_mask=mask')
 (directory/'stage1c_support.py').write_text(support,encoding='utf8')
 (OUT/'entry_delta.patch').write_text(''.join(difflib.unified_diff((directory/'stage1_case.py').read_text(encoding='utf8').splitlines(True),ENTRY.splitlines(True),fromfile='original/stage1_case.py',tofile='stage1c/stage1c_case.py')),encoding='utf8')
 preserved=[]
 for p in(A.OLD/'execution_tree').rglob('*'):
  if p.is_file()and'__pycache__'not in p.parts and p.name!='slf_match.py':
   assert same_bytes(p,tree/p.relative_to(A.OLD/'execution_tree'));preserved.append(str(p))
 assert same_bytes(A.OLD/'nav_fw.dll',OUT/'nav_fw.dll')
 jobs=[];base=A.read(A.OLD/'manifest.json')
 for case in('goal','turn_loop'):
  for prior in(False,True):
   for hi in(1500,350):
    for sn in(256,128):
     b=next(j for j in base['jobs']if j['case']==case and j['SLF_SN']==sn and j['prior']==prior)
     name=f'{case}_sn{sn}_fixed_{"prior"if prior else"nomap"}_zhi{hi}_s1';command=list(b['command']);command[3]=str(directory/'stage1c_case.py')
     command[command.index('--scene')+1]=spec['id'];command[command.index('--firmware-dll')+1]=str(OUT/'nav_fw.dll');command[command.index('--out')+1]=str(OUT/'results'/name)
     env=dict(SLF_SN_LOG2=str(sn.bit_length()-1),SLF_LF_N=str(sn),SLF_WINDOW_MODE='fixed_field',SLF_FIXED_CX='41',SLF_FIXED_CY='19',SLF_ZLO='60',SLF_ZHI=str(hi),SLF_NBR='3x3',SLF_W18='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1')
     jobs.append(dict(name=name,case=case,SLF_SN=sn,ZHI=hi,prior=prior,seed=1,command=command,environment=env,out=str(OUT/'results'/name)))
 A.write(OUT/'manifest.json',dict(recorded_at=datetime.now().astimezone().isoformat(),scene=spec,jobs=jobs,planned=16,
  all_environment_baselines_new=True,old_1a_results_reused=False,DLL_rebuilt=False,inner_scene_preserved=True,unchanged_frozen_files=preserved,
  cold_configuration='all16: actual original MAP_CLEAR + one original C py_poll before sensors/goal/firstframe; actual op5 requested vs applied audited; full original first-poll LUT generation',
  prior='same upper height filter and explicit inner XY crop before original build_package; map prior unchanged once loaded',
  execution_scope='frozen N1c C DLL + virtual lidar + fixed-point perception + velocity kinematics; safe_in_loop=false,rl_in_loop=false; matcher fixed40ms assumption; not RTL/board/WCET',
  arrival_criteria=base['arrival_criteria'],gate='paired256: no added collision/falsearrival and validXY RMSE/P95/endpoint worsening<=10mm; max diagnostic',
  automatic_stage1b=False,automatic_EDA=False,bindings=[metadata(p)for p in tree.rglob('*')if p.is_file()and'__pycache__'not in p.parts]+[metadata(OUT/'nav_fw.dll')]))
 A.write(OUT/'job.json',dict(phase='prepared_waiting_minimal_proof_and_root_review',planned=16,completed=0,full_cases_started=False));return str(OUT)

def minimal(min_phys_gib=2,min_commit_gib=3):
 outputs=[]
 for sn in(8,7):
  for hi in(1500,350):
   while True:
    memory=memory_status()
    if memory.avail_phys>=min_phys_gib*A.GIB and memory.avail_page>=min_commit_gib*A.GIB:break
    A.write(OUT/'minimal_job.json',dict(phase='waiting_memory',physical=memory.avail_phys,commit=memory.avail_page));time.sleep(15)
   env=os.environ.copy();env.update(SLF_SN_LOG2=str(sn),SLF_LF_N=str(1<<sn),SLF_WINDOW_MODE='fixed_field',SLF_FIXED_CX='41',SLF_FIXED_CY='19',SLF_ZHI=str(hi),SLF_ZLO='60',SLF_NBR='3x3',SLF_W18='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
   dest=OUT/f'minimal_sn{1<<sn}_zhi{hi}.json';cmd=[sys.executable,'-X','utf8',str(OUT/'execution_tree/sim/closed_loop/stage1c_minimal.py'),str(dest)]
   log=OUT/f'minimal_sn{1<<sn}_zhi{hi}.log'
   with log.open('w',encoding='utf8')as f:
    p=subprocess.Popen(cmd,env=env,stdout=f,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW|subprocess.BELOW_NORMAL_PRIORITY_CLASS)
    proc=psutil.Process(p.pid);proc.cpu_affinity([18,19]);A.write(OUT/'minimal_job.json',dict(phase='running',pid=p.pid,create_time=proc.create_time(),command=cmd,completed=len(outputs),admission_physical=memory.avail_phys,admission_commit=memory.avail_page,minimum_physical_gib=min_phys_gib,minimum_commit_gib=min_commit_gib))
    peak_rss=0;peak_private=0
    while p.poll()is None:
     try:
      mem=proc.memory_info();peak_rss=max(peak_rss,mem.rss);peak_private=max(peak_private,getattr(mem,'private',mem.vms))
     except psutil.NoSuchProcess:pass
     time.sleep(1)
    code=p.returncode
   assert code==0,log.read_text(encoding='utf8');item=A.read(dest);item['actual_peak_working_set_bytes']=peak_rss;item['actual_peak_private_bytes']=peak_private;outputs.append(item)
   A.write(OUT/'minimal_proof.json',dict(passed=len(outputs)==4,completed=len(outputs),planned=4,outputs=outputs))
 A.write(OUT/'minimal_job.json',dict(phase='complete',completed=4));return str(OUT/'minimal_proof.json')

def summarize():
 m=A.read(OUT/'manifest.json');rows=[]
 for j in m['jobs']:
  result=Path(j['out'])
  if not(result/'metrics.json').exists():continue
  value=A.row(result);audit=A.read(result/'height_window_audit.json')
  assert len(audit['cold_configuration'])==1
  assert audit['zrel_writes']and all(w['actual_model_ZREL'][1]==j['ZHI']for w in audit['zrel_writes'])
  assert audit['ZHI']==j['ZHI']and audit['storage_SN']==j['SLF_SN']
  assert all(c['applied_ZREL'][1]==j['ZHI']for c in audit['mask_calls'])
  baseline=next(b for b in m['jobs']if b['case']==j['case']and b['prior']==j['prior']and b['ZHI']==j['ZHI']and b['SLF_SN']==256)
  compared=None
  if(Path(baseline['out'])/'metrics.json').exists():
   old=A.row(Path(baseline['out']));delta={k:value['XY_valid'][k]-old['XY_valid'][k]for k in('RMSE_mm','P95_mm','endpoint_mm','max_mm')}
   compared=dict(baseline=baseline['out'],delta_XY_valid_mm=delta,
    relative_three_metrics_pass=all(delta[k]<=10 for k in('RMSE_mm','P95_mm','endpoint_mm')),
    no_added_false_arrival_or_collision=value['collisions']<=old['collisions']and int(value['false_arrival'])<=int(old['false_arrival']),
    baseline_real_arrived=old['arrived'],candidate_real_arrived=value['arrived'])
  rows.append(dict(name=j['name'],case=j['case'],SLF_SN=j['SLF_SN'],ZHI=j['ZHI'],prior=j['prior'],result=value,
   paired_256=compared,actual_audit=str(result/'height_window_audit.json'),cold_empty_before_firstframe=True,
   actual_live_band_writes=audit['zrel_writes'],builds=len(audit['builds']),scores=len(audit['scores']),mask_calls=len(audit['mask_calls']),
   prior_packages=audit['prior_packages']))
 summary=dict(recorded_at=datetime.now().astimezone().isoformat(),completed=len(rows),planned=16,rows=rows,
  gate=m['gate'],execution_scope=m['execution_scope'],all_baselines_new_environment_and_cold_configuration=True,
  production_approval=False,automatic_stage1b=False,automatic_EDA=False)
 A.write(OUT/'summary.json',summary);return summary

def run():
 m=A.read(OUT/'manifest.json');assert A.read(OUT/'minimal_proof.json')['passed']
 approval=A.read(OUT/'root_review_approval.json');assert approval['approved_for_16_cases']is True
 assert not any((Path(j['out'])/'metrics.json').exists()for j in m['jobs']),'Do not duplicate completed case'
 assert A.read(OUT/'job.json')['phase']in('prepared_waiting_minimal_proof_and_root_review','root_review_passed_ready')
 me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS);me.cpu_affinity([18,19])
 completed=0;started=datetime.now().astimezone().isoformat()
 def status(phase,**kw):A.write(OUT/'job.json',dict(phase=phase,started_at=started,updated_at=datetime.now().astimezone().isoformat(),
  controller_pid=me.pid,controller_created_epoch=me.create_time(),planned=16,completed=completed,automatic_EDA=False,**kw))
 try:
  for j in m['jobs']:
   while True:
    memory=memory_status();status('waiting_memory_before_case',next=j['name'],physical=memory.avail_phys,commit=memory.avail_page,minimum_physical_gib=2,minimum_commit_gib=3)
    if memory.avail_phys>=2*A.GIB and memory.avail_page>=3*A.GIB:break
    time.sleep(10)
   env=os.environ.copy();env.update(j['environment']);directory=Path(j['out']);directory.mkdir(parents=True)
   with(directory/'stdout.log').open('w',encoding='utf8')as log:
    child=subprocess.Popen(j['command'],env=env,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW|subprocess.BELOW_NORMAL_PRIORITY_CLASS)
    proc=psutil.Process(child.pid);proc.cpu_affinity([18,19]);peak_rss=0;peak_private=0;low_since=None
    status('running_case',current=j['name'],child_pid=child.pid,child_created_epoch=proc.create_time(),command=j['command'],environment=j['environment'],
     admission_physical=memory.avail_phys,admission_commit=memory.avail_page)
    while child.poll()is None:
     memory=memory_status()
     try:
      info=proc.memory_info();peak_rss=max(peak_rss,info.rss);peak_private=max(peak_private,getattr(info,'private',info.vms))
     except psutil.NoSuchProcess:pass
     low=memory.avail_phys<256*1024**2 and memory.avail_page<A.GIB
     low_since=time.monotonic()if low and low_since is None else low_since if low else None
     if low_since is not None and time.monotonic()-low_since>=30:
      child.terminate();child.wait();raise RuntimeError('Only this new worker stopped by original runtime low-memory guard')
     time.sleep(2)
   A.write(directory/'runtime.json',dict(exit=child.returncode,peak_working_set_bytes=peak_rss,peak_private_bytes=peak_private,
    child_pid=child.pid,minimum_physical_gib=2,minimum_commit_gib=3))
   assert child.returncode==0,(directory/'stdout.log').read_text(encoding='utf8')
   assert(directory/'metrics.json').exists();completed+=1;summarize();status('case_complete',last=j['name'])
  status('complete',summary=str(OUT/'summary.json'));return str(OUT/'summary.json')
 except Exception:
  status('failed',error=traceback.format_exc());raise

if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--prepare',action='store_true');ap.add_argument('--minimal',action='store_true');ap.add_argument('--run',action='store_true');ap.add_argument('--summary',action='store_true');a=ap.parse_args()
 if a.prepare:print(prepare())
 if a.minimal:print(minimal())
 if a.run:print(run())
 if a.summary:print(json.dumps(summarize(),ensure_ascii=False,indent=2))
