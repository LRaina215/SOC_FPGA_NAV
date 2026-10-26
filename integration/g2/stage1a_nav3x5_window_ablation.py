"""Frozen N1c stage1a: four new window-ablation cells, existing baselines reused."""
from pathlib import Path
from datetime import datetime
import argparse
import csv
import difflib
import json
import math
import os
import shutil
import subprocess
import sys
import time
import traceback
import psutil
from prepare_baseline_fusion_retime import metadata, same_bytes
from retry_frozen_synthesis import memory_status
from analyze_nav3x5_stage1 import stats

REPO = Path('D:/Desktop/SOC FPGA/nav')
BASE = Path('F:/nav_integration_g2_20261023')
OLD = BASE / 'nav3x5_N1c_SLF128_256_stage1_20261026'
OUT = BASE / 'nav3x5_N1c_stage1a_window_ablation_20261026'
GIB = 1024**3
PROFILES = {
    'storage256_lf128_rolling': dict(SLF_SN_LOG2='8', SLF_LF_N='128', SLF_WINDOW_MODE='rolling'),
    'storage128_lf128_fixed_field': dict(SLF_SN_LOG2='7', SLF_LF_N='128', SLF_WINDOW_MODE='fixed_field'),
}


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf8')


def derive_model(text):
    """Only LF view bounds change; storage SN/tag/update and point arithmetic remain."""
    line = 'SN = 1 << SN_LOG2            # 结构图边长\n'
    assert text.count(line) == 1
    text = text.replace(line, line + '''# Stage1a experiment: independent LF view size and no-map center policy.
LF_N = int(os.environ.get("SLF_LF_N", str(SN)))
LF_WINDOW_MODE = os.environ.get("SLF_WINDOW_MODE", "rolling")
LF_FIXED_CX = int(os.environ.get("SLF_FIXED_CX", "0"))
LF_FIXED_CY = int(os.environ.get("SLF_FIXED_CY", "0"))
assert LF_N in (128, 256) and LF_N <= SN
assert LF_WINDOW_MODE in ("rolling", "fixed_field")
''')
    begin = text.index('    def build_lf(self, cx, cy):')
    end = text.index('    def score(', begin)
    before = text[begin:end]
    after = before.replace('        w = np.arange(SN)\n',
        '        if LF_WINDOW_MODE == "fixed_field":\n'
        '            cx, cy = LF_FIXED_CX, LF_FIXED_CY\n'
        '        w = np.arange(LF_N)\n')
    after = after.replace('cx - SN // 2', 'cx - LF_N // 2').replace('cy - SN // 2', 'cy - LF_N // 2')
    assert '<< SN_LOG2' in after and '(SN - 1)' in after
    text = text[:begin] + after + text[end:]
    begin = text.index('    def score(')
    end = text.index('    def cell16(', begin)
    before = text[begin:end]
    after = before.replace('(wx0 < SN - mg)', '(wx0 < LF_N - mg)').replace('(wy0 < SN - mg)', '(wy0 < LF_N - mg)')
    assert before != after
    text = text[:begin] + after + text[end:]
    begin = text.index('def point_cost(')
    end = text.index('def taxicab_dt(', begin)
    before = text[begin:end]
    after = before.replace('    wx, wy = fx >> 4, fy >> 4\n', '    lf_n = occ.shape[0]\n    wx, wy = fx >> 4, fy >> 4\n')
    after = after.replace('SN - 1', 'lf_n - 1').replace('ux < SN', 'ux < lf_n').replace('uy < SN', 'uy < lf_n')
    assert 'SN' not in after
    return text[:begin] + after + text[end:]


ENTRY = r'''from pathlib import Path
import atexit,json,math,os,sys
import numpy as np
import stage1_case as S
R=S.R
import slf_match as M
assert '--prior' not in sys.argv, 'Stage1a only; prior is not a substitute for fixed no-map'
assert M.Z_LO==60 and M.Z_HI==1500 and M.MARGIN==8
out=Path(sys.argv[sys.argv.index('--out')+1])
start=[float(v)for v in sys.argv[sys.argv.index('--start')+1].split(',')]
assert start==[.45,-.95,0.0]
instances=[];events=[]
original_init=M.StructMap.__init__
original_build=M.StructMap.build_lf
original_score=M.StructMap.score
original_update=M.StructMap.update
def init(self,*args,**kwargs):
 original_init(self,*args,**kwargs);self.stage1a_instance=len(instances);instances.append(self)
def world_edges(self,cell_edges):
 # Original run_cl no-map w2o: R(-start_yaw)*(world-start_xy).
 c=math.cos(math.radians(start[2]));s=math.sin(math.radians(start[2]))
 x0,y0,x1,y1=[v*65536/self.inv_res/1000 for v in cell_edges]
 return [[start[0]+c*x-s*y,start[1]+s*x+c*y]for x,y in((x0,y0),(x1,y0),(x1,y1),(x0,y1))]
def build(self,cx,cy):
 original_build(self,cx,cy)
 ox,oy,occ,*_=self.lf;n=occ.shape[0]
 events.append(dict(op='build_lf',instance=self.stage1a_instance,requested_vehicle_center_cells=[int(cx),int(cy)],actual_center_cells=[ox+n//2,oy+n//2],window_origin_cells=[ox,oy],window_end_exclusive_cells=[ox+n,oy+n],LF_N=n,storage_SN=M.SN,world_window_corners_m=world_edges(self,[ox,oy,ox+n,oy+n]),world_scoring_core_corners_m=world_edges(self,[ox+M.MARGIN,oy+M.MARGIN,ox+n-M.MARGIN,oy+n-M.MARGIN]),n_struct=int(self.n_struct),prior=bool(self.prior),margin=M.MARGIN))
def score(self,px,py,cands,cx_mm,cy_mm):
 value=original_score(self,px,py,cands,cx_mm,cy_mm)
 if len(value):assert np.all(value[:,0]==value[0,0])
 events.append(dict(op='score',instance=self.stage1a_instance,points_input=len(px),effective_n_valid=int(value[0,0])if len(value)else 0,candidates=len(cands),candidate_rotation_center_vehicle_mm=[int(cx_mm),int(cy_mm)],window_origin_cells=list(self.lf[:2])if self.lf is not None else None,LF_N=int(self.lf[2].shape[0])if self.lf is not None else None,prior=bool(self.prior),margin=M.MARGIN))
 return value
def update(self,pts):
 original_update(self,pts)
 events.append(dict(op='update',instance=self.stage1a_instance,points=len(pts),prior=bool(self.prior)))
M.StructMap.__init__=init;M.StructMap.build_lf=build;M.StructMap.score=score;M.StructMap.update=update
def finish():
 profile=dict(storage_SN=M.SN,storage_SN_LOG2=M.SN_LOG2,LF_N=M.LF_N,window_mode=M.LF_WINDOW_MODE,fixed_center_cells=[M.LF_FIXED_CX,M.LF_FIXED_CY],ZREL=[M.Z_LO,M.Z_HI],MARGIN=M.MARGIN,prior_semantics=False,UPDATE_preserved=True,candidate_rotation_center_remains_vehicle=True,scene='field3x5_v1',provisional_start_world_m_deg=start,cache_namespace=f'field3x5_v1/SN{M.SN}/LF{M.LF_N}_{M.LF_WINDOW_MODE}/ZREL{M.Z_LO}_{M.Z_HI}',prior_cache_used=False)
 out.mkdir(parents=True,exist_ok=True)
 (out/'window_audit.json').write_text(json.dumps(dict(profile=profile,events=events),ensure_ascii=False,indent=2,default=lambda x:int(x)if isinstance(x,np.integer)else str(x))+'\n',encoding='utf8')
atexit.register(finish)
if __name__=='__main__':raise SystemExit(R.main())
'''

PROBE = r'''from pathlib import Path
import importlib.util,json,os,sys
import numpy as np
root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/'model'))
import slf_match as new
spec=importlib.util.spec_from_file_location('original_slf_match',Path(os.environ['STAGE1A_ORIGINAL_MODEL']))
old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
rng=np.random.default_rng(1901);a=old.StructMap();b=new.StructMap();checks=0
cands=np.array([[dx,dy,da]for dx in(-50,0,50)for dy in(-50,0,50)for da in(-180,0,180)])
assert new.LF_N==new.SN
for k,(cx,cy)in enumerate(((0,0),(-73,109),(131,-140),(41,19))):
 pts=rng.integers(-11000,11000,size=(1024,2));a.update(pts);b.update(pts)
 for name in('hit','tag','cxs','cys'):assert np.array_equal(getattr(a,name),getattr(b,name));checks+=1
 expect=(new.LF_FIXED_CX,new.LF_FIXED_CY)if new.LF_WINDOW_MODE=='fixed_field'else(cx,cy)
 a.build_lf(*expect);b.build_lf(cx,cy)
 for va,vb in zip(a.lf,b.lf):assert np.array_equal(va,vb);checks+=1
 assert np.array_equal(a.lf_slot,b.lf_slot);checks+=1
 for va,vb in zip(a.words(),b.words()):assert np.array_equal(va,vb);checks+=1
 px=rng.integers((expect[0]-new.SN//2)*50,(expect[0]+new.SN//2)*50,size=700)
 py=rng.integers((expect[1]-new.SN//2)*50,(expect[1]+new.SN//2)*50,size=700)
 assert np.array_equal(a.score(px,py,cands,cx*50,cy*50),b.score(px,py,cands,cx*50,cy*50));checks+=1
assert old.MARGIN==new.MARGIN==8 and not b.prior
print(json.dumps(dict(passed=True,SLF_SN=new.SN,LF_N=new.LF_N,window_mode=new.LF_WINDOW_MODE,checks=checks,frames=4,candidates=len(cands),arithmetic_storage_and_score_equal_at_same_configuration=True,prior_false=True)))
'''


def coordinates():
    start = [.45, -.95, 0.0]
    center = [2.5, 0.0]
    theta = math.radians(start[2])
    dx, dy = center[0]-start[0], center[1]-start[1]
    odom = [math.cos(theta)*dx+math.sin(theta)*dy, -math.sin(theta)*dx+math.cos(theta)*dy]
    mm = [int(round(v*1000)) for v in odom]
    cells = [(v*1311) >> 16 for v in mm]
    assert mm == [2050, 950] and cells == [41, 19]
    return dict(provisional_start_world_m_deg=start,field_center_world_m=center,
                field_center_odometry_m=odom,field_center_odometry_mm=mm,
                inverse_resolution=1311,actual_fixed_center_cells=cells,
                fixed_window_origin_cells=[v-64 for v in cells],
                cell_conversion='(int_mm * 1311) >> 16; original StructMap.cell semantics',
                rotation_pivot='original vehicle cx_mm/cy_mm; NOT fixed field center')


def prepare():
    assert not OUT.exists(), 'Do not repeat existing stage1a preparation or runs'
    assert read(OLD/'job.json')['completed'] == 16
    OUT.mkdir()
    for folder in ('execution_tree','firmware_source'):
        shutil.copytree(OLD/folder,OUT/folder,ignore=shutil.ignore_patterns('__pycache__'))
    for name in ('nav_fw.dll','scene.json','cases.json','scene_validation.json','scene.png'):
        shutil.copyfile(OLD/name,OUT/name)
    tree = OUT/'execution_tree'; original = (OLD/'execution_tree/model/slf_match.py').read_text(encoding='utf8')
    derived = derive_model(original)
    (tree/'model/slf_match.py').write_text(derived,encoding='utf8')
    (OUT/'source_delta.patch').write_text(''.join(difflib.unified_diff(original.splitlines(True),derived.splitlines(True),fromfile='frozen_N1c/slf_match.py',tofile='stage1a/slf_match.py')),encoding='utf8')
    (tree/'sim/closed_loop/stage1a_case.py').write_text(ENTRY,encoding='utf8')
    (tree/'sim/closed_loop/stage1a_consistency.py').write_text(PROBE,encoding='utf8')
    preserved=[]
    for p in (OLD/'execution_tree').rglob('*'):
        if not p.is_file() or '__pycache__' in p.parts or p.name=='slf_match.py':continue
        q=tree/p.relative_to(OLD/'execution_tree');assert same_bytes(p,q);preserved.append(str(q))
    assert same_bytes(OLD/'nav_fw.dll',OUT/'nav_fw.dll')
    coord=coordinates();write(OUT/'coordinate_contract.json',coord)
    base=read(OLD/'manifest.json');jobs=[];reused=[]
    for case in ('goal','turn_loop'):
        for sn in (256,128):
            cell=next(j for j in base['jobs']if j['case']==case and j['SLF_SN']==sn and not j['prior']);assert(Path(cell['out'])/'metrics.json').is_file();reused.append(cell)
        for name,profile in PROFILES.items():
            sn=1<<int(profile['SLF_SN_LOG2']);cell=next(j for j in reused if j['case']==case and j['SLF_SN']==sn)
            result=OUT/'results'/f'{case}_{name}_s1';command=list(cell['command'])
            command[3]=str(tree/'sim/closed_loop/stage1a_case.py')
            command[command.index('--firmware-dll')+1]=str(OUT/'nav_fw.dll')
            command[command.index('--out')+1]=str(result)
            env=dict(profile,SLF_FIXED_CX=str(coord['actual_fixed_center_cells'][0]),SLF_FIXED_CY=str(coord['actual_fixed_center_cells'][1]),SLF_ZLO='60',SLF_NBR='3x3',SLF_W18='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1')
            jobs.append(dict(name=result.name,case=case,profile=name,SLF_SN=sn,seed=1,prior=False,command=command,environment=env,out=str(result),cache_namespace=f'field3x5_v1/SN{sn}/LF128_{profile["SLF_WINDOW_MODE"]}/ZREL60_1500',prior_cache_used=False))
    bindings=[metadata(p)for p in tree.rglob('*')if p.is_file()and'__pycache__'not in p.parts]+[metadata(OUT/'nav_fw.dll')]
    manifest=dict(recorded_at=datetime.now().astimezone().isoformat(),task='user task stage1a only',source_stage1=str(OLD),DLL_rebuilt=False,production_RTL_or_model_modified=False,scene='field3x5_v1',cases=['goal','turn_loop'],seed=1,prior=False,arrival_scoring_unchanged=base['arrival_criteria'],relative_error_gate_metrics=['RMSE_mm','P95_mm','endpoint_mm'],relative_error_worsening_limit_mm=10,max_reported_not_added_gate=True,coordinate_contract=coord,profiles=PROFILES,jobs=jobs,reused_baselines=reused,unchanged_frozen_files=preserved,only_reference_model_file_changed='model/slf_match.py; isolated frozen copy',execution_scope='N1c navigation C DLL + fixed-point perception + virtual lidar + velocity kinematics; RL/SAFE not in loop; fixed40ms matcher simulation latency; not RTL/board/WCET',bindings=bindings,automatic_stage1b=False,automatic_EDA=False)
    write(OUT/'manifest.json',manifest);write(OUT/'job.json',dict(phase='prepared_not_started',planned=4,completed=0,automatic_EDA=False));return manifest


def consistency():
    manifest=read(OUT/'manifest.json');results=[]
    for sn,mode in ((8,'rolling'),(7,'rolling'),(7,'fixed_field')):
        env=os.environ.copy();env.update(SLF_SN_LOG2=str(sn),SLF_LF_N=str(1<<sn),SLF_WINDOW_MODE=mode,SLF_FIXED_CX='41',SLF_FIXED_CY='19',STAGE1A_ORIGINAL_MODEL=str(OLD/'execution_tree/model/slf_match.py'),OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
        command=[sys.executable,'-X','utf8',str(OUT/'execution_tree/sim/closed_loop/stage1a_consistency.py')]
        r=subprocess.run(command,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,creationflags=subprocess.CREATE_NO_WINDOW)
        item=dict(command=command,environment={k:env[k]for k in('SLF_SN_LOG2','SLF_LF_N','SLF_WINDOW_MODE','SLF_FIXED_CX','SLF_FIXED_CY')},exit=r.returncode,stdout=r.stdout);results.append(item)
        write(OUT/'minimal_consistency.json',dict(passed=r.returncode==0,results=results))
        assert r.returncode==0,r.stdout
    return results


def row(path):
    metric=read(path/'metrics.json');full=[];valid=[]
    with(path/'timeseries.csv').open(encoding='utf8')as f:
        for r in csv.DictReader(f):
            value=1000*math.hypot(float(r['ex'])-float(r['x']),float(r['ey'])-float(r['y']));full.append(value)
            if int(float(r['loc_valid'])):valid.append(value)
    return dict(outcome=metric['outcome'],arrived=metric['arrived'],false_arrival=metric['terminal']['outcome']=='false_arrival',collisions=metric['collisions'],goal_error_mm=1000*metric['goal_err_m'],XY_valid=stats(valid),XY_full=stats(full),terminal=metric['terminal'],metrics=str(path/'metrics.json'),trace=str(path/'timeseries.csv'))


def summarize():
    m=read(OUT/'manifest.json');baseline={(b['case'],b['SLF_SN']):row(Path(b['out']))for b in m['reused_baselines']};rows=[]
    for j in m['jobs']:
        path=Path(j['out'])
        if not(path/'metrics.json').exists():continue
        value=row(path);audit=read(path/'window_audit.json');builds=[v for v in audit['events']if v['op']=='build_lf'];scores=[v for v in audit['events']if v['op']=='score'];updates=[v for v in audit['events']if v['op']=='update']
        assert updates and builds and scores and all(not v['prior']and v['margin']==8 for v in builds+scores)
        if j['environment']['SLF_WINDOW_MODE']=='fixed_field':assert all(v['actual_center_cells']==[41,19]for v in builds)
        compared={}
        for sn in(256,128):
            old=baseline[j['case'],sn];delta={k:value['XY_valid'][k]-old['XY_valid'][k]for k in('RMSE_mm','P95_mm','endpoint_mm','max_mm')}
            compared[str(sn)]=dict(original=old,delta_XY_valid_mm=delta,relative_3metric_error_gate=all(delta[k]<=10 for k in('RMSE_mm','P95_mm','endpoint_mm')),no_added_false_arrival_or_collision=value['collisions']<=old['collisions']and int(value['false_arrival'])<=int(old['false_arrival']),trace_bytes_identical=same_bytes(Path(value['trace']),Path(old['trace'])))
        rows.append(dict(case=j['case'],profile=j['profile'],SLF_SN=j['SLF_SN'],result=value,comparisons=compared,actual_window_audit=str(path/'window_audit.json'),audit_summary=dict(update_calls=len(updates),update_points_total=sum(v['points']for v in updates),build_calls=len(builds),score_calls=len(scores),first_build=builds[0],last_build=builds[-1],effective_n_valid_min=min(v['effective_n_valid']for v in scores),effective_n_valid_max=max(v['effective_n_valid']for v in scores),UPDATE_not_skipped=True,prior_false=True,MARGIN=8,candidate_rotation_center_is_vehicle=True)))
    result=dict(recorded_at=datetime.now().astimezone().isoformat(),completed=len(rows),planned=4,baseline_runs_reused_not_repeated=True,baseline_metrics={f'{case}_sn{sn}':v for(case,sn),v in baseline.items()},rows=rows,gate='relative to rolling256: valid XY RMSE/P95/endpoint worsening<=10mm, no added collision/false-arrival; max diagnostic only',execution_scope=m['execution_scope'],additional_stage1b_or_EDA_started=False,production_approval=False)
    write(OUT/'summary.json',result);return result


def run():
    m=read(OUT/'manifest.json');assert read(OUT/'minimal_consistency.json')['passed']
    for binding in m['bindings']:assert metadata(Path(binding['path']))==binding,binding['path']
    me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS);me.cpu_affinity(list(range(max(0,psutil.cpu_count()-2),psutil.cpu_count())))
    with(OUT/'run_claim.json').open('x')as f:json.dump(dict(pid=me.pid,created_epoch=me.create_time()),f)
    state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),one_worker=True,priority='BELOW_NORMAL',logical_CPUs=me.cpu_affinity(),minimum_start_physical_bytes=int(2.5*GIB),minimum_start_commit_bytes=int(4.5*GIB),planned=4,completed=0,automatic_stage1b=False,automatic_EDA=False)
    def status(phase,**kw):state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**kw);write(OUT/'job.json',state)
    for cell in m['jobs']:
        assert not Path(cell['out']).exists(),'Preserve existing cell; never automatically repeat'
        while True:
            mem=memory_status();status('waiting_memory_before_case',next=cell['name'],free_bytes=mem.avail_phys,available_commit_bytes=mem.avail_page)
            if mem.avail_phys>=int(2.5*GIB)and mem.avail_page>=int(4.5*GIB):break
            time.sleep(17)
        path=Path(cell['out']);path.mkdir(parents=True)
        env=os.environ.copy();env.update(cell['environment'])
        with(path/'stdout.log').open('w',encoding='utf8')as log:
            child=subprocess.Popen(cell['command'],env=env,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
            p=psutil.Process(child.pid);p.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS);p.cpu_affinity(me.cpu_affinity());status('running_case',current=cell['name'],child_pid=child.pid,child_created_epoch=p.create_time(),command=cell['command'],environment=cell['environment'],admission_physical_bytes=mem.avail_phys,admission_commit_bytes=mem.avail_page)
            low_since=None
            while child.poll()is None:
                mem=memory_status();low=mem.avail_phys<256*1024**2 and mem.avail_page<GIB
                low_since=time.monotonic()if low and low_since is None else low_since if low else None
                if low_since is not None and time.monotonic()-low_since>=30:
                    child.terminate();child.wait();status('own_simulation_stopped_for_memory_preserved',other_tasks_untouched=True);return 2
                time.sleep(5)
        assert child.returncode==0,('preserve failed cell, no retry',cell['name'],child.returncode)
        value=summarize();status('case_complete',completed=value['completed'],last=cell['name'])
    status('all_stage1a_cases_complete',completed=summarize()['completed']);return 0


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--prepare',action='store_true');parser.add_argument('--consistency',action='store_true');parser.add_argument('--run',action='store_true');parser.add_argument('--summary',action='store_true');args=parser.parse_args();assert sum((args.prepare,args.consistency,args.run,args.summary))==1
    try:
        if args.prepare:print(json.dumps(dict(prepared=True,out=str(OUT),jobs=len(prepare()['jobs'])),ensure_ascii=False))
        elif args.consistency:print(json.dumps(consistency(),ensure_ascii=False))
        elif args.summary:print(json.dumps(summarize(),ensure_ascii=False))
        else:raise SystemExit(run())
    except Exception:
        if OUT.exists():write(OUT/'failure.json',dict(error=traceback.format_exc(),production_inputs_unchanged=True))
        raise
