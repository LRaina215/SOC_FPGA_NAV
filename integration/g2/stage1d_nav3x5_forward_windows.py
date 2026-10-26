"""Stage1d fallback: forward LF views using public estimated yaw; no RTL/EDA."""
from pathlib import Path
from datetime import datetime
import argparse
import difflib
import inspect
import json
import os
import subprocess
import sys
import time
import traceback
import psutil
import stage1a_nav3x5_window_ablation as A

STAGE1A = A.OUT
OUT = A.BASE / 'nav3x5_N1c_stage1d_forward_windows_20261026'
PROFILES = {f'storage128_lf128_forward{distance}mm':
            dict(SLF_SN_LOG2='7',SLF_LF_N='128',SLF_WINDOW_MODE='forward',SLF_FORWARD_MM=str(distance))
            for distance in (1500,2000,2400)}


def symmetric_round(numerator, denominator):
    return (1 if numerator >= 0 else -1) * ((abs(numerator)+denominator//2)//denominator)


def forward_shift(angle, distance_mm, inv_res, sin_table):
    # Same yaw>>6 and Q14 table convention as frozen fw/nav/fixmath.c.
    index = (int(angle) & 65535) >> 6
    sine = int(sin_table[index])
    cosine = int(sin_table[(index+256) & 1023])
    denominator = 16384*65536
    dx = symmetric_round(int(distance_mm)*int(inv_res)*cosine,denominator)
    dy = symmetric_round(int(distance_mm)*int(inv_res)*sine,denominator)
    return dict(yaw_angle=int(angle)&65535,yaw_table_index=index,sin_q14=sine,cos_q14=cosine,
                requested_distance_mm=int(distance_mm),offset_cells=[dx,dy],
                actual_offset_odometry_mm=[dx*65536/inv_res,dy*65536/inv_res],
                quantization='symmetric nearest cell; ties away from zero')


def forward_entry():
    text = A.ENTRY.replace('import atexit,json,math,os,sys','import atexit,ctypes,json,math,os,sys')
    text = text.replace('Stage1a only','Stage1d only')
    source = inspect.getsource(symmetric_round)+'\n'+inspect.getsource(forward_shift)+'\n'
    marker = 'instances=[];events=[]\n'
    capture = '''fw_instances=[]
original_firmware_factory=R.Firmware
def firmware_factory():
 value=original_firmware_factory();fw_instances.append(value);return value
R.Firmware=firmware_factory
'''
    assert text.count(marker)==1
    text = text.replace(marker,marker+source+capture)
    marker = ' original_build(self,cx,cy)\n'
    shifted = ''' shift=None
 if M.LF_WINDOW_MODE=='forward':
  assert len(fw_instances)==1
  state=(ctypes.c_int32*12)();fw_instances[0].py_get_state(state)
  shift=forward_shift(int(state[3])&65535,int(os.environ['SLF_FORWARD_MM']),self.inv_res,M.nf.SIN)
  shift['public_heading_source']='frozen DLL py_get_state()[3] latest estimated odometry yaw at LF rebuild'
  shift['public_pose_mm']=[int(state[0]),int(state[1])]
  original_build(self,cx+shift['offset_cells'][0],cy+shift['offset_cells'][1])
 else:original_build(self,cx,cy)
'''
    assert text.count(marker)==1
    text = text.replace(marker,shifted)
    marker = "n_struct=int(self.n_struct),prior=bool(self.prior),margin=M.MARGIN))"
    assert text.count(marker)==1
    text = text.replace(marker,"n_struct=int(self.n_struct),prior=bool(self.prior),margin=M.MARGIN,forward_shift=shift))")
    marker = "window_mode=M.LF_WINDOW_MODE,fixed_center_cells="
    assert text.count(marker)==1
    text = text.replace(marker,"window_mode=M.LF_WINDOW_MODE,forward_mm=int(os.environ['SLF_FORWARD_MM']),fixed_center_cells=")
    text = text.replace("LF{M.LF_N}_{M.LF_WINDOW_MODE}/ZREL", "LF{M.LF_N}_{M.LF_WINDOW_MODE}{os.environ[\"SLF_FORWARD_MM\"]}mm/ZREL")
    return text


def geometry_contract():
    # Pure selector arithmetic only; does not run a navigation/CPU simulation.
    sys.path.insert(0,str(OUT/'execution_tree/model'))
    import nav_fixed as nf
    angles=(0,8192,16384,24576,32768,40960,49152,57344,65535)
    rows=[]
    for distance in (1500,2000,2400):
        for angle in angles:
            v=forward_shift(angle,distance,1311,nf.SIN)
            assert forward_shift(angle,0,1311,nf.SIN)['offset_cells']==[0,0]
            target=[distance*v['cos_q14']/16384,distance*v['sin_q14']/16384]
            assert max(abs(a-b)for a,b in zip(v['actual_offset_odometry_mm'],target))<=65536/1311/2+1e-9
            rows.append(v)
    for distance,expected in ((1500,30),(2000,40),(2400,48)):
        assert forward_shift(0,distance,1311,nf.SIN)['offset_cells']==[expected,0]
        assert forward_shift(16384,distance,1311,nf.SIN)['offset_cells']==[0,expected]
        assert forward_shift(32768,distance,1311,nf.SIN)['offset_cells']==[-expected,0]
        assert forward_shift(49152,distance,1311,nf.SIN)['offset_cells']==[0,-expected]
    value=dict(passed=True,scope='27 pure window-selector geometries plus zero-offset identities; not closed-loop acceptance',
        table_source=str(OUT/'execution_tree/model/nav_fixed.py'),yaw_convention_source=str(OUT/'firmware_source/nav/fixmath.c')+':4',
        public_yaw_source=str(OUT/'firmware_source/test/hal_py.c')+':118',
        LF_center='original EM center cells + forward shift cells; EM itself unchanged',
        yaw_sampling='public estimated odometry yaw, latest at each LF rebuild; not frame R or true robot yaw',
        timing_limit='current estimated yaw may be later than the frame whose EM cx/cy is retained; this model choice is explicit',
        score_pivot='original vehicle cx_mm/cy_mm unchanged',UPDATE=True,prior=False,MARGIN=8,rows=rows)
    A.write(OUT/'forward_geometry_contract.json',value);return value


def setup_module():
    A.OUT=OUT


def prepare():
    original_derive=A.derive_model
    def derive(text):
        source=original_derive(text)
        target='assert LF_WINDOW_MODE in ("rolling", "fixed_field")'
        assert source.count(target)==1
        return source.replace(target,'assert LF_WINDOW_MODE in ("rolling", "fixed_field", "forward")')
    A.derive_model=derive;A.PROFILES=PROFILES;A.ENTRY=forward_entry()
    manifest=A.prepare()
    tree=OUT/'execution_tree'
    old_entry=tree/'sim/closed_loop/stage1a_case.py';new_entry=old_entry.with_name('stage1d_case.py');old_entry.rename(new_entry)
    for cell in manifest['jobs']:
        cell['command'][3]=str(new_entry)
        cell['cache_namespace']+=f"/forward{cell['environment']['SLF_FORWARD_MM']}mm"
    manifest.update(task='user task stage1d fallback only',automatic_stage1b=False,automatic_EDA=False,
        requires_stage1a_complete_and_fixed_hard_gate_failure=True,stage1a_job=str(STAGE1A/'job.json'),
        stage1a_summary=str(STAGE1A/'summary.json'),heading_contract='public estimated yaw latest at LF rebuild; never robot truth',
        center_contract='original EM cx/cy plus only LF shift; original candidate rotation pivot and UPDATE unchanged')
    manifest['bindings']=[A.metadata(p)for p in tree.rglob('*')if p.is_file()and'__pycache__'not in p.parts]+[A.metadata(OUT/'nav_fw.dll')]
    A.write(OUT/'manifest.json',manifest)
    A.write(OUT/'job.json',dict(phase='prepared_not_started',planned=6,completed=0,automatic_EDA=False,automatic_stage1b=False))
    entry_before=A.ENTRY # full entry frozen in manifest; delta below compares against stage1a baseline entry
    original_entry=(STAGE1A/'execution_tree/sim/closed_loop/stage1a_case.py').read_text(encoding='utf8')
    (OUT/'entry_delta.patch').write_text(''.join(difflib.unified_diff(original_entry.splitlines(True),entry_before.splitlines(True),fromfile='stage1a_entry',tofile='stage1d_entry')),encoding='utf8')
    geometry_contract();return manifest


def summarize():
    value=A.summarize();value['planned']=6;value['stage']='stage1d';A.write(OUT/'summary.json',value);return value


def run():
    previous=A.read(STAGE1A/'job.json');assert previous['phase']=='all_stage1a_cases_complete'and previous['completed']==4
    prior=A.read(STAGE1A/'summary.json');fixed=[r for r in prior['rows']if r['profile']=='storage128_lf128_fixed_field']
    assert len(fixed)==2
    assert not all(r['comparisons']['256']['relative_3metric_error_gate']and r['comparisons']['256']['no_added_false_arrival_or_collision']for r in fixed),'Stage1a passed; do not run fallback'
    assert A.read(OUT/'minimal_consistency.json')['passed']and A.read(OUT/'forward_geometry_contract.json')['passed']
    manifest=A.read(OUT/'manifest.json')
    for binding in manifest['bindings']:assert A.metadata(Path(binding['path']))==binding,binding['path']
    me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS);me.cpu_affinity(list(range(max(0,psutil.cpu_count()-2),psutil.cpu_count())))
    with(OUT/'run_claim.json').open('x')as f:json.dump(dict(pid=me.pid,created_epoch=me.create_time()),f)
    state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),one_worker=True,priority='BELOW_NORMAL',logical_CPUs=me.cpu_affinity(),minimum_start_physical_bytes=int(2.5*A.GIB),minimum_start_commit_bytes=int(4.5*A.GIB),planned=6,completed=0,automatic_stage1b=False,automatic_EDA=False,stage1a_completed_and_fixed_hard_gate_failed=True)
    def status(phase,**kwargs):state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**kwargs);A.write(OUT/'job.json',state)
    for cell in manifest['jobs']:
        assert not Path(cell['out']).exists(),'Preserve previous result, do not repeat'
        while True:
            memory=A.memory_status();status('waiting_memory_before_case',next=cell['name'],free_bytes=memory.avail_phys,available_commit_bytes=memory.avail_page)
            if memory.avail_phys>=int(2.5*A.GIB)and memory.avail_page>=int(4.5*A.GIB):break
            time.sleep(17)
        path=Path(cell['out']);path.mkdir(parents=True);env=os.environ.copy();env.update(cell['environment'])
        with(path/'stdout.log').open('w',encoding='utf8')as log:
            child=subprocess.Popen(cell['command'],env=env,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
            process=psutil.Process(child.pid);process.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS);process.cpu_affinity(me.cpu_affinity());status('running_case',current=cell['name'],child_pid=child.pid,child_created_epoch=process.create_time(),command=cell['command'],environment=cell['environment'],admission_physical_bytes=memory.avail_phys,admission_commit_bytes=memory.avail_page)
            low_since=None
            while child.poll()is None:
                memory=A.memory_status();low=memory.avail_phys<256*1024**2 and memory.avail_page<A.GIB
                low_since=time.monotonic()if low and low_since is None else low_since if low else None
                if low_since is not None and time.monotonic()-low_since>=30:
                    child.terminate();child.wait();status('own_simulation_stopped_for_memory_preserved',other_tasks_untouched=True);return 2
                time.sleep(5)
        assert child.returncode==0,('preserve failure; no automatic retry',cell['name'],child.returncode)
        status('case_complete',completed=summarize()['completed'],last=cell['name'])
    status('all_stage1d_cases_complete',completed=summarize()['completed']);return 0


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--prepare',action='store_true');parser.add_argument('--consistency',action='store_true');parser.add_argument('--run',action='store_true');parser.add_argument('--summary',action='store_true');args=parser.parse_args();assert sum((args.prepare,args.consistency,args.run,args.summary))==1
    setup_module()
    try:
        if args.prepare:print(json.dumps(dict(out=str(OUT),jobs=len(prepare()['jobs'])),ensure_ascii=False))
        elif args.consistency:print(json.dumps(A.consistency(),ensure_ascii=False))
        elif args.summary:print(json.dumps(summarize(),ensure_ascii=False))
        else:raise SystemExit(run())
    except Exception:
        if OUT.exists():A.write(OUT/'failure.json',dict(error=traceback.format_exc(),production_inputs_unchanged=True))
        raise
