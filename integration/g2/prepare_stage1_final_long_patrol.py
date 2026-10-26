"""Prepare long patrol pair using the original DLL's existing loop-waypoint flag."""
from pathlib import Path
import argparse,json,os,shutil,sys
from datetime import datetime
import stage1_final_nav3x5_multiseed as M
import stage1a_nav3x5_window_ablation as A
from prepare_baseline_fusion_retime import metadata,same_bytes

OUT=A.BASE/'nav3x5_N1c_final128_long_patrol_20261026'
ENTRY=r'''from pathlib import Path
import atexit,ctypes,json,os,sys
import final_forward_case as E
R=E.R
root=Path(__file__).resolve().parents[3]
out=Path(sys.argv[sys.argv.index('--out')+1])
audit=dict(mode='native cyclic waypoint protocol; frozen original nav_task wp_loop',simulated_time_us=1000,
 state_goal_transitions=[],packets=[],sample_period_s=.1,stop_and_arrival_criteria_unchanged=True)
original_packet=R.rk_host.waypoints
def waypoints(pts,loop=False):
 packet=original_packet(pts,loop=True)
 assert packet[3]==0x11 and packet[5]==1
 audit['packets'].append(dict(waypoints_mm=[list(p)for p in pts],loop_bit=packet[5],n=packet[4]))
 return packet
R.rk_host.waypoints=waypoints
previous=R.Firmware
class ObservedFirmware:
 def __init__(self):self.raw=previous();self.last=None
 def __getattr__(self,name):return getattr(self.raw,name)
 def py_set_time(self,value):
  audit['simulated_time_us']=int(value.value if hasattr(value,'value')else value)
  return self.raw.py_set_time(value)
 def py_get_state(self,value):
  ret=self.raw.py_get_state(value)
  key=[int(value[6]),int(value[7]),int(value[8]),int(value[9])]
  if key!=self.last:
   audit['state_goal_transitions'].append(dict(t_us=audit['simulated_time_us'],mode=key[0],state=key[1],goal_mm=key[2:]))
   self.last=key
  return ret
R.Firmware=ObservedFirmware
def finish():
 out.mkdir(parents=True,exist_ok=True)
 (out/'patrol_protocol_audit.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
atexit.register(finish)
if __name__=='__main__':raise SystemExit(R.main())
'''

def prepare():
 assert not OUT.exists(),'Do not duplicate long-patrol preparation'
 OUT.mkdir();shutil.copytree(M.OUT/'execution_tree',OUT/'execution_tree',ignore=shutil.ignore_patterns('__pycache__'))
 for name in('nav_fw.dll','scene.json','cases.json'):shutil.copyfile(M.OUT/name,OUT/name)
 path=OUT/'execution_tree/sim/closed_loop/final_long_patrol_case.py';path.write_text(ENTRY,encoding='utf8')
 manifest=A.read(M.OUT/'manifest.json');template=next(j for j in manifest['jobs']if j['profile']=='control256'and j['case']=='turn_loop'and j['seed']==1)
 command=list(template['command']);command[3]=str(path);command[command.index('--firmware-dll')+1]=str(OUT/'nav_fw.dll');command[command.index('--time')+1]='330'
 for profile in('control256','best128'):
  pass # Best128 is bound only after all12 rows per offset are available.
 static=dict(recorded_at=datetime.now().astimezone().isoformat(),final_task=manifest['final_task'],production128=True,control256_simulation_only=True,
  directory=str(OUT),source_stage1=str(M.OUT),same_route='original field3x5_v1 turn_loop four waypoints; original builtin loop flag1, not finite concatenated points',
  duration_s=330,min_actual_simulated_duration_s=300,seed=1,nomap=True,ZHI=1500,DLL_rebuilt=False,
  base_command=command,environment_template=template['environment'],best128_offset_pending_full_multiseed=True,
  minimum_physical_gib=2,minimum_commit_gib=3,one_worker_shared_with_stage1=True,do_not_launch_while_multiseed_worker_running=True,
  acceptance=dict(valid_XY_P95_mm=50,no_localization_loss=True,no_added_collision=True,multiple_laps_required=True,
   drift='Report60s window RMSE/P95/endpoint/max and linear XYerror trend; numeric sustained-growth threshold unspecified, no invented pass'),
  route_completion='Count actual native goal advances and completed ordered cycles from read-only py_get_state; do not count elapsed time as completed laps',
  stop_criteria='Original terminal judge unchanged; native wp_loop avoids normal final ARRIVED. Timeout outcome is expected diagnostic for ongoing patrol.',
  source_scope=manifest['scope'],bindings=[metadata(p)for p in(OUT/'execution_tree').rglob('*')if p.is_file()and'__pycache__'not in p.parts]+[metadata(OUT/'nav_fw.dll')])
 assert same_bytes(M.OUT/'nav_fw.dll',OUT/'nav_fw.dll')
 A.write(OUT/'manifest.json',static);A.write(OUT/'job.json',dict(phase='prepared_waiting_best_distance_and_single_worker',planned=2,completed=0,actual_simulation_started=False));return static

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--prepare',action='store_true');a=p.parse_args()
 if a.prepare:print(json.dumps(dict(root=str(OUT),planned=2,duration_s=prepare()['duration_s']),ensure_ascii=False))
