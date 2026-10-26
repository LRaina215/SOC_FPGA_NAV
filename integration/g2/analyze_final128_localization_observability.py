"""Read existing N1c traces without interpreting constant no-map loc_valid as lock."""
from pathlib import Path
import argparse,csv,json,math
from datetime import datetime
import numpy as np
import stage1_final_nav3x5_multiseed as S
import stage1a_nav3x5_window_ablation as A

def spans(mask):
 out=[];start=None
 for i,value in enumerate(np.append(mask,False)):
  if value and start is None:start=i
  if not value and start is not None:out.append([start,i-1,i-start]);start=None
 return out

def inspect_cell(job,dest):
 root=Path(job['out']);metrics=A.read(root/'metrics.json')
 with(root/'timeseries.csv').open(encoding='utf8')as f:rows=list(csv.DictReader(f))
 t=np.array([float(r['t'])for r in rows]);error=np.array([1000*math.hypot(float(r['ex'])-float(r['x']),float(r['ey'])-float(r['y']))for r in rows]);valid=np.array([int(float(r['loc_valid']))for r in rows],bool)
 with np.load(root/'final_maps.npz')as z:match=z['match_log'].copy()
 windows=[]
 for begin in np.arange(0,max(t[-1],.1),10):
  mask=(t>=begin)&(t<begin+60)
  if mask.sum():windows.append(dict(begin_s=float(begin),end_s=float(min(begin+60,t[-1])),samples=int(mask.sum()),
   RMSE_mm=float(np.sqrt(np.mean(error[mask]**2))),P95_mm=float(np.percentile(error[mask],95)),endpoint_mm=float(error[mask][-1]),max_mm=float(error[mask].max())))
 applied=match[:,11].astype(np.int64);changes=np.r_[False,np.diff(applied)>0]
 first=int(np.flatnonzero(changes)[0])if changes.any()else None
 no_success=spans(~changes);post=spans((~changes)&(np.arange(len(match))>=(first or 0)))
 nonapplied=spans(match[:,0]==0);lowvalid=spans(match[:,5]<60)
 cadence=A.read(root/'manifest.json')['arguments']['fps']
 assert cadence==10,'Keep actual frame cadence explicit'
 moving_command=np.array([(float(r['cmd_vx'])!=0 or float(r['cmd_wz'])!=0)for r in rows],bool)
 paired_moving=moving_command[:len(match)]if len(moving_command)>=len(match)else None
 moving_gaps=spans((~changes)&paired_moving)if paired_moving is not None else None
 age=np.zeros(len(match));last=None
 for i,changed in enumerate(changes):
  if changed:last=i
  age[i]=np.nan if last is None else(i-last)/10
 item=dict(name=job['name'],case=job['case'],seed=job['seed'],profile=job['profile'],prior=job['prior'],
  no_map_constant_loc_valid=not job['prior'],declared_prior_loss=dict(n_lost=metrics['n_lost'],t_lost_s=metrics['t_lost_s'],n_align_fail=metrics['n_align_fail']),
  no_localization_loss_proved=(metrics['n_lost']==0 and metrics['t_lost_s']==0)if job['prior']else None,
  valid_XY_P95_mm=float(np.percentile(error[valid],95))if valid.any()else None,
  full_XY_P95_mm=float(np.percentile(error,95)),full_XY_RMSE_mm=float(np.sqrt(np.mean(error**2))),
  full_error_linear_trend_mm_per_min=float(np.polyfit(t,error,1)[0]*60)if len(t)>1 else None,
  rolling60s_windows_every10s=windows,actual_match_counter=dict(commit_samples=len(match),successful_solve_total=int(applied[-1])if len(match)else 0,
   successful_counter_increases_observed=int(changes.sum()),first_increase_commit_index=first,
   no_counter_increase_spans_commit_indices=no_success,post_first_success_no_counter_increase_spans=post,
   longest_no_counter_increase_samples_after_first_success=max((v[2]for v in post),default=0),
   longest_no_counter_increase_nominal_s_after_first_success=max((v[2]/10 for v in post),default=0),
   latest_solve_not_applied_observed_spans=nonapplied,latest_solve_n_valid_below60_observed_spans=lowvalid,
   nonzero_command_no_counter_increase_spans=moving_gaps,
   longest_nominal_no_counter_increase_with_nonzero_command_s=max((v[2]/cadence for v in moving_gaps),default=0)if moving_gaps is not None else None,
   movement_classification='exact cmd_vx!=0 or cmd_wz!=0; command sample is nearest following tlog, not proof of physical movement or precise solve timestamp',
   fraction_commit_samples_latest_solve_not_applied=float(np.mean(match[:,0]==0)),fraction_commit_samples_latest_valid_below60=float(np.mean(match[:,5]<60)),
   nominal_success_observation_age_max_s=float(np.nanmax(age))if np.isfinite(age).any()else None,
   nominal_frame_sample_period_s=.1,exact_event_timestamps_available=False),
  limits=['match_log sampled once at frame commit; last solve state can repeat, so spans are observed statuses, not exact rejected invocation counts.',
   'Nominal seconds use requested10Hz frame cadence; exact solve timestamps and interframe arrival gaps not logged. Do not treat as precise freshness WCET.',
   'No-map runner marks localized=True unconditionally and no-map FW has no configured loss-age threshold. P95(valid) equals full P95 here but cannot establish lock.',
   'Prior loc.lost_us=2000000 and edge criterion are existing prior-mode semantics only; do not apply them as invented no-map acceptance.',
   'No numeric sustained-drift threshold is specified; slope/windows are data, not an invented pass/fail.'])
 dest.mkdir(parents=True,exist_ok=True)
 with(dest/f"{job['name']}_error_curve.csv").open('w',encoding='utf8',newline='')as f:
  w=csv.writer(f);w.writerow(['t_s','error_xy_mm','runner_loc_valid','truth_x_m','truth_y_m','estimated_x_m','estimated_y_m'])
  for tt,ee,r in zip(t,error,rows):w.writerow([tt,ee,r['loc_valid'],r['x'],r['y'],r['ex'],r['ey']])
 A.write(dest/f"{job['name']}.json",item);return item

def analyze(root):
 state=A.read(root/'job.json')if(root/'job.json').exists()else{}
 manifest=A.read(Path(state['active_manifest']))if state.get('active_manifest')else A.read(root/'manifest.json')
 directory=root/'localization_observability';items=[]
 for job in manifest['jobs']:
  if(Path(job['out'])/'metrics.json').exists():items.append(inspect_cell(job,directory))
 result=dict(recorded_at=datetime.now().astimezone().isoformat(),completed=len(items),rows=items,
  frozen_fw_rules=dict(minimum_valid_points=60,solve_false_conditions=['candidate count differs from configured245','nmax<60'],
   solve_with_degenerate_axes='Can still return true and increment n_applied even with zero corrections; not equated with full XY observability.',
   source=str(root/'firmware_source/nav/pose_match.c'),no_map_loss_threshold=None,
   prior_default_lost_us=2000000,prior_rule='low score continuously beyond lost_us or edge_cnt reaches configured edge_n; prior mode only',
   update_not_attempted_conditions=['no pending eligible frame','attitude unsettled frame within original200ms exclusion','original LUT not ready','map receiving','matching disabled',
    'stale frame after SET_POSE/MAP_BEGIN','prior LOC_LOST or ALIGN turning'],
   no_map_match_update_age_timeout=None,source_nav_app=str(root/'firmware_source/app/nav_app.c'),
   no_counter_increment_is_not_always_candidate_rejection=True),
  model_scope=manifest['scope'])
 A.write(root/'localization_observability.json',result);return result

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--root',type=Path,default=S.OUT);a=p.parse_args();v=analyze(a.root);print(json.dumps(dict(completed=v['completed'],output=str(a.root/'localization_observability.json'))))
