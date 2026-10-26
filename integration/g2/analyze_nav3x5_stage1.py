"""Read-only scoring supplement; absolute XY metrics from saved N1c trace."""
from pathlib import Path
import argparse,csv,json,math,datetime,time
BASE=Path('F:/nav_integration_g2_20261023/nav3x5_N1c_SLF128_256_stage1_20261026')
def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def stats(values):
 if not values:return dict(samples=0,RMSE_mm=None,P95_mm=None,endpoint_mm=None,max_mm=None)
 ordered=sorted(values);i=(len(values)-1)*.95;lo=int(i);hi=min(lo+1,len(values)-1);p95=ordered[lo]+(ordered[hi]-ordered[lo])*(i-lo)
 r=dict(samples=len(values),RMSE_mm=math.sqrt(sum(v*v for v in values)/len(values)),P95_mm=p95,endpoint_mm=values[-1],max_mm=max(values))
 return r
def run(root):
 manifest=load(root/'manifest.json');rows=[];traces={}
 for job in manifest['jobs']:
  p=Path(job['out']);mp=p/'metrics.json'
  if not mp.exists():continue
  m=load(mp);terminal=m['terminal'];csvp=p/'timeseries.csv';full=[];valid=[];last_valid=False;trace={}
  if csvp.exists():
   with csvp.open(encoding='utf8')as f:
    for row in csv.DictReader(f):
     error=1000*math.hypot(float(row['ex'])-float(row['x']),float(row['ey'])-float(row['y']));full.append(error);last_valid=bool(int(float(row['loc_valid'])))
     if last_valid:valid.append(error)
     trace[float(row['t'])]=dict(t=float(row['t']),truth_x=float(row['x']),truth_y=float(row['y']),XY_error_mm=error,localization_valid=last_valid)
  traces[job['case'],job['SLF_SN'],job['prior']]=trace
  false=terminal['outcome']=='false_arrival'or bool(terminal['declared_error_m']is not None and terminal['declared_error_m']>terminal['criteria']['position_m'])
  rows.append(dict(case=job['case'],seed=job['seed'],SLF_SN=job['SLF_SN'],SLF_SN_LOG2=job['SLF_SN_LOG2'],prior=job['prior'],arrived=m['arrived'],outcome=m['outcome'],terminal_outcome=terminal['outcome'],false_arrival=false,collision=m['collisions']>0,collisions=m['collisions'],firmware_arrived=m['firmware_arrived'],goal_error_mm=1000*m['goal_err_m'],XY_full_trajectory=stats(full),XY_when_localization_valid=stats(valid),last_sample_localization_valid=last_valid,loc_valid_fraction=m.get('loc_valid_fraction'),align_s=m['align_s'],n_align=m['n_align'],n_align_fail=m['n_align_fail'],align_q=m['align_q'],loc_final=m['loc_final'],t_degraded_s=m['t_degraded_s'],n_lost=m['n_lost'],t_lost_s=m['t_lost_s'],stopping_terminal=terminal,time_s=m['time_s'],wall_s=m['wall_s'],metrics=str(mp),trace=str(csvp)))
 groups=[]
 for sn in(256,128):
  for prior in(False,True):
   selected=[r for r in rows if r['SLF_SN']==sn and r['prior']==prior]
   groups.append(dict(SLF_SN=sn,prior=prior,arrivals_k=sum(r['arrived']for r in selected),completed_n=len(selected),false_arrivals=sum(r['false_arrival']for r in selected),collision_runs=sum(r['collision']for r in selected)))
 pairs=[]
 for case in manifest['cases']:
  for prior in(False,True):
   selected={r['SLF_SN']:r for r in rows if r['case']==case and r['prior']==prior}
   if set(selected)!={128,256}:continue
   a,b=selected[256],selected[128];metrics=('RMSE_mm','P95_mm','endpoint_mm','max_mm');deltas={}
   for phase in('XY_full_trajectory','XY_when_localization_valid'):
    delta={k:(b[phase][k]-a[phase][k])if a[phase][k]is not None and b[phase][k]is not None else None for k in metrics}
    deltas[phase]=dict(delta_128_minus256_mm=delta,worsening_le10mm_all_four=all(v is not None and v<=10 for v in delta.values()))
   ta,tb=traces[case,256,prior],traces[case,128,prior];same_time=[dict(t=t,truth256=[ta[t]['truth_x'],ta[t]['truth_y']],truth128=[tb[t]['truth_x'],tb[t]['truth_y']],error256_mm=ta[t]['XY_error_mm'],error128_mm=tb[t]['XY_error_mm'],delta128_minus256_mm=tb[t]['XY_error_mm']-ta[t]['XY_error_mm'],both_localization_valid=ta[t]['localization_valid']and tb[t]['localization_valid'])for t in sorted(ta.keys()&tb.keys())]
   temporal=dict(first_delta_gt10mm=next((r for r in same_time if r['delta128_minus256_mm']>10),None),largest_same_time_delta=max(same_time,key=lambda r:r['delta128_minus256_mm'])if same_time else None,scope='same physical timestamps; after motion commands diverge these need not be the same physical locations')
   pairs.append(dict(case=case,prior=prior,seed=a['seed'],absolute256={phase:a[phase]for phase in deltas},absolute128={phase:b[phase]for phase in deltas},incremental_error=deltas,same_time_trace_diagnostic=temporal,goal_error_delta128_minus256_mm=b['goal_error_mm']-a['goal_error_mm'],outcome256=a['outcome'],outcome128=b['outcome'],both_arrived_without_collision_or_false_arrival=all(r['arrived']and not r['collision']and not r['false_arrival']for r in(a,b)),production_RTL_change_approved=False))
 result=dict(recorded_at=datetime.datetime.now().astimezone().isoformat(),completed=len(rows),planned=len(manifest['jobs']),accepted_deployable=False,first_pilot_seed1_only=True,execution_scope='frozen N1c navigation C DLL + fixed-point perception + virtual lidar + velocity-command kinematic robot; SAFE/RL not in loop; no Torch4500/RTL/board proof',response_assumptions=dict(safe_in_loop=False,rl_in_loop=False),independent_real4500_evidence_not_combined_as_vehicle_loop=True,success_rates_reported_k_over_n=True,shared_failures_do_not_qualify_as_success=True,XY_error_unit='mm; absolute truth-minus-estimate Euclidean error; both full and localization-valid samples retained',comparison_threshold='128 minus256 error worsening <=10mm; NOT absolute error <=10mm',XY_P95_method='linear interpolation between saved samples',CSV_coordinates_resolution_m=.0001,original_terminal_position_criterion_mm=150,subcentimeter_accuracy_accepted=False,rows=rows,pairs=pairs,groups=groups,next_validation_required=['at least two additional fixed seeds','prior initial-pose offset cases','long-axis end occlusion with separately versioned scene/cache'])
 temp=root/'precision_summary.tmp';temp.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8');temp.replace(root/'precision_summary.json');print(json.dumps({k:v for k,v in result.items()if k not in('rows','pairs')},ensure_ascii=False),flush=True);return result
def watch(root):
 import psutil
 me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS);me.cpu_affinity([psutil.cpu_count()-1])
 with(root/'precision_watch_claim.json').open('x')as f:json.dump(dict(pid=me.pid,created_epoch=me.create_time(),read_only_to_simulation=True,only_updates_precision_summary=True),f)
 old=None
 while True:
  files=list((root/'results').glob('*/metrics.json'));now=[(str(p),p.stat().st_size,p.stat().st_mtime_ns)for p in files]
  if now!=old:
   try:run(root);old=now
   except json.JSONDecodeError:pass # producer may still be closing its metrics file
  state=load(root/'job.json')
  if state['phase']=='all_stage1_cases_complete':return 0
  if(root/'failure.json').exists()or state['phase']=='own_simulation_stopped_for_memory_preserved':return 1
  pid=state['controller_pid'];present=psutil.pid_exists(pid)and abs(psutil.Process(pid).create_time()-state['controller_created_epoch'])<1
  if not present:return 2
  time.sleep(30)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--root',type=Path,default=BASE);p.add_argument('--watch',action='store_true');a=p.parse_args()
 if a.watch:raise SystemExit(watch(a.root))
 else:run(a.root)
