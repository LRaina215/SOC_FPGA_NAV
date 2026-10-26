"""Read completed330s model traces and observe the existing stage1b process.

This entry never imports the model/DLL, starts a worker, mutates an input or
runs a build. It recomputes descriptive XY statistics from already-written
CSV files and records live process identities. Threshold durations below are
descriptions, not newly invented localization/loss acceptance thresholds.
"""
import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path

import psutil
from prepare_baseline_fusion_retime import metadata

BASE=Path('F:/nav_integration_g2_20261023')
PATROL=BASE/'nav3x5_N1c_final128_long_patrol_20261026'
OBSERVER=BASE/'stage1b_observer_serial_prepared1'


def load(p):return json.loads(p.read_text(encoding='utf8'))


def identity(pid,expected_created=None):
    try:
        p=psutil.Process(pid)
        created=p.create_time()
        return dict(pid=pid,exists=True,created_epoch=created,ppid=p.ppid(),
            status=p.status(),command=p.cmdline(),
            expected_identity_matches=expected_created is None or abs(created-expected_created)<.01)
    except psutil.NoSuchProcess:return dict(pid=pid,exists=False,expected_identity_matches=False)


def percentile(values,p):
    v=sorted(values);at=(len(v)-1)*p;lo=math.floor(at);hi=math.ceil(at)
    return v[lo]+(v[hi]-v[lo])*(at-lo)


def stat(rows):
    e=[r['error_mm']for r in rows]
    mean_t=sum(r['t']for r in rows)/len(rows);mean_e=sum(e)/len(e)
    numerator=sum((r['t']-mean_t)*(r['error_mm']-mean_e)for r in rows)
    denominator=sum((r['t']-mean_t)**2 for r in rows)
    return dict(samples=len(e),begin_s=rows[0]['t'],end_s=rows[-1]['t'],
        XY_P95_mm=percentile(e,.95),XY_RMS_mm=math.sqrt(sum(x*x for x in e)/len(e)),
        XY_max_mm=max(e),XY_last_mm=e[-1],
        linear_error_magnitude_trend_mm_per_min=60*numerator/denominator if denominator else None)


def longest_sample_run(rows,predicate):
    best=[];current=[]
    for r in rows:
        if predicate(r):
            current.append(r)
            if len(current)>len(best):best=list(current)
        else:current=[]
    return dict(samples=len(best),first_s=best[0]['t']if best else None,
        last_s=best[-1]['t']if best else None,
        observed_sample_span_s=best[-1]['t']-best[0]['t']if best else 0)


def completed_patrol(name,selected):
    directory=PATROL/'results'/name
    paths=[directory/n for n in('metrics.json','long_patrol_analysis.json','patrol_protocol_audit.json','runtime.json','timeseries.csv','window_audit.json')]
    before=[metadata(p)for p in paths]
    metric,analysis,protocol,runtime=map(load,paths[:4])
    assert runtime['exit']==0
    child=identity(runtime['child_pid'],runtime['child_creation'])
    assert not(child['exists']and child['expected_identity_matches']),('original patrol child still exists',child)
    assert metric['simulation_end_s']==analysis['actual_simulated_duration_s']==330
    with paths[4].open(encoding='utf8',newline='')as f:
        rows=[{k:float(v)for k,v in row.items()}for row in csv.DictReader(f)]
    assert len(rows)==3300 and rows[0]['t']==.1 and rows[-1]['t']==330
    assert all(rows[i]['t']<rows[i+1]['t']for i in range(len(rows)-1))
    for r in rows:r['error_mm']=1000*math.hypot(r['ex']-r['x'],r['ey']-r['y'])
    full=stat(rows)
    assert abs(full['XY_P95_mm']-analysis['valid_XY_P95_mm'])<.01
    assert abs(full['linear_error_magnitude_trend_mm_per_min']-analysis['full_error_linear_trend_mm_per_min'])<.01
    window=load(paths[5]);profile=window['profile']
    assert profile['storage_SN']==selected['SLF_SN']
    goals=[r for r in protocol['state_goal_transitions']if r['mode']==1 and r['state']==1]
    packet=protocol['packets'][0]
    assert packet['loop_bit']==1 and packet['n']==len(packet['waypoints_mm'])==4
    for i,g in enumerate(goals):assert g['goal_mm']==packet['waypoints_mm'][i%4]
    assert len(goals)==analysis['ordered_native_goal_advances']
    assert analysis['completed_cycles']==(len(goals)-1)//4
    windows=[]
    for begin,end in((0,60),(120,180),(180,240),(240,300),(270,330)):
        windows.append(dict(requested_interval_s=[begin,end],**stat([r for r in rows if begin<r['t']<=end])))
    excursions=[]
    for mm in(100,200,400):
        matching=[r for r in rows if r['error_mm']>mm]
        first=matching[0]if matching else None
        before_goal=[g for g in goals if first and g['t_us']/1e6<=first['t']]
        after_goal=[g for g in goals if first and g['t_us']/1e6>first['t']]
        excursions.append(dict(descriptive_threshold_mm=mm,samples_above=len(matching),
            first_above_s=matching[0]['t']if matching else None,
            longest_contiguous_sample_run=longest_sample_run(rows,lambda r:r['error_mm']>mm),
            first_crossing_existing_truth_estimate_command_sample=first,
            last_native_goal_change_before=before_goal[-1]if before_goal else None,
            next_native_goal_change_after=after_goal[0]if after_goal else None,
            successful_match_at_crossing=None,update_at_crossing=None,relocalization_at_crossing=None,
            absent_event_reason='Old window score/update/build records have no timestamp, accepted winner/solve result or per-apply C event. Do not assign event index to time without a separate exact callback schedule proof.',
            acceptance_threshold=False))
    event_inventory={}
    for op in sorted({e['op']for e in window['events']}):
        events=[e for e in window['events']if e['op']==op]
        event_inventory[op]=dict(count=len(events),all_recorded_keys=sorted(set(k for e in events for k in e)),
            timestamp_fields_present=sorted(set(k for e in events for k in e if k in('t','t_s','t_us','time','time_us','C_now_us'))))
    jumps=[]
    for prev,now in zip(rows,rows[1:]):
        jumps.append(dict(t_s=now['t'],previous_t_s=prev['t'],
            error_vector_step_mm=1000*math.hypot((now['ex']-now['x'])-(prev['ex']-prev['x']),
                                                (now['ey']-now['y'])-(prev['ey']-prev['y'])),
            resulting_error_mm=now['error_mm']))
    assert before==[metadata(p)for p in paths],'completed model output changed during read'
    return dict(name=name,profile=profile,selected_job=dict(SLF_SN=selected['SLF_SN'],
        forward_mm=selected['forward_mm'],prior=selected['prior'],seed=selected['seed'],case=selected['case']),
        natural_exit=runtime['exit'],original_child=child,simulated_duration_s=330,
        native_goal_selections_including_initial=len(goals),native_post_initial_goal_changes=len(goals)-1,
        native_ordered_complete_waypoint_cycles=analysis['completed_cycles'],unexpected_goal_events=analysis['unexpected_goal_events'],
        native_goal_progress_scope='Native firmware advanced goals in correct order. Initial goal selection is not an arrival; cyclic progression is not a truth/dwell/stop acceptance for every waypoint.',
        fixed_duration_terminal_outcome=metric['outcome'],terminal_single_goal_accepted=metric['terminal']['valid'],
        collision_count=metric['collisions'],full_trace_descriptive_statistics=full,interval_statistics=windows,
        error_excursions_descriptive_only=excursions,reported_match_applied=metric['match_applied'],
        original_matching_event_inventory=event_inventory,
        largest_observed100ms_error_vector_steps=sorted(jumps,key=lambda x:x['error_vector_step_mm'],reverse=True)[:5],
        old_matching_event_limits=['Window score records retain only effective_n_valid from the first returned candidate, total candidate count and geometry; no best-candidate score or applied winner/delta',
            'Original window updates are callbacks, not proof of a successful firmware solve; only aggregate match_applied is available',
            'No per-poll n_applied/degen/delta/status history, no C clock timestamp on score/update/build, no timed no-map relocalization/skip/reject reason',
            'Exact successful-match/update/relocalization assignment at first drift samples cannot be reconstructed from these old records alone'],
        reported_n_lost=metric['n_lost'],loc_valid_csv_values=sorted(set(r['loc_valid']for r in rows)),
        no_map_loss_free_or_continuous_valid_localization_proved=False,
        original_metrics_precision=dict(XY_RMS_m_rounded=metric['loc_err_xy_rms_m'],XY_final_m_rounded=metric['loc_err_xy_final_m'],XY_max_m_rounded=metric['loc_err_xy_max_m']),
        original_analysis_drift_acceptance=analysis['sustained_drift_pass'],
        original_analysis_limit=analysis['drift_limit'],input_bindings=before,inputs_unchanged=True)


def observer_snapshot():
    # Job and summary are naturally mutable. Each snapshot gets its metadata;
    # a transition between children is not classified as a model failure.
    begin=datetime.now().astimezone().isoformat()
    job=load(OBSERVER/'job.json');summary=load(OBSERVER/'summary.json')
    controller=identity(job['controller_pid'],job['controller_created_epoch'])
    child=identity(job['child_pid'],job['child_created_epoch'])if'child_pid'in job else None
    if child and child['exists']and child['expected_identity_matches']:
        child['parent_is_actual_controller']=child['ppid']==job['controller_pid']
        child['command_matches_job']=child['command']==job['command']
        assert child['parent_is_actual_controller']and child['command_matches_job']
    descendants=[]
    if controller['exists']and controller['expected_identity_matches']:
        for p in psutil.Process(job['controller_pid']).children(recursive=True):
            try:
                if 'python' in p.name().lower():descendants.append(identity(p.pid))
            except psutil.NoSuchProcess:pass
    assert len(descendants)<=1,('more than one observed model Python worker',descendants)
    ready=load(OBSERVER/'original_patrols_finished.json')
    assert ready['passed']and len(ready['evidence'])==2
    rows=[]
    for r in summary.get('rows',[]):
        m=r['metric'];o=r['observed']
        rows.append(dict(name=r['name'],outcome=m['outcome'],simulated_end_s=m['simulation_end_s'],
            collisions=m['collisions'],terminal=m['terminal'],
            XY_RMS_m=m['loc_err_xy_rms_m'],XY_max_m=m['loc_err_xy_max_m'],
            original_poll_samples=o['original_poll_samples'],dispatch_callbacks=o['dispatch_callbacks'],
            update_callbacks=o['update_callbacks'],extra_py_poll=o['extra_py_poll'],
            extra_py_get_commit=o['extra_py_get_commit'],DLL_rebuilt=o['DLL_rebuilt'],
            no_map_loss_verdict=o['no_map_loss_verdict'],no_map_loss_proved=r['no_map_loss_proved']))
    return dict(snapshot_begin=begin,snapshot_end=datetime.now().astimezone().isoformat(),
        automatically_continued_after_original_patrols=True,patrol_finished_binding=metadata(OBSERVER/'original_patrols_finished.json'),
        job=job,job_snapshot_binding=metadata(OBSERVER/'job.json'),summary_snapshot_binding=metadata(OBSERVER/'summary.json'),
        actual_controller=controller,actual_child=child,actual_Python_descendants=descendants,
        completed=summary['completed'],planned=summary['planned'],finite_completed_rows=rows,
        all16_finished=summary['completed']==summary['planned'],
        moving_files_sampled_read_only=True,child_may_finish_between_snapshot_fields=True,
        no_automatic_model_or_deployment_acceptance=summary.get('automatic_safety_or_deployment_acceptance')is False)


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args();assert not a.out.exists()
    job=load(PATROL/'job.json');summary=load(PATROL/'summary.json');selected=load(PATROL/'selected_jobs.json')
    assert job['phase']=='complete'and summary['completed']==2
    byname={r['name']:r for r in selected['jobs']}
    rows=[completed_patrol(n,byname[n])for n in('patrol330s_control256_s1','patrol330s_best128_s1')]
    source=PATROL/'execution_tree/sim/closed_loop/run_cl.py'
    code=source.read_text(encoding='utf8')
    assert 'localized = pkg is None or (align_t is not None and loc_v[0] in (2, 3))'in code
    lines=[dict(line=i,text=s.strip())for i,s in enumerate(code.splitlines(),1)
           if 'localized = pkg is None' in s or '"n_lost":'in s or '"loc_final":'in s]
    observer=observer_snapshot()
    r=dict(read_only_review_completed=True,recorded_at=datetime.now().astimezone().isoformat(),
        reviewer=metadata(Path(__file__).resolve()),original_patrol_job=metadata(PATROL/'job.json'),
        original_summary=metadata(PATROL/'summary.json'),cases=rows,
        localization_validity_evidence=dict(source=metadata(source),source_lines=lines,
            actual_no_prior_branch_sets_loc_valid_true_without_loss_test=True,
            all3300_loc_valid1_and_n_lost0_are_not_independent_no_map_health_evidence=True),
        comparison=dict(same_seed_scene_duration_no_prior=True,
            confounded_factors='256 rolling/0mm compared with128 forward/1500mm; not a one-variable map-size causal experiment',
            collision_count_both=0,complete_native_cycles_both=9,
            error_P95_delta_128_minus256_mm=rows[1]['full_trace_descriptive_statistics']['XY_P95_mm']-rows[0]['full_trace_descriptive_statistics']['XY_P95_mm'],
            reasonable_inference='This128/forward1500 seed1 run develops a large late XY error increase while goal progression continues. Collision-free progression does not prove continuous localization.',
            root_cause_known=False),
        observer=observer,
        finite_conclusions=['Both330s children exited0 naturally and continued cyclic native waypoint progression with0 observed collisions',
            '128/forward1500 has materially worse late error than256/rolling in this one seed; relaxedP95 does not waive long-drift/localization/stop requirements',
            'No-map health cannot be accepted from the original constant loc_valid flag or n_lost0',
            'Stage1b truly started automatically after patrol completion; its current identity and partial outcomes are recorded, not treated as all16 completion'],
        unproved=['Statistical reliability across long-patrol seeds and other scenes/ZHI/prior profiles',
            'Whether128 size, forward offset, aliasing, feature visibility or solver rejection causes the late error',
            'True continuous XY observability, exact solver failure/skip/at_edge counts and no-map no-loss status',
            'Truth/dwell/stop acceptance at every cyclic waypoint; fixed330s timeout is not a completed terminal-goal verdict',
            'Real FPGA CPU timing, full4500 integration, physical IO/sensors/RK/stop and vehicle deployment'],
        P95_50mm_hard_gate=False,long_drift_waived=False,all_model_or_hardware_accepted=False,
        model_or_DLL_imported=False,new_model_started=False,source_input_modified=False,
        new_simulation_or_build_started=False,no_SHA=True)
    a.out.mkdir(parents=True)
    (a.out/'result.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(dict(result=str(a.out/'result.json'),P95_mm=[v['full_trace_descriptive_statistics']['XY_P95_mm']for v in rows],
        observer_completed=observer['completed'],observer_planned=observer['planned'],
        observer_child=observer['actual_child'],new_model_started=False),ensure_ascii=False))


if __name__=='__main__':main()
