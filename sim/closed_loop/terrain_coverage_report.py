"""Separate N2 follow-up checkpoint/index; preserves the prior94-run study."""
import json
from pathlib import Path
import re
import subprocess
import numpy as np
from terrain_study import OUT,ROOT,dump


def read(path):return json.loads(path.read_text(encoding='utf-8'))


def main():
    base=OUT/'n2av';latest=['AY_flat','AY_trace_up8','AY_trench40d150'];rows=[]
    for p in sorted((base/'navigation').iterdir()):
        if not (p/'metrics.json').exists():continue
        m=read(p/'metrics.json')
        rows.append(dict(id=p.name,latest=p.name in latest,outcome=m['outcome'],time_s=m['time_s'],end_s=m['simulation_end_s'],
            collisions=m['collisions'],final_true=m['final_true'],goal_err_m=m['goal_err_m'],
            video=str((p/'video.mp4').resolve()) if (p/'video.mp4').exists() else None,
            evidence=str(p.resolve()),independence='same deterministic seed; diagnostic repetitions are not independent samples'))
    physical=[];videos=[]
    repeated={}
    ffmpeg=ROOT/'sim/build/review_videos_20261021/tools/ffmpeg-win-x86_64-v7.1.exe'
    for name in latest:
        p=base/'navigation'/name
        previous=base/'navigation'/('AX_up8' if name=='AY_trace_up8' else name.replace('AY_','AX_'))
        repeated[name]={}
        for file in ['final_maps.npz','nav_events.npz','map_frames.npz','support_frames.npz','physics_backend/full_state.npz']:
            with np.load(p/file) as aa,np.load(previous/file) as bb:
                repeated[name][file]={k:bool(np.array_equal(aa[k],bb[k],equal_nan=True)) for k in aa.files if aa[k].dtype.kind not in ['O','U','S']}
            assert all(repeated[name][file].values()),(name,file)
        with np.load(p/'physics_backend/full_state.npz') as z:
            finite={k:bool(np.isfinite(z[k]).all()) for k in z.files};t=z['t'];dt=np.diff(t)
            assert all(finite.values()) and np.allclose(dt,.002,rtol=0,atol=1e-8)
            physical.append(dict(id=name,finite=finite,states=len(t),start_s=float(t[0]),end_s=float(t[-1]),continuous_2ms=True))
        v=p/'video.mp4';result=subprocess.run([str(ffmpeg),'-hide_banner','-i',str(v),'-f','null','NUL'],capture_output=True,text=True,errors='replace',check=True)
        count=re.findall(r'frame=\s*(\d+)',result.stderr);assert count
        duration=int(count[-1])/5;assert float(t[-1])<=duration<=float(t[-1])+.201
        videos.append(dict(id=name,decoded_frames=int(count[-1]),fps=5,duration_s=duration,record_end_s=float(t[-1]),full_decode=True))
    dump(base/'state_integrity.json',physical);dump(base/'video_validation.json',videos);dump(base/'trial_index.json',rows)
    dump(base/'AY_same_recorded_trajectories.json',repeated)
    csv=np.genfromtxt(base/'navigation/AY_trench40d150/physics_backend/physics.csv',names=True,delimiter=',')
    execution={}
    for label,v,w in [('request','request_vx','request_wz'),('safe','safe_vx','safe_wz')]:
        nz=np.flatnonzero((csv[v]!=0)|(csv[w]!=0));i=int(nz[-1]+1);s=csv[i:]
        execution[label]=dict(last_nonzero_s=float(csv['t'][i-1]),zero_suffix_start_s=float(s['t'][0]),
            net_displacement_mm=float(np.hypot(s['x'][-1]-s['x'][0],s['y'][-1]-s['y'][0])*1000),
            path_mm=float(np.hypot(np.diff(s['x']),np.diff(s['y'])).sum()*1000),
            start_xy=[float(s['x'][0]),float(s['y'][0])],end_xy=[float(s['x'][-1]),float(s['y'][-1])])
    dump(base/'AY_zero_request_motion.json',dict(trial='AY_trench40d150',sample_dt_s=.01,execution=execution,
        calibrated_real_stopping_envelope=False))
    lines=['# N2覆盖/路径接续证据','','当前候选AY未接受；生产C仍AS，接受算法仍N1c。此前94例索引不包含本目录。',
        '同种子重复诊断不构成独立实车样本；AX一致性分支假到达保留。仅最新AY三例补齐本轮视频。','',
        '| 运行 | 最新候选 | 结果 | 时间(s) | 真实误差(m) | 视频 |','|---|---|---|---:|---:|---|']
    for r in rows:
        video=f'[视频](navigation/{r["id"]}/video.mp4)' if r['video'] else '仅保存完整状态/图'
        lines.append(f'| {r["id"]} | {"是" if r["latest"] else "历史/诊断"} | {r["outcome"]} | {r["time_s"]} | {r["goal_err_m"]} | {video} |')
    (base/'VIDEO_INDEX.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    # Confirm candidate patch application did not alter live AS production C/H.
    frozen=ROOT/'sim/build/nav_opt/n2as_near_retry_source/fw';checked=[]
    for src in frozen.rglob('*'):
        if src.suffix not in ('.c','.h') or src.relative_to(frozen).as_posix()=='include/nav_regs_map.h':continue
        target=ROOT/'fw'/src.relative_to(frozen)
        assert target.exists() and src.read_bytes()==target.read_bytes(),str(target)
        checked.append(src.relative_to(frozen).as_posix())
    batch=ROOT/'sim/build/batch/nav_opt_n2ay_coverage_surface_probes3_20261022'
    keys=[]
    for name in ['f5_ramp_s3','f5_gap_s3']:
        p=batch/name/'metrics.json'
        if p.exists():
            m=read(p);keys.append(dict(case=name,outcome=m['outcome'],time_s=m['time_s'],collisions=m['collisions'],goal_err_m=m['goal_err_m']))
    diag=read(base/'AY_candidate_diagnosis.json');noise=read(base/'AY_interval_noise_audit/result.json')
    checkpoint=dict(accepted=False,deployed=False,accepted_baseline='N1c',live_firmware='AS, unchanged apart from generated register header',
        candidate='AY',candidate_patch=str(ROOT/'sim/closed_loop/candidates/n2_support_coverage.patch'),
        candidate_source=str((base/'AY_verified/fw').resolve()),live_source_files_equal_frozen_AS=checked,
        nominal_gap_prefixes_rejected=13,nominal_gap_prefixes_tested=13,coverage_cell_mm=65536/5244,
        host=dict(log=str((base/'AY_verified/host.log').resolve()),failed=0,trajectory_crosschecks=90),
        rv=dict(image_bytes=51712,bss_bytes=69944,ram_bytes=131072,stack_reserved_bytes=6144,stack_bound_bytes=2736,stack_complete=True,spare_bytes=3272),
        rtl=dict(support_queries=251,support_points=14807,end_to_end_mask_reads=6144),
        board=dict(snapshot='AX_board; same coverage RTL, AX firmware image',synthesis_bsram=309,pnr_bsram=334,bsram_limit=340,
                   planning_gate=336,pnr_fmax_mhz=50.111,setup_violations=0,hold_violations=0,
                   includes_original4500_control=False,latest_AY_bitstream=False,whole_poll_deadline_verified=False),
        candidate_diagnosis={k:v for k,v in diag.items() if k not in ['instrumentation_equals_ordinary','examples','height_rejections']},
        noise_audit={k:v for k,v in noise.items() if k!='rows'},
        physical_keys=[r for r in rows if r['latest']],old_fast_keys=keys,old_fast_keys_complete=len(keys)==2,
        actual_zero_request_motion=execution,
        remaining=['terrain interval uncertainty / multi-surface evidence','path/candidate convergence','vertical anchor validity','measured execution/stopping envelope','whole-poll CPU deadline','integrated vehicle resources and calibration'])
    dump(base/'checkpoint.json',checkpoint);print(json.dumps(dict(records=len(rows),latest_videos=len(videos),old_fast_keys=keys,live_AS_files=len(checked),accepted=False),ensure_ascii=False))


if __name__=='__main__':main()
