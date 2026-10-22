"""Re-score saved states only. No mj_step, policy inference or score rewriting."""
import argparse,json,math
from pathlib import Path
import mujoco as mj
import numpy as np

S=np.array([[0.,1.,0.],[-1.,0.,0.],[0.,0.,1.]])


def dwell_summary(t,speed,yaw_rate,limit_v,limit_w):
    good=(speed<=limit_v+1e-12)&(abs(yaw_rate)<=limit_w+1e-12)
    duration=0.;longest=0.
    for dt,ok in zip(np.diff(t),good):
        duration=duration+float(dt) if ok else 0.
        longest=max(longest,duration)
    return dict(speed_limit_mmps=limit_v*1000,yaw_limit_mradps=limit_w*1000,
        final_dwell_s=duration,longest_dwell_s=longest,final_dwell_500ms=duration>=.5-1e-9,
        maximum_speed_mmps=float(np.max(speed)*1000),mean_speed_mmps=float(np.average(speed,weights=np.diff(t))*1000),
        maximum_abs_yaw_mradps=float(np.max(abs(yaw_rate))*1000))


def analyze(run):
    metrics=json.loads((run/'metrics.json').read_text(encoding='utf-8'));terminal=metrics['terminal']
    declared=terminal['declared_s'];assert declared is not None
    end=declared+2.;root=run/'physics_backend'
    with np.load(root/'full_state.npz',allow_pickle=False) as state:
        t=state['t'];keep=(t>=declared-1e-8)&(t<=end+1e-8);t=t[keep];qpos=state['qpos'][keep];qvel=state['qvel'][keep]
    assert len(t)>900 and np.max(abs(np.diff(t)-.002))<1e-8
    model=mj.MjModel.from_xml_path(str(root/'assets/robot.xml'));data=mj.MjData(model)
    base=model.body('base').id;wheel=[model.body(n).id for n in ('LeftWheel_link','RightWheel_link')]
    reference=[];center=[];yaw=[]
    for q,v in zip(qpos,qvel):
        data.qpos[:]=q;data.qvel[:]=v;mj.mj_forward(model,data)
        r=data.xmat[base].reshape(3,3);rotation=S@r@S.T
        # Same reference definitions as the audited original runtime.center
        # and mujoco_robot._refresh; compare them to the saved 10ms CSV below.
        c=S@(data.xpos[base]+r@np.array([-.08,-.006656542,0.]))
        reference.append((c-rotation@np.array([0.,0.,.1305]))[:2])
        center.append((S@np.mean(data.xpos[wheel],axis=0))[:2])
        yaw.append(math.atan2(rotation[1,0],rotation[0,0]))
    reference=np.asarray(reference);center=np.asarray(center);yaw=np.unwrap(yaw);dt=np.diff(t)
    ref_speed=np.linalg.norm(np.diff(reference,axis=0),axis=1)/dt
    wheel_speed=np.linalg.norm(np.diff(center,axis=0),axis=1)/dt
    wz=np.diff(yaw)/dt
    csv=np.genfromtxt(root/'physics.csv',names=True,delimiter=',')
    csv=csv[(csv['t']>=declared-1e-8)&(csv['t']<=end+1e-8)]
    indices=np.searchsorted(t,csv['t']-1e-10);indices=np.minimum(indices,len(t)-1)
    assert np.max(abs(t[indices]-csv['t']))<1e-8
    reference_error=float(np.max(np.linalg.norm(reference[indices]-np.c_[csv['x'],csv['y']],axis=1)))
    assert reference_error<1e-8,reference_error
    historic=np.genfromtxt(run/'timeseries.csv',names=True,delimiter=',')
    historic=historic[(historic['t']>=declared-1e-8)&(historic['t']<=end+1e-8)]
    return dict(run=str(run.resolve()),historical_outcome=metrics['outcome'],historical_terminal=terminal,
        declaration_s=declared,observation_end_s=end,states_2ms=len(t),reference_csv_match_max_error_m=reference_error,
        reference_2ms_5_20=dwell_summary(t,ref_speed,wz,.005,.020),
        wheel_midpoint_2ms_5_20=dwell_summary(t,wheel_speed,wz,.005,.020),
        reference_2ms_20_50=dwell_summary(t,ref_speed,wz,.020,.050),
        reference_100ms_5_20=dwell_summary(historic['t'],historic['true_speed_mps'][1:],historic['true_yaw_rate_rps'][1:],.005,.020),
        reference_drift_mm=float(np.max(np.linalg.norm(reference-reference[0],axis=1))*1000),
        wheel_midpoint_drift_mm=float(np.max(np.linalg.norm(center-center[0],axis=1))*1000))


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--run',type=Path,action='append',required=True)
    ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();rows=[analyze(run) for run in a.run]
    r=dict(runs=rows,new_simulation_runs=0,source='saved qpos/qvel and forward kinematics, no integration/inference',
        scope='additional sampled stop diagnostics; historical 150mm/20mmps/50mradps scores unchanged; 2ms differences do not prove continuous-time physical stop or terrain safety')
    a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(r,indent=2),encoding='utf-8')
    print(json.dumps(r))


if __name__=='__main__':main()
