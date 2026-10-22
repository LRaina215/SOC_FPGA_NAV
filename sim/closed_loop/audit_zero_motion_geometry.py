"""Disentangle virtual body reference, physical wheel midpoint and roll distance."""
import json
from pathlib import Path
import numpy as np
import mujoco as mj
from terrain_scene import S
from terrain_study import OUT,dump


def main():
    p=OUT/'n2av/navigation/AY_trench40d150';out=OUT/'n2av/n2az';out.mkdir(exist_ok=True)
    model=mj.MjModel.from_xml_path(str(p/'physics_backend/assets/robot.xml'));data=mj.MjData(model)
    with np.load(p/'physics_backend/full_state.npz') as z:state={k:z[k] for k in z.files}
    csv=np.genfromtxt(p/'physics_backend/physics.csv',delimiter=',',names=True)
    i=np.flatnonzero((csv['safe_vx']!=0)|(csv['safe_wz']!=0))[-1]+1;start=float(csv['t'][i]);rows=[]
    wheel=[model.body(n).id for n in ['LeftWheel_link','RightWheel_link']]
    for t in [start,21.,22.,25.,30.,35.,40.,45.]:
        k=int(np.searchsorted(state['t'],t));k=min(k,len(state['t'])-1)
        data.qpos[:]=state['qpos'][k];data.qvel[:]=state['qvel'][k];mj.mj_forward(model,data)
        axes=np.array([S@data.xpos[b] for b in wheel]);base=S@data.xpos[model.body('base').id]
        sample=csv[int(np.argmin(abs(csv['t']-t)))];center=axes.mean(axis=0)
        rows.append(dict(t=float(state['t'][k]),reference=[float(sample[n]) for n in ['x','y','z']],wheel_midpoint=center.tolist(),wheel_centres=axes.tolist(),base=base.tolist(),
            left_target=float(sample['left_target']),right_target=float(sample['right_target']),left_actual=float(sample['left_actual']),right_actual=float(sample['right_actual']),
            qvel=state['qvel'][k].tolist(),ctrl=state['ctrl'][k].tolist(),actuator_force=state['actuator_force'][k].tolist()))
    a,b=rows[0],rows[-1];delta=lambda key:((np.array(b[key])-np.array(a[key]))*1000).tolist()
    result=dict(start_s=start,rows=rows,delta_reference_mm=delta('reference'),delta_wheel_midpoint_mm=delta('wheel_midpoint'),delta_base_mm=delta('base'),
        mean_actual_wheel_speed_after_zero_mm_s=float(np.mean((csv['left_actual'][i:]+csv['right_actual'][i:])/2)*1000),
        reference='saved physics forward kinematics only; no policy integration or edits; native uncalibrated plant')
    dump(out/'zero_motion_geometry.json',result)
    print({k:v for k,v in result.items() if k not in ['rows','reference']})


if __name__=='__main__':main()
