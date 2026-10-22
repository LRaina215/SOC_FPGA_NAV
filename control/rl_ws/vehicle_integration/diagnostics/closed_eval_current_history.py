"""Independent MuJoCo evaluation with both rear legs and true loop constraints.
Read-only original model; actuator settings are changed only in memory.
"""
import sys
sys.path.insert(0,'/home/cyd/fpga/rl_ws2/_car_tools')
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
import torch
from torch import nn
from car_kinematics import FiveBar, SOURCE

p=argparse.ArgumentParser()
p.add_argument('checkpoint')
p.add_argument('--seconds',type=float,default=20)
p.add_argument('--case',default='0,0,0.1305')
p.add_argument('--profile',choices=['standing','full'],default='full')
p.add_argument('--output',default='closed_eval.json')
p.add_argument('--wheel-torque',type=float,default=.5)
p.add_argument('--leg-control',choices=['servo','mapped-torque'],default='servo')
p.add_argument('--min-height',type=float,default=.085)
a=p.parse_args()
torch.set_num_threads(2)
state=torch.load(a.checkpoint,map_location='cpu',weights_only=False)['model_state_dict']
def network(prefix):
    keys=sorted([k for k in state if k.startswith(prefix+'.') and k.endswith('.weight')],key=lambda k:int(k.split('.')[1]))
    modules=[]
    for i,k in enumerate(keys):
        layer=nn.Linear(state[k].shape[1],state[k].shape[0])
        layer.weight.data.copy_(state[k]); layer.bias.data.copy_(state[k.replace('weight','bias')])
        modules.append(layer)
        if i<len(keys)-1: modules.append(nn.ELU())
    return nn.Sequential(*modules).eval()
encoder,actor=network('encoder'),network('actor')
root=ET.parse(SOURCE).getroot()
meshdir=SOURCE.parent/root.find('compiler').get('meshdir')
for mesh in root.findall('asset/mesh'):
    mesh.set('file',str((meshdir/mesh.get('file')).resolve()))
root.find('compiler').set('meshdir','/')
root.find('.//body[@name="root"]').set('pos','0 0 0.1305')
for act in root.findall('actuator/position'):
    if a.leg_control=='mapped-torque':
        act.tag='motor';act.attrib.pop('kp',None);act.attrib.pop('kv',None)
        act.set('ctrlrange','-1.96 1.96')
    else:
        act.set('kp','60');act.set('kv','0.06')
    act.set('forcerange','-1.96 1.96')
for j in root.findall('.//joint'):
    if 'actuatorfrcrange' in j.attrib: j.set('actuatorfrcrange','-1.96 1.96')
for act in root.findall('actuator/motor'):
    if 'Wheel' in act.get('name'): act.set('ctrlrange','-0.5 0.5')
model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
data=mujoco.MjData(model)
geo=FiveBar()
order=['LeftHip_joint','LeftKnee_joint','LeftWheel_joint','RightHip_joint','RightKnee_joint','RightWheel_joint']
name_map={n:n.replace('Hip','Front').replace('Knee','FrontKnee') for n in order}
def jid(n): return mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,n)
def aid(n): return mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_ACTUATOR,n)
def setq(n,v): data.qpos[model.jnt_qposadr[jid(n)]]=v
nominal={'Right':(.290418,-.220039),'Left':(.290418,.220039)}
for side,(qh,qk) in nominal.items():
    y,z=geo.front_fk(side,qh,qk)
    q=geo.solve(side,y,z)
    for key,suffix in [('front','Front_joint'),('front_knee','FrontKnee_joint'),('back','Back_joint'),('back_knee','BackKnee_joint')]:
        setq(side+suffix,q[key])
    if a.leg_control=='servo':
        data.ctrl[aid(side+'Front_joint')]=q['front']
        data.ctrl[aid(side+'Back_joint')]=q['back']
mujoco.mj_forward(model,data)
body=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'root')
cmd=np.array([float(x) for x in a.case.split(',')])
last=np.zeros(6); hist=np.zeros((5,27)); refheight=.1305

def observe():
    v=np.zeros(6)
    mujoco.mj_objectVelocity(model,data,mujoco.mjtObj.mjOBJ_BODY,body,v,1)
    R=data.xmat[body].reshape(3,3)
    g=R.T@np.array([0.,0.,-1.])
    dq=np.zeros(6)
    for i in [2,5]: dq[i]=data.qvel[model.jnt_dofadr[jid(name_map[order[i]])]]
    pos=last*(0 if a.profile=='standing' else .03);pos[[2,5]]=0
    obs=np.concatenate([v[:3]*1.3,g*3,cmd*np.array([2.,1.3,5.]),pos,dq*.15,last])
    return obs,v,g

obs,_,_=observe();hist[:]=obs
metrics=[];fail_time=None;bad=0;closuremax=0.
def joint_state(name):
    joint=jid(name)
    return data.qpos[model.jnt_qposadr[joint]],data.qvel[model.jnt_dofadr[joint]]

def chain_jacobian(side,hip,knee,rear=False):
    s=geo.sides[side]
    sign=s['rear_sign'] if rear else s['front_sign']
    h=(s['rear_h0'] if rear else s['h0'])+sign*hip
    k=(s['rear_k0'] if rear else s['k0'])+knee
    return np.array([[sign*(-s['l1']*math.sin(h)-s['l2']*math.sin(h+k)), -s['l2']*math.sin(h+k)],
                     [sign*(s['l1']*math.cos(h)+s['l2']*math.cos(h+k)), s['l2']*math.cos(h+k)]])
with torch.inference_mode():
    for step in range(round(a.seconds/.008)):
        obs,vel,g=observe()
        hist=np.concatenate([hist[1:],obs[None]],axis=0)
        latent=encoder(torch.tensor(hist.reshape(1,-1),dtype=torch.float32))
        last=actor(torch.cat([torch.tensor(obs.reshape(1,-1),dtype=torch.float32),latent],dim=1))[0].numpy().clip(-1,1)
        for _ in range(8):
            refheight+=np.clip(cmd[2]-refheight,-.05*.001,.05*.001)
            for side,idx,wi in [('Right',[3,4],5),('Left',[0,1],2)]:
                if a.profile=='standing': qh,qk=nominal[side]
                else:
                    q=geo.solve(side,-.03,.0325-np.clip(refheight,a.min_height,.19))
                    qh=q['front']+last[idx[0]]*.03
                    qk=q['front_knee']+last[idx[1]]*.03
                y,z=geo.front_fk(side,qh,qk)
                q=geo.solve(side,y,z)
                if a.leg_control=='servo':
                    data.ctrl[aid(side+'Front_joint')]=q['front']
                    data.ctrl[aid(side+'Back_joint')]=q['back']
                else:
                    desired=np.array([qh,qk])
                    qf,vf=joint_state(side+'Front_joint')
                    measured_knee,vk=joint_state(side+'FrontKnee_joint')
                    qb,_=joint_state(side+'Back_joint')
                    qbk,_=joint_state(side+'BackKnee_joint')
                    jf=chain_jacobian(side,qf,measured_knee)
                    jb=chain_jacobian(side,qb,qbk,rear=True)
                    # Differentiate front FK == rear FK; use virtual work to
                    # map virtual hip/knee torques to the two physical hips.
                    derivative=np.linalg.solve(np.column_stack((jf[:,1],-jb[:,1])),
                                               np.column_stack((-jf[:,0],jb[:,0])))
                    virtual=np.clip(60*(desired-np.array([qf,measured_knee]))-.06*np.array([vf,vk]),-1.96,1.96)
                    physical=np.array([virtual[0]+derivative[0,0]*virtual[1],derivative[0,1]*virtual[1]])
                    data.ctrl[aid(side+'Front_joint')],data.ctrl[aid(side+'Back_joint')]=np.clip(physical,-1.96,1.96)
                data.ctrl[aid(side+'Wheel_joint')]=last[wi]*a.wheel_torque
            mujoco.mj_step(model,data)
        obs,vel,g=observe()
        tilt=math.degrees(math.acos(np.clip(-g[2],-1,1)))
        metrics.append([tilt,vel[4],vel[2],data.xpos[body,2]])
        bad=bad+1 if tilt>30 else 0
        for side in ['Right','Left']:
            s1=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,side+'Front_closure')
            s2=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,side+'Back_closure')
            closuremax=max(closuremax,float(np.linalg.norm(data.site_xpos[s1]-data.site_xpos[s2])))
        if bad*.008>.05:
            fail_time=float(data.time);break
mean=np.mean(metrics[len(metrics)//4:],axis=0)
report=dict(checkpoint=a.checkpoint,profile=a.profile,case=cmd.tolist(),seconds=a.seconds,
            leg_control=a.leg_control,velocity_origin='MuJoCo body velocity reference, not Isaac center metric',
            survived=fail_time is None,first_failure_s=fail_time,mean_tilt_deg=mean[0],
            mean_forward_mps=mean[1],mean_yaw_rps=mean[2],mean_height_m=mean[3],
            closure_error_m_max=closuremax,model='original closed geometry and constraints; in-memory '+a.leg_control+' actuation', history_includes_current=True, wheel_torque_nm=a.wheel_torque)
Path(a.output).write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
