"""Fixed original4500 on explicitly declared terrain/mechanism/execution models.

No policy/weight or control repository file is edited. Closed-chain IK mapping
and digital PWM-to-torque plant are declared comparison models, not verified
replicas of the deployed vehicle.
"""
import csv
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import numpy as np
from control4500 import load_runtime,DEFAULT_REPO
from safe_model import SafeModel
from deployed_execution import WheelPid,WheelEncoder
from mujoco_robot import MujocoRobot
from terrain_scene import S,add_terrain,PhysicsRayWorld


def add_closed_chain(xml,repo):
    geometry=json.loads((Path(repo)/'rl_ws/_car_tools/kinematics_report_extended.json').read_text())['geometry']
    root=xml.getroot();base=root.find('./worldbody/body[@name="base"]')
    eq=root.find('equality')
    if eq is None:eq=ET.SubElement(root,'equality')
    for side,g in geometry.items():
        front=base.find(f'.//body[@name="{side}Lower_link"]')
        wheel=front.find(f'body[@name="{side}Wheel_link"]');tip=np.fromstring(wheel.get('pos'),sep=' ')
        ET.SubElement(front,'site',name=side+'_front_tip',pos=' '.join(map(str,tip)),size='.002')
        back=ET.SubElement(base,'body',name=side+'BackUpper',pos=' '.join(map(str,g['back'])))
        ET.SubElement(back,'joint',name=side+'BackHip',axis=f"{g['rear_sign']} 0 0",range='-3.14 3.14',damping='.005')
        u=np.array([0.,g['l1']*math.cos(g['rear_h0']),g['l1']*math.sin(g['rear_h0'])])
        off=float(g['front'][0])+tip[0]-float(g['back'][0])
        v=np.array([off,g['l2']*math.cos(g['rear_h0']+g['rear_k0']),g['l2']*math.sin(g['rear_h0']+g['rear_k0'])])
        # Rear combined CAD mass0.041486kg split by link lengths; inertia is
        # uniform-capsule approximation, separately stress tested in this stage.
        ET.SubElement(back,'geom',name=side+'BackUpper_visual',type='capsule',fromto='0 0 0 '+' '.join(map(str,u)),size='.0045',
                      mass=str(.041486*g['l1']/(g['l1']+g['l2'])),contype='1',conaffinity='0',group='2',rgba='.8 .47 .2 1')
        lower=ET.SubElement(back,'body',name=side+'BackLower',pos=' '.join(map(str,u)))
        ET.SubElement(lower,'joint',name=side+'BackKnee',axis='1 0 0',range='-3.14 3.14',damping='.005')
        ET.SubElement(lower,'geom',name=side+'BackLower_visual',type='capsule',fromto='0 0 0 '+' '.join(map(str,v)),size='.0045',
                      mass=str(.041486*g['l2']/(g['l1']+g['l2'])),contype='1',conaffinity='0',group='2',rgba='.8 .47 .2 1')
        ET.SubElement(lower,'site',name=side+'_back_tip',pos=' '.join(map(str,v)),size='.002')
        ET.SubElement(eq,'connect',site1=side+'_front_tip',site2=side+'_back_tip',solref='.004 1',solimp='.95 .99 .0001')
        root.find(f'./actuator/general[@name="{side}Knee_joint_motor"]').set('joint',side+'BackHip')
    return geometry


def rear_solution(g,qh,qk):
    h=g['h0']+g['front_sign']*qh;k=g['k0']+qk
    y=g['front'][1]+g['l1']*math.cos(h)+g['l2']*math.cos(h+k)-g['back'][1]
    z=g['front'][2]+g['l1']*math.sin(h)+g['l2']*math.sin(h+k)-g['back'][2]
    c=(y*y+z*z-g['l1']**2-g['l2']**2)/(2*g['l1']*g['l2'])
    knee=math.acos(np.clip(c,-1,1));hip=math.atan2(z,y)-math.atan2(g['l2']*math.sin(knee),g['l1']+g['l2']*math.cos(knee))
    return (hip-g['rear_h0'])/g['rear_sign'],knee-g['rear_k0'],abs(c)>1+1e-8


class TerrainRobot(MujocoRobot):
    def __init__(self,scene,x=0.,y=0.,yaw=0.,out=None,seed=0,control_repo=None,
                 gyro_bias_dps=.3,gyro_noise_dps=.3,att_noise_deg=.3,scale_err=.01,**unused):
        self.s=scene;self.spec=scene.spec;self.out=Path(out);self.out.mkdir(parents=True,exist_ok=True)
        self.rt,self.policy=load_runtime(self.out/'assets',repo=control_repo or DEFAULT_REPO)
        xml=ET.parse(self.out/'assets/robot.xml');add_terrain(xml,scene)
        geometry=add_closed_chain(xml,control_repo or DEFAULT_REPO) if self.spec.mechanism=='closed' else None
        if geometry:
            xml.getroot().find('compiler').set('inertiafromgeom','auto')
        for g in xml.findall('.//geom'):
            if 'Wheel_link_collision' in g.get('name','') and self.spec.wheel_shape=='cad_cylinder':
                g.set('type','cylinder');g.set('size','.03365004135 .01329999976')
        inertial=xml.find('./worldbody/body[@name="base"]/inertial')
        mass=float(inertial.get('mass'));scale=(mass+self.spec.payload_kg)/mass
        inertial.set('mass',str(mass+self.spec.payload_kg));inertial.set('fullinertia',' '.join(str(float(z)*scale) for z in inertial.get('fullinertia').split()))
        for a in xml.findall('./actuator/general'):
            if 'Wheel' in a.get('name',''):
                limit=.5*self.spec.wheel_torque_scale;a.set('forcerange',f'{-limit} {limit}')
                if self.spec.profile=='digital':a.set('gainprm','1');a.set('biasprm','0 0 0')
        xml.write(self.out/'assets/robot.xml',encoding='unicode')
        rt=self.rt;owner=self
        class PhysicalRobot(rt.Robot):
            def reset(bot,command=(0.,0.,.1305)):
                rt.mujoco.mj_resetData(bot.model,bot.data);bot.command=np.array(command,dtype=float)
                bot.data.qpos[:3]=bot.cfg['init_state']['pos'];bot.data.qpos[2]+=scene.initial_height
                q0=bot.cfg['init_state']['rot'];q=np.array([q0[3],*q0[:3]])
                rt.mujoco.mju_mulQuat(bot.data.qpos[3:7],np.array([math.cos(yaw/2),0,0,math.sin(yaw/2)]),q)
                if scene.spec.align_start:
                    # Initial condition on a cross slope. Rotate/translate once
                    # before physics starts, never constrain the floating base later.
                    a=math.radians(scene.spec.value);cs,sn=math.cos(a),math.sin(a)
                    T=np.array([[cs,0,-sn],[0,1,0],[sn,0,cs]])
                    Y=np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1.]])
                    rt.mujoco.mju_mat2Quat(bot.data.qpos[3:7],(S.T@T@Y@S).reshape(-1))
                for j,n in enumerate(rt.ORDER):bot.data.qpos[bot.qindex[j]]=bot.cfg['init_state']['default_joint_angles'][n]
                if geometry:
                    for side,hip,knee in [('Left',0,1),('Right',3,4)]:
                        rh,rk,bad=rear_solution(geometry[side],bot.data.qpos[bot.qindex[hip]],bot.data.qpos[bot.qindex[knee]])
                        bot.data.qpos[bot.model.joint(side+'BackHip').qposadr[0]]=rh
                        bot.data.qpos[bot.model.joint(side+'BackKnee').qposadr[0]]=rk
                bot.reference_height=bot.cfg['init_state']['pos'][2];bot.actions=np.zeros(6);bot.torques=np.zeros(6)
                bot.failure_time=0.;bot.wheel_distance=0.;bot.done=False
                rt.mujoco.mj_forward(bot.model,bot.data)
                R=S@bot.data.xmat[bot.base].reshape(3,3)@S.T;c=S@bot.center()-R@np.array([0,0,.1305])
                if scene.spec.align_start:bot.data.qpos[:3]+=S.T@(np.array([x,y,scene.initial_height])-c)
                else:bot.data.qpos[:2]+=(S.T@np.array([x-c[0],y-c[1],0]))[:2]
                rt.mujoco.mj_forward(bot.model,bot.data)
                # Same single zero-action8ms startup as source runtime. No resets after this.
                bot.advance(np.zeros(6));bot.obs=bot.observation();bot.history=np.tile(bot.obs,(5,1));bot.origin=bot.center().copy()
            def targets(bot):
                p,v=super().targets()
                if geometry:
                    for side,hip,knee in [('Left',0,1),('Right',3,4)]:p[knee]=rear_solution(geometry[side],p[hip],p[knee])[0]
                return p,v
        self.robot=PhysicalRobot(forward_bias=0.);self.model,self.data=self.robot.model,self.robot.data
        self.s.w=PhysicsRayWorld(scene,self.model,self.data,self.rt.mujoco)
        self.safe=SafeModel();self.rng=np.random.default_rng(seed)
        self.gyro_bias=math.radians(gyro_bias_dps)*(1 if self.rng.random()>.5 else -1)
        self.gyro_noise=math.radians(gyro_noise_dps);self.att_noise=math.radians(att_noise_deg);self.scale=1+scale_err
        self.ms=0;self.failed=False;self.failure_reason=None;self.first_failure_s=None
        self.collisions=0;self.blocked=False;self.coll_log=[];self._contact_active=False;self._policy_due=True;self._cmd_seq=None
        self._start_wall=time.perf_counter();self._failure_dwell=0.;self._latched=None
        self.pid=[WheelPid(),WheelPid()];self.enc=[WheelEncoder(float(self.data.qpos[self.robot.qindex[i]])) for i in [2,5]]
        self._rows=[];self._states=[];self._contacts_all=[];self._closure=[];self._policy_rows=[]
        self._logfile=(self.out/'physics.csv').open('w',newline='');self._writer=csv.writer(self._logfile)
        self._writer.writerow(['t','request_vx','request_wz','safe_vx','safe_wz','x','y','z','yaw','pitch','roll','left_target','right_target','left_actual','right_actual','base_height','failed','contacts'])
        self._refresh();self._record()
        if self.spec.policy_noise:
            self.robot.obs=self._noisy_obs(self.robot.obs);self.robot.history=np.tile(self.robot.obs,(5,1))

    def _refresh(self):
        super()._refresh();self.physical_z=self.z;self.z-=self.s.initial_height

    def body_pose(self):return self.R,np.array([self.x,self.y,self.physical_z])

    def _contacts(self):
        bad=[];rows=[];wheel_contacts=0;base_contact=False
        for index,c in enumerate(self.data.contact):
            names=[self.model.geom(g).name or '' for g in [c.geom1,c.geom2]]
            if not any(n.startswith('terrain_') for n in names):continue
            robot=next((n for n in names if not n.startswith('terrain_')),'')
            legal='Wheel_link_collision' in robot
            wheel_contacts+=int(legal)
            if not legal:bad.append('/'.join(names));base_contact|=robot.startswith('base_collision')
            force=np.zeros(6);self.rt.mujoco.mj_contactForce(self.model,self.data,index,force)
            rows.append(dict(geoms=names,position=c.pos.tolist(),distance=float(c.dist),force=force.tolist(),friction=c.friction.tolist(),legal_wheel=legal))
        if bad and not self._contact_active:self.collisions+=1;self.coll_log.append((self.ms*.001,bad))
        self._contact_active=bool(bad)
        _,gravity=self.robot.state();tilt=gravity[2]>-math.cos(math.radians(self.robot.cfg['env']['tilt_fail_deg']))
        self._failure_dwell=self._failure_dwell+.002 if tilt or base_contact else 0.
        if self._failure_dwell>self.robot.cfg['env']['fail_to_terminal_time_s'] and not self.failed:
            self.failed=True;self.blocked=True;self.failure_reason='tilt' if tilt else 'base_contact';self.first_failure_s=(self.ms+1)*.001
        self._contacts_all.append(dict(t=(self.ms+1)*.001,contacts=rows))
        return wheel_contacts,len(bad)

    def _record(self,t=None):
        self._states.append((self.ms*.001 if t is None else t,self.data.qpos.copy(),self.data.qvel.copy(),self.data.qacc.copy(),self.data.ctrl.copy(),self.data.actuator_force.copy()))
        e=0.
        if self.spec.mechanism=='closed':
            for side in ['Left','Right']:
                a=self.model.site(side+'_front_tip').id;b=self.model.site(side+'_back_tip').id
                e=max(e,float(np.linalg.norm(self.data.site_xpos[a]-self.data.site_xpos[b])))
        self._closure.append(e)

    def _noisy_obs(self,obs):
        obs=obs.copy();obs[:3]+=self.rng.normal(0,math.radians(.3)*1.3,3)
        a,b=self.rng.normal(0,math.radians(.3),2);g=obs[3:6]/3
        g+=np.cross(np.array([a,b,0]),g);obs[3:6]=g/np.linalg.norm(g)*3
        return obs

    def step(self,cmd,dt):
        if abs(dt-.001)>1e-9:raise ValueError('1ms outer clock required')
        out=self.safe.tick(self.ms*1000);self.robot.command[:]=[out[0]/1000,out[2]/1000,.1305]
        if self.ms%2==1:
            if self._policy_due:
                self.robot.actions=self.policy.action(self.robot.obs,self.robot.history);self._policy_due=False
                self._policy_rows.append((self.ms*.001,self.robot.obs.copy(),self.robot.history.copy(),self.robot.actions.copy()))
            p,v=self.robot.targets();controls=p.copy();controls[self.rt.WHEELS]=v[self.rt.WHEELS]
            if self.spec.profile=='digital':
                if self._latched is None or self.ms%20==19:self._latched=p.copy()
                controls[self.rt.LEGS]=self._latched[self.rt.LEGS]
                if self.ms%10==9:
                    for j,i in enumerate([2,5]):
                        sign=-1 if i==2 else 1
                        feedback=sign*self.enc[j].sample(float(self.data.qpos[self.robot.qindex[i]]))
                        self.pid[j].sample(round(sign*v[i]*.0325*1000),feedback)
                for j,i in enumerate([2,5]):controls[i]=(-1 if i==2 else 1)*self.pid[j].duty/2400*.5*self.spec.wheel_torque_scale
            self.data.ctrl[self.robot.aindex]=controls
            self.robot.wheel_distance+=np.abs(self.data.qvel[self.robot.vindex[self.rt.WHEELS]]).mean()*.0325*.002
            self.rt.mujoco.mj_step(self.model,self.data);self.rt.mujoco.mj_forward(self.model,self.data)
            self._refresh();self._contacts();self._record((self.ms+1)*.001)
            if (self.ms+1)%8==0:
                self.robot.obs=self.robot.observation()
                if self.spec.policy_noise:self.robot.obs=self._noisy_obs(self.robot.obs)
                self.robot.history=np.concatenate((self.robot.history[1:],self.robot.obs[None]),axis=0);self._policy_due=True
        self.ms+=1
        if self.ms%10==0:
            m=self.robot.metrics();self._writer.writerow([self.ms*.001,cmd[0],cmd[2],out[0],out[2],self.x,self.y,self.z,self.yaw,self.pitch,self.roll,m['left_target_mps'],m['right_target_mps'],m['left_actual_mps'],m['right_actual_mps'],self.base_height,int(self.failed),self.collisions])

    def report(self):
        self._logfile.flush()
        return dict(backend='mujoco-original4500-terrain',mujoco=self.rt.mujoco.__version__,
                    policy='original model_4500 float32; no training or weight changes',physical_calibration=False,
                    physics_dt_s=.002,policy_dt_s=.008,safe_dt_s=.001,closed_chain=self.spec.mechanism=='closed',
                    mechanism=self.spec.mechanism,execution=self.spec.profile,wheel_shape=self.spec.wheel_shape,
                    closed_mapping='CAD front2R to rearhip IK; engineering adapter, not confirmed deployed mapping' if self.spec.mechanism=='closed' else None,
                    digital_plant='validated digital PID/encoder,20ms servo; uncalibrated duty/2400 to0.5Nm plant' if self.spec.profile=='digital' else 'native training-affine actuators',
                    closure_max_m=max(self._closure),mass_kg=float(self.model.body_mass.sum()),
                    failed=self.failed,failure_reason=self.failure_reason,first_failure_s=self.first_failure_s,
                    collisions=self.collisions,simulated_s=self.ms*.001,auto_reset=False,continues_after_failure=True,
                    wall_s=time.perf_counter()-self._start_wall)

    def close(self):
        self._logfile.close();s=self._states;p=self._policy_rows
        np.savez_compressed(self.out/'full_state.npz',t=np.array([r[0] for r in s]),qpos=np.array([r[1] for r in s]),qvel=np.array([r[2] for r in s]),qacc=np.array([r[3] for r in s]),ctrl=np.array([r[4] for r in s]),actuator_force=np.array([r[5] for r in s]),closure_error=np.array(self._closure),policy_t=np.array([r[0] for r in p]),obs=np.array([r[1] for r in p]),history=np.array([r[2] for r in p]),actions=np.array([r[3] for r in p]))
        with (self.out/'contacts.jsonl').open('w') as f:
            for r in self._contacts_all:f.write(json.dumps(r)+'\n')
