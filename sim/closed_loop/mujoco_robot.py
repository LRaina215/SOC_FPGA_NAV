"""J2 fixed original4500 physics adapter; first version supports flat/box scenes.

Runtime/policy are reused read-only. Scheduling is simulation time: 2 ms physics,
8 ms policy, 1 ms SAFE. Native training-equivalent actuators are retained (J2),
so this is not yet the deployed digital wheel PID/servo execution model (J3).
"""
import csv
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import numpy as np

from control4500 import load_runtime, DEFAULT_REPO
from safe_model import SafeModel

S = np.array([[0.,1.,0.],[-1.,0.,0.],[0.,0.,1.]])  # CAD -> navigation axes


def scene_xml(path, scene):
    xml = ET.parse(path)
    world = xml.getroot().find('worldbody')
    for i, (kind, bounds, height, name) in enumerate(scene.prims):
        if kind != 'box':
            raise ValueError('J2 physics does not yet support ramp scenes')
        x0,y0,x1,y1 = bounds
        ET.SubElement(world, 'geom', name='scene_%d_%s' % (i,name), type='box',
                      pos='%g %g %g' % (-(y0+y1)/2,(x0+x1)/2,height/2),
                      size='%g %g %g' % ((y1-y0)/2,(x1-x0)/2,height/2),
                      contype='0', conaffinity='1', rgba='.6 .4 .2 1')
    xml.write(path, encoding='unicode')


class MujocoRobot:
    def __init__(self, scene, x, y, yaw, out, seed=0, gyro_bias_dps=.3,
                 gyro_noise_dps=.3, att_noise_deg=.3, scale_err=.01, control_repo=None, **unused):
        self.s = scene
        self.out = Path(out); self.out.mkdir(parents=True, exist_ok=True)
        self.rt, self.policy = load_runtime(self.out / 'assets', repo=control_repo or DEFAULT_REPO)
        scene_xml(self.out / 'assets/robot.xml', scene)
        self.robot = self.rt.Robot(forward_bias=0.)
        self.model,self.data = self.robot.model,self.robot.data
        # Rotate both initial base orientation and translation around wheel reference.
        center = self.robot.center().copy()
        q = self.data.qpos[3:7].copy(); qyaw = np.array([math.cos(yaw/2),0.,0.,math.sin(yaw/2)])
        self.rt.mujoco.mju_mulQuat(self.data.qpos[3:7], qyaw, q)
        self.rt.mujoco.mj_forward(self.model,self.data)
        self.data.qpos[:2] += (S.T @ np.array([x,y,center[2]]))[:2] - self.robot.center()[:2]
        self.rt.mujoco.mj_forward(self.model,self.data)
        self.robot.obs = self.robot.observation()
        self.robot.history = np.tile(self.robot.obs,(5,1))
        self.safe = SafeModel()
        self.rng = np.random.default_rng(seed)
        self.gyro_bias = math.radians(gyro_bias_dps) * (1 if self.rng.random()>.5 else -1)
        self.gyro_noise = math.radians(gyro_noise_dps); self.att_noise=math.radians(att_noise_deg)
        self.scale=1.+scale_err
        self.ms=0; self.failed=False; self.failure_reason=None
        self.collisions=0; self.blocked=False; self.coll_log=[]
        self._contact_active=False; self._policy_due=True
        self._cmd_seq=None; self._submitted_us=None
        self._start_wall=time.perf_counter()
        self._logfile=(self.out/'physics.csv').open('w',newline='')
        self._writer=csv.writer(self._logfile)
        self._writer.writerow(['t','request_vx','request_wz','safe_vx','safe_wz','x','y','z',
                               'yaw','pitch','roll','left_target','right_target','left_actual',
                               'right_actual','base_height','failed','contacts'])
        self._refresh()

    def submit(self, cmd, seq):
        if seq != self._cmd_seq:
            self._cmd_seq=seq; self.safe.submit(cmd,self.ms*1000)

    def _refresh(self):
        r=self.data.xmat[self.robot.base].reshape(3,3)
        self.R = S @ r @ S.T
        c=S @ self.robot.center()
        ground_ref=c-self.R@np.array([0.,0.,.1305])
        self.x,self.y=float(ground_ref[0]),float(ground_ref[1])
        self.yaw=math.atan2(self.R[1,0],self.R[0,0])
        self.pitch=math.atan2(-self.R[2,0], math.hypot(self.R[0,0],self.R[1,0]))
        self.roll=math.atan2(self.R[2,1],self.R[2,2])
        self.base_height=float(c[2])
        # Navigation origin represents nominal ground under the wheel axle.
        # Ray origin follows real base heave, not this estimated ground reference.
        self.z=float(ground_ref[2])

    def _contacts(self):
        bad=[]
        for c in self.data.contact:
            names=[self.model.geom(g).name or '' for g in (c.geom1,c.geom2)]
            env=next((n for n in names if n=='floor' or n.startswith('scene_')),None)
            if env is None: continue
            robot=next((n for n in names if n!=env),'')
            # Legal wheel-floor contact is not a collision. Any contact with a
            # raised scene box is reported, including wheels (obstacle encounter).
            if env!='floor' or 'Wheel' not in robot:
                bad.append('/'.join(names))
        if bad and not self._contact_active:
            self.collisions+=1; self.coll_log.append((round(self.ms*.001,3),bad))
        self._contact_active=bool(bad)
        if bad:
            self.failed=True; self.failure_reason='physical_contact'; self.blocked=True
        _,gravity=self.robot.state()
        if gravity[2]>-math.cos(math.radians(self.robot.cfg['env']['tilt_fail_deg'])):
            self.robot.failure_time+=self.robot.dt
        else: self.robot.failure_time=0.
        if self.robot.failure_time>self.robot.cfg['env']['fail_to_terminal_time_s']:
            self.failed=True; self.failure_reason='tilt'; self.blocked=True
        return len(bad)

    def step(self, cmd, dt):
        if abs(dt-.001)>1e-9: raise ValueError('J2 requires a 1ms outer tick')
        out=self.safe.tick(self.ms*1000)
        self.robot.command[:]=[out[0]/1000.,out[2]/1000.,.1305]
        # One physical step per two outer ticks; infer exactly every four steps.
        if self.ms % 2 == 1 and not self.failed:
            if self._policy_due:
                self.robot.actions=self.policy.action(self.robot.obs,self.robot.history)
                self._policy_due=False
            p,v=self.robot.targets(); controls=p.copy(); controls[self.rt.WHEELS]=v[self.rt.WHEELS]
            self.data.ctrl[self.robot.aindex]=controls
            self.rt.mujoco.mj_step(self.model,self.data)
            self.rt.mujoco.mj_forward(self.model,self.data)
            self._refresh(); self._contacts()
            if (self.ms+1)%8==0:
                self.robot.obs=self.robot.observation()
                self.robot.history=np.concatenate((self.robot.history[1:],self.robot.obs[None]),axis=0)
                self._policy_due=True
        self.ms+=1
        if self.ms%10==0:
            m=self.robot.metrics()
            self._writer.writerow([self.ms*.001,cmd[0],cmd[2],out[0],out[2],self.x,self.y,self.z,
                                   self.yaw,self.pitch,self.roll,m['left_target_mps'],m['right_target_mps'],
                                   m['left_actual_mps'],m['right_actual_mps'],self.base_height,int(self.failed),self.collisions])

    def odom(self):
        m=self.robot.metrics()
        vx=(m['left_actual_mps']+m['right_actual_mps'])*.5*self.scale
        gyro=S@self.robot.state()[0]
        wz=gyro[2]+self.gyro_bias+self.rng.normal(0,self.gyro_noise)
        return round(vx*1000),0,round(wz*1000)

    @staticmethod
    def _wit(kind, values, scale):
        b=[0x55,kind]
        for value in values:
            v=max(-32768,min(32767,round(value*32768/scale))) & 0xffff
            b += [v&255,v>>8]
        b += [0,0]; b.append(sum(b)&255)
        return bytes(b)

    def imu_wit_frame(self):
        # WIT is a transport adapter to the current navigation parser, not a
        # claim that the deployed BMI088 itself emits WIT frames.
        noise=self.rng.normal(0,self.att_noise,2)
        return self._wit(0x53,[self.roll+noise[0],self.pitch+noise[1],self.yaw],math.pi)

    def imu_wit_gyro_frame(self, dt):
        gyro=S@self.robot.state()[0]
        gyro += self.rng.normal(0,self.gyro_noise,3); gyro[2]+=self.gyro_bias
        return self._wit(0x52,gyro,math.radians(2000))

    def body_pose(self):
        return self.R, np.array([self.x,self.y,self.z])

    def report(self):
        self._logfile.flush()
        wall=time.perf_counter()-self._start_wall
        return dict(backend='mujoco-original4500',mujoco=self.rt.mujoco.__version__,
                    policy_inference='Torch float32 original weights; not FPGA F12 inference',
                    physics_dt_s=.002,policy_dt_s=.008,safe_dt_s=.001,
                    wheel_feedback_hz=100,actuators='training-equivalent-native-affine',
                    command_bias_mps=0.,physical_calibration=False,auto_reset=False,
                    initial_runtime_reset_advance_s=.008,
                    closed_chain=False,failed=self.failed,failure_reason=self.failure_reason,
                    simulated_s=self.ms*.001,wall_s=wall,real_time_factor=self.ms*.001/wall,
                    safe_startup=dict(enabled=True,vmax=500,wmax=1200,dv=2,dw=3,watchdog_us=200000),
                    sensor_transport='WIT adapter for BMI088-equivalent attitude/gyro samples',
                    lidar_height='physical base heave + nominal mount offset',
                    unsupported=['ramps','negative obstacles','deployed digital execution','FPGA fixed-point inference','physical calibration'])

    def close(self):
        self._logfile.close()
