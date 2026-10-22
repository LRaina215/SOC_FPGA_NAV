"""model_4500 observation/action contract and original low-level controller."""
import json
import math
from pathlib import Path

import mujoco
import numpy as np
import torch
from torch import nn

from build_model import ROOT, PROJECT, CONFIG, build

ORDER = ['LeftHip_joint', 'LeftKnee_joint', 'LeftWheel_joint',
         'RightHip_joint', 'RightKnee_joint', 'RightWheel_joint']
WHEELS = np.array([2, 5])
LEGS = np.array([0, 1, 3, 4])
CHECKPOINT = PROJECT / 'model/wheel_velocity_original/model_4500.pt'


class Policy(nn.Module):
    def __init__(self, checkpoint=CHECKPOINT):
        super().__init__()
        self.encoder = nn.Sequential(nn.Linear(140, 128), nn.ELU(),
                                     nn.Linear(128, 64), nn.ELU(), nn.Linear(64, 3))
        self.actor = nn.Sequential(nn.Linear(31, 128), nn.ELU(), nn.Linear(128, 64),
                                   nn.ELU(), nn.Linear(64, 32), nn.ELU(), nn.Linear(32, 6))
        state = torch.load(checkpoint, map_location='cpu', weights_only=False)['model_state_dict']
        self.load_state_dict({k: v for k, v in state.items()
                              if k.startswith(('encoder.', 'actor.'))}, strict=True)
        self.eval()

    def forward(self, obs, history):
        latent = self.encoder(history)
        return self.actor(torch.cat((obs, latent), dim=-1)), latent

    def action(self, obs, history):
        with torch.inference_mode():
            raw, _ = self(torch.from_numpy(obs[None].astype(np.float32)),
                          torch.from_numpy(history.reshape(1, 140).astype(np.float32)))
        return raw[0].numpy().clip(-1, 1)


class Robot:
    def __init__(self, forward_bias=0.):
        self.forward_bias = float(forward_bias)
        self.cfg = json.loads(CONFIG.read_text())['env']
        if not (ROOT / 'robot.xml').exists():
            build()
        self.model = mujoco.MjModel.from_xml_path(str(ROOT / 'robot.xml'))
        self.data = mujoco.MjData(self.model)
        self.base = self.model.body('base').id
        self.qindex = np.array([self.model.joint(n).qposadr[0] for n in ORDER])
        self.vindex = np.array([self.model.joint(n).dofadr[0] for n in ORDER])
        self.aindex = np.array([self.model.actuator(n+'_motor').id for n in ORDER])
        self.dt = self.cfg['sim']['dt']
        self.decimation = self.cfg['control']['decimation']
        self.control_dt = self.dt*self.decimation
        self.kp = np.array([60., 60., 0., 60., 60., 0.])
        self.kd = np.array([.06, .06, .025, .06, .06, .025])
        self.limits = np.array([1.96, 1.96, .5, 1.96, 1.96, .5])
        self.reset()

    def reset(self, command=(0., 0., .1305)):
        mujoco.mj_resetData(self.model, self.data)
        self.command = np.asarray(command, dtype=np.float64).copy()
        self.data.qpos[:3] = self.cfg['init_state']['pos']
        xyzw = self.cfg['init_state']['rot']
        self.data.qpos[3:7] = [xyzw[3], *xyzw[:3]]
        for j, name in enumerate(ORDER):
            self.data.qpos[self.qindex[j]] = self.cfg['init_state']['default_joint_angles'][name]
        self.reference_height = self.cfg['init_state']['pos'][2]
        self.actions = np.zeros(6)
        self.torques = np.zeros(6)
        self.failure_time = 0.
        self.wheel_distance = 0.
        self.done = False
        mujoco.mj_forward(self.model, self.data)
        # Gym.reset advances one zero-action control interval before inference.
        self.advance(np.zeros(6))
        self.obs = self.observation()
        self.history = np.tile(self.obs, (5, 1))
        self.origin = self.center().copy()

    def state(self):
        rotation = self.data.xmat[self.base].reshape(3, 3)
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                                self.base, velocity, 0)
        gravity = rotation.T @ np.array([0., 0., -1.])
        # mjOBJ_BODY's local frame is the inertial principal-axis frame, not
        # the CAD/IMU body frame. Transform WORLD angular velocity explicitly.
        return rotation.T @ velocity[:3], gravity

    def effective_command(self):
        command = self.command.copy()
        command[0] = np.clip(command[0]+self.forward_bias, -.6, .6)
        return command

    def observation(self):
        gyro, gravity = self.state()
        pos_obs = self.actions*.03
        pos_obs[WHEELS] = 0.
        vel_obs = self.data.qvel[self.vindex].copy()*.15
        vel_obs[LEGS] = 0.
        obs = np.concatenate((gyro*1.3, gravity*3., self.effective_command()*[2., 1.3, 5.],
                              [0.], pos_obs, vel_obs, self.actions))
        assert obs.shape == (28,)
        return obs.astype(np.float32).clip(-100, 100)

    def leg_reference(self):
        cfg = self.cfg['control']
        goal = np.clip(self.command[2], cfg['min_reference_height'], .19)
        self.reference_height += np.clip(goal-self.reference_height,
                                         -cfg['height_reference_rate']*self.dt,
                                         cfg['height_reference_rate']*self.dt)
        y = -.03-.022468
        z = .0325-self.reference_height-.033
        c = (y*y+z*z-.075**2-.125**2)/(2*.075*.125)
        knee = -math.acos(np.clip(c, -1., 1.))
        hip = math.atan2(z, y)-math.atan2(.125*math.sin(knee), .075+.125*math.cos(knee))
        # Original CAD offsets and hip axis signs, actor order left then right.
        return np.array([-(hip+.527016956249), knee+1.887671992209, 0.,
                         hip+1.107851943952, knee+1.447594775649, 0.])

    def targets(self):
        target_pos = self.actions*.03
        target_pos[WHEELS] = 0.
        target_pos += self.leg_reference()
        target_vel = self.actions*self.cfg['control']['vel_action_scale']
        target_vel[LEGS] = 0.
        return target_pos, target_vel

    def advance(self, action):
        self.actions = np.asarray(action, dtype=np.float64).clip(-1, 1)
        for _ in range(self.decimation):
            target_pos, target_vel = self.targets()
            controls = target_pos.copy()
            controls[WHEELS] = target_vel[WHEELS]
            self.data.ctrl[self.aindex] = controls
            self.wheel_distance += np.abs(self.data.qvel[self.vindex[WHEELS]]).mean()*.0325*self.dt
            mujoco.mj_step(self.model, self.data)
            self.torques = self.data.actuator_force[self.aindex].copy()
        mujoco.mj_forward(self.model, self.data)
        _, gravity = self.state()
        tilt_fail = gravity[2] > -math.cos(math.radians(self.cfg['env']['tilt_fail_deg']))
        base_contact = False
        for contact in self.data.contact:
            names = [self.model.geom(g).name for g in [contact.geom1, contact.geom2]]
            if 'floor' in names and any(n and n.startswith('base_collision') for n in names):
                base_contact = True
        self.failure_time = self.failure_time+self.control_dt if tilt_fail or base_contact else 0.
        self.done = self.failure_time > self.cfg['env']['fail_to_terminal_time_s']

    def step(self, policy):
        action = policy.action(self.obs, self.history)
        self.advance(action)
        self.obs = self.observation()
        self.history = np.concatenate((self.history[1:], self.obs[None]), axis=0)
        return self.done

    def center(self):
        return self.data.xpos[self.base]+self.data.xmat[self.base].reshape(3, 3) @ np.array([-.08, -.006656542, 0.])

    def metrics(self):
        gyro, gravity = self.state()
        rotation = self.data.xmat[self.base].reshape(3, 3)
        # Free-joint translation is the CAD body origin, not its offset COM.
        body_v = rotation.T @ self.data.qvel[:3]
        forward = body_v[1]+gyro[2]*self.cfg['asset']['forward_reference_x']
        return dict(time_s=float(self.data.time), forward_mps=float(forward), yaw_radps=float(gyro[2]),
                    height_m=float(self.data.xpos[self.base, 2]),
                    tilt_deg=math.degrees(math.acos(np.clip(-gravity[2], -1., 1.))),
                    wheel_travel_m=float(self.wheel_distance),
                    displacement_m=float(np.linalg.norm((self.center()-self.origin)[:2])),
                    left_target_mps=float(-self.actions[2]*.6), right_target_mps=float(self.actions[5]*.6),
                    left_actual_mps=float(-self.data.qvel[self.vindex[2]]*.0325),
                    right_actual_mps=float(self.data.qvel[self.vindex[5]]*.0325))
