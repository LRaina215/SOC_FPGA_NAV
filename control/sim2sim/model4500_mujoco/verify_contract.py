"""Check MuJoCo glue against original executable observation and IK methods."""
import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS

import mujoco
import numpy as np
import torch

from runtime import ROOT, PROJECT, Robot, Policy, ORDER, LEGS, WHEELS


def original_method(name):
    path = PROJECT / 'model/wheel_velocity_original/vendor/wheel_legged_gym/envs/car_robot/car_robot.py'
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'CarRobot')
    node = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
    namespace = {'torch': torch}
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace[name]


def main():
    torch.set_num_threads(1)
    robot, policy = Robot(), Policy()
    rng = np.random.default_rng(42)
    reference = torch.jit.load(str(PROJECT / 'pt_to_fpga/parking_speed_4500/assets/policy_reference.pt'))
    torch.manual_seed(42)
    obs, hist = torch.randn(256, 28), torch.randn(256, 140)
    with torch.inference_mode():
        action, latent = policy(obs, hist)
        expected = reference(obs, hist)
    network_error = float((action-expected[0]).abs().max())
    latent_error = float((latent-expected[2]).abs().max())
    assert network_error == latent_error == 0.

    # Verify native affine actuators implement the original saturated PD force.
    force_error = 0.
    for _ in range(32):
        robot.actions = rng.uniform(-1, 1, 6)
        robot.data.qvel[robot.vindex] = rng.uniform(-20, 20, 6)
        pos, vel = robot.targets()
        controls = pos.copy()
        controls[WHEELS] = vel[WHEELS]
        robot.data.ctrl[robot.aindex] = controls
        mujoco.mj_forward(robot.model, robot.data)
        expected = np.clip(robot.kp*(pos-robot.data.qpos[robot.qindex]) +
                           robot.kd*(vel-robot.data.qvel[robot.vindex]), -robot.limits, robot.limits)
        force_error = max(force_error, float(np.max(np.abs(expected-robot.data.actuator_force[robot.aindex]))))
    assert force_error < 1e-12

    # Principal-inertia axes differ from CAD axes; isolate this regression.
    robot.reset()
    robot.data.qpos[3:7] = [1, 0, 0, 0]
    robot.data.qvel[3:6] = [.4, -.2, .3]
    mujoco.mj_forward(robot.model, robot.data)
    np.testing.assert_allclose(robot.state()[0], [.4, -.2, .3], atol=1e-12)
    robot.data.qpos[3:7] = [math.cos(math.pi/12), math.sin(math.pi/12), 0, 0]
    mujoco.mj_forward(robot.model, robot.data)
    np.testing.assert_allclose(robot.state()[1], [0, -.5, -math.sqrt(3)/2], atol=1e-12)

    observation_reference = original_method('compute_proprioception_observations')
    observation_error = 0.
    for _ in range(32):
        robot.actions = rng.uniform(-1, 1, 6)
        robot.command = np.array([rng.uniform(-.6, .6), rng.uniform(-2.9, 2.9), rng.uniform(.085, .19)])
        gyro, gravity = robot.state()
        fake = NS(actions=torch.tensor(robot.actions[None]),
                  dof_vel=torch.tensor(robot.data.qvel[robot.vindex][None]),
                  base_ang_vel=torch.tensor(gyro[None]), projected_gravity=torch.tensor(gravity[None]),
                  commands=torch.tensor([[*robot.command, 0.]]),
                  commands_scale=torch.tensor([2., 1.3, 5.]),
                  wheel_joint_indices=WHEELS.tolist(), leg_joint_indices=LEGS.tolist(),
                  cfg=NS(control=NS(pos_action_scale=.03)),
                  obs_scales=NS(dof_vel=.15, dof_pos=1., ang_vel=1.3, gravity=3.))
        expected = observation_reference(fake).numpy()[0].astype(np.float32)
        observation_error = max(observation_error, float(np.max(np.abs(expected-robot.observation()))))
    assert observation_error < 1e-6

    ik_reference = original_method('_leg_reference')
    robot.reset()
    fake = NS(cfg=NS(control=NS(height_reference=True, min_reference_height=.085,
                               height_reference_rate=.05, jump_enabled=False)),
              sim_params=NS(dt=.002), default_dof_pos=torch.zeros(1, 6, dtype=torch.float64),
              reference_height=torch.tensor([.1305], dtype=torch.float64),
              hip_y_offset=.022468, car_l1=.075, car_l2=.125,
              cad_hip=[-1.107851943952, -.527016956249],
              cad_knee=[-1.447594775649, -1.887671992209],
              right_indices=[3, 4, 5], left_indices=[0, 1, 2])
    ik_error = 0.
    for height in [.085, .10, .1305, .18, .19]:
        robot.command[2] = height
        fake.commands = torch.tensor([[0., 0., height, 0.]], dtype=torch.float64)
        for _ in range(40):
            ik_error = max(ik_error, float(np.max(np.abs(ik_reference(fake).numpy()[0]-robot.leg_reference()))))
    assert ik_error < 1e-10
    robot.reset()
    robot.step(policy)
    np.testing.assert_array_equal(robot.history[-1], robot.obs)
    report = dict(network_samples=256, network_max_abs_error=network_error,
                  latent_max_abs_error=latent_error, pd_force_samples=32,
                  pd_max_abs_error=force_error, observation_samples=32,
                  observation_max_abs_error=observation_error, ik_samples=200,
                  ik_max_abs_error=ik_error, gyro_frame='CAD body axes verified',
                  history='5 frames oldest to newest, includes current', action_order=ORDER)
    (ROOT / 'contract_verification.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
