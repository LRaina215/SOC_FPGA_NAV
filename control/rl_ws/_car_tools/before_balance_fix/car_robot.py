# 轮腿车（car）强化学习环境
#
# 由 sf_robot.py 改写，适配 car_open.urdf。
#
# 关节顺序（URDF 定义顺序）：
#   0 RightHip_joint    1 RightKnee_joint    2 RightWheel_joint
#   3 LeftHip_joint     4 LeftKnee_joint     5 LeftWheel_joint
#
# 控制方式沿用实车硬件约定：
#   腿部 4 个舵机 -> 位置控制（PD + 积分）
#   轮部 2 个电机 -> 速度控制（通过阻尼项实现）

import math

import torch
from wheel_legged_gym.envs.base.legged_robot import LeggedRobot
from .car_robot_config import CarRobotCfg


class CarRobot(LeggedRobot):
    def __init__(self, cfg: CarRobotCfg, sim_params, physics_engine, sim_device, headless):
        super().__init__(cfg, sim_params, physics_engine, sim_device, headless)

        # 关节索引：腿部（舵机）与轮部（电机）
        self.leg_joint_indices = [0, 1, 3, 4]
        self.wheel_joint_indices = [2, 5]

        self.last_torques = torch.zeros_like(self.torques)
        self.last_dof_vel = torch.zeros_like(self.dof_vel)

        # 舵机积分项
        self.leg_integral_error = torch.zeros(
            self.num_envs, len(self.leg_joint_indices),
            dtype=torch.float, device=self.device,
        )

        # 五连杆等效 2R 臂的几何（用于 leg_together 末端位置计算）
        self.car_l1 = self.cfg.asset.l1   # 大腿 0.075
        self.car_l2 = self.cfg.asset.l2   # 小腿 0.125
        # 标称姿态下，大腿/小腿相对机身的绝对角度（rad）
        self.nominal_hip = -0.817435    # -46.8355 deg（左右共用，对称目标）
        self.nominal_knee = -1.667633   # -95.5484 deg
        # 髋关节在 y 方向的安装偏置（左右相同）
        self.hip_y_offset = 0.022468
        # 关节轴符号：RightHip=+X, LeftHip=-X, Knee 均为 +X
        self.hip_sign = {"right": +1.0, "left": -1.0}

        # 姿态失败阈值（见 check_termination）
        # projected_gravity[2] = -cos(tilt)，所以阈值就是 -cos(tilt_fail)
        tilt_fail_rad = math.radians(float(self.cfg.env.tilt_fail_deg))
        self.tilt_fail_cos = -math.cos(tilt_fail_rad)

    # ---------- 终止条件（覆写基类）----------
    def check_termination(self):
        """判定回合是否结束。

        基类用的是 projected_gravity[2] > -0.1，等价于倾角 > 84°。
        但本车机身扁平，侧躺 55~60° 就很"舒服"，策略会学会躺平而不是站立
        （实测：训练后 64 个环境全部侧躺，倾角分布 30~60° 有 61 个）。

        因此这里把失败阈值收紧到 cfg.env.tilt_fail_deg（默认 30°）。
        另外基类的 fail_buf 是累加器，需要连续 fail_to_terminal_time_s
        都满足才终止；这里保持同样的机制，避免单帧抖动误判。
        """
        fail_buf = torch.any(
            torch.norm(
                self.contact_forces[:, self.termination_contact_indices, :], dim=-1
            )
            > 10.0,
            dim=1,
        )
        # 倾角超过阈值即失败
        fail_buf |= self.projected_gravity[:, 2] > self.tilt_fail_cos

        self.fail_buf *= fail_buf
        self.fail_buf += fail_buf

        self.time_out_buf = self.episode_length_buf > self.max_episode_length
        if self.cfg.terrain.mesh_type in ["heightfield", "trimesh"]:
            self.edge_reset_buf = self.base_position[:, 0] > self.terrain_x_max - 1
            self.edge_reset_buf |= self.base_position[:, 0] < self.terrain_x_min + 1
            self.edge_reset_buf |= self.base_position[:, 1] > self.terrain_y_max - 1
            self.edge_reset_buf |= self.base_position[:, 1] < self.terrain_y_min + 1

        self.reset_buf = (
            (self.fail_buf > self.cfg.env.fail_to_terminal_time_s / self.dt)
            | self.time_out_buf
            | self.edge_reset_buf
        )

    # ---------- 观测 ----------
    def compute_proprioception_observations(self):
        """
        - 舵机关节：位置信息有效，速度信息置 0
        - 轮子关节：速度信息有效，位置信息置 0（轮子可无限转，角度无意义）
        """
        dof_pos_obs = (self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos
        dof_vel_obs = self.dof_vel * self.obs_scales.dof_vel

        # 轮子的位置观测置 0，改用动作本身（表示期望位置增量）
        dof_pos_obs_modified = self.actions * self.cfg.control.pos_action_scale * self.obs_scales.dof_pos
        dof_pos_obs_modified[:, self.wheel_joint_indices] = 0.0

        # 舵机的速度观测置 0
        dof_vel_obs_modified = dof_vel_obs.clone()
        dof_vel_obs_modified[:, self.leg_joint_indices] = 0.0

        obs_buf = torch.cat(
            (
                self.base_ang_vel * self.obs_scales.ang_vel,
                self.projected_gravity * self.obs_scales.gravity,
                self.commands[:, :3] * self.commands_scale,
                dof_pos_obs_modified,
                dof_vel_obs_modified,
                self.actions,
            ),
            dim=-1,
        )
        return obs_buf

    # ---------- 力矩 ----------
    def _compute_torques(self, actions):
        """
        腿部：位置 PD（+ 积分项，对应真实舵机 PID）
        轮部：速度控制（d_gains * (vel_ref - 0)，p_gain=0）
        """
        pos_ref = actions * self.cfg.control.pos_action_scale
        pos_ref[:, self.wheel_joint_indices] *= 0  # 轮子不用位置参考

        vel_ref = actions * self.cfg.control.vel_action_scale
        vel_ref[:, self.leg_joint_indices] *= 0    # 舵机不用速度参考

        dof_vel_ref = self.dof_vel.clone()
        dof_vel_ref[:, self.wheel_joint_indices] *= 0.0  # 轮子用速度误差，舵机不用

        torques = self.p_gains * (
            pos_ref + self.default_dof_pos - self.dof_pos
        ) + self.d_gains * (vel_ref - dof_vel_ref)

        # 舵机积分项
        if hasattr(self.cfg.control, "leg_integral_gain"):
            dt = self.sim_params.dt
            leg_pos_error = (pos_ref + self.default_dof_pos - self.dof_pos)[:, self.leg_joint_indices]
            self.leg_integral_error += leg_pos_error * dt
            windup_limit = getattr(self.cfg.control, "integral_windup_limit", 1.0)
            self.leg_integral_error = torch.clip(self.leg_integral_error, -windup_limit, windup_limit)
            integral_torques = self.cfg.control.leg_integral_gain * self.leg_integral_error
            torques[:, self.leg_joint_indices] += integral_torques

        return torch.clip(
            torques * self.torques_scale, -self.torque_limits, self.torque_limits
        )

    # ---------- 奖励 ----------
    def _leg_end_y(self, hip_idx, knee_idx, hip_sign):
        """用 2R 正向运动学算单侧轮心的纵向(y)相对位置。

        关节角以标称姿态为零点，hip_sign 处理左右髋关节轴的镜像
        （RightHip=+X, LeftHip=-X）。
        返回的是相对机身的 y 位移（不含髋安装偏置，因为左右偏置相同，
        做差时会抵消）。
        """
        hip = self.nominal_hip + hip_sign * self.dof_pos[:, hip_idx]
        knee = self.nominal_knee + self.dof_pos[:, knee_idx]
        end_y = self.car_l1 * torch.cos(hip) + self.car_l2 * torch.cos(hip + knee)
        return end_y

    def _reward_leg_together(self):
        """左右腿轮心纵向位置之差，越小越好 —— 防止两腿前后劈叉、原地画圈。"""
        right_y = self._leg_end_y(0, 1, self.hip_sign["right"])
        left_y = self._leg_end_y(3, 4, self.hip_sign["left"])
        end_x_diff = torch.abs(right_y - left_y)
        if self.reward_scales["leg_together"] < 0:
            return end_x_diff
        return torch.exp(-end_x_diff / 0.01)

    def _reward_base_height(self):
        """机身高度跟踪指令。"""
        if self.reward_scales["base_height"] < 0:
            return torch.abs(self.base_height - self.commands[:, 2])
        base_height_error = torch.square(self.base_height - self.commands[:, 2])
        return torch.exp(-base_height_error / 0.001)

    def _reward_tracking_ang_vel(self):
        """偏航角速度跟踪。"""
        ang_vel_error = torch.square(self.commands[:, 1] - self.base_ang_vel[:, 2])
        return torch.exp(-ang_vel_error / 0.1)
