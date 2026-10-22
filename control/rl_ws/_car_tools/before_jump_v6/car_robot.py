# 轮腿车（car）强化学习环境
#
# 由 sf_robot.py 改写，适配 car_open.urdf。
#
# 关节索引动态按名称查询，IsaacGym 实测会把 Left 排在 Right 前。
#
# 控制方式沿用实车硬件约定：
#   腿部 4 个舵机 -> 位置控制（PD + 积分）
#   轮部 2 个电机 -> 开环力矩近似或轮速反馈模式（由配置选择）

import math

import torch
from wheel_legged_gym.envs.base.legged_robot import LeggedRobot
from .car_robot_config import CarRobotCfg


class CarRobot(LeggedRobot):
    def __init__(self, cfg: CarRobotCfg, sim_params, physics_engine, sim_device, headless):
        super().__init__(cfg, sim_params, physics_engine, sim_device, headless)

        # 关节索引：腿部（舵机）与轮部（电机）
        # IsaacGym 会重排 URDF 关节（实测 Left 在 Right 前），必须按名称查询。
        self.right_indices = [self.dof_names.index("Right"+n+"_joint")
                              for n in ["Hip", "Knee", "Wheel"]]
        self.left_indices = [self.dof_names.index("Left"+n+"_joint")
                             for n in ["Hip", "Knee", "Wheel"]]
        self.leg_joint_indices = self.right_indices[:2] + self.left_indices[:2]
        self.wheel_joint_indices = [self.right_indices[2], self.left_indices[2]]

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
        # URDF 的关节角是 CAD 零位的增量，不能再次加对称标称姿态。
        self.cad_hip = [-1.107851943952, -0.527016956249]
        self.cad_knee = [-1.447594775649, -1.887671992209]
        # 髋关节在 y 方向的安装偏置（左右相同）
        self.hip_y_offset = 0.022468
        # 关节轴符号：RightHip=+X, LeftHip=-X, Knee 均为 +X
        self.hip_sign = {"right": +1.0, "left": -1.0}

        # 姿态失败阈值（见 check_termination）
        # projected_gravity[2] = -cos(tilt)，所以阈值就是 -cos(tilt_fail)
        tilt_fail_rad = math.radians(float(self.cfg.env.tilt_fail_deg))
        self.tilt_fail_cos = -math.cos(tilt_fail_rad)
        self.reference_height = torch.full((self.num_envs,), self.cfg.init_state.pos[2],
                                           device=self.device)

    def reset_idx(self, env_ids):
        # 积分器属于回合状态，不能把上一回合的力矩带到下一回合。
        if hasattr(self, "leg_integral_error"):
            self.leg_integral_error[env_ids] = 0.0
            self.reference_height[env_ids] = self.cfg.init_state.pos[2]
        super().reset_idx(env_ids)

    def _resample_commands(self, env_ids):
        super()._resample_commands(env_ids)
        # 连续均匀采样几乎不包含精确的零速度，单独安排停车/原地升降。
        fraction = self.cfg.commands.standing_fraction
        if len(env_ids) and fraction > 0:
            stop_ids = env_ids[torch.rand(len(env_ids), device=self.device) < fraction]
            self.commands[stop_ids, :2] = 0.0
        if len(env_ids) and self.cfg.commands.boundary_sampling:
            category = torch.rand(len(env_ids), device=self.device)
            stop = env_ids[category < 0.25]
            forward = env_ids[(category >= 0.25) & (category < 0.45)]
            turn = env_ids[(category >= 0.45) & (category < 0.65)]
            self.commands[stop, :2] = 0.0
            self.commands[forward, 0] = (2*torch.randint(0, 2, (len(forward),), device=self.device)-1)*0.6
            self.commands[forward, 1] = 0.0
            self.commands[turn, 0] = 0.0
            self.commands[turn, 1] = (2*torch.randint(0, 2, (len(turn),), device=self.device)-1)*2.9
            edge = torch.rand(len(env_ids), device=self.device)
            self.commands[env_ids[edge < 0.1], 2] = self.cfg.control.min_reference_height
            self.commands[env_ids[edge > 0.9], 2] = 0.19

    def _reset_root_states(self, env_ids):
        super()._reset_root_states(env_ids)
        from isaacgym import gymtorch
        # 基类给每个速度轴随机 ±0.5，对这台 13cm 高的车是很大的冲击。
        self.root_states[env_ids, 7:13] *= self.cfg.env.reset_velocity_scale
        ids = env_ids.to(dtype=torch.int32)
        self.gym.set_actor_root_state_tensor_indexed(
            self.sim, gymtorch.unwrap_tensor(self.root_states),
            gymtorch.unwrap_tensor(ids), len(ids))

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
        轮部：torque 模式不减去轮速；velocity 模式使用轮速反馈。
        """
        pos_ref = actions * self.cfg.control.pos_action_scale
        pos_ref[:, self.wheel_joint_indices] *= 0  # 轮子不用位置参考

        vel_ref = actions * self.cfg.control.vel_action_scale
        vel_ref[:, self.leg_joint_indices] *= 0    # 舵机不用速度参考

        dof_vel_ref = self.dof_vel.clone()
        # wheel_control=torque 对应原来的开环力矩近似；velocity 才用轮速反馈。
        if self.cfg.control.wheel_control == "torque":
            dof_vel_ref[:, self.wheel_joint_indices] = 0.0

        reference = self._leg_reference()
        torques = self.p_gains * (
            pos_ref + reference - self.dof_pos
        ) + self.d_gains * (vel_ref - dof_vel_ref)

        # 舵机积分项
        if hasattr(self.cfg.control, "leg_integral_gain"):
            dt = self.sim_params.dt
            leg_pos_error = (pos_ref + reference - self.dof_pos)[:, self.leg_joint_indices]
            self.leg_integral_error += leg_pos_error * dt
            windup_limit = getattr(self.cfg.control, "integral_windup_limit", 1.0)
            self.leg_integral_error = torch.clip(self.leg_integral_error, -windup_limit, windup_limit)
            integral_torques = self.cfg.control.leg_integral_gain * self.leg_integral_error
            torques[:, self.leg_joint_indices] += integral_torques

        return torch.clip(
            torques * self.torques_scale, -self.torque_limits, self.torque_limits
        )

    def _leg_reference(self):
        """高度指令的解析前馈：先用 IK 给参考角，再由策略做小幅修正。"""
        if not self.cfg.control.height_reference:
            return self.default_dof_pos
        target = torch.clamp(self.commands[:, 2], self.cfg.control.min_reference_height, 0.19)
        limit = 0.05*self.sim_params.dt
        self.reference_height += torch.clamp(target-self.reference_height, -limit, limit)
        h = self.reference_height
        y = torch.full_like(h, -0.03 - self.hip_y_offset)
        z = 0.0325 - h - 0.033
        c = (y*y+z*z-self.car_l1**2-self.car_l2**2)/(2*self.car_l1*self.car_l2)
        knee = -torch.acos(torch.clamp(c, -1.0, 1.0))
        hip = torch.atan2(z, y)-torch.atan2(self.car_l2*torch.sin(knee),
                                         self.car_l1+self.car_l2*torch.cos(knee))
        reference = self.default_dof_pos.clone()
        for side, idx, sign in [(0, self.right_indices, 1), (1, self.left_indices, -1)]:
            reference[:, idx[0]] = (hip-self.cad_hip[side])/sign
            reference[:, idx[1]] = knee-self.cad_knee[side]
        return reference

    # ---------- 奖励 ----------
    def _leg_end_y(self, hip_idx, knee_idx, hip_sign):
        """用 2R 正向运动学算单侧轮心的纵向(y)相对位置。

        关节角以 CAD 姿态为零点，hip_sign 处理左右髋关节轴的镜像
        （RightHip=+X, LeftHip=-X）。
        返回的是相对机身的 y 位移（不含髋安装偏置，因为左右偏置相同，
        做差时会抵消）。
        """
        side = 0 if hip_idx == self.right_indices[0] else 1
        hip = self.cad_hip[side] + hip_sign * self.dof_pos[:, hip_idx]
        knee = self.cad_knee[side] + self.dof_pos[:, knee_idx]
        end_y = self.car_l1 * torch.cos(hip) + self.car_l2 * torch.cos(hip + knee)
        return end_y

    def _reward_leg_together(self):
        """左右腿轮心纵向位置之差，越小越好 —— 防止两腿前后劈叉、原地画圈。"""
        right_y = self._leg_end_y(*self.right_indices[:2], self.hip_sign["right"])
        left_y = self._leg_end_y(*self.left_indices[:2], self.hip_sign["left"])
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

    def _reward_tracking_lin_vel(self):
        # 此模型轮轴沿 X，行驶方向为 +Y；基类的 X 是横向速度。
        error = torch.square(self.commands[:, 0] - self.task_forward_velocity())
        return torch.exp(-error / self.cfg.rewards.tracking_sigma)

    def _reward_nominal_state(self):
        # 基类 theta0 是另一台机器的 VMC 几何，不适用于本车。
        return torch.mean(torch.square(
            (self.dof_pos - self.default_dof_pos)[:, self.leg_joint_indices]), dim=1)

    def _reward_upright(self):
        return torch.exp(-torch.sum(torch.square(self.projected_gravity[:, :2]),
                                   dim=1) / 0.04)

    def _reward_yaw_error(self):
        return torch.square(self.base_ang_vel[:, 2]-self.commands[:, 1])

    def _reward_forward_error(self):
        return torch.square(self.task_forward_velocity()-self.commands[:, 0])

    def task_forward_velocity(self):
        # v(point)=v(CAD origin)+omega cross offset. 转向时 CAD 原点有切向速度，
        # 使用车身中心线上的点，避免把原地转向误判成纵向平移。
        return self.base_lin_vel[:, 1]+self.base_ang_vel[:, 2]*self.cfg.asset.forward_reference_x
