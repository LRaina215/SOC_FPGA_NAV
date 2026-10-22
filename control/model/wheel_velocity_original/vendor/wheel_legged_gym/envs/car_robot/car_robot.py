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
        self.speed_integral = torch.zeros(self.num_envs, device=self.device)
        self.speed_reference = torch.zeros_like(self.speed_integral)
        self.yaw_integral = torch.zeros_like(self.speed_integral)
        self.jump_phase = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self.jump_timer = torch.zeros_like(self.speed_integral)
        self.jump_airborne = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.jump_push_start = self.reference_height.clone()
        self.jump_landing_timer = torch.zeros_like(self.speed_integral)
        self.jump_parameters = {}
        # 跳跃指令位是 0/1 电平，会保持到下次重采样（resampling_time = 5s）。
        # 用这个 latch 保证每条指令只触发一跳（上升沿触发）。
        self.jump_dispatched = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        from isaacgym import gymtorch
        self.car_body_states = gymtorch.wrap_tensor(
            self.gym.acquire_rigid_body_state_tensor(self.sim)).view(self.num_envs,-1,13)

    def reset_idx(self, env_ids):
        # 积分器属于回合状态，不能把上一回合的力矩带到下一回合。
        if hasattr(self, "leg_integral_error"):
            self.leg_integral_error[env_ids] = 0.0
            self.reference_height[env_ids] = self.cfg.init_state.pos[2]
            if hasattr(self, "jump_phase"):
                self.speed_integral[env_ids] = 0.
                self.speed_reference[env_ids] = 0.
                self.yaw_integral[env_ids] = 0.
                self.jump_phase[env_ids] = 0
                self.jump_timer[env_ids] = 0.
                self.jump_airborne[env_ids] = False
                self.jump_landing_timer[env_ids] = 0.
                self.jump_dispatched[env_ids] = False
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
        # 第 4 路指令 = 跳跃（0/1 电平），按 jump_fraction 采样。重采样同时解锁，
        # 让新指令有机会触发下一跳。
        if len(env_ids):
            fraction = float(getattr(self.cfg.commands, "jump_fraction", 0.0))
            self.commands[env_ids, 3] = (
                torch.rand(len(env_ids), device=self.device) < fraction
            ).float()
            self.jump_dispatched[env_ids] = False

    def _reset_root_states(self, env_ids):
        super()._reset_root_states(env_ids)
        from isaacgym import gymtorch
        # 基类给每个速度轴随机 ±0.5，对这台 13cm 高的车是很大的冲击。
        self.root_states[env_ids, 7:13] *= self.cfg.env.reset_velocity_scale

        # 2026-10-10：出生点铺满整块地形。
        # 基类只在中心 ±1m 内抖（而块中心 2m 是平平台），于是机器人总能站在平地上
        # 原地不动拿 upright 奖励 —— 实测这就是"躺平"那条捷径。把它摊到整块，
        # 一部分机器人直接出生在坡上，没有平地可站。
        # 代价：XY 变了以后 z 不再对，必须按【落点的真实地形高度】重算，
        # 否则会悬空或陷进坡里。
        if self.cfg.terrain.mesh_type in ["heightfield", "trimesh"]:
            rad = float(getattr(self.cfg.env, "spawn_radius", 0.0))
            if rad > 0:
                n = len(env_ids)
                self.root_states[env_ids, :2] = self.env_origins[env_ids, :2] + \
                    (2.0 * rad) * torch.rand((n, 2), device=self.device) - rad
                B = self.cfg.terrain.border_size
                HS = self.cfg.terrain.horizontal_scale
                VS = self.cfg.terrain.vertical_scale
                # 双线性插值取地形高度。用最近格的话，10° 坡上一个 10cm 的格内
                # 就有 17.6mm 高差，机器人会差这么多悬空/陷入（实测最大 22mm）。
                fx = (self.root_states[env_ids, 0] + B) / HS
                fy = (self.root_states[env_ids, 1] + B) / HS
                px = torch.clip(fx.long(), 0, self.height_samples.shape[0] - 2)
                py = torch.clip(fy.long(), 0, self.height_samples.shape[1] - 2)
                tx = fx - px
                ty = fy - py
                h = self.height_samples
                # height_samples 存的是 int16 原始计数，乘 vertical_scale 才是米
                z_terr = (
                    h[px, py] * (1 - tx) * (1 - ty)
                    + h[px + 1, py] * tx * (1 - ty)
                    + h[px, py + 1] * (1 - tx) * ty
                    + h[px + 1, py + 1] * tx * ty
                ).float() * VS
                self.root_states[env_ids, 2] = self.base_init_state[2] + z_terr
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
                torch.cat((self.commands[:, :3] * self.commands_scale,
                           self.commands[:, 3:4]), dim=-1),
                dof_pos_obs_modified,
                dof_vel_obs_modified,
                self.actions,
            ),
            dim=-1,
        )
        return obs_buf

    def post_physics_step(self):
        super().post_physics_step()
        self._dispatch_jumps()

    def _dispatch_jumps(self):
        """把【跳跃指令位的上升沿】转成一次 request_jump()。

        request_jump() 内部还会检查两个轮子是否接地、机身倾角是否 < 12°，
        不满足就不起飞（但 latch 照样置位，避免每个物理步都重试）。
        """
        if not self.cfg.control.jump_enabled:
            return
        want = (self.commands[:, 3] > 0.5) & (~self.jump_dispatched) & (self.jump_phase == 0)
        ids = want.nonzero(as_tuple=False).flatten()
        if len(ids) == 0:
            return
        self.request_jump(ids)
        self.jump_dispatched[ids] = True

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

        if self.cfg.control.motion_controller == "feedback":
            self._feedback_wheels(torques)

        return torch.clip(
            torques * self.torques_scale, -self.torque_limits, self.torque_limits
        )

    def _leg_reference(self):
        """高度指令的解析前馈：先用 IK 给参考角，再由策略做小幅修正。"""
        if not self.cfg.control.height_reference:
            return self.default_dof_pos
        target = torch.clamp(self.commands[:, 2], self.cfg.control.min_reference_height, 0.19)
        limit = self.cfg.control.height_reference_rate*self.sim_params.dt
        change = torch.clamp(target-self.reference_height, -limit, limit)
        if self.cfg.control.jump_enabled:
            change = torch.where((self.jump_phase == 0) | (self.jump_phase == 4), change,
                                 torch.zeros_like(change))
        self.reference_height += change
        if self.cfg.control.jump_enabled:
            self._jump_reference()
        h = self.reference_height
        y = torch.full_like(h, -0.03)
        if self.cfg.control.jump_enabled:
            jump_y = self._jump_parameter('jump_leg_y')-self.cfg.control.jump_leg_y_speed_gain*self.speed_reference
            recovering = torch.clamp(self.jump_timer/.35,0.,1.)
            jump_y = torch.where(self.jump_phase==4,jump_y+(-.03-jump_y)*recovering,jump_y)
            y = torch.where(self.jump_phase > 0, jump_y, y)
        y = y-self.hip_y_offset
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

    def request_jump(self, env_ids=None):
        """Start a motor-driven jump; commands keep their forward/yaw values."""
        if not self.cfg.control.jump_enabled:
            return 0
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        # Net force flickers around zero in steady rolling contact. Geometry
        # avoids rejecting a jump solely because this physics frame has zero
        # normal force; both wheels must be within 5 mm of the flat floor.
        contact = self.wheel_clearances().max(1).values < .005
        allowed = (self.jump_phase[env_ids] == 0) & contact[env_ids]
        allowed &= self.projected_gravity[env_ids, 2] < -math.cos(math.radians(12))
        ids = env_ids[allowed]
        self.jump_phase[ids] = 1
        self.jump_timer[ids] = 0.
        self.jump_airborne[ids] = False
        self.jump_landing_timer[ids] = 0.
        return len(ids)

    def wheel_clearances(self):
        """轮子下缘相对【本环境地面】的高度（不是相对世界 z=0）。"""
        wheels = self.car_body_states[:, self.feet_indices]
        q = wheels[:,:,3:7]
        axis_z = 2*(q[:,:,0]*q[:,:,2]-q[:,:,3]*q[:,:,1])
        extent = .0325*torch.sqrt(torch.clamp(1-axis_z.square(),0.,1.))+.009*torch.abs(axis_z)
        # 2026-10-10: 减去本环境的地面基准高度。plane 模式下 env_origins[:,2]==0，
        # 行为与以前完全一致；heightfield 模式下各块地形基准面高度不同（-0.135~0.170 m），
        # 不减的话 request_jump 里 "轮子离地 < 5mm" 的接地判据会在一半地形上永远为假，
        # 跳跃根本触发不了。
        return wheels[:,:,2]-extent-self.env_origins[:, 2].unsqueeze(-1)

    def _jump_parameter(self, name):
        return self.jump_parameters.get(name, getattr(self.cfg.control, name))

    def _jump_reference(self):
        dt = self.sim_params.dt
        active = self.jump_phase > 0
        if not torch.any(active):
            return
        self.jump_timer[active] += dt
        crouch = self.jump_phase == 1
        delta = torch.clamp(self.cfg.control.min_reference_height-self.reference_height,
                            -.25*dt, .25*dt)
        self.reference_height[crouch] += delta[crouch]
        ready = crouch & (self.jump_timer >= self._jump_parameter('jump_crouch_s'))
        self.jump_phase[ready] = 2
        self.jump_timer[ready] = 0.
        self.jump_push_start[ready] = self.reference_height[ready]
        push = self.jump_phase == 2
        push_s = self._jump_parameter('jump_push_s')
        fraction = torch.clamp(self.jump_timer/push_s, 0., 1.)
        target = self.jump_push_start+(self._jump_parameter('jump_push_height')-self.jump_push_start)*fraction
        self.reference_height[push] = target[push]
        extend_done = push & (self.jump_timer >= push_s+self._jump_parameter('jump_push_hold_s'))
        self.jump_phase[extend_done] = 3
        self.jump_timer[extend_done] = 0.
        flight = self.jump_phase == 3
        contact = (self.contact_forces[:, self.feet_indices, 2].sum(1)>1.) | (self.wheel_clearances().min(1).values < .004)
        self.jump_airborne |= flight & (~contact)
        air_target = self._jump_parameter('jump_air_height')
        self.reference_height[flight] += torch.clamp(air_target-self.reference_height,
                                                     -1.2*dt, 1.2*dt)[flight]
        landed = flight & self.jump_airborne & contact & (self.jump_timer > .12)
        timeout = flight & (self.jump_timer > 1.0)
        self.jump_phase[landed | timeout] = 4
        self.jump_timer[landed | timeout] = 0.
        landing = self.jump_phase == 4
        # Return to the user's commanded ride height and allow another jump
        # only after a stable recovery interval.
        recovered = landing & (self.jump_timer > .45)
        self.jump_phase[recovered] = 0
        self.jump_timer[recovered] = 0.

    def _feedback_wheels(self, torques):
        """Balance + PI body-speed feedback; no velocity/position injection."""
        right, left = self.right_indices[2], self.left_indices[2]
        cfg = self.cfg.control
        gain = lambda name: self._jump_parameter(name)
        velocity = self.task_forward_velocity()
        if cfg.velocity_source == "wheel":
            velocity = .0325*.5*(self.dof_vel[:, right]-self.dof_vel[:, left])
        flight_stage = (self.jump_phase >= 2) & (self.jump_phase <= 3)
        airborne = torch.zeros_like(flight_stage)
        if torch.any(flight_stage):
            contact = (self.contact_forces[:, self.feet_indices, 2].sum(1)>1.) | (self.wheel_clearances().min(1).values < .004)
            airborne = flight_stage & (~contact)
        speed_delta = torch.clamp(self.commands[:, 0]-self.speed_reference,
                                 -cfg.speed_reference_rate*self.sim_params.dt,
                                 cfg.speed_reference_rate*self.sim_params.dt)
        self.speed_reference += torch.where(airborne, torch.zeros_like(speed_delta), speed_delta)
        error = velocity-self.speed_reference
        self.speed_integral += torch.where(airborne, torch.zeros_like(error), error)*self.sim_params.dt
        self.speed_integral.clamp_(-1., 1.)
        yaw_error = self.commands[:, 1]-self.base_ang_vel[:, 2]
        self.yaw_integral += torch.where(airborne, torch.zeros_like(error), yaw_error)*self.sim_params.dt
        self.yaw_integral.clamp_(-2., 2.)
        common = gain('balance_kp')*self.projected_gravity[:, 1]-gain('balance_kd')*self.base_ang_vel[:, 0]
        common += gain('speed_kp')*error+gain('speed_ki')*self.speed_integral
        turn = gain('yaw_kp')*yaw_error+gain('yaw_ki')*self.yaw_integral
        common = torch.clamp(common, -cfg.forward_torque_limit, cfg.forward_torque_limit)
        turn = torch.clamp(turn, -cfg.turn_torque_limit, cfg.turn_torque_limit)
        # In flight, wheels provide small pitch reaction torques. Longitudinal
        # feedback/integrators stay frozen until contact returns.
        air_common = gain('balance_kp')*self.projected_gravity[:, 1]-gain('balance_kd')*self.base_ang_vel[:, 0]
        air_limit = gain('jump_air_torque_limit')
        common = torch.where(airborne, torch.clamp(air_common, -air_limit, air_limit), common)
        turn = torch.where(airborne, torch.zeros_like(turn), turn)
        torques[:, right] = common+turn
        torques[:, left] = -common+turn

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

    # ---------- 跳跃奖励 ----------
    def _jump_flying(self):
        """飞行阶段（push=2 / air=3）的布尔掩码。"""
        return (self.jump_phase >= 2) & (self.jump_phase <= 3)

    def _reward_jump_airborne(self):
        """飞行阶段且轮子真的离地（>5mm）—— 逼它真的跳起来，而不是只做个蹲起动作。"""
        return (self._jump_flying() & (self.wheel_clearances().min(1).values > 0.005)).float()

    def _reward_jump_height(self):
        """飞行阶段的离地高度 [m]，越高越好。"""
        return self.wheel_clearances().min(1).values.clamp(min=0.0) * self._jump_flying().float()

    def _reward_jump_landing(self):
        """落地阶段（phase 4）的姿态偏差 —— 惩罚跳完就摔。

        这是个惩罚项（config 里给负权重），所以返回的是误差本身而不是 exp(-err)。
        """
        tilt = torch.sum(torch.square(self.projected_gravity[:, :2]), dim=1)
        return tilt * (self.jump_phase == 4).float()

    def _reward_leg_together(self):
        """左右腿轮心纵向位置之差，越小越好 —— 防止两腿前后劈叉、原地画圈。"""
        right_y = self._leg_end_y(*self.right_indices[:2], self.hip_sign["right"])
        left_y = self._leg_end_y(*self.left_indices[:2], self.hip_sign["left"])
        end_x_diff = torch.abs(right_y - left_y)
        if self.reward_scales["leg_together"] < 0:
            return end_x_diff
        return torch.exp(-end_x_diff / 0.01)

    # ---------- 2026-10-11 奖励重设计（对照表见 car_robot_config.py 的 scales 注释）----------

    def _upright_gate(self):
        """直立门控：clamp(-重力z, 0, 1)。完全直立=1，倾 20°≈0.94，倾 30°(终止线)≈0.87。

        参照 LocoWheeledLegged 的做法：把它乘在跟踪奖励上 —— 不直立就拿不到跟踪分。
        这样"站着不动"不再有任何白拿钱的通道（upright 正奖励已删），躺平在
        结构上变得无利可图，而不是靠加大跟踪权重去"压过"它。
        """
        return torch.clamp(-self.projected_gravity[:, 2], 0.0, 1.0)

    def _reward_base_height(self):
        """机身离地高度跟踪 —— 平方误差（配负权重 = L2 惩罚）。

        self.base_height 是基类算的 root_z − 周围地形采样均值（米，见
        _get_heights 返回值乘过 vertical_scale），本来就是相对地形的，坡上不失效。
        弃用旧 exp(σ²=0.001)：系统性偏差 4~10mm 时 e^-16~-100 ≈ 0，梯度为零；
        L2 在小偏差处温和（5mm 只罚 -10×2.5e-5）、大偏差处线性施压（50mm 罚 -0.025）。
        """
        return torch.square(self.base_height - self.commands[:, 2])

    def _reward_tracking_ang_vel(self):
        """偏航角速度跟踪 × 直立门控。"""
        ang_vel_error = torch.square(self.commands[:, 1] - self.base_ang_vel[:, 2])
        return torch.exp(-ang_vel_error / 0.1) * self._upright_gate()

    def _reward_tracking_lin_vel(self):
        # 此模型轮轴沿 X，行驶方向为 +Y；基类的 X 是横向速度。
        # 2026-10-11：乘直立门控 —— 倾着走拿不满跟踪分，与 orientation 惩罚同向。
        error = torch.square(self.commands[:, 0] - self.task_forward_velocity())
        return torch.exp(-error / self.cfg.rewards.tracking_sigma) * self._upright_gate()

    def _reward_action_rate(self):
        """动作变化率惩罚 —— 轮子维 ×0.01。

        轮子是平衡的主执行器（满幅 0.5 N·m 要能快速出力救倾），按腿的力度
        罚它的动作变化会拖慢救平衡。参照 LocoWheeledLegged 把轮子加速度罚
        降一个量级的同一思路。基类用的是 last_actions[:, :, 0]（带延迟缓存）。
        """
        rate = self.actions - self.last_actions[:, :, 0]
        rate[:, self.wheel_joint_indices] *= 0.01
        return torch.sum(torch.square(rate), dim=1)

    def _reward_dof_acc(self):
        """腿关节加速度惩罚 —— 只算腿 4 维（轮子见 _reward_wheel_acc）。"""
        return torch.sum(torch.square(self.dof_acc[:, self.leg_joint_indices]), dim=1)

    def _reward_wheel_acc(self):
        """轮子加速度惩罚 —— 权重比腿轻 100 倍（-2.5e-9），别捆住救平衡的手。"""
        return torch.sum(torch.square(self.dof_acc[:, self.wheel_joint_indices]), dim=1)

    def _reward_dof_pos_limits(self):
        """腿部关节越限惩罚 —— 按名称索引（基类写死的列号 [:2]/[3:5] 恰好也是腿，
        但按名取更稳），轮子可无限转、不罚。
        """
        lo = self.dof_pos_limits[self.leg_joint_indices, 0]
        hi = self.dof_pos_limits[self.leg_joint_indices, 1]
        pos = self.dof_pos[:, self.leg_joint_indices]
        out_of_limits = -(pos - lo).clip(max=0.0) + (pos - hi).clip(min=0.0)
        return torch.sum(out_of_limits, dim=1)

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
