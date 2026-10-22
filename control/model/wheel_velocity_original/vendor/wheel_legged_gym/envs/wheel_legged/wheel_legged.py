# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

import torch
from wheel_legged_gym.envs.base.legged_robot import LeggedRobot
from .wheel_legged_config import WheelLeggedCfg


class WheelLegged(LeggedRobot):
    def __init__(self, cfg: WheelLeggedCfg, sim_params, physics_engine, sim_device, headless):
        super().__init__(cfg, sim_params, physics_engine, sim_device, headless)
        
        # SF机器人关节索引定义
        # 关节顺序：LT_joint, LC_joint, LW_joint, RT_joint, RC_joint, RW_joint
        # 索引：     0,       1,       2,       3,       4,       5
        self.leg_joint_indices = [0, 1, 3, 4]  # 腿部关节（舵机）
        self.wheel_joint_indices = [2, 5]       # 轮子关节


    def compute_proprioception_observations(self):
        """
        计算本体感受观测，针对SF轮足机器人优化：
        - 舵机关节：保留位置信息，速度信息设为0
        - 轮子关节：保留速度信息，位置信息设为0
        """
        # 创建修改后的关节位置和速度观测
        dof_pos_obs = (self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos
        dof_vel_obs = self.dof_vel * self.obs_scales.dof_vel
        
        # 对于轮子关节，将位置观测设为0
        dof_pos_obs_modified = dof_pos_obs.clone()
        # dof_pos_obs_modified[:, self.wheel_joint_indices] = 0.0
        # dof_pos_obs_modified = self.actions * self.cfg.control.pos_action_scale
        dof_pos_obs_modified[:, self.wheel_joint_indices] = 0.0

        # 对于舵机关节，将速度观测设为0
        dof_vel_obs_modified = dof_vel_obs.clone()
        dof_vel_obs_modified[:, self.leg_joint_indices] = 0.0
        
        obs_buf = torch.cat(
            (
                self.base_ang_vel * self.obs_scales.ang_vel,
                self.projected_gravity,
                self.commands[:, :3] * self.commands_scale,
                dof_pos_obs_modified,  # 修改后的位置观测（轮子位置为0）
                dof_vel_obs_modified,  # 修改后的速度观测（舵机速度为0）
                self.actions,
            ),
            dim=-1,
        )
        return obs_buf

    def _compute_torques(self, actions):
        """
        计算力矩，针对SF轮足机器人优化：
        - 舵机关节：位置控制
        - 轮子关节：速度控制
        """
        # pd controller
        pos_ref = actions * self.cfg.control.pos_action_scale
        pos_ref[:, self.wheel_joint_indices] *= 0  # 轮子关节位置参考设为0
        
        vel_ref = actions * self.cfg.control.vel_action_scale
        vel_ref[:, self.leg_joint_indices] *= 0    # 舵机关节速度参考设为0
        
        torques = self.p_gains * (
            pos_ref + self.default_dof_pos - self.dof_pos
        ) + self.d_gains * (vel_ref - self.dof_vel)
        
        return torch.clip(
            torques * self.torques_scale, -self.torque_limits, self.torque_limits
        )

