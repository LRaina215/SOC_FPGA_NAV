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

import numpy as np
from numpy.random import choice
from scipy import interpolate

from isaacgym import terrain_utils
from wheel_legged_gym.envs.base.legged_robot_config import LeggedRobotCfg


class Terrain:
    def __init__(self, cfg: LeggedRobotCfg.terrain, num_robots) -> None:

        self.cfg = cfg
        self.num_robots = num_robots
        self.type = cfg.mesh_type
        if self.type in ["none", "plane"]:
            return
        self.env_length = cfg.terrain_length
        self.env_width = cfg.terrain_width
        self.proportions = [
            np.sum(cfg.terrain_proportions[: i + 1])
            for i in range(len(cfg.terrain_proportions))
        ]

        self.cfg.num_sub_terrains = cfg.num_rows * cfg.num_cols
        self.env_origins = np.zeros((cfg.num_rows, cfg.num_cols, 3))

        self.width_per_env_pixels = int(self.env_width / cfg.horizontal_scale)
        self.length_per_env_pixels = int(self.env_length / cfg.horizontal_scale)

        self.border = int(cfg.border_size / self.cfg.horizontal_scale)
        self.tot_cols = int(cfg.num_cols * self.width_per_env_pixels) + 2 * self.border
        self.tot_rows = int(cfg.num_rows * self.length_per_env_pixels) + 2 * self.border

        self.height_field_raw = np.zeros((self.tot_rows, self.tot_cols), dtype=np.int16)
        if cfg.curriculum:
            self.curiculum()
        elif cfg.selected:
            self.selected_terrain()
        else:
            self.randomized_terrain()

        self.heightsamples = self.height_field_raw
        if self.type == "trimesh":
            self.vertices, self.triangles = (
                terrain_utils.convert_heightfield_to_trimesh(
                    self.height_field_raw,
                    self.cfg.horizontal_scale,
                    self.cfg.vertical_scale,
                    self.cfg.slope_treshold,
                )
            )

    def randomized_terrain(self):
        for k in range(self.cfg.num_sub_terrains):
            # Env coordinates in the world
            (i, j) = np.unravel_index(k, (self.cfg.num_rows, self.cfg.num_cols))

            choice = np.random.uniform(0, 1)
            difficulty = np.random.choice([0.5, 0.75, 0.9])
            terrain = self.make_terrain(choice, difficulty)
            self.add_terrain_to_map(terrain, i, j)

    def curiculum(self):
        for j in range(self.cfg.num_cols):
            for i in range(self.cfg.num_rows):
                difficulty = i / self.cfg.num_rows
                choice = j / self.cfg.num_cols + 0.001

                terrain = self.make_terrain(choice, difficulty)
                self.add_terrain_to_map(terrain, i, j)

    def selected_terrain(self):
        terrain_type = self.cfg.terrain_kwargs.pop("type")
        for k in range(self.cfg.num_sub_terrains):
            # Env coordinates in the world
            (i, j) = np.unravel_index(k, (self.cfg.num_rows, self.cfg.num_cols))

            terrain = terrain_utils.SubTerrain(
                "terrain",
                width=self.width_per_env_pixels,
                length=self.width_per_env_pixels,
                vertical_scale=self.vertical_scale,
                horizontal_scale=self.horizontal_scale,
            )

            eval(terrain_type)(terrain, **self.cfg.terrain_kwargs.terrain_kwargs)
            self.add_terrain_to_map(terrain, i, j)

    def make_terrain_custom(self, choice, difficulty):
        """四种定制地形（2026-10-10）：上坡 / 下坡 / 粗糙地面 / 飞坡。

        由 cfg.custom_terrains = True 打开。占比用 cfg.terrain_proportions，
        4 个元素即可（累积成 4 段）。

        2026-10-11：difficulty 【开始使用】—— 课程模式（curriculum=True）下
        difficulty = 行号/num_rows ∈ {0, 0.2, 0.4, 0.6, 0.8}，按
        f = 0.25 + 0.9375*difficulty 缩放坡度/起伏：
          最易行(0)    ≈ 1/4 难度：2.5° 坡 / ±1.25mm 起伏 / 65mm 飞坡
          最难行(0.8)  = 满难度：  10°  坡 / ±5mm    起伏 / 264mm 飞坡+断崖
        randomized 模式传入的 {0.5, 0.75, 0.9} 也能工作（约 0.72~1.0 难度）。
        """
        terrain = terrain_utils.SubTerrain(
            "terrain",
            width=self.width_per_env_pixels,
            length=self.width_per_env_pixels,
            vertical_scale=self.cfg.vertical_scale,
            horizontal_scale=self.cfg.horizontal_scale,
        )
        f = 0.25 + 0.9375 * float(difficulty)
        p = self.proportions
        if choice < p[0]:                                    # 上坡
            slope_terrain(terrain, self.cfg.slope_deg * f, sign=+1.0)
        elif choice < p[1]:                                  # 下坡
            slope_terrain(terrain, self.cfg.slope_deg * f, sign=-1.0)
        elif choice < p[2]:                                  # 粗糙地面
            terrain_utils.random_uniform_terrain(
                terrain,
                min_height=-self.cfg.rough_height * f,
                max_height=self.cfg.rough_height * f,
                step=terrain.vertical_scale,
                downsampled_scale=0.2,
            )
        elif choice < p[3]:                                  # 飞坡
            jump_ramp_terrain(terrain, self.cfg.slope_deg * f)
        else:                                                # 山丘（2026-10-11：连续上下坡）
            hill_terrain(terrain, self.cfg.slope_deg * f)
        return terrain

    def make_terrain(self, choice, difficulty):
        # 2026-10-10：定制模式（4 种地形）优先
        if getattr(self.cfg, "custom_terrains", False):
            return self.make_terrain_custom(choice, difficulty)
        terrain = terrain_utils.SubTerrain(
            "terrain",
            width=self.width_per_env_pixels,
            length=self.width_per_env_pixels,
            vertical_scale=self.cfg.vertical_scale,
            horizontal_scale=self.cfg.horizontal_scale,
        )
        slope = difficulty * 0.5
        random_height = 0.005 + difficulty * 0.02
        step_height = 0.05 + 0.18 * difficulty
        discrete_obstacles_height = 0.05 + difficulty * 0.1
        stepping_stones_size = 1.5 * (1.05 - difficulty)
        stone_distance = 0.05 if difficulty == 0 else 0.1
        gap_size = 1.0 * difficulty
        pit_depth = 1.0 * difficulty
        if choice < self.proportions[0]:
            terrain_utils.pyramid_sloped_terrain(terrain, slope=0, platform_size=3.0)
        elif choice < self.proportions[1]:
            # 2026-10-10：真斜面。原来用 pyramid_sloped_terrain，被 np.clip 削成了碗。
            sign = (
                -1.0
                if choice < self.proportions[0] + (self.proportions[1] - self.proportions[0]) / 2
                else 1.0
            )
            slope_terrain(
                terrain,
                getattr(self.cfg, "slope_deg", 10.0),
                platform_size=3.0,
                sign=sign,
            )
        elif choice < self.proportions[2]:
            if (
                choice
                < self.proportions[1] + (self.proportions[2] - self.proportions[1]) / 2
            ):
                slope *= -1
            # 2026-10-10：纯粗糙地面，去掉那个假的"坡"
            rh = getattr(self.cfg, "rough_height", 0.005)
            terrain_utils.random_uniform_terrain(
                terrain,
                min_height=-rh,
                max_height=rh,
                step=terrain.vertical_scale,
                downsampled_scale=0.2,
            )
        elif choice < self.proportions[4]:
            if choice < self.proportions[3]:
                step_height *= -1
            terrain_utils.pyramid_stairs_terrain(
                terrain, step_width=0.7, step_height=step_height, platform_size=4.0
            )
        elif choice < self.proportions[5]:
            num_rectangles = 20
            rectangle_min_size = 1.0
            rectangle_max_size = 2.0
            terrain_utils.discrete_obstacles_terrain(
                terrain,
                discrete_obstacles_height,
                rectangle_min_size,
                rectangle_max_size,
                num_rectangles,
                platform_size=3.0,
            )
        elif choice < self.proportions[6]:
            terrain_utils.stepping_stones_terrain(
                terrain,
                stone_size=stepping_stones_size,
                stone_distance=stone_distance,
                max_height=0.0,
                platform_size=4.0,
            )
        elif choice < self.proportions[7]:
            gap_terrain(terrain, gap_size=gap_size, platform_size=3.0)
        else:
            pit_terrain(terrain, depth=pit_depth, platform_size=4.0)

        return terrain

    def add_terrain_to_map(self, terrain, row, col):
        i = row
        j = col
        # map coordinate system
        start_x = self.border + i * self.length_per_env_pixels
        end_x = self.border + (i + 1) * self.length_per_env_pixels
        start_y = self.border + j * self.width_per_env_pixels
        end_y = self.border + (j + 1) * self.width_per_env_pixels
        self.height_field_raw[start_x:end_x, start_y:end_y] = terrain.height_field_raw

        env_origin_x = (i + 0.5) * self.env_length
        env_origin_y = (j + 0.5) * self.env_width
        x1 = int((self.env_length / 2.0 - 1) / terrain.horizontal_scale)
        x2 = int((self.env_length / 2.0 + 1) / terrain.horizontal_scale)
        y1 = int((self.env_width / 2.0 - 1) / terrain.horizontal_scale)
        y2 = int((self.env_width / 2.0 + 1) / terrain.horizontal_scale)
        env_origin_z = (
            np.max(terrain.height_field_raw[x1:x2, y1:y2]) * terrain.vertical_scale
        )
        self.env_origins[i, j] = [env_origin_x, env_origin_y, env_origin_z]


def slope_terrain(terrain, slope_deg, sign=1.0, flat_frac=2.0 / 3.0):
    """单向斜坡 —— 前 flat_frac 段保持水平（含出生平台），后段按 slope_deg 升/降。

    【为什么不用 terrain_utils.pyramid_sloped_terrain】
    它末尾有 np.clip(h, min_h, max_h)，而 min/max 取自【平台角点】—— 平台
    角点的金字塔高度只有边缘的 1/4，所以整块被 clip 成"平台那一个高度"，
    结果是个【缓碗】而不是坡。2026-10-10 实测：难度 0.8 时全块高差仅 19cm
    且中间低四周高，出生区 2m 内只有 3cm 变化。

    【为什么 flat_frac 默认 2/3】
    env_origin_z 取的是【出生点中心 2m×2m 内的最大高度】。6m 的块中心 2m 是
    [2,4]，所以坡必须从 4m 之后才开始，否则机器人会悬空出生再往下掉。

    参数:
        slope_deg   坡度 [度]
        sign        +1 = 上坡, -1 = 下坡（沿 length 轴 = 车前进方向 +Y）
        flat_frac   前多少比例保持水平，2/3 -> 坡从 4m 起（6m 块）
    """
    slope = np.tan(np.radians(slope_deg)) * sign
    w, l = terrain.width, terrain.length
    yy = np.arange(l) * terrain.horizontal_scale          # 米
    y0 = l * terrain.horizontal_scale * flat_frac          # 起坡位置 [m]
    height_m = np.maximum(yy - y0, 0.0) * slope
    hf = (height_m / terrain.vertical_scale).astype(np.int16)
    terrain.height_field_raw += np.broadcast_to(hf[None, :], (w, l))
    return terrain


def hill_terrain(terrain, slope_deg, flat_frac=1.0 / 3.0):
    """连续上下坡（山丘）—— 上坡到峰、下坡回 0，首尾都是 0 与相邻块平顺衔接。

    2026-10-11 新增：用户要求"上下坡连贯，不能分开"。原来的上坡块到块尾直接
    掉回 0（等于断崖），不是翻山。山丘形状（6m 块，车沿 +Y 开）:
        0 ~ 2m    平（出生区，flat_frac=1/3）
        2 ~ 4m    按 slope_deg 上坡 -> 峰高 2m*tan(deg) ≈ 353mm @10°
        4 ~ 6m    对称下坡 -> 回到 0

    注意：与坡类不同，这里 flat_frac 只有 1/3 —— 块中心 2m 采样窗 [2,4] 会包含
    上坡段，env_origin_z 会取到 +353mm；出生 z 由 _reset_root_states 的双线性
    插值按落点真实高度修正，所以没问题（出生点在山脚平段）。

    参数:
        slope_deg  坡度 [度]（上下两段同坡度）
        flat_frac  前多少比例保持水平（1/3 -> 2m 平 + 2m 上 + 2m 下）
    """
    slope = np.tan(np.radians(slope_deg))
    w, l = terrain.width, terrain.length
    yy = np.arange(l) * terrain.horizontal_scale
    y0 = l * terrain.horizontal_scale * flat_frac
    y1 = l * terrain.horizontal_scale * (flat_frac + (1.0 - flat_frac) / 2.0)
    up = np.clip(yy - y0, 0.0, y1 - y0) * slope    # 上坡段，到峰后保持峰高
    down = np.clip(yy - y1, 0.0, None) * slope     # 下坡段线性下降
    height_m = up - down
    hf = (height_m / terrain.vertical_scale).astype(np.int16)
    terrain.height_field_raw += np.broadcast_to(hf[None, :], (w, l))
    return terrain


def jump_ramp_terrain(terrain, slope_deg, flat_frac=2.0 / 3.0, ramp_frac=0.25):
    """飞坡 —— 上坡冲到顶后【断崖】，没有下坡，专门用来起跳。

    形状（6m 块，车沿 +Y 开）:
        0 ~ 4m        平（含出生平台，保证 env_origin_z = 0）
        4 ~ 5.5m      按 slope_deg 上坡  -> 升 1.5m*tan(deg) ≈ 0.265m @10°
        5.5 ~ 6m      直接掉回 0        <- 断崖，冲上去会飞出去

    参数:
        slope_deg  上坡段坡度 [度]
        flat_frac  前多少比例保持水平（同样是 2/3，避开中心 2m 出生区）
        ramp_frac  上坡段占整块的比例（0.25 -> 1.5m）
    """
    slope = np.tan(np.radians(slope_deg))
    w, l = terrain.width, terrain.length
    yy = np.arange(l) * terrain.horizontal_scale
    y0 = l * terrain.horizontal_scale * flat_frac
    y1 = y0 + l * terrain.horizontal_scale * ramp_frac
    # ★ 乘 slope：这里 yy 是【位置】(米)，要换成高度必须乘坡度
    height_m = (np.clip(yy, y0, y1) - y0) * slope   # 平 -> 升到 (y1-y0)*tan(deg)
    height_m = np.where(yy > y1, 0.0, height_m)     # 顶端一刀切回 0 = 断崖
    hf = (height_m / terrain.vertical_scale).astype(np.int16)
    terrain.height_field_raw += np.broadcast_to(hf[None, :], (w, l))
    return terrain


def gap_terrain(terrain, gap_size, platform_size=1.0):
    gap_size = int(gap_size / terrain.horizontal_scale)
    platform_size = int(platform_size / terrain.horizontal_scale)

    center_x = terrain.length // 2
    center_y = terrain.width // 2
    x1 = (terrain.length - platform_size) // 2
    x2 = x1 + gap_size
    y1 = (terrain.width - platform_size) // 2
    y2 = y1 + gap_size

    terrain.height_field_raw[
        center_x - x2 : center_x + x2, center_y - y2 : center_y + y2
    ] = -1000
    terrain.height_field_raw[
        center_x - x1 : center_x + x1, center_y - y1 : center_y + y1
    ] = 0


def pit_terrain(terrain, depth, platform_size=1.0):
    depth = int(depth / terrain.vertical_scale)
    platform_size = int(platform_size / terrain.horizontal_scale / 2)
    x1 = terrain.length // 2 - platform_size
    x2 = terrain.length // 2 + platform_size
    y1 = terrain.width // 2 - platform_size
    y2 = terrain.width // 2 + platform_size
    terrain.height_field_raw[x1:x2, y1:y2] = -depth
