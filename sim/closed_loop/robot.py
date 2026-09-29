"""机器人真值运动与传感器模型。

运动：差速/全向运动学 + 一阶速度响应 + 加速度限制；在 2.5D 地形上行驶：
      z = 轮下地面高度均值，pitch/roll 由前后/左右轮高差得到；
      前方轮位高出当前地面超过可跨高度或坡度过陡时被挡住（记一次碰撞）。
误差：轮速里程计 = 轮子转速（打滑时车体实际位移小于轮速），打滑率随机游走；
      陀螺 wz 带零偏与噪声；IMU 俯仰/横滚带噪声；雷达深度噪声在 VirtualLidar 中。
"""
import math

import numpy as np

import world as W


class RobotSim:
    def __init__(self, scene, x=0.0, y=0.0, yaw=0.0, holonomic=False, seed=0,
                 slip_mean=0.03, slip_std=0.02, gyro_bias_dps=0.3, gyro_noise_dps=0.3,
                 att_noise_deg=0.3, scale_err=0.01):
        self.s = scene
        self.x, self.y, self.yaw = x, y, yaw
        self.v = self.vy = self.w = 0.0          # 轮速（m/s, m/s, rad/s）
        self.holonomic = holonomic
        self.rng = np.random.default_rng(seed)
        self.slip = slip_mean
        self.slip_mean, self.slip_std = slip_mean, slip_std
        self.gyro_bias = math.radians(gyro_bias_dps) * (1 if self.rng.random() > 0.5 else -1)
        self.gyro_noise = math.radians(gyro_noise_dps)
        self.att_noise = math.radians(att_noise_deg)
        self.scale = 1.0 + scale_err              # 轮径标定误差
        self.collisions = 0
        self.blocked = False
        self.z = self.pitch = self.roll = 0.0
        self.update_attitude()

    # ---------------------------------------------------------------- 地形
    def wheel_points(self, x, y, yaw):
        L, Wd = W.Robot.length / 2, W.Robot.width / 2
        c, s = math.cos(yaw), math.sin(yaw)
        pts = []
        for fx, fy in ((L, Wd), (L, -Wd), (-L, Wd), (-L, -Wd)):
            pts.append((x + fx * c - fy * s, y + fx * s + fy * c))
        return np.array(pts)

    def ground(self, x, y, yaw):
        p = self.wheel_points(x, y, yaw)
        return self.s.height(p[:, 0], p[:, 1])

    def update_attitude(self):
        h = self.ground(self.x, self.y, self.yaw)       # FL FR RL RR
        self.z = float(h.mean())
        front, rear = (h[0] + h[1]) / 2, (h[2] + h[3]) / 2
        left, right = (h[0] + h[2]) / 2, (h[1] + h[3]) / 2
        self.pitch = -math.atan2(front - rear, W.Robot.length)   # 抬头为负（绕 Y 正转为低头）
        self.roll = math.atan2(left - right, W.Robot.width)

    def traversable_to(self, nx, ny, nyaw):
        """新位置的四个轮位与当前地面高差不超过可跨高度；车体外接圆内无高于可跨高度的障碍"""
        h_new = self.ground(nx, ny, nyaw)
        h_old = self.ground(self.x, self.y, self.yaw)
        if np.any(h_new - h_old > W.Robot.step_climb) or np.any(h_old - h_new > 0.25):
            return False
        # 车身包络：外接圆上 16 个点高出地面太多 -> 撞上
        ang = np.linspace(0, 2 * math.pi, 16, endpoint=False)
        r = W.Robot.radius
        hb = self.s.height(nx + r * np.cos(ang), ny + r * np.sin(ang))
        return not np.any(hb - h_new.mean() > W.Robot.step_climb + 0.05)

    # ---------------------------------------------------------------- 运动
    def step(self, cmd, dt):
        """cmd: (vx mm/s, vy mm/s, wz mrad/s)"""
        tv, tvy, tw = cmd[0] / 1000.0, (cmd[1] / 1000.0 if self.holonomic else 0.0), cmd[2] / 1000.0
        a = dt / 0.12
        self.v += max(-1.5 * dt, min(1.5 * dt, a * (tv - self.v)))
        self.vy += max(-1.5 * dt, min(1.5 * dt, a * (tvy - self.vy)))
        self.w += max(-6 * dt, min(6 * dt, a * (tw - self.w)))
        # 打滑率随机游走
        self.slip = float(np.clip(self.slip + self.rng.normal(0, self.slip_std) * math.sqrt(dt) * 3,
                                  0.0, self.slip_mean * 3 + 0.05))
        eff = (1 - self.slip)
        ny_ = self.yaw + self.w * dt * (1 - 0.5 * self.slip)
        cp = math.cos(getattr(self, "ground_pitch", self.pitch))     # 水平分量按地面坡度（自平衡车机身俯仰 ≠ 坡度）
        dxb, dyb = self.v * eff * cp * dt, self.vy * eff * dt
        nx = self.x + dxb * math.cos(self.yaw) - dyb * math.sin(self.yaw)
        ny = self.y + dxb * math.sin(self.yaw) + dyb * math.cos(self.yaw)
        yaw_before = self.yaw
        if self.traversable_to(nx, ny, ny_):
            self.x, self.y, self.yaw = nx, ny, ny_
            self.blocked = False
        else:
            if not self.blocked:
                self.collisions += 1
                if not hasattr(self, "coll_log"):
                    self.coll_log = []
                self.coll_log.append((round(self.x, 2), round(self.y, 2), round(math.degrees(self.yaw)), round(self.v, 2)))
            self.blocked = True
            # 受阻：电机堵转保护使轮速归零（否则里程计会把空转当成位移）
            self.v = self.vy = 0.0
            if self.traversable_to(self.x, self.y, ny_):
                self.yaw = ny_
        self.w_true = (self.yaw - yaw_before) / dt       # 陀螺测的是车体真实转速
        self.update_attitude()

    # ---------------------------------------------------------------- 传感器
    def odom(self):
        """轮速里程计（轮子线速度 × 标定误差），陀螺角速度（零偏 + 噪声）。返回 mm/s, mm/s, mrad/s"""
        vx = self.v * self.scale
        vy = self.vy * self.scale
        wz = getattr(self, "w_true", 0.0) + self.gyro_bias + self.rng.normal(0, self.gyro_noise)
        return int(round(vx * 1000)), int(round(vy * 1000)), int(round(wz * 1000))

    def imu_wit_frame(self):
        """0x55 0x53 角度帧（roll, pitch, yaw：×32768/180 度），与 fw/nav/imu_parse.c 对应"""
        def enc(rad):
            v = int(round(math.degrees(rad) / 180 * 32768))
            v = max(-32768, min(32767, v))
            return v & 0xFFFF
        r = self.roll + self.rng.normal(0, self.att_noise)
        # 固件俯仰约定：pitch>0 为低头（绕 Y 正转），与本模型一致
        p = self.pitch + self.rng.normal(0, self.att_noise)
        y = self.yaw
        b = [0x55, 0x53]
        for v in (enc(r), enc(p), enc(y)):
            b += [v & 0xFF, v >> 8]
        b += [0, 0]
        b.append(sum(b) & 0xFF)
        return bytes(b)

    def imu_wit_gyro_frame(self, dt):
        """0x55 0x52 角速度帧（wx, wy, wz：×32768/2000 °/s）：由真实姿态差分 + 零偏 + 噪声"""
        pr = getattr(self, "_imu_prev", (self.pitch, self.roll))
        self._imu_prev = (self.pitch, self.roll)
        if not hasattr(self, "_gb"):
            self._gb = [math.radians(0.1) * (1 if self.rng.random() > 0.5 else -1) for _ in range(2)]
        wy = (self.pitch - pr[0]) / dt + self._gb[0] + self.rng.normal(0, math.radians(0.3))
        wx = (self.roll - pr[1]) / dt + self._gb[1] + self.rng.normal(0, math.radians(0.3))
        wz = getattr(self, "w_true", 0.0)

        def enc(rad_s):
            v = int(round(math.degrees(rad_s) / 2000 * 32768))
            return max(-32768, min(32767, v)) & 0xFFFF
        b = [0x55, 0x52]
        for v in (enc(wx), enc(wy), enc(wz)):
            b += [v & 0xFF, v >> 8]
        b += [0, 0]
        b.append(sum(b) & 0xFF)
        return bytes(b)

    def body_pose(self):
        """机身系 -> 世界系（R, t），姿态 Rz(yaw)·Ry(pitch)·Rx(roll)"""
        import sim_world as sw
        R = sw.rotz(self.yaw) @ sw.roty(self.pitch) @ sw.rotx(self.roll)
        return R, np.array([self.x, self.y, self.z])


class BalanceBot(RobotSim):
    """两轮自平衡轮腿：左右两个着地点；机身俯仰不随坡面，而是由平衡控制决定——
    加减速时前后倾（tan θ ≈ a/g）+ 持续的小幅自激摆动 + 越过台阶/坡口后的衰减振荡；横滚由左右轮高差决定。
    osc_deg / osc_hz：持续摆动幅度与频率；kick_deg_per_cm：越过 1 cm 高差激起的摆动幅度。"""

    def __init__(self, scene, *args, osc_deg=1.5, osc_hz=1.8, kick_deg_per_cm=3.0, damp=0.35, f0=1.6, **kw):
        self.osc = math.radians(osc_deg)
        self.osc_w = 2 * math.pi * osc_hz
        self.kick = math.radians(kick_deg_per_cm) * 100.0      # rad / m 高差
        self.zeta, self.w0 = damp, 2 * math.pi * f0
        self.th_d = self.th_dd = 0.0                          # 振荡状态（相对平衡姿态）
        self.t_sim = 0.0
        self.v_prev = 0.0
        self.lean = 0.0
        self.phase = 0.0
        super().__init__(scene, *args, **kw)
        self.phase = self.rng.uniform(0, 2 * math.pi)

    def wheel_points(self, x, y, yaw):
        Wd = W.Robot.width / 2
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([(x - Wd * s, y + Wd * c), (x + Wd * s, y - Wd * c)])     # 左、右

    def update_attitude(self):
        h = self.ground(self.x, self.y, self.yaw)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        hf, hr = self.s.height(np.array([self.x + 0.05 * c, self.x - 0.05 * c]), np.array([self.y + 0.05 * s, self.y - 0.05 * s]))
        self.ground_pitch = -math.atan2(float(hf - hr), 0.10)
        z_new = float(h.mean())
        dz = z_new - getattr(self, "z", z_new)
        self.z = z_new
        self.roll = math.atan2(h[0] - h[1], W.Robot.width)
        # 高差突变（台阶、坡口）给振荡器一个冲量
        if abs(dz) > 0.004:
            self.th_d += self.kick * dz * (1 if self.v >= 0 else -1) * 8.0
        self.pitch = self.lean + self.th_dd + self.osc * math.sin(self.osc_w * self.t_sim + self.phase)

    def step(self, cmd, dt):
        super().step(cmd, dt)
        self.t_sim += dt
        # 前后倾：加速时低头（pitch > 0），一阶跟随
        acc = (self.v - self.v_prev) / dt
        self.v_prev = self.v
        self.lean += (math.atan2(acc, 9.81) - self.lean) * min(1.0, dt / 0.15)
        # 二阶衰减振荡 θ'' = -2ζω θ' - ω² θ
        a = -2 * self.zeta * self.w0 * self.th_d - self.w0 ** 2 * self.th_dd
        self.th_d += a * dt
        self.th_dd += self.th_d * dt
        self.pitch = self.lean + self.th_dd + self.osc * math.sin(self.osc_w * self.t_sim + self.phase)

    def traversable_to(self, nx, ny, nyaw):
        h_new = self.ground(nx, ny, nyaw)
        h_old = self.ground(self.x, self.y, self.yaw)
        # 轮子前方着地点（半个轮径）上的台阶
        c, s = math.cos(nyaw), math.sin(nyaw)
        sgn = 1 if self.v >= 0 else -1
        pf = self.wheel_points(nx + sgn * 0.03 * c, ny + sgn * 0.03 * s, nyaw)
        hf = self.s.height(pf[:, 0], pf[:, 1])
        if np.any(hf - h_old > W.Robot.step_climb) or np.any(h_new - h_old > W.Robot.step_climb) \
                or np.any(h_old - h_new > 0.10):
            return False
        ang = np.linspace(0, 2 * math.pi, 16, endpoint=False)
        r = W.Robot.radius
        hb = self.s.height(nx + r * np.cos(ang), ny + r * np.sin(ang))
        return not np.any(hb - h_new.mean() > W.Robot.step_climb + 0.05)
