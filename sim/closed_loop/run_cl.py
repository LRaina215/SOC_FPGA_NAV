"""闭环仿真：场景 -> 虚拟 XT-M60 -> FPGA 感知模型（与 RTL 逐位一致）-> 固件（真实 C 代码，动态库）
-> 速度指令 -> 机器人（打滑/陀螺零偏/姿态噪声）-> 下一帧。

用法:
  python sim/closed_loop/run_cl.py --scene terrain --goal 9.5,2.8 --trav v2 --match on --out sim/build/cl/run1
  python sim/closed_loop/run_cl.py --scene loop --waypoints 7,-2.5,7,2.5,-0.5,2.5,-0.5,-2.5 --laps 2
  python sim/closed_loop/run_cl.py --scene terrain --search 9.3,-3.3
输出：metrics.json、snapshot_*.png（真值标签/代价图/高程/轨迹）
"""
import argparse
import ctypes
import json
import math
import struct
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "model"))
sys.path.insert(0, str(ROOT / "tools"))

import fpga_model  # noqa: E402
import nav_fixed as nf  # noqa: E402
import robot as rb  # noqa: E402
import sim_world as sw  # noqa: E402
import world as W  # noqa: E402
import rk_host  # noqa: E402

DLL = ROOT / "sim" / "build" / "closed_loop" / "nav_fw.dll"
INV_RES = 1311
RES_MM = 65536 / INV_RES


def build_dll():
    import subprocess
    F = ROOT / "fw"
    srcs = [str(p) for p in (F / "nav").glob("*.c")] + [str(F / "app" / "nav_app.c"), str(F / "test" / "hal_py.c")]
    DLL.parent.mkdir(parents=True, exist_ok=True)
    cmd = ["gcc", "-std=c99", "-O2", "-shared", "-o", str(DLL)] + \
          [f"-I{F / d}" for d in ("include", "hal", "nav", "app", "test")] + srcs
    subprocess.run(cmd, check=True)


MATCH_CB = ctypes.CFUNCTYPE(None, ctypes.POINTER(ctypes.c_int32), ctypes.c_int32, ctypes.c_int32,
                            ctypes.POINTER(ctypes.c_int32), ctypes.c_int, ctypes.POINTER(ctypes.c_uint32))


class Firmware:
    def __init__(self):
        self.d = ctypes.CDLL(str(DLL))
        self.d.py_init()

    def __getattr__(self, n):
        return getattr(self.d, n)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default="terrain")
    ap.add_argument("--start", default="-1.0,-0.3,0")
    ap.add_argument("--goal")
    ap.add_argument("--waypoints")
    ap.add_argument("--laps", type=int, default=1)
    ap.add_argument("--search")
    ap.add_argument("--trav", default="v3")
    ap.add_argument("--match", default="off")
    ap.add_argument("--time", type=float, default=90.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--slip", type=float, default=0.03)
    ap.add_argument("--out", default=str(ROOT / "sim" / "build" / "cl" / "run"))
    ap.add_argument("--snap", type=float, default=0, help="每隔多少秒存一张快照（0 = 只存结束时）")
    ap.add_argument("--gif", action="store_true", help="把快照合成动画 run.gif（未指定 --snap 时按 1 s 一帧）")
    ap.add_argument("--no-build", action="store_true")
    ap.add_argument("--anchor", type=int, default=64)
    ap.add_argument("--tcfg", default="", help="覆盖地形参数，如 keep_r=0,age_max=100")
    ap.add_argument("--gmap", default="on", help="A* 用全局记忆栅格（on）还是局部代价图（off）")
    ap.add_argument("--noise", default="", help="只开启部分误差源，逗号分隔：slip,gyro,att,scale")
    ap.add_argument("--ideal", action="store_true", help="理想传感器（无打滑/零偏/噪声），用于排查")
    ap.add_argument("--calib", type=float, default=2.0, help="任务下发前静止标定时间 s（陀螺零偏）")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    if a.gif and a.snap <= 0:
        a.snap = 1.0
    for old in out.glob("snapshot_*.png"):
        old.unlink()
    if not a.no_build:
        build_dll()

    scene = W.SCENES[a.scene]()
    sx, sy, syaw = (float(v) for v in a.start.split(","))
    if a.ideal:
        bot = rb.RobotSim(scene, sx, sy, math.radians(syaw), seed=a.seed, slip_mean=0, slip_std=0,
                          gyro_bias_dps=0, gyro_noise_dps=0, att_noise_deg=0, scale_err=0)
    elif a.noise:
        on = set(a.noise.split(","))
        bot = rb.RobotSim(scene, sx, sy, math.radians(syaw), seed=a.seed,
                          slip_mean=a.slip if "slip" in on else 0, slip_std=0.02 if "slip" in on else 0,
                          gyro_bias_dps=0.3 if "gyro" in on else 0, gyro_noise_dps=0.3 if "gyro" in on else 0,
                          att_noise_deg=0.3 if "att" in on else 0, scale_err=0.01 if "scale" in on else 0)
    else:
        bot = rb.RobotSim(scene, sx, sy, math.radians(syaw), seed=a.seed, slip_mean=a.slip)
    lid = sw.VirtualLidar(noise_mm=4.0, seed=a.seed + 7, rng_max=8.0, step=0.015)
    Rbs, tbs = sw.mount_pose(height=0.20, pitch_down_deg=10.0, x=0.10)
    fpga = fpga_model.FpgaModel(trav=a.trav)
    for kv in filter(None, a.tcfg.split(",")):
        k, v = kv.split("=")
        fpga.cfg[k] = int(v)
    fw = Firmware()
    fw.py_set_match_enable(1 if a.match == "on" else 0)
    fw.py_set_caps(0, 500, 1200, 250)
    fw.py_set_anchor(a.anchor)
    fw.py_set_gmap_enable(1 if a.gmap == "on" else 0)

    # 里程计系 = 世界系平移 (sx, sy)、旋转 syaw
    c0, s0 = math.cos(math.radians(syaw)), math.sin(math.radians(syaw))

    def w2o(x, y):
        dx, dy = x - sx, y - sy
        return c0 * dx + s0 * dy, -s0 * dx + c0 * dy

    def o2w(x, y):
        return sx + c0 * x - s0 * y, sy + s0 * x + c0 * y

    @MATCH_CB
    def match_cb(rt, cxm, cym, cand, k, res):
        r = [rt[i] for i in range(12)]
        cs = np.array([[cand[3 * i], cand[3 * i + 1], cand[3 * i + 2]] for i in range(k)], dtype=np.int64)
        o = fpga.match_eval(r, cxm, cym, cs)
        for i in range(k):
            for j in range(4):
                res[4 * i + j] = int(o[i, j]) & 0xFFFFFFFF
    fw.py_set_match_cb(match_cb)
    KF_CB = ctypes.CFUNCTYPE(None, ctypes.c_int32)

    @KF_CB
    def kf_cb(z):
        fpga.keyframe(z)
    fw.py_set_kf_cb(kf_cb)
    H_CB = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_int32, ctypes.c_int32, ctypes.POINTER(ctypes.c_int32))

    @H_CB
    def h_cb(x, y, hp):
        ix = int((x * INV_RES) >> 16); iy = int((y * INV_RES) >> 16)
        wx, wy = ix - (fpga.cx - 64), iy - (fpga.cy - 64)
        if not (0 <= wx < 128 and 0 <= wy < 128):
            return 0
        a_ = ((iy & 127) << 7) | (ix & 127)
        em = fpga.em
        tag = (((iy >> 7) & 15) << 4) | ((ix >> 7) & 15)
        if em.cnt[a_] == 0 or em.tag[a_] != tag or em.zmax[a_] - em.zmin[a_] > 40:
            return 0
        hp[0] = int(em.zmax[a_] + em.zmin[a_]) // 2      # 区间中点：zmax 偏高（噪声取最大），用它锚定会正反馈地往上漂
        return 1
    fw.py_set_h_cb(h_cb)

    # 任务
    def send(b):
        buf = (ctypes.c_uint8 * len(b)).from_buffer_copy(b)
        fw.py_push_host(buf, len(b), ctypes.c_uint32(now_us))
    now_us = 1000
    fw.py_set_time(now_us)
    goals_w = []
    pending = []                      # 静止标定结束后再下发任务
    if a.goal:
        gx, gy = (float(v) for v in a.goal.split(","))
        goals_w = [(gx, gy)]
        ox, oy = w2o(gx, gy)
        pending.append(rk_host.goal(int(ox * 1000), int(oy * 1000)))
    elif a.waypoints:
        v = [float(t) for t in a.waypoints.split(",")]
        goals_w = list(zip(v[0::2], v[1::2])) * a.laps
        pending.append(rk_host.waypoints([tuple(int(c * 1000) for c in w2o(*g)) for g in goals_w]))
    elif a.search:
        tx, ty = (float(t) for t in a.search.split(","))
        pending.append(rk_host.mode(3))

    # 真值标签（里程计栅格对齐）
    x0, y0, x1, y1 = scene.bounds

    # 统计
    traj_true, traj_est, loc_err = [], [], []
    tlog = []                                    # 逐 0.1 s：t, 真值 x y z yaw, 估计 x y z yaw, 指令 vx wz, 受阻
    conf = np.zeros((2, 2), dtype=np.int64)      # [真值 可通行/不可][预测 可通行/致命]
    n_obs_free = 0
    match_log = []
    dt_ctrl = 0.001
    t_frame = 0.0
    t_odo = 0.0
    t_imu = 0.0
    t = 0.0
    arrived_t = None
    found_t = None
    snap_i = 0
    t_snap = a.snap if a.snap > 0 else 1e9
    wall0 = time.time()
    st = (ctypes.c_int32 * 12)()
    cmdv = (ctypes.c_int16 * 4)()
    commit_v = (ctypes.c_int32 * 16)()

    def local_o2w():
        """里程计系 -> 世界系，按"当前估计位姿 ↔ 当前真值位姿"对齐：局部地图是用估计位姿建的，
        用它评估地形判定可以排除全局漂移的影响（漂移另由定位误差指标衡量）"""
        fw.py_get_state(st)
        ex, ey, eyaw = st[0] / 1000, st[1] / 1000, (st[3] & 0xFFFF) / 65536 * 2 * math.pi
        dyaw = bot.yaw - eyaw
        cd, sd = math.cos(dyaw), math.sin(dyaw)

        def f(ox, oy):
            dx, dy = ox - ex, oy - ey
            return bot.x + cd * dx - sd * dy, bot.y + sd * dx + cd * dy
        return f

    def snapshot(tag):
        save_snapshot(out / f"snapshot_{tag}.png", scene, fpga, traj_true, traj_est, goals_w, local_o2w(), w2o, a,
                      get_gmap(fw), o2w)

    ms = 0
    n_frames = 0
    frame_pending = False
    while t < a.time:
        if pending and t >= a.calib:
            for b in pending:
                send(b)
            pending = []
        # 雷达帧：上一帧已提交（FPGA 双缓冲握手）才采集下一帧
        if t >= t_frame and not frame_pending:
            t_frame += 1.0 / a.fps
            Rwb, twb = bot.body_pose()
            Rws, tws = sw.compose(Rwb, twb, Rbs, tbs)
            depth, _ = lid.capture(scene.w, Rws, tws)
            fpga.frame(depth)
            fw.py_set_lidar_waiting(ctypes.c_uint32(now_us))
            frame_pending = True
        if frame_pending:
            fw.py_poll()
            if fw.py_get_commit(commit_v):
                frame_pending = False
                v = list(commit_v)
                fpga.commit(v[:12], v[12], v[13], v[14], v[15])
                slot = (ctypes.c_uint8 * (128 * 128)).from_buffer_copy(np.clip(fpga.cost, 0, 255).astype(np.uint8).tobytes())
                fw.py_set_map(slot, v[12], v[13], ctypes.c_uint16(fpga.seq), ctypes.c_uint16(INV_RES))
                mv = (ctypes.c_int32 * 12)()
                fw.py_get_match(mv)
                match_log.append(list(mv))
                n_frames += 1
                if n_frames % 5 == 0:
                    stat_confusion(scene, fpga, local_o2w(), conf, t)
        # 里程计 50 Hz、IMU 50 Hz
        if t >= t_odo:
            t_odo += 0.02
            vx, vy, wz = bot.odom()
            fw.py_push_odom(vx, vy, wz, ctypes.c_uint32(now_us))
        if t >= t_imu:
            t_imu += 0.02
            b = bot.imu_wit_frame()
            buf = (ctypes.c_uint8 * len(b)).from_buffer_copy(b)
            fw.py_push_imu(buf, len(b), ctypes.c_uint32(now_us))
        # 模拟 RK3588：目标可见时发 TARGET
        if a.search and ms % 100 == 0:
            vis, bx, by = target_visible(bot, scene, tx, ty)
            if vis:
                send(rk_host.target(int(bx * 1000), int(by * 1000), 0))
                if found_t is None:
                    found_t = t
        fw.py_poll()
        fw.py_get_cmd(cmdv)
        bot.step((cmdv[0], cmdv[1], cmdv[2]), dt_ctrl)
        ms += 1
        t = ms * dt_ctrl
        now_us += 1000
        fw.py_set_time(ctypes.c_uint32(now_us))
        if ms % 100 == 0:
            fw.py_get_state(st)
            ex, ey = o2w(st[0] / 1000, st[1] / 1000)
            traj_true.append((bot.x, bot.y, bot.z))
            traj_est.append((ex, ey, st[2] / 1000))
            yaw_err = math.degrees(math.remainder(((st[3] & 0xFFFF) / 65536 * 2 * math.pi) + math.radians(syaw) - bot.yaw, 2 * math.pi))
            loc_err.append((math.hypot(ex - bot.x, ey - bot.y), abs(st[2] / 1000 - bot.z), abs(yaw_err)))
            tlog.append((t, bot.x, bot.y, bot.z, math.degrees(bot.yaw), ex, ey, st[2] / 1000,
                         math.degrees(bot.yaw) + yaw_err, cmdv[0], cmdv[2], int(bot.blocked),
                         math.degrees(bot.pitch), st[4] * 360 / 65536, math.degrees(bot.roll), st[5] * 360 / 65536))
            if a.goal or a.waypoints:
                if st[7] == 2 and arrived_t is None:
                    arrived_t = t
                    break
            if a.search and found_t is not None and st[6] == 2 and math.hypot(bot.x - tx, bot.y - ty) < 1.0 \
                    and abs(cmdv[0]) < 10:
                arrived_t = t
                break
        if t >= t_snap:
            snapshot(f"{snap_i:03d}_t{t:05.1f}s")
            snap_i += 1
            t_snap += a.snap
    snapshot("final")
    if fpga.view is not None:
        _, V_, H_, RO_ = fpga.view
        gm_ = get_gmap(fw)
        fw.py_get_state(st)
        np.savez(out / "final_maps.npz", cost=cost_window(fpga), lethal=fpga.last_lethal, V=V_, H=H_, RO=RO_,
                 cx=fpga.cx, cy=fpga.cy, gmap=gm_[0], gm_o=np.array(gm_[1:4]), path=gm_[4], state=np.array(list(st)),
                 true=np.array([bot.x, bot.y, bot.z, bot.yaw]), fp_pts=np.array(ERR_PTS["fp"]).reshape(-1, 3),
                 fn_pts=np.array(ERR_PTS["fn"]).reshape(-1, 3), match_log=np.array(match_log), tlog=np.array(tlog))
    if a.gif:
        from PIL import Image
        frames = [Image.open(p) for p in sorted(out.glob("snapshot_[0-9]*.png"))] + [Image.open(out / "snapshot_final.png")]
        frames[0].save(out / "run.gif", save_all=True, append_images=frames[1:], duration=250, loop=0)

    save_err_plot(out / "trav_errors.png", scene, ERR_PTS, traj_true)
    if tlog:
        tl = np.array(tlog)
        np.savetxt(out / "timeseries.csv", tl, delimiter=",", fmt="%.4f",
                   header="t,x,y,z,yaw_deg,ex,ey,ez,eyaw_deg,cmd_vx,cmd_wz,blocked,pitch,epitch,roll,eroll", comments="")
        save_timeseries_plot(out / "loc_error.png", tl)
    le = np.array(loc_err) if loc_err else np.zeros((1, 3))
    ml = np.array(match_log) if match_log else np.zeros((1, 12))
    tp, fn, fp, tn = int(conf[1, 1]), int(conf[1, 0]), int(conf[0, 1]), int(conf[0, 0])
    m = {
        "scene": a.scene, "trav": a.trav, "match": a.match, "seed": a.seed, "slip": a.slip,
        "arrived": arrived_t is not None, "time_s": round(arrived_t if arrived_t else t, 1),
        "found_s": found_t,
        "collisions": bot.collisions,
        "coll_first": getattr(bot, "coll_log", [])[:5],
        "final_true": [round(bot.x, 3), round(bot.y, 3)],
        "goal_err_m": round(math.hypot(bot.x - goals_w[-1][0], bot.y - goals_w[-1][1]), 3) if goals_w else None,
        "loc_err_xy_rms_m": round(float(np.sqrt((le[:, 0] ** 2).mean())), 3),
        "loc_err_xy_final_m": round(float(le[-1, 0]), 3),
        "loc_err_xy_max_m": round(float(le[:, 0].max()), 3),
        "loc_err_z_max_m": round(float(le[:, 1].max()), 3),
        "loc_err_yaw_final_deg": round(float(le[-1, 2]), 2),
        "match_applied": int(ml[-1, 11]),
        "keyframes": int(fw.py_get_nkf()),
        "trav_lethal_recall": round(tp / max(1, tp + fn), 3),
        "trav_false_lethal_rate": round(fp / max(1, fp + tn), 4),
        "trav_cells": {"tp": tp, "fn": fn, "fp": fp, "tn": tn},
        "wall_s": round(time.time() - wall0, 1),
    }
    (out / "metrics.json").write_text(json.dumps(m, ensure_ascii=False, indent=1), encoding="utf-8")
    print(json.dumps(m, ensure_ascii=False))
    return 0


def save_err_plot(path, scene, pts, traj_true):
    """整个运行中地形判定错误的位置（世界系）：橙 = 误报致命，红 = 漏报；颜色深浅 = 时间"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
    fig, ax = plt.subplots(figsize=(12, 8))
    x0, y0, x1, y1 = scene.bounds
    X, Y = np.meshgrid(np.arange(x0, x1, 0.02), np.arange(y0, y1, 0.02))
    ax.imshow(scene.height(X, Y), origin="lower", extent=(x0, x1, y0, y1), cmap="gray_r", vmin=-0.3, vmax=0.8)
    for k, cm in (("fp", "Oranges"), ("fn", "Reds")):
        if pts[k]:
            p = np.array(pts[k])
            ax.scatter(p[:, 0], p[:, 1], s=3, c=p[:, 2], cmap=cm, vmin=-20, vmax=max(1, p[:, 2].max()))
    if traj_true:
        tt = np.array(traj_true)
        ax.plot(tt[:, 0], tt[:, 1], "b-", lw=1)
    ax.set_title(f"地形判定错误位置：橙=误报 {len(pts['fp'])}，红=漏报 {len(pts['fn'])}（颜色越深越晚）")
    ax.set_aspect("equal")
    fig.savefig(path, dpi=80)
    plt.close(fig)


def save_timeseries_plot(path, tl):
    """定位误差逐帧曲线：xy 误差分解到车体前向/侧向、航向误差、高度（真值与估计）、指令"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
    plt.rcParams["axes.unicode_minus"] = False
    t = tl[:, 0]
    dx, dy = tl[:, 5] - tl[:, 1], tl[:, 6] - tl[:, 2]
    yw = np.radians(tl[:, 4])
    along = dx * np.cos(yw) + dy * np.sin(yw)
    cross = -dx * np.sin(yw) + dy * np.cos(yw)
    fig, ax = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    ax[0].plot(t, np.hypot(dx, dy), "k", label="|xy 误差|")
    ax[0].plot(t, along, "b", lw=0.8, label="前向误差")
    ax[0].plot(t, cross, "r", lw=0.8, label="侧向误差")
    ax[0].set_ylabel("m"); ax[0].legend(fontsize=8); ax[0].grid(alpha=.3)
    ax[1].plot(t, (tl[:, 8] - tl[:, 4] + 180) % 360 - 180, "m")
    ax[1].set_ylabel("航向误差 °"); ax[1].grid(alpha=.3)
    ax[2].plot(t, tl[:, 3], "r", label="真值 z"); ax[2].plot(t, tl[:, 7], "b--", label="估计 z")
    ax[2].set_ylabel("m"); ax[2].legend(fontsize=8); ax[2].grid(alpha=.3)
    ax[3].plot(t, tl[:, 9], "g", label="vx 指令 mm/s"); ax[3].plot(t, tl[:, 10], "c", label="wz 指令 mrad/s")
    ax[3].fill_between(t, 0, tl[:, 11] * 500, color="r", alpha=.2, label="受阻")
    ax[3].legend(fontsize=8); ax[3].grid(alpha=.3); ax[3].set_xlabel("t / s")
    plt.tight_layout()
    fig.savefig(path, dpi=80)
    plt.close(fig)


def target_visible(bot, scene, tx, ty, fov=math.radians(70), rng=4.0):
    dx, dy = tx - bot.x, ty - bot.y
    d = math.hypot(dx, dy)
    ang = math.remainder(math.atan2(dy, dx) - bot.yaw, 2 * math.pi)
    if d > rng or abs(ang) > fov / 2:
        return False, 0, 0
    # 视线：沿途地面高于相机高度（0.35 m）则遮挡
    for k in range(1, 40):
        px, py = bot.x + dx * k / 40, bot.y + dy * k / 40
        if float(scene.height(np.array([px]), np.array([py]))[0]) - bot.z > 0.35:
            return False, 0, 0
    c, s = math.cos(bot.yaw), math.sin(bot.yaw)
    return True, c * dx + s * dy, -s * dx + c * dy


ERR_PTS = {"fp": [], "fn": []}


def stat_confusion(scene, fpga, o2w, conf, t=0.0):
    """当前窗口内：已观测（代价 != 255）栅格的预测（致命 = 254）与真值标签比较"""
    N = nf.N
    cw = cost_window(fpga)
    ix = np.arange(N) + fpga.cx - N // 2
    iy = np.arange(N) + fpga.cy - N // 2
    ox = (ix + 0.5) * RES_MM / 1000
    oy = (iy + 0.5) * RES_MM / 1000
    OX, OY = np.meshgrid(ox, oy)
    WX, WY = o2w(OX, OY)
    x0, y0, x1, y1 = scene.bounds
    inb = (WX > x0) & (WX < x1) & (WY > y0) & (WY < y1)
    obs = (cw != 255) & inb
    if not obs.any():
        return
    lab = np.full(cw.shape, 2, dtype=np.int8)
    lab[obs] = scene.gt_lookup(WX[obs], WY[obs])
    use = obs & (lab != 2)
    pred_leth = cw >= 254            # 误报：真可通行判致命
    pred_block = cw >= 253           # 漏报：真致命判可走（膨胀区对机器人中心同样不可走）
    conf[0, 0] += int((use & (lab == 0) & ~pred_leth).sum())
    conf[0, 1] += int((use & (lab == 0) & pred_leth).sum())
    conf[1, 0] += int((use & (lab == 1) & ~pred_block).sum())
    conf[1, 1] += int((use & (lab == 1) & pred_block).sum())
    fp = use & (lab == 0) & pred_leth
    fn = use & (lab == 1) & ~pred_block
    ERR_PTS["fp"] += [(x, y, t) for x, y in zip(WX[fp], WY[fp])]
    ERR_PTS["fn"] += [(x, y, t) for x, y in zip(WX[fn], WY[fn])]


def cost_window(fpga):
    import nav_fast as fast
    return fast.cost_window(fpga.cost, fpga.cx, fpga.cy)


def get_gmap(fw):
    v = (ctypes.c_int32 * 4)()
    grid = (ctypes.c_uint8 * (256 * 256))()
    path = (ctypes.c_int16 * 512)()
    n = fw.py_get_gmap(grid, v, path)
    gn = v[2]
    g = np.frombuffer(bytes(grid), dtype=np.uint8)[:gn * gn].reshape(gn, gn)
    return g, v[0], v[1], v[3], np.array(path[:2 * n]).reshape(-1, 2)


def save_snapshot(path, scene, fpga, traj_true, traj_est, goals, o2w, w2o, a, gm=None, o2w_est=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
    plt.rcParams["axes.unicode_minus"] = False
    N = nf.N
    fig, ax = plt.subplots(1, 4, figsize=(22, 5.6))
    x0, y0, x1, y1 = scene.bounds
    # 1. 场景高程 + 轨迹
    xs = np.arange(x0, x1, 0.02)
    ys = np.arange(y0, y1, 0.02)
    X, Y = np.meshgrid(xs, ys)
    Hs = scene.height(X, Y)
    ax[0].imshow(Hs, origin="lower", extent=(x0, x1, y0, y1), cmap="terrain", vmin=-0.1, vmax=0.6)
    if traj_true:
        tt = np.array(traj_true)
        te = np.array(traj_est)
        ax[0].plot(tt[:, 0], tt[:, 1], "r-", lw=1.5, label="真值")
        ax[0].plot(te[:, 0], te[:, 1], "b--", lw=1.2, label="估计")
    for g in goals:
        ax[0].plot(g[0], g[1], "k*", ms=12)
    ax[0].set_title(f"场景高程与轨迹（{a.scene}，t={len(traj_true) / 10:.1f}s）红=真值 蓝=估计")
    ax[0].legend(loc="upper left", fontsize=8)
    # 2. 代价图（窗口，里程计系 -> 世界系坐标轴）
    cw = cost_window(fpga).astype(float)
    ox0 = (fpga.cx - N // 2) * RES_MM / 1000
    oy0 = (fpga.cy - N // 2) * RES_MM / 1000
    wx0, wy0 = o2w(ox0, oy0)
    img = np.where(cw == 255, np.nan, cw)
    ax[1].imshow(img, origin="lower", extent=(wx0, wx0 + N * RES_MM / 1000, wy0, wy0 + N * RES_MM / 1000),
                 cmap="magma_r", vmin=0, vmax=254)
    if traj_true:
        ax[1].plot(tt[:, 0], tt[:, 1], "c-", lw=1)
    ax[1].set_xlim(x0, x1)
    ax[1].set_ylim(y0, y1)
    ax[1].set_title(f"FPGA 代价图（trav {a.trav}，白=未知）")
    # 3. 真值标签 vs 预测（窗口内）
    ix = np.arange(N) + fpga.cx - N // 2
    iy = np.arange(N) + fpga.cy - N // 2
    OX, OY = np.meshgrid((ix + 0.5) * RES_MM / 1000, (iy + 0.5) * RES_MM / 1000)
    WX, WY = o2w(OX, OY)
    inb = (WX > x0) & (WX < x1) & (WY > y0) & (WY < y1)
    lab = np.full(cw.shape, 2, dtype=np.int8)
    lab[inb] = scene.gt_lookup(WX[inb], WY[inb])
    obs = cw != 255
    cmp = np.full(cw.shape, np.nan)
    cmp[obs & (lab == 0) & (cw < 254)] = 0     # 正确可通行
    cmp[obs & (lab == 1) & (cw >= 254)] = 1    # 正确致命
    cmp[obs & (lab == 0) & (cw >= 254)] = 2    # 误报致命
    cmp[obs & (lab == 1) & (cw == 253)] = 1    # 真致命落在膨胀区：同样不可走
    cmp[obs & (lab == 1) & (cw < 253)] = 3     # 漏报
    from matplotlib.colors import ListedColormap
    ax[2].imshow(cmp, origin="lower", extent=(wx0, wx0 + N * RES_MM / 1000, wy0, wy0 + N * RES_MM / 1000),
                 cmap=ListedColormap(["#9be39b", "#444444", "#ff9900", "#ff0000"]), vmin=0, vmax=3)
    ax[2].set_xlim(x0, x1)
    ax[2].set_ylim(y0, y1)
    ax[2].set_title("判定对比：绿=对可通行 灰=对致命 橙=误报 红=漏报")
    # 4. 全局记忆栅格与 A* 路径（里程计系，按初始位姿换到世界系显示；不做漂移对齐）
    if gm is not None:
        g, gox, goy, ds, pth = gm
        cell = ds * RES_MM / 1000
        gx0, gy0 = o2w_est(gox * cell, goy * cell)
        img = np.where(g == 255, np.nan, np.where(g >= 254, 254.0, g.astype(float)))
        ax[3].imshow(img, origin="lower", extent=(gx0, gx0 + g.shape[1] * cell, gy0, gy0 + g.shape[0] * cell),
                     cmap="magma_r", vmin=0, vmax=254)
        if len(pth):
            px, py = o2w_est((pth[:, 0] + gox + 0.5) * cell, (pth[:, 1] + goy + 0.5) * cell)
            ax[3].plot(px, py, "g.-", lw=1, ms=3)
        if traj_est:
            ax[3].plot(te[:, 0], te[:, 1], "b--", lw=1)
        for gg in goals:
            ax[3].plot(gg[0], gg[1], "k*", ms=12)
        ax[3].set_xlim(x0, x1)
        ax[3].set_ylim(y0, y1)
        ax[3].set_title("全局记忆栅格 + A* 路径（估计位姿系）")
    for x in ax:
        x.set_aspect("equal")
    plt.tight_layout()
    fig.savefig(path, dpi=90)
    plt.close(fig)


if __name__ == "__main__":
    sys.exit(main())
