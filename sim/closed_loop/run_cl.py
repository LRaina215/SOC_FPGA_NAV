"""闭环仿真：场景 -> 虚拟 XT-M60 -> FPGA 感知模型（与 RTL 逐位一致）-> 固件（真实 C 代码，动态库）
-> 速度指令 -> 机器人（打滑/陀螺零偏/姿态噪声）-> 下一帧。

用法:
  python sim/closed_loop/run_cl.py --scene terrain --goal 9.5,2.8 --trav v2 --match on --out sim/build/cl/run1
  python sim/closed_loop/run_cl.py --scene loop --waypoints 7,-2.5,7,2.5,-0.5,2.5,-0.5,-2.5 --laps 2
  python sim/closed_loop/run_cl.py --scene terrain --search 9.3,-3.3
  有图模式（先验地图 + 地图系初始位姿对准）：
  python sim/closed_loop/run_cl.py --scene terrain --goal 9.3,2.8 --prior auto --init-err 0.3,-0.2,8
  python sim/closed_loop/run_cl.py --scene terrain --goal 9.3,2.8 --prior auto --perturb boxes --kidnap 20,0.3,0,5
输出：metrics.json、snapshot_*.png（真值标签/代价图/高程/轨迹）
"""
import argparse
import ctypes
import json
import math
import struct
import sys
import time
import platform
import subprocess
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
import nav_host as rk_host  # noqa: E402
import prior_map  # noqa: E402
from evaluation import EVALUATION_VERSION, TerminalCriteria, TerminalJudge, TerrainCounts, tracking_completion_candidate  # noqa: E402

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


MATCH_CB = ctypes.CFUNCTYPE(None, ctypes.c_int32, ctypes.c_int32, ctypes.POINTER(ctypes.c_int32), ctypes.c_int,
                            ctypes.POINTER(ctypes.c_uint32))
MU_CB = ctypes.CFUNCTYPE(None, ctypes.c_int32, ctypes.c_int32, ctypes.c_int32, ctypes.c_int32, ctypes.c_int32)


class Firmware:
    def __init__(self):
        self.d = ctypes.CDLL(str(DLL))
        self.d.py_init()

    def __getattr__(self, n):
        return getattr(self.d, n)


def main(observer=None):
    global DLL
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default="terrain")
    ap.add_argument("--backend", choices=["fast", "mujoco"], default="fast")
    ap.add_argument('--control-repo', type=Path, help='只读控制快照目录，供冻结执行树的MuJoCo后端使用')
    ap.add_argument('--match-timing',choices=['legacy40','rtl50'],default='legacy40',help='历史40ms或RTL逐次周期模型@50MHz')
    ap.add_argument("--start", default="-1.0,-0.3,0")
    ap.add_argument("--goal")
    ap.add_argument("--waypoints")
    ap.add_argument("--laps", type=int, default=1)
    ap.add_argument("--search")
    ap.add_argument("--trav", default="v3")
    ap.add_argument("--match", default="on")
    ap.add_argument("--time", type=float, default=90.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--slip", type=float, default=0.03)
    ap.add_argument("--out", default=str(ROOT / "sim" / "build" / "cl" / "run"))
    ap.add_argument("--snap", type=float, default=0, help="每隔多少秒存一张快照（0 = 只存结束时）")
    ap.add_argument("--gif", action="store_true", help="把快照合成动画 run.gif（未指定 --snap 时按 1 s 一帧）")
    ap.add_argument("--no-build", action="store_true")
    ap.add_argument("--firmware-dll", type=Path, help="加载保存的原版/候选DLL，复评时无需重编")
    ap.add_argument("--no-plots", action="store_true", help="批量评测跳过图片，仍保存地图、时序和判分")
    ap.add_argument("--anchor", type=int, default=64)
    ap.add_argument("--tcfg", default="", help="覆盖地形参数，如 keep_r=0,age_max=100")
    ap.add_argument("--support-bootstrap-mm",type=int,default=0,choices=[0,350],
                    help="显式声明已独立确认的起点平地区域；350要求起点400mm内平坦无障碍")
    ap.add_argument("--support-layer", choices=["auto", "legacy", "coverage4", "external"], default="auto",
                    help="auto按固件接口启用4x4支撑覆盖；external仅供专项原型回调")
    ap.add_argument('--support-snapshot',choices=['auto','off','verify'],default='auto',help='宿主批量HAL桥；verify逐查询与原回调核对，不改变板级实现')
    ap.add_argument("--gmap", default="on", help="A* 用全局记忆栅格（on）还是局部代价图（off）")
    ap.add_argument("--noise", default="", help="只开启部分误差源，逗号分隔：slip,gyro,att,scale")
    ap.add_argument("--ideal", action="store_true", help="理想传感器（无打滑/零偏/噪声），用于排查")
    ap.add_argument("--calib", type=float, default=2.0, help="任务下发前静止标定时间 s（陀螺零偏）")
    ap.add_argument("--prior", default="", help="有图模式：先验地图包路径，或 auto（按场景用虚拟扫图生成并缓存）")
    ap.add_argument("--init-err", default="0.25,-0.2,6", help="有图：下发的初始位姿相对真值的误差 dx,dy(m),dyaw(°)")
    ap.add_argument("--perturb", default="", help="有图：扫图之后场景的变化（先验图里没有）：boxes = 新增 3 个箱子")
    ap.add_argument("--bot", default="balance", choices=["balance", "car"],
                    help="balance = 两轮自平衡轮腿（机身持续摆动、越障后振荡，缺省）；car = 四点着地（旧模型）")
    ap.add_argument("--osc", default="1.5,1.8,3.0", help="自平衡：持续摆动幅度°,频率Hz,每 1 cm 高差激起的摆动°")
    ap.add_argument("--imu-hz", type=float, default=100.0, help="IMU 姿态输出频率")
    ap.add_argument("--imu-gyro", type=int, default=1, help="IMU 同时输出角速度帧（固件用它做陀螺辅助姿态滤波）")
    ap.add_argument("--pmcfg", default="", help="有图跟踪匹配参数 gain_q8,min_curv,min_curv_ang（0 = 固件缺省）")
    ap.add_argument("--mount", default="", help="俯角°,前伸 mm,离地 mm：覆盖标定的雷达安装位姿（安装方案对比）")
    ap.add_argument("--kidnap", default="", help="t,dx,dy,dyaw：t 秒时把机器人真值位姿突然平移/旋转（模拟被撞/严重打滑）")
    a = ap.parse_args()
    if a.control_repo: a.control_repo = str(a.control_repo.resolve())
    if a.backend == 'mujoco' and a.kidnap:
        ap.error('MuJoCo kidnapping has not been implemented; use supported cases')
    if a.firmware_dll:
        if not a.no_build:
            ap.error("--firmware-dll requires --no-build")
        DLL = a.firmware_dll.resolve()
        a.firmware_dll = str(DLL)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    ERR_PTS["fp"].clear()
    ERR_PTS["fn"].clear()
    if a.gif and a.snap <= 0:
        a.snap = 1.0
    for old in out.glob("snapshot_*.png"):
        old.unlink()
    if not a.no_build:
        build_dll()

    scene = W.SCENES[a.scene]()
    pkg = None
    if a.prior:
        pkg = get_prior(a.scene, a.prior)
        if a.perturb:
            W.perturb(scene, a.scene, a.perturb)
    sx, sy, syaw = (float(v) for v in a.start.split(","))
    Bot = rb.BalanceBot if a.bot == "balance" else rb.RobotSim
    bkw = {}
    if a.backend == 'mujoco':
        from mujoco_robot import MujocoRobot
        Bot = MujocoRobot
        bkw = dict(out=out / 'physics_backend', control_repo=a.control_repo)
    elif a.bot == "balance":
        o_ = [float(v) for v in a.osc.split(",")]
        bkw = dict(osc_deg=o_[0], osc_hz=o_[1], kick_deg_per_cm=o_[2])
    if a.ideal:
        bot = Bot(scene, sx, sy, math.radians(syaw), seed=a.seed, slip_mean=0, slip_std=0,
                  gyro_bias_dps=0, gyro_noise_dps=0, att_noise_deg=0, scale_err=0, **bkw)
    elif a.noise:
        on = set(a.noise.split(","))
        bot = Bot(scene, sx, sy, math.radians(syaw), seed=a.seed, **bkw,
                          slip_mean=a.slip if "slip" in on else 0, slip_std=0.02 if "slip" in on else 0,
                          gyro_bias_dps=0.3 if "gyro" in on else 0, gyro_noise_dps=0.3 if "gyro" in on else 0,
                          att_noise_deg=0.3 if "att" in on else 0, scale_err=0.01 if "scale" in on else 0)
    else:
        bot = Bot(scene, sx, sy, math.radians(syaw), seed=a.seed, slip_mean=a.slip, **bkw)
    lid = sw.VirtualLidar(noise_mm=4.0, seed=a.seed + 7, rng_max=8.0, step=0.015)
    mnt = [float(v) for v in a.mount.split(",")] if a.mount else [sw.MOUNT_PITCH_DEG, sw.MOUNT_X * 1000, sw.MOUNT_H * 1000]
    Rbs, tbs = sw.mount_pose(height=mnt[2] / 1000, pitch_down_deg=mnt[0], x=mnt[1] / 1000)
    fpga = fpga_model.FpgaModel(trav=a.trav, mount_z=int(round(mnt[2])))
    for kv in filter(None, a.tcfg.split(",")):
        k, v = kv.split("=")
        fpga.cfg[k] = int(v)
    fw = Firmware()
    coverage_interface = hasattr(fw, 'py_set_coverage_cb')
    coverage_mode = ('coverage4' if coverage_interface else 'legacy') if a.support_layer == 'auto' else a.support_layer
    if coverage_mode in ('coverage4', 'external') and not coverage_interface:
        ap.error('Selected support layer requires the firmware coverage interface')
    if coverage_mode == 'legacy' and coverage_interface:
        ap.error('Coverage firmware requires coverage4 or an explicit external provider')
    if coverage_mode == 'coverage4':
        fpga.enable_support_coverage()
    if hasattr(fw,'py_set_support_bootstrap'):
        fw.py_set_support_bootstrap(a.support_bootstrap_mm)
    elif a.support_bootstrap_mm and not getattr(observer,'allow_legacy_bootstrap',False):
        ap.error('This frozen firmware has no explicit support bootstrap interface')
    if a.match_timing=='rtl50' and not hasattr(fw,'py_set_match_latency_us'):
        ap.error('RTL timing requires the simulator timing shim in the host DLL')
    if a.backend == 'mujoco' and not hasattr(fw,'py_get_cmd_seq'):
        ap.error('MuJoCo requires read-only command-sequence shim; build_frozen_dll.py preserves frozen nav code')
    fw.py_set_match_enable(1 if a.match == "on" else 0)
    fw.py_set_caps(0, 500, 1200, int(round(W.Robot.radius * 1000)))
    fw.py_set_mount(int(round(mnt[0] * 100)), int(round(mnt[1])), int(round(mnt[2])))
    fw.py_set_anchor(a.anchor)
    fw.py_set_gmap_enable(1 if a.gmap == "on" else 0)
    fw.py_set_pose_decomp(1 if a.bot == "car" else 0)
    if a.pmcfg:
        fw.py_set_pm_prior(*(int(v) for v in a.pmcfg.split(",")))
    # Record resolved inputs before any simulation, without changing the algorithm.
    from trav_v3 import DEFAULT_V3
    effective_cfg = dict(DEFAULT_V3) if a.trav == "v3" else dict(fpga.cfg)
    effective_cfg.update(fpga.cfg)
    terminal_criteria = TerminalCriteria(position_m=1.0 if a.search else 0.150)
    manifest = dict(evaluation=EVALUATION_VERSION, arguments=vars(a), argv=sys.argv,
                    git_head=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
                    working_changes=subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT, text=True).splitlines(),
                    python=sys.version, executable=sys.executable, platform=platform.platform(), numpy=np.__version__,
                    terrain_cfg=effective_cfg, mount_pitch_x_z=mnt,
                    lidar_assumptions=dict(raycast=getattr(scene.w,"raycast_version",getattr(sw.World,"raycast_version","legacy_march")), noise_mm=lid.noise,
                                           min_m=lid.rng_min,max_m=lid.rng_max,legacy_step_m=lid.step),
                    capability_assumptions=dict(step_m=W.Robot.step_climb, slope_deg=W.Robot.slope_max_deg,
                                                footprint_radius_m=W.Robot.radius, wheel_track_m=W.Robot.width),
                    response_assumptions=dict(tau_s=0.12, accel_mps2=1.5, angular_accel_rps2=6.0,
                                              safe_in_loop=False, rl_in_loop=False),
                    terminal_criteria=terminal_criteria.__dict__,
                    terrain_alignment="current_pose_aligned; excludes global localization drift",
                    prior_package_bytes=len(pkg) if pkg is not None else 0)
    manifest['support_bootstrap_interface']=hasattr(fw,'py_set_support_bootstrap')
    manifest['support_layer'] = dict(mode=coverage_mode, clock='SYS_TIME_US; capture timestamp from retained raw bank',
        subcell_mm=65536/5244 if coverage_mode=='coverage4' else None,
        known_distinct_epochs=2 if coverage_mode=='coverage4' else None,
        fresh_upper_s=1, retained_static_upper_s=20, fill=False)
    manifest["firmware_dll"] = str(DLL)
    if a.backend == 'mujoco':
        manifest['response_assumptions'] = bot.report()
    manifest['response_assumptions']['match_timing']=a.match_timing
    manifest['response_assumptions']['raw_frame_buffer']='two-bank queue' if a.match_timing=='rtl50' else 'legacy capture after consumption'
    (out / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    CTL_CB = ctypes.CFUNCTYPE(None, ctypes.c_int, ctypes.c_int32, ctypes.c_int32, ctypes.c_int32)

    @CTL_CB
    def ctl_cb(op, x, y, z):
        fpga.ctl(op, x, y, z)
    fw.py_set_ctl_cb(ctl_cb)
    if coverage_mode == 'coverage4':
        COVERAGE_CB = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_int32, ctypes.c_int32, ctypes.POINTER(ctypes.c_int32))
        coverage_cache = [None, None, None]

        @COVERAGE_CB
        def coverage_cb(x, y, result):
            cov = fpga.coverage
            key = (cov.seq, fpga.clock_us // 200000, cov.cx, cov.cy)
            if key != coverage_cache[0]:
                known, fresh = cov.mask_windows(fpga.clock_us)
                coverage_cache[:] = [key, known, fresh]
            ix = ((int(x) * INV_RES) >> 16) - (cov.cx // 4 - 16)
            iy = ((int(y) * INV_RES) >> 16) - (cov.cy // 4 - 16)
            result[0] = int(coverage_cache[1][iy, ix]) if 0 <= ix < 32 and 0 <= iy < 32 else 0
            result[1] = int(coverage_cache[2][iy, ix]) if 0 <= ix < 32 and 0 <= iy < 32 else 0
            return 1
        fw.py_set_coverage_cb(coverage_cb)

    # 里程计系 = 世界系平移 (sx, sy)、旋转 syaw；有图模式下固件在 SET_POSE 之后工作在地图系 = 世界系
    c0, s0 = math.cos(math.radians(syaw)), math.sin(math.radians(syaw))
    if pkg is not None:
        c0, s0, sx0, sy0 = 1.0, 0.0, 0.0, 0.0
    else:
        sx0, sy0 = sx, sy

    def w2o(x, y):
        dx, dy = x - sx0, y - sy0
        return c0 * dx + s0 * dy, -s0 * dx + c0 * dy

    def o2w(x, y):
        return sx0 + c0 * x - s0 * y, sy0 + s0 * x + c0 * y

    callback_errors = []
    match_timing_log=[]
    @MATCH_CB
    def match_cb(cxm, cym, cand, k, res):
        cs = np.array([[cand[3 * i], cand[3 * i + 1], cand[3 * i + 2]] for i in range(k)], dtype=np.int64)
        try:
            o = fpga.match_eval(cxm, cym, cs)
            if a.match_timing=='rtl50':
                from slf_timing import evaluation_cycles,evaluation_us
                ns,nv=int(o[0,3]),int(o[0,0])
                cycles=evaluation_cycles(ns,nv,k);latency=evaluation_us(ns,nv,k)
                fw.py_set_match_latency_us(ctypes.c_uint32(latency))
                match_timing_log.append([now_us,k,ns,nv,cycles,latency])
        except Exception as error:
            callback_errors.append(error)
            o = np.zeros((k, 4), dtype=np.int64)
        for i in range(k):
            for j in range(4):
                res[4 * i + j] = int(o[i, j]) & 0xFFFFFFFF
    fw.py_set_match_cb(match_cb)

    @MU_CB
    def mu_cb(dx, dy, da, cxm, cym):
        try:
            fpga.match_update(dx, dy, da, cxm, cym)
        except Exception as error:
            callback_errors.append(error)
    fw.py_set_mu_cb(mu_cb)
    H_CB = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_int32, ctypes.c_int32, ctypes.POINTER(ctypes.c_int32))

    def support_at(x,y):
        ix = int((x * INV_RES) >> 16); iy = int((y * INV_RES) >> 16)
        wx, wy = ix - (fpga.cx - 64), iy - (fpga.cy - 64)
        if not (0 <= wx < 128 and 0 <= wy < 128):
            return None
        a_ = ((iy & 127) << 7) | (ix & 127)
        em = fpga.em
        tag = (((iy >> 7) & 15) << 4) | ((ix >> 7) & 15)
        if em.cnt[a_] == 0 or em.tag[a_] != tag:
            return None
        return (int(em.zmin[a_]),int(em.zmax[a_]),fpga.seq,int(em.cnt[a_]),
                (int(em.fc)-int(em.stamp[a_]))&255,int(em.stale[a_]))

    @H_CB
    def h_cb(x,y,hp):
        s=support_at(x,y)
        if s is None or s[5] or s[4]>fpga.cfg.get('age_max',60) or s[1]-s[0]>40:
            return 0
        hp[0] = int((s[0]+s[1])/2) # C有符号除法向零截断
        return 1
    fw.py_set_h_cb(h_cb)

    @H_CB
    def support_cb(x,y,out):
        s=support_at(x,y)
        if s is None:return 0
        for i,v in enumerate(s):out[i]=v
        return 1
    if hasattr(fw,'py_set_support_cb'):fw.py_set_support_cb(support_cb)
    snapshot_enabled=hasattr(fw,'py_set_support_snapshot') and a.support_snapshot!='off'
    snapshot_keys=[None,None]
    if snapshot_enabled:
        fw.py_set_support_snapshot.argtypes=[ctypes.c_void_p]*5+[ctypes.c_int32,ctypes.c_int32,ctypes.c_uint16,ctypes.c_uint16]
        fw.py_set_coverage_snapshot.argtypes=[ctypes.c_void_p,ctypes.c_void_p,ctypes.c_int32,ctypes.c_int32,ctypes.c_uint32]
        fw.py_set_support_snapshot_verify(a.support_snapshot=='verify')
    def sync_support_snapshot():
        if not snapshot_enabled:return
        em=fpga.em;key=(id(em),fpga.seq,fpga.cx,fpga.cy)
        if snapshot_keys[0]!=key:
            gx=np.arange(fpga.cx-64,fpga.cx+64);gy=np.arange(fpga.cy-64,fpga.cy+64)
            slots=((gy[:,None]&127)<<7)|(gx[None,:]&127);tag=(((gy[:,None]>>7)&15)<<4)|((gx[None,:]>>7)&15)
            values=[np.ascontiguousarray(em.zmin[slots],dtype=np.int16),np.ascontiguousarray(em.zmax[slots],dtype=np.int16),
                    np.ascontiguousarray(np.where(em.tag[slots]==tag,em.cnt[slots],0),dtype=np.uint8),
                    np.ascontiguousarray((int(em.fc)-em.stamp[slots])&255,dtype=np.uint8),np.ascontiguousarray(em.stale[slots],dtype=np.uint8)]
            fw.py_set_support_snapshot(*[v.ctypes.data for v in values],fpga.cx,fpga.cy,fpga.seq,INV_RES);snapshot_keys[0]=key
        if fpga.coverage is not None:
            cov=fpga.coverage;ep=fpga.clock_us//200000;key=(cov.seq,ep,cov.cx,cov.cy)
            if snapshot_keys[1]!=key:
                known,fresh=cov.mask_windows(fpga.clock_us)
                fw.py_set_coverage_snapshot(known.ctypes.data,fresh.ctypes.data,cov.cx//4,cov.cy//4,ep);snapshot_keys[1]=key

    # 任务
    upload = rk_host.map_messages(pkg) if pkg is not None else []
    loc_v = (ctypes.c_int32 * 12)()
    align_t = None
    loc_hist = []

    def send(b):
        buf = (ctypes.c_uint8 * len(b)).from_buffer_copy(b)
        fw.py_push_host(buf, len(b), ctypes.c_uint32(now_us))
    now_us = 1000
    fw.py_set_time(now_us)
    goals_w = []
    pending = []                      # 静止标定结束后再下发任务
    if pkg is not None:
        if a.init_err == "rand":             # 按种子随机：±0.4 m、±8°（人工摆放 + 目测朝向的误差量级）
            r_ = np.random.default_rng(a.seed + 100)
            ex_, ey_, ea_ = r_.uniform(-0.4, 0.4), r_.uniform(-0.4, 0.4), r_.uniform(-8, 8)
        else:
            ex_, ey_, ea_ = (float(v) for v in a.init_err.split(","))
        pending.append(rk_host.set_pose(int((sx + ex_) * 1000), int((sy + ey_) * 1000), int(round((syaw + ea_) * 100))))
    kid = [float(v) for v in a.kidnap.split(",")] if a.kidnap else None
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
    terrain_counts = TerrainCounts()
    loc_err_full = []
    loc_valid_samples = 0
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
    terminal_goal = (tx, ty) if a.search else (goals_w[-1] if goals_w else None)
    terminal = TerminalJudge(terminal_goal, terminal_criteria)
    truth_prev = (0.0, bot.x, bot.y, bot.yaw)

    def local_o2w():
        """里程计系 -> 世界系，按"当前估计位姿 ↔ 当前真值位姿"对齐：局部地图是用估计位姿建的，
        用它评估地形判定可以排除全局漂移的影响（漂移另由定位误差指标衡量）"""
        fw.py_get_state(st)
        ex, ey, eyaw = st[0] / 1000, st[1] / 1000, (st[3] & 0xFFFF) / 65536 * 2 * math.pi
        if hasattr(fw,'py_get_terrain_pose'):
            local=(ctypes.c_int32*6)();fw.py_get_terrain_pose(local)
            ex,ey,eyaw=local[0]/1000,local[1]/1000,(local[2]&65535)*2*math.pi/65536
        dyaw = bot.yaw - eyaw
        cd, sd = math.cos(dyaw), math.sin(dyaw)

        def f(ox, oy):
            dx, dy = ox - ex, oy - ey
            return bot.x + cd * dx - sd * dy, bot.y + sd * dx + cd * dy
        return f

    def snapshot(tag):
        if a.no_plots:
            return
        save_snapshot(out / f"snapshot_{tag}.png", scene, fpga, traj_true, traj_est, goals_w, local_o2w(), w2o, a,
                      get_gmap(fw), o2w)

    if observer is not None:observer.setup(bot,fpga,fw,a)

    ms = 0
    n_frames = 0
    frame_pending = False
    raw_queue=[]
    raw_dropped=0
    while t < a.time or (terminal.declared_s is not None and not terminal.finished):
        fpga.clock_us = now_us & 0xffffffff
        sync_support_snapshot()
        if callback_errors:
            raise RuntimeError('FPGA callback failed; trial invalid') from callback_errors[0]
        for _ in range(4):
            if upload:
                send(upload.pop(0))
        if pending and t >= a.calib and not upload:
            for b in pending:
                send(b)
            pending = []
        if kid and t >= kid[0]:
            c_, s_ = math.cos(bot.yaw), math.sin(bot.yaw)
            bot.x += kid[1] * c_ - kid[2] * s_
            bot.y += kid[1] * s_ + kid[2] * c_
            bot.yaw += math.radians(kid[3])
            bot.update_attitude()
            kid = None
        # 雷达帧：上一帧已提交（FPGA 双缓冲握手）才采集下一帧
        if t >= t_frame and (a.match_timing=='rtl50' or not frame_pending):
            t_frame += 1.0 / a.fps
            if a.match_timing=='rtl50' and len(raw_queue)+int(frame_pending)>=2:
                raw_dropped+=1
            else:
                Rwb, twb = bot.body_pose()
                Rws, tws = sw.compose(Rwb, twb, Rbs, tbs)
                depth, _ = lid.capture(scene.w, Rws, tws)
                if observer is not None:observer.raw(depth,Rws,tws,t)
                raw_queue.append((depth,now_us))
        if not frame_pending and raw_queue:
            depth,capture_us=raw_queue.pop(0)
            fpga.frame(depth)
            fpga.capture_us = capture_us
            fw.py_set_lidar_waiting(ctypes.c_uint32(capture_us))
            frame_pending=True
        assert len(raw_queue)+int(frame_pending)<=2
        if frame_pending:
            fw.py_poll()
            if fw.py_get_commit(commit_v):
                phase = fw.py_get_commit_phase() if hasattr(fw, "py_get_commit_phase") else 0
                frame_pending = phase == 1             # 匹配遍继续保留同一深度帧，禁止覆盖xyz
                v = list(commit_v)
                if hasattr(fw, 'py_get_lidar_capture_us'):
                    fw.py_get_lidar_capture_us.restype = ctypes.c_uint32
                    if fw.py_get_lidar_capture_us() != fpga.capture_us:
                        raise RuntimeError('Firmware fusion timestamp differs from retained raw capture')
                fpga.commit(v[:12], v[12], v[13], v[14], v[15])
                slot = (ctypes.c_uint8 * (128 * 128)).from_buffer_copy(np.clip(fpga.cost, 0, 255).astype(np.uint8).tobytes())
                fw.py_set_map(slot, fpga.cx, fpga.cy, ctypes.c_uint16(fpga.seq), ctypes.c_uint16(INV_RES))
                sync_support_snapshot()
                mv = (ctypes.c_int32 * 12)()
                fw.py_get_match(mv)
                if phase != 1: match_log.append(list(mv))
                n_frames += phase != 1
                if observer is not None and phase != 1:observer.fused(fpga,t,local_o2w(),v)
                if phase != 1 and n_frames % 5 == 0:
                    stat_confusion(scene, fpga, local_o2w(), conf, t, terrain_counts)
        # 里程计 50 Hz、IMU 50 Hz
        if t >= t_odo:
            t_odo += .01 if a.backend == 'mujoco' else .02
            vx, vy, wz = bot.odom()
            fw.py_push_odom(vx, vy, wz, ctypes.c_uint32(now_us))
        if t >= t_imu:
            t_imu += 1.0 / a.imu_hz
            b = (bot.imu_wit_gyro_frame(1.0 / a.imu_hz) if a.imu_gyro else b"") + bot.imu_wit_frame()
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
        if a.backend == 'mujoco':
            bot.submit(cmdv[:3], fw.py_get_cmd_seq())
        bot.step((cmdv[0], cmdv[1], cmdv[2]), dt_ctrl)
        ms += 1
        t = ms * dt_ctrl
        now_us += 1000
        fw.py_set_time(ctypes.c_uint32(now_us))
        if observer is not None:observer.tick(bot,fw,t,cmdv)
        if getattr(bot,'failed',False):
            tail=getattr(observer,'failure_tail_s',0.)
            first=getattr(bot,'first_failure_s',None)
            if not tail or first is None or t>=first+tail:break
        if ms % 100 == 0 and pkg is not None:
            fw.py_get_loc(loc_v)
            loc_hist.append((t, loc_v[0], loc_v[1], loc_v[2], loc_v[10], loc_v[11], loc_v[5]))
            if align_t is None and loc_v[0] in (2, 3):
                align_t = t
        localized = pkg is None or (align_t is not None and loc_v[0] in (2, 3))
        if ms % 100 == 0:
            fw.py_get_state(st)
            ex, ey = o2w(st[0] / 1000, st[1] / 1000)
            traj_true.append((bot.x, bot.y, bot.z))
            traj_est.append((ex, ey, st[2] / 1000))
            yaw0 = 0.0 if pkg is not None else math.radians(syaw)
            yaw_err = math.degrees(math.remainder(((st[3] & 0xFFFF) / 65536 * 2 * math.pi) + yaw0 - bot.yaw, 2 * math.pi))
            err = (math.hypot(ex - bot.x, ey - bot.y), abs(st[2] / 1000 - bot.z), abs(yaw_err))
            loc_err_full.append(err)
            if localized:
                loc_err.append(err)
                loc_valid_samples += 1
            prev_t, prev_x, prev_y, prev_yaw = truth_prev
            speed = math.hypot(bot.x - prev_x, bot.y - prev_y) / (t - prev_t)
            yaw_rate = math.remainder(bot.yaw - prev_yaw, 2 * math.pi) / (t - prev_t)
            truth_prev = (t, bot.x, bot.y, bot.yaw)
            tlog.append((t, bot.x, bot.y, bot.z, math.degrees(bot.yaw), ex, ey, st[2] / 1000,
                         math.degrees(bot.yaw) + yaw_err, cmdv[0], cmdv[2], int(bot.blocked),
                         math.degrees(bot.pitch), st[4] * 360 / 65536, math.degrees(bot.roll), st[5] * 360 / 65536,
                         speed, yaw_rate, int(localized)))
            declared = bool((a.goal or a.waypoints) and st[7] == 2)
            if a.search:
                # Tracking has no firmware ARRIVED state: retain the historical
                # external 1 m objective, then independently verify the stop.
                declared = tracking_completion_candidate(found_t is not None, st[6], cmdv,
                    math.hypot(bot.x - tx, bot.y - ty), terminal_criteria.position_m)
            terminal.observe(t, bot.x, bot.y, speed, yaw_rate, declared=declared and t <= a.time)
            if terminal.finished and not getattr(bot,'failed',False):
                if terminal.success and bot.collisions == 0:
                    arrived_t = terminal.declared_s
                break
        if t >= t_snap:
            snapshot(f"{snap_i:03d}_t{t:05.1f}s")
            snap_i += 1
            t_snap += a.snap
    snapshot("final")
    if snapshot_enabled:
        snapshot_stats=(ctypes.c_uint32*2)();fw.py_get_support_snapshot_stats(snapshot_stats)
        if snapshot_stats[1]:raise RuntimeError(f'Bulk HAL differs from callback: {snapshot_stats[1]}/{snapshot_stats[0]} queries')
    if fpga.view is not None:
        _, V_, H_, RO_ = fpga.view
        gm_ = get_gmap(fw)
        fw.py_get_state(st)
        np.savez(out / "final_maps.npz", cost=cost_window(fpga), lethal=fpga.last_lethal, V=V_, H=H_, RO=RO_, seen=gm_[5],
                 cx=fpga.cx, cy=fpga.cy, gmap=gm_[0], gm_o=np.array(gm_[1:4]), path=gm_[4], state=np.array(list(st)),
                 true=np.array([bot.x, bot.y, bot.z, bot.yaw]), fp_pts=np.array(ERR_PTS["fp"]).reshape(-1, 3),
                 fn_pts=np.array(ERR_PTS["fn"]).reshape(-1, 3), match_log=np.array(match_log), tlog=np.array(tlog),
                 loc_hist=np.array(loc_hist).reshape(-1, 7))
    if a.gif and not a.no_plots:
        from PIL import Image
        frames = [Image.open(p) for p in sorted(out.glob("snapshot_[0-9]*.png"))] + [Image.open(out / "snapshot_final.png")]
        frames[0].save(out / "run.gif", save_all=True, append_images=frames[1:], duration=250, loop=0)

    if not a.no_plots:
        save_err_plot(out / "trav_errors.png", scene, ERR_PTS, traj_true)
    if tlog:
        tl = np.array(tlog)
        np.savetxt(out / "timeseries.csv", tl, delimiter=",", fmt="%.4f",
                   header="t,x,y,z,yaw_deg,ex,ey,ez,eyaw_deg,cmd_vx,cmd_wz,blocked,pitch,epitch,roll,eroll,true_speed_mps,true_yaw_rate_rps,loc_valid", comments="")
        if not a.no_plots:
            save_timeseries_plot(out / "loc_error.png", tl)
    le = np.array(loc_err) if loc_err else np.zeros((1, 3))
    ml = np.array(match_log) if match_log else np.zeros((1, 12))
    le_full = np.array(loc_err_full) if loc_err_full else None
    tp, fn, fp, tn = int(conf[1, 1]), int(conf[1, 0]), int(conf[0, 1]), int(conf[0, 0])
    m = {
        'host_support_snapshot':dict(enabled=snapshot_enabled,verify=a.support_snapshot=='verify',queries=int(snapshot_stats[0]) if snapshot_enabled else 0,mismatches=int(snapshot_stats[1]) if snapshot_enabled else 0),
        "scene": a.scene, "trav": a.trav, "match": a.match, "seed": a.seed, "slip": a.slip,
        "arrived": arrived_t is not None, "time_s": round(arrived_t if arrived_t else t, 1),
        "evaluation": EVALUATION_VERSION,
        "firmware_arrived": terminal.declared_s is not None and not bool(a.search),
        "completion_candidate": terminal.declared_s is not None,
        "outcome": "collision" if bot.collisions else ("physical_failure" if getattr(bot,'failed',False) else terminal.outcome),
        "terminal": terminal.report(),
        "simulation_end_s": round(t, 3),
        "raw_frames_dropped": raw_dropped,
        "terrain": terrain_counts.report(),
        "loc_valid_fraction": loc_valid_samples / len(loc_err_full) if loc_err_full else None,
        "loc_samples": len(loc_err_full), "loc_valid_samples": loc_valid_samples,
        "loc_err_xy_full_rms_m": round(float(np.sqrt((le_full[:, 0] ** 2).mean())), 3) if le_full is not None else None,
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
        "prior": bool(pkg is not None),
        "align_s": round(align_t - a.calib, 2) if align_t is not None else None,
        "loc_final": int(loc_v[0]) if pkg is not None else 0,
        "n_align": int(loc_v[3]), "n_align_fail": int(loc_v[4]), "n_lost": int(loc_v[5]),
        "align_q": [int(loc_v[6]), int(loc_v[7])],
        "t_degraded_s": round(sum(1 for h in loc_hist if h[1] == 3) * 0.1, 1),
        "t_lost_s": round(sum(0.1 for h in loc_hist if align_t is not None and h[0] > align_t and h[1] in (1, 4)), 1),
        "trav_lethal_recall": round(tp / max(1, tp + fn), 3),
        "trav_false_lethal_rate": round(fp / max(1, fp + tn), 4),
        "trav_cells": {"tp": tp, "fn": fn, "fp": fp, "tn": tn},
        "wall_s": round(time.time() - wall0, 1),
        "astar_heap_max": int(fw.py_astar_heap_max()),
    }
    if not loc_err:
        for key in ("loc_err_xy_rms_m", "loc_err_xy_final_m", "loc_err_xy_max_m",
                    "loc_err_z_max_m", "loc_err_yaw_final_deg"):
            m[key] = None
    if a.backend == 'mujoco':
        m['physics_backend']=bot.report(); bot.close()
    if observer is not None:observer.finish(m)
    if match_timing_log:
        np.savetxt(out/'match_timing.csv',np.array(match_timing_log,dtype=np.int64),fmt='%d',delimiter=',',
                   header='now_us,candidates,sampled_points,valid,cycles,latency_us',comments='')
    (out / "metrics.json").write_text(json.dumps(m, ensure_ascii=False, indent=1), encoding="utf-8")
    print(json.dumps(m, ensure_ascii=False))
    return 0


def prior_cache_name(scene_name):
    import slf_match
    variant=scene_name if slf_match.SN==256 else f"{scene_name}_sn{slf_match.SN}"
    ray=getattr(sw.World,'raycast_version',None)
    return variant+(f"_{ray}" if ray else "")+'.nvmp'


def get_prior(scene_name, spec):
    """先验地图包：spec 为文件路径，或 auto（虚拟扫图生成，缓存到 sim/build/prior/<场景>.nvmp）"""
    if spec != "auto":
        return Path(spec).read_bytes()
    import slf_match
    p = ROOT / "sim" / "build" / "prior" / prior_cache_name(scene_name)
    if p.exists():
        return p.read_bytes()
    sc = W.SCENES[scene_name]()
    lid = sw.VirtualLidar(noise_mm=4.0, seed=11, rng_max=8.0, step=0.015)
    sp, ap_, _ = prior_map.survey_scan(sc, lid, sw.mount_pose())
    x0, y0, x1, y1 = sc.bounds
    pkg, _ = prior_map.build_package(sp, ap_, bounds_mm=(x0 * 1000 - 200, y0 * 1000 - 200, x1 * 1000 + 200, y1 * 1000 + 200))
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_bytes(pkg)
    return pkg


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


def stat_confusion(scene, fpga, o2w, conf, t=0.0, terrain_counts=None):
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
    if terrain_counts is not None:
        # Raw lethal and observed masks are window-order arrays, unlike cost slots.
        labels = np.full(cw.shape, 2, dtype=np.int8)
        labels[inb] = scene.gt_lookup(WX[inb], WY[inb])
        terrain_counts.add(labels, cw, fpga.last_lethal, fpga.view[1], inb)
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
    seen = (ctypes.c_uint8 * (gn * gn))()
    fw.py_get_seen(seen)
    return g, v[0], v[1], v[3], np.array(path[:2 * n]).reshape(-1, 2), np.frombuffer(bytes(seen), dtype=np.uint8).reshape(gn, gn)


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
        g, gox, goy, ds, pth, seen = gm
        cell = ds * RES_MM / 1000
        gx0, gy0 = o2w_est(gox * cell, goy * cell)
        img = np.where(g == 255, np.nan, np.where(g >= 254, 254.0, g.astype(float)))
        ext = (gx0, gx0 + g.shape[1] * cell, gy0, gy0 + g.shape[0] * cell)
        ax[3].imshow(img, origin="lower", extent=ext, cmap="magma_r", vmin=0, vmax=254)
        if a.prior and a.search:
            ax[3].imshow(np.where(seen > 0, 1.0, np.nan), origin="lower", extent=ext, cmap="winter", alpha=0.35, vmin=0, vmax=1)
        if len(pth):
            px, py = o2w_est((pth[:, 0] + gox + 0.5) * cell, (pth[:, 1] + goy + 0.5) * cell)
            ax[3].plot(px, py, "g.-", lw=1, ms=3)
        if traj_est:
            ax[3].plot(te[:, 0], te[:, 1], "b--", lw=1)
        for gg in goals:
            ax[3].plot(gg[0], gg[1], "k*", ms=12)
        ax[3].set_xlim(x0, x1)
        ax[3].set_ylim(y0, y1)
        ax[3].set_title("先验 + 观测记忆栅格、A* 路径（地图系）" + ("，青=相机已看过" if a.prior and a.search else "")
                        if a.prior else "全局记忆栅格 + A* 路径（估计位姿系）")
    for x in ax:
        x.set_aspect("equal")
    plt.tight_layout()
    fig.savefig(path, dpi=90)
    plt.close(fig)


if __name__ == "__main__":
    sys.exit(main())
