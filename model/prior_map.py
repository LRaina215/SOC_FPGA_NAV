"""先验地图包（有图模式）：离线建图结果 -> FPGA 定位图 + 全局代价图，打包成 .nvmp 下发给板子。

板上"只定位不建图"：定位图直接写进 SLF 结构图 RAM（槽字格式与在线建图相同，法向/门控离线算好），
匹配时关闭并图和似然场重建；全局代价图（20 cm）作为记忆栅格的静态层，供 A* 使用。

输入有两种（都在地图系，单位 mm）：
  · 结构点：逐帧深度图 + 该帧位姿（Linux SLAM 的轨迹或仿真真值），用与硬件相同的相邻行竖直判据提取；
  · 点云：SLAM 输出的整张地图点云（无序），按 5 cm 竖直柱内高差提取竖直结构。
地面点（全部点）用于高程与可通行代价。

包格式（小端）：
  头 40 B：'NVMP' | u16 版本=1 | u16 标志 | i32 ox, oy（结构窗口原点，全局 5 cm 格，窗口 256×256）
          | u32 n_struct | i32 gox, goy（代价图原点，全局 20 cm 格）| u16 gnx, gny | u32 body_crc32 | u32 body_len
  体：n_struct × {u16 槽地址, u24 槽字}，然后 gnx×gny 字节代价（行优先，0..200 可通行，254 致命，255 未知）
"""
import math
import struct
import zlib

import numpy as np

import slf_match as slf

SN = slf.SN                   # 256
RES_MM = 50
INV_RES = 1311
DS = 4                        # 代价图一格 = 4 个 5 cm 格
GM_MAX = 96                   # 固件记忆栅格边长
MAGIC = b"NVMP"
HDR_FMT = "<4sHHiiIiiHHII"
HDR_LEN = struct.calcsize(HDR_FMT)


def cell_of(v_mm):
    """与硬件相同的全局 5 cm 格下标：floor(v·INV_RES / 65536)"""
    return (np.asarray(v_mm, dtype=np.int64) * INV_RES) >> 16


def cell16_of(v_mm):
    return (np.asarray(v_mm, dtype=np.int64) * INV_RES) >> 12


# ---------------------------------------------------------------- 结构
def struct_cells_from_points(pts_xy, ox, oy, min_count=3):
    """结构点 (K,2) mm -> 窗口数组 occ, cxs, cys（256×256，[y, x]，下标 = 全局格 - 原点）。
    离线没有流式约束：每格取全部点的亚格平均（比板上"前 4 次平均后冻结"更准），点数不足 min_count 的格丢弃（噪声）"""
    fx, fy = cell16_of(pts_xy[:, 0]), cell16_of(pts_xy[:, 1])
    wx, wy = (fx >> 4) - ox, (fy >> 4) - oy
    ok = (wx >= 0) & (wx < SN) & (wy >= 0) & (wy < SN)
    fx, fy, wx, wy = fx[ok], fy[ok], wx[ok], wy[ok]
    idx = wy * SN + wx
    n = np.bincount(idx, minlength=SN * SN)
    sx = np.bincount(idx, weights=(fx & 15) + 0.5, minlength=SN * SN)
    sy = np.bincount(idx, weights=(fy & 15) + 0.5, minlength=SN * SN)
    occ = n >= min_count
    with np.errstate(invalid="ignore", divide="ignore"):
        cxs = np.where(occ, np.clip(np.floor(sx / np.maximum(n, 1)), 0, 15), 0).astype(np.int64)
        cys = np.where(occ, np.clip(np.floor(sy / np.maximum(n, 1)), 0, 15), 0).astype(np.int64)
    return occ.reshape(SN, SN), cxs.reshape(SN, SN), cys.reshape(SN, SN), n.reshape(SN, SN)


def struct_points_from_cloud(cloud, ground_win=7):
    """无序点云 (M,3) mm -> 结构点 (K,2)。5 cm 竖直柱：离局部地面（7×7 柱最低点）20 mm..Z_HI 的点，
    若柱内这些点的高差 >= 2·DZ_MIN，则认为是竖直面（墙、箱体侧面、路沿立面），取这些点。
    平台顶面、坡面在一根柱内几乎没有高差，不会被选中——与板上相邻行判据的结果一致。"""
    ix, iy = cell_of(cloud[:, 0]), cell_of(cloud[:, 1])
    x0, y0 = ix.min(), iy.min()
    nx, ny = int(ix.max() - x0 + 1), int(iy.max() - y0 + 1)
    k = (iy - y0) * nx + (ix - x0)
    z = cloud[:, 2].astype(np.int64)
    zmin = np.full(nx * ny, 1 << 40, dtype=np.int64)
    np.minimum.at(zmin, k, z)
    g = zmin.reshape(ny, nx)
    from scipy import ndimage
    g = ndimage.minimum_filter(np.where(g >= (1 << 40), 1 << 40, g), size=ground_win, mode="nearest").reshape(-1)
    zr = z - g[k]
    band = (zr >= slf.DZ_MIN) & (zr <= slf.Z_HI)
    lo = np.full(nx * ny, 1 << 40, dtype=np.int64)
    hi = np.full(nx * ny, -(1 << 40), dtype=np.int64)
    np.minimum.at(lo, k[band], z[band])
    np.maximum.at(hi, k[band], z[band])
    vert = (hi - lo) >= 2 * slf.DZ_MIN
    sel = band & vert[k]
    return cloud[sel, :2]


def prior_lf(occ, cxs, cys):
    """离线"建似然场"：法向（与硬件同一整数算法）+ 门控位（距结构 <= GATE_D）"""
    nx, ny, has, q = slf.cell_normals(occ, cxs, cys)
    d = np.minimum(2 * slf.chebyshev_dt(occ, slf.DMAX) + slf.taxicab_dt(occ, 3 * slf.DMAX), 3 * slf.DMAX)
    gate = d <= slf.GATE_D
    return q, has, gate


def struct_words(ox, oy, occ, cxs, cys, q, has, gate):
    """-> (槽地址, 23 位槽字) 列表（只含非零槽字）"""
    wy, wx = np.mgrid[0:SN, 0:SN]
    gx, gy = wx + ox, wy + oy
    tag = (((gy >> slf.SN_LOG2) & 3) << 2) | ((gx >> slf.SN_LOG2) & 3)
    hit = np.where(occ, slf.HIT_MAX, 0)
    w = (tag << 19) | (hit << 16) | (cxs << 12) | (cys << 8) | (occ.astype(np.int64) << 7) | (q << 2) \
        | (has.astype(np.int64) << 1) | gate.astype(np.int64)
    w = np.where(occ | gate, w, 0)
    slot = ((gy & (SN - 1)) << slf.SN_LOG2) | (gx & (SN - 1))
    m = w.reshape(-1) != 0
    return slot.reshape(-1)[m].astype(np.int64), w.reshape(-1)[m].astype(np.int64)


# ---------------------------------------------------------------- 高程与代价
def elevation_from_points(pts, x0c, y0c, nx, ny, zcap=1500):
    """全部点 (M,3) mm -> 5 cm 高程（每格最高点，mm；无观测 = NaN）。只用离局部地面 zcap 以内的点（去掉天花板）"""
    ix, iy = cell_of(pts[:, 0]) - x0c, cell_of(pts[:, 1]) - y0c
    ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    ix, iy, z = ix[ok], iy[ok], pts[ok, 2].astype(np.float64)
    k = iy * nx + ix
    zlo = np.full(nx * ny, np.inf)
    np.minimum.at(zlo, k, z)
    keep = z <= zlo[k] + zcap
    h = np.full(nx * ny, -np.inf)
    np.maximum.at(h, k[keep], z[keep])
    h = np.where(np.isfinite(h), h, np.nan)
    return h.reshape(ny, nx)


def cost_from_elevation(h, step_max=100, slope_max_deg=20.0, k_slope=200):
    """5 cm 高程 -> 5 cm 代价（离线、浮点，数据是整张稠密地图，比板上逐帧分析简单可靠）：
    3×3 高差 > step_max -> 致命；20 cm 基线坡度 > 上限 -> 致命；其余代价随坡度线性 0..k_slope"""
    from scipy import ndimage
    known = np.isfinite(h)
    hf = np.where(known, h, -1e9)
    hmax = ndimage.maximum_filter(hf, size=3, mode="nearest")
    hmin = ndimage.minimum_filter(np.where(known, h, 1e9), size=3, mode="nearest")
    step = np.where(known, np.maximum(hmax - h, h - hmin), 0)
    step = np.where(np.abs(step) > 1e8, 0, step)
    hz = np.where(known, h, ndimage.median_filter(np.where(known, h, 0), size=5))
    gy, gx = np.gradient(ndimage.uniform_filter(hz, size=4), RES_MM)
    slope = np.degrees(np.arctan(np.hypot(gx, gy)))
    # 台阶附近的梯度是台阶本身造成的，不当坡度（相邻格高差 45 mm 以上才算台阶：30° 坡一格也只升 29 mm）
    near_step = ndimage.maximum_filter(step > 45, size=5)
    slope = np.where(near_step, 0, slope)
    lethal = known & ((step > step_max) | (slope > slope_max_deg))
    cost = np.where(known, np.clip(slope / slope_max_deg * k_slope, 0, 200), 255).astype(np.int64)
    cost[lethal] = 254
    return cost


def coarse_cost(cost5, min_known=6):
    """5 cm 代价 -> 20 cm（与固件记忆栅格同规则：16 格中 >= 2 格致命 -> 致命；已知格 < min_known -> 未知；否则取最大代价）"""
    ny, nx = cost5.shape
    gny, gnx = ny // DS, nx // DS
    c = cost5[:gny * DS, :gnx * DS].reshape(gny, DS, gnx, DS).transpose(0, 2, 1, 3).reshape(gny, gnx, DS * DS)
    known = (c != 255).sum(-1)
    nl = (c == 254).sum(-1)
    free = np.where(c <= 200, c, -1).max(-1)
    out = np.where(known < min_known, 255, np.where(nl >= 2, 254, np.maximum(free, 0)))
    return out.astype(np.uint8)


# ---------------------------------------------------------------- 打包
def build_package(struct_xy, all_pts, bounds_mm=None, step_max=100, slope_max_deg=20.0, min_count=3):
    """struct_xy (K,2)、all_pts (M,3)：地图系 mm。bounds_mm = (x0, y0, x1, y1) 场地范围（缺省取点云范围）。
    返回 (包字节, 调试信息 dict)"""
    if bounds_mm is None:
        bounds_mm = (all_pts[:, 0].min(), all_pts[:, 1].min(), all_pts[:, 0].max(), all_pts[:, 1].max())
    bx0, by0, bx1, by1 = bounds_mm
    c0x, c0y, c1x, c1y = int(cell_of(bx0)), int(cell_of(by0)), int(cell_of(bx1)), int(cell_of(by1))
    span = max(c1x - c0x + 1, c1y - c0y + 1)
    if span > SN:
        raise ValueError(f"场地 {span * RES_MM / 1000:.1f} m 超过片上结构图 {SN * RES_MM / 1000:.1f} m（需要 DDR 分块，尚未实现）")
    # 结构窗口居中放置（窗口外围留空，候选平移出界时硬件按标签判空）
    ox = c0x - (SN - (c1x - c0x + 1)) // 2
    oy = c0y - (SN - (c1y - c0y + 1)) // 2
    occ, cxs, cys, cnt = struct_cells_from_points(struct_xy, ox, oy, min_count=min_count)
    q, has, gate = prior_lf(occ, cxs, cys)
    slot, word = struct_words(ox, oy, occ, cxs, cys, q, has, gate)
    # 代价图：20 cm 格对齐全局 5 cm 格的 4 倍数
    gox, goy = c0x // DS - 1, c0y // DS - 1
    gnx, gny = c1x // DS + 2 - gox, c1y // DS + 2 - goy
    if gnx > GM_MAX or gny > GM_MAX:
        raise ValueError(f"代价图 {gnx}×{gny} 超过记忆栅格 {GM_MAX}×{GM_MAX}")
    h = elevation_from_points(all_pts, gox * DS, goy * DS, gnx * DS, gny * DS)
    cost5 = cost_from_elevation(h, step_max, slope_max_deg)
    gcost = coarse_cost(cost5)
    body = b"".join(struct.pack("<HI", int(s), int(w))[:5] for s, w in zip(slot, word)) + gcost.tobytes()
    hdr = struct.pack(HDR_FMT, MAGIC, 1, 0, ox, oy, len(slot), gox, goy, gnx, gny, zlib.crc32(body), len(body))
    info = dict(ox=ox, oy=oy, n_struct=int(occ.sum()), n_words=len(slot), gox=gox, goy=goy, gnx=gnx, gny=gny,
                occ=occ, cxs=cxs, cys=cys, q=q, has=has, gate=gate, cnt=cnt, h=h, cost5=cost5, gcost=gcost)
    return hdr + body, info


def parse_package(b):
    """-> dict（与 build_package 的 info 同名字段：ox oy gox goy gnx gny slot word gcost）"""
    magic, ver, flags, ox, oy, n, gox, goy, gnx, gny, crc, blen = struct.unpack(HDR_FMT, b[:HDR_LEN])
    body = b[HDR_LEN:HDR_LEN + blen]
    if magic != MAGIC or ver != 1 or len(body) != blen or zlib.crc32(body) != crc:
        raise ValueError("地图包损坏")
    e = np.frombuffer(body[:5 * n], dtype=np.uint8).reshape(n, 5).astype(np.int64)
    slot = e[:, 0] | (e[:, 1] << 8)
    word = e[:, 2] | (e[:, 3] << 8) | (e[:, 4] << 16)
    gcost = np.frombuffer(body[5 * n:], dtype=np.uint8).reshape(gny, gnx)
    return dict(ox=ox, oy=oy, gox=gox, goy=goy, gnx=gnx, gny=gny, slot=slot, word=word, gcost=gcost)


# ---------------------------------------------------------------- 仿真：模拟"赛前扫图"
def survey_scan(scene, lidar, mount, spacing=1.0, yaws=6, pose_noise=(0.01, 0.2), seed=3, zcap=1.5):
    """在场景可通行区按网格布点、每点转 yaws 个朝向采集深度帧（模拟赛前推着车扫图 + Linux SLAM 得到的位姿）。
    pose_noise = (平移 σ m, 航向 σ °)：SLAM 位姿误差。返回 (结构点 (K,2) mm, 全部点 (M,3) mm, 位姿列表)"""
    import lidar_lut
    import nav_fast as fast
    import sim_world as sw
    rng = np.random.default_rng(seed)
    lut = np.array(lidar_lut.lut_entries(), dtype=np.int64)
    Rbs, tbs = mount
    x0, y0, x1, y1 = scene.bounds
    xs = np.arange(x0 + spacing / 2, x1, spacing)
    ys = np.arange(y0 + spacing / 2, y1, spacing)
    sp, ap, poses = [], [], []
    for yy in ys:
        for xx in xs:
            if int(scene.gt_lookup(np.array([xx]), np.array([yy]))[0]) != 0:
                continue
            z = float(scene.height(np.array([xx]), np.array([yy]))[0])
            for k in range(yaws):
                yaw = 2 * math.pi * k / yaws + rng.uniform(-0.2, 0.2)
                Rwb, twb = sw.rotz(yaw), np.array([xx, yy, z])
                Rws, tws = sw.compose(Rwb, twb, Rbs, tbs)
                depth, _ = lidar.capture(scene.w, Rws, tws)
                # SLAM 位姿（带误差）
                ex, ey = rng.normal(0, pose_noise[0], 2)
                ea = math.radians(rng.normal(0, pose_noise[1]))
                Rwb_e, twb_e = sw.rotz(yaw + ea), np.array([xx + ex, yy + ey, z])
                Rws_e, tws_e = sw.compose(Rwb_e, twb_e, Rbs, tbs)
                xyz, pv = fast.pc_project(depth, lut)
                R = np.round(Rws_e * 16384).astype(np.int64)
                t = np.round(tws_e * 1000).astype(np.int64)
                p = fast.pc_transform(xyz, R, t)
                m = slf.structure_mask(p, pv, int(t[2]) - int(round(tbs[2] * 1000)), slf.Z_LO_PRIOR)
                sp.append(p[m, :2])
                ap.append(p[pv & (p[:, 2] < (z + zcap) * 1000)])
                poses.append((xx, yy, yaw))
    return np.concatenate(sp), np.concatenate(ap), poses
