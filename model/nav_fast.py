"""感知链路的向量化实现（numpy），与 nav_fixed.py 的逐位模型结果完全相同，只是快两个数量级。
闭环仿真用它；tests/test_nav_fast.py 保证两者一致（nav_fixed 又与 RTL 逐位一致）。

所有函数都以整数运算实现，舍入方式与 RTL 相同。
"""
import numpy as np

import nav_fixed as nf

N_LOG2 = nf.N_LOG2
N = nf.N
TAGB = nf.TAGB


def _sat16(a):
    return np.clip(a, -32768, 32767)


# ------------------------------------------------------------------ 点云
def pc_project(depth, lut, depth_mul=256, min_mm=100, max_mm=20000, lut_frac=13):
    """depth: (H, W) 原始深度；lut: (H*W, 3) int。返回 xyz (M,3) int64、pv (M,) bool（行优先）。"""
    raw = depth.reshape(-1).astype(np.int64)
    dmm = np.minimum((raw * depth_mul) >> 8, 32767)
    pv = (raw != 0) & (dmm >= min_mm) & (dmm <= max_mm)
    xyz = _sat16((dmm[:, None] * lut.astype(np.int64)) >> lut_frac)
    return xyz, pv


def pc_transform(xyz, R, t, rot_frac=14):
    R = np.asarray(R, dtype=np.int64)
    a = xyz @ R.T                               # 整数矩阵乘
    return (a >> rot_frac) + np.asarray(t, dtype=np.int64)[None, :]


def world_cell(v, inv_res):
    return (np.clip(v, -(1 << 25), (1 << 25) - 1) * inv_res) >> 16


def pc_filter(p, pv, inv_res, cx, cy, zmin, zmax):
    """返回保留点的 (slot 地址, 标签, z16)。"""
    ix = world_cell(p[:, 0], inv_res)
    iy = world_cell(p[:, 1], inv_res)
    dx = ix - (cx - N // 2)
    dy = iy - (cy - N // 2)
    keep = pv & (p[:, 2] >= zmin) & (p[:, 2] <= zmax) & (dx >= 0) & (dx < N) & (dy >= 0) & (dy < N)
    ix, iy = ix[keep], iy[keep]
    addr = ((iy & (N - 1)) << N_LOG2) | (ix & (N - 1))
    tag = (((iy >> N_LOG2) & ((1 << TAGB) - 1)) << TAGB) | ((ix >> N_LOG2) & ((1 << TAGB) - 1))
    z16 = ((p[keep, 2] + 32768) & 0xFFFF) - 32768
    return addr, tag, z16


# ---------------------------------------------------------------- 高程图
def elev_update(em, addr, tag, z):
    """一帧的全部更新（与 ElevMap.update 对一帧数据的结果相同）：帧内被触及的槽用本帧观测替换。"""
    if len(addr):
        order = np.argsort(addr, kind="stable")
        a, tg, zz = addr[order], tag[order], z[order]
        u, start, cnt = np.unique(a, return_index=True, return_counts=True)
        zmax = np.maximum.reduceat(zz, start)
        zmin = np.minimum.reduceat(zz, start)
        em.zmax[u] = zmax
        em.zmin[u] = zmin
        em.cnt[u] = np.minimum(cnt, 15)
        em.tag[u] = tg[start]
        em.stamp[u] = em.fc
    em.fc = (em.fc + 1) & 0xFF


# ------------------------------------------------------------ 窗口视图
def window_view(em, cx, cy, cfg, kill=True):
    """把槽存储重排为窗口坐标 (N,N) 数组，并执行过期清除。返回 slot(N,N), valid, h, rough。"""
    w = np.arange(N)
    gx = cx - N // 2 + w
    gy = cy - N // 2 + w
    sx, tx = gx & (N - 1), (gx >> N_LOG2) & ((1 << TAGB) - 1)
    sy, ty = gy & (N - 1), (gy >> N_LOG2) & ((1 << TAGB) - 1)
    slot = (sy[:, None] << N_LOG2) | sx[None, :]
    etag = (ty[:, None] << TAGB) | tx[None, :]
    cnt = em.cnt[slot]
    age = (em.fc - em.stamp[slot]) & 0xFF
    fresh = age <= cfg["age_max"]
    kr = cfg.get("keep_r", 0)
    if kr:
        # 近区不过期：机器人周围（窗口中心 ±keep_r 格）落在雷达最小量程/俯视盲区内，
        # 贴近障碍时它再也看不见——按时间过期会让它从图上消失，机器人就会开上去
        d = np.abs(w - N // 2)
        fresh = fresh | ((d[:, None] <= kr) & (d[None, :] <= kr))
    live = (cnt != 0) & (em.tag[slot] == etag) & fresh
    if kill:
        dead = (cnt != 0) & ~live
        em.cnt[slot[dead]] = 0
    valid = live & (cnt >= cfg["cnt_min"])
    h = em.zmax[slot].astype(np.int64)
    rough = (em.zmax[slot] - em.zmin[slot]).astype(np.int64)
    return slot, valid, h, rough


def _shift(a, dy, dx, fill):
    """out[y, x] = a[y+dy, x+dx]，越界填 fill。"""
    out = np.full_like(a, fill)
    ys = slice(max(0, -dy), N - max(0, dy))
    xs = slice(max(0, -dx), N - max(0, dx))
    yd = slice(max(0, dy), N - max(0, -dy))
    xd = slice(max(0, dx), N - max(0, -dx))
    out[ys, xs] = a[yd, xd]
    return out


def _dilate(m, r):
    if r <= 0:
        return m.copy()
    out = np.zeros_like(m)
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            out |= _shift(m, dy, dx, False)
    return out


# ------------------------------------------------------ 可通行性 v1（现 RTL）
def trav_v1(em, cx, cy, cfg):
    """与 nav_fixed.trav_analysis 相同。返回 (cost 按槽地址, lethal 窗口坐标, 窗口视图)。"""
    slot, V, H, RO = window_view(em, cx, cy, cfg)
    nb = {}
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dy or dx:
                nb[(dy, dx)] = (_shift(V, dy, dx, False), _shift(H, dy, dx, 0))
    step = np.zeros((N, N), dtype=np.int64)
    for (dy, dx), (v, h) in nb.items():
        step = np.maximum(step, np.where(v, np.abs(h - H), 0))

    def grad(a, b):
        (va, ha), (vb, hb) = a, b
        return np.where(va & vb, hb - ha, np.where(vb, 2 * (hb - H), np.where(va, 2 * (H - ha), 0)))

    gx = np.abs(grad(nb[(0, -1)], nb[(0, 1)]))
    gy = np.abs(grad(nb[(-1, 0)], nb[(1, 0)]))
    slope = np.maximum(gx, gy) + (np.minimum(gx, gy) >> 1)
    lethal = V & ((step > cfg["step_max"]) | (RO > cfg["rough_max"]) | (slope > cfg["slope_max"]))
    s = (cfg["k_slope"] * np.minimum(slope, 65535) + cfg["k_step"] * np.minimum(step, 65535)
         + cfg["k_rough"] * np.minimum(RO, 65535))
    raw = np.where(~V, nf.COST_UNKNOWN, np.where(lethal, nf.COST_LETHAL, np.minimum(nf.COST_FREE_MAX, s >> 8)))
    infl = _dilate(lethal, cfg["infl_r"])
    raw = np.where((raw < nf.COST_INSCRIBED) & infl, nf.COST_INSCRIBED, raw)
    cost = np.full(N * N, nf.COST_UNKNOWN, dtype=np.int64)
    cost[slot.reshape(-1)] = raw.reshape(-1)
    return cost, lethal, (slot, V, H, RO)


def cost_window(cost, cx, cy):
    """按槽地址的代价图 -> 窗口坐标 (N,N)。"""
    w = np.arange(N)
    sx = (cx - N // 2 + w) & (N - 1)
    sy = (cy - N // 2 + w) & (N - 1)
    return cost[(sy[:, None] << N_LOG2) | sx[None, :]]
