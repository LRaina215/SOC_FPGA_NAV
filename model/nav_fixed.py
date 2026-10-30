"""导航流水线定点逐位模型（与 RTL 完全一致的整数运算）。

每个函数对应一个 RTL 模块，RTL 测试台以这里的输出为标准答案逐项比对。
改动 RTL 的运算（位宽、舍入、判定顺序）时必须同步改这里，反之亦然。
"""
import math

import numpy as np

N_LOG2 = 7
N = 1 << N_LOG2
TAGB = 4
COST_FREE_MAX = 200
COST_INSCRIBED = 253
COST_LETHAL = 254
COST_UNKNOWN = 255


def sat16(a):
    return max(-32768, min(32767, a))


def floor_shift(a, s):
    return a >> s          # Python 的 >> 对负数即向下取整，与 Verilog >>> 一致


# ---------------------------------------------------------------- pc_project
def pc_project(pix, lut, depth_mul=256, min_mm=100, max_mm=20000, roi=(0, 0), w=160, h=60, lut_frac=11, binning=0):
    """pix: [(u, v, raw, sof, eof)] -> [(x, y, z, pv, sof, eof)]；lut[i] = (rx, ry, rz)（12 位 Q0.11，全零 = 无效）。
    binning: bit1 水平合并、bit0 竖直合并（查表坐标左移一位）。"""
    out = []
    for u, v, raw, sof, eof in pix:
        dmm = min((raw * depth_mul) >> 8, 32767)
        uu = (u << ((binning >> 1) & 1)) + roi[0]
        vv = (v << (binning & 1)) + roi[1]
        inr = uu < w and vv < h
        rx, ry, rz = lut[vv * w + uu] if inr else lut[0]
        ok = raw != 0 and inr and (rx, ry, rz) != (0, 0, 0) and min_mm <= dmm <= max_mm
        out.append((sat16(floor_shift(dmm * rx, lut_frac)), sat16(floor_shift(dmm * ry, lut_frac)),
                    sat16(floor_shift(dmm * rz, lut_frac)), int(ok), sof, eof))
    return out


# -------------------------------------------------------------- pc_transform
def pc_transform(pts, R, t, rot_frac=14):
    """R: 3x3 整数 Q1.14，t: (tx, ty, tz) mm。每帧（sof）锁存，这里按调用方传入的固定值。"""
    out = []
    for x, y, z, pv, sof, eof in pts:
        ax = R[0][0] * x + R[0][1] * y + R[0][2] * z
        ay = R[1][0] * x + R[1][1] * y + R[1][2] * z
        az = R[2][0] * x + R[2][1] * y + R[2][2] * z
        out.append((floor_shift(ax, rot_frac) + t[0], floor_shift(ay, rot_frac) + t[1],
                    floor_shift(az, rot_frac) + t[2], pv, sof, eof))
    return out


# ----------------------------------------------------------------- pc_filter
def clip26(a):
    return max(-(1 << 25), min((1 << 25) - 1, a))


def world_cell(x, inv_res):
    return (clip26(x) * inv_res) >> 16


def pc_filter(pts, inv_res, cx, cy, zmin, zmax):
    """-> [(upd, addr, tag, z, eof)]，与 RTL 输出节拍一致（被剔除且非帧尾的点不输出）。"""
    out = []
    for x, y, z, pv, sof, eof in pts:
        ix, iy = world_cell(x, inv_res), world_cell(y, inv_res)
        dx, dy = ix - (cx - N // 2), iy - (cy - N // 2)
        inwin = 0 <= dx < N and 0 <= dy < N
        keep = pv and zmin <= z <= zmax and inwin
        if keep or eof:
            addr = ((iy & (N - 1)) << N_LOG2) | (ix & (N - 1))
            tag = (((iy >> N_LOG2) & ((1 << TAGB) - 1)) << TAGB) | ((ix >> N_LOG2) & ((1 << TAGB) - 1))
            z16 = ((z + 32768) & 0xFFFF) - 32768
            out.append((int(keep), addr, tag, z16, eof))
    return out


# ------------------------------------------------------------------ elev_map
AVG_ZLIM = 4095        # 平均模式：点高度先限幅到 ±4095 mm，再以 Q3（1/8 mm）参与平均


def avg_shift(n, shift_max):
    """平均模式下第 n+1 个点（n = 已有点数 1..15）的权重 2^-s：前几点近似算术平均，计满 15 点后取 shift_max。"""
    s = 1 if n == 1 else 2 if n <= 3 else 3 if n <= 7 else 4
    return shift_max if n == 15 else min(s, shift_max)


def avg_point(a, d, n, z, shift_max):
    """平均模式单点更新：a = Q3 高度均值，d = Q3 平均绝对偏差，n = 已有点数。返回 (a', d')。"""
    zq = max(-AVG_ZLIM, min(AVG_ZLIM, z)) * 8
    s = avg_shift(n, shift_max)
    diff = zq - a
    e = abs(diff) - d
    return a + (diff >> s), d + (e >> s)


class ElevMap:
    """每槽: zmax,zmin,cnt4,tag8,stamp8,retained_stale1。

    avg=0（旧模式）：zmax/zmin 是本帧点高度极值（mm），cnt 是本帧点数，每帧第一次观测时重置。
    avg=1（平均模式）：zmax 位置存 Q3 高度均值（有符号），zmin 位置存 Q3 平均绝对偏差（无符号），
    cnt 是累计点数（饱和 15）；只有槽空 / 标签不符 / 已标记过期时才重置，跨帧持续平均。"""

    def __init__(self, avg=0, avg_shift_max=3):
        self.zmax = np.zeros(N * N, dtype=np.int64)
        self.zmin = np.zeros(N * N, dtype=np.int64)
        self.cnt = np.zeros(N * N, dtype=np.int64)
        self.tag = np.zeros(N * N, dtype=np.int64)
        self.stamp = np.zeros(N * N, dtype=np.int64)
        self.stale = np.zeros(N * N, dtype=bool)
        self.fc = 0
        self.avg = avg
        self.avg_shift_max = avg_shift_max

    def update(self, upds):
        """输入 pc_filter 的输出（一帧或多帧），帧尾 fc 加 1。"""
        for upd, addr, tag, z, eof in upds:
            if upd:
                reset = self.stale[addr] or self.cnt[addr] == 0 or self.tag[addr] != tag
                if self.avg:
                    if reset:
                        self.zmax[addr] = max(-AVG_ZLIM, min(AVG_ZLIM, z)) * 8
                        self.zmin[addr] = 0
                        self.cnt[addr] = 1
                    else:
                        self.zmax[addr], self.zmin[addr] = avg_point(int(self.zmax[addr]), int(self.zmin[addr]),
                                                                     int(self.cnt[addr]), z, self.avg_shift_max)
                        self.cnt[addr] = min(self.cnt[addr] + 1, 15)
                elif reset or self.stamp[addr] != self.fc:
                    self.zmax[addr] = z
                    self.zmin[addr] = z
                    self.cnt[addr] = 1
                else:
                    self.zmax[addr] = max(self.zmax[addr], z)
                    self.zmin[addr] = min(self.zmin[addr], z)
                    self.cnt[addr] = min(self.cnt[addr] + 1, 15)
                self.tag[addr] = tag
                self.stamp[addr] = self.fc
                self.stale[addr] = False
            if eof:
                self.fc = (self.fc + 1) & 0xFF

    def word(self, a):
        """53bit RTL字：{retained_stale,stamp,tag,cnt,zmin,zmax}。"""
        return ((int(self.stale[a]) << 52) | (int(self.stamp[a]) << 44) | (int(self.tag[a]) << 36) | (int(self.cnt[a]) << 32)
                | ((int(self.zmin[a]) & 0xFFFF) << 16) | (int(self.zmax[a]) & 0xFFFF))


# ------------------------------------------------------------- trav_analysis
def slot_of(w, c):
    """窗口坐标 w (0..N-1) -> 槽坐标与期望标签。"""
    g = c - N // 2 + w
    return g & (N - 1), (g >> N_LOG2) & ((1 << TAGB) - 1)


def trav_analysis(em, cx, cy, cfg):
    """返回 (cost[N*N] 按槽地址, lethal 位图[N*N] 按窗口坐标, 过期清除后的 em)。
    cfg: age_max cnt_min step_max rough_max slope_max k_slope k_step k_rough infl_r
    """
    fc = em.fc
    # 读出窗口内每格的派生值（并执行过期清除，与 RTL 一样在第一遍读的同时写回）
    valid = np.zeros((N, N), dtype=bool)
    h = np.zeros((N, N), dtype=np.int64)
    rough = np.zeros((N, N), dtype=np.int64)
    slot = np.zeros((N, N), dtype=np.int64)
    for wy in range(N):
        sy, ty = slot_of(wy, cy)
        for wx in range(N):
            sx, tx = slot_of(wx, cx)
            a = (sy << N_LOG2) | sx
            slot[wy, wx] = a
            tag_ok = em.tag[a] == ((ty << TAGB) | tx)
            age = (fc - int(em.stamp[a])) & 0xFF
            live = em.cnt[a] != 0 and tag_ok and not em.stale[a] and age <= cfg["age_max"]
            if em.cnt[a] != 0 and not live:
                em.cnt[a] = 0
            valid[wy, wx] = live and em.cnt[a] >= cfg["cnt_min"]
            h[wy, wx] = em.zmax[a]
            rough[wy, wx] = em.zmax[a] - em.zmin[a]

    cost = np.full(N * N, COST_UNKNOWN, dtype=np.int64)
    lethal = np.zeros((N, N), dtype=np.int64)
    rawc = np.zeros((N, N), dtype=np.int64)
    for wy in range(N):
        for wx in range(N):
            if not valid[wy, wx]:
                rawc[wy, wx] = COST_UNKNOWN
                continue
            hc = h[wy, wx]

            def nb(dy, dx):
                yy, xx = wy + dy, wx + dx
                if 0 <= yy < N and 0 <= xx < N and valid[yy, xx]:
                    return True, h[yy, xx]
                return False, 0

            step = 0
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dy == 0 and dx == 0:
                        continue
                    ok, hn = nb(dy, dx)
                    if ok:
                        step = max(step, abs(hn - hc))

            def grad(a, b):     # a = 负方向邻居, b = 正方向邻居
                (va, ha), (vb, hb) = a, b
                if va and vb:
                    return hb - ha
                if vb:
                    return 2 * (hb - hc)
                if va:
                    return 2 * (hc - ha)
                return 0

            gx = abs(grad(nb(0, -1), nb(0, 1)))
            gy = abs(grad(nb(-1, 0), nb(1, 0)))
            slope = max(gx, gy) + (min(gx, gy) >> 1)
            r = rough[wy, wx]
            if step > cfg["step_max"] or r > cfg["rough_max"] or slope > cfg["slope_max"]:
                rawc[wy, wx] = COST_LETHAL
                lethal[wy, wx] = 1
            else:
                s = cfg["k_slope"] * min(slope, 65535) + cfg["k_step"] * min(step, 65535) + cfg["k_rough"] * min(r, 65535)
                rawc[wy, wx] = min(COST_FREE_MAX, s >> 8)

    # 方形膨胀（半径 R 格）
    R = cfg["infl_r"]
    if R > 0:
        from numpy.lib.stride_tricks import sliding_window_view
        pad = np.pad(lethal, R)
        infl = sliding_window_view(pad, (2 * R + 1, 2 * R + 1)).max(axis=(2, 3))
    else:
        infl = lethal
    for wy in range(N):
        for wx in range(N):
            c = rawc[wy, wx]
            if c < COST_INSCRIBED and infl[wy, wx]:
                c = COST_INSCRIBED
            cost[slot[wy, wx]] = c
    return cost, lethal


# ----------------------------------------------------------------- traj_eval
SIN_LOG2 = 10
SIN_FRAC = 14


def sin_table():
    return [int(round(math.sin(2 * math.pi * i / (1 << SIN_LOG2)) * (1 << SIN_FRAC))) for i in range(1 << SIN_LOG2)]


SIN = sin_table()


def traj_eval(cost, cx, cy, x0, y0, th0, cands, T, cmax=COST_INSCRIBED):
    """cost 按槽地址；cands: [(dvx, dvy, dth)]（格 Q8/步、65536/周/步）。
    返回每条: (max_cost, n_unknown, coll_step, collide, left, sum_cost, end_x, end_y)
    """
    res = []
    for dvx, dvy, dth in cands:
        x, y, th = x0, y0, th0 & 0xFFFF
        mx = nun = csum = 0
        coll = left = 0
        cstep = 0
        for t in range(1, T + 1):
            th = (th + dth) & 0xFFFF
            i = th >> (16 - SIN_LOG2)
            s, c = SIN[i], SIN[(i + (1 << (SIN_LOG2 - 2))) & ((1 << SIN_LOG2) - 1)]
            x += (dvx * c - dvy * s) >> SIN_FRAC
            y += (dvx * s + dvy * c) >> SIN_FRAC
            x = ((x + (1 << 31)) & 0xFFFFFFFF) - (1 << 31)
            y = ((y + (1 << 31)) & 0xFFFFFFFF) - (1 << 31)
            ix, iy = x >> 8, y >> 8
            wx, wy = ix - (cx - N // 2), iy - (cy - N // 2)
            if not (0 <= wx < N and 0 <= wy < N):
                left = 1
                nun = min(nun + 1, 255)
                continue
            cc = int(cost[((iy & (N - 1)) << N_LOG2) | (ix & (N - 1))])
            if cc == COST_UNKNOWN:
                nun = min(nun + 1, 255)
            elif cc >= cmax:
                coll, cstep = 1, t
                break
            else:
                csum += cc
                mx = max(mx, cc)
        res.append((mx, nun, cstep, coll, left, csum, x, y))
    return res
