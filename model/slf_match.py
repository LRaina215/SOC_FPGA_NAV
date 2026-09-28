"""结构似然场匹配（Structure Likelihood Field, SLF）：FPGA 匹配定位的定点模型。

动机：EICM（高程区间一致性）把地面点也拿来比，地面在 2.5D 图里对水平位移几乎不敏感，却对姿态噪声很敏感，
实测有约 1 个候选步长的系统偏差。水平定位真正的约束来自竖直结构（墙、箱体侧面、路沿立面）。

三步，全部是流式整数运算：
1. 结构点提取（深度图域，逐像素）：同一列上下相邻两像素变换到里程计系后，
   若连线陡于 ~63°（|Δz| > 2·|Δxy|，且 |Δz| >= DZ_MIN）则两点都是"结构点"。
   这就是 LeGO-LOAM 地面分割用的"相邻线束夹角"的反面，只需一行点的行缓存。
   10° 坡道、平台顶面是平的，不会被选中；平台侧面、墙、箱子、路沿立面会。
2. 结构图（与高程图同样的滚动寻址，256×256、5 cm）：每格命中数（每帧至多 +1）+ 格内亚格质心
   （1/16 格，前 HIT_FREEZE 次累计平均后冻结）。命中数 >= HIT_MIN 的格为结构格。建"似然场"时再算：
   距离场（八边形距离，只用于门控）与每格法向（5×5 邻域质心协方差的主方向转 90°）。
3. 匹配打分：候选增量 (dx, dy, dyaw)（绕机器人位置旋转后平移），变换后的结构点取 1/16 格亚格坐标，
   代价 = 3×3 邻域结构格的最小点到线法向距离（上限 C_NEAR）；只用预测位置 GATE_D 以内有已知结构的点。
   选择这种形式的原因与被否决的做法见 docs/algorithms.md 第 6 节。
"""
import math
import os

import numpy as np

import nav_fixed as nf

# 实验开关（环境变量）：结构图边长、近邻形式（"3x3" 或 "2x2" = 点所在象限的 2×2 格，硬件 4 体交织一拍读完）
SN_LOG2 = int(os.environ.get("SLF_SN_LOG2", "8"))
NBR = os.environ.get("SLF_NBR", "3x3")
SN = 1 << SN_LOG2            # 结构图边长
TAGB = 4
DZ_MIN = 20                  # mm：竖直段最小高差（抑制噪声）
SLOPE_K = 2                  # |Δz| > K·|Δxy|
Z_LO, Z_HI = 60, 1500        # 相对机器人高度：只用离地 6 cm 以上、1.5 m 以下的点（去掉地面接缝与天花板）
HIT_MIN = 2
HIT_FREEZE = 4               # 质心累计平均的次数，之后冻结
HIT_MAX = 7                  # 命中数 3 bit
DMAX = 8                     # 截断距离（格）
C_NEAR = 40                  # 近邻代价上限（1/16 格，2.5 格）
W_FAR = 0                    # 近邻区以外代价为常数：远处的点不施加拉力（加梯度后闭环平均误差反而变大，见 algorithms.md）
GATE_D = 6                   # 参与打分的点：预测位置的距离场 <= 2 格（更远的多是新看到的结构）
MARGIN = 8                   # 格：窗口边缘保留带（> 搜索范围 + 1）
PB_MAX = 4096                # 点缓冲容量
MAX_PTS = 1024               # 每帧参与匹配的结构点上限（均匀抽样）
W = 160


def structure_mask(p, pv, z_ref):
    """p: (M,3) 里程计系点（行优先 60×160），pv 有效标志。返回结构点布尔掩码"""
    m = p.shape[0]
    up, dn = np.arange(0, m - W), np.arange(W, m)
    v = pv[up] & pv[dn]
    dz = np.abs(p[dn, 2] - p[up, 2])
    dxy = np.maximum(np.abs(p[dn, 0] - p[up, 0]), np.abs(p[dn, 1] - p[up, 1]))
    vert = v & (dz >= DZ_MIN) & (dz > SLOPE_K * dxy)
    s = np.zeros(m, dtype=bool)
    s[up] |= vert
    s[dn] |= vert
    zr = p[:, 2] - z_ref
    return s & pv & (zr >= Z_LO) & (zr <= Z_HI)


def subsample(idx, n=MAX_PTS):
    """硬件：点缓冲最多存 PB_MAX 个结构点（按像素顺序，多余的丢弃），打分/并图时按步长 ceil(存量/n) 取点"""
    idx = idx[:PB_MAX]
    if len(idx) <= n:
        return idx
    step = (len(idx) + n - 1) // n
    return idx[::step]


class StructMap:
    def __init__(self, inv_res=1311):
        self.inv_res = inv_res
        self.hit = np.zeros(SN * SN, dtype=np.int64)
        self.tag = np.zeros(SN * SN, dtype=np.int64)
        self.cxs = np.zeros(SN * SN, dtype=np.int64)     # 格内质心（1/16 格）
        self.cys = np.zeros(SN * SN, dtype=np.int64)
        self.lf = None           # (ox, oy, occ, d3, cxs, cys) 窗口视图
        self.n_struct = 0

    def words(self):
        """硬件结构图槽字（23 位）：{tag, hit, cx, cy, occ, q, has, gate}；窗口外的槽只有前 15 位有意义（返回 mask）"""
        w = (self.tag << 19) | (self.hit << 16) | (self.cxs << 12) | (self.cys << 8)
        mask = np.full(SN * SN, 0x7FFF00, dtype=np.int64)
        if self.lf is not None:
            ox, oy, occ, d3, cxs, cys, nx, ny, has = self.lf
            s = self.lf_slot.reshape(-1)
            lo = (occ.astype(np.int64) << 7) | (self.lf_q << 2) | (has.astype(np.int64) << 1) | (d3 <= GATE_D)
            w[s] |= lo.reshape(-1)
            mask[s] = 0x7FFFFF
        return w, mask

    def cell(self, v):
        return (np.clip(v, -(1 << 25), (1 << 25) - 1) * self.inv_res) >> 16

    def update(self, pts_xy):
        """pts_xy: (K,2) 里程计系 mm（修正后位姿下的结构点，按像素顺序）。
        每格：命中数（3 bit，每帧至多 +1）+ 格内亚格质心（1/16 格）。一帧内同一格只取第一个点（流式硬件不必做帧内平均）；
        质心：前 HIT_FREEZE 次观测累计平均，之后冻结"""
        if len(pts_xy) == 0:
            return
        fx, fy = self.cell16(pts_xy[:, 0]), self.cell16(pts_xy[:, 1])
        ix, iy = fx >> 4, fy >> 4
        a = ((iy & (SN - 1)) << SN_LOG2) | (ix & (SN - 1))
        tg = (((iy >> SN_LOG2) & 3) << 2) | ((ix >> SN_LOG2) & 3)
        u, first = np.unique(a, return_index=True)
        mx, my, tg = (fx & 15)[first], (fy & 15)[first], tg[first]
        fresh = (self.tag[u] != tg) | (self.hit[u] == 0)
        h0 = np.where(fresh, 0, self.hit[u])
        upd = h0 < HIT_FREEZE
        n1 = h0 + 1
        # 累计平均：c += round((m - c) / n)（向下取整的除法，硬件用 n∈{2,3,4} 的常数除法）
        self.cxs[u] = np.where(fresh, mx, np.where(upd, self.cxs[u] + np.floor_divide(mx - self.cxs[u] + n1 // 2, n1), self.cxs[u]))
        self.cys[u] = np.where(fresh, my, np.where(upd, self.cys[u] + np.floor_divide(my - self.cys[u] + n1 // 2, n1), self.cys[u]))
        self.tag[u] = tg
        self.hit[u] = np.minimum(h0 + 1, HIT_MAX)

    def build_lf(self, cx, cy):
        """以 (cx, cy)（高程图窗口中心格）为中心的 SN×SN 窗口：结构格 -> 截断切比雪夫距离"""
        w = np.arange(SN)
        gx = cx - SN // 2 + w
        gy = cy - SN // 2 + w
        slot = ((gy[:, None] & (SN - 1)) << SN_LOG2) | (gx[None, :] & (SN - 1))
        etag = (((gy[:, None] >> SN_LOG2) & 3) << 2) | ((gx[None, :] >> SN_LOG2) & 3)
        occ = (self.tag[slot] == etag) & (self.hit[slot] >= HIT_MIN)
        self.n_struct = int(occ.sum())
        d = np.minimum(2 * chebyshev_dt(occ, DMAX) + taxicab_dt(occ, 3 * DMAX), 3 * DMAX)
        cxs, cys = self.cxs[slot], self.cys[slot]
        nx, ny, has, q = cell_normals(occ, cxs, cys)
        self.lf = (cx - SN // 2, cy - SN // 2, occ, d, cxs, cys, nx, ny, has)
        self.lf_q = q
        self.lf_slot = slot

    def score(self, px, py, cands, cx_mm, cy_mm):
        """px, py: 结构点（预测位姿下，mm）；cands: (K,3) dx dy dyaw。返回 (K,4) n_valid cost n_in 0"""
        out = np.zeros((len(cands), 4), dtype=np.int64)
        if self.lf is None or len(px) == 0:
            return out
        ox, oy, occ, d3, cxs, cys, nx, ny, has = self.lf
        # 只用预测位置离窗口边界 >= MARGIN 格的点：否则候选平移会把窗外的点"移进"窗内而降低代价（边界偏差）
        wx0, wy0 = self.cell(px) - ox, self.cell(py) - oy
        core = (wx0 >= MARGIN) & (wx0 < SN - MARGIN) & (wy0 >= MARGIN) & (wy0 < SN - MARGIN)
        # 只用预测位置 GATE_D（1/3 格）以内已有结构的点：新看到的结构（面向前延伸的部分、新出现的物体）
        # 离已知结构远，若参与打分会把位姿往"已知区"拉——沿运动方向的系统偏差（离线测试中稳定 -25 mm）
        near = np.zeros_like(core)
        near[core] = d3[wy0[core], wx0[core]] <= GATE_D
        px, py = px[near], py[near]
        for k, (dx, dy, da) in enumerate(cands):
            x, y = apply_delta(px, py, dx, dy, da, cx_mm, cy_mm)
            out[k] = (len(px),) + point_cost(self.cell16(x) - 16 * ox, self.cell16(y) - 16 * oy, occ, d3, cxs, cys,
                                             nx, ny, has)
        return out

    def cell16(self, v):
        return (np.clip(v, -(1 << 25), (1 << 25) - 1) * self.inv_res) >> 12


def apply_delta(px, py, dx, dy, da, cx_mm, cy_mm):
    """p' = Rz(da)·(p - c) + c + (dx, dy)（正弦表 1024 项，角度四舍五入到表格点：向下取整会让负角多转一格）"""
    i = ((int(da) + 32) & 0xFFFF) >> 6
    s, c = nf.SIN[i], nf.SIN[(i + 256) & 1023]
    rx, ry = px - cx_mm, py - cy_mm
    return cx_mm + ((rx * c - ry * s) >> 14) + dx, cy_mm + ((rx * s + ry * c) >> 14) + dy


CORDIC_ATAN = [8192, 4836, 2555, 1297, 651, 326, 163, 81, 41, 20]
NVEC = [(int(round(math.cos(q * math.pi / 32) * 128)), int(round(math.sin(q * math.pi / 32) * 128))) for q in range(32)]


def sector32(X, Y):
    """向量 (X, Y) 的方向量化为 32 个扇区（每扇区 11.25°，扇区 0 以 -180° 为中心）。
    与硬件相同的整数 CORDIC：先整体右移到 17 位以内，再 10 次迭代；角度单位 65536/周。"""
    X = X.astype(np.int64).copy()
    Y = Y.astype(np.int64).copy()
    m = np.maximum(np.abs(X), np.abs(Y))
    sh = np.maximum(0, np.floor(np.log2(np.maximum(m, 1))).astype(np.int64) + 1 - 17)
    X >>= sh
    Y >>= sh
    a = np.zeros_like(X)
    neg = X < 0
    X = np.where(neg, -X, X)
    Y = np.where(neg, -Y, Y)
    a = np.where(neg, 32768, 0)
    for i, t in enumerate(CORDIC_ATAN):
        up = Y > 0
        nx = np.where(up, X + (Y >> i), X - (Y >> i))
        ny = np.where(up, Y - (X >> i), Y + (X >> i))
        a = np.where(up, a + t, a - t)
        X, Y = nx, ny
    return (((a + 32768 + 1024) & 0xFFFF) >> 11) & 31


def cell_normals(occ, cxs, cys):
    """每个结构格的法向：5×5 邻域内结构格质心的主方向，法向 = 主方向转 90°，量化为 32 个方向（Q7 单位向量）。
    全整数：坐标取相对中心格的 1/16 格（16·dx + 质心），A = nΣxx − (Σx)²、B = nΣxy − ΣxΣy、C = nΣyy − (Σy)²（= n²·协方差），
    主方向角 = ½·atan2(2B, A − C)，法向扇区 q = sector32(A − C, 2B)（法向角 = q·π/32）。
    线状判定（主方向明显）：25·((A − C)² + 4B²) > 9·(A + C)²；邻域结构格 < 3 个时无法向（按点到质心 L1 计）。
    硬件：建似然场的一遍扫描里 5×5 窗口累加，CORDIC 求扇区。"""
    n = occ.shape[0]
    o = occ.astype(np.int64)
    S = {k: np.zeros((n, n), dtype=np.int64) for k in ("n", "x", "y", "xx", "xy", "yy")}
    for dy in range(-2, 3):
        for dx in range(-2, 3):
            def sh(a):
                out = np.zeros_like(a)
                ys, yd = slice(max(0, -dy), n - max(0, dy)), slice(max(0, dy), n - max(0, -dy))
                xs, xd = slice(max(0, -dx), n - max(0, dx)), slice(max(0, dx), n - max(0, -dx))
                out[ys, xs] = a[yd, xd]
                return out
            w = sh(o)
            X = (16 * dx + sh(cxs)) * w
            Y = (16 * dy + sh(cys)) * w
            S["n"] += w; S["x"] += X; S["y"] += Y
            S["xx"] += X * X; S["xy"] += X * Y; S["yy"] += Y * Y
    A = S["n"] * S["xx"] - S["x"] * S["x"]
    B = S["n"] * S["xy"] - S["x"] * S["y"]
    C = S["n"] * S["yy"] - S["y"] * S["y"]
    aniso = 25 * ((A - C) ** 2 + 4 * B * B) > 9 * (A + C) ** 2
    q = sector32(A - C, 2 * B)
    nv = np.array(NVEC, dtype=np.int64)
    has = occ & (S["n"] >= 3) & aniso
    return nv[q, 0], nv[q, 1], has, q


def point_cost(fx, fy, occ, d3, cxs, cys, nx=None, ny=None, has=None):
    """fx, fy: 点的窗口坐标（1/16 格）。代价（1/16 格）：
      3×3 邻域内有结构格 -> 到这些格的最小距离：有法向的格用点到"过质心、沿主方向的直线"的法向距离，
      否则用点到质心的 L1 距离（上限 C_NEAR）；
      否则 -> C_NEAR（常数：不对远处的点施加拉力）。
    法向距离去掉了"沿墙方向在相邻格质心之间跳"的量化噪声，沿墙方向的代价真正平坦，退化判定才干净。
    返回 (总代价, 近邻命中点数, 0)"""
    wx, wy = fx >> 4, fy >> 4
    sx, sy = fx & 15, fy & 15
    best = np.full(fx.shape, 1 << 20, dtype=np.int64)
    if NBR == "2x2":
        qx = np.where(sx >= 8, 1, -1)
        qy = np.where(sy >= 8, 1, -1)
        offs = [(0, 0), (1, 0), (0, 1), (1, 1)]
    else:
        offs = [(a, b) for a in (-1, 0, 1) for b in (-1, 0, 1)]
    for oy_s, ox_s in offs:
        ox_ = ox_s * qx if NBR == "2x2" else ox_s
        oy_ = oy_s * qy if NBR == "2x2" else oy_s
        if True:
            kx, ky = np.clip(wx + ox_, 0, SN - 1), np.clip(wy + oy_, 0, SN - 1)
            o = occ[ky, kx]
            ex = 16 * ox_ + cxs[ky, kx] - sx
            ey = 16 * oy_ + cys[ky, kx] - sy
            dd = np.abs(ex) + np.abs(ey)
            if has is not None:
                dn = np.abs(ex * nx[ky, kx] + ey * ny[ky, kx]) >> 7
                dd = np.where(has[ky, kx], dn, dd)
            best = np.where(o, np.minimum(best, dd), best)
    near = best < (1 << 20)
    far = C_NEAR + W_FAR * np.maximum(d3[np.clip(wy, 0, SN - 1), np.clip(wx, 0, SN - 1)] - 6, 0)
    cst = np.where(near, np.minimum(best, C_NEAR), far)
    return int(cst.sum()), int(near.sum()), 0


def taxicab_dt(occ, dmax):
    """两遍光栅扫描的截断曼哈顿距离变换"""
    n = occ.shape[0]
    d = np.where(occ, 0, dmax).astype(np.int64)
    for x in range(1, n):
        d[:, x] = np.minimum(d[:, x], d[:, x - 1] + 1)
    for x in range(n - 2, -1, -1):
        d[:, x] = np.minimum(d[:, x], d[:, x + 1] + 1)
    for y in range(1, n):
        d[y] = np.minimum(d[y], d[y - 1] + 1)
    for y in range(n - 2, -1, -1):
        d[y] = np.minimum(d[y], d[y + 1] + 1)
    return np.minimum(d, dmax)


def chebyshev_dt(occ, dmax):
    """两遍光栅扫描的截断切比雪夫距离变换（硬件同构：正向 4 邻 + 反向 4 邻，逐格 min(+1)）。"""
    n = occ.shape[0]
    big = dmax
    d = np.where(occ, 0, big).astype(np.int64)
    # 行方向可向量化的等价实现：切比雪夫距离 = 行方向一维距离变换后，再做列方向的"窗口最小"
    # 一维：左右两遍
    for x in range(1, n):
        d[:, x] = np.minimum(d[:, x], d[:, x - 1] + 1)
    for x in range(n - 2, -1, -1):
        d[:, x] = np.minimum(d[:, x], d[:, x + 1] + 1)
    # 列方向：d2[y] = min_{|k|<=dmax} max(|k|, d[y+k])
    out = d.copy()
    for k in range(1, dmax + 1):
        up = np.full_like(d, big)
        dn = np.full_like(d, big)
        up[k:, :] = d[:-k, :]
        dn[:-k, :] = d[k:, :]
        out = np.minimum(out, np.maximum(k, np.minimum(up, dn)))
    return np.minimum(out, dmax)
