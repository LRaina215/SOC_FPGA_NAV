"""结构似然场匹配（Structure Likelihood Field, SLF）：FPGA 匹配定位的定点模型。

动机：EICM（高程区间一致性）把地面点也拿来比，地面在 2.5D 图里对水平位移几乎不敏感，却对姿态噪声很敏感，
实测有约 1 个候选步长的系统偏差。水平定位真正的约束来自竖直结构（墙、箱体侧面、路沿立面）。

三步，全部是流式整数运算：
1. 结构点提取（深度图域，逐像素）：同一列上下相邻两像素变换到里程计系后，
   若连线陡于 ~63°（|Δz| > 2·|Δxy|，且 |Δz| >= DZ_MIN）则两点都是"结构点"。
   这就是 LeGO-LOAM 地面分割用的"相邻线束夹角"的反面，只需一行点的行缓存。
   10° 坡道、平台顶面是平的，不会被选中；平台侧面、墙、箱子、路沿立面会。
2. 结构图（与高程图同样的滚动寻址，256×256、5 cm）：每格命中数（每帧至多 +1）+ 格内亚格质心
   （18 位槽字：1/8 格，前 HIT_FREEZE 次按格号对称取整累计平均后冻结）。命中数 >= HIT_MIN 的格为结构格。建"似然场"时再算：
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
# 槽字格式：18 位（缺省）= {标签 每轴 1 位, 命中数 2 位, 质心 x/y 各 3 位（1/8 格）, occ, q5, has, gate}；
# 0 = 旧 23 位 {标签 每轴 2 位, 命中数 3 位, 质心 4 位（1/16 格）, ...}。18 位省 BSRAM（roadmap A2），
# 标签每轴 1 位 = 25.6 m 才绕回（场地 <= 12.8 m 不会混淆）。模型里 cxs/cys 存 1/16 格的 2b+1（b = 1/8 格格号，法向按它算）；
# 打分按 1/32 格、代表点 4b+1，与向下取整到 1/16 格的点对齐（point_cost）。
W18 = os.environ.get("SLF_W18", "1") == "1"
TAG_AX = 1 if W18 else 2     # 每轴标签位数
TAGB = 2 * TAG_AX
WORD_BITS = 18 if W18 else 23
DZ_MIN = 20                  # mm：竖直段最小高差（抑制噪声）
SLOPE_K = 2                  # |Δz| > K·|Δxy|
Z_LO, Z_HI = int(os.environ.get("SLF_ZLO", "60")), 1500   # 相对机器人足底高度（去掉地面接缝与天花板）
# 有图模式的下限：机器人在平台/坡顶上时，脚下的路沿、平台侧面也是可用的竖直结构（先验图里有），
# 相对足底 +60 mm 的下限会把它们全部丢掉——闭环仿真中平台上沿行驶方向漂移 0.28 m；放到 -300 mm 后最大误差 7 cm。
# 无图模式保持 +60（自建图时低处结构的收益不明显，批量评测互有胜负）。
Z_LO_PRIOR = -300
HIT_MIN = 2
HIT_FREEZE = 3 if W18 else 4 # 质心累计平均的次数，之后冻结
HIT_MAX = 3 if W18 else 7    # 命中数 2 bit（旧格式 3 bit）


def cq(c):
    """质心存储量化：1/16 格 -> 1/8 格（存 c >> 1，读回 (c >> 1) * 2 + 1 = 该 1/8 格的中点）"""
    return ((np.asarray(c) >> 1) << 1) | 1 if W18 else c


def rnd_div(d, n):
    """d / n 四舍五入、恰好一半时向 0 取整（正负对称，累计平均不产生单向偏差）"""
    d = np.asarray(d)
    return np.sign(d) * ((2 * np.abs(d) + n - 1) // (2 * n))


def pack_hi(tag, hit, cxs, cys):
    """槽字高位（低 8 位 occ/q/has/gate 由似然场给出）"""
    if W18:
        return (tag << 16) | (hit << 14) | ((cxs >> 1) << 11) | ((cys >> 1) << 8)
    return (tag << 19) | (hit << 16) | (cxs << 12) | (cys << 8)


def unpack_hi(word):
    """槽字 -> (tag, hit, cx, cy)，质心换回 1/16 格"""
    word = np.asarray(word, dtype=np.int64)
    if W18:
        return (word >> 16) & 3, (word >> 14) & 3, (((word >> 11) & 7) << 1) | 1, (((word >> 8) & 7) << 1) | 1
    return (word >> 19) & 15, (word >> 16) & 7, (word >> 12) & 15, (word >> 8) & 15


def tag_of(gx, gy):
    m = (1 << TAG_AX) - 1
    return (((gy >> SN_LOG2) & m) << TAG_AX) | ((gx >> SN_LOG2) & m)
DMAX = 8                     # 截断距离（格）
C_NEAR = 40                  # 近邻代价上限（1/16 格，2.5 格）
T_IN = 6                     # 内点门限（1/16 格，约 19 mm）：残差不超过它的点计为内点（定位质量 / 对准歧义判定）
W_FAR = 0                    # 近邻区以外代价为常数：远处的点不施加拉力（加梯度后闭环平均误差反而变大，见 algorithms.md）
GATE_D = 6                   # 参与打分的点：预测位置的距离场 <= 2 格（更远的多是新看到的结构）
MARGIN = 8                   # 格：窗口边缘保留带（> 搜索范围 + 1）
PB_MAX = 4096                # 点缓冲容量
MAX_PTS = 1024               # 每帧参与匹配的结构点上限（均匀抽样）
W = 160


def structure_mask(p, pv, z_ref, z_lo=None, z_hi=None):
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
    z_lo = Z_LO if z_lo is None else z_lo
    z_hi = Z_HI if z_hi is None else z_hi
    return s & pv & (zr >= z_lo) & (zr <= z_hi)


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
        """硬件结构图槽字：{tag, hit, cx, cy, occ, q, has, gate}（WORD_BITS 位）；窗口外的槽只有高位有意义（返回 mask）"""
        w = pack_hi(self.tag, self.hit, self.cxs, self.cys)
        full = (1 << WORD_BITS) - 1
        mask = np.full(SN * SN, full & ~0xFF, dtype=np.int64)
        if self.lf is not None:
            ox, oy, occ, d3, cxs, cys, nx, ny, has = self.lf
            s = self.lf_slot.reshape(-1)
            lo = (occ.astype(np.int64) << 7) | (self.lf_q << 2) | (has.astype(np.int64) << 1) | (d3 <= GATE_D)
            w[s] |= lo.reshape(-1)
            mask[s] = full
        return w, mask

    def load_prior(self, ox, oy, slot, word):
        """有图模式：先验定位图（槽字已含法向/门控，离线算好）直接写进结构图；窗口原点固定为 (ox, oy)。
        之后 update / build_lf 不再改动结构图（硬件：MATCH.CTRL.PRIOR=1 时 UPDATE 只清点缓冲）"""
        self.hit[:] = 0
        self.tag[:] = 0
        self.cxs[:] = 0
        self.cys[:] = 0
        low = np.zeros(SN * SN, dtype=np.int64)
        self.tag[slot], self.hit[slot], self.cxs[slot], self.cys[slot] = unpack_hi(word)
        low[slot] = word & 0xFF
        w = np.arange(SN)
        gx, gy = ox + w, oy + w
        s = ((gy[:, None] & (SN - 1)) << SN_LOG2) | (gx[None, :] & (SN - 1))
        lo = low[s]
        occ = (lo >> 7) & 1 == 1
        q = (lo >> 2) & 31
        has = (lo >> 1) & 1 == 1
        gate = lo & 1 == 1
        nv = np.array(NVEC, dtype=np.int64)
        d3 = np.where(gate, 0, 3 * DMAX)
        self.lf = (ox, oy, occ, d3, self.cxs[s], self.cys[s], nv[q, 0], nv[q, 1], has)
        self.lf_q = q
        self.lf_slot = s
        self.n_struct = int(occ.sum())
        self.prior = True

    prior = False
    nogate = False               # 初始对准/重定位：不做门控（位姿误差大时门控会把正确的点也挡掉）

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
        tg = tag_of(ix, iy)
        u, first = np.unique(a, return_index=True)
        mx, my, tg = (fx & 15)[first], (fy & 15)[first], tg[first]
        fresh = (self.tag[u] != tg) | (self.hit[u] == 0)
        h0 = np.where(fresh, 0, self.hit[u])
        upd = h0 < HIT_FREEZE
        n1 = h0 + 1
        if W18:
            # 按 1/8 格的格号平均：b += rnd_div(m - b, n)。取整必须正负对称：旧的"加 n/2 再向下取整"在 1/8 格下
            # 每次平局都往 +x/+y 偏半格，无图模式里结构图随机器人前进不断更新，偏差会累积成单向漂移
            for c, m in ((self.cxs, mx), (self.cys, my)):
                b, mb = c[u] >> 1, m >> 1
                c[u] = 2 * np.where(fresh, mb, np.where(upd, b + rnd_div(mb - b, n1), b)) + 1
        else:
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
        etag = tag_of(gx[None, :], gy[:, None])
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
        # 有图模式：窗口固定、槽字标签与点所在格比对（窗口外 = 无结构），不需要边缘保留带
        mg = 0 if self.prior else MARGIN
        core = (wx0 >= mg) & (wx0 < SN - mg) & (wy0 >= mg) & (wy0 < SN - mg)
        # 只用预测位置 GATE_D（1/3 格）以内已有结构的点：新看到的结构（面向前延伸的部分、新出现的物体）
        # 离已知结构远，若参与打分会把位姿往"已知区"拉——沿运动方向的系统偏差（离线测试中稳定 -25 mm）。
        # 有图模式下它同时剔除先验图里没有的物体（临时箱子、行人）
        near = np.zeros_like(core)
        near[core] = True if self.nogate else d3[wy0[core], wx0[core]] <= GATE_D
        px, py = px[near], py[near]
        for k, (dx, dy, da) in enumerate(cands):
            x, y = apply_delta(px, py, dx, dy, da, cx_mm, cy_mm)
            out[k] = (len(px),) + point_cost(self.cell16(x) - 16 * ox, self.cell16(y) - 16 * oy, occ, d3, cxs, cys,
                                             nx, ny, has, inwin=self.prior)
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


def point_cost(fx, fy, occ, d3, cxs, cys, nx=None, ny=None, has=None, inwin=False):
    """fx, fy: 点的窗口坐标（1/16 格）。代价（1/16 格）：
      3×3 邻域内有结构格 -> 到这些格的最小距离：有法向的格用点到"过质心、沿主方向的直线"的法向距离，
      否则用点到质心的 L1 距离（上限 C_NEAR）；
      否则 -> C_NEAR（常数：不对远处的点施加拉力）。
    法向距离去掉了"沿墙方向在相邻格质心之间跳"的量化噪声，沿墙方向的代价真正平坦，退化判定才干净。
    返回 (总代价, 内点数（残差 <= T_IN）, 0)"""
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
            ux, uy = wx + ox_, wy + oy_
            kx, ky = np.clip(ux, 0, SN - 1), np.clip(uy, 0, SN - 1)
            o = occ[ky, kx]
            if inwin:           # 硬件：槽字标签 != 该格标签 -> 视为无结构
                o = o & (ux >= 0) & (ux < SN) & (uy >= 0) & (uy < SN)
            if W18:
                # 1/32 格计算：1/8 格质心 b 的代表点取 4b+1（= 1/16 格下的 2b+0.5），与向下取整到 1/16 格的点对齐。
                # 若按 1/16 格取 2b+1，结构图相对点整体偏 +1/32 格（约 1.6 mm），无图模式里会累积成往 +x/+y 的漂移
                ex = 32 * ox_ + 2 * cxs[ky, kx] - 1 - 2 * sx
                ey = 32 * oy_ + 2 * cys[ky, kx] - 1 - 2 * sy
                dd = (np.abs(ex) + np.abs(ey)) >> 1
                sh_n = 8
            else:
                ex = 16 * ox_ + cxs[ky, kx] - sx
                ey = 16 * oy_ + cys[ky, kx] - sy
                dd = np.abs(ex) + np.abs(ey)
                sh_n = 7
            if has is not None:
                dn = np.abs(ex * nx[ky, kx] + ey * ny[ky, kx]) >> sh_n
                dd = np.where(has[ky, kx], dn, dd)
            best = np.where(o, np.minimum(best, dd), best)
    near = best < (1 << 20)
    far = C_NEAR + W_FAR * np.maximum(d3[np.clip(wy, 0, SN - 1), np.clip(wx, 0, SN - 1)] - 6, 0)
    cst = np.where(near, np.minimum(best, C_NEAR), far)
    return int(cst.sum()), int((near & (best <= T_IN)).sum()), 0


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
