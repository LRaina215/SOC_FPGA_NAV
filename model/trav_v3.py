"""可通行性分析 v3：基线坡度（Gated-Baseline Slope, GBS）+ v2 的台阶/遮挡落差/空洞填补/近区保持。

v2 在闭环里暴露的问题：坡度用相邻格（5 cm）差分，3–6 cm 的小台阶（坡道侧沿、路面接缝）没有超过"跳变"门限，
就被当成 5 cm 基线上的坡——27 mm/50 mm ≈ 28°，判致命，机器人被困在坡道脚下（理想传感器下也复现）。
但对机器人来说，坡度只有在与车体尺度相当的长度上才有意义。

v3 的坡度：沿 x、y 各取 B 格基线（默认 4 格 = 20 cm），把基线内相邻格的有符号高差逐段相加，
超过跳变门限 JUMP 的那一段不计入（它归"台阶"项判定）——即"去掉台阶后的净升高"：
    rise_x = Σ_{k=-B/2}^{B/2-1} gate(h[x+k+1] − h[x+k])，gate(d) = d (|d| ≤ JUMP) 否则 0
    slope 判据：max(|rise_x|, |rise_y|) + min/2 > B·res·tan(最大坡度)
  · 10° 坡道：每格 9 mm，4 格 35 mm，远低于 73 mm；22.6° 陡坡：每格 21 mm，84 mm，致命；
  · 5.6 cm 侧沿分到两格各 ~28 mm：净升高 56 mm，不致命（真值：可跨）；
  · 8 cm 台阶：一段 80 mm 超过 JUMP 被剔除，坡度项为 0，由台阶项按可跨高度判定；
  · 有符号相加，逐格噪声相互抵消（相当于 B 格基线上的差分，而不是 B 个噪声绝对值之和）。
硬件：x 方向是行内 B+1 级移位寄存器上的加法树；y 方向需要 B 行行缓存（每行 128×16 bit，1 块 BSRAM）。
基线内有未观测格时该方向坡度记 0（远处稀疏区只靠台阶项；走近后格子变密，坡度项自然生效）。

台阶项改为 8 邻域最大高差（v2 相同），阈值 = 可跨越高度。
"""
import numpy as np

import nav_fast as fast
import nav_fixed as nf
import trav_v2

N = nf.N

DEFAULT_V3 = dict(trav_v2.DEFAULT_V2)
# 阈值按两轮自平衡小车（轮径 65 mm）的估计能力：可跨约 2 cm、坡度 12°。场地里可跨的门槛 1.5 cm、不可跨的路沿 ≥ 5 cm，
# 阈值取在两者之间（30 mm）留出雷达噪声余量；坡度 8°/12° 可走、20° 不可走，阈值 16°（20 cm 基线净升高 57 mm）。
DEFAULT_V3.update(step_max=30, rough_max=50, drop_k=16, drop_slope=4, slope_b=4, slope_rise=57, jump=25, k_slope=160, k_step=48, k_rough=32)


def gated_rise(V, H, axis, B, jump, signed=False):
    """沿 axis 的 B 格基线净升高（去掉跳变段），基线内有未观测格则返回无效"""
    sh = fast._shift
    half = B // 2
    rise = np.zeros((N, N), dtype=np.int64)
    ok = V.copy()
    for k in range(-half, B - half):
        dy0, dx0 = (k, 0) if axis == 0 else (0, k)
        dy1, dx1 = (k + 1, 0) if axis == 0 else (0, k + 1)
        v0, h0 = sh(V, dy0, dx0, False), sh(H, dy0, dx0, 0)
        v1, h1 = sh(V, dy1, dx1, False), sh(H, dy1, dx1, 0)
        d = h1 - h0
        rise += np.where(np.abs(d) <= jump, d, 0)
        ok &= v0 & v1
    if signed:
        return np.where(ok, rise, 0), ok
    return np.where(ok, np.abs(rise), 0), ok


def trav_v3(em, cx, cy, cfg):
    c = dict(DEFAULT_V3)
    c.update(cfg)
    slot, V, H, RO = fast.window_view(em, cx, cy, c)
    sh = fast._shift
    nb = {}
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dy or dx:
                nb[(dy, dx)] = (sh(V, dy, dx, False), sh(H, dy, dx, 0))
    step = np.zeros((N, N), dtype=np.int64)
    for (v, h) in nb.values():
        step = np.maximum(step, np.where(v, np.abs(h - H), 0))
    rx, _ = gated_rise(V, H, 1, c["slope_b"], c["jump"])
    ry, _ = gated_rise(V, H, 0, c["slope_b"], c["jump"])
    rise = np.maximum(rx, ry) + (np.minimum(rx, ry) >> 1)
    # 遮挡落差：已观测格 c 沿某方向紧邻未知，隔着空洞第一个已观测格（第 k 格）比"按 c 处坡度外推到第 k 格的高度"
    # 低 step_max 以上 -> 致命。外推用本格沿该轴的基线净升高（有符号，/B 为每格坡度）：
    # 沿下坡往下看时远处自然更低，外推后不误报；平台/坡道侧沿横向坡度为 0，落差照常检出
    # 基线内有未观测格（观测边缘常见）时坡度未知，按每格 drop_slope（≈10° 下坡）放宽
    sx, okx = gated_rise(V, H, 1, c["slope_b"], c["jump"], signed=True)
    sy, oky = gated_rise(V, H, 0, c["slope_b"], c["jump"], signed=True)
    drop = np.zeros((N, N), dtype=bool)
    for dy, dx in ((0, 1), (0, -1), (1, 0), (-1, 0)):
        v1 = sh(V, dy, dx, False)
        found = np.zeros((N, N), dtype=bool)
        hfar = np.zeros((N, N), dtype=np.int64)
        kfar = np.zeros((N, N), dtype=np.int64)
        for k in range(2, c["drop_k"] + 1):
            vk = sh(V, dy * k, dx * k, False)
            hk = sh(H, dy * k, dx * k, 0)
            take = vk & ~found
            hfar = np.where(take, hk, hfar)
            kfar = np.where(take, k, kfar)
            found |= vk
        sdir = (sx * dx + sy * dy)                     # 沿该方向的基线升高（B 格）
        okd = okx if dx else oky
        hexp = np.where(okd, H + (sdir * kfar) // c["slope_b"], H - kfar * c["drop_slope"])
        drop |= V & ~v1 & found & (hexp - hfar > c["step_max"])
    lethal = V & ((step > c["step_max"]) | (RO > c["rough_max"]) | (rise > c["slope_rise"]) | drop)
    # 代价：坡度按基线净升高归一到 0..200（k_slope/256 每 mm），台阶、粗糙度同 v2
    s = (c["k_slope"] * np.minimum(rise, 65535) + c["k_step"] * np.minimum(step, 65535)
         + c["k_rough"] * np.minimum(RO, 65535))
    raw = np.where(~V, nf.COST_UNKNOWN, np.where(lethal, nf.COST_LETHAL, np.minimum(nf.COST_FREE_MAX, s >> 8)))
    # 空洞填补（同 v2）
    nval = np.zeros((N, N), dtype=np.int64)
    hmax = np.full((N, N), -(1 << 20), dtype=np.int64)
    hmin = np.full((N, N), 1 << 20, dtype=np.int64)
    nleth = np.zeros((N, N), dtype=bool)
    for (dy, dx), (v, h) in nb.items():
        nval += v
        hmax = np.where(v, np.maximum(hmax, h), hmax)
        hmin = np.where(v, np.minimum(hmin, h), hmin)
        nleth |= sh(lethal, dy, dx, False)
    fill = (~V) & (nval >= c["fill_min"]) & ((hmax - hmin) < c["fill_tol"]) & ~nleth
    raw = np.where(fill, c["fill_cost"], raw)
    disk = c.get("infl_disk", 1) and 0 < c["infl_r"] <= 4          # 硬件圆形膨胀支持半径 <= 4，更大时方形
    infl = fast.dilate_disk(lethal, c["infl_r"]) if disk else fast._dilate(lethal, c["infl_r"])
    raw = np.where((raw < nf.COST_INSCRIBED) & infl, nf.COST_INSCRIBED, raw)
    cost = np.full(N * N, nf.COST_UNKNOWN, dtype=np.int64)
    cost[slot.reshape(-1)] = raw.reshape(-1)
    return cost, lethal, (slot, V, H, RO)
