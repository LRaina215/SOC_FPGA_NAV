"""可通行性分析 v2：台阶感知的鲁棒地形分析（Step-Aware Robust Terrain, SART）。

v1 的问题（闭环评测中发现）：
  1. 坡度用中心差分，跨越台阶边沿时把"台阶"算成"陡坡"，可跨越的 8 cm 台阶边沿被判为致命；
  2. 远处扫描线稀疏，地面格之间出现大量未观测空洞，规划器只能把它们当未知处理。

v2 的做法（全部是 3×3 窗口内的整数运算，与 v1 相同的流水线结构，RTL 改动小）：
  1. 跳变门控的单侧差分（jump-gated one-sided gradient）：dxf = |h(x+1)-h(x)|，dxb = |h(x)-h(x-1)|，
     若 max(dxf, dxb) > JUMP（该侧是台阶/立面这样的跳变），坡度分量 = 2·min(dxf, dxb)；否则 = 2·max(dxf, dxb)。
     台阶处一侧是跳变，取另一侧，不再把台阶误报成陡坡——台阶只由"高差"一项按可跨越高度判定；
     坡面与平地交界处两侧都不是跳变，取大者，陡坡边缘不会被平地一侧"冲淡"。
  2. 空洞填补：未观测格若 8 邻域中至少 FILL_MIN 个已观测、这些邻居高差极差 < FILL_TOL、
     且邻居自身都不致命，则判为可通行（代价 FILL_COST），不参与邻居的计算。
     负障碍（坑、落差）处邻居高差大或数量不足，不会被填。
  3. 遮挡落差检测（occlusion drop-off）：站在平台/台阶上时，边沿外侧的地面被平台自身挡住，
     边沿格的邻居全是未知，高差项无从判断，机器人会直接开下去（闭环仿真中实际发生）。
     规则：已观测格 c 在 4 个方向之一上，紧邻的格未知，而沿该方向 DROP_K 格内第一个已观测格
     比 c 低 step_max 以上 -> c 判致命。只需每个方向一个"最近已观测格高度"的扫描寄存器，
     光栅扫描顺序可实现（左右方向行内、上下方向用列寄存器，与膨胀的实现方式相同）。
  4. 近区不过期（keep_r）：雷达最小量程 0.3 m、下俯视场，机器人周围约 0.4 m 看不见，
     贴着障碍转向时障碍按时间过期就会"消失"。窗口中心 ±keep_r 格内的格子不做过期清除（只比较坐标，硬件代价极小）。
  5. 参数按机器人能力给出：step_max = 可跨越高度，slope_max = 2·res·tan(最大坡度)。
"""
import numpy as np

import nav_fast as fast
import nav_fixed as nf

N = nf.N

DEFAULT_V2 = dict(age_max=60, cnt_min=1, step_max=100, rough_max=100, slope_max=36,
                  k_slope=96, k_step=48, k_rough=32, infl_r=5,
                  fill_min=5, fill_tol=30, fill_cost=40, jump=35, drop_k=8, keep_r=12)


def trav_v2(em, cx, cy, cfg):
    c = dict(DEFAULT_V2)
    c.update(cfg)
    slot, V, H, RO = fast.window_view(em, cx, cy, c)
    sh = fast._shift
    nb = {}
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dy or dx:
                nb[(dy, dx)] = (sh(V, dy, dx, False), sh(H, dy, dx, 0))
    # 台阶（高差）
    step = np.zeros((N, N), dtype=np.int64)
    for (v, h) in nb.values():
        step = np.maximum(step, np.where(v, np.abs(h - H), 0))

    # 单侧差分取小
    def comp(a, b):
        (va, ha), (vb, hb) = a, b
        da = np.abs(H - ha)
        db = np.abs(hb - H)
        both = va & vb
        g2 = np.where(np.maximum(da, db) > c["jump"], np.minimum(da, db), np.maximum(da, db))
        return np.where(both, 2 * g2, np.where(vb & (db <= c["jump"]), 2 * db, np.where(va & (da <= c["jump"]), 2 * da, 0)))

    gx = comp(nb[(0, -1)], nb[(0, 1)])
    gy = comp(nb[(-1, 0)], nb[(1, 0)])
    slope = np.maximum(gx, gy) + (np.minimum(gx, gy) >> 1)
    # 遮挡落差
    drop = np.zeros((N, N), dtype=bool)
    for dy, dx in ((0, 1), (0, -1), (1, 0), (-1, 0)):
        v1 = sh(V, dy, dx, False)
        found = np.zeros((N, N), dtype=bool)
        hfar = np.zeros((N, N), dtype=np.int64)
        for k in range(2, c["drop_k"] + 1):
            vk = sh(V, dy * k, dx * k, False)
            hk = sh(H, dy * k, dx * k, 0)
            take = vk & ~found
            hfar = np.where(take, hk, hfar)
            found |= vk
        drop |= V & ~v1 & found & (H - hfar > c["step_max"])
    lethal = V & ((step > c["step_max"]) | (RO > c["rough_max"]) | (slope > c["slope_max"]) | drop)
    s = (c["k_slope"] * np.minimum(slope, 65535) + c["k_step"] * np.minimum(step, 65535)
         + c["k_rough"] * np.minimum(RO, 65535))
    raw = np.where(~V, nf.COST_UNKNOWN, np.where(lethal, nf.COST_LETHAL, np.minimum(nf.COST_FREE_MAX, s >> 8)))

    # 空洞填补
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

    infl = fast._dilate(lethal, c["infl_r"])
    raw = np.where((raw < nf.COST_INSCRIBED) & infl, nf.COST_INSCRIBED, raw)
    cost = np.full(N * N, nf.COST_UNKNOWN, dtype=np.int64)
    cost[slot.reshape(-1)] = raw.reshape(-1)
    return cost, lethal, (slot, V, H, RO)
