"""traj_eval 激励与期望：随机代价图 + 若干组候选（含出窗口、碰撞、未知）。

用法: gen_traj.py <输出目录>
"""
import math
import random
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "model"))
import nav_fixed as nf  # noqa: E402

N = nf.N


def main(out):
    out = Path(out)
    rng = random.Random(7)
    # 代价图：大部分可通行，随机障碍块与未知区
    cost = np.zeros(N * N, dtype=np.int64)
    for a in range(N * N):
        cost[a] = rng.choice([0, 0, 0, 5, 20, 60, 120, 200])
    for _ in range(40):
        sx, sy, w, h = rng.randrange(N), rng.randrange(N), rng.randint(1, 6), rng.randint(1, 6)
        val = rng.choice([253, 254, 255])
        for yy in range(sy, min(N, sy + h)):
            for xx in range(sx, min(N, sx + w)):
                cost[(yy << 7) | xx] = val

    runs = []
    # (cx, cy, x0, y0, th0, T, K, cmax)
    for r in range(4):
        cx, cy = rng.randint(-300, 300), rng.randint(-300, 300)
        x0 = (cx << 8) + rng.randint(-2000, 2000)
        y0 = (cy << 8) + rng.randint(-2000, 2000)
        th0 = rng.randrange(65536)
        T = [16, 32, 64, 5][r]
        K = [8, 64, 17, 1][r]
        cmax = [253, 253, 254, 200][r]
        cands = []
        for k in range(K):
            v = rng.randint(-40, 400)                  # 格 Q8 / 步
            vy = rng.randint(-100, 100) if r == 2 else 0
            w = rng.randint(-1200, 1200)               # 65536/周/步
            cands.append((v, vy, w))
        res = nf.traj_eval(cost, cx, cy, x0, y0, th0, cands, T, cmax)
        runs.append((cx, cy, x0, y0, th0, T, K, cmax, cands, res))
        print(f"run{r}: K={K} T={T} 碰撞 {sum(q[3] for q in res)} 出窗 {sum(q[4] for q in res)}")

    (out / "cost.hex").write_text("\n".join(f"{int(c):02x}" for c in cost) + "\n")
    lines = []
    for cx, cy, x0, y0, th0, T, K, cmax, cands, res in runs:
        hdr = [cx, cy, x0, y0, th0, T, K, cmax]
        lines += [f"{v & 0xFFFFFFFF:08x}" for v in hdr]
        for dvx, dvy, dth in cands:
            lines.append(f"{((dvy & 0xFFFF) << 16) | (dvx & 0xFFFF):08x}")
            lines.append(f"{dth & 0xFFFF:08x}")
        for mx, nun, cs, coll, left, csum, ex, ey in res:
            lines.append(f"{(left << 25) | (coll << 24) | (cs << 16) | (nun << 8) | mx:08x}")
            lines.append(f"{csum & 0xFFFFFFFF:08x}")
            lines.append(f"{ex & 0xFFFFFFFF:08x}")
            lines.append(f"{ey & 0xFFFFFFFF:08x}")
    (out / "runs.hex").write_text("\n".join(lines) + "\n")
    (out / "counts.txt").write_text(f"{len(runs)}\n")


if __name__ == "__main__":
    main(sys.argv[1])
