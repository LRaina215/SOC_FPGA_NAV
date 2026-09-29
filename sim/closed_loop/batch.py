"""闭环批量评测：多场景 × 多种子并行运行 run_cl.py，汇总到 summary.md / summary.json。

用法:
  python sim/closed_loop/batch.py --tag base                  # 默认用例集
  python sim/closed_loop/batch.py --tag lf --extra "--match on"
  python sim/closed_loop/batch.py --tag x --seeds 1 2 3 --cases terrain_a
输出：sim/build/batch/<tag>/<用例>_s<种子>/（每次运行的 metrics、曲线、快照）与汇总表。
"""
import argparse
import json
import shlex
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]

CASES = {
    # 坡道 + 平台 + 台阶 + 路沿缺口，终点在右上
    "terrain_a": "--scene terrain --goal 9.3,2.8 --time 120",
    # 终点在路沿后、陡坡旁：必须绕到路沿缺口
    "terrain_b": "--scene terrain --goal 8.0,-3.0 --time 150",
    # 搜索模式：边界探索找到路沿后的目标（模拟 RK3588 视野 70°/4 m），再跟踪接近
    "search": "--scene terrain --search 8.0,-3.0 --time 200",
    # 回环跑道两圈：长距离定位漂移
    "loop": "--scene loop --waypoints 6.6,-2.6,6.8,2.3,0.2,2.0,0.0,-2.2 --laps 2 --time 200",
    # ---- 有图模式：先验地图（虚拟扫图生成）+ 初始位姿随机误差 ±0.4 m / ±8°（按种子）----
    "p_terrain_a": "--scene terrain --goal 9.3,2.8 --time 120 --prior auto --init-err rand",
    "p_terrain_b": "--scene terrain --goal 8.0,-3.0 --time 150 --prior auto --init-err rand",
    "p_search": "--scene terrain --search 8.0,-3.0 --time 200 --prior auto --init-err rand",
    "p_loop": "--scene loop --waypoints 6.6,-2.6,6.8,2.3,0.2,2.0,0.0,-2.2 --laps 2 --time 200 --prior auto --init-err rand",
    # 扫图之后场地多了 3 个箱子（先验图里没有，其中一个挡在路线上）
    "p_perturb": "--scene terrain --goal 9.3,2.8 --time 150 --prior auto --init-err rand --perturb boxes",
    # 行驶中被撞：20 s 时真值突然平移 0.3 m、转 8°（检验丢失检测与自动重定位）
    "p_kidnap": "--scene terrain --goal 9.3,2.8 --time 150 --prior auto --init-err rand --kidnap 20,0.3,-0.15,8",
}
NOMAP = ["terrain_a", "terrain_b", "search", "loop"]

KEYS = ["arrived", "time_s", "found_s", "collisions", "goal_err_m", "loc_err_xy_rms_m", "loc_err_xy_final_m",
        "loc_err_xy_max_m", "loc_err_yaw_final_deg", "loc_err_z_max_m", "trav_lethal_recall", "trav_false_lethal_rate",
        "align_s", "n_align_fail", "n_lost", "t_lost_s"]


def run_one(tag, case, seed, extra):
    out = ROOT / "sim" / "build" / "batch" / tag / f"{case}_s{seed}"
    cmd = [sys.executable, str(HERE / "run_cl.py"), "--no-build", "--seed", str(seed), "--out", str(out)] + \
        shlex.split(CASES[case]) + shlex.split(extra)
    r = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace")
    mf = out / "metrics.json"
    if r.returncode != 0 or not mf.exists():
        return case, seed, {"error": r.stderr[-800:]}
    return case, seed, json.loads(mf.read_text(encoding="utf-8"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", required=True)
    ap.add_argument("--cases", nargs="*", default=NOMAP)
    ap.add_argument("--seeds", nargs="*", type=int, default=[1, 2, 3, 4, 5])
    ap.add_argument("--extra", default="", help="附加给每次 run_cl.py 的参数")
    ap.add_argument("-j", type=int, default=10)
    a = ap.parse_args()
    sys.path.insert(0, str(HERE))
    import run_cl
    run_cl.build_dll()
    # 先验地图先串行生成（虚拟扫图较慢，避免并行运行各自重复生成）
    for c in a.cases:
        v = shlex.split(CASES[c])
        if "--prior" in v and v[v.index("--prior") + 1] == "auto":
            run_cl.get_prior(v[v.index("--scene") + 1], "auto")
    jobs = [(c, s) for c in a.cases for s in a.seeds]
    with ThreadPoolExecutor(a.j) as ex:
        res = list(ex.map(lambda cs: run_one(a.tag, cs[0], cs[1], a.extra), jobs))
    out = ROOT / "sim" / "build" / "batch" / a.tag
    out.mkdir(parents=True, exist_ok=True)
    (out / "summary.json").write_text(json.dumps([{"case": c, "seed": s, **m} for c, s, m in res],
                                                 ensure_ascii=False, indent=1), encoding="utf-8")
    lines = [f"# 批量评测 {a.tag}  参数: `{a.extra}`", "",
             "| 用例 | 种子 | " + " | ".join(KEYS) + " |", "|" + "---|" * (len(KEYS) + 2)]
    for c, s, m in res:
        if "error" in m:
            lines.append(f"| {c} | {s} | 出错: {m['error'][-120:]!r} |")
            continue
        lines.append(f"| {c} | {s} | " + " | ".join(str(m.get(k)) for k in KEYS) + " |")
    lines += ["", "## 按用例汇总（均值）", "",
              "| 用例 | 到达率 | 平均耗时(到达) | xy RMS | xy 最大 | xy 终值 | 航向终值 | 召回 | 误报率 | 碰撞 | 对准耗时 | 丢失次数 |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for c in a.cases:
        ms = [m for cc, _, m in res if cc == c and "error" not in m]
        if not ms:
            continue
        arr = [m for m in ms if m["arrived"]]

        def mean(k, src=ms):
            return round(sum(m[k] for m in src) / len(src), 3) if src else "-"
        al = [m for m in ms if m.get("align_s") is not None]
        lines.append(f"| {c} | {len(arr)}/{len(ms)} | {mean('time_s', arr)} | {mean('loc_err_xy_rms_m')} | "
                     f"{mean('loc_err_xy_max_m')} | {mean('loc_err_xy_final_m')} | {mean('loc_err_yaw_final_deg')} | "
                     f"{mean('trav_lethal_recall')} | {mean('trav_false_lethal_rate')} | {mean('collisions')} | "
                     f"{mean('align_s', al) if al else '-'} | {mean('n_lost') if ms[0].get('prior') else '-'} |")
    (out / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
