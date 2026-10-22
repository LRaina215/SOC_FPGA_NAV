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
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]

CASES = {
    'j_stop': '--scene flat --start 0,0,0 --goal 0,0 --time 3',
    'j_short': '--scene flat --start 0,0,0 --goal 0.6,0 --time 20',
    'j_turn': '--scene flat --start 0,0,0 --goal 0,0.6 --time 25',
    'j_box': '--scene one_box --start 0,0,0 --goal 2.5,0 --time 60',
    'j_waypoints': '--scene flat --start 0,0,0 --waypoints 0.6,0,0.6,0.6,0,0.6,0,0 --time 50',
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
# ---- 5 m 场地（field5，两轮自平衡车模型、10 cm 记忆栅格，docs/4_design/platform.md）----
F5 = "--scene field5 --start 0.0,-2.0,0"
CASES.update({
    "f5_goal": f"{F5} --goal 4.0,2.1 --time 120",
    "f5_ramp": f"{F5} --goal 1.0,1.6 --time 120",                    # 8° 坡上平台
    "f5_gap": "--scene field5 --start 1.4,-0.1,0 --goal 3.7,-0.1 --time 90",   # 55 cm 窄通道（或绕行）
    "f5_loop": f"{F5} --waypoints 3.9,-2.1,4.0,0.3,3.0,2.1,0.2,0.4 --laps 2 --time 240",
    "f5_search": "--scene field5 --start 0.0,1.0,0 --search 1.9,-2.2 --time 200",   # 目标在路沿后
    "pf5_goal": f"{F5} --goal 4.0,2.1 --time 120 --prior auto --init-err rand",
    "pf5_loop": f"{F5} --waypoints 3.9,-2.1,4.0,0.3,3.0,2.1,0.2,0.4 --laps 2 --time 240 --prior auto --init-err rand",
    "pf5_search": "--scene field5 --start 0.0,1.0,0 --search 1.9,-2.2 --time 200 --prior auto --init-err rand",
    "pf5_perturb": f"{F5} --goal 4.0,2.1 --time 150 --prior auto --init-err rand --perturb boxes",
    "pf5_kidnap": f"{F5} --goal 4.0,2.1 --time 150 --prior auto --init-err rand --kidnap 15,0.25,-0.1,8",
})
NOMAP = ["terrain_a", "terrain_b", "search", "loop"]
F5CASES = ["f5_goal", "f5_ramp", "f5_gap", "f5_loop", "f5_search", "pf5_goal", "pf5_loop", "pf5_search", "pf5_perturb", "pf5_kidnap"]

KEYS = ["arrived", "outcome", "firmware_arrived", "time_s", "found_s", "collisions", "goal_err_m", "loc_err_xy_rms_m", "loc_err_xy_final_m",
        "loc_err_xy_max_m", "loc_err_yaw_final_deg", "loc_err_z_max_m", "trav_lethal_recall", "trav_false_lethal_rate",
        "align_s", "n_align_fail", "n_lost", "t_lost_s", "loc_valid_fraction", "loc_err_xy_full_rms_m"]


def run_one(tag, case, seed, extra, firmware_dll=None, runner=None, motion_trace=False):
    out = ROOT / "sim" / "build" / "batch" / tag / f"{case}_s{seed}"
    cmd = [sys.executable, "-X", "utf8", str(runner or HERE / "run_cl.py"), "--no-build", "--seed", str(seed), "--out", str(out)] + \
        shlex.split(CASES[case]) + shlex.split(extra)
    if motion_trace:
        # 冻结目录内的只读包装器；原运行器/判分不改，记录实时刷盘便于尽早诊断。
        cmd[3:4] = [str((runner or HERE / "run_cl.py").parent / "trace_motion.py"),
                    "--trace-out", str(out / "diagnostics")]
    if firmware_dll:
        cmd += ["--firmware-dll", str(firmware_dll)]
    r = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace")
    mf = out / "metrics.json"
    if r.returncode != 0 or not mf.exists():
        out.mkdir(parents=True, exist_ok=True)
        (out / "error.log").write_text(r.stdout + "\n" + r.stderr, encoding="utf-8")
        print(f"{case} seed={seed}: ERROR", flush=True)
        return case, seed, {"error": r.stderr[-800:]}
    m = json.loads(mf.read_text(encoding="utf-8"))
    print(f"{case} seed={seed}: {m.get('outcome')} ({m['wall_s']} s wall)", flush=True)
    return case, seed, m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", required=True)
    ap.add_argument("--cases", nargs="*", default=F5CASES)
    ap.add_argument("--seeds", nargs="*", type=int, default=[1, 2, 3, 4, 5])
    ap.add_argument("--extra", default="", help="附加给每次 run_cl.py 的参数")
    ap.add_argument("--firmware-dll", type=Path, help="复用保存的固件；不重编或替换其他批次的DLL")
    ap.add_argument("--firmware-source", type=Path, help="显式DLL对应的fw源码目录，避免保存错配的工作区源码")
    ap.add_argument('--runner-source',type=Path,help='复用已冻结运行器，保持双方判分/执行链完全相同')
    ap.add_argument('--model-source',type=Path,help='显式冻结感知算法来源；重跑旧基线时保留其算法')
    ap.add_argument('--config-source',type=Path,help='与感知/固件来源匹配的配置目录')
    ap.add_argument('--motion-trace', action='store_true', help='快速后端只读运动记录；MuJoCo使用自身执行日志')
    ap.add_argument("-j", type=int, default=10)
    a = ap.parse_args()
    if a.firmware_dll and not a.firmware_source:
        ap.error('--firmware-dll requires --firmware-source for a consistent frozen record')
    out = ROOT / "sim" / "build" / "batch" / a.tag
    if out.exists() and any(out.iterdir()):
        ap.error("Tag directory already contains results; use a new tag to preserve evidence")
    out.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(HERE))
    import run_cl
    if a.firmware_dll:
        a.firmware_dll = a.firmware_dll.resolve()
        run_cl.DLL = a.firmware_dll
    else:
        run_cl.build_dll()
    # Freeze the actual runner/model/config and DLL, not only a git revision.
    shutil.copy2(run_cl.DLL, out / "nav_fw.dll")
    shutil.copytree(a.runner_source or HERE, out / "runner", ignore=shutil.ignore_patterns("__pycache__"))
    shutil.copytree(a.model_source or ROOT / "model", out / "reference_model", ignore=shutil.ignore_patterns("__pycache__"))
    shutil.copytree(a.config_source or ROOT / "config", out / "config")
    source = a.firmware_source.resolve() if a.firmware_source else ROOT / 'fw'
    shutil.copytree(source, out / "firmware_source", ignore=shutil.ignore_patterns("build", "__pycache__"))
    # Execute the frozen package, not live files which a later edit can change.
    tree=out/'execution_tree'
    shutil.copytree(out/'runner',tree/'sim/closed_loop')
    shutil.copytree(out/'reference_model',tree/'model')
    shutil.copytree(ROOT/'tools',tree/'tools',ignore=shutil.ignore_patterns('__pycache__','build'))
    shutil.copytree(out/'config',tree/'config')
    batch_args = dict(vars(a))
    if a.firmware_dll:
        batch_args["firmware_dll"] = str(a.firmware_dll)
        batch_args['firmware_source'] = str(source)
    if a.runner_source: batch_args['runner_source']=str(a.runner_source.resolve())
    for name in ('model_source','config_source'):
        if batch_args.get(name):batch_args[name]=str(batch_args[name].resolve())
    manifest = dict(evaluation=run_cl.EVALUATION_VERSION, arguments=batch_args, argv=sys.argv,
                    cases={c: shlex.split(CASES[c]) for c in a.cases},
                    seed_roles=dict(tuning=[1, 2, 3], historical_validation=[4, 5], reserved_holdout=[11, 12]))
    (out / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    # 先验地图先串行生成（虚拟扫图较慢，避免并行运行各自重复生成）
    for c in a.cases:
        v = shlex.split(CASES[c])
        if "--prior" in v and v[v.index("--prior") + 1] == "auto":
            scene = v[v.index("--scene") + 1]
            pkg = run_cl.get_prior(scene, "auto")
            (out / f"prior_{scene}.nvmp").write_bytes(pkg)
            cache=tree/'sim/build/prior';cache.mkdir(parents=True,exist_ok=True)
            import slf_match
            name=f'{scene}.nvmp' if slf_match.SN==256 else f'{scene}_sn{slf_match.SN}.nvmp'
            (cache/name).write_bytes(pkg)
            # 新射线版本不能误复用旧步进扫图；旧冻结运行器仍读取上面的兼容名。
            (cache/run_cl.prior_cache_name(scene)).write_bytes(pkg)
    jobs = [(c, s) for c in a.cases for s in a.seeds]
    with ThreadPoolExecutor(a.j) as ex:
        res = list(ex.map(lambda cs: run_one(a.tag, cs[0], cs[1], a.extra, out/'nav_fw.dll', tree/'sim/closed_loop/run_cl.py', a.motion_trace), jobs))
    out = ROOT / "sim" / "build" / "batch" / a.tag
    out.mkdir(parents=True, exist_ok=True)
    (out / "summary.json").write_text(json.dumps([{"case": c, "seed": s, **m} for c, s, m in res],
                                                 ensure_ascii=False, indent=1), encoding="utf-8")
    lines = [f"# 批量评测 {a.tag}  参数: `{a.extra}`", "",
             f"判分版本：{run_cl.EVALUATION_VERSION}。到达需独立位置与停稳检查，并且无模型碰撞；旧口径地形指标保留，分层结果见每例 metrics.json。", "",
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
            vals = [m[k] for m in src if m.get(k) is not None]
            return round(sum(vals) / len(vals), 3) if vals else "-"
        al = [m for m in ms if m.get("align_s") is not None]
        lines.append(f"| {c} | {len(arr)}/{len(ms)} | {mean('time_s', arr)} | {mean('loc_err_xy_rms_m')} | "
                     f"{mean('loc_err_xy_max_m')} | {mean('loc_err_xy_final_m')} | {mean('loc_err_yaw_final_deg')} | "
                     f"{mean('trav_lethal_recall')} | {mean('trav_false_lethal_rate')} | {mean('collisions')} | "
                     f"{mean('align_s', al) if al else '-'} | {mean('n_lost') if ms[0].get('prior') else '-'} |")
    (out / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
