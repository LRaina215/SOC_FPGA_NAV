"""Compare complete, paired batches after verifying the same scoring and inputs.

Usage: python sim/closed_loop/compare_batches.py BEFORE AFTER --out DIRECTORY
No hashes: frozen input packages and harness sources are compared directly.
"""
import argparse
import ast
import csv
import json
import math
from collections import Counter
from pathlib import Path
from statistics import mean

import numpy as np


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def load_batch(path):
    manifest = read_json(path / "manifest.json")
    rows = read_json(path / "summary.json")
    expected = {(c, s) for c in manifest["cases"] for s in manifest["arguments"]["seeds"]}
    indexed = {(m["case"], m["seed"]): m for m in rows}
    if len(rows) != len(indexed) or set(indexed) != expected:
        raise ValueError(f"Incomplete or duplicate batch: {path}")
    if any("error" in m for m in rows):
        raise ValueError(f"Resolve runner errors before comparison: {path}")
    return manifest, indexed


def declaration_errors(batch, key, metric):
    t = metric["terminal"]["declared_s"]
    args = read_json(batch / f"{key[0]}_s{key[1]}" / "manifest.json")["arguments"]
    if t is None or args["search"]:
        return None
    goal = [float(v) for v in (args["goal"] or args["waypoints"]).split(",")][-2:]
    with (batch / f"{key[0]}_s{key[1]}" / "timeseries.csv").open(encoding="utf-8", newline="") as f:
        row = min(csv.DictReader(f), key=lambda r: abs(float(r["t"]) - t))
    if abs(float(row["t"]) - t) > 0.001:
        raise ValueError("Declaration sample is missing")
    return dict(estimated_goal_error_m=math.hypot(float(row["ex"]) - goal[0], float(row["ey"]) - goal[1]),
                localization_error_m=math.hypot(float(row["ex"]) - float(row["x"]),
                                               float(row["ey"]) - float(row["y"])) )


def localization_diagnostics(batch, key, metric):
    """Do not treat pre-SET_POSE odometry coordinates as map coordinates.

    Include alignment and loss after the map frame exists; do not select only
    accepted matches. Initialization duration remains visible via sample counts.
    """
    case = batch / f"{key[0]}_s{key[1]}"
    with (case / "timeseries.csv").open(encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))
    states = {}
    if metric["prior"]:
        with np.load(case / "final_maps.npz") as data:
            states = {round(float(h[0]) * 1000): int(h[1]) for h in data["loc_hist"]}
    initialized = [r for r in rows if not metric["prior"] or states.get(round(float(r["t"]) * 1000), 0) != 0]
    sq = [(float(r["ex"]) - float(r["x"])) ** 2 + (float(r["ey"]) - float(r["y"])) ** 2 for r in initialized]
    return dict(total_samples=len(rows), map_frame_samples=len(initialized),
                pre_map_frame_samples=len(rows) - len(initialized),
                xy_rms_map_frame_m=math.sqrt(sum(sq) / len(sq)) if sq else None,
                accepted_samples=metric["loc_valid_samples"], accepted_fraction=metric["loc_valid_fraction"])


def scorer_core(path):
    tree = ast.parse((path / "runner/evaluation.py").read_text(encoding="utf-8"))
    names = {"TerminalCriteria", "TerminalJudge", "TerrainCounts"}
    out = {n.name: ast.dump(n, include_attributes=False) for n in tree.body
           if isinstance(n, ast.ClassDef) and n.name in names}
    if set(out) != names:
        raise ValueError("Missing scoring core")
    return out


def compare(before, after, before_search=None, algorithm_params=()):
    if set(algorithm_params)-{'step_high_only','step_max','local_frame'}:
        raise ValueError('Only explicitly supported algorithm parameters may differ')
    bm, b = load_batch(before)
    am, a = load_batch(after)
    if set(a) != set(b) or bm["cases"] != am["cases"]:
        raise ValueError("Cases or seeds differ")
    source_before = {key: before for key in b}
    compatibility = None
    if bm["evaluation"] != am["evaluation"]:
        if (bm["evaluation"], am["evaluation"]) != ("nav-opt-v1", "nav-opt-v2") or before_search is None:
            raise ValueError("Scoring differs; v1 to v2 requires completed search replay")
        sm, search_rows = load_batch(before_search)
        search_keys = {key for key in b if "--search" in bm["cases"][key[0]]}
        if set(search_rows) != search_keys or sm["evaluation"] != "nav-opt-v2":
            raise ValueError("Search replay does not exactly cover affected baseline cases")
        for key, metric in search_rows.items():
            b[key] = metric
            source_before[key] = before_search
        compatibility = "Non-search v1 traces reused: identical terminal/terrain scoring core; all search cases replayed with v2"
        if scorer_core(before) != scorer_core(after):
            raise ValueError("Non-search scoring core changed; cannot reuse old traces")
    else:
        for rel in ("runner/evaluation.py", "runner/run_cl.py"):
            if (before / rel).read_bytes() != (after / rel).read_bytes():
                raise ValueError(f"Scoring source differs: {rel}")
    for rel in ("runner/world.py", "runner/robot.py",
                "reference_model/sim_world.py", "reference_model/lidar_lut.py"):
        if (before / rel).read_bytes() != (after / rel).read_bytes():
            raise ValueError(f"Scoring or physical input source differs: {rel}; rebaseline both versions")
    backend_args = [read_json(batch/f'{key[0]}_s{key[1]}'/'manifest.json')['arguments']
                    for batch, rows in ((before,b),(after,a)) for key in rows]
    if any(v.get('backend','fast') == 'mujoco' for v in backend_args):
        if not all(v.get('backend','fast') == 'mujoco' for v in backend_args):
            raise ValueError('Physics backends differ')
        for rel in ('runner/mujoco_robot.py','runner/control4500.py','runner/safe_model.py'):
            if (before/rel).read_bytes() != (after/rel).read_bytes():
                raise ValueError(f'Physics/control adapter source differs: {rel}')
    bp = {p.name: p.read_bytes() for p in before.glob("prior_*.nvmp")}
    ap = {p.name: p.read_bytes() for p in after.glob("prior_*.nvmp")}
    if bp != ap:
        raise ValueError("Frozen prior packages differ")
    if before_search is not None:
        for rel in ("runner/evaluation.py", "runner/run_cl.py", "runner/world.py", "runner/robot.py",
                    "reference_model/sim_world.py", "reference_model/lidar_lut.py"):
            if (before_search / rel).read_bytes() != (after / rel).read_bytes():
                raise ValueError(f"Search replay source differs: {rel}")
        sp = {p.name: p.read_bytes() for p in before_search.glob("prior_*.nvmp")}
        if sp != bp:
            raise ValueError("Search replay prior differs")
    pairs = []
    algorithm_changes=[]
    visual_args = {"out", "no_build", "no_plots", "snap", "gif", "firmware_dll"}
    for key in sorted(b):
        bsource = source_before[key]
        bmf = read_json(bsource / f"{key[0]}_s{key[1]}" / "manifest.json")
        amf = read_json(after / f"{key[0]}_s{key[1]}" / "manifest.json")
        if bmf.get('lidar_assumptions') != amf.get('lidar_assumptions'):
            raise ValueError(f"{key}: lidar assumptions differ; rebaseline both versions")
        for field in ("terrain_cfg", "mount_pitch_x_z", "capability_assumptions", "response_assumptions",
                      "terminal_criteria", "prior_package_bytes", "terrain_alignment", "python", "numpy"):
            bv,av=bmf[field],amf[field]
            if field=='terrain_cfg' and algorithm_params:
                algorithm_changes.append(dict(case=key[0],seed=key[1],
                    parameters={p:dict(before=bv.get(p,0),after=av.get(p,0)) for p in algorithm_params}))
                bv={k:v for k,v in bv.items() if k not in algorithm_params}
                av={k:v for k,v in av.items() if k not in algorithm_params}
            if field=='response_assumptions' and bmf['arguments'].get('backend')=='mujoco':
                # Measurements of host runtime are output metadata, not physical inputs.
                bv={k:v for k,v in bv.items() if k not in ('wall_s','real_time_factor')}
                av={k:v for k,v in av.items() if k not in ('wall_s','real_time_factor')}
            if bv != av:
                raise ValueError(f"{key}: {field} differs")
        ba = {k: v for k, v in bmf["arguments"].items() if k not in visual_args}
        aa = {k: v for k, v in amf["arguments"].items() if k not in visual_args}
        if algorithm_params:
            def filter_cfg(value):
                d=dict(v.split('=') for v in value.split(',') if v)
                return {k:v for k,v in d.items() if k not in algorithm_params}
            ba['tcfg']=filter_cfg(ba['tcfg']);aa['tcfg']=filter_cfg(aa['tcfg'])
        if ba != aa:
            raise ValueError(f"{key}: physical/task arguments differ")
        trace_equal = None
        if ba["search"]:
            trace = f"{key[0]}_s{key[1]}/timeseries.csv"
            trace_equal = (bsource / trace).read_bytes() == (after / trace).read_bytes()
        pairs.append(dict(case=key[0], seed=key[1], before=b[key]["outcome"], after=a[key]["outcome"],
                          before_arrived=b[key]["arrived"], after_arrived=a[key]["arrived"],
                          before_goal_error_m=b[key]["goal_err_m"], after_goal_error_m=a[key]["goal_err_m"],
                          before_declaration=declaration_errors(bsource, key, b[key]),
                          after_declaration=declaration_errors(after, key, a[key]),
                          tracking_trace_equal_at_logged_precision=trace_equal,
                          before_localization=localization_diagnostics(bsource, key, b[key]),
                          after_localization=localization_diagnostics(after, key, a[key])))
    cases = []
    for case in bm["cases"]:
        keys = [key for key in b if key[0] == case]
        both = [key for key in keys if b[key]["arrived"] and a[key]["arrived"]]
        cases.append(dict(case=case, n=len(keys), before_arrived=sum(b[k]["arrived"] for k in keys),
                          after_arrived=sum(a[k]["arrived"] for k in keys),
                          before_collisions=sum(b[k]["collisions"] for k in keys),
                          after_collisions=sum(a[k]["collisions"] for k in keys),
                          mean_time_delta_both_arrived_s=round(mean(a[k]["time_s"] - b[k]["time_s"] for k in both), 3) if both else None))
    return dict(evaluation=am["evaluation"], before=str(before), after=str(after), inputs_equal=True,
                declared_algorithm_changes=algorithm_changes,
                baseline_search_replay=str(before_search) if before_search is not None else None,
                compatibility=compatibility,
                n=len(b), before_outcomes=dict(Counter(m["outcome"] for m in b.values())),
                after_outcomes=dict(Counter(m["outcome"] for m in a.values())), cases=cases, pairs=pairs)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("before", type=Path)
    p.add_argument("after", type=Path)
    p.add_argument("--out", required=True, type=Path)
    p.add_argument("--before-search", type=Path, help="v1基线仅重跑受影响搜索任务后的v2批次")
    p.add_argument('--algorithm-param',action='append',default=[],choices=['step_high_only','step_max','local_frame'],
                   help='预先声明允许变化的算法开关；几何、执行与评分输入仍严格相同')
    args = p.parse_args()
    r = compare(args.before, args.after, args.before_search,args.algorithm_param)
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "comparison.json").write_text(json.dumps(r, ensure_ascii=False, indent=2), encoding="utf-8")
    lines = ["# 配对闭环比较", "", f"判分 `{r['evaluation']}`；{r['n']} 对用例；评测、物理与任务输入已核对。", ""]
    if r['declared_algorithm_changes']:
        names = sorted({name for row in r['declared_algorithm_changes'] for name in row['parameters']})
        lines += [f"预先声明的算法配置差异：{', '.join(names)}；逐例值见comparison.json。其余受核对配置一致。", ""]
    lines += [
             "| 用例 | 原版到达 | 候选到达 | 原版碰撞 | 候选碰撞 | 双方成功时平均耗时变化(s) |", "|---|---|---|---|---|---|"]
    for c in r["cases"]:
        lines.append(f"| {c['case']} | {c['before_arrived']}/{c['n']} | {c['after_arrived']}/{c['n']} | {c['before_collisions']} | {c['after_collisions']} | {c['mean_time_delta_both_arrived_s']} |")
    lines += ["", f"原版结果：{r['before_outcomes']}", f"候选结果：{r['after_outcomes']}"]
    if r["compatibility"]:
        lines += ["", r["compatibility"], f"搜索复评：{r['baseline_search_replay']}"]
    (args.out / "comparison.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
