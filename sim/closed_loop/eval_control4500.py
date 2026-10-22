"""J1 local, headless baseline; does not reset a failed trial or edit control."""
import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

from control4500 import ROOT, DEFAULT_REPO, load_runtime


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=6.)
    ap.add_argument("--out", type=Path, default=ROOT / "sim/build/nav_opt/j1_control4500")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    rt, policy = load_runtime(args.out / "assets")
    results = []
    for name, v, w in (("stop", 0, 0), ("forward", .2, 0), ("reverse", -.2, 0),
                       ("left", 0, 1.), ("right", 0, -1.)):
        bot = rt.Robot(forward_bias=0.)
        bot.reset((v, w, .1305))
        samples = []
        with (args.out / (name + ".csv")).open("w", newline="") as f:
            writer = None
            for _ in range(round(args.seconds / bot.control_dt)):
                failed = bot.step(policy)
                row = bot.metrics()
                if writer is None:
                    writer = csv.DictWriter(f, fieldnames=list(row)); writer.writeheader()
                writer.writerow(row)
                if row["time_s"] >= 1.: samples.append(row)
                if failed: break
        r = dict(case=name, command=[v, w, .1305], survived=not bot.done,
                 seconds_simulated=bot.data.time, displacement_m=bot.metrics()["displacement_m"],
                 mean_forward_mps=float(np.mean([s["forward_mps"] for s in samples])),
                 mean_yaw_radps=float(np.mean([s["yaw_radps"] for s in samples])))
        results.append(r)
        print(json.dumps(r), flush=True)
    report = dict(stage="J1", control_snapshot=str(DEFAULT_REPO), commit="6443f56",
                  python=sys.version, mujoco=rt.mujoco.__version__, numpy=np.__version__,
                  torch=rt.torch.__version__, forward_bias_mps=0., physics_dt=bot.dt,
                  policy_dt=bot.control_dt, auto_reset=False, trials_per_case=1,
                  path_only_mjcf_adaptation=True, physical_calibration=False, results=results)
    (args.out / "evaluation.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return 0 if all(r["survived"] for r in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
