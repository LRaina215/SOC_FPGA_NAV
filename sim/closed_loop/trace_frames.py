"""Read-only F1 frame/pose recorder around the unchanged fast runner."""
import argparse
import ctypes
import csv
import json
from pathlib import Path
import sys

import numpy as np
import run_cl

FIELDS = ["now_us", "frames", "applied", "x", "y", "z", "yaw", "pitch", "roll", "pose_us",
          "odo_us", "imu_us", "hist_old_us", "hist_n", "att_old_us", "att_n", "frame_us",
          "frame_x", "frame_y", "frame_z", "frame_yaw", "dx", "dy", "dyaw", "pending", "running",
          "skip", "map_seq", "cached_seq", "cx", "cy", "motion_result"]


def main():
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("--trace-out", type=Path, required=True)
    ap.add_argument("--window", default="83,87")
    args, rest = ap.parse_known_args()
    lo, hi = map(float, args.window.split(","))
    args.trace_out.mkdir(parents=True, exist_ok=False)
    loader, commit, update = ctypes.CDLL, run_cl.fpga_model.FpgaModel.commit, run_cl.fpga_model.FpgaModel.match_update
    step = run_cl.rb.RobotSim.step
    fw = None
    bot = None
    diag = (ctypes.c_int32 * 32)()
    index = []
    last = None
    with (args.trace_out / "events.csv").open("w", newline="") as f:
        writer = csv.writer(f); writer.writerow(["event", *FIELDS])

        def read():
            fw.py_get_frame_diag(diag)
            return list(diag)

        def emit(event):
            d = read()
            if lo <= (d[0] - 1000) / 1e6 <= hi:
                writer.writerow([event, *d]); f.flush()
            return d

        def load(*a, **kw):
            nonlocal fw
            fw = loader(*a, **kw)
            original_poll = fw.py_poll

            def poll():
                nonlocal last
                original_poll()
                d = read()
                key = (d[1], d[2], d[27], d[31])
                if key != last: emit("poll"); last = key
            fw.py_poll = poll
            return fw

        def snapshot(model, event, d):
            t = (d[0] - 1000) / 1e6
            if not lo <= t <= hi: return
            name = f"{d[0]:010d}_{event}.npz"
            np.savez_compressed(args.trace_out / name, cost=model.cost, lethal=model.last_lethal,
                                V=model.view[1], H=model.view[2], RO=model.view[3], diag=np.array(d),
                                truth=np.array([bot.x, bot.y, bot.z, bot.yaw]),
                                fpga_cx=model.cx, fpga_cy=model.cy, fpga_seq=model.seq,
                                commit_rt=np.array(current_rt), em_zmin=model.em.zmin,
                                em_zmax=model.em.zmax, em_cnt=model.em.cnt, em_tag=model.em.tag,
                                em_stamp=model.em.stamp, em_fc=model.em.fc)
            index.append(dict(t=t, event=event, file=name))

        current_rt = []

        def on_commit(model, rt, *a, **kw):
            nonlocal current_rt
            current_rt = list(rt)
            d = emit("before_commit")
            result = commit(model, rt, *a, **kw)
            snapshot(model, "commit", d)
            return result

        def on_update(model, *a, **kw):
            d = emit("match_update")
            snapshot(model, "match", d)
            return update(model, *a, **kw)

        def on_step(robot, *a, **kw):
            nonlocal bot
            bot = robot
            return step(robot, *a, **kw)

        ctypes.CDLL = load
        run_cl.fpga_model.FpgaModel.commit = on_commit
        run_cl.fpga_model.FpgaModel.match_update = on_update
        run_cl.rb.RobotSim.step = on_step
        sys.argv = [str(Path(run_cl.__file__)), *rest]
        try:
            result = run_cl.main()
        finally:
            ctypes.CDLL = loader
            run_cl.fpga_model.FpgaModel.commit = commit
            run_cl.fpga_model.FpgaModel.match_update = update
            run_cl.rb.RobotSim.step = step
            (args.trace_out / "frames.json").write_text(json.dumps(index, indent=2))
        return result


if __name__ == "__main__":
    raise SystemExit(main())
