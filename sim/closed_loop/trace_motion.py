"""Read-only diagnostic wrapper around the unchanged run_cl scorer/physics.

Usage: python trace_motion.py --trace-out DIR [run_cl.py arguments]
The firmware must export py_get_motion. It writes one diagnostic row/100 ms;
it does not feed any truth or diagnostics back into navigation.
"""
import argparse
import csv
import ctypes
import math
import sys
from pathlib import Path

import run_cl


def main():
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("--trace-out", type=Path, required=True)
    args, rest = ap.parse_known_args()
    backend = next((rest[i+1] for i,v in enumerate(rest[:-1]) if v == '--backend'), 'fast')
    if backend != 'fast' or '--backend=mujoco' in rest:
        ap.error('trace_motion supports the fast backend; MuJoCo writes its own execution trace')
    args.trace_out.mkdir(parents=True, exist_ok=True)
    original_loader = ctypes.CDLL
    original_step = run_cl.rb.RobotSim.step
    fw = None
    count = 0
    motion = (ctypes.c_int32 * 4)()
    state = (ctypes.c_int32 * 12)()
    path = (ctypes.c_int32 * 2)()

    def loader(*a, **kw):
        nonlocal fw
        fw = original_loader(*a, **kw)
        fw.py_get_motion.argtypes = [ctypes.POINTER(ctypes.c_int32)]
        return fw

    with (args.trace_out / "motion.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "source", "result", "recoveries", "path_complete",
                         "vx", "vy", "wz", "x", "y", "blocked", "collisions", "look_x", "look_y",
                         "goal_x", "goal_y", "mode", "state", "true_yaw_deg", "estimated_yaw_deg",
                         "path_fine", "path_points"])

        def step(bot, cmd, dt):
            nonlocal count
            original_step(bot, cmd, dt)
            count += 1
            if count % 100 == 0:
                fw.py_get_motion(motion)
                fw.py_get_state(state)
                path_diag = [None, None]
                if hasattr(fw, 'py_get_path_diag'):
                    fw.py_get_path_diag(path); path_diag = list(path)
                writer.writerow([round(count * dt, 4), *motion, *cmd, bot.x, bot.y,
                                 int(bot.blocked), bot.collisions, state[10], state[11],
                                 state[8], state[9], state[6], state[7], math.degrees(bot.yaw),
                                 (state[3] & 65535) * 360 / 65536, *path_diag])
                f.flush()

        ctypes.CDLL = loader
        run_cl.rb.RobotSim.step = step
        sys.argv = [str(Path(run_cl.__file__)), *rest]
        try:
            return run_cl.main()
        finally:
            ctypes.CDLL = original_loader
            run_cl.rb.RobotSim.step = original_step


if __name__ == "__main__":
    sys.exit(main())
