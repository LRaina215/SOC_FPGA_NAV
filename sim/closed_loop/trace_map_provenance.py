"""Read-only fixed-cell provenance recorder around a frozen closed-loop runner.

Tracks absolute writes as well as the 8-bit hardware stamp. No map, command,
sensor or score is changed. Coordinates are in the firmware odometry frame.
"""
import argparse
import csv
import ctypes
import json
from pathlib import Path
import sys

import numpy as np


def main():
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument('--runner-dir', type=Path, required=True)
    ap.add_argument('--trace-out', type=Path, required=True)
    ap.add_argument('--window', default='108,113')
    ap.add_argument('--cells', default='75,2;75,3;75,4;75,5;75,6;76,4')
    a, rest = ap.parse_known_args()
    sys.path.insert(0, str(a.runner_dir.resolve()))
    import run_cl
    lo, hi = map(float, a.window.split(','))
    cells = [tuple(map(int, v.split(','))) for v in a.cells.split(';')]
    a.trace_out.mkdir(parents=True, exist_ok=False)
    loader = ctypes.CDLL
    commit = run_cl.fpga_model.FpgaModel.commit
    frame = run_cl.fpga_model.FpgaModel.frame
    step = run_cl.rb.RobotSim.step
    fw = bot = None
    capture = None
    diag = (ctypes.c_int32 * 32)()
    writes = {}
    index = []
    with (a.trace_out/'cells.csv').open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time_s','frame','gx','gy','written','last_write_frame','last_write_s',
                         'age','cnt','tag','zmin','zmax','raw','x','y','yaw','dx','dy','dyaw',
                         'capture_x','capture_y','capture_yaw'])

        def load(*v, **kw):
            nonlocal fw
            fw = loader(*v, **kw)
            return fw

        def on_frame(model, *v, **kw):
            nonlocal capture
            capture = [bot.x,bot.y,bot.z,bot.yaw] if bot is not None else None
            return frame(model, *v, **kw)

        def on_step(robot, *v, **kw):
            nonlocal bot
            bot = robot
            return step(robot, *v, **kw)

        def on_commit(model, rt, *v, **kw):
            oldfc = int(model.em.fc)
            result = commit(model, rt, *v, **kw)
            if model.stage == 1:
                return result
            fw.py_get_frame_diag(diag)
            d = list(diag); t = (d[0]-1000)/1e6
            for gx, gy in cells:
                slot = ((gy & 127)<<7) | (gx & 127)
                touched = int(model.em.stamp[slot]) == oldfc and int(model.em.cnt[slot]) > 0
                if touched: writes[(gx,gy)] = (int(model.frame_id),t)
                w = writes.get((gx,gy),(-1,-1))
                wx,wy = gx-model.cx+64,gy-model.cy+64
                raw = int(model.last_lethal[wy,wx]) if 0<=wx<128 and 0<=wy<128 else -1
                cap = capture or [0,0,0,0]
                writer.writerow([t,model.frame_id,gx,gy,int(touched),*w,
                    (int(model.em.fc)-int(model.em.stamp[slot]))&255,
                    int(model.em.cnt[slot]),int(model.em.tag[slot]),
                    int(model.em.zmin[slot]),int(model.em.zmax[slot]),raw,
                    d[3],d[4],d[6],d[21],d[22],d[23],cap[0],cap[1],cap[3]])
            f.flush()
            if lo <= t <= hi:
                name = f'{d[0]:010d}.npz'
                np.savez_compressed(a.trace_out/name,cost=model.cost,lethal=model.last_lethal,
                    V=model.view[1],H=model.view[2],RO=model.view[3],diag=np.array(d),
                    truth=np.array([bot.x,bot.y,bot.z,bot.yaw]),capture=np.array(capture),
                    cx=model.cx,cy=model.cy,commit_rt=np.array(rt),
                    em_zmin=model.em.zmin,em_zmax=model.em.zmax,em_cnt=model.em.cnt,
                    em_tag=model.em.tag,em_stamp=model.em.stamp,em_fc=model.em.fc)
                index.append(dict(t=t,file=name))
            return result

        ctypes.CDLL = load
        run_cl.fpga_model.FpgaModel.commit = on_commit
        run_cl.fpga_model.FpgaModel.frame = on_frame
        run_cl.rb.RobotSim.step = on_step
        sys.argv = [str(Path(run_cl.__file__)),*rest]
        try:
            return run_cl.main()
        finally:
            ctypes.CDLL = loader
            run_cl.fpga_model.FpgaModel.commit = commit
            run_cl.fpga_model.FpgaModel.frame = frame
            run_cl.rb.RobotSim.step = step
            (a.trace_out/'frames.json').write_text(json.dumps(index,indent=2))


if __name__ == '__main__':
    raise SystemExit(main())
