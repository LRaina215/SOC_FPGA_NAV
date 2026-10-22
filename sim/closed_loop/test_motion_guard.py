"""Compare the actual C guard against an independent constant-twist geometry oracle.

Only the tiny test DLL is built here; the running closed-loop DLL is untouched.
The oracle uses exact SE(2) arcs sampled at 1 ms and a circle/closed-cell test,
without calling the planner, its raster walk, or the terrain classifier.
"""
import ctypes
import argparse
import json
import math
from pathlib import Path
import subprocess

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "sim/build/nav_opt/motion_geometry"
WRAPPER = r'''
#include "motion_guard.h"
#include "support_guard.h"
/* This suite isolates body geometry; wheel-support contracts run separately. */
int support_check(const nav_pose_t *p,const lp_cfg_t *c,const hal_map_info_t *m,const nav_cmd_t *cmd)
{return SUPPORT_PASS;}
bool hal_map_info(hal_map_info_t *mi) { return false; }
bool hal_traj_eval(const hal_cand_t *c, int k, int t, int32_t x, int32_t y,
                   angle_t th, uint8_t mx, hal_traj_res_t *r) { return false; }
__declspec(dllexport) int check(const uint8_t *map, int x, int y, int yaw, int v, int w)
{
    lp_cfg_t cfg; lp_default_cfg(&cfg);
    hal_map_info_t mi = {1, 0, 0, 49, 1311};
    nav_pose_t p = {0}; p.x=x; p.y=y; p.yaw=(angle_t)yaw;
    nav_cmd_t cmd = {(int16_t)v, 0, (int16_t)w, 0};
    return motion_check(map, &mi, &p, &cfg, &cmd);
}
__declspec(dllexport) int segment(const uint8_t *map, int x, int y, int gx, int gy)
{
    lp_cfg_t cfg; lp_default_cfg(&cfg);
    hal_map_info_t mi = {1, 0, 0, 49, 1311};
    nav_pose_t p = {0}; p.x=x; p.y=y;
    return motion_segment(map, &mi, &p, &cfg, gx, gy);
}
'''


def main():
    global OUT
    ap = argparse.ArgumentParser(); ap.add_argument('--out', type=Path, default=OUT)
    ap.add_argument('--fw',type=Path,default=ROOT/'fw')
    args=ap.parse_args();OUT=args.out.resolve()
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "wrapper.c").write_text(WRAPPER, encoding="utf-8")
    dll = OUT / "guard.dll"
    fw = args.fw.resolve()
    subprocess.run(["gcc", "-std=c99", "-O2", "-shared", "-o", str(dll),
                    *[f"-I{fw / d}" for d in ("include", "hal", "nav")],
                    str(OUT / "wrapper.c"), str(fw / "nav/motion_guard.c"),
                    str(fw / "nav/local_planner.c"), str(fw / "nav/fixmath.c")], check=True)
    lib = ctypes.CDLL(str(dll))
    lib.check.argtypes = [ctypes.POINTER(ctypes.c_uint8), *([ctypes.c_int] * 5)]
    lib.check.restype = ctypes.c_int
    lib.segment.argtypes = [ctypes.POINTER(ctypes.c_uint8), *([ctypes.c_int] * 4)]
    lib.segment.restype = ctypes.c_int
    grid = np.zeros((128, 128), dtype=np.uint8)
    ptr = grid.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))
    res = 65536 / 1311
    t = np.linspace(0, 1.2, 1201)
    counts = dict(cases=0, oracle_collision=0, accepted=0, conservative_reject=0)
    misses = []
    for v in (-150, 125, 250, 375, 500):
        for w in (-1200, -600, -300, 0, 300, 600, 1200):
            for yaw in (0, 1, 16384, 32767, 65535):
                a = yaw * 2 * math.pi / 65536
                for px, py in ((0, 0), (24, 49), (49, 0)):
                    if w:
                        ang = a + w / 1000 * t
                        xs = px + v / (w / 1000) * (np.sin(ang) - math.sin(a))
                        ys = py - v / (w / 1000) * (np.cos(ang) - math.cos(a))
                    else:
                        xs = px + v * t * math.cos(a)
                        ys = py + v * t * math.sin(a)
                    cells = set()
                    for j in (0, 600, 1200):
                        heading = a + w / 1000 * t[j]
                        for side in (-1, 1):
                            for offset in (-5, 0, 5, 20):
                                ox = xs[j] - side * (210 + offset) * math.sin(heading)
                                oy = ys[j] + side * (210 + offset) * math.cos(heading)
                                cells.add((math.floor(ox / res), math.floor(oy / res)))
                    for cx, cy in sorted(cells):
                        grid[cy + 64, cx + 64] = 254
                        status = lib.check(ptr, px, py, yaw, v, w)
                        grid[cy + 64, cx + 64] = 0
                        dx = np.maximum(np.maximum(cx * res - xs, xs - (cx + 1) * res), 0)
                        dy = np.maximum(np.maximum(cy * res - ys, ys - (cy + 1) * res), 0)
                        collision = bool(np.min(dx * dx + dy * dy) <= 210 ** 2)
                        counts["cases"] += 1
                        counts["oracle_collision"] += collision
                        counts["accepted"] += status == 1
                        counts["conservative_reject"] += not collision and status != 1
                        if collision and status == 1:
                            misses.append(dict(v=v, w=w, yaw=yaw, phase=[px, py], cell=[cx, cy]))
    report = dict(**counts, false_accept=len(misses), misses=misses,
                  oracle="exact constant twist, 1 ms samples, radius210mm vs closed50mm cells",
                  limitation="nominal motion only; no actuation, braking, localization or terrain validity proof")
    (OUT / "result.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({k: v for k, v in report.items() if k != "misses"}, indent=2))
    seg = dict(cases=0, oracle_collision=0, accepted=0, conservative_reject=0, false_accept=0)
    for deg in (0, 7, 15, 37, 63, 90, 135, 179, 225):
        a = math.radians(deg)
        for length in (0, 50, 400, 800, 1500):
            for px, py in ((0, 0), (24, 49), (-49, -24)):
                gx, gy = round(px + length * math.cos(a)), round(py + length * math.sin(a))
                u = np.linspace(0, 1, max(2, length * 4 + 1))
                xs, ys = px + (gx-px)*u, py + (gy-py)*u
                cells = set()
                for f in (.013, .117, .5, .99):
                    for side in (-1, 1):
                        for offset in (-2, 0, 2, 10, 80):
                            ox = px+(gx-px)*f-side*(210+offset)*math.sin(a)
                            oy = py+(gy-py)*f+side*(210+offset)*math.cos(a)
                            cells.add((math.floor(ox/res), math.floor(oy/res)))
                for cx, cy in cells:
                    grid[cy+64,cx+64] = 254
                    status = lib.segment(ptr, px, py, gx, gy)
                    grid[cy+64,cx+64] = 0
                    dx = np.maximum(np.maximum(cx*res-xs,xs-(cx+1)*res),0)
                    dy = np.maximum(np.maximum(cy*res-ys,ys-(cy+1)*res),0)
                    collision = bool(np.min(dx*dx+dy*dy) <= 210**2)
                    seg['cases'] += 1; seg['oracle_collision'] += collision
                    seg['accepted'] += status == 1
                    seg['false_accept'] += collision and status == 1
                    seg['conservative_reject'] += not collision and status != 1
    seg['oracle'] = 'Independent linear interpolation <=0.25mm spacing and circle/closed-cell geometry'
    (OUT/'segment_result.json').write_text(json.dumps(seg,indent=2)); print(json.dumps(seg))
    low = dict(cases=0, oracle_collision=0, accepted=0, false_accept=0, misses=[])
    for v in (-75,-50,-20,-10,-5,-2,-1,1,2,5,10,20,50,75):
        cx = 5 if v > 0 else -6
        for phase in range(50):
            px = phase if v > 0 else -phase
            for w in (-1200,0,1200):
                if w:
                    ang=w/1000*t
                    xs=px+v/(w/1000)*np.sin(ang);ys=v/(w/1000)*(1-np.cos(ang))
                else: xs=px+v*t;ys=np.zeros_like(xs)
                grid[64,cx+64]=254
                status=lib.check(ptr,px,0,0,v,w)
                grid[64,cx+64]=0
                dx=np.maximum(np.maximum(cx*res-xs,xs-(cx+1)*res),0)
                dy=np.maximum(np.maximum(-ys,ys-res),0)
                collision=bool(np.min(dx*dx+dy*dy)<=210**2)
                low['cases']+=1;low['oracle_collision']+=collision;low['accepted']+=status==1
                if collision and status==1:low['misses'].append(dict(v=v,w=w,phase_mm=px))
    low['false_accept']=len(low['misses'])
    low['oracle']='Independent exact constant-twist path, 1ms samples; signed low velocities/front-cell phase sweep'
    (OUT/'low_velocity_result.json').write_text(json.dumps(low,indent=2))
    print(json.dumps({k:v for k,v in low.items() if k!='misses'}))
    return bool(misses) or bool(seg['false_accept']) or bool(low['false_accept'])


if __name__ == "__main__":
    raise SystemExit(main())
