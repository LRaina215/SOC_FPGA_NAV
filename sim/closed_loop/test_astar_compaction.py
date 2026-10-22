"""Exact path regression for the 4-bit parent store, using frozen C sources."""
import argparse
import ctypes as c
import json
import subprocess
from pathlib import Path
import numpy as np


class Config(c.Structure):
    _fields_ = [('block', c.c_uint8), ('unknown_cost', c.c_uint16),
                ('cost_scale', c.c_uint8), ('allow_unknown', c.c_bool),
                ('clear_cells', c.c_uint8), ('clear_cost', c.c_uint16),
                ('near_cells', c.c_uint8), ('near_cost', c.c_uint16)]


class Result(c.Structure):
    _fields_ = [('n', c.c_int), ('x', c.c_int16 * 256), ('y', c.c_int16 * 256)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('before', type=Path); ap.add_argument('after', type=Path)
    ap.add_argument('--out', type=Path, required=True); a = ap.parse_args()
    build = a.out.parent / 'astar_compaction'; build.mkdir(parents=True, exist_ok=True)
    wrapper = build / 'wrapper.c'
    wrapper.write_text('''#include "astar.h"
__declspec(dllexport) void test_default(astar_cfg_t *c) { astar_default_cfg(c); }
__declspec(dllexport) int test_plan(const astar_cfg_t *c, const uint8_t *g,
 int n, int sx, int sy, int gx, int gy, astar_path_t *p) {
 return astar_grid(c,g,n,sx,sy,gx,gy,p); }
__declspec(dllexport) int test_expanded(void) { return astar_last_expanded(); }
''')
    libs = []
    for i, fw in enumerate((a.before, a.after)):
        dll = (build / f'astar_{i}.dll').resolve()
        subprocess.run(['gcc', '-std=c99', '-O2', '-shared', '-o', str(dll),
                        '-I'+str(fw/'include'), '-I'+str(fw/'hal'), '-I'+str(fw/'nav'),
                        str(wrapper), str(fw/'nav/astar.c'), str(fw/'nav/scratch.c')], check=True)
        libs.append(c.CDLL(str(dll)))
    for lib in libs:
        lib.test_default.argtypes = [c.POINTER(Config)]
        lib.test_plan.argtypes = [c.POINTER(Config), c.POINTER(c.c_uint8),
                                  *([c.c_int] * 5), c.POINTER(Result)]
    rng = np.random.default_rng(20261021); count = 0
    for n in (16, 31, 32, 63, 64):
        for density in (.05, .2, .45):
            for seed in range(20):
                grid = rng.integers(0, 70, size=(n, n), dtype=np.uint8)
                grid[rng.random((n, n)) < density] = 254
                grid[rng.random((n, n)) < .1] = 255
                endpoints = list(map(int, rng.integers(0, n, size=4)))
                out = []
                for lib in libs:
                    cfg = Config(); lib.test_default(c.byref(cfg))
                    cfg.allow_unknown = bool(seed % 2)
                    cfg.clear_cells=(0,3,15)[seed%3]
                    cfg.near_cells=(seed*7)%16
                    cfg.near_cost=150
                    result = Result()
                    rc = lib.test_plan(c.byref(cfg), grid.ctypes.data_as(c.POINTER(c.c_uint8)),
                                        n, *endpoints, c.byref(result))
                    out.append((rc, result.n, list(result.x[:result.n]), list(result.y[:result.n]),
                                lib.test_expanded()))
                assert out[0] == out[1], (n, density, seed)
                count += 1
    long_count=0
    for n in (31,32,63,64):
        grid=np.full((n,n),254,dtype=np.uint8);snake=[]
        ys=list(range(1,n-1,2))
        for j,y in enumerate(ys):
            xs=range(1,n-1) if j%2==0 else range(n-2,0,-1)
            for x in xs:grid[y,x]=0;snake.append((x,y))
            if j+1<len(ys):grid[y+1,x]=0;snake.append((x,y+1))
        for index in (254,255,256,257,511,512,513,len(snake)-1):
            if index>=len(snake):continue
            out=[]
            for lib in libs:
                cfg=Config();lib.test_default(c.byref(cfg));cfg.clear_cells=cfg.near_cells=0
                p=Result();rc=lib.test_plan(c.byref(cfg),grid.ctypes.data_as(c.POINTER(c.c_uint8)),n,
                    *snake[0],*snake[index],c.byref(p))
                out.append((rc,p.n,list(p.x[:p.n]),list(p.y[:p.n]),lib.test_expanded()))
            assert out[0]==out[1],('long',n,index)
            # Existing A* permits a diagonal when only one orthogonal neighbor
            # is blocked, so a raster snake is not its exact optimal path.
            # The frozen old full sequence is the reference for preservation.
            assert (out[1][2][0],out[1][3][0])==snake[0],('prefix_start',n,index)
            if out[1][0]==1 and out[1][1]==256:
                assert (out[1][2][-1],out[1][3][-1])!=snake[index],('truncation',n,index)
            long_count+=1
    report = dict(cases=count,long_prefix_cases=long_count,exact_paths_and_expansions=True,
                  scope='Cache representation, clearance0..15; not an optimality or physical safety oracle.')
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(report, indent=2)); print(json.dumps(report))


if __name__ == '__main__': main()
