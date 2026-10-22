"""Unknown confidence versus independently sampled newly occupied geometry.

A single unknown cell partly overlaps the starting body. Boundary and corner
witnesses sampled at 1ms must not be reported as known when newly occupied.
No hypothetical hazard is injected. This is finite evidence, not WCET/proof.
"""
import argparse
import ctypes
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
RES=65536/1311


def witness(x,y,yaw,v,w,gx,gy):
    t=np.linspace(0,.2,201);a0=yaw*math.tau/65536;a=a0+w/1000*t
    if w:xx=x+v/(w/1000)*(np.sin(a)-math.sin(a0));yy=y-v/(w/1000)*(np.cos(a)-math.cos(a0))
    else:xx=x+v*math.cos(a0)*t;yy=y+v*math.sin(a0)*t
    b=np.linspace(0,math.tau,361)
    X=xx[:,None]+210*np.cos(b);Y=yy[:,None]+210*np.sin(b)
    inside=(X>=gx*RES-1e-9)&(X<=(gx+1)*RES+1e-9)&(Y>=gy*RES-1e-9)&(Y<=(gy+1)*RES+1e-9)
    if np.any(inside & ((X-x)**2+(Y-y)**2>210**2+1e-7)):return True
    for cx,cy in ((gx*RES,gy*RES),((gx+1)*RES,gy*RES),(gx*RES,(gy+1)*RES),((gx+1)*RES,(gy+1)*RES)):
        if (cx-x)**2+(cy-y)**2>210**2+1e-7 and np.any((xx-cx)**2+(yy-cy)**2<=210**2+1e-7):return True
    return False


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--before',type=Path,required=True)
    ap.add_argument('--source',type=Path,default=ROOT/'fw');ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--speed',type=int,help='指定速度的新增定例；保留原随机数序列和默认批次')
    a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=False)
    spec=importlib.util.spec_from_file_location('geometry',ROOT/'sim/closed_loop/test_motion_guard.py')
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    c=a.out/'wrapper.c';c.write_text(m.WRAPPER)
    cases=[];rng=np.random.default_rng(3281)
    for i in range(256):
        v=int(rng.choice([1,2,5,50,125,375,500]));w=int(rng.choice([0,-150,150,-600,600,-1200,1200]))
        yaw=int(rng.choice([0,1,8192,16384,32767,49152,65535]));x=int(rng.choice([-49,-25,0,24,49]));y=int(rng.choice([-25,0,24]))
        if a.speed is not None:v=a.speed
        b=(i%16)*math.tau/16;gx=math.floor((x+210*math.cos(b))/RES);gy=math.floor((y+210*math.sin(b))/RES)
        cases.append((x,y,yaw,v,w,gx,gy,witness(x,y,yaw,v,w,gx,gy)))
    rows=[]
    for label,fw in [('before',a.before.resolve()),('after',a.source.resolve())]:
        dll=(a.out/(label+'.dll')).resolve()
        subprocess.run(['gcc','-std=c99','-O2','-shared','-o',str(dll),*['-I'+str(fw/d) for d in ('include','hal','nav')],str(c),
            *[str(fw/'nav'/n) for n in ('motion_guard.c','local_planner.c','fixmath.c')]],check=True)
        lib=ctypes.CDLL(str(dll));lib.check.argtypes=[ctypes.POINTER(ctypes.c_uint8),*[ctypes.c_int]*5]
        grid=np.zeros((128,128),dtype=np.uint8);ptr=grid.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8));bad=[];n_witness=unknown=0
        for x,y,yaw,v,w,gx,gy,hit in cases:
            grid[gy+64,gx+64]=255;status=lib.check(ptr,x,y,yaw,v,w);grid[gy+64,gx+64]=0
            n_witness+=hit;unknown+=status==5
            if hit and status==1:bad.append(dict(p=[x,y,yaw],v=v,w=w,cell=[gx,gy]))
        rows.append(dict(version=label,cases=len(cases),witness_cases=n_witness,unknown=unknown,false_known=len(bad),examples=bad[:5]))
    assert rows[0]['false_known']>0 and rows[1]['false_known']==0,rows
    report=dict(scope='1ms independent boundary/corner witnesses; no hazard injection or physical guarantee',rows=rows)
    (a.out/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))


if __name__=='__main__':main()
