"""Analytic flat-plane observability using the existing XT-M60 ray model.

This is a diagnostic, never an input map. No rollout or score is modified.
"""
import argparse
import json
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'model'))
import lidar_lut
import sim_world as sw


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    a.out.mkdir(parents=True,exist_ok=False)
    R,t=sw.mount_pose();unit=lidar_lut.rays(cut=0).reshape(-1,3)
    ray=unit@R.T;down=ray[:,2]<-1e-8;distance=-t[2]/ray[down,2]
    point=t+ray[down]*distance[:,None];point=point[(distance>=.1)&(distance<=8)]
    radius=np.hypot(point[:,0],point[:,1])*1000
    # The C declaration only substitutes cells fully inside its disk.
    res=65536/1311;idx=np.arange(-20,21);ix,iy=np.meshgrid(idx,idx)
    cx,cy=(ix+.5)*res,(iy+.5)*res
    observed=np.zeros(ix.shape,dtype=bool)
    for yaw in np.deg2rad(np.arange(0,360,2)):
        c,s=np.cos(yaw),np.sin(yaw);x=1000*(point[:,0]*c-point[:,1]*s);y=1000*(point[:,0]*s+point[:,1]*c)
        gx=np.floor(x/res).astype(int)+20;gy=np.floor(y/res).astype(int)+20
        good=(gx>=0)&(gx<41)&(gy>=0)&(gy<41);observed[gy[good],gx[good]]=True
    rows=[]
    for boot in (350,400):
        known_boot=(np.abs(cx)+res/2+1)**2+(np.abs(cy)+res/2+1)**2<=boot**2
        known=known_boot|observed
        first_failure=None
        for body_x in range(0,601,5):
            ok=True
            for xx in range(body_x,body_x+61,5):
                for yy in (-135,135):
                    dx=np.maximum(0,np.maximum(ix*res-xx,xx-(ix+1)*res));dy=np.maximum(0,np.maximum(iy*res-yy,yy-(iy+1)*res))
                    if np.any((dx*dx+dy*dy<=35**2)&~known):ok=False;break
                if not ok:break
            if not ok:first_failure=body_x;break
        rows.append(dict(declared_radius_mm=boot,first_uncertified_body_x_mm=first_failure,
                         scope='old 35mm circular prototype; ideal uncut-ray plane scan; 50mm/s x1.2s preview'))
    report=dict(scope='ideal ray geometry only, no physical calibration or navigation truth input',
        mount_pitch_x_z=[sw.MOUNT_PITCH_DEG,sw.MOUNT_X*1000,sw.MOUNT_H*1000],
        sensor_min_range_m=.1,ground_returns=len(point),nearest_plane_return_radius_mm=float(radius.min()),
        nearest_plane_return_x_mm=float(point[:,0].min()*1000),rows=rows)
    (a.out/'result.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    np.savez(a.out/'coverage.npz',x=cx,y=cy,observed=observed)
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
