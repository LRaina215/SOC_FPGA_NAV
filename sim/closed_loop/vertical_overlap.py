"""Host prototype for causal vertical registration against fresh prior cells.

All current points are retained for fusion. Selection is only for estimating
pose, not for deleting hazard points or extending terrain capability.
"""
import numpy as np


def estimate(model,points,valid,rt):
    p=np.asarray(points,dtype=np.int64);inv=model.inv_res;em=model.em
    gx=(p[:,0]*inv)>>16;gy=(p[:,1]*inv)>>16
    a=((gy&127)<<7)|(gx&127);tag=(((gy>>7)&15)<<4)|((gx>>7)&15)
    age=(int(em.fc)-em.stamp[a])&255
    radius=(p[:,0]-rt[9])**2+(p[:,1]-rt[10])**2
    span=em.zmax[a]-em.zmin[a]
    selected=valid&(em.tag[a]==tag)&(em.cnt[a]>=3)&~em.stale[a]&(age<=10)&(span<=18)
    selected&=(gx>=model.cx-64)&(gx<model.cx+64)&(gy>=model.cy-64)&(gy<model.cy+64)
    selected&=(radius>=200**2)&(radius<=1000**2)&(p[:,2]>=rt[11]-255)&(p[:,2]<=rt[11]+80)
    ix=np.flatnonzero(selected)
    if len(ix)<96:return dict(ok=False,cells=0,points=len(ix),reason='insufficient overlap')
    slots,back=np.unique(a[ix],return_inverse=True);counts=np.bincount(back);sums=np.bincount(back,weights=p[ix,2])
    x=np.bincount(back,weights=p[ix,0])/counts;y=np.bincount(back,weights=p[ix,1])/counts
    residual=(em.zmin[slots]+em.zmax[slots])*.5-sums/counts
    keep=(counts>=3)&(abs(residual)<=20);residual=residual[keep];x=x[keep];y=y[keep]
    if len(residual)<24:return dict(ok=False,cells=len(residual),points=len(ix),reason='insufficient independent cells')
    median=float(np.median(residual));mad=float(np.median(abs(residual-median)))
    if np.ptp(x)<150 or np.ptp(y)<200 or mad>4:return dict(ok=False,cells=len(residual),points=len(ix),reason='spread/consistency',median_mm=median,mad_mm=mad)
    # Fractional correction is accumulated by the prototype; target implementation
    # would need a fixed-point state and a matching hardware observation contract.
    return dict(ok=True,cells=len(residual),points=len(ix),median_mm=median,mad_mm=mad,
                correction_um=int(np.clip(round(median*250),-2000,2000)))
