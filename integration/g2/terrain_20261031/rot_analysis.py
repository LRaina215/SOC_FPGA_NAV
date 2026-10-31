import sys, json, math, numpy as np
sys.path.insert(0,'.')
import trav_tune as T, trav_v3, nav_fast as fast, nav_fixed as nf
N=nf.N
def cells(fn):
    d,em=T.load(fn); cx,cy=d['lat_cx'],d['lat_cy']; c=dict(trav_v3.DEFAULT_V3); c.update(T.cfg_from_regs(d['regs']))
    slot,V,H,RO=fast.window_view(em,cx,cy,c,kill=False)
    w=np.arange(N); X=((cx-N//2+w)+0.5)*0.05; Y=((cy-N//2+w)+0.5)*0.05; XX,YY=np.meshgrid(X,Y)
    return XX[V],YY[V],H[V].astype(float)
pose={'A':(0.002,-0.029,math.radians(3.57)),'B':(-0.066,-0.107,math.radians(-10.23))}
XA,YA,HA=cells('ed_rot_A.json'); XB,YB,HB=cells('ed_rot_B.json')
def grid(X,Y,H,thr):
    m=H>thr; return set(zip(np.round(X[m]/0.05).astype(int),np.round(Y[m]/0.05).astype(int)))
OA=grid(XA,YA,HA,150)
best=None
for dyaw in np.arange(-45,45.5,1.0):
    c,s=math.cos(math.radians(dyaw)),math.sin(math.radians(dyaw))
    m=HB>150; xb,yb=XB[m],YB[m]
    xr,yr=c*xb-s*yb, s*xb+c*yb
    for dx in np.arange(-0.3,0.31,0.05):
        for dy in np.arange(-0.3,0.31,0.05):
            g=set(zip(np.round((xr+dx)/0.05).astype(int),np.round((yr+dy)/0.05).astype(int)))
            sc=len(g&OA)/max(1,len(g))
            if best is None or sc>best[0]: best=(sc,dyaw,dx,dy)
sc,dyaw,dx,dy=best
print('obstacle cells A %d B %d; best world alignment B->A: rotate %+.0f deg, shift (%.2f, %.2f) m, overlap %.0f%%'%(len(OA),int((HB>150).sum()),dyaw,dx,dy,100*sc))
c,s=math.cos(math.radians(dyaw)),math.sin(math.radians(dyaw))
def apply(X,Y): return c*X-s*Y+dx, s*X+c*Y+dy
# true robot poses in A's world frame: A as reported; B = alignment applied to B's reported pose
xBw,yBw=apply(np.array([pose['B'][0]]),np.array([pose['B'][1]])); yawBw=pose['B'][2]+math.radians(dyaw)
print('robot rotation A->B in the scene: %+.1f deg (nav pose said %+.1f deg)'%(math.degrees(yawBw-pose['A'][2]),math.degrees(pose['B'][2]-pose['A'][2])))
def floor_dict(X,Y,H): 
    m=(H<80)
    return {(int(round(x/0.05)),int(round(y/0.05))):h for x,y,h in zip(X[m],Y[m],H[m])}
def to_robot(X,Y,px,py,yaw):
    c2,s2=math.cos(-yaw),math.sin(-yaw); dx_,dy_=X-px,Y-py
    return c2*dx_-s2*dy_, s2*dx_+c2*dy_
def compare(name, A, B, ra):
    keys=[k for k in A if k in B and 0.45<ra(k)<1.5]
    a=np.array([A[k] for k in keys]); b=np.array([B[k] for k in keys])
    if len(keys)<20: print(name,'too few',len(keys)); return
    r=np.corrcoef(a-a.mean(),b-b.mean())[0,1]
    print('%-34s overlap %4d cells  corr %+.2f  rms diff %.1f mm  (std A %.1f, B %.1f)'%(name,len(keys),r,np.sqrt(np.mean((a-b)**2)),a.std(),b.std()))
# scene-fixed comparison: both in A's world frame
XBa,YBa=apply(XB,YB)
FA=floor_dict(XA,YA,HA); FBw=floor_dict(XBa,YBa,HB)
rngA=lambda k: math.hypot(k[0]*0.05-pose['A'][0],k[1]*0.05-pose['A'][1])
compare('same floor spot (scene-fixed)',FA,FBw,rngA)
# sensor-fixed comparison: each in its own robot frame
xa,ya=to_robot(XA,YA,*pose['A']); xb2,yb2=to_robot(XBa,YBa,float(xBw[0]),float(yBw[0]),yawBw)
FAr=floor_dict(xa,ya,HA); FBr=floor_dict(xb2,yb2,HB)
compare('same sensor direction (sensor-fixed)',FAr,FBr,lambda k: math.hypot(k[0]*0.05,k[1]*0.05))
