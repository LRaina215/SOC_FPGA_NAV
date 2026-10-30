import sys,numpy as np
sys.path.insert(0,'.')
import trav_tune as T
import trav_v3, nav_fast as fast, nav_fixed as nf
d,em=T.load('ed_b5.json'); cx,cy=d['lat_cx'],d['lat_cy']; base=T.cfg_from_regs(d['regs'])
N=nf.N
def analyse(cfg,label):
    c=dict(trav_v3.DEFAULT_V3); c.update(cfg)
    em2=T.EM()
    for k in ('zmin','zmax','cnt','tag','stamp','stale'): setattr(em2,k,getattr(em,k).copy())
    em2.fc=em.fc
    cost,lethal,(slot,V,H,RO)=trav_v3.trav_v3(em2,cx,cy,c)
    w=np.arange(N); X=((cx-N//2+w)+0.5)*0.05; Y=((cy-N//2+w)+0.5)*0.05; XX,YY=np.meshgrid(X,Y)
    obst=V&(H>150)
    near_obst=fast._dilate(obst,6)
    floor=V&(H<100)&(XX>0.35)&(XX<1.6)&(np.abs(YY)<0.8)&~near_obst
    cw=cost[slot]
    # criteria
    sh=fast._shift; step=np.zeros((N,N),dtype=np.int64)
    for dy in (-1,0,1):
        for dx in (-1,0,1):
            if dy or dx:
                v,h=sh(V,dy,dx,False),sh(H,dy,dx,0)
                delta=np.maximum(H-h,0) if c.get('step_high_only',0) else np.abs(h-H)
                step=np.maximum(step,np.where(v,delta,0))
    rx,_=trav_v3.gated_rise(V,H,1,c['slope_b'],c['jump']); ry,_=trav_v3.gated_rise(V,H,0,c['slope_b'],c['jump'])
    rise=np.maximum(rx,ry)+(np.minimum(rx,ry)>>1)
    f=floor
    print('%-34s floor n=%d lethal %4.1f%% blocked %4.1f%% | causes: step %4.1f%% rough %4.1f%% slope %4.1f%% other(drop) %4.1f%% | obstacles lethal %4.1f%%'%(
        label,f.sum(),100*lethal[f].mean(),100*(cw[f]>=253).mean(),100*(step[f]>c['step_max']).mean(),100*(RO[f]>c['rough_max']).mean(),
        100*(rise[f]>c['slope_rise']).mean(),100*(lethal[f]&~((step[f]>c['step_max'])|(RO[f]>c['rough_max'])|(rise[f]>c['slope_rise']))).mean(),100*lethal[obst].mean()))
    return cw,lethal,V,H,floor
analyse(base,'hardware now (20/50, cnt>=1)')
for st,ro,cm in ((30,70,2),(40,90,2),(40,90,3),(60,90,3),(40,120,3)):
    analyse(dict(base,step_max=st,rough_max=ro,cnt_min=cm),'step %d rough %d cnt>=%d'%(st,ro,cm))
