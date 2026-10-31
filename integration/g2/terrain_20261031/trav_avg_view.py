import sys,numpy as np
sys.path.insert(0,'.')
import trav_tune as T
import trav_v3, nav_fast as fast, nav_fixed as nf
fn=sys.argv[1]
d,em=T.load(fn); cx,cy=d['lat_cx'],d['lat_cy']; base=T.cfg_from_regs(d['regs'])
N=nf.N
print('avg', em.avg, 'cfg', {k:base[k] for k in ('step_max','rough_max','cnt_min','avg','avg_shift')})
c=dict(trav_v3.DEFAULT_V3); c.update(base)
em2=T.EM()
for k in ('zmin','zmax','cnt','tag','stamp','stale','avg','avg_shift_max'): setattr(em2,k,getattr(em,k).copy() if hasattr(getattr(em,k),'copy') else getattr(em,k))
em2.fc=em.fc
cost,L,(slot,V,H,RO)=trav_v3.trav_v3(em2,cx,cy,c)
hw=T.window(np.array(d['cost']),cx,cy); cw=cost[slot]
known=(hw!=255)|(cw!=255)
print('model vs hardware: identical %.1f%%, lethal agree %.1f%%'%(100*(hw[known]==cw[known]).mean(),100*((hw[known]==254)==(cw[known]==254)).mean()))
w=np.arange(N); X=((cx-N//2+w)+0.5)*0.05; Y=((cy-N//2+w)+0.5)*0.05; XX,YY=np.meshgrid(X,Y)
floor=V&(XX>0.35)&(XX<1.3)&(YY>-0.4)&(YY<0.25)&(H<80)
print('floor cells',floor.sum(),' H p10/50/90',np.percentile(H[floor],[10,50,90]),' MAD p50/90/99',np.percentile(RO[floor],[50,90,99]))
sh=fast._shift
step=np.zeros((N,N),dtype=np.int64)
for dy in (-1,0,1):
    for dx in (-1,0,1):
        if dy or dx:
            v,h=sh(V,dy,dx,False),sh(H,dy,dx,0); step=np.maximum(step,np.where(v,np.maximum(H-h,0),0))
print('floor step p50/90/99',np.percentile(step[floor],[50,90,99]),' >20: %.1f%%'%(100*(step[floor]>20).mean()))
cols=[i for i in range(N) if -0.1<X[i]<2.4]; rows=[j for j in range(N) if -1.0<Y[j]<1.0]
print('legend: .=unknown _=free o=inflated  #=lethal&H>150  S=step R=rough D=other')
for j in reversed(rows):
    s=''
    for i in cols:
        if not L[j,i]: s+='.' if cw[j,i]==255 else ('o' if cw[j,i]>=253 else '_')
        elif H[j,i]>150: s+='#'
        elif step[j,i]>c['step_max']: s+='S'
        elif RO[j,i]>c['rough_max']: s+='R'
        else: s+='D'
    print('%+.2f '%Y[j]+s)
print('H map (cm) along rows y=+0.03 and -0.28:')
for yy in (0.025,-0.275):
    j=int(np.argmin(abs(Y-yy))); print('%+.2f '%Y[j]+' '.join('%3s'%('.' if not V[j,i] else int(H[j,i]//10)) for i in cols))
