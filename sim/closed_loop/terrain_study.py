"""Reproducible fixed4500 terrain trials, independent of navigation decisions.

The true-pose map is a diagnostic branch only. Raw frames and transforms are
retained for identical-input replay. No ground truth is sent to a controller.
"""
import argparse
import json
import math
from dataclasses import asdict
from pathlib import Path
import numpy as np
from terrain_scene import TerrainSpec,TerrainScene,S
from terrain_physics import TerrainRobot
import sim_world as sw
import nav_fixed as nf
import nav_fast as fast
from fpga_model import FpgaModel

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'sim/build/terrain4500_20261022'
RES=65536/1311/1000


def dump(path,obj):
    Path(path).write_text(json.dumps(obj,indent=2,ensure_ascii=False),encoding='utf-8')


class MapRecorder:
    def __init__(self,scene,out,noise=4.,seed=3,cfg=None):
        options=dict(infl_r=4,step_high_only=1,local_frame=1);options.update(cfg or {})
        self.scene=scene;self.out=Path(out);self.fm=FpgaModel(cfg=options)
        self.lid=sw.VirtualLidar(noise_mm=noise,seed=seed+7,rng_max=8.)
        self.mount=sw.mount_pose();self.frames=[];self.maps=[];self.rows=[]

    def capture(self,bot,t):
        R,p=bot.body_pose();R,p=sw.compose(R,p,*self.mount)
        d,_=self.lid.capture(self.scene.w,R,p)
        self.add(d,R,p,t,np.array([bot.x,bot.y,bot.physical_z,bot.yaw]))

    def add(self,d,R,p,t,truth):
        fm=self.fm;fm.frame(d);Rq,tq=sw.to_fixed_pose(R,p)
        cx,cy=nf.world_cell(round(truth[0]*1000),1311),nf.world_cell(round(truth[1]*1000),1311)
        fm.commit([v for row in Rq for v in row]+tq,cx,cy,-1000,2000)
        self.frames.append((t,d.astype(np.uint16),np.array(R),np.array(p),truth))
        self.observe(fm,t,truth)

    def observe(self,fm,t,truth,transform=None):
        slot,V,H,RO=fm.view;cost=fast.cost_window(fm.cost,fm.cx,fm.cy)
        age=((int(fm.em.fc)-fm.em.stamp[slot])&255).astype(np.uint8)
        self.maps.append((t,fm.cx,fm.cy,V.copy(),H.astype(np.int16),RO.astype(np.int16),fm.last_lethal.copy(),cost.astype(np.uint8),age,fm.em.cnt[slot].astype(np.uint8),fm.em.stale[slot].copy()))
        X,Y=np.meshgrid((np.arange(128)+fm.cx-64+.5)*RES,(np.arange(128)+fm.cy-64+.5)*RES)
        if transform is not None:X,Y=transform(X,Y)
        # Fixed corridor, same denominator for every frame/version. Include
        # discontinuities separately; never hide difficult boundaries as ignore.
        roi=(X>=-.2)&(X<=self.scene.goal_x+.1)&(abs(Y)<=.4)
        danger,boundary=self.scene.geometric_labels(X,Y,RES)
        free=roi&~danger&~boundary;dg=roi&danger;ob=roi&V
        valid=V&roi&~boundary
        err=H/1000+(self.scene.initial_height if transform is not None else 0)-self.scene.height(X,Y)
        self.rows.append(dict(t=t,roi=int(roi.sum()),observed=int(ob.sum()),unknown=int((roi&~V).sum()),
            boundary=int((roi&boundary).sum()),danger=int(dg.sum()),observed_danger=int((dg&V).sum()),
            raw_tp=int((dg&V&fm.last_lethal).sum()),blocked_tp=int((dg&V&(cost>=253)&(cost!=255)).sum()),
            free=int(free.sum()),observed_free=int((free&V).sum()),raw_fp=int((free&V&fm.last_lethal).sum()),
            blocked_fp=int((free&V&(cost>=253)&(cost!=255)).sum()),
            boundary_observed=int((roi&boundary&V).sum()),boundary_raw=int((roi&boundary&fm.last_lethal).sum()),
            unknown_but_cost_free=int((roi&~V&(cost<253)).sum()),height_count=int(valid.sum()),
            height_abs_sum_mm=float(np.abs(err[valid]*1000).sum()),height_max_mm=float(np.abs(err[valid]*1000).max()) if valid.any() else None,
            stale_observed=int((roi&V&fm.em.stale[slot]).sum()),age_max=int(age[ob].max()) if ob.any() else None))

    def close(self,label='ideal_pose_diagnostic_only'):
        self.out.mkdir(parents=True,exist_ok=True)
        if self.frames:
            f=self.frames;np.savez_compressed(self.out/'raw_frames.npz',t=[a[0] for a in f],depth=[a[1] for a in f],R=[a[2] for a in f],p=[a[3] for a in f],truth=[a[4] for a in f])
        if self.maps:
            m=self.maps;names=['t','cx','cy','V','H','RO','raw','cost','age','cnt','stale']
            np.savez_compressed(self.out/'map_frames.npz',**{n:np.array([a[i] for a in m]) for i,n in enumerate(names)})
        dump(self.out/'map_metrics.json',dict(branch=label,roi='fixed world corridor x[-.2,goal+.1], y[-.4,.4]',
            counts='repeated cell-observations, not independent physical trials',frames=self.rows))


def geometry_contract(out):
    results=[]
    for spec in [TerrainSpec('flat'),TerrainSpec('ramp_up',8),TerrainSpec('ramp_down',8),TerrainSpec('ramp_pair',8),TerrainSpec('step_up',.02),TerrainSpec('step_down',.02),TerrainSpec('trench',.04)]:
        sc=TerrainScene(spec);bot=TerrainRobot(sc,out=out/spec.name,seed=3)
        x=np.linspace(-.5,2.5,121)+.0013;y=np.array([-.8,-.3,0,.3,.8])
        err=[]
        for a in x:
            for b in y:
                ray=sc.w.raycast([a,b,1.],np.array([[0.,0.,-1.]]))[0]
                err.append(abs((1.-ray)-sc.height(a,b)))
        planes=[bot.model.geom(i).name for i in range(bot.model.ngeom) if bot.model.geom_type[i]==bot.rt.mujoco.mjtGeom.mjGEOM_PLANE]
        row=dict(kind=spec.kind,vertical_rays=len(err),max_error_m=float(max(err)),remaining_planes=planes,
            trench_center_height_m=float(sc.height(spec.entry+spec.value/2,0)) if spec.kind=='trench' else None)
        assert max(err)<2e-6 and not planes,row
        rng=np.random.default_rng(42);origin=np.array([.1,0.,.5])
        targets=np.column_stack((rng.uniform(.2,2,1500),rng.uniform(-.6,.6,1500),np.zeros(1500)))
        vec=targets-origin;vec/=np.linalg.norm(vec,axis=1)[:,None]
        expected=analytic_rays(sc,origin,vec);actual=sc.w.raycast(origin,vec)
        oblique=float(np.max(abs(expected-actual)));assert oblique<2e-6,(spec.kind,oblique)
        row.update(oblique_rays=len(vec),oblique_max_error_m=oblique)
        bot.close();results.append(row)
    dump(out/'geometry_contract.json',results);print(json.dumps(results),flush=True)


def analytic_rays(sc,origin,vec):
    """Independent continuous profile intersections, including vertical walls."""
    s=sc.spec;candidates=[]
    planes=[(0.,0.),(0.,s.value),(0.,-s.depth)]
    if s.kind.startswith('ramp'):
        a=sc.rise/s.run
        planes += [(a,-a*s.entry),(-a,a*(s.entry+s.run)),(-a,a*sc.end),(0.,sc.rise)]
    for a,b in planes:
        den=vec[:,2]-a*vec[:,0];t=(a*origin[0]+b-origin[2])/np.where(abs(den)>1e-12,den,np.nan)
        p=origin+t[:,None]*vec
        ok=(t>=0)&(abs(p[:,2]-sc.height(p[:,0],p[:,1]))<1e-7)
        candidates.append(np.where(ok,t,np.inf))
    if s.kind.startswith('step') or s.kind=='trench':
        for x in ([s.entry,s.entry+s.value] if s.kind=='trench' else [s.entry]):
            t=(x-origin[0])/vec[:,0];p=origin+t[:,None]*vec
            z0,z1=(-s.depth,0.) if s.kind=='trench' else (0.,s.value)
            ok=(t>=0)&(p[:,2]>=z0)&(p[:,2]<=z1)&(abs(p[:,1])<(.65 if s.kind=='trench' else 1.5))
            candidates.append(np.where(ok,t,np.inf))
    if s.kind=='trench':
        for y in [-.65,.65]:
            t=(y-origin[1])/vec[:,1];p=origin+t[:,None]*vec
            ok=(t>=0)&(p[:,2]>=-s.depth)&(p[:,2]<=0)&(p[:,0]>=s.entry)&(p[:,0]<=s.entry+s.value)
            candidates.append(np.where(ok,t,np.inf))
    return np.min(candidates,axis=0)


def run_trial(spec,out,seed=3,maps=True):
    out.mkdir(parents=True,exist_ok=False);sc=TerrainScene(spec)
    bot=TerrainRobot(sc,x=spec.start_x,y=spec.start_y,yaw=math.radians(spec.yaw_deg),out=out,seed=seed)
    rec=MapRecorder(sc,out,seed=seed) if maps else None
    reached=None;entry=None;stop=None;left_slope_band=None
    for ms in range(round(spec.seconds*1000)):
        t=ms/1000
        beyond=bot.y>=sc.goal_y if spec.lateral_test else bot.x>=sc.goal_x
        if beyond and reached is None:reached=t
        if bot.x>=spec.entry-.04 and entry is None:entry=t
        if spec.lateral_test and t>=2 and left_slope_band is None and not spec.entry+.05<=bot.x<=spec.entry+spec.run-.05:left_slope_band=t
        # Fixed body command; there is no feedback tracker or artificial yaw correction.
        v=spec.speed if t>=2. and reached is None else 0.
        cmd=(round(v*1000),0,0)
        if ms%100==0:bot.submit(cmd,ms//100)
        bot.step(cmd,.001)
        if rec is not None and (ms+1)%100==0:rec.capture(bot,(ms+1)/1000)
        if reached is not None and t>=reached+2.:stop=t+.001;break
    report=bot.report();report.update(scene=sc.metadata(),seed=seed,command='2s zero; fixed vx until exit then2s zero; all failure dynamics continue',
        branch='direct capability; ideal-pose map diagnostic only',reached_exit_s=reached,entry_s=entry,
        initial_condition='terrain-aligned pose once before first integration; no root constraints during run' if spec.align_start else 'source nominal init on local initial plateau',
        exit_plane='y=.6m' if spec.lateral_test else 'x=goal_x',
        left_slope_band_s=left_slope_band,
        final_xyz=[bot.x,bot.y,bot.physical_z],
        outcome='physical_failure' if bot.failed else 'nonwheel_contact' if bot.collisions else 'left_slope_band' if left_slope_band is not None else 'passed' if reached is not None else 'timeout')
    dump(out/'result.json',report);bot.close()
    if rec:rec.close()
    print(json.dumps(dict(trial=out.name,outcome=report['outcome'],entry_s=entry,exit_s=reached,final=report['final_xyz'],failure=bot.first_failure_s,wall_s=report['wall_s'])),flush=True)
    return report


def matrix():
    specs=[('flat',TerrainSpec())]
    specs += [(f'up{n}',TerrainSpec('ramp_up',n)) for n in [4,8,12,16,20]]
    specs += [('down8',TerrainSpec('ramp_down',8)),('pair8',TerrainSpec('ramp_pair',8,seconds=26.))]
    specs += [(f'step{n}',TerrainSpec('step_up',n/1000)) for n in [5,10,15,20,21,25,30,40,50]]
    specs += [('downstep20',TerrainSpec('step_down',.020)),('downstep30',TerrainSpec('step_down',.030)),('oblique20',TerrainSpec('step_up',.020,yaw_deg=15.))]
    specs += [(f'trench{n}d50',TerrainSpec('trench',n/1000,depth=.05)) for n in [20,40,65,100]]
    specs += [('trench40d150',TerrainSpec('trench',.040,depth=.15)),('trench100d150',TerrainSpec('trench',.100,depth=.15)),('trench40yaw20',TerrainSpec('trench',.040,depth=.15,yaw_deg=20.))]
    return dict(specs)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,default=OUT);ap.add_argument('--geometry',action='store_true')
    ap.add_argument('--cases',default='flat,up8,step20,trench40d150');ap.add_argument('--seed',type=int,default=3)
    ap.add_argument('--seconds',type=float);ap.add_argument('--mechanism',default='open');ap.add_argument('--profile',default='native')
    ap.add_argument('--wheel-shape',default='training_capsule');ap.add_argument('--friction',type=float,default=.6)
    ap.add_argument('--payload',type=float,default=0.);ap.add_argument('--torque-scale',type=float,default=1.)
    ap.add_argument('--policy-noise',action='store_true');ap.add_argument('--no-maps',action='store_true');a=ap.parse_args()
    a.out.mkdir(parents=True,exist_ok=True)
    if a.geometry:geometry_contract(a.out);return
    specs=matrix();cases=list(specs) if a.cases=='all' else a.cases.split(',')
    for name in cases:
        spec=specs[name];spec.mechanism=a.mechanism;spec.profile=a.profile;spec.wheel_shape=a.wheel_shape
        spec.friction=a.friction;spec.payload_kg=a.payload;spec.wheel_torque_scale=a.torque_scale;spec.policy_noise=a.policy_noise
        if a.seconds:spec.seconds=a.seconds
        run_trial(spec,a.out/name,seed=a.seed,maps=not a.no_maps)


if __name__=='__main__':main()
