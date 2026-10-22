"""Read-only raw-ray provenance around a frozen runner.

The raycast/capture/frame/commit hooks retain existing return values and RNG
calls. Truth is saved only in diagnostics, never passed to firmware or maps.
"""
import argparse
import ctypes
import json
from pathlib import Path
import sys
import numpy as np


def main():
    ap=argparse.ArgumentParser(add_help=False)
    ap.add_argument('--runner-dir',type=Path,required=True)
    ap.add_argument('--trace-out',type=Path,required=True)
    ap.add_argument('--window',default='80,90')
    ap.add_argument('--cells',default='20,4;20,5;17,6')
    a,rest=ap.parse_known_args();lo,hi=map(float,a.window.split(','))
    def option(name,default):
        for i,v in enumerate(rest):
            if v==name:return rest[i+1]
            if v.startswith(name+'='):return v.split('=',1)[1]
        return default
    if option('--backend','fast')!='fast' or option('--match-timing','legacy40')=='rtl50':
        ap.error('Raw provenance currently requires fast single-retained-frame timing; queued frames need a capture-ID join')
    cells=[tuple(map(int,c.split(','))) for c in a.cells.split(';')]
    a.trace_out.mkdir(parents=True,exist_ok=False)
    sys.path.insert(0,str(a.runner_dir.resolve()))
    import run_cl as rc
    load0=ctypes.CDLL;ray0=rc.sw.World.raycast;cap0=rc.sw.VirtualLidar.capture
    frame0=rc.fpga_model.FpgaModel.frame;commit0=rc.fpga_model.FpgaModel.commit
    fw=None;now=0;ray=None;capture=None;frame_capture=None
    diag=(ctypes.c_int32*32)();terrain=(ctypes.c_int32*6)()
    with (a.trace_out/'points.jsonl').open('w',encoding='utf-8') as log:
        def load(*args,**kw):
            nonlocal fw,now
            fw=load0(*args,**kw);tick0=fw.py_set_time
            def tick(t):
                nonlocal now
                now=t.value if hasattr(t,'value') else t
                return tick0(t)
            fw.py_set_time=tick;return fw
        def raycast(world,origin,dirs,*args,**kw):
            nonlocal ray
            result=ray0(world,origin,dirs,*args,**kw)
            ray=(np.array(origin),np.array(dirs),result.copy());return result
        def cap(lid,world,R,t):
            nonlocal capture
            result=cap0(lid,world,R,t)
            if lo <= (now-1000)/1e6 <= hi:
                fw.py_get_frame_diag(diag);fw.py_get_terrain_pose(terrain)
                capture=dict(time_us=now,depth=result[0].copy(),R=np.array(R),t=np.array(t),
                             origin=ray[0],dirs=ray[1],ranges=ray[2],
                             diag=np.array(diag),terrain=np.array(terrain))
            else:capture=None
            return result
        def frame(model,depth):
            nonlocal frame_capture
            frame_capture=capture
            return frame0(model,depth)
        def commit(model,rt,cx,cy,zmin,zmax):
            if model.stage!=1 and frame_capture is not None:
                cp=frame_capture;R=np.asarray(rt[:9]).reshape(3,3);t=np.asarray(rt[9:12])
                p=rc.fpga_model.fast.pc_transform(model.xyz,R,t)
                gx=rc.fpga_model.fast.world_cell(p[:,0],model.inv_res)
                gy=rc.fpga_model.fast.world_cell(p[:,1],model.inv_res)
                pv=model.pv.copy()
                if model.cfg.get('fuse_r',0)>0:
                    pv &= (p[:,0]-t[0])**2+(p[:,1]-t[1])**2 <= model.cfg['fuse_r']**2
                pv &= (p[:,2]>=max(-32768,zmin)) & (p[:,2]<=min(32767,zmax))
                fw.py_get_frame_diag(diag);fw.py_get_terrain_pose(terrain)
                for x,y in cells:
                    ids=np.flatnonzero(pv & (gx==x) & (gy==y))
                    if not len(ids):continue
                    # Keep all contributing points for these few cells; no map snapshots.
                    world_measured=model.xyz[ids]/1000 @ cp['R'].T + cp['t']
                    world_hit=cp['origin']+cp['dirs'][ids]*cp['ranges'][ids,None]
                    row=dict(t=(now-1000)/1e6,capture_us=int(cp['time_us']),frame=model.frame_id,
                        cell=[x,y],indices=ids.tolist(),depth_mm=cp['depth'].reshape(-1)[ids].tolist(),
                        range_mm=(cp['ranges'][ids]*1000).tolist(),lut=model.lut[ids].tolist(),
                        sensor_xyz_mm=model.xyz[ids].tolist(),fused_xyz_mm=p[ids].tolist(),
                        measured_world_m=world_measured.tolist(),hit_world_m=world_hit.tolist(),
                        capture_R=cp['R'].tolist(),capture_t=cp['t'].tolist(),commit_rt=list(rt),
                        capture_diag=cp['diag'].tolist(),capture_terrain=cp['terrain'].tolist(),
                        commit_diag=list(diag),commit_terrain=list(terrain))
                    log.write(json.dumps(row)+'\n')
                log.flush()
            return commit0(model,rt,cx,cy,zmin,zmax)
        ctypes.CDLL=load;rc.sw.World.raycast=raycast;rc.sw.VirtualLidar.capture=cap
        rc.fpga_model.FpgaModel.frame=frame;rc.fpga_model.FpgaModel.commit=commit
        sys.argv=[str(Path(rc.__file__)),*rest]
        try:return rc.main()
        finally:
            ctypes.CDLL=load0;rc.sw.World.raycast=ray0;rc.sw.VirtualLidar.capture=cap0
            rc.fpga_model.FpgaModel.frame=frame0;rc.fpga_model.FpgaModel.commit=commit0


if __name__=='__main__':raise SystemExit(main())
