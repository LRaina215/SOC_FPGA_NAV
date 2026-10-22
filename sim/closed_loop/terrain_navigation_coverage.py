"""Candidate C consumes host-prototype masks from estimated capture transforms."""
import ctypes
import numpy as np
import terrain_navigation as base
from support_coverage import CompactCoverage


class CoverageObserver(base.Observer):
    def setup(self,bot,fpga,fw,args):
        super().setup(bot,fpga,fw,args);self.fpga=fpga;self.pending=[];self.snapshots=[];self.key=None
        self.builtin=fpga.coverage is not None
        self.coverage=fpga.coverage if self.builtin else CompactCoverage()
        if self.builtin:return
        frame,commit,ctl=fpga.frame,fpga.commit,fpga.ctl
        def on_frame(depth):
            frame(depth);self.capture_us=self.pending.pop(0)
        def on_commit(rt,cx,cy,zmin,zmax):
            points=commit(rt,cx,cy,zmin,zmax)
            if points is not None:
                valid=fpga.pv&(points[:,2]>=zmin)&(points[:,2]<=zmax)
                radius=fpga.cfg.get('fuse_r',0)
                if radius:valid&=(points[:,0]-rt[9])**2+(points[:,1]-rt[10])**2<=radius**2
                self.coverage.update(points,valid,cx,cy,self.capture_us,bot.ms*1000);self.key=None
            return points
        def on_ctl(op,*args):
            result=ctl(op,*args)
            if op==0:self.coverage.clear();self.key=None
            return result
        fpga.frame,fpga.commit,fpga.ctl=on_frame,on_commit,on_ctl
        CALLBACK=ctypes.CFUNCTYPE(ctypes.c_int,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32))
        def callback(x,y,out):
            now=bot.ms*1000;key=(self.coverage.seq,now//200000,self.coverage.cx,self.coverage.cy)
            if key!=self.key:
                self.known,self.fresh=self.coverage.mask_windows(now);self.key=key
            ix=((int(x)*1311)>>16)-(self.coverage.cx//4-16);iy=((int(y)*1311)>>16)-(self.coverage.cy//4-16)
            if not(0<=ix<32 and 0<=iy<32):out[0]=out[1]=0;return 1
            out[0]=int(self.known[iy,ix]);out[1]=int(self.fresh[iy,ix]);return 1
        self.callback=CALLBACK(callback);fw.py_set_coverage_cb(self.callback)

    def raw(self,d,R,p,t):
        super().raw(d,R,p,t)
        if not self.builtin:self.pending.append(round(t*1e6))

    def fused(self,fm,t,transform,commit):
        super().fused(fm,t,transform,commit)
        now=self.fpga.clock_us if self.builtin else round(t*1e6)
        known,fresh=self.coverage.mask_windows(now);self.snapshots.append((t,known.copy(),fresh.copy(),self.coverage.cx,self.coverage.cy))

    def finish(self,m):
        super().finish(m);s=self.snapshots
        np.savez_compressed(self.out/'support_frames.npz',t=[a[0] for a in s],known=[a[1] for a in s],fresh=[a[2] for a in s],cx=[a[3] for a in s],cy=[a[4] for a in s])
        m['support_coverage']=dict(prototype=True,provider='run_cl builtin' if self.builtin else 'external observer',clock='SYS_TIME_US' if self.builtin else 'physical time',cell_mm=65536/5244,known_requires='two distinct200ms capture epochs',fresh_max_s=1.,remembered_static_max_s=20.,age_upper_bound=True,ground_truth_in_controller=False)


if __name__=='__main__':
    base.Observer=CoverageObserver
    raise SystemExit(base.main())
