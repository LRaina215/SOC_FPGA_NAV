"""Passive candidate/source/snapshot prefix diagnostics for old field5 cases."""
import argparse
import ctypes
import json
from pathlib import Path
import sys
import numpy as np
import run_cl
from terrain_candidate_trace import instrument


class Observer:
    def setup(self,bot,fpga,fw,args):
        self.fpga=fpga;self.fw=fw;self.out=Path(args.out);self.frames=[];self.events=[]
        self.file=(self.out/'candidates.jsonl').open('w',encoding='utf-8')
        CB=ctypes.CFUNCTYPE(None,ctypes.c_uint32,ctypes.c_int,ctypes.POINTER(ctypes.c_int32),ctypes.c_int)
        def callback(us,kind,v,n):
            if self.lo<=us/1e6<=self.hi:self.file.write(json.dumps(dict(us=us,kind=kind,v=list(v[:n])))+'\n')
        self.cb=CB(callback);fw.py_set_candidate_trace_cb(self.cb)
    def raw(self,*args):pass
    def fused(self,fm,t,*args):
        if not self.lo<=t<=self.hi:return
        slot=fm.view[0];cov=fm.coverage;known,fresh=cov.mask_windows(fm.clock_us)
        self.frames.append((t,fm.cx,fm.cy,fm.em.zmin[slot].copy(),fm.em.zmax[slot].copy(),fm.em.cnt[slot].copy(),
            ((int(fm.em.fc)-fm.em.stamp[slot])&255).astype(np.uint8),fm.em.stale[slot].copy(),known.copy(),fresh.copy()))
    def tick(self,bot,fw,t,cmd):
        if round(t*1000)%100:return
        st=(ctypes.c_int32*12)();mo=(ctypes.c_int32*4)();pa=(ctypes.c_int32*2)();fw.py_get_state(st);fw.py_get_motion(mo);fw.py_get_path_diag(pa)
        self.events.append([t,*list(st),*list(mo),*list(pa),*list(cmd[:3])])
    def finish(self,m):
        self.file.close();names=['t','cx','cy','lo','hi','cnt','age','stale','known','fresh']
        np.savez_compressed(self.out/'support_snapshots.npz',**{n:np.array([r[i] for r in self.frames]) for i,n in enumerate(names)})
        np.savez_compressed(self.out/'decisions.npz',events=self.events)
        m['diagnostic_prefix_only']=True


def main():
    ap=argparse.ArgumentParser(add_help=False);ap.add_argument('--source-fw',type=Path,required=True);ap.add_argument('--trace-build',type=Path,required=True);ap.add_argument('--window',default='2,30');a,rest=ap.parse_known_args()
    Observer.lo,Observer.hi=map(float,a.window.split(','));dll=instrument(a.source_fw.resolve(),a.trace_build.resolve())
    sys.argv=[str(Path(run_cl.__file__)),*rest,'--no-build','--firmware-dll',str(dll)]
    return run_cl.main(Observer())


if __name__=='__main__':raise SystemExit(main())
