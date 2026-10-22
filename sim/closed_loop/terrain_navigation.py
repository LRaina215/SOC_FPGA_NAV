"""Fixed4500 + frozen C terrain trials using common run_cl, with passive logs."""
import argparse
import ctypes
import json
import math
from pathlib import Path
import sys
import numpy as np
import run_cl
import mujoco_robot
from terrain_scene import TerrainScene
from terrain_physics import TerrainRobot
from terrain_study import MapRecorder,matrix,dump


class Observer:
    allow_legacy_bootstrap=True
    failure_tail_s=3.

    def setup(self,bot,fpga,fw,args):
        self.bot=bot;self.fw=fw;self.out=Path(args.out);self.rec=MapRecorder(bot.s,self.out)
        self.raws=[];self.events=[];self.transforms=[];self.commits=[]
        self.anchor_events=[];self.stop_events=[]

    def raw(self,d,R,p,t):
        self.raws.append((t,d.astype(np.uint16),R.copy(),p.copy(),[self.bot.x,self.bot.y,self.bot.physical_z,self.bot.yaw]))

    def fused(self,fm,t,transform,commit):
        self.rec.observe(fm,t,None,transform)
        x,y=transform(0.,0.);a,b=transform(1.,0.);yaw=math.atan2(b-y,a-x)
        self.transforms.append([x,y,yaw]);self.commits.append(list(commit))

    def tick(self,bot,fw,t,cmd):
        if bot.ms%100:return
        st=(ctypes.c_int32*12)();fw.py_get_state(st)
        terrain=(ctypes.c_int32*6)()
        if hasattr(fw,'py_get_terrain_pose'):fw.py_get_terrain_pose(terrain)
        motion=(ctypes.c_int32*4)()
        if hasattr(fw,'py_get_motion'):fw.py_get_motion(motion)
        diag=(ctypes.c_int32*32)()
        if hasattr(fw,'py_get_frame_diag'):fw.py_get_frame_diag(diag)
        self.events.append([t,bot.x,bot.y,bot.physical_z,bot.yaw,*list(st),*list(terrain),*list(motion),*list(diag)])
        if hasattr(fw,'py_get_anchor_diagnostics'):
            anchor=(ctypes.c_int32*6)();fw.py_get_anchor_diagnostics(anchor);self.anchor_events.append([t,*list(anchor)])

        if hasattr(fw,'py_get_stop_diagnostics'):
            stop=(ctypes.c_int32*7)();fw.py_get_stop_diagnostics(stop);self.stop_events.append([t,*list(stop)])

    def finish(self,m):
        self.rec.frames=self.raws;self.rec.close('ordinary C navigation with estimated pose; truth alignment only for scoring')
        np.savez_compressed(self.out/'nav_events.npz',events=self.events,alignment=self.transforms,commit=self.commits)
        if self.anchor_events:np.savez_compressed(self.out/'anchor_diagnostics.npz',events=self.anchor_events)
        if self.stop_events:np.savez_compressed(self.out/'stop_diagnostics.npz',events=self.stop_events)
        dump(self.out/'terrain_model.json',self.bot.s.metadata())


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--case',required=True);ap.add_argument('--firmware',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True);ap.add_argument('--time',type=float,default=30.);ap.add_argument('--seed',type=int,default=3)
    ap.add_argument('--ideal',action='store_true');ap.add_argument('--mechanism',default='open');ap.add_argument('--profile',default='native');ap.add_argument('--wheel-shape',default='training_capsule');ap.add_argument('--tcfg-extra',default='');ap.add_argument('--support-snapshot',choices=['auto','off','verify'],default='auto');a=ap.parse_args()
    spec=matrix()[a.case];spec.mechanism=a.mechanism;spec.profile=a.profile;spec.wheel_shape=a.wheel_shape
    scene=TerrainScene(spec);run_cl.W.SCENES['terrain4500']=lambda:scene
    mujoco_robot.MujocoRobot=TerrainRobot
    args=['--scene','terrain4500','--backend','mujoco','--no-build','--firmware-dll',str(a.firmware.resolve()),
          '--start',f'0,0,{spec.yaw_deg}','--goal',f'{scene.goal_x},0','--time',str(a.time),'--seed',str(a.seed),
          '--out',str(a.out),'--no-plots','--match-timing','rtl50','--support-bootstrap-mm','350','--support-snapshot',a.support_snapshot,
          '--tcfg','step_high_only=1,local_frame=1'+(','+a.tcfg_extra if a.tcfg_extra else '')]
    if a.ideal:args+=['--ideal']
    sys.argv=[str(Path(run_cl.__file__)),*args]
    return run_cl.main(Observer())


if __name__=='__main__':raise SystemExit(main())
