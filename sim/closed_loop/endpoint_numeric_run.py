"""Same terrain_navigation runner/scoring, adding passive 10ms C pose diagnostics."""
import ctypes
import numpy as np
import terrain_navigation as tn

Base=tn.Observer


class Observer(Base):
    def setup(self,*args):
        super().setup(*args);self.numeric=[]

    def tick(self,bot,fw,t,cmd):
        super().tick(bot,fw,t,cmd)
        if bot.ms%10:return
        v=(ctypes.c_int32*9)();fw.py_get_pose_numeric(v)
        self.numeric.append([t,bot.x,bot.y,bot.yaw,*v])

    def finish(self,m):
        super().finish(m)
        np.savez_compressed(self.out/'pose_numeric.npz',events=self.numeric,
            columns=['t','true_x','true_y','true_yaw','pose_x_mm','pose_y_mm','pose_yaw_angle',
                     'odo_vx_mmps','odo_wz_mradps','bias_q8','still_us','bias_n','cmd_still'])


if __name__=='__main__':
    tn.Observer=Observer
    raise SystemExit(tn.main())
