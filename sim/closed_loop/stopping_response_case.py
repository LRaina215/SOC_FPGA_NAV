"""Fixed4500 response experiment: zero request vs odometry-only stop hold.

This lab isolates the plant. It does not claim the correction is map-admissible;
navigation must check the whole response/stop corridor before using it.
"""
import argparse
import ctypes
import json
from pathlib import Path
import numpy as np
from terrain_scene import TerrainScene,TerrainSpec
from terrain_physics import TerrainRobot
from terrain_study import dump


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--mode',choices=['zero','hold'],required=True);ap.add_argument('--turn',type=int,default=0);ap.add_argument('--law',choices=['aggressive','slow'],default='aggressive');ap.add_argument('--supervisor-dll',type=Path);a=ap.parse_args()
    a.out.mkdir(parents=True,exist_ok=False);scene=TerrainScene(TerrainSpec(kind='flat',value=0.))
    bot=TerrainRobot(scene,out=a.out/'physics_backend',seed=3);feedback=0;integral=0.;position_integral=0.;filtered=0.;rows=[];seq=0;cmd=(0,0,0)
    supervisor=ctypes.CDLL(str(a.supervisor_dll.resolve())) if a.supervisor_dll else None
    if supervisor:supervisor.reset()
    for ms in range(25000):
        t=ms*.001
        if ms%10==0:
            feedback,vy,wz=bot.odom()
            if supervisor:supervisor.feed(feedback,vy,wz,ms*1000)
            filtered+=.05*(feedback-filtered)
            if ms>=8000:
                integral+=feedback*.01;position_integral=float(np.clip(position_integral+integral*.01,-200,200))
        if ms%100==0:
            if t<2:cmd=(0,0,0)
            elif t<8:cmd=(50,0,a.turn)
            elif a.mode=='zero':cmd=(0,0,0)
            elif supervisor:
                cmd=(supervisor.hold(ms*1000,0),0,supervisor.hold_wz() if hasattr(supervisor,'hold_wz') else 0)
                if hasattr(supervisor,'admit'):supervisor.admit(1)  # lab has no terrain admission
            elif a.law=='aggressive':cmd=(int(np.clip(round(-2*integral-feedback),-50,50)),0,0)
            else:cmd=(int(np.clip(round(-.5*integral-.1*filtered-.05*position_integral),-25,25)),0,0)
            bot.submit(cmd,seq);seq+=1
        bot.step(cmd,.001)
        if ms%10==9:rows.append([bot.ms*.001,*cmd,feedback,integral,bot.x,bot.y,bot.z])
    report=bot.report();bot.close();x=np.array(rows);start=np.searchsorted(x[:,0],8.);delta=x[start:,6:8]-x[start,6:8]
    result=dict(mode=a.mode,law=a.law,turn_before_stop_mrad_s=a.turn,control_changed=False,source='original4500; native training-affine plant; no terrain guard in lab',
        simulated_s=25.,request_stop_s=8.,max_reference_displacement_after_stop_mm=float(np.linalg.norm(delta,axis=1).max()*1000),
        final_reference_displacement_after_stop_mm=float(np.linalg.norm(delta[-1])*1000),
        last5s_mean_abs_odom_mm_s=float(np.mean(abs(x[-500:,4]))),last5s_max_abs_request_mm_s=float(np.max(abs(x[-500:,1]))),
        physical=report,feedback='wheel odometry100Hz, command100ms; no true position/speed in hold calculation',
        hold_law=('actual C supervisor: P.5+D.1+I.05, tau.2s, linear cap25mm/s and yaw cap75mrad/s' if supervisor else
                  'aggressive:P2+D1 cap50; slow:P.5+D.1+I.05,velocity tau.2s,cap25; zero angular request after stop'),
        real_vehicle_calibrated=False,supervisor_c=bool(supervisor),
        final_stop_status=supervisor.status() if supervisor else None)
    np.savez_compressed(a.out/'stop_feedback.npz',columns=['t','vx','vy','wz','odo_vx','odo_integral_mm','true_x','true_y','true_z'],data=x)
    dump(a.out/'result.json',result);print(json.dumps({k:v for k,v in result.items() if k!='physical'}))


if __name__=='__main__':main()
