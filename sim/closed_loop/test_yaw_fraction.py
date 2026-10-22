"""Independent elapsed-time oracle against actual old/candidate C odometry."""
import argparse
import ctypes as c
import json
from pathlib import Path
import subprocess


def main():
    ap=argparse.ArgumentParser();ap.add_argument('before',type=Path);ap.add_argument('after',type=Path)
    ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=False)
    source=a.out/'probe.c';source.write_text('''#include "pose.h"
bool hal_map_height(int32_t x,int32_t y,int32_t *h){return false;}
__declspec(dllexport) int yaw(int rate,int dt,int total){
 pose_est_t p;pose_init(&p,0,0,0);pose_odom(&p,0,0,rate,1000);
 for(int t=dt;t<=total;t+=dt)pose_odom(&p,0,0,rate,1000+t);
 return (int16_t)p.cur.yaw;
}
__declspec(dllexport) int reset_residue(void){
 pose_est_t p;pose_init(&p,0,0,0);pose_odom(&p,0,0,3,1000);
 for(int i=1;i<=7;i++)pose_odom(&p,0,0,3,1000+i*1000);
 pose_imu_att(&p,1234,0,0,true,8001);
 pose_imu_att(&p,0,0,0,false,8002);
 pose_odom(&p,0,0,0,9000);return p.cur.yaw;
}
''')
    rows=[]
    for label,fw in [('before',a.before),('after',a.after)]:
        dll=(a.out/(label+'.dll')).resolve()
        subprocess.run(['gcc','-std=c99','-O2','-shared','-o',str(dll),
            *['-I'+str(fw/d) for d in ('include','hal','nav')],str(source),str(fw/'nav/pose.c'),str(fw/'nav/fixmath.c')],check=True)
        lib=c.CDLL(str(dll));lib.yaw.argtypes=[c.c_int]*3;tests=[]
        for rate in (-600,-100,-20,-3,3,20,100,600):
            for dt in (1000,10000,20000):
                actual=lib.yaw(rate,dt,10000000)
                exact=int(rate*10000000*65536/6283185000)
                expected=(exact+32768)%65536-32768
                error=(actual-expected+32768)%65536-32768
                tests.append(dict(rate_mradps=rate,dt_us=dt,actual=actual,expected=expected,error=error))
        rows.append(dict(version=label,cases=tests,reset=lib.reset_residue()))
    assert any(r['error'] for r in rows[0]['cases'])
    assert all(r['error']==0 for r in rows[1]['cases']),rows[1]
    assert rows[1]['reset']==1234
    result=dict(versions=rows,pass_=True,scope='24 exact elapsed-time integration cases plus absolute-yaw reset; physical bias/stop unverified')
    (a.out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(dict(pass_=True,cases=24,
        before_max_error_deg=max(abs(r['error']) for r in rows[0]['cases'])*360/65536,after_max_error_deg=0)))


if __name__=='__main__':main()
