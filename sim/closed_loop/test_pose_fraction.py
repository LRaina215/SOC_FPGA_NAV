"""Deterministic odometry/matching regression, including an old-source replay."""
import argparse
import ctypes as c
import json
from pathlib import Path
import subprocess


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('before',type=Path);ap.add_argument('after',type=Path)
    ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=False)
    src=a.out/'probe.c'
    src.write_text('''#include "pose_match.h"
#include "hal.h"
bool hal_map_height(int32_t x,int32_t y,int32_t *h) {return false;}
__declspec(dllexport) int64_t probe(int vx,int yaw,int dx) {
 pose_est_t p;pose_init(&p,0,0,0);pose_set(&p,0,0,(angle_t)yaw);
 pose_odom(&p,vx,0,0,1000);
 for(int i=1;i<=500;i++) {
  pose_odom(&p,vx,0,0,1000+i*20000);
  if(i%5==0) {pm_stat_t s={.dx=dx};pose_match_commit(&p,p.cur.x,p.cur.y,&s);}
 }
 return yaw ? p.y_um : p.x_um;
}
''')
    results=[]
    for i,fw in enumerate((a.before,a.after)):
        dll=(a.out/f'probe{i}.dll').resolve()
        subprocess.run(['gcc','-std=c99','-O2','-shared','-o',str(dll),
            *['-I'+str(fw/d) for d in ('include','hal','nav')],str(src),
            *[str(fw/'nav'/n) for n in ('pose.c','pose_match.c','fixmath.c')]],check=True)
        lib=c.CDLL(str(dll));lib.probe.restype=c.c_int64
        rows=[]
        for vx in (7,-7,125,-125):
            for yaw in (0,16384):
                actual=lib.probe(vx,yaw,0);expected=vx*10000
                rows.append(dict(vx=vx,yaw=yaw,actual_um=actual,expected_um=expected,
                                 error_um=actual-expected))
        actual=lib.probe(7,0,1)
        rows.append(dict(vx=7,yaw=0,dx_per_match_mm=1,actual_um=actual,
                         expected_um=170000,error_um=actual-170000))
        results.append(rows)
    assert any(r['error_um'] for r in results[0]),'Counterexample must fail old code'
    assert all(r['error_um']==0 for r in results[1]),results[1]
    report=dict(before=results[0],after=results[1],pass_=True,
                scope='Exact 10s odometry under 100ms zero/translation matches; no sensor/world truth input')
    (a.out/'result.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(dict(pass_=True,cases=len(results[1]),before_error_um=[r['error_um'] for r in results[0]],
                         after_error_um=[r['error_um'] for r in results[1]])))


if __name__=='__main__':main()
