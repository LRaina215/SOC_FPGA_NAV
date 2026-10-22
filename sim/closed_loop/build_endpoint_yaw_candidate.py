"""Freeze AS, with passive pose diagnostics on both sides; optionally fix yaw residue.

No production firmware/control edit. Both builds retain exact sources in --out/fw.
"""
import argparse
import difflib
import json
from pathlib import Path
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[2]


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--source',type=Path,default=ROOT/'fw');ap.add_argument('--fix-yaw',action='store_true')
    a=ap.parse_args();out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    f=out/'fw';shutil.copytree(a.source,f,ignore=shutil.ignore_patterns('build','__pycache__'))
    before={n:(f/'nav'/n).read_text(encoding='utf-8') for n in ('pose.c','pose.h')}
    if a.fix_yaw:
        h=before['pose.h'].replace('int64_t    x_um, y_um, z_um;',
            'int64_t    x_um, y_um, z_um;\n    int64_t    yaw_rem; /* angle numerator remainder / 6283185000 */')
        c=before['pose.c']
        old='''        int64_t dth = (int64_t)wzc * dt * 65536 / 6283185 / 1000;
        p->cur.yaw = (angle_t)(p->cur.yaw + (int32_t)dth);'''
        new='''        /* Keep sub-angle increments across feedback packets. Truncating each
         * packet introduces a rate-dependent deadband (9.59mrad/s at 10ms). */
        int64_t num = p->yaw_rem + (int64_t)wzc * dt * 65536;
        int32_t dth = (int32_t)(num / 6283185000LL);
        p->yaw_rem = num - (int64_t)dth * 6283185000LL;
        p->cur.yaw = (angle_t)(p->cur.yaw + dth);'''
        assert old in c;c=c.replace(old,new)
        c=c.replace('if (p->still_us > 300000) wzc = 0;',
            'if (p->still_us > 300000) { wzc = 0; p->yaw_rem = 0; }')
        c=c.replace('if (yaw_valid) p->cur.yaw = yaw;',
            'if (yaw_valid) { p->cur.yaw = yaw; p->yaw_rem = 0; }')
        c=c.replace('    p->cur.yaw = yaw;\n    p->x_um',
            '    p->cur.yaw = yaw;\n    p->yaw_rem = 0;\n    p->x_um')
        for n,s in [('pose.c',c),('pose.h',h)]:
            (f/'nav'/n).write_text(s,encoding='utf-8',newline='\n')
        diff=''.join(''.join(difflib.unified_diff(before[n].splitlines(True),(f/'nav'/n).read_text().splitlines(True),
            fromfile='a/fw/nav/'+n,tofile='b/fw/nav/'+n)) for n in ('pose.c','pose.h'))
        (out/'yaw_fraction.patch').write_text(diff,encoding='utf-8',newline='\n')
    hal=f/'test/hal_py.c';s=hal.read_text(encoding='utf-8')
    assert 'g_app.nt' in s
    # Same read-only state as py_get_state; do not change the scheduler.
    s+='''
EXPORT void py_get_pose_numeric(int32_t *v) {
 const pose_est_t *p=&g_app.nt.pose;
 v[0]=p->cur.x;v[1]=p->cur.y;v[2]=p->cur.yaw;
 v[3]=p->odo_vx;v[4]=p->odo_wz;v[5]=p->wz_bias_q8;
 v[6]=(int32_t)p->still_us;v[7]=p->bias_n;v[8]=p->cmd_still;
}
'''
    hal.write_text(s,encoding='utf-8')
    sources=list((f/'nav').glob('*.c'))+[f/'app/nav_app.c',hal]
    if 'uint16_t coverage;' in (f/'hal/hal.h').read_text(encoding='utf-8'):
        from support_snapshot_shim import apply
        if not (f/'test/support_snapshot_host.c').exists():apply(f)
        sources.append(f/'test/support_snapshot_host.c')
    cmd=['gcc','-std=c99','-O2','-shared','-o',str(out/'nav_fw.dll')]
    cmd+=['-I'+str(f/d) for d in ('include','hal','nav','app','test')]+list(map(str,sources))
    subprocess.run(cmd,check=True)
    (out/'build.json').write_text(json.dumps(dict(source=str(a.source.resolve()),baseline='AS' if a.source.resolve()==(ROOT/'fw').resolve() else 'explicit frozen source; see source path',
        fix_yaw=a.fix_yaw,accepted=False,control_changed=False,diagnostics='read-only both sides'),indent=2))
    print(out/'nav_fw.dll')


if __name__=='__main__':main()
