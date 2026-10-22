"""Build the unaccepted N2 coverage/path candidate without editing live firmware.

The reviewed patch is applied to a fresh copy of fw/. Context mismatch stops
the build. The copy is retained; later base-source changes require revalidation.
"""
import argparse
import json
from pathlib import Path
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[2]
PATCH=Path(__file__).with_name('candidates')/'n2_support_coverage.patch'


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--host-tests',action='store_true');ap.add_argument('--support-snapshot',action='store_true');ap.add_argument('--extra-patch',type=Path,action='append',default=[],help='在冻结AY补丁后按顺序应用隔离实验补丁；默认仍构建AY');a=ap.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    fw=out/'fw';shutil.copytree(ROOT/'fw',fw,ignore=shutil.ignore_patterns('build','__pycache__'))
    # Repository sources currently mix CRLF and LF. Normalize only the copy.
    for p in fw.rglob('*'):
        if p.suffix in ('.c','.h'):
            source=p.read_text(encoding='utf-8')
            with p.open('w',encoding='utf-8',newline='\n') as f:f.write(source)
    cmd=['git','apply','--unsafe-paths','--directory='+out.as_posix()]
    subprocess.run(cmd+['--check',str(PATCH)],cwd=ROOT,check=True)
    subprocess.run(cmd+[str(PATCH)],cwd=ROOT,check=True)
    for extra in a.extra_patch:
        subprocess.run(cmd+['--check',str(extra.resolve())],cwd=ROOT,check=True)
        subprocess.run(cmd+[str(extra.resolve())],cwd=ROOT,check=True)
    if a.support_snapshot:
        from support_snapshot_shim import apply
        apply(fw)
    inc=['-I'+str(fw/d) for d in ['include','hal','nav','app','test']]
    nav=list((fw/'nav').glob('*.c'));dll=out/'nav_fw.dll'
    host_extra=[fw/'test/support_snapshot_host.c'] if a.support_snapshot else []
    subprocess.run(['gcc','-std=c99','-O2','-shared','-o',str(dll)]+inc+list(map(str,nav+[fw/'app/nav_app.c',fw/'test/hal_py.c']+host_extra)),check=True)
    if a.host_tests:
        flags=['gcc','-std=c99','-O2','-Wall','-Wextra','-Werror','-Wno-unused-parameter']+inc
        with (out/'host.log').open('w',encoding='utf-8') as log:
            subprocess.run(flags+['-D_USE_MATH_DEFINES','-o',str(out/'host.exe')]+list(map(str,nav+[fw/'app/nav_app.c',fw/'test/hal_sim.c',fw/'test/test_main.c']))+['-lm'],stdout=log,stderr=subprocess.STDOUT,check=True)
            subprocess.run(flags+['-c','-o',str(out/'hal_fpga.o'),str(fw/'drv/hal_fpga.c')],stdout=log,stderr=subprocess.STDOUT,check=True)
            subprocess.run([str(out/'host.exe'),str(ROOT/'sim/build/tb_traj_eval')],stdout=log,stderr=subprocess.STDOUT,check=True)
    label='AY + '+', '.join(p.stem for p in a.extra_patch) if a.extra_patch else 'N2 AY coverage4 + separate height/coverage age + observation exit + subcell bootstrap + packed visibility'
    (out/'build.json').write_text(json.dumps(dict(candidate=label,accepted=False,
        source=str(ROOT/'fw'),patch=str(PATCH),extra_patches=[str(p.resolve()) for p in a.extra_patch],dll=str(dll),host_tests=a.host_tests,control_changed=False),ensure_ascii=False,indent=2),encoding='utf-8')
    print(dll)


if __name__=='__main__':main()
