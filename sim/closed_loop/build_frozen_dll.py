"""Build frozen navigation with one read-only host command counter export.

The original source tree is not edited. This enables actual command submission
events in the SAFE model for both N1c and candidates, including equal commands.
"""
import argparse
from pathlib import Path
import shutil
import subprocess
import json


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--source',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--timed-match',action='store_true',help='增加仿真时延接口，导航C仍不改')
    ap.add_argument('--support-snapshot',action='store_true',help='等价宿主批量HAL桥；不修改导航/控制算法')
    a=ap.parse_args()
    a.out.mkdir(parents=True,exist_ok=False)
    f=a.out/'fw'; shutil.copytree(a.source,f,ignore=shutil.ignore_patterns('build','__pycache__'))
    hal=f/'test/hal_py.c'; s=hal.read_text(encoding='utf-8')
    if 'py_get_cmd_seq' not in s:
        assert 's_ncmd' in s and '#define EXPORT' in s
        s+='\nEXPORT uint32_t py_get_cmd_seq(void) { return s_ncmd; }\n'
    if a.timed_match and 'py_set_match_latency_us' not in s:
        assert '#define PY_MATCH_LAT_US 40000u' in s
        s=s.replace('#define PY_MATCH_LAT_US 40000u','static uint32_t PY_MATCH_LAT_US = 40000u;\n'
           'EXPORT void py_set_match_latency_us(uint32_t us) { PY_MATCH_LAT_US=us>1000000u?1000000u:us; }\n//')
    hal.write_text(s,encoding='utf-8')
    if a.support_snapshot:
        from support_snapshot_shim import apply
        apply(f)
    dll=(a.out/'nav_fw.dll').resolve()
    src=list((f/'nav').glob('*.c'))+[f/'app/nav_app.c',hal]
    if a.support_snapshot:src+=[f/'test/support_snapshot_host.c']
    cmd=['gcc','-std=c99','-O2','-shared','-o',str(dll)]
    cmd+=['-I'+str(f/d) for d in ('include','hal','nav','app','test')]+list(map(str,src))
    subprocess.run(cmd,check=True)
    (a.out/'build.json').write_text(json.dumps(dict(source=str(a.source.resolve()),
        dll=str(dll),host_shim='command counter and optional simulator timing input',
        timed_match=a.timed_match,nav_source_changed=False),indent=2))
    print(dll)


if __name__=='__main__':
    main()
