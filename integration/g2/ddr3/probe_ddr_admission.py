"""DDR guard plus the exact frozen navigation safety endpoint, in isolation."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import subprocess
from derive_ddr_matcher import HERE
FROZEN=Path('F:/nav_integration_g2_20261023/sfp_lane2_private_snapshot_syn11_retry1/nav')


def run(out):
    assert not out.exists();out.mkdir()
    names=['nav_ddr_native.v','nav_ddr_health.v','nav_ddr_admission.v','safety_guard.v']
    for name in names:
        source=FROZEN/'rtl/plan'/name if name=='safety_guard.v' else HERE/name
        shutil.copyfile(source,out/name)
    shutil.copyfile(HERE/'tb_ddr_admission.sv',out/'tb_ddr_admission.sv')
    commands=[('xvlog',names),('xvlog',['-sv','tb_ddr_admission.sv']),
        ('xelab',['-debug','off','-timescale','1ns/1ps','tb_ddr_admission','-s','snap']),
        ('xsim',['snap','-runall'])]
    for i,(tool,args) in enumerate(commands):
        with (out/f'{i}_{tool}_stdout.log').open('w') as log:
            subprocess.run(['cmd','/c',str(Path('E:/2026.1/Vivado/bin')/(tool+'.bat')),*args],cwd=out,
                stdout=log,stderr=subprocess.STDOUT,timeout=180,check=True,creationflags=subprocess.CREATE_NO_WINDOW)
    text=(out/'3_xsim_stdout.log').read_text(errors='replace')
    assert 'TEST PASSED' in text and not re.search(r'FATAL|Fatal|ERROR:',text),text[-2000:]
    result=dict(tested_at=datetime.now().astimezone().isoformat(),passed=True,
        metrics={k:int(v) for k,v in re.findall(r'(\w+)=(\d+)',re.search(r'TEST PASSED[^\r\n]*',text)[0])},
        scope='DDR admission plus exact original navigation safety_guard, not complete G2/control/actuator path',
        full_joint_stop_integration_verified=False,actual_DDR_timing_verified=False,programmed=False)
    (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True)
    run(p.parse_args().out.resolve())
