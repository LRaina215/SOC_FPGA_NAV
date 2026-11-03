"""Run the unmodified candidate/core sources under WSL Icarus (no license requirement)."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import shlex
import shutil
import subprocess
import sys

HERE=Path(__file__).resolve().parent

def linux_path(path):
    path=path.resolve()
    return '/mnt/'+path.drive[0].lower()+'/'+path.as_posix()[3:]

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--candidate',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();a.out.mkdir(parents=True,exist_ok=False)
    manifest=json.loads((a.candidate/'manifest.json').read_text(encoding='utf8'))
    files=[]
    for name in manifest['verilog_files']:
        src=Path(name);dest=a.out/src.name
        assert not dest.exists(),dest
        shutil.copyfile(src,dest);files.append(dest.name)
    source=Path(manifest['source']);shutil.copytree(source/'src',a.out/'src')
    shutil.copyfile(HERE/'sim/tb_core.sv',a.out/'tb_core.sv');files.append('tb_core.sv')
    command=['iverilog','-g2012','-I',linux_path(source/'nav/rtl/regs'),'-s','tb_core','-o','probe.vvp',*files]
    (a.out/'run.sh').write_text('set -eu\n'+shlex.join(command)+'\nvvp probe.vvp\n',encoding='utf8',newline='\n')
    state=dict(phase='running',started_at=datetime.now().astimezone().isoformat(),
               candidate=str(a.candidate),simulation_only=True,board_programmed=False,
               simulation_source_changes=False,command=command)
    def save(name): (a.out/name).write_text(json.dumps(state,indent=2)+'\n',encoding='utf8')
    save('job.json')
    with (a.out/'stdout.log').open('w',encoding='utf8') as log:
        proc=subprocess.Popen(['wsl.exe','-d','Ubuntu-20.04','--cd',linux_path(a.out),'--exec','bash','run.sh'],
                              stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
        state['pid']=proc.pid;save('job.json');code=proc.wait()
    text=(a.out/'stdout.log').read_text(encoding='utf8',errors='replace')
    marker=next((s.strip() for s in text.splitlines() if 'TEST PASSED CORE' in s),'')
    passed=code==0 and bool(marker) and 'FATAL:' not in text
    state.update(phase='complete' if passed else 'failed_preserved',passed=passed,exit_code=code,
                 marker=marker,ended_at=datetime.now().astimezone().isoformat())
    save('result.json');save('job.json');print(json.dumps(state,indent=2));return 0 if passed else 1

if __name__=='__main__':sys.exit(main())
