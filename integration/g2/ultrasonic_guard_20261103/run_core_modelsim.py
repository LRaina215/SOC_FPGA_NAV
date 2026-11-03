"""Use the installed single-session AE license after preserved XSim core elaboration failure."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import psutil

BIN=Path('E:/modelsim_ae/win32aloem')
LICENSE=Path('C:/Users/Li/Downloads/LR-204362_License.dat')

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--xsim-core-folder',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args()
    busy=[q.info for q in psutil.process_iter(['pid','name']) if
          (q.info['name'] or '').lower() in ('vsim.exe','vsimk.exe','vish.exe','vlog.exe','vlm.exe')]
    assert not busy,('AE single running session',busy)
    assert LICENSE.is_file()
    a.out.mkdir(parents=True,exist_ok=False)
    files=json.loads((a.xsim_core_folder/'inputs.json').read_text(encoding='utf8'))
    copied=[]
    for name in files:
        source=Path(name);dest=a.out/source.name
        assert not dest.exists(),('duplicate basename',dest)
        shutil.copyfile(source,dest);copied.append(dest.name)
    # Additional original late localparams rejected by ModelSim 2020.
    # Move the exact two statements; definitions, arithmetic and candidate hardware stay unchanged.
    core=a.out/'control4500_core_sim_declaration.v'
    text=core.read_text(encoding='utf8')
    constants=''.join(s+'\n' for s in text.splitlines() if
                      s.startswith('    localparam signed [31:0] LEFT_TARGET_MMPS=') or
                      s.startswith('    localparam signed [31:0] RIGHT_TARGET_MMPS='))
    assert constants.count(';')==2 and text.count(constants)==1
    text=text.replace(constants,'',1)
    anchor=text.index('    wire signed [31:0] model_left_mmps,model_right_mmps;')
    core.write_text(text[:anchor]+constants+text[anchor:],encoding='utf8')
    (a.out/'additional_simulation_only_declaration_move.json').write_text(json.dumps(dict(constants=constants.strip(),hardware_source_changed=False),indent=2)+'\n')
    shutil.copytree(a.xsim_core_folder/'src',a.out/'src')
    source_root=Path('F:/nav_integration_g2_20261023/K_terrain19_follow1')
    commands=['onerror {quit -code 2 -f}','onbreak {quit -code 3 -f}',
              'vlib work','vlog -sv {+incdir+'+(source_root/'nav/rtl/regs').as_posix()+'} '+
              ' '.join('{'+f+'}' for f in copied),
              'vsim work.tb_core','run -all','quit -code 0 -f']
    (a.out/'run.do').write_text('\n'.join(commands)+'\n',encoding='utf8')
    env=os.environ.copy();env['MGLS_LICENSE_FILE']=str(LICENSE);env['LM_LICENSE_FILE']=str(LICENSE)
    state=dict(phase='running',started_at=datetime.now().astimezone().isoformat(),
               input_folder=str(a.xsim_core_folder),simulation_only=True,board_programmed=False,
               license_only_passed_to_child=True)
    def save(name): (a.out/name).write_text(json.dumps(state,indent=2)+'\n')
    save('job.json')
    try:
        with (a.out/'stdout.log').open('w',encoding='utf8') as log:
            proc=subprocess.Popen([str(BIN/'vsim.exe'),'-c','-l','transcript.log','-do','run.do'],
                                  cwd=a.out,env=env,stdout=log,stderr=subprocess.STDOUT,
                                  creationflags=subprocess.CREATE_NO_WINDOW)
            state.update(pid=proc.pid,created_epoch=psutil.Process(proc.pid).create_time());save('job.json')
            code=proc.wait()
        text=(a.out/'transcript.log').read_text(encoding='utf8',errors='replace')
        assert code==0,('ModelSim',code)
        marker=next((s.strip() for s in text.splitlines() if 'TEST PASSED CORE' in s),'')
        assert marker and '** Fatal:' not in text and '** Error:' not in text,'Read actual transcript'
        state.update(phase='complete',passed=True,exit_code=code,marker=marker)
    except Exception as exc: state.update(phase='failed_preserved',passed=False,error=str(exc))
    state['ended_at']=datetime.now().astimezone().isoformat();save('job.json');save('result.json')
    print(json.dumps(state,indent=2));return 0 if state['passed'] else 1

if __name__=='__main__':sys.exit(main())
