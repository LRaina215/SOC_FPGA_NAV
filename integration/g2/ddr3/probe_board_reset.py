"""Focused check of the newly added board/protocol reset contract."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import subprocess


def run(out, candidate):
    assert not out.exists()
    out.mkdir()
    here=Path(__file__).resolve().parent
    assert (candidate/'ddr3/nav_ddr_board_reset.v').read_bytes()==(here/'nav_ddr_board_reset.v').read_bytes()
    for name in ('nav_ddr_board_reset.v','tb_ddr_board_reset.sv'):
        shutil.copyfile(here/name,out/name)
    commands=[('xvlog',['-sv','nav_ddr_board_reset.v','tb_ddr_board_reset.sv']),
              ('xelab',['-debug','off','-timescale','1ns/1ps','tb_ddr_board_reset','-s','snap']),
              ('xsim',['snap','-runall'])]
    for i,(tool,args) in enumerate(commands):
        with (out/f'{i}_{tool}.log').open('w') as log:
            subprocess.run(['cmd','/c',str(Path('E:/2026.1/Vivado/bin')/(tool+'.bat')),*args],
                cwd=out,stdout=log,stderr=subprocess.STDOUT,timeout=180,check=True,
                creationflags=subprocess.CREATE_NO_WINDOW)
    log=(out/'2_xsim.log').read_text(errors='replace')
    assert 'TEST PASSED BOARD_RESET' in log and not re.search(r'FATAL|Fatal|ERROR:',log),log[-2000:]
    result=dict(tested_at=datetime.now().astimezone().isoformat(),passed=True,candidate=str(candidate),
                checks=10,startup_us=200,known_fault_survives_protocol_reset=True,
                scope='isolated board reset sequencer; not PHY calibration, full stop chain or timing signoff')
    (out/'result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--candidate',type=Path,required=True)
    a=p.parse_args();run(a.out.resolve(),a.candidate.resolve())
