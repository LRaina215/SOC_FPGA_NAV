"""Validate the passive bus observer only; synthetic wires are not sensor data."""
import argparse,json,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--original-master',action='store_true');a=p.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    viv=Path('E:/2026.1/Vivado/bin')
    def run(tool,args):
        with (out/(tool+'_stdout.log')).open('w')as log:
            subprocess.run(['cmd','/c',str(viv/(tool+'.bat')),*args],cwd=out,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
    run('xvlog',[str(ROOT/'integration/g3/i2c_id_observer.v')])
    if a.original_master:
        run('xvlog',[str(ROOT/'control/fpga/src/bmi088.v'),str(ROOT/'control/fpga/src/oled_driver.v'),str(ROOT/'control/tools/bmi088_oled_validation/tb_bmi088.v')])
    tb='tb_observer_original_master'if a.original_master else 'tb_i2c_id_observer'
    run('xvlog',['-sv',str(ROOT/'integration/g3'/(tb+'.sv'))])
    run('xelab',['-debug','off','-timescale','1ns/1ps',tb,'-s','snap'])
    run('xsim',['snap','-runall'])
    marker='ORIGINAL_MASTER_OBSERVER_PASS'if a.original_master else 'I2C_OBSERVER_PASS'
    text=(out/'xsim.log').read_text(errors='replace');assert marker in text and 'Fatal:' not in text,text[-2500:]
    result=dict(passed=True,build=str(out),cases=['original unmodified BMI/I2C master, register ID, address fallback and model slave']if a.original_master else ['address ACK/NACK','wrong ID observed','register filtering','unknown device ignored','master NACK normal','STOP discards pointer','reset discards partial transaction'],
        scope='RTL observer contract with original master and model slave; not hardware waveform, electrical or vendor-netlist validation'if a.original_master else 'passive observer contract on synthetic bus; no original-driver or real sensor/power validation',programmed=False)
    (out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result))


if __name__=='__main__':main()
