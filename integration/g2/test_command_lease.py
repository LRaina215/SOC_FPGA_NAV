"""Fault injection through actual original UART parser and the motion lease RTL."""
import argparse,json,re,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False);viv=Path('E:/2026.1/Vivado/bin')
    def run(tool,args):
        with (out/(tool+'_stdout.log')).open('w') as log:
            subprocess.run(['cmd','/c',str(viv/(tool+'.bat')),*args],cwd=out,stdout=log,stderr=subprocess.STDOUT,check=True)
    run('xvlog',[str(ROOT/'integration/control4500/original_src/uart_speed_rx.v'),str(ROOT/'integration/g2/command_lease_guard.v')])
    run('xvlog',['-sv',str(ROOT/'integration/g2/tb_command_lease.sv')])
    run('xelab',['-debug','off','-timescale','1ns/1ps','tb_command_lease','-s','snap'])
    run('xsim',['snap','-runall'])
    s=(out/'xsim.log').read_text();assert 'TEST PASSED' in s and 'Fatal:' not in s
    cases=list(dict.fromkeys(line[5:] for line in s.splitlines() if line.startswith('PASS ')))
    assert len(cases)==int(re.search(r'TEST PASSED (\d+) contracts',s).group(1))
    result=dict(pass_=True,contracts=len(cases),cases=cases,balance_disabled_on_communication_loss=False,
        actual_original_uart_parser=True,simulation_clock_scaled=True,
        test_parameters=dict(clock_hz=50000,uart_clocks_per_bit=8,partial_frame_timeout_ms=10,
            lease_timeout_ms=200,feedback_timeout_ms=50,stop_dwell_ms=10,production_default_stop_dwell_ms=500),
        scope='RTL contracts; not plant braking, calibrated stopping envelope, integration PNR or real vehicle acceptance',
        legacy_protocol_limitation='no wire sequence/session; transport replay cannot be guaranteed by local event numbering')
    (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))


if __name__=='__main__':main()
