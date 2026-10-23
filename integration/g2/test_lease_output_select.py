"""Compare every guard output on every cycle of the original fault contracts."""
import argparse,json,re,subprocess
from pathlib import Path
from prepare_lease_output_select import generate

ROOT=Path(__file__).resolve().parents[2]


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    original=(ROOT/'integration/g2/command_lease_guard.v').read_text()
    (out/'original.v').write_text(original.replace('module command_lease_guard','module original_guard'))
    candidate=out/'candidate.v';candidate.write_text(original);generate(candidate)
    tb=(ROOT/'integration/g2/tb_command_lease.sv').read_text()
    instance=re.search(r' command_lease_guard #.*?\.balance_enable\(balance\)\);',tb,re.S)[0]
    reference=instance.replace('command_lease_guard','original_guard').replace(' dut(',' gold(')
    signals=['ov','ow','armed','expired','stopped','age','accepted','rejected','balance']
    for name in signals:reference=reference.replace('('+name+')','(gold_'+name+')')
    declared=''' wire signed [23:0] gold_ov,gold_ow;
 wire gold_armed,gold_expired,gold_stopped,gold_balance;
 wire [31:0] gold_age,gold_accepted,gold_rejected;
 integer compared_cycles=0;
'''
    check=' always @(negedge clk)begin #1;compared_cycles=compared_cycles+1;\n'
    check+='  if({'+','.join(signals)+'}!=={'+','.join('gold_'+n for n in signals)+'})$fatal(1,"guard output differs cycle %0d",compared_cycles);\n end\n'
    tb=tb.replace(instance,instance+'\n'+declared+reference+'\n'+check)
    tb=tb.replace('$display("TEST PASSED %0d contracts",checks);', '$display("EQUIVALENT %0d cycles",compared_cycles);$display("TEST PASSED %0d contracts",checks);')
    (out/'tb.sv').write_text(tb)
    sources=[ROOT/'control/fpga/src/uart_speed_rx.v',out/'original.v',candidate]
    for tool,args in [('xvlog',[str(p)for p in sources]),('xvlog',['-sv',str(out/'tb.sv')]),
                      ('xelab',['-debug','off','-timescale','1ns/1ps','tb_command_lease','-s','snap']),('xsim',['snap','-R'])]:
        with(out/(tool+'_stdout.log')).open('w')as log:
            subprocess.run(['cmd','/c',f'E:/2026.1/Vivado/bin/{tool}.bat',*args],cwd=out,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=180)
    log=(out/'xsim.log').read_text();assert 'TEST PASSED 24 contracts'in log and 'Fatal:'not in log
    result=dict(passed=True,contracts=24,compared_cycles=int(re.search(r'EQUIVALENT (\d+) cycles',log)[1]),
                all_outputs_every_cycle=True,scope='digital lease-guard equivalence, not actual braking or joint timing')
    (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))


if __name__=='__main__':main()
