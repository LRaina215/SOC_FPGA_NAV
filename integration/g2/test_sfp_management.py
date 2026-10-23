"""Joint SFP management digital reset/clock-loss contract."""
import argparse,json,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    files=[ROOT/'rtl/common/nav_cdc.v',ROOT/'integration/g2/sfp_management.v',ROOT/'integration/g2/tb_sfp_management.sv']
    for name,args in [('xvlog',['-sv',*[str(f)for f in files]]),
       ('xelab',['-debug','off','-timescale','1ns/1ps','tb_sfp_management','-s','snap']),('xsim',['snap','-R'])]:
        with(out/(name+'.log')).open('w')as f:
            r=subprocess.run(['cmd','/c',f'E:/2026.1/Vivado/bin/{name}.bat',*args],cwd=out,stdout=f,stderr=subprocess.STDOUT,timeout=120)
        assert r.returncode==0,(name,out)
    s=(out/'xsim.log').read_text(errors='replace');assert 'TEST PASSED'in s and 'Fatal'not in s,s[-1500:]
    result=dict(passed=True,scope='digital MAC boundary model; no vendor PCS, analog link or actuator acceptance',
                cases=['SFP2 registers','startup hold','mode reconfiguration','link-loss counters','missing RX clock asynchronous reset','synchronous release after RX clock returns','SGMII PCS up but copper link down'])
    (out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result))


if __name__=='__main__':main()
