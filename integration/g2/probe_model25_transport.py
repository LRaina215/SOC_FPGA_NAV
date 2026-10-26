"""Small Vivado simulation; does not occupy the licensedModelSim session."""
import argparse,json,re,subprocess,shutil
from pathlib import Path
from datetime import datetime
from prepare_baseline_fusion_retime import metadata,save
VIV=Path('E:/2026.1/Vivado/bin')
ROOT=Path(__file__).resolve().parents[2]
LIB=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')

def command(out,index,tool,args):
    with(out/f'{index}_{tool}.log').open('w')as log:
        p=subprocess.run(['cmd','/c',str(VIV/(tool+'.bat')),*args],cwd=out,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
    assert p.returncode==0,(tool,p.returncode,str(out/f'{index}_{tool}.log'))

def run(source,out):
    assert not out.exists();out.mkdir(parents=True)
    rtl=source/'model25_related_mailbox.v';tb=ROOT/'integration/g2/tb_model25_transport.sv'
    assert rtl.read_bytes()==(ROOT/'integration/g2/model25_related_mailbox.v').read_bytes()
    shutil.copyfile(rtl,out/rtl.name);shutil.copyfile(tb,out/tb.name)
    commands=[('xvlog',[str(out/rtl.name)]),('xvlog',['-sv',str(out/tb.name)]),('xelab',['tb_model25_transport','-s','transport_snap']),('xsim',['transport_snap','-runall'])]
    for index,(tool,args)in enumerate(commands):command(out,index,tool,args)
    log=(out/'xsim.log').read_text();assert 'MODEL25_TRANSPORT_PASS'in log and 'Fatal:'not in log
    match=re.search(r'MODEL25_TRANSPORT_PASS[^\r\n]+',log)[0]
    result=dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),summary=match,
       scope='Finite related50/25MHz mailbox/latest/guard test, no full4500 or wholeG2 acceptance',
       real_PLL_or_physical25MHz_test=False,binding=[metadata(rtl),metadata(out/tb.name)])
    save(out/'result.json',result)
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    print(json.dumps(run(a.source,a.out),indent=2))
