"""Real child-process budget checks; no Gowin, FPGA, hashes or physics."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
from bounded_tool import run_tool


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    unrelated=subprocess.Popen([sys.executable,'-c','import time;time.sleep(60)'],creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    rows=[]
    try:
        fixtures=[
            ('normal',"print('completed',flush=True)",2,0.5,False,None),
            ('runtime',"import time;time.sleep(60)",0.3,0,True,'maximum runtime exceeded'),
            ('cpu_without_progress',"import time;print('routing',flush=True);end=time.monotonic()+60\nwhile time.monotonic()<end: pass",2,0.3,True,'no observable log/artifact progress within budget'),
            ('observable_progress',"import time\nfor i in range(6):\n print('phase',i,flush=True);time.sleep(.1)",2,0.3,False,None)]
        for name,code,limit,idle,cancelled,reason in fixtures:
            run=out/name;run.mkdir()
            result=run_tool([sys.executable,'-c',code],run,max_runtime_s=limit,max_no_progress_s=idle,poll_s=.02)
            assert result['cancelled']==cancelled,(name,result)
            if reason:assert result['reason']==reason
            else:assert result['tool_exit']==0
            assert unrelated.poll()is None,'unrelated process was affected'
            rows.append(dict(name=name,cancelled=cancelled,reason=result.get('reason'),elapsed_s=result['elapsed_s'],tool_exit=result['tool_exit']))
    finally:unrelated.terminate();unrelated.wait()
    result=dict(pass_=True,cases=rows,unrelated_child_unaffected=True,
        cpu_busy_not_treated_as_progress=True,scope='owned child termination and observable-progress budgets only; not routing correctness')
    (out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result))


if __name__=='__main__':main()
