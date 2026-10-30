"""Finite RTL checks for new image/HUD storage and selected upstream PID fix."""
import argparse,json,re,shutil,subprocess,time
from pathlib import Path
from datetime import datetime
from retry_frozen_synthesis import memory_status

HERE=Path(__file__).resolve().parent
FEATURE=HERE/'video_20261030'
BASE=Path('F:/nav_integration_g2_20261023/K_timing_syn50')
VIV=Path('E:/2026.1/Vivado/bin')

def run(source,out):
    assert not out.exists();out.mkdir(parents=True)
    m=memory_status()
    assert m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3,'EDA memory admission failed before any tool'
    frozen=out/'source';frozen.mkdir()
    for rel in ('video/g2_video_framebuffer.v','video/g2_video_hud.v','control/motor520_closed_loop.v'):
        shutil.copyfile(source/rel,frozen/Path(rel).name)
    ref=(BASE/'control/motor520_closed_loop.v').read_text(encoding='utf-8-sig')
    for name in ('motor520_closed_loop','motor520_velocity_pid','motor520_pwm'):
        ref=re.sub(r'\b'+name+r'\b',name+'_ref',ref)
    (out/'pid_ref.v').write_text(ref,encoding='utf-8')
    jobs=[('framebuffer',[frozen/'g2_video_framebuffer.v']),
          ('hud',[FEATURE/'reference/lcd_sensor_hud.v',frozen/'g2_video_hud.v']),
          ('pid',[out/'pid_ref.v',frozen/'motor520_closed_loop.v'])]
    result=dict(started_at=datetime.now().astimezone().isoformat(),source=str(source),checks=[],passed=False,
        scope='Finite RTL units only; no physical timing, full camera SCCB/capture, wholeG2 or vehicle acceptance')
    (out/'job.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    for name,rtl in jobs:
        d=out/name;d.mkdir();tb=FEATURE/f'tb_{name}.sv'
        commands=[('xvlog',[str(p)for p in rtl]),('xvlog',['-sv',str(tb)]),
                  ('xelab',[f'tb_{name}','-s',name+'_snap']),('xsim',[name+'_snap','-runall'])]
        for i,(tool,args) in enumerate(commands):
            with(d/f'{i}_{tool}.log').open('w')as log:
                p=subprocess.run(['cmd','/c',str(VIV/(tool+'.bat')),*args],cwd=d,stdout=log,
                    stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW,timeout=300)
            assert p.returncode==0,(name,tool,p.returncode,str(d/f'{i}_{tool}.log'))
        log=(d/'xsim.log').read_text(errors='replace')
        match=re.search(name.upper()+r'_PASS[^\r\n]*',log)
        assert match and 'Fatal:'not in log,(name,str(d/'xsim.log'))
        result['checks'].append(dict(unit=name,summary=match[0]))
        print(match[0],flush=True)
        (out/'job.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    result.update(passed=True,finished_at=datetime.now().astimezone().isoformat())
    (out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();run(a.source,a.out)
