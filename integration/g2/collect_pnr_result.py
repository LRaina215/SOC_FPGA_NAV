"""Collect actual completed place/route resources and clocks, not SYN estimates."""
import argparse,json,re
import xml.etree.ElementTree as ET
from pathlib import Path


def collect(build,top='g2_top'):
    p=build/'impl/pnr';report=p/(top+'.rpt.txt');timing=p/(top+'.tr')
    s=report.read_text(encoding='utf-8');t=timing.read_text(encoding='utf-8') if timing.exists() else ''
    logs=[x for x in (p/(top+'.log'),build/'gw_sh.log') if x.exists()]
    messages=s+'\n'+'\n'.join(x.read_text(encoding='utf-8',errors='replace') for x in logs)
    errors=sorted(set(x.strip() for x in messages.splitlines() if re.search(r'\bERROR\s*\(|Error:',x)))
    unrouted=re.findall(r'total\s+(\d+)\s+unrouted nets',messages)
    r=dict(build=str(build.resolve()),report=str(report),timing_report=str(timing) if timing.exists() else None,
        errors=errors,unrouted_nets=int(unrouted[-1]) if unrouted else None,programmed=False,
        manifest=json.loads((build/'manifest.json').read_text()) if (build/'manifest.json').exists() else None)
    command=(p/'cmd.do').read_text(encoding='utf-8')
    r['actual_pnr_global_freq_mhz']=float(re.search(r'-global_freq\s+(\S+)',command)[1])
    prj=build/'impl/gwsynthesis'/ (top+'.prj')
    if prj.exists():
        values={x.get('type'):x.get('value') for x in ET.parse(prj).getroot().findall('./OptionList/Option')}
        r['actual_synthesis_global_freq_mhz']=float(values['global_freq'])
    for name in ('Logic','Register','BSRAM','DSP','I/O Port'):
        m=re.search(r'^\s*'+re.escape(name)+r'\s*\|\s*([\d.]+)/([\d.]+)',s,re.M)
        if m:r[name.lower().replace('/','').replace(' ','_')]=dict(used=float(m[1]),capacity=float(m[2]))
    for key,tag in [('setup_violations','Setup'),('hold_violations','Hold')]:
        m=re.search(r'<Numbers of '+tag+r' Violated Endpoints>:\s*(\d+)',t)
        r[key]=int(m[1]) if m else None
    clocks=[]
    for m in re.finditer(r'^\s*\d+\s+(\S+)\s+([\d.]+)\(MHz\)\s+([\d.]+)\(MHz\)\s+(\d+)\s+(\S+)',t,re.M):
        clocks.append(dict(name=m[1],constraint_mhz=float(m[2]),actual_fmax_mhz=float(m[3]),levels=int(m[4]),entity=m[5]))
    r['clocks']=clocks
    section=t.split('3.1.1 Setup Paths Table\n')[-1].split('3.1.2 Hold Paths Table')[0]
    r['top_setup_rows']=[x.strip() for x in section.splitlines() if re.match(r'^\s*\d+\s+[-\d.]+\s+\S+',x)][:10]
    core=next((c for c in clocks if c['name']=='clk50'),None)
    r['routing_passed']=bool(t and not errors and r['unrouted_nets'] in (None,0))
    r['bsram_work_gate_336']=r['bsram']['used']<=336
    r['core_50mhz_timing_pass']=bool(r['routing_passed'] and core and core['actual_fmax_mhz']>=50
        and r['setup_violations']==0 and r['hold_violations']==0)
    r['g2_resource_core_gate_pass']=r['routing_passed'] and r['bsram_work_gate_336'] and r['core_50mhz_timing_pass']
    r['scope']='actual reported resource/core timing; physical device revision, IO/CDC signoff and G3/vehicle acceptance remain separate'
    return r


def main():
    ap=argparse.ArgumentParser();ap.add_argument('build',type=Path);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    r=collect(a.build);a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(r,indent=2),encoding='utf-8',newline='\n')
    print(json.dumps({k:v for k,v in r.items() if k not in ('manifest','top_setup_rows')},ensure_ascii=False))


if __name__=='__main__':main()
