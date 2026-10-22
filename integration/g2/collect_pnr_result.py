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
    r['actual_pnr_replicate_resources']=bool(re.search(r'(?m)^-replicate(?:_resources)?(?:\s|$)',command))
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
    # The clock-domain summary excludes cross-clock paths. Keep both it and
    # the global totals: a receive-domain pass must not hide global failures.
    per_clock={}
    for name,kind,tns,count in re.findall(r'^\s*(\S+)\s+(setup|hold)\s+(-?[\d.]+)\s+(\d+)\s*$',t,re.M):
        per_clock.setdefault(name,{})[kind]=dict(tns_ns=float(tns),violated_endpoints=int(count))
    r['per_clock_violations']=per_clock
    r['violations_outside_per_clock_summary']={
        kind:(r[kind+'_violations']-sum(x.get(kind,{}).get('violated_endpoints',0) for x in per_clock.values())
              if r[kind+'_violations'] is not None else None) for kind in ('setup','hold')}
    section=t.split('3.1.1 Setup Paths Table\n')[-1].split('3.1.2 Hold Paths Table')[0]
    r['top_setup_rows']=[x.strip() for x in section.splitlines() if re.match(r'^\s*\d+\s+[-\d.]+\s+\S+',x)][:10]
    section=t.split('3.1.2 Hold Paths Table\n')[-1].split('3.1.3 Recovery Paths Table')[0]
    r['negative_reported_hold_paths']=[]
    for row in section.splitlines():
        fields=row.split()
        if len(fields)==9 and fields[0].isdigit() and float(fields[1])<0:
            r['negative_reported_hold_paths'].append(dict(slack_ns=float(fields[1]),source=fields[2],destination=fields[3],
                source_clock=fields[4],destination_clock=fields[5],cross_clock=fields[4]!=fields[5]))
    secondary={}
    for name,title,following in [
        ('recovery','3.1.3 Recovery Paths Table',r'3\.1\.4 '),
        ('removal','3.1.4 Removal Paths Table',r'3\.2 '),
        ('minimum_pulse_width','3.2 Minimum Pulse Width Table',r'3\.3 ')]:
        m=re.search(r'^'+re.escape(title)+r'\s*$',t,re.M)
        section=t[m.end():] if m else ''
        stop=re.search(r'^'+following,section,re.M)
        if stop:section=section[:stop.start()]
        slacks=[float(x) for x in re.findall(r'^\s*\d+\s+(-?[\d.]+)\s+\S+',section,re.M)]
        secondary[name]=dict(available=m is not None,reported_rows=len(slacks),
            worst_slack_ns=min(slacks) if slacks else None,
            negative_reported_rows=sum(x<0 for x in slacks) if m else None)
    r['reported_reset_pulse_checks']=secondary
    r['reported_reset_pulse_checks_pass']=all(x['available'] and x['negative_reported_rows']==0 for x in secondary.values())
    core=next((c for c in clocks if c['name']=='clk50'),None)
    receive=next((c for c in clocks if c['name']=='rgmii_rxc'),None)
    r['routing_passed']=bool(t and not errors and r['unrouted_nets'] in (None,0))
    r['bsram_work_gate_336']=r['bsram']['used']<=336
    def domain_clean(name):
        counts=per_clock.get(name,{})
        return all(counts.get(kind,{}).get('violated_endpoints')==0 for kind in ('setup','hold'))
    r['core_50mhz_timing_pass']=bool(r['routing_passed'] and core and core['constraint_mhz']==50 and core['actual_fmax_mhz']>=50
        and domain_clean('clk50'))
    r['receive_100mhz_report_pass']=bool(r['routing_passed'] and receive and receive['constraint_mhz']==100
        and receive['actual_fmax_mhz']>=100 and domain_clean('rgmii_rxc'))
    r['g2_resource_core_gate_pass']=r['routing_passed'] and r['bsram_work_gate_336'] and r['core_50mhz_timing_pass'] and r['receive_100mhz_report_pass'] and r['reported_reset_pulse_checks_pass'] and r['setup_violations']==0 and r['hold_violations']==0
    r['scope']='actual reported resource/core timing; physical device revision, IO/CDC signoff and G3/vehicle acceptance remain separate'
    return r


def finalize_gate(build):
    """A successful vendor process can still generate a timing-failing bitstream."""
    build=Path(build)
    try:
        r=collect(build)
    except FileNotFoundError as e:
        r=dict(build=str(build.resolve()),g2_resource_core_gate_pass=False,programmed=False,
               reason='required implementation report unavailable',missing_report=str(e.filename))
    (build/'g2_gate.json').write_text(json.dumps(r,indent=2),encoding='utf-8',newline='\n')
    print('G2 resource/core gate: '+('PASS' if r['g2_resource_core_gate_pass'] else 'FAIL / UNVERIFIED'))
    return r['g2_resource_core_gate_pass']


def main():
    ap=argparse.ArgumentParser();ap.add_argument('build',type=Path);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    r=collect(a.build);a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(r,indent=2),encoding='utf-8',newline='\n')
    print(json.dumps({k:v for k,v in r.items() if k not in ('manifest','top_setup_rows')},ensure_ascii=False))


if __name__=='__main__':main()
