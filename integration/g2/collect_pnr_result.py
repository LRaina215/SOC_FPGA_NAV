"""Collect actual completed place/route resources and clocks, not SYN estimates."""
import argparse,json,re
import xml.etree.ElementTree as ET
from pathlib import Path


def diagnostic_policy(result):
    """A diagnostic report can contain useful metrics but cannot release G2."""
    result['diagnostic_only']=bool((result.get('manifest') or {}).get('diagnostic_only',False))
    if result['diagnostic_only']:
        result['g2_resource_core_gate_pass']=False
        result['scope']='diagnostic implementation comparison only; never G2 or deployment acceptance'
    return result


def has_real_sfp(manifest):
    while manifest:
        if manifest.get('real_sfp'):return True
        manifest=manifest.get('source_manifest')
    return False


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
    r['resources']={key:r[key] for key in ('logic','register','bsram','dsp','io_port') if key in r}
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
    sdc=(build/'g2.sdc').read_text(encoding='utf-8')
    exception_commands=[x.strip() for x in sdc.splitlines() if re.match(r'^\s*set_(false_path|clock_groups|multicycle_path|max_delay|min_delay)\b',x)]
    reference_file=build/'cdc_reference_audit.json';reviewed=[];cdc_pass=True
    if reference_file.exists():
        from audit_cdc_first_stages import audit,sdc_fragment
        reference=json.loads(reference_file.read_text(encoding='utf-8'))
        reviewed=[x for x in sdc_fragment(reference).splitlines() if x.startswith('set_')]
        physical_netlist=p/(top+'.vo')
        physical=audit(physical_netlist) if physical_netlist.exists() else dict(passed=False,paths=[],errors=['physical netlist unavailable'])
        def endpoints(data):return {(x['source_register'],x['first_register'],x['second_register'],x['source_clock'],x['target_clock']) for x in data['paths']}
        endpoints_match=physical['passed'] and endpoints(physical)==endpoints(reference)
        active_commands=re.findall(r'^\s*TC_\S+\s+Actived\s+(.+?)\s*$',t,re.M)
        normalize=lambda x:' '.join(x.split())
        active={normalize(x) for x in active_commands}
        missing=[x for x in reviewed if normalize(x) not in active]
        cdc_pass=bool(endpoints_match and not missing)
        r['cdc_first_stage_candidate']=dict(physical_audit=physical,endpoints_match_reference=endpoints_match,
            required_active_constraints=len(reviewed),missing_active_constraints=missing,passed=cdc_pass,
            scope='only audited first-stage hold exceptions and explicit max delays; no full CDC/IO/MTBF signoff')
    sfp_file=build/'sfp_cdc_reference.json'
    if sfp_file.exists():
        from audit_sfp_cdc import audit as audit_sfp,sdc_fragment as sfp_fragment
        reference=json.loads(sfp_file.read_text(encoding='utf-8'))
        commands=[x for x in sfp_fragment(reference).splitlines() if x.startswith('set_')]
        physical_netlist=p/(top+'.vo')
        physical=audit_sfp(physical_netlist) if physical_netlist.exists() else dict(passed=False,paths=[],errors=['physical netlist unavailable'])
        def sfp_endpoints(data):return {(x['source_register'],x['first_register'],x['second_register'],x['source_clock'],x['target_clock']) for x in data['paths']}
        active={' '.join(x.split()) for x in re.findall(r'^\s*TC_\S+\s+Actived\s+(.+?)\s*$',t,re.M)}
        missing=[x for x in commands if ' '.join(x.split()) not in active]
        matches=physical['passed'] and sfp_endpoints(physical)==sfp_endpoints(reference)
        r['sfp_host_cdc_candidate']=dict(physical_audit=physical,endpoints_match_reference=matches,
            required_active_constraints=len(commands),missing_active_constraints=missing,passed=bool(matches and not missing))
        reviewed+=commands
        cdc_pass=cdc_pass and r['sfp_host_cdc_candidate']['passed']
    r['unreviewed_timing_exceptions']=[x for x in exception_commands if ' '.join(x.split()) not in {' '.join(y.split()) for y in reviewed}]
    r['g2_resource_core_gate_pass']=r['routing_passed'] and r['bsram_work_gate_336'] and r['core_50mhz_timing_pass'] and r['receive_100mhz_report_pass'] and r['reported_reset_pulse_checks_pass'] and r['setup_violations']==0 and r['hold_violations']==0 and cdc_pass and not r['unreviewed_timing_exceptions']
    r['scope']='actual reported resource/core timing; physical device revision, IO/CDC signoff and G3/vehicle acceptance remain separate'
    if has_real_sfp(r['manifest']):
        r['sfp_125mhz_report_pass']=all(any(c['name']==name and c['constraint_mhz']==125 and c['actual_fmax_mhz']>=125 for c in clocks) and domain_clean(name) for name in ('sfp_rx_clk','sfp_tx_clk'))
        # The 51 new host FIFO/toggle paths do not cover vendor PCS and reset/IO.
        # Keep acceptance closed until those remaining crossings are reviewed.
        r['sfp_complete_cdc_acceptance']=False
        r['g2_resource_core_gate_pass']=False
        r['scope']='complete SFP candidate: 26 lidar and 51 host paths checked separately; vendor PCS/status/reset/IO acceptance pending'
    plan_file=build/'clock_resource_plan.json'
    if plan_file.exists():
        from audit_global_muxes import audit as global_audit
        plan=json.loads(plan_file.read_text(encoding='utf-8'))
        actual=global_audit(build)
        assigned={x['net']:x['resource'] for x in actual['global_nets']}
        checks=[]
        for x in plan['actions']:
            expected='PRIMARY' if x['action']=='promote' else x.get('expected_resource','not listed as global')
            actual_resource=assigned.get(x['net'],'not listed as global')
            checks.append(dict(net=x['net'],expected=expected,actual=actual_resource,passed=actual_resource==expected))
        for name in plan.get('real_primary_clocks',[]):
            checks.append(dict(net=name,expected='PRIMARY',actual=assigned.get(name),passed=assigned.get(name)=='PRIMARY'))
        r['clock_resource_plan_physical']=dict(passed=all(x['passed'] for x in checks),checks=checks,
            scope='reported global assignment only; individual BUFG index and route delays require separate evidence')
        r['g2_resource_core_gate_pass']=r['g2_resource_core_gate_pass'] and r['clock_resource_plan_physical']['passed']
    return diagnostic_policy(r)


def finalize_gate(build):
    """A successful vendor process can still generate a timing-failing bitstream."""
    build=Path(build)
    try:
        r=collect(build)
    except FileNotFoundError as e:
        r=dict(build=str(build.resolve()),g2_resource_core_gate_pass=False,programmed=False,
               reason='required implementation report unavailable',missing_report=str(e.filename))
    manifest_file=build/'manifest.json'
    if 'manifest' not in r and manifest_file.exists():r['manifest']=json.loads(manifest_file.read_text())
    diagnostic_policy(r)
    (build/'g2_gate.json').write_text(json.dumps(r,indent=2),encoding='utf-8',newline='\n')
    print('G2 resource/core gate: '+('DIAGNOSTIC ONLY' if r['diagnostic_only'] else 'PASS' if r['g2_resource_core_gate_pass'] else 'FAIL / UNVERIFIED'))
    return r['g2_resource_core_gate_pass']


def main():
    ap=argparse.ArgumentParser();ap.add_argument('build',type=Path);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    r=collect(a.build);a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(r,indent=2),encoding='utf-8',newline='\n')
    print(json.dumps({k:v for k,v in r.items() if k not in ('manifest','top_setup_rows')},ensure_ascii=False))


if __name__=='__main__':main()
