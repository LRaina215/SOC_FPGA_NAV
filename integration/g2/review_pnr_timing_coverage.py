"""Read actual Gowin TR coverage without loading VO or starting EDA.

Summaries, printed path tables and detail paths are separate evidence.  A
max_paths request, a title, or a positive printed prefix is not full coverage.
Clock MPW is not an external asynchronous reset assertion pulse measurement.
"""
import argparse
from collections import Counter,defaultdict
from datetime import datetime
import json
from pathlib import Path
import re

from audit_cdc_first_stages import sdc_fragment
from check_setup_endpoint_coverage import scan_report
from collect_routing_diagnostic import collect as routing_diagnostics

KINDS=('setup','hold','recovery','removal')
NUMBER=r'[-+]?\d+(?:\.\d+)?'


def meta(path):
    s=path.stat();return dict(path=str(path.resolve()),bytes=s.st_size,mtime_ns=s.st_mtime_ns)


def section(text,title,next_title):
    m=re.search(r'^'+re.escape(title)+r'\s*$',text,re.M)
    if not m:return None
    tail=text[m.end():]
    stop=re.search(r'^'+re.escape(next_title)+r'\b',tail,re.M)
    return tail[:stop.start()] if stop else tail


def options(command):
    out={}
    for key in ('max_paths','max_common_paths','nworst','max_nets','max_grids'):
        m=re.search('-'+key+r'\s+(\d+)\b',command)
        out[key]=int(m[1]) if m else None
    return out


def path_tables(text):
    table=section(text,'3.1 Path Slacks Table','3.2')
    reports=[];errors=[];current=None;spans=None
    if table is None:return reports,['Path Slacks Table missing']
    for line in table.splitlines():
        c=re.match(r'\s*<Report Command>\s*:\s*(.*)',line)
        if c:
            command=c[1];mode=re.search(r'-(setup|hold|recovery|removal)\b',command)
            current=dict(command=command,analysis=mode[1] if mode else None,rows=[],**options(command))
            reports.append(current);spans=None;continue
        if re.fullmatch(r'\s*=+(?:\s+=+)+\s*',line):
            spans=[(m.start(),m.end()) for m in re.finditer('=+',line)];continue
        if current is None or not re.match(r'^\s*\d+\s+'+NUMBER+r'\s+',line):continue
        if spans is None or len(spans)!=9:
            errors.append(dict(error='unsupported path columns',line=line));continue
        fields=[line[start:stop].strip() for start,stop in spans]
        if not fields[0].isdigit() or not re.fullmatch(NUMBER,fields[1]):
            errors.append(dict(error='unparsed numeric path',line=line,fields=fields));continue
        names=('number','slack_text','from_node','to_node','from_clock','to_clock','relation_ns','clock_skew_ns','data_delay_ns')
        row=dict(zip(names,fields));row['number']=int(row['number']);row['slack_ns']=float(row['slack_text'])
        row['printed_negative_slack']=row['slack_text'].startswith('-')
        if not row['from_node'] or not row['to_node']:errors.append(dict(error='missing endpoint',line=line))
        current['rows'].append(row)
    return reports,errors


def detailed_paths(text):
    body=section(text,'3.3 Timing Report By Analysis Type','3.4')
    if body is None:return []
    rows=[];analysis=None;command=None;pending=None
    keys={'Slack':'slack_text','From':'from_node','To':'to_node','Launch Clk':'from_clock','Latch Clk':'to_clock',
          'Data Arrival Time':'data_arrival_ns','Data Required Time':'data_required_ns'}
    def finish():
        nonlocal pending
        if pending is not None:
            pending['slack_ns']=float(pending['slack_text']) if 'slack_text' in pending else None
            pending['printed_negative_slack']=pending.get('slack_text','').startswith('-')
            rows.append(pending);pending=None
    for line in body.splitlines():
        c=re.match(r'\s*Report Command\s*:\s*(.*)',line)
        if c:
            finish();command=c[1];m=re.search(r'-(setup|hold|recovery|removal)\b',command)
            analysis=m[1] if m else None
        if line.strip()=='Path Summary:':
            finish();pending=dict(analysis=analysis,command=command)
        elif pending is not None:
            if line.strip()=='Data Arrival Path:':finish();continue
            for label,name in keys.items():
                m=re.match(r'\s*'+re.escape(label)+r'\s*:\s*(.*)',line)
                if m:pending[name]=m[1].strip();break
    finish();return rows


def coverage(rows,expected):
    failures=[r for r in rows if r['printed_negative_slack']]
    unique={(r.get('to_clock',''),r.get('to_node','')) for r in failures}
    all_unique={(r.get('to_clock',''),r.get('to_node','')) for r in rows}
    if expected is None:status='global_violated_endpoint_count_not_reported'
    elif len(unique)<expected:status='partial_failed_endpoint_coverage'
    elif len(unique)==expected:status='failed_endpoint_count_matches_vendor_summary_under_printed_key'
    else:status='printed_endpoint_count_exceeds_vendor_summary_requires_grouping_review'
    return dict(listed_rows=len(rows),listed_unique_endpoints=len(all_unique),listed_negative_rows=len(failures),
                listed_unique_negative_endpoints=len(unique),worst_listed_slack_ns=min((r['slack_ns'] for r in rows if r['slack_ns'] is not None),default=None),
                summary_violated_endpoints=expected,
                unlisted_failed_endpoints_if_same_grouping=max(expected-len(unique),0) if expected is not None else None,
                status=status,endpoint_key='Printed To Clock including edge plus printed To Node',
                interpretation='Count match does not enumerate alternate paths or prove unconstrained endpoints covered')


def clock_name(printed):return printed.split(':',1)[0]


def collect(build,top='g2_top'):
    pnr=build/'impl/pnr';tr=pnr/(top+'.tr');sdc=build/'g2.sdc';reference=build/'cdc_reference_audit.json'
    runtime=build/'tool_runtime.json'
    rt=json.loads(runtime.read_text(encoding='utf8')) if runtime.exists() else None
    final=bool(rt and rt.get('ended_at') and rt.get('tool_exit') is not None)
    out=dict(recorded_at=datetime.now().astimezone().isoformat(),build=str(build.resolve()),
             timing_report_available=tr.exists(),runtime_final=final,review_completed=True,
             read_only=True,VO_loaded=False,EDA_started=False,deployment_accepted=False,
             whole_timing_signoff_accepted=False,
             runtime=rt,report_input=meta(tr) if tr.exists() else None)
    if not tr.exists():
        out.update(status='unknown_no_TR',actual_report_parse_complete=False,global_violations={k:None for k in KINDS},per_clock={},
                   path_coverage=None,effective_52_constraints=dict(verified=False,status='unknown_no_TR'),
                   clock_MPW=None,external_reset_assertion_pulse=dict(verified=False,status='not_measured_by_clock_MPW'),
                   report_commands=[],routing_evidence_presence=dict(known=False))
        return out
    before=meta(tr);text=tr.read_text(encoding='utf-8-sig',errors='replace')
    tables,errors=path_tables(text);details=detailed_paths(text)
    global_counts={}
    for k in KINDS:
        m=re.search(r'<Numbers of '+k.title()+r' Violated Endpoints>\s*:\s*(\d+)',text)
        global_counts[k]=int(m[1]) if m else None
    clock_tns={}
    body=section(text,'2.4 Total Negative Slack Summary','3.') or ''
    # Stop the summary at Timing Details even if its numbering spacing differs.
    body=body.split('3. Timing Details',1)[0]
    for name,kind,tns,count in re.findall(r'^\s*(\S+)\s+(setup|hold|recovery|removal)\s+('+NUMBER+r')\s+(\d+)\s*$',body,re.M):
        clock_tns.setdefault(name,{})[kind]=dict(TNS_ns=float(tns),violated_endpoints=int(count))
    clocks=[]
    clockbody=section(text,'2.2 Clock Summary','2.3') or ''
    for m in re.finditer(r'^\s*\d+\s+(\S+)\s+(\S+)\s+('+NUMBER+r')\s+('+NUMBER+r')MHz\s+('+NUMBER+r')\s+('+NUMBER+r')',clockbody,re.M):
        clocks.append(dict(name=m[1],type=m[2],period_ns=float(m[3]),frequency_MHz=float(m[4]),rise_ns=float(m[5]),fall_ns=float(m[6])))
    fmax={m[1]:dict(constraint_MHz=float(m[2]),actual_Fmax_MHz=float(m[3]),levels=int(m[4]),entity=m[5])
          for m in re.finditer(r'^\s*\d+\s+(\S+)\s+([\d.]+)\(MHz\)\s+([\d.]+)\(MHz\)\s+(\d+)\s+(\S+)',text,re.M)}
    per_clock={}
    for name in sorted({c['name'] for c in clocks}|set(clock_tns)):
        analyses={}
        for kind in KINDS:
            rows=[r for t in tables if t['analysis']==kind for r in t['rows']
                  if clock_name(r['from_clock'])==name and clock_name(r['to_clock'])==name]
            analyses[kind]=dict(**clock_tns.get(name,{}).get(kind,dict(TNS_ns=None,violated_endpoints=None)),
                worst_listed_same_clock_slack_ns=min((r['slack_ns'] for r in rows),default=None),listed_same_clock_rows=len(rows),
                actual_domain_WNS_available_in_summary=False)
        per_clock[name]=dict(frequency=fmax.get(name),analyses=analyses)
    reports=[];path_coverage={};detailed_coverage={}
    for table in tables:
        rows=table['rows'];reports.append({**{k:v for k,v in table.items() if k!='rows'},**coverage(rows,None)})
    for kind in KINDS:
        rows=[r for t in tables if t['analysis']==kind for r in t['rows']]
        path_coverage[kind]=coverage(rows,global_counts[kind])
        d=[r for r in details if r['analysis']==kind]
        detailed_coverage[kind]=coverage(d,None)
    existing_setup,unused=scan_report(tr)
    assert existing_setup['unique_failed_setup_endpoints_in_report_union']==path_coverage['setup']['listed_unique_negative_endpoints']
    assert existing_setup['listed_setup_path_rows']==path_coverage['setup']['listed_rows']
    mpwbody=section(text,'3.2 Minimum Pulse Width Table','3.3')
    mpw=[];spans=None
    for line in (mpwbody or '').splitlines():
        if re.fullmatch(r'\s*=+(?:\s+=+)+\s*',line):spans=[(m.start(),m.end()) for m in re.finditer('=+',line)];continue
        if spans and len(spans)==7 and re.match(r'^\s*\d+\s+'+NUMBER+r'\s+',line):
            values=[line[x:y].strip() for x,y in spans]
            mpw.append(dict(number=int(values[0]),slack_ns=float(values[1]),printed_negative_slack=values[1].startswith('-'),
                            actual_width_ns=float(values[2]),required_width_ns=float(values[3]),type=values[4],clock=values[5],object=values[6]))
    constraints=section(text,'3.8 Timing Constraints Report','4.')
    actual=[dict(type=m[1],state=m[2],command=' '.join(m[3].split())) for m in re.finditer(r'^\s*(TC_\S+)\s+(\S+)\s+(.+?)\s*$',constraints or '',re.M)]
    expected=[];reference_ok=False
    if reference.exists():
        ref=json.loads(reference.read_text(encoding='utf8'));expected=[x for x in sdc_fragment(ref).splitlines() if x.startswith('set_')]
        reference_ok=len(expected)==52
    normalized=lambda x:' '.join(x.split())
    req=[]
    for command in expected:
        matching=[r for r in actual if r['command']==normalized(command)]
        req.append(dict(command=command,actual_states=[r['state'] for r in matching],actual_types=[r['type'] for r in matching],
                        exactly_one_Actived=len(matching)==1 and matching[0]['state']=='Actived'))
    sdc_text=sdc.read_text(encoding='utf8') if sdc.exists() else ''
    exceptions=[normalized(x) for x in sdc_text.splitlines() if re.match(r'^\s*set_(false_path|max_delay|min_delay|clock_groups|multicycle_path)\b',x)]
    expected_set={normalized(x) for x in expected}
    commands=[dict(line=i,text=m[1].strip()) for i,line in enumerate(text.splitlines(),1)
              if (m:=re.match(r'\s*(?:<Report Command>|Report Command)\s*:\s*(.*)',line))]
    diagnostics=routing_diagnostics(build)
    unchanged=before==meta(tr)
    out.update(status='actual_TR_snapshot',report_file_unchanged_during_read=unchanged,
        actual_report_parse_complete=bool(final and unchanged and not errors and constraints is not None),
        table_parse_errors=errors,clock_summary=clocks,global_violations=global_counts,per_clock=per_clock,
        violated_endpoints_outside_same_clock_summary={k:(global_counts[k]-sum(d['analyses'][k]['violated_endpoints'] or 0 for d in per_clock.values()) if global_counts[k] is not None else None) for k in KINDS},
        path_reports=reports,path_coverage=path_coverage,detailed_path_coverage=detailed_coverage,
        detail_vs_table_endpoint_forms_not_blindly_unioned=True,
        printed_path_rows_file='printed_path_rows.json',existing_setup_parser_crosscheck=existing_setup,
        report_commands=commands,
        effective_52_constraints=dict(reference_available=reference.exists(),reference_schema_26_ok=reference_ok,
            required_count=len(req),Actived_count=sum(x['exactly_one_Actived'] for x in req),
            verified=bool(reference_ok and len(req)==52 and all(x['exactly_one_Actived'] for x in req)),
            statuses=req,unexpected_SDC_exception_commands=[x for x in exceptions if x not in expected_set],
            expected_missing_from_SDC=sorted(expected_set-set(exceptions)),
            clock_rows=[x for x in actual if x['type']=='TC_CLOCK'],
            scope='Printed effective state only, not a new physical26 endpoint/CDC/MTBF audit'),
        clock_MPW=dict(table_present=mpwbody is not None,commands=[x['text'] for x in commands if x['text'].startswith('report_min_pulse_width')],
            printed_rows=len(mpw),printed_negative_rows=sum(r['printed_negative_slack'] for r in mpw),
            worst_printed_slack_ns=min((r['slack_ns'] for r in mpw),default=None),rows=mpw,
            all_clock_endpoints_covered=False),
        external_reset_assertion_pulse=dict(verified=False,status='not_measured_by_clock_MPW',
            reason='High/low clock pulse rows do not measure external key/reset pulse duration or asynchronous reset assertion and release at every destination'),
        routing_evidence_presence=dict(
            congestion_title_present=bool(re.search(r'^3\.6 Route Congestions Report\s*$',text,re.M)),
            congestion_rows=len(diagnostics['congestion_grids']),congestion=diagnostics['congestion_grids'],
            high_fanout_title_present=bool(re.search(r'^3\.5 High Fanout Nets Report\s*$',text,re.M)),
            high_fanout_rows=len(diagnostics['high_fanout_nets']),high_fanout=diagnostics['high_fanout_nets'],
            commands=diagnostics['actual_report_commands'],
            failed_net_identities_or_complete_congestion_coverage_proved=False),
        actual_target_clock_requirements={'clk50_MHz':50,'rgmii_rxc_MHz':100},
        same_clock_summary_excludes_cross_clock_paths=True,
        recovery_removal_global_zero_not_inferred_from_positive_prefix=True,
        input_bindings=[meta(x) for x in (tr,sdc,reference,runtime) if x.exists()])
    out['_rows']=dict(tables=tables,detailed_paths=details)
    return out


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('build',type=Path)
    ap.add_argument('--out',type=Path,required=True);ap.add_argument('--top',default='g2_top');a=ap.parse_args()
    result=collect(a.build,a.top);rows=result.pop('_rows',None)
    a.out.mkdir(parents=True,exist_ok=False)
    if rows:(a.out/'printed_path_rows.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    (a.out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(dict(status=result['status'],global_violations=result['global_violations'],
                         constraints52_verified=result['effective_52_constraints'].get('verified'),
                         setup_coverage=result.get('path_coverage',{}).get('setup') if result.get('path_coverage') else None),ensure_ascii=False))


if __name__=='__main__':main()
