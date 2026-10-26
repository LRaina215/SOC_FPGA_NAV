"""Classify every printed failed endpoint and stream representative TR paths.

Uses the completed light coverage result, not VO or a connectivity graph. The
class counts cover all printed negative endpoints; cell/route fractions refer
only to the worst representative in each named hierarchy, not a class average.
"""
import argparse
from collections import Counter,defaultdict
from datetime import datetime
import json
from pathlib import Path
import re


def meta(p):
    s=p.stat();return dict(path=str(p),bytes=s.st_size,mtime_ns=s.st_mtime_ns)


def category(node):
    prefixes=[
        ('u_control/model_thread/policy/snapshot_word','control_ring_prefetch_word'),
        ('u_control/model_thread/observation/g_ring.','control_ring_storage'),
        ('u_control/model_thread/observation/','control_observation_arithmetic'),
        ('u_control/model_thread/policy/core/','control_inference'),
        ('u_control/model_thread/policy/','control_policy_other'),
        ('u_control/model_thread/actuator_map/','control_action_map'),
        ('u_control/model_thread/fusion/','control_fusion'),
        ('u_control/model_thread/ik/','control_IK'),
        ('u_control/wheel_control/left_control/','control_left_PID'),
        ('u_control/wheel_control/right_control/','control_right_PID'),
        ('u_soc/u_cpu/','NAV_CPU'),
        ('u_nav/u_match/u_ex/','NAV_match_extract'),
        ('u_nav/u_match/','NAV_match_other'),
        ('u_nav/u_map/u_em/','NAV_elevation_map'),
        ('u_nav/u_map/u_tv/','NAV_traversability'),
        ('u_nav/u_map/','NAV_map_other'),
        ('u_nav/u_xt/','NAV_lidar_frame_RX'),
        ('u_nav/u_pp/','NAV_point_project'),
        ('u_nav/u_pt/','NAV_point_transform'),
        ('u_nav/u_pf/','NAV_point_filter'),
        ('u_nav/g_support.','NAV_support'),
        ('u_nav/pt_','NAV_point_top_counters'),
        ('u_nav/bus_rdata','NAV_MMIO_readback'),
        ('u_nav/','NAV_other'),('u_control/','control_other'),
        ('u_soc/','SOC_other')]
    return next((name for prefix,name in prefixes if node.startswith(prefix)),'other_top')


def parse_path(text):
    summary={}
    first=text.split('Data Arrival Path:',1)[0]
    for key,value in re.findall(r'^\s*(Slack|Data Arrival Time|Data Required Time|From|To|Launch Clk|Latch Clk)\s*:\s*(.*?)\s*$',first,re.M):summary[key]=value
    data=text.split('Data Arrival Path:',1)[-1].split('Data Required Path:',1)[0]
    rows=[]
    for line in data.splitlines():
        fields=line.split()
        if len(fields)>=7 and re.fullmatch(r'-?\d+\.\d+',fields[0]) and fields[2].startswith('t'):
            rows.append(dict(arrival_ns=float(fields[0]),delay_ns=float(fields[1]),type=fields[2],transition=fields[3],fanout=int(fields[4]),location=fields[5],node=' '.join(fields[6:])))
    cq=next((i for i,r in enumerate(rows) if r['type']=='tC2Q'),None)
    data_rows=rows[cq:] if cq is not None else []
    statistics={}
    block=text.split('Arrival Data Path Delay:',1)[-1].split('Required Clock Path Delay:',1)[0]
    for kind,delay,percent in re.findall(r'(cell|route|tC2Q):\s*(-?\d+\.\d+)\s+(-?\d+\.\d+)%',block):
        statistics[kind]=dict(delay_ns=float(delay),percent=float(percent))
    level=re.search(r'^Logic Level:\s*(\d+)',text,re.M)
    return dict(summary=summary,reported_logic_level=int(level[1]) if level else None,
                vendor_data_statistics=statistics,actual_data_rows=data_rows,
                largest_data_route_segments=sorted([r for r in data_rows if r['type']=='tNET'],key=lambda r:r['delay_ns'],reverse=True)[:5],
                data_gate_and_route_sums=dict(cell_ns=round(sum(r['delay_ns'] for r in data_rows if r['type']=='tINS'),3),
                    route_ns=round(sum(r['delay_ns'] for r in data_rows if r['type']=='tNET'),3),
                    C2Q_ns=round(sum(r['delay_ns'] for r in data_rows if r['type']=='tC2Q'),3)) if cq is not None else None)


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--build',type=Path,required=True)
    ap.add_argument('--coverage',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    resultpath=a.coverage/'result.json';rowpath=a.coverage/'printed_path_rows.json'
    cov=json.loads(resultpath.read_text());assert cov['actual_report_parse_complete']
    assert cov['path_coverage']['setup']['status']=='failed_endpoint_count_matches_vendor_summary_under_printed_key'
    allrows=json.loads(rowpath.read_text());failed=[r for t in allrows['tables'] if t['analysis']=='setup' for r in t['rows'] if r['printed_negative_slack']]
    assert len({(r['to_clock'],r['to_node']) for r in failed})==cov['global_violations']['setup']==len(failed)
    groups=defaultdict(list)
    for r in failed:groups[category(r['to_node'])].append(r)
    representatives={k:min(v,key=lambda r:(r['slack_ns'],r['number'])) for k,v in groups.items()}
    wanted={r['number'] for r in representatives.values()};texts={};current=None;buffer=[];active=False
    tr=a.build/'impl/pnr/g2_top.tr';before=meta(tr)
    with tr.open(encoding='utf8') as f:
        for line in f:
            # Ignore the indented table-of-contents headings. Only the real
            # unindented analysis section owns the subsequent Path1..N blocks.
            if line.rstrip('\r\n')=='3.3.1 Setup Analysis Report':active=True
            if not active:continue
            if line.rstrip('\r\n')=='3.3.2 Hold Analysis Report':
                if current in wanted:texts[current]=''.join(buffer)
                break
            m=re.fullmatch(r'\s*Path(\d+)\s*',line)
            if m:
                if current in wanted:texts[current]=''.join(buffer)
                current=int(m[1]);buffer=[]
            if current in wanted:buffer.append(line)
    assert wanted==set(texts),(wanted-set(texts))
    assert before==meta(tr)
    details={str(n):parse_path(t) for n,t in texts.items()}
    classes=[]
    for name,rs in groups.items():
        rep=representatives[name];path=details[str(rep['number'])]
        assert float(path['summary']['Slack'])==rep['slack_ns']
        classes.append(dict(category=name,negative_unique_endpoints=len(rs),worst_slack_ns=rep['slack_ns'],worst_path_number=rep['number'],
            worst_table_row=rep,representative_vendor_cell_route_statistics=path['vendor_data_statistics'],
            representative_largest_route_segments=path['largest_data_route_segments'],
            representative_reported_logic_level=path['reported_logic_level']))
    classes.sort(key=lambda r:r['worst_slack_ns'])
    rootcounts=dict(Counter(r['to_node'].split('/')[0] for r in failed))
    worst=details[str(min(failed,key=lambda r:r['slack_ns'])['number'])]
    result=dict(recorded_at=datetime.now().astimezone().isoformat(),complete=True,read_only=True,VO_loaded=False,EDA_started=False,
        actual_negative_setup_paths=len(failed),actual_unique_negative_setup_endpoints=len(failed),matches_global_summary=True,
        classification_method='Longest explicit printed endpoint hierarchy prefix; no inferred semantic aliases',
        counts_by_actual_top=rootcounts,classes=classes,
        zero_failed_endpoint_categories_within_this_complete_reported_set=[
            name for name in ('control_inference','control_policy_other','control_IK','NAV_traversability','control_other','NAV_other','SOC_other','other_top')
            if name not in groups],
        zero_category_scope='No printed failed To endpoint with that literal hierarchy in the complete reported3565 set; not a separate unconstrained-IO or full functional signoff',
        physical_timing_failure_is_not_control_only=True,NAV_50MHz_setup_clean=False,
        worst_path=worst,
        worst_ring_path_stage_interpretation=[
            'load_index -> policy read_index=load_index+1 (3actualLUTs)',
            'read_index -> segment bound decode (3actualLUTs)',
            'segment -> n22100_s.A has6.328ns route; same-named SYN43 primitive computes segment*28 with all regsBYPASS',
            'DSP n22100_s intrinsic3.779ns; DOUT -> first read_addressALU has6.714ns route',
            'Address ALU carry/SUM -> distributed RAM RAD -> DO',
            'Four RAMbank-output-selection LUTs -> six current clip/output-selection LUTs -> snapshot_word'],
        implications=['H2 health/freeze decoupling alone does not remove this load_index/segment path',
            'Timer/fusion-only changes cannot establish NAV50 closure: actual covered negative endpoints exist in NAV and CPU',
            'Large two-way DSP travel is evidenced for this path; it does not uniquely explain all3565 endpoint failures or route congestion',
            'No RTL candidate is generated before both layouts finish; use actual53/54 paths to choose one tested boundary/address hypothesis'],
        fraction_scope='Cell/route fractions are exact vendor statistics for each class worst representative, not a class-wide mean',
        input_bindings=[meta(resultpath),meta(rowpath),before],
        remaining='Physical source/VO preservation, all clock/26 pins and memory/INIT audits are separate; zero hold and printed positive recovery/MPW do not establish reset/CDC/IO or deployment acceptance',
        deployment_accepted=False)
    a.out.mkdir(parents=True,exist_ok=False)
    (a.out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    (a.out/'representative_path_details.json').write_text(json.dumps(details,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    (a.out/'all_failed_endpoint_classes.json').write_text(json.dumps([dict(**r,category=category(r['to_node'])) for r in failed],ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    for n,t in texts.items():(a.out/f'worst_class_path{n}.txt').write_text(t,encoding='utf8')
    print(json.dumps(dict(complete=True,failed=len(failed),top_counts=rootcounts,categories=len(classes),all_negative_endpoint_coverage=True)))


if __name__=='__main__':main()
