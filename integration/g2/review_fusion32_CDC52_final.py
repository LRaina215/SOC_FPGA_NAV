"""Read completed PNR35 reports and audit the exact 26 first-stage paths.

Never invokes Gowin or changes a build.  The second-stage limitations and CDC
protocol/reset/MTBF/IO gaps remain explicit even if all 52 commands are active.
"""
import argparse
from collections import Counter, defaultdict
from datetime import datetime
import json
from pathlib import Path
import re

from audit_cdc_first_stages import sdc_fragment
from prepare_baseline_fusion_retime import metadata, same_bytes, save

BASE=Path('F:/nav_integration_g2_20261023')
BUILD=BASE/'baseline_fusion32_CDC52_pnr35'
BASELINE=BASE/'baseline_fusion_only_pnr32'
REFERENCE=BASE/'fusion32_CDC52_preflight1/review_result.json'
OUTPUT=BASE/'fusion32_CDC52_final_review1/review_result.json'


def load(path):return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def table(text,title,following):
    section=text.rsplit(title+'\n',1)[1].split(following+'\n',1)[0]
    records=[]
    for line in section.splitlines():
        cells=line.split()
        if len(cells)==9 and cells[0].isdigit():
            records.append(dict(index=int(cells[0]),slack_ns=float(cells[1]),source_pin=cells[2],
                target_pin=cells[3],source_clock=cells[4],target_clock=cells[5],relation_ns=float(cells[6]),
                skew_ns=float(cells[7]),data_delay_ns=float(cells[8])))
    return records


def physical_identity(before,after):
    """Byte/line equality covers every instance/port/INIT, not a name heuristic."""
    changed=[];count=0
    with before.open('r',encoding='utf-8',newline='') as a,after.open('r',encoding='utf-8',newline='') as b:
        while True:
            old,new=a.readline(),b.readline()
            assert bool(old)==bool(new),'physical netlist line count changed'
            if not old:break
            count+=1
            if old!=new:
                assert count==5 and old.startswith('//Created Time: ') and new.startswith('//Created Time: '),(
                    'physical netlist changed outside creation header',count,old,new)
                changed.append(dict(line=count,before=old.rstrip(),after=new.rstrip()))
    assert len(changed)==1
    return dict(passed=True,lines=count,differences=changed,
                all_actual_primitive_connections_parameters_and_INIT_identical=True,
                placement_or_route_delay_identity_claimed=False)


def parse_exception_delays(timing):
    section=timing.rsplit('3.7.1 Setup Analysis Report\n',1)[1].split('3.7.2 Hold Analysis Report\n',1)[0]
    pattern=r'^Timing Path Constraint\[(\d+)\]: ([^\n]+)\n(.*?)(?=^Timing Path Constraint\[|\Z)'
    rows={}
    for match in re.finditer(pattern,section,re.M|re.S):
        index,command,block=match.groups()
        endpoint=re.fullmatch(r'set_max_delay -from \[get_pins \{(.+)/Q\}\] -to \[get_pins \{(.+)/D\}\] ([\d.]+)',command)
        assert endpoint,command
        source,target,budget=endpoint.groups()
        value=lambda label: re.findall(r'^'+re.escape(label)+r'\s*:\s*(.+?)\s*$',block,re.M)
        assert value('From')==[source] and value('To')==[target]
        assert len(value('Slack'))==1,'need one exact physical path for each pair'
        data=re.findall(r'Arrival Data Path Delay: \(cell: ([\d.]+) [\d.]+%,\s*route: ([\d.]+) [\d.]+%,\s*tC2Q: ([\d.]+) [\d.]+%\)',block)
        assert len(data)==1,('missing exact data-path statistics',command)
        cell,route,c2q=map(float,data[0])
        assert cell==0.0,'audited path should be direct registered-Q to D'
        source_loc=re.findall(r'^\s*[-\d.]+\s+[-\d.]+\s+tC2Q\s+\S+\s+\d+\s+(\S+)\s+'+re.escape(source+'/Q')+r'\s*$',block,re.M)
        target_loc=re.findall(r'^\s*[-\d.]+\s+[-\d.]+\s+tNET\s+\S+\s+\d+\s+(\S+)\s+'+re.escape(target+'/D')+r'\s*$',block,re.M)
        assert len(source_loc)==len(target_loc)==1
        assert (source,target) not in rows
        rows[source,target]=dict(constraint_index=int(index),command=command,budget_ns=float(budget),
            source_register=source,first_register=target,launch_clock=value('Launch Clk')[0],
            capture_clock=value('Latch Clk')[0],vendor_setup_exception_slack_ns=float(value('Slack')[0]),
            data_arrival_ns=float(value('Data Arrival Time')[0]),data_required_ns=float(value('Data Required Time')[0]),
            clock_skew_ns=float(value('Clock Skew')[0]),relationship_ns=float(value('Setup Relationship')[0]),
            route_Q_to_D_ns=route,combinational_cell_ns=cell,clock_to_Q_ns=c2q,
            data_clock_to_Q_plus_route_ns=round(cell+route+c2q,3),
            physical_source_location=source_loc[0],physical_first_location=target_loc[0],
            bounded_delay_passed=route+cell+c2q<=float(budget) and float(value('Slack')[0])>=0)
    assert len(rows)==26
    return rows


def review(build=BUILD,baseline=BASELINE,reference=REFERENCE,out=OUTPUT):
    build,baseline,reference,out=[Path(p).resolve() for p in (build,baseline,reference,out)]
    assert not out.exists()
    job=load(build/'job.json');runtime=load(build/'tool_runtime.json')
    assert job['phase']=='completed_requires_actual_route_setup_hold52_constraint_review'
    assert runtime['tool_exit']==0 and not runtime['cancelled'] and 'ended_at' in runtime
    assert runtime['max_runtime_s']==runtime['max_no_observable_progress_s']==0
    gate=load(build/'g2_gate.json');oldgate=load(baseline/'g2_gate.json')
    proof=load(reference)
    assert proof['passed'] and proof['reviewed_paths']==26 and proof['actual_registers_audited']==78
    assert Path(proof['actual_physical_netlist']).resolve()==(baseline/'impl/pnr/g2_top.vo').resolve()
    for path,record in proof['metadata'].items():
        assert metadata(Path(path))==record,('original physical proof changed',path)
    assert same_bytes(build/'g2_top.vg',baseline/'g2_top.vg') and same_bytes(build/'g2.cst',baseline/'g2.cst')
    physical=physical_identity(baseline/'impl/pnr/g2_top.vo',build/'impl/pnr/g2_top.vo')
    fragment=sdc_fragment(dict(passed=True,paths=proof['paths']))
    assert (build/'g2.sdc').read_bytes()==(baseline/'g2.sdc').read_bytes()+b'\n'+fragment.encode('utf-8')
    sdc=(build/'g2.sdc').read_text(encoding='utf-8')
    commands=[line.strip() for line in sdc.splitlines() if line.strip() and not line.lstrip().startswith('#')]
    assert len(commands)==54 and sum(c.startswith('create_clock ') for c in commands)==2
    assert sum(c.startswith('set_false_path ') and c.endswith(' -hold') for c in commands)==26
    assert sum(c.startswith('set_max_delay ') for c in commands)==26
    timing=(build/'impl/pnr/g2_top.tr').read_text(encoding='utf-8')
    active=re.findall(r'^\s*(TC_\S+)\s+(\S+)\s+(.+?)\s*$',timing.rsplit('3.8 Timing Constraints Report\n',1)[1],re.M)
    normalize=lambda s:' '.join(s.split())
    assert len(active)==54 and {normalize(x[2]) for x in active}==set(map(normalize,commands))
    assert all(x[1]=='Actived' for x in active)
    exceptions=[c for c in commands if c.startswith('set_')]
    maxpaths=parse_exception_delays(timing)
    samples={name:table(timing,start,stop) for name,start,stop in [
        ('setup','3.1.1 Setup Paths Table','3.1.2 Hold Paths Table'),
        ('hold','3.1.2 Hold Paths Table','3.1.3 Recovery Paths Table')]}
    assert all(len(rows)==25 for rows in samples.values())
    rows=[]
    for path in proof['paths']:
        source,first,second=[path[key] for key in ('source_register','first_register','second_register')]
        record=maxpaths[source,first]
        assert record['budget_ns']==path['route_max_delay_ns'] and record['bounded_delay_passed']
        assert path['same_first_second_reset_connections']
        assert path['all_vendor_direction_first_Q_loads']==[[second,'D']]
        first_ports=path['actual_source_first_second']['first_register']['primitive']['ports']
        second_ports=path['actual_source_first_second']['second_register']['primitive']['ports']
        assert first_ports['Q']==second_ports['D']
        assert first_ports['CLK']==second_ports['CLK']==[path['target_clock']]
        assert not any(second in command for command in exceptions),'second stage incorrectly excepted'
        target_domain={'clk50_d':'clk50','rgmii_rxc_d':'rgmii_rxc'}[path['target_clock']]
        checks=gate['per_clock_violations'][target_domain]
        specific={name:[sample for sample in values if sample['source_pin']==first+'/Q' and sample['target_pin']==second+'/D']
                  for name,values in samples.items()}
        domain_setup_clean=checks['setup']['violated_endpoints']==0
        individual_setup=bool(specific['setup']) and all(s['slack_ns']>=0 for s in specific['setup'])
        rows.append(dict(group=path['group'],**record,
            second_register=second,actual_source_first_second_and_INIT_inherited_by_exact_physical_netlist_identity=True,
            actual_first_Q_unique_consumer=[second,'D'],all52_exact_endpoint_constraints_active=True,
            second_stage=dict(source_pin=first+'/Q',target_pin=second+'/D',clock=target_domain,
                same_clock=True,hold_or_setup_exceptions_on_second_stage=False,
                setup_constraint_period_ns={'clk50':20.0,'rgmii_rxc':10.0}[target_domain],
                matching_public_report_rows=specific,hold_passed_by_global_zero_unexcepted_endpoints=gate['hold_violations']==0,
                setup_passed_by_clean_same_clock_domain_or_explicit_path=domain_setup_clean or individual_setup,
                setup_unknown_when_domain_has_other_violations_and_no_individual_path=not(domain_setup_clean or individual_setup),
                individual_Q_to_D_delay_and_metastability_resolution_time_available=False)))
    grouped={}
    for group in sorted({row['group'] for row in rows}):
        group_rows=[row for row in rows if row['group']==group]
        delays=[row['route_Q_to_D_ns'] for row in group_rows]
        total=[row['data_clock_to_Q_plus_route_ns'] for row in group_rows]
        grouped[group]=dict(paths=len(group_rows),budget_ns=group_rows[0]['budget_ns'],
            route_min_ns=min(delays),route_max_ns=max(delays),route_spread_ns=round(max(delays)-min(delays),3),
            data_C2Q_plus_route_min_ns=min(total),data_C2Q_plus_route_max_ns=max(total),
            minimum_reported_slack_ns=min(row['vendor_setup_exception_slack_ns'] for row in group_rows),
            second_stage_setup_clean_domain_paths=sum(row['second_stage']['setup_passed_by_clean_same_clock_domain_or_explicit_path'] for row in group_rows))
    source=BASE/'baseline_fusion_only_syn23'
    incomplete=[dict(topic='first-to-second metastability resolution/MTBF',status='not_signed_off',
          evidence='26 unique two-flop chains and equal INIT/reset/CE proven; individual stage1-Q to stage2-D delays absent from these limited reports; device tau/T0 and event rates unavailable'),
        dict(topic='second-stage setup in clk50 domain',status='not_fully_available',
          evidence='13 destination-clk50 chains remain timed, global setup3781; not listed in 25 worst setup rows, which cannot prove their individual setup slack'),
        dict(topic='Gray pointer coherency and payload',status='partially_supported_not_protocol_signoff',
          evidence='Both 12-bit groups have registered b2g(pointer+1) RTL and actual direct endpoint delays below source-period budget; no formal FIFO occupancy/reset-phase proof or dual-port memory collision/read-after-write/CDC payload acceptance is supplied here'),
        dict(topic='reset assertion/deassertion and clock absence',status='partially_supported_not_full_reset_signoff',
          evidence='RTL nav_rst_sync async-assert/sync-release; reported recovery/removal sampled worst slacks positive. Need actual clock-start/stop/reset sequencing, domain-release invariants and all independently asynchronous reset sources'),
        dict(topic='pulse width and source-synchronous/async board IO',status='internal_report_supported_board_IO_pending',
          evidence='Internal MPW worst table positive under declared clocks. RGMII external PHY timing/skew, async UART/IR/encoder capture, I2C/MDIO electrical/protocol and actuator pulse/reset/fault acceptance remain separate; missing input/output_delay does not make every IO fail'),
        dict(topic='static RX edge selection',status='hardware_sync_proven_software_protocol_pending',
          evidence='rx_neg has a two-stage synchronizer but changing it while a frame is active can alter sampling; safe configuration/quiescence protocol must be accepted'),
        dict(topic='all remaining crossings',status='not_claimed',
          evidence='These52 commands cover only four explicit RGMII/core groups; no blanket asynchronous clock group, resets or other multi-bit/level/pulse crossing is signed off by this audit')]
    result=dict(passed=True,review_completed=True,recorded_at=datetime.now().astimezone().isoformat(),
        build=str(build),baseline=str(baseline),reference=str(reference),runtime=runtime,
        scope='exact26 first-stage physical identity,52 active commands and26 reported delays; not full CDC or deployment acceptance',
        routing_passed=gate['routing_passed'],actual_BSRAM=gate['bsram']['used'],
        source_and_CST_byte_identical=True,physical_netlist_identity_except_creation_header=physical,
        all78_actual_register_kinds_connections_effective_INIT_and_reset_CE_clock_preserved=True,
        source32_actual_replica_proofs_inherited=proof['proved_actual_replication_aliases'],
        clocks=gate['clocks'],setup_violations=gate['setup_violations'],hold_violations=gate['hold_violations'],
        baseline_setup_violations=oldgate['setup_violations'],baseline_hold_violations=oldgate['hold_violations'],
        active_commands=len(active),hold_only_false_paths_active=26,max_delay_paths_active=26,
        all26_explicit_max_delay_slacks_and_physical_data_budgets_passed=True,groups=grouped,paths=rows,
        second_stage_hold_timing_passed_paths=sum(row['second_stage']['hold_passed_by_global_zero_unexcepted_endpoints'] for row in rows),
        second_stage_setup_clean_domain_or_explicit_paths=sum(row['second_stage']['setup_passed_by_clean_same_clock_domain_or_explicit_path'] for row in rows),
        second_stage_setup_individually_unverified_paths=sum(row['second_stage']['setup_unknown_when_domain_has_other_violations_and_no_individual_path'] for row in rows),
        second_stage_timing_or_same_domain_setup_not_excepted=True,reported_reset_and_pulse_checks=gate['reported_reset_pulse_checks'],
        unmet_signoff=incomplete,all_physical_paths_compared_not_retimed_or_rebuilt=True,
        input_files_modified=False,new_build_started=False,programmed=False,full_CDC_signoff=False,deployment_accepted=False,
        evidence_metadata=[metadata(p) for p in [reference,build/'job.json',build/'tool_runtime.json',
          build/'g2.sdc',build/'g2_top.vg',build/'g2.cst',build/'impl/pnr/g2_top.tr',
          build/'impl/pnr/g2_top.vo',baseline/'impl/pnr/g2_top.vo',baseline/'impl/pnr/g2_top.tr',
          source/'nav/rtl/common/nav_async_fifo.v',source/'nav/rtl/common/nav_cdc.v',source/'nav/rtl/net/rgmii_mac100.v']])
    out.parent.mkdir(parents=True,exist_ok=True);save(out,result)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--build',type=Path,default=BUILD)
    parser.add_argument('--baseline',type=Path,default=BASELINE);parser.add_argument('--reference',type=Path,default=REFERENCE)
    parser.add_argument('--out',type=Path,default=OUTPUT);args=parser.parse_args()
    import psutil
    proc=psutil.Process();proc.nice(psutil.IDLE_PRIORITY_CLASS);proc.cpu_affinity([18])
    result=review(args.build,args.baseline,args.reference,args.out)
    print(json.dumps({key:result[key] for key in ('passed','routing_passed','actual_BSRAM','setup_violations','hold_violations',
        'hold_only_false_paths_active','max_delay_paths_active','groups','second_stage_setup_individually_unverified_paths','full_CDC_signoff')},ensure_ascii=False))
