"""Audit frozen PNR61 evidence without EDA, hardware writes, or hashes.

Closes the checks available in existing files. Missing IO budgets, analog CDC
parameters and physical tests are recorded as open, never inferred from STA0.
"""
import argparse
from collections import Counter
from datetime import datetime
from functools import lru_cache
import gc
import json
from pathlib import Path
import re

from review_pnr50_bsram_init import extract, effective, init_vector, LIB, metadata
from review_baseline_single_factor_syn import cpu_image
from review_slf128_pair_physical_memories import distributed_review
from review_pnr53_54_attributed_IO_FF import HEAD
from synthesis_connectivity import PORT, read
from revalidate_lidar_cdc import definitions
from review_small_block_stores import parameter_reader
from audit_netlist_memories import PARAM
from review_pnr50_bsram_init import value
from audit_cdc_first_stages import sdc_fragment, bits

BASE = Path('F:/nav_integration_g2_20261023')
SOURCE = BASE/'K_timing_syn49'
PHYSICAL = BASE/'K_pnr61_s49p2r0'


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf8')


def exact_data_FF_copies(pairs):
    """Prove observed TX-shadow data copies by INIT and every state input."""
    names={f'u_nav/wd_q_{i}_s0' for i in range(32)}
    cells=read(SOURCE/'impl/gwsynthesis/g2_top.vg',keep=lambda n:n=='u_nav' or n in names)
    assert set(cells)==names
    params=parameter_reader(definitions(SOURCE/'impl/gwsynthesis/g2_top.vg'))
    gold={c['ports']['Q'][0]:dict(c,parameters=params(c)) for c in cells.values()}
    wanted={a for g,a in pairs};actual={}
    norm=lambda s:re.sub(r'\s+(?=\[)','',s.strip().lstrip('\\'))
    text=(PHYSICAL/'impl/pnr/g2_top.vo').read_text()
    for kind,name,body in re.findall(r'^(DFF\w*)\s+(\\\S+|[^\s(]+)\s*\(\s*\n(.*?)\)\s*;',text,re.M|re.S):
        ports={p:[norm(v)] for p,v in PORT.findall(body)}
        q=ports.get('Q',[None])[0]
        if q in wanted:
            assert q not in actual
            actual[q]=dict(name=norm(name),kind=kind,ports=ports,parameters={})
    assert set(actual)==wanted
    byname={c['name']:c for c in actual.values()}
    for name,key,v in PARAM.findall(text):
        name=norm(name)
        if name in byname:byname[name]['parameters'][key]=v.strip()
    macro=re.search(r'\bmodule\s+DFFRE\b(.*?)\bendmodule\b',LIB.read_text(),re.S)[1]
    defaults=dict(re.findall(r'\bparameter\s+(\w+)\s*=\s*([^;]+);',macro))
    eff=lambda c:{k:value(v.strip()) for k,v in (defaults|c['parameters']).items()}
    proof=[]
    for g,a in pairs:
        s,p=gold[g],actual[a]
        checks=dict(kind=s['kind']==p['kind']=='DFFRE',
            every_nonQ_input={k:v for k,v in s['ports'].items() if k!='Q'}=={k:v for k,v in p['ports'].items() if k!='Q'},
            all_effective_parameters_INIT=eff(s)==eff(p))
        proof.append(dict(source_net=g,replica_net=a,source=s,physical=p,checks=checks))
    return dict(passed=all(all(x['checks'].values()) for x in proof),bits=len(proof),rows=proof,
        proof='Identical initial state and all D/CLK/CE/RESET inputs imply inductive state equality; no generated-name alias assumed')


def memories(out):
    syn, phy, defaults = extract(SOURCE, PHYSICAL)
    rows, used = [], set()
    for name, s in syn.items():
        p = phy[name]
        used.add(name)
        sp, pp = effective(s, defaults), effective(p, defaults)
        keys = [k for k in ('BIT_WIDTH', 'BIT_WIDTH_0', 'BIT_WIDTH_1') if k in sp]
        width = max(sp[k] for k in keys)
        iv, capacity = init_vector(s, defaults)
        checks = dict(kind=s['kind']==p['kind'])
        if width not in (32, 36):
            checks.update(effective_parameters=sp==pp, INIT=iv==init_vector(p, defaults)[0])
            rows.append(dict(name=name, split=False, checks=checks))
            continue
        parent, leaf = name.rsplit('/', 1)
        qnames = [n for n in (parent+'/gowin_add_'+leaf,
                  parent+'/gowin_add_'+s['kind']+'_'+leaf) if n in phy]
        assert len(qnames)==1, (name, qnames)
        q = phy[qnames[0]]
        used.add(qnames[0])
        qp, half = effective(q, defaults), width//2
        checks.update(second_kind=q['kind']==s['kind'],
            widths=all(pp[k]==qp[k]==sp[k]//2 for k in keys),
            other_parameters=all(sp[k]==pp[k]==qp[k] for k in sp if k not in keys))
        addr = ('AD',) if s['kind'].startswith('pROM') else ('ADA', 'ADB')
        checks['address_high'] = all(s['ports'][k][:-5]==p['ports'][k][:-5]==q['ports'][k][:-5] for k in addr)
        checks['address_half_select'] = all(p['ports'][k][-5]=='GND' and q['ports'][k][-5]=='VCC' for k in addr)
        checks['DO_halves'] = p['ports']['DO'][-half:]==s['ports']['DO'][-half:] and q['ports']['DO'][-half:]==s['ports']['DO'][:half]
        if 'DI' in s['ports']:
            checks['DI_halves'] = p['ports']['DI'][-half:]==s['ports']['DI'][-half:] and q['ports']['DI'][-half:]==s['ports']['DI'][:half]
            if not checks['DI_halves']:
                actual=q['ports']['DI'][-half:]+p['ports']['DI'][-half:]
                pairs=[(g,a) for g,a in zip(s['ports']['DI'],actual) if g!=a]
                replica=exact_data_FF_copies(pairs)
                save(out/'TXshadow_data_FF_copy_supplement.json',replica)
                # Retain the direct-name rejection separately. This closure is
                # conditional on every actual copy passing an independent proof.
                save(out/'TXshadow_direct_name_rejection.json',dict(source=name,pairs=pairs,direct_DI_name_match=False))
                checks['DI_halves']=replica['passed']
            checks['byte_enables'] = p['ports']['ADA'][-2:]==s['ports']['ADA'][-2:] and q['ports']['ADA'][-2:]==s['ports']['ADA'][-4:-2]
        cp = [k for k in s['ports'] if k not in ('DI', 'DO', 'AD', 'ADA', 'ADB')]
        checks['clock_enable_reset_block_select'] = all(s['ports'][k]==p['ports'][k]==q['ports'][k] for k in cp)
        low, high = init_vector(p, defaults)[0], init_vector(q, defaults)[0]
        mask, wmask = (1<<half)-1, (1<<width)-1
        checks['every_logical_INIT_word'] = all(
            ((iv>>(a*width))&wmask)==(((low>>(2*a*half))&mask)|(((high>>((2*a+1)*half))&mask)<<half))
            for a in range(capacity//width))
        rows.append(dict(name=name, added=qnames[0], split=True, checks=checks))
    d = {k:v for k,v in defaults['SP'].items() if k.startswith('INIT_RAM_')}
    si, pi = cpu_image(syn, d), cpu_image(phy, d)
    binary = (SOURCE/'firmware/nav_fw.bin').read_bytes()
    words = b''.join(int(x,16).to_bytes(4,'little') for x in (SOURCE/'ram_w.hex').read_text().split())
    lanes = [bytes(int(x,16) for x in (SOURCE/f'ram_b{i}.hex').read_text().split()) for i in range(4)]
    lane_image = bytes(lanes[i%4][i//4] for i in range(131072))
    cpu = dict(physical_bitplanes=sum(n.startswith('u_soc/g_lane[') for n in phy),
        BIN_bytes=len(binary), image_bytes=len(pi),
        source_physical_equal=si==pi, BIN_zero_tail_equal=pi==binary+bytes(131072-len(binary)),
        word_hex_equal=pi==words, lane_hex_equal=pi==lane_image)
    cpu_ok = cpu['physical_bitplanes']==64 and cpu['BIN_bytes']==54164 and all(
        cpu[k] for k in ('source_physical_equal','BIN_zero_tail_equal','word_hex_equal','lane_hex_equal'))
    result = dict(source_count=len(syn),physical_count=len(phy),split_pairs=sum(x['split'] for x in rows),
        all_physical_leaves_bound=used==set(phy),CPU128KiB=cpu,CPU_checks_passed=cpu_ok,
        every_geometry_and_INIT_check_passed=all(all(x['checks'].values()) for x in rows),rows=rows,
        scope='Memory geometry/effective INIT and split address/data/control only; no complete RAM caller equivalence claim')
    result['passed'] = len(syn)==269 and len(phy)==280 and result['split_pairs']==11 and cpu_ok and result['all_physical_leaves_bound'] and result['every_geometry_and_INIT_check_passed']
    save(out/'BSRAM_CPU_INIT_review.json',result)
    del syn,phy
    gc.collect()
    dist=distributed_review(SOURCE,PHYSICAL,out)
    result['distributed_memories']={k:dist[k] for k in ('passed','source_count','physical_count','source_by_kind')}
    return result


def physical_clock_reset_inventory(out):
    cells, drivers, clocks, resets = {}, {}, [], Counter()
    active, body, heads, attrs = None, [], 0, 0
    with (PHYSICAL/'impl/pnr/g2_top.vo').open(encoding='utf8') as f:
        for line in f:
            if active is None:
                m=HEAD.match(line)
                if not m or m[2]=='module': continue
                attribute,kind,name=m.groups()
                active=(kind,name.lstrip('\\'))
                body=[line];heads+=1;attrs+=bool(attribute)
            else: body.append(line)
            if active is not None and ');' in line:
                kind,name=active
                ports={p.lstrip('\\'):v.strip() for p,v in PORT.findall(''.join(body))}
                for pin,net in ports.items():
                    if re.fullmatch(r'CLK\d*|CLKA|CLKB|WCLK|RCLK|CLOCK',pin):
                        for clock_net in bits(net):
                            if clock_net not in ('GND','VCC'):
                                clocks.append((name,pin,clock_net))
                    if kind.startswith('DFF') and pin in ('CLEAR','PRESET','RESET','SET') and net not in ('GND','VCC'):
                        resets[(kind,pin,net)]+=1
                if kind in ('IBUF','INV','BUF','BUFG'):
                    ports={p:v.lstrip('\\').strip() for p,v in ports.items()}
                    cells[name]=(kind,ports)
                    drivers.setdefault(ports['O'],[]).append(name)
                active=None;body=[]
    assert active is None
    @lru_cache(None)
    def root(net):
        ns=drivers.get(net,[])
        assert len(ns)==1,('Clock driver not unique',net,ns)
        kind,p=cells[ns[0]]
        if kind=='IBUF': return p['I'],False
        r,i=root(p['I']);return r,i^(kind=='INV')
    counts=Counter(root(net) for _,_,net in clocks)
    result=dict(passed={k[0] for k in counts}=={'clk50','rgmii_rxc'},
        primitive_heads_scanned=heads,attributed_heads_included=attrs,nonconstant_clock_pins=len(clocks),
        roots=[dict(root=k[0],inverted=k[1],pins=v) for k,v in sorted(counts.items())],
        clock_nets=[dict(net=n,root=root(n)) for n in sorted({x[2] for x in clocks})],
        largest_reset_control_loads=[dict(kind=k[0],pin=k[1],net=k[2],loads=v) for k,v in resets.most_common(12)],
        scope='Every physical primitive clock pin token, including attributed IO FF; reset counts include synchronous RESET and asynchronous CLEAR/PRESET, not electrical reset signoff')
    save(out/'all_physical_clock_reset_inventory.json',result)
    return result


def timing(out,cdc):
    tables={k:[] for k in ('setup','hold','recovery','removal','clock_pulse')}
    state=None;constraints=[];header=[];detail_kind=None;detail=None;details=[]
    names={'3.1.1 Setup Paths Table':'setup','3.1.2 Hold Paths Table':'hold',
           '3.1.3 Recovery Paths Table':'recovery','3.1.4 Removal Paths Table':'removal',
           '3.2 Minimum Pulse Width Table':'clock_pulse'}
    interesting={(p['source_register'],p['first_register']) for p in cdc['paths']}
    interesting|={(p['first_register'],p['second_register']) for p in cdc['paths']}
    def retain():
        if detail and (detail.get('source'),detail.get('destination')) in interesting:
            details.append(dict(detail))
    with (PHYSICAL/'impl/pnr/g2_top.tr').open(encoding='utf8') as f:
        for i,line in enumerate(f):
            s=line.strip()
            if i<85:header.append(line)
            if s in names:state=names[s]
            elif s=='3.3 Timing Report By Analysis Type':state=None
            elif s.startswith('3.3.'):
                retain();detail=None;detail_kind={'3.3.1':'setup','3.3.2':'hold','3.3.3':'recovery','3.3.4':'removal'}.get(s.split()[0])
            elif s.startswith(('3.4 ','3.5 ','3.6 ','3.7.','3.8 ')):
                retain();detail=None;detail_kind=None;state=None
            m=re.match(r'\s*(TC_\w+)\s+(\w+)\s+(.+)',line)
            if m:constraints.append(dict(kind=m[1],status=m[2],command=m[3].strip()))
            if state:
                x=s.split()
                if state=='clock_pulse':
                    if len(x)>=9 and x[0].isdigit():
                        try:tables[state].append(dict(number=int(x[0]),slack_ns=float(x[1]),actual_ns=float(x[2]),required_ns=float(x[3]),clock=x[-2],object=x[-1]))
                        except ValueError:pass
                elif len(x)==9 and x[0].isdigit():
                    try:tables[state].append(dict(number=int(x[0]),slack_ns=float(x[1]),source=x[2],destination=x[3],from_clock=x[4],to_clock=x[5],relation_ns=float(x[6]),skew_ns=float(x[7]),data_delay_ns=float(x[8])))
                    except ValueError:pass
            if detail_kind:
                if re.fullmatch(r'Path\d+',s):
                    retain();detail=dict(analysis=detail_kind,path=s)
                elif detail is not None:
                    for prefix,key in [('From','source'),('To','destination'),('Slack','slack_ns'),('Launch Clk','launch_clock'),('Latch Clk','latch_clock')]:
                        m=re.match(re.escape(prefix)+r'\s*:\s*(.+)',s)
                        if m:detail[key]=float(m[1]) if key=='slack_ns' else m[1].strip()
                    m=re.match(r'([-\d.]+)\s+([-\d.]+)\s+(tC2Q|tNET|tSU)\s+\S+\s+\S+\s+(\S+)\s+(\S+)',s)
                    if m and m[5].endswith(('/Q','/D')):
                        detail.setdefault('endpoint_events',[]).append(dict(type=m[3],delay_ns=float(m[2]),AT_ns=float(m[1]),location=m[4],pin=m[5]))
    retain()
    head=''.join(header)
    summary={k:dict(reported_rows=len(v),worst_reported_slack_ns=min((x['slack_ns'] for x in v),default=None),
        negative_rows=sum(x['slack_ns']<0 for x in v),unique_endpoints=len({x.get('destination',x.get('object')) for x in v}),
        by_clock=dict(Counter(x.get('to_clock',x.get('clock')) for x in v))) for k,v in tables.items()}
    sdc=(PHYSICAL/'g2.sdc').read_text()
    requests=[x.strip() for x in sdc.splitlines() if x.strip() and not x.lstrip().startswith('#')]
    expected=[x for x in sdc_fragment(cdc).splitlines() if x.startswith('set_')]
    actual=[x for x in requests if x.startswith('set_')]
    rows=[]
    for p in cdc['paths']:
        first=[x for x in tables['setup'] if x['source']==p['source_register']+'/Q' and x['destination']==p['first_register']+'/D']
        second=[x for x in tables['setup'] if x['source']==p['first_register']+'/Q' and x['destination']==p['second_register']+'/D']
        rows.append(dict(group=p['group'],source=p['source_register'],first=p['first_register'],second=p['second_register'],
            route_budget_ns=p['route_max_delay_ns'],reported_first_setup_paths=first,reported_second_setup_paths=second,
            all_path_delays_available=bool(first and second),MTBF_not_proved=True))
    result=dict(report_created=re.search(r'<Created Time>:(.+)',head)[1].strip(),
        setup_violated_endpoints=int(re.search(r'<Numbers of Setup Violated Endpoints>:(\d+)',head)[1]),
        hold_violated_endpoints=int(re.search(r'<Numbers of Hold Violated Endpoints>:(\d+)',head)[1]),
        paths_analyzed=int(re.search(r'<Numbers of Paths Analyzed>:(\d+)',head)[1]),
        endpoints_analyzed=int(re.search(r'<Numbers of Endpoints Analyzed>:(\d+)',head)[1]),
        corners=re.findall(r'<(Setup Delay Model|Hold Delay Model)>:([^\n]+)',head),
        tables=summary,constraint_status=dict(Counter(x['status'] for x in constraints)),
        constraint_kinds=dict(Counter(x['kind'] for x in constraints)),exact_CDC52_match=actual==expected,
        IO_delay_constraints=sum(bool(re.match(r'set_(input|output)_delay\b',x)) for x in requests),
        CDC_route_delay_rows=rows,CDC_detail_paths=details,
        external_IO_timing_accepted=False,asynchronous_reset_pulse_accepted=False,CDC_MTBF_accepted=False,
        missing_report_rows_are_not_pass=True,all_target_clock_setup_hold_report_passed=False)
    result['all_target_clock_setup_hold_report_passed']=result['setup_violated_endpoints']==result['hold_violated_endpoints']==0
    save(out/'timing_tables.json',tables)
    save(out/'timing_CDC_reset_IO_review.json',result)
    return result


def pins(out):
    cst=(PHYSICAL/'g2.cst').read_text()
    requested={n:p for n,p in re.findall(r'IO_LOC\s+"([^"]+)"\s+(\w+)\s*;',cst)}
    report=(PHYSICAL/'impl/pnr/g2_top.rpt.txt').read_text()
    observed={}
    for line in report.splitlines():
        x=[v.strip() for v in line.split('|')]
        if len(x)>14 and x[0] in requested and re.match(r'^\w+/\d+$',x[2]):
            observed[x[0]]=dict(pin=x[2].split('/')[0],bank=x[2].split('/')[1],direction=x[4],site=x[5],IO_type=x[7],pull=x[9])
    mismatch=[dict(signal=n,requested=p,actual=observed[n]['pin']) for n,p in requested.items() if n in observed and observed[n]['pin']!=p]
    top=(SOURCE/'g2_top.v').read_text()
    result=dict(requested_pins=len(requested),observed_pins=len(observed),mismatches=mismatch,
        requested_but_unplaced=sorted(set(requested)-set(observed)),placed=observed,
        pin_binding_check_passed=not mismatch and not(set(requested)-set(observed)),
        P11_39_external_control_input_present='control_rxd' in requested,
        control_UART_RXD_tied_high=".UART_RXD(1'b1)" in top,
        old_external_IMU_imu_rxd_used=False,
        inactive_SFP_or_unused_top_ports_require_review=True,
        voltage_and_external_electrical_measurement_accepted=False)
    save(out/'IO_pin_binding_review.json',result)
    return result


def main():
    global PHYSICAL
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--physical',type=Path,default=PHYSICAL,help='Explicit actual physical result of the same SYN49 VG')
    a=ap.parse_args();PHYSICAL=a.physical
    out=a.out;out.mkdir(parents=True,exist_ok=True)
    assert not(out/'summary.json').exists(),'This frozen audit already has a final result; read it instead'
    inputs=[SOURCE/'impl/gwsynthesis/g2_top.vg',PHYSICAL/'g2_top.vg',PHYSICAL/'impl/pnr/g2_top.vo',
            PHYSICAL/'impl/pnr/g2_top.tr',PHYSICAL/'g2.sdc',PHYSICAL/'g2.cst',SOURCE/'firmware/nav_fw.bin']
    before=[metadata(p) for p in inputs]
    assert inputs[0].read_bytes()==inputs[1].read_bytes(),'PNR input VG differs from SYN49'
    cdc=json.loads((out/'cdc26_connectivity.json').read_text())
    direct=json.loads((out/'direct_cdc_inventory.json').read_text())
    assert cdc['passed'] and direct['direct_crossings']==26 and all(p['single_second_register'] for p in direct['paths'])
    mem=memories(out)
    cl=physical_clock_reset_inventory(out)
    tr=timing(out,cdc)
    io=pins(out)
    unchanged=before==[metadata(p) for p in inputs]
    result=dict(recorded_at=datetime.now().astimezone().isoformat(),read_only=True,EDA_started=False,
        hardware_access=False,hashes_computed=False,source=str(SOURCE),physical=str(PHYSICAL),
        input_VG_byte_identical=True,inputs_unchanged=unchanged,
        memories={k:v for k,v in mem.items() if k!='rows'},clock_inventory=cl,
        CDC26_structure_passed=cdc['passed'],direct_DFF_CDC_inventory_count=direct['direct_crossings'],
        timing_summary={k:v for k,v in tr.items() if k not in ('CDC_route_delay_rows','CDC_detail_paths')},IO_summary=io,
        complete_signoff=False,vehicle_acceptance=False,
        open_items=['External RGMII min/max/both-edge timing and board/PHY delays',
            'Asynchronous reset phase and assertion pulse, physical grade/PVT coverage',
            'Full CDC payload stability, all stage2 route/resolution data and characterized MTBF',
            'Firmware loaded at04:10 differs from baked INIT; live identity/config must be recorded',
            'Independent stop with stalled50MHz, actual IR/key and actuator physical stopping',
            'IMU repair/axes/calibration, lidar link, actual motion/terrain/no-map/prior tasks'])
    result['selected_UART_architecture']=dict(command_and_task_status='CH9102 bidirectional',
        control_telemetry='P11-40 output',P11_39_receive_required=False,
        unused_control_RX_is_not_a_functional_defect=True,
        authority='User clarification and exact Zcode session final summary,2026-10-27')
    save(out/'summary.json',result)
    print(json.dumps(dict(out=str(out),memory_passed=mem['passed'],distributed_INIT_passed=mem['distributed_memories']['passed'],
        clock_roots=cl['roots'],CDC26_passed=cdc['passed'],setup=tr['setup_violated_endpoints'],hold=tr['hold_violated_endpoints'],
        printed_table_summary=tr['tables'],IO_pin_mismatches=io['mismatches'],unplaced_IO=io['requested_but_unplaced'],
        complete_signoff=False),ensure_ascii=False))


if __name__=='__main__':main()
