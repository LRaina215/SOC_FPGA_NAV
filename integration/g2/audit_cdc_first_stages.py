"""Audit the four known RX/core synchronizer groups in a flat PNR netlist.

This is an explicit endpoint/connectivity inventory, not complete CDC signoff.
An optional SDC fragment constrains only audited source-Q -> first-D data;
second-stage timing, resets and all other crossings remain timed normally.
"""
import argparse,json,re
from collections import defaultdict
from pathlib import Path
from audit_pin_loads import CELLS,PORTS,NETS,INPUTS


def bits(expression):
    result=[]
    for net,hi,lo in NETS.findall(expression):
        net=net.lstrip('\\')
        if hi:
            result += [net+'['+str(i)+']' for i in range(min(int(hi),int(lo or hi)),max(int(hi),int(lo or hi))+1)]
        else:result.append(net)
    return result


def audit(netlist):
    cells={};drivers=defaultdict(list);loads=defaultdict(list)
    for match in CELLS.finditer(Path(netlist).read_text(encoding='utf-8')):
        kind,name,body=match.groups();name=name.lstrip('\\');ports=dict(PORTS.findall(body))
        cells[name]=dict(name=name,kind=kind,ports=ports)
        if kind.startswith('DFF'):
            for net in bits(ports['Q']):drivers[net].append(name)
        for pin,expression in ports.items():
            if pin in INPUTS:
                for net in bits(expression):loads[net].append((name,pin))
    groups=[('rx_write_gray',r'u_nav/u_netl/u_rxf/wp_g_r1_(\d+)_s\d+',12,'rgmii_rxc_d','clk50_d',8.0),
            ('core_read_gray',r'u_nav/u_netl/u_rxf/rp_g_w1_(\d+)_s\d+',12,'clk50_d','rgmii_rxc_d',18.0),
            ('rx_clock_monitor',r'u_lmac/rsync_0_s\d+',1,'rgmii_rxc_d','clk50_d',8.0),
            ('rx_sampling_select',r'u_lmac/neg_s_0_s\d+',1,'clk50_d','rgmii_rxc_d',18.0)]
    rows=[];errors=[]
    for group,pattern,expected,source_clock,target_clock,budget in groups:
        selected=[c for n,c in cells.items() if re.fullmatch(pattern,n)]
        if len(selected)!=expected:errors.append(f'{group}: expected {expected} first registers, found {len(selected)}')
        for first in selected:
            name=first['name'];p=first['ports'];failures=[]
            source_nets=bits(p['D']);source_names=drivers.get(source_nets[0],[]) if len(source_nets)==1 else []
            source=cells[source_names[0]] if len(source_names)==1 else None
            first_q=bits(p['Q']);fanout=loads.get(first_q[0],[]) if len(first_q)==1 else []
            second=cells[fanout[0][0]] if len(fanout)==1 and fanout[0][1]=='D' else None
            if source is None:failures.append('first D is not driven directly by exactly one register')
            elif bits(source['ports']['CLK'])!=[source_clock]:failures.append('source clock mismatch')
            if bits(p['CLK'])!=[target_clock]:failures.append('first-stage clock mismatch')
            if bits(p.get('CE','VCC'))!=['VCC']:failures.append('first-stage enable is not constant one')
            if second is None or not second['kind'].startswith('DFF'):failures.append('first Q fanout is not exactly one second-register D')
            else:
                if bits(second['ports']['CLK'])!=[target_clock]:failures.append('second-stage clock mismatch')
                if bits(second['ports'].get('CE','VCC'))!=['VCC']:failures.append('second-stage enable is not constant one')
                expected_second=name.replace('/wp_g_r1_','/wp_g_r2_').replace('/rp_g_w1_','/rp_g_w2_').replace('/rsync_0_','/rsync_1_').replace('/neg_s_0_','/neg_s_1_')
                # Synthesis may change the trailing reset/init suffix, but
                # the bit and intended second synchronizer must still match.
                if re.sub(r'_s\d+$','',second['name'])!=re.sub(r'_s\d+$','',expected_second):failures.append('unexpected second synchronizer register')
            row=dict(group=group,source_register=source['name'] if source else None,first_register=name,
                second_register=second['name'] if second else None,source_clock=source_clock,target_clock=target_clock,
                route_max_delay_ns=budget,first_q_direct_loads=fanout,passed=not failures,failures=failures)
            rows.append(row);errors.extend(name+': '+f for f in failures)
    return dict(netlist=str(Path(netlist).resolve()),passed=not errors,errors=errors,paths=rows,
        expected_paths=26,scope='four explicit synchronizer groups only; not full CDC, reset, IO, MTBF or physical skew signoff')


def sdc_fragment(result):
    assert result['passed'] and len(result['paths'])==26
    from collections import Counter
    budgets={'rx_write_gray':8.0,'core_read_gray':18.0,'rx_clock_monitor':8.0,'rx_sampling_select':18.0}
    assert Counter(p['group'] for p in result['paths'])==dict(rx_write_gray=12,core_read_gray=12,rx_clock_monitor=1,rx_sampling_select=1)
    lines=['# Only audited source-Q -> first-stage D. Keep stage1 -> stage2 timed.',
           '# Hold-only exception; maximum data-path budgets remain active.',
           '# No set_clock_groups and no setup false paths.']
    for path in result['paths']:
        assert path['route_max_delay_ns']==budgets[path['group']]
        assert all(re.fullmatch(r'[A-Za-z0-9_/\[\].$-]+',path[key]) for key in ('source_register','first_register'))
        source='[get_pins {'+path['source_register']+'/Q}]'
        target='[get_pins {'+path['first_register']+'/D}]'
        lines += [f'set_false_path -from {source} -to {target} -hold',
                  f'set_max_delay -from {source} -to {target} {path["route_max_delay_ns"]:.3f}']
    return '\n'.join(lines)+'\n'


def main():
    ap=argparse.ArgumentParser();ap.add_argument('netlist',type=Path);ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--emit-sdc',type=Path);a=ap.parse_args();r=audit(a.netlist)
    a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(r,indent=2),encoding='utf-8')
    if a.emit_sdc:
        a.emit_sdc.parent.mkdir(parents=True,exist_ok=True);a.emit_sdc.write_text(sdc_fragment(r),encoding='utf-8',newline='\n')
    print(json.dumps(dict(passed=r['passed'],paths=len(r['paths']),errors=r['errors'])))
    return not r['passed']


if __name__=='__main__':raise SystemExit(main())
