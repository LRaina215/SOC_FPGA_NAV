"""Explicit host FIFO/toggle CDC inventory; vendor PCS/reset review is separate.

SYN candidates are checked against the finished flat physical netlist later.
No clock-domain-wide exception is generated. Protected vendor logic is not
silently treated as reviewed by the structural reader.
"""
import argparse,json,re
from collections import defaultdict,Counter
from pathlib import Path
from audit_pin_loads import CELLS,PORTS,INPUTS
from audit_cdc_first_stages import bits

GROUPS=[
 ('host_rx_write_gray',r'u_nav/u_neth/u_rxf/wp_g_r1_(\d+)_s\d+',13,'h_rx_clk','clk50_d',6.),
 ('host_rx_read_gray',r'u_nav/u_neth/u_rxf/rp_g_w1_(\d+)_s\d+',13,'clk50_d','h_rx_clk',18.),
 ('host_tx_write_gray',r'u_nav/u_neth/u_txf/wp_g_r1_(\d+)_s\d+',12,'clk50_d','h_tx_clk',18.),
 ('host_tx_read_gray',r'u_nav/u_neth/u_txf/rp_g_w1_(\d+)_s\d+',12,'h_tx_clk','clk50_d',6.),
 ('host_frame_toggle',r'u_nav/u_neth/u_fs/u_s/s1_0_s\d+',1,'clk50_d','h_tx_clk',18.)]


def audit(path, synthesis=False):
    if synthesis:
        from synthesis_connectivity import read
        # Only these hierarchy nodes are inspected. Encrypted PCS is not parsed.
        cells=read(path,lambda n: not n.startswith(('u_control/','u_cpu/')) and
                   (not n.startswith('u_nav/') or n.startswith('u_nav/u_neth')))
    else:
        cells={name.lstrip('\\'):dict(name=name.lstrip('\\'),kind=kind,
               ports={p:bits(e) for p,e in PORTS.findall(body)})
               for kind,name,body in CELLS.findall(Path(path).read_text(encoding='utf-8'))}
    drivers=defaultdict(list);loads=defaultdict(list)
    for n,c in cells.items():
        if c['kind'].startswith('DFF'):
            for net in c['ports']['Q']:drivers[net].append(n)
        for pin,nets in c['ports'].items():
            if pin in INPUTS:
                for net in nets:loads[net].append((n,pin))
    rows=[];errors=[]
    for group,pattern,expected,srcclk,dstclk,budget in GROUPS:
        selected=[c for n,c in cells.items() if re.fullmatch(pattern,n)]
        if len(selected)!=expected:errors.append(f'{group}: expected {expected}, found {len(selected)}')
        for first in selected:
            n=first['name'];p=first['ports'];fails=[]
            sources=drivers[p['D'][0]] if len(p['D'])==1 else []
            source=cells[sources[0]] if len(sources)==1 else None
            fanout=loads[p['Q'][0]] if len(p['Q'])==1 else []
            second=cells[fanout[0][0]] if len(fanout)==1 and fanout[0][1]=='D' else None
            if source is None or source['ports']['CLK']!=[srcclk]:fails.append('source register/clock mismatch')
            if p['CLK']!=[dstclk] or p.get('CE',['VCC'])!=['VCC']:fails.append('first clock/enable mismatch')
            expected_second=n.replace('/wp_g_r1_','/wp_g_r2_').replace('/rp_g_w1_','/rp_g_w2_').replace('/s1_','/s2_')
            if second is None or not second['kind'].startswith('DFF'):fails.append('first Q does not drive exactly one second D')
            elif (second['ports']['CLK']!=[dstclk] or second['ports'].get('CE',['VCC'])!=['VCC'] or
                  re.sub(r'_s\d+$','',second['name'])!=re.sub(r'_s\d+$','',expected_second)):
                fails.append('second register/clock/enable mismatch')
            rows.append(dict(group=group,source_register=source['name'] if source else None,first_register=n,
                second_register=second['name'] if second else None,source_clock=srcclk,target_clock=dstclk,
                route_max_delay_ns=budget,first_q_direct_loads=fanout,passed=not fails,failures=fails))
            errors += [n+': '+f for f in fails]
    return dict(netlist=str(Path(path).resolve()),stage='synthesis' if synthesis else 'physical',
                passed=not errors,paths=rows,expected_paths=51,errors=errors,
                scope='host FIFO Gray and frame-toggle connections only; vendor PCS, status, IO/reset and MTBF remain separate')


def sdc_fragment(result):
    assert result['passed'] and len(result['paths'])==51
    assert Counter(p['group'] for p in result['paths'])=={g:n for g,_,n,*_ in GROUPS}
    budgets={g:b for g,_,_,_,_,b in GROUPS};lines=['# Host FIFO Gray and frame-toggle source Q to first D only.',
      '# 6 ns is below the 125 MHz source period; 18 ns is below the 50 MHz source period.',
      '# Keep every synchronizer second stage timed normally.']
    for p in result['paths']:
        assert p['route_max_delay_ns']==budgets[p['group']]
        assert all(re.fullmatch(r'[A-Za-z0-9_/\[\].$-]+',p[k]) for k in ('source_register','first_register'))
        source='[get_pins {'+p['source_register']+'/Q}]';target='[get_pins {'+p['first_register']+'/D}]'
        lines += [f'set_false_path -from {source} -to {target} -hold',
                  f'set_max_delay -from {source} -to {target} {p["route_max_delay_ns"]:.3f}']
    return '\n'.join(lines)+'\n'


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('netlist',type=Path);ap.add_argument('--synthesis',action='store_true')
    ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();result=audit(a.netlist,a.synthesis)
    a.out.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='paths'}))
    raise SystemExit(not result['passed'])
