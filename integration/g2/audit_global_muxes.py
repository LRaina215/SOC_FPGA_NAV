"""Trace published global nets to physical loads; not an unrouted-net list."""
import argparse,collections,json,re
from pathlib import Path
from audit_pin_loads import CELLS,PORTS,INPUTS,NETS


def bits(expression):
    """Expand explicit flat net references, including escaped vector names."""
    result=[]
    for name,hi,lo in NETS.findall(expression):
        name=name.lstrip('\\')
        if hi:
            first,last=int(hi),int(lo or hi)
            result.extend(f'{name}[{i}]' for i in range(first,last+(1 if last>=first else -1),1 if last>=first else -1))
        else:result.append(name)
    return result


def audit(build, requested_nets=()):
    p=Path(build)/'impl/pnr';report=(p/'g2_top.rpt.txt').read_text()
    section=report.split('6. Global Clock Signals',1)[1].split('7. Pinout',1)[0]
    global_nets={line.split('|')[0].strip():line.split('|')[1].strip()for line in section.splitlines()if '| PRIMARY'in line or '| LW 'in line}
    norm=lambda s:re.sub(r'\s+(?=\[)','',s.replace('\\','').strip())
    cells=[(kind,norm(name),dict(PORTS.findall(body)))for kind,name,body in CELLS.findall((p/'g2_top.vo').read_text())]
    users=collections.defaultdict(list);drivers=collections.defaultdict(list)
    for kind,name,ports in cells:
        for pin,net in ports.items():
            if pin in INPUTS or pin in ('CLK','CLKA','CLKB','CLOCK'):
                for bit in bits(net):users[bit].append((kind,name,pin))
            elif pin in ('F','Q','O','DO','DOUT'):
                for bit in bits(net):drivers[bit].append(dict(kind=kind,name=name,ports=ports))
    rows=[]
    # Explicit additional targets may use ordinary routing in the report.
    # Keep that absence visible rather than labelling them as global nets.
    targets=dict(global_nets)
    for net in requested_nets:targets.setdefault(net,None)
    for net,resource in targets.items():
        row=dict(net=net,resource=resource,drivers=drivers[net],direct_pin_counts=dict(collections.Counter(x[2]for x in users[net])),
            direct_load_kinds=dict(collections.Counter(x[0]for x in users[net])),load_examples=users[net][:4])
        if net.startswith('u_control/model_thread/policy/')and resource=='PRIMARY':
            muxes=[(kind,name,ports)for kind,name,ports in cells if kind.startswith('LUT')and net in [norm(v)for k,v in ports.items()if k.startswith('I')]]
            sinks=[dst for kind,name,ports in muxes for dst in users[norm(ports['F'])]if dst[0].startswith('DFF')and dst[2]=='D'and '/policy/snapshot_'in dst[1]]
            row.update(selector_luts=len(muxes),direct_snapshot_data_sinks=len(sinks),snapshot_examples=sinks[:3])
        rows.append(row)
    return dict(build=str(Path(build).resolve()),global_nets=rows,
        scope='actual flat-netlist direct loads and reported global assignment; not route status, congestion or identification of unrouted nets')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('build',type=Path);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    result=audit(a.build);a.out.write_text(json.dumps(result,indent=2));print(json.dumps([r for r in result['global_nets']if 'selector_luts'in r]))
