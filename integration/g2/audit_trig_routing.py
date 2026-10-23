"""Trace initialized navigation tables to actual SYN cells and local AD loads.

Counts structural evidence only. This is neither a congestion measurement nor
a global fanout calculation: hierarchy ports/aliases are not merged.
"""
import argparse,json,re
from collections import Counter
from pathlib import Path
from audit_netlist_memories import MODULE,INST
from audit_pin_loads import PORTS,NETS

CELLS=re.compile(r'^\s{2}(\w+)\s+(\\\S+|[^\s(]+)\s*\(\s*\n(.*?)^\);',re.M|re.S)


def audit(build):
    path=build/'impl/gwsynthesis/g2_top.vg'
    modules=dict(MODULE.findall(path.read_text(encoding='utf-8')))
    rows=[]
    def walk(kind,path):
        for child,instance in INST.findall(modules[kind]):
            name=instance.lstrip('\\');full=path+'/'+name if path else name
            if child not in modules:continue
            if name in ('u_sin','u_cos'):
                body=modules[child];cells=Counter(t for t,n in INST.findall(body));loads=Counter()
                for typ,inst,pins in CELLS.findall(body):
                    if typ!='ROM16':continue
                    for port,expression in PORTS.findall(pins):
                        if port!='AD':continue
                        for net,hi,lo in NETS.findall(expression):
                            net=net.lstrip('\\')
                            if net in ('VCC','GND'):continue
                            bits=[net] if not hi else [net+'['+str(x)+']' for x in range(min(int(hi),int(lo or hi)),max(int(hi),int(lo or hi))+1)]
                            loads.update(bits)
                rows.append(dict(path=full,module_type=child,primitive_counts=dict(cells),
                    local_ROM16_address_loads=dict(loads.most_common())))
            walk(child,full)
    walk('g2_top','')
    assert len(rows)==4,('expected exactly four frozen navigation trig tables',rows)
    return dict(build=str(build),netlist=str(path),tables=rows,
        total_table_primitives=dict(sum((Counter(x['primitive_counts'])for x in rows),Counter())),
        scope='actual SYN primitive connectivity; local AD pin references only, no alias merge, physical congestion or timing acceptance')


def main():
    ap=argparse.ArgumentParser();ap.add_argument('build',type=Path);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    result=audit(a.build.resolve());a.out.parent.mkdir(parents=True,exist_ok=True)
    a.out.write_text(json.dumps(result,indent=2),encoding='utf-8',newline='\n')
    print(json.dumps(result))


if __name__=='__main__':main()
