"""Count direct non-clock input pin references in the actual flat PNR netlist.

This does not identify unrouted nets or resolve assignment aliases. It supplies
measured load evidence for a subsequent, separately validated PNR experiment.
"""
import argparse,json,re
from collections import Counter,defaultdict
from pathlib import Path

INPUTS=set('I I0 I1 I2 I3 D CE CEN RESET SET CLEAR PRESET S SE WRE OCE DI AD ADA ADB DIA DIB CEA CEB RESETA RESETB OCEA OCEB WREA WREB BLKSEL BLKSELA BLKSELB A B C AIN BIN CIN CASI ASIGN BSIGN ACCLOAD ALUSEL MODESEL ADDNSUB CE0 CE1 CE2 CE3'.split())
CELLS=re.compile(r'^(\w+)\s+(\\\S+|\w+)\s*\(\s*\n(.*?)^\);',re.M|re.S)
PORTS=re.compile(r'\.([A-Z][A-Z0-9_]*)\(([^()]*)\)')
NETS=re.compile(r"(?<![A-Za-z0-9_'])"+r'(\\[^\s,{}]+|[A-Za-z_][A-Za-z0-9_$]*)\s*(?:\[(\d+)(?::(\d+))?\])?')


def main():
    ap=argparse.ArgumentParser();ap.add_argument('netlist',type=Path);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    text=a.netlist.read_text(encoding='utf-8');counts=Counter();pins=defaultdict(Counter);examples=defaultdict(list);n=0
    for cell in CELLS.finditer(text):
        kind,name,body=cell.groups();n+=1
        for pin,expression in PORTS.findall(body):
            if pin not in INPUTS:continue
            for net,hi,lo in NETS.findall(expression):
                net=net.replace('\\','')
                if net in ('GND','VCC'):continue
                bits=[net] if not hi else [net+'['+str(x)+']' for x in range(min(int(hi),int(lo or hi)),max(int(hi),int(lo or hi))+1)]
                for bit in bits:
                    counts[bit]+=1;pins[bit][pin]+=1
                    if len(examples[bit])<3:examples[bit].append(kind+' '+name.lstrip('\\')+'.'+pin)
    result=dict(netlist=str(a.netlist),cells_scanned=n,direct_load_pins=sum(counts.values()),
        top_direct_loads=[dict(net=net,pins=count,port_counts=dict(pins[net]),examples=examples[net]) for net,count in counts.most_common(24)],
        scope='direct input-pin references only; excludes clocks/constants, does not resolve assign aliases or identify unrouted nets')
    a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(dict(cells_scanned=n,top_direct_loads=result['top_direct_loads'][:8]),ensure_ascii=False))


if __name__=='__main__':main()
