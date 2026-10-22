"""Expand Gowin's structural Verilog hierarchy and account for every RAM leaf.

No synthesis estimates or hashes. This is a netlist inventory; PNR remains the
authority for physical sites. Unknown non-module primitives are listed by type.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

BSRAM={'SP','SPX9','SDPB','SDPX9B','DPB','DPX9B','pROM','pROMX9','SDP36KE'}
INST=re.compile(r'^\s{2}(\w+)\s+(\\\S+|[^\s(]+)\s*\(',re.M)
MODULE=re.compile(r'^module\s+(\S+)\s*\((.*?)^endmodule',re.M|re.S)
PARAM=re.compile(r'^defparam\s+(.*?)\.([A-Za-z0-9_]+)\s*=\s*(.*?);',re.M)


def inventory(build):
    text=(build/'impl/gwsynthesis/g2_top.vg').read_text(encoding='utf-8')
    mods={name:body for name,body in MODULE.findall(text)}
    assert 'g2_top' in mods
    children={name:INST.findall(body) for name,body in mods.items()}
    parameters={name:{} for name in mods}
    for name,body in mods.items():
        for inst,key,value in PARAM.findall(body):
            if key.startswith('INIT_RAM'):continue
            parameters[name].setdefault(inst.strip(),{})[key]=value.strip()
    leaves=[];primitive_types=Counter();multiplicity=Counter();definitions=Counter()
    for name,items in children.items():
        definitions.update(kind for kind,inst in items if kind in BSRAM)
    def walk(name,path):
        multiplicity[name]+=1
        for kind,inst in children[name]:
            p=path+'/'+inst
            if kind in mods:walk(kind,p)
            else:
                primitive_types[kind]+=1
                if kind in BSRAM:
                    leaves.append(dict(path=p,module_type=name,primitive=kind,
                        parameters=parameters[name].get(inst,{})))
    walk('g2_top','g2_top')
    syn=ET.parse(build/'impl/gwsynthesis/g2_top_syn_rsc.xml').getroot()
    log=(build/'gw_sh.log').read_text(encoding='utf-8',errors='replace')
    if (build/'impl/pnr/g2_top.log').exists():log+='\n'+(build/'impl/pnr/g2_top.log').read_text(encoding='utf-8',errors='replace')
    pnr=re.search(r'The number\((\d+)\) of BSRAM',log)
    totals=Counter(x['path'].split('/')[1] for x in leaves)
    wide=[x for x in leaves if any(x['parameters'].get(k) in ('32','36') for k in ('BIT_WIDTH','BIT_WIDTH_0','BIT_WIDTH_1'))]
    command=(build/'impl/pnr/cmd.do').read_text(encoding='utf-8')
    return dict(build=str(build.resolve()),synthesis_report_bsram=syn.get('T_Bsram'),
        definition_bsram_count=sum(definitions.values()),expanded_bsram_count=len(leaves),
        pnr_bsram_demand=int(pnr[1]) if pnr else None,by_top_instance=dict(totals),
        by_primitive=dict(Counter(x['primitive'] for x in leaves)),
        wide32_36_count=len(wide),wide32_36_by_top=dict(Counter(x['path'].split('/')[1] for x in wide)),
        pnr_wide_split_enabled='-convert_sdp32_36_to_sdp16_18' in command,
        predicted_split_bsram=len(leaves)+len(wide),
        predicted_bsram_under_actual_option=len(leaves)+(len(wide) if '-convert_sdp32_36_to_sdp16_18' in command else 0),
        reused_modules={k:v for k,v in multiplicity.items() if v>1},
        primitive_types=dict(primitive_types),leaves=leaves)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('build',type=Path);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    r=inventory(a.build);a.out.parent.mkdir(exist_ok=True,parents=True)
    a.out.write_text(json.dumps(r,indent=2),encoding='utf-8',newline='\n')
    print(json.dumps({k:v for k,v in r.items() if k not in ('leaves','primitive_types','reused_modules')},ensure_ascii=False))


if __name__=='__main__':main()
