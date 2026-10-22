"""Collect tool-reported G2 facts; never turn failed mapping into PNR acceptance."""
import argparse,json,re,xml.etree.ElementTree as ET
from pathlib import Path


def main():
    ap=argparse.ArgumentParser();ap.add_argument('build',type=Path);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    s=(a.build/'gw_sh.log').read_text(encoding='utf-8',errors='replace')
    r=dict(build=str(a.build.resolve()),errors=[x for x in s.splitlines() if re.search(r'ERROR\s*\(',x)],
        programmed=False,manifest=json.loads((a.build/'manifest.json').read_text()))
    match=re.search(r'The number\((\d+)\) of BSRAM.*?limit\((\d+)\)',s)
    if match:r.update(pnr_mapping_bsram_requirement=int(match[1]),bsram_limit=int(match[2]),placement_routing_passed=False)
    if 'RP0006' in s:r.update(synthesis_logic_overflow=True,placement_routing_passed=False)
    xml=a.build/'impl/gwsynthesis/g2_top_syn_rsc.xml'
    if xml.exists():
        root=ET.parse(xml).getroot();rows=[]
        def walk(e,parent):
            path=parent+'/'+e.get('name','');rows.append(dict(path=path,**e.attrib))
            for child in e:walk(child,path)
        walk(root,'');r['synthesis_hierarchy']=rows
    a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(r,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in r.items() if k not in ('synthesis_hierarchy','manifest')},ensure_ascii=False))


if __name__=='__main__':main()
