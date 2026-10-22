"""Collect tool-reported G2 facts; never turn failed mapping into PNR acceptance."""
import argparse,html,json,re,xml.etree.ElementTree as ET
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
    project=a.build/'impl/gwsynthesis/g2_top.prj'
    if project.exists():
        options={x.get('type'):x.get('value') for x in ET.parse(project).getroot().findall('./OptionList/Option')}
        r['actual_synthesis_global_freq_mhz']=float(options['global_freq'])
    timing=a.build/'impl/gwsynthesis/g2_top_syn.rpt.html'
    if timing.exists():
        report=timing.read_text(encoding='utf-8')
        def cells(table):
            return [html.unescape(re.sub('<[^>]+>','',x)).strip() for x in re.findall(r'<td[^>]*>(.*?)</td>',table,re.S)]
        perf=report.split('<a name="performance">',1)[1].split('</table>',1)[0]
        values=cells(perf);clocks=[]
        for i in range(0,len(values),6):
            n,name,constraint,fmax,levels,entity=values[i:i+6]
            clocks.append(dict(name=name,constraint_mhz=float(constraint.replace('(MHz)','')),
                estimated_fmax_mhz=float(fmax.replace('(MHz)','')),levels=int(levels),entity=entity))
        path=report.split('Path Summary:',1)[1].split('</table>',1)[0]
        fields=cells(path)
        r['synthesis_timing']=dict(clocks=clocks,first_path=dict(zip(fields[::2],fields[1::2])),
            scope='synthesis estimate only; actual PNR resource/timing acceptance must come from PNR reports')
    a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(r,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in r.items() if k not in ('synthesis_hierarchy','manifest')},ensure_ascii=False))


if __name__=='__main__':main()
