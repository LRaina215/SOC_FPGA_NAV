"""Reconcile actual post-PNR memory primitives with the frozen synthesis tree."""
import argparse,json,re
from collections import Counter
from pathlib import Path
from audit_netlist_memories import BSRAM,PARAM,inventory


def main():
    ap=argparse.ArgumentParser();ap.add_argument('source',type=Path);ap.add_argument('physical',type=Path)
    ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    syn=inventory(a.source);p=a.physical/'impl/pnr'
    text=(p/'g2_top.vo').read_text(encoding='utf-8')
    kinds='|'.join(sorted(BSRAM,key=len,reverse=True))
    def normalize(name):return name.strip().replace('\\','')
    instances={normalize(name):dict(primitive=kind,parameters={}) for kind,name in
        re.findall(r'^('+kinds+r')\s+(\\\S+|\w+)\s*\(',text,re.M)}
    for name,key,value in PARAM.findall(text):
        name=normalize(name)
        if name in instances and not key.startswith('INIT_RAM'):
            instances[name]['parameters'][key]=value.strip()
    source={normalize(x['path'].split('/',1)[1]):x for x in syn['leaves']}
    command=(p/'cmd.do').read_text();split='-convert_sdp32_36_to_sdp16_18' in command
    wide=[(name,x) for name,x in source.items() if any(x['parameters'].get(k) in ('32','36')
        for k in ('BIT_WIDTH','BIT_WIDTH_0','BIT_WIDTH_1'))]
    details=[]
    for name,x in wide:
        parent,leaf=name.rsplit('/',1)
        candidates=[parent+'/gowin_add_'+leaf,parent+'/gowin_add_'+x['primitive']+'_'+leaf]
        added=next((n for n in candidates if n in instances),candidates[0])
        details.append(dict(source=name,source_primitive=x['primitive'],source_parameters=x['parameters'],
            original_after_pnr=instances.get(name),added_name=added if added in instances else None,added_after_pnr=instances.get(added)))
    report=(p/'g2_top.rpt.txt').read_text();actual=int(re.search(r'^\s*BSRAM\s*\|\s*(\d+)/',report,re.M)[1])
    predicted=len(source)+(len(wide) if split else 0)
    result=dict(source=str(a.source),physical=str(a.physical),source_count=len(source),wide_split_enabled=split,
        wide_count=len(wide),wide_by_primitive=dict(Counter(x['primitive'] for name,x in wide)),
        wide_by_top=dict(Counter(name.split('/')[0] for name,x in wide)),predicted_count=predicted,
        actual_report_count=actual,post_pnr_primitive_count=len(instances),
        counts_reconciled=predicted==actual==len(instances),
        matched_split_pairs=sum(bool(x['original_after_pnr'] and x['added_after_pnr']) for x in details),
        source_by_primitive=dict(Counter(x['primitive'] for x in source.values())),
        actual_by_primitive=dict(Counter(x['primitive'] for x in instances.values())),
        actual_by_top=dict(Counter(name.split('/')[0] for name in instances)),
        added_instances=[dict(name=name,**x) for name,x in instances.items() if name not in source],
        missing_source_instances=[name for name in source if name not in instances],wide_details=details,
        scope='post-PNR storage accounting only; routing failure and timing acceptance remain separate')
    a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('added_instances','wide_details','missing_source_instances')},ensure_ascii=False))


if __name__=='__main__':main()
