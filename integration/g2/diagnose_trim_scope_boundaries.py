"""Read mapped tradeoff boundaries; no waivers, SAT, build or hardware actions."""
import argparse
from collections import Counter,defaultdict
from datetime import datetime
import gc,json,re
from pathlib import Path
from synthesis_connectivity import read
from revalidate_lidar_cdc import definitions
from review_small_block_stores import parameter_reader
from review_delivery_trim_joint import within
from audit_netlist_memories import BSRAM

def group(name,depth=3):
    return '/'.join(name.split('/')[:-1][:depth]) if '/' in name else '<top>'

def snapshot(path,allowed):
    # The SoC has a separate completed retained-next-state SAT proof. Do not
    # duplicate its memory-heavy expressions or classify it as a free waiver.
    cells=read(path,keep=lambda n:not n.startswith('u_soc/'))
    params=parameter_reader(definitions(path))
    records={};states={};opaque={};cut_outputs=defaultdict(list)
    for name,c in cells.items():
        row=dict(kind=c['kind'],ports=c['ports'],parameters=params(c))
        if within(name,allowed):
            continue
        records[name]=json.dumps(row,sort_keys=True,separators=(',',':'))
        if c['kind'].startswith('DFF'):
            q,=c['ports']['Q'];assert q not in states,('Q alias',q)
            states[q]=dict(name=name,**row)
        if c['kind'] in BSRAM or c['kind']=='RAM16SDP4' or c['kind'].startswith(('MULT','ALU54','PADD')):
            opaque[name]=row
    del cells,params;gc.collect()
    return records,states,opaque

def diagnose(origin,candidate,profile,out):
    origin,candidate,out=map(lambda p:Path(p).resolve(),(origin,candidate,out))
    profile=json.loads(Path(profile).read_text(encoding='utf-8'));allowed=profile['allowed_changed_prefixes']
    old,qs_old,ops_old=snapshot(origin/'impl/gwsynthesis/g2_top.vg',allowed)
    new,qs_new,ops_new=snapshot(candidate/'impl/gwsynthesis/g2_top.vg',allowed)
    changes=dict(added=sorted(new.keys()-old.keys()),removed=sorted(old.keys()-new.keys()),changed=sorted(n for n in old.keys()&new.keys() if old[n]!=new[n]))
    rows=[];counts=defaultdict(Counter);kinds=defaultdict(Counter)
    for category,names in changes.items():
        for name in names:
            a=json.loads(old[name]) if name in old else None;b=json.loads(new[name]) if name in new else None
            delta=None
            if a and b:
                delta=dict(kind_changed=a['kind']!=b['kind'],changed_ports=[p for p in a['ports'].keys()|b['ports'].keys() if a['ports'].get(p)!=b['ports'].get(p)],changed_parameters=[k for k in a['parameters'].keys()|b['parameters'].keys() if a['parameters'].get(k)!=b['parameters'].get(k)])
            rows.append(dict(category=category,name=name,group=group(name),before=a,after=b,delta=delta))
            counts[group(name)][category]+=1;kinds[category][(a or b)['kind']]+=1
    retained=[];type_changes=[];initial_changes=[];clock_changes=[]
    for q in sorted(qs_old.keys()&qs_new.keys()):
        a,b=qs_old[q],qs_new[q]
        if a['kind']!=b['kind']:type_changes.append(dict(Q=q,before=a,after=b))
        def initial(x):return x['parameters'].get('INIT',"1'b1" if x['kind'] in ('DFFPE','DFFSE') else "1'b0")
        if initial(a)!=initial(b):initial_changes.append(dict(Q=q,before=initial(a),after=initial(b)))
        if a['ports']['CLK']!=b['ports']['CLK']:clock_changes.append(dict(Q=q,before=a['ports']['CLK'],after=b['ports']['CLK']))
        if a!=b:retained.append(dict(Q=q,before=a,after=b))
    opaque_changes=dict(added=[dict(name=n,**ops_new[n]) for n in sorted(ops_new.keys()-ops_old.keys())],removed=[dict(name=n,**ops_old[n]) for n in sorted(ops_old.keys()-ops_new.keys())],changed=[dict(name=n,before=ops_old[n],after=ops_new[n]) for n in sorted(ops_new.keys()&ops_old.keys()) if ops_old[n]!=ops_new[n]])
    result=dict(recorded_at=datetime.now().astimezone().isoformat(),origin=str(origin),candidate=str(candidate),allowed_scopes=allowed,SoC_excluded_only_because_separate_actual_retained_state_proof_is_owned_elsewhere=True,scope_outside_primitive_change_counts={k:len(v) for k,v in changes.items()},scope_outside_primitive_changes_total=sum(map(len,changes.values())),by_group={k:dict(v) for k,v in sorted(counts.items(),key=lambda x:-sum(x[1].values()))},by_primitive={k:dict(v) for k,v in kinds.items()},DFF_Q=dict(before=len(qs_old),after=len(qs_new),added=[dict(Q=q,**qs_new[q]) for q in sorted(qs_new.keys()-qs_old.keys())],removed=[dict(Q=q,**qs_old[q]) for q in sorted(qs_old.keys()-qs_new.keys())],retained_modified=retained,retained_kind_changes=type_changes,retained_INIT_changes=initial_changes,retained_CLK_changes=clock_changes),retained_opaque_primitives=opaque_changes,all_changes=rows,passed=False,scope='Classification evidence only. Changed mapped logic still requires explicit boundary equivalence or a declared and source-bound functional tradeoff; no new prefix waiver.',SYN_started=False,PNR_started=False,programmed=False)
    qdiff=result['DFF_Q']
    added_by_name={r['name']:r for r in qdiff['added']}
    removed_by_name={r['name']:r for r in qdiff['removed']}
    aliases=[]
    for name in sorted(added_by_name.keys()&removed_by_name.keys()):
        a,b=removed_by_name[name],added_by_name[name]
        aliases.append(dict(cell=name,kind_before=a['kind'],kind_after=b['kind'],old_Q=a['Q'],new_Q=b['Q'],same_CLK=a['ports']['CLK']==b['ports']['CLK'],same_INIT=a['parameters'].get('INIT')==b['parameters'].get('INIT'),nonQ_changed_ports=[k for k in a['ports'].keys()|b['ports'].keys() if k!='Q' and a['ports'].get(k)!=b['ports'].get(k)]))
    qdiff['same_primitive_name_Q_net_alias_changes']=aliases
    qdiff['removed_unmatched_primitive_names']=[removed_by_name[n] for n in sorted(removed_by_name.keys()-added_by_name.keys())]
    qdiff['added_unmatched_primitive_names']=[added_by_name[n] for n in sorted(added_by_name.keys()-removed_by_name.keys())]
    out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(path=str(out),changes=result['scope_outside_primitive_change_counts'],total=result['scope_outside_primitive_changes_total'],groups=result['by_group'],DFF_Q=dict(before=len(qs_old),after=len(qs_new),added=len(result['DFF_Q']['added']),removed=len(result['DFF_Q']['removed']),retained_modified=len(retained),kind_changes=len(type_changes),INIT_changes=len(initial_changes),CLK_changes=len(clock_changes)),opaque_changes={k:len(v) for k,v in opaque_changes.items()})))
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser()
    for key in ('origin','candidate','profile','out'):p.add_argument('--'+key,type=Path,required=True)
    a=p.parse_args();diagnose(a.origin,a.candidate,a.profile,a.out)
