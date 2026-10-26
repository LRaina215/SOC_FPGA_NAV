"""Close only proven post-PNR duplicated data FF names in a memory review.

Original rejection is immutable. No suffix-based equivalence, EDA, RTL writes,
or universal memory-caller claim. Each changed DI bit needs an actual FF proof.
"""
import argparse,json,re
from pathlib import Path
from datetime import datetime
from synthesis_connectivity import read,PORT
from audit_netlist_memories import PARAM
from revalidate_lidar_cdc import definitions
from review_small_block_stores import parameter_reader
from review_pnr50_bsram_init import metadata,value,LIB,save


def review(first,out):
    r=json.loads((first/'result.json').read_text());mem=json.loads((first/'actual_BSRAM_cells.json').read_text())
    assert r['passed'] is False
    assert all(metadata(Path(x['path']))==x for x in r['inputs'])
    source=Path(r['source']);physical=Path(r['physical'])
    bad=[x for x in r['all_rows'] if not all(x['checks'].values())]
    assert bad and all([k for k,v in x['checks'].items() if not v]==['DI_halves_reassembled'] for x in bad)
    needed=[]
    for row in bad:
        s=mem['source'][row['source']];half=row['physical_width_each']
        actual=mem['physical'][row['added']]['ports']['DI'][-half:]+mem['physical'][row['source']]['ports']['DI'][-half:]
        gold=s['ports']['DI']
        assert len(actual)==len(gold)
        for g,a in zip(gold,actual):
            if g!=a:needed.append((g,a))
    # This completion intentionally supports only the observed bus-data FFs.
    names={f'u_nav/wd_q_{i}_s0' for i in range(32)}
    cells=read(source/'impl/gwsynthesis/g2_top.vg',keep=lambda n:n=='u_nav' or n in names)
    assert set(cells)==names
    get=parameter_reader(definitions(source/'impl/gwsynthesis/g2_top.vg'))
    sg={c['ports']['Q'][0]:dict(c,parameters=get(c)) for c in cells.values()}
    text=(physical/'impl/pnr/g2_top.vo').read_text()
    wanted={a for g,a in needed};pc={}
    rx=re.compile(r'^(DFF\w*)\s+(\\\S+|[^\s(]+)\s*\(\s*\n(.*?)\)\s*;',re.M|re.S)
    norm=lambda s:re.sub(r'\s+(?=\[)','',s.strip().lstrip('\\'))
    for kind,raw,body in rx.findall(text):
        ports={p:[norm(e)] for p,e in PORT.findall(body)}
        q=ports.get('Q',[None])[0]
        if q in wanted:
            assert q not in pc
            pc[q]=dict(name=norm(raw),kind=kind,ports=ports,parameters={})
    assert set(pc)==wanted
    byname={c['name']:c for c in pc.values()}
    for raw,key,v in PARAM.findall(text):
        name=norm(raw)
        if name in byname:byname[name]['parameters'][key]=v.strip()
    macro=LIB.read_text();proof=[]
    for g,a in needed:
        s,p=sg[g],pc[a]
        assert s['kind']==p['kind']=='DFFRE'
        body=re.search(r'\bmodule\s+DFFRE\b(.*?)\bendmodule\b',macro,re.S)[1]
        defaults=dict(re.findall(r'\bparameter\s+(\w+)\s*=\s*([^;]+);',body))
        effective=lambda c:{k:value(v.strip()) for k,v in (defaults|c['parameters']).items()}
        checks=dict(same_kind=s['kind']==p['kind'],
            every_nonQ_input_equal={k:v for k,v in s['ports'].items() if k!='Q'}=={k:v for k,v in p['ports'].items() if k!='Q'},
            every_effective_parameter_INIT_equal=effective(s)==effective(p))
        proof.append(dict(original_net=g,physical_replica_net=a,source=s,physical=p,checks=checks))
    assert all(all(x['checks'].values()) for x in proof),'Exact replica comparison failed'
    after=all(metadata(Path(x['path']))==x for x in r['inputs'])
    assert after
    result=dict(recorded_at=datetime.now().astimezone().isoformat(),passed=True,
        scope='Supplement to first memory audit: only all needed TX-shadow DI bus FF copies, same clock/D/CE/RESET/effective INIT implies identical trajectories. No arbitrary name aliases or memory caller proof.',
        original_rejection=str(first/'result.json'),source=r['source'],physical=r['physical'],
        first_all_other_memory_checks_passed=True,actual_data_replica_bits_proved=len(proof),replica_checks=proof,
        actual_physical_BSRAM=r['actual_physical_BSRAM'],split_pairs=r['split_pairs'],
        CPU128KiB=r['CPU128KiB'],all_distributed_INIT_preserved=r['all_distributed_INIT_preserved'],
        source_and_physical_metadata_unchanged=after,inputs=r['inputs'],
        complete_memory_geometry_INIT_split_and_data_replica_binding_passed=True,
        whole_memory_caller_logic_or_runtime_acceptance=False,programmed=False,deployment_accepted=False)
    save(out/'result.json',result)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--first',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();a.out.mkdir(exist_ok=False);r=review(a.first,a.out)
    print(json.dumps({k:v for k,v in r.items() if k not in ('replica_checks','inputs')},ensure_ascii=False))
