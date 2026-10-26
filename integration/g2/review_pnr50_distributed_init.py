"""Read-only actual SYN37 -> PNR50 RAM16/ROM16 parameter and INIT binding.

No EDA and no mutation of frozen source, physical output, or BSRAM evidence.
Expands source hierarchy and reads only selected primitive headers/defparams.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import re

from audit_netlist_memories import MODULE, INST, PARAM

BASE = Path('F:/nav_integration_g2_20261023')
LIB = Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
SELECT = re.compile(r'(?:RAM16\w*|ROM16\w*)\Z')
PHYSICAL_CELL = re.compile(r'^(RAM16\w*|ROM16\w*)\s+(\\\S+|[^\s(]+)\s*\(\s*\n(.*?)\)\s*;', re.M|re.S)


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')


def meta(path):
    s=path.stat()
    return dict(path=str(path.resolve()), bytes=s.st_size, mtime_ns=s.st_mtime_ns)


def literal(raw):
    m=re.fullmatch(r"(\d+)'([bhd])([0-9a-fA-F]+)",raw)
    if m:
        n=int(m[3],{'b':2,'h':16,'d':10}[m[2]])
        assert n < 1<<int(m[1]), ('out-of-width parameter',raw)
        return n
    if re.fullmatch(r'\d+',raw):return int(raw)
    if raw.startswith('"') and raw.endswith('"'):return raw[1:-1]
    raise ValueError(('unsupported parameter literal',raw))


def source_rows(vg):
    modules=dict(MODULE.findall(vg.read_text(encoding='utf-8')))
    children={k:[(kind,name.lstrip('\\')) for kind,name in INST.findall(body)] for k,body in modules.items()}
    tables={}
    for k,body in modules.items():
        table={}
        for n,p,v in PARAM.findall(body):
            row=table.setdefault(n.strip().lstrip('\\'),{})
            assert p not in row, ('duplicate source parameter',k,n,p)
            row[p]=v.strip()
        tables[k]=table
    result={}
    def walk(kind,parent):
        for child,local in children[kind]:
            name=(parent+'/'+local).strip('/')
            if child in modules:walk(child,name)
            elif SELECT.fullmatch(child):
                assert name not in result
                result[name]=dict(name=name,kind=child,parameters=tables[kind].get(local,{}))
    walk('g2_top','')
    return result


def physical_rows(vo):
    text=vo.read_text(encoding='utf-8');result={}
    for kind,raw,body in PHYSICAL_CELL.findall(text):
        name=raw.lstrip('\\')
        assert not re.search(r'^defparam\s|^(?:DFF\w*|RAM16\w*|ROM16\w*)\s',body,re.M), ('absorbed following cell',kind,name)
        assert name not in result
        result[name]=dict(name=name,kind=kind,parameters={})
    headers=re.findall(r'^(RAM16\w*|ROM16\w*)\s+(\\\S+|[^\s(]+)\s*\(',text,re.M)
    assert len(headers)==len(result), ('physical header coverage',len(headers),len(result))
    for raw,p,v in PARAM.findall(text):
        name=raw.strip().lstrip('\\')
        if name in result:
            assert p not in result[name]['parameters'], ('duplicate physical parameter',name,p)
            result[name]['parameters'][p]=v.strip()
    return result


def library_geometry(kinds):
    modules=dict(MODULE.findall(LIB.read_text(encoding='utf-8')));result={}
    for kind in kinds:
        body=modules[kind]
        defaults=dict(re.findall(r'^parameter\s+(\w+)\s*=\s*(.*?);',body,re.M))
        match=re.search(r'^output\s+(?:\[(\d+):(\d+)\]\s+)?DO\s*;',body,re.M)
        assert match, ('vendor output width',kind)
        width=abs(int(match[1])-int(match[2]))+1 if match[1] else 1
        addr=re.findall(r'^input\s+\[(\d+):(\d+)\]\s+(?:AD|WAD|RAD)\s*;',body,re.M)
        assert addr and all(abs(int(a)-int(b))+1==4 for a,b in addr), ('vendor memory depth',kind,addr)
        assert set(defaults)=={f'INIT_{i}' for i in range(width)}, ('unexpected vendor parameters',kind,defaults)
        assert all(re.fullmatch(r"16'h[0-9a-fA-F]+",v) for v in defaults.values())
        result[kind]=dict(depth=16,width=width,capacity_bits=16*width,defaults=defaults,
                          vendor_INIT_semantics='INIT_i is the complete 16-address bitplane for DO[i]; address indexes that bitplane directly')
    return result


def review(a):
    vg=a.source/'impl/gwsynthesis/g2_top.vg';vo=a.physical/'impl/pnr/g2_top.vo';pv=a.physical/'g2_top.vg'
    before=[meta(p) for p in (vg,vo,pv,LIB)]
    old=json.loads(a.bsram_proof.read_text(encoding='utf-8-sig'))
    old_inputs={str(Path(x['path']).resolve()):x for x in old['inputs_before']}
    reused=old['passed'] and old['frozen_physical_input_VG_equals_SYN37'] and all(old_inputs.get(x['path'])==x for x in before)
    assert reused, 'Existing exact input-VG binding cannot be reused: an input metadata binding changed'
    source=source_rows(vg);physical=physical_rows(vo)
    geometry=library_geometry(sorted({x['kind'] for x in source.values()}|{x['kind'] for x in physical.values()}))
    names_equal=source.keys()==physical.keys();missing=sorted(source.keys()-physical.keys());added=sorted(physical.keys()-source.keys())
    rows={};diff=[];explicit_diff=[]
    for name in sorted(source.keys()&physical.keys()):
        s,p=source[name],physical[name]
        sd=dict(geometry[s['kind']]['defaults']);sd.update(s['parameters'])
        pd=dict(geometry[p['kind']]['defaults']);pd.update(p['parameters'])
        checks=dict(kind_equal=s['kind']==p['kind'],all_explicit_parameters_equal=s['parameters']==p['parameters'],
                    all_effective_parameters_equal={k:literal(v) for k,v in sd.items()}=={k:literal(v) for k,v in pd.items()},
                    geometry_equal=all(geometry[s['kind']][k]==geometry[p['kind']][k] for k in ('depth','width','capacity_bits')),
                    all_source_explicit_parameters_preserved=all(p['parameters'].get(k)==v for k,v in s['parameters'].items()),
                    physical_added_parameters_equal_vendor_defaults=all(k in geometry[p['kind']]['defaults'] and literal(v)==literal(geometry[p['kind']]['defaults'][k]) for k,v in p['parameters'].items() if k not in s['parameters']))
        added_parameters={k:v for k,v in p['parameters'].items() if k not in s['parameters']}
        row=dict(kind_source=s['kind'],kind_physical=p['kind'],source_parameters=s['parameters'],physical_parameters=p['parameters'],physical_added_default_parameters=added_parameters,checks=checks)
        rows[name]=row
        if not checks['all_explicit_parameters_equal']:explicit_diff.append(dict(name=name,**row))
        if not all(v for k,v in checks.items() if k!='all_explicit_parameters_equal'):diff.append(dict(name=name,**row))
    total_bits=sum(geometry[x['kind']]['capacity_bits'] for x in physical.values())
    sc=Counter(x['kind'] for x in source.values());pc=Counter(x['kind'] for x in physical.values())
    after=[meta(p) for p in (vg,vo,pv,LIB)]
    result=dict(passed=names_equal and not diff and before==after and reused,
                source=str(a.source),physical=str(a.physical),scope='All actual distributed RAM16/ROM16 instance names, kinds, geometry and every explicit/effective INIT parameter; no input-cone or whole-design equivalence claim',
                source_count=len(source),physical_count=len(physical),source_by_kind=dict(sc),physical_by_kind=dict(pc),
                exact_instance_names_equal=names_equal,missing_source_instances=missing,added_physical_instances=added,
                every_matched_kind_geometry_and_effective_parameter_equal=not diff,effective_parameter_or_kind_differences=diff,
                every_source_explicit_parameter_preserved=all(r['checks']['all_source_explicit_parameters_preserved'] for r in rows.values()),
                exact_explicit_parameter_text_equal=not explicit_diff,
                explicit_text_different_instances=len(explicit_diff),explicit_text_differences_by_kind=dict(Counter(r['kind_source'] for r in explicit_diff)),
                physical_added_default_INIT_parameters=sum(len(r['physical_added_default_parameters']) for r in rows.values()),
                explicit_text_difference_explanation='Source RAM16 leaves omit INIT; PNR emits the same vendor-default zero INIT explicitly. Each added value and every original explicit parameter was individually checked. Full exact differences are retained per_instance_checks.',
                source_explicit_INIT_parameter_count=sum(len(x['parameters']) for x in source.values()),
                physical_explicit_INIT_parameter_count=sum(len(x['parameters']) for x in physical.values()),
                source_nonzero_INIT_parameter_count=sum(literal(v)!=0 for x in source.values() for v in x['parameters'].values()),
                physical_nonzero_INIT_parameter_count=sum(literal(v)!=0 for x in physical.values() for v in x['parameters'].values()),
                vendor_geometry_and_defaults=geometry,total_distributed_capacity_bits=total_bits,
                reused_exact_PNR_input_VG_binding=dict(proof=str(a.bsram_proof),passed=reused,method='Prior bytewise binding plus unchanged size/mtime_ns of its exact SYN VG, PNR input VG, actual VO and vendor library'),
                inputs_before=before,inputs_unchanged_after_read_only_review=before==after,
                name_handling='Only Verilog escape marker removal; preserve all actual net/instance suffixes including _Z',
                parser_note='Accept same-line final-port/instance closure and independently count all selected physical headers',
                all_instance_rows='actual_distributed_parameters.json',all_input_port_logic_equivalence_verified=False,no_EDA_invoked=True,no_SHA_used=True)
    save(a.out/'actual_distributed_parameters.json',dict(source=source,physical=physical,per_instance_checks=rows))
    save(a.out/'result.json',result)
    return result


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--source',type=Path,default=BASE/'baseline_fusion_model25_bounded_MAX_syn37')
    ap.add_argument('--physical',type=Path,default=BASE/'baseline_fusion_model25_bounded_MAX_pnr50')
    ap.add_argument('--bsram-proof',type=Path,default=BASE/'PNR50_BSRAM_INIT_review1/result.json')
    ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    r=review(a)
    print(json.dumps({k:v for k,v in r.items() if k not in ('inputs_before','vendor_geometry_and_defaults','effective_parameter_or_kind_differences')},ensure_ascii=False))
    assert r['passed'],'See saved exact differences in result.json'


if __name__=='__main__':main()
