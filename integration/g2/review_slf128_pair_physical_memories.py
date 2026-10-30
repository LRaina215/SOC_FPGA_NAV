"""Read-only SYN43 -> PNR53/54 complete BSRAM/INIT and split audit.

Reuses the previously exercised memory-only parsers. It builds no full graph,
starts no EDA, and does not accept routing or timing from a resource count.
"""
import argparse
from collections import Counter
from datetime import datetime
from pathlib import Path
import json,re
from review_pnr50_bsram_init import extract,effective,init_vector,metadata,save,LIB
from review_baseline_single_factor_syn import cpu_image
from review_pnr50_distributed_init import source_rows,physical_rows,library_geometry,literal

BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'SLF128_storage_safeFW_syn43'


def distributed_review(source,physical,out):
    """Same proven narrow parser, with a new explicit SYN43 metadata binding."""
    paths=[source/'impl/gwsynthesis/g2_top.vg',physical/'g2_top.vg',
           physical/'impl/pnr/g2_top.vo',LIB]
    before=[metadata(p) for p in paths]
    exact_input=paths[0].read_bytes()==paths[1].read_bytes()
    syn,phy=source_rows(paths[0]),physical_rows(paths[2])
    geom=library_geometry(sorted({c['kind'] for c in syn.values()}|{c['kind'] for c in phy.values()}))
    rows=[]
    for name in sorted(syn.keys()&phy.keys()):
        s,p=syn[name],phy[name]
        sd=dict(geom[s['kind']]['defaults']);sd.update(s['parameters'])
        pd=dict(geom[p['kind']]['defaults']);pd.update(p['parameters'])
        checks=dict(kind_equal=s['kind']==p['kind'],
            complete_effective_INIT_equal={k:literal(v) for k,v in sd.items()}=={k:literal(v) for k,v in pd.items()},
            geometry_equal=all(geom[s['kind']][k]==geom[p['kind']][k] for k in ('depth','width','capacity_bits')),
            original_explicit_parameters_preserved=all(p['parameters'].get(k)==v for k,v in s['parameters'].items()),
            added_parameters_equal_unmodified_vendor_defaults=all(k in geom[p['kind']]['defaults'] and literal(v)==literal(geom[p['kind']]['defaults'][k]) for k,v in p['parameters'].items() if k not in s['parameters']))
        rows.append(dict(name=name,source=s,physical=p,checks=checks))
    unchanged=before==[metadata(p) for p in paths]
    result=dict(passed=exact_input and syn.keys()==phy.keys() and unchanged and all(all(r['checks'].values()) for r in rows),
        scope='All RAM16/ROM16 names/kinds/geometry/effective INIT only; no caller or input-port equivalence claim',
        physical_input_byte_equal_SYN43=exact_input,source_count=len(syn),physical_count=len(phy),
        missing_source_instances=sorted(syn.keys()-phy.keys()),added_physical_instances=sorted(phy.keys()-syn.keys()),
        source_by_kind=dict(Counter(c['kind'] for c in syn.values())),physical_by_kind=dict(Counter(c['kind'] for c in phy.values())),
        all_instance_checks=rows,inputs=before,inputs_unchanged_after_review=unchanged)
    save(out/'distributed_INIT_result.json',result)
    return result


def review(source,physical,out,expected_source_count=269,expected_physical_count=280,expected_BIN_bytes=53572):
    runtime=json.loads((physical/'tool_runtime.json').read_text())
    assert runtime.get('ended_at') and runtime.get('tool_exit') is not None
    assert runtime['cancelled'] is False,'Natural completed result required'
    inputs=[source/'impl/gwsynthesis/g2_top.vg',physical/'g2_top.vg',
            physical/'impl/pnr/g2_top.vo',physical/'impl/pnr/g2_top.rpt.txt',
            physical/'impl/pnr/cmd.do',physical/'tool_runtime.json',
            source/'firmware/nav_fw.bin',source/'ram_w.hex',LIB]
    inputs += [source/f'ram_b{i}.hex' for i in range(4)]
    before=[metadata(p) for p in inputs]
    syn,phys,defaults=extract(source,physical)
    assert len(syn)==expected_source_count,('bound source count differs',len(syn),expected_source_count)
    assert set(syn)<=set(phys),('Source leaf missing',sorted(set(syn)-set(phys)))
    rows=[];used=set();split_count=0
    for name,s in syn.items():
        p=phys[name];used.add(name)
        assert s['kind']==p['kind'],('Kind changed',name,s['kind'],p['kind'])
        sp,pp=effective(s,defaults),effective(p,defaults)
        widths=[k for k in ('BIT_WIDTH','BIT_WIDTH_0','BIT_WIDTH_1') if k in s['parameters']]
        assert widths,('No memory geometry',name)
        width=max(sp[k] for k in widths)
        source_init,capacity=init_vector(s,defaults)
        if width not in (32,36):
            checks=dict(all_effective_parameters_equal=sp==pp,
                        every_effective_INIT_bit_equal=init_vector(p,defaults)[0]==source_init)
            rows.append(dict(source=name,kind=s['kind'],wide_split=False,checks=checks))
            continue
        split_count+=1
        parent,leaf=name.rsplit('/',1)
        candidates=[parent+'/gowin_add_'+leaf,parent+'/gowin_add_'+s['kind']+'_'+leaf]
        found=[n for n in candidates if n in phys]
        assert len(found)==1,('No unique physical half',name,found)
        added=found[0];q=phys[added];used.add(added)
        assert q['kind']==s['kind'],('Split kind changed',added)
        qp=effective(q,defaults);half=width//2
        checks=dict(half_geometry=all(pp[k]==qp[k]==sp[k]//2 for k in widths),
            all_other_effective_parameters_equal=all(sp[k]==pp[k]==qp[k] for k in sp if k not in widths))
        pins=('AD',) if s['kind'].startswith('pROM') else ('ADA','ADB')
        checks['address_high_bits_equal']=all(s['ports'][k][:-5]==p['ports'][k][:-5]==q['ports'][k][:-5] for k in pins)
        checks['even_odd_address_select']=all(p['ports'][k][-5]=='GND' and q['ports'][k][-5]=='VCC' for k in pins)
        checks['DO_halves_reassembled']=p['ports']['DO'][-half:]==s['ports']['DO'][-half:] and q['ports']['DO'][-half:]==s['ports']['DO'][:half]
        if 'DI' in s['ports']:
            checks['DI_halves_reassembled']=p['ports']['DI'][-half:]==s['ports']['DI'][-half:] and q['ports']['DI'][-half:]==s['ports']['DI'][:half]
            checks['write_byte_enables_preserved']=p['ports']['ADA'][-2:]==s['ports']['ADA'][-2:] and q['ports']['ADA'][-2:]==s['ports']['ADA'][-4:-2]
        control=[k for k in s['ports'] if k not in ('DI','DO','AD','ADA','ADB')]
        checks['actual_clock_enable_reset_block_select_equal']=all(s['ports'][k]==p['ports'][k]==q['ports'][k] for k in control)
        lo,hi=init_vector(p,defaults)[0],init_vector(q,defaults)[0]
        hm=(1<<half)-1;wm=(1<<width)-1;differences=[]
        for address in range(capacity//width):
            expected=(source_init>>(address*width))&wm
            actual=((lo>>(2*address*half))&hm)|(((hi>>((2*address+1)*half))&hm)<<half)
            if expected!=actual: differences.append(dict(address=address,source=expected,physical=actual))
        checks['all_initialized_words_reconstructed']=not differences
        rows.append(dict(source=name,added=added,kind=s['kind'],wide_split=True,
            logical_width=width,physical_width_each=half,all_words_checked=capacity//width,
            checks=checks,INIT_mismatches=differences))
    assert used==set(phys),('Unbound physical leaf',sorted(set(phys)-used))
    assert split_count==11,('Unexpected wide leaf count',split_count)
    initdefaults={k:v for k,v in defaults['SP'].items() if k.startswith('INIT_RAM_')}
    synimage,phyimage=cpu_image(syn,initdefaults),cpu_image(phys,initdefaults)
    binary=(source/'firmware/nav_fw.bin').read_bytes()
    wordimage=b''.join(int(x,16).to_bytes(4,'little') for x in (source/'ram_w.hex').read_text().split())
    lanes=[bytes(int(x,16) for x in (source/f'ram_b{i}.hex').read_text().split()) for i in range(4)]
    laneimage=bytes(lanes[i%4][i//4] for i in range(131072))
    cpucount=sum(n.startswith('u_soc/g_lane[') for n in phys)
    cpu=dict(physical_bitplanes=cpucount,image_bytes=len(phyimage),BIN_bytes=len(binary),
        checks=dict(bitplanes64=cpucount==64,source_equals_physical=synimage==phyimage,
            selected_BIN_length=len(binary)==expected_BIN_bytes,
            BIN_and_zero_tail=phyimage==binary+bytes(131072-len(binary)),
            all_word_hex_equal=phyimage==wordimage,all_lane_hex_equal=phyimage==laneimage))
    report=(physical/'impl/pnr/g2_top.rpt.txt').read_text()
    count=int(re.search(r'^\s*BSRAM\s*\|\s*(\d+)/',report,re.M)[1])
    binding=(physical/'g2_top.vg').read_bytes()==(source/'impl/gwsynthesis/g2_top.vg').read_bytes()
    split='-convert_sdp32_36_to_sdp16_18' in (physical/'impl/pnr/cmd.do').read_text()
    immutable=before==[metadata(p) for p in inputs]
    counts=(len(syn)+split_count==len(phys)==count==expected_physical_count)
    passed=binding and split and immutable and counts and all(all(x['checks'].values()) for x in rows) and all(cpu['checks'].values())
    dist=distributed_review(source,physical,out)
    passed=passed and dist['passed']
    result=dict(recorded_at=datetime.now().astimezone().isoformat(),passed=passed,source=str(source),physical=str(physical),
        scope='All physical BSRAM geometry/effective INIT, split address/data/control reconstruction and CPU image only; not complete RAM caller/CDC/timing/vehicle acceptance',
        physical_input_VG_byte_equal_SYN43=binding,split_option_enabled=split,source_BSRAM=len(syn),
        split_pairs=split_count,actual_physical_BSRAM=len(phys),report_BSRAM=count,counts_match=counts,
        source_by_kind=dict(Counter(x['kind'] for x in syn.values())),
        physical_by_kind=dict(Counter(x['kind'] for x in phys.values())),
        all_rows=rows,CPU128KiB=cpu,inputs=before,inputs_unchanged_after_review=immutable,
        all_distributed_INIT_preserved=dist['passed'],distributed_memory_count=dist['physical_count'],
        distributed_memory_proof=str(out/'distributed_INIT_result.json'),actual_caller_port_aliases_not_relaxed=True,
        programmed=False,deployment_accepted=False)
    save(out/'actual_BSRAM_cells.json',dict(source=syn,physical=phys,defaults=defaults))
    save(out/'result.json',result)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,default=SOURCE)
    p.add_argument('--physical',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();a.out.mkdir(exist_ok=False)
    r=review(a.source,a.physical,a.out)
    print(json.dumps({k:v for k,v in r.items() if k not in ('all_rows','inputs')},ensure_ascii=False))
    assert r['passed'],'Read result.json for actual differences; do not bypass the gate'
