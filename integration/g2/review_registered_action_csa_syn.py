"""Full RAM/INIT/clock/26CDC review plus real private-product FF boundaries."""
from collections import Counter
from pathlib import Path
import re
import review_baseline_action_preview_syn as preview

BASE=Path('F:/nav_integration_g2_20261023')
preview.CANDIDATE=BASE/'action_registered_CSA_candidate1'
engine=preview.engine
original_snapshot=engine.snapshot
original_review=engine.review
actual_mapping={}
PREFIX='u_control/model_thread/actuator_map/'
CORE='u_control/model_thread/policy/core/'
GROUPS=dict(lh='0',lk='1',lw='2',rh='3',rk='4',rw='5')

def bind(source,prepared,proof):
    preview.bind(source,prepared,proof)
    assert proof['registered_product_arithmetic_only_vs_SYN28']
    for root in(prepared,source):
        manifest=engine.load(root/'manifest.json')
        assert manifest['preview_constant_CSA']and manifest['registered_product_arithmetic_only_vs_SYN28']
    proof0=engine.load(preview.CANDIDATE/'prepared_review.json')
    assert proof0['private_register_capture_edges_and_64bit_products_unchanged']
    assert proof0['source_SINGLE_delta_vs_SYN28']=='control/policy_action_map.v'
    for row in proof0['source_bindings']+proof0['candidate_bindings']:assert engine.metadata(Path(row['path']))==row

def snapshot(vg,directions):
    result=original_snapshot(vg,directions)
    if Path(vg).resolve()==(engine.BASELINE/'g2_top.vg').resolve():return result
    cells,params,memory,drivers,clock,inventory=result
    map_cells={n:c for n,c in cells.items()if n.startswith(PREFIX)}
    assert map_cells
    dsps={n:c['kind']for n,c in map_cells.items()if any(w in c['kind']for w in('MULT','PADD','ALU54','MAC','DSP'))}
    assert not dsps,('registered constants still mapped to DSP',dsps)
    def roots(nets):
        pending=list(nets);seen=set();boundaries=set()
        while pending:
            net=pending.pop()
            if net in seen or net in('GND','VCC'):continue
            seen.add(net);d=drivers.get(net,[]);assert len(d)==1,(net,d)
            n,pin=d[0];c=cells[n]
            if c['kind'].startswith('DFF'):boundaries.add(n);continue
            assert c['kind']in('LUT1','LUT2','LUT3','LUT4','ALU','INV','BUF'),(n,c['kind'])
            for p,values in c['ports'].items():
                if directions[c['kind']][p]=='input':pending.extend(values)
        return sorted(boundaries)
    groups={}
    synchronous_data_resets=[]
    for group,index in GROUPS.items():
        ffs={n:c for n,c in map_cells.items()if c['kind'].startswith('DFF')and n.startswith(PREFIX+'product_'+group+'_')}
        assert ffs,('private registered product absent',group)
        for n,c in ffs.items():
            assert c['ports']['CLK']==['clk50_d'],(n,c)
            for pin,values in c['ports'].items():
                if pin in('CE',):assert set(values)=={'VCC'},(n,pin,values)
                if pin in('CLEAR','PRESET','SET'):assert set(values)=={'GND'},(n,pin,values)
                if pin=='RESET'and set(values)!={'GND'}:
                    # The actual vendor DFFRE is synchronous: if RESET then0,
                    # else if CE thenD, only on posedge CLK. Its data-zeroing
                    # input is an implementation of the combinational product,
                    # not a system reset. Prove its exact roots as well as D.
                    assert c['kind']=='DFFRE',(n,c['kind'],pin,values)
                    reset_roots=roots(values)
                    assert reset_roots and all(s.startswith(CORE+'action'+index+'_')for s in reset_roots),(n,values,reset_roots)
                    synchronous_data_resets.append(dict(register=n,kind=c['kind'],pin=pin,nets=values,actual_input_FF_boundaries=reset_roots))
        r=roots(net for c in ffs.values()for pin in('D','RESET')for net in c['ports'].get(pin,[]))
        assert r and all(n.startswith(CORE+'action'+index+'_')for n in r),(group,r)
        groups[group]=dict(registers=sorted(ffs),input_actual_FF_boundaries=r,core_action_index=index)
    actual_mapping.update(actual_product_groups=groups,actual_action_DSP_cells=dsps,
        actual_primitive_counts=dict(Counter(c['kind']for c in map_cells.values())),
        actual_product_FF_count=sum(len(g['registers'])for g in groups.values()),
        synchronous_data_zeroing_resets=synchronous_data_resets,
        product_async_controls_all_constant_zero=True,product_system_reset_loads=0,
        private_products_clocked_on_original_core_and_come_from_same_core_actions=True,
        source384bit_count_not_used_as_mapping_count=True,physical_timing_not_proven=True)
    return result

def review(source,prepared,out):
    r=original_review(source,prepared,out)
    assert r['passed']and actual_mapping['actual_product_FF_count']>0
    r['registered_constant_CSA_actual_mapping']=actual_mapping
    engine.save(out,r);return r

engine.bind_action_contract=bind
engine.snapshot=snapshot
engine.review=review
if __name__=='__main__':raise SystemExit(engine.main())
