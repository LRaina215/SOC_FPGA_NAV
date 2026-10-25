"""Full RAM/INIT/clock/CDC review and strict actual private DSP register audit."""
from collections import Counter
from pathlib import Path
import re
import prepare_registered_dsp_round_bias as bias
import review_baseline_action_preview_syn as preview

preview.CANDIDATE = bias.BASE/'action_DSP_round_bias_candidate1'
engine = preview.engine
original_snapshot = engine.snapshot
original_review = engine.review
mapping = {}
MAP = 'u_control/model_thread/actuator_map/'
CORE = 'u_control/model_thread/policy/core/'


def bind(source, prepared, proof):
    preview.bind(source, prepared, proof)
    assert proof['round_bias_only_vs_SYN28']
    candidate = preview.CANDIDATE
    origin = engine.load(candidate/'prepared_review.json')
    assert origin['source_single_delta_vs_SYN28'] == bias.ACTION
    assert origin['original_signed64_product_domain_and_E2_E3_edges_preserved']
    for row in origin['source_bindings']+origin['candidate_bindings']:
        assert engine.metadata(Path(row['path'])) == row
    assert (source/bias.ACTION).read_text() == bias.transform((bias.SOURCE/bias.ACTION).read_text())
    for name in preview.CHANGED:
        if name != bias.ACTION:
            assert engine.same(source/name, bias.SOURCE/name)
    math = engine.load(candidate/'math_proof.json')
    assert math['passed'] and math['combinations'] == 32772
    assert math['cases'] == bias.math_proof()['cases']
    for row in (math['source_binding'], math['candidate_binding']):
        assert engine.metadata(Path(row['path'])) == row
    for root in (source, prepared):
        manifest = engine.load(root/'manifest.json')
        assert manifest['private_DSP_round_bias_capture'] and manifest['round_bias_only_vs_SYN28']


def snapshot(vg, directions):
    result = original_snapshot(vg, directions)
    if Path(vg).resolve() == (engine.BASELINE/'g2_top.vg').resolve():
        return result
    cells, params, memory, drivers, clock, inventory = result
    def roots(nets):
        pending=list(nets);seen=set();boundary=set()
        while pending:
            net=pending.pop()
            if net in seen or net in ('GND','VCC'):continue
            seen.add(net);d=drivers.get(net,[]);assert len(d)==1,(net,d)
            name,pin=d[0];c=cells[name]
            if c['kind'].startswith('DFF'):boundary.add(name);continue
            assert c['kind']in('LUT1','LUT2','LUT3','LUT4','ALU','INV','BUF'),(name,c['kind'])
            pending.extend(n for p,values in c['ports'].items() if directions[c['kind']][p]=='input' for n in values)
        return sorted(boundary)
    dsps={n:c for n,c in cells.items()if n.startswith(MAP)and c['kind'].startswith('MULT')}
    assert len(dsps)==6 and all(c['kind']=='MULTALU27X18'for c in dsps.values()),('private multiply/add did not map to six DSPs',[(n,c['kind'])for n,c in dsps.items()])
    reports=[];groups=[]
    for name,c in dsps.items():
        p=params(c)
        assert p['OREG_CLK']=='"CLK0"'and p['OREG_CE']=='"CE0"',(name,p)
        assert all(p[k]=='"BYPASS"'for k in('AREG_CLK','BREG_CLK','DREG_CLK','PREG_CLK')),(name,p)
        assert c['ports']['CLK'][-1]=='clk50_d'and c['ports']['CE'][-1]=='VCC'
        assert set(c['ports']['RESET'])=={'GND'}
        data=[net for pin,values in c['ports'].items()if pin in('A','B','C','D')for net in values]
        r=roots(data);indices={re.fullmatch(re.escape(CORE)+r'action([0-5])_.*',n)[1]for n in r if re.fullmatch(re.escape(CORE)+r'action([0-5])_.*',n)}
        assert len(indices)==1 and all(n.startswith(CORE+'action'+next(iter(indices))+'_')for n in r),(name,r)
        assert all(cells[n]['ports']['CLK']==['clk50_d']for n in r)
        groups.extend(indices)
        reports.append(dict(name=name,kind=c['kind'],ports=c['ports'],parameters=p,input_actual_FF_boundaries=r,core_action_index=next(iter(indices))))
    assert sorted(groups)==list('012345')
    ffs=[n for n,c in cells.items()if n.startswith(MAP+'product_')and c['kind'].startswith('DFF')]
    assert not ffs,('separate product FFs require a different pipeline review',ffs)
    mapping.update(action_DSPs=reports,action_DSP_count=6,private_product_explicit_FF_count=0,
                   OREG_core_clock_CE1_reset0_and_all_other_registers_bypass=True,
                   all_six_original_core_actions_are_sources=True,
                   actual_action_primitive_counts=dict(Counter(c['kind']for n,c in cells.items()if n.startswith(MAP))),
                   exact_DSP_ALU_mode_and_bias_ports_require_root_review=True,
                   physical_timing_not_proven=True)
    return result


def review(source, prepared, out):
    result=original_review(source,prepared,out)
    assert result['passed']and mapping['action_DSP_count']==6
    result['private_DSP_round_bias_actual_mapping']=mapping
    engine.save(out,result)
    return result


engine.bind_action_contract=bind
engine.snapshot=snapshot
engine.review=review
if __name__=='__main__':
    raise SystemExit(engine.main())
