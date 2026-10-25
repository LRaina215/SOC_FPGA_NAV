"""Inspect the real SYN28 preview-to-DSP register boundary before PNR."""
from collections import Counter,defaultdict
from datetime import datetime
import json
from pathlib import Path
import re
import psutil
from audit_routing_topology import primitive_directions
from revalidate_lidar_cdc import definitions
from review_small_block_stores import parameter_reader
from synthesis_connectivity import read
from prepare_baseline_fusion_retime import metadata

BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'baseline_fusion_action_preview_syn28'
CONTROL=BASE/'baseline_fusion_action_preview_one_shot'
PREFIX='u_control/model_thread/'
MAP=PREFIX+'actuator_map/'
CORE=PREFIX+'policy/core/'
LIB=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')

def review():
    prior=json.loads((CONTROL/'actual_SYN28_review.json').read_text());assert prior['passed']
    for row in prior['reviewed_input_metadata']:assert metadata(Path(row['path']))==row
    vg=SOURCE/'impl/gwsynthesis/g2_top.vg';directions=primitive_directions(LIB)
    cells=read(vg,keep=lambda n:PREFIX.startswith(n+'/')or n.startswith(PREFIX))
    params=parameter_reader(definitions(vg));drivers=defaultdict(list)
    for name,c in cells.items():
        for pin,nets in c['ports'].items():
            if directions[c['kind']][pin]=='output':
                for net in nets:
                    if net not in('GND','VCC'):drivers[net].append((name,pin))
    def roots(nets):
        pending=list(nets);seen=set();boundaries=set()
        while pending:
            net=pending.pop()
            if net in seen or net in('GND','VCC'):continue
            seen.add(net);sources=drivers.get(net,[]);assert len(sources)==1,(net,sources)
            name,pin=sources[0];c=cells[name]
            if c['kind'].startswith('DFF'):
                boundaries.add(name);continue
            assert c['kind']in('LUT1','LUT2','LUT3','LUT4','ALU','INV','BUF'),(name,c['kind'])
            for p,values in c['ports'].items():
                if directions[c['kind']][p]=='input':pending.extend(values)
        return sorted(boundaries)
    dsps={n:c for n,c in cells.items()if n.startswith(MAP)and c['kind'].startswith('MULT')}
    assert len(dsps)==6 and all(c['kind']=='MULTALU27X18'for c in dsps.values())
    reports=[];groups=[]
    for name,c in dsps.items():
        p=params(c)
        assert p['OREG_CLK']=='"CLK0"'and p['OREG_CE']=='"CE0"'
        assert p['AREG_CLK']==p['BREG_CLK']==p['DREG_CLK']==p['PREG_CLK']=='"BYPASS"'
        # Port vectors are MSB first: CLK0/CE0 are the final element.
        assert c['ports']['CLK'][-1]=='clk50_d'and c['ports']['CE'][-1]=='VCC'
        assert set(c['ports']['RESET'])=={'GND'}
        data=[net for pin,values in c['ports'].items()if pin in('A','B','C','D')for net in values]
        r=roots(data);assert r
        indices={re.match(re.escape(CORE)+r'action([0-5])_',n)[1]for n in r if re.match(re.escape(CORE)+r'action([0-5])_',n)}
        assert len(indices)==1 and all(n.startswith(CORE+'action'+next(iter(indices))+'_')for n in r),(name,r)
        assert all(cells[n]['ports']['CLK']==['clk50_d']for n in r)
        groups.extend(indices)
        reports.append(dict(name=name,kind=c['kind'],ports=c['ports'],parameters=p,actual_input_FF_boundaries=r,core_action_index=next(iter(indices)),registered_multiplier_output=True,registered_policy_actions_not_used_as_preview=True))
    assert sorted(groups)==list('012345')
    product_FFs=[n for n,c in cells.items()if n.startswith(MAP+'product_')and c['kind'].startswith('DFF')]
    assert not product_FFs,'unexpected separately mapped product FFs require a fresh review'
    result=dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),source=str(SOURCE),scope='Actual complete-SYN private preview roots and DSP OREG boundary, not physical timing or full sequential equivalence',
        action_DSP_count=6,actual_products_absorbed_in_DSP_output_registers=True,all_six_original_core_actions_are_actual_sources=True,clock='clk50_d',CE_constant_1=True,RESET_constant_0=True,
        additional_explicit_product_DFFs=0,action_primitive_counts=dict(Counter(c['kind']for n,c in cells.items()if n.startswith(MAP))),DSPs=reports,
        physical_RAM_or50MHz_timing_not_proved=True,evidence_bindings=[metadata(vg),metadata(CONTROL/'actual_SYN28_review.json'),metadata(SOURCE/'control/policy_action_map.v'),metadata(SOURCE/'control/rl_policy_tick_v5.v'),metadata(SOURCE/'control/mem_policy.v')])
    target=CONTROL/'actual_preview_product_mapping.json';target.write_text(json.dumps(result,indent=2)+'\n')
    return result

if __name__=='__main__':
    me=psutil.Process();me.nice(psutil.IDLE_PRIORITY_CLASS);me.cpu_affinity([16])
    r=review();print(json.dumps({k:r[k]for k in('passed','action_DSP_count','all_six_original_core_actions_are_actual_sources','additional_explicit_product_DFFs')}))
