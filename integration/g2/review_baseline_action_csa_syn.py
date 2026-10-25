"""New actual SYN27 audit, retaining the proven RAM/INIT/clock/CDC engine."""
from pathlib import Path
from collections import Counter
import review_baseline_fusion_action_width_syn as engine
from prepare_baseline_action_csa import bind_contract


engine.bind_action_contract=bind_contract
original_snapshot=engine.snapshot
action_mapping={}


def snapshot(vg,directions):
    result=original_snapshot(vg,directions)
    cells=result[0]
    if Path(vg).resolve()!=(engine.BASELINE/'g2_top.vg').resolve():
        prefix='u_control/model_thread/actuator_map/'
        primitives=Counter(c['kind'] for name,c in cells.items() if name.startswith(prefix))
        dsps={name:c['kind'] for name,c in cells.items() if name.startswith(prefix) and
              any(word in c['kind'] for word in ('MULT','PADD','ALU54','MAC'))}
        assert primitives,'actual joint action module disappeared'
        assert not dsps,('actual joint mapping reintroduced action DSP',dsps)
        # A hierarchy-only count could miss arithmetic moved across a module
        # boundary. Walk the actual output-register D cones, cutting only at
        # real FF Qs/primary inputs/constants, and reject DSPs anywhere.
        drivers=result[3]
        outputs=[c for name,c in cells.items() if name.startswith(prefix) and
                 c['kind'].startswith('DFF') and any('/'+side+'_mmps_' in name for side in ('left','right'))]
        assert outputs and all(any('/'+side+'_mmps_' in c['name'] for c in outputs) for side in ('left','right'))
        seen=set();boundaries=set();cone_dsps={}
        pending=[net for c in outputs for net in c['ports']['D']]
        while pending:
            net=pending.pop()
            if net in seen or net in ('GND','VCC'):continue
            seen.add(net)
            sources=drivers.get(net,[])
            if not sources:
                boundaries.add('primary_or_literal:'+net);continue
            assert len(sources)==1,('multiple output-cone drivers',net,sources)
            name,pin=sources[0];cell=cells[name]
            if cell['kind'].startswith('DFF'):
                boundaries.add(name+'/'+pin);continue
            if any(word in cell['kind'] for word in ('MULT','PADD','ALU54','MAC','DSP')):
                cone_dsps[name]=cell['kind'];continue
            for port,nets in cell['ports'].items():
                if directions[cell['kind']][port] in ('input','inout'):
                    pending.extend(nets)
        assert not cone_dsps,('action wheel-output D cone still passes through DSP',cone_dsps)
        action_mapping.update(actual_prefix=prefix,primitive_counts=dict(primitives),DSP_cells=dsps,
            wheel_output_registers=[c['name'] for c in outputs],wheel_output_D_cone_nets=len(seen),
            wheel_output_state_or_primary_boundaries=sorted(boundaries),
            wheel_output_D_cone_DSP_cells=cone_dsps,
            target_action_DSP_removed_in_actual_joint_mapping=True)
    return result


engine.snapshot=snapshot
original_review=engine.review


def review(source,prepared,out):
    result=original_review(source,prepared,out)
    assert result['passed'] and action_mapping['target_action_DSP_removed_in_actual_joint_mapping']
    result.update(action_constant_CSA_actual_mapping=action_mapping,
        target_action_DSP_removed_in_actual_joint_mapping=True,
        action_CSA_source_numeric_cycle_and_module_mapping_bound=True)
    engine.save(out,result)
    return result


engine.review=review

if __name__=='__main__':
    raise SystemExit(engine.main())
