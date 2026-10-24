"""Apply a measured, explicit PRIMARY allocation without changing logic.

The referenced completed physical run supplies load evidence. The new frozen
SYN must still contain each requested net. Four real clocks remain automatic;
PRIMARY4..7 are reserved for the large control reset/snapshot networks.
"""
import json
from pathlib import Path
from audit_global_muxes import audit
from synthesis_connectivity import read


def apply(source, out, plan_file):
    source,out=Path(source),Path(out)
    plan=json.loads(Path(plan_file).read_text(encoding='utf-8'))
    requested=[r['net'] for r in plan['promote']]+[r['net'] for r in plan.get('prefer_global',[])]+plan['local']
    assert len(requested)==len(set(requested)), 'duplicate routing target'
    aliases=plan.get('reference_net_aliases',{})
    driver_proof=None
    if plan.get('point_planes_driver_revalidation'):
        from revalidate_point_planes_routing import validate
        driver_proof=validate(source,plan)
    elif aliases:
        from revalidate_observation_routing import validate
        driver_proof=validate(source,plan)
    reference_name=lambda n:aliases.get(n,n)
    measured=audit(Path(plan['reference_physical_build']),[reference_name(n) for n in requested])
    previous={r['net']:r for r in measured['global_nets']}
    for name,prior in aliases.items():previous[name]=previous[prior]
    cells=read(out/'g2_top.vg')
    nets={net for cell in cells.values() for values in cell['ports'].values() for net in values}
    cst=out/'g2.cst';original=cst.read_text(encoding='utf-8')
    assert 'CLOCK_LOC' not in original,'do not silently replace existing clock routing constraints'
    rows=[];commands=[];seen=set();indices=set()
    for item in plan['promote']:
        name,index=item['net'],item['primary_index']
        assert name in nets and name in previous,(name,'net absent from frozen source or physical reference')
        assert name not in seen and index in range(4,8) and index not in indices
        row=previous[name];pins=row['direct_pin_counts']
        assert sum(pins.values())>=3000,(name,'promotion lacks large measured load')
        assert set(pins).isdisjoint({'CLK','CLKA','CLKB','CLOCK'}),'not a reviewed control network'
        types=item['signal_types'].split('|')
        assert set(types)<= {'SR','CE','LOGIC'}
        commands.append(f'CLOCK_LOC "{name}" BUFG[{index}] = {item["signal_types"]};')
        rows.append(dict(action='promote',net=name,primary_index=index,reference=row))
        seen.add(name);indices.add(index)
    assert indices=={4,5,6,7},'reserve four control tracks, leaving four PRIMARY tracks for true clocks'
    for item in plan.get('prefer_global',[]):
        name=item['net']
        assert name in nets and name in previous and name not in seen
        row=previous[name];pins=row['direct_pin_counts']
        assert sum(pins.values())>=800,(name,'global request lacks large measured load')
        assert set(pins).isdisjoint({'CLK','CLKA','CLKB','CLOCK'})
        assert set(item['signal_types'].split('|')) <= {'SR','CE','LOGIC'}
        if item['signal_types']=='CE':assert set(pins)=={'CE'}, 'CE request includes non-CE consumers'
        if item['signal_types']=='LOGIC':assert all(p.startswith('I') for p in pins), 'data request includes state control consumers'
        # BUFS is not supported on Arora V. Generic BUFG is documented and
        # the small CE probe mapped it to LW. Actual LW use must be reported.
        commands.append(f'CLOCK_LOC "{name}" BUFG = {item["signal_types"]};')
        rows.append(dict(action='prefer_global',net=name,expected_resource='LW',reference=row))
        seen.add(name)
    for name in plan['local']:
        assert name in nets and name in previous and name not in seen
        row=previous[name];pins=row['direct_pin_counts']
        assert 0<sum(pins.values())<500,(name,'local reroute requires a bounded measured load')
        assert set(pins).isdisjoint({'CLK','CLKA','CLKB','CLOCK','RESET','SET','CLEAR','PRESET'})
        commands.append(f'CLOCK_LOC "{name}" LOCAL_CLOCK;')
        rows.append(dict(action='local',net=name,reference=row));seen.add(name)
    fragment='\n// Explicit measured control routing plan; original IO constraints preserved.\n'+'\n'.join(commands)+'\n'
    cst.write_text(original+fragment,encoding='utf-8',newline='\n')
    result=dict(plan=plan,source=str(source),actions=rows,cst_fragment=fragment,
        mapped_driver_proof=driver_proof,
        logic_unchanged=True,clock_periods_unchanged=True,
        real_primary_clocks=['clk50_d','rgmii_rxc_d','h_rx_clk','h_tx_clk'],
        scope='directed physical routing candidate; no timing, routability, full CDC or hardware acceptance claimed')
    (out/'clock_resource_plan.json').write_text(json.dumps(result,indent=2),encoding='utf-8',newline='\n')
    return result
