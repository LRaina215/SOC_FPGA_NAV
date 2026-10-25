"""Prove fresh PNR39 control objects before one paired global-role contrast."""
from datetime import datetime
from pathlib import Path
from collections import Counter
import json
import re
import psutil
from audit_pin_loads import PORTS
from audit_global_muxes import bits
from audit_netlist_memories import PARAM
from audit_routing_topology import primitive_directions
from run_baseline_single_factor_pnr import metadata

BASE=Path('F:/nav_integration_g2_20261023')
OLD=BASE/'baseline_fusion_only_pnr32'
NEW=BASE/'baseline_fusion_action_preview_pnr39'
OUT=BASE/'pnr39_final_actual_review1/observation_pair_request_preflight.json'
LIB=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
CE='u_control/model_thread/observation/n15679_10'
RESET='u_control/model_thread/observation/n331_7'
LOCALS=['u_nav/u_pf/n3967_6','u_nav/u_map/u_tv/n25284_7']
REQUESTS=[f'CLOCK_LOC "{CE}" BUFG = CE|LOGIC;',f'CLOCK_LOC "{RESET}" BUFG = SR|LOGIC;']+[f'CLOCK_LOC "{n}" LOCAL_CLOCK;'for n in LOCALS]

def graph(root):
    directions=primitive_directions(LIB);cells={};drivers={};current=None;lines=[]
    start=re.compile(r'^(?!module\b)(\w+)\s+(\\\S+|\w+)\s*\(\s*$')
    norm=lambda x:re.sub(r'\s+(?=\[)','',x.replace('\\','').strip())
    for line in(root/'impl/pnr/g2_top.vo').open():
        if current:
            lines.append(line)
            if not line.rstrip().endswith(');'):continue
            kind,raw=current;name=norm(raw)
            if kind.startswith('DFF')or re.fullmatch(r'LUT[1-6]|ALU|INV|BUF|BUFG|IBUF',kind):
                ports={p:bits(v)for p,v in PORTS.findall(''.join(lines))};cell=dict(name=name,kind=kind,ports=ports,parameters={});cells[name]=cell
                for pin,nets in ports.items():
                    if directions[kind][pin]=='output':
                        for index,net in enumerate(nets):
                            if net in('GND','VCC'):continue
                            assert net not in drivers;drivers[net]=(name,pin,index)
            current=None;lines=[];continue
        match=start.match(line)
        if match:current=match.groups();lines=[line];continue
        if line.startswith('defparam '):
            for raw,key,value in PARAM.findall(line):
                name=norm(raw)
                if name in cells:cells[name]['parameters'][key]=value.strip()
    return cells,drivers

def main():
    actual=json.loads((OUT.parent/'final_review.json').read_text());assert actual['passed']and actual['unrouted']==293
    inventory=json.loads((OUT.parent/'actual_storage_controls.json').read_text())
    alloc={r['net']:r['resource']for r in actual['global_allocations']['candidate']}
    assert CE not in alloc and RESET not in alloc and all(alloc[n]=='LW'for n in LOCALS)
    a,ad=graph(OLD);b,bd=graph(NEW);boundaries={};proofs=[]
    def expr(net,cells,drivers,seen=()):
        if net in('GND','VCC'):return ('literal',net)
        assert net not in seen and net in drivers,('unknown or cyclic boundary',net)
        name,pin,index=drivers[net];c=cells[name]
        if c['kind'].startswith('DFF'):
            assert name in a and name in b and a[name]==b[name],('FF input/control/INIT changed',name)
            boundaries[name]=c
            return ('same_actual_FF',name,pin,index)
        if c['kind']in('INV','BUF','BUFG'):
            return (c['kind'],expr(c['ports']['I'][0],cells,drivers,seen+(net,)))
        if c['kind']=='ALU':
            inputs=[(p,[expr(n,cells,drivers,seen+(net,))for n in c['ports'][p]])for p in sorted(c['ports'])if p not in('SUM','COUT')]
            return ('ALU',tuple(sorted(c['parameters'].items())),pin,index,inputs)
        assert re.fullmatch(r'LUT[1-6]',c['kind']),('non-state boundary',c)
        inputs=[expr(c['ports']['I'+str(i)][0],cells,drivers,seen+(net,))for i in range(int(c['kind'][3:]))]
        return (c['kind'],c['parameters']['INIT'],inputs)
    for net in[CE,RESET]+LOCALS:
        left,right=expr(net,a,ad),expr(net,b,bd);assert left==right,('actual logic cone differs',net)
        proofs.append(dict(net=net,passed=True,expression=right,actual_driver=b[bd[net][0]]))
    loads={n:inventory['selected_nets'][n]for n in[CE,RESET]+LOCALS}
    assert loads[CE]['loads']==4001 and loads[RESET]['loads']==4366
    assert loads[LOCALS[0]]['loads']==361 and loads[LOCALS[1]]['loads']==357
    assert set(loads[LOCALS[0]]['pins'])=={'CE'}and set(loads[LOCALS[1]]['pins'])=={'CE','I3'}
    result=dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),baseline=str(OLD),candidate=str(NEW),requested_CST=REQUESTS,
        single_factor='Prefer both original observation CE and reset global roles; free two new small LW controls, unchanged netlist/SDC/options.',
        distinction_from_PNR38='PNR39 lost BOTH4001CE and4366reset LW roles. PNR36/38 retained resetLW and tested only the CE role; source here isSYN28 productOREG, not failedSYN26 square2.',
        actual_four_cone_checks=proofs,actual_FF_boundary_cells=boundaries,all_boundary_D_CLK_CE_reset_INIT_byte_identical=True,
        selected_actual_loads=loads,only_known_clock_root_for_boundary=sorted({x['ports']['CLK'][0]for x in boundaries.values()}),
        no_numeric_aliases=True,no_RTL_VG_clock_or_timing_exception_change=True,
        actual_LW_assignment_and_routing_not_guaranteed=True,failed_net_identities_unknown=True,new_build_started=False,
        official_source='SUG1541-1.0E section2.8; genericBUFG SR/CE/LOGIC and LOCAL_CLOCK. BUFS is not supported on this AroraV device.',
        evidence_bindings=[metadata(p)for p in(OLD/'impl/pnr/g2_top.vo',NEW/'impl/pnr/g2_top.vo',OUT.parent/'final_review.json',OUT.parent/'actual_storage_controls.json')])
    OUT.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(dict(passed=True,cones=len(proofs),FFs=len(boundaries),requests=REQUESTS)))

if __name__=='__main__':
    me=psutil.Process();me.nice(psutil.IDLE_PRIORITY_CLASS);me.cpu_affinity([16]);main()
