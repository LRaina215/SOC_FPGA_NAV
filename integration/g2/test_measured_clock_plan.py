"""Exercise routing safety checks with the actual correct-Lane2 frozen netlist.

No vendor build, source mutation or timing acceptance. Negative cases ensure
the new data/CE requests and LOCAL targets cannot silently change signal roles.
"""
import argparse
import copy
import json
import os
from pathlib import Path

from prepare_clock_resource_plan import apply


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    root=Path(__file__).resolve().parent
    source=Path('F:/nav_integration_g2_20261023/sfp_lane2_rowbanks_syn7')
    plan=json.loads((root/'clock_resource_plan_lane2_telemetry.json').read_text())
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    cases=[('actual_measured_plan',plan,True)]
    wrong=copy.deepcopy(plan);wrong['prefer_global'][0]['signal_types']='CE'
    cases.append(('reject_data_net_as_ce',wrong,False))
    wrong=copy.deepcopy(plan);wrong['prefer_global'][1]['signal_types']='LOGIC'
    cases.append(('reject_ce_net_as_data',wrong,False))
    wrong=copy.deepcopy(plan);wrong['local'][-1]='u_nav/g_support.u_support/a1_13_6'
    cases.append(('reject_local_net_with_reset_sinks',wrong,False))
    wrong=copy.deepcopy(plan);wrong['local'].append(wrong['prefer_global'][0]['net'])
    cases.append(('reject_duplicate_target',wrong,False))
    rows=[]
    for name,config,expected in cases:
        folder=out/name;folder.mkdir()
        # apply only reads the netlist. A hard link keeps the test from making
        # multiple large copies while the original frozen file stays untouched.
        os.link(source/'impl/gwsynthesis/g2_top.vg',folder/'g2_top.vg')
        (folder/'g2.cst').write_bytes((source/'g2.cst').read_bytes())
        config_file=folder/'plan.json';config_file.write_text(json.dumps(config,indent=2))
        try:
            result=apply(source,folder,config_file);accepted=True;error=None
        except AssertionError as exc:
            accepted=False;error=str(exc)
        assert accepted==expected,(name,accepted,error)
        rows.append(dict(case=name,accepted=accepted,expected_accept=expected,error=error))
        if accepted:
            targets={r['net']:r for r in result['actions']}
            data=targets['u_control/telemetry_thread/n1315_4']['reference']
            enable=targets['u_control/telemetry_thread/snapshot_959_10']['reference']
            assert data['resource'] is None and sum(data['direct_pin_counts'].values())==962
            assert enable['resource'] is None and enable['direct_pin_counts']=={'CE':960}
            assert len(result['actions'])==12
    result=dict(passed=True,cases=rows,source=str(source),programmed=False,
                scope='actual measured clock-plan generation safety; physical allocation/routing/timing still require PNR')
    (out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result))


if __name__=='__main__':main()
