"""One local comparison without the spare global tracks absent in joint PNR15.

Reuse both measured netlists. Do not alter the active joint build or its inputs.
"""
import argparse
import json
from pathlib import Path
import re

import psutil

from inspect_physical_nets import inspect
from measure_row_read_context import Runner, TOOL, now, reference_snapshot, write
from review_row_read_context import review


def scalarize_addresses(source):
    """Rename only four local wires, with an exact inverse proof; no logic edit."""
    start=source.index('module slf_extract (')
    stop=source.index('\nendmodule',start)
    module=source[start:stop]
    declaration='wire [3:0] rb_ra;'
    assert module.count(declaration)==1 and module.count('rb_ra[3:0]')==1
    names=['diagnostic_read_address_'+str(i) for i in range(4)]
    assert not any(n in source for n in names)
    scalar_declaration='\n'.join('wire '+n+';' for n in names)
    concat='{'+', '.join(reversed(names))+'}'
    changed=module.replace(declaration,scalar_declaration).replace('rb_ra[3:0]',concat)
    for i,name in enumerate(names):
        assert changed.count('rb_ra['+str(i)+']')==1
        changed=changed.replace('rb_ra['+str(i)+']',name)
    assert not re.search(r'\brb_ra\[',changed)
    inverse=changed.replace(scalar_declaration,declaration).replace(concat,'rb_ra[3:0]')
    for i,name in enumerate(names):inverse=re.sub(r'\b'+name+r'\b','rb_ra['+str(i)+']',inverse)
    assert inverse==module, 'no primitive, parameter, port or connection may change'
    return source[:start]+changed+source[stop:], {'u_extract/rb_ra['+str(i)+']':'u_extract/'+n for i,n in enumerate(names)}


class LocalRoutes(Runner):
    def __init__(self, prior, out, joint_reference, bus_net_format='literal',scalar_addresses=False,explicit_logic_type=False):
        old=json.loads((prior/'result.json').read_text())
        comparison=json.loads((prior/'physical_review.json').read_text())
        assert old['phase']=='local_context_analysis_complete_requires_path_review'
        assert old['protected_joint_build_input_metadata_unchanged']
        assert not out.exists(), 'unique comparison directory required'
        globals_by_version={name:{row['net'] for row in comparison[name]['global_allocations'] if row['net']!='clk_d'}
                            for name in ('baseline','shared')}
        assert globals_by_version['baseline']==globals_by_version['shared']
        targets=sorted(globals_by_version['baseline'])
        assert len(targets)==7 and all(n.startswith('u_extract/') for n in targets)
        joint_report=(joint_reference/'impl/pnr/g2_top.rpt.txt').read_text()
        joint_globals=joint_report.split('6. Global Clock Signals',1)[1].split('7. Pinout',1)[0]
        joint_nets={line.split('|')[0].strip() for line in joint_globals.splitlines()
                    if '| PRIMARY' in line or '| LW ' in line}
        assert all(n.replace('u_extract/','u_nav/u_match/u_ex/',1) not in joint_nets for n in targets)
        me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
        me.cpu_affinity([max(me.cpu_affinity())])
        out.mkdir()
        self.out=out
        self.state=dict(created_at=now(),phase='prepared',prior=str(prior),results={},
            controller_pid=me.pid,controller_created_epoch=me.create_time(),assigned_cpu=me.cpu_affinity(),
            protected_joint_build=old['protected_joint_build'],joint_build_modified=False,
            full_joint_acceptance=False,programmed=False,source_Synthesis_reused=True,
            function_validation_reused=old['context_behavior'],execution_policy=old['execution_policy'],
            joint_reference=str(joint_reference),local_only_targets=targets,
            bus_net_format=bus_net_format,
            scalar_address_names_only=scalar_addresses,
            explicit_LOCAL_logic_type_diagnostic=explicit_logic_type,
            reason='Both earlier local reports used four spare PRIMARY and three LW tracks. These seven extractor nets are not global in actual joint PNR15.',
            scope='Conservative local routing-resource comparison, not reproduction of whole-chip placement or congestion.')
        for name in ('baseline','shared'):
            original=Path(comparison[name]['build'])
            endpoints=inspect(original,targets)
            assert all(len(n['drivers'])==1 for n in endpoints['nets'])
            folder=out/name;folder.mkdir()
            write(folder/'verified_target_connections.json',endpoints)
            cst=folder/'local_only.cst'
            names=targets
            tcl=(original/'physical.tcl').read_text()
            if scalar_addresses:
                source=prior/name/'impl/gwsynthesis/g2_top.vg'
                changed,mapping=scalarize_addresses(source.read_text(encoding='utf-8'))
                renamed=folder/'scalar_addresses.vg'
                renamed.write_text(changed,encoding='utf-8',newline='\n')
                assert tcl.count(source.as_posix())==1
                tcl=tcl.replace(source.as_posix(),renamed.as_posix())
                names=[mapping.get(n,n) for n in names]
                write(folder/'rename_proof.json',dict(source=str(source),renamed=str(renamed),
                      mapping=mapping,exact_inverse_check_passed=True,logic_parameters_connections_changed=False))
            if bus_net_format=='escaped_bits':
                names=[n.replace('[',r'\[').replace(']',r'\]') for n in names]
            elif bus_net_format=='whole_bus':
                names=sorted({n.split('[')[0] for n in names})
            cst.write_text('\n'.join('CLOCK_LOC "'+n+'" LOCAL_CLOCK'+
                  (' = LOGIC' if explicit_logic_type and ('rb_ra' in n or 'diagnostic_read_address_' in n) else '')+';'
                  for n in names)+'\n',encoding='utf-8')
            assert 'add_file -type cst' not in tcl
            tcl=tcl.replace('run pnr', 'add_file -type cst {'+cst.as_posix()+'}\nrun pnr',1)
            assert tcl.count('run pnr')==1
            (folder/'physical.tcl').write_text(tcl,encoding='utf-8')
        self.save('prepared')

    def execute(self):
        for name in ('baseline','shared'):
            folder=self.out/name
            self.run(folder,'physical',[str(TOOL),'physical.tcl'],limit=0,physical=True)
            row=review(folder)
            row['requested_local_only_assignments_verified']=not any(
                g['net']!='clk_d' for g in row['global_allocations'])
            self.state['results'][name]=row
            self.save(name+'_local_only_report_collected')
            assert row['requested_local_only_assignments_verified'], 'unexpected spare global allocation requires review'
        before=self.state['protected_joint_build']
        after=reference_snapshot(Path(before['build']))
        self.state['protected_joint_build_input_metadata_unchanged']=before['input_metadata']==after['input_metadata']
        assert self.state['protected_joint_build_input_metadata_unchanged']
        self.state['protected_joint_build_latest']=after
        self.save('local_only_comparison_complete_requires_review')


def main():
    p=argparse.ArgumentParser();p.add_argument('--prior',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);p.add_argument('--joint-reference',type=Path,required=True)
    p.add_argument('--bus-net-format',choices=['literal','escaped_bits','whole_bus'],default='literal')
    p.add_argument('--scalar-addresses',action='store_true')
    p.add_argument('--explicit-local-logic-type',action='store_true',help='Parser diagnostic: current manual disallows a type on LOCAL_CLOCK; never presume accepted or effective.')
    a=p.parse_args();r=LocalRoutes(a.prior.resolve(),a.out.resolve(),a.joint_reference.resolve(),a.bus_net_format,a.scalar_addresses,a.explicit_local_logic_type)
    try:r.execute()
    except BaseException as exc:
        r.state['failure']=str(exc);r.save('local_analysis_failed_requires_review');raise


if __name__=='__main__':main()
