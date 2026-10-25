"""Local timing with the joint design's global-resource budget already occupied.

The extra measurement-only FFs stand in for other joint clock/control nets.
Original extractor and row-memory primitives/ports/parameters are unchanged.
This is not a reproduction of the joint design's placement or congestion.
"""
import argparse
import json
from pathlib import Path

import psutil

from measure_row_read_context import Runner, TOOL, now, reference_snapshot, write
from review_row_read_context import review


PRIMARY=['measurement_primary_'+str(i) for i in range(1,8)]
LW=['measurement_lw_'+str(i) for i in range(8)]


def add_budget_fixture(source):
    start=source.index('module g2_top (')
    top=source[start:]
    port='  busy\n)\n;'
    port_new='  busy,\n  measurement_global_outputs\n)\n;'
    declaration='output busy;'
    decl_new=declaration+'\noutput [14:0] measurement_global_outputs;'
    assert top.count(port)==top.count(declaration)==top.count('wire GND;')==1
    assert 'measurement_' not in source
    added=[]
    for i,name in enumerate(PRIMARY+LW):
        added += ['wire '+name+';', 'wire measurement_sink_'+str(i)+';',
            'DFFRE measurement_source_ff_'+str(i)+' (.Q('+name+'), .D(data_d['+str(i)+']), .CLK(clk_d), .CE(VCC), .RESET(GND));',
            'defparam measurement_source_ff_'+str(i)+'.INIT=1\'b0;',
            'DFFRE measurement_sink_ff_'+str(i)+' (.Q(measurement_sink_'+str(i)+'), .D(data_d['+str(16+i)+']), .CLK(clk_d), .CE('+name+'), .RESET(GND));',
            'defparam measurement_sink_ff_'+str(i)+'.INIT=1\'b0;',
            'OBUF measurement_output_'+str(i)+' (.O(measurement_global_outputs['+str(i)+']), .I(measurement_sink_'+str(i)+'));']
    insertion='\n'+'\n'.join(added)+'\n'
    changed=top.replace(port,port_new,1).replace(declaration,decl_new,1).replace('wire GND;','wire GND;'+insertion,1)
    inverse=changed.replace(port_new,port,1).replace(decl_new,declaration,1).replace(insertion,'',1)
    assert inverse==top
    return source[:start]+changed


class GlobalBudget(Runner):
    def __init__(self,prior,out):
        old=json.loads((prior/'result.json').read_text())
        auto=json.loads((prior/'physical_review.json').read_text())
        assert old['phase']=='local_context_analysis_complete_requires_path_review'
        assert old['protected_joint_build_input_metadata_unchanged']
        assert not out.exists(), 'unique local build required'
        targets={n['net'] for n in auto['baseline']['global_allocations'] if n['net']!='clk_d'}
        assert targets=={n['net'] for n in auto['shared']['global_allocations'] if n['net']!='clk_d'}
        me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS);me.cpu_affinity([max(me.cpu_affinity())])
        out.mkdir();self.out=out
        self.state=dict(created_at=now(),phase='prepared',results={},prior=str(prior),
            controller_pid=me.pid,controller_created_epoch=me.create_time(),assigned_cpu=me.cpu_affinity(),
            protected_joint_build=old['protected_joint_build'],joint_build_modified=False,
            full_joint_acceptance=False,programmed=False,
            execution_policy=old['execution_policy'],function_validation_reused=old['context_behavior'],
            measured_global_budget=dict(real_clock_PRIMARY=1,other_joint_PRIMARY_standins=7,other_joint_LW_standins=8),
            measurement_only_added_registers=30,measurement_only_added_output_ports=15,
            original_extractor_memory_logic_parameters_unchanged=True,
            scope='Local complete-extractor with global budget occupied; not complete joint placement/congestion or IO/CDC signoff.')
        for name in ('baseline','shared'):
            original=prior/name/'impl/gwsynthesis/g2_top.vg'
            text=original.read_text(encoding='utf-8')
            folder=out/name;folder.mkdir()
            measured=folder/'g2_top.vg'
            measured.write_text(add_budget_fixture(text),encoding='utf-8',newline='\n')
            write(folder/'source_proof.json',dict(original_netlist=str(original),measured_netlist=str(measured),
                exact_inverse_proof_passed=True,original_extractor_and_memory_text_unchanged=True,
                added_registers=30,added_outputs=15,synthesis_repeated=False))
            cst=folder/'occupied_globals.cst'
            lines=['CLOCK_LOC "clk_d" BUFG[0] = CLK;']
            lines += ['CLOCK_LOC "'+n+'" BUFG['+str(i)+'] = CE;' for i,n in enumerate(PRIMARY,1)]
            lines += ['CLOCK_LOC "'+n+'" BUFG = CE;' for n in LW]
            lines += ['CLOCK_LOC "'+n+'" LOCAL_CLOCK;' for n in sorted(targets)]
            cst.write_text('\n'.join(lines)+'\n',encoding='utf-8')
            original_tcl=prior/('baseline_physical_retry1' if name=='baseline' else 'shared')/'physical.tcl'
            tcl=original_tcl.read_text()
            assert tcl.count(original.as_posix())==1 and 'add_file -type cst' not in tcl
            tcl=tcl.replace(original.as_posix(),measured.as_posix()).replace('run pnr','add_file -type cst {'+cst.as_posix()+'}\nrun pnr',1)
            (folder/'physical.tcl').write_text(tcl,encoding='utf-8')
        self.save('prepared')

    def execute(self):
        expected={'clk_d':'PRIMARY',**{n:'PRIMARY' for n in PRIMARY},**{n:'LW' for n in LW}}
        for name in ('baseline','shared'):
            folder=self.out/name
            self.run(folder,'physical',[str(TOOL),'physical.tcl'],limit=0,physical=True)
            row=review(folder)
            actual={n['net']:n['resource'] for n in row['global_allocations']}
            row['global_assignments_match_budget']=actual==expected
            row['extractor_extra_global_resources_used']=any(n.startswith('u_extract/') for n in actual)
            row['original_extractor_FF']=row['resources']['Register']-30
            self.state['results'][name]=row;self.save(name+'_occupied_budget_report_collected')
            assert actual==expected, ('actual global budget must match, no spare global benefit',actual,expected)
            assert row['internal_clock_50MHz_pass'], 'local clock timing failed; retain reports'
        before=self.state['protected_joint_build']
        latest=reference_snapshot(Path(before['build']))
        self.state['protected_joint_build_input_metadata_unchanged']=before['input_metadata']==latest['input_metadata']
        assert self.state['protected_joint_build_input_metadata_unchanged']
        self.state['protected_joint_build_latest']=latest
        self.save('local_50MHz_passed_without_extra_extractor_globals')


def main():
    p=argparse.ArgumentParser();p.add_argument('--prior',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();r=GlobalBudget(a.prior.resolve(),a.out.resolve())
    try:r.execute()
    except BaseException as exc:
        r.state['failure']=str(exc);r.save('local_analysis_failed_requires_review');raise


if __name__=='__main__':main()
