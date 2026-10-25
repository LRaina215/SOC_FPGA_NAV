"""Freeze one proved row-port mode fix after the PA2122 netlist rejection."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import shutil
import re


def prepare(source, proof, out, explicit=False):
    assert not out.exists()
    result=json.loads((proof/'functional_result.json').read_text())
    assert result['passed'] and result['metrics']['cases']==8 and result['metrics']['words']==524288
    assert 'row bank read/write overlap violates normal-mode contract' in (proof/'tb_slf_row_update.sv').read_text()
    before=(source/'ddr3/nav_ddr_row_stream.v').read_text()
    after=(proof/'nav_ddr_row_stream.v').read_text()
    old='            if(in_re[b])q<=input_row[in_addr[b*8+:8]];'
    new='''            // The caller never reads a bank while writing that bank. Keep
            // its last read result during writes; GW5A true dual-port RAM
            // supports normal mode, but rejects read-before-write mode.
            else if(in_re[b])q<=input_row[in_addr[b*8+:8]];'''
    if explicit:
        evidence=json.loads((proof/'explicit_mode_result.json').read_text())
        assert evidence['actual_vendor_DPX9B_model'] and evidence['original_8_case_cycle_counts_unchanged']
        assert len(evidence['actual_SYN_input_RAM_parameters'])==2
        assert all(p['WRITE_MODE0']==p['WRITE_MODE1']=="2'b00"
                   for p in evidence['actual_SYN_input_RAM_parameters'].values())
        after=(proof/'hardware_explicit_row_stream.v').read_text()
        plain=after.removeprefix('`define GOWIN_ROW_PRIMITIVES\n')
        header='`ifdef SYNTHESIS\n`define NAV_DDR_ROW_PHYSICAL\n`endif\n`ifdef GOWIN_ROW_PRIMITIVES\n`define NAV_DDR_ROW_PHYSICAL\n`endif\n'
        tail='`ifdef NAV_DDR_ROW_PHYSICAL\n`undef NAV_DDR_ROW_PHYSICAL\n`endif\n'
        assert plain.count(header)==plain.count(tail)==1
        plain=plain.replace(header,'').replace(tail,'')
        plain,n=re.subn(r'(?ms)^`ifdef NAV_DDR_ROW_PHYSICAL\n.*?^`else\n(.*?)^`endif\n',r'\1',plain)
        assert n==1 and plain==before,'delta must only select explicit equivalent input-row RAM'
    else:
        assert before.count(old)==1 and after==before.replace(old,new)
    shutil.copytree(source,out,ignore=shutil.ignore_patterns('impl','__pycache__','gw_sh.log',
        'preflight.log','job.json','tool_runtime.json','synthesis_summary.json','memory_inventory.json',
        'expanded_cells_readonly.json'))
    (out/'ddr3/nav_ddr_row_stream.v').write_text(after)
    m=json.loads((source/'manifest.json').read_text())
    modified=[]
    for name in m['source_files']:
        relative=Path(name).relative_to(source)
        if Path(name).read_bytes()!=(out/relative).read_bytes():modified.append(relative.as_posix())
    assert modified==['ddr3/nav_ddr_row_stream.v']
    def relocate(value):
        if isinstance(value,str):return value.replace(str(source),str(out)).replace(source.as_posix(),out.as_posix())
        if isinstance(value,list):return [relocate(v) for v in value]
        if isinstance(value,dict):return {k:relocate(v) for k,v in value.items()}
        return value
    m=relocate(m)
    m.update(frozen_source_build=str(source),row_normalmode_proof=str(proof),row_read_write_mutex_verified=True,
             PA2122_fixed_in_RTL=True,automatic_pnr=False,programmed=False)
    if explicit:m['row_explicit_mode_proof']=str(proof)
    (out/'manifest.json').write_text(json.dumps(m,indent=2))
    (out/'run.tcl').write_text(relocate((source/'run.tcl').read_text()))
    report=dict(prepared_at=datetime.now().astimezone().isoformat(),source=str(source),out=str(out),proof=str(proof),
        exact_modified_inputs=modified,checks=524288,read_write_mutex_checked=True,
        all_original_control_and_CPU_inputs_unchanged=True,original_assets_unchanged=True,
        RAM_capacity_and_clock_periods_unchanged=True,physical_RAM_verified=False,programmed=False)
    (out/'row_normalmode_preparation.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ('source','proof','out'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--explicit-mode',action='store_true')
    a=p.parse_args();prepare(a.source.resolve(),a.proof.resolve(),a.out.resolve(),a.explicit_mode)
