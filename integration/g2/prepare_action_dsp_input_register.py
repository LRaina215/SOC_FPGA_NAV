"""Move the private capture to the actual DSP A input, preserving E2/E3.

Routed35's worst action path has14.237ns through the DSP A pin, then15.829ns
through multiply/round/output. Most delay is routing; these are path partitions,
not guarantees for a new placement. No new latency or frequency exception.
"""
from pathlib import Path
from datetime import datetime
import argparse
import re
import json

import prepare_action_preview_product as preview
import prepare_baseline_action_preview as joint
from prepare_baseline_fusion_retime import metadata, save

BASE = Path('F:/nav_integration_g2_20261023')
SOURCE = BASE/'baseline_fusion_action_preview_syn28'
LIBRARY = Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
ACTION = 'control/policy_action_map.v'
GAINS = [('lh',0,19),('lk',24,-19),('lw',48,-600),('rh',72,19),('rk',96,19),('rw',120,600)]


def transform(original):
    blocks=[]
    for group,offset,gain in GAINS:
        value = ("-18'sd"+str(-gain)) if gain<0 else ("18'sd"+str(gain))
        blocks.append(f'''    wire signed [23:0] clipped_{group}=clip(preview_actions[{offset}+:24]);
    wire [47:0] product48_{group};
    wire signed [63:0] product_{group}={{{{16{{product48_{group}[47]}}}},product48_{group}}};
    MULTALU27X18 #(.AREG_CLK("CLK0"),.AREG_CE("CE0"),.AREG_RESET("RESET0"),
        .BREG_CLK("BYPASS"),.DREG_CLK("BYPASS"),.C_IREG_CLK("BYPASS"),.C_PREG_CLK("BYPASS"),
        .PREG_CLK("BYPASS"),.OREG_CLK("BYPASS"),.MULT_RESET_MODE("SYNC"),
        .A_SEL(1'b0),.P_SEL(1'b0),.C_SEL(1'b0),.CASI_SEL(1'b0),.ACC_SEL(1'b0),
        .ADD_SUB_0(1'b0),.ADD_SUB_1(1'b0),.PRE_LOAD(48'd0)) dsp_input_{group} (
        .DOUT(product48_{group}),.CASO(),.SOA(),
        .A({{{{3{{clipped_{group}[23]}}}},clipped_{group}}}),.B({value}),.C(48'd0),.D(26'd0),
        .SIA(27'd0),.CASI(48'd0),.PSEL(1'b0),.ASEL(1'b0),.CSEL(1'b0),.CASISEL(1'b0),
        .PADDSUB(1'b0),.ACCSEL(1'b0),.ADDSUB(2'b0),.CLK({{1'b0,clk}}),.CE(2'b11),.RESET(2'b0));
''')
    start=preview.PRODUCTS.index('    function signed [31:0] rounded_product;')
    products=''.join(blocks)+preview.PRODUCTS[start:]
    assert original.count(preview.PRODUCTS)==1
    return original.replace(preview.PRODUCTS,products,1)


def prepare(out, prepared):
    proof=preview.prepare(out)
    original=(out/ACTION).read_text();new=transform(original)
    (out/ACTION).write_text(new,newline='\n')
    # Use exactly the installed unprotected public macro models. No substitute
    # arithmetic model or protected PHY body participates in this simulation.
    library=LIBRARY.read_text();models=[]
    for name in ['GSR','MULTALU27X18']:
        m=re.search(r'\bmodule '+name+r'\b.*?\bendmodule',library,re.S)
        assert m;models.append(m[0])
    (out/'vendor_action_macros.v').write_text('`timescale 1ns/1ps\n'+'\n\n'.join(models)+'\n')
    tb=out/'tb_arithmetic.sv';text=tb.read_text()
    assert text.count('module tb_action_map_equivalence;')==1
    tb.write_text(text.replace('module tb_action_map_equivalence;',"module tb_action_map_equivalence;\n GSR GSR(.GSRI(1'b1));",1))
    proof.update(recorded_at=datetime.now().astimezone().isoformat(),
        candidate_bindings=[metadata(out/n)for n in proof['changed_RTL']],
        source_single_delta_vs_SYN28=ACTION,explicit_DSP_AREG_private_capture=True,
        original_signed64_rounding_and_E2_E3_edges_preserved=True,
        private_output_product_registers=0,private_AREG_stages_requested=6,
        source_basis_actual_routed35_A_pin_partition_ns=14.237,
        source_basis_actual_routed35_remaining_partition_ns=15.829,
        these_are_old_path_partitions_not_new_timing_predictions=True,
        vendor_library_binding=metadata(LIBRARY),unmodified_public_models_only=True)
    save(out/'prepared_review.json',proof)
    preview.simulate(out);preview.simulate_policy(out)
    joint.CANDIDATE=out;result=joint.prepare(prepared)
    manifest=json.loads((prepared/'manifest.json').read_text())
    manifest.update(explicit_DSP_AREG_private_capture=True,AREG_only_vs_SYN28=True,automatic_pnr=False)
    save(prepared/'manifest.json',manifest)
    result.update(AREG_only_vs_SYN28=True,actual_DSP_AREG_mapping_or_timing_passed=False)
    save(prepared/'prepared_source_review.json',result)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--out',type=Path,required=True);parser.add_argument('--prepared',type=Path,required=True)
    a=parser.parse_args();r=prepare(a.out,a.prepared);print(json.dumps(dict(passed=r['passed'],prepared=str(a.prepared))))
