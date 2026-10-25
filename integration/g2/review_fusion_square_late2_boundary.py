from pathlib import Path
from datetime import datetime
import json,re,difflib,argparse

parser=argparse.ArgumentParser(description='Independent exact fusion late2 source/reset boundary review; no build')
parser.add_argument('--out',type=Path,required=True)
parser.add_argument('--candidate-dir',type=Path,default=Path('F:/nav_integration_g2_20261023/fusion_square_pipeline2_candidate1'))
parser.add_argument('--source',type=Path,default=Path('F:/nav_integration_g2_20261023/baseline_fusion_only_syn23/control/imu_fusion.v'))
parser.add_argument('--math-proof',type=Path,required=True)
args=parser.parse_args()
OUT=args.out.resolve();P=args.candidate_dir.resolve();actual=args.source.resolve();math_proof=args.math_proof.resolve()
assert not OUT.exists(),'preserve existing review; choose a new output directory'
OUT.mkdir(parents=True)
old=(P/'source_imu_fusion.v').read_text(encoding='utf-8')
new=(P/'imu_fusion.v').read_text(encoding='utf-8')
assert old==actual.read_text(encoding='utf-8')
source_byte_equal=(P/'source_imu_fusion.v').read_bytes()==actual.read_bytes()
work=new
declarations='''    // The original unsigned32/24 absolute operands are retained in32bits;
    // the product and square storage stay64bits, without quantization.
    reg [31:0] norm_abs_x=0,norm_abs_y=0,norm_abs_z=0;
    reg [63:0] norm_product_x=0,norm_product_y=0,norm_product_z=0;
    reg norm_abs_valid=0,norm_product_valid=0;
'''
pipeline='''            norm_abs_valid<=0;
            norm_product_valid<=norm_abs_valid;
            if(norm_abs_valid) begin
                norm_product_x<={32'd0,norm_abs_x}*norm_abs_x;
                norm_product_y<={32'd0,norm_abs_y}*norm_abs_y;
                norm_product_z<={32'd0,norm_abs_z}*norm_abs_z;
            end
            if(norm_product_valid) begin
                norm_square_x<=norm_product_x;
                norm_square_y<=norm_product_y;
                norm_square_z<=norm_product_z;
            end
'''
for text in (declarations,pipeline,'            norm_abs_valid<=0;norm_product_valid<=0;\n'):
    assert work.count(text)==1
    work=work.replace(text,'',1)
work=work.replace('imu_fusion_isqrt_late2 u_sqrt (','imu_isqrt u_sqrt (',1)
for axis in 'xyz':
    work=work.replace(f'norm_abs_{axis}<=abs16(accel_{axis});',f'norm_square_{axis}<={{32\'d0, abs16(accel_{axis})}} * abs16(accel_{axis});',1)
    work=work.replace(f'norm_abs_{axis}<={{8\'d0,abs24(eg{axis})}};',f'norm_square_{axis}<=abs24(eg{axis}) * abs24(eg{axis});',1)
assert work.count('                norm_abs_valid<=1;\n')==2
work=work.replace('                norm_abs_valid<=1;\n','')
marker='// Private to this fusion contract: NOT a generic64bit sqrt replacement.'
assert work.count(marker)==1
helper=work[work.index(marker):work.rindex('`endif')]
work=work[:work.index(marker)]+work[work.rindex('`endif'):]
assert work==old, 'unlisted fusion code change'
sqrt=actual.with_name('imu_isqrt.v').read_text(encoding='utf-8')
sqrt_module=sqrt[sqrt.index('module imu_isqrt ('):sqrt.index('\nendmodule')+len('\nendmodule')]
helper_module=helper[helper.index('module imu_fusion_isqrt_late2 ('):helper.index('\nendmodule')+len('\nendmodule')]
back=helper_module.replace('module imu_fusion_isqrt_late2 (','module imu_isqrt (',1)
back=back.replace('rem <= 0; // valid sum arrives during the two zero trials','rem <= value;',1)
skip='''            end else if(cnt<6'd2) begin
                // Fusion's exact sum is<=3*2^46<2^48. The original trials
                // at2^62 and2^60 therefore both leave acc=0,rem=value.
                // Capture the two-cycle-late sum on the second zero trial;
                // cnt,bitv,busy,done and the final root retain every cycle.
                if(cnt==6'd1) rem<=value;
                acc<=0;bitv<=bitv>>2;cnt<=cnt+1'b1;
'''
assert back.count(skip)==1
back=back.replace(skip,'',1)
# The removed skip block carried the previous end token; restore it.
back=back.replace("root <= acc[31:0];\n            end else begin", "root <= acc[31:0];\n            end else begin",1)
assert back==sqrt_module, 'unlisted helper change'
assert new.count('imu_fusion_isqrt_late2 u_sqrt')==1
finite=json.loads((P/'result.json').read_text(encoding='utf-8'))
assert finite['passed'] and finite['clock_cycles']==200000 and finite['pipeline_reset_phases']==4
math=json.loads(math_proof.read_text(encoding='utf-8'))
assert math['passed'] and math['reset_interruption_phases_checked']==36
result=dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),
    candidate=str(P/'imu_fusion.v'),origin=str(actual),
    source_frozen_origin_byte_equal=source_byte_equal,source_frozen_origin_text_equal=True,
    source_snapshot_only_newline_difference=('none: byte-identical' if source_byte_equal else 'only line endings differ; all normalized text exactly equal'),
    exact_inverse_transform_recovers_source=True,
    helper_exact_inverse_transform_recovers_original_isqrt=True,
    only_changes=['three32bit_abs_registers','three64bit_product_registers','two_private_valid_bits','st4/st9_capture_abs_instead_of_square','private_late2_sqrt_instance_and_helper'],
    unchanged=['ports','parameters','clocks','FSM_state_transitions','sqrt_start_pulses','published_outputs_flags_valid','reciprocal_dividers','original64bit_square_storage'],
    actual_helper_only_one_private_fusion_call=True,
    init_and_reset_argument='New abs/product data initialize0 but need no data reset: async reset clears both valid stages and original sqrt_start/helper state. Thereafter a fresh st4/st9 capture at E produces product E+1 and square E+2; late2 helper samples sum E+3. A reset at any of these edges cancels the pending valid chain; no pre-reset product can be consumed without fresh abs capture.',
    timing={'st4_or_st9':'E','sqrt_accept':'E+1','product_capture':'E+1','square_capture':'E+2','helper_cnt0_skip':'E+2','helper_cnt1_capture':'E+3','matching_cnt2_state':'E+3','root_done':'E+34'},
    independent_math=str(math_proof),candidate_finite_RTL_proof=str(P/'result.json'),
    eligible_for_exploratory_joint_SYN26=True,physical_timing_or_deployment_passed=False,
    risks=['Adds290 private register bits before synthesis optimization; placement/fanout and timing remain unmeasured.',
           'late2 is not a generic64-bit sqrt replacement: N>=2^60 has explicit counterexamples.',
           '200000-cycle RTL proof and control-reset edge cases do not establish all fusion state-space formal equivalence.',
           'Actual SYN must bind this exact source, retain clocks/capacity/control contracts and confirm arithmetic mapping; PNR must prove50MHz.'])
(OUT/'boundary_review.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
(OUT/'candidate_exact.patch').write_text(''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='source_imu_fusion.v',tofile='imu_fusion.v')),encoding='utf-8')
print(json.dumps({'passed':True,'boundary_review':str(OUT/'boundary_review.json'),'eligible_for_exploratory_joint_SYN26':True}))
