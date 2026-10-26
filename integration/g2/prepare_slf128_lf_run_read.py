"""Freeze the minimal LF array-read-enable repair, with no tool launches."""
import argparse
import json
from pathlib import Path
from review_stage3_slf128_memories import meta

OLD='    wire [CW-1:0] l1 = lb1[c1], l2 = lb2[c1], l3 = lb3[c1], l4 = lb4[c1];'
NEW='''    // T0 prefetches the old words needed by the following T1 write.  The
    // private read registers need no reset: v1 can become valid only after a
    // run edge has fetched all four words.  In the final drain cycle v1 stays
    // high for its last write while run is low, so the physical array read
    // port must be disabled rather than merely holding an output with OCE.
    reg [CW-1:0] l1, l2, l3, l4;
    always @(posedge clk) begin
        if (!rst && run) begin
            l1 <= lb1[c]; l2 <= lb2[c]; l3 <= lb3[c]; l4 <= lb4[c];
        end
    end'''


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--source',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    assert not a.out.exists();a.out.mkdir(parents=True)
    old=a.source/'nav/rtl/match/slf_lf.v';text=old.read_text(encoding='utf8');assert text.count(OLD)==1
    new=text.replace(OLD,NEW);assert new.replace(NEW,OLD)==text
    (a.out/'slf_lf.v').write_text(new,encoding='utf8')
    (a.out/'old.v').write_text(text.replace('module slf_lf #(','module slf_lf_old #(',1),encoding='utf8')
    (a.out/'new.v').write_text(new.replace('module slf_lf #(','module slf_lf_new #(',1),encoding='utf8')
    tb=Path(__file__).with_name('tb_slf128_lf_run_read.sv');(a.out/'tb.sv').write_bytes(tb.read_bytes())
    r=dict(prepared=True,source=meta(old),only_replacement=OLD,repaired_source=meta(a.out/'slf_lf.v'),frozen_validation_inputs=[meta(a.out/n) for n in ('old.v','new.v','tb.sv')],
           preserved='All four arrays, T1 writes, cell/coordinate arithmetic and public pipeline/latency unchanged; no array/private-register initialization added',
           required_before_new_G2=['Full128 old/new four-state miter and interrupt/reset/restart','Original five-frame128 integer-model regression with only LF source replaced','One C-device small actual mapping: CEB/BLKSEL read active equivalent !rst&&run; OCE alone is not sufficient','Actual mapped complete scan plus drain has no overlapping18-bit chunk read/write'],
           original_collision_report='F:/nav_integration_g2_20261023/SLF128_LF_actual_control_trace1/result.json',tools_started=False,automatic_joint_SYN=False,automatic_PNR=False)
    (a.out/'preparation.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf8');print(json.dumps(r,ensure_ascii=False))


if __name__=='__main__':main()
