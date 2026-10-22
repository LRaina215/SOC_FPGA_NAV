"""Isolated timing candidate; retain original policy values and external cycles.

Only prepare wheel numerators earlier than their existing consumption point.
The maintained original RTL is not overwritten by this generator.
"""
from pathlib import Path

CONTROL=Path(__file__).resolve().parents[1]


def generate(target):
    s=(CONTROL/'fpga/src/rl_observation_v5.v').read_text(encoding='utf-8')
    def replace(old,new):
        nonlocal s
        assert s.count(old)==1,old
        s=s.replace(old,new)
    replace('    reg signed [23:0] words [0:27];','''    // Wheel capture -> partial products -> exact sum. Lane18 is consumed
    // hundreds of cycles later; current/history/valid retain their old cycles.
    localparam signed [63:0] WHEEL_LOW=WHEEL_NUM & 64'h3ffff;
    localparam signed [63:0] WHEEL_HIGH=WHEEL_NUM >>> 18;
    reg [1:0] wheel_phase;
    reg signed [15:0] left_captured,right_captured;
    reg signed [63:0] left_lo,left_hi,right_lo,right_hi;
    reg signed [23:0] words [0:27];''')
    replace('            dividend<=0;quotient<=0;remainder<=0;denominator<=1;',
            '''            dividend<=0;quotient<=0;remainder<=0;denominator<=1;
            wheel_phase<=0;left_captured<=0;right_captured<=0;
            left_lo<=0;left_hi<=0;right_lo<=0;right_hi<=0;''')
    replace('            valid<=0;\n            case(state)', '''            valid<=0;
            wheel_phase<={wheel_phase[0],1'b0};
            if(wheel_phase[0]) begin
                left_lo<=$signed(left_captured)*(WHEEL_LOW*LEFT_ENCODER_SIGN);
                left_hi<=$signed(left_captured)*(WHEEL_HIGH*LEFT_ENCODER_SIGN);
                right_lo<=$signed(right_captured)*(WHEEL_LOW*RIGHT_ENCODER_SIGN);
                right_hi<=$signed(right_captured)*(WHEEL_HIGH*RIGHT_ENCODER_SIGN);
            end
            if(wheel_phase[1]) begin
                numerators[18]<=left_lo+(left_hi<<<18);
                numerators[21]<=right_lo+(right_hi<<<18);
            end
            case(state)''')
    replace('''                    numerators[18]<=$signed(left_delta)*WHEEL_NUM*LEFT_ENCODER_SIGN;
                    numerators[21]<=$signed(right_delta)*WHEEL_NUM*RIGHT_ENCODER_SIGN;''', '''                    left_captured<=left_delta;right_captured<=right_delta;
                    wheel_phase<=2'b01;''')
    Path(target).write_text(s,encoding='utf-8',newline='\n')


if __name__=='__main__':
    import argparse
    ap=argparse.ArgumentParser();ap.add_argument('target',type=Path);a=ap.parse_args();generate(a.target)
