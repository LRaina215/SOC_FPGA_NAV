"""Exact width for the live raw-gyro absolute sum, with no extra registers.

For signed16 inputs, each magnitude is <=32768, so their sum is <=98304
and fits unsigned17. Explicit zero-extension preserves the original unsigned32
comparisons and modulo32 additions for every parameter and bias-sum value.
"""
from pathlib import Path
import re


def generate(target):
    target=Path(target);s=target.read_text(encoding='utf-8')
    pattern=r'abs16\(gyro_x\)\s*\+\s*abs16\(gyro_y\)\s*\+\s*abs16\(gyro_z\)'
    assert len(re.findall(pattern,s))==3,'expected the two bias uses and live accel-trust gate'
    s=re.sub(pattern,'gyro_abs_sum32',s)
    anchor='    reg rejected_sample = 0;'
    assert s.count(anchor)==1 and 'wire [16:0] gyro_abs_sum' not in s
    s=s.replace(anchor,'''    // Preserve live-input semantics in all three original consumers.
    // Every |signed16| <=32768; max sum98304 fits17 bits, including -32768.
    wire [15:0] gyro_abs_x=abs16(gyro_x),gyro_abs_y=abs16(gyro_y),gyro_abs_z=abs16(gyro_z);
    wire [16:0] gyro_abs_sum={1'b0,gyro_abs_x}+{1'b0,gyro_abs_y}+{1'b0,gyro_abs_z};
    wire [31:0] gyro_abs_sum32={15'd0,gyro_abs_sum};
'''+anchor)
    target.write_text(s,encoding='utf-8',newline='\n')
