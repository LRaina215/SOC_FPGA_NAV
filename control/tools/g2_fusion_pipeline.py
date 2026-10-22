"""Equivalent IMU norm retiming; no filter/calibration or output-cycle change.

Register the three original squares where the old design registered their sum.
The existing sqrt input register captures the sum on the next clock,
exactly when it captured the old sum. Start/state/output timing is untouched.
"""
from pathlib import Path
CONTROL=Path(__file__).resolve().parents[1]


def generate(target):
    s=(CONTROL/'fpga/src/imu_fusion.v').read_text(encoding='utf-8')
    def replace(old,new):
        nonlocal s
        assert s.count(old)==1,old
        s=s.replace(old,new)
    replace('    reg [63:0] sqrt_value = 0;', '''    // Preserve the original absolute-value/square spelling and numeric range.
    // These registers, like the old sqrt_value, retain their value on reset;
    // sqrt_start and the engine reset still prevent stale values being used.
    reg [63:0] norm_square_x=0,norm_square_y=0,norm_square_z=0;
    // The maximum sum is 3*2^46, so 64 bits retain the original exact range.
    // The signed-square candidates triggered SP00019 both with and without
    // carry-save; the exact tool-internal cause remains unverified.
    wire [63:0] sqrt_value=norm_square_x+norm_square_y+norm_square_z;''')
    replace('''                sqrt_value <= {32'd0, abs16(accel_x)} * abs16(accel_x)
                            + {32'd0, abs16(accel_y)} * abs16(accel_y)
                            + {32'd0, abs16(accel_z)} * abs16(accel_z);''', '''                norm_square_x<={32'd0, abs16(accel_x)} * abs16(accel_x);
                norm_square_y<={32'd0, abs16(accel_y)} * abs16(accel_y);
                norm_square_z<={32'd0, abs16(accel_z)} * abs16(accel_z);''')
    replace('''                sqrt_value <= abs24(egx) * abs24(egx)
                            + abs24(egy) * abs24(egy)
                            + abs24(egz) * abs24(egz);''', '''                norm_square_x<=abs24(egx) * abs24(egx);
                norm_square_y<=abs24(egy) * abs24(egy);
                norm_square_z<=abs24(egz) * abs24(egz);''')
    Path(target).write_text(s,encoding='utf-8',newline='\n')
