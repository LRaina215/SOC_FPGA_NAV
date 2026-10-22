"""Equivalent IMU norm retiming; no filter/calibration or output-cycle change.

Register the three signed squares where the old design registered their sum.
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
    replace('    reg [63:0] sqrt_value = 0;', '''    // x*x == abs(x)*abs(x), including the signed minimum. No abs carry chain.
    // These registers, like the old sqrt_value, retain their value on reset;
    // sqrt_start and the engine reset still prevent stale values being used.
    reg [47:0] norm_square_x=0,norm_square_y=0,norm_square_z=0;
    function [47:0] signed_square24;
        input signed [23:0] value;
        reg signed [47:0] product;
        begin product=$signed(value)*$signed(value);signed_square24=product;end
    endfunction
    // Explicit extensions preserve the full sum. Avoid the carry-save/shift
    // spelling that triggers SP00019 in the installed Gowin synthesizer.
    wire [48:0] norm_xy={1'b0,norm_square_x}+{1'b0,norm_square_y};
    wire [49:0] norm_xyz={1'b0,norm_xy}+{2'b00,norm_square_z};
    wire [63:0] sqrt_value={14'd0,norm_xyz};''')
    replace('''                sqrt_value <= {32'd0, abs16(accel_x)} * abs16(accel_x)
                            + {32'd0, abs16(accel_y)} * abs16(accel_y)
                            + {32'd0, abs16(accel_z)} * abs16(accel_z);''', '''                norm_square_x<=signed_square24(accel_x);
                norm_square_y<=signed_square24(accel_y);
                norm_square_z<=signed_square24(accel_z);''')
    replace('''                sqrt_value <= abs24(egx) * abs24(egx)
                            + abs24(egy) * abs24(egy)
                            + abs24(egz) * abs24(egz);''', '''                norm_square_x<=signed_square24(egx);
                norm_square_y<=signed_square24(egy);
                norm_square_z<=signed_square24(egz);''')
    Path(target).write_text(s,encoding='utf-8',newline='\n')
