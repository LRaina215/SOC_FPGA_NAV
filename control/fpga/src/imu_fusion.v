`ifndef IMU_FUSION_V_INCLUDED
`define IMU_FUSION_V_INCLUDED
`timescale 1ns/1ps

// BMI088 attitude fusion: gyro integration corrected by the accelerometer.
// Emits the policy's projected_gravity, plus body angular velocity.
//
// The calibration this implements was measured on the bench 2026-10-06; the raw
// captures and the three static poses are in IMU_CALIBRATION.md.
//
//   * AXIS MAP IS THE IDENTITY. Three held poses showed nose-down moves only ay
//     (0.0008 -> -0.6081) and right-roll moves only ax (0.0367 -> -0.6439), so
//     x=right, y=forward, z=up already match the simulator body frame
//     (forward = local +Y, upright projected_gravity = [0,0,-1]). The two
//     motions do not cross-couple, which is itself the evidence that there is
//     no installation rotation to undo.
//
//   * projected_gravity = -normalize(accel). The accelerometer measures
//     specific force, so at rest it points UP while the policy wants DOWN.
//     Dropping this minus shows the network an upside-down world, silently.
//
//   * Gyro zero bias is averaged at startup, so the part must be still then.
//     A 1 dps bias integrates to about 10 deg in 10 s and cannot be skipped.
//     If the bias window looks like it was taken while moving, bias_suspect
//     latches so a caller can refuse to arm on a bad bias.
//
// TWO SIGN ERRORS HERE ARE SILENT -- no flag, no fault, just a policy fed a
// wrong attitude. Each is fixed at the line that would break:
//   1. dg/dt = -(w x g). The minus is required; +w x g integrates BACKWARDS,
//      so the estimate rotates opposite to the vehicle.
//   2. projected_gravity = -normalize(accel).
//
// Outputs are physical units in signed 24-bit F12 (word/4096). The observation
// scale factors (gravity x3.0, ang_vel x1.3) are NOT applied here; the
// observation assembler owns that.
//
// Attitude is carried as the gravity unit vector itself rather than as a
// quaternion: that is exactly what the policy consumes, so no conversion is
// needed. The cost is that this tracks the gravity direction only, not full
// heading, which is all the observation contract asks for -- there is no
// absolute-yaw input slot.
//
// Numeric notes:
//   gyro  -> rad/s: raw is 16.384 LSB/dps, so rad/s = raw/(16.384*57.2958)
//                   = raw/938.73. In F12 that is raw * 4096/938.73
//                   = raw * 4.3634, so the constant below is 4466/1024
//                   (0.06% high, far below sensor noise).
//   accel -> unit : raw is 10920 LSB/g. Normalized by the measured magnitude.
//   Integration uses dg = -(w x g) * dt with the measured sample interval.
//   CLK_HZ must match the input clock; this module does not resample.
//
// Implementation notes:
//   Normalization computes q = 2^28/|v| with imu_recip (bit-serial long
//   division, verified exact against Python over 20000 random divisors), then
//   (v*q)>>16 lands directly in F12. An earlier Newton-iteration reciprocal was
//   discarded: its seed buckets were scaled for the wrong power of two, so
//   |v|~4080 came back as [0,0,482] -- 12% of true -- while |v|~4000 was
//   correct. A magnitude-dependent error that no check inside the hardware would
//   have caught, and that the Python model found immediately.
//
//   Measured accuracy against an exact rational reference: worst component
//   error 1.05 LSB (one quantization step), worst length error 0.31%, both well
//   under the 0.6% sensor noise seen on the bench.
//
//   The accelerometer is trusted only when |a| is inside the 1 g window AND the
//   part is not rotating fast. Without that gate, linear and centripetal
//   acceleration corrupt the estimate exactly when the vehicle is moving --
//   which is when the policy most needs a correct attitude. Every rejection
//   raises accel_rejected so a caller can tell "correcting" from "coasting".
module imu_fusion #(
    // Accelerometer trust in the complementary filter, 1/4096 units.
    // 82 = 0.02. Larger trusts the accelerometer more: faster correction of
    // gyro drift, but more noise and more kinematic-acceleration bleed.
    parameter integer ACCEL_TRUST = 82,
    // Samples averaged for the startup bias, at the 10 ms sensor period.
    parameter integer BIAS_SAMPLES = 200,   // 2 s
    // Mean |raw gyro| over the bias window above which the device was clearly
    // moving, so the measured "bias" is meaningless. Latched, not hidden.
    parameter integer BIAS_MOVE_LIMIT = 300,
    // |accel| window (raw LSB; 1 g = 10920) treated as pure gravity. Outside
    // it, kinematic acceleration is present and the direction is untrustworthy.
    parameter integer ACCEL_MIN = 8500,
    parameter integer ACCEL_MAX = 13400,
    // Sum |raw gyro| above which the part is rotating fast enough that the
    // accelerometer is not a clean gravity reference. 600 counts is ~37 dps.
    parameter integer GYRO_MOVE_LIMIT = 600,
    // Current bmi088.v configures +/-2000dps: use 4466.
    parameter signed [31:0] GYRO_MUL = 32'sd4466,
    parameter integer CLK_HZ = 50000000
) (
    input wire clk,
    input wire rst_n,
    // One-clock pulse when a fresh six-axis sample has been published.
    input wire sample_valid,
    input wire signed [15:0] accel_x,
    input wire signed [15:0] accel_y,
    input wire signed [15:0] accel_z,
    input wire signed [15:0] gyro_x,
    input wire signed [15:0] gyro_y,
    input wire signed [15:0] gyro_z,
    // body angular velocity, rad/s, F12
    output reg signed [23:0] ang_vel_x = 0,
    output reg signed [23:0] ang_vel_y = 0,
    output reg signed [23:0] ang_vel_z = 0,
    // gravity unit vector, F12; upright is (0, 0, -4096)
    output reg signed [23:0] gravity_x = 0,
    output reg signed [23:0] gravity_y = 0,
    output reg signed [23:0] gravity_z = -24'sd4096,
    // one-clock pulse when gravity/ang_vel have been updated
    output reg valid = 0,
    // high while the startup bias average is still running
    output reg calibrating = 1,
    // sticky: set if the bias window looked like it was taken while moving
    output reg bias_suspect = 0,
    // one-clock pulse when this sample skipped the accelerometer correction
    output reg accel_rejected = 0
);
    localparam GYRO_SHIFT = 10;
    reg [31:0] sample_clock, last_sample_clock, integration_ticks;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sample_clock<=0; last_sample_clock<=0; integration_ticks<=CLK_HZ/100;
        end else begin
            sample_clock<=sample_clock+1;
            if (sample_valid) begin
                integration_ticks<=sample_clock-last_sample_clock;
                last_sample_clock<=sample_clock;
            end
        end
    end

    // ---- startup gyro bias average ----
    reg signed [31:0] bias_sum_x = 0, bias_sum_y = 0, bias_sum_z = 0;
    reg [31:0] bias_abs_sum = 0;
    reg [31:0] bias_count = 0;
    reg signed [15:0] bias_x = 0, bias_y = 0, bias_z = 0;

    // ---- attitude state: gravity unit vector, F12 ----
    reg signed [23:0] gx = 0, gy = 0, gz = -24'sd4096;

    // ---- sqrt engine ----
    reg sqrt_start = 0;
    reg [63:0] sqrt_value = 0;
    wire [31:0] sqrt_root;
    wire sqrt_done;
    imu_isqrt u_sqrt (
        .clk(clk), .rst_n(rst_n), .start(sqrt_start), .value(sqrt_value),
        .root(sqrt_root), .done(sqrt_done), .busy()
    );

    // ---- reciprocal engine: q = 2^28/d, exact ----
    reg recip_start = 0;
    reg [31:0] recip_d = 0;
    wire signed [31:0] recip_q;
    wire recip_done;
    imu_recip u_recip (
        .clk(clk), .rst_n(rst_n), .start(recip_start), .d(recip_d),
        .q(recip_q), .done(recip_done), .busy()
    );

    // ---- scratch ----
    reg signed [63:0] crs, d1, d2, d3;
    reg signed [23:0] n1, n2, n3;       // gyro-integrated candidate, F12
    reg signed [23:0] egx, egy, egz;    // candidate after accel correction
    reg signed [23:0] nx, ny, nz;       // normalized accel direction, F12
    reg signed [31:0] avx, avy, avz;
    reg [31:0] mag32;

    function [31:0] abs16;
        input signed [15:0] v;
        begin abs16 = v[15] ? -v : v; end
    endfunction

    function [23:0] abs24;
        input signed [23:0] v;
        begin abs24 = v[23] ? -v : v; end
    endfunction

    reg rejected_sample = 0;
    reg signed [55:0] p0,p1,p2,p3,p4,p5;
    reg signed [63:0] div_x,div_y,div_z;
    reg div_start=0;
    reg bias_div_start=0;
    reg signed [63:0] bias_div_x,bias_div_y,bias_div_z;
    wire signed [63:0] bias_qx,bias_qy,bias_qz;
    wire bias_done;
    imu_div100_3 #(.DIVISOR(BIAS_SAMPLES)) bias_divider(.clk(clk),.rst_n(rst_n),.start(bias_div_start),
        .x(bias_div_x),.y(bias_div_y),.z(bias_div_z),.qx(bias_qx),.qy(bias_qy),.qz(bias_qz),.done(bias_done));
    wire signed [63:0] div_qx,div_qy,div_qz;
    wire div_done;
    imu_div100_3 #(.DIVISOR(CLK_HZ)) divider(.clk(clk),.rst_n(rst_n),.start(div_start),
        .x(div_x),.y(div_y),.z(div_z),.qx(div_qx),.qy(div_qy),.qz(div_qz),.done(div_done));
    reg [4:0] st = 0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= 0;
            rejected_sample <= 0;
            egx <= 0; egy <= 0; egz <= -4096;
            nx <= 0; ny <= 0; nz <= -4096;
            bias_sum_x <= 0; bias_sum_y <= 0; bias_sum_z <= 0;
            bias_abs_sum <= 0; bias_count <= 0;
            bias_x <= 0; bias_y <= 0; bias_z <= 0;
            bias_suspect <= 0; calibrating <= 1; valid <= 0; accel_rejected <= 0;
            gx <= 0; gy <= 0; gz <= -24'sd4096;
            gravity_x <= 0; gravity_y <= 0; gravity_z <= -24'sd4096;
            ang_vel_x <= 0; ang_vel_y <= 0; ang_vel_z <= 0;
            sqrt_start <= 0; recip_start <= 0;div_start<=0;bias_div_start<=0;
        end else begin
            valid <= 0;
            accel_rejected <= 0;
            sqrt_start <= 0;
            recip_start <= 0;
            div_start <= 0;
            bias_div_start<=0;
            case (st)
            5'd0: if (sample_valid) st <= 5'd1;

            // ---- startup gyro bias: the device must be still here ----
            5'd1: if (sample_valid) begin
                bias_sum_x <= bias_sum_x + gyro_x;
                bias_sum_y <= bias_sum_y + gyro_y;
                bias_sum_z <= bias_sum_z + gyro_z;
                bias_abs_sum <= bias_abs_sum
                    + abs16(gyro_x) + abs16(gyro_y) + abs16(gyro_z);
                if (bias_count == BIAS_SAMPLES - 1) begin
                    bias_div_x<=bias_sum_x+gyro_x;
                    bias_div_y<=bias_sum_y+gyro_y;
                    bias_div_z<=bias_sum_z+gyro_z;
                    bias_div_start<=1;
                    // Mean |raw| over the window. 300 counts is ~18 dps, far
                    // beyond any real BMI088 bias, so the part was moving and
                    // the average is not a bias. Latch that rather than
                    // silently subtracting a bogus number.
                    bias_suspect <= ((bias_abs_sum
                        + abs16(gyro_x) + abs16(gyro_y) + abs16(gyro_z))
                        ) >= ((BIAS_MOVE_LIMIT+1)*BIAS_SAMPLES);
                    bias_count <= 0;
                    bias_sum_x <= 0; bias_sum_y <= 0; bias_sum_z <= 0;
                    bias_abs_sum <= 0;
                    st <= 5'd16;
                end else bias_count <= bias_count + 1'b1;
            end

            5'd16: if(bias_done) begin
                bias_x<=bias_qx[15:0];bias_y<=bias_qy[15:0];bias_z<=bias_qz[15:0];
                calibrating<=0;st<=5'd2;
            end

            // ---- gyro to rad/s (F12), bias removed ----
            5'd2: if (sample_valid) begin
                rejected_sample <= 0;
                avx <= ((gyro_x - bias_x) * GYRO_MUL) >>> GYRO_SHIFT;
                avy <= ((gyro_y - bias_y) * GYRO_MUL) >>> GYRO_SHIFT;
                avz <= ((gyro_z - bias_z) * GYRO_MUL) >>> GYRO_SHIFT;
                st <= 5'd3;
            end

            // ---- integrate attitude using measured dt ----
            // w is F12 rad/s and g an F12 unit vector, so the cross product is
            // Q24. Multiply by interval ticks and divide by CLK_HZ; >>>12 returns to F12.
            //
            // THE MINUS IS REQUIRED. The attitude satisfies R^T_dot = -[w]x R^T,
            // so dg/dt = -(w x g). Writing +w x g instead integrates the
            // attitude BACKWARDS: the estimate rotates opposite to the vehicle
            // and nothing anywhere reports an error.
            5'd3: begin
                p0<=avy*gz;p1<=avz*gy;p2<=avz*gx;
                p3<=avx*gz;p4<=avx*gy;p5<=avy*gx;
                st<=5'd15;
            end
            5'd15: begin
                // dt is the measured interval between raw publications. The I2C
                // poll includes bus transfer time, so it is not exactly 10ms.
                div_x<=($signed(p0)-$signed(p1))*$signed({1'b0,integration_ticks});
                div_y<=($signed(p2)-$signed(p3))*$signed({1'b0,integration_ticks});
                div_z<=($signed(p4)-$signed(p5))*$signed({1'b0,integration_ticks});
                div_start<=1;st<=5'd14;
            end
            5'd14: if(div_done) begin
                n1<=gx-(div_qx >>> 12);
                n2<=gy-(div_qy >>> 12);
                n3<=gz-(div_qz >>> 12);
                st<=5'd4;
            end

            // ---- |accel| through the sequential sqrt engine ----
            // States 5, 7, 10 and 12 wait on the ENGINES (sqrt_done, recip_done)
            // rather than on sample_valid. That is deliberate: the sensor pulse
            // is one clock wide while sqrt takes 33 and recip takes 29, so
            // gating the waits on sample_valid would stall the machine until the
            // next sample and make every result one sample stale.
            5'd4: begin
                sqrt_value <= {32'd0, abs16(accel_x)} * abs16(accel_x)
                            + {32'd0, abs16(accel_y)} * abs16(accel_y)
                            + {32'd0, abs16(accel_z)} * abs16(accel_z);
                sqrt_start <= 1'b1;
                st <= 5'd5;
            end

            5'd5: if (sqrt_done) begin
                mag32 <= sqrt_root;
                st <= 5'd6;
            end

            // ---- is the accelerometer usable for THIS sample? ----
            5'd6: begin
                if (mag32 > ACCEL_MIN && mag32 < ACCEL_MAX && mag32 != 0
                    && (abs16(gyro_x) + abs16(gyro_y) + abs16(gyro_z))
                       < GYRO_MOVE_LIMIT) begin
                    recip_d <= mag32;
                    recip_start <= 1'b1;
                    st <= 5'd7;
                end else begin
                    // Rotating, or |a| is off 1 g: the accelerometer is carrying
                    // kinematic acceleration, so integrate the gyro alone. This
                    // gate is what keeps the estimate sane while driving.
                    egx <= n1; egy <= n2; egz <= n3;
                    rejected_sample <= 1;
                    st <= 5'd9;
                end
            end

            // ---- wait for 1/|accel|, then NEGATE ----
            // unit = accel * (2^28/|a|) >> 16 gives F12 directly.
            // The accelerometer points UP at rest; projected_gravity points
            // DOWN. Dropping this minus shows the network an upside-down world,
            // silently: no flag, no fault.
            5'd7: if (recip_done) begin
                nx <= -((accel_x * recip_q) >>> 16);
                ny <= -((accel_y * recip_q) >>> 16);
                nz <= -((accel_z * recip_q) >>> 16);
                st <= 5'd8;
            end

            // ---- blend: g <- g + k*(accel_dir - g), k = ACCEL_TRUST/4096 ----
            // The rejection path skips this state and puts n1/n2/n3 directly
            // into egx/egy/egz, preserving gyro-only integration.
            5'd8: begin
                egx <= n1 + (((nx - n1) * ACCEL_TRUST) >>> 12);
                egy <= n2 + (((ny - n2) * ACCEL_TRUST) >>> 12);
                egz <= n3 + (((nz - n3) * ACCEL_TRUST) >>> 12);
                st <= 5'd9;
            end

            // ---- renormalize so the vector cannot drift off the unit sphere --
            // Components are F12 (|v| <= 4096), so each square is <= 2^24 and
            // the 64-bit sum cannot overflow.
            5'd9: begin
                sqrt_value <= abs24(egx) * abs24(egx)
                            + abs24(egy) * abs24(egy)
                            + abs24(egz) * abs24(egz);
                sqrt_start <= 1'b1;
                st <= 5'd10;
            end

            5'd10: if (sqrt_done) begin
                mag32 <= sqrt_root;
                st <= 5'd11;
            end

            5'd11: begin
                if (mag32 > 32'd64) begin
                    recip_d <= mag32;
                    recip_start <= 1'b1;
                    st <= 5'd12;
                end else begin
                    // Degenerate vector: hold the last good attitude rather than
                    // emitting a garbage direction.
                    st <= 5'd13;
                end
            end

            5'd12: if (recip_done) begin
                gravity_x <= (egx * recip_q) >>> 16;
                gravity_y <= (egy * recip_q) >>> 16;
                gravity_z <= (egz * recip_q) >>> 16;
                gx <= (egx * recip_q) >>> 16;
                gy <= (egy * recip_q) >>> 16;
                gz <= (egz * recip_q) >>> 16;
                st <= 5'd13;
            end

            5'd13: begin
                ang_vel_x <= avx;
                ang_vel_y <= avy;
                ang_vel_z <= avz;
                valid <= 1;
                accel_rejected <= rejected_sample;
                st <= 5'd2;
            end

            default: st <= 5'd0;
            endcase
        end
    end
endmodule

// Three exact signed64 /100 lanes, truncating toward zero. 64 clocks.
module imu_div100_3 #(parameter integer DIVISOR=100, parameter integer RB=$clog2(DIVISOR)+1)(input wire clk,rst_n,start,
    input wire signed [63:0] x,y,z,
    output reg signed [63:0] qx,qy,qz,output reg done);
    reg [63:0] dividend [0:2];
    reg [63:0] quotient [0:2];
    reg [RB-1:0] remainder [0:2];
    reg [2:0] negative;
    reg [6:0] count;
    reg busy;
    reg [RB-1:0] trial;
    reg [63:0] nextq;
    integer lane;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            qx<=0;qy<=0;qz<=0;done<=0;busy<=0;count<=0;negative<=0;
            for(lane=0;lane<3;lane=lane+1) begin dividend[lane]<=0;quotient[lane]<=0;remainder[lane]<=0;end
        end else begin
            done<=0;
            if(start && !busy) begin
                dividend[0]<=x[63] ? -x : x;
                dividend[1]<=y[63] ? -y : y;
                dividend[2]<=z[63] ? -z : z;
                negative<={z[63],y[63],x[63]};count<=64;busy<=1;
                for(lane=0;lane<3;lane=lane+1) begin quotient[lane]<=0;remainder[lane]<=0;end
            end
            if(busy) begin
                for(lane=0;lane<3;lane=lane+1) begin
                    trial={remainder[lane][RB-2:0],dividend[lane][63]};
                    nextq={quotient[lane][62:0],1'b0};
                    if(trial>=DIVISOR) begin trial=trial-DIVISOR;nextq[0]=1;end
                    remainder[lane]<=trial;quotient[lane]<=nextq;
                    dividend[lane]<={dividend[lane][62:0],1'b0};
                    if(count==1) begin
                        case(lane)
                            0:qx<=negative[lane] ? -$signed(nextq) : $signed(nextq);
                            1:qy<=negative[lane] ? -$signed(nextq) : $signed(nextq);
                            2:qz<=negative[lane] ? -$signed(nextq) : $signed(nextq);
                        endcase
                    end
                end
                count<=count-1;
                if(count==1) begin busy<=0;done<=1;end
            end
        end
    end
endmodule

`endif
