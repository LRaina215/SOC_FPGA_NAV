// Two 520 velocity loops. Targets: signed mm/s; feedback: signed cm/s.
// u_PWM = KFF*v_ref + KP*error + integral - KD*filtered_delta_velocity, all Q12.
// Update once per encoder sample (default100Hz); gains are PWM ticks per mm/s.
module motor520_closed_loop #(
    parameter integer LEFT_KP_Q12=10240, RIGHT_KP_Q12=10240, // 2.5
    parameter integer LEFT_KI_Q12=1434, RIGHT_KI_Q12=1434,  // .35/sample
    parameter integer LEFT_KD_Q12=8192, RIGHT_KD_Q12=8192, // 2.0 on measurement delta/sample
    parameter integer LEFT_KFF_Q12=3277, RIGHT_KFF_Q12=3277,// .8
    parameter integer PWM_PERIOD_CYCLES=2500,
    parameter integer DUTY_MAX=2400,
    parameter integer TARGET_LIMIT_MMPS=2000,
    parameter integer RAMP_STEP_MMPS=100,
    parameter USE_MMPS_FEEDBACK=0,DERIVATIVE_ALPHA_Q16=0,DERIVATIVE_FILTER_SHIFT=2,
    parameter CLOSED_LOOP_AT_ZERO=1'b0, // Legacy stand-alone default; main explicitly enables feedback braking.
    parameter integer FEEDBACK_TIMEOUT_CYCLES=10_000_000,
    parameter integer STALL_SAMPLES=100,
    parameter integer WRONG_DIRECTION_SAMPLES=10,
    parameter integer LEFT_DRIVE_SIGN=1,RIGHT_DRIVE_SIGN=1
)(
    input wire clk,reset_n,enable,
    input wire signed [31:0] target_left_mmps,target_right_mmps,
    input wire signed [15:0] speed_left_cms,speed_right_cms,
    input wire valid_left,valid_right,
    input wire signed [31:0] speed_left_mmps,speed_right_mmps,
    output wire pwm_left,dir_left1,dir_left2,pwm_right,dir_right1,dir_right2,
    output wire signed [15:0] duty_left,duty_right,
    output wire [2:0] fault_left,fault_right,
    output wire signed [31:0] reference_left_mmps,reference_right_mmps,error_left_mmps,error_right_mmps,
    output wire control_valid_left,control_valid_right
);
    motor520_velocity_pid #(.KP_Q12(LEFT_KP_Q12),.KI_Q12(LEFT_KI_Q12),.KD_Q12(LEFT_KD_Q12),.KFF_Q12(LEFT_KFF_Q12),
        .DUTY_MAX(DUTY_MAX),.TARGET_LIMIT_MMPS(TARGET_LIMIT_MMPS),.RAMP_STEP_MMPS(RAMP_STEP_MMPS),
        .CLOSED_LOOP_AT_ZERO(CLOSED_LOOP_AT_ZERO),.USE_MMPS_FEEDBACK(USE_MMPS_FEEDBACK),.DERIVATIVE_ALPHA_Q16(DERIVATIVE_ALPHA_Q16),.DERIVATIVE_FILTER_SHIFT(DERIVATIVE_FILTER_SHIFT),
        .FEEDBACK_TIMEOUT_CYCLES(FEEDBACK_TIMEOUT_CYCLES),.STALL_SAMPLES(STALL_SAMPLES),
        .WRONG_DIRECTION_SAMPLES(WRONG_DIRECTION_SAMPLES)) left_control(
        .clk(clk),.reset_n(reset_n),.enable(enable),.target_mmps(target_left_mmps),
        .feedback_cms(speed_left_cms),.feedback_precise_mmps(speed_left_mmps),.sample_valid(valid_left),.duty_ticks(duty_left),
        .ramped_target_mmps(reference_left_mmps),.error_mmps(error_left_mmps),
        .control_valid(control_valid_left),.fault_code(fault_left));
    motor520_velocity_pid #(.KP_Q12(RIGHT_KP_Q12),.KI_Q12(RIGHT_KI_Q12),.KD_Q12(RIGHT_KD_Q12),.KFF_Q12(RIGHT_KFF_Q12),
        .DUTY_MAX(DUTY_MAX),.TARGET_LIMIT_MMPS(TARGET_LIMIT_MMPS),.RAMP_STEP_MMPS(RAMP_STEP_MMPS),
        .CLOSED_LOOP_AT_ZERO(CLOSED_LOOP_AT_ZERO),.USE_MMPS_FEEDBACK(USE_MMPS_FEEDBACK),.DERIVATIVE_ALPHA_Q16(DERIVATIVE_ALPHA_Q16),.DERIVATIVE_FILTER_SHIFT(DERIVATIVE_FILTER_SHIFT),
        .FEEDBACK_TIMEOUT_CYCLES(FEEDBACK_TIMEOUT_CYCLES),.STALL_SAMPLES(STALL_SAMPLES),
        .WRONG_DIRECTION_SAMPLES(WRONG_DIRECTION_SAMPLES)) right_control(
        .clk(clk),.reset_n(reset_n),.enable(enable),.target_mmps(target_right_mmps),
        .feedback_cms(speed_right_cms),.feedback_precise_mmps(speed_right_mmps),.sample_valid(valid_right),.duty_ticks(duty_right),
        .ramped_target_mmps(reference_right_mmps),.error_mmps(error_right_mmps),
        .control_valid(control_valid_right),.fault_code(fault_right));
    motor520_pwm #(.PERIOD_CYCLES(PWM_PERIOD_CYCLES),.DRIVE_SIGN(LEFT_DRIVE_SIGN)) left_pwm(
        .clk(clk),.reset_n(reset_n),.enable(enable && (CLOSED_LOOP_AT_ZERO || target_left_mmps!=0) && fault_left==0),
        .duty_ticks(duty_left),.pwm(pwm_left),.dir1(dir_left1),.dir2(dir_left2));
    motor520_pwm #(.PERIOD_CYCLES(PWM_PERIOD_CYCLES),.DRIVE_SIGN(RIGHT_DRIVE_SIGN)) right_pwm(
        .clk(clk),.reset_n(reset_n),.enable(enable && (CLOSED_LOOP_AT_ZERO || target_right_mmps!=0) && fault_right==0),
        .duty_ticks(duty_right),.pwm(pwm_right),.dir1(dir_right1),.dir2(dir_right2));
endmodule

module motor520_velocity_pid #(
    parameter integer KP_Q12=10240,KI_Q12=1434,KD_Q12=8192,KFF_Q12=3277,
    parameter integer DERIVATIVE_FILTER_SHIFT=2,D_MAX_TICKS=250,
    parameter integer DUTY_MAX=2400,TARGET_LIMIT_MMPS=2000,RAMP_STEP_MMPS=100,
    parameter integer FEEDBACK_TIMEOUT_CYCLES=10_000_000,
    parameter integer STALL_SAMPLES=100,WRONG_DIRECTION_SAMPLES=10,
    parameter USE_MMPS_FEEDBACK=0,DERIVATIVE_ALPHA_Q16=0,
    parameter CLOSED_LOOP_AT_ZERO=1'b0
)(
    input wire clk,reset_n,enable,
    input wire signed [31:0] target_mmps,
    input wire signed [15:0] feedback_cms,
    input wire signed [31:0] feedback_precise_mmps,
    input wire sample_valid,
    output reg signed [15:0] duty_ticks,
    output reg signed [31:0] ramped_target_mmps,error_mmps,
    output reg control_valid,
    output reg [2:0] fault_code // 1 missing samples;2 stall;3 opposite motion; clear with disable/reset (legacy also zero)
);
    localparam IDLE=0,MULTIPLY=1,ACCUMULATE=2,LIMIT_INTEGRAL=3,TRIAL=4,SELECT=5,CLAMP=6,COMMIT=7;
    localparam signed [63:0] OUTPUT_LIMIT_Q=64'sd1*DUTY_MAX*4096;
    localparam signed [63:0] D_LIMIT_Q=64'sd1*D_MAX_TICKS*4096;
    reg [2:0] state;
    reg signed [63:0] integral_q,p_q,i_step_q,ff_q,output_q;
    reg signed [63:0] d_q,d_clamped_q,pf_q;
    reg signed [31:0] previous_feedback,derivative_q8,d_input_q8;
    reg measurement_seen;
    reg signed [63:0] integral_candidate_raw,integral_candidate,base_q,trial_output,requested_output;
    reg [31:0] feedback_age,stall_count,opposite_count;
    reg signed [31:0] reference;
    wire signed [31:0] feedback_mmps=USE_MMPS_FEEDBACK ? feedback_precise_mmps : $signed(feedback_cms)*32'sd10;
    // D is on measured speed, so changing the target causes no derivative kick.
    // Keep8 fractional bits; alpha=1/4 per10ms sample reduces encoder quantization noise.
    wire signed [31:0] delta_q8=(feedback_mmps-previous_feedback)<<<8;
    wire signed [63:0] derivative_correction=($signed(delta_q8)-$signed(derivative_q8))*$signed(DERIVATIVE_ALPHA_Q16);
    wire signed [31:0] derivative_next=measurement_seen ?
        (derivative_q8+(DERIVATIVE_ALPHA_Q16!=0 ? (derivative_correction>>>16) : ((delta_q8-derivative_q8)>>>DERIVATIVE_FILTER_SHIFT))) : 32'sd0;
    wire signed [31:0] bounded_target=(target_mmps>TARGET_LIMIT_MMPS) ? TARGET_LIMIT_MMPS :
        ((target_mmps< -TARGET_LIMIT_MMPS) ? -TARGET_LIMIT_MMPS : target_mmps);
    wire signed [31:0] next_reference=(bounded_target>ramped_target_mmps+RAMP_STEP_MMPS) ?
        ramped_target_mmps+RAMP_STEP_MMPS :
        ((bounded_target<ramped_target_mmps-RAMP_STEP_MMPS) ? ramped_target_mmps-RAMP_STEP_MMPS : bounded_target);
    wire [15:0] duty_abs=duty_ticks[15] ? (~duty_ticks+16'd1) : duty_ticks;
    wire low_speed=(feedback_cms>=-2 && feedback_cms<=2);
    wire running_reference=(next_reference>=200 || next_reference<=-200);
    wire opposite_motion=(next_reference>=200 && feedback_cms<=-5) ||
        (next_reference<=-200 && feedback_cms>=5);
    // Split add/limit/anti-windup across clocks to meet50MHz; all feedback freezes in IDLE.
    wire hold_integral=(trial_output>OUTPUT_LIMIT_Q && error_mmps>0) ||
        (trial_output< -OUTPUT_LIMIT_Q && error_mmps<0);
    wire signed [63:0] rounded_ticks=(output_q<0) ? -((-output_q+64'sd2048)>>>12) :
        ((output_q+64'sd2048)>>>12);
    always @(posedge clk) begin
        control_valid<=0;
        if(!reset_n || !enable || (!CLOSED_LOOP_AT_ZERO && target_mmps==0)) begin
            state<=IDLE;duty_ticks<=0;ramped_target_mmps<=0;error_mmps<=0;reference<=0;
            integral_q<=0;p_q<=0;i_step_q<=0;ff_q<=0;output_q<=0;
            d_q<=0;d_clamped_q<=0;pf_q<=0;previous_feedback<=0;
            derivative_q8<=0;d_input_q8<=0;measurement_seen<=0;
            integral_candidate_raw<=0;integral_candidate<=0;base_q<=0;trial_output<=0;requested_output<=0;
            feedback_age<=0;stall_count<=0;opposite_count<=0;fault_code<=0;
        end else if(fault_code!=0) begin duty_ticks<=0;state<=IDLE;integral_q<=0;end
        else if(!sample_valid && feedback_age>=FEEDBACK_TIMEOUT_CYCLES-1) begin
            fault_code<=1;duty_ticks<=0;integral_q<=0;state<=IDLE;
        end else begin
            if(sample_valid) feedback_age<=0;else feedback_age<=feedback_age+1'b1;
            case(state)
                IDLE:if(sample_valid) begin
                    if(STALL_SAMPLES>0 && running_reference && low_speed && duty_abs>=500 && stall_count>=STALL_SAMPLES-1) begin
                        fault_code<=2;duty_ticks<=0;integral_q<=0;
                    end else if(WRONG_DIRECTION_SAMPLES>0 && opposite_motion && opposite_count>=WRONG_DIRECTION_SAMPLES-1) begin
                        fault_code<=3;duty_ticks<=0;integral_q<=0;
                    end else begin
                        if(running_reference && low_speed && duty_abs>=500) stall_count<=stall_count+1'b1;
                        else stall_count<=0;
                        if(opposite_motion) opposite_count<=opposite_count+1'b1;else opposite_count<=0;
                        ramped_target_mmps<=next_reference;reference<=next_reference;
                        error_mmps<=next_reference-feedback_mmps;
                        previous_feedback<=feedback_mmps;measurement_seen<=1;
                        derivative_q8<=derivative_next;d_input_q8<=derivative_next;state<=MULTIPLY;
                    end
                end
                MULTIPLY:begin
                    p_q<=$signed(error_mmps)*$signed(KP_Q12);
                    i_step_q<=$signed(error_mmps)*$signed(KI_Q12);
                    ff_q<=$signed(reference)*$signed(KFF_Q12);
                    d_q<=(-($signed(d_input_q8)*$signed(KD_Q12)))>>>8;state<=ACCUMULATE;
                end
                ACCUMULATE:begin
                    // Bleed stored load compensation only near a genuine zero-speed request.
                    // Preserve measurement history and braking P/D; never reset at sign crossings.
                    if(CLOSED_LOOP_AT_ZERO && reference==0 && target_mmps==0 &&
                        previous_feedback>=-10 && previous_feedback<=10)
                        integral_candidate_raw<=integral_q-(integral_q>>>3);
                    else integral_candidate_raw<=integral_q+i_step_q;
                    pf_q<=p_q+ff_q;
                    if(d_q>D_LIMIT_Q) d_clamped_q<=D_LIMIT_Q;
                    else if(d_q< -D_LIMIT_Q) d_clamped_q<= -D_LIMIT_Q;
                    else d_clamped_q<=d_q;
                    state<=LIMIT_INTEGRAL;
                end
                LIMIT_INTEGRAL:begin
                    base_q<=pf_q+d_clamped_q;
                    if(integral_candidate_raw>OUTPUT_LIMIT_Q) integral_candidate<=OUTPUT_LIMIT_Q;
                    else if(integral_candidate_raw< -OUTPUT_LIMIT_Q) integral_candidate<= -OUTPUT_LIMIT_Q;
                    else integral_candidate<=integral_candidate_raw;
                    state<=TRIAL;
                end
                TRIAL:begin trial_output<=base_q+integral_candidate;state<=SELECT;end
                SELECT:begin
                    if(hold_integral) requested_output<=base_q+integral_q;
                    else begin integral_q<=integral_candidate;requested_output<=trial_output;end
                    state<=CLAMP;
                end
                CLAMP:begin
                    if(requested_output>OUTPUT_LIMIT_Q) output_q<=OUTPUT_LIMIT_Q;
                    else if(requested_output< -OUTPUT_LIMIT_Q) output_q<= -OUTPUT_LIMIT_Q;
                    else output_q<=requested_output;
                    state<=COMMIT;
                end
                COMMIT:begin duty_ticks<=rounded_ticks[15:0];control_valid<=1;state<=IDLE;end
            endcase
        end
    end
endmodule

// Center-aligned changes are unnecessary here: duty is latched only at carrier boundaries.
// On a sign reversal, coast for a whole carrier period before applying the new direction.
module motor520_pwm #(
    parameter integer PERIOD_CYCLES=2500,DRIVE_SIGN=1
)(
    input wire clk,reset_n,enable,
    input wire signed [15:0] duty_ticks,
    output wire pwm,dir1,dir2
);
    reg [31:0] counter;
    reg [15:0] active_magnitude;
    reg active_reverse;
    wire requested_reverse=duty_ticks[15] ^ (DRIVE_SIGN<0);
    wire [15:0] magnitude=duty_ticks[15] ? (~duty_ticks+16'd1) : duty_ticks;
    wire [15:0] bounded_magnitude=(magnitude>PERIOD_CYCLES) ? PERIOD_CYCLES : magnitude;
    assign pwm=reset_n && enable && active_magnitude!=0 && counter<active_magnitude;
    assign dir1=reset_n && enable && active_magnitude!=0 && !active_reverse;
    assign dir2=reset_n && enable && active_magnitude!=0 && active_reverse;
    always @(posedge clk) begin
        if(!reset_n || !enable) begin counter<=0;active_magnitude<=0;active_reverse<=0;end
        else if(counter==PERIOD_CYCLES-1) begin
            counter<=0;
            if(active_magnitude!=0 && requested_reverse!=active_reverse) active_magnitude<=0;
            else begin active_magnitude<=bounded_magnitude;active_reverse<=requested_reverse;end
        end else counter<=counter+1'b1;
    end
endmodule
