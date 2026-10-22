// Integrated .mem policy: six raw BMI axes -> fusion -> observations/history
// -> parameterized inference -> physical leg PWM commands and wheel velocity targets.
// All command ports are signed Q12: real_value * 4096. No floating-point HW.
module mem_policy #(
    parameter integer WEIGHT_BITS=16,
    parameter integer BIAS_SAMPLES=200,
    parameter integer PERIOD_CLKS=400000,
    parameter integer SENSOR_TIMEOUT_CLKS=2500000,
    parameter integer LH_ZERO_US=1500,LK_ZERO_US=1500,RH_ZERO_US=1500,RK_ZERO_US=1500
)(
    input wire clk,rst_n,enable,sensor_ready,sensor_error,sample_valid,external_fault,
    input wire signed [15:0] accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,
    // Raw AB x4 deltas, BEFORE display/physical direction correction; 10ms gate.
    input wire signed [31:0] left_encoder_delta,right_encoder_delta,
    input wire signed [23:0] command_forward_q12,command_yaw_q12,command_height_q12,
    output wire signed [31:0] target_left_mmps,target_right_mmps,
    output wire [15:0] servo_lh_us,servo_lk_us,servo_rh_us,servo_rk_us,
    output wire calibrating,bias_suspect,accel_rejected,
    output wire ready,active,action_valid,
    output wire [7:0] status_flags,
    output wire [2:0] policy_fault,
    output wire [31:0] completed_frames,inference_cycles,
    output wire [143:0] executed_actions,
    output wire debug_fusion_valid,
    output wire signed [23:0] debug_wx,debug_wy,debug_wz,debug_gx,debug_gy,debug_gz
);
    wire signed [23:0] wx,wy,wz,gx,gy,gz;
    assign debug_wx=wx;assign debug_wy=wy;assign debug_wz=wz;
    assign debug_gx=gx;assign debug_gy=gy;assign debug_gz=gz;
    wire fusion_valid;
    assign debug_fusion_valid=fusion_valid;
    imu_fusion #(.BIAS_SAMPLES(BIAS_SAMPLES),.GYRO_MUL(32'sd4466)) fusion(
        .clk(clk),.rst_n(rst_n),.sample_valid(sample_valid),
        .accel_x(accel_x),.accel_y(accel_y),.accel_z(accel_z),
        .gyro_x(gyro_x),.gyro_y(gyro_y),.gyro_z(gyro_z),
        .ang_vel_x(wx),.ang_vel_y(wy),.ang_vel_z(wz),
        .gravity_x(gx),.gravity_y(gy),.gravity_z(gz),.valid(fusion_valid),
        .calibrating(calibrating),.bias_suspect(bias_suspect),.accel_rejected(accel_rejected));
    reg [31:0] sensor_age;
    reg fusion_seen,stop_latched;
    wire sensor_ok=sensor_ready && !sensor_error && !calibrating && !bias_suspect &&
                   sensor_age<SENSOR_TIMEOUT_CLKS;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin sensor_age<=SENSOR_TIMEOUT_CLKS;fusion_seen<=0;stop_latched<=0;end
        else begin
            if(sample_valid) sensor_age<=0;
            else if(sensor_age<SENSOR_TIMEOUT_CLKS) sensor_age<=sensor_age+1'b1;
            if(fusion_valid && !calibrating) fusion_seen<=1;
            if(!enable) stop_latched<=0;
            else if(fusion_seen && (!sensor_ok || external_fault || policy_fault!=0)) stop_latched<=1;
        end
    end
    assign ready=fusion_seen && sensor_ok && !stop_latched && policy_fault==0 && !external_fault;
    wire run=enable && ready;
    wire have_action;
    assign active=run && have_action;
    // flags[0..7]: BMI ready, calibrating, suspect bias, stale sample,
    //              policy fault, actuator fault, latched stop, active.
    assign status_flags={active,stop_latched,external_fault,(policy_fault!=0),
                         (sensor_age>=SENSOR_TIMEOUT_CLKS),bias_suspect,calibrating,sensor_ready};
    wire capture,frame_valid,history_initialized;
    wire [671:0] current_flat;
    wire [3359:0] history_flat;
    // Clamp height once, share the SAME command between observation and IK.
    wire signed [23:0] height=command_height_q12<348 ? 24'sd348 :
                             command_height_q12>778 ? 24'sd778 : command_height_q12;
    reg [23:0] captured_height;
    always @(posedge clk or negedge rst_n)
        if(!rst_n) captured_height<=24'd535;
        else if(capture) captured_height<=height;
    function signed [15:0] delta16;
        input signed [31:0] x;
        begin delta16=x>32767 ? 16'sd32767 : x < -32768 ? -16'sd32768 : x[15:0];end
    endfunction
    rl_observation_v5 #(.LEFT_ENCODER_SIGN(-1),.RIGHT_ENCODER_SIGN(-1)) observation(
        .clk(clk),.rst_n(rst_n && enable && !stop_latched),.capture(capture),
        .angular_velocity_q12({wz,wy,wx}),.gravity_q12({gz,gy,gx}),
        .left_delta(delta16(left_encoder_delta)),.right_delta(delta16(right_encoder_delta)),
        .linear_command_q12(command_forward_q12),.yaw_command_q12(command_yaw_q12),
        .height_command_q12(height),.last_executed_actions(executed_actions),
        .valid(frame_valid),.initialized(history_initialized),
        .current_flat(current_flat),.history_flat(history_flat));
    wire [143:0] actions;
    rl_policy_tick_v5 #(.WEIGHT_BITS(WEIGHT_BITS),.PERIOD_CLKS(PERIOD_CLKS)) policy(
        .clk(clk),.rst_n(rst_n),.enable(enable && fusion_seen),
        .sensor_ok(sensor_ok && !external_fault && !stop_latched),
        .frame_valid(frame_valid),.current_flat(current_flat),.history_flat(history_flat),
        .capture(capture),.action_valid(action_valid),.actions(actions),
        .completed_frames(completed_frames),.inference_cycles(inference_cycles),.fault(policy_fault),
        .diag_last_fault(),.diag_core_detail(),.diag_fault_count(),.diag_retry_count(),
        .diag_expected_id(),.diag_result_id());
    wire signed [11:0] ff_lh,ff_lk,ff_rh,ff_rk;
    leg_ik_ff ik(.clk(clk),.height_q12(captured_height),
        .ff_lh(ff_lh),.ff_lk(ff_lk),.ff_rh(ff_rh),.ff_rk(ff_rk),.idx_out());
    policy_action_map #(.LH_ZERO_US(LH_ZERO_US),.LK_ZERO_US(LK_ZERO_US),
        .RH_ZERO_US(RH_ZERO_US),.RK_ZERO_US(RK_ZERO_US)) actuator_map(
        .clk(clk),.rst_n(rst_n),.run(run),.action_valid(action_valid),.actions(actions),
        .ff_lh(ff_lh),.ff_lk(ff_lk),.ff_rh(ff_rh),.ff_rk(ff_rk),
        .left_mmps(target_left_mmps),.right_mmps(target_right_mmps),
        .lh_us(servo_lh_us),.lk_us(servo_lk_us),.rh_us(servo_rh_us),.rk_us(servo_rk_us),
        .last_actions(executed_actions),.have_action(have_action));
endmodule
