// model_4500 + BMI088 + servo IK/PWM + 520 velocity PID + OLED;50MHz.
module control4500_core #(
    parameter NAV_INPUT_ENABLE=1'b1,
    parameter UART_SPEED_ENABLE=1'b0, // UART_RXD接收车体目标线速度/角速度，首个有效帧后接管两者
    parameter IR_ENABLE=1'b1, // 2/8前后、4/6转向、+/-升降、5急停
    parameter real MODEL_WHEEL_SPEED_GAIN=1.0, // 原始4500模型轮速1倍
    parameter MODEL_WHEEL_FILTER_ENABLE=1'b0, // 恢复4500原版参数：关闭新增轮速目标延迟/滤波
    parameter FIXED_FORWARD_ENABLE=1'b0, // 1=使用常量前进指令；0=红外前后速度
    // ===== 模型/指令接口：修改这里即可，运行时接口在下方 command_* 信号 =====
    parameter OLED_MODEL_DEBUG=1'b1, // 首行显示模型状态；0恢复轮速标题
    parameter MODEL_ENABLE=1'b1,      // 1=当前ROM模型；0=原先手动轮速/舵机接口
    parameter integer MODEL_WEIGHT_BITS=16, // 原始4500：16位F12权重
    parameter CONTROL_ENABLE=1'b1,    // 总运行开关，0停轮；舵机保持最后姿态
    parameter EMERGENCY_STOP=1'b0,    // 编译期急停；遥控5急停锁定后需硬件复位
    parameter real COMMAND_FORWARD_MPS=0.00, // 前进m/s，负数后退；无固定后退偏置
    parameter real COMMAND_YAW_RADPS=0.00,   // 训练坐标系转向rad/s
    parameter real COMMAND_HEIGHT_M=0.1305,  // 底盘高度m，限幅0.085~0.19
    parameter integer IMU_BIAS_SAMPLES=200,  // 上电约2秒静止标定
    // ===== 用户速度接口：直接填m/s，正数向前，负数向后，0停止 =====
    parameter real TARGET_LEFT_MPS=0.00,
    parameter real TARGET_RIGHT_MPS=0.00,
    parameter MOTOR_ENABLE=1'b1,
    // Gains are PWM ticks/mm/s; KI is the increment per10ms sample.
    // 恢复4500原版速度环：10ms计数测速，PID每10ms更新。
    parameter real LEFT_KP=3.00,LEFT_KI=0.35,LEFT_KD=2.00,LEFT_KFF=0.80,
    parameter real RIGHT_KP=3.00,RIGHT_KI=0.35,RIGHT_KD=2.00,RIGHT_KFF=0.80,
    parameter integer MOTOR_DUTY_MAX=2400, // 96%; old50% limit capped measured speed at~0.5m/s
    parameter integer MOTOR_RAMP_STEP_MMPS=100, // 原版每10ms变化0.1m/s
    parameter MOTOR_CLOSED_LOOP_AT_ZERO=1'b0, // 恢复原版零目标关PWM行为
    parameter integer TELEMETRY_FRAME_HZ=100,
    parameter integer UART_CLKS_PER_BIT=217, // 50MHz /217 ~=230400baud; old bitstream uses115200
    // ===== 四路舵机PWM控制接口（µs），默认1500µs中位 =====
    parameter [15:0] SERVO_LH_US=16'd1500, // 舵机1左髋
    parameter [15:0] SERVO_LK_US=16'd1500, // 舵机2左膝
    parameter [15:0] SERVO_RH_US=16'd1500, // 舵机3右髋
    parameter [15:0] SERVO_RK_US=16'd1500, // 舵机4右膝
    parameter SERVO_ENABLE=1'b1,
    parameter SERVO_APPLY_DIRECTION=1'b1 // 上层已做方向映射时改0
)( 
    input wire clk,reset_n,
    input wire signed [23:0] nav_forward_q12,nav_yaw_q12,
    output wire signed [15:0] feedback_vx_mmps,feedback_wz_mradps,
    output wire feedback_valid,
    input wire UART_RXD, // P11-39 / A15; UART target chassis speed RX
    output wire UART_TXD, // P11-40 / A16
    input wire IRIN, // Reference IR receiver: A21, idle high
    inout wire sck,sda,        // OLED SCL F19 / SDA F20, P11 pins5/6
    inout wire BMI088_SCL,     // E18, P11 pin8
    inout wire BMI088_SDA,     // F18, P11 pin7
    input wire M1_ENCA,M1_ENCB, // Right wheel: D14/F13, P11 pins15/17
    input wire M2_ENCA,M2_ENCB, // Left wheel: D15/F14, P11 pins16/18
    output wire M1_PWM,M1_DIR1,M1_DIR2, // Right: D17/B20/B17
    output wire M2_PWM,M2_DIR1,M2_DIR2, // Left: C17/B18/A20
    output wire SERVO1,SERVO2,SERVO3,SERVO4 // LH C18 / LK C19 / RH E19 / RK D19
);
    reg [7:0] startup=0;
    always @(posedge clk) begin
        if(!reset_n) startup<=0;
        else if(!(&startup)) startup<=startup+1'b1;
    end
    wire device_reset_n=reset_n && (&startup);
    // ===== 后续接UART/按键/上位机时，将下列常量替换成寄存器或模块输出 =====
    localparam signed [23:0] FORWARD_Q12=$rtoi(COMMAND_FORWARD_MPS*4096.0+(COMMAND_FORWARD_MPS<0 ? -0.5 : 0.5));
    localparam signed [23:0] YAW_Q12=$rtoi(COMMAND_YAW_RADPS*4096.0+(COMMAND_YAW_RADPS<0 ? -0.5 : 0.5));
    localparam signed [23:0] HEIGHT_Q12=$rtoi(COMMAND_HEIGHT_M*4096.0+0.5);
    wire command_run=CONTROL_ENABLE;
    wire ir_emergency_stop;
    wire command_emergency_stop=EMERGENCY_STOP || (IR_ENABLE && ir_emergency_stop);
    wire control_run=command_run && !command_emergency_stop;
    wire signed [23:0] ir_forward_q12,ir_yaw_q12,ir_height_q12;
    wire [7:0] ir_key_code;
    wire ir_key_active,ir_key_event,ir_repeat_event;
    ir_remote_control #(.HEIGHT_INITIAL_Q12(HEIGHT_Q12)) remote_thread(
        .clk(clk),.reset_n(device_reset_n),.ir_in(IRIN),
        .forward_q12(ir_forward_q12),.yaw_q12(ir_yaw_q12),
        .height_q12(ir_height_q12),.emergency_stop(ir_emergency_stop),
        .key_code(ir_key_code),.key_active(ir_key_active),
        .key_event(ir_key_event),.repeat_event(ir_repeat_event));
    wire signed [23:0] uart_forward_q12,uart_yaw_q12;
    wire uart_speed_seen,uart_speed_valid;
    uart_speed_rx #(.CLKS_PER_BIT(UART_CLKS_PER_BIT)) speed_command_rx(
        .clk(clk),.reset_n(device_reset_n),.rx(UART_RXD),
        .forward_q12(uart_forward_q12),.yaw_q12(uart_yaw_q12),.command_seen(uart_speed_seen),.command_valid(uart_speed_valid));
    wire signed [23:0] command_forward_q12=command_emergency_stop ? 24'sd0 : NAV_INPUT_ENABLE ? nav_forward_q12 : FIXED_FORWARD_ENABLE ? FORWARD_Q12 :
        (UART_SPEED_ENABLE && uart_speed_seen) ? uart_forward_q12 :
        (IR_ENABLE ? ir_forward_q12 : FORWARD_Q12);
    wire signed [23:0] command_yaw_q12=command_emergency_stop ? 24'sd0 : NAV_INPUT_ENABLE ? nav_yaw_q12 :
        (UART_SPEED_ENABLE && uart_speed_seen) ? uart_yaw_q12 : (IR_ENABLE ? ir_yaw_q12 : YAW_Q12);
    wire signed [23:0] command_height_q12=IR_ENABLE ? ir_height_q12 : HEIGHT_Q12;
    wire signed [31:0] model_left_mmps,model_right_mmps;
    // Scale model wheel speeds only while a live forward/backward key is held.
    // Release, turn and idle use the original model output; servos are unchanged.
    wire amplify_model_wheels=IR_ENABLE && !FIXED_FORWARD_ENABLE &&
        !(UART_SPEED_ENABLE && uart_speed_seen) && ir_key_active &&
        !command_emergency_stop && (ir_key_code==8'h18 || ir_key_code==8'h52);
    localparam signed [31:0] MODEL_WHEEL_GAIN_Q12=$rtoi(MODEL_WHEEL_SPEED_GAIN*4096.0+0.5);
    function signed [31:0] amplified_model_mmps;
        input signed [31:0] speed_mmps;
        reg signed [63:0] product,rounded;
        begin
            product=$signed(speed_mmps)*$signed(MODEL_WHEEL_GAIN_Q12);
            rounded=product<0 ? -((-product+64'sd2048)>>>12) : ((product+64'sd2048)>>>12);
            amplified_model_mmps=rounded>600 ? 32'sd600 :
                rounded < -600 ? -32'sd600 : rounded[31:0];
        end
    endfunction
    wire signed [31:0] wheel_left_target_mmps=MODEL_ENABLE ?
        (amplify_model_wheels ? amplified_model_mmps(model_left_mmps) : model_left_mmps) : LEFT_TARGET_MMPS;
    wire signed [31:0] wheel_right_target_mmps=MODEL_ENABLE ?
        (amplify_model_wheels ? amplified_model_mmps(model_right_mmps) : model_right_mmps) : RIGHT_TARGET_MMPS;
    wire [15:0] model_lh_us,model_lk_us,model_rh_us,model_rk_us;
    wire model_ready,model_active,model_action_valid,imu_calibrating,imu_bias_suspect,imu_accel_rejected;
    wire [7:0] model_status_flags;
    wire [2:0] model_fault;
    wire [31:0] model_completed_frames,model_inference_cycles;
    wire [143:0] model_executed_actions;
    wire debug_fusion_valid;
    wire signed [23:0] debug_wx,debug_wy,debug_wz,debug_gx,debug_gy,debug_gz;
    // 以后可以将这里的常量替换为寄存器、角度计算结果或模型输出。
    wire [15:0] servo_lh_command_us=MODEL_ENABLE ? model_lh_us : SERVO_LH_US;
    wire [15:0] servo_lk_command_us=MODEL_ENABLE ? model_lk_us : SERVO_LK_US;
    wire [15:0] servo_rh_command_us=MODEL_ENABLE ? model_rh_us : SERVO_RH_US;
    wire [15:0] servo_rk_command_us=MODEL_ENABLE ? model_rk_us : SERVO_RK_US;
    servo_driver #(.LH_SIGN(1),.LK_SIGN(-1),.RH_SIGN(1),.RK_SIGN(1),
        .APPLY_CALIBRATED_DIRECTION(MODEL_ENABLE ? 0 : SERVO_APPLY_DIRECTION)) servo_thread(
        .clk(clk),.reset_n(device_reset_n),.enable(SERVO_ENABLE),
        .pulse_lh_us(servo_lh_command_us),.pulse_lk_us(servo_lk_command_us),
        .pulse_rh_us(servo_rh_command_us),.pulse_rk_us(servo_rk_command_us),
        .SERVO1(SERVO1),.SERVO2(SERVO2),.SERVO3(SERVO3),.SERVO4(SERVO4),
        .frame_tick(),.active_lh_us(),.active_lk_us(),.active_rh_us(),.active_rk_us());
    wire sensor_ready,sensor_error,sample_valid;
    wire [3:0] sensor_error_code;
    wire signed [15:0] accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z;
    bmi088 sensor(
        .clk(clk),.reset_n(device_reset_n),.IIC_SCL(BMI088_SCL),.IIC_SDA(BMI088_SDA),
        .init_done(sensor_ready),.init_error(sensor_error),.data_valid(sample_valid),
        .accel_x(accel_x),.accel_y(accel_y),.accel_z(accel_z),
        .gyro_x(gyro_x),.gyro_y(gyro_y),.gyro_z(gyro_z),.error_code(sensor_error_code),
        .operation(),.accel_addr(),.gyro_addr(),.accel_id(),.gyro_id(),.last_read());

    // ===== 520轮速参数：原工程已确认的轮径、减速比与编码器计数 =====
    localparam integer ENCODER_CPR=1320; // 11*4*30 counts per wheel turn
    localparam integer WHEEL_DIAMETER_UM=65000;
    wire signed [15:0] left_speed_cms,right_speed_cms;
    wire left_speed_valid,right_speed_valid;
    wire signed [31:0] left_encoder_delta,right_encoder_delta,left_position,right_position;
    motor520_speed #(.ENCODER_CPR(ENCODER_CPR),.WHEEL_DIAMETER_UM(WHEEL_DIAMETER_UM),
        .ENCODER_SIGN(1)) wheel_left(
        .clk(clk),.reset_n(device_reset_n),.enc_a(M2_ENCA),.enc_b(M2_ENCB),
        .speed_cms(left_speed_cms),.sample_valid(left_speed_valid),.position(left_position),.speed_delta(left_encoder_delta),.invalid_transitions());
    motor520_speed #(.ENCODER_CPR(ENCODER_CPR),.WHEEL_DIAMETER_UM(WHEEL_DIAMETER_UM),
        .ENCODER_SIGN(-1)) wheel_right(
        .clk(clk),.reset_n(device_reset_n),.enc_a(M1_ENCA),.enc_b(M1_ENCB),
        .speed_cms(right_speed_cms),.sample_valid(right_speed_valid),.position(right_position),.speed_delta(right_encoder_delta),.invalid_transitions());

    // Original4500 path: the same10ms count speed feeds policy observations and PID.

    // real parameters are converted at compilation only; the FPGA runs integer Q12 arithmetic.
    localparam signed [31:0] LEFT_TARGET_MMPS=$rtoi(TARGET_LEFT_MPS*1000.0+(TARGET_LEFT_MPS<0 ? -0.5 : 0.5));
    localparam signed [31:0] RIGHT_TARGET_MMPS=$rtoi(TARGET_RIGHT_MPS*1000.0+(TARGET_RIGHT_MPS<0 ? -0.5 : 0.5));
    localparam integer L_KP_Q12=$rtoi(LEFT_KP*4096.0+0.5),L_KI_Q12=$rtoi(LEFT_KI*4096.0+0.5),L_KFF_Q12=$rtoi(LEFT_KFF*4096.0+0.5);
    localparam integer R_KP_Q12=$rtoi(RIGHT_KP*4096.0+0.5),R_KI_Q12=$rtoi(RIGHT_KI*4096.0+0.5),R_KFF_Q12=$rtoi(RIGHT_KFF*4096.0+0.5);
    localparam integer L_KD_Q12=$rtoi(LEFT_KD*4096.0+0.5),R_KD_Q12=$rtoi(RIGHT_KD*4096.0+0.5);
    wire signed [15:0] left_duty,right_duty;
    wire signed [31:0] left_pid_reference,right_pid_reference,left_pid_error,right_pid_error;
    wire left_control_valid,right_control_valid;
    wire [2:0] left_motor_fault,right_motor_fault;
    wire wheel_control_enable=MOTOR_ENABLE && control_run && (!MODEL_ENABLE || model_active);
    wire signed [31:0] filtered_model_left_mmps,filtered_model_right_mmps;
    model_wheel_response model_wheel_filter(
        .clk(clk),.reset_n(device_reset_n),
        .enable(MODEL_ENABLE && MODEL_WHEEL_FILTER_ENABLE && wheel_control_enable),
        .left_target_mmps(wheel_left_target_mmps),.right_target_mmps(wheel_right_target_mmps),
        .left_filtered_mmps(filtered_model_left_mmps),.right_filtered_mmps(filtered_model_right_mmps));
    wire signed [31:0] wheel_left_pid_target_mmps=(MODEL_ENABLE && MODEL_WHEEL_FILTER_ENABLE) ?
        filtered_model_left_mmps : wheel_left_target_mmps;
    wire signed [31:0] wheel_right_pid_target_mmps=(MODEL_ENABLE && MODEL_WHEEL_FILTER_ENABLE) ?
        filtered_model_right_mmps : wheel_right_target_mmps;
    motor520_closed_loop #(
        .LEFT_KP_Q12(L_KP_Q12),.LEFT_KI_Q12(L_KI_Q12),.LEFT_KD_Q12(L_KD_Q12),.LEFT_KFF_Q12(L_KFF_Q12),
        .RIGHT_KP_Q12(R_KP_Q12),.RIGHT_KI_Q12(R_KI_Q12),.RIGHT_KD_Q12(R_KD_Q12),.RIGHT_KFF_Q12(R_KFF_Q12),
        .DUTY_MAX(MOTOR_DUTY_MAX),.RAMP_STEP_MMPS(MOTOR_RAMP_STEP_MMPS),
        .USE_MMPS_FEEDBACK(0),.DERIVATIVE_ALPHA_Q16(0),.DERIVATIVE_FILTER_SHIFT(2),
        .STALL_SAMPLES(100),.WRONG_DIRECTION_SAMPLES(10),
        .CLOSED_LOOP_AT_ZERO(MOTOR_CLOSED_LOOP_AT_ZERO),.LEFT_DRIVE_SIGN(1),.RIGHT_DRIVE_SIGN(1)
    ) wheel_control(
        .clk(clk),.reset_n(device_reset_n),.enable(wheel_control_enable),
        .target_left_mmps(wheel_left_pid_target_mmps),
        .target_right_mmps(wheel_right_pid_target_mmps),
        .speed_left_cms(left_speed_cms),.speed_right_cms(right_speed_cms),
        .speed_left_mmps(32'sd0),.speed_right_mmps(32'sd0),
        .valid_left(left_speed_valid),.valid_right(right_speed_valid),
        .pwm_left(M2_PWM),.dir_left1(M2_DIR1),.dir_left2(M2_DIR2),
        .pwm_right(M1_PWM),.dir_right1(M1_DIR1),.dir_right2(M1_DIR2),
        .duty_left(left_duty),.duty_right(right_duty),
        .reference_left_mmps(left_pid_reference),.reference_right_mmps(right_pid_reference),
        .error_left_mmps(left_pid_error),.error_right_mmps(right_pid_error),
        .control_valid_left(left_control_valid),.control_valid_right(right_control_valid),
        .fault_left(left_motor_fault),.fault_right(right_motor_fault));

    mem_policy #(.WEIGHT_BITS(MODEL_WEIGHT_BITS),.BIAS_SAMPLES(IMU_BIAS_SAMPLES),
        .LH_ZERO_US(SERVO_LH_US),.LK_ZERO_US(SERVO_LK_US),
        .RH_ZERO_US(SERVO_RH_US),.RK_ZERO_US(SERVO_RK_US)) model_thread(
        .clk(clk),.rst_n(device_reset_n),.enable(MODEL_ENABLE && command_run),
        .sensor_ready(sensor_ready),.sensor_error(sensor_error),.sample_valid(sample_valid),
        .external_fault(command_emergency_stop || left_motor_fault!=0 || right_motor_fault!=0 || !MOTOR_ENABLE || !SERVO_ENABLE),
        .accel_x(accel_x),.accel_y(accel_y),.accel_z(accel_z),
        .gyro_x(gyro_x),.gyro_y(gyro_y),.gyro_z(gyro_z),
        .left_encoder_delta(left_encoder_delta),.right_encoder_delta(right_encoder_delta),
        .command_forward_q12(command_forward_q12),.command_yaw_q12(command_yaw_q12),
        .command_height_q12(command_height_q12),
        .target_left_mmps(model_left_mmps),.target_right_mmps(model_right_mmps),
        .servo_lh_us(model_lh_us),.servo_lk_us(model_lk_us),.servo_rh_us(model_rh_us),.servo_rk_us(model_rk_us),
        .calibrating(imu_calibrating),.bias_suspect(imu_bias_suspect),.accel_rejected(imu_accel_rejected),
        .ready(model_ready),.active(model_active),.action_valid(model_action_valid),
        .status_flags(model_status_flags),.policy_fault(model_fault),
        .completed_frames(model_completed_frames),.inference_cycles(model_inference_cycles),
        .executed_actions(model_executed_actions),
        .debug_fusion_valid(debug_fusion_valid),.debug_wx(debug_wx),.debug_wy(debug_wy),.debug_wz(debug_wz),
        .debug_gx(debug_gx),.debug_gy(debug_gy),.debug_gz(debug_gz));

    // Keep complete valid BMI/fusion samples, avoiding partial multi-cycle updates.
    reg [95:0] imu_telemetry_snapshot;
    reg signed [23:0] telemetry_wx,telemetry_wy,telemetry_wz,telemetry_gx,telemetry_gy,telemetry_gz;
    always @(posedge clk or negedge device_reset_n) begin
        if(!device_reset_n) begin
            imu_telemetry_snapshot<=0;
            telemetry_wx<=0;telemetry_wy<=0;telemetry_wz<=0;
            telemetry_gx<=0;telemetry_gy<=0;telemetry_gz<=-24'sd4096;
        end else begin
            if(sample_valid) imu_telemetry_snapshot<={gyro_z,gyro_y,gyro_x,accel_z,accel_y,accel_x};
            if(debug_fusion_valid) begin
                telemetry_wx<=debug_wx;telemetry_wy<=debug_wy;telemetry_wz<=debug_wz;
                telemetry_gx<=debug_gx;telemetry_gy<=debug_gy;telemetry_gz<=debug_gz;
            end
        end
    end
    // Capture diagnostic PID fields only after a complete8-clock controller update.
    reg signed [31:0] telemetry_left_reference,telemetry_right_reference;
    reg signed [31:0] telemetry_left_error,telemetry_right_error;
    reg [15:0] telemetry_left_control_sequence,telemetry_right_control_sequence;
    always @(posedge clk or negedge device_reset_n) begin
        if(!device_reset_n) begin
            telemetry_left_reference<=0;telemetry_right_reference<=0;
            telemetry_left_error<=0;telemetry_right_error<=0;
            telemetry_left_control_sequence<=0;telemetry_right_control_sequence<=0;
        end else begin
            if(left_control_valid) begin
                telemetry_left_reference<=left_pid_reference;telemetry_left_error<=left_pid_error;
                telemetry_left_control_sequence<=telemetry_left_control_sequence+1'b1;
            end
            if(right_control_valid) begin
                telemetry_right_reference<=right_pid_reference;telemetry_right_error<=right_pid_error;
                telemetry_right_control_sequence<=telemetry_right_control_sequence+1'b1;
            end
        end
    end
    // V2:120-byte payload,100Hz at230400baud. First100 bytes retain V1 layout.
    // Low48 bits are filled by uart_telemetry: timestamp_ms32, sequence16.
    wire [959:0] telemetry_payload={
        telemetry_right_control_sequence,telemetry_left_control_sequence,
        telemetry_right_error,telemetry_left_error,telemetry_right_reference,telemetry_left_reference,
        right_encoder_delta,left_encoder_delta,model_inference_cycles,model_completed_frames,
        model_rk_us,model_rh_us,model_lk_us,model_lh_us,right_duty,left_duty,
        right_speed_cms,left_speed_cms,wheel_right_target_mmps,wheel_left_target_mmps,
        {{8{command_height_q12[23]}},command_height_q12},
        {{8{command_yaw_q12[23]}},command_yaw_q12},
        {{8{command_forward_q12[23]}},command_forward_q12},
        {{8{telemetry_gz[23]}},telemetry_gz},{{8{telemetry_gy[23]}},telemetry_gy},{{8{telemetry_gx[23]}},telemetry_gx},
        {{8{telemetry_wz[23]}},telemetry_wz},{{8{telemetry_wy[23]}},telemetry_wy},{{8{telemetry_wx[23]}},telemetry_wx},
        imu_telemetry_snapshot,
        {5'd0,imu_accel_rejected,imu_bias_suspect,imu_calibrating},
        {5'd0,diagnostic_right_motor_fault},{5'd0,diagnostic_left_motor_fault},
        {4'd0,sensor_error_code},{5'd0,model_fault},model_status_flags,48'd0};
    uart_telemetry #(.FRAME_HZ(TELEMETRY_FRAME_HZ),.CLKS_PER_BIT(UART_CLKS_PER_BIT))
        telemetry_thread(.clk(clk),.reset_n(device_reset_n),
        .payload(telemetry_payload),.tx(UART_TXD));

    // Retain motor fault evidence even after the controller disables its PID.
    reg [2:0] diagnostic_left_motor_fault,diagnostic_right_motor_fault;
    always @(posedge clk or negedge device_reset_n) begin
        if(!device_reset_n) begin diagnostic_left_motor_fault<=0;diagnostic_right_motor_fault<=0;end
        else if(!command_run) begin diagnostic_left_motor_fault<=0;diagnostic_right_motor_fault<=0;end
        else begin
            if(left_motor_fault!=0) diagnostic_left_motor_fault<=left_motor_fault;
            if(right_motor_fault!=0) diagnostic_right_motor_fault<=right_motor_fault;
        end
    end
    wire [35:0] oled_debug_status={1'b0,diagnostic_left_motor_fault,1'b0,diagnostic_right_motor_fault,
        model_completed_frames[15:0],1'b0,model_fault,model_status_flags};

    // ===== OLED显示接口：修改标签或连接的数值即可 =====
    // 12号字、左右两列：左AX/AY/AZ，右GX/GY/GZ；加速度显示g(两位小数)，陀螺显示dps(一位小数)。
    // VAL输入仍接signed16 ADC值；显示线程按当前±3g/±2000dps配置换算。
    oled #(
        .SIX_AXIS_MODE(1),.WHEEL_HEADER_MODE(1),.MODEL_DEBUG_MODE(OLED_MODEL_DEBUG),
        .AXIS0_TEXT("AX:"),.AXIS1_TEXT("AY:"),.AXIS2_TEXT("AZ:"),
        .AXIS3_TEXT("GX:"),.AXIS4_TEXT("GY:"),.AXIS5_TEXT("GZ:")
    ) display(
        .clk(clk),.reset_n(device_reset_n),.LED_NUM(32'd0),
        .VAL0(accel_x),.VAL1(accel_y),.VAL2(accel_z),
        .VAL3(gyro_x),.VAL4(gyro_y),.VAL5(gyro_z),
        .SPEED_L(left_speed_cms),.SPEED_R(right_speed_cms),
        .sample_valid(sample_valid),.sensor_ready(sensor_ready),.sensor_error(sensor_error),
        .sensor_error_code(sensor_error_code),.DEBUG_STATUS(oled_debug_status),
        .IIC_SCL(sck),.IIC_SDA(sda),.ready(),.error());
    wire signed [31:0] feedback_sum=$signed(left_speed_cms)+$signed(right_speed_cms);
    wire signed [47:0] feedback_gyro=$signed(telemetry_wz)*48'sd1000;
    assign feedback_vx_mmps=feedback_sum*5;
    assign feedback_wz_mradps=feedback_gyro>>>12;
    assign feedback_valid=left_speed_valid && right_speed_valid && model_ready;
endmodule
