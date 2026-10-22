// BMI088 I2C 六轴驱动；基于原工程 bmi088_iic.v。
// 复用 oled_driver.v 内的 iic 模块；无须另外加入 iic.v。
// 地址探测：加速度 0x18/0x19，陀螺 0x68/0x69；配置回读校验；故障自动重试。
// 原始输出：signed16；六字节连续读，两组全部成功后同时发布，data_valid 高1周期。
// 沿用原工程配置：加速度±3g/100Hz；陀螺±2000°/s/2kHz、230Hz带宽。
// POLL_CYCLES=500000 表示每10ms开始一次采集，陀螺与加速度采样时刻非硬件同步。
module bmi088 #(
    parameter STARTUP_CYCLES = 1_500_000,
    parameter WRITE_WAIT_CYCLES = 50_000,
    parameter ACCEL_CONF_WAIT_CYCLES = 250_000,
    parameter ACCEL_POWER_WAIT_CYCLES = 2_500_000,
    parameter GYRO_WAIT_CYCLES = 1_500_000,
    parameter POLL_CYCLES = 500_000,
    parameter QUARTER_CYCLES = 125,
    parameter TIMEOUT_CYCLES = 50_000,
    parameter BUS_CLEAR = 0,
    parameter RETRY_CYCLES = 5_000_000 // 50 MHz下出错后等待100 ms重试
) 
//配置普通引脚 和GAO配置文件
(
    input wire clk,
    input wire reset_n,
    inout wire IIC_SCL,
    inout wire IIC_SDA,
    output reg init_done = 1'b0 /* synthesis syn_preserve = 1 */,
    output reg init_error = 1'b0 /* synthesis syn_preserve = 1 */,
    output reg data_valid = 1'b0 /* synthesis syn_preserve = 1 */,
    output reg signed [15:0] accel_x = 16'd0 /* synthesis syn_preserve = 1 */,
    output reg signed [15:0] accel_y = 16'd0 /* synthesis syn_preserve = 1 */,
    output reg signed [15:0] accel_z = 16'd0 /* synthesis syn_preserve = 1 */,
    output reg signed [15:0] gyro_x = 16'd0 /* synthesis syn_preserve = 1 */,
    output reg signed [15:0] gyro_y = 16'd0 /* synthesis syn_preserve = 1 */,
    output reg signed [15:0] gyro_z = 16'd0 /* synthesis syn_preserve = 1 */,
    output reg [3:0] error_code = 4'd0 /* synthesis syn_preserve = 1 */,
    output reg [5:0] operation = 6'd0 /* synthesis syn_preserve = 1 */,
    output reg [6:0] accel_addr = 7'h18 /* synthesis syn_preserve = 1 */,
    output reg [6:0] gyro_addr = 7'h68 /* synthesis syn_preserve = 1 */,
    output reg [7:0] accel_id = 8'd0 /* synthesis syn_preserve = 1 */,
    output reg [7:0] gyro_id = 8'd0 /* synthesis syn_preserve = 1 */,
    output reg [7:0] last_read = 8'd0 /* synthesis syn_preserve = 1 */
);
    localparam [1:0] WAIT = 2'd0, ISSUE = 2'd1,
        RECEIVE = 2'd2, RETRY = 2'd3;
    // 0/1: accel ID at 18/19; 2/3: gyro ID at 68/69.
    // 4..10: write 7C,7D,40,41,11,0F,10; 11..17: same readbacks.
    // 18: accel burst at 12; 19: gyro burst at 02 and atomic publication.
    reg [1:0] state = WAIT;
    wire bus_reset_n = reset_n && state != RETRY;
    reg [31:0] wait_cnt = 32'd0, poll_cnt = 32'd0;
    reg start_read1 = 1'b0, start_read6 = 1'b0, start_write = 1'b0;
    reg [6:0] device_addr = 7'h18;
    reg [7:0] register_addr = 8'd0, write_data = 8'd0;
    reg [47:0] accel_pending = 48'd0;
    wire busy_read1, busy_read6, busy_write;
    wire done_read1, done_read6, done_write;
    wire error_read1, error_read6, error_write;
    wire [3:0] code_read1, code_read6, code_write;
    wire [7:0] data_read1, data_read6;
    wire [63:0] bytes_read6;
    wire is_write = operation >= 6'd4 && operation <= 6'd10;
    wire is_burst = operation == 6'd18 || operation == 6'd19;
    wire engine_done = is_write ? done_write :
        (is_burst ? done_read6 : done_read1);
    wire engine_error = is_write ? error_write :
        (is_burst ? error_read6 : error_read1);
    wire [3:0] engine_code = is_write ? code_write :
        (is_burst ? code_read6 : code_read1);
    wire [7:0] engine_data = is_burst ? data_read6 : data_read1;
    
//读取bmi088 地址函数
    iic #(
        .QUARTER_CYCLES(QUARTER_CYCLES),
        .TIMEOUT_CYCLES(TIMEOUT_CYCLES), 
        .BUS_CLEAR(BUS_CLEAR), 
        .READ_BYTES(1)
        ) 
        u_read1 (
        .clk(clk), 
        .reset_n(bus_reset_n), 
        .start(start_read1),
        .device_addr(device_addr), 
        .register_addr(register_addr), 
        .write_data(8'd0),
        .IIC_SCL(IIC_SCL), 
        .IIC_SDA(IIC_SDA), 
        .busy(busy_read1),
        .done(done_read1), 
        .error(error_read1), 
        .error_code(code_read1),
        .read_data(data_read1), 
        .read_bytes()
    );
//读取bmi088 寄存器函数
    iic #(.QUARTER_CYCLES(QUARTER_CYCLES),
        .TIMEOUT_CYCLES(TIMEOUT_CYCLES), 
        .BUS_CLEAR(BUS_CLEAR), 
        .READ_BYTES(6)) 
        u_read6 (
        .clk(clk), 
        .reset_n(bus_reset_n),
        .start(start_read6),
        .device_addr(device_addr),
        .register_addr(register_addr), 
        .write_data(8'd0),
        .IIC_SCL(IIC_SCL),
        .IIC_SDA(IIC_SDA), 
        .busy(busy_read6),
        .done(done_read6),
        .error(error_read6),
        .error_code(code_read6),
        .read_data(data_read6), 
        .read_bytes(bytes_read6)
    );
//写bmi088寄存器函数
    iic #(.QUARTER_CYCLES(QUARTER_CYCLES),
        .TIMEOUT_CYCLES(TIMEOUT_CYCLES), 
        .BUS_CLEAR(BUS_CLEAR), 
        .WRITE_MODE(1)) 
        u_write (
        .clk(clk), 
        .reset_n(bus_reset_n), 
        .start(start_write),
        .device_addr(device_addr), 
        .register_addr(register_addr), 
        .write_data(write_data),
        .IIC_SCL(IIC_SCL), 
        .IIC_SDA(IIC_SDA), 
        .busy(busy_write),
        .done(done_write), 
        .error(error_write), 
        .error_code(code_write),
        .read_data(), 
        .read_bytes()
    );
//GOWIN函数  名字叫config_item 总共返回17位
//该函数用来配置bmi088 的功能 
    function [16:0] config_item;
        input [5:0] index;
        begin
            case (index)
                // 加速度退出省电模式（ACC_PWR_CONF=0） 
                6'd4: config_item = {1'b0, 8'h7c, 8'h00};
                // 加速度启用（ACC_PWR_CTRL=4）
                6'd5: config_item = {1'b0, 8'h7d, 8'h04};
                // 加速度100Hz、正常滤波（ACC_CONF=A8）
                6'd6: config_item = {1'b0, 8'h40, 8'ha8};
                // 加速度量程±3g（ACC_RANGE=0）
                6'd7: config_item = {1'b0, 8'h41, 8'h00};
                // 陀螺正常工作模式（GYRO_LPM1=0）
                6'd8: config_item = {1'b1, 8'h11, 8'h00};
                // 陀螺量程±2000°/s（GYRO_RANGE=0）
                6'd9: config_item = {1'b1, 8'h0f, 8'h00};
                // 陀螺ODR=2000Hz、带宽230Hz（GYRO_BANDWIDTH=1）；量程由0x0F决定
                6'd10: config_item = {1'b1, 8'h10, 8'h01};
                default: config_item = 17'd0;
            endcase
        end
    endfunction

    wire [16:0] item = config_item(is_write ? operation : operation - 6'd7);
    wire [7:0] verify_mask = operation == 6'd17 ? 8'h7f : 8'hff;
    wire scanning = operation <= 6'd3;
    wire expected_id = operation <= 6'd1 ? engine_data == 8'h1e :
        engine_data == 8'h0f;
    wire address_nack = engine_code == 4'd1 || engine_code == 4'd3;

    always @(posedge clk) begin
        start_read1 <= 1'b0;
        start_read6 <= 1'b0;
        start_write <= 1'b0;
        data_valid <= 1'b0;
//如果复位开始 state切到 WAIT状态  一切归零
        if (!reset_n) begin
            state <= WAIT;
            operation <= 6'd0;
            wait_cnt <= STARTUP_CYCLES > 0 ? STARTUP_CYCLES - 1 : 0;
            poll_cnt <= 32'd0;
            init_done <= 1'b0;
            init_error <= 1'b0;
            error_code <= 4'd0;
            accel_addr <= 7'h18; gyro_addr <= 7'h68;
            accel_id <= 8'd0; gyro_id <= 8'd0; last_read <= 8'd0;
            device_addr <= 7'h18; register_addr <= 8'd0; write_data <= 8'd0;
            accel_pending <= 48'd0;
            accel_x <= 16'd0; accel_y <= 16'd0; accel_z <= 16'd0;
            gyro_x <= 16'd0; gyro_y <= 16'd0; gyro_z <= 16'd0;
        end 
        else begin
            if (poll_cnt != 0) poll_cnt <= poll_cnt - 32'd1;
            case (state)
                WAIT: begin
                    if (wait_cnt != 0) wait_cnt <= wait_cnt - 32'd1;
                    else if (operation != 6'd18 || poll_cnt == 0) state <= ISSUE;
                end
//任务发起状态： 准备开始一次任务
//主要功能是检查寄存器 观察现在是否具备任务发起的条件
//若满足条件 发起任务  并且等待IIC完成此次读写 并进入RECEIVE状态
              ISSUE: begin
                    // All inactive engines remain idle with both pins released.
                    if (busy_read1 || busy_read6 || busy_write || operation > 6'd19) begin
                        init_done <= 1'b0; init_error <= 1'b1;
                        error_code <= 4'd6;
                        wait_cnt <= RETRY_CYCLES > 0 ? RETRY_CYCLES - 1 : 0;
                        state <= RETRY;
                    end else begin
                        state <= RECEIVE;
                        if (scanning) begin
                            device_addr <= operation <= 6'd1 ? accel_addr : gyro_addr;
                            register_addr <= 8'h00;
                            start_read1 <= 1'b1;
                        end else if (is_write || operation <= 6'd17) begin
                            device_addr <= item[16] ? gyro_addr : accel_addr;
                            register_addr <= item[15:8];
                            write_data <= item[7:0];
                            if (is_write) start_write <= 1'b1;
                            else start_read1 <= 1'b1;
                        end else begin
                            device_addr <= operation == 6'd18 ? accel_addr : gyro_addr;
                            register_addr <= operation == 6'd18 ? 8'h12 : 8'h02;
                            start_read6 <= 1'b1;
                            if (operation == 6'd18)
                                poll_cnt <= POLL_CYCLES > 0 ? POLL_CYCLES - 1 : 0;
                        end
                    end
                end
//任务结束状态  
                RECEIVE: begin
                    if (engine_done) begin
                        if (!is_write && !engine_error) begin
                            last_read <= engine_data;
                            if (operation <= 6'd1) accel_id <= engine_data;
                            else if (operation <= 6'd3) gyro_id <= engine_data;
                        end
                        if (scanning && ((engine_error && address_nack) ||
                            (!engine_error && !expected_id)) &&
                            (operation == 6'd0 || operation == 6'd2)) begin
                            // Retry only the other documented BMI088 address.
                            operation <= operation + 6'd1;
                            if (operation == 6'd0) accel_addr <= 7'h19;
                            else gyro_addr <= 7'h69;
                            wait_cnt <= 32'd0; state <= WAIT;
                        end else if (engine_error || (scanning && !expected_id) ||
                            (operation >= 6'd11 && operation <= 6'd17 &&
                             (engine_data & verify_mask) != item[7:0])) begin
                            init_done <= 1'b0; init_error <= 1'b1;
                            error_code <= engine_error ?
                                (engine_code == 0 ? 4'd6 : engine_code) :
                                (scanning ? 4'd7 : 4'd8);
                            wait_cnt <= RETRY_CYCLES > 0 ? RETRY_CYCLES - 1 : 0;
                            state <= RETRY;
                        end else begin
                            state <= WAIT;
                            wait_cnt <= 32'd0;
                            operation <= operation + 6'd1;
                            if (operation <= 6'd1) operation <= 6'd2;
                            else if (operation <= 6'd3) operation <= 6'd4;
                            if (is_write)
                                wait_cnt <= WRITE_WAIT_CYCLES > 0 ? WRITE_WAIT_CYCLES - 1 : 0;
                            if (operation == 6'd4)
                                wait_cnt <= ACCEL_CONF_WAIT_CYCLES > 0 ? ACCEL_CONF_WAIT_CYCLES - 1 : 0;
                            if (operation == 6'd5)
                                wait_cnt <= ACCEL_POWER_WAIT_CYCLES > 0 ? ACCEL_POWER_WAIT_CYCLES - 1 : 0;
                            if (operation == 6'd8)
                                wait_cnt <= GYRO_WAIT_CYCLES > 0 ? GYRO_WAIT_CYCLES - 1 : 0;
                            if (operation == 6'd17) begin
                                init_done <= 1'b1;
                                init_error <= 1'b0;
                                error_code <= 4'd0;
                            end
                            if (operation == 6'd18) accel_pending <= bytes_read6[47:0];
                            if (operation == 6'd19) begin
                                accel_x <= {accel_pending[39:32], accel_pending[47:40]};
                                accel_y <= {accel_pending[23:16], accel_pending[31:24]};
                                accel_z <= {accel_pending[7:0], accel_pending[15:8]};
                                gyro_x <= {bytes_read6[39:32], bytes_read6[47:40]};
                                gyro_y <= {bytes_read6[23:16], bytes_read6[31:24]};
                                gyro_z <= {bytes_read6[7:0], bytes_read6[15:8]};
                                data_valid <= 1'b1;
                                operation <= 6'd18;
                            end
                        end
                    end
                end
                RETRY: begin
                    // 重试仅复位I2C引擎，六轴输出保留至下一完整样本。
                    init_done <= 1'b0;
                    if (wait_cnt != 0)
                        wait_cnt <= wait_cnt - 32'd1;
                    else begin
                        operation <= 6'd0;
                        accel_addr <= 7'h18;
                        gyro_addr <= 7'h68;
                        accel_id <= 8'd0;
                        gyro_id <= 8'd0;
                        last_read <= 8'd0;
                        accel_pending <= 48'd0;
                        poll_cnt <= 32'd0;
                        device_addr <= 7'h18;
                        register_addr <= 8'd0;
                        write_data <= 8'd0;
                        wait_cnt <= STARTUP_CYCLES > 0 ? STARTUP_CYCLES - 1 : 0;
                        state <= WAIT;
                    end
                end
                default: begin
                    init_done <= 1'b0;
                    init_error <= 1'b1;
                    error_code <= 4'd6;
                    wait_cnt <= RETRY_CYCLES > 0 ? RETRY_CYCLES - 1 : 0;
                    state <= RETRY;
                end
            endcase
        end
    end
endmodule
