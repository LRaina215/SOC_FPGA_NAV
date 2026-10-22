// Temporary SRAM diagnostic. Original BMI088/I2C driver and configuration are reused.
// No policy or motor/servo controller is instantiated. All ten actuator outputs are low.
module bmi_probe_top #(
    parameter CLOCK_FREQ_HZ=50_000_000,
    parameter FRAME_HZ=10,
    parameter CLKS_PER_BIT=217
)(
    input wire clk, reset_n,
    inout wire BMI088_SDA, BMI088_SCL,
    output wire uart_txd,
    output wire M1_PWM,M1_DIR1,M1_DIR2,M2_PWM,M2_DIR1,M2_DIR2,
    output wire SERVO1,SERVO2,SERVO3,SERVO4
);
    assign {M1_PWM,M1_DIR1,M1_DIR2,M2_PWM,M2_DIR1,M2_DIR2,
            SERVO1,SERVO2,SERVO3,SERVO4}=10'd0;
    reg [7:0] startup=0;
    always @(posedge clk) begin
        if(!reset_n) startup<=0;
        else if(!(&startup)) startup<=startup+1'b1;
    end
    wire device_reset_n=reset_n && (&startup);
    wire init_done,init_error,sample_valid;
    wire [3:0] error_code;
    wire [5:0] operation;
    wire [6:0] accel_address,gyro_address;
    wire [7:0] accel_id,gyro_id,last_read;
    wire signed [15:0] ax,ay,az,gx,gy,gz;
    bmi088 sensor(.clk(clk),.reset_n(device_reset_n),
        .IIC_SDA(BMI088_SDA),.IIC_SCL(BMI088_SCL),
        .init_done(init_done),.init_error(init_error),.data_valid(sample_valid),
        .accel_x(ax),.accel_y(ay),.accel_z(az),.gyro_x(gx),.gyro_y(gy),.gyro_z(gz),
        .error_code(error_code),.operation(operation),.accel_addr(accel_address),
        .gyro_addr(gyro_address),.accel_id(accel_id),.gyro_id(gyro_id),.last_read(last_read));
    reg [31:0] sample_count;
    always @(posedge clk or negedge device_reset_n) begin
        if(!device_reset_n) sample_count<=0;
        else if(sample_valid) sample_count<=sample_count+1'b1;
    end
    // A5 5A B0 20,32-byte LE payload,CRC16-CCITT-FALSE LE.
    // Bytes0..5: timestamp/sequence inserted by the unchanged telemetry transmitter.
    // Bytes6..17: status,error,operation,addresses,IDs,last_read,sample_count.
    // Bytes18..29: six signed16 raw axes. Bytes30..31: diagnostic signature BC01.
    wire [255:0] payload={16'hbc01,gz,gy,gx,az,ay,ax,sample_count,
        last_read,gyro_id,accel_id,{1'b0,gyro_address},{1'b0,accel_address},
        {2'd0,operation},{4'd0,error_code},
        {5'd0,(sample_count!=0),init_error,init_done},48'd0};
    uart_telemetry #(.VERSION(8'hb0),.PAYLOAD_BYTES(32),
        .CLOCK_FREQ_HZ(CLOCK_FREQ_HZ),.FRAME_HZ(FRAME_HZ),.CLKS_PER_BIT(CLKS_PER_BIT))
        transmitter(.clk(clk),.reset_n(device_reset_n),.payload(payload),.tx(uart_txd));
endmodule
