`timescale 1ns/1ps
module tb_bmi_probe;
    localparam CPB=8;
    reg clk=0,reset_n=0;
    always #5 clk=~clk;
    tri1 sda,scl;
    wire tx;
    wire [9:0] actuators;
    bmi_probe_top #(.CLOCK_FREQ_HZ(1000000),.FRAME_HZ(100),.CLKS_PER_BIT(CPB)) dut(
        .clk(clk),.reset_n(reset_n),.BMI088_SDA(sda),.BMI088_SCL(scl),.uart_txd(tx),
        .M1_PWM(actuators[0]),.M1_DIR1(actuators[1]),.M1_DIR2(actuators[2]),
        .M2_PWM(actuators[3]),.M2_DIR1(actuators[4]),.M2_DIR2(actuators[5]),
        .SERVO1(actuators[6]),.SERVO2(actuators[7]),.SERVO3(actuators[8]),.SERVO4(actuators[9]));
    initial begin
        #30 reset_n=1;
        wait(dut.device_reset_n);
        force dut.init_done=1;force dut.init_error=0;force dut.error_code=0;
        force dut.operation=19;force dut.accel_address=7'h18;force dut.gyro_address=7'h68;
        force dut.accel_id=8'h1e;force dut.gyro_id=8'h0f;force dut.last_read=8'h81;
        force dut.ax=-32768;force dut.ay=32767;force dut.az=10920;
        force dut.gx=-123;force dut.gy=456;force dut.gz=-789;
        force dut.sample_valid=1;
        @(posedge clk);#1;force dut.sample_valid=0;
    end
    always @(posedge clk) if(actuators!==10'd0) $fatal(1,"Actuator output was not zero");
    integer count=0,j,fd;
    reg [7:0] byte_value;
    initial begin
        fd=$fopen("probe_uart_bytes.hex","w");
        wait(dut.device_reset_n);
        while(count<114) begin
            @(negedge tx);#(CPB*15);
            for(j=0;j<8;j=j+1) begin byte_value[j]=tx;#(CPB*10);end
            if(tx!==1'b1) $fatal(1,"Missing stop bit");
            $fdisplay(fd,"%02x",byte_value);count=count+1;
        end
        $fclose(fd);$display("BMI_PROBE_WIRE_PASS 114 bytes, actuator outputs zero");$finish;
    end
    initial begin #600000;$fatal(1,"Probe UART timeout");end
endmodule
