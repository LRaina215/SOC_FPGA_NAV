`timescale 1ns/1ps
module tb_response_profile;
reg clk=0;always #10 clk=~clk;reg rst=0;
wire sck,sda,scl,sd;
pullup(sck);pullup(sda);pullup(scl);pullup(sd);
main #(.MOTOR_ENABLE(1'b0),.SERVO_ENABLE(1'b0)) dut(.clk(clk),.reset_n(rst),.UART_RXD(1'b1),.IRIN(1'b1),
 .sck(sck),.sda(sda),.BMI088_SCL(scl),.BMI088_SDA(sd),
 .M1_ENCA(1'b0),.M1_ENCB(1'b0),.M2_ENCA(1'b0),.M2_ENCB(1'b0));
initial begin
 if(dut.MODEL_WHEEL_SPEED_GAIN!=1.0 || dut.L_KP_Q12!=12288 || dut.R_KP_Q12!=12288 || dut.L_KI_Q12!=1434 ||
    dut.L_KD_Q12!=8192 || dut.L_KFF_Q12!=3277 || dut.MOTOR_RAMP_STEP_MMPS!=100)
  $fatal(1,"original4500 10ms gain elaboration");
 if(dut.wheel_control.USE_MMPS_FEEDBACK || dut.MODEL_WHEEL_FILTER_ENABLE || dut.wheel_left.SPEED_GATE_CYCLES!=500000 || dut.wheel_right.SPEED_GATE_CYCLES!=500000 || dut.wheel_control.STALL_SAMPLES!=100 || dut.wheel_control.WRONG_DIRECTION_SAMPLES!=10 || dut.wheel_control.DERIVATIVE_FILTER_SHIFT!=2) $fatal(1,"original4500 10ms clocks/protection/model observation");
 if(dut.wheel_control.CLOSED_LOOP_AT_ZERO || dut.servo_thread.FRAME_US!=20000 ||
    dut.telemetry_thread.FRAME_HZ!=100 || dut.telemetry_thread.CLKS_PER_BIT!=217)
  $fatal(1,"control/telemetry defaults");
 force dut.model_left_mmps=-300;force dut.model_right_mmps=300;
 force dut.model_active=1;
 force dut.left_speed_cms=16'sd12;force dut.right_speed_cms=-16'sd23;
 force dut.left_speed_valid=1;force dut.right_speed_valid=0;#1;
 if(dut.wheel_control.speed_left_cms!=12 || dut.wheel_control.speed_right_cms!=-23 || !dut.wheel_control.valid_left || dut.wheel_control.valid_right) $fatal(1,"10ms speed path wiring");
 if(dut.wheel_control.target_left_mmps!=-300 || dut.wheel_control.target_right_mmps!=300) $fatal(1,"original target path must bypass filter");
 force dut.ir_key_active=1;force dut.ir_key_code=8'h18;#1;
 if(dut.wheel_left_target_mmps!=-300 || dut.wheel_right_target_mmps!=300)$fatal(1,"forward key must not amplify outputs");
 force dut.ir_key_code=8'h52;#1;
 if(dut.wheel_left_target_mmps!=-300 || dut.wheel_right_target_mmps!=300)$fatal(1,"reverse key must not amplify outputs");
 $display("PASS main response profile: gain1, tuned Q12 gains, zero feedback, servo20ms, telemetry100Hz/230400");$finish;
end
endmodule
