`timescale 1ns/1ps
module tb_feedback_probe;
 localparam CPB=2;
 reg clk=0,reset_n=0;always #5 clk=~clk;
 reg[1:0]m1=0,m2=0;
 tri1 sda,scl;reg bus_sda=1,bus_scl=1;
 assign sda=bus_sda?1'bz:1'b0;assign scl=bus_scl?1'bz:1'b0;
 wire tx;wire[9:0]actuators;
 feedback_probe_top #(.CLOCK_FREQ_HZ(1000000),.FRAME_HZ(100),.CLKS_PER_BIT(CPB))dut(
  .clk(clk),.reset_n(reset_n),.M1_ENCA(m1[1]),.M1_ENCB(m1[0]),.M2_ENCA(m2[1]),.M2_ENCB(m2[0]),
  .BMI088_SDA(sda),.BMI088_SCL(scl),.uart_txd(tx),
  .M1_PWM(actuators[0]),.M1_DIR1(actuators[1]),.M1_DIR2(actuators[2]),
  .M2_PWM(actuators[3]),.M2_DIR1(actuators[4]),.M2_DIR2(actuators[5]),
  .SERVO1(actuators[6]),.SERVO2(actuators[7]),.SERVO3(actuators[8]),.SERVO4(actuators[9]));
 always@(posedge clk)if(actuators!==0)$fatal(1,"nonzero actuator");
 task start_bus;begin bus_scl=0;#100;bus_sda=1;#100;bus_scl=1;#100;bus_sda=0;#100;bus_scl=0;#100;end endtask
 task stop_bus;begin bus_scl=0;bus_sda=0;#100;bus_scl=1;#100;bus_sda=1;#100;end endtask
 task byte_bus(input[7:0]v,input bit nack);begin
  for(integer k=7;k>=0;k--)begin bus_scl=0;bus_sda=v[k];#100;bus_scl=1;#100;bus_scl=0;#100;end
  bus_sda=nack;#100;bus_scl=1;#100;bus_scl=0;#100;
 end endtask
 initial begin
  #30;reset_n=1;wait(dut.ready_reset_n);#100;
  // Synthetic publication tests the transport only; no sensor model is asserted.
  force dut.init_done=1;force dut.init_error=0;force dut.error_code=0;force dut.operation=19;
  force dut.accel_address=7'h18;force dut.gyro_address=7'h68;
  force dut.accel_id=8'h1e;force dut.gyro_id=8'h0f;force dut.last_read=8'h81;
  force dut.ax=-32768;force dut.ay=32767;force dut.az=10920;
  force dut.gx=-123;force dut.gy=456;force dut.gz=-789;
  @(negedge clk);force dut.sample_valid=1;@(negedge clk);force dut.sample_valid=0;
  // Change live values without publishing: the old complete sample must hold.
  force dut.ax=1234;force dut.gx=3456;
  repeat(20)@(negedge clk);
  m2=1;m1=2;repeat(20)@(negedge clk);
  m2=3;m1=3;repeat(20)@(negedge clk);
  m2=2;m1=1;repeat(20)@(negedge clk);
  m2=0;m1=0;repeat(20)@(negedge clk);
  start_bus();byte_bus(8'hd0,1);stop_bus();#200;
  start_bus();byte_bus(8'hd2,0);byte_bus(0,0);start_bus();byte_bus(8'hd3,0);byte_bus(8'ha2,1);stop_bus();
 end
 integer count=0,j,fd;reg[7:0]value;
 initial begin
  fd=$fopen("feedback_uart_bytes.hex","w");wait(dut.ready_reset_n);
  while(count<378)begin
   @(negedge tx);#(CPB*15);
   for(j=0;j<8;j=j+1)begin value[j]=tx;#(CPB*10);end
   if(tx!==1)$fatal(1,"UART stop bit");
   $fdisplay(fd,"%02x",value);count=count+1;
  end
  $fclose(fd);$display("FEEDBACK_PROBE_WIRE_PASS 378 bytes, constant actuator outputs");$finish;
 end
 initial begin #500000;$fatal(1,"feedback UART timeout");end
endmodule
