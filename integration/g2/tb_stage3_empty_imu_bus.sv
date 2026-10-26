`timescale 1ns/1ps
// Real nav_subsystem register path, idle network and UART; no Pico CPU claim.
module stage3_empty_imu_bus(output reg done=0);
 reg clk=0,rst=1,rx_rst=1,req=0;reg[15:0]addr=0;
 wire ack;wire[31:0]rdata;integer checks=0;
 always #10 if(!done)clk=~clk;
 nav_subsystem #(.CLK_HZ(50000000),.SLF_LOG2N(7),.SIN_INIT("sin_lut.hex"))dut(
  .clk(clk),.rst(rst),.bus_req(req),.bus_we(1'b0),.bus_addr(addr),.bus_wdata(32'd0),.bus_ack(ack),.bus_rdata(rdata),.irq(),
  .l_rx_clk(clk),.l_rx_rst(rx_rst),.l_rx_valid(1'b0),.l_rx_data(8'd0),.l_rx_last(1'b0),.l_rx_error(1'b0),
  .l_tx_clk(clk),.l_tx_rst(rst),.l_tx_valid(),.l_tx_data(),.l_tx_last(),.l_tx_ready(1'b1),.l_phy_stat(32'd0),.l_rx_neg(),
  .h_rx_clk(clk),.h_rx_rst(rx_rst),.h_rx_valid(1'b0),.h_rx_data(8'd0),.h_rx_last(1'b0),.h_rx_error(1'b0),
  .h_tx_clk(clk),.h_tx_rst(rst),.h_tx_valid(),.h_tx_data(),.h_tx_last(),.h_tx_ready(1'b1),
  .imu_rx(1'b1),.imu_tx(),.ch_vx(),.ch_vy(),.ch_wz(),.ch_mode(),.ch_stop(),.ch_wdog(),.ch_valid(),.ch_seq(),
  .odo_valid(1'b0),.odo_vx(16'd0),.odo_vy(16'd0),.odo_wz(16'd0),.ext_stop(4'd0));
 task read_check(input[15:0]a,input[31:0]wanted);
  integer count;
  begin
   @(negedge clk);req=1;addr=a;count=0;
   do begin @(posedge clk);#1;count++;if(count>16)$fatal(1,"bus ACK timeout addr%h",a);end while(!ack);
   checks++;if(rdata!==wanted)$fatal(1,"bus addr%h got%h expected%h",a,rdata,wanted);
   @(negedge clk);req=0;repeat(3)@(posedge clk);
  end
 endtask
 initial begin
  repeat(12)@(negedge clk);rst=0;rx_rst=0;repeat(12)@(negedge clk);
  read_check(16'h0000,32'h4e415630);
  if(dut.iu_valid!==0)$fatal(1,"idle IMU not empty");
  // Poison only invalid payload, retaining real FIFO valid and register ACK path.
  force dut.iu_data=8'hxx;force dut.iu_first=1'bx;
  repeat(3)read_check(16'h6004,32'd0);
  @(negedge clk);rx_rst=1;repeat(5)@(negedge clk);rx_rst=0;
  repeat(8)@(negedge clk);read_check(16'h6004,32'd0);
  @(negedge clk);rst=1;rx_rst=1;repeat(7)@(negedge clk);rst=0;rx_rst=0;
  repeat(12)@(negedge clk);read_check(16'h0000,32'h4e415630);read_check(16'h6004,32'd0);
  release dut.iu_data;release dut.iu_first;
  $display("STAGE3_EMPTY_IMU_BUS_PASS checks=%0d cold_and_warm_reset=1 payload_X=1",checks);done=1;
 end
endmodule
