`timescale 1ns/1ps
module tb_sfp_management;
    reg clk=0,rst=1,req=0,we=0,los=0;
    reg [7:0] addr=0;reg [31:0] wdata=0;
    always #10 clk=~clk;
    wire ack;wire[31:0] rdata;wire rxclk,txclk,hr,rr,tr,disable_tx;
    sfp_management dut(.clk(clk),.rst(rst),.req(req),.we(we),.addr(addr),.wdata(wdata),.ack(ack),.rdata(rdata),
      .telemetry_accepted(32'd123),.telemetry_dropped(32'd9),.telemetry_sent(32'd111),
      .sfp_los(los),.sfp_tx_disable(disable_tx),.rx_clk(rxclk),.rx_valid(),.rx_last(),.rx_error(),.rx_data(),
      .tx_clk(txclk),.tx_valid(1'b0),.tx_last(1'b0),.tx_data(8'd0),.tx_ready(),.host_rst(hr),.rx_rst(rr),.tx_rst(tr));
    task read_check(input [7:0] a,input [31:0] expected);
      @(negedge clk);addr=a;req=1;we=0;#1;
      if(!ack||rdata!==expected)$fatal(1,"read %x = %x expected %x",a,rdata,expected);
      @(negedge clk);req=0;
    endtask
    task config_write(input [2:0] value);
      @(negedge clk);req=1;we=1;addr=4;wdata=value;
      @(negedge clk);req=0;we=0;
      if(!disable_tx||!hr||!rr||!tr)$fatal(1,"config must assert IP/network resets");
    endtask
    initial begin
      repeat(8)@(negedge clk);rst=0;
      read_check(0,32'h53465032);read_check(4,2);read_check(8'h18,32'h00010083);
      repeat(65000)@(negedge clk);
      if(!hr||!disable_tx)$fatal(1,"startup reset released early");
      repeat(1000)@(negedge clk);
      if(hr||rr||tr||disable_tx)$fatal(1,"startup did not release network");
      read_check(8'h20,123);read_check(8'h24,9);read_check(8'h28,111);
      config_write(7);read_check(4,7);
      repeat(66000)@(negedge clk);
      if(hr||rr||tr)$fatal(1,"configuration recovery");
      // Simulate recovered RX clock stopping when cable is removed. Reset must
      // assert without that clock; release waits for its returning edges.
      dut.u_mac.rx_clock_running=0;los=1;
      repeat(20)@(negedge clk);
      if(!hr||!rr||!tr)$fatal(1,"link loss must flush all network domains");
      los=0;repeat(400)@(negedge clk);
      if(hr||tr||!rr)$fatal(1,"RX reset released despite absent RX clock");
      dut.u_mac.rx_clock_running=1;repeat(10)@(negedge clk);
      if(rr)$fatal(1,"RX reset did not release after returning clock");
      read_check(8'h14,2);
      dut.u_mac.copper_up=0;repeat(20)@(negedge clk);
      if(!hr||!rr||!tr)$fatal(1,"SGMII PCS up must not hide copper link down");
      dut.u_mac.copper_up=1;repeat(400)@(negedge clk);
      if(hr||rr||tr)$fatal(1,"SGMII copper link recovery");
      read_check(8'h14,3);
      config_write(0);repeat(66000)@(negedge clk);
      if(hr||rr||tr)$fatal(1,"forced BASE-X recovery");
      if(rst)$fatal(1,"network test changed core reset");
      $display("TEST PASSED: joint SFP2 config, counters, link loss, missing RX clock, reset release");$finish;
    end
    initial begin #10000000;$fatal(1,"test timeout");end
endmodule

// Digital MAC boundary model only; vendor PCS and analog SerDes are not tested.
module sfp_host_mac(input clk,rst,cfg_sgmii,cfg_an,cfg_use_los,sfp_los,output sfp_tx_disable,
 output reg rx_clk=0,output rx_valid,output [7:0]rx_data,output rx_last,rx_error,
 output reg tx_clk=0,input tx_valid,tx_last,input [7:0]tx_data,output tx_ready,
 output [10:0]status,output [15:0]partner,output qpll_ok,rx_lock,aligned);
 reg rx_clock_running=1,copper_up=1;
 always #4 if(rx_clock_running)rx_clk=~rx_clk;
 initial begin #1;forever #4 tx_clk=~tx_clk;end
 wire live=!rst&&(!cfg_use_los||!sfp_los);
 assign sfp_tx_disable=rst;assign status=live?((cfg_an?11'd7:11'd3)|(cfg_sgmii&&copper_up?11'h180:0)):0;
 assign partner=16'h1234;assign qpll_ok=!rst;assign rx_lock=live;assign aligned=live;
 assign rx_valid=0;assign rx_data=0;assign rx_last=0;assign rx_error=0;assign tx_ready=live;
endmodule
