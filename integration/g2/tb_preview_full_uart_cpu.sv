`timescale 1ns/1ps
// Actual frozen G2 RTL and boot image. External sensors are idle/pulled up;
// no claim of moving-vehicle or completed 4500 inference is made by this test.
module tb_preview_full_uart_cpu;
`include "guard_symbol.vh"
`include "guard_probe_limits.vh"
  reg clk=0,rxc=0,key_n=0,uart_rx=1;
  always #10 clk=~clk;
  always #20 rxc=~rxc;
  tri1 sck,sda,bmi_scl,bmi_sda,mdio;
  wire uart_tx,trap;
  g2_top dut(.clk50(clk),.key_rst_n(key_n),.uart_rxd(uart_rx),.uart_txd(uart_tx),
    .imu_rxd(1'b1),.IRIN(1'b1),.M1_ENCA(1'b0),.M1_ENCB(1'b0),.M2_ENCA(1'b0),.M2_ENCB(1'b0),
    .sck(sck),.sda(sda),.BMI088_SCL(bmi_scl),.BMI088_SDA(bmi_sda),
    .rgmii_rxc(rxc),.rgmii_rx_ctl(1'b0),.rgmii_rxd(4'd0),.eth_mdio(mdio),.sfp1_los(1'b1),.led_trap(trap));
  integer writes=0,checks=0;reg monitoring=0;
  function [31:0] ram_word(input integer address);
    ram_word={dut.u_soc.g_lane[3].u_ram.mem[address>>2],dut.u_soc.g_lane[2].u_ram.mem[address>>2],
              dut.u_soc.g_lane[1].u_ram.mem[address>>2],dut.u_soc.g_lane[0].u_ram.mem[address>>2]};
  endfunction
  reg [31:0] guard_flags=0,accepted=0,rejected=0,last_rx=0;
  // A constant-argument continuous function call need not be re-evaluated on
  // hierarchical RAM changes. Sample the real storage explicitly, read-only,
  // after the CPU write edge; this never drives CPU memory or guard state.
  always @(negedge clk)begin
    guard_flags=ram_word(GUARD_ADDRESS+20);
    accepted=ram_word(GUARD_ADDRESS+8);rejected=ram_word(GUARD_ADDRESS+12);
    last_rx=ram_word(GUARD_ADDRESS);
  end
  wire have_rx=guard_flags[0],locked=guard_flags[8],timed_out=guard_flags[16];
  always @(posedge clk)if(monitoring && trap)$fatal(1,"actual firmware trapped pc=%h",dut.u_soc.u_cpu.reg_pc);
  localparam real BIT_NS=1e9/921600.0;
  task send(input [7:0] b);
    begin
      uart_rx=0;#(BIT_NS);
      for(integer i=0;i<8;i=i+1)begin uart_rx=b[i];#(BIT_NS);end
      uart_rx=1;#(BIT_NS);
    end
  endtask
  task recv(output [7:0] b);
    begin
      @(negedge uart_tx);#(BIT_NS*1.5);
      for(integer i=0;i<8;i=i+1)begin b[i]=uart_tx;#(BIT_NS);end
    end
  endtask
  task wr32(input [15:0] addr,input [31:0] value);
    reg [7:0] ack;
    begin
      fork
        begin send("W");send(addr[15:8]);send(addr[7:0]);send(value[31:24]);send(value[23:16]);send(value[15:8]);send(value[7:0]);end
        begin recv(ack);if(ack!=="K")$fatal(1,"UART bridge write response %h",ack);end
      join
      writes=writes+1;
    end
  endtask
  task mode(input [7:0] value);
    begin wr32(16'h1230,32'h3001564e);wr32(16'h1230,{24'd0,value});wr32(16'h1234,8);wr32(16'h1238,1);end
  endtask
  task wait_count(input bit reject_count,input integer target);
    reg [31:0] start;
    begin
      start=dut.u_nav.time_us;
      while((reject_count?rejected:accepted)<target && dut.u_nav.time_us-start<100000)repeat(500)@(negedge clk);
      if((reject_count?rejected:accepted)<target)$fatal(1,"actual CPU did not process host message reject=%b target=%0d",reject_count,target);
      checks=checks+1;
    end
  endtask
  initial begin : stimulus
    integer prior_count;reg [31:0] lost_at,start,elapsed,zero_age;
    #200;key_n=1;wait(!dut.rst);monitoring=1;
    start=dut.u_nav.time_us;
    while(dut.u_soc.status<3 && dut.u_nav.time_us-start<100000)repeat(500)@(negedge clk);
    if(dut.u_soc.status<3 || !locked || timed_out || have_rx)
      $fatal(1,"CPU boot/initial lock contract failed flags=%h actual_RAM=%h loops=%0d",guard_flags,ram_word(GUARD_ADDRESS+20),dut.u_soc.status);
    $display("ACTUAL_CPU_BOOT us=%0d loops=%0d console_bytes=%0d guard=%h",dut.u_nav.time_us,dut.u_soc.status,dut.u_soc.con_wp,GUARD_ADDRESS);
    checks=checks+1;
    prior_count=rejected;mode(1);wait_count(1,prior_count+1);
    if(!locked)$fatal(1,"motion message released initial lock");
    prior_count=accepted;mode(0);wait_count(0,prior_count+1);
    if(locked || !have_rx || timed_out)$fatal(1,"fresh idle failed to unlock");
    prior_count=accepted;mode(1);wait_count(0,prior_count+1);
    if(locked || timed_out)$fatal(1,"fresh motion request failed after idle");
    lost_at=last_rx;start=dut.u_nav.time_us;
    while(!timed_out && dut.u_nav.time_us-lost_at<HOST_TIMEOUT_US+100000)repeat(500)@(negedge clk);
    if(!locked || !timed_out)$fatal(1,"host timeout did not latch lock");
    elapsed=dut.u_nav.time_us-lost_at;
    if(elapsed<HOST_TIMEOUT_US)$fatal(1,"host timeout happened before configured threshold");
    start=dut.u_nav.time_us;
    while(dut.u_nav.r_SAFE_CMD_W[23:16]!=4 && dut.u_nav.time_us-start<100000)repeat(500)@(negedge clk);
    if(dut.u_nav.r_SAFE_CMD_W[23:16]!=4 || dut.u_nav.r_SAFE_CMD_W[15:0]!=0 || dut.u_nav.r_SAFE_CMD_V!=0)
      $fatal(1,"timeout did not publish zero numeric SAFE command with status mode4");
    zero_age=dut.u_nav.time_us-lost_at;
    if(MAX_ZERO_AGE_US>0 && zero_age>MAX_ZERO_AGE_US)
      $fatal(1,"actual zero publication late age_us=%0d limit_us=%0d",zero_age,MAX_ZERO_AGE_US);
    $display("ACTUAL_HOST_TIMEOUT last_rx=%0d observed=%0d elapsed_us=%0d zero_publication_us=%0d",lost_at,lost_at+elapsed,elapsed,dut.u_nav.time_us);
    checks=checks+1;
    prior_count=rejected;mode(1);wait_count(1,prior_count+1);
    if(!locked || !timed_out)$fatal(1,"motion request automatically recovered timed-out task");
    prior_count=accepted;mode(0);wait_count(0,prior_count+1);
    if(locked || timed_out)$fatal(1,"post-timeout fresh idle failed to re-arm");
    checks=checks+1;
    // Independent hardware ESTOP bypasses the software host-lock status mode.
    wr32(16'h5000,5);start=dut.u_nav.time_us;
    while(!dut.ch_stop && dut.u_nav.time_us-start<25000)repeat(500)@(negedge clk);
    if(!dut.ch_stop || dut.nav_forward_q12!=0 || dut.nav_yaw_q12!=0)$fatal(1,"independent hardware ESTOP did not override NAV demand");
    checks=checks+1;
    $display("FULL_G2_UART_CPU TEST PASSED checks=%0d UART_writes=%0d loops=%0d time_us=%0d timeout_elapsed_us=%0d zero_publication_age_us=%0d",checks,writes,dut.u_soc.status,dut.u_nav.time_us,elapsed,zero_age);
    $finish;
  end
  initial begin #1600000000;$fatal(1,"finite actual CPU/UART stimulus watchdog");end
  initial begin
    repeat(16)begin #100000000;$display("ACTUAL_CPU_PROGRESS time_us=%0d loops=%0d flags=%h pc=%h",dut.u_nav.time_us,dut.u_soc.status,guard_flags,dut.u_soc.u_cpu.reg_pc);end
  end
endmodule
