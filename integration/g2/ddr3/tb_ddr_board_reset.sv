`timescale 1ns/1ps
module tb_ddr_board_reset;
  reg clk=0,board_reset=1,memory_fault=0;
  wire power_reset,protocol_reset,sticky_fault;
  integer checks=0;
  always #10 clk=~clk;
  nav_ddr_board_reset #(.STARTUP_CYCLES(10000)) dut(.*);
  task tick; begin @(posedge clk); #1; end endtask
  task expect_state(input bit p,input bit r,input bit f);
    begin
      checks=checks+1;
      if({power_reset,protocol_reset,sticky_fault}!=={p,r,f})
        $fatal(1,"reset state got=%b%b%b expected=%b%b%b",power_reset,protocol_reset,sticky_fault,p,r,f);
    end
  endtask
  initial begin
    tick();expect_state(1,1,0);
    @(negedge clk);board_reset=0;
    repeat(9998)tick();expect_state(1,1,0);
    tick();expect_state(0,1,0);
    tick();expect_state(0,0,0);
    @(negedge clk);memory_fault=1;
    tick();expect_state(0,0,1);
    @(negedge clk);memory_fault=0;board_reset=1;
    #1;expect_state(0,1,1);
    repeat(3)tick();expect_state(0,1,1);
    @(negedge clk);board_reset=0;
    repeat(9999)tick();expect_state(0,1,1);
    tick();expect_state(0,0,1);
    repeat(20)tick();expect_state(0,0,1);
    $display("TEST PASSED BOARD_RESET checks=%0d startup_cycles=10000 clock_ns=20",checks);
    $finish;
  end
  initial begin #500000;$fatal(1,"finite reset stimulus incomplete");end
endmodule
