`timescale 1ns/1ps
`ifndef EXPECT_STICKY
`define EXPECT_STICKY 0
`endif
module tb_model25_short_lock;
 reg c50=0,c25=0,rn=0,lock=0;always #10 c50=~c50;always #20 c25=~c25;
 wire ir,mr,f;
 g2_model25_guard guard(.clk50(c50),.clk25(c25),.reset_n(rn),.pll_lock(lock),
  .transport_fault(1'b0),.io_rst_n(ir),.model_rst_n(mr),.fault(f));
 initial begin
  #100;rn=1;lock=1;repeat(20)@(negedge c50);
  if(!ir||!mr||f)$fatal(1,"not operating before loss");
  // Strictly between50MHz sampling edges: both domains asynchronously reset.
  #3;lock=0;#0.1;if(ir||mr)$fatal(1,"PLL loss did not assert reset");
  #1.9;lock=1;repeat(30)@(negedge c50);
  if(`EXPECT_STICKY)begin
   if(!f||ir||mr)$fatal(1,"short loss auto-recovered");
   rn=0;repeat(3)@(negedge c50);rn=1;repeat(20)@(negedge c50);
   if(f||!ir||!mr)$fatal(1,"explicit physical reset failed to rearm");
   $display("MODEL25_SHORT_LOCK_FIXED_PASS pulse_ns=2 fault_sticky_until_physical_reset=1");
  end else begin
   if(f||!ir||!mr)$fatal(1,"old counterexample not reproduced");
   $display("MODEL25_SHORT_LOCK_COUNTEREXAMPLE pulse_ns=2 safety_contract_pass=0 auto_recovered=1");
  end
  $finish;
 end
 initial begin #100000;$fatal(1,"short loss fixture watchdog");end
endmodule
