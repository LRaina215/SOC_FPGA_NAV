`timescale 1ns/1ps
// Finite related-clock transport/guard proof. No4500 inference is substituted
// into a whole-design acceptance claim: this fixture tests only these channels.
module tb_model25_transport;
 reg clk50=0,clk25=0,run25=1,rst_n=0,lock=0,force_transport_fault=0;
 always #10 clk50=~clk50;
 always #20 if(run25)clk25=~clk25;
 wire io_n,model_n,fault;
 g2_model25_guard guard(.clk50(clk50),.clk25(clk25),.reset_n(rst_n),.pll_lock(lock),
  .transport_fault(force_transport_fault),.io_rst_n(io_n),.model_rst_n(model_n),.fault(fault));
 reg sv=0;reg[127:0]sd=0;wire ready,overflow,dv;wire[127:0]dd;
 g2_model25_mailbox #(.WIDTH(128)) event50to25(.src_clk(clk50),.src_rst_n(io_n),.src_valid(sv),.src_data(sd),
  .src_ready(ready),.src_overflow(overflow),.dst_clk(clk25),.dst_rst_n(model_n),.dst_valid(dv),.dst_data(dd));
 reg rv=0;reg[335:0]rd=0;wire rready,roverflow,rdv;wire[335:0]rdd;
 g2_model25_mailbox #(.WIDTH(336)) event25to50(.src_clk(clk25),.src_rst_n(model_n),.src_valid(rv),.src_data(rd),
  .src_ready(rready),.src_overflow(roverflow),.dst_clk(clk50),.dst_rst_n(io_n),.dst_valid(rdv),.dst_data(rdd));
 reg[139:0]latest=0;wire lv;wire[139:0]ld;
 g2_model25_latest #(.WIDTH(140)) atomic_latest(.src_clk(clk50),.src_rst_n(io_n),.src_data(latest),
  .dst_clk(clk25),.dst_rst_n(model_n),.dst_valid(lv),.dst_data(ld));
 integer sent=0,got=0,rsent=0,rgot=0,latest_checks=0,reset_cases=0,fault_cases=0;
 reg compare=0,previous_dv=0,previous_rdv=0;
 function[127:0]word128(input integer k);word128={32'(k),32'(~k),32'(k*7),32'(k^32'h8ad3f225)};endfunction
 function[335:0]word336(input integer k);word336={word128(k),word128(~k),64'(k*157),16'(k)};endfunction
 always @(negedge clk25)begin
  if(model_n&&compare)begin
   if(dv)begin
    if(previous_dv)$fatal(1,"destination event must be one clock");
    if(dd!==word128(got))$fatal(1,"50to25 mismatch index%0d",got);
    got=got+1;
   end
   if(lv)begin
    if(ld[31:0]!==~ld[63:32])$fatal(1,"atomic latest pair tore");
    latest_checks=latest_checks+1;
   end
  end
  previous_dv=dv;
 end
 always @(negedge clk50)begin
  if(io_n&&compare&&rdv)begin
   if(previous_rdv)$fatal(1,"reverse event duplicated");
   if(rdd!==word336(rgot))$fatal(1,"25to50 mismatch index%0d",rgot);
   rgot=rgot+1;
  end
  previous_rdv=rdv;
 end
 task reset_both;
  begin
   compare=0;sv=0;rv=0;rst_n=0;lock=0;force_transport_fault=0;
   #83;if(io_n!==0||model_n!==0)$fatal(1,"async reset did not assert");
   run25=1;#54;rst_n=1;lock=1;
   repeat(12)@(negedge clk50);
   if(fault||!io_n||!model_n)$fatal(1,"local release failed");
   reset_cases=reset_cases+1;
  end
 endtask
 initial begin
  latest={76'd0,32'hffffffff,32'd0};
  reset_both();compare=1;
  fork
   begin
    repeat(1000)begin
     @(negedge clk50);while(!ready)@(negedge clk50);
     sd=word128(sent);sv=1;sent=sent+1;
     @(negedge clk50);sv=0;
    end
   end
   begin
    repeat(1000)begin
     @(negedge clk25);while(!rready)@(negedge clk25);
     rd=word336(rsent);rv=1;rsent=rsent+1;
     @(negedge clk25);rv=0;
    end
   end
   begin
    for(integer k=1;k<=10000;k=k+1)begin
     @(negedge clk50);latest={76'd0,32'(~k),32'(k)};
    end
   end
  join
  repeat(30)@(negedge clk50);
  if(got!=1000||rgot!=1000||overflow||roverflow||fault)$fatal(1,"transaction count/fault");
  if(ld!==latest)$fatal(1,"latest update not eventually delivered");
  // Clock stopped in a transaction; common reset must suppress the old record.
  compare=0;@(negedge clk25);run25=0;
  @(negedge clk50);sv=1;sd='hbad;
  @(negedge clk50);sv=0;#30;rst_n=0;#1;
  if(io_n!==0||model_n!==0)$fatal(1,"reset with stopped clock");
  #80;rst_n=1;repeat(4)@(negedge clk50);
  if(model_n!==0)$fatal(1,"stopped domain released without clock");
  reset_both();repeat(20)@(negedge clk50);
  if(dv||rdv)$fatal(1,"old transaction replayed after reset");
  @(negedge clk25);run25=0;
  repeat(24)@(negedge clk50);
  if(!fault||io_n||model_n)$fatal(1,"clock halt not latched");
  fault_cases=fault_cases+1;run25=1;repeat(20)@(negedge clk50);
  if(!fault)$fatal(1,"clock restart auto-cleared fault");
  reset_both();lock=0;repeat(6)@(negedge clk50);
  if(!fault)$fatal(1,"PLL loss not latched");
  fault_cases=fault_cases+1;lock=1;repeat(10)@(negedge clk50);
  if(!fault)$fatal(1,"PLL recovery auto-cleared fault");
  reset_both();force_transport_fault=1;repeat(3)@(negedge clk50);
  if(!fault)$fatal(1,"transport failure not latched");
  fault_cases=fault_cases+1;
  $display("MODEL25_TRANSPORT_PASS forward=%0d reverse=%0d latest_checks=%0d resets=%0d faults=%0d",got,rgot,latest_checks,reset_cases,fault_cases);
  $finish;
 end
 initial begin #2000000;$fatal(1,"fixture watchdog");end
endmodule
