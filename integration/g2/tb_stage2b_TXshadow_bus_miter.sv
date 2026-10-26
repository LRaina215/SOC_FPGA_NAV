`timescale 1ns/1ps
module tb_stage2b_TXshadow_bus_miter;
 reg clk=0;always #10 clk=~clk;reg rst=1;
 reg q0=0,w0=0,q1=0,w1=0;reg[15:0]a0=0,a1=0;reg[31:0]d0=0,d1=0;
 wire k00,k01,k10,k11;wire[31:0]r00,r01,r10,r11;
 shadow_reference ref0(clk,rst,q0,w0,a0,d0,k00,r00,q1,w1,a1,d1,k01,r01);
 shadow_candidate dut(clk,rst,q0,w0,a0,d0,k10,r10,q1,w1,a1,d1,k11,r11);
 reg[31:0]gold[0:511],expected=0;integer checks=0,reads=0,writes=0,arbitration=0;
 reg[9:0]gwp=0;
 always@(posedge clk)begin
  if(dut.shadow_read&&dut.tx_we&&!dut.mbx.twp[9])$fatal(1,"physical CE overlap independent of AD[13:4]");
  if(ref0.tx_we&&!gwp[9])begin gold[gwp[8:0]]<=ref0.wd_q;writes++;end
  if(rst)gwp<=0;
  else begin
   if(ref0.tx_we&&!gwp[9])gwp<=gwp+1;
   if(ref0.tx_go)gwp<=0;
  end
  if(!rst&&ref0.bs==0&&ref0.bus_req&&{ref0.bus_addr[15:2],2'b00}==16'h1240)begin expected<=gold[ref0.txb_addr[8:0]];reads++;end
  if(q0&&q1)arbitration++;
 end
 always@(negedge clk)if($time>0&&!rst)begin
  checks++;
  if($isunknown({k00,k01,k10,k11})||{k00,k01}!=={k10,k11})$fatal(1,"master ACK mismatch");
  if(k00&&($isunknown({r00,r10})||r00!==r10))$fatal(1,"master0 data mismatch");
  if(k01&&($isunknown({r01,r11})||r01!==r11))$fatal(1,"master1 data mismatch");
  if(ref0.bus_ack&&ref0.a_q==16'h1240&&ref0.bus_rdata!==expected)$fatal(1,"independent TX shadow word mismatch");
 end
 task automatic bus0(input bit wr,input[15:0]a,input[31:0]d);
  begin
   @(negedge clk);#1;q0=1;w0=wr;a0=a;d0=d;
   do @(posedge clk);while(!k00);
   @(negedge clk);#1;q0=0;repeat(2)@(negedge clk);
  end
 endtask
 task automatic bus1(input bit wr,input[15:0]a,input[31:0]d);
  begin
   @(negedge clk);#1;q1=1;w1=wr;a1=a;d1=d;
   do @(posedge clk);while(!k01);
   @(negedge clk);#1;q1=0;repeat(2)@(negedge clk);
  end
 endtask
 initial begin
  for(integer i=0;i<512;i++)gold[i]=0;
  repeat(5)@(negedge clk);rst=0;
  // Default readback index0 plus first write was the old unwanted collision.
  bus0(1,16'h1220,32'h12345678);bus0(0,16'h1240,0);
  bus0(1,16'h1240,32'hffffffff); // RO write still returns the original RF value.
  for(integer i=1;i<512;i++)bus0(1,16'h1220,32'h85000000^i);
  bus0(1,16'h1220,32'hbadbad00); // Full buffer ignores extra write as before.
  for(integer i=0;i<512;i++)begin bus0(1,16'h123c,i);bus0(0,16'h1240,0);end
  bus0(1,16'h1228,0);bus0(1,16'h123c,0);
  fork
   begin for(integer i=0;i<32;i++)begin bus0(1,16'h1220,32'h55aa0000^i);bus0(0,16'h1240,0);end end
   begin for(integer j=0;j<32;j++)begin bus1(1,16'h123c,j);bus1(0,16'h1240,0);end end
  join
  @(negedge clk);#1;rst=1;repeat(3)@(negedge clk);rst=0;
  for(integer i=0;i<512;i++)begin bus1(1,16'h123c,i);bus1(0,16'h1240,0);end
  if(reads<1024||writes<544||arbitration==0)$fatal(1,"incomplete TX shadow coverage");
  $display("TEST PASSED TXSHADOW BUS checks=%0d reads=%0d writes=%0d arbitration=%0d",checks,reads,writes,arbitration);$finish;
 end
 initial begin #3000000;$fatal(1,"finite TX shadow bus stimulus watchdog");end
endmodule
