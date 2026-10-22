`timescale 1ns/1ps
module tb_command_lease;
 reg clk=0;always #5 clk=~clk;
 reg reset_n=0,rx=1;
 wire signed [23:0] uv,uw;wire seen,uvld;
 uart_speed_rx #(.CLKS_PER_BIT(8),.FRAME_TIMEOUT_CLKS(500)) parser(clk,reset_n,rx,uv,uw,seen,uvld);
 reg [1:0] owner=2,source=2;
 reg [31:0] session=1,psession=1;
 reg [15:0] seq=0,uart_seq=0;
 always @(posedge clk)if(!reset_n)uart_seq<=0;else if(uvld)uart_seq<=uart_seq+1;
 reg direct=0,valid=0,fault=0,estop=0,feedback=1;
 reg signed [23:0] v=0,w=0;
 reg signed [15:0] mv=0,mw=0;
 wire signed [23:0] ov,ow;
 wire armed,expired,stopped,balance;wire [31:0] age,accepted,rejected;
 command_lease_guard #(.CLK_HZ(50000),.STOP_DWELL_MS(10)) dut(
 .clk(clk),.reset_n(reset_n),.owner_select(owner),.source_id(source),
 .session_select(session),.packet_session(psession),.packet_seq(direct?seq:uart_seq),
 .valid(direct?valid:uvld),.source_fault(fault),.estop(estop),
 .forward_q12(direct?v:uv),.yaw_q12(direct?w:uw),.feedback_valid(feedback),
 .measured_vx_mmps(mv),.measured_wz_mradps(mw),.out_forward_q12(ov),.out_yaw_q12(ow),
 .armed(armed),.expired(expired),.stop_confirmed(stopped),.age_ms(age),
 .accepted_count(accepted),.rejected_count(rejected),.balance_enable(balance));
 integer checks=0;reg signed [23:0] pv=0,pw=0;
 always @(negedge clk)if(reset_n)begin
  if(!balance)$fatal(1,"communication disabled balance");
  if(!estop && ($signed(ov)-$signed(pv)>41 || $signed(pv)-$signed(ov)>41 ||
                $signed(ow)-$signed(pw)>410 || $signed(pw)-$signed(ow)>410))$fatal(1,"unbounded ramp");
  pv=ov;pw=ow;
 end else begin pv=0;pw=0;end
 task clocks(input integer n);repeat(n)@(negedge clk);endtask
 task ms(input integer n);clocks(n*50);endtask
 task check(input bit ok,input string name);
  if(!ok)$fatal(1,"%s",name);checks++;$display("PASS %s",name);
 endtask
 function [15:0] crcbyte(input [15:0] prev,input [7:0] b);
  reg[15:0] c;integer i;begin c=prev^{b,8'b0};for(i=0;i<8;i++)c=c[15]?(c<<1)^16'h1021:c<<1;crcbyte=c;end
 endfunction
 task byte_tx(input [7:0] b);
  rx=0;clocks(8);for(integer i=0;i<8;i++)begin rx=b[i];clocks(8);end rx=1;clocks(8);
 endtask
 task frame(input signed [15:0] fv,fw,input bit corrupt);
  reg[15:0] c;begin
   c=16'hffff;c=crcbyte(c,8'h10);c=crcbyte(c,8'h04);
   c=crcbyte(c,fv[7:0]);c=crcbyte(c,fv[15:8]);c=crcbyte(c,fw[7:0]);c=crcbyte(c,fw[15:8]);
   if(corrupt)c=c^16'h0100;
   byte_tx('ha5);byte_tx('h5a);byte_tx('h10);byte_tx('h04);
   byte_tx(fv[7:0]);byte_tx(fv[15:8]);byte_tx(fw[7:0]);byte_tx(fw[15:8]);
   byte_tx(c[7:0]);byte_tx(c[15:8]);clocks(8);
  end
 endtask
 task packet(input [15:0] sn,input signed [23:0] fv,fw);
  seq=sn;v=fv;w=fw;valid=1;clocks(1);valid=0;clocks(2);
 endtask
 integer oldacc;
 initial begin
  clocks(5);reset_n=1;ms(12);
  check(stopped && !armed,"reset awaits fresh zero and measured stop");
  frame(100,200,0);check(!armed,"nonzero cannot arm after reset");
  frame(0,0,0);check(armed && !expired,"CRC-valid zero arms");
  frame(100,200,0);ms(12);check(ov==410 && ow==819,"atomic UART forward/yaw and Q12 conversion");
  ms(210);check(expired && !armed && ov==0 && ow==0,"disconnect controlled ramp to zero, balance active");
  frame(100,200,0);check(!armed,"nonzero does not automatically resume");
  frame(0,0,0);frame(100,200,0);oldacc=accepted;
  repeat(15)frame(200,400,1);ms(20);
  check(expired && accepted==oldacc,"continuous bad CRC does not renew lease");
  frame(0,0,0);frame(601,0,0);check(accepted==oldacc+1,"range error does not refresh valid command");
  byte_tx('ha5);byte_tx('h5a);byte_tx('h10);byte_tx('h04);byte_tx('h01);ms(220);
  check(expired && !armed,"partial packet cannot refresh command age");
  direct=1;owner=1;source=1;session=2;psession=2;ms(12);
  packet(65534,0,0);check(armed,"new owner requires own zero and measured stop");
  packet(65535,800,-1200);ms(25);check(ov==800 && ow==-1200,"direct owner target");
  oldacc=accepted;packet(65535,400,400);packet(65534,400,400);
  check(accepted==oldacc,"duplicate and late sequence rejected");
  packet(0,500,600);check(accepted==oldacc+1,"sequence wrap accepted within half range");
  source=2;packet(1,100,100);source=1;psession=1;packet(1,100,100);psession=2;
  check(accepted==oldacc+1,"wrong owner and stale session rejected");
  feedback=0;ms(60);check(expired && !armed,"missing actual feedback revokes motion");
  feedback=1;mv=6;ms(100);packet(1,0,0);check(!armed && !stopped,"zero request with measured drift cannot recover");
  mv=0;mw=30;ms(15);packet(1,0,0);check(!armed && !stopped,"mean wheel zero with yaw drift cannot recover");
  mw=0;ms(15);packet(1,0,0);check(armed,"fresh stopped zero recovers with later sequence");
  packet(2,800,1200);ms(25);owner=2;source=2;session=3;psession=3;clocks(2);
  check(!armed,"owner switch clears lease");
  ms(30);packet(1,800,1200);check(!armed,"new owner cannot replay nonzero target");
  ms(15);packet(1,0,0);packet(2,800,1200);ms(25);
  fault=1;clocks(2);fault=0;ms(30);check(!armed && ov==0,"source fault stops and requires rearm");
  ms(15);packet(3,0,0);packet(4,800,1200);ms(25);estop=1;clocks(2);
  check(ov==0 && ow==0 && !armed,"physical emergency overrides ramp");
  estop=0;ms(15);packet(4,800,1200);check(!armed,"estop release cannot replay last target");
  reset_n=0;clocks(2);reset_n=1;ms(15);packet(5,800,1200);
  check(!armed,"reset does not restore previous motion");
  $display("TEST PASSED %0d contracts",checks);$finish;
 end
 initial begin clocks(1000000);$fatal(1,"test timeout");end
endmodule
