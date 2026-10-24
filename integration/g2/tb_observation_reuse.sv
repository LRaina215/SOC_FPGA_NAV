`timescale 1ns/1ps
module observation_reuse_pair #(parameter LS=-1,RS=-1)(input clk,rst_n,capture,
 input[71:0]gyro,gravity,input signed[15:0]ld,rd,
 input signed[23:0]linear,yaw,height,input[143:0]actions,output valid);
 wire v0,v1,i0,i1;wire[671:0]c0,c1;wire[3359:0]h0,h1;
`ifdef OBS_REUSE_VENDOR
 g2_old a(clk,rst_n,capture,gyro,gravity,ld,rd,linear,yaw,height,actions,v0,i0,c0,h0);
 g2_new b(clk,rst_n,capture,gyro,gravity,ld,rd,linear,yaw,height,actions,v1,i1,c1,h1);
`else
 obs_original #(.LEFT_ENCODER_SIGN(LS),.RIGHT_ENCODER_SIGN(RS)) a(
  clk,rst_n,capture,gyro,gravity,ld,rd,linear,yaw,height,actions,v0,i0,c0,h0);
 obs_candidate #(.LEFT_ENCODER_SIGN(LS),.RIGHT_ENCODER_SIGN(RS)) b(
  clk,rst_n,capture,gyro,gravity,ld,rd,linear,yaw,height,actions,v1,i1,c1,h1);
`endif
 assign valid=v0;
 always @(negedge clk)if({v0,i0,c0,h0}!=={v1,i1,c1,h1})
  $fatal(1,"observation reuse mismatch LS=%0d RS=%0d time=%0t valid=%b/%b",LS,RS,$time,v0,v1);
endmodule

module tb;
 reg clk=0;always #10 clk=~clk;
 reg rst_n=0,capture=0;
 reg[71:0]gyro=0,gravity=0;reg signed[15:0]ld=0,rd=0;
 reg signed[23:0]linear=0,yaw=0,height=0;reg[143:0]actions=0;
 wire valid;
 observation_reuse_pair p0(clk,rst_n,capture,gyro,gravity,ld,rd,linear,yaw,height,actions,valid);
`ifndef OBS_REUSE_VENDOR
 observation_reuse_pair #(.LS(1),.RS(-1)) p1(clk,rst_n,capture,gyro,gravity,ld,rd,linear,yaw,height,actions,);
 observation_reuse_pair #(.LS(-1),.RS(1)) p2(clk,rst_n,capture,gyro,gravity,ld,rd,linear,yaw,height,actions,);
 observation_reuse_pair #(.LS(1),.RS(1)) p3(clk,rst_n,capture,gyro,gravity,ld,rd,linear,yaw,height,actions,);
`endif
 reg[31:0]rng=32'd4500544;integer cycle=0,frames=0;
 function[31:0]next(input[31:0]x);next=x^(x<<13)^(x>>17)^(x<<5);endfunction
 function[23:0]edge_value(input integer index);
  case(index%8)
   0:edge_value=24'h800000;1:edge_value=24'h7fffff;
   2:edge_value=24'hffffff;3:edge_value=0;4:edge_value=1;
   5:edge_value=24'h064000;6:edge_value=24'hf9c000;default:edge_value=rng[23:0];
  endcase
 endfunction
 always @(negedge clk)begin
  cycle=cycle+1;#1;
  rng=next(rng);ld=rng[15:0];rd=rng[31:16];
  for(integer j=0;j<3;j++)begin
   rng=next(rng);gyro[j*24+:24]=rng[23:0];
   rng=next(rng);gravity[j*24+:24]=rng[23:0];
  end
  for(integer j=0;j<6;j++)begin rng=next(rng);actions[j*24+:24]=rng[23:0];end
  rng=next(rng);linear=rng[23:0];rng=next(rng);yaw=rng[23:0];rng=next(rng);height=rng[23:0];
  if(cycle%1700==1)begin
   for(integer j=0;j<3;j++)begin gyro[j*24+:24]=edge_value(cycle/1700+j);gravity[j*24+:24]=edge_value(cycle/1700+j+3);end
   for(integer j=0;j<6;j++)actions[j*24+:24]=edge_value(cycle/1700+j);
   linear=edge_value(cycle/1700);yaw=edge_value(cycle/1700+1);height=edge_value(cycle/1700+2);
   case((cycle/1700)%4)
    0:begin ld=-32768;rd=32767;end
    1:begin ld=32767;rd=-32768;end
    2:begin ld=0;rd=-1;end
    default:begin end
   endcase
  end
  capture=cycle%1700==1 || cycle%1700==2 || cycle%1700==27;
  rst_n=cycle>=5 && !(cycle>=10300 && cycle<10305);
  if(valid)frames++;
  if(cycle==56000)begin
   if(frames<25)$fatal(1,"insufficient complete frames");
   $display("TEST PASSED cycles=%0d frames=%0d signed24_extremes=1 changing_inputs=1 busy_capture=1 reset_busy=1",cycle,frames);$finish;
  end
 end
 initial begin #1500000;$fatal(1,"simulation timeout");end
endmodule
