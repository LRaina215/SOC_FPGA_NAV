`timescale 1ns/1ps
module fusion_pair #(parameter HZ=50000000)(input clk,rst_n,sample,
 input signed[15:0] ax,ay,az,wx,wy,wz,output wire valid);
 wire [71:0] g0,g1,w0,w1;wire v0,v1,cal0,cal1,b0,b1,r0,r1;
 fusion_original #(.CLK_HZ(HZ),.BIAS_SAMPLES(4)) original(
  clk,rst_n,sample,ax,ay,az,wx,wy,wz,w0[23:0],w0[47:24],w0[71:48],g0[23:0],g0[47:24],g0[71:48],v0,cal0,b0,r0);
 imu_fusion #(.CLK_HZ(HZ),.BIAS_SAMPLES(4)) candidate(
  clk,rst_n,sample,ax,ay,az,wx,wy,wz,w1[23:0],w1[47:24],w1[71:48],g1[23:0],g1[47:24],g1[71:48],v1,cal1,b1,r1);
 assign valid=v0;
 integer frames=0,accepted=0,rejected=0;
 always @(negedge clk)begin
  if({g0,w0,v0,cal0,b0,r0}!=={g1,w1,v1,cal1,b1,r1})$fatal(1,"fusion external cycle mismatch HZ=%0d",HZ);
  if($time>0 && {original.st,original.sqrt_start,original.sqrt_value}!=={candidate.st,candidate.sqrt_start,candidate.sqrt_value})
   $fatal(1,"fusion state or sqrt input cycle mismatch HZ=%0d",HZ);
  if(v0)begin frames++;if(r0)rejected++;else accepted++;end
 end
endmodule
module tb_fusion_equivalence;
 reg clk=0;always #10 clk=~clk;
 reg rst_n=0,sample=0;reg signed[15:0] ax=0,ay=0,az=10920,wx=2,wy=-3,wz=1;
 wire v0,v1,vm;
 fusion_pair p0(clk,rst_n,sample,ax,ay,az,wx,wy,wz,v0);
 fusion_pair #(.HZ(100000)) p1(clk,rst_n,sample,ax,ay,az,wx,wy,wz,v1);
 fusion_pair pm(clk,rst_n,1'b0,16'sd0,16'sd0,16'sd0,16'sd0,16'sd0,16'sd0,vm);
 function[31:0] next(input[31:0] x);next=x^(x<<13)^(x>>17)^(x<<5);endfunction
 reg[31:0] rng=4500,math_rng=32'h8884500;integer cycle=0,math_cases=0,busy_reset=0;
 reg math_done=0;reg signed[23:0] mx,my,mz;
 initial begin
  wait(rst_n);repeat(3)@(negedge clk);
  for(integer k=0;k<64;k++)begin
   math_rng=next(math_rng);mx=math_rng[23:0];math_rng=next(math_rng);my=math_rng[23:0];math_rng=next(math_rng);mz=math_rng[23:0];
   case(k%8)
    0:begin mx=0;my=0;mz=0;end
    1:begin mx=-8388608;my=8388607;mz=-1;end
    2:begin mx=8388607;my=8388607;mz=8388607;end
    3:begin mx=-8388608;my=-8388608;mz=-8388608;end
    4:begin mx=0;my=0;mz=-4096;end
    5:begin mx=4096;my=0;mz=0;end
   endcase
   @(negedge clk);#1;
   pm.original.egx=mx;pm.original.egy=my;pm.original.egz=mz;
   pm.candidate.egx=mx;pm.candidate.egy=my;pm.candidate.egz=mz;
   pm.original.avx=0;pm.original.avy=0;pm.original.avz=0;
   pm.candidate.avx=0;pm.candidate.avy=0;pm.candidate.avz=0;
   pm.original.st=9;pm.candidate.st=9;
   @(negedge clk);while(!vm)@(negedge clk);
   math_cases++;repeat(2)@(negedge clk);
  end
  math_done=1;
 end
 always @(negedge clk)begin
  cycle++;
  rng=next(rng);wx=$signed(rng[15:0])%100;rng=next(rng);wy=$signed(rng[15:0])%100;wz=wx-wy;
  ax=0;ay=0;az=10920;
  case((cycle/700)%8)
   1:begin ax=-32768;ay=32767;az=0;end
   2:begin ax=3735;az=10261;end
   3:begin ax=0;ay=0;az=0;end
   4:begin wx=10000;wy=-10000;wz=12000;end
   5:begin rng=next(rng);ax=rng[15:0];ay=rng[31:16];rng=next(rng);az=rng[15:0];end
   6:begin ax=10920;ay=0;az=0;end
   7:begin ax=0;ay=-10920;az=0;end
  endcase
  if(p0.cal0)begin wx=2;wy=-3;wz=1;end
  sample=cycle%700==0 || (cycle>30000 && (cycle%5000==1 || cycle%5000==2));
  if(cycle==90305 && p0.original.st!=2 && p0.original.st!=0)busy_reset++;
  rst_n=cycle>=5 && !(cycle>=90305 && cycle<90310);
  if(cycle==200000)begin
   if(!math_done || math_cases!=64 || busy_reset==0 || p0.frames<200 || p1.frames<200 || p0.accepted==0 || p0.rejected==0)$fatal(1,"insufficient fusion stimulus coverage");
   $display("TEST PASSED frames50m=%0d frames100k=%0d accepted=%0d rejected=%0d math=%0d busy_reset=%0d",p0.frames,p1.frames,p0.accepted,p0.rejected,math_cases,busy_reset);$finish;
  end
 end
endmodule
