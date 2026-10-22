`timescale 1ns/1ps
// Compare actual Gowin primitive models, not inferred behavioral RAMs.
// Same-address collisions are excluded: device manual does not guarantee them.
module primitive_pair #(parameter W=32)(input clk,we,re,input[8:0] wa,ra,
 input[35:0] wd,input[3:0] mask);
 wire [35:0] native,split;
 generate if(W==32)begin:g32
   SDPB #(.BIT_WIDTH_0(32),.BIT_WIDTH_1(32)) n(.DO(native[31:0]),.DI(wd[31:0]),
    .CLKA(clk),.CLKB(clk),.CEA(we),.CEB(re),.OCE(1'b0),.RESET(1'b0),
    .ADA({wa,1'b0,mask}),.ADB({ra,5'b0}),.BLKSELA(3'b0),.BLKSELB(3'b0));
   for(genvar i=0;i<2;i++)begin:b
     wire[31:0] q;
     SDPB #(.BIT_WIDTH_0(16),.BIT_WIDTH_1(16)) s(.DO(q),.DI({16'b0,wd[i*16+:16]}),
      .CLKA(clk),.CLKB(clk),.CEA(we),.CEB(re),.OCE(1'b0),.RESET(1'b0),
      .ADA({1'b0,wa,2'b0,mask[i*2+:2]}),.ADB({1'b0,ra,4'b0}),.BLKSELA(3'b0),.BLKSELB(3'b0));
     assign split[i*16+:16]=q[15:0];
   end
   assign native[35:32]=0;assign split[35:32]=0;
 end else begin:g36
   SDPX9B #(.BIT_WIDTH_0(36),.BIT_WIDTH_1(36)) n(.DO(native),.DI(wd),
    .CLKA(clk),.CLKB(clk),.CEA(we),.CEB(re),.OCE(1'b0),.RESET(1'b0),
    .ADA({wa,1'b0,mask}),.ADB({ra,5'b0}),.BLKSELA(3'b0),.BLKSELB(3'b0));
   for(genvar i=0;i<2;i++)begin:b
     wire[35:0] q;
     SDPX9B #(.BIT_WIDTH_0(18),.BIT_WIDTH_1(18)) s(.DO(q),.DI({18'b0,wd[i*18+:18]}),
      .CLKA(clk),.CLKB(clk),.CEA(we),.CEB(re),.OCE(1'b0),.RESET(1'b0),
      .ADA({1'b0,wa,2'b0,mask[i*2+:2]}),.ADB({1'b0,ra,4'b0}),.BLKSELA(3'b0),.BLKSELB(3'b0));
     assign split[i*18+:18]=q[17:0];
   end
 end endgenerate
 reg[W-1:0] model[0:511];reg[W-1:0] expected;integer j;
 initial begin for(j=0;j<512;j++)model[j]=0;expected=0;end
 always @(posedge clk)begin
   if(re)expected=model[ra];
   if(we)for(integer k=0;k<4;k++)if(mask[k])model[wa][k*(W/4)+:(W/4)]=wd[k*(W/4)+:(W/4)];
   #1;if($time>100 && (native!==split || native[W-1:0]!==expected))
       $fatal(1,"primitive mismatch width%0d actual%h split%h expected%h",W,native,split,expected);
 end
endmodule
module tb_native_bsram;
 reg clk=0;always #10 clk=~clk;
 GSR GSR(.GSRI(1'b1));
 reg we=0,re=0;reg[8:0] wa=0,ra=0;reg[35:0] wd=0;reg[3:0] mask=15;
 primitive_pair #(.W(32)) p32(clk,we,re,wa,ra,wd,mask);
 primitive_pair #(.W(36)) p36(clk,we,re,wa,ra,wd,mask);
 integer j;
 initial begin
  repeat(6)@(negedge clk);
  for(j=0;j<8192;j++)begin
   @(negedge clk);wa=j[8:0];ra=(j*37+19)&511;
   we=j<512 || j>=1024 && j%3!=0;re=j>=512 && j%5!=0;
   if(we && ra==wa)ra=ra+1'b1;
   wd={j[17:0]^18'h2d567,j[17:0]*18'd73};mask=j<512?15:j%16;
  end
  @(negedge clk);we=0;re=0;repeat(3)@(negedge clk);
  $display("TEST PASSED: 32/36 native vs split primitives and independent byte-enable model,8192 cycles");$finish;
 end
endmodule
