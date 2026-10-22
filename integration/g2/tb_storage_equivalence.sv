`timescale 1ns/1ps
module ram_pair #(parameter DW=16,AW=10,DEPTH=(1<<AW),parameter IV=0)(
 input clk,we,re,input [15:0] wa,ra,input [63:0] wd);
 wire [DW-1:0] oldq,newq;
 wire [AW-1:0] w=wa%DEPTH,r=ra%DEPTH;
 nav_ram_original #(.DW(DW),.AW(AW),.DEPTH(DEPTH),.INIT_VAL(IV)) oldmem(clk,we,w,wd[DW-1:0],re,r,oldq);
 nav_ram_sdp #(.DW(DW),.AW(AW),.DEPTH(DEPTH),.INIT_VAL(IV)) newmem(clk,we,w,wd[DW-1:0],re,r,newq);
 always @(negedge clk)if($time>20 && oldq!==newq)$fatal(1,"RAM mismatch DW=%0d AW=%0d",DW,AW);
endmodule

module tb_storage_equivalence;
 reg clk=0;always #10 clk=~clk;
 reg rst_n=0,wr_en=0,wr_history=0,start=0;reg [7:0] wr_addr=0;
 reg signed [23:0] wr_data=0;reg [31:0] frame_id=0;
 wire b0,b1,d0,d1,s0,s1,v0,v1;wire [4:0] e0,e1;
 wire [31:0] id0,id1,c0,c1;wire [143:0] a0,a1;
 wire [2:0] l0,l1;wire [7:0] n0,n1;wire signed [23:0] z0,z1;
 fc_original oldcore(.clk(clk),.rst_n(rst_n),.wr_en(wr_en),.wr_history(wr_history),.wr_addr(wr_addr),.wr_data(wr_data),.start(start),.frame_id(frame_id),
 .busy(b0),.done(d0),.error_flags(e0),.saturated(s0),.result_frame_id(id0),.cycles(c0),
 .action0(a0[23:0]),.action1(a0[47:24]),.action2(a0[71:48]),.action3(a0[95:72]),.action4(a0[119:96]),.action5(a0[143:120]),
 .dbg_valid(v0),.dbg_layer(l0),.dbg_index(n0),.dbg_data(z0));
 fc_v4_top newcore(.clk(clk),.rst_n(rst_n),.wr_en(wr_en),.wr_history(wr_history),.wr_addr(wr_addr),.wr_data(wr_data),.start(start),.frame_id(frame_id),
 .busy(b1),.done(d1),.error_flags(e1),.saturated(s1),.result_frame_id(id1),.cycles(c1),
 .action0(a1[23:0]),.action1(a1[47:24]),.action2(a1[71:48]),.action3(a1[95:72]),.action4(a1[119:96]),.action5(a1[143:120]),
 .dbg_valid(v1),.dbg_layer(l1),.dbg_index(n1),.dbg_data(z1));
 integer neurons=0,frames=0,cycles=0;
 always @(negedge clk)if(rst_n)begin
   cycles=cycles+1;
   if({b0,d0,e0,s0,id0,c0,v0}!=={b1,d1,e1,s1,id1,c1,v1})$fatal(1,"core cycle/status mismatch");
   if(v0)begin
     if({l0,n0,z0}!=={l1,n1,z1})$fatal(1,"neuron mismatch layer%0d index%0d",l0,n0);
     neurons=neurons+1;
   end
   if(d0)begin if(a0!==a1)$fatal(1,"actions mismatch");frames=frames+1;end
   if(cycles>600000)$fatal(1,"timeout");
 end
 reg ram_we=0,ram_re=0;reg [15:0] wa=0,ra=0;reg [63:0] wd=0;
 ram_pair #(.DW(53),.AW(14)) r0(clk,ram_we,ram_re,wa,ra,wd);
 ram_pair #(.DW(18),.AW(16)) r1(clk,ram_we,ram_re,wa,ra,wd);
 ram_pair #(.DW(36),.AW(12)) r2(clk,ram_we,ram_re,wa,ra,wd);
 ram_pair #(.DW(8),.AW(14),.IV(255)) r3(clk,ram_we,ram_re,wa,ra,wd);
 ram_pair #(.DW(16),.AW(15),.DEPTH(19200)) r4(clk,ram_we,ram_re,wa,ra,wd);
 reg ram_done=0;
 integer j;
 initial begin
   repeat(5)@(negedge clk);
   for(j=0;j<196608;j=j+1)begin
      @(posedge clk);#1;
      ram_we=j<65536 || j>=131072 && j%3!=0;
      ram_re=j<131072 || j%5!=0;
      wa=j[15:0];ra=j<131072?j[15:0]:(j%7==0?j[15:0]:(j*173+7)&65535);
      wd={j[15:0],~j[15:0],j[15:0],j[15:0]^16'h5aa5};
   end
   @(negedge clk);ram_done=1;
 end
 reg [23:0] vectors[0:503];integer f,i;
 initial begin
   $readmemh("vectors.mem",vectors);
   repeat(5)@(negedge clk);rst_n=1;
   for(f=0;f<3;f=f+1)begin
     for(i=0;i<168;i=i+1)begin
       @(negedge clk);wr_en=1;wr_history=i>=28;wr_addr=i>=28?i-28:i;wr_data=vectors[f*168+i];
     end
     @(negedge clk);wr_en=0;start=1;frame_id=f+1;
     @(negedge clk);start=0;
     wait(d0);repeat(3)@(negedge clk);
   end
   wait(ram_done);repeat(3)@(negedge clk);
   if(frames!=3 || neurons!=1275)$fatal(1,"incomplete frame/neuron comparisons");
   $display("TEST PASSED: 3 frames,1275 neurons,cycle-exact actions;5 RAM geometries x196608 cycles");$finish;
 end
endmodule
