`timescale 1ns/1ps
// Future RTL-only contract fixture. Never starts tools by being prepared.
module memory_contract #(parameter DW=16,AW=10,DEPTH=(1<<AW),ID=0,
  parameter [DW-1:0] IV=0,parameter FILE="",parameter READONLY=0)(input clk,output reg done=0);
  reg we=0,re=0;reg[AW-1:0]wa=0,ra=0;reg[DW-1:0]wd=0;
  wire[DW-1:0]a,b;reg[DW-1:0]gold[0:DEPTH-1],expected=0;
  reg[31:0]rng=32'h2b450026^ID;integer checks=0,collisions=0,holds=0;
  old_ram #(.DW(DW),.AW(AW),.DEPTH(DEPTH),.INIT_VAL(IV),.INIT_FILE(FILE)) oldmem(clk,we,wa,wd,re,ra,a);
  new_ram #(.DW(DW),.AW(AW),.DEPTH(DEPTH),.INIT_VAL(IV),.INIT_FILE(FILE),.STAGE2B_BLOCK(1)) newmem(clk,we,wa,wd,re,ra,b);
  always@(posedge clk)begin
    if(re)expected<=gold[ra];else holds++;
    if(we)gold[wa]<=wd;
    if(we&&re&&wa==ra)collisions++;
  end
  // Ignore only the initialization X-to-0 clock event at time0.
  // The first real falling edge still checks q=0 before any enabled read.
  always@(negedge clk)if($time>0)begin
    checks++;
    if(a!==expected || b!==expected)$fatal(1,"SDP contract id=%0d a=%h b=%h expected=%h wa=%0d ra=%0d",ID,a,b,expected,wa,ra);
  end
  task step(input w,r,input[31:0]x,y,data);
    begin @(negedge clk);#1;we=w;re=r;wa=x%DEPTH;ra=y%DEPTH;wd=data;end
  endtask
  initial begin
    if(FILE!="")$readmemh(FILE,gold);else for(integer i=0;i<DEPTH;i++)gold[i]=IV;
    repeat(4)step(0,0,0,DEPTH-1,0);
    step(0,1,0,0,0);step(0,1,0,DEPTH-1,0);step(0,0,DEPTH-1,0,0);
    for(integer i=0;i<AW;i++)begin
      step(!READONLY,1,1<<i,1<<i,32'h80a55a01^i^ID);
      step(0,1,0,1<<i,0);step(0,0,0,DEPTH-1,0);
    end
    for(integer i=0;i<256;i++)begin
      rng=rng^(rng<<13);rng=rng^(rng>>17);rng=rng^(rng<<5);
      step(!READONLY&&rng[31],rng[30],rng,rng[29]?rng:(rng>>8),rng^ID);
    end
    repeat(6)step(0,0,0,DEPTH-1,0);
    if(!READONLY&&collisions==0)$fatal(1,"No collision sampled id=%0d",ID);
    $display("RAM_CONTRACT_DONE id=%0d checks=%0d collisions=%0d holds=%0d",ID,checks,collisions,holds);
    done=1;
  end
endmodule

module tb_stage2b_storage_contract;
 reg clk=0;always #10 clk=~clk;wire[12:0]done;
 memory_contract #(.DW(1),.AW(14),.IV(1'b1),.ID(0)) bitmap(clk,done[0]);
 memory_contract #(.DW(32),.AW(9),.IV(32'h12345678),.ID(1)) injection(clk,done[1]);
 memory_contract #(.DW(32),.AW(9),.IV(32'haabbccdd),.ID(2)) shadow(clk,done[2]);
 memory_contract #(.DW(16),.AW(10),.IV(16'ha55a),.ID(3)) slist(clk,done[3]);
 genvar i;
 generate for(i=0;i<4;i++)begin:con
   memory_contract #(.DW(8),.AW(10),.IV(8'hc3),.ID(4+i)) lane(clk,done[4+i]);
 end
 for(i=0;i<4;i++)begin:trig
   memory_contract #(.DW(16),.AW(10),.FILE("sin_lut.hex"),.READONLY(1),.ID(8+i)) table_port(clk,done[8+i]);
 end endgenerate
 reg en=0;reg[8:0]address=0;wire signed[31:0]oldq,newq;
 bias_old bo(clk,en,address,oldq);bias_new bn(clk,en,address,newq);
 reg[31:0]rng=32'hb1a526;reg seen=0;reg signed[31:0]held;
 reg bias_done=0;assign done[12]=bias_done;
 always@(negedge clk)if(seen)begin
   if(oldq!==newq)$fatal(1,"Bias enabled-read/sign/latency mismatch");
   if(!en&&oldq!==held)$fatal(1,"Bias hold mismatch");held=oldq;
 end
 initial begin
   // Original bias_q is uninitialized until the first enabled read.
   repeat(4)@(negedge clk);
   for(integer j=0;j<96;j++)begin
     @(negedge clk);#1;rng=rng^(rng<<13);rng=rng^(rng>>17);rng=rng^(rng<<5);
     en=(j==0)||rng[31];address=j==0?0:j==1?424:j==2?128:j==3?127:rng%425;
     if(en)seen=1;
   end
   @(negedge clk);#1;en=0;repeat(6)@(negedge clk);bias_done=1;
 end
 initial begin wait(&done);repeat(3)@(negedge clk);$display("STAGE2B_RTL_CONTRACTS_PASS cases=13 no_mapped_primitive_or_consumer_acceptance=1");$finish;end
 initial begin #100000;$fatal(1,"Finite storage contract watchdog");end
endmodule
