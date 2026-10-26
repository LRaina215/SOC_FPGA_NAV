`timescale 1ns/1ps
module point_pair #(parameter [35:0] IV=0)(output reg done=0);
 reg clk=0;always #10 clk=~clk;
 reg we=0,re=0;reg[11:0]wa=0,ra=0;reg[35:0]wd=0;
 wire[35:0]oldq,newq;reg[35:0]gold[0:4095],expected=0;
 ram_reference #(.DW(36),.AW(12),.DEPTH(4096),.INIT_VAL(IV)) a(clk,we,wa,wd,re,ra,oldq);
 ram_candidate #(.DW(36),.AW(12),.DEPTH(4096),.INIT_VAL(IV)) b(clk,we,wa,wd,re,ra,newq);
 integer checks=0,reads=0,writes=0,collisions=0,holds=0;
 reg[31:0]rng=32'h45004096;
 always@(posedge clk)begin
  if(re)begin expected<=gold[ra];reads++;end else holds++;
  if(we)begin gold[wa]<=wd;writes++;end
  if(we&&re&&wa==ra)collisions++;
 end
 always@(negedge clk)begin
  checks++;
  if($isunknown({oldq,newq,expected}))$fatal(1,"unknown initialized point data");
  if(oldq!==expected || newq!==expected)
   $fatal(1,"point mismatch IV=%h cycle=%0d old=%h new=%h expected=%h",IV,checks,oldq,newq,expected);
 end
 task step(input w,r,input[11:0]x,y,input[35:0]d);
  begin @(negedge clk);#1;we=w;re=r;wa=x;ra=y;wd=d;end
 endtask
 initial begin
  for(integer k=0;k<4096;k++)gold[k]=IV;
  repeat(5)step(0,0,4095,0,0);
  for(integer k=0;k<4096;k++)step(0,1,0,k,0);
  // Every exact read/write collision returns old data, then new, then holds.
  for(integer k=0;k<4096;k++)begin
   step(1,1,k,k,{4'hf,32'h89abcdef}^k);
   step(0,1,4095-k,k,0);step(0,0,k,4095-k,0);
  end
  for(integer k=0;k<8192;k++)begin
   rng=rng^(rng<<13);rng=rng^(rng>>17);rng=rng^(rng<<5);
   step(rng[31],rng[30],rng[11:0],rng[23:12],{rng[3:0],rng});
  end
  for(integer k=4095;k>=0;k--)step(0,1,k,k,0);
  repeat(5)step(0,0,0,0,0);
  if(reads<4096 || collisions<4096 || holds<4096)$fatal(1,"incomplete point coverage");
  $display("POINT PASS IV=%h checks=%0d reads=%0d writes=%0d collisions=%0d holds=%0d",IV,checks,reads,writes,collisions,holds);done=1;
 end
endmodule
module tb_stage2b_pointplanes;
 wire z,n;point_pair #(.IV(0))zero(z);point_pair #(.IV(36'h123456789))nonzero(n);
 initial begin wait(z&&n);$display("TEST PASSED: POINT PLANES all4096addresses, productionzero and nonzeroINIT, readfirst/hold");$finish;end
 initial begin #2000000;$fatal(1,"finite point stimulus watchdog");end
endmodule
