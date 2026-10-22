`timescale 1ns/1ps
module tb_support_coverage;
 reg clk=0,rst=1,clear=0;always #10 clk=~clk;
 reg [6:0] epoch=0,cap=0;reg fs=0,validcap=0,sv=0,su=0;
 reg signed [31:0] cx=0,cy=0;reg [13:0] sa=0,ca=0;reg req=0;
 wire ready,busy,ack;wire [31:0] q;
 support_coverage dut(.clk(clk),.rst(rst),.clear(clear),.age_epoch(epoch),.frame_start(fs),.frame_cx(cx),.frame_cy(cy),.capture_epoch(cap),.capture_valid(validcap),.s_valid(sv),.s_upd(su),.s_ready(ready),.s_addr(sa),.cpu_req(req),.cpu_addr(ca),.cpu_ack(ack),.cpu_q(q),.busy(busy));
 integer fd,n,a,b,c,d,queries=0,points=0,waits;reg [31:0] expected;string op;
 task drain;begin
  @(negedge clk);sv=0;su=0;waits=0;
  while(busy)begin @(negedge clk);waits++;if(waits>100000)$fatal(1,"drain timeout");end
  repeat(3)@(negedge clk);
 end endtask
 initial begin
  fd=$fopen("coverage.ops","r");if(!fd)$fatal(1,"missing operations");
  repeat(8)@(negedge clk);rst=0;drain();
  while(!$feof(fd))begin
   n=$fscanf(fd,"%s",op);
   if(op=="E")begin n=$fscanf(fd,"%h",a);drain();epoch=a;@(negedge clk);drain();end
   else if(op=="F")begin
    n=$fscanf(fd,"%h %h %h %h",a,b,c,d);drain();cx=a;cy=b;cap=c;validcap=d;fs=1;@(negedge clk);fs=0;drain();
   end else if(op=="P")begin
    n=$fscanf(fd,"%h",a);sa=a;sv=1;su=1;while(!ready)@(negedge clk);@(negedge clk);points++;
   end else if(op=="D")drain();
   else if(op=="C")begin drain();clear=1;@(negedge clk);clear=0;drain();end
   else if(op=="Q")begin
    n=$fscanf(fd,"%h %h",a,expected);drain();ca=a;req=1;waits=0;
    while(!ack)begin @(negedge clk);waits++;if(waits>80)$fatal(1,"query timeout");end
    if(q!==expected)$fatal(1,"query%0d addr=%h expected=%h actual=%h",queries,a,expected,q);
    queries++;repeat(8)begin @(negedge clk);if(ack)$fatal(1,"request re-acknowledged");end
    req=0;@(negedge clk);
   end else if(op=="END")break;
   else $fatal(1,"bad operation %s",op);
  end
  $display("SUPPORT queries=%0d points=%0d",queries,points);$display("TEST PASSED");$finish;
 end
endmodule
