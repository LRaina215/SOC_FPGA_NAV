`timescale 1ns/1ps
module tb_model_filter;
reg clk=0;always #5 clk=~clk;reg rst=0,en=0;reg signed[31:0]l=0,r=0;
wire signed[31:0]lo,ro;reg[63:0]vectors[0:599];integer i,expected_l,expected_r;
model_wheel_response #(.UPDATE_CYCLES(16)) dut(.clk(clk),.reset_n(rst),.enable(en),.left_target_mmps(l),.right_target_mmps(r),.left_filtered_mmps(lo),.right_filtered_mmps(ro));
task sample;begin wait(dut.counter==15);@(posedge clk);repeat(6)@(negedge clk);end endtask
initial begin
$readmemh("vectors.mem",vectors);repeat(4)@(negedge clk);rst=1;en=1;
for(i=0;i<600;i=i+1)begin
 l=$signed(vectors[i][63:48]);r=$signed(vectors[i][47:32]);expected_l=$signed(vectors[i][31:16]);expected_r=$signed(vectors[i][15:0]);sample;
 if(lo-expected_l>1||lo-expected_l< -1||ro-expected_r>1||ro-expected_r< -1)$fatal(1,"float mismatch at%0d got%0d,%0d expected%0d,%0d",i,lo,ro,expected_l,expected_r);
 if(lo>600||lo< -600||ro>600||ro< -600)$fatal(1,"bound");
 if(i<5 && (lo!=0||ro!=0))$fatal(1,"10ms delay not held");
 if(i>=430 && i<490 && (lo>20||lo< -20||ro>20||ro< -20))$fatal(1,"alternating action suppression");
end
// Immediate bypass cut, and no queued stale motion after re-enable.
en=0;#1;if(lo!=0||ro!=0)$fatal(1,"disable must immediately cut outputs");
repeat(2)@(negedge clk);l=0;r=0;en=1;
repeat(10)begin sample;if(lo!=0||ro!=0)$fatal(1,"queued command survived disable");end
// Disable while multiply/accumulate is in flight.
l=600;repeat(7)sample;
wait(dut.state==1);@(negedge clk);en=0;#1;if(lo!=0)$fatal(1,"inflight disable");
repeat(4)@(negedge clk);l=0;r=0;en=1;repeat(8)begin sample;if(lo!=0||ro!=0)$fatal(1,"pipeline resurrected command");end
$display("PASS model target filter:600 float comparisons<=1mm/s,5sample delay,45ms step response,sign/zero/limits,alternating suppression,immediate disable,pipeline/history flush");$finish;
end
initial begin #1000000;$fatal(1,"timeout");end
endmodule
