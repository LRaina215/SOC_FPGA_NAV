// Freshness/near retention over an 8-bit frame wrap; no LiDAR or navigation oracle.
`timescale 1ns/1ps
module tb_map_retention;
 reg clk=0,rst=1;always #5 clk=~clk;
 reg valid=0,upd=0,eof=0;wire ready,done,busy;
 reg [7:0] addr=0,tag=0;reg signed [15:0] z=0;
 reg signed [31:0] cx=0,cy=0;
 reg ereq=0;reg [7:0] ea=0;wire eack;wire [52:0] eq;
 reg are=0;reg [7:0] aa=0;wire [7:0] aq;wire [7:0] fc;
 map_subsys #(.LOG2N(4)) dut(
  .clk(clk),.rst(rst),.cfg_enable(1'b1),.cfg_auto_sweep(1'b1),.cmd_sweep(1'b0),.cmd_clear(1'b0),
  .cfg_cx(cx),.cfg_cy(cy),.cfg_age_max(8'd2),.cfg_cnt_min(4'd1),.cfg_keep_r(4'd3),
`ifdef MAP_AVG_TEST
  .cfg_avg(1'b1),
`else
  .cfg_avg(1'b0),
`endif
  .cfg_avg_shift(3'd3),
  .cfg_step_max(16'd20),.cfg_step_high_only(1'b1),.cfg_rough_max(16'd50),
  .cfg_slope_rise(16'd57),.cfg_jump(8'd25),.cfg_drop_k(5'd4),.cfg_drop_slope(8'd4),
  .cfg_k_slope(8'd160),.cfg_k_step(8'd48),.cfg_k_rough(8'd32),
  .cfg_fill_min(4'd5),.cfg_fill_tol(8'd30),.cfg_fill_cost(8'd40),.cfg_infl_r(4'd0),.cfg_infl_disk(1'b1),
  .s_valid(valid),.s_ready(ready),.s_upd(upd),.s_addr(addr),.s_tag(tag),.s_z(z),.s_eof(eof),.s_cx(cx),.s_cy(cy),
  .a_re(are),.a_addr(aa),.a_q(aq),.b_req(1'b0),.b_addr(8'd0),.b_ack(),.b_q(),
  .e_req(ereq),.e_addr(ea),.e_ack(eack),.e_q(eq),.busy(busy),.front(),.seq(),
  .lat_cx(),.lat_cy(),.frame_cnt(fc),.map_ready(done));
 task automatic point(input bit u,input bit e,input [7:0] a,input signed [15:0] h);
  @(negedge clk);valid=1;upd=u;eof=e;addr=a;z=h;
  do @(posedge clk);while(!ready);
  @(negedge clk);valid=0;upd=0;eof=0;
 endtask
 task automatic finish_frame;
  wait(done);@(negedge clk);
 endtask
 task automatic check_word(input bit stale,input bit occupied);
  @(negedge clk);ereq=1;ea=0;
  do @(posedge clk);while(!eack);
  #1;
  if(eq[52]!==stale || (eq[35:32]!=0)!==occupied)
   $fatal(1,"retention word fc=%0d word=%h stale=%b occupied=%b",fc,eq,stale,occupied);
  @(negedge clk);ereq=0;
  repeat(3) @(posedge clk);
 endtask
 task automatic check_cost(input [7:0] expected);
  @(negedge clk);are=1;aa=0;
  @(posedge clk);#1;
  if(aq!==expected) $fatal(1,"retained cost fc=%0d got=%0d expected=%0d",fc,aq,expected);
  @(negedge clk);are=0;
 endtask
 initial begin
  repeat(8) @(posedge clk);@(negedge clk);rst=0;wait(!busy);
  point(1,0,0,75);point(1,1,1,0);finish_frame();
  check_word(0,1);check_cost(254);
  for(int f=2;f<=257;f++) begin
   point(0,1,0,0);finish_frame();
   if(f==3 || f==255 || f==256 || f==257) begin check_word(1,1);check_cost(254);end
  end
  // At fc=2, the old modulo age appears fresh; the latched flag must prevent revival.
  cx=6;cy=6;point(0,1,0,0);finish_frame();check_word(1,0);check_cost(255);
  cx=0;cy=0;point(1,1,0,-12);finish_frame();check_word(0,1);
`ifdef MAP_AVG_TEST
  // 平均模式：重新初始化为 A = -12 mm（Q3）、D = 0
  if(eq[15:0]!==16'hffa0 || eq[31:16]!==16'h0000) $fatal(1,"new sample must replace stale average");
`else
  if(eq[15:0]!==16'hfff4 || eq[31:16]!==16'hfff4) $fatal(1,"new sample must replace stale interval");
`endif
  $display("TEST PASSED: retention preserved hazard, blocked wrap revival, fresh observation replaced interval");
  $finish;
 end
 initial begin repeat(3000000) @(posedge clk);$fatal(1,"retention timeout");end
endmodule
