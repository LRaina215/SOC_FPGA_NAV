`timescale 1ns/1ps
module fifo_pair #(parameter DW=10, AW=11, ID=0)(output reg done=0);
 reg wc=0,rc=0,wr_run=1,rd_run=1,arst=0,xmode=0,xw=0,xr=0;
 always #10 if(wr_run)wc=~wc;
 initial begin #3;forever #14 if(rd_run)rc=~rc;end
 wire ws,rs;
 nav_rst_sync sw(.clk(wc),.arst(arst),.rst(ws));
 nav_rst_sync sr(.clk(rc),.arst(arst),.rst(rs));
 wire wrst=xmode?xw:ws,rrst=xmode?xr:rs;
 reg sending=0,reading=0,sv=0,mready=0,score=1;
 reg [DW-1:0] sd=0;reg[31:0] rng=32'h31286577+ID;
 wire ar,br,av,bv;wire[DW-1:0] ad,bd;wire[AW:0] af,bf;
 nav_async_fifo_before #(.DW(DW),.AW(AW)) before_u(.wclk(wc),.wrst(wrst),.s_valid(sv),.s_data(sd),.s_ready(ar),.wfree(af),.rclk(rc),.rrst(rrst),.m_valid(av),.m_data(ad),.m_ready(mready));
 nav_async_fifo #(.DW(DW),.AW(AW)) after_u(.wclk(wc),.wrst(wrst),.s_valid(sv),.s_data(sd),.s_ready(br),.wfree(bf),.rclk(rc),.rrst(rrst),.m_valid(bv),.m_data(bd),.m_ready(mready));
 integer expected[$],writes=0,reads=0,edges=0,resets=0,stops=0,xevents=0;
 reg[DW-1:0] next_word=ID*151;
 task compare;
 begin
  edges=edges+1;
  if({ar,av,ad,af}!=={br,bv,bd,bf})$fatal(1,"external four-state mismatch id%0d t%0t",ID,$time);
  if({before_u.wp_b,before_u.wp_g,before_u.rp_b,before_u.rp_g,before_u.rp_g_w1,before_u.rp_g_w2,before_u.rp_b_w,before_u.wp_g_r1,before_u.wp_g_r2,before_u.wp_b_r,before_u.u_ob.pend,before_u.u_ob.cnt,before_u.u_ob.b0,before_u.u_ob.b1}!==
     {after_u.wp_b,after_u.wp_g,after_u.rp_b,after_u.rp_g,after_u.rp_g_w1,after_u.rp_g_w2,after_u.rp_b_w,after_u.wp_g_r1,after_u.wp_g_r2,after_u.wp_b_r,after_u.u_ob.pend,after_u.u_ob.cnt,after_u.u_ob.b0,after_u.u_ob.b1})
   $fatal(1,"internal next-state mismatch id%0d t%0t",ID,$time);
 end endtask
 always @(negedge wc)begin
  rng={rng[30:0],rng[31]^rng[21]^rng[1]^rng[0]};
  sv=sending && !wrst;sd=next_word;
 end
 always @(negedge rc)mready=reading && (rng[2]||rng[5]);
 always @(posedge wc)begin
  if(score && !wrst && sv && ar)begin expected.push_back(sd);next_word=next_word+1;writes=writes+1;end
  #1;compare();
 end
 always @(posedge rc)begin
  if(score && !rrst && av && mready)begin
   if(expected.size()==0)$fatal(1,"unexpected/replayed read id%0d",ID);
   if(ad!==expected.pop_front())$fatal(1,"ordered data mismatch id%0d",ID);
   reads=reads+1;
  end
  #1;compare();
 end
 always @(wrst or rrst)begin #1;compare();end
 always @(posedge arst)begin expected.delete();next_word=next_word+37;resets=resets+1;end
 task reset_common;
 begin #3;arst=1;repeat(8)@(negedge wc);#3;arst=0;wait(!ws&&!rs);end endtask
 task fill(input integer n);
 integer limit;
 begin reading=0;sending=1;limit=0;
  while(expected.size()<n && limit<n*4+100)begin @(negedge wc);limit=limit+1;end
  sending=0;@(negedge wc);
  if(expected.size()<n)$fatal(1,"fill failed id%0d",ID);
 end endtask
 task drain;
 integer limit;
 begin sending=0;reading=1;limit=0;
  while((expected.size()!=0||av) && limit<(1<<AW)*8+100)begin @(negedge rc);limit=limit+1;end
  if(expected.size()!=0||av)$fatal(1,"drain failed id%0d count%0d",ID,expected.size());
  repeat(12)@(negedge wc);
 end endtask
 initial begin
  reset_common();sending=1;reading=1;repeat(6000)@(negedge wc);sending=0;drain();
  fill((1<<AW)+2);repeat(20)@(negedge wc);
  if(ar!==0 || expected.size()!=(1<<AW)+2)$fatal(1,"actual full capacity failed id%0d q%0d",ID,expected.size());
  drain();
  fill(33);@(negedge wc);wr_run=0;#3;arst=1;repeat(8)@(negedge rc);#3;arst=0;repeat(8)@(negedge rc);
  if(ws!==1||ar!==0||av!==0||before_u.wp_b!==0||after_u.wp_b!==0)$fatal(1,"stopped write reset id%0d",ID);
  wr_run=1;wait(!ws&&!rs);stops=stops+1;reading=1;sending=1;repeat(1000)@(negedge wc);sending=0;drain();
  fill(37);@(negedge rc);rd_run=0;#3;arst=1;repeat(8)@(negedge wc);#3;arst=0;repeat(8)@(negedge wc);
  if(rs!==1||av!==0||before_u.rp_b!==0||after_u.rp_b!==0)$fatal(1,"stopped read reset id%0d",ID);
  rd_run=1;wait(!ws&&!rs);stops=stops+1;sending=1;reading=1;
  for(integer k=0;k<12;k=k+1)begin repeat(31+k)@(negedge wc);reset_common();end
  sending=0;drain();score=0;reading=0;
  // External reset inputs only; never force a pointer/RAM/internal state.
  @(negedge wc);xmode=1;xw=1;xr=1;repeat(4)@(negedge wc);
  for(integer k=0;k<4;k=k+1)begin
   xw=0;xr=0;repeat(3)@(negedge wc);
   case(k)0:begin xw=1'bx;xr=1'bx;end 1:begin xw=1'bz;xr=1'bz;end 2:begin xw=1;xr=1;end 3:begin xw=0;xr=0;end endcase
   repeat(4)@(negedge wc);xevents=xevents+1;
  end
  xw=1;xr=1;repeat(4)@(negedge wc);arst=1;xmode=0;repeat(8)@(negedge wc);arst=0;wait(!ws&&!rs);score=1;
  sending=1;reading=1;repeat(1000)@(negedge wc);sending=0;drain();
  if(writes<5000||reads<4000||stops!=2||xevents!=4)$fatal(1,"coverage id%0d writes%0d reads%0d",ID,writes,reads);
  $display("FIFO_CANONICAL_PAIR_PASS id=%0d DW=%0d AW=%0d edges=%0d writes=%0d reads=%0d common_resets=%0d stopped_domains=%0d XZ_reset_events=%0d capacity=%0d",ID,DW,AW,edges,writes,reads,resets,stops,xevents,(1<<AW)+2);
  done=1;
 end
endmodule
module tb_fifo_reset_syntax;
 wire[2:0] done;
 fifo_pair #(.DW(10),.AW(11),.ID(0))a(.done(done[0]));
 fifo_pair #(.DW(10),.AW(12),.ID(1))b(.done(done[1]));
 fifo_pair #(.DW(9),.AW(11),.ID(2))c(.done(done[2]));
 reg [0:0] rst_test,d_test,old_expr,new_expr;
 function canonical(input r,input d);begin if(r)canonical=0;else canonical=r?0:d;end endfunction
 initial begin
  for(integer r=0;r<4;r=r+1)for(integer d=0;d<4;d=d+1)begin
   case(r)0:rst_test=0;1:rst_test=1;2:rst_test=1'bx;3:rst_test=1'bz;endcase
   case(d)0:d_test=0;1:d_test=1;2:d_test=1'bx;3:d_test=1'bz;endcase
   old_expr=rst_test?0:d_test;new_expr=canonical(rst_test,d_test);
   if(old_expr!==new_expr)$fatal(1,"four-state pointwise identity failed");
  end
  wait(&done);$display("TEST PASSED FIFO CANONICAL RESET three_actual_parameter_profiles=3 four_state_bit_cases=16 no_force=1");$finish;
 end
 initial begin #3000000;$fatal(1,"finite FIFO syntax stimulus watchdog");end
endmodule
