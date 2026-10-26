`timescale 1ns/1ps
module ownership_slf_pair(output reg done=0);
 reg clk=0;always #10 clk=~clk;
 reg rst=1,miter=1;
 reg fire=0,pv=0,sof=0,eof=0,begin_frame=0,hold_frame=1,enabled=1,en=1;
 reg signed[31:0]x=0,y=0,z=0;
 reg ev=0,up=0,clear=0,prior=0,cw=0,rr=0;
 reg[8:0]ca=0,ra=0;reg[31:0]cd=0;
 wire b0,b1,p0,p1,d0,d1,admit;wire[12:0]n0,n1;wire[31:0]q0,q1;
 slf_match_ref #(.SIN_INIT("sin_lut.hex")) reference(
  .clk(clk),.rst(rst),.s_fire(fire),.s_x(x),.s_y(y),.s_z(z),.s_pv(pv),.s_sof(sof),.s_eof(eof),
  .frame_en(en),.cfg_zlo(-32'sd1000),.cfg_zhi(32'sd1000),.cfg_tx(32'sd0),.cfg_ty(32'sd0),.cfg_inv_res(16'd1311),
  .cmd_eval(ev),.cmd_update(up),.cmd_clear(clear),.cfg_cx_mm(32'sd0),.cfg_cy_mm(32'sd0),.cfg_ncand(9'd2),
  .upd_dx(32'sd0),.upd_dy(32'sd0),.upd_dyaw(16'd0),.lf_cx(32'sd0),.lf_cy(32'sd0),
  .cfg_prior(prior),.cfg_nogate(1'b1),.cfg_lfo_x(-32'sd128),.cfg_lfo_y(-32'sd128),
  .pw_we(1'b0),.pw_addr(16'd0),.pw_data(18'd0),.cand_we(cw),.cand_waddr(ca),.cand_wdata(cd),
  .res_re(rr),.res_raddr(ra),.res_q(q0),.dbg_req(1'b0),.dbg_addr(16'd0),.dbg_ack(),.dbg_q(),
  .busy(b0),.pts_ready(p0),.n_pts(n0),.eval_done(d0));
 slf_match #(.SIN_INIT("sin_lut.hex")) candidate(
  .clk(clk),.rst(rst),.s_fire(fire),.s_x(x),.s_y(y),.s_z(z),.s_pv(pv),.s_sof(sof),.s_eof(eof),
  .frame_begin(begin_frame),.frame_hold(hold_frame),.match_enable(enabled),.frame_can_start(admit),
  .frame_en(en),.cfg_zlo(-32'sd1000),.cfg_zhi(32'sd1000),.cfg_tx(32'sd0),.cfg_ty(32'sd0),.cfg_inv_res(16'd1311),
  .cmd_eval(ev),.cmd_update(up),.cmd_clear(clear),.cfg_cx_mm(32'sd0),.cfg_cy_mm(32'sd0),.cfg_ncand(9'd2),
  .upd_dx(32'sd0),.upd_dy(32'sd0),.upd_dyaw(16'd0),.lf_cx(32'sd0),.lf_cy(32'sd0),
  .cfg_prior(prior),.cfg_nogate(1'b1),.cfg_lfo_x(-32'sd128),.cfg_lfo_y(-32'sd128),
  .pw_we(1'b0),.pw_addr(16'd0),.pw_data(18'd0),.cand_we(cw),.cand_waddr(ca),.cand_wdata(cd),
  .res_re(rr),.res_raddr(ra),.res_q(q1),.dbg_req(1'b0),.dbg_addr(16'd0),.dbg_ack(),.dbg_q(),
  .busy(b1),.pts_ready(p1),.n_pts(n1),.eval_done(d1));
 integer checks=0,miter_cycles=0,wait_cycles=0,reads=0,writes=0,cases=0;
 always@(posedge clk)if(!rst)begin
  if(candidate.pb_we && candidate.pb_re)$fatal(1,"PB read/write simultaneous");
  if(candidate.pb_we)writes++;
  if(candidate.pb_re)begin reads++;if(candidate.points_pending || candidate.ex_busy)$fatal(1,"consumer before capture drain");end
  if(candidate.st==27)wait_cycles++;
 end
 always@(negedge clk)if($time>0 && !rst)begin
  checks++;
  if(miter)begin
   miter_cycles++;
   if($isunknown({b0,b1,p0,p1,n0,n1,d0,d1,q0,q1}))$fatal(1,"unknown normal miter output");
   if({b0,p0,n0,d0,q0}!=={b1,p1,n1,d1,q1})$fatal(1,"normal cycle miter mismatch cycle%0d",checks);
   if(reference.pb_re!==candidate.pb_re || reference.pb_we!==candidate.pb_we)$fatal(1,"normal PB enable timing changed");
   if(candidate.pb_re && (reference.pb_ra!==candidate.pb_ra || reference.pb_q!==candidate.pb_q))$fatal(1,"normal PB read data/address changed");
  end
 end
 task tick;begin @(negedge clk);#1;end endtask
 task command(input integer which);
  begin tick();ev=(which==0);up=(which==1);clear=(which==2);tick();ev=0;up=0;clear=0;end
 endtask
 task start_pass;
  begin
   wait(admit);tick();begin_frame=1;tick();begin_frame=0;
   if(en && (!candidate.points_pending || admit))$fatal(1,"raw acceptance did not reserve before SOF");
   if(!en && candidate.points_pending)$fatal(1,"disabled acceptance created pending capture");
  end
 endtask
 task pixels(input integer pattern);
  begin
   for(integer i=0;i<9600;i++)begin
    tick();fire=1;sof=(i==0);eof=(i==9599);x=(i%160)*20;y=(i/160)*20;z=((i/160)&1)?100:0;
    pv=(pattern==1)||((pattern==2)&&(i/160>=58));
   end
   tick();fire=0;sof=0;eof=0;
  end
 endtask
 task capture(input integer pattern);
  begin start_pass();repeat(6)tick();pixels(pattern);wait(p1);tick();end
 endtask
 task evaluate;
  begin
   command(0);wait(d1);tick();
   for(integer i=0;i<4;i++)begin tick();rr=1;ra=i;tick();rr=0;tick();end
  end
 endtask
 task update_points;
  begin command(1);repeat(4)tick();wait(!b1);tick();if(!admit)$fatal(1,"UPDATE did not release owner");end
 endtask
 initial begin
  repeat(8)tick();rst=0;wait(!b1);tick();
  for(integer i=0;i<4;i++)begin tick();cw=1;ca=i;cd=0;end tick();cw=0;
  // Valid normal protocol: cycle-by-cycle original state/data/enable timing.
  capture(1);if(n1!==4096)$fatal(1,"4096 point capacity lost");evaluate();update_points();cases++;
  capture(0);if(n1!==0)$fatal(1,"zero point frame wrong");evaluate();prior=1;update_points();prior=0;cases++;
  hold_frame=0;capture(0);if(!admit)$fatal(1,"unused NORMAL frame deadlock");cases++;
  // Sticky old ready may differ on the next raw accepted generation by design.
  miter=0;hold_frame=1;
  // Early command in project/transform empty window must wait, not snapshot0.
  start_pass();command(0);if(candidate.st!=27)$fatal(1,"early pre-SOF command not retained");
  repeat(6)tick();pixels(2);wait(d1);tick();
  if(candidate.nsnap!==320 || n1!==320)$fatal(1,"early command did not snapshot completed320 points");
  update_points();cases++;
  // Command exactly while ex_done and final registered pb_we are still high.
  start_pass();repeat(6)tick();pixels(2);wait(candidate.ex_done);tick();
  if(!candidate.pb_we)$fatal(1,"fixture did not reach final-write boundary");
  ev=1;tick();ev=0;if(candidate.st!=27)$fatal(1,"final-write command not deferred");
  wait(d1);tick();update_points();cases++;
  // HOLD cancellation releases; it must not be a permanent wait-for-UPDATE.
  capture(0);hold_frame=0;repeat(3)tick();if(!admit)$fatal(1,"HOLD cancel deadlock");cases++;
  // Disable releases but lets an already accepted producer finish all its points.
  hold_frame=1;start_pass();enabled=0;repeat(6)tick();pixels(2);wait(p1);repeat(3)tick();
  if(n1!==320 || !admit)$fatal(1,"disable truncated accepted pass or kept owner");enabled=1;cases++;
  // Soft-clear of a held completed generation releases ownership.
  capture(0);command(2);repeat(3)tick();wait(!b1);tick();if(!admit)$fatal(1,"soft-clear owner deadlock");cases++;
  // No accepted capture and ready0: original empty command must not wait forever.
  rst=1;repeat(4)tick();rst=0;wait(!b1);tick();command(0);
  if(candidate.st==27)$fatal(1,"unready/no-frame command incorrectly waits");wait(d1);tick();
  prior=1;update_points();prior=0;cases++;
  // Invalid/disabled extraction pass must not create a pending generation.
  en=0;enabled=0;start_pass();repeat(6)tick();pixels(0);repeat(20)tick();
  if(candidate.points_pending || b1 || !admit)$fatal(1,"disabled pass deadlock");cases++;
  if(wait_cycles<1 || reads<1024 || writes<4096 || miter_cycles<65536)$fatal(1,"incomplete ownership coverage");
  $display("SLF OWNERSHIP PASS cases=%0d checks=%0d miter_cycles=%0d wait_cycles=%0d reads=%0d writes=%0d",cases,checks,miter_cycles,wait_cycles,reads,writes);done=1;
 end
endmodule

module ownership_xt_queue(output reg done=0);
 reg clk=0;always #10 clk=~clk;
 reg rst=1,sv=0,sl=0,commit=0,admit=1,retain=1;reg[7:0]sd=0;
 reg[8:0]stim[0:5000];integer length;
 wire v0,v1,s0,s1,e0,e1,w0,w1,accept;wire[7:0]u0,u1,y0,y1;wire[15:0]q0,q1;
 wire[31:0]f0,f1,b0,b1;reg compare=1;
 xt_frame_rx_ref #(.CLK_HZ(50000000)) reference(
  .clk(clk),.rst(rst),.time_us(32'd0),.cfg_enable(1'b1),.cfg_hdr_le(1'b1),.cfg_pix_le(1'b1),.cfg_subhdr(8'd20),
  .cfg_amp_min(16'd0),.cfg_raw_inv(16'd64000),.cfg_wait_pose(1'b1),.pose_commit(commit),.cfg_retain(retain),
  .s_valid(sv),.s_data(sd),.s_last(sl),.s_err(1'b0),.m_valid(v0),.m_ready(1'b1),.m_u(u0),.m_v(y0),.m_depth(q0),.m_sof(s0),.m_eof(e0),
  .pose_waiting(w0),.stat_frames(f0),.stat_bad(b0));
 xt_frame_rx #(.CLK_HZ(50000000)) candidate(
  .clk(clk),.rst(rst),.time_us(32'd0),.cfg_enable(1'b1),.cfg_hdr_le(1'b1),.cfg_pix_le(1'b1),.cfg_subhdr(8'd20),
  .cfg_amp_min(16'd0),.cfg_raw_inv(16'd64000),.cfg_wait_pose(1'b1),.pose_commit(commit),.cfg_retain(retain),
  .read_admit(admit),.frame_accept(accept),
  .s_valid(sv),.s_data(sd),.s_last(sl),.s_err(1'b0),.m_valid(v1),.m_ready(1'b1),.m_u(u1),.m_v(y1),.m_depth(q1),.m_sof(s1),.m_eof(e1),
  .pose_waiting(w1),.stat_frames(f1),.stat_bad(b1));
 integer pixels=0,passes=0,accepted=0,checks=0;
 always@(posedge clk)if(!rst)begin
  if(accept)begin if(!admit)$fatal(1,"XT start bypassed admission");accepted++;end
  if(v1)begin
   if({s1,e1,u1,y1,q1}!=={(pixels%1024==0),(pixels%1024==1023),8'(pixels%32),8'((pixels%1024)/32),16'(1000+pixels%1024)})$fatal(1,"XT retained payload/order mismatch index%0d",pixels);
   pixels++;if(e1)passes++;
  end
 end
 always@(negedge clk)if($time>0&&!rst&&compare)begin
  checks++;
  if(v0!==v1 || w0!==w1 || f0!==f1 || b0!==b1)$fatal(1,"XT immediate normal timing changed");
  if(v1 && {s0,e0,u0,y0,q0}!=={s1,e1,u1,y1,q1})$fatal(1,"XT normal pixel miter changed");
 end
 task tick;begin @(negedge clk);#1;end endtask
 task request;begin tick();commit=1;tick();commit=0;end endtask
 task inject;
  begin
   for(integer i=0;i<length;i++)begin tick();sv=1;{sl,sd}=stim[i];tick();sv=0;sl=0;repeat(6)tick();end
   tick();sv=0;sl=0;
  end
 endtask
 initial begin
  integer fd;fd=$fopen("xt_count.txt","r");if(!fd)$fatal(1,"missing XT input");
  if($fscanf(fd,"%d",length)!=1)$fatal(1,"bad XT count");$fclose(fd);$readmemh("xt_stim.hex",stim,0,length-1);
  repeat(8)tick();rst=0;inject();wait(w1);request();
  // First pass normal miter. Queue second-pass request before its read_wait exists.
  wait(pixels>=512);compare=0;admit=0;retain=0;request();wait(passes==1);repeat(20)tick();
  if(pixels!=1024 || !w1 || !candidate.pose_pending || accepted!=1)$fatal(1,"busy-boundary request lost or bypassed");
  admit=1;wait(passes==2);repeat(8)tick();
  if(pixels!=2048 || accepted!=2)$fatal(1,"retained replay incomplete");
  // A request before any new completed frame is waiting remains pending.
  admit=0;request();inject();wait(w1);repeat(10)tick();
  if(pixels!=2048 || !candidate.pose_pending)$fatal(1,"pre-wait request not retained");
  admit=1;wait(passes==3);repeat(8)tick();
  if(pixels!=3072 || accepted!=3 || b1!=0 || f1!=2 || checks<1000)$fatal(1,"XT queue coverage failed");
  $display("XT OWNERSHIP PASS pixels=%0d passes=%0d accepted=%0d normal_miter_cycles=%0d",pixels,passes,accepted,checks);done=1;
 end
endmodule

module tb_stage2b_point_ownership;
 wire a,b;ownership_slf_pair slf(a);ownership_xt_queue xt(b);
 initial begin wait(a&&b);$display("TEST PASSED: POINT OWNERSHIP normal-cycle miter and finite boundary/cancel/zero cases; RTL only");$finish;end
 initial begin #30000000;$fatal(1,"finite ownership fixture watchdog");end
endmodule
