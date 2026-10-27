`timescale 1ns/1ps
// Replays recorded ground feedback (one "vx_mmps wz_mradps" line per 10 ms telemetry frame)
// into command_lease_guard and measures how long a zero-command lease takes to arm.
// Scaled clock: CLK_HZ=50000, so 1 ms = 50 clocks and one feedback sample every 500 clocks.
// Each cycle sends zero packets every 100 ms until armed, then withholds packets for 300 ms
// so the 200 ms deadline disarms the lease, and starts the next cycle.
module tb_lease_replay;
 reg clk=0;always #5 clk=~clk;
 reg reset_n=0,valid=0,fbv=0;
 reg [15:0] seq=0;
 reg signed [15:0] mv=0,mw=0;
 wire signed [23:0] ov,ow;wire armed,expired,stopped,balance;wire [31:0] age,acc,rej;
 command_lease_guard #(.CLK_HZ(50000)) dut(.clk(clk),.reset_n(reset_n),.owner_select(2'd1),.source_id(2'd1),
  .session_select(32'd1),.packet_session(32'd1),.packet_seq(seq),.valid(valid),.source_fault(1'b0),.estop(1'b0),
  .forward_q12(24'sd0),.yaw_q12(24'sd0),.feedback_valid(fbv),.measured_vx_mmps(mv),.measured_wz_mradps(mw),
  .out_forward_q12(ov),.out_yaw_q12(ow),.armed(armed),.expired(expired),.stop_confirmed(stopped),.age_ms(age),
  .accepted_count(acc),.rejected_count(rej),.balance_enable(balance));
 string fname;
 integer fd,code,a,b,k,cycle_start,pause_left,cycles,lat_sum,lat_max,first_arm,still_samples;
 initial begin
  if(!$value$plusargs("FB=%s",fname))$fatal(1,"missing +FB=file");
  fd=$fopen(fname,"r");if(fd==0)$fatal(1,"cannot open %s",fname);
  repeat(5)@(negedge clk);reset_n=1;
  k=0;cycle_start=0;pause_left=0;cycles=0;lat_sum=0;lat_max=0;first_arm=-1;still_samples=0;
  while(!$feof(fd))begin
   code=$fscanf(fd,"%d %d\n",a,b);
   if(code!=2)break;
   mv=a;mw=b;
   @(negedge clk);fbv=1;@(negedge clk);fbv=0;
   if(stopped)still_samples=still_samples+1;
   if(pause_left>0)begin
    pause_left=pause_left-1;
    if(pause_left==0)cycle_start=k+1;
   end else if(armed)begin
    if(first_arm<0)first_arm=k;
    cycles=cycles+1;lat_sum=lat_sum+(k-cycle_start);
    if(k-cycle_start>lat_max)lat_max=k-cycle_start;
    pause_left=30;
   end else if(k%10==0)begin
    seq=seq+1;valid=1;@(negedge clk);valid=0;
   end
   repeat(495)@(negedge clk);
   k=k+1;
  end
  $display("REPLAY %s samples %0d first_arm_after %0d ms armed_cycles %0d mean_latency %0d ms max_latency %0d ms stop_confirmed %0d%%",
           fname,k,first_arm*10,cycles,cycles?lat_sum*10/cycles:-1,lat_max*10,100*still_samples/k);
  $finish;
 end
endmodule
