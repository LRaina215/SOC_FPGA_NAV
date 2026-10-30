`timescale 1ns/1ps
// HUD at the 480x272 placement: reference lcd_sensor_hud (full 21 columns) against g2_video_hud
// with TEXT_W=168 (14 columns). Pixels in columns 0..13 must match; the reference must have no
// glyph ink in columns 14..20, so narrowing drops no character.
module tb_hud480;
 reg clk=0,lc=0,rst=0,fs=0,active=1,sv=0,sready=0,serr=0,dv=0;
 reg [10:0] x=0;reg [9:0] y=0;reg [95:0] values=0;reg [15:0] distance=0;
 wire b0,b1,i0,i1;integer checks=0,blank=0,pages=0;
 always #10 clk=~clk;
 always #15 lc=~lc;
 lcd_sensor_hud #(.UPDATE_CYCLES(1000),.PANEL_X(308),.PANEL_Y(64)) reference(.clk(clk),.reset_n(rst),.lcd_clk(lc),
  .lcd_reset_n(rst),.frame_start(fs),.x(x),.y(y),.active(active),.imu_values(values),
  .sample_valid(sv),.sensor_ready(sready),.sensor_error(serr),.distance_mm(distance),
  .distance_valid(dv),.box(b0),.ink(i0));
 g2_video_hud #(.UPDATE_CYCLES(1000),.PANEL_X(308),.PANEL_Y(64),.TEXT_W(168)) dut(.clk(clk),.reset_n(rst),.lcd_clk(lc),
  .lcd_reset_n(rst),.frame_start(fs),.x(x),.y(y),.active(active),.imu_values(values),
  .sample_valid(sv),.sensor_ready(sready),.sensor_error(serr),.distance_mm(distance),
  .distance_valid(dv),.box(b1),.ink(i1));
 task commit_page;
  begin
   wait(dut.req!==dut.ack);repeat(4)@(negedge lc);
   if(reference.req!==dut.req)$fatal(1,"publication timing differs");
   fs=1;@(negedge lc);fs=0;pages=pages+1;
  end
 endtask
 task scan_page;
  begin
   for(integer row=0;row<8;row=row+1)
    for(integer col=0;col<21;col=col+1)
     for(integer gy=0;gy<7;gy=gy+1)
      for(integer gx=0;gx<5;gx=gx+1)begin
       @(negedge lc);x=308+col*12+gx*2;y=64+row*18+gy*2;
       @(posedge lc);#1;
       if(col<14)begin
        if({b0,i0}!=={b1,i1})$fatal(1,"HUD pixel page=%0d row=%0d col=%0d glyph=%0d/%0d",pages,row,col,gx,gy);
        checks=checks+1;
       end else begin
        if(i0)$fatal(1,"reference draws a glyph in column %0d row %0d: narrowing would drop it",col,row);
        if(x>=480 && (b1||i1))$fatal(1,"dut draws past the 480-pixel line");
        blank=blank+1;
       end
      end
  end
 endtask
 initial begin
  #73;rst=1;sready=1;
  for(integer vector=0;vector<8;vector=vector+1)begin
   if(vector!=0)commit_page();
   @(negedge clk);sv=1;dv=vector!=6;distance=vector==7 ? 65535 : 1234+vector;
   sready=vector!=5;serr=vector==4;
   case(vector)
    0:values={16'sd0,16'sd0,16'sd0,16'sd0,16'sd0,16'sd0};
    1:values={16'sd16384,-16'sd16384,16'sd8192,16'sd10920,-16'sd10920,16'sd5460};
    2:values={16'h8000,16'h7fff,16'hffff,16'h8000,16'h7fff,16'hffff};
    3:values={16'sd1,-16'sd1,16'sd16,-16'sd16,16'sd54,-16'sd54};
    default:values={16'sd3276,-16'sd3276,16'sd7654,16'sd9876,-16'sd8765,16'sd4321};
   endcase
   @(negedge clk);sv=0;
   commit_page();scan_page();
  end
  $display("HUD480_PASS checks=%0d blank_checks=%0d pages=%0d vectors=8",checks,blank,pages);$finish;
 end
 initial begin #40000000;$fatal(1,"watchdog");end
endmodule
