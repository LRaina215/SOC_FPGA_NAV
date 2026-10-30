`timescale 1ns/1ps
// g2_camera_lcd LCD timing and layout on the 480x272 TFT, camera idle (no frames, so the
// picture area shows the not-ready colour bars). Checks per line/frame DE counts, HSYNC/VSYNC
// widths, line/frame totals, colour bars at sample points and the HUD panel position.
module tb_lcd480;
 reg clk=0,lcd=0,pclk=0,reset_n=0;
 always #10 clk=~clk;
 always #55.75 lcd=~lcd;
 always #20 pclk=~pclk;
 wire sclk,sda,cam_rst,cam_ready,cam_error,disp_ready;
 pullup(sda);
 wire [4:0] r,b;wire [5:0] g;wire hs,vs,de,pck,bl;
 g2_camera_lcd dut(.clk(clk),.reset_n(reset_n),.lcd_clk(lcd),.imu_values(96'd0),.imu_sample_valid(1'b0),
  .imu_ready(1'b1),.imu_error(1'b0),.distance_mm(16'd532),.distance_valid(1'b1),
  .CAMERA_D(8'd0),.CAMERA_PCLK(pclk),.CAMERA_HREF(1'b0),.CAMERA_VSYNC(1'b0),
  .CAMERA_SCLK(sclk),.CAMERA_SDA(sda),.CAMERA_RESET_N(cam_rst),
  .LCD_R(r),.LCD_B(b),.LCD_G(g),.LCD_HSYNC(hs),.LCD_VSYNC(vs),.LCD_DE(de),.LCD_PCLK(pck),.LCD_BL(bl),
  .camera_ready(cam_ready),.camera_error(cam_error),.display_ready(disp_ready));
 integer h_in_line=0,de_in_line=0,hs_low=0,lines=0,de_lines=0,vs_low_lines=0,errors=0,frames=0;
 integer px=0,py=0,line_total=-1,last_vs=1,started=0;
 reg prev_de=0,prev_hs=1,prev_vs=1;
 wire [15:0] rgb={r,g,b};
 task expect_px(input integer ex,input integer ey,input [15:0] want,input [127:0] what);
  if(px==ex && py==ey && rgb!==want)begin
   $display("ERROR %0s at (%0d,%0d): rgb=%h want %h",what,ex,ey,rgb,want);errors=errors+1;
  end
 endtask
 always @(posedge lcd) if(reset_n) begin
  if(de) begin
   if(started && frames>=2)begin
    expect_px(10,10,16'hf800,"red bar");
    expect_px(330,30,16'h07ff,"cyan bar");
    expect_px(150,265,16'h001f,"blue bar below picture");
    expect_px(305,100,16'hffff,"HUD border");
    expect_px(470,100,16'h0000,"HUD background");
    expect_px(470,230,16'h0000,"right of colour bars");
   end
   px=px+1;
  end
  if(prev_de && !de) begin py=py+1;px=0; end
  h_in_line=h_in_line+1;
  if(!hs) hs_low=hs_low+1;
  if(de) de_in_line=de_in_line+1;
  if(prev_hs && !hs) begin
   if(started && frames>=1) begin
    if(line_total>=0 && h_in_line!=525) begin $display("ERROR line length %0d",h_in_line);errors=errors+1;end
    if(de_in_line!=0 && de_in_line!=480) begin $display("ERROR DE per line %0d",de_in_line);errors=errors+1;end
   end
   if(de_in_line!=0) de_lines=de_lines+1;
   line_total=h_in_line;h_in_line=0;de_in_line=0;lines=lines+1;
   if(!vs) vs_low_lines=vs_low_lines+1;
  end
  if(prev_hs==0 && hs && started && frames>=1 && hs_low!=41) begin $display("ERROR HSYNC low %0d",hs_low);errors=errors+1;end
  if(!prev_hs && hs) hs_low=0;
  if(prev_vs && !vs) begin
   if(started && frames>=1) begin
    if(lines!=286) begin $display("ERROR lines per frame %0d",lines);errors=errors+1;end
    if(de_lines!=272) begin $display("ERROR DE lines %0d",de_lines);errors=errors+1;end
    if(vs_low_lines!=10) begin $display("ERROR VSYNC low lines %0d",vs_low_lines);errors=errors+1;end
   end
   started=1;frames=frames+1;lines=0;de_lines=0;vs_low_lines=0;py=0;px=0;
  end
  prev_de=de;prev_hs=hs;prev_vs=vs;
 end
 initial begin
  #200;reset_n=1;
  wait(frames==4);
  $display("LCD480 frames=%0d errors=%0d display_ready=%0d backlight=%0d",frames,errors,disp_ready,bl);
  if(errors==0)$display("LCD480_PASS");else $fatal(1,"LCD480 failed");
  $finish;
 end
 initial begin #120000000;$fatal(1,"watchdog");end
endmodule
