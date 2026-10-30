// Compact integration of the reviewed OV5640 capture and packed numeric HUD.
// Full640x480 field of view, one RGB565 sample per4x4 cell,160x120 stored.
module g2_camera_lcd #(
    parameter integer H_ACTIVE=800,H_FRONT=40,H_SYNC=128,H_BACK=88,
    parameter integer V_ACTIVE=480,V_FRONT=10,V_SYNC=2,V_BACK=33,
    parameter integer FRESH_CYCLES=12500000
)(
    input wire clk,reset_n,lcd_clk,
    input wire [95:0] imu_values,
    input wire imu_sample_valid,imu_ready,imu_error,
    input wire [15:0] distance_mm,
    input wire distance_valid,
    input wire [7:0] CAMERA_D,
    input wire CAMERA_PCLK,CAMERA_HREF,CAMERA_VSYNC,
    output wire CAMERA_SCLK,inout wire CAMERA_SDA,
    output wire CAMERA_RESET_N,
    output wire [4:0] LCD_R,LCD_B,output wire [5:0] LCD_G,
    output reg LCD_HSYNC,LCD_VSYNC,LCD_DE,
    output wire LCD_PCLK,LCD_BL,
    output wire camera_ready,camera_error,
    output wire display_ready
);
    wire pixel_valid,write_bank,frame_req,frame_ack;
    wire [16:0] pixel_addr;
    wire [15:0] pixel_data,ram_q;
    g2_ov5640_camera #(.DECIMATE(4)) capture(
        .clk(clk),.reset_n(reset_n),.CAMERA_D(CAMERA_D),
        .CAMERA_PCLK(CAMERA_PCLK),.CAMERA_HREF(CAMERA_HREF),.CAMERA_VSYNC(CAMERA_VSYNC),
        .CAMERA_SCLK(CAMERA_SCLK),.CAMERA_SDA(CAMERA_SDA),.CAMERA_RESET_N(CAMERA_RESET_N),
        .init_done(camera_ready),.init_error(camera_error),
        .pixel_valid(pixel_valid),.pixel_addr(pixel_addr),.pixel_data(pixel_data),
        .write_bank(write_bank),.frame_req(frame_req),.frame_ack(frame_ack));
    (* syn_preserve=1 *) reg [1:0] lcd_reset_pipe;
    always @(posedge lcd_clk or negedge reset_n)
        if(!reset_n)lcd_reset_pipe<=0;else lcd_reset_pipe<={lcd_reset_pipe[0],1'b1};
    wire lcd_reset_n=lcd_reset_pipe[1];
    // Missing camera publications blank the video; never reset the control core.
    (* syn_preserve=1 *) reg request_meta,request_sync;
    reg request_previous;
    reg [$clog2(FRESH_CYCLES+1)-1:0] age;
    wire stale=(age>=FRESH_CYCLES);
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n)begin request_meta<=0;request_sync<=0;request_previous<=0;age<=FRESH_CYCLES;end
        else begin
            request_meta<=frame_req;request_sync<=request_meta;request_previous<=request_sync;
            if(request_sync!=request_previous)age<=0;
            else if(age<FRESH_CYCLES)age<=age+1'b1;
        end
    end
    (* syn_preserve=1 *) reg stale_meta,stale_sync;
    always @(posedge lcd_clk or negedge lcd_reset_n)
        if(!lcd_reset_n)begin stale_meta<=1;stale_sync<=1;end
        else begin stale_meta<=stale;stale_sync<=stale_meta;end
    reg [10:0] h;
    reg [9:0] v;
    localparam H_TOTAL=H_ACTIVE+H_FRONT+H_SYNC+H_BACK;
    localparam V_TOTAL=V_ACTIVE+V_FRONT+V_SYNC+V_BACK;
    localparam X_START=(H_ACTIVE-640)/2;
    wire active=(h>=H_SYNC+H_BACK && h<H_SYNC+H_BACK+H_ACTIVE &&
                 v>=V_SYNC+V_BACK && v<V_SYNC+V_BACK+V_ACTIVE);
    wire [10:0] x=h-(H_SYNC+H_BACK);
    wire [9:0] y=v-(V_SYNC+V_BACK);
    wire in_image=active && x>=X_START && x<X_START+640 && y<480;
    wire [10:0] image_x=x-X_START;
    wire [14:0] row={8'd0,y[8:2]};
    wire [7:0] column=image_x[9:2];
    wire [14:0] read_addr=(row<<7)+(row<<5)+{7'd0,column};
    wire frame_start=(h==0 && v==0);
    g2_video_framebuffer store(
        .write_clk(CAMERA_PCLK),.write_reset_n(reset_n),
        .pixel_valid(pixel_valid),.write_bank(write_bank),.pixel_addr(pixel_addr),.pixel_data(pixel_data),
        .frame_req(frame_req),.read_clk(lcd_clk),.read_reset_n(lcd_reset_n),
        .frame_start(frame_start),.read_enable(in_image),.read_addr(read_addr),
        .invalidate(stale_sync),.frame_ack(frame_ack),.display_ready(display_ready),.read_data(ram_q));
    wire hud_box,hud_ink;
    g2_video_hud hud(.clk(clk),.reset_n(reset_n),.lcd_clk(lcd_clk),.lcd_reset_n(lcd_reset_n),
        .frame_start(frame_start),.x(x),.y(y),.active(active),
        .imu_values(imu_values),.sample_valid(imu_sample_valid),.sensor_ready(imu_ready),.sensor_error(imu_error),
        .distance_mm(distance_mm),.distance_valid(distance_valid),.box(hud_box),.ink(hud_ink));
    reg image_d,active_d,ready_d,image_dd,active_dd,ready_dd;
    reg [15:0] ram_qq,bars_d,bars_dd,rgb;
    reg hud_box_d,hud_ink_d,hs_d,vs_d,hs_dd,vs_dd;
    always @(posedge lcd_clk or negedge lcd_reset_n) begin
        if(!lcd_reset_n)begin
            h<=0;v<=0;image_d<=0;active_d<=0;ready_d<=0;
            image_dd<=0;active_dd<=0;ready_dd<=0;ram_qq<=0;bars_d<=0;bars_dd<=0;rgb<=0;
            hud_box_d<=0;hud_ink_d<=0;hs_d<=1;vs_d<=1;hs_dd<=1;vs_dd<=1;
            LCD_DE<=0;LCD_HSYNC<=1;LCD_VSYNC<=1;
        end else begin
            if(h==H_TOTAL-1)begin h<=0;v<=v==V_TOTAL-1 ? 0 : v+1'b1;end else h<=h+1'b1;
            image_d<=in_image;active_d<=active;ready_d<=display_ready;
            image_dd<=image_d;active_dd<=active_d;ready_dd<=ready_d;ram_qq<=ram_q;bars_dd<=bars_d;
            hud_box_d<=hud_box;hud_ink_d<=hud_ink;
            bars_d<=x<100 ? 16'hf800 : x<200 ? 16'h07e0 : x<300 ? 16'h001f :
                x<400 ? 16'hffe0 : x<500 ? 16'hf81f : x<600 ? 16'h07ff : x<700 ? 16'hffff : 0;
            LCD_DE<=active_dd;
            rgb<=!active_dd ? 0 : hud_box_d ? (hud_ink_d ? 16'hffff : 0) :
                !ready_dd ? bars_dd : image_dd ? ram_qq : 0;
            hs_d<=h>=H_SYNC;vs_d<=v>=V_SYNC;hs_dd<=hs_d;vs_dd<=vs_d;
            LCD_HSYNC<=hs_dd;LCD_VSYNC<=vs_dd;
        end
    end
    assign LCD_R=rgb[15:11];assign LCD_G=rgb[10:5];assign LCD_B=rgb[4:0];
    assign LCD_PCLK=~lcd_clk;
    assign LCD_BL=lcd_reset_pipe[1];
endmodule
