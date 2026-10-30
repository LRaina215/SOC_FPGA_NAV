// Text is formatted at 10Hz in the 50MHz domain using one serial divider.
// A held-bus toggle handshake transfers a coherent page to LCD at frame boundary.
module lcd_sensor_hud # (parameter UPDATE_CYCLES=5000000,parameter PANEL_X=274,parameter PANEL_Y=168)(
 input wire clk,reset_n,lcd_clk,lcd_reset_n,frame_start,
 input wire [10:0] x,input wire [9:0] y,input wire active,
 input wire [95:0] imu_values,input wire sample_valid,sensor_ready,sensor_error,
 input wire [15:0] distance_mm,input wire distance_valid,
 output wire box,output wire ink
);
 reg [95:0] latest,snapshot;
 reg seen,usable,range_ok;
 reg [15:0] range_value;
 reg [22:0] timer;
 reg [2:0] state,lane;
 reg [4:0] step;
 reg negative;
 reg [28:0] numerator,quotient;
 reg [14:0] remainder,denominator;
 reg [15:0] binary,magnitude_reg;
 reg [19:0] bcd,adjusted;
 reg [167:0] rows[0:7];
 reg [1343:0] held;
 reg req,ack_meta,ack_sync;
 reg [1343:0] page;
 reg req_meta,req_sync,ack;
 wire [15:0] value=snapshot[lane*16 +:16];
 wire [15:0] mag=value[15] ? (~value+16'd1) : value;
 wire [15:0] trial={remainder,numerator[28]};
 wire take=trial>={1'b0,denominator};
 wire [15:0] reduced=take ? trial-{1'b0,denominator} : trial;
 wire [28:0] qnext={quotient[27:0],take};
 wire [19:0] bnext={adjusted[18:0],binary[15]};
 wire [7:0] sign_char=negative ? "-" : "+";
 wire [23:0] label=lane==0 ? "AX:" : lane==1 ? "AY:" : lane==2 ? "AZ:" :
                        lane==3 ? "GX:" : lane==4 ? "GY:" : "GZ:";
 function [7:0] digit;
 input [3:0] d;begin digit=8'h30+{4'd0,d};end
 endfunction
 integer i,j;
 always @* begin
  adjusted=bcd;
  for(j=0;j<5;j=j+1) if(bcd[j*4 +:4]>=5) adjusted[j*4 +:4]=bcd[j*4 +:4]+4'd3;
 end
 always @(posedge clk or negedge reset_n) begin
  if(!reset_n) begin
   latest<=0;snapshot<=0;seen<=0;usable<=0;range_ok<=0;range_value<=0;
   timer<=0;state<=0;lane<=0;step<=0;negative<=0;numerator<=0;quotient<=0;
   remainder<=0;denominator<=0;binary<=0;magnitude_reg<=0;bcd<=0;held<={168{8'h20}};
   req<=0;ack_meta<=0;ack_sync<=0;
   for(i=0;i<8;i=i+1) rows[i]<={21{8'h20}};
  end else begin
   ack_meta<=ack;ack_sync<=ack_meta;
   if(!sensor_ready || sensor_error) seen<=0;
   else if(sample_valid) begin latest<=imu_values;seen<=1;end
   if(timer<UPDATE_CYCLES-1)timer<=timer+1'b1;
   case(state)
    0: if(timer==UPDATE_CYCLES-1 && req==ack_sync) begin
     timer<=0;snapshot<=latest;usable<=seen && sensor_ready && !sensor_error;
     range_value<=distance_mm;range_ok<=distance_valid;lane<=0;state<=1;
     rows[0]<="IMU  g / dps         ";
    end
    1: begin
     negative<=value[15];magnitude_reg<=mag;quotient<=0;remainder<=0;step<=0;bcd<=0;
     if(lane==6) begin binary<=range_value;negative<=0;state<=3;end
     else state<=6;
    end
    6:begin
     numerator<=lane<3 ? ({13'd0,magnitude_reg}*29'd100+29'd5460) : ({13'd0,magnitude_reg}*29'd10000+29'd8192);
     denominator<=lane<3 ? 15'd10920 : 15'd16384;state<=2;
    end
    2: begin
     numerator<={numerator[27:0],1'b0};remainder<=reduced[14:0];quotient<=qnext;
     if(step==28)begin binary<=qnext[15:0];negative<=negative && qnext!=0;step<=0;state<=3;end
     else step<=step+1'b1;
    end
    3: begin
     bcd<=bnext;binary<={binary[14:0],1'b0};
     if(step==15)state<=4;else step<=step+1'b1;
    end
    4: begin
     if(lane==6) begin
      rows[7]<=range_ok ? {"DIST: ",digit(bcd[15:12]),digit(bcd[11:8]),digit(bcd[7:4]),digit(bcd[3:0])," mm        "} : "DIST: ---- mm        ";
      state<=5;
     end else begin
      if(!usable) rows[lane+1]<={label," ----             "};
      else if(lane<3) rows[lane+1]<={label,sign_char,digit(bcd[11:8]),".",digit(bcd[7:4]),digit(bcd[3:0])," g           "};
      else rows[lane+1]<={label,sign_char,digit(bcd[19:16]),digit(bcd[15:12]),digit(bcd[11:8]),digit(bcd[7:4]),".",digit(bcd[3:0])," dps       "};
      lane<=lane+1'b1;state<=1;
     end
    end
    5:begin
     held<={rows[7],rows[6],rows[5],rows[4],rows[3],rows[2],rows[1],rows[0]};req<=~req;state<=0;
    end
    default:state<=0;
   endcase
  end
 end
 always @(posedge lcd_clk or negedge lcd_reset_n) begin
  if(!lcd_reset_n) begin req_meta<=0;req_sync<=0;ack<=0;page<={168{8'h20}};end
  else begin
   req_meta<=req;req_sync<=req_meta;
   if(frame_start && req_sync!=ack)begin page<=held;ack<=req_sync;end
  end
 end
 // 21 columns of 12x18 cells, crisp 2x5x7 glyphs on a black panel.
 // Within the panel these 8-bit differences cover exactly x=0..251,y=0..143.
 wire [7:0] dx=x[7:0]-PANEL_X[7:0];
 wire [7:0] dy=y[7:0]-PANEL_Y[7:0];
 reg [4:0] col;
 reg [2:0] line;
 reg [167:0] text_row;
 integer k;
 always @* begin
  col=0;line=0;
  for(k=1;k<21;k=k+1) if(dx>=k*12) col=k;
  for(k=1;k<8;k=k+1) if(dy>=k*18) line=k;
  case(line)
   0:text_row=page[0+:168];1:text_row=page[168+:168];
   2:text_row=page[336+:168];3:text_row=page[504+:168];
   4:text_row=page[672+:168];5:text_row=page[840+:168];
   6:text_row=page[1008+:168];default:text_row=page[1176+:168];
  endcase
  // Static labels remain visible even before the first data page arrives.
  case(line)
   0:text_row="IMU  g / dps         ";
   1:text_row[167:144]="AX:";2:text_row[167:144]="AY:";3:text_row[167:144]="AZ:";
   4:text_row[167:144]="GX:";5:text_row[167:144]="GY:";6:text_row[167:144]="GZ:";
   7:text_row[167:120]="DIST: ";
  endcase
 end
 wire [7:0] cell_x=dx-({3'd0,col}<<3)-({3'd0,col}<<2);
 wire [7:0] cell_y=dy-({5'd0,line}<<4)-({5'd0,line}<<1);
 reg [7:0] character;
 reg [3:0] gx,gy;
 reg panel,border,text_area;
 always @(posedge lcd_clk or negedge lcd_reset_n) begin
  if(!lcd_reset_n)begin character<=8'h20;gx<=0;gy<=0;panel<=0;border<=0;text_area<=0;end
  else begin
   character<=text_row[(20-col)*8 +:8];gx<=cell_x[4:1];gy<=cell_y[4:1];
   panel<=active && x>=PANEL_X-4 && x<PANEL_X+256 && y>=PANEL_Y-4 && y<PANEL_Y+148;
   text_area<=active && x>=PANEL_X && x<PANEL_X+252 && y>=PANEL_Y && y<PANEL_Y+144;
   border<=active && x>=PANEL_X-4 && x<PANEL_X+256 && y>=PANEL_Y-4 && y<PANEL_Y+148 &&
       (x<PANEL_X-2 || x>=PANEL_X+254 || y<PANEL_Y-2 || y>=PANEL_Y+146);
  end
 end
 wire [34:0] glyph=font(character);
 assign box=panel;
 assign ink=border || (text_area && gx<5 && gy<7 && glyph[34-(gy*5+gx)]);
 function [34:0] font;
 input [7:0] c;
 begin case(c)
 8'h30:font=35'b01110100011001110101110011000101110;
 8'h31:font=35'b00100011000010000100001000010001110;
 8'h32:font=35'b01110100010000100010001000100011111;
 8'h33:font=35'b11110000010000101110000010000111110;
 8'h34:font=35'b00010001100101010010111110001000010;
 8'h35:font=35'b11111100001000011110000010000111110;
 8'h36:font=35'b01110100001000011110100011000101110;
 8'h37:font=35'b11111000010001000100010000100001000;
 8'h38:font=35'b01110100011000101110100011000101110;
 8'h39:font=35'b01110100011000101111000010000101110;
 8'h41:font=35'b01110100011000111111100011000110001;
 8'h58:font=35'b10001100010101000100010101000110001;
 8'h59:font=35'b10001100010101000100001000010000100;
 8'h5a:font=35'b11111000010001000100010001000011111;
 8'h47:font=35'b01110100011000010111100011000101110;
 8'h49:font=35'b01110001000010000100001000010001110;
 8'h4d:font=35'b10001110111010110101100011000110001;
 8'h55:font=35'b10001100011000110001100011000101110;
 8'h44:font=35'b11110100011000110001100011000111110;
 8'h53:font=35'b01111100001000001110000010000111110;
 8'h54:font=35'b11111001000010000100001000010000100;
 8'h67:font=35'b00000011111000110001011110000101110;
 8'h64:font=35'b00001000010110110011100011000101111;
 8'h70:font=35'b00000111101000110001111101000010000;
 8'h73:font=35'b00000011111000001110000010000111110;
 8'h6d:font=35'b00000000001101010101101011010110101;
 8'h2b:font=35'b00000001000010011111001000010000000;
 8'h2d:font=35'b00000000000000011111000000000000000;
 8'h2e:font=35'b00000000000000000000000000011000110;
 8'h3a:font=35'b00000001100011000000001100011000000;
 8'h2f:font=35'b00001000100001000100010000100010000;
 default:font=0;
 endcase end
 endfunction
endmodule

