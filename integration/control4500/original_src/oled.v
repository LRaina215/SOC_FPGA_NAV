// OLED 显示线程：依次写 TEXT/TEXT2/TEXT3/TEXT4 字符串 -> 写 LED_NUM 十进制数 -> 刷新 -> 循环。
// Unsigned 32-bit integer. DIGITS=1..10; fewer digits display the low decimal digits with leading zeros.
module oled #(
    // ===== 用户显示接口：字符串参数可以在 main 的实例化处设置 =====
    // SIX_AXIS_MODE=1: six signed values; 0: original TEXT/LED_NUM display.
    parameter SIX_AXIS_MODE=0,
    parameter WHEEL_HEADER_MODE=0,
    parameter MODEL_DEBUG_MODE=0,
    parameter [23:0] AXIS0_TEXT="AX:", AXIS1_TEXT="AY:", AXIS2_TEXT="AZ:",
    parameter [23:0] AXIS3_TEXT="GX:", AXIS4_TEXT="GY:", AXIS5_TEXT="GZ:",
    parameter TEXT_MAX=21,
    parameter [TEXT_MAX*8-1:0] TEXT="", // ASCII 字符串，例如 "HELLO"
    parameter TEXT_LEN=0,              // 字符个数，0 关闭文本显示，最大 TEXT_MAX
    parameter [7:0] TEXT_X=0, TEXT_Y=24,
    parameter [4:0] TEXT_FONT_SIZE=16,  // 8/12/16/24
    parameter TEXT_MODE=1'b1,          // 1 正常，0 反色
    parameter [TEXT_MAX*8-1:0] TEXT2="",
    parameter TEXT2_LEN=0,
    parameter [7:0] TEXT2_X=0, TEXT2_Y=16,
    parameter [4:0] TEXT2_FONT_SIZE=16,
    parameter TEXT2_MODE=1'b1,
    parameter [TEXT_MAX*8-1:0] TEXT3="",
    parameter TEXT3_LEN=0,
    parameter [7:0] TEXT3_X=0, TEXT3_Y=32,
    parameter [4:0] TEXT3_FONT_SIZE=16,
    parameter TEXT3_MODE=1'b1,
    parameter [TEXT_MAX*8-1:0] TEXT4="",
    parameter TEXT4_LEN=0,
    parameter [7:0] TEXT4_X=0, TEXT4_Y=48,
    parameter [4:0] TEXT4_FONT_SIZE=16,
    parameter TEXT4_MODE=1'b1,
    parameter DIGITS=4,
    parameter [7:0] X=0, Y=0,
    parameter [4:0] FONT_SIZE=16,
    parameter [6:0] ADDRESS=7'h3c,
    parameter STARTUP_CYCLES=10_000_000,
    parameter IIC_QUARTER_CYCLES=125,
    parameter IIC_TIMEOUT_CYCLES=50_000
)(
    input wire clk,reset_n,
    input wire [31:0] LED_NUM,
    input wire signed [15:0] VAL0,VAL1,VAL2,VAL3,VAL4,VAL5,
    input wire signed [15:0] SPEED_L,SPEED_R,
    input wire sample_valid,sensor_ready,sensor_error,
    input wire [3:0] sensor_error_code,
    input wire [35:0] DEBUG_STATUS, // {motor faults8, frames16, policy fault4, flags8}
    inout wire IIC_SCL,IIC_SDA,
    output wire ready,error
);
    generate if(SIX_AXIS_MODE) begin: axes_display
        oled_axes_thread #(
            .WHEEL_HEADER_MODE(WHEEL_HEADER_MODE),.MODEL_DEBUG_MODE(MODEL_DEBUG_MODE),
            .AXIS0_TEXT(AXIS0_TEXT),.AXIS1_TEXT(AXIS1_TEXT),.AXIS2_TEXT(AXIS2_TEXT),
            .AXIS3_TEXT(AXIS3_TEXT),.AXIS4_TEXT(AXIS4_TEXT),.AXIS5_TEXT(AXIS5_TEXT),
            .ADDRESS(ADDRESS),.STARTUP_CYCLES(STARTUP_CYCLES),
            .IIC_QUARTER_CYCLES(IIC_QUARTER_CYCLES),.IIC_TIMEOUT_CYCLES(IIC_TIMEOUT_CYCLES)
        ) thread(
            .clk(clk),.reset_n(reset_n),.IIC_SCL(IIC_SCL),.IIC_SDA(IIC_SDA),
            .VAL0(VAL0),.VAL1(VAL1),.VAL2(VAL2),.VAL3(VAL3),.VAL4(VAL4),.VAL5(VAL5),
            .SPEED_L(SPEED_L),.SPEED_R(SPEED_R),
            .sample_valid(sample_valid),.sensor_ready(sensor_ready),.sensor_error(sensor_error),
            .sensor_error_code(sensor_error_code),.DEBUG_STATUS(DEBUG_STATUS),.ready(ready),.error(error));
    end else begin: text_display
        oled_text_thread #(
            .TEXT_MAX(TEXT_MAX),
            .TEXT(TEXT),
            .TEXT_LEN(TEXT_LEN),
            .TEXT_X(TEXT_X),
            .TEXT_Y(TEXT_Y),
            .TEXT_FONT_SIZE(TEXT_FONT_SIZE),
            .TEXT_MODE(TEXT_MODE),
            .TEXT2(TEXT2),
            .TEXT2_LEN(TEXT2_LEN),
            .TEXT2_X(TEXT2_X),
            .TEXT2_Y(TEXT2_Y),
            .TEXT2_FONT_SIZE(TEXT2_FONT_SIZE),
            .TEXT2_MODE(TEXT2_MODE),
            .TEXT3(TEXT3),
            .TEXT3_LEN(TEXT3_LEN),
            .TEXT3_X(TEXT3_X),
            .TEXT3_Y(TEXT3_Y),
            .TEXT3_FONT_SIZE(TEXT3_FONT_SIZE),
            .TEXT3_MODE(TEXT3_MODE),
            .TEXT4(TEXT4),
            .TEXT4_LEN(TEXT4_LEN),
            .TEXT4_X(TEXT4_X),
            .TEXT4_Y(TEXT4_Y),
            .TEXT4_FONT_SIZE(TEXT4_FONT_SIZE),
            .TEXT4_MODE(TEXT4_MODE),
            .DIGITS(DIGITS),
            .X(X),
            .Y(Y),
            .FONT_SIZE(FONT_SIZE),
            .ADDRESS(ADDRESS),
            .STARTUP_CYCLES(STARTUP_CYCLES),
            .IIC_QUARTER_CYCLES(IIC_QUARTER_CYCLES),
            .IIC_TIMEOUT_CYCLES(IIC_TIMEOUT_CYCLES)
        ) thread(.clk(clk),.reset_n(reset_n),.LED_NUM(LED_NUM),
            .IIC_SCL(IIC_SCL),.IIC_SDA(IIC_SDA),.ready(ready),.error(error));
    end endgenerate
endmodule

// OLED 显示线程：依次写 TEXT/TEXT2/TEXT3/TEXT4 字符串 -> 写 LED_NUM 十进制数 -> 刷新 -> 循环。
// Unsigned 32-bit integer. DIGITS=1..10; fewer digits display the low decimal digits with leading zeros.
module oled_text_thread #(
    // ===== 用户显示接口：字符串参数可以在 main 的实例化处设置 =====
    parameter TEXT_MAX=21,
    parameter [TEXT_MAX*8-1:0] TEXT="", // ASCII 字符串，例如 "HELLO"
    parameter TEXT_LEN=0,              // 字符个数，0 关闭文本显示，最大 TEXT_MAX
    parameter [7:0] TEXT_X=0, TEXT_Y=24,
    parameter [4:0] TEXT_FONT_SIZE=16,  // 8/12/16/24
    parameter TEXT_MODE=1'b1,          // 1 正常，0 反色
    parameter [TEXT_MAX*8-1:0] TEXT2="",
    parameter TEXT2_LEN=0,
    parameter [7:0] TEXT2_X=0, TEXT2_Y=16,
    parameter [4:0] TEXT2_FONT_SIZE=16,
    parameter TEXT2_MODE=1'b1,
    parameter [TEXT_MAX*8-1:0] TEXT3="",
    parameter TEXT3_LEN=0,
    parameter [7:0] TEXT3_X=0, TEXT3_Y=32,
    parameter [4:0] TEXT3_FONT_SIZE=16,
    parameter TEXT3_MODE=1'b1,
    parameter [TEXT_MAX*8-1:0] TEXT4="",
    parameter TEXT4_LEN=0,
    parameter [7:0] TEXT4_X=0, TEXT4_Y=48,
    parameter [4:0] TEXT4_FONT_SIZE=16,
    parameter TEXT4_MODE=1'b1,
    parameter DIGITS=4,
    parameter [7:0] X=0, Y=0,
    parameter [4:0] FONT_SIZE=16,
    parameter [6:0] ADDRESS=7'h3c,
    parameter STARTUP_CYCLES=10_000_000,
    parameter IIC_QUARTER_CYCLES=125,
    parameter IIC_TIMEOUT_CYCLES=50_000
)(
    input wire clk,reset_n,
    input wire [31:0] LED_NUM,
    inout wire IIC_SCL,IIC_SDA,
    output wire ready,error
);
    localparam WAIT_READY=0,CONVERT=1,DRAW=2,DRAW_WAIT=3,REFRESH=4,REFRESH_WAIT=5,TEXT_DRAW=6,TEXT_WAIT=7;
    reg [2:0] state;
    reg [31:0] binary;
    reg [39:0] bcd,adjusted;
    reg [5:0] bit_count;
    reg [3:0] digit_index;
    reg [7:0] text_index;
    reg [1:0] text_line;
    localparam [7:0] COUNT1=(TEXT_LEN>TEXT_MAX) ? TEXT_MAX : TEXT_LEN;
    localparam [7:0] COUNT2=(TEXT2_LEN>TEXT_MAX) ? TEXT_MAX : TEXT2_LEN;
    localparam [7:0] COUNT3=(TEXT3_LEN>TEXT_MAX) ? TEXT_MAX : TEXT3_LEN;
    localparam [7:0] COUNT4=(TEXT4_LEN>TEXT_MAX) ? TEXT_MAX : TEXT4_LEN;
    wire [7:0] text_count=(text_line==0) ? COUNT1 :
        (text_line==1) ? COUNT2 : (text_line==2) ? COUNT3 : COUNT4;
    wire [TEXT_MAX*8-1:0] text_content=(text_line==0) ? TEXT :
        (text_line==1) ? TEXT2 : (text_line==2) ? TEXT3 : TEXT4;
    wire [7:0] text_origin_x=(text_line==0) ? TEXT_X :
        (text_line==1) ? TEXT2_X : (text_line==2) ? TEXT3_X : TEXT4_X;
    wire [7:0] text_y=(text_line==0) ? TEXT_Y :
        (text_line==1) ? TEXT2_Y : (text_line==2) ? TEXT3_Y : TEXT4_Y;
    wire [4:0] text_size=(text_line==0) ? TEXT_FONT_SIZE :
        (text_line==1) ? TEXT2_FONT_SIZE : (text_line==2) ? TEXT3_FONT_SIZE : TEXT4_FONT_SIZE;
    wire text_mode=(text_line==0) ? TEXT_MODE :
        (text_line==1) ? TEXT2_MODE : (text_line==2) ? TEXT3_MODE : TEXT4_MODE;
    wire [4:0] text_width=(text_size==8) ? 5'd6 : {1'b0,text_size[4:1]};
    wire text_active=(state==TEXT_DRAW || state==TEXT_WAIT);
    wire [7:0] text_char=(text_count>0) ? text_content[(text_count-1-text_index)*8 +: 8] : 8'd32;
    wire [15:0] text_x_full={8'd0,text_origin_x}+text_index*text_width;
    wire [7:0] text_x=(text_x_full<128) ? text_x_full[7:0] : 8'hff;
    reg start;
    wire busy,done;
    integer i;
    always @* begin
        adjusted=bcd;
        for(i=0;i<10;i=i+1)
            if(bcd[i*4 +: 4]>=5) adjusted[i*4 +: 4]=bcd[i*4 +: 4]+4'd3;
    end
    localparam CHAR_WIDTH=(FONT_SIZE==8) ? 6 : FONT_SIZE/2;
    wire [7:0] char_x=X+digit_index*CHAR_WIDTH;
    wire [7:0] digit={4'd0,bcd[(DIGITS-1-digit_index)*4 +: 4]};
    oled_driver #(.ADDRESS(ADDRESS),.AUTO_ADDRESS(1),.STARTUP_CYCLES(STARTUP_CYCLES),
        .IIC_QUARTER_CYCLES(IIC_QUARTER_CYCLES),.IIC_TIMEOUT_CYCLES(IIC_TIMEOUT_CYCLES)) driver(
        .clk(clk),.reset_n(reset_n),.OLED_SCL(IIC_SCL),.OLED_SDA(IIC_SDA),
        .start(start),.op((state==REFRESH || state==REFRESH_WAIT) ? 3'd5 : (text_active ? 3'd2 : 3'd7)),
        .x(text_active ? text_x : char_x),.y(text_active ? text_y : Y),
        .data(text_active ? text_char : digit),
        .font_size(text_active ? text_size : FONT_SIZE),
        .mode(text_active ? text_mode : 1'b1),
        .busy(busy),.ready(ready),.done(done),.error(error),.iic_error_code());
    always @(posedge clk) begin
        start<=0;
        if(!reset_n) begin
            state<=WAIT_READY;binary<=0;bcd<=0;bit_count<=0;digit_index<=0;text_index<=0;text_line<=0;
        end else case(state)
            WAIT_READY:if(ready && !busy) begin
                // Snapshot once per frame so changes cannot mix digits from two values.
                binary<=LED_NUM;bcd<=0;bit_count<=0;digit_index<=0;text_index<=0;text_line<=0;
                state<=TEXT_DRAW;
            end
            TEXT_DRAW:if(!busy) begin
                if(text_count==0) begin
                    if(text_line==3) state<=CONVERT;
                    else begin text_line<=text_line+1'b1;text_index<=0;end
                end else begin start<=1;state<=TEXT_WAIT;end
            end
            TEXT_WAIT:if(done) begin
                if(error) state<=WAIT_READY;
                else if(text_index==text_count-1) begin
                    text_index<=0;
                    if(text_line==3) state<=CONVERT;
                    else begin text_line<=text_line+1'b1;state<=TEXT_DRAW;end
                end
                else begin text_index<=text_index+1'b1;state<=TEXT_DRAW;end
            end
            CONVERT:begin
                bcd<={adjusted[38:0],binary[31]};binary<={binary[30:0],1'b0};
                if(bit_count==31) state<=DRAW;else bit_count<=bit_count+1'b1;
            end
            DRAW:if(!busy) begin start<=1;state<=DRAW_WAIT;end
            DRAW_WAIT:if(done) begin
                if(error) state<=WAIT_READY;
                else if(digit_index==DIGITS-1) state<=REFRESH;
                else begin digit_index<=digit_index+1'b1;state<=DRAW;end
            end
            REFRESH:if(!busy) begin start<=1;state<=REFRESH_WAIT;end
            REFRESH_WAIT:if(done) state<=WAIT_READY;
            default:state<=WAIT_READY;
        endcase
    end
endmodule


// Six-axis physical-unit display: 12px font; left acceleration(g), right gyro(dps).
// Capture a complete sample once per frame; never mix axes across sensor updates.
module oled_axes_thread #(
    parameter WHEEL_HEADER_MODE=0,
    parameter MODEL_DEBUG_MODE=0,
    parameter [23:0] AXIS0_TEXT="AX:",AXIS1_TEXT="AY:",AXIS2_TEXT="AZ:",
    parameter [23:0] AXIS3_TEXT="GX:",AXIS4_TEXT="GY:",AXIS5_TEXT="GZ:",
    parameter [6:0] ADDRESS=7'h3c,
    parameter STARTUP_CYCLES=10_000_000,
    parameter IIC_QUARTER_CYCLES=125,
    parameter IIC_TIMEOUT_CYCLES=50_000
)(
    input wire clk,reset_n,
    input wire signed [15:0] VAL0,VAL1,VAL2,VAL3,VAL4,VAL5,
    input wire signed [15:0] SPEED_L,SPEED_R,
    input wire sample_valid,sensor_ready,sensor_error,
    input wire [3:0] sensor_error_code,
    input wire [35:0] DEBUG_STATUS, // {motor faults8, frames16, policy fault4, flags8}
    inout wire IIC_SCL,IIC_SDA,
    output wire ready,error
);
    localparam IDLE=0,PREPARE=1,CONVERT=2,DRAW=3,DRAW_WAIT=4,REFRESH=5,REFRESH_WAIT=6,SCALE=7,WHEEL_PREPARE=8,WHEEL_CONVERT=9;
    reg [3:0] state;
    reg [2:0] segment;
    reg [4:0] char_index;
    reg [3:0] bit_index;
    reg [95:0] latest_values,frame_values;
    reg [15:0] frame_speed_l,frame_speed_r;
    reg [39:0] speed_bcd;
    reg [1:0] speed_negative;
    reg speed_lane;
    wire [15:0] speed_value=speed_lane ? frame_speed_r : frame_speed_l;
    reg sample_seen,frame_seen,frame_sensor_ready,frame_sensor_error;
    reg [3:0] frame_error_code;
    reg [35:0] frame_debug_status;
    function [7:0] hex_ascii;
        input [3:0] digit;
        begin hex_ascii=(digit<10) ? (8'h30+{4'd0,digit}) : (8'h41+{4'd0,digit}-8'd10);end
    endfunction
    reg [15:0] binary;
    reg [19:0] bcd,adjusted;
    reg negative,start;
    // Fixed-point units: accelerometer in 0.01g, gyro in 0.1 degree/s.
    // Bosch sensitivities for configured ranges: 10920 LSB/g; 16.384 LSB/dps.
    // Round absolute magnitude to nearest, then restore the sign (ties away from zero).
    reg [28:0] div_numerator,div_quotient;
    reg [14:0] div_remainder,div_denominator;
    reg [4:0] div_index;
    wire [15:0] magnitude=current_value[15] ? (~current_value+16'd1) : current_value;
    wire [28:0] acc_numerator={13'd0,magnitude}*29'd100+29'd5460;
    wire [28:0] gyro_numerator={13'd0,magnitude}*29'd10000+29'd8192;
    wire [15:0] div_step={div_remainder,div_numerator[28]};
    wire div_take=(div_step>={1'b0,div_denominator});
    wire [15:0] div_reduced=div_take ? (div_step-{1'b0,div_denominator}) : div_step;
    wire [28:0] quotient_next={div_quotient[27:0],div_take};
    wire busy,done;
    wire [2:0] axis_index=segment-3'd1;
    wire [15:0] current_value=frame_values[axis_index*16 +: 16];
    wire values_usable=frame_seen && frame_sensor_ready && !frame_sensor_error;
    integer i;
    always @* begin
        adjusted=bcd;
        for(i=0;i<5;i=i+1)
            if(bcd[i*4 +: 4]>=5) adjusted[i*4 +: 4]=bcd[i*4 +: 4]+4'd3;
    end
    // Full-width header clears any previous status string.
    reg [167:0] header;
    reg [23:0] label;
    reg [7:0] ascii,char_x,char_y;
    reg [7:0] hex_code;
    reg [19:0] wheel_digits;
    always @* begin
        header="BMI088: WAIT g/dps   ";
        if(frame_sensor_error) header="BMI088: ERR 0 g/dps  ";
        else if(values_usable) header="BMI088: OK g/dps     ";
        hex_code=(frame_error_code<10) ? 8'h30+{4'd0,frame_error_code} :
            8'h41+{4'd0,frame_error_code}-8'd10;
        case(axis_index)
            0:label=AXIS0_TEXT;1:label=AXIS1_TEXT;2:label=AXIS2_TEXT;
            3:label=AXIS3_TEXT;4:label=AXIS4_TEXT;default:label=AXIS5_TEXT;
        endcase
        char_x=0;char_y=0;ascii=8'h20;wheel_digits=0;
        if(segment==0) begin
            char_x={3'd0,char_index}*8'd6;
            ascii=header[167-char_index*8 -: 8];
            if(frame_sensor_error && char_index==12) ascii=hex_code;
            // Keep BMI WAIT/ERR visible when axes are unavailable.
            if(WHEEL_HEADER_MODE && values_usable) begin
                // Exactly21 chars, 126px at12px font: L+000.98 R+001.02 m/s.
                wheel_digits=(char_index>=9) ? speed_bcd[39:20] : speed_bcd[19:0];
                case(char_index)
                    0:ascii="L";9:ascii="R";
                    1:ascii=speed_negative[0] ? "-" : "+";
                    10:ascii=speed_negative[1] ? "-" : "+";
                    2,11:ascii=8'h30+{4'd0,wheel_digits[19:16]};
                    3,12:ascii=8'h30+{4'd0,wheel_digits[15:12]};
                    4,13:ascii=8'h30+{4'd0,wheel_digits[11:8]};
                    5,14:ascii=".";
                    6,15:ascii=8'h30+{4'd0,wheel_digits[7:4]};
                    7,16:ascii=8'h30+{4'd0,wheel_digits[3:0]};
                    18:ascii="m";19:ascii="/";20:ascii="s";
                    default:ascii=" ";
                endcase
            end
            if(MODEL_DEBUG_MODE && values_usable) begin
                // Exactly 21 ASCII chars: M:81 F:0 N:0001 E:00 + trailing space.
                case(char_index)
                    0:ascii="M";1,6,10,17:ascii=":";
                    2:ascii=hex_ascii(frame_debug_status[7:4]);
                    3:ascii=hex_ascii(frame_debug_status[3:0]);
                    5:ascii="F";7:ascii=hex_ascii(frame_debug_status[11:8]);
                    9:ascii="N";
                    11:ascii=hex_ascii(frame_debug_status[27:24]);
                    12:ascii=hex_ascii(frame_debug_status[23:20]);
                    13:ascii=hex_ascii(frame_debug_status[19:16]);
                    14:ascii=hex_ascii(frame_debug_status[15:12]);
                    16:ascii="E";
                    18:ascii=hex_ascii(frame_debug_status[35:32]);
                    19:ascii=hex_ascii(frame_debug_status[31:28]);
                    default:ascii=" ";
                endcase
            end
        end else begin
            char_x=((axis_index>=3) ? 8'd64 : 8'd0)+{3'd0,char_index}*8'd6;
            case(axis_index)
                0,3:char_y=8'd16;1,4:char_y=8'd32;default:char_y=8'd48;
            endcase
            case(char_index)
                0:ascii=label[23:16];1:ascii=label[15:8];2:ascii=label[7:0];
                3:ascii=values_usable ? (negative ? 8'h2d : 8'h2b) : 8'h20;
                default:begin
                    if(axis_index<3) begin
                        case(char_index)
                            4:ascii=values_usable ? (8'h30+{4'd0,bcd[11:8]}) : 8'h2d;
                            5:ascii=8'h2e;
                            6:ascii=values_usable ? (8'h30+{4'd0,bcd[7:4]}) : 8'h2d;
                            7:ascii=values_usable ? (8'h30+{4'd0,bcd[3:0]}) : 8'h2d;
                            8:ascii=8'h67; // g
                            default:ascii=8'h20;
                        endcase
                    end else begin
                        case(char_index)
                            4:ascii=values_usable ? (8'h30+{4'd0,bcd[19:16]}) : 8'h2d;
                            5:ascii=values_usable ? (8'h30+{4'd0,bcd[15:12]}) : 8'h2d;
                            6:ascii=values_usable ? (8'h30+{4'd0,bcd[11:8]}) : 8'h2d;
                            7:ascii=values_usable ? (8'h30+{4'd0,bcd[7:4]}) : 8'h2d;
                            8:ascii=8'h2e;
                            default:ascii=values_usable ? (8'h30+{4'd0,bcd[3:0]}) : 8'h2d;
                        endcase
                    end
                end
            endcase
        end
    end
    oled_driver #(.ADDRESS(ADDRESS),.AUTO_ADDRESS(1),.STARTUP_CYCLES(STARTUP_CYCLES),
        .IIC_QUARTER_CYCLES(IIC_QUARTER_CYCLES),.IIC_TIMEOUT_CYCLES(IIC_TIMEOUT_CYCLES)) driver(
        .clk(clk),.reset_n(reset_n),.OLED_SCL(IIC_SCL),.OLED_SDA(IIC_SDA),.start(start),
        .op((state==REFRESH || state==REFRESH_WAIT) ? 3'd5 : 3'd2),
        .x(char_x),.y(char_y),.data(ascii),.font_size(5'd12),.mode(1'b1),
        .busy(busy),.done(done),.ready(ready),.error(error),.iic_error_code());
    always @(posedge clk) begin
        if(!reset_n) begin latest_values<=0;sample_seen<=0;end
        else begin
            if(!sensor_ready || sensor_error) sample_seen<=0;
            else if(sample_valid) begin
                latest_values<={VAL5,VAL4,VAL3,VAL2,VAL1,VAL0};sample_seen<=1;
            end
        end
    end
    always @(posedge clk) begin
        start<=0;
        if(!reset_n) begin
            state<=IDLE;segment<=0;char_index<=0;bit_index<=0;binary<=0;bcd<=0;
            frame_values<=0;frame_seen<=0;frame_sensor_ready<=0;frame_sensor_error<=0;
            frame_error_code<=0;frame_debug_status<=0;negative<=0;
            frame_speed_l<=0;frame_speed_r<=0;speed_bcd<=0;speed_negative<=0;speed_lane<=0;
            div_numerator<=0;div_quotient<=0;div_remainder<=0;div_denominator<=0;div_index<=0;
        end else case(state)
            IDLE:if(ready && !busy) begin
                frame_values<=latest_values;frame_seen<=sample_seen;
                frame_sensor_ready<=sensor_ready;frame_sensor_error<=sensor_error;
                frame_error_code<=sensor_error_code;frame_debug_status<=DEBUG_STATUS;segment<=0;char_index<=0;
                frame_speed_l<=SPEED_L;frame_speed_r<=SPEED_R;speed_lane<=0;
                state<=WHEEL_HEADER_MODE ? WHEEL_PREPARE : DRAW;
            end
            WHEEL_PREPARE:begin
                binary<=speed_value[15] ? (~speed_value+16'd1) : speed_value;
                speed_negative[speed_lane]<=speed_value[15] && (speed_value!=0);
                bcd<=0;bit_index<=0;state<=WHEEL_CONVERT;
            end
            WHEEL_CONVERT:begin
                bcd<={adjusted[18:0],binary[15]};binary<={binary[14:0],1'b0};
                if(bit_index==15) begin
                    speed_bcd[speed_lane*20 +: 20]<={adjusted[18:0],binary[15]};
                    if(speed_lane) state<=DRAW;
                    else begin speed_lane<=1;state<=WHEEL_PREPARE;end
                end else bit_index<=bit_index+1'b1;
            end
            PREPARE:begin
                negative<=current_value[15];
                div_numerator<=(axis_index<3) ? acc_numerator : gyro_numerator;
                div_denominator<=(axis_index<3) ? 15'd10920 : 15'd16384;
                div_quotient<=0;div_remainder<=0;div_index<=0;
                bcd<=0;bit_index<=0;char_index<=0;state<=SCALE;
            end
            SCALE:begin
                div_numerator<={div_numerator[27:0],1'b0};
                div_remainder<=div_reduced[14:0];div_quotient<=quotient_next;
                if(div_index==28) begin
                    binary<=quotient_next[15:0];
                    negative<=negative && (quotient_next!=0); // Avoid -0.00g / -0000.0.
                    state<=CONVERT;
                end else div_index<=div_index+1'b1;
            end
            CONVERT:begin
                bcd<={adjusted[18:0],binary[15]};binary<={binary[14:0],1'b0};
                if(bit_index==15) state<=DRAW;else bit_index<=bit_index+1'b1;
            end
            DRAW:if(!busy) begin start<=1;state<=DRAW_WAIT;end
            DRAW_WAIT:if(done) begin
                if(error) state<=IDLE;
                else if((segment==0 && char_index==20) || (segment!=0 && char_index==9)) begin
                    char_index<=0;
                    if(segment==6) state<=REFRESH;
                    else begin segment<=segment+1'b1;state<=PREPARE;end
                end else begin char_index<=char_index+1'b1;state<=DRAW;end
            end
            REFRESH:if(!busy) begin start<=1;state<=REFRESH_WAIT;end
            REFRESH_WAIT:if(done) state<=IDLE;
            default:state<=IDLE;
        endcase
    end
endmodule
