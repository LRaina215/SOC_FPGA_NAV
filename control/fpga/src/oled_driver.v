// OLED 功能资源：初始化、清屏、画点、写字符/字母、写单个数字、汉字、位图、刷新。
// 线程提供 start/op/坐标/数据；busy/done/error 返回操作状态。内部管理显存与 I2C。
// 字符串和多位数由 oled.v 线程连续调用字符/单个数字操作。
module oled_driver #(
    parameter [6:0] ADDRESS=7'h3c,
    parameter AUTO_ADDRESS=0, // Retry the other SSD1306 address once during boot.
    parameter STARTUP_CYCLES=10_000_000,
    parameter IIC_QUARTER_CYCLES=125,
    parameter IIC_TIMEOUT_CYCLES=50_000
)(
    input wire clk, reset_n,
    inout wire OLED_SCL, OLED_SDA,
    input wire start,
    input wire [2:0] op, // 0 clear, 1 pixel, 2 ASCII, 3 Chinese16, 4 byte, 5 refresh, 6 command, 7 decimal digit
    input wire [7:0] x, y, data,
    input wire [4:0] font_size, // ASCII: 8,12,16,24; Chinese: fixed 16
    input wire mode, // pixel: value; glyph: 1 normal, 0 inverted
    output wire busy,
    output reg ready, done, error,
    output wire [3:0] iic_error_code
);
    localparam BOOT=0,CLEAR=1,ISSUE=2,BUS_WAIT=3,IDLE=4,
        PIX_READ=5,PIX_WRITE=6,PIX_NEXT=7,FETCH=8,FINISH=9,FAULT=10;
    localparam INIT=0,FRAME=1,ON=2,COMMAND=3;
    reg [3:0] state;
    reg [1:0] phase;
    reg [31:0] delay_count;
    reg [9:0] clear_addr;
    reg [4:0] init_index;
    reg [2:0] page;
    reg [7:0] position;
    reg [7:0] gram [0:1023];
    reg [7:0] ram_q;
    reg [9:0] ram_addr;
    reg tx_start;
    reg [7:0] control, payload;
    wire tx_done, tx_error;
    reg booting;
    reg [6:0] oled_addr;
    reg address_tried;
    reg [7:0] x0,y0,code;
    reg [4:0] size_i,width,height,col,row;
    reg chinese_i,mode_i,pixel_only;
    wire [8:0] px={1'b0,x0}+col, py={1'b0,y0}+row;
    wire [5:0] font_index=(row>>3)*width+col;
    wire [7:0] font_data;
    wire [7:0] frame_column=position-8'd3;
    wire pixel_value=pixel_only ? mode_i : (font_data[row[2:0]] ^ !mode_i);
    assign busy=(state!=IDLE) || !ready;
    oled_front font(.chinese(chinese_i),.code(code),.size(size_i),
        .byte_index(font_index),.data(font_data));
    iic #(.WRITE_MODE(1),.QUARTER_CYCLES(IIC_QUARTER_CYCLES),
        .TIMEOUT_CYCLES(IIC_TIMEOUT_CYCLES)) bus(
        .clk(clk),.reset_n(reset_n),.start(tx_start),.device_addr(oled_addr),
        .register_addr(control),.write_data(payload),.IIC_SCL(OLED_SCL),
        .IIC_SDA(OLED_SDA),.done(tx_done),.error(tx_error),.error_code(iic_error_code),
        .busy(),.read_data(),.read_bytes());
    function [7:0] init_command;
        input [4:0] index;
        begin
            case(index)
                0:init_command=8'hae; 1:init_command=8'h00; 2:init_command=8'h10;
                3:init_command=8'h40; 4:init_command=8'h81; 5:init_command=8'hcf;
                6:init_command=8'ha1; 7:init_command=8'hc8; 8:init_command=8'ha6;
                9:init_command=8'ha8; 10:init_command=8'h3f; 11:init_command=8'hd3;
                12:init_command=8'h00; 13:init_command=8'hd5; 14:init_command=8'h80;
                15:init_command=8'hd9; 16:init_command=8'hf1; 17:init_command=8'hda;
                18:init_command=8'h12; 19:init_command=8'hdb; 20:init_command=8'h40;
                21:init_command=8'h20; 22:init_command=8'h02; 23:init_command=8'h8d;
                24:init_command=8'h14; 25:init_command=8'ha4; default:init_command=8'ha6;
            endcase
        end
    endfunction
    always @(posedge clk) begin
        done<=0; tx_start<=0;
        if(!reset_n) begin
            state<=BOOT;phase<=INIT;delay_count<=0;clear_addr<=0;init_index<=0;
            page<=0;position<=0;ready<=0;error<=0;booting<=1;
            oled_addr<=ADDRESS;address_tried<=0;
            control<=0;payload<=0;ram_q<=0;ram_addr<=0;
            x0<=0;y0<=0;code<=0;size_i<=8;width<=6;height<=8;
            col<=0;row<=0;chinese_i<=0;mode_i<=1;pixel_only<=0;
        end else case(state)
            BOOT: begin
                if(STARTUP_CYCLES==0 || delay_count>=STARTUP_CYCLES-1) begin
                    clear_addr<=0;state<=CLEAR;
                end else delay_count<=delay_count+1'b1;
            end
            CLEAR: begin
                gram[clear_addr]<=0;
                if(clear_addr==1023) begin
                    if(booting) begin phase<=INIT;state<=ISSUE;end
                    else state<=FINISH;
                end else clear_addr<=clear_addr+1'b1;
            end
            IDLE: if(start) begin
                error<=0;
                case(op)
                    0:begin clear_addr<=0;state<=CLEAR;end
                    1,2,3,7:begin
                        x0<=x;y0<=y;code<=(op==7) ? (8'h30+data) : data;mode_i<=mode;
                        col<=0;row<=0;chinese_i<=(op==3);pixel_only<=(op==1);
                        size_i<=font_size;state<=PIX_READ;
                        if(op==1) begin width<=1;height<=1;end
                        else if(op==3) begin width<=16;height<=16;end
                        else case(font_size)
                            8:begin width<=6;height<=8;end
                            12:begin width<=6;height<=12;end
                            16:begin width<=8;height<=16;end
                            24:begin width<=12;height<=24;end
                            default:begin error<=1;state<=FINISH;end
                        endcase
                        if(op==7 && data>9) begin error<=1;state<=FINISH;end
                    end
                    4:begin
                        // y is a pixel coordinate: its upper bits select the page.
                        if(x<128 && y<64) gram[{y[5:3],x[6:0]}]<=data;
                        state<=FINISH;
                    end
                    5:begin page<=0;position<=0;phase<=FRAME;state<=ISSUE;end
                    6:begin phase<=COMMAND;control<=0;payload<=data;state<=ISSUE;end
                    default:begin error<=1;state<=FINISH;end
                endcase
            end
            PIX_READ: begin
                if(px<128 && py<64) begin
                    ram_addr<={py[5:3],px[6:0]};
                    ram_q<=gram[{py[5:3],px[6:0]}];state<=PIX_WRITE;
                end else state<=PIX_NEXT;
            end
            PIX_WRITE:begin
                gram[ram_addr]<=pixel_value ? (ram_q | (8'h01<<py[2:0])) :
                    (ram_q & ~(8'h01<<py[2:0]));
                state<=PIX_NEXT;
            end
            PIX_NEXT:begin
                if(row==height-1) begin
                    row<=0;
                    if(col==width-1) state<=FINISH;
                    else begin col<=col+1'b1;state<=PIX_READ;end
                end else begin row<=row+1'b1;state<=PIX_READ;end
            end
            ISSUE:begin
                if(phase==INIT) begin control<=0;payload<=init_command(init_index);end
                else if(phase==ON) begin control<=0;payload<=8'haf;end
                else if(phase==FRAME) begin
                    control<=0;
                    case(position)
                        0:payload<=8'hb0+page;
                        1:payload<=0;
                        2:payload<=8'h10;
                        default:begin state<=FETCH;ram_addr<={page,frame_column[6:0]};end
                    endcase
                end
                if(phase!=FRAME || position<3) begin tx_start<=1;state<=BUS_WAIT;end
            end
            FETCH:begin control<=8'h40;payload<=gram[ram_addr];tx_start<=1;state<=BUS_WAIT;end
            BUS_WAIT:if(tx_done) begin
                if(tx_error) begin
                    error<=1;
                    if(booting && AUTO_ADDRESS && !address_tried && iic_error_code==1) begin
                        oled_addr<=oled_addr ^ 7'h01;address_tried<=1;error<=0;
                        init_index<=0;phase<=INIT;state<=ISSUE;
                    end else if(booting) state<=FAULT;else state<=FINISH;
                end else case(phase)
                    INIT:if(init_index==26) begin
                        page<=0;position<=0;phase<=FRAME;state<=ISSUE;
                    end else begin init_index<=init_index+1'b1;state<=ISSUE;end
                    FRAME:if(position==130) begin
                        position<=0;
                        if(page==7) begin
                            if(booting) begin phase<=ON;state<=ISSUE;end
                            else state<=FINISH;
                        end else begin page<=page+1'b1;state<=ISSUE;end
                    end else begin position<=position+1'b1;state<=ISSUE;end
                    ON:begin ready<=1;booting<=0;state<=IDLE;end
                    COMMAND:state<=FINISH;
                endcase
            end
            FINISH:begin done<=1;state<=IDLE;end
            FAULT:begin ready<=0;end // Reset to retry a failed boot.
            default:state<=BOOT;
        endcase
    end
endmodule

// 通用 I2C 主机：收在驱动文件内，工程无需另加 iic.v。
// I2C主机：默认单字节寄存器读取；WRITE_MODE=1时写控制/寄存器和一个数据字节。
// 50 MHz输入，默认100 kHz；SCL/SDA均只拉低或释放，需要外部3.3V上拉。
// 按实际SCL电平等待时钟延展；超时释放总线，done/error返回给调用模块。
module iic #(
    parameter QUARTER_CYCLES = 125,
    parameter TIMEOUT_CYCLES = 50_000,
    parameter WRITE_MODE = 0, // 0读寄存器；1写控制/寄存器字节后再写一个数据字节
    parameter READ_BYTES = 1,
    parameter BUS_CLEAR = 0 // SDA卡低时发送9个时钟和STOP；默认保持原行为
) (
    input wire clk,
    input wire reset_n,
    input wire start,
    input wire [6:0] device_addr,
    input wire [7:0] register_addr,
    input wire [7:0] write_data,
    inout wire IIC_SCL,
    inout wire IIC_SDA,
    output reg busy = 1'b0,
    output reg done = 1'b0,
    output reg error = 1'b0,
    output reg [7:0] read_data = 8'd0,
    output reg [63:0] read_bytes = 64'd0,
    // 最近一次事务结果：0正常，1地址W无ACK，2控制/寄存器无ACK，3地址R/写数据无ACK，
    // 4等待SCL拉高超时，5空闲SDA被拉低超时。保留至下一事务结束。
    output reg [3:0] error_code = 4'd0
);
    localparam [4:0] IDLE=0, WAIT_IDLE=1, START_HOLD=2,
        TX_LOW=3, TX_SETUP=4, TX_RISE=5, TX_HIGH=6,
        ACK_LOW=7, ACK_SETUP=8, ACK_RISE=9, ACK_HIGH=10, ACK_END=11,
        REP_SETUP=12, REP_RISE=13, REP_HOLD=14,
        RX_LOW=15, RX_SETUP=16, RX_RISE=17, RX_HIGH=18,
        NA_LOW=19, NA_SETUP=20, NA_RISE=21, NA_HIGH=22,
        STOP_LOW=23, STOP_SETUP=24, STOP_RISE=25, STOP_RELEASE=26, FREE=27,
        CLEAR_LOW=28, CLEAR_SETUP=29, CLEAR_RISE=30, CLEAR_HIGH=31;
    reg [4:0] state=IDLE;
    reg [3:0] clear_index=0;
    reg scl_low=1'b0, sda_low=1'b0;
    assign IIC_SCL = scl_low ? 1'b0 : 1'bz;
    assign IIC_SDA = sda_low ? 1'b0 : 1'bz;
    reg scl_sync_0=1'b1, scl_sync_1=1'b1;
    reg sda_sync_0=1'b1, sda_sync_1=1'b1;
    reg [31:0] quarter_cnt=32'd0, timeout_cnt=32'd0;
    reg [7:0] tx_shift=8'd0, register_latched=8'd0;
    reg [7:0] write_latched=8'd0;
    reg [6:0] addr_latched=7'd0;
    reg [2:0] bit_index=3'd7;
    reg [3:0] bytes_received=4'd0;
    reg [1:0] tx_stage=2'd0;
    wire high_wait = state==WAIT_IDLE || state==TX_RISE || state==ACK_RISE ||
        state==REP_RISE || state==RX_RISE || state==NA_RISE || state==STOP_RISE || state==CLEAR_RISE;
    wire line_blocked = high_wait && (!scl_sync_1 || (state==WAIT_IDLE && !sda_sync_1));
    always @(posedge clk) begin
        if (!reset_n) begin
            scl_sync_0<=1'b1; scl_sync_1<=1'b1;
            sda_sync_0<=1'b1; sda_sync_1<=1'b1;
        end else begin
            scl_sync_0<=IIC_SCL; scl_sync_1<=scl_sync_0;
            sda_sync_0<=IIC_SDA; sda_sync_1<=sda_sync_0;
        end
    end
    always @(posedge clk) begin
        done<=1'b0;
        if (!reset_n) begin
            state<=IDLE; scl_low<=1'b0; sda_low<=1'b0;
            busy<=1'b0; error<=1'b0; read_data<=8'd0; read_bytes<=64'd0;
            bytes_received<=4'd0;
            error_code<=4'd0;
            quarter_cnt<=32'd0; timeout_cnt<=32'd0;
            tx_shift<=8'd0; register_latched<=8'd0; addr_latched<=7'd0;
            write_latched<=8'd0;
            bit_index<=3'd7; tx_stage<=2'd0; clear_index<=0;
        end else if (state==IDLE) begin
            if (start) begin
                busy<=1'b1; error<=1'b0; read_data<=8'd0; read_bytes<=64'd0;
                bytes_received<=4'd0;
                addr_latched<=device_addr; register_latched<=register_addr;
                write_latched<=write_data;
                tx_shift<={device_addr,1'b0}; bit_index<=3'd7; tx_stage<=2'd0;
                scl_low<=1'b0; sda_low<=1'b0;
                quarter_cnt<=32'd0; timeout_cnt<=32'd0; state<=WAIT_IDLE;
            end
        end else if (line_blocked) begin
            quarter_cnt<=32'd0;
            if (timeout_cnt==TIMEOUT_CYCLES-1) begin
                error<=1'b1; done<=1'b1; busy<=1'b0;
                error_code<=!scl_sync_1 ? 4'd4 : 4'd5;
                scl_low<=1'b0; sda_low<=1'b0; state<=IDLE;
                if (BUS_CLEAR && state==WAIT_IDLE && scl_sync_1) begin
                    // 保留事务失败结果；上层重试前恢复总线，不伪造有效数据。
                    done<=1'b0; busy<=1'b1; clear_index<=0;
                    scl_low<=1'b1; state<=CLEAR_LOW;
                end
                timeout_cnt<=32'd0;
            end else timeout_cnt<=timeout_cnt+32'd1;
        end else begin
            timeout_cnt<=32'd0;
            if (quarter_cnt==QUARTER_CYCLES-1) begin
                quarter_cnt<=32'd0;
                case (state)
                    CLEAR_LOW: state<=CLEAR_SETUP;
                    CLEAR_SETUP: begin scl_low<=1'b0; state<=CLEAR_RISE; end
                    CLEAR_RISE: state<=CLEAR_HIGH;
                    CLEAR_HIGH: begin
                        scl_low<=1'b1;
                        if (clear_index==8) state<=STOP_LOW;
                        else begin clear_index<=clear_index+1'b1; state<=CLEAR_LOW; end
                    end
                    WAIT_IDLE: begin sda_low<=1'b1; state<=START_HOLD; end
                    START_HOLD: begin scl_low<=1'b1; state<=TX_LOW; end
                    TX_LOW: begin sda_low<=~tx_shift[7]; state<=TX_SETUP; end
                    TX_SETUP: begin scl_low<=1'b0; state<=TX_RISE; end
                    TX_RISE: state<=TX_HIGH;
                    TX_HIGH: begin
                        scl_low<=1'b1;
                        if (bit_index==0) state<=ACK_LOW;
                        else begin
                            bit_index<=bit_index-3'd1;
                            tx_shift<={tx_shift[6:0],1'b0}; state<=TX_LOW;
                        end
                    end
                    ACK_LOW: begin sda_low<=1'b0; state<=ACK_SETUP; end
                    ACK_SETUP: begin scl_low<=1'b0; state<=ACK_RISE; end
                    ACK_RISE: begin
                        if (sda_sync_1) begin
                            error<=1'b1;
                            error_code<={2'b00,tx_stage}+4'd1;
                        end
                        state<=ACK_HIGH;
                    end
                    ACK_HIGH: begin scl_low<=1'b1; state<=ACK_END; end
                    ACK_END: begin
                        bit_index<=3'd7;
                        if (error) state<=STOP_LOW;
                        else if (tx_stage==0) begin
                            tx_stage<=2'd1; tx_shift<=register_latched;
                            sda_low<=~register_latched[7]; state<=TX_SETUP;
                        end else if (tx_stage==1) begin
                            if (WRITE_MODE) begin
                                tx_stage<=2'd2; tx_shift<=write_latched;
                                sda_low<=~write_latched[7]; state<=TX_SETUP;
                            end else begin sda_low<=1'b0; state<=REP_SETUP; end
                        end else if (WRITE_MODE) begin
                            state<=STOP_LOW;
                        end else begin
                            sda_low<=1'b0; state<=RX_SETUP;
                        end
                    end
                    REP_SETUP: begin scl_low<=1'b0; state<=REP_RISE; end
                    REP_RISE: begin sda_low<=1'b1; state<=REP_HOLD; end
                    REP_HOLD: begin
                        scl_low<=1'b1; tx_stage<=2'd2;
                        tx_shift<={addr_latched,1'b1}; state<=TX_LOW;
                    end
                    RX_LOW: state<=RX_SETUP;
                    RX_SETUP: begin scl_low<=1'b0; state<=RX_RISE; end
                    RX_RISE: begin
                        read_data<={read_data[6:0],sda_sync_1};
                        read_bytes<={read_bytes[62:0],sda_sync_1};
                        state<=RX_HIGH;
                    end
                    RX_HIGH: begin
                        scl_low<=1'b1;
                        if (bit_index==0) state<=NA_LOW;
                        else begin bit_index<=bit_index-3'd1; state<=RX_LOW; end
                    end
                    NA_LOW: begin
                        sda_low<=bytes_received<READ_BYTES-1;
                        state<=NA_SETUP;
                    end
                    NA_SETUP: begin scl_low<=1'b0; state<=NA_RISE; end
                    NA_RISE: state<=NA_HIGH;
                    NA_HIGH: begin
                        scl_low<=1'b1;
                        if (bytes_received<READ_BYTES-1) begin
                            bytes_received<=bytes_received+4'd1;
                            bit_index<=3'd7;
                            sda_low<=1'b0;
                            state<=RX_LOW;
                        end else state<=STOP_LOW;
                    end
                    STOP_LOW: begin sda_low<=1'b1; state<=STOP_SETUP; end
                    STOP_SETUP: begin scl_low<=1'b0; state<=STOP_RISE; end
                    STOP_RISE: begin sda_low<=1'b0; state<=STOP_RELEASE; end
                    STOP_RELEASE: state<=FREE;
                    FREE: begin
                        busy<=1'b0; done<=1'b1; state<=IDLE;
                        if (!error) error_code<=4'd0;
                    end
                    default: begin
                        error<=1'b1; busy<=1'b0; done<=1'b1;
                        error_code<=4'd6;
                        scl_low<=1'b0; sda_low<=1'b0; state<=IDLE;
                    end
                endcase
            end else quarter_cnt<=quarter_cnt+32'd1;
        end
    end
endmodule
