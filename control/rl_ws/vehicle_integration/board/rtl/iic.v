// I2C主机：默认单字节寄存器读取；WRITE_MODE=1时写控制/寄存器和一个数据字节。
// 50 MHz输入，默认100 kHz；SCL/SDA均只拉低或释放，需要外部3.3V上拉。
// 按实际SCL电平等待时钟延展；超时释放总线，done/error返回给调用模块。
module iic #(
    parameter QUARTER_CYCLES = 125,
    parameter TIMEOUT_CYCLES = 50_000,
    parameter WRITE_MODE = 0, // 0读寄存器；1写控制/寄存器字节后再写一个数据字节
    parameter READ_BYTES = 1
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
        STOP_LOW=23, STOP_SETUP=24, STOP_RISE=25, STOP_RELEASE=26, FREE=27;
    reg [4:0] state=IDLE;
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
        state==REP_RISE || state==RX_RISE || state==NA_RISE || state==STOP_RISE;
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
            bit_index<=3'd7; tx_stage<=2'd0;
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
                timeout_cnt<=32'd0;
            end else timeout_cnt<=timeout_cnt+32'd1;
        end else begin
            timeout_cnt<=32'd0;
            if (quarter_cnt==QUARTER_CYCLES-1) begin
                quarter_cnt<=32'd0;
                case (state)
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
