// Read-only copy of the ORIGINAL UART telemetry bytes, including its CRC.
// NV1 type 0x83 = four-byte NV header + one complete 126-byte control V2 frame.
// Two 128-byte slots; a busy slot is never overwritten. UART/control never wait.
// The existing mailbox retains priority and its own pending slot; no second MAC.
module host_control_tx (
    input wire clk, rst, peer_valid, input wire [31:0] time_us,
    input wire tap_valid, tap_first, tap_last, input wire [7:0] tap_data,
    input wire mb_go, input wire [15:0] mb_len, output wire mb_busy,
    output wire mb_re, output wire [10:0] mb_addr, input wire [7:0] mb_q,
    output wire tx_go, output wire [15:0] tx_len, input wire tx_busy,
    input wire pl_re, input wire [10:0] pl_addr, output wire [7:0] pl_q,
    output reg [31:0] accepted, dropped, sent
);
    reg [1:0] full;
    reg write_bank, read_bank, writing;
    reg [6:0] offset;
    reg mb_pending;
    reg [15:0] mb_length;
    reg [31:0] stamp0,stamp1;
    localparam IDLE=0, MAILBOX=1, CONTROL=2;
    reg [1:0] owner;
    wire begin_capture = tap_first && peer_valid && !full[write_bank];
    wire fresh = (time_us-(read_bank ? stamp1 : stamp0)) <= 32'd100000;
    wire capture = tap_valid && (tap_first ? begin_capture : writing);
    wire [6:0] write_offset = tap_first ? 7'd0 : offset;
    wire [7:0] ram_q;
    wire [6:0] read_offset = pl_addr[6:0] - 7'd4;
    reg header_read;
    reg [7:0] header_q;
    nav_ram_sdp #(.DW(8), .AW(8)) u_frames (
        .clk(clk), .we(capture && !rst), .waddr({write_bank,write_offset}), .wdata(tap_data),
        .re(pl_re && owner==CONTROL), .raddr({read_bank,read_offset}), .rdata(ram_q)
    );
    // Mailbox can commit while a control packet is being copied to the TX FIFO.
    // Expose its own pending/active state so a later writer cannot reuse its RAM.
    assign mb_busy = mb_pending || owner==MAILBOX || mb_go;
    assign tx_go = !rst && owner==IDLE && !tx_busy &&
                   (mb_pending || (peer_valid && full[read_bank] && fresh));
    assign tx_len = mb_pending ? mb_length : 16'd130;
    assign mb_re = pl_re && owner==MAILBOX;
    assign mb_addr = pl_addr;
    assign pl_q = owner==MAILBOX ? mb_q : (header_read ? header_q : ram_q);
    always @(posedge clk) begin
        if (pl_re) begin
            header_read <= pl_addr < 4;
            case(pl_addr[1:0])
                0: header_q <= 8'h4e;
                1: header_q <= 8'h56;
                2: header_q <= 8'h01;
                3: header_q <= 8'h83;
            endcase
        end
        if (rst) begin
            full<=0;write_bank<=0;read_bank<=0;writing<=0;offset<=0;
            mb_pending<=0;mb_length<=0;owner<=IDLE;
            stamp0<=0;stamp1<=0;
            accepted<=0;dropped<=0;sent<=0;
        end else begin
            if (mb_go) begin mb_pending<=1;mb_length<=mb_len;end
            if(owner==IDLE && full[read_bank] && !fresh) begin
                full[read_bank]<=0;read_bank<=~read_bank;
            end
            if (tx_go) begin
                if(mb_pending) begin owner<=MAILBOX;mb_pending<=0;end
                else owner<=CONTROL;
            end else if(owner!=IDLE && !tx_busy) begin
                if(owner==CONTROL) begin
                    full[read_bank]<=0;read_bank<=~read_bank;sent<=sent+1'b1;
                end
                owner<=IDLE;
            end
            if(tap_valid) begin
                if(tap_first) begin
                    writing<=begin_capture;offset<=1;
                    if(!begin_capture) dropped<=dropped+1'b1;
                end else if(writing) begin
                    offset<=offset+1'b1;
                    if(tap_last) begin
                        writing<=0;
                        if(offset==125) begin
                            full[write_bank]<=1;write_bank<=~write_bank;accepted<=accepted+1'b1;
                            if(write_bank) stamp1<=time_us;else stamp0<=time_us;
                        end else dropped<=dropped+1'b1;
                    end else if(offset==125) begin writing<=0;dropped<=dropped+1'b1;end
                end
            end
        end
    end
endmodule
