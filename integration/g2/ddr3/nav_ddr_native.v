// One outstanding 32-byte transaction, across independent core/UI clocks.
// Native Gowin x32, BL8, 1:4 interface: addr is in 32-bit DDR words,
// low three bits zero; cmd 000=write, 001=read; mask 1=preserve byte.
// cold_rst must assert in BOTH domains. It is NOT the CPU warm reset.
module nav_ddr_native (
    input wire core_clk, ui_clk, cold_rst,
    input wire req, we,
    input wire [24:0] line_addr,
    input wire [255:0] wdata,
    input wire [31:0] wmask,
    output reg ack,
    output reg [255:0] rdata,
    input wire calib_done,
    output wire [2:0] cmd,
    output wire cmd_en,
    output wire [27:0] addr,
    input wire cmd_ready,
    output wire [255:0] wr_data,
    output wire [31:0] wr_data_mask,
    output wire wr_data_en, wr_data_end,
    input wire wr_data_rdy,
    input wire [255:0] rd_data,
    input wire rd_data_valid, rd_data_end,
    output reg fault
);
    wire core_reset, ui_reset;
    nav_ddr_reset u_core_reset(core_clk,cold_rst,core_reset);
    nav_ddr_reset u_ui_reset(ui_clk,cold_rst,ui_reset);
    reg request_toggle, done_toggle;
    (* syn_keep = 1 *) reg rq1, rq2, dn1, dn2;
    reg [24:0] held_addr;
    reg [255:0] held_data, reply;
    reg [31:0] held_mask;
    reg held_we;
    reg [1:0] source_state;
    // Bundled payload is held until the round-trip toggle completes. No
    // per-bit synchronizers: these payload paths require bounded delay/skew
    // and an actual CDC audit before physical integration.
    always @(posedge core_clk or posedge core_reset) begin
        if (core_reset) begin
            dn1 <= 0; dn2 <= 0; request_toggle <= 0;
            source_state <= 0; ack <= 0; rdata <= 0;
            held_addr <= 0; held_data <= 0; held_mask <= 0; held_we <= 0;
        end else begin
            dn1 <= done_toggle; dn2 <= dn1;
            case (source_state)
                0: if (req) begin
                    held_addr <= line_addr; held_data <= wdata;
                    held_mask <= wmask; held_we <= we;
                    request_toggle <= ~request_toggle;
                    source_state <= 1;
                end
                1: if (dn2 == request_toggle) begin
                    rdata <= reply; ack <= 1; source_state <= 2;
                end
                2: if (!req) begin ack <= 0; source_state <= 0; end
                default: source_state <= 0;
            endcase
        end
    end
    reg [1:0] ui_state;
    reg seen_calibration;
    reg command_sent, data_sent;
    wire active = ui_state == 1 && calib_done && !fault;
    assign cmd = held_we ? 3'b000 : 3'b001;
    assign cmd_en = active && !command_sent;
    assign addr = {held_addr, 3'b000};
    assign wr_data = held_data;
    assign wr_data_mask = held_mask;
    assign wr_data_en = active && held_we && !data_sent;
    assign wr_data_end = wr_data_en;
    always @(posedge ui_clk or posedge ui_reset) begin
        if (ui_reset) begin
            rq1 <= 0; rq2 <= 0; done_toggle <= 0; reply <= 0;
            ui_state <= 0; command_sent <= 0; data_sent <= 0; fault <= 0;
            seen_calibration <= 0;
        end else begin
            rq1 <= request_toggle; rq2 <= rq1;
            // Calibration loss after admission is fail-closed; do not
            // acknowledge a possibly incomplete transaction or re-zero RAM.
            if (calib_done) seen_calibration <= 1;
            if (seen_calibration && !calib_done) fault <= 1;
            if (!fault) case (ui_state)
                0: if (rq2 != done_toggle && calib_done) begin
                    command_sent <= 0; data_sent <= 0; ui_state <= 1;
                end
                1: if (calib_done) begin
                    if (cmd_en && cmd_ready) command_sent <= 1;
                    if (wr_data_en && wr_data_rdy) data_sent <= 1;
                    if (held_we && (command_sent || cmd_ready) &&
                        (data_sent || wr_data_rdy)) begin
                        reply <= 0; done_toggle <= rq2; ui_state <= 0;
                    end else if (!held_we &&
                        (command_sent || cmd_ready) && rd_data_valid) begin
                        if (!rd_data_end) fault <= 1;
                        else begin reply <= rd_data; done_toggle <= rq2; ui_state <= 0; end
                    end else if (!held_we && (command_sent || cmd_ready)) ui_state <= 2;
                end
                2: if (rd_data_valid && calib_done) begin
                    if (!rd_data_end) fault <= 1;
                    else begin reply <= rd_data; done_toggle <= rq2; ui_state <= 0; end
                end
                default: fault <= 1;
            endcase
        end
    end
endmodule

// Common asynchronous assertion, independently synchronized release. The
// clocks may start at different times; neither mailbox is rearmed warm.
module nav_ddr_reset(input wire clk, arst, output wire rst);
    (* syn_keep = 1 *) reg r1, r2;
    always @(posedge clk or posedge arst)
        if(arst)begin r1<=1; r2<=1; end
        else begin r1<=0; r2<=r1; end
    assign rst=r2;
endmodule
