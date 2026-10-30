// elev_map 平均模式合并环路的独立时序估计（寄存器打拍的输入输出，扫描口闲置）
module em_top (
    input  wire clk, input wire rst, input wire clear, input wire cfg_avg, input wire [2:0] cfg_avg_shift,
    input  wire s_valid, input wire s_upd, input wire [13:0] s_addr, input wire [7:0] s_tag,
    input  wire signed [15:0] s_z, input wire s_eof, input wire cpu_req, input wire [13:0] cpu_raddr,
    output reg  [7:0] q_out, output wire s_ready, output reg cpu_ack_q
);
    reg v, u, e, cr; reg [13:0] a, ca; reg [7:0] t; reg signed [15:0] z; reg avg; reg [2:0] sh;
    always @(posedge clk) begin v <= s_valid; u <= s_upd; a <= s_addr; t <= s_tag; z <= s_z; e <= s_eof;
        avg <= cfg_avg; sh <= cfg_avg_shift; cr <= cpu_req; ca <= cpu_raddr; end
    wire [7:0] fc; wire fd, busy, ack; wire [52:0] rd;
    elev_map u_em (.clk(clk), .rst(rst), .clear(clear), .cfg_avg(avg), .cfg_avg_shift(sh),
        .s_valid(v), .s_ready(s_ready), .s_upd(u), .s_addr(a), .s_tag(t), .s_z(z), .s_eof(e),
        .fc(fc), .frame_done(fd), .busy(busy), .swp_sel(1'b0), .swp_re(1'b0), .swp_raddr(14'd0), .swp_rdata(rd),
        .swp_we(1'b0), .swp_waddr(14'd0), .swp_wdata(53'd0), .cpu_req(cr), .cpu_raddr(ca), .cpu_ack(ack));
    always @(posedge clk) begin
        cpu_ack_q <= ack;
        q_out <= {^rd[52:44], ^rd[43:36], ^rd[35:32], ^rd[31:24], ^rd[23:16], ^rd[15:8], ^rd[7:0], fd ^ busy ^ ^fc};
    end
endmodule
