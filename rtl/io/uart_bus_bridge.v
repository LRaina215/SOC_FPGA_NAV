// 串口 -> 寄存器总线桥（调试/上板联调用，软核搭好之前 PC 可直接读写导航子系统全部寄存器）。
// 协议（8N1，二进制，多字节大端）：
//   写：'W' A1 A0 D3 D2 D1 D0      -> 回 'K'
//   读：'R' A1 A0                  -> 回 'D' D3 D2 D1 D0
//   其它字节忽略（可用于同步）；帧内字节间隔超过 ~1 ms 丢弃该帧。
// 主控仲裁：与软核共用总线时放在 nav_bus_arb 前面，见 gowin/nav_board_top.v。
module uart_bus_bridge #(
    parameter integer CLK_HZ = 100000000,
    parameter integer BAUD   = 921600
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        rxd,
    output wire        txd,

    output reg         bus_req,
    output reg         bus_we,
    output reg  [15:0] bus_addr,
    output reg  [31:0] bus_wdata,
    input  wire        bus_ack,
    input  wire [31:0] bus_rdata
);
    localparam integer DIV = CLK_HZ / BAUD;
    localparam integer TO  = CLK_HZ / 1000;          // 帧内超时 1 ms

    // ---------------- 接收 ----------------
    wire rs;
    nav_sync2 #(.W(1)) u_s (.clk(clk), .d(rxd), .q(rs));
    reg [15:0] rc;
    reg [3:0]  rbit;
    reg        rbusy, rv;
    reg [7:0]  rsh, rb;
    always @(posedge clk) begin
        rv <= 1'b0;
        if (rst) begin
            rbusy <= 1'b0;
        end else if (!rbusy) begin
            if (!rs) begin rbusy <= 1'b1; rc <= DIV / 2; rbit <= 0; end
        end else if (rc == 0) begin
            rc <= DIV - 1;
            if (rbit == 0) begin
                if (rs) rbusy <= 1'b0;
                rbit <= 1;
            end else if (rbit <= 8) begin
                rsh <= {rs, rsh[7:1]};
                rbit <= rbit + 1'b1;
            end else begin
                rbusy <= 1'b0;
                if (rs) begin rv <= 1'b1; rb <= rsh; end
            end
        end else rc <= rc - 1'b1;
    end

    // ---------------- 发送 ----------------
    reg        tgo;
    reg [7:0]  tbyte;
    reg        tact;
    reg [15:0] tc;
    reg [3:0]  tbit;
    reg [9:0]  tsh;
    reg        txr;
    assign txd = txr;
    wire tdone = tact && tc == 0 && tbit == 10;
    always @(posedge clk) begin
        if (rst) begin
            tact <= 1'b0; txr <= 1'b1;
        end else if (!tact) begin
            txr <= 1'b1;
            if (tgo) begin tact <= 1'b1; tsh <= {1'b1, tbyte, 1'b0}; tbit <= 0; tc <= 0; end
        end else if (tc == 0) begin
            tc <= DIV - 1;
            txr <= tsh[0];
            tsh <= {1'b1, tsh[9:1]};
            if (tbit == 10) tact <= 1'b0;
            tbit <= tbit + 1'b1;
        end else tc <= tc - 1'b1;
    end

    // ---------------- 命令解析 ----------------
    localparam [2:0] P_CMD = 3'd0, P_ARG = 3'd1, P_BUS = 3'd2, P_RSP = 3'd3;
    reg [2:0]  ps;
    reg        is_w;
    reg [2:0]  n;          // 已收参数字节
    reg [47:0] args;
    reg [31:0] rsp;
    reg [2:0]  rn;         // 待发字节数
    reg [23:0] idle;

    always @(posedge clk) begin
        tgo <= 1'b0;
        if (rst) begin
            ps <= P_CMD; bus_req <= 1'b0; idle <= 0;
        end else begin
            if (rv) idle <= 0; else if (idle != TO) idle <= idle + 1'b1;
            case (ps)
            P_CMD: if (rv) begin
                if (rb == "W" || rb == "R") begin
                    is_w <= (rb == "W"); n <= 0; ps <= P_ARG;
                end
            end
            P_ARG: begin
                if (idle == TO) ps <= P_CMD;
                else if (rv) begin
                    args <= {args[39:0], rb};
                    n <= n + 1'b1;
                    if ((is_w && n == 3'd5) || (!is_w && n == 3'd1)) ps <= P_BUS;
                end
            end
            P_BUS: begin
                if (!bus_req) begin
                    bus_req <= 1'b1;
                    bus_we  <= is_w;
                    bus_addr <= is_w ? args[47:32] : args[15:0];
                    bus_wdata <= args[31:0];
                end else if (bus_ack) begin
                    bus_req <= 1'b0;
                    rsp <= bus_rdata;
                    tbyte <= is_w ? "K" : "D";
                    tgo <= 1'b1;
                    rn <= is_w ? 3'd0 : 3'd4;
                    ps <= P_RSP;
                end
            end
            P_RSP: if (!tact && !tgo) begin
                if (rn == 0) ps <= P_CMD;
                else begin
                    tbyte <= rsp[31:24]; rsp <= {rsp[23:0], 8'd0};
                    tgo <= 1'b1; rn <= rn - 1'b1;
                end
            end
            default: ps <= P_CMD;
            endcase
        end
    end
endmodule
