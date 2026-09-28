// 两主机（软核、调试串口桥）共用导航总线的仲裁：先到先得，同时请求时软核优先。
// 协议同 nav_subsystem：req 保持到 ack。
module nav_bus_arb (
    input  wire        clk,
    input  wire        rst,
    // 主机 0（软核）
    input  wire        m0_req, input wire m0_we, input wire [15:0] m0_addr, input wire [31:0] m0_wdata,
    output wire        m0_ack, output wire [31:0] m0_rdata,
    // 主机 1（串口桥）
    input  wire        m1_req, input wire m1_we, input wire [15:0] m1_addr, input wire [31:0] m1_wdata,
    output wire        m1_ack, output wire [31:0] m1_rdata,
    // 从机
    output wire        s_req, output wire s_we, output wire [15:0] s_addr, output wire [31:0] s_wdata,
    input  wire        s_ack, input wire [31:0] s_rdata
);
    reg busy, sel, hold;       // hold：ack 后一拍，等主机撤销 req
    always @(posedge clk) begin
        if (rst) begin
            busy <= 1'b0; sel <= 1'b0; hold <= 1'b0;
        end else begin
            hold <= 1'b0;
            if (!busy && !hold) begin
                if (m0_req) begin busy <= 1'b1; sel <= 1'b0; end
                else if (m1_req) begin busy <= 1'b1; sel <= 1'b1; end
            end else if (busy && s_ack) begin
                busy <= 1'b0; hold <= 1'b1;
            end
        end
    end
    assign s_req   = busy & (sel ? m1_req : m0_req);
    assign s_we    = sel ? m1_we : m0_we;
    assign s_addr  = sel ? m1_addr : m0_addr;
    assign s_wdata = sel ? m1_wdata : m0_wdata;
    assign m0_ack  = busy & ~sel & s_ack;
    assign m1_ack  = busy &  sel & s_ack;
    assign m0_rdata = s_rdata;
    assign m1_rdata = s_rdata;
endmodule
