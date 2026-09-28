// 按 1K 深度分组的简单双口 RAM：用于深度不是 2 的幂的大存储（如帧缓冲 19200×16），
// 让综合器按 1K×18 等宽配置装箱，避免被整体推断为 16K×1 而浪费 BSRAM。
// 时序与 nav_ram_sdp 相同：同步读，延迟 1 拍；re=0 时输出保持。
// 不支持初始化文件（高云综合器不会为 generate 内的各组分别载入初始化文件）。
module nav_ram_banked #(
    parameter integer DW    = 16,
    parameter integer DEPTH = 19200,
    parameter integer BAW   = 10,          // 每组地址位宽
    parameter integer AW    = $clog2(DEPTH)
) (
    input  wire          clk,
    input  wire          we,
    input  wire [AW-1:0] waddr,
    input  wire [DW-1:0] wdata,
    input  wire          re,
    input  wire [AW-1:0] raddr,
    output wire [DW-1:0] rdata
);
    localparam integer NB = (DEPTH + (1 << BAW) - 1) >> BAW;
    localparam integer BW = (NB > 1) ? $clog2(NB) : 1;

    wire [DW-1:0] q [0:NB-1];
    reg  [BW-1:0] rbank;
    initial rbank = 0;
    always @(posedge clk) if (re) rbank <= raddr[AW-1:BAW];

    genvar b;
    generate
        for (b = 0; b < NB; b = b + 1) begin : g_bank
            reg [DW-1:0] mem [0:(1<<BAW)-1];
            reg [DW-1:0] qb;
            integer i;
            initial begin
                for (i = 0; i < (1 << BAW); i = i + 1) mem[i] = {DW{1'b0}};
                qb = {DW{1'b0}};
            end
            always @(posedge clk) if (we && waddr[AW-1:BAW] == b) mem[waddr[BAW-1:0]] <= wdata;
            always @(posedge clk) if (re && raddr[AW-1:BAW] == b) qb <= mem[raddr[BAW-1:0]];
            assign q[b] = qb;
        end
    endgenerate
    assign rdata = q[rbank];
endmodule
