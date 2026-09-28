// 简单双口 RAM：一写一读，同步读（读延迟 1 拍），读写同址时读出旧值（read-first）。
// 综合器推断为 BSRAM。INIT_FILE 非空时用 $readmemh 初始化（仿真与 Gowin 综合均支持）。
module nav_ram_sdp #(
    parameter integer DW = 16,
    parameter integer AW = 10,
    parameter integer DEPTH = (1 << AW),
    parameter INIT_FILE = "",
    parameter [DW-1:0] INIT_VAL = {DW{1'b0}}
) (
    input  wire          clk,
    input  wire          we,
    input  wire [AW-1:0] waddr,
    input  wire [DW-1:0] wdata,
    input  wire          re,
    input  wire [AW-1:0] raddr,
    output reg  [DW-1:0] rdata
);
    reg [DW-1:0] mem [0:DEPTH-1];

    integer i;
    initial begin
        if (INIT_FILE != "") begin
            $readmemh(INIT_FILE, mem);
        end else begin
            for (i = 0; i < DEPTH; i = i + 1) mem[i] = INIT_VAL;
        end
        rdata = {DW{1'b0}};
    end

    always @(posedge clk) begin
        if (we) mem[waddr] <= wdata;
    end

    always @(posedge clk) begin
        if (re) rdata <= mem[raddr];
    end
endmodule
