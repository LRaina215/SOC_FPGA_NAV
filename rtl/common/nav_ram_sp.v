// 单口 RAM，普通写模式（写周期读出保持不变）。读写同址的存储（软核 RAM）用它：
// 同址的简单双口写法会被综合成"先读后写"单口 RAM，Gowin 布局布线不支持（PA2122）。
module nav_ram_sp #(
    parameter integer DW = 8,
    parameter integer AW = 10,
    parameter integer DEPTH = (1 << AW),
    parameter INIT_FILE = ""
) (
    input  wire          clk,
    input  wire          we,
    input  wire [AW-1:0] addr,
    input  wire [DW-1:0] wdata,
    output reg  [DW-1:0] rdata
);
    reg [DW-1:0] mem [0:DEPTH-1];
    integer i;
    initial begin
        if (INIT_FILE != "") $readmemh(INIT_FILE, mem);
        else for (i = 0; i < DEPTH; i = i + 1) mem[i] = {DW{1'b0}};
        rdata = {DW{1'b0}};
    end
    always @(posedge clk) begin
        if (we) mem[addr] <= wdata;
        else rdata <= mem[addr];
    end
endmodule
