// 软核 SoC：PicoRV32（RV32IMC，开源，rtl/soc/third_party/picorv32.v）+ 片上 RAM + 导航总线主口 + 控制台 + 调试口。
//
// 软核地址映射：
//   0x0000_0000  RAM（RAM_KB，代码 + 数据；上电内容来自 INIT_* 初始化文件，也可经调试口重新装载）
//   0x4000_0000  导航子系统寄存器（低 16 位为 nav_subsystem 偏移，按字访问）-> m_*（接 nav_bus_arb）
//   0x8000_0000  外设：+0 控制台写字节，+4 LED[3:0]，+8 状态字，+C RAM 字节数（只读）
// 调试口 d_*（板级顶层把串口桥 0xE000–0xFFFF 的访问转到这里，PC 用 tools/nav_uart.py）：
//   0xE000–0xEFFF  控制台环形缓冲（4 KB，按字读：字 i = 第 4i..4i+3 个字节）
//   0xF000 CTRL     [0] 软核保持复位（装载程序时置 1）
//   0xF004 RAM_ADDR 字地址（写 RAM_DATA 后自增）
//   0xF008 RAM_DATA 写：RAM[RAM_ADDR] = 数据；读：RAM[RAM_ADDR]（仅在保持复位时有效）
//   0xF00C CON_WPTR 控制台已写字节总数（环形下标 = 低 12 位）
//   0xF010 STATUS   软核写的状态字
//   0xF014 INFO     {RAM_KB[15:0], 版本[15:0]}
module nav_soc #(
    parameter integer RAM_KB = 160,
    parameter INIT_B0 = "",
    parameter INIT_B1 = "",
    parameter INIT_B2 = "",
    parameter INIT_B3 = ""
) (
    input  wire        clk,
    input  wire        rst,
    // 导航总线主口（req 保持到 ack）
    output reg         m_req,
    output reg         m_we,
    output reg  [15:0] m_addr,
    output reg  [31:0] m_wdata,
    input  wire        m_ack,
    input  wire [31:0] m_rdata,
    // 调试口（同一协议）
    input  wire        d_req,
    input  wire        d_we,
    input  wire [15:0] d_addr,
    input  wire [31:0] d_wdata,
    output reg         d_ack,
    output reg  [31:0] d_rdata,
    output reg  [3:0]  led,
    output wire        trap
);
    localparam integer WORDS = RAM_KB * 256;
    localparam integer AW = $clog2(WORDS);
    localparam [15:0] VERSION = 16'h0001;

    // ================================================================ 控制 / 调试寄存器
    reg          hold;                  // 软核保持复位
    reg [AW-1:0] dbg_ra;                // 调试口 RAM 字地址
    reg [31:0]   status;
    reg [31:0]   con_wp;
    wire         cpu_rst = rst | hold;

    // ================================================================ PicoRV32
    wire        mem_valid, mem_instr;
    wire [31:0] mem_addr, mem_wdata;
    wire [3:0]  mem_wstrb;
    reg         ram_ready, nav_ready, per_ready;
    reg  [31:0] nav_q, per_q;
    wire [31:0] ram_q;
    wire        mem_ready = ram_ready | nav_ready | per_ready;
    wire [31:0] mem_rdata = ram_ready ? ram_q : nav_ready ? nav_q : per_q;

    picorv32 #(
        .ENABLE_COUNTERS(1), .ENABLE_COUNTERS64(0), .ENABLE_REGS_16_31(1), .ENABLE_REGS_DUALPORT(1),
        .BARREL_SHIFTER(1), .TWO_CYCLE_COMPARE(0), .TWO_CYCLE_ALU(0), .COMPRESSED_ISA(1),
        .CATCH_MISALIGN(1), .CATCH_ILLINSN(1), .ENABLE_PCPI(0), .ENABLE_MUL(0), .ENABLE_FAST_MUL(1),
        .ENABLE_DIV(1), .ENABLE_IRQ(0), .ENABLE_TRACE(0),
        .PROGADDR_RESET(32'h0000_0000), .STACKADDR(RAM_KB * 1024)
    ) u_cpu (
        .clk(clk), .resetn(~cpu_rst), .trap(trap),
        .mem_valid(mem_valid), .mem_instr(mem_instr), .mem_ready(mem_ready), .mem_addr(mem_addr),
        .mem_wdata(mem_wdata), .mem_wstrb(mem_wstrb), .mem_rdata(mem_rdata),
        .mem_la_read(), .mem_la_write(), .mem_la_addr(), .mem_la_wdata(), .mem_la_wstrb(),
        .pcpi_valid(), .pcpi_insn(), .pcpi_rs1(), .pcpi_rs2(), .pcpi_wr(1'b0), .pcpi_rd(32'd0),
        .pcpi_wait(1'b0), .pcpi_ready(1'b0), .irq(32'd0), .eoi(), .trace_valid(), .trace_data()
    );

    wire sel_ram = (mem_addr[31:28] == 4'h0) && (mem_addr[27:2] < WORDS);
    wire sel_nav = (mem_addr[31:28] == 4'h4);
    wire sel_per = !sel_ram && !sel_nav;           // 外设与未映射地址（读 0、写忽略）
    wire req     = mem_valid && !mem_ready;

    // ================================================================ RAM（4 个字节通道）
    wire          dbg_ram_we;
    wire [31:0]   dbg_ram_wd = d_wdata;
    wire [AW-1:0] ram_a = hold ? dbg_ra : mem_addr[AW+1:2];
    genvar b;
    generate
        for (b = 0; b < 4; b = b + 1) begin : g_lane
            wire we = hold ? dbg_ram_we : (req && sel_ram && mem_wstrb[b]);
            wire [7:0] wd = hold ? dbg_ram_wd[8*b +: 8] : mem_wdata[8*b +: 8];
            nav_ram_sp #(.DW(8), .AW(AW), .DEPTH(WORDS),
                         .INIT_FILE(b == 0 ? INIT_B0 : b == 1 ? INIT_B1 : b == 2 ? INIT_B2 : INIT_B3)) u_ram (
                .clk(clk), .we(we), .addr(ram_a), .wdata(wd), .rdata(ram_q[8*b +: 8])
            );
        end
    endgenerate

    // ================================================================ 控制台环形缓冲（4 KB，4 个字节通道）
    wire        con_we = req && (mem_addr[31:28] == 4'h8) && (mem_addr[7:0] == 8'h00) && (mem_wstrb != 4'd0);
    wire [31:0] con_q;
    reg  [9:0]  con_ra;
    generate
        for (b = 0; b < 4; b = b + 1) begin : g_con
            nav_ram_sdp #(.DW(8), .AW(10)) u_con (
                .clk(clk), .we(con_we && con_wp[1:0] == b), .waddr(con_wp[11:2]), .wdata(mem_wdata[7:0]),
                .re(1'b1), .raddr(con_ra), .rdata(con_q[8*b +: 8])
            );
        end
    endgenerate

    // ================================================================ 访问应答
    reg nav_busy;
    always @(posedge clk) begin
        ram_ready <= 1'b0; nav_ready <= 1'b0; per_ready <= 1'b0;
        if (cpu_rst) begin
            m_req <= 1'b0; nav_busy <= 1'b0;
            if (rst) begin led <= 4'd0; status <= 0; con_wp <= 0; end
        end else begin
            if (req && sel_ram) ram_ready <= 1'b1;
            if (req && sel_per) begin
                per_ready <= 1'b1;
                case (mem_addr[7:0])
                    8'h04: per_q <= {28'd0, led};
                    8'h08: per_q <= status;
                    8'h0C: per_q <= RAM_KB * 1024;
                    default: per_q <= 32'd0;
                endcase
                if (mem_wstrb != 4'd0) begin
                    if (mem_addr[7:0] == 8'h00 && mem_addr[31:28] == 4'h8) con_wp <= con_wp + 1'b1;
                    if (mem_addr[7:0] == 8'h04 && mem_addr[31:28] == 4'h8) led <= mem_wdata[3:0];
                    if (mem_addr[7:0] == 8'h08 && mem_addr[31:28] == 4'h8) status <= mem_wdata;
                end
            end
            if (req && sel_nav && !nav_busy) begin
                nav_busy <= 1'b1; m_req <= 1'b1; m_we <= (mem_wstrb != 4'd0);
                m_addr <= mem_addr[15:0]; m_wdata <= mem_wdata;
            end
            if (m_req && m_ack) begin
                m_req <= 1'b0; nav_q <= m_rdata; nav_ready <= 1'b1;
            end
            if (nav_ready) nav_busy <= 1'b0;
        end
    end

    // ================================================================ 调试口
    reg [1:0] d_st;                           // 0 空闲 1、3 等 RAM/缓冲读出（地址寄存 + 同步读各一拍） 2 已应答（等 req 撤销）
    wire      d_ram = (d_addr == 16'hF008);
    wire      d_con = (d_addr[15:12] == 4'hE);
    assign dbg_ram_we = d_req && d_we && d_ram && (d_st == 2'd0) && hold;
    always @(posedge clk) begin
        d_ack <= 1'b0;
        if (rst) begin
            hold <= 1'b0; dbg_ra <= 0; d_st <= 2'd0;
        end else begin
            case (d_st)
            2'd0: if (d_req) begin
                if (d_we) begin
                    case (d_addr)
                        16'hF000: hold <= d_wdata[0];
                        16'hF004: dbg_ra <= d_wdata[AW-1:0];
                        16'hF008: if (hold) dbg_ra <= dbg_ra + 1'b1;
                        default: ;
                    endcase
                    d_ack <= 1'b1; d_st <= 2'd2;
                end else if (d_ram || d_con) begin
                    con_ra <= d_addr[11:2];
                    d_st <= 2'd1;                        // 地址寄存一拍 + 同步读一拍
                end else begin
                    case (d_addr)
                        16'hF000: d_rdata <= {31'd0, hold};
                        16'hF004: d_rdata <= dbg_ra;
                        16'hF00C: d_rdata <= con_wp;
                        16'hF010: d_rdata <= status;
                        16'hF014: d_rdata <= {RAM_KB[15:0], VERSION};
                        default:  d_rdata <= 32'hDEAD_BEEF;
                    endcase
                    d_ack <= 1'b1; d_st <= 2'd2;
                end
            end
            2'd1: d_st <= 2'd3;
            2'd3: begin
                d_rdata <= d_ram ? (hold ? ram_q : 32'd0) : con_q;
                d_ack <= 1'b1; d_st <= 2'd2;
            end
            2'd2: if (!d_req) d_st <= 2'd0;
            endcase
        end
    end
endmodule
