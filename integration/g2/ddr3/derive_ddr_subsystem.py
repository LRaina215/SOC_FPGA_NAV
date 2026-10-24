"""Derive explicit DDR SLF wiring and prior-write MMIO backpressure.

No register address, payload, control algorithm or firmware change. The master
keeps req/address/data until ACK, as required by the original subsystem bus.
"""


def derive(original):
    s=original
    def one(a,b):
        nonlocal s
        assert s.count(a)==1,a[:100]
        s=s.replace(a,b)
    one('module nav_subsystem #(', 'module nav_subsystem_ddr #(')
    one('    input  wire [3:0]  ext_stop         // 外部急停源（ToF 近距、急停按钮等）', '''    input  wire [3:0] ext_stop,
    input wire ddr_ui_clk,ddr_calib_done,output wire ddr_memory_fault,
    output wire[2:0]ddr_cmd,output wire ddr_cmd_en,output wire[27:0]ddr_addr,
    input wire ddr_cmd_ready,output wire[255:0]ddr_wr_data,
    output wire[31:0]ddr_wr_mask,output wire ddr_wr_en,ddr_wr_end,
    input wire ddr_wr_ready,input wire[255:0]ddr_rd_data,
    input wire ddr_rd_valid,ddr_rd_end''')
    one('    wire        mt_busy, mt_ready, mt_eval_done, mt_dbg_ack;',
        '    wire mt_pw_ready;\n    wire        mt_busy, mt_ready, mt_eval_done, mt_dbg_ack;')
    one('    slf_match #(.SIN_INIT(SIN_INIT)) u_match (',
        '    slf_match_ddr #(.SIN_INIT(SIN_INIT)) u_match (')
    one('        .busy(mt_busy), .pts_ready(mt_ready), .n_pts(mt_npts), .eval_done(mt_eval_done)', '''        .busy(mt_busy), .pts_ready(mt_ready), .n_pts(mt_npts), .eval_done(mt_eval_done),
        .pw_ready(mt_pw_ready),.ui_clk(ddr_ui_clk),.calib_done(ddr_calib_done),
        .memory_fault(ddr_memory_fault),.ddr_cmd(ddr_cmd),.ddr_cmd_en(ddr_cmd_en),
        .ddr_addr(ddr_addr),.ddr_cmd_ready(ddr_cmd_ready),.ddr_wr_data(ddr_wr_data),
        .ddr_wr_mask(ddr_wr_mask),.ddr_wr_en(ddr_wr_en),.ddr_wr_end(ddr_wr_end),
        .ddr_wr_ready(ddr_wr_ready),.ddr_rd_data(ddr_rd_data),
        .ddr_rd_valid(ddr_rd_valid),.ddr_rd_end(ddr_rd_end)''')
    one("B_MRR = 4'd7, B_MRR2 = 4'd8, B_LRR = 4'd9, B_LRR2 = 4'd10;",
        "B_MRR = 4'd7, B_MRR2 = 4'd8, B_LRR = 4'd9, B_LRR2 = 4'd10, B_MPW = 4'd11;")
    one('''                end else begin
                    if (bus_we) begin
                        rf_wr <= 1'b1;''', '''                end else if(bus_we&&{bus_addr[15:2],2'b00}==RA_MATCH_SM_WDATA)begin
                    // Address/data are held in a_q/wd_q. Do not generate the
                    // single RF pulse or CPU ACK until the pending slot is free.
                    bs<=B_MPW;
                end else begin
                    if (bus_we) begin
                        rf_wr <= 1'b1;''')
    one('            B_RF: begin', '''            B_MPW:if(mt_pw_ready)begin rf_wr<=1'b1;bs<=B_RF;end
            B_RF: begin''')
    return s


def mmio_shell(original):
    """Exercise the EXACT derived bus FSM with the actual generated regfile.

    Non-map windows have quiescent endpoints; this is a bus/SLF proof, not a
    simulation of Ethernet, terrain fusion, safety or the complete subsystem.
    """
    s=derive(original)
    marker='    // ================================================================ 总线状态机\n'
    assert s.count(marker)==1
    fsm=s[s.index(marker):]
    return '''module nav_ddr_mmio_shell(
    input wire clk,rst,bus_req,bus_we,input wire[15:0]bus_addr,
    input wire[31:0]bus_wdata,output reg bus_ack,output reg[31:0]bus_rdata,
    input wire mt_pw_ready,output wire prior_valid,
    output wire[15:0]prior_addr,output wire[17:0]prior_data
);
`include "nav_regs_map.vh"
    reg[15:0]a_q;reg[31:0]wd_q;
    reg rf_wr,rf_rd,tr_res_re,tr_cand_we,mb_rd,mt_res_re,mt_cand_we,lr_tx_we,pose_predecoded_q;
    reg[7:0]tr_res_addr;reg[6:0]tr_cand_addr;
    reg[8:0]mt_res_addr,mt_cand_addr;
    reg[4:0]lr_tx_wa,lr_rx_ra;
    reg cm_b_req;reg[13:0]cm_b_addr;
    wire cm_b_ack=1'b1,mb_rd_ack=1'b1;
    wire[7:0]cm_b_q=0;wire[31:0]tr_res_q=0,mt_res_q=0,mb_word=0,lr_rx_q=0;
    wire[31:0]rf_rdata,prior_addr_reg;wire rf_hit;
    nav_regfile u_rf(.clk(clk),.rst(rst),.wr(rf_wr),.rd(rf_rd),.addr(a_q),
        .wdata(wd_q),.rdata(rf_rdata),.hit(rf_hit),
        .r_MATCH_SM_ADDR(prior_addr_reg),.p_MATCH_SM_WDATA(prior_valid));
    assign prior_addr=prior_addr_reg[15:0];assign prior_data=wd_q[17:0];
''' + fsm
