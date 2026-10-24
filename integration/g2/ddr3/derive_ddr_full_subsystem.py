"""Compose the verified DDR map adapters with the entire frozen NAV top.

No CPU/firmware/control changes. ddr_cold_rst resets the physical controller
and memory protocol together and is distinct from PicoRV32 debug HOLD.
The ordinary NAV reset must be asserted with that cold reset when restarting
the memory protocol. A CPU-only HOLD leaves this whole subsystem running.
"""
from derive_ddr_subsystem import derive as derive_bus


def elevation(source):
    old='    input wire memory_fault\n);'
    new='    input wire memory_fault,output wire memory_initialized\n);'
    assert source.count(old)==1
    source=source.replace(old,new)
    old='    assign busy = clr_act | v1 | v2 | fw1_v | mem_req | !step_ce | clear_pending;'
    assert source.count(old)==1
    return source.replace(old,old+'\n    assign memory_initialized=!clr_act&&!clear_pending&&!memory_fault;')


def map_top(source):
    old='    input wire memory_fault\n);'
    new='    input wire memory_fault,output wire memory_initialized\n);'
    assert source.count(old)==1
    source=source.replace(old,new)
    old='.mem_ack(mem_ack),.mem_reply(mem_reply),.memory_fault(memory_fault)'
    assert source.count(old)==1
    return source.replace(old,old+',.memory_initialized(memory_initialized)')


def matcher(source):
    old='    input wire em_flush,output wire em_flushed,output wire sm_initialized\n);'
    new='''    input wire em_flush,output wire em_flushed,output wire sm_initialized,
    input wire ddr_cold_rst,external_fault,output wire store_busy
);'''
    assert source.count(old)==1
    source=source.replace(old,new)
    source=source.replace('module slf_match_ddr #(','module slf_match_ddr_full #(',1)
    old='    wire memory_ack,store_ready,store_busy,store_done,record_ready;'
    assert source.count(old)==1
    source=source.replace(old,'    wire memory_ack,store_ready,store_done,record_ready,raw_memory_fault;\n    assign memory_fault=raw_memory_fault|external_fault;')
    old='nav_ddr_structure_store_full u_store(clk,ui_clk,rst,calib_done,'
    assert source.count(old)==1
    source=source.replace(old,'nav_ddr_structure_store_full u_store(clk,ui_clk,ddr_cold_rst,calib_done,')
    old='record_ready,store_ready,store_busy,store_done,memory_fault,'
    assert source.count(old)==1
    return source.replace(old,'record_ready,store_ready,store_busy,store_done,raw_memory_fault,')


def subsystem(original):
    source=derive_bus(original)
    def one(a,b):
        nonlocal source
        assert source.count(a)==1,a[:100]
        source=source.replace(a,b)
    one('module nav_subsystem_ddr #(','module nav_subsystem_ddr_full #(')
    one('input wire ddr_ui_clk,ddr_calib_done,output wire ddr_memory_fault,',
        '''input wire ddr_ui_clk,ddr_calib_done,ddr_cold_rst,ddr_pll_locked,
    output wire ddr_memory_fault,ddr_initialized,''')
    one('    wire        el_ack;', '''    wire em_mem_req,em_mem_we,em_mem_ack,em_initialized;
    wire[13:0]em_mem_addr;wire[52:0]em_mem_data,em_mem_reply;
    wire sm_initialized,ddr_store_busy,ddr_stop,raw_memory_fault;
    wire[2:0]raw_cmd;wire raw_cmd_en,raw_wen,raw_wend;
    (* syn_keep=1 *)reg stop_ui1,stop_ui2;
    wire ui_cold;
    nav_ddr_reset u_stop_ui_reset(ddr_ui_clk,ddr_cold_rst,ui_cold);
    always@(posedge ddr_ui_clk or posedge ui_cold)
        if(ui_cold)begin stop_ui1<=0;stop_ui2<=0;end
        else begin stop_ui1<=ddr_memory_fault;stop_ui2<=stop_ui1;end
    assign ddr_cmd=raw_cmd;
    assign ddr_cmd_en=raw_cmd_en&&!stop_ui2;
    assign ddr_wr_en=raw_wen&&!stop_ui2;
    assign ddr_wr_end=raw_wend&&!stop_ui2;
    nav_ddr_admission u_ddr_guard(clk,ddr_ui_clk,ddr_cold_rst,ddr_pll_locked,ddr_calib_done,
        sm_initialized&&em_initialized,raw_memory_fault,ddr_initialized,ddr_stop,ddr_memory_fault);
    wire        el_ack;''')
    one('    map_subsys u_map (','    map_subsys_ddr u_map (')
    one('        .frame_cnt(map_fc), .map_ready(map_ready)', '''        .frame_cnt(map_fc), .map_ready(map_ready),
        .mem_req(em_mem_req),.mem_we(em_mem_we),.mem_addr(em_mem_addr),.mem_wdata(em_mem_data),
        .mem_ack(em_mem_ack),.mem_reply(em_mem_reply),.memory_fault(ddr_memory_fault),
        .memory_initialized(em_initialized)''')
    one('map_front, map_busy|cov_busy};','map_front, map_busy|cov_busy|ddr_store_busy|!ddr_initialized};')
    one('    slf_match_ddr #(.SIN_INIT(SIN_INIT)) u_match (',
        '    slf_match_ddr_full #(.SIN_INIT(SIN_INIT)) u_match (')
    one('.memory_fault(ddr_memory_fault),.ddr_cmd(ddr_cmd),.ddr_cmd_en(ddr_cmd_en),',
        '.memory_fault(raw_memory_fault),.ddr_cmd(raw_cmd),.ddr_cmd_en(raw_cmd_en),')
    one('.ddr_addr(ddr_addr),.ddr_cmd_ready(ddr_cmd_ready),.ddr_wr_data(ddr_wr_data),',
        '.ddr_addr(ddr_addr),.ddr_cmd_ready(ddr_cmd_ready&&!stop_ui2),.ddr_wr_data(ddr_wr_data),')
    one('.ddr_wr_mask(ddr_wr_mask),.ddr_wr_en(ddr_wr_en),.ddr_wr_end(ddr_wr_end),',
        '.ddr_wr_mask(ddr_wr_mask),.ddr_wr_en(raw_wen),.ddr_wr_end(raw_wend),')
    one('.ddr_wr_ready(ddr_wr_ready),.ddr_rd_data(ddr_rd_data),',
        '.ddr_wr_ready(ddr_wr_ready&&!stop_ui2),.ddr_rd_data(ddr_rd_data),')
    one('.ddr_rd_valid(ddr_rd_valid),.ddr_rd_end(ddr_rd_end)', '''.ddr_rd_valid(ddr_rd_valid&&!stop_ui2),.ddr_rd_end(ddr_rd_end),
        .em_req(em_mem_req),.em_we(em_mem_we),.em_addr(em_mem_addr),.em_wdata(em_mem_data),
        .em_ack(em_mem_ack),.em_reply(em_mem_reply),.em_flush(1'b0),.em_flushed(),
        .sm_initialized(sm_initialized),.store_busy(ddr_store_busy),
        .ddr_cold_rst(ddr_cold_rst),.external_fault(ddr_memory_fault)''')
    one('.cfg_estop(r_SAFE_CTRL[2]),','.cfg_estop(r_SAFE_CTRL[2]|ddr_stop),')
    # Serialize only mode/command writes which would otherwise change an
    # in-flight matcher's interpretation. No timestamp/firmware limit changes.
    # The CPU holds its bus transaction until ACK, per the original bus ABI.
    one("B_LRR2 = 4'd10, B_MPW = 4'd11;", "B_LRR2 = 4'd10, B_MPW = 4'd11, B_MODE = 4'd12;")
    one('''                end else if(bus_we&&{bus_addr[15:2],2'b00}==RA_MATCH_SM_WDATA)begin''',
        '''                end else if(bus_we&&({bus_addr[15:2],2'b00}==RA_MATCH_CTRL||
                    {bus_addr[15:2],2'b00}==RA_MATCH_CMD))begin
                    bs<=B_MODE;
                end else if(bus_we&&{bus_addr[15:2],2'b00}==RA_MATCH_SM_WDATA)begin''')
    one("            B_MPW:if(mt_pw_ready)begin rf_wr<=1'b1;bs<=B_RF;end", '''            B_MODE:if(!mt_busy&&ddr_initialized&&!ddr_memory_fault)begin rf_wr<=1'b1;bs<=B_RF;end
            B_MPW:if(mt_pw_ready)begin rf_wr<=1'b1;bs<=B_RF;end''')
    return source
