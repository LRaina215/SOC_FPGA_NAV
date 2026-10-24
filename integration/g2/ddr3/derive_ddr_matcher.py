"""Derive the complete original SLF arithmetic with a stallable DDR map port.

This generator checks exact source anchors. It does not modify the accepted
RTL, launch a joint build, or claim a DDR service bound. The caller must wire
pw_ready into MMIO backpressure before this can become a joint candidate.
"""
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def derive(original):
    s = original

    def one(old, new):
        nonlocal s
        assert s.count(old) == 1, old[:100]
        s = s.replace(old, new, 1)

    one('module slf_match #(', 'module slf_match_ddr #(')
    one('    localparam integer AW = 2 * LOG2N;',
        '    localparam integer AW = 2 * LOG2N;\n    wire step_ce;')
    one('    output reg         eval_done\n);', '''    output reg         eval_done,
    output wire pw_ready,
    input wire ui_clk,calib_done,output wire memory_fault,
    output wire[2:0]ddr_cmd,output wire ddr_cmd_en,output wire[27:0]ddr_addr,
    input wire ddr_cmd_ready,output wire[255:0]ddr_wr_data,
    output wire[31:0]ddr_wr_mask,output wire ddr_wr_en,ddr_wr_end,
    input wire ddr_wr_ready,input wire[255:0]ddr_rd_data,
    input wire ddr_rd_valid,ddr_rd_end
);''')
    a = s.index('    nav_ram_sdp #(.DW(18), .AW(AW)) u_sm (')
    b = s.index('    reg          lf_start;', a)
    s = s[:a] + '''    // One original RAM step completes only after its variable-latency
    // transaction. All dependent registers/RAM enables share step_ce.
    reg[1:0]memory_state;
    reg memory_req,memory_we;
    reg[15:0]memory_addr;
    reg[17:0]memory_data,memory_q,memory_return;
    reg memory_fast_pending;
    wire memory_ack,store_ready,store_busy,store_done,record_ready;
    wire memory_peek_hit;
    wire[17:0]memory_reply,memory_peek_q;
    wire fast_read=sm_re&&!sm_we&&memory_state==0&&memory_peek_hit;
    wire step_ce=!memory_fault&&(!(sm_re||sm_we)||fast_read||memory_state==3)&&
        (st!=S_IDLE||store_ready);
    assign sm_q=memory_fast_pending?memory_peek_q:memory_q;
    assign pw_ready=!pw_pend&&!memory_fault;
    always@(posedge clk)begin
        if(rst)begin memory_state<=0;memory_req<=0;memory_we<=0;
            memory_addr<=0;memory_data<=0;memory_q<=0;memory_return<=0;memory_fast_pending<=0;end
        else if(!memory_fault)begin
            if(memory_fast_pending)memory_q<=memory_peek_q;
            memory_fast_pending<=step_ce&&fast_read;
            case(memory_state)
            0:if(sm_we||(sm_re&&!memory_peek_hit))begin memory_req<=1;memory_we<=sm_we;
                memory_addr<=sm_we?sm_wa:sm_ra;memory_data<=sm_wd;memory_state<=1;end
            1:if(memory_ack)begin
                if(!memory_we)memory_return<=memory_reply;
                memory_req<=0;memory_state<=2;
            end
            2:if(!memory_ack)memory_state<=3;
            3:if(step_ce)begin
                // Emulate the original synchronous RAM edge: arithmetic at
                // this CE sees the previous Q, not the next neighbor early.
                if(!memory_we)memory_q<=memory_return;
                memory_state<=0;
            end
            endcase
        end
    end
    nav_ddr_structure_store_full u_store(clk,ui_clk,rst,calib_done,
        memory_req,memory_we,memory_addr,memory_data,memory_ack,memory_reply,
        cmd_update&&!cmd_eval&&!cmd_clear&&!cfg_prior&&st==S_IDLE&&
            !pw_pend&&store_ready&&step_ce,
        lf_cx,lf_cy,md==MD_UPD&&st==S_U0,u_slot,
        {u_tag,u_mx[3:1],u_my[3:1]},md==MD_UPD&&st==S_UC0,
        record_ready,store_ready,store_busy,store_done,memory_fault,
        ddr_cmd,ddr_cmd_en,ddr_addr,ddr_cmd_ready,ddr_wr_data,ddr_wr_mask,
        ddr_wr_en,ddr_wr_end,ddr_wr_ready,ddr_rd_data,ddr_rd_valid,ddr_rd_end,
        st==S_CLR&&clr_a==0&&store_ready,
        sm_re&&!sm_we&&memory_state==0,step_ce,sm_ra,memory_peek_hit,memory_peek_q);
''' + s[b:]
    a = s.index('    slf_lf #(.LOG2N(LOG2N)) u_lf (')
    b = s.index('    reg          ac_we, ac_wd, ac_re;', a)
    # The row engine owns the original touch bitmap/LF. No shadow map or old
    # touch/list RAM remains here; obsolete UPDATE states become unreachable.
    s = s[:a] + '''    assign lf_busy=1'b0;
    assign lf_done=1'b0;
    assign lf_re=1'b0;assign lf_we=1'b0;
    assign lf_ra=0;assign lf_wa=0;assign lf_wd=0;
    reg tb_we,tb_wd,tb_re,sl_we,sl_re;
    reg[AW-1:0]tb_wa,tb_ra,sl_wd;
    reg[9:0]sl_wa,sl_ra;
    wire tb_q=1'b0;wire[AW-1:0]sl_q=0;
''' + s[b:]
    # Declare the new port machinery after all original state/record symbols;
    # keep an explicit forward declaration for RAM enable expressions.
    a = s.index('    // One original RAM step completes')
    b = s.index('    reg          lf_start;', a)
    machinery = s[a:b].replace('    wire step_ce=', '    assign step_ce=')
    s = s[:a] + s[b:]
    one('    assign busy = (st != S_IDLE) || lf_busy || ex_busy;', machinery+
        '    assign busy = (st != S_IDLE) || ex_busy || store_busy || memory_state!=0 || pw_pend;')
    # Extraction writes, CPU candidate writes and CPU result reads are not
    # stalled by scoring. Only RAM operations belonging to the sequencer stop.
    for port in ('pb_re', 'ac_we', 'ac_re', 'cd_re', 'rs_we'):
        old = f'.{"we" if port.endswith("we") else "re"}({port})'
        one(old, old[:-1] + '&&step_ce)')
    assert s.count('.re(lut_re)') == 2
    s = s.replace('.re(lut_re)', '.re(lut_re&&step_ce)')
    a = s.index('    always @(posedge clk) begin\n        pb_re')
    b = s.index('        pb_re', a)
    s = s[:b] + '''        // Capture asynchronous-to-the-sequencer events even during a RAM
        // stall. The extractor and CPU register ports remain on the core clock.
        eval_done<=0;dbg_ack<=0;
        if(!rst)begin
            if(s_fire&&s_sof&&frame_en)begin base_x<=cfg_tx;base_y<=cfg_ty;end
            if(ex_done)pts_ready<=1;
            if(pw_we&&pw_ready)begin pw_pend<=1;pw_a<=pw_addr;pw_d<=pw_data;end
        end
        if(rst||step_ce)begin
''' + s[b:]
    one('''            if (s_fire && s_sof && frame_en) begin base_x <= cfg_tx; base_y <= cfg_ty; end
            if (ex_done) pts_ready <= 1'b1;
            if (pw_we) begin pw_pend <= 1'b1; pw_a <= pw_addr; pw_d <= pw_data; end
''', '')
    one("                    if (!pw_we) pw_pend <= 1'b0;", "                    pw_pend <= 1'b0;")
    one('''                    sm_re_m <= 1'b1; sm_ra_m <= {wya[LOG2N-1:0], wxa[LOG2N-1:0]};
                    tb_re <= 1'b1; tb_ra <= {wya[LOG2N-1:0], wxa[LOG2N-1:0]};
                    st <= S_U0;''', '                    st <= S_U0;')
    one('            S_U0: st <= S_U1;', '''            S_U0: if(record_ready)begin
                j<=jn;jj<=jj+1'b1;st<=S_T0;
            end''')
    a = s.index('            S_UC0: begin\n')
    b = s.index('            S_UC1:', a)
    s = s[:a] + '''            S_UC0: begin
                if(cli==2)begin lfo_x<=lf_cx-N/2;lfo_y<=lf_cy-N/2;st<=S_LFW;end
                else cli<=cli+1'b1;
            end
''' + s[b:]
    one('            S_LFW: if (lf_done) st <= S_IDLE;',
        '            S_LFW: if (store_done) st <= S_IDLE;')
    a = s.index('            S_CLR: begin\n')
    b = s.index('            // ---------------- 候选', a)
    s = s[:a] + '''            S_CLR: begin
                if(clr_a==0&&store_ready)clr_a<=1;
                if(clr_a!=0&&store_done)st<=S_IDLE;
            end
''' + s[b:]
    one('            endcase\n        end\n    end\nendmodule',
        '            endcase\n        end\n        end // step_ce\n    end\nendmodule')
    return s


def derive_store(original):
    """Extend the verified ownership handover to the same engine's clear path."""
    s=original
    changes=[('module nav_ddr_structure_store(', 'module nav_ddr_structure_store_full('),
        ('input wire wr_ready,input wire[255:0]rd_data,input wire rd_valid,rd_end',
         'input wire wr_ready,input wire[255:0]rd_data,input wire rd_valid,rd_end,\n    input wire clear_start,peek_req,peek_go,input wire[15:0]peek_addr,\n    output wire peek_hit,output wire[17:0]peek_q'),
        ('reg signed[31:0]held_cx,held_cy;', 'reg signed[31:0]held_cx,held_cy;reg held_clear;'),
        ('nav_slf_row_update u_update', 'nav_slf_row_update_full u_update'),
        ('nav_ddr_struct_cache u_cache', 'nav_ddr_struct_cache_fast u_cache'),
        ('barrier_req,barrier_done,line_req,line_we,line_addr,line_wdata,line_ack,line_rdata);',
         'barrier_req,barrier_done,line_req,line_we,line_addr,line_wdata,line_ack,line_rdata,\n        peek_req,peek_go,peek_addr,peek_hit,peek_q);'),
        ('record_ready,row_ready,row_busy,row_done,rf,', 'held_clear,record_ready,row_ready,row_busy,row_done,rf,'),
        ('owner_request<=0;end', 'owner_request<=0;held_clear<=0;end'),
        ('CLIENT:if(update_start&&ready)begin', 'CLIENT:if((update_start||clear_start)&&ready)begin\n                held_clear<=clear_start;')]
    for a,b in changes:
        assert s.count(a)==1,a
        s=s.replace(a,b)
    return s


def derive_fast_cache(original):
    """An idle read-only hit uses the existing cache BRAM each core cycle.

    Misses use the verified request/ACK path; writes/flush/UPDATE exclude this
    port. Metadata is read combinationally, data keeps synchronous latency.
    There is no duplicated map or additional data RAM.
    """
    s=original
    changes=[('module nav_ddr_struct_cache(', 'module nav_ddr_struct_cache_fast('),
        ('input wire[255:0]line_rdata\n', 'input wire[255:0]line_rdata,\n    input wire peek_req,peek_go,input wire[15:0]peek_addr,\n    output wire peek_hit,output wire[17:0]peek_q\n'),
        ('wire ram_re=(state==LOOKUP&&hit&&!saved_we)||state==COLLECT;',
         'wire ram_re=(state==LOOKUP&&hit&&!saved_we)||state==COLLECT||peek_fire;'),
        ('wire[11:0]ram_addr={active_index,(state==FILL||state==COLLECT)?slot:request_slot};',
         'wire[11:0]ram_addr=peek_fire?{peek_line[7:0],peek_slot}:\n        {active_index,(state==FILL||state==COLLECT)?slot:request_slot};'),
        ('    wire hit=mq[6]&&mq[4:0]==request_line[12:8];', '''    wire hit=mq[6]&&mq[4:0]==request_line[12:8];
    wire[12:0]peek_line=peek_addr[15:8]*19+peek_addr[7:0]/14;
    wire[3:0]peek_slot=peek_addr[7:0]%14;
    wire[6:0]peek_meta=meta[peek_line[7:0]];
    assign peek_hit=peek_req&&state==IDLE&&!barrier_req&&!req&&
        peek_meta[6]&&peek_meta[4:0]==peek_line[12:8];
    wire peek_fire=peek_hit&&peek_go;'''),
        ('    wire meta_we=', '    assign peek_q=bank_q[read_bank];\n    wire meta_we=')]
    for a,b in changes:
        assert s.count(a)==1,a
        s=s.replace(a,b)
    return s


def derive_row_engine(original):
    """Use the already allocated output row for serialized bulk-zero writes.

    Clear uses no additional RAM, waits for native write admission and then
    an ordered read fence. Its runtime is measured, not inferred from BL8.
    """
    s=original
    def one(a,b):
        nonlocal s
        assert s.count(a)==1,a
        s=s.replace(a,b)
    one('module nav_slf_row_update (', 'module nav_slf_row_update_full (')
    one('    output wire record_ready,output wire ready,',
        '    input wire clear_only,\n    output wire record_ready,output wire ready,')
    one('localparam COLD=0,IDLE=1,HEADS=2,COLLECT=3,RUN=4,CLEAR=5;',
        'localparam COLD=0,IDLE=1,HEADS=2,COLLECT=3,RUN=4,CLEAR=5,ZERO=6,ZERO_FENCE=7;\n    reg[7:0]zero_row,zero_col;reg[1:0]zero_state;')
    one('dma_ack,fault,ire,iwe,ia,id,iq,lw&&ce,lwa[8],lwa[7:0],lwd,', '''dma_ack,fault,ire,iwe,ia,id,iq,
        (phase==ZERO&&zero_state==0)||(lw&&ce),
        phase==ZERO?1'b0:lwa[8],phase==ZERO?zero_col:lwa[7:0],
        phase==ZERO?18'd0:lwd,''')
    one('lf_started<=0;lf_finished<=0;fenced<=0;rec_state<=0;',
        'lf_started<=0;lf_finished<=0;fenced<=0;rec_state<=0;zero_row<=0;zero_col<=0;zero_state<=0;')
    one('IDLE:if(start&&!fault)begin phase<=HEADS;busy<=1;head_index<=0;',
        'IDLE:if(start&&!fault)begin phase<=clear_only?ZERO:HEADS;busy<=1;head_index<=0;\n                zero_row<=0;zero_col<=0;zero_state<=0;')
    one('            CLEAR:case(rec_state)', '''            ZERO:case(zero_state)
                0:if(zero_col==255)zero_state<=1;else zero_col<=zero_col+1'b1;
                1:if(!dma_req&&!dma_ack)begin dma_req<=1;dma_we<=1;
                    dma_bank<=0;dma_row<=zero_row;zero_state<=2;end
                2:if(dma_ack)begin dma_req<=0;zero_state<=3;end
                3:if(!dma_ack)begin
                    if(zero_row==255)begin phase<=ZERO_FENCE;zero_state<=0;end
                    else begin zero_row<=zero_row+1'b1;zero_col<=0;zero_state<=0;end
                end
            endcase
            ZERO_FENCE:case(zero_state)
                0:if(!dma_req&&!dma_ack)begin dma_req<=1;dma_we<=0;
                    dma_bank<=0;dma_row<=255;zero_state<=1;end
                1:if(dma_ack)begin dma_req<=0;zero_state<=2;end
                2:if(!dma_ack)begin phase<=IDLE;busy<=0;done<=1;end
            endcase
            CLEAR:case(rec_state)''')
    return s
