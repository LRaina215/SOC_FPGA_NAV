"""One native controller for both full-capacity maps and row UPDATE.

Only complete cache-line transactions arbitrate. UPDATE drains the entire
active elevation cache operation, not just its current writeback BL8: an
eviction may still need a read fill after its write command has completed.
The already verified structure barrier and UI ownership handshake remain.
"""


def derive_cache(source):
    source = source.split('// CPU remains on chip.')[0]
    for old, new in [
        ('module nav_ddr_map_cache #(', 'module nav_ddr_em_cache #('),
        ('input wire [255:0] line_rdata\n',
         'input wire [255:0] line_rdata,\n    output wire quiescent\n'),
        ('    assign line_req=',
         '    assign quiescent=state==IDLE&&!ack&&!flush_req;\n    assign line_req=')]:
        assert source.count(old) == 1, old
        source = source.replace(old, new, 1)
    return source


def derive_store(source):
    """Input is the complete-SLF store produced by derive_ddr_matcher."""
    def one(old, new):
        nonlocal source
        assert source.count(old) == 1, old
        source = source.replace(old, new, 1)
    one('output wire peek_hit,output wire[17:0]peek_q', '''output wire peek_hit,output wire[17:0]peek_q,
    input wire em_req,em_we,input wire[13:0]em_addr,input wire[52:0]em_wdata,
    output wire em_ack,output wire[52:0]em_reply,
    input wire em_flush,output wire em_flushed,
    output reg sm_initialized''')
    one('    wire core_reset,ui_reset;\n', '')
    one('    wire ca;wire cf,rf,row_ready,row_busy,row_done;', '''    wire ca;wire cf,rf,row_ready,row_busy,row_done;
    wire core_reset,ui_reset;
    wire er,ew,ea,em_quiet;wire[24:0]eaddr;wire[255:0]ed;
    wire native_ack;reg[1:0]arb_state;reg line_owner,last_owner;
    wire client_period=state==CLIENT||state==DRAIN;
    wire native_req=client_period&&arb_state==1&&(line_owner?er:line_req);
    assign line_ack=arb_state==1&&!line_owner&&native_ack;
    assign ea=arb_state==1&&line_owner&&native_ack;
    nav_ddr_em_cache #(.DW(53),.AW(14),.WORD_BITS(2),.INDEX_BITS(8),.BASE_LINE(8192)) u_em
        (clk,rst,em_req&&state==CLIENT&&row_ready&&sm_initialized&&!fault,em_we,em_addr,em_wdata,
        em_ack,em_reply,em_flush&&state==CLIENT,,em_flushed,
        er,ew,eaddr,ed,ea,line_rdata,em_quiet);
    always@(posedge clk)begin
        if(core_reset)begin arb_state<=0;line_owner<=0;last_owner<=1;end
        else case(arb_state)
            0:if(client_period&&(line_req||er))begin
                line_owner<=line_req&&er?!last_owner:er;arb_state<=1;
            end
            1:if(native_ack)begin last_owner<=line_owner;arb_state<=2;end
            2:if(!native_ack)arb_state<=0;
            default:arb_state<=0;
        endcase
    end''')
    one('nav_ddr_native u_native(clk,ui_clk,rst,line_req,line_we,line_addr,line_wdata,32\'d0,\n        line_ack,line_rdata,',
        '''nav_ddr_native u_native(clk,ui_clk,rst,native_req,line_owner?ew:line_we,
        line_owner?eaddr:line_addr,line_owner?ed:line_wdata,32'd0,
        native_ack,line_rdata,''')
    one('DRAIN:if(barrier_done)begin',
        'DRAIN:if(barrier_done&&em_quiet&&arb_state==0&&!native_ack)begin')
    one('        peek_req,peek_go,peek_addr,peek_hit,peek_q);',
        '        peek_req&&state==CLIENT&&sm_initialized,peek_go&&state==CLIENT&&sm_initialized,peek_addr,peek_hit,peek_q);')
    one('assign ready=state==CLIENT&&row_ready&&!fault;',
        'assign ready=state==CLIENT&&row_ready&&sm_initialized&&!fault;')
    one('assign busy=state!=CLIENT||!row_ready;',
        'assign busy=state!=CLIENT||!row_ready||!sm_initialized;')
    one('req&&state==CLIENT&&row_ready&&!fault,we,word_addr',
        'req&&state==CLIENT&&row_ready&&sm_initialized&&!fault,we,word_addr')
    one('owner_request<=0;held_clear<=0;end',
        'owner_request<=0;held_clear<=0;sm_initialized<=0;end')
    one('CLIENT:if((update_start||clear_start)&&ready)begin', '''CLIENT:if(!sm_initialized&&row_ready&&!fault)begin
                // Backing DDR has no FPGA INIT contents. Reuse bulk clear
                // and its ordered read fence before accepting either map.
                held_clear<=1;held_cx<=0;held_cy<=0;barrier_req<=1;state<=DRAIN;
            end else if((update_start||clear_start)&&ready)begin''')
    one('SWITCH_CLIENT:if(!own_ack2)begin state<=CLIENT;done<=1;end',
        '''SWITCH_CLIENT:if(!own_ack2)begin state<=CLIENT;
                if(!sm_initialized)sm_initialized<=1;else done<=1;
            end''')
    return source


def derive_matcher(source):
    a='    input wire ddr_rd_valid,ddr_rd_end\n);'
    b='''    input wire ddr_rd_valid,ddr_rd_end,
    input wire em_req,em_we,input wire[13:0]em_addr,input wire[52:0]em_wdata,
    output wire em_ack,output wire[52:0]em_reply,
    input wire em_flush,output wire em_flushed,output wire sm_initialized
);'''
    assert source.count(a)==1
    source=source.replace(a,b)
    a='sm_ra,memory_peek_hit,memory_peek_q);'
    b='''sm_ra,memory_peek_hit,memory_peek_q,
        em_req,em_we,em_addr,em_wdata,em_ack,em_reply,em_flush,em_flushed,sm_initialized);'''
    assert source.count(a)==1
    return source.replace(a,b)
