// Isolated map-offload feasibility prototype. Not connected to G2.
// Full-word write-back cache. All external access must use this cache; a
// completed flush makes backing DDR coherent. Backing initialization and
// algorithm ready/valid adapters are separate admission requirements.
module nav_ddr_map_cache #(
    parameter DW=18, AW=16, WORD_BITS=3, INDEX_BITS=7,
    parameter [24:0] BASE_LINE=0
)(
    input wire clk, rst, req, we,
    input wire [AW-1:0] word_addr,
    input wire [DW-1:0] wdata,
    output reg ack,
    output reg [DW-1:0] rdata,
    input wire flush_req,
    output reg flushing, flush_done,
    output wire line_req, line_we,
    output wire [24:0] line_addr,
    output wire [255:0] line_wdata,
    input wire line_ack,
    input wire [255:0] line_rdata
);
    localparam WORDS=1<<WORD_BITS, LINES=1<<INDEX_BITS;
    localparam TAG_BITS=AW-WORD_BITS-INDEX_BITS;
    localparam PLANES=(DW+17)/18, RAM_AW=WORD_BITS+INDEX_BITS;
    localparam DDR_STRIDE=256/WORDS;
    localparam IDLE=0, LOOKUP=1, READ_RESULT=2, RESPONSE=3,
        COLLECT_REQUEST=4, COLLECT_CAPTURE=5, WRITE_LINE=6,
        WRITE_GAP=7, READ_LINE=8, READ_GAP=9, FILL=10,
        FLUSH_SCAN=11, FLUSH_ADVANCE=12, FLUSH_RESPONSE=13, INIT_META=14;
    reg [3:0] state;
    reg [AW-1:0] saved_addr;
    reg [DW-1:0] saved_data;
    reg saved_we;
    // Sequential cold invalidation avoids a resettable 512-way tag/dirty
    // register decoder. No request is accepted until every entry is cleared.
    (* syn_ramstyle="distributed_ram" *) reg [TAG_BITS+1:0] meta[0:LINES-1];
    reg [INDEX_BITS-1:0] scan_index;
    reg [WORD_BITS-1:0] slot;
    reg [24:0] evict_addr;
    reg [255:0] transfer;
    wire [INDEX_BITS-1:0] request_index=saved_addr[WORD_BITS+:INDEX_BITS];
    wire [TAG_BITS-1:0] request_tag=saved_addr[AW-1:WORD_BITS+INDEX_BITS];
    wire [INDEX_BITS-1:0] active_index=flushing?scan_index:request_index;
    wire [TAG_BITS+1:0] meta_q=meta[active_index];
    wire hit=meta_q[TAG_BITS+1] && meta_q[TAG_BITS-1:0]==request_tag;
    wire hit_write=(state==LOOKUP) && hit && saved_we;
    wire ram_we=hit_write || state==FILL;
    wire ram_re=(state==LOOKUP && hit && !saved_we) || state==COLLECT_REQUEST;
    wire [RAM_AW-1:0] ram_addr=(state==FILL || state==COLLECT_REQUEST)?
        {active_index,slot}:saved_addr[RAM_AW-1:0];
    wire [PLANES*18-1:0] expanded_data=hit_write?saved_data:
        (transfer >> (slot*DDR_STRIDE));
    wire [PLANES*18-1:0] ram_q;
    genvar p,b;
    generate if(RAM_AW<=10)begin:g_small
        for(p=0;p<PLANES;p=p+1)begin:g_plane
            (* syn_ramstyle="block_ram" *) reg [17:0] data[0:(1<<RAM_AW)-1];
            reg [17:0] q;
            always @(posedge clk)begin
                if(ram_we)data[ram_addr]<=expanded_data[p*18+:18];
                if(ram_re)q<=data[ram_addr];
            end
            assign ram_q[p*18+:18]=q;
        end
    end else begin:g_row_interleaved
        localparam BANK_BITS=RAM_AW-10, BANKS=1<<BANK_BITS;
        wire [BANK_BITS-1:0] bank=ram_addr[8+:BANK_BITS];
        wire [9:0] local_addr={ram_addr[RAM_AW-1:8+BANK_BITS],ram_addr[7:0]};
        wire [PLANES*18-1:0] bank_q[0:BANKS-1];
        reg [BANK_BITS-1:0] read_bank;
        always @(posedge clk)if(ram_re)read_bank<=bank;
        for(b=0;b<BANKS;b=b+1)begin:g_bank
            for(p=0;p<PLANES;p=p+1)begin:g_plane
                (* syn_ramstyle="block_ram" *) reg [17:0] data[0:1023];
                reg [17:0] q;
                always @(posedge clk)begin
                    if(ram_we && bank==b)data[local_addr]<=expanded_data[p*18+:18];
                    if(ram_re && bank==b)q<=data[local_addr];
                end
                assign bank_q[b][p*18+:18]=q;
            end
        end
        assign ram_q=bank_q[read_bank];
    end endgenerate
    wire meta_we=!rst && (state==INIT_META || hit_write ||
        (state==WRITE_LINE && line_ack) || (state==FILL && slot==WORDS-1));
    wire [INDEX_BITS-1:0] meta_addr=state==INIT_META?scan_index:active_index;
    wire [TAG_BITS+1:0] meta_data=state==INIT_META?0:
        state==WRITE_LINE?{1'b1,1'b0,meta_q[TAG_BITS-1:0]}:
        {1'b1,hit_write,request_tag};
    always @(posedge clk)if(meta_we)meta[meta_addr]<=meta_data;
    assign line_req=state==WRITE_LINE || state==READ_LINE;
    assign line_we=state==WRITE_LINE;
    assign line_addr=line_we?evict_addr:
        BASE_LINE+(saved_addr >> WORD_BITS);
    assign line_wdata=transfer;
    always @(posedge clk)begin
        if(rst)begin
            state<=INIT_META;ack<=0;rdata<=0;
            saved_addr<=0;saved_data<=0;saved_we<=0;slot<=0;
            scan_index<=0;transfer<=0;evict_addr<=0;
            flushing<=0;flush_done<=0;
        end else case(state)
            INIT_META:if(scan_index==LINES-1)begin scan_index<=0;state<=IDLE;end
                else scan_index<=scan_index+1'b1;
            IDLE:begin
                ack<=0;flush_done<=0;
                if(flush_req)begin flushing<=1;scan_index<=0;state<=FLUSH_SCAN;end
                else if(req)begin saved_addr<=word_addr;saved_data<=wdata;
                    saved_we<=we;state<=LOOKUP;end
            end
            LOOKUP:if(hit)begin
                if(saved_we)begin ack<=1;state<=RESPONSE;end
                else state<=READ_RESULT;
            end else if(meta_q[TAG_BITS+1] && meta_q[TAG_BITS])begin
                evict_addr<=BASE_LINE+{meta_q[TAG_BITS-1:0],request_index};
                slot<=0;transfer<=0;state<=COLLECT_REQUEST;
            end else state<=READ_LINE;
            READ_RESULT:begin rdata<=ram_q[DW-1:0];ack<=1;state<=RESPONSE;end
            RESPONSE:if(!req)begin ack<=0;state<=IDLE;end
            COLLECT_REQUEST:state<=COLLECT_CAPTURE;
            COLLECT_CAPTURE:begin
                transfer[slot*DDR_STRIDE+:DDR_STRIDE]<=ram_q[DW-1:0];
                if(slot==WORDS-1)state<=WRITE_LINE;
                else begin slot<=slot+1'b1;state<=COLLECT_REQUEST;end
            end
            WRITE_LINE:if(line_ack)state<=WRITE_GAP;
            WRITE_GAP:if(!line_ack)state<=flushing?FLUSH_ADVANCE:READ_LINE;
            READ_LINE:if(line_ack)begin transfer<=line_rdata;slot<=0;state<=READ_GAP;end
            READ_GAP:if(!line_ack)state<=FILL;
            FILL:if(slot==WORDS-1)begin
                state<=LOOKUP;
            end else slot<=slot+1'b1;
            FLUSH_SCAN:if(meta_q[TAG_BITS+1] && meta_q[TAG_BITS])begin
                evict_addr<=BASE_LINE+{meta_q[TAG_BITS-1:0],scan_index};
                slot<=0;transfer<=0;state<=COLLECT_REQUEST;
            end else state<=FLUSH_ADVANCE;
            FLUSH_ADVANCE:if(scan_index==LINES-1)begin
                flushing<=0;flush_done<=1;state<=FLUSH_RESPONSE;
            end else begin scan_index<=scan_index+1'b1;state<=FLUSH_SCAN;end
            FLUSH_RESPONSE:if(!flush_req)begin flush_done<=0;state<=IDLE;end
            default:state<=IDLE;
        endcase
    end
endmodule

// CPU remains on chip. Only these two map clients share the native mailbox.
// Round robin is per complete line transaction, including request release.
module nav_ddr_maps_probe #(
    parameter SM_INDEX_BITS=9
)(
    input wire clk, ui_clk, rst,
    input wire sm_req,sm_we,input wire [15:0]sm_addr,input wire[17:0]sm_wdata,
    output wire sm_ack,output wire[17:0]sm_rdata,
    input wire em_req,em_we,input wire[13:0]em_addr,input wire[52:0]em_wdata,
    output wire em_ack,output wire[52:0]em_rdata,
    input wire flush_req,output wire flush_done,
    input wire calib_done,output wire fault,
    output wire[2:0]cmd,output wire cmd_en,output wire[27:0]addr,
    input wire cmd_ready,output wire[255:0]wr_data,output wire[31:0]wr_mask,
    output wire wr_en,wr_end,input wire wr_ready,
    input wire[255:0]rd_data,input wire rd_valid,rd_end
);
    wire sr,sw,sa,er,ew,ea,sflush,eflush;
    wire[24:0]saddr,eaddr;
    wire[255:0]sd,ed,line_rdata;
    wire native_ack;
    reg[1:0]arb_state;
    reg owner,last_owner;
    wire native_req=arb_state==1 && (owner?er:sr);
    assign sa=arb_state==1 && !owner && native_ack;
    assign ea=arb_state==1 && owner && native_ack;
    assign flush_done=sflush && eflush;
    nav_ddr_map_cache #(.DW(18),.AW(16),.WORD_BITS(3),.INDEX_BITS(SM_INDEX_BITS),.BASE_LINE(0)) u_sm
        (clk,rst,sm_req,sm_we,sm_addr,sm_wdata,sm_ack,sm_rdata,
         flush_req,,sflush,sr,sw,saddr,sd,sa,line_rdata);
    nav_ddr_map_cache #(.DW(53),.AW(14),.WORD_BITS(2),.INDEX_BITS(8),.BASE_LINE(8192)) u_em
        (clk,rst,em_req,em_we,em_addr,em_wdata,em_ack,em_rdata,
         flush_req,,eflush,er,ew,eaddr,ed,ea,line_rdata);
    nav_ddr_native u_native(clk,ui_clk,rst,native_req,owner?ew:sw,
        owner?eaddr:saddr,owner?ed:sd,32'd0,native_ack,line_rdata,calib_done,
        cmd,cmd_en,addr,cmd_ready,wr_data,wr_mask,wr_en,wr_end,wr_ready,
        rd_data,rd_valid,rd_end,fault);
    always @(posedge clk)begin
        if(rst)begin arb_state<=0;owner<=0;last_owner<=1;end
        else case(arb_state)
            0:if(sr || er)begin owner<=sr&&er?!last_owner:er;arb_state<=1;end
            1:if(native_ack)begin last_owner<=owner;arb_state<=2;end
            2:if(!native_ack)arb_state<=0;
            default:arb_state<=0;
        endcase
    end
endmodule
