// Full65536x18 map behind a256-line write-back cache. Same packed row format
// as nav_ddr_row_stream: native_line=row*19+column/14, cell18=(column%14)*18.
// Barrier drains ALL dirty lines and invalidates ALL entries before an
// exclusive streaming UPDATE. A held barrier_done prevents early re-entry.
module nav_ddr_struct_cache(
    input wire clk,rst,req,we,input wire[15:0]word_addr,
    input wire[17:0]wdata,output reg ack,output reg[17:0]rdata,
    input wire barrier_req,output reg barrier_done,
    output wire line_req,line_we,output wire[24:0]line_addr,
    output wire[255:0]line_wdata,input wire line_ack,input wire[255:0]line_rdata
);
    localparam INIT=0,IDLE=1,LOOKUP=2,READ_RESULT=3,RESPONSE=4,
        COLLECT=5,CAPTURE=6,WRITE_LINE=7,WRITE_GAP=8,READ_LINE=9,
        READ_GAP=10,FILL=11,SCAN=12,CLEAR=13,BARRIER_RESPONSE=14;
    reg[3:0]state,slot;
    reg[15:0]saved_addr;reg[17:0]saved_data;reg saved_we,barriering;
    reg[7:0]scan_index;reg[12:0]evict_addr;reg[255:0]transfer;
    (* syn_ramstyle="distributed_ram" *)reg[6:0]meta[0:255];
    wire[12:0]request_line=saved_addr[15:8]*19+saved_addr[7:0]/14;
    wire[3:0]request_slot=saved_addr[7:0]%14;
    wire[7:0]active_index=barriering?scan_index:request_line[7:0];
    wire[6:0]mq=meta[active_index];
    wire hit=mq[6]&&mq[4:0]==request_line[12:8];
    wire hit_write=state==LOOKUP&&hit&&saved_we;
    wire ram_we=hit_write||state==FILL;
    wire ram_re=(state==LOOKUP&&hit&&!saved_we)||state==COLLECT;
    wire[11:0]ram_addr={active_index,(state==FILL||state==COLLECT)?slot:request_slot};
    wire[17:0]ram_wd=hit_write?saved_data:transfer[slot*18+:18];
    wire[17:0]bank_q[0:3];reg[1:0]read_bank;
    always@(posedge clk)if(ram_re)read_bank<=ram_addr[11:10];
    genvar b;
    generate for(b=0;b<4;b=b+1)begin:g_bank
        (* syn_ramstyle="block_ram" *)reg[17:0]data[0:1023];reg[17:0]q;
        always@(posedge clk)begin
            if(ram_we&&ram_addr[11:10]==b)data[ram_addr[9:0]]<=ram_wd;
            if(ram_re&&ram_addr[11:10]==b)q<=data[ram_addr[9:0]];
        end
        assign bank_q[b]=q;
    end endgenerate
    wire meta_we=!rst&&(state==INIT||state==CLEAR||hit_write||
        (state==FILL&&slot==13)||(state==WRITE_LINE&&line_ack));
    wire[7:0]meta_addr=state==INIT?scan_index:active_index;
    wire[6:0]meta_wd=(state==INIT||state==CLEAR)?0:
        state==WRITE_LINE?{1'b1,1'b0,mq[4:0]}:{1'b1,hit_write,request_line[12:8]};
    always@(posedge clk)if(meta_we)meta[meta_addr]<=meta_wd;
    assign line_req=state==WRITE_LINE||state==READ_LINE;
    assign line_we=state==WRITE_LINE;
    assign line_addr={12'd0,line_we?evict_addr:request_line};
    assign line_wdata=transfer;
    always@(posedge clk)begin
        if(rst)begin state<=INIT;scan_index<=0;ack<=0;barrier_done<=0;
            saved_addr<=0;saved_data<=0;saved_we<=0;barriering<=0;
            slot<=0;evict_addr<=0;transfer<=0;rdata<=0;end
        else case(state)
            INIT:if(scan_index==255)begin scan_index<=0;state<=IDLE;end
                else scan_index<=scan_index+1'b1;
            IDLE:begin ack<=0;barrier_done<=0;
                if(barrier_req)begin barriering<=1;scan_index<=0;state<=SCAN;end
                else if(req)begin saved_addr<=word_addr;saved_data<=wdata;saved_we<=we;state<=LOOKUP;end
            end
            LOOKUP:if(hit)begin
                if(saved_we)begin ack<=1;state<=RESPONSE;end else state<=READ_RESULT;
            end else if(mq[6]&&mq[5])begin
                evict_addr<={mq[4:0],active_index};slot<=0;transfer<=0;state<=COLLECT;
            end else state<=READ_LINE;
            READ_RESULT:begin rdata<=bank_q[read_bank];ack<=1;state<=RESPONSE;end
            RESPONSE:if(!req)begin ack<=0;state<=IDLE;end
            COLLECT:state<=CAPTURE;
            CAPTURE:begin transfer[slot*18+:18]<=bank_q[read_bank];
                if(slot==13)state<=WRITE_LINE;else begin slot<=slot+1'b1;state<=COLLECT;end
            end
            WRITE_LINE:if(line_ack)state<=WRITE_GAP;
            WRITE_GAP:if(!line_ack)state<=barriering?CLEAR:READ_LINE;
            READ_LINE:if(line_ack)begin transfer<=line_rdata;slot<=0;state<=READ_GAP;end
            READ_GAP:if(!line_ack)state<=FILL;
            FILL:if(slot==13)state<=LOOKUP;else slot<=slot+1'b1;
            SCAN:if(mq[6]&&mq[5])begin
                evict_addr<={mq[4:0],scan_index};slot<=0;transfer<=0;state<=COLLECT;
            end else state<=CLEAR;
            CLEAR:if(scan_index==255)begin barrier_done<=1;state<=BARRIER_RESPONSE;end
                else begin scan_index<=scan_index+1'b1;state<=SCAN;end
            BARRIER_RESPONSE:if(!barrier_req)begin barrier_done<=0;barriering<=0;state<=IDLE;end
            default:state<=INIT;
        endcase
    end
endmodule
