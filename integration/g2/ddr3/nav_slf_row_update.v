// Isolated streaming update/build-LF candidate. CPU and control are untouched.
// Records are the ORIGINAL transformed point's slot/tag/centroid. A touch
// bitmap retains the first record per slot, including aliases with other tags.
// The unchanged LF arithmetic advances under a common CE, including all RAMs.
module nav_slf_row_update (
    input wire clk,ui_clk,rst,calib_done,
    input wire start,input wire signed[31:0]cx,cy,
    input wire record_valid,input wire[15:0]record_addr,
    input wire[7:0]record_meta,input wire records_end,
    output wire record_ready,output wire ready,
    output reg busy,done,output wire fault,
    output wire[2:0]cmd,output wire cmd_en,output wire[27:0]addr,
    input wire cmd_ready,output wire[255:0]wr_data,
    output wire[31:0]wr_mask,output wire wr_en,wr_end,
    input wire wr_ready,input wire[255:0]rd_data,input wire rd_valid,rd_end
);
    localparam COLD=0,IDLE=1,HEADS=2,COLLECT=3,RUN=4,CLEAR=5;
    reg[2:0]phase;
    reg[15:0]clear_addr;
    reg[7:0]head_index;
    reg[1:0]rec_state;
    reg[15:0]ra;reg[7:0]rm;
    reg[10:0]count,clear_index;
    wire[35:0]node_q;
    (* syn_ramstyle="distributed_ram" *)reg[10:0]heads[0:255];
    (* syn_ramstyle="block_ram" *)reg touch[0:65535];
    reg tq;
    wire touch_re=phase==COLLECT&&rec_state==0&&record_valid;
    wire unique_record=phase==COLLECT&&rec_state==2&&!tq;
    wire touch_we=phase==COLD||unique_record||(phase==CLEAR&&rec_state==2);
    wire[15:0]touch_addr=phase==COLD?clear_addr:
        phase==CLEAR?node_q[23:8]:ra;
    always@(posedge clk)begin
        if(touch_re)tq<=touch[record_addr];
        if(touch_we)touch[touch_addr]<=unique_record;
    end
    wire node_we=unique_record;
    wire[35:0]node_wd={1'b0,heads[ra[15:8]],ra,rm};
    reg[10:0]pointer;
    reg[2:0]up_state;
    reg up_bank;reg[15:0]up_addr;reg[7:0]up_meta;
    wire node_re=(phase==RUN&&up_state==1)||(phase==CLEAR&&rec_state==0);
    wire[9:0]node_ra=phase==CLEAR?clear_index[9:0]:pointer[9:0]-1'b1;
    genvar p;
    generate for(p=0;p<2;p=p+1)begin:g_nodes
        (* syn_ramstyle="block_ram" *)reg[17:0]data[0:1023];
        reg[17:0]q;
        always@(posedge clk)begin
            if(node_we)data[count[9:0]]<=node_wd[p*18+:18];
            if(node_re)q<=data[node_ra];
        end
        assign node_q[p*18+:18]=q;
    end endgenerate
    assign ready=phase==IDLE;
    assign record_ready=phase==COLLECT&&rec_state==0&&count<1024;

    reg dma_req,dma_we,dma_bank,dma_fence;reg[7:0]dma_row;
    wire dma_ack;
    wire[35:0]iq;
    reg[1:0]ire,iwe;reg[15:0]ia;reg[35:0]id;
    reg[1:0]input_ready,output_ready;
    reg[7:0]input_tag[0:1],output_tag[0:1];
    reg[8:0]next_fetch;
    reg signed[31:0]cx_l,cy_l;
    wire[7:0]first_row=cy_l[7:0]-8'd128;
    wire[7:0]fetch_row=first_row+next_fetch[7:0];
    wire[7:0]last_col=cx_l[7:0]+8'd127;
    reg lf_start,lf_started,lf_finished,fenced;
    wire lf_busy,lf_done,lr,lw;
    wire[15:0]la,lwa;wire[17:0]lwd;
    reg read_bank,read_pending;reg[17:0]read_hold;
    wire[17:0]lf_q=read_pending?(read_bank?iq[35:18]:iq[17:0]):read_hold;
    wire ce=phase==RUN&&(!lr||(input_ready[la[8]]&&input_tag[la[8]]==la[15:8]))&&
        (!lw||!output_ready[lwa[8]]);
    slf_lf_ce u_lf(clk,rst,ce,lf_start,cx_l,cy_l,lf_busy,lf_done,lr,la,
        lf_q,lw,lwa,lwd);
    function[2:0]mdiv(input[2:0]a,input[2:0]n);
        case(n)2:mdiv=a>>1;4:mdiv=({1'b0,a}+1)>>2;
            default:mdiv=a>=5?2:a>=2?1:0;endcase
    endfunction
    function[17:0]updated(input[17:0]old,input[7:0]m);
        reg fresh;reg[1:0]h;reg signed[3:0]dx,dy;
        reg[2:0]mx,my,x,y;
        begin
            fresh=old[17:16]!=m[7:6]||old[15:14]==0;
            h=fresh?0:old[15:14];
            dx=$signed({1'b0,m[5:3]})-$signed({1'b0,old[13:11]});
            dy=$signed({1'b0,m[2:0]})-$signed({1'b0,old[10:8]});
            mx=mdiv(dx<0?-dx:dx,{1'b0,h}+1);
            my=mdiv(dy<0?-dy:dy,{1'b0,h}+1);
            x=fresh?m[5:3]:h>=3?old[13:11]:dx<0?old[13:11]-mx:old[13:11]+mx;
            y=fresh?m[2:0]:h>=3?old[10:8]:dy<0?old[10:8]-my:old[10:8]+my;
            updated={m[7:6],h>=3?2'd3:h+2'd1,x,y,old[7:0]};
        end
    endfunction
    always@*begin
        ire=0;iwe=0;ia=0;id=0;
        if(lr&&ce)begin ire[la[8]]=1;ia[la[8]*8+:8]=la[7:0];end
        if(up_state==3)begin ire[up_bank]=1;ia[up_bank*8+:8]=node_q[15:8];end
        if(up_state==5)begin iwe[up_bank]=1;ia[up_bank*8+:8]=up_addr[7:0];
            id[up_bank*18+:18]=updated(up_bank?iq[35:18]:iq[17:0],up_meta);end
    end
    nav_ddr_row_stream u_dma(clk,ui_clk,rst,calib_done,dma_req,dma_we,dma_bank,dma_row,
        dma_ack,fault,ire,iwe,ia,id,iq,lw&&ce,lwa[8],lwa[7:0],lwd,
        cmd,cmd_en,addr,cmd_ready,wr_data,wr_mask,wr_en,wr_end,wr_ready,rd_data,rd_valid,rd_end);
    always@(posedge clk)begin
        // A paused pipeline still owns its previous synchronous read result.
        // Preserve it before either row bank is reused by an update or DMA.
        read_pending<=lr&&ce;
        if(read_pending)read_hold<=read_bank?iq[35:18]:iq[17:0];
        if(lr&&ce)read_bank<=la[8];
        done<=0;lf_start<=0;
        if(rst)begin phase<=COLD;clear_addr<=0;busy<=1;count<=0;
            dma_req<=0;dma_we<=0;dma_bank<=0;dma_row<=0;dma_fence<=0;
            input_ready<=0;output_ready<=0;next_fetch<=0;up_state<=0;
            lf_started<=0;lf_finished<=0;fenced<=0;rec_state<=0;
        end else case(phase)
            COLD:if(clear_addr==65535)begin phase<=IDLE;busy<=0;end
                else clear_addr<=clear_addr+1'b1;
            IDLE:if(start&&!fault)begin phase<=HEADS;busy<=1;head_index<=0;
                count<=0;rec_state<=0;cx_l<=cx;cy_l<=cy;
                input_ready<=0;output_ready<=0;next_fetch<=0;
                lf_started<=0;lf_finished<=0;fenced<=0;end
            HEADS:begin
                heads[head_index]<=0;
                if(head_index==255)phase<=COLLECT;else head_index<=head_index+1'b1;
            end
            COLLECT:case(rec_state)
                0:if(record_valid&&record_ready)begin ra<=record_addr;rm<=record_meta;rec_state<=1;end
                    else if(records_end)begin phase<=RUN;up_state<=0;end
                1:rec_state<=2;
                2:begin
                    if(!tq)begin heads[ra[15:8]]<=count+1'b1;count<=count+1'b1;end
                    rec_state<=0;
                end
                default:rec_state<=0;
            endcase
            RUN:begin
                if(lf_done&&ce)lf_finished<=1;
                if(!lf_started&&input_ready[first_row[0]])begin lf_start<=1;lf_started<=1;end
                if(lr&&ce&&la[7:0]==last_col)input_ready[la[8]]<=0;
                if(lw&&ce)begin
                    output_tag[lwa[8]]<=lwa[15:8];
                    if(lwa[7:0]==last_col)output_ready[lwa[8]]<=1;
                end
                if(dma_req&&dma_ack&&(dma_we||dma_fence||up_state==0))begin
                    dma_req<=0;
                    if(dma_we)output_ready[dma_bank]<=0;
                    else if(dma_fence)fenced<=1;
                    else begin
                        input_tag[dma_bank]<=dma_row;up_bank<=dma_bank;
                        pointer<=heads[dma_row];
                        if(heads[dma_row]==0)input_ready[dma_bank]<=1;
                        else up_state<=1;
                    end
                end else if(!dma_req&&!dma_ack)begin
                    // Drain completed output rows first: no unbounded client
                    // traffic may preempt this admitted update transaction.
                    if(output_ready!=0)begin dma_req<=1;dma_we<=1;dma_fence<=0;
                        dma_bank<=output_ready[0]?0:1;
                        dma_row<=output_ready[0]?output_tag[0]:output_tag[1];end
                    else if(next_fetch<256&&!input_ready[fetch_row[0]]&&up_state==0)begin
                        dma_req<=1;dma_we<=0;dma_fence<=0;dma_bank<=fetch_row[0];
                        dma_row<=fetch_row;next_fetch<=next_fetch+1'b1;
                    end else if(lf_finished&&!fenced&&up_state==0)begin
                        // Ordered read drains the native write queue. This
                        // relies on the controller's ordering contract, which
                        // must be established before joint integration.
                        dma_req<=1;dma_we<=0;dma_fence<=1;dma_bank<=0;
                        dma_row<=first_row-1'b1;
                    end
                end
                case(up_state)
                    1:up_state<=2;
                    2:up_state<=3;
                    3:begin up_addr<=node_q[23:8];up_meta<=node_q[7:0];
                        pointer<=node_q[34:24];up_state<=4;end
                    4:up_state<=5;
                    5:if(pointer==0)begin input_ready[up_bank]<=1;up_state<=0;end
                        else up_state<=1;
                    default:;
                endcase
                if(fenced&&!dma_req&&!dma_ack)begin
                    phase<=CLEAR;clear_index<=0;rec_state<=0;
                end
            end
            CLEAR:case(rec_state)
                0:if(clear_index==count)begin phase<=IDLE;busy<=0;done<=1;end
                    else rec_state<=1;
                1:rec_state<=2;
                2:begin clear_index<=clear_index+1'b1;rec_state<=0;end
                default:rec_state<=0;
            endcase
            default:phase<=COLD;
        endcase
    end
endmodule
