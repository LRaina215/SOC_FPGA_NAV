// Isolated 256-word structure-map row transport. No G2 connection yet.
// One descriptor crosses each way per ROW, never per map word. Two native
// read/write credits hide serialization. Native ready has no assumed timeout.
`ifdef SYNTHESIS
`define NAV_DDR_ROW_PHYSICAL
`endif
`ifdef GOWIN_ROW_PRIMITIVES
`define NAV_DDR_ROW_PHYSICAL
`endif
module nav_ddr_row_stream (
    input wire clk,ui_clk,rst,calib_done,
    input wire req,we,bank,input wire[7:0]row,
    output reg ack,output reg fault,
    input wire[1:0] in_re,in_we,input wire[15:0]in_addr,
    input wire[35:0]in_data,output wire[35:0]in_q,
    input wire out_we,out_bank,input wire[7:0]out_addr,
    input wire[17:0]out_data,
    output wire[2:0]cmd,output wire cmd_en,output wire[27:0]addr,
    input wire cmd_ready,output wire[255:0]wr_data,
    output wire[31:0]wr_mask,output wire wr_en,wr_end,
    input wire wr_ready,input wire[255:0]rd_data,input wire rd_valid,rd_end
);
    wire cr,ur;
    nav_ddr_reset u_cr(clk,rst,cr);
    nav_ddr_reset u_ur(ui_clk,rst,ur);
    reg rq,dn,held_we,held_bank;reg[7:0]held_row;
    (* syn_keep=1 *)reg rq1,rq2,dn1,dn2,ft1,ft2;
    reg[1:0]cs;
    reg ui_fault,seen_calib;
    always@(posedge clk or posedge cr)begin
        if(cr)begin rq<=0;cs<=0;ack<=0;held_we<=0;held_bank<=0;
            held_row<=0;dn1<=0;dn2<=0;ft1<=0;ft2<=0;fault<=0;end
        else begin
            dn1<=dn;dn2<=dn1;ft1<=ui_fault;ft2<=ft1;
            if(ft2)fault<=1;
            case(cs)
                0:if(req&&!fault)begin held_we<=we;held_bank<=bank;
                    held_row<=row;rq<=~rq;cs<=1;end
                1:if(dn2==rq&&!fault)begin ack<=1;cs<=2;end
                2:if(!req)begin ack<=0;cs<=0;end
                default:cs<=0;
            endcase
        end
    end
    localparam IDLE=0,RUN_READ=1,RUN_WRITE=2;
    reg[1:0]us;
    reg[5:0]issued,received,completed,packed_lines,data_sent;
    // Fourteen complete 18-bit cells per BL8 beat (252/256 bits). This
    // preserves every cell bit and reduces row traffic from32 to19 beats.
    reg[3:0]slot,fetch_slot,capture_slot;
    reg[4:0]fetch_line,capture_line;
    reg[8:0]fetched;
    reg capture;reg[7:0]capture_addr;
    reg[255:0]beats[0:1];
    wire[5:0]reclaimed=issued<data_sent?issued:data_sent;
    wire live=calib_done&&!ui_fault;
    wire unpack=us==RUN_READ&&received>completed&&live;
    wire fetch=us==RUN_WRITE&&fetched<256&&
        (fetch_line-reclaimed<2)&&live;
    reg[17:0]out_q[0:1];
    genvar b;
    generate for(b=0;b<2;b=b+1)begin:g_bank
        // True dual-port: UI fills an owned row; core applies updates/reads it.
        // Ownership is checked by the caller before a row changes domains.
`ifdef NAV_DDR_ROW_PHYSICAL
        wire[7:0]fill_address=completed*14+slot;
        wire fill_write=unpack&&held_bank==b;
        // Explicit mode: the inference engine retained WRITE_MODE0=10 even
        // after the mutual-exclusion read enable was made explicit in RTL.
        DPX9B input_row_primitive(
            .DOA(in_q[b*18+:18]),.DOB(),
            .CLKA(clk),.CEA(in_re[b]|in_we[b]),.OCEA(1'b0),.RESETA(1'b0),
            .CLKB(ui_clk),.CEB(fill_write),.OCEB(1'b0),.RESETB(1'b0),
            .WREA(in_we[b]),.WREB(1'b1),
            .ADA({2'b00,in_addr[b*8+:8],4'b0011}),
            .ADB({2'b00,fill_address,4'b0011}),
            .DIA(in_data[b*18+:18]),.DIB(beats[completed[0]][slot*18+:18]),
            .BLKSELA(3'b000),.BLKSELB(3'b000));
        defparam input_row_primitive.BIT_WIDTH_0=18;
        defparam input_row_primitive.BIT_WIDTH_1=18;
        defparam input_row_primitive.READ_MODE0=1'b0;
        defparam input_row_primitive.READ_MODE1=1'b0;
        defparam input_row_primitive.WRITE_MODE0=2'b00;
        defparam input_row_primitive.WRITE_MODE1=2'b00;
        defparam input_row_primitive.RESET_MODE="SYNC";
        defparam input_row_primitive.BLK_SEL_0=3'b000;
        defparam input_row_primitive.BLK_SEL_1=3'b000;
`else
        (* syn_ramstyle="block_ram" *)reg[17:0]input_row[0:255];
        reg[17:0]q;
        always@(posedge clk)begin
            if(in_we[b])input_row[in_addr[b*8+:8]]<=in_data[b*18+:18];
            // The caller never reads a bank while writing that bank. Keep
            // its last read result during writes; GW5A true dual-port RAM
            // supports normal mode, but rejects read-before-write mode.
            else if(in_re[b])q<=input_row[in_addr[b*8+:8]];
        end
        always@(posedge ui_clk)
            if(unpack&&held_bank==b)
                input_row[completed*14+slot]<=beats[completed[0]][slot*18+:18];
        assign in_q[b*18+:18]=q;
`endif
        (* syn_ramstyle="block_ram" *)reg[17:0]output_row[0:255];
        always@(posedge clk)if(out_we&&out_bank==b)output_row[out_addr]<=out_data;
        always@(posedge ui_clk)if(fetch&&held_bank==b)out_q[b]<=output_row[fetched[7:0]];
    end endgenerate
    assign cmd=held_we?3'b000:3'b001;
    assign cmd_en=live&&((us==RUN_READ&&issued<19&&issued-completed<2)||
        (us==RUN_WRITE&&issued<packed_lines));
    wire[12:0]native_line=held_row*19+issued;
    assign addr={12'd0,native_line,3'd0};
    assign wr_en=live&&us==RUN_WRITE&&data_sent<packed_lines;
    assign wr_end=wr_en;
    assign wr_data=beats[data_sent[0]];
    assign wr_mask=0;
    always@(posedge ui_clk or posedge ur)begin
        if(ur)begin rq1<=0;rq2<=0;dn<=0;us<=IDLE;ui_fault<=0;
            seen_calib<=0;issued<=0;received<=0;completed<=0;
            packed_lines<=0;data_sent<=0;slot<=0;fetched<=0;
            fetch_slot<=0;fetch_line<=0;capture_slot<=0;capture_line<=0;
            capture<=0;capture_addr<=0;end
        else begin
            rq1<=rq;rq2<=rq1;capture<=fetch;
            if(calib_done)seen_calib<=1;
            if(seen_calib&&!calib_done)ui_fault<=1;
            if(live)case(us)
                IDLE:if(rq2!=dn)begin us<=held_we?RUN_WRITE:RUN_READ;
                    issued<=0;received<=0;completed<=0;slot<=0;
                    packed_lines<=0;data_sent<=0;fetched<=0;capture<=0;
                    fetch_slot<=0;fetch_line<=0;end
                RUN_READ:begin
                    if(cmd_en&&cmd_ready)issued<=issued+1'b1;
                    if(rd_valid)begin
                        if(!rd_end||received>=issued||received-completed>=2)ui_fault<=1;
                        else begin beats[received[0]]<=rd_data;received<=received+1'b1;end
                    end
                    if(unpack)begin
                        if(slot==13||(completed==18&&slot==3))begin slot<=0;completed<=completed+1'b1;end
                        else slot<=slot+1'b1;
                    end
                    if(completed==19)begin dn<=rq2;us<=IDLE;end
                end
                RUN_WRITE:begin
                    if(fetch)begin fetched<=fetched+1'b1;capture_addr<=fetched[7:0];
                        capture_line<=fetch_line;capture_slot<=fetch_slot;
                        if(fetch_slot==13)begin fetch_slot<=0;fetch_line<=fetch_line+1'b1;end
                        else fetch_slot<=fetch_slot+1'b1;
                    end
                    if(capture)begin
                        if(capture_slot==0)beats[capture_line[0]]<={238'd0,out_q[held_bank]};
                        else beats[capture_line[0]][capture_slot*18+:18]<=out_q[held_bank];
                        if(capture_slot==13||capture_addr==255)packed_lines<=packed_lines+1'b1;
                    end
                    if(cmd_en&&cmd_ready)issued<=issued+1'b1;
                    if(wr_en&&wr_ready)data_sent<=data_sent+1'b1;
                    if(issued==19&&data_sent==19)begin dn<=rq2;us<=IDLE;end
                end
                default:ui_fault<=1;
            endcase
        end
    end
endmodule
`ifdef NAV_DDR_ROW_PHYSICAL
`undef NAV_DDR_ROW_PHYSICAL
`endif
