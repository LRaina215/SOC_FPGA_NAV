// Isolated coherent structure-map memory/UPDATE owner. The existing control
// and CPU remain independent. This is not yet a complete matcher/DDR system.
module nav_ddr_structure_store(
    input wire clk,ui_clk,rst,calib_done,
    input wire req,we,input wire[15:0]word_addr,input wire[17:0]wdata,
    output wire ack,output wire[17:0]rdata,
    input wire update_start,input wire signed[31:0]cx,cy,
    input wire record_valid,input wire[15:0]record_addr,input wire[7:0]record_meta,
    input wire records_end,output wire record_ready,
    output wire ready,output wire busy,output reg done,output wire fault,
    output wire[2:0]cmd,output wire cmd_en,output wire[27:0]addr,input wire cmd_ready,
    output wire[255:0]wr_data,output wire[31:0]wr_mask,output wire wr_en,wr_end,
    input wire wr_ready,input wire[255:0]rd_data,input wire rd_valid,rd_end
);
    localparam CLIENT=0,DRAIN=1,RELEASE=2,SWITCH_ROW=3,UPDATE=4,SWITCH_CLIENT=5;
    reg[2:0]state;reg barrier_req,row_start,owner_request;
    reg signed[31:0]held_cx,held_cy;
    wire barrier_done,line_req,line_we,line_ack;
    wire[24:0]line_addr;wire[255:0]line_wdata,line_rdata;
    wire ca;wire cf,rf,row_ready,row_busy,row_done;
    wire[2:0]cc,rc;wire ce,re,cwe,rwe,cwend,rwend;
    wire[27:0]caddr,raddr;wire[255:0]cwd,rwd;wire[31:0]cwm,rwm;
    (* syn_keep=1 *)reg own1,own2,own_ack1,own_ack2,cf1,cf2;
    wire row_owner=own2;
    wire core_reset,ui_reset;
    nav_ddr_reset u_core_reset(clk,rst,core_reset);
    nav_ddr_reset u_ui_reset(ui_clk,rst,ui_reset);
    always@(posedge ui_clk or posedge ui_reset)
        if(ui_reset)begin own1<=0;own2<=0;end
        else begin own1<=owner_request;own2<=own1;end
    always@(posedge clk or posedge core_reset)
        if(core_reset)begin own_ack1<=0;own_ack2<=0;cf1<=0;cf2<=0;end
        else begin own_ack1<=own2;own_ack2<=own_ack1;cf1<=cf;cf2<=cf1;end
    assign ack=ca;
    assign ready=state==CLIENT&&row_ready&&!fault;
    assign busy=state!=CLIENT||!row_ready;
    assign fault=cf2||rf;
    nav_ddr_struct_cache u_cache(clk,rst,req&&state==CLIENT&&row_ready&&!fault,we,word_addr,wdata,ca,rdata,
        barrier_req,barrier_done,line_req,line_we,line_addr,line_wdata,line_ack,line_rdata);
    nav_ddr_native u_native(clk,ui_clk,rst,line_req,line_we,line_addr,line_wdata,32'd0,
        line_ack,line_rdata,calib_done,cc,ce,caddr,cmd_ready&&!row_owner,cwd,cwm,cwe,cwend,
        wr_ready&&!row_owner,rd_data,rd_valid&&!row_owner,rd_end,cf);
    nav_slf_row_update u_update(clk,ui_clk,rst,calib_done,row_start,held_cx,held_cy,
        record_valid&&state==UPDATE,record_addr,record_meta,records_end&&state==UPDATE,
        record_ready,row_ready,row_busy,row_done,rf,
        rc,re,raddr,cmd_ready&&row_owner,rwd,rwm,rwe,rwend,wr_ready&&row_owner,
        rd_data,rd_valid&&row_owner,rd_end);
    assign cmd=row_owner?rc:cc;assign cmd_en=row_owner?re:ce;
    assign addr=row_owner?raddr:caddr;assign wr_data=row_owner?rwd:cwd;
    assign wr_mask=row_owner?rwm:cwm;assign wr_en=row_owner?rwe:cwe;
    assign wr_end=row_owner?rwend:cwend;
    always@(posedge clk)begin
        row_start<=0;done<=0;
        if(core_reset)begin state<=CLIENT;barrier_req<=0;row_start<=0;done<=0;held_cx<=0;held_cy<=0;owner_request<=0;end
        else case(state)
            CLIENT:if(update_start&&ready)begin
                held_cx<=cx;held_cy<=cy;barrier_req<=1;state<=DRAIN;
            end
            DRAIN:if(barrier_done)begin barrier_req<=0;state<=RELEASE;end
            RELEASE:if(!barrier_done)begin owner_request<=1;state<=SWITCH_ROW;end
            SWITCH_ROW:if(own_ack2)begin state<=UPDATE;row_start<=1;end
            UPDATE:if(row_done&&!fault)begin owner_request<=0;state<=SWITCH_CLIENT;end
            SWITCH_CLIENT:if(!own_ack2)begin state<=CLIENT;done<=1;end
            default:state<=CLIENT;
        endcase
    end
endmodule
