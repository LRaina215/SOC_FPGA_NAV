// Isolated full16384x53 elevation backing. Native base8192 is disjoint from
// the structure map's4864 packed lines. Shared physical arbitration comes
// later; no second DDR controller is permitted in the joint design.
module nav_ddr_elevation_store(
    input wire clk,ui_clk,rst,calib_done,
    input wire req,we,input wire[13:0]word_addr,input wire[52:0]wdata,
    output wire ack,output wire[52:0]q,
    input wire flush_req,output wire flush_done,output wire fault,
    output wire[2:0]cmd,output wire cmd_en,output wire[27:0]addr,input wire cmd_ready,
    output wire[255:0]wr_data,output wire[31:0]wr_mask,output wire wr_en,wr_end,
    input wire wr_ready,input wire[255:0]rd_data,input wire rd_valid,rd_end
);
    wire line_req,line_we,line_ack,native_fault;
    wire[24:0]line_addr;wire[255:0]line_wdata,line_reply;
    (* syn_keep=1 *)reg ft1,ft2;
    wire cr;
    nav_ddr_reset u_reset(clk,rst,cr);
    always@(posedge clk or posedge cr)
        if(cr)begin ft1<=0;ft2<=0;end
        else begin ft1<=native_fault;ft2<=ft1;end
    assign fault=ft2;
    nav_ddr_map_cache #(.DW(53),.AW(14),.WORD_BITS(2),.INDEX_BITS(8),.BASE_LINE(8192))
        u_cache(clk,rst,req,we,word_addr,wdata,ack,q,flush_req,,flush_done,
            line_req,line_we,line_addr,line_wdata,line_ack,line_reply);
    nav_ddr_native u_native(clk,ui_clk,rst,line_req,line_we,line_addr,line_wdata,32'd0,
        line_ack,line_reply,calib_done,cmd,cmd_en,addr,cmd_ready,
        wr_data,wr_mask,wr_en,wr_end,wr_ready,rd_data,rd_valid,rd_end,native_fault);
endmodule
