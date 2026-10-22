// G2 area trade: keep both 128x128 byte cost maps at full capacity, but use
// distributed RAM. Other memories retain the original inference description.
module nav_ram_sdp #(
 parameter integer DW=16,AW=10,DEPTH=(1<<AW),
 parameter INIT_FILE="",parameter [DW-1:0] INIT_VAL={DW{1'b0}}
)(input wire clk,we,input wire [AW-1:0] waddr,input wire [DW-1:0] wdata,
 input wire re,input wire [AW-1:0] raddr,output wire [DW-1:0] rdata);
 generate if(DW==8 && AW==14 && INIT_FILE=="")begin:g_cost_ssram
   (* syn_ramstyle="distributed_ram" *)reg [DW-1:0] mem[0:DEPTH-1];
   reg [DW-1:0] qr;integer i;
   initial begin for(i=0;i<DEPTH;i=i+1)mem[i]=INIT_VAL;qr=0;end
   always @(posedge clk)if(we)mem[waddr]<=wdata;
   always @(posedge clk)if(re)qr<=mem[raddr];
   assign rdata=qr;
 end else begin:g_original
   reg [DW-1:0] mem[0:DEPTH-1];reg [DW-1:0] qr;integer i;
   initial begin
      if(INIT_FILE!="")$readmemh(INIT_FILE,mem);
      else for(i=0;i<DEPTH;i=i+1)mem[i]=INIT_VAL;
      qr=0;
   end
   always @(posedge clk)if(we)mem[waddr]<=wdata;
   always @(posedge clk)if(re)qr<=mem[raddr];
   assign rdata=qr;
 end endgenerate
endmodule
