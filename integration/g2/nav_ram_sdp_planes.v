// G2 exact bit-plane layout. Only the 53-bit elevation array changes storage:
// low32=zmin/zmax in block RAM, high21=stale/stamp/tag/count in distributed RAM.
// Capacity, initialized values and synchronous enabled read semantics unchanged.
module nav_ram_sdp #(
 parameter integer DW=16,AW=10,DEPTH=(1<<AW),
 parameter INIT_FILE="",parameter [DW-1:0] INIT_VAL={DW{1'b0}}
)(input wire clk,we,input wire [AW-1:0] waddr,input wire [DW-1:0] wdata,
 input wire re,input wire [AW-1:0] raddr,output wire [DW-1:0] rdata);
 generate if(DW==53 && AW==14 && INIT_FILE=="")begin:g_planes
   (* syn_ramstyle="block_ram" *)reg [31:0] heights[0:DEPTH-1];
   (* syn_ramstyle="distributed_ram" *)reg [20:0] metadata[0:DEPTH-1];
   reg [52:0] qr;integer i;
   initial begin
     for(i=0;i<DEPTH;i=i+1)begin heights[i]=INIT_VAL[31:0];metadata[i]=INIT_VAL[52:32];end
     qr=0;
   end
   always @(posedge clk)if(we)begin heights[waddr]<=wdata[31:0];metadata[waddr]<=wdata[52:32];end
   always @(posedge clk)if(re)qr<={metadata[raddr],heights[raddr]};
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
