// Full CPU SRAM, unchanged single-port normal-write / read-hold semantics.
module nav_ram_sp #(
 parameter integer DW=8,AW=10,DEPTH=(1<<AW),parameter INIT_FILE=""
)(input wire clk,we,input wire [AW-1:0] addr,input wire [DW-1:0] wdata,
 output reg [DW-1:0] rdata);
 (* syn_ramstyle="block_ram" *)reg [DW-1:0] mem[0:DEPTH-1];integer i;
 initial begin
   if(INIT_FILE!="")$readmemh(INIT_FILE,mem);
   else for(i=0;i<DEPTH;i=i+1)mem[i]=0;
   rdata=0;
 end
 always @(posedge clk)begin
   if(we)mem[addr]<=wdata;
   else rdata<=mem[addr];
 end
endmodule
