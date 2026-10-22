// Explicit storage classes: large writable arrays are banked block RAM;
// small arrays / lookup ROMs stay distributed, avoiding automatic repromotion.
module nav_ram_sdp #(
 parameter integer DW=16,AW=10,DEPTH=(1<<AW),
 parameter INIT_FILE="",parameter [DW-1:0] INIT_VAL={DW{1'b0}}
)(input wire clk,we,input wire [AW-1:0] waddr,input wire [DW-1:0] wdata,
 input wire re,input wire [AW-1:0] raddr,output wire [DW-1:0] rdata);
 localparam integer BAW=DW<=1?14:DW<=2?13:DW<=4?12:DW<=9?11:DW<=18?10:DW<=36?9:10;
 generate if(INIT_FILE=="" && DEPTH>(1<<BAW))begin:g_banked
   localparam integer NB=(DEPTH+(1<<BAW)-1)>>BAW;
   localparam integer BW=$clog2(NB);
   wire [DW-1:0] q[0:NB-1];reg [BW-1:0] selected;
   initial selected=0;
   always @(posedge clk)if(re)selected<=raddr[AW-1:BAW];
   genvar b;
   for(b=0;b<NB;b=b+1)begin:g_bank
     (* syn_ramstyle="block_ram",syn_romstyle="block_rom" *)reg [DW-1:0] mem[0:(1<<BAW)-1];
     reg [DW-1:0] qr;integer i;
     initial begin for(i=0;i<(1<<BAW);i=i+1)mem[i]=INIT_VAL;qr=0;end
     always @(posedge clk)if(we && waddr[AW-1:BAW]==b)mem[waddr[BAW-1:0]]<=wdata;
     always @(posedge clk)if(re && raddr[AW-1:BAW]==b)qr<=mem[raddr[BAW-1:0]];
     assign q[b]=qr;
   end
   assign rdata=q[selected];
 end else begin:g_distributed
   (* syn_ramstyle="distributed_ram",syn_romstyle="distributed_rom" *)reg [DW-1:0] mem[0:DEPTH-1];
   reg [DW-1:0] qr;integer i;
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
