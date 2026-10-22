// Keep the full frame buffer and calibrated ray LUT in explicit block banks.
module nav_ram_banked #(
 parameter integer DW=16,DEPTH=19200,BAW=10,AW=$clog2(DEPTH)
)(input wire clk,we,input wire [AW-1:0] waddr,input wire [DW-1:0] wdata,
 input wire re,input wire [AW-1:0] raddr,output wire [DW-1:0] rdata);
 localparam integer NB=(DEPTH+(1<<BAW)-1)>>BAW;
 localparam integer BW=(NB>1)?$clog2(NB):1;
 wire [DW-1:0] q[0:NB-1];reg [BW-1:0] selected;
 initial selected=0;
 always @(posedge clk)if(re)selected<=raddr[AW-1:BAW];
 genvar b;
 generate for(b=0;b<NB;b=b+1)begin:g_bank
   (* syn_ramstyle="block_ram",syn_romstyle="block_rom" *)reg [DW-1:0] mem[0:(1<<BAW)-1];
   reg [DW-1:0] qr;integer i;
   initial begin for(i=0;i<(1<<BAW);i=i+1)mem[i]=0;qr=0;end
   always @(posedge clk)if(we && waddr[AW-1:BAW]==b)mem[waddr[BAW-1:0]]<=wdata;
   always @(posedge clk)if(re && raddr[AW-1:BAW]==b)qr<=mem[raddr[BAW-1:0]];
   assign q[b]=qr;
 end endgenerate
 assign rdata=q[selected];
endmodule
