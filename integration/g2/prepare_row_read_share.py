"""Share the row store's output register without changing its read edge.

Only the 160x98 distributed SLF row store changes. Ten independent 16-word
write banks remain. A decoded combinational read is sampled by one register,
instead of sampling ten banks then selecting one registered value. No new
clock, reset, memory capacity, valid cycle or block RAM is introduced.
"""
from pathlib import Path
from prepare_slf_row_banks import BRANCH


SHARED = ''' end else if(INIT_FILE=="" && DW==98 && AW==8 && DEPTH==160)begin:g_slf_row_banks
   localparam integer ROW_DEPTH=16,ROW_BANKS=10;
   wire [DW-1:0] row_q[0:ROW_BANKS-1];
   wire [ROW_BANKS-1:0] row_read_bank /* synthesis syn_keep=1 syn_maxfan=128 */;
   reg [DW-1:0] row_mux,qr;
   integer rb_select;
   initial qr=0;
   genvar rb;
   for(rb=0;rb<ROW_BANKS;rb=rb+1)begin:g_row_bank
     (* syn_ramstyle="distributed_ram",syn_romstyle="distributed_rom" *)reg [DW-1:0] mem[0:ROW_DEPTH-1];
     integer i;
     initial for(i=0;i<ROW_DEPTH;i=i+1)mem[i]=INIT_VAL;
     always @(posedge clk)if(we && waddr[7:4]==rb)mem[waddr[3:0]]<=wdata;
     assign row_q[rb]=mem[raddr[3:0]];
     assign row_read_bank[rb]=(raddr[7:4]==rb);
   end
   always @* begin
     row_mux=0;
     for(rb_select=0;rb_select<ROW_BANKS;rb_select=rb_select+1)
       row_mux=row_mux | (row_q[rb_select] & {DW{row_read_bank[rb_select]}});
     // The original bank array has no defined word for addresses160..255.
     if(raddr[7:4]>=ROW_BANKS)row_mux={DW{1'bx}};
   end
   always @(posedge clk)if(re)qr<=row_mux;
   assign rdata=qr;
'''


def transform(text):
    assert text.count(BRANCH)==1, 'requires the measured ten-bank row store'
    changed=text.replace(BRANCH,SHARED,1)
    assert changed.replace(SHARED,BRANCH,1)==text
    return changed


def generate(path):
    path=Path(path)
    path.write_text(transform(path.read_text(encoding='utf-8')),encoding='utf-8',newline='\n')
