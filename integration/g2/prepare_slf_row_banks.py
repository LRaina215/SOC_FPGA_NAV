"""Partition only the measured 160x98 SLF row store, without another cycle.

The registered bank index and each bank's read register update on the original
read edge. Reads still return the pre-write value, and re=0 holds all outputs.
No padding, BSRAM promotion, reset, frame logic or control change is introduced.
"""
from pathlib import Path

BRANCH = ''' end else if(INIT_FILE=="" && DW==98 && AW==8 && DEPTH==160)begin:g_slf_row_banks
   localparam integer ROW_AW=4,ROW_DEPTH=16,ROW_BANKS=10;
   wire [DW-1:0] row_q[0:ROW_BANKS-1];
   reg [3:0] row_selected /* synthesis syn_maxfan=128 */;
   initial row_selected=0;
   always @(posedge clk)if(re)row_selected<=raddr[7:4];
   genvar rb;
   for(rb=0;rb<ROW_BANKS;rb=rb+1)begin:g_row_bank
     (* syn_ramstyle="distributed_ram",syn_romstyle="distributed_rom" *)reg [DW-1:0] mem[0:ROW_DEPTH-1];
     reg [DW-1:0] qr;integer i;
     initial begin for(i=0;i<ROW_DEPTH;i=i+1)mem[i]=INIT_VAL;qr=0;end
     always @(posedge clk)if(we && waddr[7:4]==rb)mem[waddr[3:0]]<=wdata;
     always @(posedge clk)if(re && raddr[7:4]==rb)qr<=mem[raddr[3:0]];
     assign row_q[rb]=qr;
   end
   assign rdata=row_q[row_selected];
'''


def transform(text):
    marker = ' end else begin:g_distributed\n'
    assert text.count(marker) == 1 and 'g_slf_row_banks' not in text
    changed = text.replace(marker, BRANCH+marker, 1)
    assert changed.replace(BRANCH, '', 1) == text
    return changed


def generate(path):
    path = Path(path)
    path.write_text(transform(path.read_text(encoding='utf-8')), encoding='utf-8', newline='\n')
