"""Pack the full 4096x36 point store into eight 1024x18 BSRAM leaves.

The old 512x36 banks each become two half-depth physical blocks when the
required wide-memory conversion is enabled. Two explicit 18-bit planes use
the full depth, without changing a value, address, enable or read edge.
"""
from pathlib import Path

MARKER = ' generate if(INIT_FILE=="" && DEPTH>(1<<BAW))begin:g_banked\n'
BRANCH = ''' generate if(INIT_FILE=="" && DW==36 && AW==12 && DEPTH==4096)begin:g_slf_point_planes
   wire [35:0] point_q[0:3];
   reg [1:0] point_selected /* synthesis syn_maxfan=128 */;
   initial point_selected=0;
   always @(posedge clk)if(re)point_selected<=raddr[11:10];
   genvar pb;
   for(pb=0;pb<4;pb=pb+1)begin:g_point_bank
     (* syn_ramstyle="block_ram",syn_romstyle="block_rom" *)reg [17:0] low_mem[0:1023];
     (* syn_ramstyle="block_ram",syn_romstyle="block_rom" *)reg [17:0] high_mem[0:1023];
     reg [35:0] qr;integer i;
     initial begin
       for(i=0;i<1024;i=i+1)begin low_mem[i]=INIT_VAL[17:0];high_mem[i]=INIT_VAL[35:18];end
       qr=0;
     end
     always @(posedge clk)if(we && waddr[11:10]==pb)begin
       low_mem[waddr[9:0]]<=wdata[17:0];high_mem[waddr[9:0]]<=wdata[35:18];
     end
     always @(posedge clk)if(re && raddr[11:10]==pb)qr<={high_mem[raddr[9:0]],low_mem[raddr[9:0]]};
     assign point_q[pb]=qr;
   end
   assign rdata=point_q[point_selected];
 end else if(INIT_FILE=="" && DEPTH>(1<<BAW))begin:g_banked
'''


def transform(text):
    assert text.count(MARKER) == 1 and 'g_slf_point_planes' not in text
    assert 'g_slf_row_banks' in text and 'syn_maxfan=128' in text
    changed = text.replace(MARKER, BRANCH, 1)
    assert changed.replace(BRANCH, MARKER, 1) == text
    return changed


def generate(path):
    path = Path(path)
    path.write_text(transform(path.read_text(encoding='utf-8')), encoding='utf-8', newline='\n')
