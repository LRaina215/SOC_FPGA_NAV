"""Keep exact SDP behavior while mapping four full-size distributed stores.

Only the 16384x1 traversability bitmap and the three 512x32 mailbox stores
change. The latter describe two narrow planes; Gowin may merge them into one
32-bit source leaf, then physically split it into two 16-bit blocks. Both
source and physical mappings must be checked. Other branches stay exact.
"""
from pathlib import Path

ANCHOR = ' generate if(INIT_FILE=="" && DW==36 && AW==12 && DEPTH==4096)begin:g_slf_point_planes'
BRANCHES = ''' generate if(INIT_FILE=="" && DW==1 && AW==14 && DEPTH==16384)begin:g_bitmap_block
   (* syn_ramstyle="block_ram",syn_romstyle="block_rom" *)reg [0:0] bitmap[0:16383];
   reg [0:0] qr;integer i;
   initial begin for(i=0;i<16384;i=i+1)bitmap[i]=INIT_VAL;qr=0;end
   always @(posedge clk)if(we)bitmap[waddr]<=wdata;
   always @(posedge clk)if(re)qr<=bitmap[raddr];
   assign rdata=qr;
 end else if(INIT_FILE=="" && DW==32 && AW==9 && DEPTH==512)begin:g_mailbox_planes
   (* syn_ramstyle="block_ram",syn_romstyle="block_rom" *)reg [15:0] low_mem[0:511];
   (* syn_ramstyle="block_ram",syn_romstyle="block_rom" *)reg [15:0] high_mem[0:511];
   reg [31:0] qr;integer i;
   initial begin
     for(i=0;i<512;i=i+1)begin low_mem[i]=INIT_VAL[15:0];high_mem[i]=INIT_VAL[31:16];end
     qr=0;
   end
   always @(posedge clk)if(we)begin low_mem[waddr]<=wdata[15:0];high_mem[waddr]<=wdata[31:16];end
   always @(posedge clk)if(re)qr<={high_mem[raddr],low_mem[raddr]};
   assign rdata=qr;
 end else if(INIT_FILE=="" && DW==36 && AW==12 && DEPTH==4096)begin:g_slf_point_planes'''


def transform(text):
    assert text.count(ANCHOR) == 1, 'requires the frozen point-plane source'
    assert 'g_bitmap_block' not in text and 'g_mailbox_planes' not in text
    result = text.replace(ANCHOR, BRANCHES, 1)
    assert result.replace(BRANCHES, ANCHOR, 1) == text
    return result


def generate(path):
    path = Path(path)
    path.write_text(transform(path.read_text(encoding='utf-8')), encoding='utf-8', newline='\n')


def transform_selected(text):
    """Opt in at the mailbox instances; candidate/result stores stay default."""
    result=transform(text)
    header='parameter INIT_FILE="",parameter [DW-1:0] INIT_VAL={DW{1\'b0}}'
    changed=header+',parameter integer BLOCK_MAILBOX=0'
    condition='INIT_FILE=="" && DW==32 && AW==9 && DEPTH==512'
    assert result.count(header)==result.count(condition)==1
    result=result.replace(header,changed).replace(condition,'BLOCK_MAILBOX && '+condition)
    assert result.replace(changed,header).replace('BLOCK_MAILBOX && ','')==transform(text)
    return result


def select_mailboxes(text):
    result=text
    for name in ('u_injb','u_txb','u_txs'):
        old='nav_ram_sdp #(.DW(32), .AW(TX_WAW)) '+name
        new='nav_ram_sdp #(.DW(32), .AW(TX_WAW), .BLOCK_MAILBOX(1)) '+name
        assert result.count(old)==1
        result=result.replace(old,new)
    assert result.replace(', .BLOCK_MAILBOX(1)','')==text
    return result
