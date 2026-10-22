"""Precompute frame-static 32-bit terms, preserving every coverage cycle.

Keep original wrapped signed arithmetic and state transitions. Only writes a
candidate copy; no observed support, timestamps or storage capacities change.
"""
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]


def generate(target):
    s=(ROOT/'rtl/map/support_coverage.v').read_text(encoding='utf-8')
    def replace(old,new):
        nonlocal s
        assert s.count(old)==1,old
        s=s.replace(old,new)
    replace(' reg pending_frame,cap_valid;', ''' reg signed [31:0] roll_base_x,roll_base_y,old_min_x,old_max_x,old_min_y,old_max_y;
 reg pending_frame,cap_valid;''')
    replace("wire signed [31:0] roll_x=cx-32'sd16+$signed({1'b0,ptr[8:4]});",
            "wire signed [31:0] roll_x=roll_base_x+$signed({1'b0,ptr[8:4]});")
    replace("wire signed [31:0] roll_y=cy-32'sd16+$signed({1'b0,ptr[13:9]});",
            "wire signed [31:0] roll_y=roll_base_y+$signed({1'b0,ptr[13:9]});")
    replace('wire exposed=roll_x<oldcx-16 || roll_x>=oldcx+16 || roll_y<oldcy-16 || roll_y>=oldcy+16;',
            'wire exposed=roll_x<old_min_x || roll_x>=old_max_x || roll_y<old_min_y || roll_y>=old_max_y;')
    replace('state<=CLR;ptr<=0;cx<=0;cy<=0;oldcx<=0;oldcy<=0;', '''state<=CLR;ptr<=0;cx<=0;cy<=0;oldcx<=0;oldcy<=0;
   roll_base_x<=-32'sd16;roll_base_y<=-32'sd16;
   old_min_x<=-32'sd16;old_max_x<=32'sd16;old_min_y<=-32'sd16;old_max_y<=32'sd16;''')
    replace('oldcx<=cx;oldcy<=cy;cx<=nextcx;cy<=nextcy;', '''oldcx<=cx;oldcy<=cy;cx<=nextcx;cy<=nextcy;
       roll_base_x<=nextcx-32'sd16;roll_base_y<=nextcy-32'sd16;
       old_min_x<=cx-32'sd16;old_max_x<=cx+32'sd16;
       old_min_y<=cy-32'sd16;old_max_y<=cy+32'sd16;''')
    Path(target).write_text(s,encoding='utf-8',newline='\n')
