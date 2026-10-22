"""Exact intermediate widths for the four private action-map gains.

No quantization: clip gives [-4096,4096], |gain|<=600, magnitude product is
at most 2457600 and rounded magnitude at most 600. Output widths stay intact.
"""
from pathlib import Path
import re
CONTROL=Path(__file__).resolve().parents[1]


def generate(target):
    s=(CONTROL/'fpga/src/policy_action_map.v').read_text(encoding='utf-8')
    assert re.findall(r'scale\(actions\[[^\]]+\],(-?\d+)\)',s)==['-600','600','19','-19','19','19']
    old='''        reg signed [63:0] product;
        begin
            product=$signed(clip(a))*gain;
            scale=product<0 ? -((-product+2048)>>>12) : ((product+2048)>>>12);
        end'''
    new='''        reg signed [23:0] bounded;
        reg [12:0] magnitude;
        reg [9:0] gain_magnitude;
        reg [22:0] product,rounded_product;
        reg [10:0] rounded;
        reg negative;
        begin
            bounded=clip(a);
            // |bounded| <= 4096. Negation needs only its low 13 bits.
            magnitude=bounded[23] ? (~bounded[12:0]+13'd1) : bounded[12:0];
            gain_magnitude= gain<0 ? -gain : gain;
            product=magnitude*gain_magnitude;
            rounded_product=product+23'd2048;
            rounded=rounded_product[22:12];
            negative=bounded[23]^(gain<0);
            scale=negative ? -$signed({1'b0,rounded}) : $signed({1'b0,rounded});
        end'''
    assert s.count(old)==1
    Path(target).write_text(s.replace(old,new),encoding='utf-8',newline='\n')
