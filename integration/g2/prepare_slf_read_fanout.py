"""Bound the SLF row-buffer read-address data mux fanout in a build copy.

Only a synthesis attribute is added. No expression, storage, register,
initialization, enable, reset, or cycle changes are made.

The first full SYN experiment retained 275 address loads: this attribute did
not replicate the current RAM-input driver. Keep that negative result and do
not treat generation alone as a measured improvement or a PNR prerequisite.
"""
from pathlib import Path


def generate(path):
    path=Path(path)
    original=path.read_text(encoding='utf-8')
    old='wire [UW-1:0] rb_ra;'
    new='wire [UW-1:0] rb_ra /* synthesis syn_maxfan=64 */;'
    assert original.count(old)==1 and 'syn_maxfan=64' not in original
    result=original.replace(old,new)
    assert result.replace(new,old)==original
    path.write_text(result,encoding='utf-8',newline='\n')
