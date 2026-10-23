"""Limit the measured bank-read data selector at its actual register driver.

This is distinct from the ineffective SLF rb_ra wire attribute. The selected
register drives a data mux, not clock/reset/enable pins. No RTL operation,
initialization, address, storage capacity, read latency or hold changes.
"""
from pathlib import Path


def generate(path, maximum=128):
    path = Path(path)
    original = path.read_text(encoding='utf-8')
    old = 'reg [BW-1:0] selected;'
    new = f'reg [BW-1:0] selected /* synthesis syn_maxfan={maximum} */;'
    assert maximum == 128 and original.count(old) == 1
    changed = original.replace(old, new)
    assert changed.replace(new, old) == original
    path.write_text(changed, encoding='utf-8', newline='\n')
