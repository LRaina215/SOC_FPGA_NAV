"""Map the original exact 425x26 signed bias ROM to native block storage.

Only two storage attributes on bias_mem change. File, signed width, enabled
read statement, state machine, all values and inference timing stay unchanged.
"""
from pathlib import Path

OLD = '(* syn_ramstyle="distributed_ram",syn_romstyle="distributed_rom" *)reg signed [25:0] bias_mem [0:424];'
NEW = OLD.replace('distributed_ram','block_ram').replace('distributed_rom','block_rom')


def transform(text):
    assert text.count(OLD) == 1 and NEW not in text, 'unexpected original bias declaration'
    changed = text.replace(OLD,NEW,1)
    assert changed.replace(NEW,OLD,1) == text
    return changed


def generate(path):
    path=Path(path)
    path.write_text(transform(path.read_text(encoding='utf-8')),encoding='utf-8',newline='\n')
