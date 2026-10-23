"""Use native BSRAM for the frozen, initialized navigation lookup tables.

Only the storage attribute changes. The original read/write/initial/hold RTL
is copied verbatim into a separate generate branch. Never touches control.
"""
from pathlib import Path


def transform(text):
    marker=' end else begin:g_distributed\n'
    assert text.count(marker)==1 and 'g_init_block' not in text
    body=text.split(marker,1)[1].rsplit(' end endgenerate',1)[0]
    old='(* syn_ramstyle="distributed_ram",syn_romstyle="distributed_rom" *)'
    new='(* syn_ramstyle="block_ram",syn_romstyle="block_rom" *)'
    assert body.count(old)==1
    branch=' end else if(INIT_FILE!="")begin:g_init_block\n'+body.replace(old,new,1)
    text=text.replace(marker,branch+marker,1)
    return text


def generate(path):
    path=Path(path)
    path.write_text(transform(path.read_text(encoding='utf-8')),encoding='utf-8',newline='\n')
