"""Read back Gowin's effective options before starting an expensive build."""
from pathlib import Path
import re
import subprocess


def validate(tool,out,settings,frequency,wide_split,extra_expected=None):
    out=Path(out);dump=out/'effective_options.tcl';script=out/'preflight.tcl'
    tail=f'saveto -all_options {{{dump.as_posix()}}}'
    # Some gw_sh sessions remain in their event loop after saveto.  Exit
    # explicitly and bound this settings-only operation; never bound PNR here.
    script.write_text('\n'.join(settings+[tail,'exit'])+'\n',encoding='utf-8',newline='\n')
    with (out/'preflight.log').open('w',encoding='utf-8') as log:
        subprocess.run([str(tool),script.name],cwd=out,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=60)
    text=dump.read_text(encoding='utf-8')
    def option(name):
        m=re.search(r'^set_option\s+-'+name+r'\s+(.+?)\s*$',text,re.M)
        if not m:raise ValueError('missing actual option '+name)
        return m[1].strip('{}" ')
    actual_frequency=float(option('global_freq'))
    actual_split=int(option('convert_sdp32_36_to_sdp16_18'))
    assert actual_frequency==frequency,(actual_frequency,frequency)
    assert actual_split==wide_split,(actual_split,wide_split)
    extras={}
    for name,expected in (extra_expected or {}).items():
        extras[name]=int(option(name));assert extras[name]==expected,(name,extras[name],expected)
    return dict(global_freq_mhz=actual_frequency,wide_split=actual_split,effective_options=str(dump),extra_options=extras)
