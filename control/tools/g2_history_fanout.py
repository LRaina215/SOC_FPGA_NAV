"""Bound the observation-history data selector's fanout in a build copy.

The initialized register and its consumers keep their original RTL behavior.
This attribute targets data-mux loads only; it does not constrain reset/CE.
"""
from pathlib import Path


def generate(path, maximum=128):
    path=Path(path);source=path.read_text(encoding='utf-8')
    before='    output reg initialized,'
    after=f'    output reg initialized /* synthesis syn_maxfan = {maximum} */,'
    assert maximum>=8 and source.count(before)==1 and 'syn_maxfan' not in source
    result=source.replace(before,after)
    assert result.replace(after,before)==source
    path.write_text(result,encoding='utf-8',newline='\n')
