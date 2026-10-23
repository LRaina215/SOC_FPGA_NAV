"""Apply only the reviewed SGMII copper-link admission check to frozen input."""
from pathlib import Path


def generate(path):
    p=Path(path);s=p.read_text(encoding='utf-8')
    old='wire link_complete=status[0] && status[1] && (!cfg[1] || status[2]);'
    new='wire link_complete=status[0] && status[1] && (!cfg[1] || status[2]) && (!cfg[0] || status[8]);'
    assert s.count(old)==1
    p.write_text(s.replace(old,new),encoding='utf-8',newline='\n')
