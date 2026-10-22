"""Validate timing model against observed RTL cycles and reference gate counts."""
from pathlib import Path
import re
import sys
import json
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'model'))
from slf_timing import evaluation_cycles


def main(path):
    p=Path(path)
    n,k,prior_words,*_=map(int,(p/'counts.txt').read_text().split())
    rexp=[int(x,16) for x in (p/'res.hex').read_text().split()]
    pexp=[int(x,16) for x in (p/'pres.hex').read_text().split()]
    log=p/'xsim_stdout.log' if (p/'xsim_stdout.log').exists() else p/'xsim.log'
    body=log.read_text(errors='replace')
    if 'TEST PASSED' not in body:raise ValueError('Timing requires a completed functional test')
    # Old runner captured stdout into the simulator's own log filename, which
    # could repeat the footer. Only the completed test prefix is relevant.
    observed=re.findall(r'SLF_TIMING points=(\d+) candidates=(\d+) cycles=(\d+)',body.split('TEST PASSED')[0])
    np_=len(pexp)//512
    if len(observed)!=n+np_:raise ValueError('Expected exactly one timing row per evaluation')
    rows=[]
    for i,(raw,cands,cycles) in enumerate(observed):
        raw,cands,cycles=map(int,(raw,cands,cycles))
        stride=max(1,(raw+1023)//1024);samples=(raw+stride-1)//stride
        valid=(rexp[i*512+1] if i<n else pexp[(i-n)*512+1])&0xffff
        expected=evaluation_cycles(samples,valid,cands)
        assert cycles==expected,(i,cycles,expected,raw,samples,valid)
        rows.append(dict(raw_points=raw,sampled_points=samples,valid=valid,candidates=cands,
                         observed_cycles=cycles,predicted_cycles=expected,us_at_50MHz=cycles/50))
    (p/'timing.json').write_text(json.dumps(dict(pass_=True,rows=rows,
        scope='evaluation only; CPU, bus, structure update and map fusion are separate'),indent=2))
    print('SLF timing: %d/%d exact cycle matches' % (len(rows),len(rows)))


if __name__=='__main__':main(sys.argv[1])
