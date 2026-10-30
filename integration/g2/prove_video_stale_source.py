"""Induction step for registering video stale at the unchanged age threshold."""
import argparse,json
from pathlib import Path
from prove_soc_console_remap import z3

def prove(source,out):
    text=(source/'video/g2_camera_lcd.v').read_text(encoding='utf-8')
    assert 'reg stale;'in text
    assert 'age<=FRESH_CYCLES;stale<=1;'in text
    assert 'if(request_sync!=request_previous)begin age<=0;stale<=0;end'in text
    assert 'else if(age<FRESH_CYCLES)begin age<=age+1\'b1;stale<=age>=FRESH_CYCLES-1;end'in text
    age=z3.Int('age');fresh=z3.Int('fresh');event=z3.Bool('event');stale=z3.Bool('stale')
    next_age=z3.If(event,0,z3.If(age<fresh,age+1,age))
    next_stale=z3.If(event,False,z3.If(age<fresh,age>=fresh-1,stale))
    solver=z3.Solver();solver.add(fresh>=1,age>=0,age<=fresh,stale==(age>=fresh))
    solver.add(next_stale!=(next_age>=fresh));result=solver.check();assert result==z3.unsat
    value=dict(passed=True,result=str(result),scope='Source age/stale invariant and reset, any positive threshold/all reachable ages; not mapped CDC or physical timing',
        reset_age_equals_threshold=True,reset_stale=True,extra_threshold_latency_cycles=0)
    out.write_text(json.dumps(value,indent=2),encoding='utf-8');return value

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();print(json.dumps(prove(a.source,a.out)))
