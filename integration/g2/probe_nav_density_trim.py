"""Targeted candidate-cap arithmetic/guard checks; never runs Gowin."""
import argparse
import json
from pathlib import Path
import shutil
import sys
import random

from prepare_lf_line_compact import simulate
from prepare_nav_density_trim import PROFILES,MATCH,TRAJ

ROOT=Path(__file__).resolve().parents[2]

def probe(source,out,tier):
    source,out=Path(source).resolve(),Path(out).resolve();assert not out.exists();out.mkdir()
    p=PROFILES[tier];results=[]
    # Original complete map/matcher data; only scored candidate prefix is reduced.
    old=source.parent/'ddr_complete_matcher_dirty_probe1'
    slf=out/'matcher';slf.mkdir()
    for name in ('pts.hex','frm.hex','cand.hex','res.hex','sm.hex','smm.hex','prior.hex','pres.hex','sin_lut.hex'):
        shutil.copyfile(old/name,slf/name)
    counts=(old/'counts.txt').read_text().split();assert counts[:2]==['5','245'];counts[1]=str(1<<p['match_log']);(slf/'counts.txt').write_text(' '.join(counts)+'\n')
    rels=['nav/rtl/common/nav_ram_sdp.v','nav/rtl/match/slf_extract.v','nav/rtl/match/slf_lf.v',MATCH]
    for rel in rels:shutil.copyfile(source/rel,slf/Path(rel).name)
    tb=(ROOT/'rtl/match/tb/tb_slf.sv').read_text(encoding='utf-8')
    # Public invalid high address must return zero and hold; cannot alias a low slot.
    end='        if (errs == 0) $display("TEST PASSED");'
    check=f'''        @(posedge clk); res_re <= 1; res_ra <= {2*(1<<p['match_log'])};
        @(posedge clk); res_re <= 0; #1;
        if (res_q !== 32'd0) begin errs++; $display("invalid result address aliased"); end
        repeat(2) @(posedge clk); #1;
        if (res_q !== 32'd0) begin errs++; $display("invalid result zero did not hold"); end
'''
    assert tb.count(end)==1;tb=tb.replace(end,check+end);(slf/'tb.sv').write_text(tb,encoding='utf-8')
    result=simulate(slf,[slf/Path(r).name for r in rels]+[slf/'tb.sv'],'tb_slf')
    result.update(scope=f'Original complete SLF five-frame/full327680-map-word and prior test; first {1<<p["match_log"]} original candidates retain exact selected-candidate arithmetic; invalid result address zero',candidate_cap=1<<p['match_log'])
    results.append(result)
    # Native fixed point model generates boundary-count trajectories.
    sys.path.insert(0,str(ROOT/'model'));import nav_fixed as nf
    import numpy as np
    traj=out/'trajectory';traj.mkdir()
    rng=random.Random(27);cost=np.array([rng.choice([0,0,0,5,20,60,120,200,253,254,255]) for _ in range(16384)],dtype=np.int64)
    (traj/'cost.hex').write_text('\n'.join(f'{int(v):02x}' for v in cost)+'\n')
    lines=[];cap=1<<p['traj_log']
    for r,(K,T) in enumerate([(cap,20),(cap,255),(1,1),(max(2,cap-1),64)]):
        cx,cy=[(0,0),(-75,100),(120,-90),(500,-500)][r];x0,y0=(cx<<8)+r*100,(cy<<8)-r*100;th=r*16001;cmax=253+r%2
        cands=[(rng.randint(-400,600),rng.randint(-180,180),rng.randint(-1600,1600)) for _ in range(K)]
        answer=nf.traj_eval(cost,cx,cy,x0,y0,th,cands,T,cmax)
        lines += [f'{v&0xffffffff:08x}' for v in [cx,cy,x0,y0,th,T,K,cmax]]
        for dvx,dvy,dth in cands:lines += [f'{((dvy&65535)<<16)|(dvx&65535):08x}',f'{dth&65535:08x}']
        for mx,nun,cs,coll,left,csum,ex,ey in answer:lines += [f'{(left<<25)|(coll<<24)|(cs<<16)|(nun<<8)|mx:08x}',f'{csum&0xffffffff:08x}',f'{ex&0xffffffff:08x}',f'{ey&0xffffffff:08x}']
    (traj/'runs.hex').write_text('\n'.join(lines)+'\n');(traj/'counts.txt').write_text('4\n')
    for rel in ['nav/rtl/common/nav_ram_sdp.v',TRAJ]:shutil.copyfile(source/rel,traj/Path(rel).name)
    shutil.copyfile(source/'sin_lut.hex',traj/'sin_lut.hex')
    tb=(ROOT/'rtl/plan/tb/tb_traj_eval.sv').read_text(encoding='utf-8')
    check=f'''        @(posedge clk); res_re <= 1; res_addr <= {4*cap};
        @(posedge clk); res_re <= 0; @(posedge clk); #1;
        if (res_rdata !== 32'd0) begin errs++; $display("invalid trajectory result address aliased"); end
'''
    tb=tb.replace(end,check+end);(traj/'tb.sv').write_text(tb,encoding='utf-8')
    result=simulate(traj,[traj/'nav_ram_sdp.v',traj/'traj_eval.v',traj/'tb.sv'],'tb_traj_eval')
    result.update(scope=f'4 fixed-point trajectory boundary runs including max {cap} candidates and T255, plus invalid result address zero',candidate_cap=cap)
    results.append(result)
    state=dict(passed=True,selected_matcher_candidate_arithmetic_passed=True,trajectory_cap_and_exact_arithmetic_passed=True,invalid_result_address_returns_zero=True,source=str(source),tier=tier,results=results,full_navigation_quality_accepted=False,whole_joint_timing_or_routing_accepted=False)
    (out/'result.json').write_text(json.dumps(state,indent=2)+'\n');return state

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',required=True);p.add_argument('--out',required=True);p.add_argument('--tier',choices=PROFILES,required=True);a=p.parse_args();print(json.dumps(probe(a.source,a.out,a.tier)))
