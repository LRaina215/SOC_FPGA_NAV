"""Check the restricted shared-reader fix on the actual six VO cell blocks.

Only stream/extract the three attributed instances in PNR53 and PNR54. The
whole parse_vo/full graph is never called. No EDA, simulation or source edit.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re

from prepare_baseline_fusion_retime import metadata
from review_lease_tree_pnr50_final import CELL, IO_ATTRIBUTE
from review_pnr53_54_attributed_IO_FF import BASE,TARGETS,physical_stream,OLD_VO_CELL


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args();assert not a.out.exists()
    rows=[];bindings=[]
    for build in('SLF128_storage_place1_pnr53','SLF128_storage_place2_pnr54'):
        p=BASE/build/'impl/pnr/g2_top.vo';bindings.append(metadata(p))
        cells,inventory=physical_stream(p,set(TARGETS))
        assert {x['name']for x in inventory['all_attributed_primitives']}==set(TARGETS)
        assert inventory['all_attributed_primitives_count']==3
        for n in TARGETS:
            c=cells[n];raw=c['raw']
            assert len(OLD_VO_CELL.findall(raw))==0
            new=CELL.findall(raw);assert len(new)==1 and len(new[0])==3
            plain=raw.replace(c['attribute']+' ','',1)
            old_plain=OLD_VO_CELL.findall(plain)
            assert len(old_plain)==1 and old_plain==new==CELL.findall(plain)
            assert len(re.findall(r'^'+IO_ATTRIBUTE+r'DFF\w*\s+(?:\\\S+|[^\s(]+)\s*\(',raw,re.M))==1
            rows.append(dict(build=build,instance=n,line=c['line'],exact_physical_head=raw.splitlines()[0],
                old_parser_matches=0,new_parser_matches=1,kind=new[0][0],raw_name=new[0][1],
                same3_capture_groups_and_exact_port_body_as_plain_original=True,
                independent_DFF_head_count_now_includes_attribute=True))
    # The reader deliberately admits no arbitrary attributes/expressions. An
    # unsupported annotation must be rejected by the pre-parse head check.
    allowed=re.compile(r'^'+IO_ATTRIBUTE+r'\w+\s+(?:\\\S+|[^\s(]+)\s*\(')
    rejected=[]
    for prefix in('(*unknown = "FALSE" *)','(*gowin_io_reg = "OTHER" *)',
                   '(*gowin_io_reg = secret_expression + 1 *)','(*gowin_io_reg = "FALSE", other = 1 *)'):
        test=prefix+' DFFRE example ('
        assert not allowed.match(test)
        rejected.append(test)
    assert bindings==[metadata(Path(x['path']))for x in bindings]
    reader=Path(__file__).with_name('review_lease_tree_pnr50_final.py')
    result=dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),
        checks='Original attributed6 physical cell blocks only; no full parse_vo, graph, EDA or simulator',
        reader=metadata(reader),checker=metadata(Path(__file__).resolve()),
        restricted_prefix=IO_ATTRIBUTE,CELL_capture_groups=CELL.groups,
        actual_six_rows=rows,unsupported_attribute_examples_rejected=rejected,
        input_bindings=bindings,source_RTL_or_frozen_VO_changed=False,
        full54_clock_stop_review_still_root_owned=True,physical_timing_accepted=False)
    a.out.parent.mkdir(parents=True,exist_ok=True)
    a.out.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(dict(passed=True,actual_cells=6,exact_original_port_bodies_preserved=True,
        full_graph_loaded=False,out=str(a.out)),ensure_ascii=False))


if __name__=='__main__':main()
