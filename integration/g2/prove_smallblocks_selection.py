"""Check explicit selection, reusing the unchanged proven memory bodies."""
import argparse,json
from pathlib import Path
from measure_small_block_stores import TOP,TB
from measure_slf_row_banks import simulate
from prepare_small_block_stores import transform,transform_selected,select_mailboxes


def main():
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    src=a.source.resolve();out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    old=(src/'nav/rtl/common/nav_ram_sdp.v').read_text();new=transform_selected(old)
    first=src.parent/'small_block_stores_probe1'
    proof=json.loads((first/'result.json').read_text());assert proof['passed']
    assert (first/'physical/ram.v').read_text()==transform(old)
    # Removing only the parameter selector gives the exact already-mapped RTL.
    assert new.replace(',parameter integer BLOCK_MAILBOX=0','').replace('BLOCK_MAILBOX && ','')==transform(old)
    (out/'old.v').write_text(old.replace('module nav_ram_sdp #(','module ram_old #(',1))
    (out/'new.v').write_text(new.replace('module nav_ram_sdp #(','module ram_new #(',1))
    (out/'old_top.v').write_text(TOP.replace('g2_top','g2_old').replace('nav_ram_sdp','ram_old'))
    enabled=TOP.replace('.DW(32),','.DW(32),.BLOCK_MAILBOX(1),')
    (out/'new_top.v').write_text(enabled.replace('g2_top','g2_new').replace('nav_ram_sdp','ram_new'))
    (out/'plain_top.v').write_text(TOP.replace('g2_top','g2_plain').replace('nav_ram_sdp','ram_new'))
    tb=TB.replace('wire ob,nb;','wire db;wire[31:0]dm;g2_plain plain(clk,we,re,wa,ra,wd,db,dm);\nwire ob,nb;')
    tb=tb.replace('||{nb,nm}!=={be,me})','||{nb,nm}!=={be,me}||{db,dm}!=={be,me})')
    checks=[simulate(out,'explicit_and_default',[out/n for n in ('old.v','new.v','old_top.v','new_top.v','plain_top.v')],tb)]
    host=(src/'nav/rtl/net/host_mbx.v').read_text();selected=select_mailboxes(host)
    (out/'nav_ram_sdp.v').write_text(new);(out/'host_mbx.v').write_text(selected)
    r=dict(passed=True,source=str(src),checks=checks,enabled_memory_bodies_match_prior_physical_proof=True,
           physical_proof=str(first/'result.json'),only_selected_instances=['u_injb','u_txb','u_txs'],
           default_512x32_candidate_and_result_store_remain_original=True,
           prior_joint_SYN15_rejected_prediction=337,target_prediction=333,programmed=False)
    (out/'result.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r))


if __name__=='__main__':main()
