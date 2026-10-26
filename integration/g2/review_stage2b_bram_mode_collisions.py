"""Read-only current stage2B consumer/mode risk inventory, no EDA or tests.

Vendor documentation is read in place (or an F-drive text extraction); no SDK
source or complete manual is copied into the repository. Conditions below are
source facts and proposals, not a substitute for actual mapped mode/bus proofs.
"""
from pathlib import Path
from datetime import datetime
import argparse,json,re,subprocess

BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'stage2B_bitmap_mailbox_slist_trig_console_bias_prepared1'
DOC=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/doc/EN/UG300-1.3.6E_Arora V BSRAM & SSRAM User Guide.pdf')
LIB=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
REVIEW=BASE/'stage2B_BSRAM_mode_collision_review1'

def metadata(p):
 s=p.stat();return dict(path=str(p),bytes=s.st_size,mtime_ns=s.st_mtime_ns)
def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def anchors(rel,needles):
 p=SOURCE/rel;lines=p.read_text(encoding='utf-8-sig').splitlines();rows=[]
 for needle in needles:
  matches=[dict(line=i+1,text=x.strip())for i,x in enumerate(lines)if needle in x]
  assert matches,(rel,needle);rows.append(dict(needle=needle,matches=matches))
 return dict(file=metadata(p),anchors=rows)

def review():
 REVIEW.mkdir(exist_ok=True);text=REVIEW/'UG300.txt'
 if not text.exists():subprocess.run(['D:/latex/texlive/2026/bin/windows/pdftotext.exe','-layout',str(DOC),str(text)],check=True)
 manual=text.read_text(encoding='utf8');pages=manual.split('\f')
 sdp=[(i+1,x)for i,x in enumerate(pages)if 'SDPB/SDPX9B Port A is for write operation'in x];assert len(sdp)==1
 sdp_text=' '.join(sdp[0][1].split())
 assert 'matching bits [13:4]'in sdp_text and'one port cannot perform a read operation'in sdp_text
 assert 'supporting Normal mode'in sdp_text
 assert 'Arora V 138K/75K/60K/25K/15K devices do not support Read-before-Write mode.'in manual
 lib=LIB.read_text(encoding='utf-8-sig');interfaces=[]
 for name in ['SDPB','SDPX9B','DPB','DPX9B']:
  body=re.search(r'\bmodule\s+'+name+r'\b.*?\bendmodule\b',lib,re.S)[0]
  modes={k:v.strip()for k,v in re.findall(r'parameter\s+(READ_MODE[01]?|WRITE_MODE[01]?)\s*=\s*([^;]+);',body)}
  interfaces.append(dict(primitive=name,mode_parameter_defaults=modes,has_WRITE_MODE_parameter=any(x.startswith('WRITE')for x in modes)))
 assert not interfaces[0]['has_WRITE_MODE_parameter']and not interfaces[1]['has_WRITE_MODE_parameter']
 pa=BASE/'sfp_lane2_ddr_maps_pnr16/gw_sh.log';errors=[x.strip()for x in pa.read_text(errors='replace').splitlines()if 'PA2122'in x]
 assert len(errors)==2 and all("WRITE_MODE0 = 2'b10"in x for x in errors)
 old=load(BASE/'small_block_stores_probe1/result.json')
 assert old['physical_run']['runtime']['tool_exit']==0
 oldvg=BASE/'small_block_stores_probe1/physical/impl/gwsynthesis/g2_top.vg'
 vg=oldvg.read_text();assert len(re.findall(r'^\s+SDPB\s+',vg,re.M))==2
 oldtcl=(oldvg.parents[2]/'run.tcl').read_text();assert 'set_device -device_version C GW5AT-LV138PG484AC1/I0'in oldtcl
 source_rows=[
  anchors('nav/rtl/net/host_mbx.v',['.we(inj_we & ~iwp[TX_WAW])','.re(i_re)','if (ia) begin','i_re <= 1\'b1; i_ra <= ioff','iwp <= 0','u_txs (','.we(tx_we & ~twp[TX_WAW])','.re(txb_re)','if (tx_we && !twp[TX_WAW])']),
  anchors('nav/rtl/top/nav_subsystem.v',[".txb_re(1'b1)",'B_IDLE: if (bus_req)','a_q <= {bus_addr[15:2]','if (bus_we) rf_wr','B_RF: begin','bus_rdata <= rf_rdata','B_END: bs <= B_IDLE']),
  anchors('nav/rtl/regs/nav_regfile.v',["p_MBX_TX_DATA = wr && (addr == 16'h1220)","r_MBX_TXB_ADDR <= 32'h00000000","16'h1240: rdata = i_MBX_TXB_Q"]),
  anchors('nav/rtl/soc/nav_soc.v',['wire req     = mem_valid && !mem_ready','wire        con_we = req','con_wp[1:0] == b',".re(1'b1)",'if (req && sel_per)','per_ready <= 1\'b1','con_wp <= con_wp + 1\'b1','con_ra <= d_addr[11:2]','d_st <= 2\'d1','2\'d1: d_st <= 2\'d3','d_rdata <= d_ram']),
  anchors('nav/rtl/map/trav_v3.v',['assign bm_re','flush <= 5\'d16','if (flush == 0)','bm_we <= 1\'b1']),
  anchors('nav/rtl/match/slf_match.v',['sl_we <= 1\'b1','sl_re <= 1\'b1'])]
 rows=[
  dict(store='bitmap',condition='Writes are issued only from delayed P_A/P_B; reads only during P_C/P_D. P_FLUSH counts16 through0 between phases.',status='Plausible source-level exclusion, parent companion review owns complete pipeline/physical-granule proof; not accepted merely from these anchors.',requirement='Prove no actual write-enable overlaps read-enable, including phase drain/reset, before mapping; then address granularity cannot create a cross-port conflict.'),
  dict(store='slot_list',condition='sl_we only in S_U3; sl_re only in S_UC0, both registered default0 every clock.',status='Distinct state phases with a read-wait gap; parent companion review owns complete reachable-state proof.',requirement='Prove sl_we && sl_re is unreachable at the RAM edge, including reset/pending registered control.'),
  dict(store='injection_mailbox',condition='Writer accepts inj_we independently of ia/iv1/iv2; i_re streams ioff. iwp resets at last-byte scheduling, before all read pipeline registers drain.',status='No hardware exclusion. A caller can write a physical conflict group while streaming, and following-packet writes must respect the drain.',requirement='Prove the actual mailbox bus/firmware producer never writes while read-active or drain; otherwise arbitrate/store accepted writes or change protocol without dropping/duplicating them. Current source-only read-first test is insufficient.'),
  dict(store='TX_shadow',condition='Actual top txb_re is tied1. RAM writes each TX_DATA transaction while twp not full; reset readback index0 and first TX word0 already conflict.',status='Concrete normal-use unwanted cross-port collision; must change before the one combined SYN.',needed_read_edges='For a TXB_Q read accepted at nav_subsystem B_IDLE edgeE0, read RAM at E0; original B_RF edgeE1 captures rf_rdata and acknowledges. B_RF is the actual TX_DATA write edge of a separate serialized transaction.',minimal_proposal='Replace always-on read with B_IDLE && bus_req && !bus_we && aligned(bus_addr)==RA_MBX_TXB_Q; leave r_MBX_TXB_ADDR as address. Since all TX writes arise from rf_wr at B_RF and one bus transaction is active, prove WE&&RE==0 independent of physical address width.',miter='Exercise TX_DATA writes, TXB_ADDR changes, immediate next legal TXB_Q reads, heldreq, two-master arbitration, reset, twp limit/wrap and ACK gaps. Compare bus_ack/bus_rdata and all committed words; check actual CEA/CEB and ADA[13:4]/ADB[13:4]. Pending previous-transaction rf_wr must be included, not assumed absent.'),
  dict(store='console_lanes',condition='Each lane read-enable is tied1; con_ra remains at last debug word, while independent CPU console stores append and wrap. Matching physical row groups can collide even if logical byte addresses differ.',status='Concrete independent-port risk. Valid-read gating alone does not remove a write at the demanded read edge.',needed_read_edges='Debug E0 (d_st0): capture con_ra; E1 (d_st1): read target; E2 (d_st3): capture con_q and original ACK. E0/E2 dummy reads are unnecessary. CPU con_we is an independent port.',minimal_proposal='Use read only at valid console d_st1. Give that single edge read priority; if a CPU log write is accepted at it, capture8bit data +10bit wordaddr +2bit lane +valid and commit at E2 when no read is needed. Keep original con_wp/per_ready at E1. Deferring all writes at the demanded read edge avoids any assumption about physical collision granularity.',why_one_entry_may_suffice='con_we implies req && sel_per, so per_ready becomes1 after E1; at E2 req=mem_valid&&!mem_ready is0, leaving the queued commit uncontended. This is a source argument requiring bus miter, not a completed implementation/proof.',miter='Compare debug ACK/data at unchanged E2 against old read-first behavior; CPU per_ready/con_wp/log contents; E0/E1/E2 byte writes for all4lanes, same logical word and adjacent physical row words, last4095byte wrap, concurrent CPU/UART/debug heldreq, reset/HOLD and accepted pending-write drain. Do not cancel an already-ACKed write on reset: old RAM had no reset and had already committed it.')]
 result=dict(recorded_at=datetime.now().astimezone().isoformat(),scope='Read-only source consumer and official C-mode risk review; no EDA/numeric tests/source edits',candidate=str(SOURCE),
  official_document=dict(file=metadata(DOC),title='UG300-1.3.6E Arora V BSRAM & SSRAM User Guide',local_text=str(text),SDP_section='3.3, printed24 / PDF33',unsupported_RBW='Features note printed4 / PDF13; DP printed6 and SP printed18',
   facts=['138K does not support Read-before-Write write mode.','SDPB/SDPX9B support only Normal write mode; READ_MODE chooses bypass vs an extra output pipeline.','The SDP note prohibits a cross-port read and write at the same address, defined by physical14bit AD[13:4]. Same clock does not waive the note.']),
  vendor_library=dict(file=metadata(LIB),interfaces=interfaces,interpretation='The functional model has nonblocking storage updates and can deterministically simulate old-data collisions; that is not a silicon timing or legal-operation guarantee.'),
  actual_prior_evidence=dict(PA2122=dict(log=metadata(pa),errors=errors,conclusion='DPX9B WRITE_MODE0=10 was rejected before placement;00/01 were requested by actual C implementation. No per-net congestion claim.'),
    small_store_C_mapping=dict(result=str(BASE/'small_block_stores_probe1/result.json'),actual_vendor_exit=0,source_primitives='Two SDPB leaves, widths1 and32; actual physical split1+16+16=3',READ_MODE=0,WRITE_MODE_parameter=False,conclusion='Prior actual legal C normal-mode mapping, not proof of prohibited same-row simultaneous operations.')),
  consumers=rows,source_anchors=source_rows,
  actual_stage3_checklist=['Reject DPB/DPX9B/SP/SPX9 with any WRITE_MODE=2 before PNR; do not repair a netlist parameter without re-proving source behavior.',
   'Confirm actual target instance geometry, source/physical wide split, INIT and enabled one-cycle read/hold; READ_MODE0 bypass does not mean asynchronous public read.',
   'For each physical write/read pair prove no simultaneous enabled conflict group: CEA && CEB && BLKSEL-match && ADA[13:4]==ADB[13:4] must be unreachable, including adjacent logical addresses folded to the same physical row.',
   'Re-run bus/consumer miters on any RDEN gating or write deferral. A suppressed demanded read or an ignored collision output is not a preservation proof.',
   'Read-only trig/bias/weight leaves must have actual write enables disabled; future mapped primitive tests check initialization and latency but cannot authorize prohibited collision use.'],
  selected_injection_action='Keep u_injb distributed: remove only its STAGE2B_BLOCK opt-in in new consumer candidates. Preserve all INJ_DATA/GO/length/ACK behavior and capacity; do not invent a busy/error protocol just to force the migration.',
  revised_combined_B_route_prediction=dict(previous=335,injection_source_and_wide_split_not_added=-2,new=333,prediction_not_actual=True,about1536_old_logic_cells_not_reclaimed=True),
  source_level_RTL_collision_pass_is_physical_acceptance=False,seven_store_collision_contract_closed=False,known_unsupported_modes_excluded_from_future_PNR=False,
  source_modified=False,new_EDA_or_simulation_started=False,SDK_code_copied_to_repo=False,programmed=False,no_SHA_used=True)
 return result

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();r=review();a.out.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
 print(json.dumps(dict(output=str(a.out),physical_contract_closed=False,critical_stores=['TX_shadow','console_lanes','injection_mailbox'],EDA_started=False),ensure_ascii=False))
