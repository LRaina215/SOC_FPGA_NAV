"""Read-only SLF128/256 RTL/FW/NVMP/coordinate contract inventory.

No source generation, simulation, EDA, firmware recompilation or board action.
Arithmetic examples are nominal geometry, not localization acceptance.
"""
from pathlib import Path
from datetime import datetime
import argparse,json,math,re
from prepare_baseline_fusion_retime import metadata,same_bytes

ROOT=Path(__file__).resolve().parents[2]
FROZEN=Path('F:/nav_integration_g2_20261023/baseline_fusion50_ring_MAX_syn40')

def matches(path,needles):
 lines=path.read_text(encoding='utf-8-sig').splitlines();rows=[]
 for needle in needles:
  found=[dict(line=i+1,text=t.strip())for i,t in enumerate(lines)if needle in t]
  assert found,('Missing reviewed contract anchor',str(path),needle)
  rows.append(dict(anchor=needle,matches=found))
 return dict(path=str(path),metadata=metadata(path),anchors=rows)

def review():
 specs=[(FROZEN/'nav/rtl/top/nav_subsystem.v',['slf_match #(','pw_addr(r_MATCH_SM_ADDR[15:0])','dbg_addr(r_MATCH_DBG_ADDR[15:0])']),
  (FROZEN/'nav/rtl/match/slf_match.v',['parameter integer LOG2N','localparam integer AW','input  wire [2*LOG2N-1:0] pw_addr',
     'slf_lf #(.LOG2N(LOG2N))','DW(18), .AW(AW)','DW(1), .AW(AW)','DW(AW), .AW(10)','cfg_prior ? 32\'sd0 : MARGIN',
     'wire [1:0]  ptag','clr_a[AW-1:0] == {AW{1\'b1}}','lfo_x <= lf_cx - N/2','10\'d256']),
  (FROZEN/'nav/rtl/match/slf_lf.v',['localparam integer LB = N + 2','sm_raddr = {gy0[LOG2N-1:0]','etag1 <= {gy0[LOG2N]']),
  (FROZEN/'fw/nav/prior.c',['hal_prior_write(rd16(r->ent)','rd16(h + 4) != 2','r->n_struct > 65536u','r->body_len != r->n_struct * 5']),
  (FROZEN/'fw/nav/prior.h',['#define PRIOR_HDR_LEN 40','uint8_t  ent[5]','不在软核内存里存整包']),
  (FROZEN/'fw/drv/hal_fpga.c',['void hal_prior_begin','match_wait_idle(50000)','match_wait_idle(5000)','void hal_prior_write(uint16_t slot','nav_wr(NAV_MATCH_SM_ADDR, slot)']),
  (FROZEN/'fw/nav/gmap.h',['#define GM_N 64','#define GM_DS 2','uint8_t prior[GM_N * GM_N]']),
  (ROOT/'config/nav_params.yaml',['MAP_LOG2N:','MAP_RES_MM:']),
  (ROOT/'tools/gen_config.py',['def gen_params','"MAP_N":','"MAP_CELLS":']),
  (ROOT/'model/slf_match.py',['SN_LOG2 =','SN = 1 << SN_LOG2','def tag_of','gx >> SN_LOG2','SN // 2','core = (wx0 >= mg)']),
  (ROOT/'model/prior_map.py',['SN = slf.SN','DS = 2','GM_MAX = 64','HDR_FMT =','slot = ((gy & (SN - 1))','if span > SN:',
     'hdr = struct.pack(HDR_FMT, MAGIC, PKG_VER, 0,','if ver != PKG_VER:']),
  (ROOT/'tools/map_tool.py',['s = (((wy + oy) & 255) << 8)','(ox + 256)','cloud = cloud[m]']),
  (ROOT/'rtl/match/tb/tb_slf.sv',['NSLOT = 65536','2*65536','pw_addr <= prior[2*i][15:0]']),
  (ROOT/'fw/nav/loc.c',['c->range_mm = 500','c->reloc_mm = 500']),
  (ROOT/'tools/nav_host.py',['def map_messages','map_begin(len(pkg))'])]
 anchors=[matches(p,words)for p,words in specs]
 top=specs[0][0].read_text();assert 'slf_match #(.SIN_INIT(SIN_INIT))'in top
 prior=(FROZEN/'fw/nav/prior.c').read_text();assert not re.search(r'rd16\(h\s*\+\s*6\)',prior)
 assert 'r->ent[0]'not in prior and 'rd16(r->ent)'in prior
 fw_equality={str(Path(n)):same_bytes(ROOT/n,FROZEN/n)for n in['fw/nav/prior.c','fw/nav/prior.h','fw/nav/gmap.h','fw/drv/hal_fpga.c']}
 actions=[
  dict(order=1,area='One distinct SLF size parameter',files=['config/nav_params.yaml','tools/gen_config.py','rtl/regs/nav_params.vh','fw/include/nav_params.h','model/nav_params.py','rtl/top/nav_subsystem.v'],
    change='After scenario acceptance, add an independent SLF_LOG2N=7/8 and derived SLF_N/CELLS; pass it explicitly into u_match. Do not change MAP_LOG2N7, the existing elevation/cost map, G2 clock overrides, point capacity or candidate counts.'),
  dict(order=2,area='MMIO address validity',files=['rtl/top/nav_subsystem.v','config/regmap.yaml','fw/drv/hal_fpga.c'],
    change='Use explicit2*SLF_LOG2N slices for write/debug ports; preserve32bit CSR layout. Reject/out-of-range diagnose slot>=SLF_CELLS before the HAL write; define direct-MMIO invalid address behavior so discarded high bits cannot silently alias.'),
  dict(order=3,area='Address/tag/clear geometry',files=['rtl/match/slf_match.v','rtl/match/slf_lf.v','model/slf_match.py'],
    change='Keep50mm INV_RES1311,18bit word and all candidates/point limits; ensure masks, concat row/column, tag extraction at bitLOG2N, N/2 rolling origin, MARGIN8 and clear exactlyN² slots all use the same selected size. Recreate all prior tags/addresses for that size.'),
  dict(order=4,area='NVMP geometry discrimination',files=['model/prior_map.py','fw/nav/prior.c','fw/nav/prior.h','tools/nav_host.py'],
    change='Define geometry metadata/version before using128 packages. A new format version carryingLOG2N is safer than only flags: legacyv2 firmware ignores flags and otherwise accepts a128 map into256. Keep18bit word payload and transport/ACK semantics; reject wrong geometry and slot>=N² before enabling prior mode. Bounds-check n_struct<=N² and body lengths; legacyv2 can be explicitly256-only.'),
  dict(order=5,area='No imaginary CPU prior-cache saving',files=['fw/nav/prior.h','fw/nav/gmap.h','model/prior_map.py'],
    change='Leave streamed40B header/5B record and independent64² prior-cost layer unless separately approved. Size the sparse package by actual nonzero records; do not budget a fourfold UART download or CPU RAM reduction from smaller SLF alone.'),
  dict(order=6,area='Host/viewer/test parameter binding',files=['tools/map_tool.py','rtl/match/tb/tb_slf.sv','sim/gen/gen_slf.py','sim/gen/check_slf_cycles.py'],
    change='Replace viewer masks255/shift8/extents256 with selected geometry; parameterize TB NSLOT/address vectors and pass LOG2N explicitly. Bind model env/profile per process to frozen RTL/FW and map format, without mutating a global default.'),
  dict(order=7,area='Coordinate and observation envelope',files=['tools/map_tool.py','model/prior_map.py','fw/nav/loc.c','fw/nav/pose_match.c','RK/goal/initial-pose integration'],
    change='Fix one map frame/origin for point cloud, initial pose, goals and observations. Reserve both candidate/relocalization shift and observed structure/normal/gating halos; test prior fixed-origin separately from no-prior robot-centered rolling mode. Bare3x5 fitting is not proof of preserved localization.'),
  dict(order=8,area='Acceptance before production capacity change',files=['RTL/model128-vs256 fixtures','FW NVMP tests','closed-loop navigation scenario runner'],
    change='Retain256 as production until both maps pass relevant no-prior/prior scenarios. Compare arrival against true pose and localization-valid state, false arrival, collision/stops, loss/relocalization, boundary structures and negative/rotated origins. Test clear/highest slot, wrong-size/legacy map rejection and tag wrap, not only average error or resource count.')]
 diagonal=math.sqrt(34.0)
 sizes={str(n):dict(LOG2N=int(math.log2(n)),address_bits=2*int(math.log2(n)),nominal_window_m=n*.05,
      tag_repeat_axis_m=2*n*.05,struct_words=n*n,struct_word_bits=18,touch_words=n*n,
      nominal_gate_margin_each_side_m=8*.05,no_prior_nominal_core_width_m=(n-16)*.05,
      dense_NVMP_record_bytes=n*n*5,dense_NVMP_upper_bytes_with_4096cost=40+n*n*5+4096)
      for n in(128,256)}
 return dict(recorded_at=datetime.now().astimezone().isoformat(),scope='Static actual frozen-SYN40 RTL/FW and current model/tool contract review; no production change or new run',
   confirmed=dict(current_SLF_default_LOG2N=8,current_SLF_N=256,current_independent_local_elevation_MAP_LOG2N=7,
      LOG2N_propagates_inside_match_to_struct_touch_lf_slot_tags_clear_bounds=True,
      top_does_not_yet_pass_SLF_LOG2N=True,top_write_debug_inputs_fixed16bits=True,
      NVMP_v2_has_no_validated_geometry_field=True,NVMP_v2_flags_emitted_zero_and_FW_ignored=True,
      FW_entry_slot_has_no_capacity_check=True,CPU_stores_no_complete_SLF_prior_package=True,
      CPU_prior_cost_layer_bytes=4096,CPU_prior_cost_GM_DS=2,CPU_prior_cost_resolution_nominal_cm=10,
      sparse_NVMP_entries_are_only_occ_or_gate_nonzero=True),
   sizes=sizes,firmware_contract_files_current_equal_frozen=fw_equality,
   non_quartered=['Point buffer4096x36','Candidate/result arrays256 candidates','Sample accept bitmap1024','Touched-slot list1024 entries (slot width16->14 only)','LF line storage linear N+2, notN²','CPU streamed NVMP receive state','CPU64x64 prior cost layer','Sparse prior payload for the same unclipped physical feature set'],
   coordinate_geometry=dict(field_m=[3,5],bare_axis_aligned_surplus_m=[3.4,1.4],maximum_AABB_axis_span_any_rotation_m=diagonal,
      minimum_centered_nominal_margin_each_axis_any_rotation_m=(6.4-diagonal)/2,
      bare_rectangle_fits_at_any_rotation=True,rotation_is_not_needed_merely_to_fit_bare_rectangle=True,
      with_500mm_relocalization_each_side_worst_axis_m=diagonal+1,
      all_halos_fit_unconditionally=False,prior_window_is_fixed_origin=True,no_prior_window_is_robot_centered=True,
      robot_at_5m_end_can_lose_other_end_from_3p2m_halfwindow=True,
      actual_quantization='floor(mm*1311/65536), with1/16-cell positions from >>12; endpoint and negative rounding matter'),
   not_proved=['SLF128 mission/navigation acceptance','Actual post-shrink BSRAM/FF/LUT/CLS/routing or timing','FullFW/new-format interoperability','Real field coordinate registration and LiDAR structures outside driving rectangle'],
   actionable_change_list=actions,source_anchors=anchors,model_rotated_geometry_not_a_navigation_simulation=True,
   production_RTL_FW_map_format_modified=False,simulation_started=False,EDA_started=False,board_action=False,no_SHA_used=True)

if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();assert not a.out.exists()
 r=review();a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
 print(json.dumps(dict(output=str(a.out),actions=len(r['actionable_change_list']),sources=len(r['source_anchors']),production_modified=False),ensure_ascii=False))
