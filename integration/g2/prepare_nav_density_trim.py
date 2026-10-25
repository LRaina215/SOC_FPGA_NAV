"""Prepare explicit navigation-density/peripheral tradeoffs. Never starts Gowin."""
import argparse
import copy
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
from datetime import datetime

from prepare_lf_line_joint import assets, source_files
from trim_peripheral_rtl import trim_oled, trim_tx_shadow, OLED_RELATIVE, TX_SHADOW_RELATIVE

MATCH='nav/rtl/match/slf_match.v'
TRAJ='nav/rtl/plan/traj_eval.v'
PROFILES={
 'medium':dict(match_log=7,traj_log=5,half=2,half_ang=2,step=38,angle=128,coarse=100,coarse_angle=182,nv=3,nw=5,fine1=38,fine2=15,fine_angle1=128,fine_angle2=64),
 'strong':dict(match_log=6,traj_log=4,half=1,half_ang=1,step=75,angle=256,coarse=125,coarse_angle=364,nv=1,nw=1,fine1=75,fine2=30,fine_angle1=256,fine_angle2=128),
}

def replace(text,old,new):
    assert text.count(old)==1,old
    return text.replace(old,new,1)

def transform_match(text,p):
    text=text.replace('\r\n','\n');log=p['match_log']
    text=replace(text,'    localparam integer N  = 1 << LOG2N;',f'    localparam integer CAND_LOG = {log};\n    localparam integer N  = 1 << LOG2N;')
    text=replace(text,'nav_ram_sdp #(.DW(32), .AW(9)) u_cand','nav_ram_sdp #(.DW(32), .AW(CAND_LOG+1)) u_cand')
    text=replace(text,'.we(cand_we), .waddr(cand_waddr)', '.we(cand_we && cand_waddr < (1 << (CAND_LOG+1))), .waddr(cand_waddr[CAND_LOG:0])')
    text=replace(text,'.re(cd_re), .raddr(cd_ra)', '.re(cd_re), .raddr(cd_ra[CAND_LOG:0])')
    text=replace(text,'nav_ram_sdp #(.DW(32), .AW(9)) u_res', '''wire [31:0] stored_res_q;
    reg result_address_valid = 1'b0;
    always @(posedge clk) begin
        if (rst) result_address_valid <= 1'b0;
        else if (res_re) result_address_valid <= res_raddr < (1 << (CAND_LOG+1));
    end
    assign res_q = result_address_valid ? stored_res_q : 32'd0;
    nav_ram_sdp #(.DW(32), .AW(CAND_LOG+1)) u_res''')
    text=replace(text,'.we(rs_we), .waddr(rs_wa)', '.we(rs_we), .waddr(rs_wa[CAND_LOG:0])')
    text=replace(text,'.re(res_re), .raddr(res_raddr), .rdata(res_q)', '.re(res_re && res_raddr < (1 << (CAND_LOG+1))), .raddr(res_raddr[CAND_LOG:0]), .rdata(stored_res_q)')
    text=replace(text,'ncand <= cfg_ncand;', 'ncand <= cfg_ncand > (1 << CAND_LOG) ? (1 << CAND_LOG) : cfg_ncand;')
    return text

def transform_traj(text,p):
    text=text.replace('\r\n','\n');log=p['traj_log']
    text=replace(text,'    localparam integer N = 1 << LOG2N;',f'    localparam integer ACTIVE_K_LOG = {log};\n    localparam integer N = 1 << LOG2N;')
    text=replace(text,'.AW(KMAX_LOG + 1)) u_cand', '.AW(ACTIVE_K_LOG + 1)) u_cand')
    text=replace(text,'.we(cand_we), .waddr(cand_addr)', '.we(cand_we && cand_addr < (1 << (ACTIVE_K_LOG+1))), .waddr(cand_addr[ACTIVE_K_LOG:0])')
    text=replace(text,'.re(c_re), .raddr(c_ra)', '.re(c_re), .raddr(c_ra[ACTIVE_K_LOG:0])')
    text=replace(text,'    nav_ram_sdp #(.DW(32), .AW(KMAX_LOG + 2)) u_res', '''    wire [31:0] stored_res_q;
    reg result_address_valid = 1'b0;
    always @(posedge clk) begin
        if (rst) result_address_valid <= 1'b0;
        else if (res_re) result_address_valid <= res_addr < (1 << (ACTIVE_K_LOG+2));
    end
    assign res_rdata = result_address_valid ? stored_res_q : 32'd0;
    nav_ram_sdp #(.DW(32), .AW(ACTIVE_K_LOG + 2)) u_res''')
    text=replace(text,'.we(r_we), .waddr(r_wa)', '.we(r_we), .waddr(r_wa[ACTIVE_K_LOG+1:0])')
    text=replace(text,'.re(res_re), .raddr(res_addr), .rdata(res_rdata)', '.re(res_re && res_addr < (1 << (ACTIVE_K_LOG+2))), .raddr(res_addr[ACTIVE_K_LOG+1:0]), .rdata(stored_res_q)')
    text=replace(text,'K <= (cfg_k > (1 << KMAX_LOG)) ? (1 << KMAX_LOG) : cfg_k;', 'K <= (cfg_k > (1 << ACTIVE_K_LOG)) ? (1 << ACTIVE_K_LOG) : cfg_k;')
    return text

def firmware_transform(relative,text,p):
    if relative=='nav/local_planner.h':return replace(text,'#define LP_MAX_CAND 64',f'#define LP_MAX_CAND {1<<p["traj_log"]}')
    if relative=='nav/pose_match.h':return replace(text,'#define PM_MAX_CAND 256',f'#define PM_MAX_CAND {1<<p["match_log"]}')
    if relative=='nav/local_planner.c':
        return replace(replace(text,'c->nv = 4;',f'c->nv = {p["nv"]};'),'c->nw = 9;',f'c->nw = {p["nw"]};')
    if relative=='nav/pose_match.c':
        text=replace(text,'/* 7×7 平移（25 mm）× 5 角度（0.7°）= 245 个候选，搜索范围 ±75 mm、±1.4° */',f'/* Explicit density tradeoff: {(2*p["half"]+1)**2*(2*p["half_ang"]+1)} tracking candidates; quality thresholds unchanged. */')
        for old,new in [('c->step_mm = 25;',f'c->step_mm = {p["step"]};'),('c->step_ang = 128;',f'c->step_ang = {p["angle"]};'),('c->half = 3;',f'c->half = {p["half"]};'),('c->half_ang = 2;',f'c->half_ang = {p["half_ang"]};')]:text=replace(text,old,new)
        return text
    if relative=='nav/loc.c':
        text=replace(text,'c->step_mm = 50;',f'c->step_mm = {p["coarse"]};')
        text=replace(text,'c->step_ang = ANG_DEG;',f'c->step_ang = {p["coarse_angle"]};')
        text=replace(text,'const int32_t sm = l->phase == 1 ? 25 : 10, sa = l->phase == 1 ? 128 : 64;',f'const int32_t sm = l->phase == 1 ? {p["fine1"]} : {p["fine2"]}, sa = l->phase == 1 ? {p["fine_angle1"]} : {p["fine_angle2"]};\n        if (kmax < {(2*p["half"]+1)**2*(2*p["half_ang"]+1)}) {{ l->st=LOC_LOST; l->phase=3; return 0; }}')
        for old,new in [('for (int ia = -2; ia <= 2; ia++)',f'for (int ia = -{p["half_ang"]}; ia <= {p["half_ang"]}; ia++)'),('for (int iy = -3; iy <= 3; iy++)',f'for (int iy = -{p["half"]}; iy <= {p["half"]}; iy++)'),('for (int ix = -3; ix <= 3; ix++)',f'for (int ix = -{p["half"]}; ix <= {p["half"]}; ix++)')]:text=replace(text,old,new)
        text=replace(text,'int sm=l->phase==2?25:10,sa=l->phase==2?128:64;',f'int sm=l->phase==2?{p["fine1"]}:{p["fine2"]},sa=l->phase==2?{p["fine_angle1"]}:{p["fine_angle2"]};')
        w=2*p['half']+1;h=p['half'];ha=p['half_ang']
        return replace(text,'dx=seed.dx+(j%7-3)*sm;dy=seed.dy+((j/7)%7-3)*sm;da=seed.da+(j/49-2)*sa;',f'dx=seed.dx+(j%{w}-{h})*sm;dy=seed.dy+((j/{w})%{w}-{h})*sm;da=seed.da+(j/{w*w}-{ha})*sa;')
    if relative=='drv/hal_fpga.c':
        text=replace(text,'    for (int i = 0; i < k; i++) {\n        nav_wr(NAV_WIN_TRAJ_CAND_BASE',f'    if (k <= 0 || k > {1<<p["traj_log"]} || t < 0 || t > 255) return false;\n    for (int i = 0; i < k; i++) {{\n        nav_wr(NAV_WIN_TRAJ_CAND_BASE')
        text=replace(text,'|| k > 256)',f'|| k <= 0 || k > {1<<p["match_log"]})')
        return replace(text,'bool hal_match_result(hal_match_res_t *res, int k)\n{',f'bool hal_match_result(hal_match_res_t *res, int k)\n{{\n    if (k <= 0 || k > {1<<p["match_log"]}) return false;')
    return text

def prepare(source,out,baseline,tier):
    source,out,baseline=[Path(x).resolve() for x in (source,out,baseline)]
    assert not out.exists();p=PROFILES[tier]
    manifest=json.loads((source/'manifest.json').read_text());hardware=source_files(source,manifest)
    data=assets(source);assert len(data)==104
    extra={'g2.cst','g2.sdc','ip/serdes.csr'}
    extra.update(x.relative_to(source).as_posix() for x in (source/'ip').iterdir() if x.is_file() and x.suffix.lower() in {'.json','.toml','.mod','.ipc'})
    headers={x.relative_to(source).as_posix() for x in source.rglob('*') if x.is_file() and x.suffix in {'.h','.vh'} and 'impl' not in x.relative_to(source).parts}
    out.mkdir()
    for rel in set(hardware)|set(data)|extra|headers:
        (out/rel).parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(source/rel,out/rel)
    shutil.copytree(source/'fw',out/'fw',dirs_exist_ok=True)
    changes={MATCH:transform_match((source/MATCH).read_text(encoding='utf-8'),p),TRAJ:transform_traj((source/TRAJ).read_text(encoding='utf-8'),p),OLED_RELATIVE:trim_oled((source/OLED_RELATIVE).read_text(encoding='utf-8')),TX_SHADOW_RELATIVE:trim_tx_shadow((source/TX_SHADOW_RELATIVE).read_text(encoding='utf-8'))}
    for rel,text in changes.items():(out/rel).write_text(text,encoding='utf-8',newline='\n')
    fw_changed=[]
    for rel in ['nav/local_planner.h','nav/local_planner.c','nav/pose_match.h','nav/pose_match.c','nav/loc.c','drv/hal_fpga.c']:
        f=out/'fw'/rel;before=f.read_text(encoding='utf-8');after=firmware_transform(rel,before,p);assert before!=after;f.write_text(after,encoding='utf-8',newline='\n');fw_changed.append('fw/'+rel)
    for rel in ('g2.cst','g2.sdc','ip/serdes.csr'):shutil.copyfile(baseline/rel,out/rel)
    def rewrite(value):
        if isinstance(value,str):return value.replace(str(source),str(out)).replace(source.as_posix(),out.as_posix())
        if isinstance(value,list):return [rewrite(x) for x in value]
        if isinstance(value,dict):return {k:rewrite(v) for k,v in value.items()}
        return value
    manifest=rewrite(copy.deepcopy(manifest));manifest.pop('effective_options',None)
    manifest.update(delivery_trim_profile='trim_profile.json',delivery_trim_profile_id='nav_'+tier,delivery_trim_origin=str(source),programmed=False,complete_sfp_cdc_acceptance=False)
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (out/'run.tcl').write_text(rewrite((source/'run.tcl').read_text()),encoding='utf-8',newline='\n')
    changed=sorted(rel for rel in hardware if (source/rel).read_bytes()!=(out/rel).read_bytes());assert changed==sorted(changes)
    profile=dict(schema_version=1,profile_id='nav_'+tier,kind='rtl',origin=str(source),constraint_source=str(baseline),modified_rtl_inputs=changed,added_rtl_inputs=[],removed_rtl_inputs=[],allowed_changed_prefixes=['u_nav/u_match/','u_nav/u_tr/','u_control/display/','u_nav/u_mbx/u_txs/'],allowed_firmware_init_prefixes=['u_soc/'],allowed_memory_change_prefixes=['u_nav/u_match/u_cand/','u_nav/u_match/u_res/','u_nav/u_tr/u_cand/','u_nav/u_tr/u_res/','u_nav/u_mbx/u_txs/'],firmware_modified_inputs=fw_changed,allowed_changed_assets=[],firmware_provenance={},proofs=[],tradeoffs=[dict(feature='OLED',before='display enabled',after='pins released; display unavailable'),dict(feature='offline TX shadow',before='sent payload readback',after='readback zero; live TX/RX and telemetry retained'),dict(feature='matching batch',before=256,after=1<<p['match_log'],tracking_before=245,tracking_after=(2*p['half']+1)**2*(2*p['half_ang']+1),tracking_halfwidth_mm=p['half']*p['step'],tracking_halfwidth_angle_units=p['half_ang']*p['angle'],coarse_step_mm=p['coarse'],coarse_step_angle=p['coarse_angle'],global_range_limit_mm=32000,risk='coarser localization grid may reject or mislocalize scenes; physical positioning performance unverified'),dict(feature='trajectory candidates',before=64,after=1<<p['traj_log'],default_generated_before=46,default_generated_after=p['nv']*p['nw']+10,holonomic_generated_after=p['nv']*p['nw']+14,steps=20,dt_ms=100,safe_steps=12,risk='fewer high-speed curve choices; strong tier keeps straight high-speed and slow arcs, may detour or stop more often')],preserved_contracts=['4500 and all original weights','125Hz control','complete SFP telemetry and stop priority','CPU128KiB','65536x18 structure map','16384x53 elevation map','4096x36 points','150mm goal threshold and original motion/support guards'])
    profile['allowed_memory_change_prefixes'].append('u_control/display/')
    (out/'trim_profile.json').write_text(json.dumps(profile,indent=2)+'\n')
    state=dict(passed=False,phase='prepared_source_waiting_for_firmware_and_targeted_proofs',source=str(source),prepared=str(out),modified_rtl_inputs=changed,unchanged_hardware_inputs=64,assets_total=104,unchanged_assets=104,firmware_modified_inputs=fw_changed,SYN_started=False,PNR_started=False,programmed=False)
    (out/'prepared_source_review.json').write_text(json.dumps(state,indent=2)+'\n');(out/'job.json').write_text(json.dumps(state,indent=2)+'\n')
    return state

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--source',required=True);ap.add_argument('--out',required=True);ap.add_argument('--baseline',required=True);ap.add_argument('--tier',choices=PROFILES,required=True);a=ap.parse_args();print(json.dumps(prepare(a.source,a.out,a.baseline,a.tier)))
