"""Freeze SLF128/LF-forward RTL, selected safe FW, profile/map tools only.

No compiler, simulator, synthesis, PNR or programmer is launched. Production
SLF=128; LOG2N=8 remains available only to explicit validation fixtures.
"""
from pathlib import Path
from datetime import datetime
import argparse,copy,difflib,importlib.util,json,re,shutil
from prepare_baseline_fusion_retime import metadata,same_bytes,relocate

BASE=Path('F:/nav_integration_g2_20261023')
SRC=BASE/'baseline_fusion50_ring_MAX_syn40'
SAFE=BASE/'CPU250_host_drain_isolated_validation1/isolated/fw'
OUT=BASE/'stage2_SLF128_forward_safeFW_prepared7'
ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).parent
PROFILE=0x33580001
FLAGS=0x0107
def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def save(p,x):p.write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
def once(t,a,b):assert t.count(a)==1,(a,t.count(a));return t.replace(a,b,1)
def edit(out,rel,fn):
 p=out/rel;p.write_text(fn(p.read_text(encoding='utf-8-sig')),encoding='utf8')

def subsystem(t):
 t=once(t,'    parameter SIN_INIT = "sin_lut.hex",','    parameter SIN_INIT = "sin_lut.hex",\n    parameter integer SLF_LOG2N = 7,')
 t=once(t,'    slf_match #(.SIN_INIT(SIN_INIT)) u_match (','''    // Full CSR address validation precedes narrowing: invalid addresses never alias.
    wire mt_pw_valid = (r_MATCH_SM_ADDR >> (2*SLF_LOG2N)) == 0;
    wire mt_dbg_valid = (r_MATCH_DBG_ADDR >> (2*SLF_LOG2N)) == 0;
    assign s_MATCH_ADDR_ERR = {30'd0,
        (p_MATCH_DBG_CMD && wd_q[0] && !mt_dbg_valid),
        (p_MATCH_SM_WDATA && !mt_pw_valid)};
    assign i_MATCH_PROFILE = 32'h534C0100 | SLF_LOG2N;
    slf_match #(.LOG2N(SLF_LOG2N), .SIN_INIT(SIN_INIT)) u_match (''')
 t=once(t,'.lf_cx(r_MAP_CENTER_X), .lf_cy(r_MAP_CENTER_Y),','.lf_cx(r_MATCH_LF_CX), .lf_cy(r_MATCH_LF_CY),')
 t=once(t,'.pw_we(p_MATCH_SM_WDATA), .pw_addr(r_MATCH_SM_ADDR[15:0]),','.pw_we(p_MATCH_SM_WDATA && mt_pw_valid), .pw_addr(r_MATCH_SM_ADDR[2*SLF_LOG2N-1:0]),')
 t=once(t,'.dbg_req(mt_dbg_req), .dbg_addr(r_MATCH_DBG_ADDR[15:0]),','.dbg_req(mt_dbg_req), .dbg_addr(r_MATCH_DBG_ADDR[2*SLF_LOG2N-1:0]),')
 t=once(t,'.dbg_addr(r_MATCH_DBG_ADDR[2*SLF_LOG2N-1:0]),','.dbg_addr(mt_dbg_address),')
 t=once(t,'    reg  [17:0] mt_dbg_word;', '''    reg  [17:0] mt_dbg_word;
    reg [2*SLF_LOG2N-1:0] mt_dbg_address;
    reg mt_dbg_pending; // latest command owns completion, including invalid commands''')
 t=once(t,"mt_dbg_req <= 1'b0; mt_dbg_done <= 1'b0; end", "mt_dbg_req <= 1'b0; mt_dbg_pending <= 1'b0; mt_dbg_address <= 0; mt_dbg_done <= 1'b0; end")
 t=once(t,"if (p_MATCH_DBG_CMD & wd_q[0]) begin mt_dbg_req <= 1'b1; mt_dbg_done <= 1'b0; end",'''if (p_MATCH_DBG_CMD & wd_q[0]) begin
                mt_dbg_req <= 1'b0;
                mt_dbg_pending <= mt_dbg_valid;
                mt_dbg_address <= r_MATCH_DBG_ADDR[2*SLF_LOG2N-1:0];
                mt_dbg_done <= !mt_dbg_valid;
                if (!mt_dbg_valid) mt_dbg_word <= 18'd0;
            end''')
 t=once(t,"else if (mt_dbg_ack) begin mt_dbg_req <= 1'b0; mt_dbg_done <= 1'b1; mt_dbg_word <= mt_dbg_q; end",'''else if (mt_dbg_pending) begin
                // Drain any superseded transaction/ACK before starting the new one.
                if (!mt_busy && !mt_dbg_ack) begin mt_dbg_req <= 1'b1; mt_dbg_pending <= 1'b0; end
            end else if (mt_dbg_req && mt_dbg_ack) begin
                mt_dbg_req <= 1'b0; mt_dbg_done <= 1'b1; mt_dbg_word <= mt_dbg_q;
            end''')
 return t

def match(t):
 t=once(t,'parameter integer LOG2N  = 8,','parameter integer LOG2N  = 7,')
 t=once(t,'    reg signed [31:0] lfo_x, lfo_y;', '    reg signed [31:0] lf_build_cx, lf_build_cy; // snapshot at accepted UPDATE, not live CSR\n    reg signed [31:0] lfo_x, lfo_y;')
 t=once(t,'.start(lf_start), .cx(lf_cx), .cy(lf_cy),','.start(lf_start), .cx(lf_build_cx), .cy(lf_build_cy),')
 t=once(t,"st <= S_CLR; clr_a <= 0; pts_ready <= 1'b0; lfo_x <= -(N/2);", "st <= S_CLR; clr_a <= 0; pts_ready <= 1'b0; lf_build_cx <= 0; lf_build_cy <= 0; lfo_x <= -(N/2);")
 t=once(t,'else if (cmd_eval || cmd_update) begin','''else if (cmd_eval || cmd_update) begin
                    if (cmd_update) begin lf_build_cx <= lf_cx; lf_build_cy <= lf_cy; end''')
 t=once(t,'lfo_x <= lf_cx - N/2; lfo_y <= lf_cy - N/2;','lfo_x <= lf_build_cx - N/2; lfo_y <= lf_build_cy - N/2;')
 return t

def hal_fpga(t):
 t=once(t,'#include "nav_regs_map.h"','#include "nav_regs_map.h"\n#include "nav_params.h"\n#include "fixmath.h"')
 t=once(t,'static bool support_have_capture;', '''static bool support_have_capture;
static int32_t lf_vehicle_cx,lf_vehicle_cy; // exactly the last commit MAP_CENTER
static int32_t lf_round_cell(int64_t n)
{
    const int64_t den=16384LL*65536LL;
    return (int32_t)(n<0 ? -((-n+den/2)/den) : (n+den/2)/den);
}
void hal_match_lf_forward(int32_t distance_mm, angle_t yaw)
{
    uint32_t inv=nav_rd(NAV_MAP_INV_RES)&65535u;
    int32_t dx=lf_round_cell((int64_t)distance_mm*inv*fx_cos(yaw));
    int32_t dy=lf_round_cell((int64_t)distance_mm*inv*fx_sin(yaw));
    nav_wr(NAV_MATCH_LF_CX,(uint32_t)lf_vehicle_cx+(uint32_t)dx);
    nav_wr(NAV_MATCH_LF_CY,(uint32_t)lf_vehicle_cy+(uint32_t)dy);
}''')
 t=once(t,'    nav_wr(NAV_MAP_CENTER_X, (uint32_t)cx);','    lf_vehicle_cx=cx; lf_vehicle_cy=cy;\n    nav_wr(NAV_MAP_CENTER_X, (uint32_t)cx);')
 t=once(t,'    nav_wr(NAV_MATCH_SM_ADDR, slot);','    if ((uint32_t)slot >= NP_SLF_CELLS) return; // never truncate an invalid slot\n    nav_wr(NAV_MATCH_SM_ADDR, slot);')
 return t

def app(t):
 # Use the actual model public source after any pose correction at each UPDATE.
 t=t.replace('hal_match_update(', 'app_match_update(')
 t=once(t,'bool nav_app_have_prior(void)', '''static void app_match_update(int32_t dx,int32_t dy,int32_t da,int32_t cx,int32_t cy)
{
    hal_match_lf_forward(g_app.lf_forward_mm,g_app.nt.pose.cur.yaw);
    hal_match_update(dx,dy,da,cx,cy);
}
bool nav_app_have_prior(void)''')
 t=once(t,'    g_app.zband_lo = -600;','    g_app.lf_forward_mm = APP_LF_FORWARD_MM;\n    g_app.match_zhi = APP_ZHI;\n    g_app.zband_lo = -600;')
 t=t.replace('hal_match_zband(APP_ZLO_ODOM, APP_ZHI)','hal_match_zband(APP_ZLO_ODOM, g_app.match_zhi)')
 t=t.replace('hal_match_zband(APP_ZLO_PRIOR, APP_ZHI)','hal_match_zband(APP_ZLO_PRIOR, g_app.match_zhi)')
 t=once(t,'    hal_lidar_init();','    hal_lidar_init();\n    hal_match_zband(APP_ZLO_ODOM,g_app.match_zhi);')
 t=once(t,'void nav_app_init(void)', '''bool nav_app_set_slf_window(int32_t forward_mm,int32_t zhi_mm)
{
    if (forward_mm<0 || forward_mm>3000 || zhi_mm<APP_ZLO_ODOM || zhi_mm>1500 ||
        g_app.nt.mode!=NAV_MODE_IDLE || g_app.pm_pend || g_app.pm_run ||
        nav_app_have_prior() || g_app.prx.state==PRIOR_RECV) return false;
    if(g_app.lf_forward_mm!=forward_mm || g_app.match_zhi!=zhi_mm)
        hal_prior_enable(false,0,0); // clear accumulated old-height/window SLF, EM unchanged
    g_app.lf_forward_mm=forward_mm;g_app.match_zhi=zhi_mm;
    hal_match_zband(APP_ZLO_ODOM,zhi_mm);return true;
}
void nav_app_init(void)''')
 t=once(t,'prior_rx_begin(&g_app.prx, &nt->gm, m->u.map_begin.total);','prior_rx_begin_profile(&g_app.prx, &nt->gm, m->u.map_begin.total,(int16_t)g_app.match_zhi);')
 t=once(t,'    case MSG_MAP_BEGIN:', '''    case MSG_SLF_CONFIG: {
        uint32_t result=0; // query0/applied1/busy2/range3/prior4
        if(m->u.slf.set) {
            int32_t d=m->u.slf.forward_mm,z=m->u.slf.zhi_mm;
            if(d<0||d>3000||z<APP_ZLO_ODOM||z>1500) result=3;
            else if(nav_app_have_prior()||g_app.prx.state==PRIOR_RECV) result=4;
            else result=nav_app_set_slf_window(d,z)?1:2;
        }
        uint8_t reply[20];
        int n=proto_build_slf_ack(reply,result,g_app.lf_forward_mm,g_app.match_zhi,0x534C0107u);
        hal_host_send(reply,n);break;
    }
    case MSG_MAP_BEGIN:''')
 return t

def prior_c(t):
 t=once(t,'#include "hal.h"','#include "hal.h"\n#include "nav_params.h"')
 t=once(t,'void prior_rx_begin(prior_rx_t *r, gmap_t *g, uint32_t total)', '''static uint8_t reject_prior(prior_rx_t *r,gmap_t *g,uint8_t state)
{
    r->state=state;hal_prior_enable(false,0,0);gmap_prior_drop(g);return state;
}
void prior_rx_begin(prior_rx_t *r,gmap_t *g,uint32_t total)
{ prior_rx_begin_profile(r,g,total,350); } // compatibility entry, explicit default
void prior_rx_begin_profile(prior_rx_t *r, gmap_t *g, uint32_t total,int16_t zhi)''')
 t=once(t,'    r->total = total;','    r->total = total;\n    r->zhi_expected=zhi; // immutable geometry/profile for this receive session')
 t=once(t,'                hal_prior_write(rd16(r->ent), w & 0x3FFFF);','''                uint32_t slot=rd16(r->ent);
                if (slot>=NP_SLF_CELLS || w>0x3FFFFu) {
                    // No partial prior is left usable when a later record is bad.
                    reject_prior(r,g,PRIOR_EFMT); return;
                }
                hal_prior_write((uint16_t)slot, w);''')
 t=once(t,'rd16(h + 4) != 2','rd16(h + 4) != 3 || rd16(h + 6) != PRIOR_GEOMETRY_FLAGS || rd32(h + 40) != PRIOR_PROFILE_ID || (int16_t)rd16(h + 44) != r->zhi_expected || (int16_t)rd16(h + 46) != -300')
 t=t.replace('/* 版本 2 = 18 位槽字 */','/* v3 = explicit geometry/profile/ZREL; legacy packages rejected */')
 t=once(t,'r->n_struct > 65536u','r->n_struct > NP_SLF_CELLS')
 t=t.replace('r->state = PRIOR_EFMT; return true;', 'reject_prior(r,g,PRIOR_EFMT); return true;')
 t=once(t,'            r->state = PRIOR_EFMT;\n            return true;','            reject_prior(r,g,PRIOR_EFMT);\n            return true;')
 t=once(t,'{ r->state = PRIOR_EFMT; return r->state; }','{ return reject_prior(r,g,PRIOR_EFMT); }')
 t=once(t,'''        r->state = PRIOR_ECRC;
        hal_prior_enable(false, 0, 0);
        gmap_prior_drop(g);
        return r->state;''','        return reject_prior(r,g,PRIOR_ECRC);')
 return t

def prior_py(t):
 t=once(t,'PKG_VER = 2 if slf.W18 else 1', 'PKG_VER = 3 if slf.W18 else 1')
 t=once(t,'HDR_FMT = "<4sHHiiIiiHHII"','''PROFILE_ID = 0x33580001
GEOMETRY_FLAGS = 0x0100 | slf.SN_LOG2
PROFILE_ZHI, PROFILE_ZLO = 350, -300
HDR_FMT = "<4sHHiiIiiHHIIIhh"''')
 t=once(t,'MAGIC, PKG_VER, 0, ox, oy, len(slot), gox, goy, gnx, gny, zlib.crc32(body), len(body))','MAGIC, PKG_VER, GEOMETRY_FLAGS, ox, oy, len(slot), gox, goy, gnx, gny, zlib.crc32(body), len(body), PROFILE_ID, int(slf.Z_HI), PROFILE_ZLO)')
 t=once(t,'magic, ver, flags, ox, oy, n, gox, goy, gnx, gny, crc, blen = struct.unpack(HDR_FMT, b[:HDR_LEN])','''if len(b)<HDR_LEN: raise ValueError("short NVMPv3 header")
    magic, ver, flags, ox, oy, n, gox, goy, gnx, gny, crc, blen, profile, zhi, zlo = struct.unpack(HDR_FMT, b[:HDR_LEN])
    if flags!=GEOMETRY_FLAGS or profile!=PROFILE_ID or zhi<60 or zhi>1500 or zlo!=PROFILE_ZLO:
        raise ValueError("NVMP geometry/profile/ZREL mismatch")
    if n>SN*SN or gnx>GM_MAX or gny>GM_MAX or blen!=5*n+gnx*gny or len(b)!=HDR_LEN+blen:
        raise ValueError("NVMP length/capacity mismatch")''')
 t=once(t,'    word = e[:, 2] | (e[:, 3] << 8) | (e[:, 4] << 16)','''    word = e[:, 2] | (e[:, 3] << 8) | (e[:, 4] << 16)
    if np.any(slot>=SN*SN) or np.any(word>0x3ffff): raise ValueError("invalid NVMP slot/word")''')
 return t

def proto_h(t):
 t=once(t,'MSG_MAP_CLEAR = 0x43,','MSG_MAP_CLEAR = 0x43, MSG_SLF_CONFIG = 0x44,')
 t=once(t,'MSG_MAP_ACK = 0x82,','MSG_MAP_ACK = 0x82, MSG_CONTROL_V2 = 0x83, MSG_SLF_ACK = 0x84,')
 t=once(t,'        struct { uint32_t total; } map_begin;', '        struct { uint32_t set; int32_t forward_mm,zhi_mm; } slf;\n        struct { uint32_t total; } map_begin;')
 t=once(t,'int proto_build_map_ack(', 'int proto_build_slf_ack(uint8_t *buf,uint32_t result,int32_t d,int32_t zhi,uint32_t profile);\nint proto_build_map_ack(')
 return t

def proto_c(t):
 t=once(t,'    case MSG_MAP_BEGIN:', '''    case MSG_SLF_CONFIG:
        if(n!=12 || (uint32_t)rd32(p)>1u) return false;
        m->u.slf.set=(uint32_t)rd32(p);m->u.slf.forward_mm=rd32(p+4);m->u.slf.zhi_mm=rd32(p+8);return true;
    case MSG_MAP_BEGIN:''')
 t=once(t,'int proto_build_map_ack(', '''int proto_build_slf_ack(uint8_t *buf,uint32_t result,int32_t d,int32_t zhi,uint32_t profile)
{
    uint8_t *p=hdr(buf,MSG_SLF_ACK);p=wr32(p,result);p=wr32(p,(uint32_t)d);
    p=wr32(p,(uint32_t)zhi);p=wr32(p,profile);return (int)(p-buf);
}
int proto_build_map_ack(''')
 return t

def prepare():
 assert not OUT.exists();m=load(SRC/'manifest.json');names=[Path(x).relative_to(SRC) for x in m['source_files']];assert len(names)==63
 shutil.copytree(SRC,OUT,ignore=shutil.ignore_patterns('impl','__pycache__','*.log','job.json','tool_runtime.json','system_telemetry.jsonl'))
 # Select one frozen safe firmware tree, not the live worktree and not stale images.
 shutil.rmtree(OUT/'fw');shutil.copytree(SAFE,OUT/'fw')
 fw_delta=[]
 for p in sorted((OUT/'fw').rglob('*')):
  if p.is_file():
   rel=p.relative_to(OUT);old=SRC/rel
   if not old.exists() or not same_bytes(p,old):fw_delta.append(str(rel))
 # Generate only config artifacts in a separate staging tree; first demand that
 # the unmodified YAML/generator regenerates the original frozen regfile/headers.
 work=OUT/'configuration_source';work.mkdir();(work/'config').mkdir()
 for n in ['regmap.yaml','nav_params.yaml']:shutil.copyfile(ROOT/'config'/n,work/'config'/n)
 for d in ['rtl/regs','fw/include','model','docs/2_interface']:(work/d).mkdir(parents=True)
 spec=importlib.util.spec_from_file_location('stage2cfg',ROOT/'tools/gen_config.py');gen=importlib.util.module_from_spec(spec);spec.loader.exec_module(gen);gen.ROOT=work
 reg=gen.load('regmap.yaml');par=gen.load('nav_params.yaml');gen.gen_regmap(reg);gen.gen_regfile(reg);gen.gen_params(par)
 for a,b in [('rtl/regs/nav_regfile.v','nav/rtl/regs/nav_regfile.v'),('rtl/regs/nav_subsystem_rf.vh','nav/rtl/regs/nav_subsystem_rf.vh'),('fw/include/nav_regs_map.h','fw/include/nav_regs_map.h')]:
  assert (work/a).read_text()==(SRC/b).read_text(),('generator baseline logical-text mismatch',a)
 par['slf']={'SLF_LOG2N':7,'SLF_N':128,'SLF_CELLS':16384}
 matchblock=next(x for x in reg['blocks'] if x['name']=='MATCH')
 matchblock['regs'] += [dict(name='LF_CX',off=0x48,acc='rw',reset=0,desc='Independent LF center x, world5cm cells; snapshotted on accepted UPDATE'),dict(name='LF_CY',off=0x4c,acc='rw',reset=0,desc='Independent LF center y; MAP_CENTER is unchanged'),dict(name='PROFILE',off=0x50,acc='ro',reset=0,desc='534C0107 production geometry identity, read-only'),dict(name='ADDR_ERR',off=0x54,acc='w1c',reset=0,desc='bit0 rejected prior address, bit1 rejected debug address')]
 (work/'config/regmap.yaml').write_text(gen.yaml.safe_dump(reg,sort_keys=False,allow_unicode=True),encoding='utf8')
 (work/'config/nav_params.yaml').write_text(gen.yaml.safe_dump(par,sort_keys=False,allow_unicode=True),encoding='utf8')
 gen.gen_regmap(reg);gen.gen_regfile(reg);gen.gen_params(par)
 for a,b in [('rtl/regs/nav_regfile.v','nav/rtl/regs/nav_regfile.v'),('rtl/regs/nav_subsystem_rf.vh','nav/rtl/regs/nav_subsystem_rf.vh'),('rtl/regs/nav_regs_map.vh','nav/rtl/regs/nav_regs_map.vh'),('rtl/regs/nav_params.vh','nav/rtl/regs/nav_params.vh'),('fw/include/nav_regs_map.h','fw/include/nav_regs_map.h'),('fw/include/nav_params.h','fw/include/nav_params.h')]:shutil.copyfile(work/a,OUT/b)
 edit(OUT,'g2_top.v',lambda t:once(once(t,'module g2_top (','module g2_top #(parameter integer SLF_LOG2N = 7) ('),'nav_subsystem #(.CLK_HZ(CLK_HZ),','nav_subsystem #(.SLF_LOG2N(SLF_LOG2N), .CLK_HZ(CLK_HZ),'))
 edit(OUT,'nav/rtl/top/nav_subsystem.v',subsystem)
 edit(OUT,'nav/rtl/match/slf_match.v',match)
 edit(OUT,'nav/rtl/match/slf_lf.v',lambda t:once(t,'parameter integer LOG2N = 8,','parameter integer LOG2N = 7,'))
 edit(OUT,'fw/drv/hal_fpga.c',hal_fpga)
 edit(OUT,'fw/hal/hal.h',lambda t:once(t,'void hal_match_update(','void hal_match_lf_forward(int32_t distance_mm, angle_t yaw);\nvoid hal_match_update('))
 edit(OUT,'fw/app/nav_app.c',app)
 edit(OUT,'fw/app/nav_app.h',lambda t:once(once(once(t,'#define APP_ZHI        1500','#define APP_ZHI        350\n#define APP_LF_FORWARD_MM 1500'),'    bool         use_imu_yaw;','    int32_t      lf_forward_mm,match_zhi; // runtime parameters; geometry is fixed128\n    bool         use_imu_yaw;'),'void nav_app_init(void);','bool nav_app_set_slf_window(int32_t forward_mm,int32_t zhi_mm);\nvoid nav_app_init(void);'))
 edit(OUT,'fw/nav/prior.c',prior_c)
 edit(OUT,'fw/nav/prior.h',lambda t:once(t,'#define PRIOR_HDR_LEN 40','#define PRIOR_HDR_LEN 48\n#define PRIOR_PROFILE_ID 0x33580001u\n#define PRIOR_GEOMETRY_FLAGS 0x0107u'))
 edit(OUT,'fw/nav/prior.h',lambda t:once(once(t,'    bool     hdr_ok;','    bool     hdr_ok;\n    int16_t  zhi_expected;'),'void prior_rx_begin(', 'void prior_rx_begin_profile(prior_rx_t *r,gmap_t *g,uint32_t total,int16_t zhi);\nvoid prior_rx_begin('))
 edit(OUT,'fw/nav/proto.h',proto_h);edit(OUT,'fw/nav/proto.c',proto_c)
 edit(OUT,'fw/nav/host_link_guard.c',lambda t:once(t,'case MSG_MAP_CLEAR: break;', 'case MSG_MAP_CLEAR: case MSG_SLF_CONFIG: break;'))
 edit(OUT,'fw/test/hal_py.c',lambda t:once(once(t,'#include "hal.h"','#include "hal.h"\n#include "fixmath.h"'),'void hal_prior_begin(void)', '''// This callback is present only in newly compiled isolated test DLLs.
// Existing stage1/model processes and their DLLs remain untouched.
void hal_match_lf_forward(int32_t distance_mm, angle_t yaw)
{
    const int64_t den=16384LL*65536LL;
    int64_t nx=(int64_t)distance_mm*s_mi.inv_res*fx_cos(yaw);
    int64_t ny=(int64_t)distance_mm*s_mi.inv_res*fx_sin(yaw);
    int32_t dx=(int32_t)(nx<0?-((-nx+den/2)/den):(nx+den/2)/den);
    int32_t dy=(int32_t)(ny<0?-((-ny+den/2)/den):(ny+den/2)/den);
    ctl(7,(int32_t)((uint32_t)s_commit_v[12]+(uint32_t)dx),
          (int32_t)((uint32_t)s_commit_v[13]+(uint32_t)dy),0);
}
void hal_prior_begin(void)'''))
 # Tool/model changes are independent frozen copies, never imported into an active model run.
 for d in ['model','tools']:(OUT/'host_profile'/d).mkdir(parents=True)
 for n in ['slf_match.py','prior_map.py','nav_fixed.py','nav_params.py']:
  shutil.copyfile(ROOT/'model'/n,OUT/'host_profile/model'/n)
 edit(OUT,'host_profile/model/slf_match.py',lambda t:once(once(t,'os.environ.get("SLF_SN_LOG2", "8")','os.environ.get("SLF_SN_LOG2", "7")'),'Z_LO, Z_HI = int(os.environ.get("SLF_ZLO", "60")), 1500','Z_LO, Z_HI = int(os.environ.get("SLF_ZLO", "60")), 350'))
 edit(OUT,'host_profile/model/prior_map.py',prior_py)
 shutil.copyfile(ROOT/'tools/map_tool.py',OUT/'host_profile/tools/map_tool.py')
 edit(OUT,'host_profile/tools/map_tool.py',lambda t:t.replace('((wy + oy) & 255) << 8','((wy + oy) & (pm.SN-1)) << pm.slf.SN_LOG2').replace('((wx + ox) & 255)','((wx + ox) & (pm.SN-1))').replace('(ox + 256)','(ox + pm.SN)').replace('(oy + 256)','(oy + pm.SN)'))
 edit(OUT,'host_profile/tools/map_tool.py',lambda t:once(once(t,'    a = ap.parse_args()','    ap.add_argument("--zhi",type=int,default=350,choices=range(60,1501),metavar="60..1500",help="selected runtime SLF height upper bound mm; package records it")\n    a = ap.parse_args()'),'    sp = pm.struct_points_from_cloud','    if b is None: raise ValueError("explicit enclosure --bounds required for field profile")\n    pm.slf.Z_HI=a.zhi\n    sp = pm.struct_points_from_cloud'))
 shutil.copyfile(ROOT/'tools/nav_host.py',OUT/'host_profile/tools/nav_host.py')
 edit(OUT,'host_profile/tools/nav_host.py',lambda t:once(t,'    msgs = [map_begin(len(pkg))]','''    # Validate the complete package before sending even MAP_BEGIN.
    import sys
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"model"))
    import prior_map
    prior_map.parse_package(pkg)
    msgs = [map_begin(len(pkg))]'''))
 edit(OUT,'host_profile/tools/nav_host.py',lambda t:once(once(t,'def map_messages(pkg):','''def slf_config(forward_mm=1500,zhi_mm=350,apply=True):
    return hdr(0x44)+struct.pack("<Iii",int(apply),forward_mm,zhi_mm)

def parse_slf_ack(b):
    if len(b)!=20 or b[:4]!=b"NV\\x01\\x84": raise ValueError("bad SLF_CONFIG_ACK")
    result,d,zhi,profile=struct.unpack("<IiiI",b[4:])
    return dict(type="slf_ack",result=result,forward_mm=d,zhi_mm=zhi,profile=profile)

def map_messages(pkg):'''),'    """FPGA -> 主机报文 -> dict（STATUS / MAP_ACK），其他返回 None"""','    """FPGA -> 主机报文 -> dict（STATUS / MAP_ACK），其他返回 None"""\n    if len(b)==20 and b[:4]==b"NV\\x01\\x84": return parse_slf_ack(b)'))
 edit(OUT,'host_profile/tools/nav_host.py',lambda t:once(t,'    if c == "map":','''    if c == "slf":
        if len(v) not in (0,2): raise ValueError("slf [forward_mm zhi_mm]; empty queries current parameters")
        ln.send(slf_config(*(map(int,v) if v else (1500,350)),apply=bool(v)))
        for _ in range(10):
            raw=ln.recv(0.3);reply=parse(raw) if raw else None
            if reply and reply.get("type")=="slf_ack":
                print(reply);return 0 if reply["result"] in (0,1) else 1
        raise IOError("missing SLF_CONFIG_ACK")
    if c == "map":'''))
 # A separate future C/model bridge knows the explicit LF center callback.
 (OUT/'host_profile/sim/closed_loop').mkdir(parents=True)
 shutil.copyfile(ROOT/'sim/closed_loop/fpga_model.py',OUT/'host_profile/sim/closed_loop/fpga_model.py')
 edit(OUT,'host_profile/sim/closed_loop/fpga_model.py',lambda t:once(once(t,'        elif op == 6:', '        elif op == 7:\n            self.explicit_lf_center=(a,b)\n        elif op == 6:'),'        self.smap.build_lf(cx,cy)','        cx,cy=getattr(self,"explicit_lf_center",(cx,cy))\n        self.smap.build_lf(cx,cy)'))
 # Do not silently treat the inherited old HEX files as the new safe/profile FW.
 changed=sorted(str(n).replace('\\','/') for n in names if not same_bytes(SRC/n,OUT/n))
 diff=''.join(''.join(difflib.unified_diff((SRC/n).read_text().splitlines(True),(OUT/n).read_text().splitlines(True),fromfile='SYN40/'+str(n),tofile='SLF128/'+str(n))) for n in names if not same_bytes(SRC/n,OUT/n))
 (OUT/'RTL_delta.patch').write_text(diff,encoding='utf8')
 m=relocate(m,SRC,OUT);m.update(candidate='SLF128_forward_safeFW',SLF_LOG2N=7,slf_log2n=7,NP_MAP_LOG2N_unchanged=7,source_preparation_only=True,automatic_PNR=False,automatic_SYN=False,firmware_rebuild_pending=True,inherited_CPU_INIT_not_new_FW=True)
 save(OUT/'manifest.json',m)
 contract=dict(production_SLF_LOG2N=7,SLF_cells=16384,word_bits=18,tag_axes_bits=1,tag_repeat_axis_cells=256,MARGIN=8,LF_halo_linear_Nplus2=True,MAP_CENTER_unchanged=True,NP_MAP_LOG2N_unchanged=7,selected_FW_source=str(SAFE),selected_52824B_image_is_provenance_only=True,selected_FW_goal_pose_margin_mm=50,selected_FW_host_guard_us=250000,selected_FW_bounded_host_drain=True,forward_mm_default=1500,ZHI_default=350,prior_ZLO=-300,no_prior_ZLO=60,forward_rounding='symmetric nearest; ties away fromzero of distance_mm*actual_MAP_INV_RES*sin/cosQ14/(16384*65536)',yaw='g_app.nt.pose.cur.yaw immediately before every original UPDATE, after any pose correction; same frozen model py_get_state()[3] source',center_base='last actual hal_lidar_commit MAP_CENTER cell pair; no changed EM center or candidate rotation pivot',latch='LF_CX andLF_CY written before command; matcher snapshots both on accepted UPDATE and uses them for the whole LF scan',prior='cfg_prior continues fixed LFO; UPDATE only releases point buffer',prior_address='All32 CSR bits above2*SLF_LOG2N rejected; no RAM write. ADDR_ERR bit0 sticky W1C.',debug_address='All32 high bits rejected; return0 withDONE andADDR_ERR bit1, no RAM access',NVMP=dict(version=3,header_bytes=48,geometry_flags=FLAGS,profile_ID=PROFILE,ZHI=350,ZLO=-300,legacy_rejected=True,per_entry_slot_and_word_validation=True),camera_development=False,IR_OLED_preserved=True,control4500_weights_ring_125Hz_unchanged=True,validation_pending=['generated CSR and invalid-address exact bus checks','model/RTL128 LF/tags/clear/prior/negative wraps bitwise','forward C/actual1024 LUT all-yaw/negative rounding comparison','new safe FW RV compilation/128KiB INIT and busy host-loss proof','merged point ownership/storage actual mapping','full physical resource/timing/CDC/reset/IO/signoff'],no_tools_started=True)
 contract.update(runtime_parameter_protocol=dict(request_type='0x44 SLF_CONFIG',reply_type='0x84 SLF_CONFIG_ACK',CONTROL_V2_0x83_preserved=True,request_bytes=16,reply_bytes=20,query_and_set=True,host_command='nav_host.py --link ... slf [forward_mm zhi_mm]',set_only_when='NAV_MODE_IDLE, no pm_pend/run, no active prior/receive; range d0..3000mm/ZHI60..1500mm',changes_clear_only_SLF_before_accept=True,normal_GO_MAP_ACK_unchanged=True),NVMP_runtime_ZHI='MAP_BEGIN snapshots selected runtime ZHI; each v3 header must match that immutable receive value. SET rejected during receive; MAP_END uses the same selection.',all_prior_terminal_failures_clear_and_drop=True,debug_latest_command_contract='A new command supersedes the prior debug completion; invalid returns0/DONE, old outstanding ACK ignored. Valid snapshots address and waits matcher idle plus ACK low before launching. Debug command bus ACK unchanged; debug completion may be deferred.',firmware_backend_note='New isolated hal_py.c uses explicit center callback7; existing model DLL/processes remain unchanged.',protocol_namespace_review=['Current fw/nav/proto.h and docs/2_interface/icd.md reserve CONTROL_V2=0x83, absent in older selected safe FW.','QHXD_Web/yuntuo/fpga_link/nv_proto.py and bridge/core.py/docs checked;0x44/0x84 not assigned.'])
 save(OUT/'implementation_contract.json',contract)
 save(OUT/'prepared_source_review.json',dict(prepared=True,source=str(SRC),candidate=str(OUT),source_file_count=63,changed_RTL=changed,selected_FW_from=str(SAFE),safe_FW_vs_SYN40_changed_files=fw_delta,non_FW_assets_unchanged=all(same_bytes(p,OUT/p.relative_to(SRC)) for p in SRC.rglob('*') if p.is_file() and p.suffix in ['.mem','.hex','.mif'] and 'impl' not in p.parts and 'fw' not in p.relative_to(SRC).parts),hardware_bindings=[metadata(OUT/n) for n in names],all_FW_bindings=[metadata(p) for p in sorted((OUT/'fw').rglob('*')) if p.is_file()],contract=str(OUT/'implementation_contract.json'),RTL_TB_not_yet_frozen_together=True,RTL_validation_started=False,SYN_started=False,PNR_started=False,programmed=False))
 return dict(candidate=str(OUT),changed_RTL=changed,selected_FW_source=str(SAFE),source_frozen=True,TB_pending=True,tools_started=False)

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--prepare',action='store_true');a=p.parse_args();assert a.prepare;print(json.dumps(prepare(),ensure_ascii=False))
