"""Freeze an independent, unaccepted FW integration patch; no build/EDA.

All nonzero proposals still traverse nav_task's common motion/support checks,
hal_send_cmd's R0 filter and original SAFE/lease. Runtime WIT selection remains.
"""
from pathlib import Path
from datetime import datetime
import argparse,json,shutil,difflib
from prepare_baseline_fusion_retime import metadata,save
BASE=Path('F:/nav_integration_g2_20261023');SRC=BASE/'P11_shared_guard_FW_experiment_prepared5'

APP_HELPERS=r'''
/* The P11 raw UART is selected explicitly; BMI088/control fusion is separate. */
static p11_command_guard_t p11;
static p11_raw_epoch_t p11_epoch;
static bool p11_protocol, p11_zero_published;
static uint32_t p11_last_raw_us;

uint32_t p11_raw_hal_count(void) { return nav_rd(NAV_IMU_RX_COUNT)&1023u; }
uint32_t p11_raw_hal_data(void) { return nav_rd(NAV_IMU_RX_DATA); }
uint32_t p11_raw_hal_timestamp(void) { return nav_rd(NAV_IMU_RX_TS); }
uint32_t p11_raw_hal_overflow(void) { return nav_rd(NAV_IMU_STAT_OVF); }
uint32_t p11_raw_hal_time_us(void) { return hal_time_us(); }
static void p11_set_protocol(bool selected)
{
    p11_protocol=selected;p11_epoch.clean_epoch=false;p11_last_raw_us=hal_time_us();
    nav_wr(NAV_IMU_BAUD_DIV,selected?217u:868u);nav_wr(NAV_IMU_IDLE_US,500u);
    imu_parser_init(&g_app.imu,IMU_PROTO_WIT55);
}
/* The quiet flag is never synthesized by the parser: drain actual MMIO first.
 * Fixed work bound32; repeated flood/backlog remains quarantined. */
static void p11_raw_work(uint32_t now)
{
    if(!p11_protocol)return;
    if(p11.state==P11_WAIT_ZERO||p11.state==P11_ARMED){
        bool edge;p11_raw_service(&p11,&p11_epoch,!g_host_link_guard.locked,&edge);
        return;
    }
    bool had=false;
    for(unsigned i=0;i<P11_RAW_BYTE_QUOTA;i++){
        uint32_t data=p11_raw_hal_data();if(!(data&0x80000000u))break;
        had=true;if(data&0x100u)(void)p11_raw_hal_timestamp();
    }
    if(had||p11_raw_hal_count()){
        p11_last_raw_us=now;p11_epoch.clean_epoch=false;return;
    }
    /* Extra quiet wait concerns flushing only, not a changed motion deadline.
     * Existing idle delimiter500us + a full43.4us character fit inside2ms. */
    if((uint32_t)(now-p11_last_raw_us)>=2000u){
        for(unsigned i=0;i<34;i++)(void)p11_raw_hal_timestamp();
        p11_epoch.overflow_baseline=p11_raw_hal_overflow();
        p11_epoch.clean_epoch=p11_raw_hal_count()==0;
    }
}
static void p11_note_proposal(uint32_t now)
{
    int16_t v=0,w=0;
    g_app.nt.manual_owner=p11_guard_proposal(&p11,now,!g_host_link_guard.locked,&v,&w);
    g_app.nt.manual_v=v;g_app.nt.manual_w=w;
    if(g_app.nt.manual_owner&&p11.state!=P11_ARMED){
        if(!p11_zero_published){
            nav_cmd_t zero={0};hal_send_cmd(&zero);p11_zero_published=true;
        }
    }else p11_zero_published=false;
}
void nav_app_p11_final_filter(uint32_t now,nav_cmd_t *cmd)
{
    p11_guard_poll(&p11,now,!g_host_link_guard.locked);
    /* Zero-only final filter: never replace a rejected/limited proposal. */
    if(p11.state!=P11_OWNER_OFF&&p11.state!=P11_ARMED)cmd->vx=cmd->vy=cmd->wz=0;
}
static void p11_manage(const proto_msg_t *m)
{
    nav_task_t *nt=&g_app.nt;uint32_t now=hal_time_us();
    uint32_t action=m->u.uart.action,result=P11_OK;
    if(action){
        if(!g_host_link_guard.locked&&nt->mode==NAV_MODE_IDLE&&
           !g_app.pm_pend&&!g_app.pm_run&&p11_guard_stationary(&p11,now)){
            if(action==1&&!p11_protocol)p11_set_protocol(true);
            result=p11_guard_manage(&p11,action==3?P11_RELEASE:(p11_action_t)action,
                       m->u.uart.session,now,true,true,p11_epoch.clean_epoch);
            if(result==P11_OK){
                nt->tgt.valid=false;nt->wp_n=nt->wp_i=0;nt->wp_loop=false;
                nt->path_valid=false;nt->path.n=0;nt->search_goal_valid=false;nt->hold_wz=0;
                nt->cmd=(nav_cmd_t){0};hal_send_cmd(&nt->cmd);
                if(action==3)p11_set_protocol(false);
            }
        }else result=g_host_link_guard.locked?P11_NOT_TRUSTED:
            nt->mode!=NAV_MODE_IDLE?P11_NOT_IDLE:P11_NOT_STILL;
    }
    p11_note_proposal(now);uint8_t reply[28];
    uint32_t age=p11.have_arrival?(uint32_t)(now-p11.arrival_us):0xffffffffu;
    int n=proto_build_uart_ack(reply,result,p11.session,(uint32_t)p11.state|
            ((uint32_t)p11_protocol<<8),age,p11.accepted,p11.rejected);
    hal_host_send(reply,n);
}
'''

def prepare(out):
 assert not out.exists();out.mkdir();shutil.copytree(SRC/'fw',out/'fw');changed=[]
 def edit(rel,fn):
  p=out/'fw'/rel;a=p.read_text(encoding='utf-8-sig');b=fn(a);assert a!=b,rel;p.write_text(b,encoding='utf8');changed.append(rel)
 def replace(t,a,b):assert t.count(a)==1,(a,t.count(a));return t.replace(a,b)
 edit('nav/proto.h',lambda t:replace(replace(replace(t,'MSG_SLF_CONFIG = 0x44,','MSG_SLF_CONFIG = 0x44, MSG_UART_OWNER = 0x45,'),'MSG_SLF_ACK = 0x84,','MSG_SLF_ACK = 0x84, MSG_UART_ACK = 0x85,'),'        struct { uint32_t set;','        struct { uint32_t action,session; } uart;\n        struct { uint32_t set;').replace('/* 组包：返回长度 */','int proto_build_uart_ack(uint8_t *b,uint32_t result,uint32_t session,uint32_t state,uint32_t age,uint32_t accepted,uint32_t rejected);\n/* 组包：返回长度 */'))
 edit('nav/proto.c',lambda t:replace(t,'    case MSG_SLF_CONFIG:',"    case MSG_UART_OWNER:\n        if(n!=8 || (uint32_t)rd32(p)>3u) return false;\n        m->u.uart.action=(uint32_t)rd32(p);m->u.uart.session=(uint32_t)rd32(p+4);return true;\n    case MSG_SLF_CONFIG:")+'''\nint proto_build_uart_ack(uint8_t *b,uint32_t result,uint32_t session,uint32_t state,uint32_t age,uint32_t accepted,uint32_t rejected)
{
    uint8_t *p=hdr(b,MSG_UART_ACK);p=wr32(p,result);p=wr32(p,session);p=wr32(p,state);
    p=wr32(p,age);p=wr32(p,accepted);p=wr32(p,rejected);return (int)(p-b);
}
''')
 edit('nav/host_link_guard.c',lambda t:replace(t,'case MSG_SLF_CONFIG: break;','case MSG_SLF_CONFIG: case MSG_UART_OWNER: break;'))
 edit('nav/nav_task.h',lambda t:replace(t,'    nav_mode_t mode;','    bool manual_owner; int16_t manual_v,manual_w;\n    nav_mode_t mode;'))
 edit('nav/motion_guard.h',lambda t:replace(t,'CMD_GROUND_EXIT };','CMD_GROUND_EXIT,CMD_P11 };'))
 edit('nav/nav_task.c',lambda t:replace(t,'    switch (nt->mode) {\n    case NAV_MODE_IDLE:\n    case NAV_MODE_ESTOP:',
   '    if(nt->manual_owner && nt->mode!=NAV_MODE_ESTOP){\n        nt->cmd=(nav_cmd_t){nt->manual_v,0,nt->manual_w,nt->mode};\n        nt->cmd_source=CMD_P11;nt->state=(nt->cmd.vx||nt->cmd.wz)?ST_MOVING:ST_IDLE;\n    }else switch (nt->mode) {\n    case NAV_MODE_IDLE:\n    case NAV_MODE_ESTOP:'))
 edit('hal/hal.h',lambda t:replace(t,'void hal_send_cmd(const nav_cmd_t *cmd);','void hal_send_cmd(const nav_cmd_t *cmd);\nvoid nav_app_p11_final_filter(uint32_t now,nav_cmd_t *cmd);'))
 edit('drv/hal_fpga.c',lambda t:replace(t,'    host_link_guard_filter(&g_host_link_guard,hal_time_us(),&guarded);','    host_link_guard_filter(&g_host_link_guard,hal_time_us(),&guarded);\n    nav_app_p11_final_filter(hal_time_us(),&guarded);'))
 def app(t):
  t=replace(t,'#include "host_link_guard.h"','#include "host_link_guard.h"\n#include "p11_raw_service.h"')
  t=replace(t,'static uint32_t host_pending_rx;','static uint32_t host_pending_rx;\n'+APP_HELPERS)
  t=replace(t,'    imu_parser_init(&g_app.imu, IMU_PROTO_WIT55);','    imu_parser_init(&g_app.imu, IMU_PROTO_WIT55);\n    p11_guard_init(&p11);p11_set_protocol(true);')
  t=replace(t,'        g_app.nt.mode!=NAV_MODE_IDLE ||','        g_app.nt.mode!=NAV_MODE_IDLE || p11.state!=P11_OWNER_OFF ||')
  t=replace(t,'    switch (m->type) {\n    case MSG_SET_POSE:',
    '''    if(m->type==MSG_UART_OWNER){p11_manage(m);return;}
    if(p11.state!=P11_OWNER_OFF){
        if(m->type==MSG_MODE&&(m->u.mode.mode==NAV_MODE_IDLE||m->u.mode.mode==NAV_MODE_ESTOP))p11_guard_quarantine(&p11);
        else if(!(m->type==MSG_TARGET&&!m->u.target.found))return;
    }
    switch (m->type) {
    case MSG_SET_POSE:''')
  t=replace(t,'static void apply_host_link_lock(uint32_t now)\n{','static void apply_host_link_lock(uint32_t now)\n{\n    p11_note_proposal(now);')
  t=replace(t,'bool nav_app_search_checkpoint(void)\n{','bool nav_app_search_checkpoint(void)\n{\n    p11_raw_work(hal_time_us());')
  t=replace(t,'    while (hal_odom(&vx, &vy, &wz, &t)) pose_odom(&nt->pose, vx, vy, wz, t);',
    '''    while (hal_odom(&vx, &vy, &wz, &t)) {
        p11_guard_feedback(&p11,vx,wz,t,hal_time_us(),nt->cmd.vx==0&&nt->cmd.vy==0&&nt->cmd.wz==0);
        pose_odom(&nt->pose,vx,vy,wz,t);
    }''')
  t=replace(t,'    for (int i = 0; i < 64 && hal_imu_byte(&b, &first, &tb); i++) {',
    '    p11_raw_work(hal_time_us());\n    for (int i = 0; !p11_protocol && i < 64 && hal_imu_byte(&b, &first, &tb); i++) {')
  return t
 edit('app/nav_app.c',app)
 patches=[];unchanged=[]
 for p in sorted((out/'fw').rglob('*')):
  if not p.is_file():continue
  rel=p.relative_to(out/'fw');old=SRC/'fw'/rel
  if old.read_bytes()==p.read_bytes():unchanged.append(str(rel))
  else:patches+=list(difflib.unified_diff(old.read_text().splitlines(True),p.read_text().splitlines(True),fromfile='a/fw/'+str(rel),tofile='b/fw/'+str(rel)))
 (out/'integration.patch').write_text(''.join(patches),encoding='utf8')
 files=[metadata(p)for p in sorted((out/'fw').rglob('*'))if p.is_file()]
 save(out/'manifest.json',dict(at=datetime.now().astimezone().isoformat(),source=str(SRC),stage='integrated experimental FW; no validation or link yet',modified_files=changed,unchanged_files=unchanged,all_FW_bindings=files,
   production_53572_BIN_43_unchanged=True,RTL_changed=False,hardware_owner_remains_NAV=True,manual_selected_before_common_motion_and_support=True,final_filter_zero_only=True,WIT_runtime_action3_preserved=True,
   management='45/85; action0 query,1 authorize P11,2 release NAV with P11 listener retained,3 release/select legacy WIT; fresh distinct session required for changes',
   pending=['Native full proto/app/MMIO integration fixture','quiet flush and source switching against real MMIO semantics','RV original flags LTO/GC link/strict stack/BSS budget','physical350/500ms stop and actual RK/UI integration'],new_ELF_BIN_INIT=False,deployment_ready=False))
 print(json.dumps(dict(output=str(out),modified_files=changed,unchanged_files=len(unchanged),production_changed=False),ensure_ascii=False))

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=BASE/'P11_shared_guard_FW_integrated_experiment_prepared1');a=p.parse_args();prepare(a.out)
