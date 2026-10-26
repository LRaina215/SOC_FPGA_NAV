"""Freeze software-only snapshot P11 experiment, never a selected image.

The four CSRs are proposed hardware interfaces, implemented only by an explicit
test substitute until a separately authorized RTL candidate exists.
"""
from pathlib import Path
from datetime import datetime
import argparse,difflib,json,shutil
from prepare_baseline_fusion_retime import metadata,save

BASE=Path('F:/nav_integration_g2_20261023')
SRC=BASE/'P11_shared_guard_FW_integrated_experiment_prepared3'

HEADER=r'''#ifndef P11_COMMAND_GUARD_H
#define P11_COMMAND_GUARD_H
#include <stdbool.h>
#include <stdint.h>
#define P11_COMMAND_AGE_US 200000u
#define P11_FEEDBACK_AGE_US 50000u
#define P11_STILL_DWELL_US 500000u
typedef enum{P11_OWNER_OFF,P11_WAIT_ZERO,P11_ARMED,P11_LOCKED}p11_state_t;
typedef enum{P11_QUERY,P11_AUTHORIZE,P11_RELEASE}p11_action_t;
typedef enum{P11_OK,P11_NOT_TRUSTED,P11_NOT_IDLE,P11_NOT_STILL,P11_NOT_DRAINED,P11_BAD_SESSION,P11_BAD_ACTION}p11_result_t;
typedef struct{
    p11_state_t state;
    uint32_t session,authorized_us,arrival_us,feedback_us,still_since_us;
    int16_t forward_mmps,yaw_mradps;
    bool have_arrival,have_feedback,still;
}p11_command_guard_t;
void p11_guard_init(p11_command_guard_t *g);
void p11_guard_feedback(p11_command_guard_t *g,int16_t vx,int16_t wz,uint32_t stamp,uint32_t now,bool published_zero);
bool p11_guard_stationary(const p11_command_guard_t *g,uint32_t now);
p11_result_t p11_guard_manage(p11_command_guard_t *g,p11_action_t action,uint32_t session,uint32_t now,bool trusted_R0,bool nav_idle,bool source_epoch_ready);
bool p11_guard_poll(p11_command_guard_t *g,uint32_t now,bool R0_permit);
bool p11_guard_quarantine(p11_command_guard_t *g);
/* The source is a coherent new hardware snapshot, never CPU-arrival time.
 * Authorization half-cycle comparison applies ONLY while waiting for zero.
 * ARMED sessions may run beyond2^31us; per-command age/order remain bounded. */
bool p11_guard_capture(p11_command_guard_t *g,int16_t forward,int16_t yaw,uint32_t stamp,uint32_t now,bool R0_permit);
bool p11_guard_proposal(p11_command_guard_t *g,uint32_t now,bool R0_permit,int16_t *forward,int16_t *yaw);
#endif
'''

SNAP_H=r'''#ifndef P11_SNAPSHOT_SERVICE_H
#define P11_SNAPSHOT_SERVICE_H
#include "p11_command_guard.h"
typedef struct{int16_t forward_mmps,yaw_mradps;uint32_t start_us,sequence;}p11_snapshot_t;
typedef enum{P11_SNAPSHOT_NONE,P11_SNAPSHOT_NEW,P11_SNAPSHOT_UNSTABLE,P11_SNAPSHOT_LOST}p11_snapshot_result_t;
uint32_t p11_snapshot_hal_seq(void);
uint32_t p11_snapshot_hal_command(void);
uint32_t p11_snapshot_hal_timestamp(void);
uint32_t p11_snapshot_hal_time_us(void);
p11_snapshot_result_t p11_snapshot_read(uint32_t previous,p11_snapshot_t *out);
/* Returns the actual new lock edge for the app's immediate common-HAL zero.
 * <=4snapshot CSR reads plus actual SYS_TIME after reading, never an unbounded retry. */
bool p11_snapshot_service(p11_command_guard_t *g,uint32_t *previous,bool R0_permit);
#endif
'''

SNAP_C=r'''#include "p11_snapshot_service.h"
p11_snapshot_result_t p11_snapshot_read(uint32_t previous,p11_snapshot_t *out)
{
    uint32_t a=p11_snapshot_hal_seq();
    uint32_t word=p11_snapshot_hal_command();
    uint32_t stamp=p11_snapshot_hal_timestamp();
    uint32_t b=p11_snapshot_hal_seq();
    if(a!=b)return P11_SNAPSHOT_UNSTABLE;
    if(!b)return previous?P11_SNAPSHOT_LOST:P11_SNAPSHOT_NONE;
    if(b==0xffffffffu)return P11_SNAPSHOT_LOST;
    if(b==previous)return P11_SNAPSHOT_NONE;
    if((uint32_t)(b-previous)!=1u)return P11_SNAPSHOT_LOST;
    out->forward_mmps=(int16_t)word;out->yaw_mradps=(int16_t)(word>>16);
    out->start_us=stamp;out->sequence=b;return P11_SNAPSHOT_NEW;
}
bool p11_snapshot_service(p11_command_guard_t *g,uint32_t *previous,bool R0_permit)
{
    bool edge=p11_guard_poll(g,p11_snapshot_hal_time_us(),R0_permit);
    if(g->state!=P11_WAIT_ZERO&&g->state!=P11_ARMED)return edge;
    p11_snapshot_t value;p11_snapshot_result_t r=p11_snapshot_read(*previous,&value);
    uint32_t now=p11_snapshot_hal_time_us();
    edge=p11_guard_poll(g,now,R0_permit)||edge;
    if(r==P11_SNAPSHOT_LOST)return p11_guard_quarantine(g)||edge;
    if(r==P11_SNAPSHOT_NEW){
        *previous=value.sequence;
        p11_guard_capture(g,value.forward_mmps,value.yaw_mradps,value.start_us,now,R0_permit);
    }
    return edge;
}
'''

APP_HEAD=r'''/* Explicit software owner; hardware only decodes wire frames. */
static p11_command_guard_t p11;
static uint32_t p11_sequence,p11_last_raw_us;
static bool p11_protocol,p11_zero_published,p11_raw_quiet,p11_wit_pending;
/* mode0: BOOT P11 desired but parser disabled;1: explicitly authorized P11;
   mode2: disabled parser and quarantined WIT handover. */
static void p11_set_protocol(unsigned mode)
{
    bool selected=mode!=2;
    hal_p11_parser_control(0); /* serialize flush before changing baud/owner */
    p11_protocol=selected;p11_wit_pending=!selected;p11_sequence=0;p11_raw_quiet=false;
    p11_last_raw_us=hal_time_us();hal_raw_uart_config(selected?217u:868u,500u);
    imu_parser_init(&g_app.imu,IMU_PROTO_WIT55);
    if(mode==1)hal_p11_parser_control(1);
}
static void p11_raw_work(uint32_t now)
{
    if(!p11_protocol&&!p11_wit_pending)return;
    if(p11_protocol&&p11_snapshot_service(&p11,&p11_sequence,!g_host_link_guard.locked)){
        nav_cmd_t zero={0};hal_send_cmd(&zero);p11_zero_published=true;
    }
    /* Preserve the raw/WIT facility without making it the command parser's
       input. Cleanup is bounded and cannot block snapshot consumption. */
    uint8_t b;bool first;uint32_t stamp=0;unsigned n=0;
    while(n<32&&hal_imu_byte(&b,&first,&stamp))n++;
    if(n||p11_raw_hal_count()){
        p11_last_raw_us=now;p11_raw_quiet=false;
    }else if((uint32_t)(now-p11_last_raw_us)>=2000u){
        for(unsigned i=0;i<34;i++)(void)p11_raw_hal_timestamp();
        p11_raw_quiet=p11_raw_hal_count()==0;
        if(p11_raw_quiet&&p11_wit_pending){
            imu_parser_init(&g_app.imu,IMU_PROTO_WIT55);p11_wit_pending=false;
        }
    }
}
'''

MANAGE=r'''static void p11_manage(const proto_msg_t *m)
{
    nav_task_t *nt=&g_app.nt;uint32_t now=hal_time_us();
    uint32_t action=m->u.uart.action,result=P11_OK;
    if(action){
        if(!g_host_link_guard.locked&&nt->mode==NAV_MODE_IDLE&&
           !g_app.pm_pend&&!g_app.pm_run&&p11_guard_stationary(&p11,now)){
            bool ready=action!=3||(p11_raw_quiet&&p11_raw_hal_count()==0);
            /* Reject session/quiet errors BEFORE touching the hardware epoch. */
            result=p11_guard_manage(&p11,action==3?P11_RELEASE:(p11_action_t)action,
                       m->u.uart.session,now,true,true,ready);
            if(result==P11_OK){
                if(action==1){
                    p11_set_protocol(1);
                    p11.authorized_us=hal_time_us(); /* after ENABLE write ACK */
                }else if(action==3)p11_set_protocol(2);
                nt->tgt.valid=false;nt->wp_n=nt->wp_i=0;nt->wp_loop=false;
                nt->path_valid=false;nt->path.n=0;nt->search_goal_valid=false;nt->hold_wz=0;
                nt->cmd=(nav_cmd_t){0};hal_send_cmd(&nt->cmd);
            }
        }else result=g_host_link_guard.locked?P11_NOT_TRUSTED:
            nt->mode!=NAV_MODE_IDLE?P11_NOT_IDLE:P11_NOT_STILL;
    }
    p11_note_proposal(hal_time_us());uint8_t reply[16];
    int n=proto_build_uart_ack(reply,result,p11.session,(uint32_t)p11.state|
            ((uint32_t)p11_protocol<<8)|((uint32_t)p11_wit_pending<<9));hal_host_send(reply,n);
}
'''

def prepare(out):
    assert not out.exists();shutil.copytree(SRC,out)
    def edit(rel,fn):
        p=out/'fw'/rel;t=p.read_text(encoding='utf-8-sig');n=fn(t);assert t!=n,rel;p.write_text(n,encoding='utf8')
    (out/'fw/nav/p11_command_guard.h').write_text(HEADER,encoding='utf8')
    def core(t):
        a=t.index('#if defined(__GNUC__)');b=t.index('static bool lock_zero')
        t=t[:a]+t[b:]
        t=t.replace('g->have_arrival=false;g->count=0;g->have_burst=false;','g->have_arrival=false;')
        t=t.replace('    g->count=0;g->total=0;g->have_burst=false;g->burst_used=false;\n','')
        a=t.index('bool p11_guard_byte(');b=t.index('bool p11_guard_proposal(',a)
        capture='''bool p11_guard_capture(p11_command_guard_t *g,int16_t v,int16_t w,uint32_t stamp,uint32_t now,bool permit)
{
    p11_guard_poll(g,now,permit);
    if(g->state!=P11_WAIT_ZERO&&g->state!=P11_ARMED)return false;
    if(!stamp||(uint32_t)(now-stamp)>=P11_COMMAND_AGE_US)return false;
    if(g->state==P11_WAIT_ZERO&&(int32_t)(stamp-g->authorized_us)<0)return false;
    if(g->have_arrival&&(int32_t)(stamp-g->arrival_us)<=0)return false;
    if(v< -600||v>600||w< -2900||w>2900)return false;
    if(g->state==P11_WAIT_ZERO&&(v||w||!p11_guard_stationary(g,now)))return false;
    g->forward_mmps=v;g->yaw_mradps=w;g->arrival_us=stamp;
    g->have_arrival=true;g->state=P11_ARMED;return true;
}
'''
        return t[:a]+capture+t[b:]
    edit('nav/p11_command_guard.c',core)
    for n in('p11_raw_service.c','p11_raw_service.h'):(out/'fw/nav'/n).unlink()
    (out/'fw/nav/p11_snapshot_service.c').write_text(SNAP_C,encoding='utf8')
    (out/'fw/nav/p11_snapshot_service.h').write_text(SNAP_H,encoding='utf8')
    def app(t):
        t=t.replace('#include "p11_raw_service.h"','#include "p11_snapshot_service.h"')
        a=t.index('/* The P11 raw UART');b=t.index('static void p11_note_proposal',a)
        t=t[:a]+APP_HEAD+t[b:]
        a=t.index('static void p11_manage(');b=t.index('\n\n\nstatic void app_match_update',a)
        t=t[:a]+MANAGE+t[b:]
        t=t.replace('if(p11.state!=P11_OWNER_OFF&&p11.state!=P11_ARMED)cmd->vx=cmd->vy=cmd->wz=0;',
                    'if(p11_wit_pending||(p11.state!=P11_OWNER_OFF&&p11.state!=P11_ARMED))cmd->vx=cmd->vy=cmd->wz=0;')
        t=t.replace('g_app.nt.mode!=NAV_MODE_IDLE || p11.state!=P11_OWNER_OFF ||',
                    'g_app.nt.mode!=NAV_MODE_IDLE || p11.state!=P11_OWNER_OFF || p11_wit_pending ||')
        t=t.replace('if(p11.state!=P11_OWNER_OFF){','if(p11.state!=P11_OWNER_OFF||p11_wit_pending){')
        t=t.replace('p11_guard_init(&p11);p11_set_protocol(true);','p11_guard_init(&p11);p11_set_protocol(0);')
        return t.replace('!p11_protocol && i < 64','!p11_protocol && !p11_wit_pending && i < 64')
    edit('app/nav_app.c',app)
    def hal(t):
        t=t.replace('#include "p11_raw_service.h"','#include "p11_snapshot_service.h"')
        a=t.index('uint32_t p11_raw_hal_count(');b=t.index('void hal_raw_uart_config(',a)
        return t[:a]+'''uint32_t p11_raw_hal_count(void){return nav_rd(NAV_IMU_RX_COUNT)&1023u;}
uint32_t p11_raw_hal_timestamp(void){return nav_rd(NAV_IMU_RX_TS);}
uint32_t p11_snapshot_hal_seq(void){return nav_rd(NAV_IMU_P11_SEQ);}
uint32_t p11_snapshot_hal_command(void){return nav_rd(NAV_IMU_P11_CMD);}
uint32_t p11_snapshot_hal_timestamp(void){return nav_rd(NAV_IMU_P11_TS);}
uint32_t p11_snapshot_hal_time_us(void){return hal_time_us();}
void hal_p11_parser_control(uint32_t enable){nav_wr(NAV_IMU_P11_CTRL,enable);}
'''+t[b:]
    edit('drv/hal_fpga.c',hal)
    edit('hal/hal.h',lambda t:t.replace('void hal_raw_uart_config(uint32_t baud_div,uint32_t idle_us);',
        'void hal_raw_uart_config(uint32_t baud_div,uint32_t idle_us);\nvoid hal_p11_parser_control(uint32_t enable);\nuint32_t p11_raw_hal_count(void);\nuint32_t p11_raw_hal_timestamp(void);'))
    edit('include/nav_regs_map.h',lambda t:t.replace('#endif','''/* Proposed parser-offload CSR contract; current43 does not implement it. */
#define NAV_IMU_P11_CTRL 0x6020u
#define NAV_IMU_P11_CMD 0x6024u
#define NAV_IMU_P11_TS 0x6028u
#define NAV_IMU_P11_SEQ 0x602Cu
#endif'''))
    files=[metadata(p)for p in sorted((out/'fw').rglob('*'))if p.is_file()]
    changed=[];patch=[]
    for p in sorted((out/'fw').rglob('*')):
        if not p.is_file():continue
        rel=p.relative_to(out/'fw');old=SRC/'fw'/rel
        if not old.is_file()or old.read_bytes()!=p.read_bytes():
            changed.append(str(rel))
            patch.extend(difflib.unified_diff(old.read_text().splitlines(True)if old.is_file()else[],p.read_text().splitlines(True),fromfile='old/'+str(rel),tofile='snapshot/'+str(rel)))
    (out/'software_snapshot.patch').write_text(''.join(patch),encoding='utf8')
    save(out/'snapshot_manifest.json',dict(at=datetime.now().astimezone().isoformat(),source=str(SRC),
        phase='software-only experimental snapshot replacement',changed_files=changed,all_FW_bindings=files,
        removed_only_new_raw_parser_files=['nav/p11_raw_service.c','nav/p11_raw_service.h'],
        proposed_CSRs_not_in_current_hardware=True,all_current_RTL_and_selected_FW_unchanged=True,
        raw_WIT_cleanup_quota_bytes=32,command_source='four snapshot reads; actual hardware FIRST timestamp, never cleanup/dequeue time',
        long_session_fix='authorization halfcycle applies WAIT_ZERO only; ARMED checks short per-command age/order',
        WIT_transition='after disable/baud868, bounded paired raw drain and a new2ms quiet period; parser halfstate cleared both before/after; final HAL zero and NV motion rejected during pending',
        CPU_HOLD_boot='cpu_rst=rst|hold; init explicitlyCTRL0 flush/disable, ownerOFF and R0locked; only explicit NV authorization enables parser',
        sequence_assumptions=['FW noIRQ exclusively serializes CTRL writes outside four reads','common rst also restartsCPU/owner'],
        new_BIN_INIT=False,EDA=False,deployment_ready=False))
    print(json.dumps(dict(output=str(out),changed=changed,hardware_changed=False),ensure_ascii=False))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=BASE/'P11_snapshot_shared_guard_FW_prepared1');prepare(p.parse_args().out)
