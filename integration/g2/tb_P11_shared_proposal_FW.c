/* Reuse the unchanged strong finite app contract, then exercise the real new
 * source branch and original common consumer body. Map/motion/support below
 * are explicit substitutes; this is neither full APP nor physical stopping. */
#define main previous_quiesce_contract
#include "tb_P11_quiesce_FW.c"
#undef main
#include "support_guard.h"
static uint8_t test_motion=MOTION_OK;
static int test_support=SUPPORT_PASS;
static bool test_map=true;
static unsigned motion_calls,support_calls;
static uint32_t support_delay;
static nav_cmd_t motion_seen,support_seen;
bool hal_map_info(hal_map_info_t *mi){*mi=g_app.nt.mi;return test_map;}
bool motion_map_same(const hal_map_info_t *a,const hal_map_info_t *b)
{(void)a;(void)b;return true;}
uint8_t motion_check(const uint8_t *map,const hal_map_info_t *mi,
 const nav_pose_t *p,const lp_cfg_t *cfg,const nav_cmd_t *cmd)
{(void)map;(void)mi;(void)p;(void)cfg;motion_calls++;motion_seen=*cmd;return test_motion;}
int support_check(const nav_pose_t *p,const lp_cfg_t *cfg,
 const hal_map_info_t *mi,const nav_cmd_t *cmd)
{(void)p;(void)cfg;(void)mi;support_calls++;support_seen=*cmd;now_us+=support_delay;return test_support;}
void support_note_cmd(const nav_cmd_t *cmd){(void)cmd;}
static void actual_source_and_checked_exit(nav_task_t *nt,uint32_t now,bool fresh)
{
    const nav_pose_t *p=&nt->pose.cur;
    nt->cmd=(nav_cmd_t){0};
#include "actual_direct_proposal_branch.inc"
    else {nt->cmd.mode=(uint8_t)nt->mode;nt->cmd_source=CMD_ESTOP;}
#include "actual_common_send_checked.inc"
}
static void arm(void)
{
    app_setup();app_manage(1,1);hw_ready=true;
    uint32_t auth=p11.authorized_us;now_us=auth+2000;
    publish(0,0,auth+1000);read_index=0;p11_raw_work(now_us);
    now_us=auth+21000;publish(100,20,auth+20000);read_index=0;p11_raw_work(now_us);
    CHECK(p11.state==P11_ARMED);
    g_app.nt.mode=NAV_MODE_IDLE;g_app.nt.have_map=true;
    test_motion=MOTION_OK;test_support=SUPPORT_PASS;test_map=true;
    support_delay=0;motion_calls=support_calls=0;
}
int main(void)
{
    CHECK(previous_quiesce_contract()==0);
    arm();actual_source_and_checked_exit(&g_app.nt,now_us,true);
    CHECK(motion_calls==1&&support_calls==1);
    CHECK(motion_seen.vx==100&&motion_seen.wz==20&&motion_seen.vy==0);
    CHECK(support_seen.vx==100&&support_seen.wz==20);
    CHECK((int16_t)last_safe_v==100&&(int16_t)last_safe_w==20);
    /* No stale nav_task command cache exists: the next snapshot is consumed
       once and the branch samples the current guarded tuple directly. */
    now_us+=20000;publish(200,-40,now_us-500);read_index=0;p11_raw_work(now_us);
    actual_source_and_checked_exit(&g_app.nt,now_us,true);
    CHECK(motion_seen.vx==200&&motion_seen.wz==-40);
    arm();test_motion=MOTION_HAZARD;actual_source_and_checked_exit(&g_app.nt,now_us,true);
    CHECK(!support_calls&&!last_safe_v&&!(last_safe_w&65535u));
    arm();test_support=SUPPORT_BLOCK;actual_source_and_checked_exit(&g_app.nt,now_us,true);
    CHECK(support_calls==1&&!last_safe_v&&!(last_safe_w&65535u));
    arm();actual_source_and_checked_exit(&g_app.nt,now_us,false);
    CHECK(!motion_calls&&!last_safe_v&&!(last_safe_w&65535u));
    arm();g_app.nt.slow=true;actual_source_and_checked_exit(&g_app.nt,now_us,true);
    CHECK(motion_seen.vx==50&&motion_seen.wz==10);
    /* The direct proposal never supersedes original ESTOP selection. */
    arm();g_app.nt.mode=NAV_MODE_ESTOP;actual_source_and_checked_exit(&g_app.nt,now_us,true);
    CHECK(!motion_calls&&!support_calls&&!last_safe_v&&!(last_safe_w&65535u));
    CHECK((last_safe_w>>16)==NAV_MODE_ESTOP);
    /* A long support check can expire a valid proposal: final HAL is zero-only. */
    arm();support_delay=200000;actual_source_and_checked_exit(&g_app.nt,now_us,true);
    CHECK(motion_seen.vx==100&&support_seen.vx==100&&p11.state==P11_LOCKED);
    CHECK(!last_safe_v&&!(last_safe_w&65535u));
    /* Immediate zero at exactly one new lock edge, without waiting for task_step. */
    arm();now_us+=200000;feedback(&p11,now_us,false);unsigned z=zero_writes;
    read_index=0;p11_raw_work(now_us);CHECK(p11.state==P11_LOCKED&&zero_writes==z+1);
    p11_raw_work(now_us);CHECK(zero_writes==z+1);
    nav_cmd_t cmd={9,0,8,0};CHECK(nav_app_p11_proposal(now_us,&cmd)&&!cmd.vx&&!cmd.wz);
    CHECK(zero_writes==z+1);
    arm();z=zero_writes;p11_zero_edge(p11_guard_quarantine(&p11));
    CHECK(p11.state==P11_LOCKED&&zero_writes==z+1);
    p11_poll_stop(now_us);CHECK(zero_writes==z+1);
    /* OWNER_OFF is an absent proposal; it cannot clobber an ordinary NAV tuple. */
    app_setup();cmd=(nav_cmd_t){123,2,-45,7};CHECK(!nav_app_p11_proposal(now_us,&cmd));
    CHECK(cmd.vx==123&&cmd.vy==2&&cmd.wz==-45&&cmd.mode==7);
    printf("PASS P11_SHARED_PROPOSAL checks=%u actual_branch_and_common_tail=1 motion_support_are_substitutes=1 no_postcheck_restore=1 independent_lock_edge_zero=1 no_command_cache=1\n",checks);
    return 0;
}
