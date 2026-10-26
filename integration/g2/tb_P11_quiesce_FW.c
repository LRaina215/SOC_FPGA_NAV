/* The hardware-ready bit is an explicit proposed CSR substitute. This tests
 * software reactions, not implementation or timing of the quiesce hardware. */
#include "csr_model_actual_app.inc"
int main(void)
{
    app_setup();hw_seq=7;hw_word=600;hw_ts=now_us;raw_n=258;raw_tn=34;parser_enabled=true;hw_ready=true;
    p11_guard_init(&p11);host_link_guard_init(&g_host_link_guard,now_us);p11_set_protocol(0);
    CHECK(!hw_seq&&!hw_word&&!hw_ts&&!raw_n&&!raw_tn&&!parser_enabled&&!hw_ready);
    CHECK(p11.state==P11_OWNER_OFF&&g_host_link_guard.locked&&p11_source_pending);
    nav_cmd_t cmd={100,0,20,0};hal_send_cmd(&cmd);CHECK(!last_safe_v&&!(last_safe_w&65535u));
    /* A stuck-low/not-ready source must not spin or deferP11/R0deadline polls. */
    app_setup();app_manage(1,1);CHECK(p11.state==P11_WAIT_ZERO&&p11_source_pending&&!hw_ready&&parser_enabled);
    uint32_t auth=p11.authorized_us;proto_msg_t beat={.type=MSG_TARGET};
    for(unsigned i=1;i<=21;i++){
        now_us=auth+i*10000;feedback(&p11,now_us,true);
        CHECK(host_link_guard_observe(&g_host_link_guard,&beat,now_us,now_us));
        unsigned before=ready_reads;p11_raw_work(now_us);CHECK(ready_reads==before+1);
        CHECK(!p11.have_arrival);hal_send_cmd(&cmd);CHECK(!last_safe_v&&!(last_safe_w&65535u));
    }
    CHECK(p11.state==P11_LOCKED);hw_ready=true;publish(0,0,now_us);read_index=0;p11_raw_work(now_us);
    CHECK(p11.state==P11_LOCKED); /* READY cannot heal the latchedstop. */
    /* Explicit trustednewsession, followed by actualTS/newzero, is required. */
    app_manage(1,2);CHECK(p11.state==P11_WAIT_ZERO&&!hw_ready&&p11_source_pending);
    auth=p11.authorized_us;now_us=auth+1000;hw_ready=true;
    publish(0,0,auth-1);read_index=0;p11_raw_work(now_us);CHECK(p11.state==P11_WAIT_ZERO&&p11_sequence==1);
    now_us=auth+2000;publish(0,0,auth+1100);read_index=0;p11_raw_work(now_us);CHECK(p11.state==P11_ARMED);
    now_us=auth+21000;publish(100,20,auth+20000);read_index=0;p11_raw_work(now_us);hal_send_cmd(&cmd);
    CHECK((int16_t)last_safe_v==100&&(int16_t)last_safe_w==20);
    /* ContinuingP11 snapshots do not renew the actualR0 hostguard. */
    uint32_t lastR0=g_host_link_guard.last_rx_us;
    for(unsigned i=3;i<=30;i++){
        now_us=auth+i*10000;feedback(&p11,now_us,false);host_link_guard_poll(&g_host_link_guard,now_us);
        publish(100,20,now_us);read_index=0;p11_raw_work(now_us);hal_send_cmd(&cmd);
        if((uint32_t)(now_us-lastR0)>=250000)CHECK(!last_safe_v&&!(last_safe_w&65535u));
    }
    CHECK(g_host_link_guard.locked&&p11.state==P11_LOCKED);
    /* ExplicitWIT mode usesonlysource-ready, with no softwareDATA/TSdrain.
       The simultaneous queueclear here is the PROPOSED hardware model. */
    app_setup();raw_n=258;raw_tn=34;g_app.imu.n=7;app_manage(3,3);
    CHECK(!raw_n&&!raw_tn&&!p11_protocol&&p11_source_pending&&!hw_ready&&baud==868&&g_app.imu.n==0);
    unsigned before=ready_reads;
    for(unsigned i=0;i<10;i++){now_us+=100;p11_raw_work(now_us);hal_send_cmd(&cmd);CHECK(!last_safe_v&&!(last_safe_w&65535u));}
    CHECK(ready_reads==before+10&&p11_source_pending);
    hw_ready=true;p11_raw_work(now_us);CHECK(!p11_source_pending&&g_app.imu.n==0);
    printf("PASS P11_QUIESCE_SOFTWARE checks=%u one_ready_read_per_pending_call=1 pending_TTL200ms=1 R0_timeout250ms=1 warmHOLD_flush=1 old258B34TS_clear_IS_CSR_SUBSTITUTE_ONLY=1\n",checks);
    return 0;
}
