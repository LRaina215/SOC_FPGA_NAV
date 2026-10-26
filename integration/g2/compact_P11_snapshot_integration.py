"""One structural cleanup of new app glue, preserving frozen predecessors.

Reuse the common guard checker and zero-publication helper. Raw UART draining
is needed only during the explicit quarantined WIT handover: hardware command
parsing is independent of both raw FIFOs. No old application feature is cut.
"""
from pathlib import Path
from datetime import datetime
import argparse,difflib,shutil
from prepare_baseline_fusion_retime import metadata,save
from prepare_P11_snapshot_FW_experiment import MANAGE
BASE=Path('F:/nav_integration_g2_20261023');SRC=BASE/'P11_snapshot_shared_guard_FW_prepared3'

def prepare(out):
    assert not out.exists();shutil.copytree(SRC,out)
    p=out/'fw/app/nav_app.c';before=p.read_text();t=before
    t=t.replace('static bool p11_protocol,p11_zero_published,p11_raw_quiet,p11_wit_pending;',
        'static bool p11_protocol,p11_zero_published,p11_wit_pending;\nstatic void p11_note_proposal(uint32_t now);')
    t=t.replace('p11_sequence=0;p11_raw_quiet=false;','p11_sequence=0;')
    a=t.index('static void p11_raw_work(');b=t.index('static void p11_note_proposal(uint32_t now)\n{',a)
    t=t[:a]+'''static void p11_raw_work(uint32_t now)
{
    if(p11_protocol){
        (void)p11_snapshot_service(&p11,&p11_sequence,!g_host_link_guard.locked);
        /* A poll/reader can latch the stop; the shared helper publishes zero
           from the latched state, so the edge cannot be lost by polling twice. */
        p11_note_proposal(hal_time_us());
    }
    if(!p11_wit_pending)return;
    /* Only an explicitly selected WIT handover consumes old raw queues.
       All bytes/timestamps remain discarded until a fresh post-switch quiet
       period and empty count; WIT parser and NAV motion stay quarantined. */
    uint8_t b;bool first;uint32_t stamp=0;unsigned n=0;
    while(n<32&&hal_imu_byte(&b,&first,&stamp))n++;
    if(n||p11_raw_hal_count())p11_last_raw_us=now;
    else if((uint32_t)(now-p11_last_raw_us)>=2000u){
        for(unsigned i=0;i<34;i++)(void)p11_raw_hal_timestamp();
        if(!p11_raw_hal_count()){
            imu_parser_init(&g_app.imu,IMU_PROTO_WIT55);p11_wit_pending=false;
        }
    }
}
'''+t[b:]
    a=t.index('static void p11_manage(');b=t.index('\n\n\nstatic void app_match_update',a)
    manage=MANAGE
    x=manage.index('        if(!g_host_link_guard.locked');y=manage.index('            if(result==P11_OK)',x)
    manage=manage[:x]+'''        result=p11_guard_manage(&p11,action==3?P11_RELEASE:(p11_action_t)action,
             m->u.uart.session,now,!g_host_link_guard.locked,
             nt->mode==NAV_MODE_IDLE&&!g_app.pm_pend&&!g_app.pm_run,true);
'''+manage[y:]
    manage=manage.replace('''        }else result=g_host_link_guard.locked?P11_NOT_TRUSTED:
            nt->mode!=NAV_MODE_IDLE?P11_NOT_IDLE:P11_NOT_STILL;
''','')
    t=t[:a]+manage+t[b:];assert t!=before
    p.write_text(t,encoding='utf8')
    (out/'app_structure_cleanup.patch').write_text(''.join(difflib.unified_diff(before.splitlines(True),t.splitlines(True),fromfile='prepared3/app/nav_app.c',tofile='prepared4/app/nav_app.c')),encoding='utf8')
    save(out/'snapshot_structure_manifest.json',dict(at=datetime.now().astimezone().isoformat(),source=str(SRC),
       changed_files=['fw/app/nav_app.c'],
       same_guard_service_files=[metadata(out/'fw/nav'/n)for n in('p11_command_guard.c','p11_command_guard.h','p11_snapshot_service.c','p11_snapshot_service.h')],
       reasons=['one common guard_manage checker instead of app+guard repeated checks',
                'one immediate-zero helper after snapshot polling rather than repeated localzero construction',
                'rawFIFO drain only inside WIT_DRAIN; parser does not depend on FIFOready'],
       WIT_handover_stays_quarantined=True,old_feature_removed=False,compiler_options_unchanged=True,
       new_BIN_INIT=False,all_FW_bindings=[metadata(x)for x in sorted((out/'fw').rglob('*'))if x.is_file()],
       RTL_or_PNR_inputs_changed=False,deployment_ready=False))
    print(str(out))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=BASE/'P11_snapshot_shared_guard_FW_prepared4');prepare(p.parse_args().out)
