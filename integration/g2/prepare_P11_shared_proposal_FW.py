"""One independent direct proposal source; never edit RTL or selected images."""
from pathlib import Path
from datetime import datetime
import argparse,difflib,json,shutil
from prepare_baseline_fusion_retime import metadata,save
BASE=Path('F:/nav_integration_g2_20261023')
SRC=BASE/'P11_quiesce_shared_guard_FW_prepared1'

def prepare(out):
    assert not out.exists();shutil.copytree(SRC,out)
    contract=dict(at=datetime.now().astimezone().isoformat(),source=str(SRC),
        old_call_flow='bounded raw service -> note_proposal -> manual_owner/manual_v/manual_w cache in nav_task -> original manual branch -> common motion/support -> HAL host/P11 zero filter -> SAFE/lease',
        new_call_flow='bounded raw service updates only guard snapshot -> direct static nav_app_p11_proposal at the exact old manual branch -> common motion/support -> identical HAL host/P11 zero filter -> SAFE/lease',
        immediate_stop='poll and snapshot/quarantine return real lock edges; publish a zero once at that edge through the same original HAL. No command tuple cache or later periodic planner is required for this edge',
        priorities=['existing no-map/HOLD and ground scan/exits retain their original earlier branches',
                    'original hard ESTOP branch bypasses P11 proposal',
                    'P11 owner OFF falls through to original NAV mode switch',
                    'WAIT_ZERO/LOCKED own the source but only propose zero; ARMED proposes the current guarded tuple',
                    'motion/map/support can reduce or reject the proposal; no P11 action restores it after these checks',
                    'HAL host and P11 filters can only zero the final command, including expiry during expensive support/planning work'],
        cache_lifetime='only p11_command_guard_t holds accepted signed16 tuple, real arrival timestamp and owner state; no duplicate command tuple/owner fields in nav_task',
        static_linkage='ordinary direct C call and declaration; no function pointer, callback registry or jalr introduced',
        unchanged_contracts='R0=250ms,P11=200ms,feedback=50ms,still dwell=500ms,original motion/support/SAFE/lease,CPU128KiB/stack6144,all original NAV functionality',
        proposed_hardware_dependency='quiesce plan2 remains unimplemented; READY and queue flush are test CSR substitutes',
        validation_plan=['rerun preserved quiesce software contract against this new app prefix',
                         'exercise actual selected source branch and exact original send_checked body with explicit motion/support/map substitutes, including no post-check restoration',
                         'reuse byte-identical four guard/snapshot C/H exhaustive finite proof; do not rerun unrelated long loop',
                         'one original flags full LTO link, strict stack only if legal link exists'],
        not_full_APP_Pico_or_hardware_proof=True)
    save(out/'shared_proposal_contract.json',contract)
    p=out/'fw/app/nav_app.c';old=p.read_text();t=old
    t=t.replace('static bool p11_protocol,p11_zero_published,p11_source_pending;\nstatic void p11_note_proposal(uint32_t now);',
                '''static bool p11_protocol,p11_source_pending;
static void p11_zero_edge(bool edge)
{
    if(edge){nav_cmd_t zero={0};hal_send_cmd(&zero);}
}
static void p11_poll_stop(uint32_t now)
{
    p11_zero_edge(p11_guard_poll(&p11,now,!g_host_link_guard.locked));
}
bool nav_app_p11_proposal(uint32_t now,nav_cmd_t *cmd)
{
    p11_poll_stop(now);
    return p11_guard_proposal(&p11,now,!g_host_link_guard.locked,&cmd->vx,&cmd->wz);
}''')
    a=t.index('static void p11_raw_work(');b=t.index('void nav_app_p11_final_filter(',a)
    t=t[:a]+'''static void p11_raw_work(uint32_t now)
{
    p11_poll_stop(now);
    if(p11_source_pending){
        if(!hal_p11_parser_ready())return;
        p11_source_pending=false;
    }
    if(p11_protocol)
        p11_zero_edge(p11_snapshot_service(&p11,&p11_sequence,!g_host_link_guard.locked));
}
'''+t[b:]
    t=t.replace('p11_note_proposal(hal_time_us());','p11_poll_stop(hal_time_us());').replace('p11_note_proposal(now);','p11_poll_stop(now);')
    t=t.replace('p11_guard_quarantine(&p11);','p11_zero_edge(p11_guard_quarantine(&p11));')
    assert 'manual_' not in t and 'p11_note_proposal' not in t and 'p11_zero_published' not in t
    p.write_text(t,encoding='utf8')
    p=out/'fw/nav/nav_task.h';t=p.read_text()
    assert t.count('    bool manual_owner; int16_t manual_v,manual_w;\n')==1
    t=t.replace('    bool manual_owner; int16_t manual_v,manual_w;\n','')
    t=t.replace('void nav_task_init(nav_task_t *nt);','/* Single statically linked source provider; all motion/support checks follow. */\nbool nav_app_p11_proposal(uint32_t now,nav_cmd_t *cmd);\nvoid nav_task_init(nav_task_t *nt);')
    p.write_text(t,encoding='utf8')
    p=out/'fw/nav/nav_task.c';t=p.read_text()
    oldbranch='''    if(nt->manual_owner && nt->mode!=NAV_MODE_ESTOP){
        nt->cmd=(nav_cmd_t){nt->manual_v,0,nt->manual_w,nt->mode};'''
    newbranch='''    if(nt->mode!=NAV_MODE_ESTOP && nav_app_p11_proposal(now,&nt->cmd)){
        nt->cmd.mode=(uint8_t)nt->mode;'''
    assert t.count(oldbranch)==1;t=t.replace(oldbranch,newbranch)
    assert 'manual_' not in t;p.write_text(t,encoding='utf8')
    changed=[];patch=[]
    for f in sorted((out/'fw').rglob('*')):
        if not f.is_file():continue
        r=f.relative_to(out/'fw');original=SRC/'fw'/r
        if f.read_bytes()!=original.read_bytes():
            changed.append(str(r));patch.extend(difflib.unified_diff(original.read_text().splitlines(True),f.read_text().splitlines(True),fromfile='old/'+str(r),tofile='direct/'+str(r)))
    assert set(changed)=={'app\\nav_app.c','nav\\nav_task.c','nav\\nav_task.h'},changed
    for n in('p11_command_guard.c','p11_command_guard.h','p11_snapshot_service.c','p11_snapshot_service.h'):
        assert (out/'fw/nav'/n).read_bytes()==(SRC/'fw/nav'/n).read_bytes()
    # The entire original common consumer block is byte-identical.
    def tail(s):return s[s.index('send_checked:'):s.index('void nav_task_map_work(')]
    assert tail((out/'fw/nav/nav_task.c').read_text())==tail((SRC/'fw/nav/nav_task.c').read_text())
    (out/'shared_proposal.patch').write_text(''.join(patch),encoding='utf8')
    save(out/'shared_proposal_manifest.json',dict(at=datetime.now().astimezone().isoformat(),source=str(SRC),changed_files=changed,
         all_FW_bindings=[metadata(x)for x in sorted((out/'fw').rglob('*'))if x.is_file()],
         original_common_send_checked_identical=True,four_core_guard_service_C_H_identical=True,
         selected_image_changed=False,RTL_generated=False,EDA=False,new_BIN_INIT=False,deployment_ready=False))
    print(json.dumps(dict(output=str(out),changed=changed),ensure_ascii=False))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=BASE/'P11_shared_proposal_FW_prepared1');prepare(p.parse_args().out)
