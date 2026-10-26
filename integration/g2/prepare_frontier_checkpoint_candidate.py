"""Prepare an isolated bounded-checkpoint frontier candidate, never deploy it.

The queue, distances and scoring remain on the original no-IRQ call stack.
Only no-op TARGET heartbeats are consumed while that stack is active. Other
messages cancel it and stay in the existing1100-byte receive buffer until the
coordinate view has unwound. No map/seen/visited snapshot array is added.
"""
from __future__ import annotations
import argparse
from datetime import datetime
import difflib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys

BASE = Path('F:/nav_integration_g2_20261023/baseline_fusion_preview_safety250_emptyIMU_prepared3')
ROOT = Path(__file__).resolve().parents[2]
GCC = Path('D:/tools/xpack-riscv-none-elf-gcc-15.2.0-1/bin/riscv-none-elf-gcc.exe')


def once(s, old, new):
    assert s.count(old)==1, (s.count(old),old[:100])
    return s.replace(old,new,1)


HEADER = '''#ifndef NAV_SEARCH_CHECKPOINT_H
#define NAV_SEARCH_CHECKPOINT_H
#include <stdbool.h>
extern bool g_nav_search_interrupted;
bool nav_app_search_checkpoint(void);
#endif
'''

CHECKPOINT = '''/* Only a no-op heartbeat can continue the frozen frontier view. Other
 * packets remain in the existing buffer until nav_task_step has unwound. */
bool nav_app_search_checkpoint(void)
{
    if(!host_pending_len) {
        uint32_t rx;
        int len=hal_host_recv(host_rx_buf,sizeof host_rx_buf,&rx);
        if(len>0) {
            proto_msg_t m;
            if(proto_parse(host_rx_buf,len,&m)) {
                if(m.type==MSG_TARGET && !m.u.target.found) {
                    if(host_link_guard_observe(&g_host_link_guard,&m,hal_time_us(),rx)) g_app.n_msgs++;
                } else { host_pending_len=len;host_pending_rx=rx; }
            }
        }
    }
    apply_host_link_lock(hal_time_us());
    if(host_pending_len || g_host_link_guard.locked) {
        g_nav_search_interrupted=true;
        nav_cmd_t zero={0,0,0,(uint8_t)g_app.nt.mode};
        hal_send_cmd(&zero);
        return false;
    }
    return true;
}

static void drain_host_messages(void)
{
    uint32_t rx_us;
    int len;
    for(;;) {
        if(host_pending_len) {
            len=host_pending_len;rx_us=host_pending_rx;host_pending_len=0;
        } else len=hal_host_recv(host_rx_buf,sizeof host_rx_buf,&rx_us);
        if(len<=0) break;
        proto_msg_t m;
        if(proto_parse(host_rx_buf,len,&m) &&
           host_link_guard_observe(&g_host_link_guard,&m,hal_time_us(),rx_us)) {
            on_msg(&g_app.nt,&m,rx_us);
            g_app.n_msgs++;
        }
    }
}

'''


def make_candidate(out: Path):
    assert not out.exists(), out
    out.mkdir(parents=True);shutil.copytree(BASE/'fw',out/'fw')
    fw=out/'fw'
    files=['nav/explore.c','nav/nav_task.c','app/nav_app.c','nav/host_link_guard.c']
    old={n:(fw/n).read_text(encoding='utf-8-sig') for n in files}
    t=old['nav/explore.c'];t='#include "nav_search_checkpoint.h"\n'+t
    t=once(t,'    astar_cancel(); /* BFS与分周期A*复用缓存，切换前显式取消旧搜索 */',
        '    astar_cancel(); /* BFS与分周期A*复用缓存，切换前显式取消旧搜索 */\n    if(!nav_app_search_checkpoint()) return false;')
    t=once(t,'    for (int i = 0; i < GM_N * GM_N; i++) s_bd[i] = 0xFFFF;',
        '''    for(int first=0;first<GM_N*GM_N;first+=256) {
        int end=nav_min32(first+256,GM_N*GM_N);
        for(int i=first;i<end;i++) s_bd[i]=0xFFFF;
        if(!nav_app_search_checkpoint()) return false;
    }''')
    t=once(t,'    while (qh < qt) {','    while (qh < qt) {\n        if((qh&31)==0 && !nav_app_search_checkpoint()) return false;')
    t=once(t,'    if (best < 0) return false;','    if(!nav_app_search_checkpoint()) return false;\n    if (best < 0) return false;')
    (fw/'nav/explore.c').write_text(t,encoding='utf-8')
    t='#include "nav_search_checkpoint.h"\n'+old['nav/nav_task.c']
    t=once(t,'            if (!nt->search_goal_valid && seen_mode && nt->exp.n_seen > 0) {',
        '            if(g_nav_search_interrupted) {nt->cmd=(nav_cmd_t){0};goto send_checked;}\n            if (!nt->search_goal_valid && seen_mode && nt->exp.n_seen > 0) {')
    t=once(t,'            if (!nt->search_goal_valid)\n                nt->search_goal_valid = explore_pick',
        '            if(g_nav_search_interrupted) {nt->cmd=(nav_cmd_t){0};goto send_checked;}\n            if (!nt->search_goal_valid)\n                nt->search_goal_valid = explore_pick')
    (fw/'nav/nav_task.c').write_text(t,encoding='utf-8')
    t='#include "nav_search_checkpoint.h"\n'+old['app/nav_app.c']
    t=once(t,'nav_app_t g_app;', '''nav_app_t g_app;
bool g_nav_search_interrupted;
static uint8_t host_rx_buf[PROTO_MAX_LEN]; /* moved from nav_app_poll; same capacity */
static int host_pending_len;
static uint32_t host_pending_rx;''')
    t=once(t,'void nav_app_poll(void)\n{',CHECKPOINT+'void nav_app_poll(void)\n{')
    start=t.index('    /* 4. 主机报文 */');end=t.index('    apply_host_link_lock(hal_time_us());',start)
    original_host=t[start:end]
    assert 'static uint8_t buf[PROTO_MAX_LEN];' in original_host and original_host.count('on_msg(nt, &m, rx_us)')==1
    t=t[:start]+'    /* 4. 主机报文（包括已取消搜索保留的原报文）。 */\n    drain_host_messages();\n\n'+t[end:]
    t=once(t,'        nav_task_step(nt, now);','        g_nav_search_interrupted=false;\n        nav_task_step(nt, now);')
    t=once(t,'    } else nav_task_map_work(nt,4);\n\n    apply_host_link_lock(hal_time_us());',
        '    } else nav_task_map_work(nt,4);\n\n    if(host_pending_len) drain_host_messages(); /* only after local-coordinate view unwinds */\n    apply_host_link_lock(hal_time_us());')
    # Existing status code used the same buffer; retain its capacity/lifetime.
    tail=t[t.index('    /* 6. 状态回传 */'):]
    tail=re.sub(r'\bbuf\b','host_rx_buf',tail)
    t=t[:t.index('    /* 6. 状态回传 */')]+tail
    (fw/'app/nav_app.c').write_text(t,encoding='utf-8')
    t=old['nav/host_link_guard.c']
    t=once(t,'    host_link_guard_poll(g,now);\n    bool motion=false,clear=false;',
        '    bool motion=false,clear=false;')
    t=once(t,'    default:g->rejected++;return false;',
        '    default:host_link_guard_poll(g,now);g->rejected++;return false;')
    t=once(t,'        if(m->u.mode.mode>NAV_MODE_ESTOP) {g->rejected++;return false;}',
        '        if(m->u.mode.mode>NAV_MODE_ESTOP) {host_link_guard_poll(g,now);g->rejected++;return false;}')
    t=once(t,'        (int32_t)(rx_us-g->last_rx_us)<0)) {g->rejected++;return false;}\n    g->have_rx=true;g->last_rx_us=rx_us;',
        '''        (int32_t)(rx_us-g->last_rx_us)<0)) {host_link_guard_poll(g,now);g->rejected++;return false;}
    /* A fresh queued arrival inside an uninterrupted lease precedes this poll.
     * Never heal an already latched timeout or a real arrival gap>=250ms. */
    bool continuous=g->have_rx && !g->locked && !g->timed_out &&
        (int32_t)(rx_us-g->last_rx_us)>=0 && (uint32_t)(rx_us-g->last_rx_us)<HOST_LINK_TIMEOUT_US;
    if(continuous) g->last_rx_us=rx_us;
    host_link_guard_poll(g,now);
    g->have_rx=true;g->last_rx_us=rx_us;''')
    (fw/'nav/host_link_guard.c').write_text(t,encoding='utf-8')
    (fw/'nav/nav_search_checkpoint.h').write_text(HEADER)
    patch='\n'.join(''.join(difflib.unified_diff(old[n].splitlines(True),(fw/n).read_text().splitlines(True),
        fromfile='frozen/'+n,tofile='candidate/'+n)) for n in files)
    (out/'candidate.patch').write_text(patch,encoding='utf-8')
    record={'recorded_at':datetime.now().astimezone().isoformat(),'source':str(BASE),'out':str(out),
        'scope':'Independent FW checkpoint candidate, not an integrated hardware source or deployment image',
        'changed_existing_FW':files,'new_FW':['nav/nav_search_checkpoint.h'],
        'new_map_snapshot_arrays':0,'shared_scratch_source_byte_identical':(fw/'nav/scratch.h').read_bytes()==(BASE/'fw/nav/scratch.h').read_bytes(),
        'algorithm_BFS_queue_distance_scoring_and_tie_order_unchanged':True,
        'init_items_between_checkpoints':256,'BFS_nodes_between_checkpoints':32,
        'checkpoint_processes_at_most_one_existing_mailbox_message':True,
        'continues_only_for_noop_TARGET_heartbeat':True,'other_message_retained_in_original1100B_buffer_until_stack_unwinds':True,
        'host_timeout_us':250000,'threshold_relaxed':False,'no_board_SYN_PNR_or_production_FW_edits':True}
    (out/'preparation.json').write_text(json.dumps(record,indent=2))
    return record


def compile_firmware(out: Path):
    fw=out/'fw';built=out/'firmware';built.mkdir()
    prior=json.loads((BASE/'firmware_compile.json').read_text())['command']
    command=[];next_output=False
    for arg in prior:
        if next_output:command.append(str(built/'nav_fw.elf'));next_output=False;continue
        if arg=='-o':command.append(arg);next_output=True;continue
        if arg.startswith('-Wl,-Map='):command.append('-Wl,-Map='+str(built/'nav_fw.map'));continue
        normalized=arg.replace('\\','/')
        if '/fw/' in normalized:
            relative=normalized.split('/fw/',1)[1]
            command.append(('-I' if arg.startswith('-I') else '')+str(fw/relative))
        else:command.append(arg)
    run=subprocess.run(command,cwd=built,capture_output=True,text=True,encoding='utf-8')
    (built/'compile.json').write_text(json.dumps({'command':command,'returncode':run.returncode,'stdout':run.stdout,'stderr':run.stderr},indent=2))
    if run.returncode:raise RuntimeError(run.stderr)
    sys.path.insert(0,str(ROOT/'integration/g2'))
    from prepare_uart_loss_250 import read_image
    image,layout=read_image(built/'nav_fw.elf')
    (built/'nav_fw.bin').write_bytes(image)
    sys.path.insert(0,str(fw/'bsp'))
    from analyze_stack import analyze
    su=next(built.glob('*.ltrans.su'));assembly=next(built.glob('*.ltrans.s'))
    stack=analyze(built/'nav_fw.elf',GCC.with_name('riscv-none-elf-objdump.exe'),su,assembly,
        '.ENABLE_IRQ(0)' in (BASE/'nav/rtl/soc/nav_soc.v').read_text())
    (built/'stack_analysis.json').write_text(json.dumps(stack,indent=2))
    assert stack['complete'] and stack['static_bound_bytes']<=3072,stack
    result={'image_bytes':len(image),'layout':layout,'stack':stack,'full_fw_compile_passed':True,
        'source_only_not_hardware_candidate':True,'new_snapshot_arrays':False}
    (out/'firmware_review.json').write_text(json.dumps(result,indent=2))
    return result


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',required=True,type=Path);ap.add_argument('--compile',action='store_true');a=ap.parse_args()
    result=make_candidate(a.out.resolve())
    if a.compile:result['firmware']=compile_firmware(a.out.resolve())
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
