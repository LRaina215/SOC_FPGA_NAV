"""Software-only candidate for a NOT-YET-IMPLEMENTED explicit quiesce CSR."""
from pathlib import Path
from datetime import datetime
import argparse,difflib,json,shutil
from prepare_baseline_fusion_retime import metadata,save
BASE=Path('F:/nav_integration_g2_20261023');SRC=BASE/'P11_snapshot_shared_guard_FW_prepared4'

def prepare(out):
    assert not out.exists();shutil.copytree(SRC,out)
    p=out/'fw/app/nav_app.c';old=p.read_text();t=old
    t=t.replace('static uint32_t p11_sequence,p11_last_raw_us;','static uint32_t p11_sequence;')
    t=t.replace('p11_wit_pending','p11_source_pending')
    t=t.replace('hal_p11_parser_control(0); /* serialize flush before changing baud/owner */',
                'hal_p11_parser_control(2); /* PROPOSED HW explicit RX+FIFO quiesce/flush */')
    t=t.replace('    p11_last_raw_us=hal_time_us();hal_raw_uart_config(selected?217u:868u,500u);',
                '    hal_raw_uart_config(selected?217u:868u,500u);')
    t=t.replace('p11_source_pending=!selected;','p11_source_pending=true;')
    a=t.index('static void p11_raw_work(');b=t.index('static void p11_note_proposal(uint32_t now)\n{',a)
    t=t[:a]+'''static void p11_raw_work(uint32_t now)
{
    if(p11_source_pending){
        /* A single proposed READY read, never an unbounded wait. Continue
           real-time zero/timeout processing even iftheline staysLOW. */
        if(!hal_p11_parser_ready()){p11_note_proposal(now);return;}
        p11_source_pending=false;
    }
    if(p11_protocol){
        (void)p11_snapshot_service(&p11,&p11_sequence,!g_host_link_guard.locked);
        p11_note_proposal(hal_time_us());
    }
}
'''+t[b:]
    assert old!=t;p.write_text(t,encoding='utf8')
    h=out/'fw/hal/hal.h';t=h.read_text().replace('void hal_p11_parser_control(uint32_t enable);',
          'void hal_p11_parser_control(uint32_t enable);\nbool hal_p11_parser_ready(void);');h.write_text(t,encoding='utf8')
    d=out/'fw/drv/hal_fpga.c';t=d.read_text().replace('void hal_p11_parser_control(uint32_t enable){nav_wr(NAV_IMU_P11_CTRL,enable);}',
          'void hal_p11_parser_control(uint32_t enable){nav_wr(NAV_IMU_P11_CTRL,enable);}\nbool hal_p11_parser_ready(void){return(nav_rd(NAV_IMU_P11_CTRL)&0x100u)!=0;}');d.write_text(t,encoding='utf8')
    lines=[];changed=[]
    for f in sorted((out/'fw').rglob('*')):
        if not f.is_file():continue
        rel=f.relative_to(out/'fw');a=SRC/'fw'/rel
        if a.read_bytes()!=f.read_bytes():
            changed.append(str(rel));lines.extend(difflib.unified_diff(a.read_text().splitlines(True),f.read_text().splitlines(True),fromfile='old/'+str(rel),tofile='quiesce/'+str(rel)))
    (out/'proposed_quiesce_FW.patch').write_text(''.join(lines),encoding='utf8')
    save(out/'quiesce_manifest.json',dict(at=datetime.now().astimezone().isoformat(),source=str(SRC),changed_files=changed,
        all_FW_bindings=[metadata(f)for f in sorted((out/'fw').rglob('*'))if f.is_file()],
        proposed_READY_and_SOURCE_FLUSH_not_implemented_in_current_hardware=True,
        core_owner_feedback_R0_TTL_service_files_unchanged=True,rawFIFO_and_WIT_cleanup_offloaded_only_on_explicit_source_switch=True,
        READY_poll_bounded_one_per_service=True,selected_image_changed=False,RTL_generated=False,EDA=False,new_BIN_INIT=False,deployment_ready=False))
    print(json.dumps(dict(output=str(out),changed=changed),ensure_ascii=False))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=BASE/'P11_quiesce_shared_guard_FW_prepared1');prepare(p.parse_args().out)
