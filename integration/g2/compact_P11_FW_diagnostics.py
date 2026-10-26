"""Remove only newly invented P11 diagnostics; retain its safety behavior.

The experimental ACK is reduced to result/session/state (16 total bytes).
No pre-existing protocol, application, RAM capacity or compiler flag changes.
"""
from pathlib import Path
import argparse,difflib,json,shutil
from datetime import datetime
from prepare_baseline_fusion_retime import metadata,save
BASE=Path('F:/nav_integration_g2_20261023')
SRC=BASE/'P11_shared_guard_FW_integrated_experiment_prepared2'

def prepare(out):
    assert not out.exists();shutil.copytree(SRC,out)
    def patch(rel,fn):
        p=out/'fw'/rel;t=p.read_text(encoding='utf-8-sig');n=fn(t);assert n!=t,rel;p.write_text(n,encoding='utf8')
    patch('nav/p11_command_guard.h',lambda t:t.replace('    uint32_t accepted, rejected, crc_bad, range_bad, time_bad, locks;\n',''))
    def guard(t):
        for v in('if(edge)g->locks++;','g->time_bad++;','g->crc_bad++;','g->range_bad++;','g->rejected++;','g->accepted++;'):
            assert v in t,v;t=t.replace(v,'')
        return t
    patch('nav/p11_command_guard.c',guard)
    a='uint32_t result,uint32_t session,uint32_t state,uint32_t age,uint32_t accepted,uint32_t rejected'
    b='uint32_t result,uint32_t session,uint32_t state'
    patch('nav/proto.h',lambda t:t.replace(a,b))
    patch('nav/proto.c',lambda t:t.replace(a,b).replace('    p=wr32(p,age);p=wr32(p,accepted);p=wr32(p,rejected);return (int)(p-b);','    return (int)(p-b);'))
    def app(t):
        t=t.replace('p11_note_proposal(now);uint8_t reply[28];\n    uint32_t age=p11.have_arrival?(uint32_t)(now-p11.arrival_us):0xffffffffu;',
                    'p11_note_proposal(now);uint8_t reply[16];')
        t=t.replace('((uint32_t)p11_protocol<<8),age,p11.accepted,p11.rejected);',
                    '((uint32_t)p11_protocol<<8));')
        return t
    patch('app/nav_app.c',app)
    changed=[];lines=[]
    for p in sorted((out/'fw').rglob('*')):
        if not p.is_file():continue
        rel=p.relative_to(out/'fw');old=SRC/'fw'/rel
        if old.read_bytes()!=p.read_bytes():
            changed.append(str(rel));lines.extend(difflib.unified_diff(old.read_text().splitlines(True),p.read_text().splitlines(True),fromfile='integrated2/'+str(rel),tofile='integrated3/'+str(rel)))
    (out/'new_diagnostics_only.patch').write_text(''.join(lines),encoding='utf8')
    m=json.loads((SRC/'manifest.json').read_text());m.update(at=datetime.now().astimezone().isoformat(),source=str(SRC),
        stage='experimental P11 full FW; new diagnostics reduced, same safety checks',
        changed_files=changed,UART_ACK_body='12B: result,session,state incl protocol selected',
        removed_only_new_debug_counters=True,
        all_FW_bindings=[metadata(p)for p in sorted((out/'fw').rglob('*'))if p.is_file()],
        pending=['Native component contract for compact copy','Full original-flags link budget','App/proto/MMIO integration and physical stop'],
        deployment_ready=False,new_ELF_BIN_INIT=False)
    save(out/'manifest.json',m);print(json.dumps(dict(out=str(out),changed=changed),ensure_ascii=False))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=BASE/'P11_shared_guard_FW_integrated_experiment_prepared3');prepare(p.parse_args().out)
