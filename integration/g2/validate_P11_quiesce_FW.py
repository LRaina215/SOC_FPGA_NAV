"""One lightweight native compile/run against an explicit proposedCSR model."""
from pathlib import Path
from datetime import datetime
import argparse,json,os,psutil,subprocess,traceback
from prepare_baseline_fusion_retime import metadata,save
BASE=Path('F:/nav_integration_g2_20261023');REPO=Path(__file__).resolve().parents[2]
CC=Path('C:/msys64/mingw64/bin/gcc.exe')

def run(source,out,shared_proposal=False):
    assert not out.exists();out.mkdir();nav=source/'fw/nav';prior=BASE/'P11_snapshot_FW_C_probe1/result.json'
    pr=json.loads(prior.read_text());assert pr['passed'];old_source=Path(pr['source'])
    for n in('p11_command_guard.c','p11_command_guard.h','p11_snapshot_service.c','p11_snapshot_service.h'):
        assert(old_source/'fw/nav'/n).read_bytes()==(nav/n).read_bytes(),n
    inputs=[metadata(x)for x in sorted((source/'fw').rglob('*'))if x.is_file()]
    state=dict(source=str(source),output=str(out),passed=False,controller_pid=os.getpid(),controller_created_epoch=psutil.Process().create_time(),EDA=False,proposed_CSR_substitute_only=True)
    save(out/'job.json',state)
    def invoke(cmd,label):
        env=os.environ.copy();env['PATH']=str(CC.parent)+';'+env.get('PATH','');cmd=[str(x)for x in cmd]
        with(out/(label+'.log')).open('w',encoding='utf8')as f:
            p=subprocess.Popen(cmd,cwd=out,env=env,stdout=f,stderr=subprocess.STDOUT)
            r=dict(pid=p.pid,created_epoch=psutil.Process(p.pid).create_time(),command=cmd);save(out/(label+'_runtime.json'),r)
            code=p.wait();r.update(exit_code=code,at=datetime.now().astimezone().isoformat());save(out/(label+'_exit.json'),r)
        assert code==0,(label,code,str(out/(label+'.log')))
        return(out/(label+'.log')).read_text()
    try:
        prefix=(REPO/'integration/g2/tb_P11_snapshot_FW.c').read_text().split('int main(void)')[0]
        prefix=prefix.replace('static bool parser_enabled;','static bool parser_enabled,hw_ready;\nstatic unsigned ready_reads;')
        if shared_proposal:
            prefix=prefix.replace('p11_set_protocol(0);p11_zero_published=false;','p11_set_protocol(0);')
        prefix=prefix.replace('if(hw_seq!=UINT32_MAX)','if(hw_ready&&hw_seq!=UINT32_MAX)')
        old='''void hal_p11_parser_control(uint32_t enable)
{CHECK(enable<=1);control_writes++;hw_seq=hw_word=hw_ts=0;parser_enabled=enable!=0;}'''
        new='''void hal_p11_parser_control(uint32_t enable)
{CHECK(enable<=3);control_writes++;hw_seq=hw_word=hw_ts=0;parser_enabled=(enable&1)!=0;
 if(enable&2){raw_n=raw_tn=0;hw_ready=false;}}
bool hal_p11_parser_ready(void){ready_reads++;return hw_ready;}'''
        assert prefix.count(old)==1;prefix=prefix.replace(old,new)
        (out/'csr_model_actual_app.inc').write_text(prefix,encoding='utf8')
        app=(source/'fw/app/nav_app.c').read_text();(out/'actual_app_prefix.inc').write_text(app[:app.index('static void app_match_update')],encoding='utf8')
        hal=(source/'fw/drv/hal_fpga.c').read_text();send=hal[hal.index('void hal_send_cmd('):hal.index('bool hal_odom(')]
        imu=hal[hal.index('bool hal_imu_byte('):hal.index('int hal_host_recv(')]
        (out/'actual_hal_send_imu.inc').write_text(send+'\n'+imu,encoding='utf8')
        if shared_proposal:
            task=(source/'fw/nav/nav_task.c').read_text()
            branch=task[task.index('    if(nt->mode!=NAV_MODE_ESTOP && nav_app_p11_proposal('):task.index('else switch (nt->mode) {')]
            tail=task[task.index('send_checked:'):task.index('void nav_task_map_work(')]
            assert tail.endswith('}\n\n')
            tail=tail[len('send_checked:'):].rstrip()[:-1]
            (out/'actual_direct_proposal_branch.inc').write_text(branch,encoding='utf8')
            (out/'actual_common_send_checked.inc').write_text(tail,encoding='utf8')
        tb=REPO/'integration/g2'/('tb_P11_shared_proposal_FW.c' if shared_proposal else 'tb_P11_quiesce_FW.c');inc=['-I'+str(source/'fw'/d)for d in('include','nav','hal','app')]+['-I'+str(out)]
        invoke([CC,'-std=c99','-O2','-Wall','-Wextra','-Werror','-Wno-unused-function','-Wno-unused-variable',*inc,
                tb,nav/'p11_command_guard.c',nav/'p11_snapshot_service.c',nav/'host_link_guard.c',nav/'proto.c',nav/'imu_parse.c','-o',out/'test.exe'],'native_compile')
        text=invoke([out/'test.exe'],'actual_C');assert'PASS P11_QUIESCE_SOFTWARE'in text and'FAIL'not in text
        if shared_proposal:assert 'PASS P11_SHARED_PROPOSAL'in text
        assert inputs==[metadata(Path(x['path']))for x in inputs]
        state.update(passed=True,phase='proposed_ready_CSR_software_contract_passed',output_text=text.strip(),
                     reused_core_proof=str(prior),same_four_core_C_H_files=True,
                     actual_app_prefix_and_finalHAL=True,shared_proposal_actual_source_branch=shared_proposal,real_HW_ready_flush_timing_tested=False,
                     physical_stop_or_busyCPU_not_tested=True,input_bindings=inputs)
    except Exception as e:state.update(phase='failed_preserved',error=str(e),traceback=traceback.format_exc())
    save(out/'result.json',state);save(out/'job.json',state);print(json.dumps(dict(passed=state['passed'],output=str(out),error=state.get('error')),ensure_ascii=False));return 0 if state['passed']else 1
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,default=BASE/'P11_quiesce_shared_guard_FW_prepared1');p.add_argument('--out',type=Path,required=True);p.add_argument('--shared-proposal',action='store_true');a=p.parse_args();raise SystemExit(run(a.source,a.out,a.shared_proposal))
