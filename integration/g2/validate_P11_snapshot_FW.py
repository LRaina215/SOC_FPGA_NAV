"""One lightweight native compiler/executable at a time; no EDA or image export."""
from pathlib import Path
from datetime import datetime
import argparse,json,os,psutil,shutil,subprocess,traceback
from prepare_baseline_fusion_retime import metadata,save
BASE=Path('F:/nav_integration_g2_20261023');REPO=Path(__file__).resolve().parents[2]
CC=Path('C:/msys64/mingw64/bin/gcc.exe')

def run(source,out,reuse=None):
    assert not out.exists();out.mkdir();nav=source/'fw/nav'
    inputs=[metadata(p)for p in sorted((source/'fw').rglob('*'))if p.is_file()]
    state=dict(source=str(source),output=str(out),passed=False,controller_pid=os.getpid(),
               controller_created_epoch=psutil.Process().create_time(),EDA=False,new_BIN_INIT=False)
    save(out/'job.json',state)
    def invoke(cmd,label,expected=0):
        env=os.environ.copy();env['PATH']=str(CC.parent)+';'+env.get('PATH','')
        cmd=[str(x)for x in cmd]
        with(out/(label+'.log')).open('w',encoding='utf8')as f:
            p=subprocess.Popen(cmd,cwd=out,env=env,stdout=f,stderr=subprocess.STDOUT)
            r=dict(pid=p.pid,created_epoch=psutil.Process(p.pid).create_time(),command=cmd,log=str(out/(label+'.log')))
            save(out/(label+'_runtime.json'),r);state.update(phase=label,tool=r);save(out/'job.json',state)
            code=p.wait();r.update(exit_code=code,at=datetime.now().astimezone().isoformat());save(out/(label+'_exit.json'),r)
        assert code==expected,(label,code,str(out/(label+'.log')))
        return(out/(label+'.log')).read_text()
    try:
        old=BASE/'P11_shared_guard_FW_integrated_experiment_prepared3/fw/nav'
        oldfixture=out/'old_long_session.c';shutil.copyfile(REPO/'integration/g2/tb_P11_old_long_session.c',oldfixture)
        flags=['-std=c99','-O2','-Wall','-Wextra','-Werror','-Wno-unused-function','-Wno-unused-variable']
        if reuse:
            prior=json.loads(reuse.read_text());assert prior['passed']
            prior_source=Path(prior['source'])
            for n in('p11_command_guard.c','p11_command_guard.h','p11_snapshot_service.c','p11_snapshot_service.h'):
                assert(prior_source/'fw/nav'/n).read_bytes()==(nav/n).read_bytes(),('changed reused component',n)
            oldtext=prior['old_long_session_counterexample'];flags+=['-DAPP_ONLY','-Wno-unused-but-set-variable']
            state['reused_core_proof']=dict(path=str(reuse),same_four_C_H_files=True,
                scope='range,atomic4reads,longsession,guardedge checks unchanged; app/boot/WIT/HAL rerun')
        else:
            invoke([CC,*flags,'-I',old,oldfixture,old/'p11_command_guard.c','-o',out/'old.exe'],'old_compile')
            oldtext=invoke([out/'old.exe'],'old_long_session_actual_counterexample',1)
            assert 'EXPECTED_OLD_LONG_SESSION_FAILURE'in oldtext
        app=(source/'fw/app/nav_app.c').read_text();prefix=app[:app.index('static void app_match_update')]
        (out/'actual_app_prefix.inc').write_text(prefix,encoding='utf8')
        hal=(source/'fw/drv/hal_fpga.c').read_text();send=hal[hal.index('void hal_send_cmd('):hal.index('bool hal_odom(')]
        imu=hal[hal.index('bool hal_imu_byte('):hal.index('int hal_host_recv(')]
        (out/'actual_hal_send_imu.inc').write_text(send+'\n'+imu,encoding='utf8')
        shutil.copyfile(REPO/'integration/g2/tb_P11_snapshot_FW.c',out/'tb.c')
        inc=['-I'+str(source/'fw'/d)for d in('include','nav','hal','app')]+['-I'+str(out)]
        invoke([CC,*flags,*inc,out/'tb.c',nav/'p11_command_guard.c',nav/'p11_snapshot_service.c',nav/'host_link_guard.c',nav/'proto.c',nav/'imu_parse.c','-o',out/'component.exe'],'snapshot_compile')
        text=invoke([out/'component.exe'],'snapshot_actual_C')
        assert 'PASS P11_SNAPSHOT_COMPONENT 'in text and'FAIL'not in text
        if not reuse:assert 'KNOWN_EXTERNAL_CTRL_ABA'in text and'LONG_SESSION'in text
        assert inputs==[metadata(Path(x['path']))for x in inputs]
        state.update(passed=True,phase='finite_C_components_and_actual_app_helpers_passed',old_long_session_counterexample=oldtext.strip(),
                     output=text.strip(),frozen_inputs_unchanged=True,
                     known_external_CTRL_ABA='retained; excludes concurrent externalCTRL reset/re-publish during the four noIRQ FW reads',
                     tested_scope='proposed CSR test substitute + actual guard/snapshot C, exact app prefix and original final HAL gate; not real RTL/Pico/RK/physical stop',
                     motion_support_not_executed=True,strict_every_period_APP_consumption_not_proved=True)
    except Exception as e:state.update(phase='failed_preserved',error=str(e),traceback=traceback.format_exc())
    state.update(input_bindings=inputs,at=datetime.now().astimezone().isoformat());save(out/'result.json',state);save(out/'job.json',state)
    print(json.dumps(dict(passed=state['passed'],phase=state['phase'],output=str(out),error=state.get('error')),ensure_ascii=False))
    return 0 if state['passed']else 1
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,default=BASE/'P11_snapshot_shared_guard_FW_prepared2');p.add_argument('--out',type=Path,required=True)
    p.add_argument('--reuse-core-result',type=Path);a=p.parse_args();raise SystemExit(run(a.source,a.out,a.reuse_core_result))
