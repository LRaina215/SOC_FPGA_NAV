"""One light native numeric comparison using explicit original-table CSR model."""
from pathlib import Path
from datetime import datetime
import argparse,json,os,psutil,re,subprocess,traceback
from prepare_baseline_fusion_retime import metadata,save
BASE=Path('F:/nav_integration_g2_20261023');REPO=Path(__file__).resolve().parents[2]
CC=Path('C:/msys64/mingw64/bin/gcc.exe')

def run(source,out):
    assert not out.exists();out.mkdir();parent=BASE/'P11_quiesce_shared_guard_FW_prepared1'
    bindings=[metadata(p)for p in sorted((source/'fw').rglob('*'))if p.is_file()]
    state=dict(source=str(source),output=str(out),passed=False,controller_pid=os.getpid(),controller_created_epoch=psutil.Process().create_time(),EDA=False,hardware_CSR_substitute=True)
    save(out/'job.json',state)
    def invoke(cmd,label):
        cmd=[str(x)for x in cmd];env=os.environ.copy();env['PATH']=str(CC.parent)+';'+env.get('PATH','')
        with(out/(label+'.log')).open('w',encoding='utf8')as log:
            p=subprocess.Popen(cmd,cwd=out,env=env,stdout=log,stderr=subprocess.STDOUT)
            r=dict(pid=p.pid,created_epoch=psutil.Process(p.pid).create_time(),command=cmd,at=datetime.now().astimezone().isoformat());save(out/(label+'_runtime.json'),r)
            code=p.wait();r.update(exit_code=code,ended=datetime.now().astimezone().isoformat());save(out/(label+'_exit.json'),r)
        assert code==0,(label,code)
        return (out/(label+'.log')).read_text()
    try:
        oldfix=(parent/'fw/nav/fixmath.c').read_text();reference=re.sub(r'\b(fx_\w+)(?=\s*\()',r'reference_\1',oldfix)
        # Keep the real original header/type definitions; only public function identifiers change.
        (out/'reference_fixmath.c').write_text(reference,encoding='utf8')
        (out/'reference_traj_sw.c').write_text((parent/'fw/nav/traj_sw.c').read_text().replace('traj_eval_sw(','reference_traj_eval_sw('),encoding='utf8')
        inc=['-I'+str(source/'fw'/d)for d in('include','nav','hal','app')]
        invoke([CC,'-std=c99','-O2','-Wall','-Wextra','-Werror','-DNAV_FW_SIN_CSR_MODEL',*inc,
             REPO/'integration/g2/tb_FW_sin_MMIO.c',source/'fw/nav/fixmath.c',source/'fw/nav/traj_sw.c',out/'reference_fixmath.c',out/'reference_traj_sw.c','-o',out/'test.exe'],'native_compile')
        log=invoke([out/'test.exe'],'actual_C');assert 'PASS FW_SIN_MMIO'in log and 'FAIL' not in log
        assert bindings==[metadata(Path(x['path']))for x in bindings]
        state.update(passed=True,phase='original_table_CSR_numerical_contract_passed',output_text=log.strip(),frozen_inputs_unchanged=True,
             all_original1024_words=True,all65536_angles=True,actual_rotations=4096,trajectory_scenarios=32,
             real_HW_BSRAM_bus_timing_or_busyCPU_stop_tested=False,input_bindings=bindings,
             reference_sources=[metadata(parent/'fw/nav/fixmath.c'),metadata(parent/'fw/nav/traj_sw.c'),metadata(parent/'fw/include/nav_sin_lut.h')])
    except Exception as e:state.update(phase='failed_preserved',error=str(e),traceback=traceback.format_exc())
    save(out/'result.json',state);save(out/'job.json',state);print(json.dumps(dict(output=str(out),passed=state['passed'],error=state.get('error'))));return 0 if state['passed']else 1
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,default=BASE/'P11_quiesce_FW_sin_MMIO_prepared1');p.add_argument('--out',type=Path,required=True);a=p.parse_args();raise SystemExit(run(a.source,a.out))
