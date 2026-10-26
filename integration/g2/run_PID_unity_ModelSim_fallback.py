"""One AE fallback for the preserved xelab-crash PID component probe.

Copies the identical four DUT/fixture inputs from probe1 and changes only its
watchdog #3000000000 to explicit SV #3s. No source RTL, synthesis, firmware or
hardware action. A license rejection is final for this invocation, not retried.
"""
from pathlib import Path
from datetime import datetime
import argparse,json,os,re,shutil,subprocess,time,traceback
import psutil
from prepare_baseline_fusion_retime import metadata,save
from prepare_FW_sin_ROM_RTL_overlay import gate54
from retry_frozen_synthesis import memory_status

BASE=Path('F:/nav_integration_g2_20261023')
FIRST=BASE/'PID_unity_actual_control_probe1'
BIN=Path('E:/modelsim_ae/win32aloem')
LICENSE=Path('C:/Users/Li/Downloads/LR-204362_License.dat')
FILES=('policy_action_map.v','motor520_closed_loop.v','target_select_pair.sv','tb_PID_unity_equivalence.sv')
DO='''onerror {quit -code 2 -f}
onbreak {quit -code 3 -f}
vlib work
vlog -sv policy_action_map.v motor520_closed_loop.v target_select_pair.sv tb_PID_unity_equivalence.sv
vsim work.tb_PID_unity_equivalence
run -all
quit -code 0 -f
'''

def other_AE():
    names={'vsim.exe','vsimk.exe','vish.exe','vlm.exe','vlog.exe'};found=[]
    for p in psutil.process_iter(['pid','name','create_time','cmdline']):
        try:
            if (p.info['name']or'').lower()in names:found.append(p.info)
        except(psutil.AccessDenied,psutil.NoSuchProcess):pass
    return found

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    assert not a.out.exists();assert LICENSE.is_file();assert not other_AE(),('singleAEsession',other_AE())
    gate=gate54(True);first=json.loads((FIRST/'result.json').read_text())
    assert first['phase']=='failed_preserved_no_retry' and not first['passed']
    assert first['runtimes'][-1]['tool']=='xelab' and first['runtimes'][-1]['exit_code']==3221225477
    math=json.loads((FIRST/'actual_math_checks.json').read_text())
    assert math['passed'] and len(math['checks'])==7 and all(c['result']=='unsat'for c in math['checks'])
    m=memory_status();assert m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3,('actual2/4admission',m)
    a.out.mkdir();before=[metadata(FIRST/f)for f in FILES]
    for name in FILES:shutil.copy2(FIRST/name,a.out/name)
    tb=a.out/'tb_PID_unity_equivalence.sv';old=tb.read_text()
    assert old.count('#3000000000;')==1
    tb.write_text(old.replace('#3000000000;','#3s;'),encoding='utf8')
    assert tb.read_text().replace('#3s;','#3000000000;')==old
    for name in FILES[:-1]:assert(FIRST/name).read_bytes()==(a.out/name).read_bytes()
    shutil.copy2(FIRST/'actual_math_checks.json',a.out/'actual_math_checks.json')
    (a.out/'run.do').write_text(DO,encoding='utf8')
    frozen=[metadata(a.out/f)for f in FILES]+[metadata(a.out/'run.do')]
    state=dict(at=datetime.now().astimezone().isoformat(),phase='prepared',passed=False,
      controller_pid=os.getpid(),controller_created_epoch=psutil.Process().create_time(),
      source_probe=str(FIRST),first_failure_preserved=metadata(FIRST/'result.json'),first_hardware_inputs=before,
      gate54=gate,math_reused_without_rerun=True,math_checks=math['checks'],
      only_fixture_change='unsized signed32 overflow watchdog replaced by explicit SV3s time literal',
      frozen_inputs=frozen,EDA_started=False,available_physical_bytes=m.avail_phys,available_commit_bytes=m.avail_page,
      actual_50MHz_PID_defaults_unchanged=True,production_feedback100Hz=True,
      license_content_not_read_or_copied=True,license_env_subprocess_only=['MGLS_LICENSE_FILE','LM_LICENSE_FILE'],
      no_synthesis_PNR_FW_or_board=True,full4500_or_fullG2_acceptance=False,deployment_ready=False)
    save(a.out/'job.json',state)
    try:
        env=os.environ.copy();env['MGLS_LICENSE_FILE']=str(LICENSE);env['LM_LICENSE_FILE']=str(LICENSE)
        cmd=[str(BIN/'vsim.exe'),'-c','-l','transcript.log','-do','run.do']
        with(a.out/'stdout.log').open('w',encoding='utf8')as log:
            p=subprocess.Popen(cmd,cwd=a.out,env=env,stdout=log,stderr=subprocess.STDOUT,
                 creationflags=subprocess.CREATE_NO_WINDOW|subprocess.BELOW_NORMAL_PRIORITY_CLASS)
            r=dict(command=cmd,pid=p.pid,created_epoch=psutil.Process(p.pid).create_time(),started_at=datetime.now().astimezone().isoformat(),children=[])
            state.update(phase='ModelSim_running',EDA_started=True,tool_runtime=r);save(a.out/'job.json',state)
            print(json.dumps(dict(output=str(a.out),pid=r['pid'],created_epoch=r['created_epoch'],phase=state['phase'])),flush=True)
            while p.poll()is None:
                try:r['children']=[dict(pid=q.pid,created_epoch=q.create_time(),name=q.name(),command=q.cmdline())for q in psutil.Process(p.pid).children(recursive=True)]
                except(psutil.NoSuchProcess,psutil.AccessDenied):pass
                r['observed_at']=datetime.now().astimezone().isoformat();save(a.out/'tool_runtime.json',r)
                time.sleep(2)
            r.update(tool_exit=p.returncode,ended_at=datetime.now().astimezone().isoformat())
            save(a.out/'tool_runtime.json',r);state['tool_runtime']=r
        logs='\n'.join(x.read_text(errors='replace')for x in (a.out/'stdout.log',a.out/'transcript.log')if x.is_file())
        state['license_rejected']=bool(re.search(r'Unable to checkout|License checkout failed|No valid license|Invalid.*license',logs,re.I))
        assert p.returncode==0,('AE exit',p.returncode,'license_rejected',state['license_rejected'])
        passed=re.search(r'TEST PASSED PID_UNITY[^\r\n]*',logs)
        assert passed and not re.search(r'\*\* (?:Fatal|Error):|\bFATAL_ERROR\b',logs),('actual PASS/noFatal',logs[-1600:])
        fields={k:int(v)for k,v in re.findall(r'(\w+)=(\d+)',passed[0])}
        assert fields['checked_cycles']>0 and fields['stop_phases']==8 and fields['disable_phases']==8 and fields['reset_phases']==8
        assert all(fields[k]>0 for k in('fault1','fault2','fault3','forward','reverse'))
        state.update(phase='finite_ModelSim_PID_unity_passed',passed=True,actual_PASS=passed[0],coverage=fields)
    except Exception as e:
        state.update(phase='failed_preserved_no_retry',error=str(e),traceback=traceback.format_exc())
    for item in before+frozen:assert metadata(Path(item['path']))==item,('frozenfilechanged',item['path'])
    state['ended_at']=datetime.now().astimezone().isoformat();save(a.out/'result.json',state);save(a.out/'job.json',state)
    print(json.dumps(dict(output=str(a.out),passed=state['passed'],phase=state['phase'],coverage=state.get('coverage'),error=state.get('error'))),flush=True)

if __name__=='__main__':main()
