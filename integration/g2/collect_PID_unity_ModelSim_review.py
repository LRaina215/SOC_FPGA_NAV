"""Read the single natural AE result; never invoke EDA or mutate its inputs."""
from pathlib import Path
from datetime import datetime
import argparse,json,re
import psutil
from prepare_baseline_fusion_retime import metadata,save
BASE=Path('F:/nav_integration_g2_20261023');REPO=Path(__file__).resolve().parents[2]

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--probe',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);a=p.parse_args();assert not a.out.exists()
    final=json.loads((a.probe/'result.json').read_text());rt=final['tool_runtime']
    assert rt.get('tool_exit')is not None and rt.get('ended_at')
    ids=[dict(pid=final['controller_pid'],created_epoch=final['controller_created_epoch']),rt]+rt.get('children',[])
    status=[]
    for i in ids:
        alive=False
        try:alive=abs(psutil.Process(i['pid']).create_time()-i['created_epoch'])<.02
        except psutil.NoSuchProcess:pass
        status.append(dict(pid=i['pid'],created_epoch=i['created_epoch'],original_alive=alive))
    assert not any(i['original_alive']for i in status),status
    logs='\n'.join(p.read_text(errors='replace')for p in(a.probe/'stdout.log',a.probe/'transcript.log'))
    before=BASE/'PID_unity_actual_control_probe1';old=json.loads((before/'result.json').read_text())
    assert old['runtimes'][-1]['exit_code']==3221225477
    unchanged={n:(before/n).read_bytes()==(a.probe/n).read_bytes()for n in('policy_action_map.v','motor520_closed_loop.v','target_select_pair.sv')}
    assert all(unchanged.values())
    oldtb=(before/'tb_PID_unity_equivalence.sv').read_text()
    newtb=(a.probe/'tb_PID_unity_equivalence.sv').read_text()
    assert oldtb.count('#3000000000;')==1 and newtb.count('#3s;')==1
    assert oldtb.replace('#3000000000;','#3s;')==newtb
    math=json.loads((a.probe/'actual_math_checks.json').read_text());assert all(c['result']=='unsat'for c in math['checks'])and len(math['checks'])==7
    failed=bool(re.search(r'\*\* (?:Fatal|Error):|FATAL_ERROR',logs))
    passed=final['passed']and rt['tool_exit']==0 and not failed and'TEST PASSED PID_UNITY'in logs
    assert passed==final['passed'],'terminal PASS evidence mismatch'
    r=dict(at=datetime.now().astimezone().isoformat(),passed=passed,phase=final['phase'],
      first_failure=dict(path=str(before),tool='xelab',exit=3221225477,
        actual='static elaboration then EXCEPTION_ACCESS_VIOLATION, no functional simulation',
        fixture_warning='VRFC10-9277 unsized3000000000=>signed32 -1294967296; fixedonly#3s',
        original_preserved=metadata(before/'result.json')),
      actual_AE_result=metadata(a.probe/'result.json'),tool_runtime=rt,all_original_processes_exited=status,
      source_equal=unchanged,only_TB_watchdog_changed=True,
      seven_math_miters_reused_UNSAT=math['checks'],coverage=final.get('coverage'),
      PASS=final.get('actual_PASS'),error=final.get('error'),noFatal=not failed,
      actual_production_contract='50MHz clock/100Hz feedback,400000clock publication, original PID8states/10M timeout/stall100/wrong10/PWM2500; real signed24mapper and old/new2xL/R PID/PWM',
      coverage_meaning='IRstop/enable/reset eachactual8PIDphases, everyclocktargets/duty/reference/error/fault/control_valid/6pins, explicitknown-stateXrejection afterrealreset',
      limits=['finitecomponent boundary, not full4500inference/fusion/IRdecoder/wholeG2',
        'nonunity original expression retainedbutnotsequentiallytested here',
        'noactualmapping/physicalrouting/STA benefitproved','noRK/board/physicalstopping proof'],
      license_content_read_copied_or_system_environment_modified=False,
      SYN_PNR_FW_or_board_actions=False,deployment_ready=False)
    a.out.mkdir();save(a.out/'result.json',r)
    save(REPO/'docs/4_design/evidence/g2_20261023/PID_unity_actual_ModelSim_component_20261027.json',r)
    print(json.dumps(dict(output=str(a.out),passed=passed,coverage=final.get('coverage'),all_exited=True)))

if __name__=='__main__':main()
