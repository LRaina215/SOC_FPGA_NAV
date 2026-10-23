"""One power-confirmed B1 capture; restore only with actuators disconnected."""
import argparse,json,re,subprocess,sys,time
from datetime import datetime
from pathlib import Path
from audit_feedback_probe import audit
ROOT=Path(__file__).resolve().parents[2]
PROGRAMMER='D:/Gowin2/Gowin_V1.9.12.04_x64/Programmer/bin/programmer_cli.exe'


def main():
    p=argparse.ArgumentParser();p.add_argument('build',type=Path);p.add_argument('--port',required=True);p.add_argument('--location',required=True)
    p.add_argument('--restore',type=Path,required=True);p.add_argument('--power-confirmed-at',required=True);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--power-state',choices=('actuators-disconnected','powered-supported'),default='actuators-disconnected');a=p.parse_args()
    confirmed=datetime.fromisoformat(a.power_confirmed_at)
    if confirmed.tzinfo is None or not 0<time.time()-confirmed.timestamp()<900:p.error('fresh human power/support confirmation within15 minutes required')
    gate=audit(a.build)
    if not gate['diagnostic_build_gate_pass']:p.error('actual diagnostic build gate failed')
    if not a.restore.is_file()or a.restore.stat().st_size<1000000:p.error('saved restoration candidate required')
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False);(out/'build_gate.json').write_text(json.dumps(gate,indent=2))
    state=dict(started_at=datetime.now().astimezone().isoformat(),power_confirmation_recorded_at=confirmed.isoformat(),
        power_statement=('human confirmed actuators physically disconnected; board USB only'if a.power_state=='actuators-disconnected'else 'human confirmed whole vehicle powered and elevated with safety measures'),build=str(a.build.resolve()),port=a.port,
        power_state=a.power_state,restore_policy=('restore saved control'if a.power_state=='actuators-disconnected'else 'leave zero-output diagnostic; no automatic control restoration while powered'),
        steps=[],flash_written=False,motion_commands=0,uart_tx_bytes=0,restore_candidate=str(a.restore.resolve()),
        full_prior_image_identity_proven=False,scope='temporary independent diagnostic; not joint G2 or physical-stop acceptance')
    base=[PROGRAMMER,'--cable-index','1','--location',a.location]
    def run(name,args,timeout=45):
        result=subprocess.run(args,capture_output=True,text=True,errors='replace',timeout=timeout)
        (out/(name+'.log')).write_text(result.stdout+result.stderr,encoding='utf-8')
        state['steps'].append(dict(name=name,args=args,returncode=result.returncode))
        if result.returncode:raise RuntimeError(name+' failed: '+result.stdout[-600:])
        return result.stdout
    attempted=False;failure=None
    try:
        scan=run('scan',base+['--scan']);assert '0011081B'in scan.upper(),'unexpected device'
        before=run('usercode_before',base+['--device','GW5AT-138C','--run','73'])
        assert re.search(r'User Code is:\s*0x0*99A6',before,re.I),'current UserCode differs from restoration candidate'
        attempted=True
        run('program_probe',base+['--device','GW5AT-138C','--run','2','--fsFile',str((a.build/'impl/pnr/feedback_probe.fs').resolve())])
        run('usercode_probe',base+['--device','GW5AT-138C','--run','73'])
        run('capture',[sys.executable,'-X','utf8',str(ROOT/'integration/g3/read_feedback_probe.py'),'--port',a.port,'--seconds','30','--out',str(out/'capture')],timeout=40)
    except Exception as error:failure=repr(error)
    finally:
        state['probe_programming_attempted']=attempted
        if attempted and a.power_state=='actuators-disconnected':
            try:
                run('restore_control',base+['--device','GW5AT-138C','--run','2','--fsFile',str(a.restore.resolve())])
                restored=run('usercode_restored',base+['--device','GW5AT-138C','--run','73'])
                state['restored_usercode_99a6']=bool(re.search(r'User Code is:\s*0x0*99A6',restored,re.I))
            except Exception as error:state['restore_error']=repr(error);state['restored_usercode_99a6']=False
        if attempted and a.power_state=='powered-supported':
            state['control_restore_skipped_while_powered']=True
            state['zero_output_diagnostic_programmed']=any(s['name']=='program_probe'and s['returncode']==0 for s in state['steps'])
        state.update(ended_at=datetime.now().astimezone().isoformat(),error=failure)
        (out/'session_result.json').write_text(json.dumps(state,indent=2,ensure_ascii=False),encoding='utf-8')
    completed=state.get('restored_usercode_99a6')if a.power_state=='actuators-disconnected'else state.get('zero_output_diagnostic_programmed')
    print(json.dumps(state));return 0 if not failure and completed else 2


if __name__=='__main__':raise SystemExit(main())
