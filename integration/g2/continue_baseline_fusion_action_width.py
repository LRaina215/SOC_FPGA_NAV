"""Unique SYN25 completion -> actual netlist review -> unbounded PNR34.

This never runs synthesis. An unsuccessful proof leaves all artifacts and stops.
The original SYN25 identity is explicit; a reused PID is not the old process.
"""
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback

import psutil


ROOT = Path('F:/nav_integration_g2_20261023')
REPO = Path(__file__).resolve().parent
SOURCE = ROOT/'baseline_fusion_action_width_syn25'
PREPARED = ROOT/'baseline_fusion_action_width_prepared1'
OUTPUT = ROOT/'baseline_fusion_action_width_pnr34'
STATE = ROOT/'baseline_fusion_action_width_continuation1'
SYN_TOOL = dict(pid=38716, created_epoch=1792946675.6184423)
SYN_CONTROLLER = dict(pid=64928, created_epoch=1792946671.5259821)


def load(path):
    for attempt in range(8):
        try:
            return json.loads(Path(path).read_text(encoding='utf-8-sig'))
        except json.JSONDecodeError:
            if attempt == 7:
                raise
            time.sleep(.1)


def alive(record):
    try:
        return abs(psutil.Process(record['pid']).create_time()-record['created_epoch']) < .01
    except psutil.NoSuchProcess:
        return False


def main():
    STATE.mkdir(exist_ok=True)
    me = psutil.Process()
    state = dict(controller_pid=me.pid, controller_created_epoch=me.create_time(),
        source=str(SOURCE), prepared=str(PREPARED), output=str(OUTPUT),
        source_synthesis_tool=SYN_TOOL, source_synthesis_controller=SYN_CONTROLLER,
        starts_synthesis=False, PNR_total_limit_seconds=0, PNR_silent_limit_seconds=0,
        minimum_PNR_start_physical_GiB=2, minimum_PNR_start_commit_GiB=4,
        actual_SYN_review_required=True, added_timing_exceptions=0,
        programmed=False, deployment_accepted=False)
    with (STATE/'launch_claim.json').open('x', encoding='utf-8') as file:
        json.dump(state,file,ensure_ascii=False,indent=2)
    def save(phase, **values):
        state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**values)
        temporary=STATE/'job.json.tmp'
        temporary.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        os.replace(temporary,STATE/'job.json')
    try:
        assert not OUTPUT.exists(), 'PNR34 directory already exists; refuse duplicate physical work'
        assert load(PREPARED/'prepared_source_review.json')['passed'] is True
        save('waiting_for_existing_SYN25_natural_completion')
        while True:
            job=load(SOURCE/'job.json')
            assert job['tool_pid'] == SYN_TOOL['pid']
            assert abs(job['tool_created_epoch']-SYN_TOOL['created_epoch']) < .01
            if job['phase'] == 'synthesis_complete':
                assert job['tool_exit'] == 0
                assert not alive(SYN_TOOL), 'complete job still has original actual tool alive'
                break
            if not alive(SYN_TOOL) and not alive(SYN_CONTROLLER):
                raise AssertionError(('SYN25 ended without completed result',job['phase']))
            save('waiting_for_existing_SYN25_natural_completion',synthesis_phase=job['phase'])
            time.sleep(10)
        review=STATE/'actual_netlist_review.json'
        command=[sys.executable,'-X','utf8',str(REPO/'review_baseline_fusion_action_width_syn.py'),
                 '--source',str(SOURCE),'--prepared',str(PREPARED),'--out',str(review)]
        save('starting_actual_netlist_review',actual_review=str(review))
        with (STATE/'review.stdout.log').open('w',encoding='utf-8') as stream:
            child=subprocess.Popen(command,cwd=str(REPO.parent.parent),stdout=stream,stderr=subprocess.STDOUT,
                creationflags=subprocess.CREATE_NO_WINDOW|subprocess.IDLE_PRIORITY_CLASS)
            save('actual_netlist_review_running',reviewer_pid=child.pid,
                 reviewer_created_epoch=psutil.Process(child.pid).create_time(),review_command=command)
            code=child.wait()
        result=load(review)
        assert code == 0 and result['passed'] is True, ('actual review failed',code,result.get('error'))
        assert result['actual_RAM_INIT_clock_26CDC_preflight_passed'] is True
        save('actual_netlist_review_passed_starting_unique_PNR34')
        command=[sys.executable,'-X','utf8',str(REPO/'run_baseline_fusion_action_width_pnr.py'),
                 '--review',str(review)]
        with (STATE/'pnr_controller.stdout.log').open('w',encoding='utf-8') as stream:
            child=subprocess.Popen(command,cwd=str(REPO.parent.parent),stdout=stream,stderr=subprocess.STDOUT,
                creationflags=subprocess.CREATE_NO_WINDOW)
            save('PNR34_controller_running',pnr_controller_pid=child.pid,
                 pnr_controller_created_epoch=psutil.Process(child.pid).create_time(),pnr_command=command,
                 PNR_controller_job=str(ROOT/'baseline_single_factor_pnr_controllers1/fusion_action_width/job.json'))
            code=child.wait()
        save('PNR34_controller_exited_actual_result_requires_review',pnr_controller_exit=code)
        return code
    except Exception as error:
        save('failed_preserved_without_retry',error_type=type(error).__name__,error=str(error),
             traceback=traceback.format_exc())
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
