"""One lightweight admitted launch after the existing real4500 gets a tool.

This waits without invoking the CPU runner. It invokes the one frozen --start
entry at most once, after actual process identity and the original2/4GiB gate.
"""
from datetime import datetime
import json
from pathlib import Path
import runpy
import subprocess
import time
import psutil

OUT=Path('F:/nav_integration_g2_20261023/CPU250_bounded_drain_UART_Pico_prepared1')
FIRST=Path('F:/nav_integration_g2_20261023/observation_ring50_real4500_probe1')
FIRST_PID=3460
FIRST_CREATED=1792999899.5300996

def save(value):
    value['updated_at']=datetime.now().astimezone().isoformat()
    p=OUT/'queue_status.tmp';p.write_text(json.dumps(value,indent=2)+'\n');p.replace(OUT/'queue_status.json')

def main():
    me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),phase='waiting_real4500_actual_tool',
      first_controller_pid=FIRST_PID,first_controller_created_epoch=FIRST_CREATED,CPU_runner_started=False,
      minimum_physical_bytes=2*1024**3,minimum_commit_bytes=4*1024**3,programmed=False)
    with (OUT/'queue_claim.json').open('x') as stream:json.dump(state,stream,indent=2)
    namespace=runpy.run_path(str(OUT/'run_once.py'),run_name='queue_admission_only')
    seen=None
    try:
      while True:
        job=json.loads((FIRST/'job.json').read_text(encoding='utf-8-sig'))
        assert job['controller_pid']==FIRST_PID and abs(job['controller_created_epoch']-FIRST_CREATED)<0.01
        if not seen:
          for p in [FIRST/'tool_runtime.json',*sorted(FIRST.glob('*_runtime.json'))]:
            if not p.exists():continue
            runtime=json.loads(p.read_text(encoding='utf-8-sig'))
            for row in runtime.get('processes',[]):
              if row.get('name','').lower() in ('xvlog.exe','xelab.exe','xsim.exe'):
                seen=dict(recorded_runtime=str(p),tool=runtime.get('tool'),process=row);break
            if seen:break
        m=namespace['memory_status']()
        sessions=[dict(pid=p.pid,name=p.info['name']) for p in psutil.process_iter(['name'])
                  if (p.info['name'] or '').lower() in ('vsim.exe','vsimk.exe')]
        state.update(phase='waiting_actual_memory_admission' if seen else 'waiting_real4500_actual_tool',
          first_actual_tool=seen,available_physical_bytes=m.avail_phys,available_commit_bytes=m.avail_page,
          existing_AE_simulators=sessions)
        assert not (OUT/'launch_claim.json').exists(),'Frozen CPU runner already claimed; no second start'
        save(state)
        if seen and m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3 and not sessions:
          with (OUT/'controller_stdout.log').open('x') as stdout,(OUT/'controller_stderr.log').open('x') as stderr:
            child=subprocess.Popen(['D:/Anaconda/python.exe','-B','-X','utf8',str(OUT/'run_once.py'),'--start'],
              cwd=OUT,stdout=stdout,stderr=stderr,creationflags=subprocess.CREATE_NO_WINDOW)
          state.update(phase='single_frozen_runner_dispatched',CPU_runner_started=True,CPU_controller_pid=child.pid)
          save(state);return
        time.sleep(30)
    except Exception as error:
      state.update(phase='queue_failed_preserved_no_retry',error=str(error));save(state);raise

if __name__=='__main__':main()
