"""One postprocessor for PNR62; reads/reviews only after its natural exit.

Does not supervise, kill, pause, reconfigure or restart the PNR process. No
deployment or automatic next design is allowed by this evidence collector.
"""
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys
import time

from audit_cdc_first_stages import audit
from inventory_direct_cdc import inventory

ROOT=Path(__file__).resolve().parents[2]
BASE=Path('F:/nav_integration_g2_20261023')
PNR=BASE/'K_pnr62_s49p1r0'
OUT=BASE/'PNR62_after_natural_exit_review1'


def save(p,s):p.write_text(json.dumps(s,ensure_ascii=False,indent=2)+'\n',encoding='utf8')


def main():
    assert not OUT.exists(),'Read existing collector instead of duplicating it'
    OUT.mkdir();state=dict(phase='waiting_existing_PNR62_natural_completion',started_at=datetime.now().astimezone().isoformat(),
        source_job=str(BASE/'PNR61_second_place_one_shot/job.json'),own_EDA=False,process_control=False)
    save(OUT/'job.json',state)
    resultfile=PNR/'second_place_result.json'
    while not resultfile.exists():time.sleep(15)
    actual=json.loads(resultfile.read_text())
    assert actual['runtime']['cancelled'] is False
    result=dict(actual=actual,checked_at=datetime.now().astimezone().isoformat(),physical_audit_completed=False,
        complete_signoff=False,programmed=False,requires_root_analysis=True)
    if actual['routing_completed'] and actual['timing_report_present']:
        cdc=audit(PNR/'impl/pnr/g2_top.vo');save(OUT/'cdc26_connectivity.json',cdc)
        save(OUT/'direct_cdc_inventory.json',inventory(PNR/'impl/pnr/g2_top.vo'))
        with (OUT/'actual_audit.log').open('w',encoding='utf8') as f:
            r=subprocess.run([sys.executable,'-B','-X','utf8',str(ROOT/'integration/g2/audit_PNR61_independent_signoff.py'),
                '--physical',str(PNR),'--out',str(OUT)],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,
                creationflags=subprocess.CREATE_NO_WINDOW)
        result.update(physical_audit_exit=r.returncode,physical_audit_completed=r.returncode==0,
            physical_audit=str(OUT/'summary.json'))
    save(OUT/'postprocess_result.json',result)
    state.update(phase='complete_requires_root_analysis',ended_at=datetime.now().astimezone().isoformat(),result=str(OUT/'postprocess_result.json'))
    save(OUT/'job.json',state)
    machine=BASE/'lane2_ddr_map_evaluation/job.json';s=json.loads(machine.read_text(encoding='utf-8-sig'))
    active=s.get('current_active_PNR')
    if isinstance(active,dict) and active.get('directory')==str(PNR):
        s.update(updated_at=state['ended_at'],phase='PNR62_naturally_complete_actual_result_ready',current_active_PNR=None,
            PNR62_final_evidence=str(OUT/'postprocess_result.json'),
            next_step='Read actual PNR62 final/postprocess evidence; keep PNR61 onboard and complete IO/CDC/reset under selected CH command/P11-40 telemetry split; no automatic reprogramming or same-input retry')
        save(machine,s)


if __name__=='__main__':main()
