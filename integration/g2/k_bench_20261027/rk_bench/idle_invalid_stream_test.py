"""Actual host-guard test: continuous invalid packets cannot renew a lease.

Only IDLE/zero no-op targets and explicitly invalid NV messages are sent.
No motor commands, map/config changes, CPU reset, SAFE writes, or reprogramming.
"""
from collections import Counter
from datetime import datetime
import json
from pathlib import Path
import sys
import time

sys.path.insert(0,str(Path(__file__).resolve().parent/'tools'))
import nav_host as nh


def main():
    link=nh.open_link('uart:/dev/ttyACM0')
    t0=time.monotonic();rows=[];sends=[]

    def read(phase,duration):
        end=time.monotonic()+duration;got=[]
        while time.monotonic()<end:
            b=link.recv(min(0.004,max(0.001,end-time.monotonic())))
            m=nh.parse(b) if b else None
            if m and m.get('type')=='status':
                assert m['vx']==m['wz']==0,('Nonzero target, abort',m)
                r=dict(phase=phase,host_s=time.monotonic()-t0,**m)
                rows.append(r);got.append(r)
        return got

    def send(b,kind):
        # Safe accepted frames are only IDLE and found=0 with zero coordinates.
        assert b in [nh.mode(0),nh.target(0,0,found=0),b'NV\x02\x20'+bytes(20),b'NV\x01\x7f'+bytes(20),b'NV\x01\x20']
        link.send(b);sends.append(dict(kind=kind,host_s=time.monotonic()-t0))

    initial=read('initial',0.3)
    assert initial and all(x['mode'] in (0,4) for x in initial),'Vehicle is not idle/locked'
    send(nh.mode(0),'explicit_idle')
    for _ in range(30):
        send(nh.target(0,0,found=0),'valid_zero_heartbeat');read('healthy',0.02)
    healthy=[x for x in rows if x['phase']=='healthy']
    assert healthy and all(x['mode']==0 for x in healthy),'Healthy heartbeat phase not IDLE'
    last_legal=sends[-1]['host_s']
    invalid=[(b'NV\x02\x20'+bytes(20),'wrong_version'),(b'NV\x01\x7f'+bytes(20),'unknown_type'),(b'NV\x01\x20','short_target')]
    end=time.monotonic()+1.15;i=0
    while time.monotonic()<end:
        b,k=invalid[i%3];send(b,k);read('invalid_stream',0.01);i+=1
    stream=[x for x in rows if x['phase']=='invalid_stream']
    stopped=next((x for x in stream if x['mode']==4),None)
    assert stopped,'Invalid traffic deferred timeout for the whole stream'
    latency=stopped['host_s']-last_legal
    assert latency<0.5,('Timeout status observation delayed',latency)
    assert stream[-1]['mode']==4,'Invalid traffic cleared the latch'
    for _ in range(10):
        send(nh.target(0,0,found=0),'fresh_zero_after_timeout');read('locked_fresh_zero',0.03)
    locked=[x for x in rows if x['phase']=='locked_fresh_zero']
    assert locked and all(x['mode']==4 for x in locked)
    result=dict(passed=True,ended_at=datetime.now().astimezone().isoformat(),
        sent_counts=dict(Counter(x['kind'] for x in sends)),status_count=len(rows),
        first_timeout_status_after_last_legal_packet_s=latency,
        all_observed_numeric_targets_zero=True,invalid_traffic_did_not_renew=True,
        fresh_heartbeat_did_not_automatically_rearm=True,final_mode=locked[-1]['mode'],
        status_rows=rows,sends=sends,CPU_reset=False,IMU_dependency=False,
        physical_stopping_proved=False,busy_full_APP_WCET_proved=False,
        caveat='Finite actual idle firmware workload, not transport saturation, busy planning,350ms zero-publication or500ms mechanical stopping proof')
    print(json.dumps(result,ensure_ascii=False))


if __name__=='__main__':main()
