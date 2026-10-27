"""Exercise host timeout/latching with IDLE and zero no-op heartbeats only.

No GOAL, TRACK, SEARCH, pose/map change, SAFE write, CPU hold, or actuator
enable is issued. Status timing is observation latency, not wheel-stop time.
Run from the RK bench folder containing the frozen tools/nav_host.py.
"""
import json
from pathlib import Path
import sys
import time

sys.path.insert(0,str(Path(__file__).resolve().parent/'tools'))
import nav_host as nh


def main():
    link=nh.open_link('uart:/dev/ttyACM0')
    t0=time.monotonic();rows=[];sent=[]

    def receive(phase,duration):
        end=time.monotonic()+duration
        got=[]
        while time.monotonic()<end:
            raw=link.recv(min(0.03,max(0.001,end-time.monotonic())))
            msg=nh.parse(raw) if isinstance(raw,(bytes,bytearray)) else raw
            if isinstance(msg,dict) and msg.get('type')=='status':
                row=dict(phase=phase,host_s=round(time.monotonic()-t0,6),**msg)
                rows.append(row);got.append(row)
                assert msg['vx']==msg['wz']==0,('Nonzero NAV status; test aborted',row)
        return got

    def send(payload,label):
        # Both message forms below express zero/idle only. Reject any other
        # packet even if future edits change call sites.
        assert payload==nh.mode(0) or payload==nh.target(0,0,found=0)
        started=time.monotonic()
        link.send(payload)
        finished=time.monotonic()
        sent.append(dict(label=label,started_s=round(started-t0,6),
                         finished_s=round(finished-t0,6),hex=payload.hex()))
        return finished

    initial=receive('initial_readonly',0.3)
    assert initial and all(x['mode'] in (0,4) for x in initial),'No idle/ESTOP-only initial state'
    send(nh.mode(0),'explicit_IDLE_initial')
    healthy=[]
    for _ in range(12):
        send(nh.target(0,0,found=0),'zero_noop_heartbeat')
        healthy+=receive('heartbeats',0.05)
    assert healthy and any(x['mode']==0 for x in healthy),'No IDLE status under healthy heartbeats'
    # Reference the final complete packet, rather than first entry into listen.
    last_heartbeat=sent[-1]['finished_s']
    silence=receive('silence_after_heartbeat',0.85)
    stopped=next((x for x in silence if x['mode']==4),None)
    assert stopped,'No timeout ESTOP status'
    observed_delay=stopped['host_s']-last_heartbeat
    assert observed_delay<=0.45,('Host observed timeout too late',observed_delay)
    latched=[]
    for _ in range(8):
        send(nh.target(0,0,found=0),'heartbeat_after_latched_timeout')
        latched+=receive('fresh_heartbeat_does_not_rearm',0.05)
    assert latched and all(x['mode']==4 for x in latched),'Fresh no-op heartbeat cleared a latched timeout'
    send(nh.mode(0),'explicit_IDLE_rearm')
    rearmed=receive('explicit_rearm',0.18)
    assert rearmed and any(x['mode']==0 for x in rearmed),'No explicit IDLE recovery'
    final=receive('final_silence_leave_locked_zero',0.65)
    assert final and final[-1]['mode']==4,'Test did not leave host timeout locked'
    result=dict(passed=True,ended_at_local=time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        test='IDLE-only host timeout latch and explicit recovery',status_samples=len(rows),
        messages_sent=len(sent),first_timeout_status_after_last_packet_s=round(observed_delay,6),
        all_observed_numeric_targets_zero=True,latched_noop_status_count=len(latched),
        firmware_timeout_constant_us=250000,physical_stopping_accepted=False,
        busy_planner_latency_accepted=False,IMU_or_motor_dependency=False,
        final_mode=final[-1]['mode'],sent=sent,status_rows=rows,
        caveat='Status publication/serial observation includes report cadence. Zero-start test proves neither a moving-car stop nor worst-case planner latency.')
    print(json.dumps(result,ensure_ascii=False))


if __name__=='__main__':main()
