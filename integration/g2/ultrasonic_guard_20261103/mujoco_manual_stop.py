"""One physical approach/stop case in the existing conda mujoco environment.

No navigation, no hardware connection, no RTL regression. Remote forward remains
the deployed +0.3m/s (Q12=1229). Original4500 weights and native actuator model
are reused; the sonar is a declared nine-ray approximation, not a calibrated unit.
"""
import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

HERE=Path(__file__).resolve().parent
NAV=HERE.parents[2]
sys.path.insert(0,str(NAV/'sim/closed_loop'))
from control4500 import load_runtime


class ManualGuard:
    """1ms behavioral counterpart of the selected guard, not an RTL simulator."""
    def __init__(self):
        self.enabled=False;self.age=65535;self.good=0;self.clear=0
        self.latched=False;self.ramp=0;self.cap=0;self.seen=False

    def tick(self,request,sample):
        self.age=min(65535,self.age+1)
        if not self.enabled:
            self.good=self.clear=self.ramp=self.cap=0;self.latched=False;self.seen=False
            return request
        if sample is not None:
            valid,mm=sample
            if valid:
                self.age=0;self.good=min(2,self.good+1)
                self.clear=min(2,self.clear+1) if mm>=350 else 0
                self.cap=0 if mm<=250 else 1229 if mm>=600 else (mm-250)*1229//350
                if mm<=250:self.latched=True
            else:
                self.good=self.clear=self.cap=0
                if self.seen and request>0:self.latched=True
        healthy=self.good>=2 and self.age<150
        if healthy:self.seen=True
        if self.seen and self.age>=150 and request>0:self.latched=True
        if request<=0 and healthy and self.clear>=2:self.latched=False
        if request<=0:self.ramp=0;return request
        if not healthy or self.latched:self.ramp=0;return 0
        target=min(request,self.cap)
        self.ramp+=min(2,target-self.ramp) if self.ramp<target else max(-4,target-self.ramp)
        return min(request,self.ramp)


def add_scene(path):
    tree=ET.parse(path);world=tree.find('worldbody');base=world.find("body[@name='base']")
    # CAD forward is world+Y. The enclosed field is3m across by5m forward.
    for name,pos,size in [
        ('side_left',(-1.525,2.5,.3),(.025,2.55,.3)),
        ('side_right',(1.525,2.5,.3),(.025,2.55,.3)),
        ('back_wall',(0,-.025,.3),(1.5,.025,.3)),
        ('end_wall',(0,5.025,.3),(1.5,.025,.3)),
        ('front_obstacle',(0,2.6,.3),(.5,.2,.3))]:
        ET.SubElement(world,'geom',name=name,type='box',pos=' '.join(map(str,pos)),
                      size=' '.join(map(str,size)),contype='0',conaffinity='1',
                      group='5',rgba='.72 .35 .18 1')
    # Existing project envelope27cm forward x33cm across; no mass/inertia retuning.
    ET.SubElement(base,'geom',name='ultrasonic_body_envelope',type='box',
                  pos='-.08 -.006656542 .035',size='.165 .135 .025',mass='0',
                  contype='1',conaffinity='0',group='3',rgba='.2 .5 .8 .18')
    # Mount is an explicit assumption: front face145mm ahead of wheel reference,
    # 30mm above that body's nominal130.5mm height. SG90 angle stays at center.
    ET.SubElement(base,'site',name='fixed_front_sonar',pos='-.08 .138343458 .03',
                  size='.012',rgba='0 1 .8 1')
    tree.write(path,encoding='unicode')


def ray_sample(bot):
    origin=bot.data.site('fixed_front_sonar').xpos.copy()
    r=bot.data.xmat[bot.base].reshape(3,3)
    directions=[np.array([0.,1.,0.])]
    a=math.radians(7.5)
    for phi in np.arange(8)*math.pi/4:
        directions.append(np.array([math.sin(a)*math.cos(phi),math.cos(a),math.sin(a)*math.sin(phi)]))
    mask=np.array([1,0,0,0,0,1],dtype=np.uint8) # floor and real scene solids; not robot visuals
    hits=[]
    for direction in directions:
        geom=np.zeros(1,dtype=np.int32)
        distance=mj.mj_ray(bot.model,bot.data,origin,r@direction,mask,1,bot.base,geom)
        if distance>=0:hits.append((distance,bot.model.geom(int(geom[0])).name))
    distance,name=min(hits) if hits else (float('inf'),'none')
    #343m/s is a declared sound-speed assumption. Keep the existing58us/cm decoder.
    if .02<=distance<=4:
        echo_us=math.floor(2*distance/343*1e6)
        return (True,max(0,(echo_us-1)*10//58)),math.ceil((212+echo_us)/1000),distance,name
    return (False,65535),30,distance,name


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=20)
    p.add_argument('--stop-speed-hold',action='store_true',
                   help='Candidate: only after obstacle stop, cancel residual creep with encoder feedback')
    a=p.parse_args();a.out.mkdir(parents=True,exist_ok=False)
    rt,policy=load_runtime(a.out/'assets');add_scene(a.out/'assets/robot.xml')
    # Robot.reset advances8ms immediately. Place it INSIDE the enclosure before
    # that advance, not at the back wall and then translated after a collision.
    initial=json.loads(rt.CONFIG.read_text())
    initial['env']['init_state']['pos']=[.08,.606656542,.1305]
    rt.CONFIG=a.out/'assets/initial_pose_config.json'
    rt.CONFIG.write_text(json.dumps(initial,indent=2)+'\n')
    bot=rt.Robot(forward_bias=0.)
    bot.obs=bot.observation();bot.history=np.tile(bot.obs,(5,1))
    guard=ManualGuard();pending=None;measured=65535;ray_hit='none'
    due=True;rows=[];states=[];times=[];contacts=[];inferences=0
    first_brake=None;first_zero=None;stopped_at=None;stopped_ms=0;failure_ms=0
    encoder_previous=np.rint(bot.data.qpos[bot.qindex[[2,5]]]*[-1,1]*1320/(2*math.pi)).astype(np.int64)
    feedback_mmps=0;hold_sum=0;hold_q12=0
    envelope=bot.model.geom('ultrasonic_body_envelope').id
    corners=np.array([[x,y,z] for x in (-.165,.165) for y in (-.135,.135) for z in (-.025,.025)])
    state=dict(phase='running',started_at=datetime.now().astimezone().isoformat(),
               executable=sys.executable,mujoco=mj.__version__,remote_forward_mps=.3,remote_forward_q12=1229,
               SG90_scan=False,slow_mm=600,stop_mm=250,scenario_count=1)
    state['stop_speed_hold']=a.stop_speed_hold
    (a.out/'job.json').write_text(json.dumps(state,indent=2)+'\n')
    for ms in range(round(a.seconds*1000)):
        if ms==1000:guard.enabled=True;pending=None # one remote0 activation
        request=1229 if ms>=2000 else 0 # unchanged production forward key, held throughout
        if ms>=300 and (ms-300)%65==0:
            sample,latency,distance,ray_hit=ray_sample(bot)
            pending=(ms+latency,sample)
        sample=None
        if pending is not None and ms>=pending[0]:
            sample=pending[1];measured=sample[1];pending=None
        adopted=guard.tick(request,sample)
        if ms%10==0:
            encoder=np.rint(bot.data.qpos[bot.qindex[[2,5]]]*[-1,1]*1320/(2*math.pi)).astype(np.int64)
            delta=encoder-encoder_previous;encoder_previous=encoder
            # Same100Hz encoder gate and integer cm/s feedback granularity.
            cms=np.trunc(delta*(math.pi*.065)*100/(1320*.01)).astype(int)
            feedback_mmps=int(cms.sum()*5)
            if a.stop_speed_hold and guard.latched and request>0:
                hold_sum=max(-10000,min(10000,hold_sum+feedback_mmps))
                hold_q12=max(-205,min(205,round(-feedback_mmps*2048/1000-hold_sum*1024/50000)))
            else:hold_sum=0;hold_q12=0
        if a.stop_speed_hold and guard.latched and request>0:adopted=hold_q12
        bot.command[:]=[adopted/4096,0.,.1305]
        if request and adopted<request and first_brake is None and measured<=600:first_brake=ms/1000
        if request and guard.latched and first_zero is None:first_zero=ms/1000
        if ms%2==1:
            if due:
                bot.actions=policy.action(bot.obs,bot.history);inferences+=1;due=False
            position,velocity=bot.targets();control=position.copy();control[rt.WHEELS]=velocity[rt.WHEELS]
            bot.data.ctrl[bot.aindex]=control
            mj.mj_step(bot.model,bot.data);mj.mj_forward(bot.model,bot.data)
            if (ms+1)%8==0:
                bot.obs=bot.observation();bot.history=np.concatenate((bot.history[1:],bot.obs[None]),axis=0);due=True
            for contact in bot.data.contact:
                names=[bot.model.geom(g).name or '' for g in (contact.geom1,contact.geom2)]
                obstacle=any(n in ('front_obstacle','side_left','side_right','back_wall','end_wall') for n in names)
                base_floor='floor' in names and any(n.startswith('base_collision') or n=='ultrasonic_body_envelope' for n in names)
                if obstacle or base_floor:
                    if not contacts or contacts[-1]['names']!=names:contacts.append(dict(t=ms/1000,names=names))
            metrics=bot.metrics()
            failure_ms=failure_ms+2 if metrics['tilt_deg']>bot.cfg['env']['tilt_fail_deg'] else 0
            if first_zero is not None:
                stopped_ms=stopped_ms+2 if abs(metrics['forward_mps'])<=.003 else 0
                if stopped_ms>=500 and stopped_at is None:stopped_at=(ms-498)/1000
            if ms%10==9 or contacts:
                front=float((corners@bot.data.geom_xmat[envelope].reshape(3,3).T+bot.data.geom_xpos[envelope])[:,1].max())
                rows.append([ms/1000,request/4096,adopted/4096,measured/1000,
                    metrics['forward_mps'],2.4-front,metrics['tilt_deg'],bot.center()[1],
                    int(guard.latched),int(guard.enabled),ray_hit,feedback_mmps,hold_q12])
            if ms%40==39:times.append(ms/1000);states.append(bot.data.qpos.copy())
            if contacts or failure_ms>bot.cfg['env']['fail_to_terminal_time_s']*1000:break
    header=['t','remote_request_mps','guarded_target_mps','sonar_m','actual_forward_mps',
            'body_clearance_m','tilt_deg','wheel_reference_forward_m','stop_latched','mode_enabled','sonar_hit',
            'encoder_feedback_mmps','parking_correction_q12']
    with (a.out/'motion.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(header);w.writerows(rows)
    np.savez_compressed(a.out/'states.npz',t=np.array(times),qpos=np.array(states))
    tail=[r for r in rows if r[0]>=rows[-1][0]-2]
    summary=dict(state,phase='complete',ended_at=datetime.now().astimezone().isoformat(),
        simulated_seconds=rows[-1][0],policy_inferences=inferences,first_brake_s=first_brake,
        zero_target_s=first_zero,physical_stop_s=stopped_at,
        physical_stop_definition='|forward speed|<=0.003m/s sustained500ms AFTER latched zero-motion demand; finite simulation criterion only',
        min_body_clearance_m=min(r[5] for r in rows),contacts=contacts,
        maximum_tilt_deg=max(r[6] for r in rows),final_speed_mps=rows[-1][4],
        last2s_max_abs_speed_mps=max(abs(r[4]) for r in tail),
        last2s_forward_drift_m=tail[-1][7]-tail[0][7],
        no_collision=len(contacts)==0,no_fall=failure_ms<=bot.cfg['env']['fail_to_terminal_time_s']*1000,
        automatic_stop_observed=first_zero is not None and stopped_at is not None,
        model='original4500 float32 + original native affine actuators; NOT real FPGA fixed-point/wheel PID',
        assumptions=['3x5m flat scene; one front obstacle','sonar9 ideal rays over15deg cone',
                     'sensor145mm ahead of wheel reference and30mm above nominal body height; not measured',
                     '27x33cm project collision envelope; original mass/inertia unchanged',
                     'no navigation, no actual HC-SR04 noise/soft-surface/dropout model',
                     '1ms behavioral guard,65ms ranging,2ms physics,8ms original4500'])
    (a.out/'result.json').write_text(json.dumps(summary,indent=2)+'\n')
    (a.out/'job.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
