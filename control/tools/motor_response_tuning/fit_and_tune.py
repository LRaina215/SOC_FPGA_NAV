"""Data-informed reduced-plant tuning candidates, NOT real hardware validation.
Fits a low-order motor response from closed-loop telemetry. Body motion is an
unobserved disturbance, so fitting does not identify unique physical dynamics.
"""
import csv,json,itertools,math,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
SOURCE=ROOT/'tools/fpga_telemetry/logs/fpga_20261017_213102.csv'
rows=list(csv.DictReader(SOURCE.open()))
fits={}
for side in ('left','right'):
 x=[];y=[]
 for a,b in zip(rows,rows[1:]):
  if int(b['timestamp_ms'])-int(a['timestamp_ms'])!=20:continue
  if not (int(a['status'])&128 and int(b['status'])&128):continue
  if any(int(a[k]) or int(b[k]) for k in ['left_fault','right_fault','sensor_error','policy_fault']):continue
  if abs(float(a['roll_deg']))>10 or abs(float(b['roll_deg']))>10 or abs(int(a[side+'_duty']))>=2300:continue
  x.append([int(a[side+'_duty'])/2500,-float(a[side+'_speed']),1.]);y.append((float(b[side+'_speed'])-float(a[side+'_speed']))/.02)
 x=np.asarray(x);y=np.asarray(y);coef=np.linalg.lstsq(x,y,rcond=None)[0]
 fits[side]={'samples':len(x),'acceleration_coefficients':coef.tolist(),'tau_s':float(1/coef[1]),'gain_mps_per_full_duty':float(coef[0]/coef[1]),'one_step_accel_rmse':float(np.sqrt(np.mean((x@coef-y)**2)))}
# Conservative common controller search; covers left/right fitted plants with
# +/-25% gain and +/-30% time constant variation. Not confidence intervals.
plants=[(fits[s]['gain_mps_per_full_duty']*g,fits[s]['tau_s']*t) for s in ('left','right') for g in (.75,1.,1.25) for t in (.7,1.,1.3)]
candidates=list(itertools.product((2.5,4.,6.,8.),(.05,.1,.2,.35),(0.,.5,1.,2.),(2.5,3.5,4.5,5.5),(100,600)))
candidates.insert(0,(2.5,.35,2.,.8,100))
C=len(candidates);P=len(plants)
k=np.asarray(candidates);kp=np.rint(k[:,0]*4096).astype('int64')[:,None];ki=np.rint(k[:,1]*4096).astype('int64')[:,None];kd=np.rint(k[:,2]*4096).astype('int64')[:,None];ff=np.rint(k[:,3]*4096).astype('int64')[:,None];ramp=k[:,4:5].astype('int64')
gain=np.array([p[0] for p in plants])[None,:];tau=np.array([p[1] for p in plants])[None,:]
shape=(C,P);velocity=np.zeros(shape);position=np.zeros(shape);last_position=np.zeros(shape)
reference=np.zeros(shape,dtype='int64');integral=reference.copy();derivative=reference.copy();previous=reference.copy();duty=reference.copy();count=0;desired=0.;stall=reference.copy();opposite=reference.copy();fault=reference.copy()
tracking=np.zeros(shape);energy=np.zeros(shape);jitter=np.zeros(shape);overshoot=np.zeros(shape);last_duty=duty.copy();steady_jitter=np.zeros(shape);snapshots=[]
# Default controller for comparison uses legacy zero->coast; candidate uses
# closed-loop zero and small stationary integral bleed.
for step in range(2000):
 t=step*.002
 target=0 if t<.3 or 1.3<=t<1.9 else (300 if t<1.3 or t>=2.9 else -300)
 delayed_target=0 if t<.31 or 1.31<=t<1.91 else (.3 if t<1.31 or t>=2.91 else -.3)
 desired+=(1-math.exp(-.002/.045))*(delayed_target-desired)
 if step%5==0:
  # Quadrature count quantization and cm/s rounding match10ms FPGA acquisition.
  counts=np.floor(position/.0001546996);delta=counts-last_position;last_position=counts
  feedback=np.sign(delta)*np.floor(np.abs(delta)*1.547+.5)*10;feedback=feedback.astype('int64')
  reference+=np.clip(target-reference,-ramp,ramp)
  running=np.abs(reference)>=200
  low=np.abs(feedback)<=20
  wrong=((reference>=200)&(feedback<=-50))|((reference<=-200)&(feedback>=50))
  trip=np.where(running&low&(np.abs(duty)>=500)&(stall>=99),2,np.where(wrong&(opposite>=9),3,0))
  fault=np.maximum(fault,trip)
  stall=np.where(running&low&(np.abs(duty)>=500),stall+1,0)
  opposite=np.where(wrong,opposite+1,0)
  error=reference-feedback
  derivative=derivative+(((feedback-previous)*256-derivative)>>2) if count else derivative
  previous=feedback.copy();d=np.clip((-derivative*kd)>>8,-250*4096,250*4096)
  base=error*kp+reference*ff+d
  proposal=np.clip(integral+error*ki,-2400*4096,2400*4096)
  quiet=(reference==0)&(target==0)&(np.abs(feedback)<=10)
  proposal=np.where(quiet,integral-(integral>>3),proposal)
  trial=base+proposal;hold=((trial>2400*4096)&(error>0))|((trial< -2400*4096)&(error<0))
  integral=np.where(hold,integral,proposal)
  out=np.clip(base+integral,-2400*4096,2400*4096)
  duty=np.sign(out)*((np.abs(out)+2048)>>12)
  if target==0:duty[0]=0;integral[0]=0;derivative[0]=0;reference[0]=0;fault[0]=0;stall[0]=0;opposite[0]=0
  duty=np.where(fault!=0,0,duty)
  if count:
   jitter+=np.abs(duty-last_duty)/2500
   if .8<t<1.25 or 2.5<t<2.85:steady_jitter+=np.abs(duty-last_duty)/2500
  last_duty=duty.copy();count+=1
 a=1-np.exp(-.002/tau);velocity+=a*(gain*duty/2500-velocity)
 position+=velocity*.002
 if t>=.3:
  tracking+=(velocity-desired)**2*.002
  overshoot=np.maximum(overshoot,np.maximum(0,np.abs(velocity)-.33))
  energy+=(np.abs(duty)>=2375)*.002
# Penalize both typical and worst uncertainty cases, excess overshoot and
# excessive sustained dithering. Objective is openly heuristic.
per=tracking+overshoot**2*3+energy*.001+steady_jitter*.0002+(fault!=0)*10
score=per.mean(1)+.3*per.max(1)
order=np.argsort(score)
best=int(order[0]);choice=candidates[best]
result={'source':str(SOURCE),'fits':fits,'model_warning':'closed-loop observational fit with body-motion confounding; not uniquely identified physical motor','uncertainty':'chosen +/-25% gain, +/-30% tau around both side fits; not statistical bounds','plant_count':P,'candidate_count':C,'nominal_simulation_target':{'tau_s':.045,'delay_s':.01},'baseline':{'gains':candidates[0],'score':float(score[0]),'mean_squared_tracking_integral':float(tracking[0].mean()),'max_overshoot_mps':float(overshoot[0].max())},'selected':{'gains':choice,'score':float(score[best]),'mean_squared_tracking_integral':float(tracking[best].mean()),'max_overshoot_mps':float(overshoot[best].max()),'relative_score_change':float(score[best]/score[0]-1),'motor_faults':fault[best].tolist()},'top5':[{'gains':candidates[int(i)],'score':float(score[i])} for i in order[:5]],'hardware_validated':False}
Path(__file__).with_name('tuning_result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
