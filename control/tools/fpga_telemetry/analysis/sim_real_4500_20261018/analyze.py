import csv,json,math,statistics
from pathlib import Path
root=Path(__file__).parent
real=list(csv.DictReader(open('/home/cyd/fpga/tools/fpga_telemetry/logs/fpga_20261017_213102.csv')))[:2740]
sim=list(csv.DictReader((root/'simulation.csv').open()))
def summarize(a):
 if not a:return {'n':0}
 def avg(f):return statistics.mean(f(x) for x in a)
 return dict(n=len(a),target_mean=avg(lambda x:(float(x['left_target'])+float(x['right_target']))/2),wheel_mean=avg(lambda x:(float(x['left_speed'])+float(x['right_speed']))/2),wheel_mae=avg(lambda x:(abs(float(x['left_target'])-float(x['left_speed']))+abs(float(x['right_target'])-float(x['right_speed'])))/2),roll_mean=avg(lambda x:float(x['roll_deg'])),roll_rms=math.sqrt(avg(lambda x:float(x['roll_deg'])**2)),pitch_mean=avg(lambda x:float(x['pitch_deg'])),wx_abs_mean=avg(lambda x:abs(float(x['gyro_x']))),wx_rms=math.sqrt(avg(lambda x:float(x['gyro_x'])**2)),wy_abs_mean=avg(lambda x:abs(float(x['gyro_y']))),wz_abs_mean=avg(lambda x:abs(float(x['gyro_z']))))
report={}
for sign in [1,-1]:
 def match(x):return abs(float(x['left_target'])-.3*sign)<=.03 and abs(float(x['right_target'])-.3*sign)<=.03
 a=[x for x in real if x['active']=='True' and match(x)]
 b=[x for x in sim if float(x['alive'])==1 and float(x['done'])==0 and match(x)]
 c=[x for x in sim if float(x['alive'])==1 and 5<float(x['time_s'])<13 and x['case']==('forward' if sign==1 else 'reverse')]
 report[str(sign)]={'real_at_target_band':summarize(a),'sim_at_target_band':summarize(b),'sim_steady_command':summarize(c)}
print(json.dumps(report,indent=2));(root/'summary.json').write_text(json.dumps(report,indent=2))
