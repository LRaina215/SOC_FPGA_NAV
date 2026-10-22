from pathlib import Path
import csv,json,statistics
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
root=Path(__file__).parent;out=root.parent/'figures'
font=fm.FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False,'font.size':11,'axes.spines.top':False,'axes.spines.right':False})
C={'实物':'#cf4b24','仿真':'#2166ac'}
real=list(csv.DictReader(open('/home/cyd/fpga/tools/fpga_telemetry/logs/fpga_20261017_213102.csv')))[:2740]
left=[dict(x,t=(int(x['timestamp_ms'])-19839)/1000) for x in real if x['host_timestamp'].startswith('2026-10-17T21:33') and 19439<=int(x['timestamp_ms'])<=22639]
right=[]
with open('/home/cyd/fpga/tools/fpga_telemetry/logs/fpga_20261017_134514.csv') as f:
 for i,x in enumerate(csv.DictReader(f)):
  if 151190<=i<=151350:right.append(dict(x,t=(int(x['timestamp_ms'])-69439)/1000))
ls=[dict(x,t=float(x['time_s'])-3.008) for x in csv.DictReader((root/'simulation.csv').open()) if x['env_id']=='32' and 2.608<=float(x['time_s'])<=5.808]
rs=[dict(x,t=float(x['time_s'])-3.008) for x in csv.DictReader((root/'right_biased/simulation.csv').open()) if x['env_id']=='0' and 2.608<=float(x['time_s'])<=5.808]
def v(a,k):return np.array([float(x[k]) for x in a])
def common(ax):ax.grid(alpha=.2);ax.axhline(0,c='#666',lw=.6);ax.axvline(0,c='#888',ls=':',lw=.8)
def scalar(ax,r,s,key,ylabel):
 for a,label in [(r,'实物'),(s,'仿真')]:ax.plot(v(a,'t'),v(a,key),c=C[label],lw=1.8,label=label)
 ax.set_ylabel(ylabel);common(ax)
summary={}
for name,r,s,cn,rate,off,idx in [('left',left,ls,'左转',1.,1.32,'04'),('right',right,rs,'右转',-1.,1.10,'05')]:
 bias='0' if name=='left' else '−0.05'
 fig,axs=plt.subplots(6,1,figsize=(11,12.5),sharex=True)
 scalar(axs[0],r,s,'gyro_z','转向角速度\n(rad/s)')
 axs[0].step(v(r,'t'),v(r,'yaw_command'),where='post',c='#222',ls='--',lw=1.3,label='实物目标角速度')
 axs[0].step(v(s,'t'),v(s,'yaw_command'),where='post',c='#888',ls=':',lw=1.2,label='仿真目标角速度')
 axs[0].legend(ncol=4,fontsize=9,loc='upper right')
 for ax,side,label in [(axs[1],'left','左轮'),(axs[2],'right','右轮')]:
  for a,src in [(r,'实物'),(s,'仿真')]:
   ax.plot(v(a,'t'),v(a,side+'_target'),c=C[src],ls='--',lw=1.4,alpha=.8,label=src+'目标')
   ax.plot(v(a,'t'),v(a,side+'_speed'),c=C[src],lw=1.9,label=src+'实际')
  ax.set_ylabel(label+'速度\n(m/s)');common(ax)
 axs[1].legend(ncol=4,fontsize=9,loc='upper right')
 scalar(axs[3],r,s,'roll_deg','前后倾角\n(°)')
 scalar(axs[4],r,s,'pitch_deg','横向倾角\n(°)')
 scalar(axs[5],r,s,'gyro_x','前后转动角速度\n(rad/s)')
 for ax in axs:ax.axvline(off,c='#999',ls=':',lw=.8);ax.set_xlim(-.4,2.8)
 axs[-1].set_xlabel('相对转向指令开始的时间 (s)；第二条竖线为指令结束')
 fig.suptitle(f'{idx} {cn}：目标 {rate:+.1f} rad/s，原4500、1倍输出\n橙色＝实物；蓝色＝仿真；车身前后指令均为 {bias} m/s',fontsize=15,y=.985)
 note='实物：21:33:22.909开始的原始1倍记录；仿真 env32。' if name=='left' else '右转实物来自更早的固定−0.05 m/s版本，不与左转作严格版本一致比较；仿真采用相同偏置。'
 fig.text(.5,.012,note+' 不同初始状态，按指令开始对齐。',ha='center',fontsize=9,color='#555')
 fig.tight_layout(rect=[0,.035,1,.945]);fig.savefig(out/f'{idx}_{name}_turn.png',dpi=165);plt.close(fig)
 # Additional complete IMU axes.
 fig,axs=plt.subplots(3,2,figsize=(12,9),sharex=True)
 for j,axis in enumerate('xyz'):
  scalar(axs[j,0],r,s,'gyro_'+axis,axis.upper()+'角速度\n(rad/s)')
  for a,src in [(r,'实物'),(s,'仿真')]:
   g=v(a,'gravity_'+axis+'_q12')/4096 if src=='实物' else v(a,'gravity_'+axis)
   axs[j,1].plot(v(a,'t'),g,c=C[src],lw=1.8,label=src)
  axs[j,1].set_ylabel(axis.upper()+'投影重力\n(无量纲)');common(axs[j,1])
  if axis=='z':axs[j,1].set_ylim(-1.015,-.96)
  for ax in axs[j]:ax.set_xlim(-.4,2.8);ax.axvline(off,c='#999',ls=':',lw=.8)
 axs[0,0].legend(ncol=2);axs[0,1].legend(ncol=2)
 for ax in axs[-1]:ax.set_xlabel('相对转向指令开始的时间 (s)')
 fig.suptitle(cn+'过程 IMU 各轴：实物 vs 仿真\nX向右、Y向前、Z向上；Z角速度为转向角速度',fontsize=15,y=.985)
 fig.text(.5,.012,'实物为BMI088融合估计，仿真为无噪声真值；右侧是投影重力，不是原始加速度。',ha='center',fontsize=9,color='#555')
 fig.tight_layout(rect=[0,.04,1,.93]);fig.savefig(out/f'{idx}_{name}_imu_axes.png',dpi=165);plt.close(fig)
 stats={}
 for a,src in [(r,'real'),(s,'sim')]:
  b=[x for x in a if float(x['yaw_command'])*rate>.5 and 0<=float(x['t'])<=off]
  stats[src]={'n':len(b),'yaw_mean':statistics.mean(float(x['gyro_z']) for x in b),'yaw_mae':statistics.mean(abs(float(x['gyro_z'])-rate) for x in b),'forward_command_mean':statistics.mean(float(x['forward_command']) for x in b)}
 summary[name]=stats
 with (root/f'{name}_real_window.csv').open('w') as f:
  w=csv.DictWriter(f,fieldnames=list(r[0]));w.writeheader();w.writerows(r)
(root/'plot_summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
