from pathlib import Path
import csv,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
root=Path(__file__).parent
out=root/'figures';out.mkdir(exist_ok=True)
font=fm.FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False,'font.size':11,'axes.spines.top':False,'axes.spines.right':False,'savefig.facecolor':'white'})
real=list(csv.DictReader(open('/home/cyd/fpga/tools/fpga_telemetry/logs/fpga_20261017_213102.csv')))[:2740]
sim=list(csv.DictReader((root/'simulation.csv').open()))
colors={'real':'#cf4b24','sim':'#2166ac'}
# Use FPGA time, not the USB batch-receive time, for real time axes.
reverse_start=min(int(x['timestamp_ms']) for x in real if x['host_timestamp'].startswith('2026-10-17T21:33:19') and float(x['forward_command'])<0)
rev_real=[dict(x,t=(int(x['timestamp_ms'])-reverse_start)/1000) for x in real if x['host_timestamp'].startswith('2026-10-17T21:33') and reverse_start-400<=int(x['timestamp_ms'])<=reverse_start+2600]
rev_sim=[dict(x,t=float(x['time_s'])-3.008) for x in sim if x['env_id']=='64' and 2.608<=float(x['time_s'])<=5.608]
positive=[x for x in sim if x['env_id']=='32' and float(x['time_s'])>=3 and abs(float(x['left_target'])-.3)<=.03 and abs(float(x['right_target'])-.3)<=.03]
sim_positive_start=float(positive[0]['time_s'])
fwd_real=[dict(x,t=(int(x['timestamp_ms'])-15119)/1000) for x in real if x['host_timestamp'].startswith('2026-10-17T21:33') and 15019<=int(x['timestamp_ms'])<=15659]
fwd_sim=[dict(x,t=float(x['time_s'])-sim_positive_start) for x in sim if x['env_id']=='32' and -.1<=float(x['time_s'])-sim_positive_start<=.54]
def vector(a,k):return np.asarray([float(x[k]) for x in a])
def common(ax):
 ax.grid(True,alpha=.2);ax.axhline(0,color='#555',linewidth=.7,alpha=.6);ax.axvline(0,color='#777',linewidth=.8,linestyle=':')
def speed(ax,side,r,s):
 for a,name in [(r,'real'),(s,'sim')]:
  label='实物' if name=='real' else '仿真'
  ax.plot(vector(a,'t'),vector(a,side+'_target'),color=colors[name],linestyle='--',alpha=.8,lw=1.5,label=label+'目标')
  ax.plot(vector(a,'t'),vector(a,side+'_speed'),color=colors[name],lw=2,label=label+'实际')
 ax.set_ylabel(('左轮' if side=='left' else '右轮')+'速度\n(m/s)');common(ax)
def scalar(ax,k,label,r,s):
 for a,name in [(r,'real'),(s,'sim')]:ax.plot(vector(a,'t'),vector(a,k),color=colors[name],lw=1.8,label='实物' if name=='real' else '仿真')
 ax.set_ylabel(label);common(ax)
fig,axs=plt.subplots(5,1,figsize=(11,11.5),sharex=True)
for a,name in [(rev_real,'real'),(rev_sim,'sim')]:axs[0].plot(vector(a,'t'),vector(a,'forward_command'),color=colors[name],lw=2,linestyle='-' if name=='real' else '--',label='实物指令' if name=='real' else '仿真指令')
axs[0].set_ylabel('车身速度指令\n(m/s)');common(axs[0]);axs[0].legend(ncol=2,loc='upper right')
speed(axs[1],'left',rev_real,rev_sim);axs[1].legend(ncol=4,loc='upper right',fontsize=9)
speed(axs[2],'right',rev_real,rev_sim)
scalar(axs[3],'roll_deg','前后倾角\n(°)',rev_real,rev_sim)
scalar(axs[4],'gyro_x','前后转动角速度\n(rad/s)',rev_real,rev_sim);axs[4].legend(ncol=2,loc='upper right')
axs[-1].set_xlabel('相对按下后退键的时间 (s)');axs[-1].set_xlim(-.4,2.6)
fig.suptitle('① 后退指令 −0.3 m/s：原始4500、1倍输出\n橙色＝实物；蓝色＝仿真；轮速虚线＝目标，实线＝实际',fontsize=15,y=.985)
fig.text(.5,.012,'按指令开始时刻对齐；不同试验与初始状态。实物 21:33:19.772 起；仿真 env64、3秒后施加指令。',ha='center',fontsize=9,color='#555')
fig.tight_layout(rect=[0,.035,1,.94]);fig.savefig(out/'01_reverse_command.png',dpi=170);plt.close(fig)
fig,axs=plt.subplots(5,1,figsize=(11,11.5),sharex=True)
for a,name in [(fwd_real,'real'),(fwd_sim,'sim')]:axs[0].plot(vector(a,'t'),vector(a,'forward_command'),color=colors[name],lw=2,label='实物指令' if name=='real' else '仿真指令')
axs[0].set_ylabel('车身速度指令\n(m/s)');common(axs[0]);axs[0].legend(ncol=2,loc='upper right')
speed(axs[1],'left',fwd_real,fwd_sim);axs[1].legend(ncol=4,loc='upper right',fontsize=9)
speed(axs[2],'right',fwd_real,fwd_sim)
scalar(axs[3],'roll_deg','前后倾角\n(°)',fwd_real,fwd_sim)
scalar(axs[4],'gyro_x','前后转动角速度\n(rad/s)',fwd_real,fwd_sim);axs[4].legend(ncol=2,loc='upper right')
axs[-1].set_xlabel('相对选定“输出约 +0.3”时刻的时间 (s)');axs[-1].set_xlim(-.1,.54)
fig.suptitle('② 模型轮速输出接近 +0.3 m/s 的片段\n注意：实物车身指令为0，仿真为+0.3，属于不同任务状态',fontsize=15,y=.985)
fig.text(.5,.012,f'仅按相近输出选窗口，不是同状态A/B。实物 FPGA 15.119秒；仿真 env32、{sim_positive_start:.3f}秒。',ha='center',fontsize=9,color='#555')
fig.tight_layout(rect=[0,.035,1,.94]);fig.savefig(out/'02_forward_output_near_030.png',dpi=170);plt.close(fig)
fig,axs=plt.subplots(3,2,figsize=(12,9),sharex=True)
for row,axis in enumerate('xyz'):
 scalar(axs[row,0],'gyro_'+axis,f'{axis.upper()}角速度\n(rad/s)',rev_real,rev_sim)
 for a,name in [(rev_real,'real'),(rev_sim,'sim')]:
  g=vector(a,'gravity_'+axis+'_q12')/4096 if name=='real' else vector(a,'gravity_'+axis)
  axs[row,1].plot(vector(a,'t'),g,color=colors[name],lw=1.8,label='实物' if name=='real' else '仿真')
 axs[row,1].set_ylabel(f'{axis.upper()}投影重力\n(无量纲)');common(axs[row,1])
 if axis=='z':axs[row,1].set_ylim(-1.015,-.975)
 for ax in axs[row]:ax.set_xlim(-.4,2.6)
axs[0,0].legend(ncol=2,loc='upper right');axs[0,1].legend(ncol=2,loc='upper right')
for ax in axs[-1]:ax.set_xlabel('相对按下后退键的时间 (s)')
fig.suptitle('③ 后退过程 IMU 六路状态对照\nX向右、Y向前、Z向上；X角速度对应前后转动',fontsize=15,y=.985)
fig.text(.5,.012,'实物：BMI088校准后的角速度与融合重力；仿真：无噪声真值。投影重力不是原始加速度。',ha='center',fontsize=9,color='#555')
fig.tight_layout(rect=[0,.04,1,.93]);fig.savefig(out/'03_imu_axes_reverse.png',dpi=170);plt.close(fig)
manifest={'real_source':'fpga_20261017_213102.csv first 2740 rows, original gain=1','sim_source':'simulation.csv original model4500 gain=1','reverse_real_start_timestamp_ms':reverse_start,'reverse_sim_start_time_s':3.008,'reverse_sim_env':64,'forward_real_start_timestamp_ms':15119,'forward_sim_start_time_s':sim_positive_start,'forward_sim_env':32,'limitations':['Trials not state-matched','Forward real command zero vs simulation +0.3','Simulation IMU is noiseless ground truth; real IMU fused estimates','No raw accelerometer comparison; projected gravity is not acceleration']}
(out/'plot_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
print('GENERATED',*[str(p) for p in sorted(out.glob('*.png'))],sep='\n')
