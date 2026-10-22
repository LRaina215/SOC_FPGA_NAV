from pathlib import Path
import json,csv,statistics,math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
root=Path(__file__).parent
d=json.loads((root/'results.json').read_text());lookup={(r['mode'],r['case']):r for r in d['results']}
font=fm.FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False,'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
mode_labels=['原4500仿真','原P控制＋实物时序','FPGA PID / 假设0.15 N·m','FPGA PID / 假设0.30 N·m','FPGA PID / 假设0.50 N·m','FPGA PID / 0.30 N·m＋1.8倍']
case_labels=['停车','前进0.3','后退0.3','左转1','右转1','降低底盘','抬高底盘']
heat=np.array([[lookup[m,c]['physical_survival'] for c in d['cases']] for m in d['modes']])
fig,ax=plt.subplots(figsize=(12,6));im=ax.imshow(heat,cmap='RdYlGn',vmin=0,vmax=1,aspect='auto')
ax.set_xticks(range(7),case_labels);ax.set_yticks(range(6),mode_labels)
for i in range(6):
 for j in range(7):ax.text(j,i,f'{round(heat[i,j]*10)}/10',ha='center',va='center',fontsize=12)
fig.colorbar(im,ax=ax,label='18秒未发生首次倾倒的比例')
ax.set_title('同一4500权重：加入FPGA时序/接口后的敏感性评估\n每格10种时序相位；PWM到电机力矩尚未实测标定',fontsize=14)
fig.text(.5,.025,'相位覆盖0～18 ms；3秒开始指令，13秒结束运动，18秒停止；两档底盘高度差约10 cm。',ha='center',fontsize=9)
fig.tight_layout(rect=[0,.06,1,1]);fig.savefig(root/'survival.png',dpi=170);plt.close(fig)
records=list(csv.DictReader((root/'telemetry.csv').open()))
colors={'original':'#2166ac','fpga_pid_tau030':'#dc6a22','fpga_pid_tau030_gain18':'#ad2548'}
labels={'original':'原仿真','fpga_pid_tau030':'FPGA时序/PID，1倍','fpga_pid_tau030_gain18':'FPGA时序/PID，1.8倍'}
fig,axs=plt.subplots(4,2,figsize=(13,11),sharex=True)
for col,case in enumerate(['forward','reverse']):
 for mode in colors:
  a=[r for r in records if r['mode']==mode and r['case']==case]
  t=np.array([float(r['time_s']) for r in a]);valid=np.array([float(r['alive'])>0 and float(r['motor_stop'])==0 for r in a])
  target=np.array([(float(r['left_target'])+float(r['right_target']))/2 for r in a]);wheel=np.array([(float(r['left_speed'])+float(r['right_speed']))/2 for r in a])
  target[~valid]=np.nan;wheel[~valid]=np.nan
  axs[0,col].plot(t,target,c=colors[mode],ls='--',lw=1,alpha=.7)
  axs[0,col].plot(t,wheel,c=colors[mode],lw=1.7,label=labels[mode])
  for row,key in [(1,'roll_deg'),(2,'gyro_x'),(3,'body_forward')]:
   y=np.array([float(r[key]) for r in a]);y[~valid]=np.nan
   axs[row,col].plot(t,y,c=colors[mode],lw=1.6,label=labels[mode])
 for row,label in enumerate(['两轮平均速度 (m/s)\n虚线目标／实线实际','前后倾角 (°)','前后角速度 (rad/s)','车身前进速度 (m/s)']):
  ax=axs[row,col];ax.set_ylabel(label);ax.grid(alpha=.2);ax.axvline(3,c='#777',ls=':',lw=1);ax.axhline(0,c='#666',lw=.6);ax.set_xlim(0,8)
 axs[0,col].set_title('前进指令 +0.3 m/s' if case=='forward' else '后退指令 −0.3 m/s')
 axs[-1,col].set_xlabel('仿真时间 (s)；3秒开始给速度指令')
axs[0,0].legend(fontsize=9)
fig.suptitle('加入FPGA响应参数后：代表相位的轮速与IMU对照\n电机代理取假设堵转扭矩0.30 N·m、空载线速度1.0 m/s；首次失败后不再画线',fontsize=14,y=.99)
fig.text(.5,.01,'仅为时序/控制器敏感性实验，不是实物数字孪生；舵机机械动态和电机参数未标定。',ha='center',fontsize=9)
fig.tight_layout(rect=[0,.035,1,.935]);fig.savefig(root/'response_comparison.png',dpi=170);plt.close(fig)
# Summary report: avoid reporting censored conditional mean speeds as successful steady tracking.
lines=['# FPGA时序与PID接入原4500仿真：评估报告','', '## 结论','', '同一4500在原仿真中全部工况稳定；加入实际采样/输出节奏和整数PID后，多项工况失稳。因此原策略对执行器动态差异明显敏感。由于PWM到力矩、电机空载速度和舵机机械动态尚未实测标定，本实验不能定量复刻真车，也不能把失稳唯一归因于延迟。','', '## 固定和移植的参数','', '- 权重未修改：`'+d['checkpoint_sha256']+'`。模型仍为28维当前输入、5帧历史、6路动作。','- 物理步长2 ms；模型8 ms更新；1.67 ms计算延迟用一个2 ms物理步近似。','- 编码器1320计数/圈、65 mm轮径；10 ms计数窗；PID反馈四舍五入到cm/s；模型轮速观测使用原始计数换算。','- 轮速PID每10 ms更新，P/I/D/FF Q12分别为10240/1434/8192/3277；导数滤波α=1/4、D限幅250；积分抗饱和；PWM限幅2400/2500；目标斜坡每次100 mm/s；零目标清PID并滑行。','- 保留堵转100样本与反向10样本保护；本次未出现电机保护触发，记录到的失败来自仿真倾倒/碰撞判据。','- 舵机50 Hz锁存；完整接口组使用工程IK ROM及19 µs动作修正、500~2500 µs限幅，假设500~2500 µs对应π rad且机械零位等于CAD默认位。','- IMU按10 ms保持，使用真值经Q12量化；未仿真BMI088噪声、零偏、融合滤波及IIC传输。','- 前后/转向指令阶跃与原IR相同；高度按IR每10 ms变化一个Q12单位；高低目标差约100 mm。','', '## 方法','', f"共{len(d['modes'])*len(d['cases'])*d['environments_per_mode_case']}个环境，6组控制条件×7种工况×10种时序相位。无噪声、无外部推力，不修改质量/质心。3秒开始指令，13秒松开，18秒结束。每个环境第一次倾倒后不再计入有效速度均值，自动重置后的数据不算成功。",'', '以下分数为18秒内未发生首次倾倒的数量，不是统计置信区间。','', '| 控制条件 | 停车 | 前进 | 后退 | 左转 | 右转 | 降低 | 抬高 |','|---|---:|---:|---:|---:|---:|---:|---:|']
for m,label in zip(d['modes'],mode_labels):lines.append('| '+label+' | '+' | '.join(f"{round(lookup[m,c]['physical_survival']*10)}/10" for c in d['cases'])+' |')
lines+=['','## 首次失败时间中位数（秒，含最初3秒零指令）','','| 控制条件 | 停车 | 前进 | 后退 | 左转 | 右转 | 降低 | 抬高 |','|---|---:|---:|---:|---:|---:|---:|']
for m,label in zip(d['modes'],mode_labels):
 vals=[]
 for c in d['cases']:
  a=lookup[m,c];med=statistics.median(a['first_failure_s']);vals.append('≥18' if med>=18 else f'{med:.2f}')
 lines.append('| '+label+' | '+' | '.join(vals)+' |')
lines+=['','## 如何解释','', '- “原P控制＋实物时序”保留原仿真P参数，只更改采样/保持节奏，是诊断组，不代表实物PID；其失稳不能称为实物必然如此。','- 完整PID组不是直接套用仿真P值，而是使用已与RTL核对的整数PID。PWM映射为平均电压驱动的简化直流电机模型：τ=τ_stall×(duty/2500−v/v_no_load)，零目标和保护时滑行。','- τ_stall=0.15/0.30/0.50 N·m、v_no_load=1.0 m/s均为明确假设，扫描用于检查敏感性；不能当成520真实参数。','- 三种驱动力假设结果不同，说明驱动匹配和时序一起重要；没有证据证明单独提高FPGA主频可以解决。','- 不同工况和模式占据不同环境位置，且没有多随机种子统计；10个样本主要覆盖时序相位。','- 高度阶跃的初轮诊断另存step_height_diagnostic；正式结果采用真实IR高度变化速率。','', '## 下一步','', '先用实物轮速/PWM数据拟合电机代理的加减速响应，尤其双向制动和低速死区；必要时加入舵机动态与质心范围。确认仿真能复现主要波形后，再微调策略并考核80%车速指标。当前不应仅据本实验选择新PID参数。','', '## 文件','', '- `results.json`：有效配置、源文件哈希、逐工况结果和未仿真项目。','- `telemetry.csv`：各组代表时序相位完整轨迹。','- `pid_verification.json` / `pid_rtl.log`：417组实际RTL对照验证。','- `response_comparison.png` / `survival.png`：曲线和未失败数量图。','', '本次没有训练新模型、没有修改原训练工程或FPGA源码、没有编译或烧录硬件。']
(root/'report.md').write_text('\n'.join(lines)+'\n')
print('REPORT_READY')
for m in d['modes']:
 print(m,[(c,round(lookup[m,c]['physical_survival']*10),round(statistics.median(lookup[m,c]['first_failure_s']),3)) for c in ['stop','forward','reverse','left','right']])
