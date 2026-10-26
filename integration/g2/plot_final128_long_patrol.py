"""Render actual completed patrol errors; no simulation or acceptance threshold invention."""
from pathlib import Path
import argparse,csv,json,math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import prepare_stage1_final_long_patrol as P

def run(root):
 selected=json.loads((root/'selected_jobs.json').read_text(encoding='utf8'));fig,axes=plt.subplots(2,1,figsize=(12,7),sharex=True)
 completed=[]
 for job in selected['jobs']:
  directory=Path(job['out'])
  if not(directory/'metrics.json').exists():continue
  with(directory/'timeseries.csv').open(encoding='utf8')as f:rows=list(csv.DictReader(f))
  t=np.array([float(r['t'])for r in rows]);error=np.array([1000*math.hypot(float(r['ex'])-float(r['x']),float(r['ey'])-float(r['y']))for r in rows])
  centers=[];p95=[]
  for end in np.arange(60,t[-1]+.01,10):
   mask=(t>end-60)&(t<=end)
   if mask.any():centers.append(end);p95.append(float(np.percentile(error[mask],95)))
  label='256 reference'if job['SLF_SN']==256 else f"128 forward{job['forward_mm']/1000:g}m"
  axes[0].plot(t,error,lw=.7,alpha=.8,label=label);axes[1].plot(centers,p95,lw=1.5,label=label)
  completed.append(dict(name=job['name'],samples=len(t),duration_s=float(t[-1]),full_P95_mm=float(np.percentile(error,95)),linear_trend_mm_per_min=float(np.polyfit(t,error,1)[0]*60)))
 for ax in axes:
  ax.axhline(50,color='#b73939',ls='--',lw=1,label='50mm task budget');ax.set_ylabel('XY error (mm)');ax.grid(alpha=.25);ax.legend(loc='upper left')
 axes[0].set_title('Actual N1c patrol: full XY error over time (no-map valid flag is constant)')
 axes[1].set_title('Trailing60s P95 sampled every10s');axes[1].set_xlabel('Simulation time (s)')
 fig.tight_layout();fig.savefig(root/'error_vs_time.png',dpi=160);plt.close(fig)
 result=dict(completed=len(completed),rows=completed,plot=str(root/'error_vs_time.png'),sustained_drift_pass=None,
  limitation='Numeric sustained-growth threshold unspecified. Plot, full P95 and trend are reported; no invented drift pass. No-map lock not established by constant loc_valid.')
 (root/'error_curve_plot_metadata.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8');return result

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--root',type=Path,default=P.OUT);a=p.parse_args();print(json.dumps(run(a.root),ensure_ascii=False))
