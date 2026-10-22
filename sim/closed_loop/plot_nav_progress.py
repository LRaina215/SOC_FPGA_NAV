"""Export the fixed loop regression comparison; truth is used for reporting."""
import argparse
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
import world


def load(path):
    with (path/'timeseries.csv').open(encoding='utf-8') as f:
        rows=list(csv.DictReader(f))
    a=np.array([[float(r[k]) for k in ('t','x','y')] for r in rows])
    m=json.loads((path/'metrics.json').read_text(encoding='utf-8'))
    return a,m


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--baseline',type=Path,required=True);ap.add_argument('--failed',type=Path,required=True)
    ap.add_argument('--candidate',type=Path,required=True);ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args()
    plt.rcParams.update({'font.family':'Microsoft YaHei','axes.unicode_minus':False,'font.size':10})
    fig,(ax,bx)=plt.subplots(1,2,figsize=(11.5,4.8),gridspec_kw={'width_ratios':[1,1.05]})
    for kind,(x0,y0,x1,y1),height,name in world.scene_field5().prims:
        ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,facecolor='#d9e5d1' if kind=='ramp' else '#e3e6ea',
                               edgecolor='#abb3bd',linewidth=.5))
    for path,label,color in [(a.baseline,'N1c 基线','#7f8996'),(a.failed,'N2g 退化','#c34e52'),(a.candidate,'精度联合候选','#1679b6')]:
        data,m=load(path);t,x,y=data.T
        ax.plot(x,y,color=color,lw=1.7,alpha=.85,label=label)
        distance=np.r_[0,np.cumsum(np.hypot(np.diff(x),np.diff(y)))]
        bx.plot(t,distance,color=color,lw=1.8,label=f"{label}：{m['time_s']:.1f}s / {m['outcome']}")
        ax.scatter(x[-1],y[-1],s=30,color=color,zorder=5)
    goals=np.array([(3.9,-2.1),(4,.3),(3,2.1),(.2,.4)])
    ax.scatter(goals[:,0],goals[:,1],marker='x',s=35,color='#222',zorder=6)
    ax.set(xlim=(-.4,4.5),ylim=(-2.7,2.7),xlabel='仿真真值 x / m',ylabel='仿真真值 y / m',title='两圈任务的仿真真值轨迹')
    ax.set_aspect('equal');ax.legend(loc='upper left',frameon=False,fontsize=9)
    bx.set(xlabel='仿真时间 / s',ylabel='累计真值路程 / m',title='N2g 停滞；联合候选完成两圈')
    bx.grid(alpha=.2);bx.legend(loc='lower right',frameon=False,fontsize=9)
    fig.suptitle('相同场景/种子/快速执行模型；三者均 0 模型碰撞',fontsize=12)
    fig.tight_layout();a.out.parent.mkdir(parents=True,exist_ok=True);fig.savefig(a.out,dpi=160);plt.close(fig)


if __name__=='__main__':main()
