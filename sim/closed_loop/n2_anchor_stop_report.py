"""Index the isolated AZ follow-up, without merging it into the prior94 trials."""
import json
from pathlib import Path
import numpy as np
from terrain_study import ROOT,OUT,dump


def read(p):return json.loads(p.read_text(encoding='utf-8'))


def main():
    base=OUT/'n2av/n2az';rows=[]
    for p in sorted(base.iterdir()):
        if not (p/'metrics.json').exists():continue
        m=read(p/'metrics.json')
        r=dict(id=p.name,outcome=m['outcome'],time_s=m['time_s'],end_s=m['simulation_end_s'],
            error_m=m['terminal']['final_error_m'],collisions=m['collisions'],
            backend=m.get('physics_backend',{}).get('backend','fast kinematic; not4500 dynamics'),
            query_bridge=m.get('host_support_snapshot'),path=str(p.resolve()))
        if (p/'stop_diagnostics.npz').exists():
            x=np.load(p/'stop_diagnostics.npz')['events'];r['stop_last']=x[-1].tolist()
        if (p/'anchor_diagnostics.npz').exists():
            x=np.load(p/'anchor_diagnostics.npz')['events'];r['anchor_last']=x[-1].tolist()
        rows.append(r)
    labs=[]
    for p in sorted((base/'stop_response').iterdir()):
        if not (p/'result.json').exists():continue
        r=read(p/'result.json');r.pop('physical',None);r['id']=p.name
        csv=p/'physics_backend/physics.csv'
        if csv.exists():
            x=np.genfromtxt(csv,names=True,delimiter=',');x=x[x['t']>=8.]
            yaw=np.unwrap(x['yaw']);r['max_true_yaw_change_deg']=float(np.max(abs(yaw-yaw[0]))*180/np.pi)
            corners=np.array([(f,l) for f in [-34,34] for l in [-149,-121,121,149]])/1000
            c=np.cos(yaw)[:,None];s=np.sin(yaw)[:,None]
            pts=np.stack([x['x'][:,None]+c*corners[:,0]-s*corners[:,1],x['y'][:,None]+s*corners[:,0]+c*corners[:,1]],axis=-1)
            r['max_declared_planar_tread_corner_excursion_mm']=float(np.max(np.linalg.norm(pts-pts[0],axis=-1))*1000)
        labs.append(r)
    # Compare actual arrays / bytes directly; no hash checks.
    frozen=ROOT/'sim/build/nav_opt/n2as_near_retry_source/fw';checked=[]
    for p in frozen.rglob('*'):
        if p.suffix not in ('.c','.h') or p.relative_to(frozen).as_posix()=='include/nav_regs_map.h':continue
        target=ROOT/'fw'/p.relative_to(frozen)
        assert p.read_bytes()==target.read_bytes(),str(target)
        checked.append(p.relative_to(frozen).as_posix())
    result=dict(accepted=False,deployed=False,accepted_baseline='N1c',live_firmware='AS',
        control_changed=False,production_C_H_equal_frozen_AS=checked,
        candidates='AZ anchors; BA/BE/BG/BJ observation; BB/BC/BH/BH2 stopping (separate branches)',
        trials=rows,stop_labs=labs,
        contracts={n:read(base/n/'result.json') for n in ('BB_contract','BC_contract','BH_contract','BH2_contract')},
        rv={n:dict(image_bytes=(base/n/'nav_fw.bin').stat().st_size,
            stack=read(base/n/'stack_analysis.json')) for n in ('BC_RV','BH_RV','BH2_RV','BJ_RV')},
        host_BJ=(base/'BJ_reproducible_v2/host.log').read_text(encoding='utf-8'),
        boundaries=['same seed is not independent validation','native MuJoCo is not real vehicle calibration',
                    'fast gap success does not validate4500 dynamics','no target whole-loop CPU timing or new bitstream'])
    dump(base/'checkpoint.json',result)
    parent=OUT/'n2av/checkpoint.json';p=read(parent)
    p['followup']=dict(checkpoint=str((base/'checkpoint.json').resolve()),accepted=False,
        summary='N2 anchors / observation and N3 response / stop remain under joint validation')
    dump(parent,p)
    lines=['# AZ接续：锚定、观察接入与停止包络','','生产AS、接受基线N1c；下表均为隔离候选，未部署。',
           '原4500控制不变。fast到达不能代替原4500物理运行；同种子重复不计独立样本。','',
           '| 运行 | 模型 | 结果 | 用时(s) | 真实误差(mm) | 碰撞 | 视频 |',
           '|---|---|---|---:|---:|---:|---|']
    for r in rows:
        video=f'[回放]({r["id"]}/video.mp4)' if (base/r['id']/'video.mp4').exists() else '—'
        lines.append(f'| [{r["id"]}]({r["id"]}/metrics.json) | {"原4500" if r["backend"].startswith("mujoco-original4500") else "fast"} | {r["outcome"]} | {r["time_s"]} | {r["error_m"]*1000:.1f} | {r["collisions"]} | {video} |')
    lines+=['','## 停止隔离实验','','实验未加地图准入，不能当作地形停住验证。',
            '| 运行 | 最大位移(mm) | 最终位移(mm) | 末5s平均绝对轮速(mm/s) |',
            '|---|---:|---:|---:|']
    for r in labs:
        lines.append(f'| [{r["id"]}](stop_response/{r["id"]}/result.json) | {r["max_reference_displacement_after_stop_mm"]:.2f} | {r["final_reference_displacement_after_stop_mm"]:.2f} | {r["last5s_mean_abs_odom_mm_s"]:.3f} |')
    lines+=['','完整状态/原始帧/地图/停止诊断保存在对应目录；失败分支保留。详见仓库规划中的AZ章节。']
    (base/'INDEX.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    # Scientific diagnostic figure; ground truth is used only for scoring.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(1,2,figsize=(12,4.4),layout='constrained')
    for label,p in [('AY timeout',ROOT/'sim/build/batch/nav_opt_n2ay_coverage_surface_probes3_20261022/f5_gap_s3'),('BA side observation',base/'BA_gap90')]:
        if not (p/'timeseries.csv').exists():continue
        x=np.genfromtxt(p/'timeseries.csv',names=True,delimiter=',');ax[0].plot(x['x'],x['y'],label=label)
    from matplotlib.patches import Rectangle
    for x,y,w,h in [(2.2,-.75,.3,.4),(2.2,.2,.3,.4),(3.1,-.3,.1,.9)]:
        ax[0].add_patch(Rectangle((x,y),w,h,color='gray',alpha=.5))
    ax[0].scatter([1.4,3.7],[-.1,-.1],c=['green','red']);ax[0].set(xlabel='x (m)',ylabel='y (m)',title='Field5 gap, seed3; fast model',aspect='equal');ax[0].legend()
    for name,label in [('straight_zero','zero request'),('straight_hold','aggressive hold (oscillates)'),('c_hold_xyaw','C linear + yaw hold')]:
        p=base/'stop_response'/name/'stop_feedback.npz'
        if not p.exists():continue
        x=np.load(p)['data'];i=np.searchsorted(x[:,0],8.);d=np.linalg.norm(x[i:,6:8]-x[i,6:8],axis=1)*1000
        ax[1].plot(x[i:,0]-8,d,label=label)
    ax[1].set(xlabel='time after stop request (s)',ylabel='reference displacement (mm)',title='Fixed4500 flat lab; no terrain guard');ax[1].legend(fontsize=8)
    for a in ax:a.grid(alpha=.25)
    fig.savefig(base/'progress.png',dpi=160);plt.close(fig)
    print(json.dumps(dict(trials=len(rows),labs=len(labs),unchanged_C_H=len(checked),accepted=False)))


if __name__=='__main__':main()
