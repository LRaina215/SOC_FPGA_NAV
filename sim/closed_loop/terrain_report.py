"""Aggregate denominators, full trial/video index and reproducible figures."""
import json
from pathlib import Path
import numpy as np
from terrain_study import OUT,ROOT,RES,matrix,dump


def aggregate(p):
    rows=json.loads((p/'map_metrics.json').read_text())['frames'];keys=['roi','observed','unknown','danger','observed_danger','raw_tp','blocked_tp','free','observed_free','raw_fp','blocked_fp','boundary','boundary_observed','boundary_raw','unknown_but_cost_free','height_count','height_abs_sum_mm','stale_observed']
    s={k:sum(a[k] for a in rows) for k in keys}
    for key,num,den in [('unknown_fraction','unknown','roi'),('raw_recall_observed','raw_tp','observed_danger'),('blocked_recall_observed','blocked_tp','observed_danger'),('raw_false_positive_observed','raw_fp','observed_free'),('height_mae_mm','height_abs_sum_mm','height_count')]:s[key]=s[num]/s[den] if s[den] else None
    s['max_height_error_mm']=max((a['height_max_mm'] or 0 for a in rows),default=0)
    s['frames']=len(rows);return s


def primary_paths():
    return {name:OUT/('primary_recovered' if name.startswith('trench') and name!='trench20d50' else 'primary')/name for name in matrix()}


def run_index():
    rows=[]
    for f in sorted(list(OUT.rglob('result.json'))+list(OUT.rglob('metrics.json'))):
        if 'frozen' in f.parts or 'fast_AU' in f.parts or 'n2av' in f.parts:continue
        p=f.parent;nav=f.name=='metrics.json';d=json.loads(f.read_text());phy=p/'physics_backend' if nav else p
        if not (phy/'full_state.npz').exists():continue
        video=p/('video_map_v2.mp4' if (p/'maps_v2/map_frames.npz').exists() else 'video.mp4')
        row=dict(id=str(p.relative_to(OUT)),mode='navigation' if nav else 'fixed_command',
                 outcome=d['outcome'],time_s=d.get('simulation_end_s',d.get('simulated_s')),
                 video=str(video.resolve()) if video.exists() else None,
                 video_relative=str(video.relative_to(OUT)),physical_state=str((phy/'full_state.npz').relative_to(OUT)),
                 contacts=str((phy/'contacts.jsonl').relative_to(OUT)),
                 map_source='maps_v2' if (p/'maps_v2/map_frames.npz').exists() else '.',
                 physical_trial_independence='Repeated deterministic seeds are not independent vehicle evidence')
        if d.get('analysis_outcome'):row['analysis_outcome']=d['analysis_outcome']
        if p.parent.name=='navigation':row['map_source_superseded']=True
        if p.parent.name=='friction_low':row['analysis_outcome']='superseded: requested0.4 but actual contact0.6; use friction_low_v2'
        if nav:
            t=np.genfromtxt(p/'timeseries.csv',delimiter=',',names=True);invalid=(t['x']<-.79)|(t['x']>3.79)|(abs(t['y'])>1.29)
            row['left_declared_domain_s']=float(t['t'][np.argmax(invalid)]) if invalid.any() else None
            if invalid.any():row['analysis_outcome']='left_declared_domain; later collisions not attributed to terrain'
            else:row['analysis_outcome']=d['outcome']
        rows.append(row)
    dump(OUT/'trial_index.json',rows)
    lines=['# 原4500地形试验与视频索引','','数据实际保存在 F:\\nav_terrain4500_20261022\\evidence；仓库sim/build目录为目录联接。','',
           '每个视频从对应完整状态生成。固定命令“passed”仅指越过出口平面且未失稳/非轮接触；导航仍按150mm、停稳、2s观察评分。',
           '重复的pilot、补跑和同种子确定性轨迹不是独立实车样本。maps_v2替代旧批量求交观测，完整旧记录仍保留。','',
           '| 运行 | 模式 | 结果 | 连续记录(s) | 视频 |','|---|---|---|---:|---|']
    for r in rows:
        link='[完整视频](<'+r['video_relative'].replace('\\','/')+'>)' if r['video'] else '**尚未生成**'
        lines.append(f'| {r["id"]} | {r["mode"]} | {r.get("analysis_outcome",r["outcome"])} | {r["time_s"]:.1f} | {link} |')
    (OUT/'VIDEO_INDEX.md').write_text('\n'.join(lines)+'\n',encoding='utf-8');return rows


def make_figures():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap,BoundaryNorm
    plt.rcParams['font.sans-serif']=['Microsoft YaHei'];plt.rcParams['axes.unicode_minus']=False
    out=ROOT/'docs/4_design/figures/terrain4500_20261022';out.mkdir(parents=True,exist_ok=True)
    fig,axes=plt.subplots(3,4,figsize=(14,9),layout='constrained')
    cases=[('up8',7.0),('step20',5.0),('trench40d150',4.4)]
    for row,(name,t) in enumerate(cases):
        p=primary_paths()[name];d=json.loads((p/'result.json').read_text());m=np.load(p/'maps_v2/map_frames.npz');i=min(len(m['t'])-1,np.searchsorted(m['t'],t-1e-6))
        x=(np.arange(128)+m['cx'][i]-64+.5)*RES;y=(np.arange(128)+m['cy'][i]-64+.5)*RES
        extent=[x[0]-RES/2,x[-1]+RES/2,y[0]-RES/2,y[-1]+RES/2]
        from terrain_scene import TerrainSpec,TerrainScene
        sc=TerrainScene(TerrainSpec(**d['scene']['spec']));X,Y=np.meshgrid(x,y)
        arrays=[sc.height(X,Y)*1000,np.where(m['V'][i],m['H'][i],np.nan),np.where(m['V'][i],m['raw'][i].astype(float),np.nan),np.select([m['cost'][i]==255,m['cost'][i]==254,m['cost'][i]==253],[3,2,1],default=0)]
        for col,a in enumerate(arrays):
            ax=axes[row,col]
            if col<2:im=ax.imshow(a,origin='lower',extent=extent,vmin=-150,vmax=100,cmap='terrain')
            elif col==2:ax.imshow(a,origin='lower',extent=extent,vmin=0,vmax=1,cmap=ListedColormap(['#47995b','#e34739']))
            else:ax.imshow(a,origin='lower',extent=extent,cmap=ListedColormap(['#47995b','#e5ae32','#e34739','#464a51']),vmin=0,vmax=3)
            ax.set_xlim(.2,1.3);ax.set_ylim(-.4,.4);ax.set_aspect('equal');ax.axvline(.55,color='black',ls='--',lw=.6)
            if row==0:ax.set_title(['独立真实高程(mm)','理想位姿高程／灰=未知','原始危险／灰=未知','绿自由 黄膨胀 红危险 灰未知'][col],fontsize=10)
            if col==0:ax.set_ylabel(f'{name} @ {m["t"][i]:.1f}s\ny(m)')
            ax.set_xlabel('x(m)');ax.set_facecolor('#b1b1b1')
    fig.savefig(out/'map_examples.png',dpi=150);plt.close(fig)
    # Show physical tracking separately from map classification or navigation success.
    fig,axes=plt.subplots(1,2,figsize=(12,4),layout='constrained')
    for name in ['flat','up8','step20','trench40d150']:
        p=primary_paths()[name];a=np.genfromtxt(p/'physics.csv',delimiter=',',names=True)
        axes[0].plot(a['t'],a['x'],label=name);axes[1].plot(a['t'],np.degrees(a['pitch']),label=name)
    axes[0].set(xlabel='固定命令试验时间(s)',ylabel='真实轮轴参考x(m)');axes[1].set(xlabel='时间(s)',ylabel='真实俯仰角(°)')
    for ax in axes:ax.legend();ax.grid(alpha=.25)
    fig.savefig(out/'physical_response.png',dpi=150);plt.close(fig)


def main():
    direct=[]
    for name,p in primary_paths().items():
        d=json.loads((p/'result.json').read_text());a=aggregate(p/'maps_v2')
        direct.append(dict(case=name,physical=d,diagnostic_map=a))
    nav=[]
    for p in sorted((OUT/'navigation_v2').glob('*/metrics.json')):
        nav.append(dict(case=p.parent.name,metrics=json.loads(p.read_text()),map=aggregate(p.parent)))
    pose=[]
    for p in sorted((OUT/'navigation_v2').glob('*/pose_replay_ideal/map_metrics.json')):
        pose.append(dict(case=p.parent.parent.name,ideal=aggregate(p.parent),estimated=aggregate(p.parent.parent/'pose_replay_estimated')))
    dump(OUT/'analysis_summary.json',dict(primary=direct,navigation=nav,pose_replays=pose,
        count_unit='Repeated fixed-corridor cell observations; boundary counted separately; independent geometry envelope is not original4500 capability'))
    index=run_index();make_figures()
    table=['## 9. 本轮逐例与统计表（自动生成）','','### 27个主工况','','| 工况 | 固定命令结果 | 进入/越出(s) | 高程MAE(mm) | 直接观测/总格观测 | 参考危险已观测：原始/膨胀阻塞 | 未知率 |','|---|---|---|---:|---|---|---:|']
    for a in direct:
        p=a['physical'];m=a['diagnostic_map'];en=p['entry_s'];ex=p['reached_exit_s'];times=f'{en:.2f}' if en is not None else '未入场';times+=' / '+(f'{ex:.2f}' if ex is not None else '未越出')
        table.append(f'| {a["case"]} | {p["outcome"]} | {times} | {m["height_mae_mm"]:.2f} | {m["observed"]}/{m["roi"]} | {m["observed_danger"]}：{m["raw_tp"]}/{m["blocked_tp"]} | {m["unknown_fraction"]:.1%} |')
    table += ['','危险参考与物理能力独立，分母为重复格观测；0个参考危险格不显示虚构召回率。边界数、原始误报分母和填补/过期格见analysis_summary.json。','','### 同场景真实导航','','| 场景 | N1c | AS | AU（否决） |','|---|---|---|---|']
    lookup={a['case']:a['metrics'] for a in nav}
    for name in ['flat','up8','step5','step20','downstep20','trench20d50','trench40d150','trench100d150']:
        line=[name]
        for alg in ['N1c','AS','AU']:
            d=lookup[alg+'_'+name];label=d['outcome']
            if alg=='N1c' and name in ['step20','downstep20']:label='离开声明域，后续跌落不归因地形'
            line.append(f'{label}，{d["time_s"]:.1f}s')
        table.append('| '+' | '.join(line)+' |')
    table += ['','导航实际路径不相同，不能把各自地图累计百分比直接当作同输入算法改进。','','### 相同原始帧：估计/理想采集位姿','','| 运行 | 帧数 | 估计回放与原地图逐项一致 | 高程MAE估计→理想(mm) | 原始误报/直接观测自由格：估计→理想 |','|---|---:|---|---|---|']
    checks=json.loads((OUT/'pose_replay_comparison.json').read_text()) if (OUT/'pose_replay_comparison.json').exists() else []
    for a in pose:
        e,i=a['estimated'],a['ideal'];c=next(x for x in checks if x['case']==a['case'])
        table.append(f'| {a["case"]} | {c["frames"]} | {c["estimated_replay_equals_recorded_maps"]} | {e["height_mae_mm"]:.2f}→{i["height_mae_mm"]:.2f} | {e["raw_fp"]}/{e["observed_free"]}→{i["raw_fp"]}/{i["observed_free"]} |')
    table += ['','理想采集位姿含真实机体升沉和姿态。改用理想位姿会改变落格和地图中心，这是定位误差隔离，不是受测导航获得了真值。各帧采用相同深度/顺序和同一分类配置。','','### 旧N2关键例AU复验','','| 同一新表面射线，种子3 | 保留AS | AU |','|---|---|---|','| ramp | 120s超时，误差369mm | 120s超时，误差397mm |','| gap（55cm窄通道） | 61.8s到达，误差90mm | 90s超时，误差2.277m |','','AU没有联合收益，未合入；同口径N1c原两例18.3/13.7s到达仍保留，历史28/50仍仅属于旧射线源。']
    report=ROOT/'docs/4_design/local_map_terrain_feasibility_report_20261021.md'
    prefix=report.read_text(encoding='utf-8').split('<!-- GENERATED_TERRAIN_TABLES -->')[0]
    report.write_text(prefix+'<!-- GENERATED_TERRAIN_TABLES -->\n\n'+'\n'.join(table)+'\n',encoding='utf-8')
    print(dict(primary_cases=len(direct),navigation_cases=len(nav),physical_trial_records=len(index),missing_videos=sum(a['video'] is None for a in index)),flush=True)


if __name__=='__main__':main()
