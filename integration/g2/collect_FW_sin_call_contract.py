"""Freeze reviewed source-count formulas, not measured lookup rates or WCET."""
from pathlib import Path
from datetime import datetime
import argparse,json
from prepare_baseline_fusion_retime import metadata,save
BASE=Path('F:/nav_integration_g2_20261023');REPO=Path(__file__).resolve().parents[2]
FW=BASE/'P11_quiesce_shared_guard_FW_prepared1/fw'

def collect(out):
    assert not out.exists();out.mkdir()
    named=['nav/fixmath.c','nav/traj_sw.c','nav/local_frame.c','nav/explore.c','nav/motion_guard.c',
        'nav/support_guard.c','nav/local_planner.c','nav/pose.c','nav/pose_match.c','nav/nav_task.c','app/nav_app.c','drv/hal_fpga.c']
    fs={r:(FW/r).read_text()for r in named}
    assert 'for (int r = 0; r <= NR; r++)' in fs['nav/explore.c'] and 'const int NR = 24;'in fs['nav/explore.c']
    assert 'if (!end && horizon > 64)'in fs['nav/motion_guard.c'] and 'if(steps>64)'in fs['nav/support_guard.c']
    assert 'do st = nav_rd(NAV_TRAJ_STAT); while (!(st & NAV_TRAJ_STAT_DONE_MASK));'in fs['drv/hal_fpga.c']
    items=[
        dict(path='nav/fixmath.c',function='fx_sin/fx_cos',source_lookup_calls='1 each',frequency='all relevant consumers below; real rate not measured'),
        dict(path='nav/fixmath.c',function='fx_rot_zyx',source_lookup_calls='12 per call',frequency='initial mounting and per lidar transform, not a500Hz inference loop'),
        dict(path='nav/traj_sw.c',function='traj_eval_sw',source_lookup_calls='2*k*t maximum, early collisions reduce',frequency='software reference/fallback; actual selected HAL uses hardware and this function is absent in new linked ELF'),
        dict(path='nav/local_frame.c',function='local_frame_point/local_frame_pose',source_lookup_calls='4 per call',frequency='planning-frame transforms'),
        dict(path='nav/local_frame.c',function='local_frame_preserve',source_lookup_calls='0 unchanged;8 when correction changes frame',frequency='pose/frame corrections'),
        dict(path='nav/explore.c',function='explore_seen_mark',source_lookup_calls='50 (25 rays *sin/cos)',frequency='with prior+map and no hold in task_step_local'),
        dict(path='nav/explore.c',function='explore_pick',source_lookup_calls='32 on failed selection;34 on successful selection',frequency='after frontier fallback, not every BFS node'),
        dict(path='nav/explore.c',function='explore_frontier',source_lookup_calls='0 internally',frequency='BFS still has existing bounded search checkpoints'),
        dict(path='nav/motion_guard.c',function='motion_check',source_lookup_calls='4*(h+1)+4*U0 maximum;h=ceil(safe_steps*dt_ms/50)<=64',
            default='100+4*U0 at h24',U0='s0 unknown cells not completely contained in original footprint, each recomputes a sin/cos interpolated pair',
            warning='260 is not a complete upper bound: the s0 unknown cell loop adds4 per qualifying cell',frequency='candidate detail checks and final checked command exit'),
        dict(path='nav/motion_guard.c',function='motion_check pure rotation/motion_segment',source_lookup_calls='pure rotation<=4 depending unknown cells;motion_segment0',frequency='special branches retain original semantics'),
        dict(path='nav/support_guard.c',function='support_check',source_lookup_calls='8*(steps+1),steps<=64; maximum520;default200;pure rotation40',frequency='up to six selected localplanner details plus unshared final source check; early query failure reduces'),
        dict(path='nav/local_planner.c',function='lp_plan',source_lookup_calls='default<=1800+4*sum(U_i), at most3+3=6 CPU detailed candidates',frequency='final send_checked motion adds separately; support may be reused when exact CMD_LOCAL tuple agrees'),
        dict(path='nav/pose.c',function='pose_odom',source_lookup_calls='2 normal;4 when pitch decomposition enabled',frequency='per consumed odometry feedback, actual observed rate not measured'),
        dict(path='nav/pose.c',function='pose_set',source_lookup_calls='2; history<=32 does not multiply these calls',frequency='explicit pose/frame reset'),
        dict(path='nav/pose.c',function='pose_at future prediction',source_lookup_calls='4 normal;5 pitch-decomposed',frequency='frame alignment/extrapolation'),
        dict(path='nav/pose.c',function='pose_lidar_tf/pose_body_to_odom/anchor_z_at',source_lookup_calls='12/2/8 respectively',frequency='real frame/target/anchor transformations'),
        dict(path='nav/pose_match.c',function='match commit including pose_set',source_lookup_calls='4',frequency='accepted localization correction'),
        dict(path='drv/hal_fpga.c',function='hal_match_lf_forward',source_lookup_calls='2',frequency='each accepted LF update wrapper')]
    anchors=[]
    for r,t in fs.items():
        for n,line in enumerate(t.splitlines(),1):
            if any(k in line for k in('fx_sin(', 'fx_cos(', 'NAV_SIN_LUT[','nav_app_search_checkpoint(', 'do st = nav_rd(NAV_TRAJ_STAT);')):
                anchors.append(dict(path=str(FW/r),line=n,text=line.strip()))
    result=dict(at=datetime.now().astimezone().isoformat(),scope='reviewed source formulas only; no EDA/CPU timing run',
        source_bindings=[metadata(FW/r)for r in named],counts=items,actual_source_anchors=anchors,
        checkpoints=dict(frontier='per256 clearing cells/per32 BFS nodes',
             no_internal_checkpoint_in='explore_pick, localplanner, motion/support, localframe/pose transforms',
             HAL_busy_wait='actual hardware hal_traj_eval DONE wait has no timeout or search checkpoint; preexisting issue cannot be closed by LUT relocation'),
        performance_caveats=['100ms planner interval is a target, not measured actual period or completed work budget',
             'original C table reads could be merged by compiler; new volatile MMIO calls must execute when reached, so equal numeric outputs do not imply same instructions or latency',
             'proposed CPU-local ROM avoids NAV/host bus arbitration but still adds response and instruction latency, unmeasured until real bus/Pico tests',
             'the native numeric probe201058 modeled reads describes that explicit finite test only; it is not a real task lookup frequency',
             'P11 strict sequential snapshots may safely lock on missed publications during busy planning; usability and350ms zero/500ms stop remain separate gates'],
        parent_summary=str(BASE/'FW_sin_MMIO_actual_review1/result.json'),
        RTL_generated=False,EDA_started=False,selected_image_changed=False,actual_frequency_or_WCET_proven=False,deployment_ready=False)
    save(out/'result.json',result);save(REPO/'docs/4_design/evidence/g2_20261023/FW_sin_source_call_contract_20261026.json',result)
    print(json.dumps(dict(output=str(out),source_consumers=len(items),timing_proven=False)))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=BASE/'FW_sin_source_call_contract_review2');collect(p.parse_args().out)
