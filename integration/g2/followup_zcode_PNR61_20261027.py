"""Reconcile the requested Zcode session with reports and a live read-only check.

This records evidence already obtained in this follow-up. It never connects to
the board, starts EDA, repeats hardware tests, or calculates hashes.
"""
from collections import Counter
from datetime import datetime
import difflib
import json
from pathlib import Path
import re
import subprocess

from prepare_baseline_fusion_retime import load, metadata, save

ROOT = Path(__file__).resolve().parents[2]
BASE = Path('F:/nav_integration_g2_20261023')
BUILD = BASE/'K_pnr61_s49p2r0'
SESSION = Path('C:/Users/Li/.zcode/cli/rollout/model-io-sess_6ac1dbd7-1a60-4d06-a50e-b27b1a031b31.jsonl')


def main():
    tr = BUILD/'impl/pnr/g2_top.tr'
    status, clocks, totals = Counter(), [], {}
    header = []
    for line in tr.open(encoding='utf8'):
        if len(header) < 85:
            header.append(line)
        m = re.match(r'\s*(TC_\w+)\s+(\w+)\s+',line)
        if m:
            status[m[2]] += 1
    head = ''.join(header)
    for kind in ('Setup','Hold'):
        totals[kind.lower()] = int(re.search(r'<Numbers of '+kind+r' Violated Endpoints>:(\d+)',head)[1])
    for m in re.finditer(r'^\s*\d+\s+(clk50|rgmii_rxc)\s+([\d.]+)\(MHz\)\s+([\d.]+)\(MHz\)',head,re.M):
        clocks.append(dict(name=m[1],target_MHz=float(m[2]),reported_Fmax_MHz=float(m[3])))
    assert totals == dict(setup=0,hold=0)
    assert clocks == [dict(name='clk50',target_MHz=50.0,reported_Fmax_MHz=50.114),
                      dict(name='rgmii_rxc',target_MHz=100.0,reported_Fmax_MHz=152.934)]
    assert status == Counter(Actived=61)
    rpt = BUILD/'impl/pnr/g2_top.rpt.txt'
    report = rpt.read_text(encoding='utf8')
    resources = {}
    for key in ('Logic','Register','CLS','BSRAM'):
        m = re.search(r'^\s*'+key+r'\s+\|\s+(\d+)/(\d+)',report,re.M)
        resources[key] = dict(used=int(m[1]),capacity=int(m[2]))
    assert resources['BSRAM']['used'] == 280
    assert resources['Register']['used'] == 32017
    assert resources['CLS']['used'] == 42995
    log = (BUILD/'gw_sh.log').read_text(encoding='utf8')
    assert 'Placement and routing completed' in log and 'PR0004' not in log
    raw = [json.loads(line) for line in SESSION.read_text(encoding='utf8').splitlines() if line.strip()]
    ids = sorted({str(x.get('sessionId')) for x in raw})
    assert all('6ac1dbd7-1a60-4d06-a50e-b27b1a031b31' in item for item in ids),ids

    # Exact public values from the single read-only RK selftest in this turn.
    # NaN attitude is represented as unavailable, never converted to a valid0.
    live = dict(SYS_ID='0x4E415630',SYS_VERSION='0x00010000',
        soc=dict(ram_kb=128,status='0x000015BC',hold_reset=0,console_bytes=1661322),
        sys_time_dt_s=3.059,lidar_fps=0.0,lidar_bad_delta=0,lidar_phy='0x00270006',
        odo_rate_hz=0.0,
        safe=dict(SAFE_CTRL='0x00000001',SAFE_STAT='0x1E810000',
                  SAFE_OUT_V='0x00000000',SAFE_OUT_W='0x00040000'),
        telemetry=dict(frames=304,rate_hz=101.3,bad_crc=0,skipped=0,
            last=dict(time_s=16391.789,sequence=778,status=10,policy_fault=0,sensor_error=5,
                left_fault=0,right_fault=0,fusion_flags=1,gyro_x=0.0,gyro_y=0.0,gyro_z=0.0,
                accel_x_g=0.0,accel_y_g=0.0,accel_z_g=0.0,pitch_deg=None,roll_deg=None,
                forward_command=0.0,yaw_command=0.0,height_command=.1306,
                left_target=0.0,right_target=0.0,left_speed=0.0,right_speed=0.0,
                left_duty=0,right_duty=0,lh_us=1500,lk_us=1500,rh_us=1500,rk_us=1500,
                completed_frames=0,inference_cycles=0,left_delta=0,right_delta=0,active=False)))
    recorded = datetime.now().astimezone().isoformat()
    evidence = dict(recorded_at=recorded,requested_session_id=ids,
        requested_export_path_exists=False,recovered_exact_session_rollout=str(SESSION),
        session_records=len(raw),session_last_completed=raw[-1].get('completedAt'),
        observed_repository_head=subprocess.check_output(['git','rev-parse','--short','HEAD'],cwd=ROOT,text=True).strip(),
        current_build=str(BUILD),routing_completed=True,explicit_unrouted_numeric_field=None,
        global_setup_hold=totals,clock_rows=clocks,constraints_status=dict(status),resources=resources,
        deployment_record=dict(source='Zcode final response and committed k_bench README',
            SRAM_download_time='2026-10-27 03:50 +08:00',UserCode='0xC30F',Flash_written=False,
            independent_JTAG_or_complete_bitstream_readback_this_turn=False),
        firmware_record=dict(forward_mm_loaded=1000,loaded_time='2026-10-27 04:10 +08:00',
            baked_PNR61_INIT_forward_mm=1500,
            re_download_requires_reload_or_explicit_config=True,
            live_binary_identity_not_read_back_this_turn=True),
        live_readonly_selftest=live,
        collection_window='After RK date2026-10-27T08:22:51+08:00 and before local08:25:25+08:00; exact test start was not saved',
        remote_bench_source_byte_identical_to_committed_source=True,
        remote_motion_watch=dict(last_line='10-27 07:00:14 deadline reached without vehicle power',
            powered_results_files_found=False,watch_process_observed=False,
            note='That log message is the script interpretation; electrical supply was not remotely measured'),
        electrical_root_cause_confirmed=False,
        current_blockers=['BMI088 error5/invalid attitude and no policy inference',
            'Lidar PHY link0 and0fps', 'Actual axis signs/support/power and vehicle stopping still unverified'],
        finite_navigation_models_not_original4500_vehicle_closed_loop=True,
        active_PNR_or_SYN_observed=False,old55_56_status_is_historical=True,
        new_EDA_started=False,programmed_this_turn=False,motion_commands_sent_this_turn=False,
        passed_full_vehicle_acceptance=False,hash_checks_performed=False,
        binding_policy='Source metadata at read time, before this documentation synchronization; not immutable EDA admission tokens',
        bindings=[metadata(p) for p in (SESSION,tr,rpt,BUILD/'gw_sh.log',BUILD/'g2.sdc',
            ROOT/'integration/g2/k_bench_20261027/README.md',
            ROOT/'integration/g2/k_bench_20261027/rk_bench/bench.py')])
    receipt=ROOT/'docs/4_design/evidence/g2_20261023/Zcode_PNR61_live_readonly_followup_20261027.json'
    assert not receipt.exists()
    save(receipt,evidence)
    section='''**2026-10-27 08:27 已跟进指定Zcode会话，PNR61报告时序收敛；实车仍受传感器阻塞。** 指定导出路径当前不存在，找到精确同ID原始rollout，并核对后续Git提交与实际K_pnr61_s49p2r0报告，未重跑旧55/56。61于03:44完成routing/位流；实测BSRAM280/340、Logic60893、FF32017、CLS42995/69120，PRIMARY/LW各8/8。目标50/100MHz下Fmax50.114/152.934MHz、全局及各域setup/hold均0，61条报告约束全部Actived。这是实际物理报告结论；全部IO/CDC载荷/外部复位脉宽、完整原4500实车闭环和最终停止仍不据此自动接受。

会话及已提交k_bench记录03:50仅SRAM下载G3候选、UserCode C30F；04:10调试口装载前移1000固件，位流内INIT仍为1500，重新下载后需重装或显式配置。本轮未重新JTAG确认完整镜像身份、未下载或发动作。08:22以后经RK做唯一3秒只读selftest（远端bench源先与仓库逐字核对）：CPU128KiB/未HOLD、时间递增，304帧遥测/CRC0；BMI status10/error5、姿态无效、策略active=false/完成推理0，雷达PHY00270006/link0/0fps、里程反馈0，指令/占空比为0。不能把这些0当有效停稳证明。远端watch.log于07:00:14自然到期，未有powered_results文件，也未观察到旧watch进程；脚本“未供电”只是解释，实际供电/接线/I2C拉低根因未测。当前已询问供电状态，接下来先新鲜六轴/雷达、轴向与反馈，再做支撑低幅测试；无需再跑旧布线。[会话与现场核对证据](../4_design/evidence/g2_20261023/Zcode_PNR61_live_readonly_followup_20261027.json)。

'''
    table=('| G2共同资源 | PNR61已布通280RAM，FF32017/CLS42995 | '
        '50/100MHz报告setup/hold均0；61约束Actived | '
        '会话记录G3候选已SRAM；本轮只读CPU/遥测正常 | '
        'BMI088错误5、雷达无链路，实际原4500闭环/轴向/停稳仍待；无活动构建 |')
    plan=ROOT/'docs/1_plan/system_integration_plan_20261021.md'
    def amend(text):
        assert '08:27 已跟进指定Zcode会话' not in text
        text,n=re.subn(r'^\| G2共同资源 \|.*$',table,text,flags=re.M)
        assert n==1
        marker='## 2. 先完成G2：真实共同工程资源\n\n'
        assert text.count(marker)==1
        text=text.replace(marker,marker+section,1)
        marker='“源码实现”“专项通过”“联合通过”“实车通过”是不同结论；空缺不能按已完成理解。\n\n'
        notice='**最新现场（2026-10-27 08:27）：** PNR61已布通，50/100MHz报告setup/hold均0；会话记录G3候选已SRAM下载。此次RK只读检查CPU/遥测正常，BMI088错误5、雷达无链路，当前转入供电/传感器/轴向与整车测试；下列较早独立SFP镜像状态保留为历史。\n\n'
        assert text.count(marker)==1
        return text.replace(marker,marker+notice,1)
    plan.write_text(amend(plan.read_text(encoding='utf8')),encoding='utf8',newline='\n')
    rel=plan.relative_to(ROOT).as_posix()
    old=subprocess.check_output(['git','show',':'+rel],cwd=ROOT).decode('utf8')
    patch=''.join(difflib.unified_diff(old.splitlines(True),amend(old).splitlines(True),fromfile='a/'+rel,tofile='b/'+rel))
    subprocess.run(['git','apply','--cached','-'],cwd=ROOT,input=patch.encode('utf8'),check=True)
    bench=ROOT/'integration/g2/k_bench_20261027/README.md'
    note='''## 08:27 跟进现场（只读）

PNR61实际报告已重新核对：50/100MHz setup/hold均0，61条Actived，BSRAM280/340，FF32017/CLS42995；PNR57的对应值为31491/42760。经RK唯一3秒只读检查收到304帧/CRC0，CPU运行，但BMI088仍error5/active=false、姿态NaN，雷达link0/0fps，尚无原4500有效推理。07:00等待脚本已经退出、没有powered_results；日志“without vehicle power”不是供电电压测量，仍需区分供电、接线和I2C线故障。本轮没有发动作或重新下载。3种子窗口结果继续只归有限导航模型，不是原4500实车闭环或普遍不漂移的证明。

'''
    bt=bench.read_text(encoding='utf8')
    assert '## 08:27 跟进现场' not in bt
    bt=bt.replace('## 结论\n',note+'## 结论\n',1)
    bt=bt.replace('## G3：BMI088 姿态接入导航（候选，未部署）','## G3：BMI088 姿态接入导航（候选已下SRAM，传感器与轴向待验）')
    bt=bt.replace('128 前移 1.5 m（当前固件缺省）','128 前移 1.5 m（早期固件缺省）')
    bench.write_text(bt,encoding='utf8',newline='\n')
    machine=BASE/'lane2_ddr_map_evaluation/job.json'
    state=load(machine)
    state.update(phase='PNR61_report_timing_closed_live_sensor_blocked',current_active_SYN=None,
        current_active_PNR=None,queued_PNR=None,active_readonly_supervision=None,last_pending_admission=None,
        latest_joint_route_result=dict(build=str(BUILD),routing_completed=True,
            explicit_unrouted_numeric_field=None,BSRAM=280,FF=32017,CLS=42995,
            setup=0,hold=0,clock_rows=clocks,constraints_status=dict(status)),
        latest_PNR_resources_BSRAM=280,latest_PNR_setup=0,latest_PNR_hold=0,
        latest_root_timing_checkpoint=str(receipt),latest_known_commit=evidence['observed_repository_head'],
        latest_live_readonly_selftest=live,programmed_in_this_turn=False,
        next_step='Resolve fresh BMI088/lidar supply or wiring, verify axes and actual feedback, then supported low-amplitude vehicle tests; do not resume obsolete55/56')
    save(machine,state)
    print(json.dumps(dict(receipt=str(receipt),PNR61_setup=0,PNR61_hold=0,
        actual_BSRAM=280,live_sensor_error=5,new_EDA=False)))


if __name__=='__main__':
    main()
