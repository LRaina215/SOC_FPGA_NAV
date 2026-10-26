"""Record completed synthesis and finite command proof without accepting STA."""
from datetime import datetime
from pathlib import Path
import difflib
import json
import re
import subprocess

from prepare_baseline_fusion_retime import metadata, save

ROOT = Path(__file__).resolve().parents[2]
BASE = Path('F:/nav_integration_g2_20261023')


def load(p):
    return json.loads(p.read_text(encoding='utf-8-sig'))


def main():
    syn = BASE/'SLF128_ring_stride32_syn44'
    job = load(syn/'job.json')
    assert job['phase'] == 'synthesis_complete' and job['tool_exit'] == 0
    summary = load(syn/'synthesis_summary.json')
    top = summary['synthesis_hierarchy'][0]
    assert top['path'] == '/g2_top' and not summary['errors']
    nav = BASE/'NAV_command_predecode_bus_probe2'
    result = load(nav/'result.json')
    assert result['passed'] and result['inputs_unchanged']
    log = (nav/'3_xsim.log').read_text(encoding='utf8')
    line = re.search(r'TEST PASSED NAV_COMMAND_BUS[^\r\n]*', log)[0]
    metrics = {k:int(v) for k,v in re.findall(r'(\w+)=(\d+)', line)}
    assert metrics == dict(checks=1449, transactions=192, resets=2,
                          arbitration=127, xz=3, unknown_read_words=0)
    pid = BASE/'PID_unity_actual_control_probe2_modelsim'
    pj = load(pid/'job.json')
    stamp = datetime.now().astimezone().isoformat()
    evidence = dict(recorded_at=stamp, user_priority='timing closure before new P11/model experiments',
        actual_SYN44_complete=True, synthesis_job=job, actual_source_resources=top,
        synthesis_estimates_only=summary['synthesis_timing'],
        source_BSRAM=269, predicted_physical_BSRAM=280, predicted_wide_split_pairs=11,
        physical_resources_not_measured_for_SYN44=True,
        ring_actual_mapping_review_pending=True,
        observer_frame_RAM_actual_count=78, observer_frame_RAM_expected_count=72,
        unexpected_count_must_be_explained_before_mapping_admission=True,
        NAV_finite_bus_probe_passed=True, NAV_metrics=metrics,
        NAV_scope='Real old/new NAV and two original arbiters, six pulses and bus/SAFE outputs compared; MAC RX idle',
        NAV_full_RAW_FIFO_MBX_console_concurrency_not_exercised=True,
        first_NAV_probe_failure='Module-local register-map constants suppressed by shared include guard in simulation copies; original failure retained',
        second_NAV_probe_change='Only comparison copies undef the header guard before including original declarations; frozen hardware unchanged',
        PID_math_all7_UNSAT=True,
        first_PID_tool_failure='xvlog passed, xelab crashed after static elaboration with EXCEPTION_ACCESS_VIOLATION; no functional simulation result',
        PID_actual_fallback_job=pj,
        PID_fallback_not_yet_a_functional_pass=True,
        prior_two_place_options_both_routed=True,
        prior_PNR54=dict(BSRAM=280, CLS=41981, FF=30714,
                         clk50_setup=2709, clk50_hold=0, RX100_setup=0, RX100_hold=0),
        new_PNR_started=False, programmed=False, deployment_accepted=False,
        timing_IO_reset_CDC_and_vehicle_acceptance_pending=True,
        bindings=[metadata(p) for p in (syn/'job.json', syn/'synthesis_summary.json',
            nav/'result.json', nav/'3_xsim.log', pid/'job.json')])
    receipt = ROOT/'docs/4_design/evidence/g2_20261023/SYN44_complete_NAV_bus_and_PID_timing_checkpoint_20261027.json'
    assert not receipt.exists()
    save(receipt, evidence)

    table = ('| G2共同资源 | 两种布局均布通280RAM/11拆分、CLS61%/FF30714 | '
             '全部INIT/CPU/31877钟脚/26CDC/IR/lease保持；52有效 | '
             '54的50域setup2709/hold0，RX100域0/0；仍未时序准入 | '
             'SYN44完成，实际映射审阅中；NAV预译码有限对照通过，PID同拍等价验证运行，未部署 |')
    section = '''**2026-10-27 00:10 优先时序；SYN44实际结束，NAV有限对照通过。** SYN44于23:43:08自然结束0，原51268退出；FF29493/LUT38933/ALU9606、源BSRAM269，SSRAM1885，对43分别+11/+80/−24/0/+12，MULT12X12减少3。observer实际78个frame RAM与预计72不同，正在逐项核地址/写门/读选择和游标，不以总数改预期代替语义审阅。综合50MHz估计44.852，最差转到actions→lh_us，22.610ns/slack−2.296；仅是综合估计，尚无这一版物理时序或280块实测，不宣布50MHz通过。

NAV六脉冲预译码probe2自然通过：真实两套NAV/原总线仲裁，1449公共采样/192事务、两次复位/127次双请求/三项XZ，原命令/ACK/受理值/SAFE逐拍相同，未知读字0。probe1因比较模块共用include guard导致第二份缺localparam而编译失败，保留；仅比较副本补undef，冻结硬件未改。此测试MAC空闲，不冒称真实RAW/FIFO/MBX/控制台并发全覆盖。PID unity全24位动作可达范围及保持归纳的7项数学miter均UNSAT；首xsim静态展开后代码生成访问异常保留，AE替代会话已许可并加载原mapper/PID/PWM、生产50MHz/100Hz及原完整故障阈值，实际逐拍终局仍待。当前不新增P11/BMI或模型试验，已有模型队列保持原输入自然推进；后续只把已闭合实际映射与功能合同的时序修改接入联合构建，无新截止和时序例外。[实际综合与验证接续](../4_design/evidence/g2_20261023/SYN44_complete_NAV_bus_and_PID_timing_checkpoint_20261027.json)。

330秒巡检只读补证发现SLF128后段持续漂移：末444.804mm、最大610.843mm，而256对照末15.45mm；两者均0碰撞/九轮目标。用户取消50mm P95硬门槛不等于长期漂移接受；旧匹配记录缺时间/成功winner，根因仍未知。保留独立反例，硬件时序工作继续，当前不重跑或调模型。

'''
    path = ROOT/'docs/1_plan/system_integration_plan_20261021.md'
    def amend(text):
        assert '2026-10-27 00:10 优先时序' not in text
        text, count = re.subn(r'^\| G2共同资源 \|.*$', table, text, flags=re.M)
        assert count == 1
        marker = '## 2. 先完成G2：真实共同工程资源\n\n'
        assert text.count(marker) == 1
        return text.replace(marker, marker+section, 1)
    path.write_text(amend(path.read_text(encoding='utf8')), encoding='utf8', newline='\n')
    relative = path.relative_to(ROOT).as_posix()
    original = subprocess.check_output(['git', 'show', ':'+relative], cwd=ROOT).decode('utf8')
    changed = amend(original)
    patch = ''.join(difflib.unified_diff(original.splitlines(True), changed.splitlines(True),
                                       fromfile='a/'+relative, tofile='b/'+relative))
    subprocess.run(['git', 'apply', '--cached', '-'], cwd=ROOT,
                   input=patch.encode('utf8'), check=True)
    special = ROOT/'docs/4_design/g2_joint_resources_20261022.md'
    with special.open('a', encoding='utf8', newline='\n') as f:
        f.write('\n## 2026-10-27 00:10 时序主线接续\n\n'+section)
    machine_path = BASE/'lane2_ddr_map_evaluation/job.json'
    machine = load(machine_path)
    machine.update(phase='SYN44_complete_NAV_bus_passed_PID_equivalence_active', updated_at=stamp,
        current_active_SYN=None, current_active_PNR=None, queued_PNR=None,
        latest_complete_SYN=str(syn), latest_complete_SYN_runtime=job,
        latest_root_timing_checkpoint=str(receipt),
        timing_repair_priority=True, latest_known_commit='c9ba84c',
        next_step='Close actual ring mapping and NAV/PID functional proofs, then compose the measured timing repairs for one full synthesis; no deployment')
    save(machine_path, machine)
    print(json.dumps(dict(receipt=str(receipt), actual_SYN_complete=True,
                          NAV_finite_passed=True, PID_phase=pj.get('phase'))))


if __name__ == '__main__':
    main()
