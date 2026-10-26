"""Record real PNR55 identity/settings and the independent memory queue56."""
from datetime import datetime
from pathlib import Path
import difflib
import json
import re
import subprocess

import psutil

from audit_cdc_first_stages import sdc_fragment
from prepare_baseline_fusion_retime import load, metadata, same_bytes, save
from run_slf128_storage_place_pair import REPORTS

ROOT = Path(__file__).resolve().parents[2]
BASE = Path('F:/nav_integration_g2_20261023')


def main():
    source = BASE/'SLF128_measured_timing_syn45'
    build = BASE/'SLF128_measured_timing_place1_pnr55'
    reference = BASE/'baseline_fusion32_CDC52_pnr35'
    runtime = load(build/'tool_runtime.json')
    assert runtime['max_runtime_s'] == runtime['max_no_observable_progress_s'] == 0
    assert not runtime['cancelled'] and 'tool_exit' not in runtime
    proc = psutil.Process(runtime['pid'])
    assert abs(proc.create_time()-1793033079.044567) < .01
    assert Path(proc.cwd()).resolve() == build.resolve()
    assert same_bytes(source/'impl/gwsynthesis/g2_top.vg',build/'g2_top.vg')
    assert same_bytes(reference/'g2.cst',build/'g2.cst')
    assert same_bytes(reference/'impl/pnr/device.cfg',build/'impl/pnr/device.cfg')
    oldcmd = (reference/'impl/pnr/cmd.do').read_text()
    actual = (build/'impl/pnr/cmd.do').read_text()
    assert oldcmd.replace(str(reference),str(build)) == actual
    root_review = BASE/'SYN45_root_PNR_admission1/result.json'
    admission = load(root_review)
    roles = load(Path(admission['radar26_file']))
    expected = ('create_clock -name clk50 -period 20 [get_ports {clk50}]\n'
        'create_clock -name rgmii_rxc -period 10 [get_ports {rgmii_rxc}]\n'
        +sdc_fragment(roles)+'\n# One documented report per type in this same SDC.\n'
        +'\n'.join(REPORTS)+'\n')
    assert (build/'g2.sdc').read_text() == expected
    queue = load(BASE/'SLF128_measured_timing_place2_pnr56_one_shot/job.json')
    assert queue['phase'] == 'waiting_for_actual_memory_admission'
    assert queue['total_limit_s'] == queue['silent_limit_s'] == 0
    qproc = psutil.Process(queue['controller_pid'])
    assert abs(qproc.create_time()-queue['controller_created_epoch']) < .01
    evidence = dict(at=datetime.now().astimezone().isoformat(),actual_PNR55_started=True,
        build=str(build),actual_runtime=runtime,
        tool=dict(pid=proc.pid,created_epoch=proc.create_time(),parent_pid=proc.ppid(),command=proc.cmdline()),
        source_Syn45_VG_byte_identical=True,CST_and_actual_options_device_same_routed35=True,
        actual_single_SDC_same_actual26_50_100MHz_and_reports=True,
        constraints52_actual_final_effectiveness_unverified=True,
        new_global_clock_requests_or_setup_exceptions=0,total_limit_s=0,silent_limit_s=0,
        actual_resource_and_setup_hold_results_pending=True,
        PNR56_independent_controller=queue,PNR56_tool_not_started=True,
        PNR56_has_no_PNR55_exit_dependency=True,
        PNR56_admission_includes_remaining_PNR55_peak_growth=True,
        fallback_fusion_plan_only=str(BASE/'SYN45_fusion_gyro600_combination_plan2/result.json'),
        fallback_RTL_not_materialized=True,programmed=False,deployment_accepted=False,
        bindings=[metadata(p) for p in (source/'job.json',root_review,build/'g2_top.vg',
            build/'g2.sdc',build/'g2.cst',build/'impl/pnr/cmd.do',build/'impl/pnr/device.cfg')])
    receipt = ROOT/'docs/4_design/evidence/g2_20261023/PNR55_actual_start_PNR56_memory_queue_20261027.json'
    assert not receipt.exists()
    save(receipt,evidence)
    section = '''**2026-10-27 00:49 PNR55实际开始，56按真实余量排队。** SYN45于00:27:44自然结束0，FF29535/LUT39011/ALU9606、源BSRAM269/SSRAM1885、各DSP类别保持；50MHz综合估计48.744（非物理通过），首gyro_y→fusion/state20.829ns/slack−0.515。新NAV/PID8896项实际查询全部UNSAT：6个真实脉冲FF的INIT0/50MHz/复位/受理D和8815消费者转移过、867控制脚移除旧地址依赖；PID目标D的IR/key依赖移除且立即停止保持。动作六DSP仅OREG为50MHz/CE1/reset0，源为真实core24位动作，每组clip/gain实际输入锥共12UNSAT。最后18个观测高位按真实INIT/复位/CE及完整next-state的72项归纳闭合，不靠任意FF相等放行；ring1038/全部存储CPU/两钟26/FIFO/IR182/lease248补证过。最初切口/类型/非捕获D反例保留，没有修改硬件或重综。

00:44:39工具30740（创建1793033079.044567，监管68008）实际进入PNR55布局；入场物理3669053440B/提交7085187072B，总0/静默0无截止。实际VG同45，CST/cmd.do/device同布通35，单SDC保留20/10ns、真实新26组hold-only与8/18ns max-delay及原报告命令，未增加时序例外/全局请求。源269+11预计280，不当新物理结果。PNR56独立控制39296已运行，仅等2/4GiB加在途55剩余峰值增长的真实余量，没有55退出依赖；尚无56实际工具。两组冻结输入不动。融合600阈值的精确组合备用仅Python/数学计划，未生成RTL或启动另一综合；只在实际同锥失败后选择。[实际启动与独立队列](../4_design/evidence/g2_20261023/PNR55_actual_start_PNR56_memory_queue_20261027.json)。

'''
    table = ('| G2共同资源 | 原两布局已布通280RAM/11拆分、CLS61%；45源269RAM | '
        '45存储/CPU/两钟26/停止和NAV/PID/动作实际映射通过 | '
        '新55实际布局中，未有物理时序；原54仍setup2709/hold0 | '
        '55无截止运行；56只按真实内存排队；未部署 |')
    plan = ROOT/'docs/1_plan/system_integration_plan_20261021.md'
    def amend(text):
        assert '00:49 PNR55实际开始' not in text
        text,n = re.subn(r'^\| G2共同资源 \|.*$',table,text,flags=re.M)
        assert n == 1
        anchor='## 2. 先完成G2：真实共同工程资源\n\n'
        assert text.count(anchor) == 1
        return text.replace(anchor,anchor+section,1)
    plan.write_text(amend(plan.read_text(encoding='utf8')),encoding='utf8',newline='\n')
    rel = plan.relative_to(ROOT).as_posix()
    original = subprocess.check_output(['git','show',':'+rel],cwd=ROOT).decode('utf8')
    patch=''.join(difflib.unified_diff(original.splitlines(True),amend(original).splitlines(True),fromfile='a/'+rel,tofile='b/'+rel))
    subprocess.run(['git','apply','--cached','-'],cwd=ROOT,input=patch.encode('utf8'),check=True)
    special=ROOT/'docs/4_design/g2_joint_resources_20261022.md'
    special.write_text(special.read_text(encoding='utf8').rstrip()+'\n\n## 2026-10-27 00:49 实际PNR55\n\n'+section.rstrip()+'\n',encoding='utf8',newline='\n')
    machine = BASE/'lane2_ddr_map_evaluation/job.json'
    state=load(machine)
    state.update(phase='PNR55_actual_active_PNR56_actual_memory_queue',current_active_SYN=None,
        current_active_PNR=dict(build=str(build),**evidence['tool']),queued_PNR=queue,
        latest_root_timing_checkpoint=str(receipt),latest_known_commit='9eeff1f',
        next_step='Read original55/56 runtime and real final reports; no deadlines or duplicate runs; analyze actual routed paths before adopting fallback')
    save(machine,state)
    print(json.dumps(dict(actual_PNR55=proc.pid,PNR56_controller=qproc.pid,receipt=str(receipt))))


if __name__ == '__main__':
    main()
