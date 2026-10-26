"""Record actual tool identity and completed source proofs; no new EDA."""
from datetime import datetime
from pathlib import Path
import difflib
import json
import re
import subprocess

import psutil

from prepare_baseline_fusion_retime import metadata, save

ROOT = Path(__file__).resolve().parents[2]
BASE = Path('F:/nav_integration_g2_20261023')


def load(p):
    return json.loads(p.read_text(encoding='utf-8-sig'))


def main():
    build = BASE/'SLF128_measured_timing_syn45'
    job = load(build/'job.json')
    assert job['phase'] == 'synthesizing'
    proc = psutil.Process(job['tool_pid'])
    assert abs(proc.create_time()-job['tool_created_epoch']) < .01
    assert Path(proc.cwd()).resolve() == build.resolve()
    assert proc.name().lower() == 'gw_sh.exe' and proc.cmdline()[-1] == 'run.tcl'
    preparation = BASE/'SLF128_measured_timing_SYN45_preparation1.json'
    sources = load(preparation)
    assert sources['passed'] and sources['all_assets_byte_identical']
    proof_paths = [BASE/n for n in (
        'NAV_predecode_source_relation_review2/result.json',
        'PID_unity_actual_ModelSim_review1/result.json',
        'SLF128_ring44_action_preview_overlay2/prepared_review.json',
        'SLF128_ring44_action_preview_overlay2/edge_relation_proof.json',
        'SYN44_ring_actual_final1/result.json')]
    assert all(load(p)['passed'] for p in proof_paths)
    receipt = ROOT/'docs/4_design/evidence/g2_20261023/Measured_timing_SYN45_actual_start_20261027.json'
    assert not receipt.exists()
    evidence = dict(recorded_at=datetime.now().astimezone().isoformat(),
        actual_SYN45_started=True, actual_runtime=job,
        actual_process=dict(pid=proc.pid, created_epoch=proc.create_time(),
            parent_pid=proc.ppid(), command=proc.cmdline(), cwd=proc.cwd()),
        source_preparation=str(preparation), modified_RTL_relative44=sources['changed_RTL_relative_SYN44'],
        unchanged_original_RTL=58, unchanged_assets=104, unchanged_FW_BIN_bytes=53572,
        original4500_all40704_weights_125Hz_and_CAPACITY_preserved=True,
        clocks_MHz=dict(core=50, RX=100), no_new_clock_or_timing_exception=True,
        actual44_ring_78_resource_difference_explained=True,
        selected_repairs=['stride32 ring addressing', 'original-edge NAV command predecode',
                          'reachable unity model wheel target bypass', 'private same-publication action product capture'],
        source_proofs_passed=True, actual_SYN45_mapping_resource_and_physical_timing_pending=True,
        next_PNR_entry=str(ROOT/'integration/g2/run_measured_timing_place_pair.py'),
        PNR55_PNR56_started=False, automatic_PNR=False,
        max_synthesis_minutes=40, original_runtime_memory_guard=True,
        physical_run_total_limit_s=0, physical_run_silent_limit_s=0,
        programmed=False, deployment_accepted=False,
        bindings=[metadata(preparation)]+[metadata(p) for p in proof_paths])
    save(receipt,evidence)
    section = '''**2026-10-27 00:24 修复版SYN45已实际启动。** 00:19:09工具60260（创建1793031549.3947003）、持久监管51704已开始唯一SLF128_measured_timing_syn45；NORMAL全20核，2/4GiB入场、40分钟与原运行内存保护。不是布线已启动。源相对44只改5RTL：NAV六条原受理拍预译码、core已证unity旁路和动作preview三文件；其余58RTL、104资产/53572B固件、原4500/全部40704权值/125Hz、SLF128/地图点容量、50/100MHz/CST/SDC及停止优先级保持。NAV源级10UNSAT/288四态与实际1449拍、PID真实99,999,993拍/44发布/三类故障/急停失能复位各八状态均通过。动作preview补19项任意144位发布→下拍消费关系UNSAT，保留原64位乘积/舍入/最新ff/输出拍；不把旧SYN28映射或旧替身调度测试当新实际DSP证明。

SYN44实际审阅已闭合1006项ring/写边界，额外bank12不可写且不能影响消费，78原语/4992bit为实际值，较原66增加768bit，不再沿用72预估。全部非ring几何INIT/CPU128KiB、两真钟/原26与IR/lease复位停止复核通过；396个core输入RAM写边界通过，不称整mapped core等价。45结束后分工审新NAV6FF/INIT及消费锥、PID/IR立即停止、六动作DSP OREG的真实core源和时钟边界、全RAM/CPU/两钟/26/停止，再按实际资源和路径启动55/56；两PNR目前未启动，入口已备，总0/静默0。当前不再扩P11/BMI或模型试验，也未部署。[实际启动与证明绑定](../4_design/evidence/g2_20261023/Measured_timing_SYN45_actual_start_20261027.json)。

'''
    table = ('| G2共同资源 | 两种布局均布通280RAM/11拆分、CLS61%/FF30714 | '
             '44存储/两钟/26/IR/lease与NAV/PID源级及组件通过 | '
             '54的50域setup2709/hold0，RX100域0/0；新物理时序仍待 | '
             'SYN45已00:19实际运行，结束审新映射后接55/56；未部署 |')
    path = ROOT/'docs/1_plan/system_integration_plan_20261021.md'
    def amend(text):
        assert '00:24 修复版SYN45' not in text
        text,n = re.subn(r'^\| G2共同资源 \|.*$',table,text,flags=re.M)
        assert n == 1
        anchor = '## 2. 先完成G2：真实共同工程资源\n\n'
        assert text.count(anchor) == 1
        return text.replace(anchor,anchor+section,1)
    path.write_text(amend(path.read_text(encoding='utf8')),encoding='utf8',newline='\n')
    rel = path.relative_to(ROOT).as_posix()
    old = subprocess.check_output(['git','show',':'+rel],cwd=ROOT).decode('utf8')
    patch = ''.join(difflib.unified_diff(old.splitlines(True),amend(old).splitlines(True),
        fromfile='a/'+rel,tofile='b/'+rel))
    subprocess.run(['git','apply','--cached','-'],cwd=ROOT,input=patch.encode('utf8'),check=True)
    special = ROOT/'docs/4_design/g2_joint_resources_20261022.md'
    special.write_text(special.read_text(encoding='utf8').rstrip()+'\n\n## 2026-10-27 00:24 实际SYN45\n\n'+section.rstrip()+'\n',encoding='utf8',newline='\n')
    machine = BASE/'lane2_ddr_map_evaluation/job.json'
    state = load(machine)
    state.update(phase='SYN45_measured_timing_repairs_active',current_active_SYN=evidence['actual_process'],
        actual_SYN45_build=str(build),current_active_PNR=None,queued_PNR=None,
        latest_completed_source_proofs=[str(p) for p in proof_paths],
        latest_root_timing_checkpoint=str(receipt),latest_known_commit='10fda05',
        latest_priority='Timing repair first; no new P11/model experiments',
        next_step='Read actual SYN45 mapping/resource/clock/CDC/stop reviews, then prepare and start unique unlimited PNR55/56')
    save(machine,state)
    print(json.dumps(dict(actual_tool=proc.pid,created_epoch=proc.create_time(),receipt=str(receipt))))


if __name__ == '__main__':
    main()
