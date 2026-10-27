"""Record completed non-IMU work and the actual second-place run."""
from datetime import datetime
import difflib
import json
from pathlib import Path
import re
import subprocess

ROOT=Path(__file__).resolve().parents[2]
BASE=Path('F:/nav_integration_g2_20261023')


def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def save(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf8')


def main():
    now=datetime.now().astimezone();stamp=now.strftime('%Y-%m-%d %H:%M')
    out=BASE/'PNR61_independent_signoff_20261027'
    audit=load(out/'summary.json');net=load(BASE/'PNR61_MAC_UDP_XT_pipeline_probe2/result.json')
    stack=load(BASE/'PNR61_exact_firmware_stack_review1/result.json')
    live=load(out/'idle_host_loss_stdout.json');query=load(out/'live_runtime_SLF_query.json')
    job=load(BASE/'PNR61_second_place_one_shot/job.json');runtime=load(BASE/'K_pnr62_s49p1r0/tool_runtime.json')
    assert audit['memories']['passed'] and net['passed'] and stack['passed'] and live['passed']
    assert job['phase']=='PNR_running' and not runtime['cancelled']
    assert runtime['max_runtime_s']==runtime['max_no_observable_progress_s']==0
    pnr=BASE/'K_pnr62_s49p1r0';ref=BASE/'K_pnr61_s49p2r0'
    cmd=(pnr/'impl/pnr/cmd.do').read_text().replace(str(pnr),str(ref)).replace('-place_option 1','-place_option 2')
    assert cmd==(ref/'impl/pnr/cmd.do').read_text()
    assert (pnr/'impl/pnr/device.cfg').read_bytes()==(ref/'impl/pnr/device.cfg').read_bytes()
    receipt=ROOT/'docs/4_design/evidence/g2_20261023/PNR61_independent_signoff_and_tests_20261027.json'
    assert not receipt.exists()
    evidence=dict(recorded_at=now.isoformat(),source='K_timing_syn49',PNR='K_pnr61_s49p2r0',
        memory_clock_IO_CDC_audit=audit,exact_firmware_stack=stack,
        idle_only_live_host_test=live,live_runtime_query=query,network_pipeline_test=net,
        first_network_probe_failed_due_to_unconnected_new_public_read_admit_input=True,
        first_network_failure_retained=str(BASE/'PNR61_MAC_UDP_XT_pipeline_probe1/xsim.log'),
        first_live_time_field_note='started_at_local in captured stdout is actually emitted at completion; all latency assertions use relative monotonic timestamps',
        PNR62=dict(job=job,runtime_at_read=runtime,actual_cmd_only_path_place_change=True,actual_device_cfg_equal=True,
            outcome_not_known=True),no_IMU_hardware_diagnosis_repeated=True,no_motion_targets_sent=True,
        no_CPU_hold_reset_or_reprogramming=True,no_Flash=True,full_signoff_or_vehicle_acceptance=False)
    save(receipt,evidence)
    plan=ROOT/'docs/1_plan/system_integration_plan_20261021.md'
    section=f'''**{stamp} IMU以外的签核和测试已推进，PNR62同网表第二布局已实际启动。** 独立核对PNR61全部280RAM/11宽拆分/1885分布式INIT、CPU128KiB/54164B；32个TX-shadow复制FF按全部输入/INIT归纳闭合。34431个实际时钟脚仅50MHz和雷达根，26条直接跨域均两级且首级唯一负载，51个IO位置匹配。报告setup/hold0，最差+0.045/+0.163ns；恢复/移除/clock MPW已列最差+3.813/+0.258/+2.848ns，仍非完整外部IO/复位/MTBF签核。

仅IDLE/零no-op的实板主机测试22包/16状态，末包后约287ms观察超时、新心跳不解锁、显式IDLE恢复，最终锁定零目标；不代运动中停稳或繁忙规划。只查询确认前移1000/ZHI350/profile534C0108。两份实际BIN均逐字节重建并严格核栈2576/6144，替代原指旧ELF的复制JSON。新MAC流水经实际4帧/38400像素/70包及两次复位通过；首旧夹具漏新read_admit而0像素的反例保留。通信按最终会话分工：CH收发命令/状态，P11-40输出遥测，P11-39不启用不列功能缺口。

09:03:55工具4328（创建1793063035.664706）、原监管38488实际启动K_pnr62_s49p1r0，仅place2→1、route0；VG/CST/SDC/device和其余真实cmd一致，总0/静默0无截止，入场3380453376B物理/10890719232B提交。没有重综或改变板上镜像。结束后先审完整实际结果，再决定后续。IMU待修只阻塞相关实车项目，其余缺口见[签核与整车测试清单](../4_design/g2_PNR61_signoff_20261027.md)和[机器证据](../4_design/evidence/g2_20261023/PNR61_independent_signoff_and_tests_20261027.json)。

'''
    def amend(text):
        assert 'IMU以外的签核和测试已推进' not in text
        row='| G2共同资源 | PNR61布通280RAM/FF32017/CLS42995；同VG第二布局PNR62在途 | 50/100MHz报告setup/hold0；存储/两钟/26CDC结构/51引脚独立核过 | 空闲主机失联锁存及当前MAC→UDP→XT有限通过；固件严格栈闭合 | IMU待修；外部IO/CDC延迟/复位/硬停待签核；CH命令/P11-40遥测分工保持；完整实车未通过 |'
        text,n=re.subn(r'^\| G2共同资源 \|.*$',row,text,flags=re.M);assert n==1
        marker='## 2. 先完成G2：真实共同工程资源\n\n';assert marker in text
        return text.replace(marker,marker+section,1)
    plan.write_text(amend(plan.read_text(encoding='utf8')),encoding='utf8',newline='\n')
    rel=plan.relative_to(ROOT).as_posix()
    old=subprocess.check_output(['git','show',':'+rel],cwd=ROOT).decode('utf8')
    diff=''.join(difflib.unified_diff(old.splitlines(True),amend(old).splitlines(True),fromfile='a/'+rel,tofile='b/'+rel))
    subprocess.run(['git','apply','--cached','-'],cwd=ROOT,input=diff.encode('utf8'),check=True)
    machine=BASE/'lane2_ddr_map_evaluation/job.json';s=load(machine)
    s.update(updated_at=now.isoformat(),phase='PNR61_nonIMU_signoff_tests_complete_PNR62_second_place_running',
        current_active_PNR=dict(directory=str(pnr),tool_pid=4328,tool_created_epoch=1793063035.664706,
            controller_pid=38488,job=str(BASE/'PNR61_second_place_one_shot/job.json'),total_limit_s=0,silent_limit_s=0),
        latest_nonIMU_signoff_evidence=str(receipt),further_IMU_diagnosis_awaits_user_repair=True,
        further_hardware_diagnosis_awaits_user_repair=False,
        next_step='Read actual PNR62 result without deadline; review external IO/CDC/reset under selected CH command and P11-40 telemetry split; retain current PNR61 SRAM and await user IMU repair only for dependent vehicle tests')
    save(machine,s)
    print(json.dumps(dict(receipt=str(receipt),PNR62_running=True)))


if __name__=='__main__':main()
