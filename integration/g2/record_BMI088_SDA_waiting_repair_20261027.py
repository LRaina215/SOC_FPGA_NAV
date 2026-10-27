"""Record the diagnosis boundary and human request to await onsite repair.

No hardware access, test, rebuild, reset, or programming is performed here.
"""
from datetime import datetime
from pathlib import Path
import difflib
import json
import re
import subprocess

from prepare_baseline_fusion_retime import metadata, load, save

ROOT=Path(__file__).resolve().parents[2]
BASE=Path('F:/nav_integration_g2_20261023')
SRC=BASE/'K_timing_syn49'
BUILD=BASE/'K_pnr61_s49p2r0'


def main():
    iic=SRC/'control/oled_driver.v'
    bmi=SRC/'control/bmi088.v'
    core=SRC/'control4500_core.v'
    t=iic.read_text(encoding='utf8')
    for anchor in ('parameter BUS_CLEAR = 0',
        "error_code<=!scl_sync_1 ? 4'd4 : 4'd5;",
        'wire line_blocked = high_wait && (!scl_sync_1 || (state==WAIT_IDLE && !sda_sync_1));',
        "scl_low<=1'b0; sda_low<=1'b0; state<=IDLE;"):
        assert anchor in t,anchor
    assert 'parameter BUS_CLEAR = 0' in bmi.read_text(encoding='utf8')
    assert 'bmi088 sensor(' in core.read_text(encoding='utf8')
    evidence=dict(recorded_at=datetime.now().astimezone().isoformat(),
        human_power_source='车上的电源板',
        human_steering='如果你实在检测不到那就算了，等我去修',
        requested_disposition='Await user onsite IMU repair; no further active hardware diagnosis in this turn',
        latest_passive_capture=dict(started_at='2026-10-27T08:36:52.257079+08:00',
            duration_requested_s=4,frames=403,bad_crc=0,
            sensor_error_counts={'5':403},status_counts={'10':403},active_counts={'False':403},
            first_sequence=13192,last_sequence=13594,completed_frames=0),
        source_verified_error5='WAIT_IDLE, sampled SCL high and sampled SDA low for the full timeout, before START/address transfer',
        timeout_cycles=50000,core_clock_Hz=50000000,timeout_ms=1,
        sensor_pins=dict(SDA='P11-7/F18',SCL='P11-8/E18'),
        current_and_previous_working_pinout_same=True,
        physical_constraints=dict(IO_TYPE='LVCMOS33',PULL_MODE='NONE',external_pullups_required_by_design=True),
        driver_retry=dict(retry_wait_cycles=5000000,startup_wait_cycles=1500000,
            controller_and_pins_released_on_retry=True,BUS_CLEAR=0,
            nine_clock_recovery_not_enabled_in_current_source=True),
        no_lidar_gate_in_sensor_reset='device_reset_n depends on physical reset/startup, not lidar link; bmi088 uses device_reset_n directly',
        exact_electrical_root_cause_known=False,
        remaining_possible_causes=['Local IMU VCC/VDDIO or external pullup supply absent',
            'Ground/signal connection or short', 'Target remains in unfinished I2C transfer',
            'Physical line/readback problem not yet measured'],
        cannot_infer=['That only lidar power is missing proves IMU supply voltage',
            'Error5 identifies which device is pulling SDA low',
            'Zero six-axis fields represent valid stationary measurements'],
        authoritative_bus_clear_reference='https://www.nxp.com/docs/en/user-guide/UM10204.pdf',
        sensor_reference='https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf',
        current_PNR61_preserved=True,programmed=False,Flash_written=False,motion_commands=0,
        resets_or_bus_clear_performed=False,new_EDA_started=False,new_tests_by_this_recorder=False,
        metadata_at_read=[metadata(p)for p in(iic,bmi,core,BUILD/'g2.cst')])
    receipt=ROOT/'docs/4_design/evidence/g2_20261023/BMI088_SDA_fault_waiting_onsite_repair_20261027.json'
    assert not receipt.exists()
    save(receipt,evidence)
    section='''**2026-10-27 08:40 BMI088已定位到I2C空闲SDA超时，按用户要求等现场处理。** 用户确认模块VCC/GND来自车上电源板，但尚无模块端电压测量；“只缺雷达供电”不能据此直接证明IMU电源/上拉正常。08:36:52起唯一4秒被动采集403帧、CRC0，全部status10/error5/active=false，完成推理仍0。源代码确证error5为WAIT_IDLE阶段SCL采样高、SDA采样低持续1ms，发生在START/地址/ID传输之前；不是导航解析遗漏六轴数据。当前与先前成功引脚均SDA=P11-7/F18、SCL=P11-8/E18；现CST无片内上拉，依赖外部上拉。BMI复位不依赖雷达链路。驱动有错误后重试，但BUS_CLEAR=0，未启用九时钟总线解锁；本轮没有执行复位、解锁或改位流。尚不能识别实际拉低来源，供电支路/上拉/接线/器件未结束事务仍需现场区分。

用户回复“等我去修”，据此停止继续硬件诊断和动作测试，保留PNR61 SRAM环境及全部记录。等待现场处理后，再由新鲜六轴、有效标定、轴向和反馈接续，未把IMU问题记为已解决，也未暂停其它已授权目标。[诊断边界与用户安排](../4_design/evidence/g2_20261023/BMI088_SDA_fault_waiting_onsite_repair_20261027.json)。

'''
    plan=ROOT/'docs/1_plan/system_integration_plan_20261021.md'
    def amend(text):
        assert '08:40 BMI088已定位' not in text
        row='| G2共同资源 | PNR61已布通280RAM，FF32017/CLS42995 | 50/100MHz报告setup/hold均0；61约束Actived | G3候选SRAM记录/只读CPU遥测正常 | BMI SDA空闲超时，用户现场待修；用户称雷达未供电；未有实车闭环/停稳通过 |'
        text,n=re.subn(r'^\| G2共同资源 \|.*$',row,text,flags=re.M)
        assert n==1
        marker='## 2. 先完成G2：真实共同工程资源\n\n'
        assert text.count(marker)==1
        return text.replace(marker,marker+section,1)
    plan.write_text(amend(plan.read_text(encoding='utf8')),encoding='utf8',newline='\n')
    rel=plan.relative_to(ROOT).as_posix()
    old=subprocess.check_output(['git','show',':'+rel],cwd=ROOT).decode('utf8')
    diff=''.join(difflib.unified_diff(old.splitlines(True),amend(old).splitlines(True),fromfile='a/'+rel,tofile='b/'+rel))
    subprocess.run(['git','apply','--cached','-'],cwd=ROOT,input=diff.encode('utf8'),check=True)
    bench=ROOT/'integration/g2/k_bench_20261027/README.md'
    note='''## 08:40 IMU诊断结论及等待现场处理

用户确认BMI088取电自车上电源板，并安排稍后现场处理。4秒被动遥测403帧/CRC0，全部error5、未激活。已核源/实际引脚：错误发生在START前的空闲SDA低超时，不是ID不符或导航解析问题；SDA=P11-7/F18、SCL=P11-8/E18，与之前成功接线一致。电源电压、外部上拉和拉低来源尚未量测，当前驱动BUS_CLEAR=0，重试没有九时钟解锁。本轮没有复位、解锁、动作或重新下载，保留PNR61候选等待现场修复。

'''
    bt=bench.read_text(encoding='utf8')
    assert '## 08:40 IMU诊断结论' not in bt
    bt=bt.replace('## 08:27 跟进现场（只读）\n',note+'## 08:27 跟进现场（只读）\n',1)
    bt=bt.replace('判断是车上传感器和执行器没有供电，需要整车上电后补测。',
                  '当时怀疑供电未接通，未测得电压；08:40用户确认IMU来自车上电源板，真实故障原因仍待现场检查。')
    bench.write_text(bt,encoding='utf8',newline='\n')
    machine=BASE/'lane2_ddr_map_evaluation/job.json'
    state=load(machine)
    state.update(updated_at=evidence['recorded_at'],
        phase='PNR61_timing_report_closed_IMU_waiting_user_onsite_repair',
        latest_BMI_diagnosis=str(receipt),latest_passive_IMU_status=evidence['latest_passive_capture'],
        user_IMU_power_source='vehicle power board, module voltage not measured',
        further_hardware_diagnosis_awaits_user_repair=True,
        next_step='Await onsite IMU supply/pullup/wiring or bus repair; retain PNR61; afterwards read fresh valid sensor/axis/feedback before supported motion tests')
    save(machine,state)
    print(json.dumps(dict(receipt=str(receipt),passive403_frames_error5=True)))


if __name__=='__main__':
    main()
