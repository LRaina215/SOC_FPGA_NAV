"""Create an isolated PNR85-source candidate. Does not run SYN/PNR or program a board."""
import argparse
import json
from pathlib import Path
import re
import shutil

HERE=Path(__file__).resolve().parent
DEFAULT=Path('F:/nav_integration_g2_20261023/K_terrain19_follow1')

def replace_once(text,old,new):
    assert text.count(old)==1,(old[:100],text.count(old))
    return text.replace(old,new,1)

def prepare(source,out):
    out.mkdir(parents=True,exist_ok=False)
    core=(source/'control4500_core.v').read_text(encoding='utf8')
    core=replace_once(core,'    input wire range_distance_valid,',
        '    input wire range_distance_valid,\n'
        '    input wire range_sample_strobe,range_forward_ready,\n'
        '    output wire range_hold_forward,range_protection_ready,')
    core=replace_once(core,'wire signed [23:0] command_forward_q12=',
                            'wire signed [23:0] requested_forward_q12=')
    core=replace_once(core,'wire signed [23:0] command_yaw_q12=',
                            'wire signed [23:0] requested_yaw_q12=')
    marker='    wire signed [23:0] command_height_q12='
    assert core.count(marker)==1
    start=core.index(marker)
    core=core[:start]+'''    wire signed [23:0] command_forward_q12,command_yaw_q12;
    ultrasonic_drive_guard u_ultrasonic_guard(
        .clk(clk),.reset_n(device_reset_n),
        .remote_key(ir_key_code),.remote_key_event(IR_ENABLE && ir_key_event),
        .remote_key_active(ir_key_active),.manual_owner(ir_drive),
        .hard_stop(command_emergency_stop),
        .requested_forward_q12(requested_forward_q12),.requested_yaw_q12(requested_yaw_q12),
        .sample_strobe(range_sample_strobe),.sample_valid(range_distance_valid),
        .forward_ready(range_forward_ready),.distance_mm(range_distance_mm),
        .enabled(range_hold_forward),.forward_q12(command_forward_q12),.yaw_q12(command_yaw_q12),
        .sample_healthy(range_protection_ready),.braking(),.stop_latched(),.sample_age_ms(),.range_target_q12());
'''+core[start:]
    top=(source/'g2_top.v').read_text(encoding='utf8')
    top=replace_once(top,'wire range_distance_valid,range_timeout;',
        'wire range_distance_valid,range_timeout;\n'
        '    wire range_hold_forward,range_sample_strobe,range_forward_ready,range_protection_ready;')
    top=replace_once(top,'.reset_n(~rst),.echo(HCSR04_ECHO),',
        '.reset_n(~rst),.echo(HCSR04_ECHO),.hold_forward(range_hold_forward),\n'
        '        .sample_strobe(range_sample_strobe),.forward_ready(range_forward_ready),')
    top=replace_once(top,'.range_distance_mm(range_distance_mm),.range_distance_valid(range_distance_valid),',
        '.range_distance_mm(range_distance_mm),.range_distance_valid(range_distance_valid),\n'
        '        .range_sample_strobe(range_sample_strobe),.range_forward_ready(range_forward_ready),\n'
        '        .range_hold_forward(range_hold_forward),.range_protection_ready(range_protection_ready),')
    top=replace_once(top,'    nav_soc #(', '    wire [3:0] soc_led;\n    nav_soc #(')
    top=replace_once(top,'.led(led_cpu), .trap(led_trap)', '.led(soc_led), .trap(led_trap)')
    top=replace_once(top,'    assign led = {ch_wdog, ch_stop, irq, hb[25]};',
        '    assign led = {ch_wdog, ch_stop, irq, hb[25]};\n'
        '    ultrasonic_mode_led u_sonar_led(.clk(clk),.reset_n(~rst),.enabled(range_hold_forward),\n'
        '        .sample_healthy(range_protection_ready),\n'
        '        .cpu_leds(soc_led),.board_cpu_leds(led_cpu));')
    (out/'control4500_core.v').write_text(core,encoding='utf8')
    (out/'g2_top.v').write_text(top,encoding='utf8')
    for name in ('g2_range_servo.v','ultrasonic_drive_guard.v','ultrasonic_mode_led.v'):
        shutil.copyfile(HERE/'rtl'/name,out/name)
    old_files=re.findall(r'add_file -type verilog \{([^}]+)\}',(source/'run.tcl').read_text())
    files=[]
    for old in old_files:
        p=Path(old)
        new=out/p.name
        files.append(str(new if p.name in ('control4500_core.v','g2_top.v','g2_range_servo.v') else p))
    files.append(str(out/'ultrasonic_drive_guard.v'))
    files.append(str(out/'ultrasonic_mode_led.v'))
    manifest=dict(source=str(source),candidate=str(out),verilog_files=files,
        original_ir_decoder=str(source/'control/ir_remote_control.v'),
        changed=['control4500_core.v','g2_top.v','video/g2_range_servo.v'],
        added=['ultrasonic_drive_guard.v','ultrasonic_mode_led.v'],simulation_only=True,
        unchanged='All other RTL, firmware, model weights, IO constraints and clocks',
        build_started=False,board_programmed=False)
    (out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    return manifest

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,default=DEFAULT)
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();print(json.dumps(prepare(a.source,a.out),ensure_ascii=False,indent=2))
