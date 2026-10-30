"""Freeze the accepted video160/PID candidate without editing the shared builder.

Reads reviewed SYN50 inputs, preserves model/CPU/map assets, adds only the
selected video/ranging peripherals. Does not invoke EDA, program or publish.
"""
import argparse,json,re,shutil
from pathlib import Path
from datetime import datetime

ROOT=Path(__file__).resolve().parents[2]
FEATURE=Path(__file__).with_name('video_20261030')
BASE=Path('F:/nav_integration_g2_20261023/K_timing_syn50')
DEFAULT=Path('F:/nav_integration_g2_20261023/K_video160_20261030_prepared1')

def save(p,value):
    p.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8',newline='\n')

def replace_once(s,old,new):
    assert s.count(old)==1,(old[:100],s.count(old))
    return s.replace(old,new)

def prepare(out):
    out=out.resolve();base=BASE.resolve()
    assert not out.exists() and out!=base and base not in out.parents
    tcl=(base/'run.tcl').read_text(encoding='utf-8-sig')
    assert 'run syn' in tcl and 'run pnr' not in tcl
    assert 'set_device -device_version C GW5AT-LV138PG484AC1/I0' in tcl
    original_files=re.findall(r'add_file -type (\w+) \{([^}]+)\}',tcl)
    assert all(Path(p).is_file() for _,p in original_files)
    out.mkdir(parents=True)
    copied=[];assets=[]
    allowed={'.v','.vh','.mem','.hex','.c','.h','.S','.ld','.bin','.elf'}
    excluded={'impl','configuration_source','host_profile','sim_lease','src'}
    for p in sorted(base.rglob('*')):
        rel=p.relative_to(base)
        if not p.is_file() or rel.parts[0] in excluded:continue
        if p.suffix not in allowed and rel.as_posix() not in ('run.tcl','g2.cst','g2.sdc'):continue
        q=out/rel;q.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,q)
        copied.append(rel.as_posix())
        if p.suffix in {'.mem','.hex','.bin','.elf'}:
            assert p.read_bytes()==q.read_bytes();assets.append(rel.as_posix())
        elif p.suffix in {'.v','.vh'}:
            text=q.read_text(encoding='utf-8-sig').replace(base.as_posix(),out.as_posix())
            q.write_text(text,encoding='utf-8',newline='\n')
    assert all((out/Path(p).relative_to(base)).is_file() for _,p in original_files)
    video=out/'video';shutil.copytree(FEATURE/'rtl',video)
    for n in ('oled.v','motor520_closed_loop.v'):
        shutil.copyfile(FEATURE/'reference'/n,out/'control'/n)
    core=(out/'control4500_core.v').read_text(encoding='utf-8')
    core=replace_once(core,"parameter OLED_MODEL_DEBUG=1'b1","parameter OLED_MODEL_DEBUG=1'b0")
    core=replace_once(core,'    output wire signed [23:0] attitude_wx,attitude_wy,attitude_wz\n',
        '    output wire signed [23:0] attitude_wx,attitude_wy,attitude_wz,\n'
        '    input wire [15:0] range_distance_mm,\n    input wire range_distance_valid,\n'
        '    output wire [95:0] video_imu_values,\n'
        '    output wire video_imu_valid,video_imu_ready,video_imu_error\n')
    core=replace_once(core,'.SIX_AXIS_MODE(1),.WHEEL_HEADER_MODE(1),',
        '.SIX_AXIS_MODE(1),.RANGE_HEADER_MODE(1),.WHEEL_HEADER_MODE(1),')
    core=replace_once(core,'.IIC_SCL(sck),.IIC_SDA(sda),.ready(),.error());',
        '.DISTANCE_MM(range_distance_mm),.DISTANCE_VALID(range_distance_valid),\n'
        '        .IIC_SCL(sck),.IIC_SDA(sda),.ready(),.error());')
    core=replace_once(core,'endmodule',
        '    assign video_imu_values={gyro_z,gyro_y,gyro_x,accel_z,accel_y,accel_x};\n'
        '    assign video_imu_valid=sample_valid;\n'
        '    assign video_imu_ready=sensor_ready;\n'
        '    assign video_imu_error=sensor_error;\nendmodule')
    (out/'control4500_core.v').write_text(core,encoding='utf-8',newline='\n')
    top=(out/'g2_top.v').read_text(encoding='utf-8')
    top=replace_once(top,'    output wire SERVO1,SERVO2,SERVO3,SERVO4,\n',
        '    output wire SERVO1,SERVO2,SERVO3,SERVO4,\n'
        '    input wire [7:0] CAMERA_D,\n'
        '    input wire CAMERA_PCLK,CAMERA_HREF,CAMERA_VSYNC,\n'
        '    output wire CAMERA_RESET_N,CAMERA_SCLK,inout wire CAMERA_SDA,\n'
        '    output wire [4:0] LCD_R,LCD_B,output wire [5:0] LCD_G,\n'
        '    output wire LCD_PCLK,LCD_HSYNC,LCD_VSYNC,LCD_DE,LCD_BL,\n'
        '    input wire HCSR04_ECHO,output wire HCSR04_TRIG,SCAN_SERVO_PWM,\n')
    video_top='''    wire [15:0] range_distance_mm;
    wire range_distance_valid,range_timeout;
    wire [1:0] scan_position;
    wire [95:0] video_imu_values;
    wire video_imu_valid,video_imu_ready,video_imu_error;
    wire video_clock,video_locked,camera_ready,camera_error,display_ready;
    g2_range_servo u_range(.clk(clk),.reset_n(~rst),.echo(HCSR04_ECHO),
        .trig(HCSR04_TRIG),.servo_pwm(SCAN_SERVO_PWM),.distance_mm(range_distance_mm),
        .distance_valid(range_distance_valid),.distance_timeout(range_timeout),.servo_position(scan_position));
    G2Video_PLL u_video_pll(.clkin(clk),.init_clk(clk),.reset(rst),
        .clkout0(),.clkout1(video_clock),.lock(video_locked));
    g2_camera_lcd u_video(.clk(clk),.reset_n(~rst && video_locked),.lcd_clk(video_clock),
        .imu_values(video_imu_values),.imu_sample_valid(video_imu_valid),
        .imu_ready(video_imu_ready),.imu_error(video_imu_error),
        .distance_mm(range_distance_mm),.distance_valid(range_distance_valid),
        .CAMERA_D(CAMERA_D),.CAMERA_PCLK(CAMERA_PCLK),.CAMERA_HREF(CAMERA_HREF),.CAMERA_VSYNC(CAMERA_VSYNC),
        .CAMERA_SCLK(CAMERA_SCLK),.CAMERA_SDA(CAMERA_SDA),.CAMERA_RESET_N(CAMERA_RESET_N),
        .LCD_R(LCD_R),.LCD_G(LCD_G),.LCD_B(LCD_B),.LCD_PCLK(LCD_PCLK),
        .LCD_HSYNC(LCD_HSYNC),.LCD_VSYNC(LCD_VSYNC),.LCD_DE(LCD_DE),.LCD_BL(LCD_BL),
        .camera_ready(camera_ready),.camera_error(camera_error),.display_ready(display_ready));
'''
    top=replace_once(top,'    wire att_valid, att_txd;\n',video_top+'    wire att_valid, att_txd;\n')
    top=replace_once(top,'.attitude_wx(att_wx),.attitude_wy(att_wy),.attitude_wz(att_wz));',
        '.attitude_wx(att_wx),.attitude_wy(att_wy),.attitude_wz(att_wz),\n'
        '        .range_distance_mm(range_distance_mm),.range_distance_valid(range_distance_valid),\n'
        '        .video_imu_values(video_imu_values),.video_imu_valid(video_imu_valid),\n'
        '        .video_imu_ready(video_imu_ready),.video_imu_error(video_imu_error));')
    (out/'g2_top.v').write_text(top,encoding='utf-8',newline='\n')
    cst=(out/'g2.cst').read_text(encoding='utf-8-sig')
    added=(FEATURE/'video_io.cst').read_text(encoding='utf-8')
    old_locs=dict(re.findall(r'IO_LOC\s+"([^"]+)"\s+([^;\s]+)\s*;',cst))
    new_locs=dict(re.findall(r'IO_LOC\s+"([^"]+)"\s+([^;\s]+)\s*;',added))
    assert len(new_locs)==38
    assert not(set(old_locs.values())&set(new_locs.values()))
    (out/'g2.cst').write_text(cst+'\n'+added,encoding='utf-8',newline='\n')
    sdc=(out/'g2.sdc').read_text(encoding='utf-8-sig')
    assert not any(x in sdc for x in ('CAMERA_PCLK','u_video_pll'))
    sdc+='\n# Video input clock;33MHz LCD clock is derived by the reviewed PLL.\n'
    sdc+='create_clock -name camera_pclk -period 20 [get_ports {CAMERA_PCLK}]\n'
    sdc+='create_generated_clock -name lcd33 -source [get_ports {clk50}] -multiply_by 33 -divide_by 50 [get_pins {u_video_pll/u_pll/PLL_inst/CLKOUT1}]\n'
    sdc+='# No imported video false paths. Review actual new synchronizers/payload after SYN.\n'
    (out/'g2.sdc').write_text(sdc,encoding='utf-8',newline='\n')
    tcl=tcl.replace(base.as_posix(),out.as_posix())
    extras=''.join('add_file -type verilog {'+p.as_posix()+'}\n' for p in sorted(video.glob('*.v')))
    tcl=replace_once(tcl,'add_file -type cst',extras+'add_file -type cst')
    (out/'run.tcl').write_text(tcl,encoding='utf-8',newline='\n')
    manifest=dict(prepared_at=datetime.now().astimezone().isoformat(),source=str(base),output=str(out),
        source_commit='Chenyudog/fpga@715378a',original_verilog_count=sum(k=='verilog' for k,_ in original_files),
        added_verilog=[p.name for p in sorted(video.glob('*.v'))],copied_files=copied,
        original_asset_byte_comparisons_passed=assets,original_CPU_bytes=131072,
        changed_original_RTL=['g2_top.v','control4500_core.v','control/oled.v','control/motor520_closed_loop.v'],
        actual_joint_BSRAM=None,predicted_image_BSRAM=38,baseline_plus_image_prediction=318,
        image_pixels=19200,image_format='RGB565',pixel_RAM_reset=False,telemetry_version=2,
        compact_HUD_payload_bits=144,upstream_character_storage_bits=4032,
        new_IO_locations=new_locs,new_IO_collisions=0,
        model_weights_changed=False,CPU_firmware_changed=False,SLF128_and_capacities_preserved=True,
        no_SFP_or_DDR_added=True,video_CDC_and_IO_budget_review_pending=True,
        unit_simulation_pending=True,automatic_PNR=False,programmed=False,deployment_accepted=False)
    save(out/'video_candidate_manifest.json',manifest)
    return manifest

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,default=DEFAULT);a=ap.parse_args()
    m=prepare(a.out)
    print(json.dumps({k:m[k] for k in ('output','original_verilog_count','added_verilog','new_IO_collisions','actual_joint_BSRAM','automatic_PNR')},ensure_ascii=False))
