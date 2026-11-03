"""Collect already-completed finite tests and plot the Echo-driven setpoint trace."""
import csv
from datetime import datetime
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
matplotlib.rcParams['svg.fonttype']='none'
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

HERE=Path(__file__).resolve().parent
BASE=Path('F:/nav_integration_g2_20261023')
FINAL=BASE/'ultrasonic_manual_guard_prepared4'

def main():
    out=HERE/'evidence';out.mkdir(exist_ok=True)
    profiles=[]
    sources=[('ultrasonic_manual_guard_sim1','guard'),
             ('ultrasonic_manual_guard_sim2','range'),
             ('ultrasonic_manual_guard_sim2','nec'),
             ('ultrasonic_manual_guard_sensor_sim1','sensor_guard'),
             ('ultrasonic_manual_guard_sim3','policy'),
             ('ultrasonic_manual_guard_led_sim2','led')]
    for folder,name in sources:
        result=json.loads((BASE/folder/'result.json').read_text(encoding='utf8'))
        row=next(p for p in result['profiles'] if p['name']==name)
        assert row['passed'],row
        profiles.append({k:row[k] for k in ('name','output','passed','marker')})
    core_folder=BASE/'ultrasonic_manual_guard_core_iverilog1'
    core=json.loads((core_folder/'result.json').read_text(encoding='utf8'))
    assert core['passed'] and not core['simulation_source_changes']
    profiles.append(dict(name='core',output=str(core_folder),passed=True,marker=core['marker']))
    reused=[]
    for folder,name in [('ultrasonic_manual_guard_prepared1','ultrasonic_drive_guard.v'),
                        ('ultrasonic_manual_guard_prepared2','g2_range_servo.v')]:
        assert (BASE/folder/name).read_bytes()==(FINAL/name).read_bytes()
        reused.append(dict(source=str(BASE/folder/name),final=str(FINAL/name),byte_identical=True))
    for name in ('ultrasonic_drive_guard.v','g2_range_servo.v','ultrasonic_mode_led.v'):
        assert (HERE/'rtl'/name).read_bytes()==(FINAL/name).read_bytes()
    manifest=json.loads((FINAL/'manifest.json').read_text(encoding='utf8'))
    assert (BASE/'ultrasonic_manual_guard_prepared3/control4500_core.v').read_bytes()==(FINAL/'control4500_core.v').read_bytes()
    reused.append(dict(source=str(BASE/'ultrasonic_manual_guard_prepared3/control4500_core.v'),
                       final=str(FINAL/'control4500_core.v'),byte_identical=True))
    state=dict(at=datetime.now().astimezone().isoformat(),passed=True,scope='finite_RTL_simulation',
               candidate=str(FINAL),source=manifest['source'],verilog_input_count=len(manifest['verilog_files']),
               tested_modules_reused_without_repeating_same_test=reused,profiles=profiles,
               board_programmed=False,joint_synthesis_started=False,PNR_started=False,
               LED=dict(name='LED7',ball='N17',signal='led_cpu[3]',off='disabled',
                        blink='enabled but settling/invalid/stale ranging',on='enabled and fresh valid ranging',
                        blink_hz=2,resettable_independent_counter=True,
                        review_change='Existing top hb has no RTL initial/reset; final LED uses its own resettable phase',
                        replaced='firmware LED counter high bit only; remaining indicators retained'),
               preserved_failures=[
                   dict(path=str(BASE/'ultrasonic_manual_guard_sim1'),reason='range/NEC unsized watchdog overflow in fixture; corrected to explicit SV ms literal'),
                   dict(path=str(BASE/'ultrasonic_manual_guard_sim2'),reason='legacy OLED late declarations rejected by XSim; earlier policy input monitor unreachable; fixed monitor and repeated policy test'),
                   dict(path=str(BASE/'ultrasonic_manual_guard_sim3'),reason='legacy core diagnostic late declaration rejected'),
                   dict(path=str(BASE/'ultrasonic_manual_guard_core_sim1'),reason='XSim elaboration access violation after declaration-only simulation copies'),
                   dict(path=str(BASE/'ultrasonic_manual_guard_core_sim2'),reason='same elaboration exception after also ordering original constants'),
                   dict(path=str(BASE/'ultrasonic_manual_guard_core_AE1'),reason='Tcl include path escaping in launcher'),
                   dict(path=str(BASE/'ultrasonic_manual_guard_core_AE2'),reason='original late localparams rejected by ModelSim'),
                   dict(path=str(BASE/'ultrasonic_manual_guard_core_AE3'),reason='all compilation passed; installed license rejected Verilog simulation; no license bypass')],
               simulator_fallback='Icarus10.3 installed from existing Ubuntu20.04 package repository (1.859MB download); unmodified source elaboration and run passed',
               limitations=['No full-vehicle dynamics or physical stopping-distance acceptance',
                            'No new mapped resource or timing acceptance',
                            'Sensor center/mounting offset and physical remote0 code not verified on board',
                            'Real4500 test uses synthetic100Hz stationary IMU and BIAS_SAMPLES4, not a closed loop',
                            'Legacy bias/ELU read-width warnings retained; model assets unchanged',
                            'NAV commands intentionally bypass this manual-only protection'])
    (out/'simulation_results.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    csv_path=BASE/'ultrasonic_manual_guard_sensor_sim1/sensor_guard/sensor_guard_profile.csv'
    with csv_path.open() as f: rows=list(csv.DictReader(f))
    t=[int(r['time_us'])/1e6 for r in rows]
    font=FontProperties(fname='C:/Windows/Fonts/msyh.ttc')
    fig,axes=plt.subplots(3,1,figsize=(10,7),sharex=True)
    axes[0].plot(t,[int(r['obstacle_mm']) for r in rows],label='Echo stimulus')
    axes[0].plot(t,[int(r['measured_mm']) if int(r['measured_mm'])<5000 else float('nan') for r in rows],label='Measured')
    axes[0].axhline(250,ls='--',color='tab:red',label='Stop threshold 250 mm')
    axes[0].set_ylabel('Distance (mm)');axes[0].legend(loc='upper right',fontsize=8)
    axes[1].plot(t,[int(r['forward_q12'])/4096 for r in rows],color='tab:green')
    axes[1].set_ylabel('Forward target (m/s)')
    axes[2].step(t,[int(r['healthy']) for r in rows],where='post',label='Fresh front measurement')
    axes[2].step(t,[int(r['stop_latched']) for r in rows],where='post',label='Stop latch')
    axes[2].set_xlabel('Simulation time (s)');axes[2].set_yticks([0,1]);axes[2].legend(loc='upper left',fontsize=8)
    for ax in axes:ax.grid(alpha=.25)
    fig.suptitle('超声波Echo协议链仿真：减速目标与停车锁存（非车体速度）',fontproperties=font)
    fig.tight_layout();fig.savefig(out/'approach_profile.png',dpi=160)
    svg=out/'approach_profile.svg';fig.savefig(svg);plt.close(fig)
    svg.write_text('\n'.join(line.rstrip() for line in svg.read_text(encoding='utf8').splitlines())+'\n',encoding='utf8',newline='\n')
    print(json.dumps(dict(passed=True,profiles=len(profiles),evidence=str(out)),ensure_ascii=False))

if __name__=='__main__':main()
