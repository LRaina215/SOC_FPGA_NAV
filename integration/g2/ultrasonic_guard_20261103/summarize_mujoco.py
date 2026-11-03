"""Summarize existing motion.csv files only. Does not run physics or RTL."""
import csv
import json
from pathlib import Path
import shutil

HERE=Path(__file__).resolve().parent
BASE=Path('F:/nav_integration_g2_20261023')

def analyze(folder):
    original=json.loads((folder/'result.json').read_text())
    with (folder/'motion.csv').open() as f: rows=list(csv.DictReader(f))
    stationary_start=None;stationary=None
    for r in rows:
        t=float(r['t'])
        if r['stop_latched']=='1' and abs(float(r['actual_forward_mps']))<=.003:
            if stationary_start is None:stationary_start=t
            if t-stationary_start>=.5 and stationary is None:stationary=stationary_start
        else:stationary_start=None
    obstacle=next((r for r in rows if r['sonar_hit']=='front_obstacle' and
                  float(r['sonar_m'])<=.6 and float(r['remote_request_mps'])>0),None)
    floor=next((r for r in rows if r['sonar_hit']=='floor' and
                float(r['sonar_m'])<=.6 and float(r['remote_request_mps'])>0),None)
    result={k:original[k] for k in ('executable','mujoco','remote_forward_mps','remote_forward_q12',
        'SG90_scan','slow_mm','stop_mm','simulated_seconds','zero_target_s','min_body_clearance_m',
        'maximum_tilt_deg','final_speed_mps','last2s_max_abs_speed_mps','last2s_forward_drift_m',
        'no_collision','no_fall')}
    result.update(output=str(folder),encoder_parking_hold=original.get('stop_speed_hold',False),
        stationary_speed_band_mps=.003,stationary_minimum_duration_s=.5,
        stationary_band_observed_from_s=stationary,
        obstacle_slowing_first_observed_s=float(obstacle['t']) if obstacle else None,
        modeled_floor_early_trigger_s=float(floor['t']) if floor else None,
        initial_coarse30mmps_stop_flag_not_used_as_acceptance=True)
    return result

def main():
    out=HERE/'evidence/mujoco';out.mkdir(exist_ok=True)
    summary=dict(scope='one MuJoCo physical scenario, necessary zero-target/parking-hold comparison',
        raw_remote_speed_unchanged=True,physical_scenarios=1,valid_runs=2,
        zero_target_only=analyze(BASE/'ultrasonic_fixed600_mujoco3'),
        parking_hold_candidate=analyze(BASE/'ultrasonic_fixed600_mujoco_hold1'),
        FPGA_image_changed=False,parking_hold_integrated_into_RTL=False,
        operating_hardware_speed_source='K_terrain19_follow1/control/ir_remote_control.v: FORWARD_Q12=1229 (~0.3m/s)',
        limitation='Native original4500 floating-point model and idealized sonar; no hardware stop certification',
        known_problem='The assumed15deg cone hits floor during acceleration and causes one early reduction; real mounting/echo behavior unmeasured',
        initialization_failure_preserved=[str(BASE/'ultrasonic_fixed600_mujoco1'),str(BASE/'ultrasonic_fixed600_mujoco2')],
        initialization_fix='Set initial XY in the runtime configuration copy BEFORE Robot.reset advances8ms; avoid spawning across the back wall. No gains/mass/weights/forward speed changed.')
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    for name in ('ultrasonic_mujoco.mp4','mujoco_stop_frame.png','mujoco_stop_comparison.png','render.json'):
        shutil.copyfile(BASE/'ultrasonic_mujoco_replay3'/name,out/name)
    print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
