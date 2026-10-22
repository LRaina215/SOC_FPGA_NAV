"""Run the unchanged 4500 policy in MuJoCo, interactively or headlessly."""
import argparse
import csv
import json
import math
from pathlib import Path
import queue
import subprocess
import time

import mujoco
import numpy as np
import torch

from runtime import ROOT, PROJECT, Policy, Robot

PLOT = PROJECT / 'model/original4500_to7000/live_sensor_plot.py'
PLOT_PYTHON = Path('/home/cyd/miniforge3/envs/sfgym/bin/python')
FIELDS = ['time_s', 'roll_deg', 'pitch_deg', 'gx', 'gy', 'gz', 'forward_cmd',
          'forward', 'yaw_cmd', 'left_target', 'left_actual', 'right_target',
          'right_actual', 'height_cmd', 'height', 'reset']
CASES = [('stop', 0., 0., .1305), ('forward', .2, 0., .1305),
         ('reverse', -.2, 0., .1305), ('left', 0., 1., .1305),
         ('right', 0., -1., .1305), ('low', 0., 0., .10), ('high', 0., 0., .18)]


def telemetry(robot, t, reset=False):
    m = robot.metrics()
    x, y, z, w = [*robot.data.qpos[4:7], robot.data.qpos[3]]
    roll = math.degrees(math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)))
    pitch = math.degrees(math.asin(np.clip(2*(w*y-z*x), -1, 1)))
    gyro, _ = robot.state()
    return [t, roll, pitch, *gyro, robot.effective_command()[0], m['forward_mps'], robot.command[1],
            m['left_target_mps'], m['left_actual_mps'], m['right_target_mps'],
            m['right_actual_mps'], robot.command[2], m['height_m'], int(reset)]


def evaluate(robot, policy, seconds):
    results = []
    out = ROOT / ('results' if robot.forward_bias == 0 else 'results_bias_%+.4f' % robot.forward_bias)
    out.mkdir(exist_ok=True)
    for name, v, yaw, height in CASES:
        robot.reset((v, yaw, height))
        effective = robot.effective_command()
        samples = []
        survived = True
        with (out / (name+'.csv')).open('w') as f:
            writer = csv.writer(f)
            writer.writerow(FIELDS)
            for step in range(round(seconds/robot.control_dt)):
                done = robot.step(policy)
                metrics = robot.metrics()
                if step % 6 == 0:
                    writer.writerow(telemetry(robot, (step+1)*robot.control_dt, done))
                if done:
                    survived = False
                    break
                if step*robot.control_dt >= 1.:
                    samples.append(metrics)
        result = dict(case=name, command=effective.tolist(), requested_command=[v, yaw, height], survived=survived,
                      seconds_simulated=(step+1)*robot.control_dt,
                      wheel_travel_m=robot.wheel_distance,
                      displacement_m=robot.metrics()['displacement_m'])
        if samples:
            result.update(mean_forward_mps=float(np.mean([s['forward_mps'] for s in samples])),
                          forward_mae_mps=float(np.mean([abs(s['forward_mps']-effective[0]) for s in samples])),
                          mean_yaw_radps=float(np.mean([s['yaw_radps'] for s in samples])),
                          yaw_mae_radps=float(np.mean([abs(s['yaw_radps']-yaw) for s in samples])),
                          height_mae_m=float(np.mean([abs(s['height_m']-height) for s in samples])),
                          tilt_rms_deg=float(np.mean([s['tilt_deg']**2 for s in samples])**.5))
        results.append(result)
        print('MUJOCO_EVAL', json.dumps(result), flush=True)
        (out / 'evaluation.json').write_text(json.dumps(
            dict(engine='MuJoCo '+mujoco.__version__, checkpoint='model_4500.pt',
                 control_dt=robot.control_dt, physics_dt=robot.dt,
                 tilt_threshold_deg=30, forward_bias_mps=robot.forward_bias,
                 trials_per_case=1, results=results), indent=2))
    return results


def interactive(robot, policy, seconds, plot):
    import mujoco.viewer
    events = queue.SimpleQueue()
    telemetry_dir = ROOT / 'telemetry'
    telemetry_dir.mkdir(exist_ok=True)
    path = telemetry_dir / ('mujoco_4500_'+time.strftime('%Y%m%d_%H%M%S')+'.csv')
    csv_file = path.open('w', buffering=1)
    writer = csv.writer(csv_file)
    writer.writerow(FIELDS)
    if plot:
        log = path.with_suffix('.log').open('w')
        subprocess.Popen([str(PLOT_PYTHON), '-u', str(PLOT), str(path),
                          '--title', 'MuJoCo | model 4500 | forward bias %+.3f m/s' % robot.forward_bias],
                         stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    start = time.monotonic()
    sim_elapsed = 0.
    next_render = next_sample = next_print = 0.
    resets = 0
    with mujoco.viewer.launch_passive(robot.model, robot.data,
                                     key_callback=events.put) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        viewer.cam.lookat[:] = robot.center()
        viewer.cam.distance = .75
        viewer.cam.azimuth = 125
        viewer.cam.elevation = -20
        # Show CAD visuals, hide the transparent collision geometry.
        viewer.opt.geomgroup[3] = 0
        print('MUJOCO_PREVIEW_READY model_4500 | forward bias %+.3f m/s | W/S speed A/D yaw U/J height SPACE zero-request R reset' % robot.forward_bias, flush=True)
        while viewer.is_running() and time.monotonic()-start < seconds:
            while not events.empty():
                key = events.get()
                if key == ord('W'): robot.command[0] = min(.6, robot.command[0]+.1)
                elif key == ord('S'): robot.command[0] = max(-.6, robot.command[0]-.1)
                elif key == ord('A'): robot.command[1] = min(2.9, robot.command[1]+.5)
                elif key == ord('D'): robot.command[1] = max(-2.9, robot.command[1]-.5)
                elif key == ord('U'): robot.command[2] = min(.19, robot.command[2]+.01)
                elif key == ord('J'): robot.command[2] = max(.085, robot.command[2]-.01)
                elif key == ord('1'): robot.command[2] = .085
                elif key == ord('2'): robot.command[2] = .19
                elif key == ord(' '): robot.command[:2] = 0.
                elif key == ord('R'):
                    robot.reset(tuple(robot.command));resets += 1
            done = robot.step(policy)
            sim_elapsed += robot.control_dt
            if sim_elapsed >= next_sample:
                writer.writerow(telemetry(robot, sim_elapsed, done))
                next_sample = sim_elapsed+.05
            if sim_elapsed >= next_print:
                state = dict(model='4500', command=robot.effective_command().tolist(),
                             requested_command=robot.command.tolist(), forward_bias_mps=robot.forward_bias, resets=resets,
                             **robot.metrics())
                (ROOT / 'viewer_status.json').write_text(json.dumps(state, indent=2))
                print(json.dumps(state), flush=True)
                next_print = sim_elapsed+1.
            if done:
                print('EPISODE_FAILED -> reset', json.dumps(robot.metrics()), flush=True)
                robot.reset(tuple(robot.command));resets += 1
            if sim_elapsed >= next_render:
                viewer.sync()
                next_render = sim_elapsed+1/30
            time.sleep(max(0., start+sim_elapsed-time.monotonic()))
    csv_file.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--eval', action='store_true')
    parser.add_argument('--seconds', type=float, default=None)
    parser.add_argument('--no-plot', action='store_true')
    parser.add_argument('--forward-bias', type=float, default=0.,
                        help='Constant offset to body forward command, m/s; negative = reverse')
    args = parser.parse_args()
    if not math.isfinite(args.forward_bias) or abs(args.forward_bias) > .6:
        parser.error('--forward-bias must be finite and within +/-0.6m/s')
    torch.set_num_threads(1)
    robot, policy = Robot(args.forward_bias), Policy()
    if args.eval:
        evaluate(robot, policy, args.seconds or 20.)
    else:
        interactive(robot, policy, args.seconds or 3600., not args.no_plot)


if __name__ == '__main__':
    main()
