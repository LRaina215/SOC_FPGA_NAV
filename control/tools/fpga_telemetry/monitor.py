"""Read-only serial monitor: validated V1/V2 packets -> CSV and live plots."""
import argparse
from collections import deque
import csv
from datetime import datetime
import json
from pathlib import Path
import queue
import threading
import time

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import serial
from serial.tools import list_ports

from protocol import Decoder


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='auto')
    parser.add_argument('--baud', type=int, default=230400, help='New V2 firmware:230400; old V1 firmware: --baud 115200')
    parser.add_argument('--record-only', action='store_true', help='Record continuously without opening a plot window')
    parser.add_argument('--output-dir', type=Path, default=Path(__file__).with_name('logs'))
    parser.add_argument('--status-file', type=Path, default=Path(__file__).with_name('monitor_status.json'))
    args = parser.parse_args()
    data = queue.Queue(maxsize=5000)
    stop = threading.Event()
    decoder = Decoder()
    status = {'connection': 'Searching for USB serial port', 'last_frame': 0., 'rx_bytes': 0, 'frames': 0}
    directory = args.output_dir.resolve()
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / ('fpga_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.csv')

    def receive():
        writer = None
        last_ms = None
        offset_ms = 0
        first_time_s = None
        with path.open('w', buffering=1) as csv_file:
            while not stop.is_set():
                candidates = list(list_ports.comports())
                preferred = [p for p in candidates if p.vid == 0x1a86]
                if args.port == 'auto' and not preferred:
                    status['connection'] = 'No CH340 detected; waiting for USB'
                    stop.wait(1.)
                    continue
                port = preferred[0].device if args.port == 'auto' else args.port
                try:
                    with serial.Serial(port, args.baud, timeout=.1, exclusive=True) as device:
                        status['connection'] = port+' connected; waiting for valid telemetry'
                        decoder.buffer.clear()
                        decoder.last_sequence = None
                        while not stop.is_set():
                            chunk = device.read(4096)
                            status['rx_bytes'] += len(chunk)
                            for row in decoder.feed(chunk):
                                ms = row['timestamp_ms']
                                if last_ms is not None and ms < last_ms:
                                    offset_ms += 2**32 if last_ms-ms > 2**31 else last_ms+(10 if row['protocol_version']==2 else 20)
                                last_ms = ms
                                row['time_s'] = (ms+offset_ms)/1000.
                                if first_time_s is None:
                                    first_time_s = row['time_s']
                                    status['first_valid_at'] = datetime.now().isoformat(timespec='milliseconds')
                                row['record_time_s'] = row['time_s']-first_time_s
                                row['host_timestamp'] = datetime.now().isoformat(timespec='milliseconds')
                                if writer is None:
                                    writer = csv.DictWriter(csv_file, fieldnames=list(row))
                                    writer.writeheader()
                                writer.writerow(row)
                                status['last_frame'] = time.monotonic()
                                status['frames'] += 1
                                status['connection'] = port
                                if data.full():
                                    data.get_nowait()
                                data.put_nowait(row)
                except (serial.SerialException, OSError) as error:
                    status['connection'] = str(error)
                    stop.wait(1.)

    thread = threading.Thread(target=receive, daemon=True)
    thread.start()

    def save_status(latest):
        snapshot = dict(status, crc_errors=decoder.bad_crc, missing_frames=decoder.skipped,
                        csv=str(path), latest=latest, record_only=args.record_only)
        temporary = args.status_file.with_suffix('.tmp')
        temporary.write_text(json.dumps(snapshot, indent=2))
        temporary.replace(args.status_file)

    if args.record_only:
        print('RECORDER_READY', 'port='+args.port, 'CSV='+str(path), flush=True)
        latest = None
        try:
            while not stop.is_set():
                while not data.empty():
                    latest = data.get_nowait()
                save_status(latest)
                stop.wait(.5)
        except KeyboardInterrupt:
            pass
        finally:
            stop.set()
            thread.join(timeout=2.)
        return
    panels = [
        ('Fused body tilt', 'deg', [('roll_deg', 'X rotation'), ('pitch_deg', 'Y rotation')]),
        ('Fused angular velocity', 'rad/s', [('gyro_x', 'X'), ('gyro_y', 'Y'), ('gyro_z', 'Z')]),
        ('Left wheel velocity', 'm/s', [('left_target', 'Model target'), ('left_reference', 'PID reference'), ('left_speed', 'Encoder')]),
        ('Right wheel velocity', 'm/s', [('right_target', 'Model target'), ('right_reference', 'PID reference'), ('right_speed', 'Encoder')]),
        ('Forward command / encoder estimate', 'm/s', [('forward_command', 'Body command'), ('wheel_mean_speed', 'Wheel mean')]),
        ('Yaw rate', 'rad/s', [('yaw_command', 'Command'), ('gyro_z', 'Gyro Z')]),
        ('Signed motor PWM', '%', [('left_pwm_percent', 'Left'), ('right_pwm_percent', 'Right')]),
        ('Raw accelerometer', 'g', [('accel_x_g', 'X'), ('accel_y_g', 'Y'), ('accel_z_g', 'Z')]),
    ]
    fig, axes = plt.subplots(4, 2, figsize=(12, 8.5), constrained_layout=True)
    fig.canvas.manager.set_window_title('FPGA sensor telemetry | REAL HARDWARE')
    curves = []
    for ax, (title, unit, series) in zip(axes.flat, panels):
        ax.set(title=title, xlabel='FPGA time (s)', ylabel=unit)
        ax.grid(alpha=.25)
        for key, label in series:
            line, = ax.plot([], [], '--' if 'target' in key or 'command' in key else '-', lw=1.2, label=label)
            curves.append((line, key))
        ax.legend(loc='upper right', fontsize=7, ncol=3)
    heading = fig.suptitle('Waiting for FPGA telemetry', fontsize=10)
    rows = deque(maxlen=2500)
    screenshot_saved = False

    def update(_):
        nonlocal screenshot_saved
        while not data.empty():
            rows.append(data.get_nowait())
        save_status(rows[-1] if rows else None)
        if not rows:
            heading.set_text(status['connection']+f' | {args.baud} 8N1 | no valid frames yet\n'
                             'RX bytes=%d | CRC errors=%d' % (status['rx_bytes'], decoder.bad_crc))
            return
        last = rows[-1]
        visible = [r for r in rows if r['time_s'] >= last['time_s']-20]
        ts = [r['time_s'] for r in visible]
        for line, key in curves:
            line.set_data(ts, [r[key] for r in visible])
        for ax in axes.flat:
            ax.set_xlim(max(0., last['time_s']-20), max(1., last['time_s']))
            ax.relim();ax.autoscale_view(scalex=False)
        live = time.monotonic()-status['last_frame'] < 1.
        heading.set_text('%s | %s | model active=%s flags=%02X fault=%d motors=%d/%d | frames=%d\n'
                         'seq=%d CRC errors=%d missing=%d | HEIGHT TARGET ONLY %.3fm | CSV %s' %
                         (status['connection'], 'LIVE' if live else 'STALE / DISCONNECTED',
                          last['active'], last['status'], last['policy_fault'], last['left_fault'],
                          last['right_fault'], last['completed_frames'], last['sequence'],
                          decoder.bad_crc, decoder.skipped, last['height_command'], path.name))
        if len(rows) >= 50 and not screenshot_saved:
            fig.savefig(Path(__file__).with_name('hardware_preview.png'), dpi=120)
            screenshot_saved = True

    animation = FuncAnimation(fig, update, interval=200, cache_frame_data=False)
    print('MONITOR_READY', 'port='+args.port, 'CSV='+str(path), flush=True)
    plt.show()
    stop.set()
    thread.join(timeout=1.)
    return animation


if __name__ == '__main__':
    main()
