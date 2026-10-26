"""Wait for the car to be powered (BMI088 ready on P11-40 telemetry), then run the
bench motor sequence once and save the results.

Runs until DEADLINE (local time) and exits; polls the telemetry port for ~1.5 s every
20 s so the ports stay mostly free for other tools.
"""
import contextlib
import io
import json
import sys
import time
from datetime import datetime

import bench

DEADLINE_HHMM = sys.argv[1] if len(sys.argv) > 1 else '07:00'
LOG = 'watch.log'


def log(msg):
    with open(LOG, 'a', encoding='utf-8') as f:
        f.write(f'{datetime.now():%m-%d %H:%M:%S} {msg}\n')


def telemetry_state():
    try:
        t = bench.Telemetry()
    except Exception as e:  # port busy or missing
        return None, f'telemetry open failed: {e}'
    frames = []
    t_end = time.time() + 1.5
    while time.time() < t_end:
        frames += t.poll()
    t.s.close()
    if not frames:
        return None, 'no telemetry'
    last = frames[-1]
    ready = (last['status'] & 1) == 1 and last['sensor_error'] == 0 and not (last['status'] & 2)
    return ready, f"status={last['status']} sensor_error={last['sensor_error']} active={last['active']}"


def run_capture(fn, *args):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        try:
            rc = fn(*args)
        except Exception as e:
            print(json.dumps({'exception': repr(e)}))
            rc = 1
    return rc, buf.getvalue().strip()


def main():
    hh, mm = (int(v) for v in DEADLINE_HHMM.split(':'))
    now = datetime.now()
    deadline = now.replace(hour=hh, minute=mm, second=0, microsecond=0)
    if deadline <= now:
        deadline = deadline.replace(day=now.day + 1)
    log(f'start, deadline {deadline:%m-%d %H:%M}')
    ready_count = 0
    while datetime.now() < deadline:
        ready, info = telemetry_state()
        log(f'poll ready={ready} {info}')
        ready_count = ready_count + 1 if ready else 0
        if ready_count >= 2:
            log('BMI088 ready on two polls, running bench sequence')
            out = {'started': f'{datetime.now():%Y-%m-%d %H:%M:%S}'}
            b = bench.Bridge()
            out['selftest'] = run_capture(bench.cmd_selftest, b, 3.0)[1]
            out['poweredtest'] = run_capture(bench.cmd_poweredtest, b)[1]
            out['estoptest'] = run_capture(bench.cmd_estoptest, b, 50)[1]
            name = f'powered_results_{datetime.now():%Y%m%d_%H%M%S}.json'
            with open(name, 'w', encoding='utf-8') as f:
                json.dump(out, f, ensure_ascii=False, indent=1)
            log(f'done, results in {name}')
            return 0
        time.sleep(20)
    log('deadline reached without vehicle power')
    return 1


if __name__ == '__main__':
    sys.exit(main())
