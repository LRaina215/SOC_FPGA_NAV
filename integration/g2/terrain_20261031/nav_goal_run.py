"""Ground navigation run: IDLE -> GOAL (odometry frame, mm), host heartbeats every 100 ms, STATUS logged
at 5 Hz to <log>.jsonl and printed at 2 Hz. Ends on ARRIVED (held 1 s), timeout, or SIGINT/SIGTERM;
always sends IDLE before exiting so the chassis gets zero commands.
Usage: python3 nav_goal_run.py <goal_x_mm> <goal_y_mm> <seconds> <log_prefix>
"""
import json
import signal
import sys
import time

sys.path.insert(0, 'tools')
import nav_host  # noqa: E402

stop = False


def on_signal(*_):
    global stop
    stop = True


def main():
    gx, gy, secs, prefix = int(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    link = nav_host.UartLink('/dev/ttyACM0')
    log = open(prefix + '.jsonl', 'w')
    link.send(nav_host.mode(nav_host.MODES['idle']))
    time.sleep(0.2)
    link.send(nav_host.goal(gx, gy))
    t0 = time.time()
    last_hb = last_print = last_log = -1.0
    arrived_t = None
    try:
        while not stop and time.time() - t0 < secs:
            now = time.time() - t0
            if now - last_hb >= 0.1:
                link.send(nav_host.target(0, 0, found=0))
                last_hb = now
            msg = nav_host.parse(link.recv(0.03) or b'')
            if not msg or msg.get('type') != 'status':
                continue
            if now - last_log >= 0.2:
                last_log = now
                log.write(json.dumps(dict(msg, t=round(now, 2)), ensure_ascii=False) + '\n')
                log.flush()
            if now - last_print >= 0.5:
                last_print = now
                print('t=%5.1f state=%s pose=(%5d,%5d) yaw=%6.1f cmd vx=%4d wz=%5d loc=%s'
                      % (now, msg['state'], msg['x'], msg['y'], msg['yaw'], msg['vx'], msg['wz'], msg.get('loc')),
                      flush=True)
            if msg['state'] in ('到达', 'arrived', 'ARRIVED'):
                arrived_t = arrived_t or now
                if now - arrived_t > 1.0:
                    print('arrived', flush=True)
                    break
            else:
                arrived_t = None
    finally:
        for _ in range(3):
            link.send(nav_host.mode(nav_host.MODES['idle']))
            time.sleep(0.05)
        link.u.s.close()
        log.close()
        print('sent IDLE, done', flush=True)


if __name__ == '__main__':
    main()
