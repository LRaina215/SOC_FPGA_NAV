"""Host link-loss stop test over the CH9102 mailbox.

Unlock with IDLE, then keep sending GOAL at 10 Hz for `secs`, then go silent and
measure how long the firmware takes to report ESTOP (HOST_LINK_TIMEOUT_US = 250 ms).
"""
import json
import sys
import time

sys.path.insert(0, 'tools')
sys.path.insert(0, 'model')
import nav_host as nh  # noqa: E402


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    link = nh.open_link('uart:/dev/ttyACM0')
    link.send(nh.mode(nh.MODES['idle']))
    time.sleep(0.15)
    statuses = []
    t0 = time.time()
    while time.time() - t0 < secs:
        link.send(nh.goal(500, 0))
        r = link.recv(0.08)
        if r:
            m = nh.parse(r) if isinstance(r, (bytes, bytearray)) else r
            if isinstance(m, dict) and m.get('type') == 'status':
                statuses.append((round(time.time() - t0, 2), m['mode'], m['state'], m['vx'], m['wz']))
    t_silent = time.time()
    after = []
    while time.time() - t_silent < 1.5:
        r = link.recv(0.05)
        if r:
            m = nh.parse(r) if isinstance(r, (bytes, bytearray)) else r
            if isinstance(m, dict) and m.get('type') == 'status':
                after.append((round(time.time() - t_silent, 3), m['mode'], m['state']))
    first_estop = next((t for t, mode, _ in after if mode == 4), None)
    print(json.dumps({'while_sending': statuses[-4:], 'after_silence': after[:8],
                      'first_estop_status_after_s': first_estop}, ensure_ascii=False))


if __name__ == '__main__':
    main()
