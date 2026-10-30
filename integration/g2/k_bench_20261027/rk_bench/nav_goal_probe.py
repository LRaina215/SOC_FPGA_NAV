"""Send IDLE then a GOAL (odometry frame, mm) and keep the host link alive with found=0 TARGET
heartbeats every 100 ms; print STATUS (mode/state/goal/commanded vx,wz) twice a second.
The chassis lease decides whether these commands move anything; on a stopped chassis they do not.
Usage: python3 nav_goal_probe.py <goal_x_mm> <goal_y_mm> [seconds=10]
"""
import sys
import time

sys.path.insert(0, 'tools')
import nav_host  # noqa: E402


def main():
    gx, gy = int(sys.argv[1]), int(sys.argv[2])
    secs = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
    link = nav_host.UartLink('/dev/ttyACM0')
    link.send(nav_host.mode(nav_host.MODES['idle']))
    time.sleep(0.1)
    link.send(nav_host.goal(gx, gy))
    t0 = time.time()
    last_hb = 0.0
    last_print = -1.0
    states = {}
    try:
        while time.time() - t0 < secs:
            now = time.time() - t0
            if now - last_hb >= 0.1:
                link.send(nav_host.target(0, 0, found=0))
                last_hb = now
            msg = nav_host.parse(link.recv(0.05) or b'')
            if msg and msg.get('type') == 'status':
                states[msg['state']] = states.get(msg['state'], 0) + 1
                if now - last_print >= 0.5:
                    last_print = now
                    print('t=%4.1f mode=%d state=%s pose=(%d,%d) yaw=%.1f goal=%s cmd vx=%d mm/s wz=%d mrad/s loc=%s'
                          % (now, msg['mode'], msg['state'], msg['x'], msg['y'], msg['yaw'], msg['goal'],
                             msg['vx'], msg['wz'], msg.get('loc')))
    finally:
        link.send(nav_host.mode(nav_host.MODES['idle']))
        link.u.s.close()
    print('state counts:', states)


if __name__ == '__main__':
    main()
