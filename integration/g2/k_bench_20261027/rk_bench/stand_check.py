"""Ground standing monitor for the PNR61 joint image (control telemetry on /dev/ttyUSB0).

Live:    python3 stand_check.py live <seconds> [out.jsonl]   (stop yuntuo-fpga-bridge first: it owns ttyUSB0)
Offline: python3 stand_check.py file <capture.jsonl>        (telem_capture.py output)

Besides attitude/wheel/fault statistics it evaluates the command lease arming rule of
command_lease_guard.v: every 10 ms feedback sample must satisfy |vx| <= 5 mm/s and
|wz| <= 20 mrad/s for 500 ms in a row (vx = (left_cms + right_cms) * 5, wz = wz_q12 * 1000 >> 12).
If that never happens while the car balances, navigation speed commands can never be armed.
"""
import json
import math
import statistics as st
import sys
import time


def lease_still(r):
    vx = (r['left_cms'] + r['right_cms']) * 5
    wz = (r['wz_q12'] * 1000) >> 12
    return -5 <= vx <= 5 and -20 <= wz <= 20, vx, wz


def summarize(rows):
    if not rows:
        return {'frames': 0}
    seg = rows
    for i in range(1, len(rows)):
        if rows[i]['time_s'] < rows[i - 1]['time_s']:
            seg = rows[i:]              # keep only the part after the last FPGA restart
    active = [r for r in seg if r['status'] & 128]
    out = {'frames': len(seg), 'active_frames': len(active),
           'first_ts': seg[0]['time_s'], 'last_ts': seg[-1]['time_s'],
           'last_status': seg[-1]['status'], 'policy_fault': seg[-1]['policy_fault'],
           'sensor_error': seg[-1]['sensor_error'],
           'wheel_faults': [seg[-1]['left_fault'], seg[-1]['right_fault']],
           'completed_frames': seg[-1]['completed_frames']}
    if not active:
        return out
    def stat(key, scale=1.0):
        v = [r[key] * scale for r in active if r.get(key) is not None]
        return [round(st.mean(v), 2), round(st.pstdev(v), 2), round(min(v), 2), round(max(v), 2)] if v else None
    for r in active:
        gx, gy, gz = (r['gravity_' + a + '_q12'] / 4096.0 for a in 'xyz')
        r['nav_pitch_deg'] = math.degrees(math.atan2(gy, math.hypot(gx, gz)))   # nose down positive
        r['nav_roll_deg'] = math.degrees(math.atan2(gx, -gz))                  # left side up positive
    out['nav_pitch_deg_mean_std_min_max'] = stat('nav_pitch_deg')
    out['nav_roll_deg_mean_std_min_max'] = stat('nav_roll_deg')
    out['left_cms'] = stat('left_cms')
    out['right_cms'] = stat('right_cms')
    out['left_duty'] = stat('left_duty')
    out['right_duty'] = stat('right_duty')
    out['left_target_mmps'] = stat('left_target_mmps')
    out['right_target_mmps'] = stat('right_target_mmps')
    out['servo_us_last'] = [active[-1][k] for k in ('lh_us', 'lk_us', 'rh_us', 'rk_us')]
    out['forward_cmd_last'] = active[-1]['forward_command']
    run = best = n_still = 0
    vxs, wzs = [], []
    for r in active:
        ok, vx, wz = lease_still(r)
        vxs.append(vx)
        wzs.append(wz)
        n_still += ok
        run = run + 1 if ok else 0
        best = max(best, run)
    out['lease_feedback_vx_mmps'] = [round(st.mean(vxs), 1), round(st.pstdev(vxs), 1)]
    out['lease_feedback_wz_mradps'] = [round(st.mean(wzs), 1), round(st.pstdev(wzs), 1)]
    out['lease_still_fraction'] = round(n_still / len(active), 3)
    out['lease_longest_still_ms'] = best * 10
    out['lease_can_arm'] = best >= 50
    return out


def main():
    mode = sys.argv[1]
    if mode == 'file':
        rows = [json.loads(l) for l in open(sys.argv[2])]
    else:
        from bench import Telemetry
        secs = float(sys.argv[2])
        path = sys.argv[3] if len(sys.argv) > 3 else None
        t = Telemetry()
        rows = []
        t0 = time.time()
        while time.time() - t0 < secs:
            rows += t.poll()
        rows = [{k: (None if isinstance(v, float) and v != v else v) for k, v in r.items()} for r in rows]
        if path:
            with open(path, 'w') as f:
                for r in rows:
                    f.write(json.dumps(r) + '\n')
        print(json.dumps({'bad_crc': t.dec.bad_crc, 'skipped': t.dec.skipped}))
    print(json.dumps(summarize(rows), ensure_ascii=False, indent=1))


if __name__ == '__main__':
    main()
