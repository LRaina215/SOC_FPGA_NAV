"""Ground low-speed command test through SAFE -> command lease -> original 4500 policy.

The nav CPU is held in reset, so SAFE_CMD_V/W written here are the only motion source.
Commands are refreshed every 50 ms. Phases: zero (lease must arm), forward, zero, reverse,
zero, left turn, zero, right turn, zero, forward, then writes stop (host loss) and the
SAFE watchdog / lease deadline must bring the commands back to zero.
Usage: python3 ground_cmd_test.py <out.jsonl> [vx_mmps=100] [wz_mradps=500]
"""
import json
import statistics as st
import sys
import time

import nav_regs_map as rm
from bench import Bridge, Telemetry, SOC_CTRL


def main():
    path = sys.argv[1]
    vx = int(sys.argv[2]) if len(sys.argv) > 2 else 100
    wz = int(sys.argv[3]) if len(sys.argv) > 3 else 500
    phases = [('arm_zero', 0, 0, 4.0), ('forward', vx, 0, 2.0), ('zero1', 0, 0, 2.0),
              ('reverse', -vx, 0, 2.0), ('zero2', 0, 0, 2.0), ('left', 0, wz, 2.0),
              ('zero3', 0, 0, 2.0), ('right', 0, -wz, 2.0), ('zero4', 0, 0, 2.0),
              ('pre_loss', vx, 0, 1.5)]
    b = Bridge()
    t = Telemetry()
    lim_v, lim_w = b.rd(rm.REGS['SAFE_LIM_V']), b.rd(rm.REGS['SAFE_LIM_W'])
    log = []
    t0 = time.time()

    def poll(name):
        for f in t.poll():
            f = {k: (None if isinstance(v, float) and v != v else v) for k, v in f.items()}
            f['phase'] = name
            f['host_t'] = round(time.time() - t0, 3)
            log.append(f)

    b.wr(SOC_CTRL, 1)
    last_write = None
    try:
        b.wr(rm.REGS['SAFE_LIM_V'], (lim_v & 0xFFFF0000) | (abs(vx) + 20))
        b.wr(rm.REGS['SAFE_LIM_W'], abs(wz) + 100)
        b.wr(rm.REGS['SAFE_CTRL'], 1)
        for name, v, w, secs in phases:
            end = time.time() + secs
            while time.time() < end:
                b.wr(rm.REGS['SAFE_CMD_V'], v & 0xFFFF)
                b.wr(rm.REGS['SAFE_CMD_W'], w & 0xFFFF)
                last_write = time.time() - t0
                poll(name)
                time.sleep(0.05)
        end = time.time() + 3.0
        while time.time() < end:
            poll('loss')
            time.sleep(0.01)
    finally:
        b.wr(rm.REGS['SAFE_CMD_V'], 0)
        b.wr(rm.REGS['SAFE_CMD_W'], 0)
        b.wr(rm.REGS['SAFE_LIM_V'], lim_v)
        b.wr(rm.REGS['SAFE_LIM_W'], lim_w)
        b.wr(SOC_CTRL, 0)
    with open(path, 'w') as fh:
        for f in log:
            fh.write(json.dumps(f) + '\n')

    order = [p[0] for p in phases] + ['loss']
    print(json.dumps({'frames': len(log), 'bad_crc': t.dec.bad_crc, 'skipped': t.dec.skipped}))
    for name in order:
        g = [f for f in log if f['phase'] == name]
        if not g:
            continue
        tail = g[len(g) // 2:]
        disp_cm = sum((f['left_cms'] + f['right_cms']) / 2 * 0.01 for f in g)
        yaw_deg = sum(f['wz_q12'] for f in g) / 4096 * 0.01 * 57.2958
        print(f"{name:9s} n={len(g):4d} fwd_cmd={st.mean(f['forward_command'] for f in tail):+.3f} "
              f"yaw_cmd={st.mean(f['yaw_command'] for f in tail):+.3f} "
              f"wheel={st.mean((f['left_cms'] + f['right_cms']) / 2 for f in g):+6.1f}cm/s "
              f"disp={disp_cm:+6.1f}cm yaw={yaw_deg:+6.1f}deg "
              f"status={sorted(set(f['status'] for f in g))} faults={sorted(set((f['left_fault'], f['right_fault'], f['policy_fault']) for f in g))}")
    loss = [f for f in log if f['phase'] == 'loss']
    zero = next((f for f in loss if f['forward_command'] == 0 and f['yaw_command'] == 0), None)
    if zero and last_write is not None:
        print(f"host loss: forward/yaw command zero {1000 * (zero['host_t'] - last_write):.0f} ms after the last write")
    armed = next((f for f in log if f['phase'] == 'forward' and f['forward_command'] > 0), None)
    print('lease armed and passed forward command:', armed is not None)


if __name__ == '__main__':
    main()
