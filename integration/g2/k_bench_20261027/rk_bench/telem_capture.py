"""Record every control telemetry frame (P11-40, 230400) with host time to a JSON-lines file.
Usage: python3 telem_capture.py <seconds> <out.jsonl>
"""
import json
import sys
import time

from bench import Telemetry


def main():
    secs, path = float(sys.argv[1]), sys.argv[2]
    t = Telemetry()
    t0 = time.time()
    n = 0
    with open(path, 'w') as f:
        while time.time() - t0 < secs:
            for fr in t.poll():
                fr = {k: (None if isinstance(v, float) and v != v else v) for k, v in fr.items()}
                fr['host_t'] = round(time.time() - t0, 3)
                f.write(json.dumps(fr) + '\n')
                n += 1
    print(json.dumps({'frames': n, 'bad_crc': t.dec.bad_crc, 'skipped': t.dec.skipped}))


if __name__ == '__main__':
    main()
