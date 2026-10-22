"""Receive the temporary B0/32-byte BMI diagnostic stream; no UART writes."""
import argparse
from collections import Counter
import csv
from datetime import datetime
import json
from pathlib import Path
import statistics
import struct
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "control/tools/fpga_telemetry"))
from protocol import crc16  # noqa: E402

PAYLOAD = struct.Struct("<IH8BI6hH")
NAMES = ("timestamp_ms sequence status error_code operation accel_address gyro_address "
         "accel_id gyro_id last_read sample_count ax_raw ay_raw az_raw gx_raw gy_raw gz_raw signature").split()


class Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_crc = 0

    def feed(self, data):
        self.buffer.extend(data)
        rows = []
        while len(self.buffer) >= 4:
            offset = self.buffer.find(b"\xa5\x5a\xb0\x20")
            if offset < 0:
                self.buffer[:] = self.buffer[-3:]
                break
            del self.buffer[:offset]
            if len(self.buffer) < 38:
                break
            packet = self.buffer[:38]
            if crc16(packet[2:36]) != int.from_bytes(packet[36:38], "little"):
                self.bad_crc += 1
                del self.buffer[0]
                continue
            del self.buffer[:38]
            row = dict(zip(NAMES, PAYLOAD.unpack(packet[4:36])))
            if row["signature"] != 0xbc01:
                continue
            row["sample_available"] = row["status"] == 5 and row["sample_count"] > 0
            rows.append(row)
        return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port")
    source.add_argument("--replay", type=Path)
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    if not 0.1 <= args.seconds <= 3600:
        parser.error("--seconds must be between 0.1 and 3600")
    args.out.mkdir(parents=True, exist_ok=False)
    decoder = Decoder()
    rows = []
    start = time.monotonic()
    error = None
    with (args.out / "rx.bin").open("wb") as raw:
        def consume(chunk):
            raw.write(chunk)
            raw.flush()
            rows.extend(decoder.feed(chunk))
        try:
            if args.replay:
                consume(args.replay.read_bytes())
            else:
                import serial
                device = serial.Serial(port=None, baudrate=230400, timeout=.1,
                                       rtscts=False, dsrdtr=False, xonxoff=False)
                device.dtr = False
                device.rts = False
                device.port = args.port
                with device:
                    while time.monotonic() - start < args.seconds:
                        consume(device.read(4096))
        except (OSError, ImportError) as exc:
            error = str(exc)
    if rows:
        with (args.out / "frames.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    valid = [row for row in rows if row["sample_available"]]
    summary = dict(source="offline_replay" if args.replay else "hardware_uart",
                   recorded_at=datetime.now().astimezone().isoformat(), port=args.port,
                   elapsed_s=time.monotonic() - start, tx_bytes=0, error=error,
                   diagnostic_frames=len(rows), crc_errors=decoder.bad_crc,
                   available_sample_snapshots=len(valid),
                   distinct_sample_counts=len({row["sample_count"] for row in valid}),
                   error_histogram=dict(Counter(str(row["error_code"]) for row in rows)),
                   last_frame=rows[-1] if rows else None,
                   raw_ranges={key: dict(min=min(row[key] for row in valid),
                                         max=max(row[key] for row in valid),
                                         mean=statistics.mean(row[key] for row in valid),
                                         population_std=statistics.pstdev(row[key] for row in valid))
                               for key in ("ax_raw", "ay_raw", "az_raw", "gx_raw", "gy_raw", "gz_raw")} if valid else {},
                   scope="BMI driver diagnostic only; no fusion, navigation, control or physical stillness acceptance")
    (args.out / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0 if rows and error is None else 2


if __name__ == "__main__":
    raise SystemExit(main())
