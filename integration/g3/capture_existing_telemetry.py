"""Bounded RX-only capture of the existing control V1/V2 UART protocol."""
import argparse
from collections import Counter
import csv
from datetime import datetime
import json
from pathlib import Path
import statistics
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "control/tools/fpga_telemetry"))
from protocol import Decoder  # noqa: E402


def summarize(rows, decoder):
    steps = Counter()
    sequence_events = Counter()
    for previous, current in zip(rows, rows[1:]):
        delta = (current["timestamp_ms"] - previous["timestamp_ms"]) & 0xffffffff
        steps[str(delta) if delta < 0x80000000 else "backward_reset_or_reorder"] += 1
        advance = (current["sequence"] - previous["sequence"]) & 0xffff
        event = "duplicate" if advance == 0 else "next" if advance == 1 else (
            "forward_gap" if advance < 0x8000 else "reset_or_reorder")
        sequence_events[event] += 1
    ranges = {}
    for key in ("ax_raw", "ay_raw", "az_raw", "gx_raw", "gy_raw", "gz_raw",
                "wx_q12", "wy_q12", "wz_q12", "left_delta", "right_delta",
                "left_cms", "right_cms", "forward_q12", "yaw_q12", "inference_cycles"):
        values = [row[key] for row in rows]
        if values:
            ranges[key] = dict(min=min(values), max=max(values), mean=statistics.mean(values),
                               population_std=statistics.pstdev(values))
    return dict(
        valid_frames=len(rows), crc_errors=decoder.bad_crc,
        protocol_versions=dict(Counter(str(row["protocol_version"]) for row in rows)),
        timestamp_step_ms_histogram=dict(steps), sequence_events=dict(sequence_events),
        flag_histograms={key: dict(Counter(str(row[key]) for row in rows)) for key in (
            "status", "fusion_flags", "sensor_error", "policy_fault", "left_fault", "right_fault")},
        observed_values=ranges,
        limitations=[
            "Existing image identity, sensor supply, axis mapping and physical stillness are not established by decoding.",
            "Telemetry contains snapshots; sensor values are not necessarily simultaneous or newly sampled per packet.",
            "Raw encoder deltas have no dedicated sample sequence. Do not sum repeated/missed snapshots as odometry.",
            "Nominal physical-unit columns come from the existing decoder; they are not measured calibration.",
            "No stationary-bias or physical-stop acceptance is inferred from these statistics.",
        ],
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="Explicit control telemetry adapter; no automatic port selection")
    source.add_argument("--replay", type=Path, help="Offline raw UART bytes, not a hardware measurement")
    parser.add_argument("--baud", type=int, choices=(115200, 230400), default=230400)
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--out", type=Path, required=True, help="New directory, never overwrite an earlier capture")
    args = parser.parse_args()
    if not 0.1 <= args.seconds <= 3600:
        parser.error("--seconds must be between 0.1 and 3600")
    args.out.mkdir(parents=True, exist_ok=False)
    decoder = Decoder()
    rows = []
    writer = None
    rx_bytes = 0
    error = None
    start = time.monotonic()
    result = dict(started_at=datetime.now().astimezone().isoformat(),
                  source="offline_replay" if args.replay else "hardware_uart",
                  port=args.port, baud=args.baud, tx_bytes=0,
                  requested_dtr=False, requested_rts=False,
                  hardware_acceptance="not_assessed")
    with (args.out / "rx.bin").open("wb") as raw, (args.out / "frames.csv").open(
            "w", newline="", encoding="utf-8") as csv_file:
        def consume(chunk):
            nonlocal writer, rx_bytes
            raw.write(chunk)
            raw.flush()
            rx_bytes += len(chunk)
            for frame in decoder.feed(chunk):
                row = dict(frame, host_received_at=datetime.now().astimezone().isoformat(),
                           host_elapsed_s=time.monotonic() - start)
                if writer is None:
                    writer = csv.DictWriter(csv_file, fieldnames=list(row))
                    writer.writeheader()
                writer.writerow(row)
                rows.append(row)
            csv_file.flush()
        try:
            if args.replay:
                with args.replay.open("rb") as replay:
                    while chunk := replay.read(4096):
                        consume(chunk)
            else:
                import serial
                # Set modem-line requests before opening. Driver/electrical transients are not measured here.
                device = serial.Serial(port=None, baudrate=args.baud, timeout=0.1,
                                       write_timeout=0.1, rtscts=False, dsrdtr=False, xonxoff=False)
                device.dtr = False
                device.rts = False
                device.port = args.port
                with device:
                    while time.monotonic() - start < args.seconds:
                        consume(device.read(4096))
        except (OSError, ImportError) as exc:
            error = str(exc)
    result.update(elapsed_s=time.monotonic() - start, rx_bytes=rx_bytes, error=error,
                  **summarize(rows, decoder))
    (args.out / "summary.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps({key: result[key] for key in (
        "source", "port", "rx_bytes", "valid_frames", "crc_errors", "error")}, indent=2))
    return 0 if rows and error is None else 2


if __name__ == "__main__":
    raise SystemExit(main())
