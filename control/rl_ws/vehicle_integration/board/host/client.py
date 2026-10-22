#!/usr/bin/env python3
"""Explicit UART client; dry-run and captured-reply checks never open devices."""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys

from golden import DEFAULT_VECTOR_DIR, load_vectors, select_samples, compare_result
from protocol import DIGEST, EXPECTED_CYCLES, INFER, Packet, encode, pack_s24
from transport import SerialExchange, ping, infer, check_capture

DEFAULT_REPORT_DIR = Path(__file__).resolve().parents[1] / "reports"


def make_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--dry-run", action="store_true", help="validate vectors/encode requests only; no serial imports")
    modes.add_argument("--offline", type=Path, metavar="HEX_FILE", help="validate one captured RESULT with JSON sidecar")
    parser.add_argument("--port", help="required in hardware mode, explicit device path; never scanned")
    parser.add_argument("--vector-dir", type=Path, default=DEFAULT_VECTOR_DIR)
    parser.add_argument("--samples", type=int, default=1)
    parser.add_argument("--start-index", type=int, default=0)
    parser.add_argument("--frame-id-start", type=int, default=1, help="monotonically increasing INFER IDs; PING uses preceding ID")
    parser.add_argument("--frame-id", type=int, help="expected frame ID for offline capture (otherwise sidecar)")
    parser.add_argument("--initial-sticky-saturation", type=int, choices=(0, 1), default=None,
                        help="known initial sticky bit4; omitted means infer baseline from first valid result")
    parser.add_argument("--timeout", type=float, default=1.0, help="request-response deadline in seconds")
    parser.add_argument("--report-dir", type=Path, default=DEFAULT_REPORT_DIR)
    parser.add_argument("--no-report", action="store_true", help="print JSON only, do not write JSON/log files")
    return parser


def execute(args, report, exchange_factory=SerialExchange):
    if args.offline:
        report["capture"] = str(args.offline.resolve())
        report["results"].append(check_capture(args.offline, args.vector_dir, args.start_index,
                                                args.frame_id, args.initial_sticky_saturation))
        return
    vectors = load_vectors(args.vector_dir)
    selected = select_samples(vectors, args.start_index, args.samples)
    report["available_samples"] = len(vectors)
    if args.dry_run:
        for offset, (index, sample) in enumerate(selected):
            wire = encode(Packet(INFER, args.frame_id_start + offset, pack_s24(sample.inputs)))
            report["results"].append({"sample_index": index, "frame_id": args.frame_id_start + offset,
                                       "request_bytes": len(wire), "expected_cycles": EXPECTED_CYCLES,
                                       "expected_saturated": sample.saturated, "comparison": "not performed"})
        return
    exchange = exchange_factory(args.port, args.timeout)
    try:
        report["ping_roundtrip_seconds"] = ping(exchange, args.frame_id_start - 1)
        sticky = args.initial_sticky_saturation
        for offset, (index, sample) in enumerate(selected):
            frame_id = args.frame_id_start + offset
            result, elapsed = infer(exchange, frame_id, sample)
            sticky = compare_result(result, sample, sticky)
            report["results"].append({"sample_index": index, "frame_id": frame_id,
                                       "core_cycles": result.cycles, "core_seconds_at_50mhz": result.cycles / 50_000_000,
                                       "host_roundtrip_seconds": elapsed, "flags": result.flags,
                                       "saturated": result.saturated, "layers_checked": 425,
                                       "latent_checked": 3, "actions_checked": 6, "comparison": "bit-exact"})
    finally:
        exchange.close()


def main(argv=None, exchange_factory=SerialExchange):
    parser = make_parser()
    args = parser.parse_args(argv)
    if not args.dry_run and not args.offline and not args.port:
        parser.error("--port is required in hardware mode; use --dry-run or --offline without hardware")
    if (args.dry_run or args.offline) and args.port:
        parser.error("--port cannot be combined with --dry-run or --offline")
    if args.samples < 1 or args.start_index < 0:
        parser.error("--samples must be positive and --start-index nonnegative")
    if args.offline and args.samples != 1:
        parser.error("--offline checks one packet; --samples must be 1")
    if args.frame_id is not None and not args.offline:
        parser.error("--frame-id is only for --offline")
    if args.frame_id is not None and not 0 <= args.frame_id <= 0xFFFFFFFF:
        parser.error("--frame-id must be uint32")
    if not 1 <= args.frame_id_start <= 0xFFFFFFFF or args.frame_id_start + args.samples - 1 > 0xFFFFFFFF:
        parser.error("INFER frame IDs must be monotonically increasing uint32 without wrap")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be positive and finite")
    mode = "dry-run" if args.dry_run else "offline" if args.offline else "hardware"
    report = {"timestamp_utc": datetime.now(timezone.utc).isoformat(), "mode": mode,
              "checkpoint_sha256": DIGEST.hex(), "vector_dir": str(args.vector_dir.resolve()),
              "initial_sticky_saturation": args.initial_sticky_saturation, "results": [],
              "physical_board_claim": False, "status": "running"}
    status = 0
    try:
        execute(args, report, exchange_factory)
        report["status"] = "validated" if mode != "dry-run" else "preflight-only"
    except (ValueError, OSError, RuntimeError, TimeoutError) as exc:
        report["status"] = "failed"
        report["error"] = f"{type(exc).__name__}: {exc}"
        status = 1
    if not args.no_report:
        args.report_dir.mkdir(parents=True, exist_ok=True)
        stem = "host_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        output = args.report_dir / (stem + ".json")
        report["report_json"] = str(output.resolve())
        output.write_text(json.dumps(report, indent=2) + "\n", encoding="ascii")
        (args.report_dir / (stem + ".log")).write_text(
            f"mode={mode} status={report['status']} checkpoint={DIGEST.hex()}\n"
            + "\n".join(json.dumps(row, sort_keys=True) for row in report["results"])
            + "\n" + report.get("error", "") + "\n", encoding="ascii")
    print(json.dumps(report, indent=2))
    return status


if __name__ == "__main__":
    sys.exit(main())
