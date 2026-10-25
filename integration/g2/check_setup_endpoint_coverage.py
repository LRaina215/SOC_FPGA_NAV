"""Count setup endpoint coverage in a Gowin .tr Path Slacks Table.

Read-only: no EDA process, project, or build input is opened or modified.
Endpoint key: (To Clock including printed edge, To Node), matching the
vendor's same-clock/same-object rule as represented in the public report.
Uses only the Python standard library.
"""
import argparse
import json
import re
from collections import Counter
from pathlib import Path


def scan_report(path):
    text = path.read_text(encoding="utf-8-sig", errors="replace")
    total_match = re.search(r"<Numbers of Setup Violated Endpoints>\s*:\s*(\d+)", text)
    expected = int(total_match.group(1)) if total_match else None
    begin = re.search(r"(?m)^3\.1\s+Path Slacks Table\s*$", text)
    if not begin:
        raise ValueError("Could not locate the public Path Slacks Table; unsupported .tr layout.")
    finish = re.search(r"(?m)^3\.2\s+", text[begin.end():])
    if not finish:
        raise ValueError("Could not locate the end of Path Slacks Table; unsupported .tr layout.")
    table = text[begin.end(): begin.end() + finish.start()]
    row_pattern = re.compile(r"^\s*(\d+)\s+(-?\d+(?:\.\d+)?)\s+(\S+)\s+(\S+)\s+(\S+)\s+(\S+)\s+")
    reports = []
    current = None
    unique_failures = {}
    setup_rows = 0
    failed_rows = 0
    for line in table.splitlines():
        command = re.match(r"\s*<Report Command>\s*:\s*(.*)", line)
        if command:
            mode = re.search(r"-(setup|hold|recovery|removal)\b", command.group(1))
            current = {"command": command.group(1), "analysis": mode.group(1) if mode else None, "rows": []}
            reports.append(current)
            continue
        row = row_pattern.match(line)
        if current is None or current["analysis"] != "setup" or not row:
            continue
        number, slack, startpoint, endpoint, launch_clock, latch_clock = row.groups()
        item = {"number": int(number), "slack_ns": float(slack), "printed_negative_slack": slack.startswith("-"), "from_node": startpoint,
                "to_node": endpoint, "from_clock": launch_clock, "to_clock": latch_clock}
        current["rows"].append(item)
        setup_rows += 1
        if item["printed_negative_slack"]:
            failed_rows += 1
            key = (latch_clock, endpoint)
            old = unique_failures.get(key)
            if old is None or item["slack_ns"] < old["slack_ns"]:
                unique_failures[key] = item
    output_reports = []
    for report in reports:
        if report["analysis"] != "setup":
            continue
        rows = report["rows"]
        failures = [row for row in rows if row["printed_negative_slack"]]
        unique = {(row["to_clock"], row["to_node"]) for row in failures}
        max_paths = re.search(r"-max_paths\s+(\d+)\b", report["command"])
        common = re.search(r"-max_common_paths\s+(\d+)\b", report["command"])
        output_reports.append({"command": report["command"], "listed_paths": len(rows),
            "negative_slack_paths": len(failures), "unique_failed_endpoints": len(unique),
            "max_paths": int(max_paths.group(1)) if max_paths else 25,
            "max_common_paths": int(common.group(1)) if common else 1,
            "worst_listed_slack_ns": min((row["slack_ns"] for row in rows), default=None)})
    unique_count = len(unique_failures)
    if expected is None:
        status = "summary_count_missing"
    elif unique_count < expected:
        status = "partial_endpoint_coverage"
    elif unique_count == expected:
        status = "endpoint_count_matches_summary"
    else:
        status = "endpoint_count_exceeds_summary_check_clock_edge_grouping"
    result = {"report": str(path.resolve()), "summary_setup_violated_endpoints": expected,
        "listed_setup_path_rows": setup_rows, "listed_negative_setup_path_rows": failed_rows,
        "unique_failed_setup_endpoints_in_report_union": unique_count,
        "repeated_negative_setup_path_rows": failed_rows - unique_count,
        "unlisted_failed_endpoint_count_if_same_vendor_grouping": max(expected - unique_count, 0) if expected is not None else None,
        "coverage_fraction_if_same_vendor_grouping": unique_count / expected if expected else (1.0 if expected == 0 else None),
        "status": status, "setup_reports": output_reports,
        "unique_failures_by_latch_clock": dict(Counter(clock for clock, _ in unique_failures)),
        "interpretation": "Counts use printed (To Clock, To Node). A matching count supports complete endpoint coverage under this grouping; it does not prove timing closure, cover alternate paths, or validate a modified candidate from an older report."}
    return result, sorted(unique_failures.values(), key=lambda row: (row["slack_ns"], row["to_clock"], row["to_node"]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path, help="Gowin final timing report .tr")
    parser.add_argument("--output", type=Path, help="Optional JSON result path")
    parser.add_argument("--endpoints-out", type=Path, help="Optional TSV of unique failed setup endpoints")
    args = parser.parse_args()
    result, endpoints = scan_report(args.report)
    serialized = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.write_text(serialized, encoding="utf-8")
    if args.endpoints_out:
        lines = ["slack_ns\tto_clock\tto_node\tfrom_clock\tfrom_node"]
        for row in endpoints:
            lines.append("\t".join(str(row[key]) for key in ["slack_ns", "to_clock", "to_node", "from_clock", "from_node"]))
        args.endpoints_out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(serialized, end="")


if __name__ == "__main__":
    main()
