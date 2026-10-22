"""Reject incomplete or malformed verification inputs before simulation."""
import argparse
from pathlib import Path
import re


def check_memory(path, width, expected_count=None):
    words = path.read_text(encoding="ascii").split()
    if not words or any(re.fullmatch(r"[0-9a-fA-F]{%d}" % width, word) is None for word in words):
        raise ValueError(f"Invalid {width}-digit hex memory: {path}")
    if expected_count is not None and len(words) != expected_count:
        raise ValueError(f"Wrong memory length {path}: {len(words)}, expected {expected_count}")
    return words


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--vector-dir", type=Path, required=True)
    parser.add_argument("--samples", type=int, required=True)
    args = parser.parse_args()
    for filename, width, count in (("weights.mem", 4, 40704), ("bias.mem", 8, 425), ("elu.mem", 6, 3073)):
        check_memory(args.root / "assets" / filename, width, count)
    available = None
    for filename, stride in (("inputs.mem", 168), ("expected_layers.mem", 425), ("expected_raw.mem", 6),
                             ("expected_clipped.mem", 6), ("expected_latent.mem", 3)):
        words = check_memory(args.vector_dir / filename, 6)
        count, remainder = divmod(len(words), stride)
        if remainder or count < args.samples or (available is not None and count != available):
            raise ValueError(f"Incomplete/inconsistent vectors: {filename} has {len(words)} words")
        available = count
    saturation = args.vector_dir / "expected_saturated.mem"
    if saturation.exists():
        words = check_memory(saturation, 6, available)
        if any(int(word, 16) not in (0, 1) for word in words):
            raise ValueError("Saturation expected values must be 0/1")
    print(f"PASS PREFLIGHT complete known ROMs and vectors available={available} requested={args.samples}")


if __name__ == "__main__":
    main()
