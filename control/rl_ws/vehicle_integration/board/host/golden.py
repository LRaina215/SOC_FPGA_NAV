"""Strict sample-major golden memories and bit-exact result checks."""
from dataclasses import dataclass
from pathlib import Path
import re

from protocol import EXPECTED_CYCLES, ProtocolError

DEFAULT_VECTOR_DIR = Path(__file__).resolve().parents[2] / "assets" / "vectors"
SPECS = {"inputs": 162, "expected_layers": 425, "expected_raw": 6,
         "expected_clipped": 6, "expected_latent": 3, "expected_saturated": 1}


@dataclass(frozen=True)
class GoldenSample:
    inputs: tuple
    layers: tuple
    raw: tuple
    actions: tuple
    latent: tuple
    saturated: int


def load_vectors(directory=DEFAULT_VECTOR_DIR):
    directory = Path(directory)
    memories = {}
    count = None
    for name, stride in SPECS.items():
        path = directory / (name + ".mem")
        words = path.read_text(encoding="ascii").split()
        if not words or any(re.fullmatch(r"[0-9a-fA-F]{6}", word) is None for word in words):
            raise ValueError(f"{path}: expected nonempty fixed six-digit hexadecimal words")
        samples, remainder = divmod(len(words), stride)
        if remainder or (count is not None and samples != count):
            raise ValueError(f"{path}: inconsistent count {len(words)} (stride {stride}, samples {count})")
        count = samples
        values = [int(word, 16) for word in words]
        memories[name] = tuple(v - (1 << 24) if v & (1 << 23) else v for v in values)
    if any(value not in (0, 1) for value in memories["expected_saturated"]):
        raise ValueError("expected_saturated.mem must contain only 000000 or 000001")
    result = []
    for index in range(count):
        row = {name: values[index * SPECS[name]:(index + 1) * SPECS[name]] for name, values in memories.items()}
        if row["expected_layers"][192:195] != row["expected_latent"]:
            raise ValueError(f"sample {index}: latent inconsistent with layer trace")
        if row["expected_layers"][419:425] != row["expected_raw"]:
            raise ValueError(f"sample {index}: raw actions inconsistent with layer trace")
        if tuple(max(-4096, min(4096, v)) for v in row["expected_raw"]) != row["expected_clipped"]:
            raise ValueError(f"sample {index}: clipped actions inconsistent with raw actions")
        result.append(GoldenSample(row["inputs"], row["expected_layers"], row["expected_raw"],
                                   row["expected_clipped"], row["expected_latent"], row["expected_saturated"][0]))
    return tuple(result)


def select_samples(vectors, start_index=0, samples=1):
    if start_index < 0 or samples < 1 or start_index + samples > len(vectors):
        raise ValueError(f"invalid sample range start={start_index}, samples={samples}, available={len(vectors)}")
    return tuple(enumerate(vectors[start_index:start_index + samples], start_index))


def compare_result(result, golden, initial_sticky=None):
    """Return next sticky baseline, validating unknown startup without assuming reset."""
    if result.cycles != EXPECTED_CYCLES:
        raise ProtocolError(f"cycles {result.cycles}, expected {EXPECTED_CYCLES}")
    if result.saturated != golden.saturated:
        raise ProtocolError(f"saturated {result.saturated}, expected {golden.saturated}")
    if result.flags & ~0x10:
        raise ProtocolError(f"unexpected error flags 0x{result.flags:02x} (only bit4 is permitted)")
    sticky = bool(result.flags & 0x10)
    if golden.saturated and not sticky:
        raise ProtocolError("saturation must set sticky flag bit4")
    if initial_sticky is not None and sticky != (bool(initial_sticky) or bool(golden.saturated)):
        raise ProtocolError("sticky saturation flag bit4 mismatch")
    for name in ("layers", "latent", "actions"):
        actual, expected = getattr(result, name), getattr(golden, name)
        if len(actual) != len(expected):
            raise ProtocolError(f"{name} count {len(actual)}, expected {len(expected)}")
        for index, (got, want) in enumerate(zip(actual, expected)):
            if got != want:
                raise ProtocolError(f"{name}[{index}] {got} (0x{got & 0xFFFFFF:06x}), expected {want} (0x{want & 0xFFFFFF:06x})")
    return sticky
