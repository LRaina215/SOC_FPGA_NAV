"""Bit-exact offline policy reference; inference results are signed Q12 integers."""
from dataclasses import dataclass
import json
from pathlib import Path
import numpy as np

FRAC = 12
SCALE = 4096
ACT_MIN, ACT_MAX = -(1 << 23), (1 << 23) - 1
ACC_MIN, ACC_MAX = -(1 << 47), (1 << 47) - 1
LAYER_NAMES = ("encoder_0", "encoder_2", "encoder_4", "actor_0", "actor_2", "actor_4", "actor_6")
SHAPES = ((128, 140), (64, 128), (3, 64), (128, 31), (64, 128), (32, 64), (6, 32))


def quantize(values, bits=24, fraction_bits=12):
    """Nearest ties-even quantization, saturated integers and saturation count."""
    values = np.asarray(values, dtype=np.float64)
    if not np.all(np.isfinite(values)):
        raise ValueError("Quantization requires finite values")
    with np.errstate(over="ignore"):
        rounded = np.rint(values * (1 << fraction_bits))
    lo, hi = -(1 << (bits - 1)), (1 << (bits - 1)) - 1
    count = int(np.count_nonzero((rounded < lo) | (rounded > hi)))
    return np.clip(rounded, lo, hi).astype(np.int64), count


def round_shift_away(values, shift=12):
    """Signed48 nearest rounding; exact ties go away from zero."""
    values = np.asarray(values)
    if not np.issubdtype(values.dtype, np.integer):
        raise ValueError("Accumulator must be integer")
    values = values.astype(np.int64, copy=False)
    if shift < 1 or shift > 47:
        raise ValueError("Shift must be in [1, 47]")
    if np.any((values < ACC_MIN) | (values > ACC_MAX)):
        raise OverflowError("Signed48 accumulator overflow")
    magnitude = (np.abs(values) + (1 << (shift - 1))) >> shift
    return np.where(values < 0, -magnitude, magnitude)


def make_elu_table():
    return np.rint(np.expm1(-np.arange(3073, dtype=np.float64) / 256) * SCALE).astype(np.int64)


def elu_fixed(values, table):
    values = np.asarray(values, dtype=np.int64)
    index = np.minimum(3072, (-np.minimum(values, 0) + 8) >> 4)
    return np.where(values >= 0, values, table[index])


def read_mem(path, bits):
    tokens = Path(path).read_text().splitlines()
    if any(len(t) != bits // 4 for t in tokens):
        raise ValueError("Invalid memory word width")
    words = np.array([int(t, 16) for t in tokens], dtype=np.int64)
    if np.any(words >= 1 << bits):
        raise ValueError("Memory word exceeds width")
    words[words >= 1 << (bits - 1)] -= 1 << bits
    return words


def write_mem(path, values, bits):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    values = np.asarray(values, dtype=np.int64).reshape(-1)
    if np.any((values < -(1 << (bits - 1))) | (values >= 1 << (bits - 1))):
        raise ValueError("Memory values outside signed range")
    mask = (1 << bits) - 1
    path.write_text("".join(f"{int(x) & mask:0{bits // 4}x}\n" for x in values))
    np.testing.assert_array_equal(read_mem(path, bits), values)


@dataclass
class Layer:
    name: str
    weight: np.ndarray
    bias: np.ndarray
    activation: str


@dataclass
class PolicyResult:
    raw: np.ndarray
    clipped: np.ndarray
    latent: np.ndarray
    layers: dict
    diagnostics: dict

    @property
    def all_layers(self):
        return np.concatenate(list(self.layers.values()), axis=-1)


class FixedPolicy:
    def __init__(self, layers, elu_table=None):
        self.layers = list(layers)
        self.elu_table = make_elu_table() if elu_table is None else np.asarray(elu_table)
        if self.elu_table.shape != (3073,) or len(self.layers) != 7:
            raise ValueError("Expected seven layers and 3073 ELU entries")
        for layer, name, shape in zip(self.layers, LAYER_NAMES, SHAPES):
            if layer.name != name or layer.weight.shape != shape or layer.bias.shape != (shape[0],):
                raise ValueError(f"Invalid topology for {name}")
            expected = "linear" if name in ("encoder_4", "actor_6") else "elu"
            if layer.activation != expected:
                raise ValueError(f"Invalid activation for {name}")
            layer.weight = self._integer(layer.weight, -(1 << 15), (1 << 15) - 1, "weight")
            layer.bias = self._integer(layer.bias, -(1 << 31), (1 << 31) - 1, "bias")
        self.elu_table = self._integer(self.elu_table, ACT_MIN, ACT_MAX, "ELU")

    @staticmethod
    def _integer(values, lo, hi, name):
        values = np.asarray(values)
        if not np.issubdtype(values.dtype, np.integer):
            raise ValueError(f"{name} must be integer")
        if np.any((values < lo) | (values > hi)):
            raise ValueError(f"{name} outside signed range")
        return values.astype(np.int64, copy=False)

    @classmethod
    def from_npz(cls, path):
        layers = []
        with np.load(path, allow_pickle=False) as data:
            for name in LAYER_NAMES:
                key = name.replace("_", ".")
                weight, ws = quantize(data[key + ".weight"], 16, 12)
                bias, bs = quantize(data[key + ".bias"], 32, 24)
                if ws or bs:
                    raise OverflowError(f"Parameter saturation in {name}")
                layers.append(Layer(name, weight, bias, "linear" if name in ("encoder_4", "actor_6") else "elu"))
        return cls(layers)

    @classmethod
    def from_assets(cls, directory):
        directory = Path(directory)
        manifest = json.loads((directory / "manifest.json").read_text())
        weights, biases = read_mem(directory / "weights.mem", 16), read_mem(directory / "bias.mem", 32)
        if weights.size != 40704 or biases.size != 425:
            raise ValueError("Incorrect combined parameter counts")
        layers = []
        for entry in manifest["layers"]:
            wo, wc = entry["weight_offset"], entry["weight_count"]
            bo, bc = entry["bias_offset"], entry["bias_count"]
            layers.append(Layer(entry["name"], weights[wo:wo + wc].reshape(entry["output_size"], entry["input_size"]),
                                biases[bo:bo + bc], entry["activation"]))
        return cls(layers, read_mem(directory / "elu.mem", 24))

    def infer_float(self, obs, history):
        obs, os = quantize(obs)
        history, hs = quantize(history)
        result = self.infer_fixed(obs, history)
        result.diagnostics["input_saturation"] = {"obs": os, "history": hs, "flag": bool(os or hs)}
        return result

    def infer_fixed(self, obs, history):
        obs = self._integer(obs, ACT_MIN, ACT_MAX, "obs")
        history = self._integer(history, ACT_MIN, ACT_MAX, "history")
        if obs.ndim < 1 or obs.shape[-1] != 28 or history.shape != obs.shape[:-1] + (140,):
            raise ValueError("Expected matching leading dimensions and obs28/history140")
        outputs, diagnostics = {}, {}
        x, latent = history, None
        for i, layer in enumerate(self.layers):
            if i == 3:
                x = np.concatenate((obs, latent), axis=-1)
            # With bounded signed24/signed16 inputs, every partial dot fits signed48.
            accum = x @ layer.weight.T + layer.bias
            rounded = round_shift_away(accum)
            saturation = int(np.count_nonzero((rounded < ACT_MIN) | (rounded > ACT_MAX)))
            linear = np.clip(rounded, ACT_MIN, ACT_MAX)
            x = elu_fixed(linear, self.elu_table) if layer.activation == "elu" else linear
            outputs[layer.name] = x
            diagnostics[layer.name] = {
                "accumulator_min": int(accum.min()) if accum.size else None,
                "accumulator_max": int(accum.max()) if accum.size else None,
                "rounded_min": int(rounded.min()) if rounded.size else None,
                "rounded_max": int(rounded.max()) if rounded.size else None,
                "pre_elu_saturation_count": saturation,
                "output_min": int(x.min()) if x.size else None,
                "output_max": int(x.max()) if x.size else None,
            }
            if i == 2:
                latent = x
        return PolicyResult(x, np.clip(x, -SCALE, SCALE), latent, outputs, {
            "input_saturation": {"obs": 0, "history": 0, "flag": False},
            "layers": diagnostics,
            "action_clip_count": int(np.count_nonzero((x < -SCALE) | (x > SCALE))),
        })

    __call__ = infer_float
