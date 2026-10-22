"""Single in-flight serial exchange, plus offline captured-packet validation."""
import json
from pathlib import Path
import time

from protocol import (Packet, PacketParser, PING, PONG, INFER, RESULT, encode,
                      pack_s24, parse_result, validate_reply, PacketTimeout,
                      ProtocolError, DIGEST, decode)
from golden import compare_result, load_vectors, select_samples


class SerialExchange:
    def __init__(self, port, timeout=1.0, baudrate=115200):
        if not port:
            raise ValueError("--port is required for hardware mode")
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("hardware mode requires pyserial; install with pip install pyserial") from exc
        self._serial = serial.Serial(port=None, baudrate=baudrate, bytesize=8,
                                     parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                     timeout=0.02, write_timeout=timeout, dsrdtr=False, rtscts=False)
        self._serial.port = port
        # Set modem controls before opening. No reset command or port scanning is performed.
        self._serial.dtr = False
        self._serial.rts = False
        self._serial.open()
        self.timeout = timeout
        self.parser = PacketParser(inter_byte_timeout=timeout)
        self.pending = []

    def close(self):
        if self._serial.is_open:
            self._serial.close()

    def exchange(self, packet, expected_type):
        if self.pending or self.parser.buffer or getattr(self._serial, "in_waiting", 0):
            raise ProtocolError("unsolicited/queued reply before request; no new request sent")
        wire = encode(packet)
        started = time.monotonic()
        written = self._serial.write(wire)
        if written != len(wire):
            raise ProtocolError(f"incomplete serial write: {written}/{len(wire)} bytes; no retry")
        self._serial.flush()
        while time.monotonic() - started < self.timeout:
            chunk = self._serial.read(4096)
            if chunk:
                replies = self.parser.feed(chunk)
                if replies:
                    reply = replies[0]
                    self.pending.extend(replies[1:])
                    validate_reply(reply, expected_type, packet.frame_id)
                    return reply, time.monotonic() - started
        raise PacketTimeout(f"timeout waiting for frame {packet.frame_id}")


def ping(exchange, frame_id=0):
    reply, elapsed = exchange.exchange(Packet(PING, frame_id), PONG)
    if reply.payload != DIGEST:
        raise ProtocolError("PING digest mismatch")
    return elapsed


def infer(exchange, frame_id, sample):
    payload = pack_s24(sample.inputs)
    if len(payload) != 486:
        raise ValueError(f"INFER payload is {len(payload)} bytes")
    reply, elapsed = exchange.exchange(Packet(INFER, frame_id, payload), RESULT)
    return parse_result(reply, frame_id), elapsed


def _capture_bytes(path):
    text = Path(path).read_text(encoding="ascii")
    compact = "".join(text.split())
    if compact.lower().startswith("0x"):
        compact = compact[2:]
    if any(char not in "0123456789abcdefABCDEF" for char in compact) or len(compact) % 2:
        raise ValueError(f"{path}: expected hexadecimal wire bytes")
    return bytes.fromhex(compact)


def _capture_metadata(path):
    candidates = [Path(str(path) + ".json"), Path(path).with_suffix(".json")]
    for candidate in candidates:
        if candidate.exists():
            data = json.loads(candidate.read_text(encoding="utf-8"))
            if not isinstance(data, dict):
                raise ValueError(f"{candidate}: metadata must be an object")
            return data
    raise ValueError(f"missing capture metadata sidecar; expected {candidates[0]} or {candidates[1]}")


def check_capture(path, vector_dir, sample_index, expected_frame_id=None, initial_sticky=None):
    metadata = _capture_metadata(path)
    if metadata.get("sample_index") is not None and int(metadata["sample_index"]) != sample_index:
        raise ProtocolError("capture metadata sample_index mismatch")
    if metadata.get("frame_id") is not None:
        metadata_frame = int(metadata["frame_id"])
        if expected_frame_id is not None and metadata_frame != expected_frame_id:
            raise ProtocolError("capture metadata frame_id mismatch")
        expected_frame_id = metadata_frame
    if expected_frame_id is None:
        raise ValueError("capture metadata must include frame_id or caller must provide --frame-id")
    packet = decode(_capture_bytes(path))
    result = parse_result(packet, expected_frame_id)
    vectors = load_vectors(vector_dir)
    selected = select_samples(vectors, sample_index, 1)[0][1]
    compare_result(result, selected, initial_sticky)
    return {"sample_index": sample_index, "frame_id": expected_frame_id,
            "cycles": result.cycles, "flags": result.flags, "saturated": result.saturated,
            "layers": len(result.layers), "latent": len(result.latent), "actions": len(result.actions),
            "metadata": metadata}
