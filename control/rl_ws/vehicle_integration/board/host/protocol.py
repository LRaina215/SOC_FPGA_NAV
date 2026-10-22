"""UART inference framing and signed24 codecs; no hardware dependencies."""
from dataclasses import dataclass
import struct
import time

MAGIC = b"\xa5\x5a"
VERSION = 1
INFER, PING, RESULT, PONG, ERROR = 0x01, 0x02, 0x81, 0x82, 0xFF
LENGTHS = {INFER: 486, PING: 0, RESULT: 1340, PONG: 32, ERROR: 1, 3: 0, 0x83: 64}
DIGEST = bytes.fromhex("29b12e353b84380fa57ed7c3f8928fb1d51e9df99c0fa01bd3f4aec2de5a8c03")
EXPECTED_CYCLES = 81998
ERROR_NAMES = {1: "version", 2: "type", 3: "length", 4: "CRC",
               5: "rx frame/timeout", 6: "busy", 7: "duplicate last accepted INFER", 8: "core error"}


class ProtocolError(ValueError):
    pass


class PacketTimeout(TimeoutError):
    pass


class RemoteError(ProtocolError):
    def __init__(self, code):
        self.code = code
        super().__init__(f"board ERROR {code}: {ERROR_NAMES.get(code, 'unknown code')}")


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def pack_s24(values):
    output = bytearray()
    for value in values:
        if not isinstance(value, int) or not -(1 << 23) <= value < (1 << 23):
            raise ValueError(f"signed24 out of range: {value}")
        output.extend((value & 0xFFFFFF).to_bytes(3, "little"))
    return bytes(output)


def unpack_s24(data):
    if len(data) % 3:
        raise ProtocolError("signed24 byte count must be divisible by three")
    return tuple(int.from_bytes(data[i:i + 3], "little", signed=True) for i in range(0, len(data), 3))


@dataclass(frozen=True)
class Packet:
    type: int
    frame_id: int
    payload: bytes = b""
    version: int = VERSION


def validate_packet(packet):
    if packet.version != VERSION:
        raise ProtocolError(f"version {packet.version}, expected {VERSION}")
    if packet.type not in LENGTHS:
        raise ProtocolError(f"unknown type 0x{packet.type:02x}")
    if len(packet.payload) != LENGTHS[packet.type]:
        raise ProtocolError(f"length {len(packet.payload)}, expected {LENGTHS[packet.type]}")
    if not 0 <= packet.frame_id <= 0xFFFFFFFF:
        raise ProtocolError("frame_id outside uint32")


def encode(packet):
    validate_packet(packet)
    body = struct.pack("<BBHI", packet.version, packet.type, len(packet.payload), packet.frame_id) + packet.payload
    return MAGIC + body + struct.pack("<H", crc16(body))


def decode(data):
    if len(data) < 12 or data[:2] != MAGIC:
        raise ProtocolError("missing magic or truncated packet")
    version, kind, length, frame_id = struct.unpack("<BBHI", data[2:10])
    if length > max(LENGTHS.values()):
        raise ProtocolError("oversize payload")
    if len(data) != length + 12:
        raise ProtocolError(f"wire length {len(data)}, expected {length + 12}")
    if crc16(data[2:-2]) != int.from_bytes(data[-2:], "little"):
        raise ProtocolError("CRC mismatch")
    packet = Packet(kind, frame_id, data[10:-2], version)
    validate_packet(packet)
    return packet


class PacketParser:
    """Incremental, bounded frame parser. Malformed candidates fail visibly.

    After an error the next feed can resynchronize at the next magic. A partial
    candidate is discarded on inter-byte timeout; callers decide whether to retry.
    """
    def __init__(self, inter_byte_timeout=0.5, clock=time.monotonic):
        if inter_byte_timeout <= 0:
            raise ValueError("inter-byte timeout must be positive")
        self.buffer = bytearray()
        self.clock = clock
        self.inter_byte_timeout = inter_byte_timeout
        self.last_byte = None

    def expire(self, now=None):
        now = self.clock() if now is None else now
        if self.buffer and self.last_byte is not None and now - self.last_byte >= self.inter_byte_timeout:
            self.buffer.clear()
            self.last_byte = None
            raise PacketTimeout("partial packet inter-byte timeout")

    def feed(self, data, now=None):
        now = self.clock() if now is None else now
        self.expire(now)
        if data:
            self.buffer.extend(data)
            self.last_byte = now
        packets = []
        while self.buffer:
            position = self.buffer.find(MAGIC)
            if position < 0:
                keep = self.buffer[-1:] if self.buffer[-1] == MAGIC[0] else b""
                self.buffer[:] = keep
                break
            if position:
                del self.buffer[:position]
            if len(self.buffer) < 10:
                break
            version, kind, length, _ = struct.unpack("<BBHI", self.buffer[2:10])
            if version != VERSION or kind not in LENGTHS or length != LENGTHS.get(kind) or length > 1340:
                del self.buffer[:2]
                raise ProtocolError(f"invalid header version={version} type=0x{kind:02x} length={length}")
            size = length + 12
            if len(self.buffer) < size:
                break
            wire = bytes(self.buffer[:size])
            try:
                packet = decode(wire)
            except ProtocolError:
                del self.buffer[:2]
                raise
            del self.buffer[:size]
            packets.append(packet)
        return packets


def validate_reply(packet, kind, frame_id):
    validate_packet(packet)
    if packet.frame_id != frame_id:
        raise ProtocolError(f"reply frame_id {packet.frame_id}, expected {frame_id}")
    if packet.type == ERROR:
        raise RemoteError(packet.payload[0])
    if packet.type != kind:
        raise ProtocolError(f"reply type 0x{packet.type:02x}, expected 0x{kind:02x}")
    if packet.type in (PONG, RESULT) and packet.payload[:32] != DIGEST:
        raise ProtocolError("checkpoint digest mismatch")


@dataclass(frozen=True)
class InferenceResult:
    cycles: int
    flags: int
    saturated: int
    actions: tuple
    latent: tuple
    layers: tuple


def parse_result(packet, frame_id):
    validate_reply(packet, RESULT, frame_id)
    payload = packet.payload
    return InferenceResult(int.from_bytes(payload[32:36], "little"), payload[36], payload[37],
                           unpack_s24(payload[38:56]), unpack_s24(payload[56:65]), unpack_s24(payload[65:1340]))
