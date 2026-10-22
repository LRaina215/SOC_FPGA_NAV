"""Dependency-free, fragmented serial model for host exchange tests, not RTL proof."""
from protocol import (Packet, encode, decode, ProtocolError, INFER, PING, PONG,
                      RESULT, ERROR, DIGEST, EXPECTED_CYCLES, pack_s24)


def result_packet(frame_id, sample, sticky=False):
    payload = (DIGEST + EXPECTED_CYCLES.to_bytes(4, "little")
               + bytes((0x10 if sticky or sample.saturated else 0, sample.saturated))
               + pack_s24(sample.actions) + pack_s24(sample.latent) + pack_s24(sample.layers))
    return Packet(RESULT, frame_id, payload)


class MockSerial:
    """Reply from packaged goldens, recording requests and modem control ordering."""
    def __init__(self, samples=(), fragment=7, response_mutator=None, **kwargs):
        self.samples = list(samples)
        self.fragment = fragment
        self.response_mutator = response_mutator
        self.rx = bytearray()
        self.tx = bytearray()
        self.requests = []
        self.events = []
        self.is_open = False
        self._dtr = True
        self._rts = True
        self.port = kwargs.get("port")
        self.last_accepted = None
        self.sticky = False
        self.core_writes = 0

    @property
    def dtr(self):
        return self._dtr

    @dtr.setter
    def dtr(self, value):
        self._dtr = value
        self.events.append(("dtr", value))

    @property
    def rts(self):
        return self._rts

    @rts.setter
    def rts(self, value):
        self._rts = value
        self.events.append(("rts", value))

    def open(self):
        self.events.append(("open", self.port, self.dtr, self.rts))
        self.is_open = True

    def close(self):
        self.events.append(("close",))
        self.is_open = False

    def flush(self):
        pass

    def write(self, data):
        self.tx.extend(data)
        try:
            request = decode(bytes(data))
        except ProtocolError:
            frame_id = int.from_bytes(data[6:10], "little") if len(data) >= 10 else 0
            reply = Packet(ERROR, frame_id, b"\x04")
        else:
            self.requests.append(request)
            if request.type == PING:
                reply = Packet(PONG, request.frame_id, DIGEST)
            elif request.type == INFER and request.frame_id == self.last_accepted:
                reply = Packet(ERROR, request.frame_id, b"\x07")
            elif request.type == INFER and self.samples:
                sample = self.samples.pop(0)
                if request.payload != pack_s24(sample.inputs):
                    raise AssertionError("mock received unexpected inference inputs")
                self.core_writes += 162
                self.last_accepted = request.frame_id
                self.sticky |= bool(sample.saturated)
                reply = result_packet(request.frame_id, sample, self.sticky)
            else:
                reply = Packet(ERROR, request.frame_id, b"\x02")
        wire = encode(reply)
        self.rx.extend(self.response_mutator(wire) if self.response_mutator else wire)
        return len(data)

    def read(self, size):
        size = min(size, self.fragment, len(self.rx))
        output = bytes(self.rx[:size])
        del self.rx[:size]
        return output
