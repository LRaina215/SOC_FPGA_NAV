import contextlib
from dataclasses import replace
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

HOST = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HOST))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from protocol import (MAGIC, DIGEST, Packet, PacketParser, ProtocolError, PacketTimeout,
                      RemoteError, INFER, PING, PONG, RESULT, ERROR, crc16, encode, decode,
                      pack_s24, unpack_s24, parse_result, validate_reply)
from golden import DEFAULT_VECTOR_DIR, SPECS, load_vectors, select_samples, compare_result
from transport import SerialExchange, ping, infer, check_capture
from client import main
from mock_serial import MockSerial, result_packet


def raw_packet(version=1, kind=PING, length=0, frame_id=5, payload=b""):
    body = struct.pack("<BBHI", version, kind, length, frame_id) + payload
    return MAGIC + body + struct.pack("<H", crc16(body))


def fake_serial(mock):
    return types.SimpleNamespace(Serial=lambda **kwargs: mock, PARITY_NONE="N", STOPBITS_ONE=1)


class CodecTests(unittest.TestCase):
    def test_crc_known_answer(self):
        self.assertEqual(crc16(b"123456789"), 0x29B1)

    def test_signed24_extrema_and_order(self):
        values = (-8388608, -1, 0, 1, 8388607)
        wire = bytes.fromhex("000080 ffffff 000000 010000 ffff7f")
        self.assertEqual(pack_s24(values), wire)
        self.assertEqual(unpack_s24(wire), values)
        for bad in (-8388609, 8388608, 0.5):
            with self.assertRaises(ValueError):
                pack_s24((bad,))
        with self.assertRaises(ProtocolError):
            unpack_s24(b"\x00")

    def test_header_crc_byte_order(self):
        wire = encode(Packet(PING, 0x78563412))
        self.assertEqual(wire[:10], bytes.fromhex("a55a0102000012345678"))
        self.assertEqual(int.from_bytes(wire[-2:], "little"), crc16(wire[2:-2]))
        self.assertEqual(decode(wire), Packet(PING, 0x78563412))
        wire = encode(Packet(INFER, 1, pack_s24((0,) * 162)))
        self.assertEqual(wire[4:6], b"\xe6\x01")
        self.assertEqual(len(wire), 498)

    def test_invalid_wire(self):
        cases = [b"", b"\0" * 12, encode(Packet(PING, 1))[:-1],
                 raw_packet(version=2), raw_packet(kind=0x33), raw_packet(length=65535),
                 raw_packet(kind=PONG, length=0), encode(Packet(PING, 1)) + b"\0"]
        for case in cases:
            with self.subTest(case=case):
                with self.assertRaises(ProtocolError):
                    decode(case)
        with self.assertRaises(ProtocolError):
            encode(Packet(INFER, 1, b""))

    def test_crc_corruption(self):
        wire = bytearray(encode(Packet(PONG, 1, DIGEST)))
        for index in (2, 6, 10, len(wire) - 1):
            corrupt = wire.copy()
            corrupt[index] ^= 1
            with self.assertRaises(ProtocolError):
                decode(bytes(corrupt))


class ParserTests(unittest.TestCase):
    def test_every_split_and_byte_fragment(self):
        wire = encode(Packet(PONG, 123, DIGEST))
        for split in range(len(wire) + 1):
            parser = PacketParser()
            packets = parser.feed(wire[:split]) + parser.feed(wire[split:])
            self.assertEqual(packets, [Packet(PONG, 123, DIGEST)])
        parser = PacketParser()
        packets = []
        for byte in wire:
            packets.extend(parser.feed(bytes((byte,))))
        self.assertEqual(len(packets), 1)

    def test_noise_magic_overlap_multiple(self):
        parser = PacketParser()
        packets = parser.feed(b"noise\xa5" + encode(Packet(PING, 1)) + encode(Packet(PING, 2)))
        self.assertEqual([p.frame_id for p in packets], [1, 2])
        self.assertFalse(parser.buffer)

    def test_corrupt_resync(self):
        parser = PacketParser()
        corrupt = bytearray(encode(Packet(PONG, 1, DIGEST)))
        corrupt[-1] ^= 1
        with self.assertRaises(ProtocolError):
            parser.feed(bytes(corrupt) + encode(Packet(PING, 2)))
        self.assertEqual(parser.feed(b""), [Packet(PING, 2)])

    def test_oversize_resync(self):
        parser = PacketParser()
        with self.assertRaises(ProtocolError):
            parser.feed(raw_packet(length=65535) + encode(Packet(PING, 3)))
        self.assertEqual(parser.feed(b""), [Packet(PING, 3)])
        self.assertLess(len(parser.buffer), 1340 + 12)

    def test_inter_byte_timeout_and_recovery(self):
        parser = PacketParser(inter_byte_timeout=0.5)
        parser.feed(b"\xa5\x5a\x01", now=0)
        with self.assertRaises(PacketTimeout):
            parser.expire(now=0.6)
        self.assertFalse(parser.buffer)
        self.assertEqual(parser.feed(encode(Packet(PING, 3)), now=0.7), [Packet(PING, 3)])

    def test_invalid_header_types_versions_lengths(self):
        for wire in (raw_packet(version=2), raw_packet(kind=0), raw_packet(length=1)):
            with self.assertRaises(ProtocolError):
                PacketParser().feed(wire)


class GoldenTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.nominal = load_vectors(DEFAULT_VECTOR_DIR)
        cls.stress = load_vectors(DEFAULT_VECTOR_DIR / "stress")

    def test_real_mem_counts_and_stress(self):
        self.assertEqual(len(self.nominal), 256)
        self.assertEqual(len(self.stress), 8)
        self.assertEqual(self.stress[1].saturated, 1)
        self.assertEqual(len(self.nominal[0].layers), 425)

    def test_all_outputs_and_mutations(self):
        sample = self.nominal[0]
        result = parse_result(result_packet(1, sample), 1)
        self.assertFalse(compare_result(result, sample, False))
        for field in ("layers", "latent", "actions"):
            values = getattr(result, field)
            for index in range(len(values)):
                mutated = list(values)
                mutated[index] ^= 1
                with self.subTest(field=field, index=index):
                    with self.assertRaises(ProtocolError):
                        compare_result(replace(result, **{field: tuple(mutated)}), sample, False)

    def test_metadata_digest_frame_cycles_flags(self):
        sample = self.nominal[0]
        packet = result_packet(1, sample)
        for mutated in (replace(packet, frame_id=2), replace(packet, version=2),
                        replace(packet, type=PONG), replace(packet, payload=packet.payload[:-1]),
                        replace(packet, payload=b"\0" * 32 + packet.payload[32:])):
            with self.assertRaises(ProtocolError):
                parse_result(mutated, 1)
        result = parse_result(packet, 1)
        for changes in ({"cycles": 81999}, {"saturated": 1}, {"saturated": 2},
                        {"flags": 1}, {"flags": 2}, {"flags": 4}, {"flags": 8},
                        {"flags": 32}, {"flags": 16}):
            with self.assertRaises(ProtocolError):
                compare_result(replace(result, **changes), sample, False)
        for code in range(1, 9):
            with self.assertRaises(RemoteError) as error:
                validate_reply(Packet(ERROR, 1, bytes((code,))), RESULT, 1)
            self.assertEqual(error.exception.code, code)

    def test_sticky_unknown_then_persistent(self):
        nominal = parse_result(result_packet(1, self.nominal[0], True), 1)
        self.assertTrue(compare_result(nominal, self.nominal[0], None))
        saturated = parse_result(result_packet(2, self.stress[1]), 2)
        self.assertTrue(compare_result(saturated, self.stress[1], False))
        with self.assertRaises(ProtocolError):
            compare_result(replace(saturated, flags=0), self.stress[1], None)
        with self.assertRaises(ProtocolError):
            compare_result(replace(nominal, flags=0), self.nominal[0], True)

    def test_invalid_sample_ranges(self):
        for start, count in ((-1, 1), (0, 0), (256, 1), (255, 2)):
            with self.assertRaises(ValueError):
                select_samples(self.nominal, start, count)

    def test_mem_width_count_missing_and_saturation(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            for name, stride in SPECS.items():
                (path / (name + ".mem")).write_text("000000\n" * stride, encoding="ascii")
            self.assertEqual(len(load_vectors(path)), 1)
            for bad in ("00000", "0000000", "xxxxxx", "", "000000\n000000"):
                (path / "expected_saturated.mem").write_text(bad, encoding="ascii")
                with self.assertRaises(ValueError):
                    load_vectors(path)
            (path / "expected_saturated.mem").write_text("000002", encoding="ascii")
            with self.assertRaises(ValueError):
                load_vectors(path)
            (path / "expected_saturated.mem").unlink()
            with self.assertRaises(FileNotFoundError):
                load_vectors(path)

    def test_offline_capture_and_sidecar(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "reply.hex"
            path.write_text(encode(result_packet(9, self.stress[1])).hex(" "), encoding="ascii")
            metadata = path.with_suffix(".json")
            metadata.write_text(json.dumps({"frame_id": 9, "sample_index": 1}), encoding="ascii")
            result = check_capture(path, DEFAULT_VECTOR_DIR / "stress", 1, initial_sticky=0)
            self.assertEqual(result["layers"], 425)
            with self.assertRaises(ProtocolError):
                check_capture(path, DEFAULT_VECTOR_DIR / "stress", 1, expected_frame_id=8)
            with self.assertRaises(ProtocolError):
                check_capture(path, DEFAULT_VECTOR_DIR / "stress", 0)
            path.write_text(encode(result_packet(9, self.stress[1])).hex()[:-2], encoding="ascii")
            with self.assertRaises(ProtocolError):
                check_capture(path, DEFAULT_VECTOR_DIR / "stress", 1)


class SerialTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.nominal = load_vectors(DEFAULT_VECTOR_DIR)
        cls.stress = load_vectors(DEFAULT_VECTOR_DIR / "stress")

    def test_end_to_end_nominal_and_stress_first_frames(self):
        for vectors in (self.nominal, self.stress):
            mock = MockSerial(vectors[:3], fragment=7)
            with patch.dict(sys.modules, {"serial": fake_serial(mock)}):
                exchange = SerialExchange("explicit-mock-port")
                elapsed = ping(exchange, 100)
                self.assertGreaterEqual(elapsed, 0)
                sticky = False
                for index, sample in enumerate(vectors[:3]):
                    result, elapsed = infer(exchange, 101 + index, sample)
                    sticky = compare_result(result, sample, sticky)
                    self.assertGreaterEqual(elapsed, 0)
                exchange.close()
            self.assertEqual([p.type for p in mock.requests], [PING, INFER, INFER, INFER])
            self.assertEqual([p.frame_id for p in mock.requests], [100, 101, 102, 103])
            self.assertEqual(mock.events[:3], [("dtr", False), ("rts", False), ("open", "explicit-mock-port", False, False)])
            self.assertEqual(mock.core_writes, 486)

    def test_bad_crc_does_not_write_or_accept_and_duplicate_last_only(self):
        mock = MockSerial(self.nominal[:3])
        request = encode(Packet(INFER, 3, pack_s24(self.nominal[0].inputs)))
        corrupt = bytearray(request)
        corrupt[-1] ^= 1
        mock.write(bytes(corrupt))
        self.assertEqual(mock.core_writes, 0)
        self.assertIsNone(mock.last_accepted)
        self.assertEqual(decode(bytes(mock.rx)).payload, b"\x04")
        mock.rx.clear()
        mock.write(request)
        self.assertEqual(mock.last_accepted, 3)
        mock.rx.clear()
        mock.write(request)
        self.assertEqual(decode(bytes(mock.rx)).payload, b"\x07")
        self.assertEqual(mock.core_writes, 162)
        mock.rx.clear()
        mock.write(encode(Packet(INFER, 4, pack_s24(self.nominal[1].inputs))))
        mock.rx.clear()
        mock.write(encode(Packet(INFER, 3, pack_s24(self.nominal[2].inputs))))
        self.assertEqual(mock.last_accepted, 3)
        self.assertEqual(mock.core_writes, 486)

    def test_reply_corruption_wrongframe_digest_timeout(self):
        def corrupt(wire):
            return wire[:-1] + bytes((wire[-1] ^ 1,))
        def wrongframe(wire):
            return encode(replace(decode(wire), frame_id=9))
        def wrongdigest(wire):
            return encode(replace(decode(wire), payload=b"\0" * 32))
        for mutation, error in ((corrupt, ProtocolError), (wrongframe, ProtocolError),
                                (wrongdigest, ProtocolError), (lambda wire: b"", PacketTimeout),
                                (lambda wire: wire[:5], PacketTimeout)):
            mock = MockSerial(response_mutator=mutation)
            with patch.dict(sys.modules, {"serial": fake_serial(mock)}):
                exchange = SerialExchange("mock", timeout=0.01)
                with self.assertRaises(error):
                    ping(exchange)
                exchange.close()

    def test_client_mock_reports_in_temporary_dir(self):
        mock = MockSerial(self.stress[:3])
        with tempfile.TemporaryDirectory() as directory:
            with patch.dict(sys.modules, {"serial": fake_serial(mock)}), contextlib.redirect_stdout(io.StringIO()):
                code = main(["--port", "mock", "--samples", "3", "--frame-id-start", "7",
                             "--vector-dir", str(DEFAULT_VECTOR_DIR / "stress"), "--initial-sticky-saturation", "0",
                             "--report-dir", directory])
            self.assertEqual(code, 0)
            files = list(Path(directory).glob("*.json"))
            self.assertEqual(len(files), 1)
            report = json.loads(files[0].read_text())
            self.assertEqual(report["mode"], "hardware")
            self.assertFalse(report["physical_board_claim"])
            self.assertEqual([row["frame_id"] for row in report["results"]], [7, 8, 9])
            self.assertIn("host_roundtrip_seconds", report["results"][0])
            self.assertEqual(len(list(Path(directory).glob("*.log"))), 1)

    def test_dry_run_no_serial_and_no_default_reports(self):
        with patch.dict(sys.modules, {"serial": None}), contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(main(["--dry-run", "--samples", "2", "--start-index", "4", "--no-report"]), 0)
        report = json.loads(output.getvalue())
        self.assertEqual(report["mode"], "dry-run")
        self.assertEqual(report["status"], "preflight-only")
        self.assertEqual([row["sample_index"] for row in report["results"]], [4, 5])
        self.assertNotIn("report_json", report)

    def test_cli_rejects_no_port_and_frame_wrap(self):
        for argv in ([], ["--dry-run", "--port", "mock"],
                     ["--dry-run", "--samples", "2", "--frame-id-start", str(0xFFFFFFFF)]):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                main(argv)
            self.assertEqual(error.exception.code, 2)

    def test_short_write_and_queued_reply_block_new_request(self):
        mock = MockSerial()
        mock.write = lambda data: len(data) - 1
        with patch.dict(sys.modules, {"serial": fake_serial(mock)}):
            exchange = SerialExchange("mock")
            with self.assertRaisesRegex(ProtocolError, "incomplete serial write"):
                ping(exchange)
            exchange.close()
        mock = MockSerial(self.nominal[:1], fragment=4096,
                          response_mutator=lambda wire: wire + encode(Packet(ERROR, 7, b"\x06")))
        with patch.dict(sys.modules, {"serial": fake_serial(mock)}):
            exchange = SerialExchange("mock")
            ping(exchange)
            with self.assertRaisesRegex(ProtocolError, "queued reply"):
                infer(exchange, 1, self.nominal[0])
            self.assertEqual(len(mock.requests), 1)
            exchange.close()

    def test_optional_serial_missing(self):
        with patch.dict(sys.modules, {"serial": None}):
            with self.assertRaisesRegex(RuntimeError, "pyserial"):
                SerialExchange("mock")
        with self.assertRaises(ValueError):
            SerialExchange("")

    def test_cli_offline_has_no_serial(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.hex"
            path.write_text(encode(result_packet(1, self.nominal[0])).hex(), encoding="ascii")
            path.with_suffix(".json").write_text(json.dumps({"frame_id": 1, "sample_index": 0}))
            with patch.dict(sys.modules, {"serial": None}), contextlib.redirect_stdout(io.StringIO()) as output:
                self.assertEqual(main(["--offline", str(path), "--initial-sticky-saturation", "0", "--no-report"]), 0)
            self.assertEqual(json.loads(output.getvalue())["status"], "validated")

    def test_ping_failure_sends_no_infer(self):
        mock = MockSerial(self.nominal[:1], response_mutator=lambda wire: encode(Packet(PONG, 0, b"\0" * 32)))
        with patch.dict(sys.modules, {"serial": fake_serial(mock)}), contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(main(["--port", "mock", "--no-report"]), 1)
        self.assertEqual([p.type for p in mock.requests], [PING])
        self.assertFalse(mock.is_open)


if __name__ == "__main__":
    unittest.main()
