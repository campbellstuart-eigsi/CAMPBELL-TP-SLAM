"""Tests of the robot link protocol v1 (autonomy/protocol.py). No hardware.

    python -m unittest test_autonomy_protocol -v
"""
import random
import unittest
from pathlib import Path

from autonomy import protocol as p

VECTOR_HEADER = Path(__file__).resolve().parent / "firmware" / "tests" / "protocol_vectors.h"


def crc16_bitwise(data):
    """Independent bit-by-bit CRC-16/CCITT-FALSE for checking the table."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class CrcAndSequenceTests(unittest.TestCase):
    def test_crc_check_value(self):
        self.assertEqual(p.crc16_ccitt(b"123456789"), p.CRC16_CHECK)

    def test_crc8_check_value(self):
        self.assertEqual(p.crc8(b"123456789"), p.CRC8_CHECK)

    def test_crc_table_matches_bitwise(self):
        rng = random.Random(1)
        for size in (0, 1, 7, 64, 522):
            data = bytes(rng.randrange(256) for _ in range(size))
            self.assertEqual(p.crc16_ccitt(data), crc16_bitwise(data))

    def test_seq_newer_wraps(self):
        self.assertTrue(p.seq_newer(2, 1))
        self.assertTrue(p.seq_newer(0, 0xFFFF))
        self.assertTrue(p.seq_newer(5, 0xFFF0))
        self.assertFalse(p.seq_newer(1, 1))
        self.assertFalse(p.seq_newer(1, 2))
        self.assertFalse(p.seq_newer(0xFFF0, 5))

    def test_uptime_order_and_age_wrap(self):
        self.assertTrue(p.uptime_after(5, 0xFFFFFFF0))
        self.assertFalse(p.uptime_after(0xFFFFFFF0, 5))
        self.assertFalse(p.uptime_after(7, 7))
        self.assertEqual(p.uptime_age(10, 0xFFFFFFFB), 15)
        self.assertGreater(p.uptime_age(10, 20), p.HEARTBEAT_ECHO_MAX_MS)  # future echo

    def test_counter_wraps(self):
        counter = p.SequenceCounter(0xFFFE)
        self.assertEqual([counter.next() for _ in range(4)], [0xFFFE, 0xFFFF, 0, 1])


class MessageTests(unittest.TestCase):
    SAMPLES = (p.Hello(0xCAFEF00D), p.Heartbeat(123456), p.ScanResend(4, 2), p.Arm(), p.Disarm(), p.Stop(), p.ClearFault(),
               p.ScanRequest(), p.SAMPLE_MOVE,
               p.HelloReply(2, 100, 0xDEADBEEF, p.MAX_PAYLOAD, p.MotorDriverKind.MOCK, 99),
               p.Ack(5, p.MsgType.ARM), p.Nack(6, p.MsgType.MOVE, p.NackReason.STALE_SEQ),
               p.SAMPLE_TELEMETRY, p.MoveDone(3, p.MoveOutcome.STOPPED, 0),
               p.ScanChunk(1, 0, 2, ((0, 1000, 10), (35999, 12000, 255))))

    def test_round_trip_through_frames(self):
        decoder = p.FrameDecoder()
        for seq, message in enumerate(self.SAMPLES, start=100):
            frames = decoder.feed(p.encode(message, seq))
            self.assertEqual(len(frames), 1)
            self.assertEqual(frames[0].seq, seq)
            self.assertEqual(p.decode_message(frames[0]), message)
        self.assertEqual((decoder.crc_errors, decoder.framing_errors), (0, 0))

    def test_every_type_has_a_message(self):
        self.assertEqual(set(p.MESSAGES), set(p.MsgType))
        self.assertEqual(p.HOST_TO_ROBOT, {t for t in p.MsgType if t < 0x80})

    def test_wrong_payload_length_is_bad_length(self):
        for payload in (b"", b"\x00" * 10, b"\x00" * 15):  # v2 MOVE is 14 bytes
            with self.assertRaises(p.ProtocolError) as caught:
                p.decode_message(p.Frame(p.MsgType.MOVE, 1, payload))
            self.assertEqual(caught.exception.reason, p.NackReason.BAD_LENGTH)
        with self.assertRaises(p.ProtocolError):
            p.decode_message(p.Frame(p.MsgType.STOP, 1, b"\x00"))

    def test_unknown_type(self):
        with self.assertRaises(p.ProtocolError) as caught:
            p.decode_message(p.Frame(0x42, 1, b""))
        self.assertEqual(caught.exception.reason, p.NackReason.UNKNOWN_TYPE)

    def test_out_of_range_field_refused_when_encoding(self):
        with self.assertRaises(ValueError):
            p.Move(40000, 0, 100, 100, 1000).payload()
        with self.assertRaises(ValueError):
            p.encode_frame(p.MsgType.SCAN_CHUNK, 1, b"\x00" * (p.MAX_PAYLOAD + 1))

    def test_bad_scan_chunks(self):
        good = p.ScanChunk(1, 0, 1, ((0, 1, 2),)).payload()
        for payload in (good[:4], good + b"\x00", good[:4] + b"\x00" * 4):
            with self.assertRaises(p.ProtocolError):
                p.ScanChunk.from_payload(payload)
        with self.assertRaises(p.ProtocolError):  # chunk index >= count
            p.ScanChunk.from_payload(bytes([1, 0, 1, 1]) + good[4:])
        with self.assertRaises(p.ProtocolError):  # angle 360.00 deg
            p.ScanChunk.from_payload(good[:4] + bytes([0xA0, 0x8C, 1, 0, 2]))
        with self.assertRaises(ValueError):
            p.ScanChunk(1, 0, 1, ()).payload()


class MoveBoundsTests(unittest.TestCase):
    def move(self, **changes):
        values = dict(forward_mm=300, turn_cdeg=0, speed_mm_s=150, turn_rate_cdeg_s=9000,
                      timeout_ms=2000)
        values.update(changes)
        return p.Move(**values)

    def test_sample_and_limits_accepted(self):
        self.assertIsNone(p.validate_move(p.SAMPLE_MOVE))
        self.assertIsNone(p.validate_move(self.move(forward_mm=-300, speed_mm_s=200)))
        self.assertIsNone(p.validate_move(self.move(forward_mm=0, turn_cdeg=-9000)))

    def test_out_of_bounds_refused(self):
        cases = (dict(forward_mm=301, speed_mm_s=200), dict(forward_mm=-301, speed_mm_s=200),
                 dict(turn_cdeg=9001), dict(speed_mm_s=201), dict(turn_rate_cdeg_s=9001),
                 dict(timeout_ms=0), dict(timeout_ms=2001), dict(forward_mm=0),
                 dict(speed_mm_s=0), dict(forward_mm=0, turn_cdeg=100, turn_rate_cdeg_s=0),
                 dict(forward_mm=300, speed_mm_s=100))  # 3 s > 2 s timeout
        for case in cases:
            with self.subTest(case=case):
                self.assertEqual(p.validate_move(self.move(**case)), p.NackReason.OUT_OF_BOUNDS)

    def test_duration(self):
        self.assertAlmostEqual(p.SAMPLE_MOVE.duration_ms(), 500.0 + 200 * 1000.0 / 150)

    def test_interim_limits(self):
        # docs/decisions/2026-11-04_interim_motion_limits_and_no_http.md
        self.assertEqual((p.MAX_FORWARD_MM, p.MAX_TIMEOUT_MS), (300, 2000))


class ScanAngleTests(unittest.TestCase):
    """CCW scan angles (docs/decisions/2026-11-04_scan_angles_ccw.md)."""

    def test_ccw_axes(self):
        xy = p.points_to_sensor_xy(((0, 1000, 9), (9000, 2000, 9), (18000, 500, 9),
                                    (27000, 1500, 9), (4500, 0, 9)))
        expected = [(1.0, 0.0), (0.0, 2.0), (-0.5, 0.0), (0.0, -1.5)]   # no-echo dropped
        for (x, y), (ex, ey) in zip(xy, expected, strict=True):
            self.assertAlmostEqual(x, ex, places=9)
            self.assertAlmostEqual(y, ey, places=9)

    def test_matches_code01_for_library_angles(self):
        # library angle = 360 - LD06 physical (clockwise) angle; Code 01 converts
        # the physical angle with polar_to_cartesian(): both give the same points.
        import numpy as np
        import localisation_common as common
        part1 = common.day1("part1")
        physical_cdeg = np.arange(0, 36000, 79)
        distances = 300 + (physical_cdeg % 4000)
        library = [((36000 - a) % 36000, int(d), 0) for a, d in zip(physical_cdeg, distances)]
        ours = np.array(p.points_to_sensor_xy(library))
        x, y = part1.polar_to_cartesian(physical_cdeg / 100.0, distances.astype(float))
        np.testing.assert_allclose(ours, np.column_stack([x, y]), atol=1e-9)


class DecoderTests(unittest.TestCase):
    def check(self, vector, chunks):
        decoder = p.FrameDecoder()
        frames = [f for chunk in chunks for f in decoder.feed(chunk)]
        self.assertEqual(tuple(frames), vector.frames)
        self.assertEqual(decoder.crc_errors, vector.crc_errors)
        self.assertEqual(decoder.framing_errors, vector.framing_errors)
        self.assertLessEqual(len(decoder.buffer), p.MAX_FRAME)
        return decoder

    def test_golden_vectors_whole(self):
        for vector in p.golden_vectors():
            with self.subTest(vector=vector.name):
                self.check(vector, [vector.data])

    def test_golden_vectors_byte_at_a_time(self):
        for vector in p.golden_vectors():
            with self.subTest(vector=vector.name):
                self.check(vector, [bytes([b]) for b in vector.data])

    def test_golden_vectors_random_splits(self):
        rng = random.Random(7)
        for vector in p.golden_vectors():
            for _ in range(20):
                cuts = sorted(rng.sample(range(len(vector.data) + 1), 3))
                parts = [vector.data[a:b] for a, b in zip([0] + cuts, cuts + [len(vector.data)])]
                with self.subTest(vector=vector.name, cuts=cuts):
                    self.check(vector, parts)

    def test_trailing_sync_byte_is_kept(self):
        decoder = self.check(p.golden_vectors()[-1], [p.golden_vectors()[-1].data])
        self.assertEqual(bytes(decoder.buffer), b"\xa5")

    def test_noise_never_raises_and_stays_bounded(self):
        rng = random.Random(3)
        decoder = p.FrameDecoder()
        for _ in range(2000):
            data = bytes(rng.choice((0xA5, 0x5A, 0x01, rng.randrange(256)))
                         for _ in range(rng.randrange(1, 40)))
            for frame in decoder.feed(data):  # a chance frame is still well-formed
                self.assertEqual(frame.version, p.VERSION)
                self.assertLessEqual(len(frame.payload), p.MAX_PAYLOAD)
            self.assertLessEqual(len(decoder.buffer), p.MAX_FRAME)

    def test_every_single_bit_error_is_detected(self):
        frame = p.encode(p.SAMPLE_MOVE, 3)
        for bit in range(16, len(frame) * 8):  # every bit after the sync pair
            corrupted = bytearray(frame)
            corrupted[bit // 8] ^= 1 << (bit % 8)
            self.assertEqual(p.FrameDecoder().feed(bytes(corrupted)), [], f"bit {bit}")

    def test_valid_frame_after_corruption_recovered(self):
        rng = random.Random(11)
        good = p.encode(p.Heartbeat(), 77)
        for _ in range(200):
            junk = bytearray(p.encode(p.SAMPLE_TELEMETRY, 1))
            junk[rng.randrange(2, len(junk))] ^= 1 << rng.randrange(8)
            decoder = p.FrameDecoder()
            frames = decoder.feed(bytes(junk) + good + good)
            # Both heartbeats arrive at once: no corruption (header or payload)
            # may make the decoder wait for bytes that never come.
            self.assertEqual(frames, [p.Frame(p.MsgType.HEARTBEAT, 77, bytes(4))] * 2)


class ScanTransferTests(unittest.TestCase):
    def scan(self, count=456):
        return [(round(i * 36000 / count) % 36000, 500 + i, i % 256) for i in range(count)]

    def test_456_points_round_trip(self):
        chunks = p.scan_to_chunks(42, self.scan())
        self.assertEqual(len(chunks), 5)
        self.assertTrue(all(len(c.payload()) <= p.MAX_PAYLOAD for c in chunks))
        decoder, assembler, result = p.FrameDecoder(), p.ScanAssembler(), None
        for seq, chunk in enumerate(chunks):
            for frame in decoder.feed(p.encode(chunk, seq)):
                result = assembler.add(p.decode_message(frame)) or result
        self.assertEqual(result, tuple(self.scan()))

    def test_missing_chunk_is_listed_and_completes_later(self):
        chunks = p.scan_to_chunks(1, self.scan())
        assembler = p.ScanAssembler()
        for chunk in (chunks[0], chunks[2], chunks[3], chunks[4]):   # chunk 1 lost
            self.assertIsNone(assembler.add(chunk))
        self.assertEqual(assembler.missing(), [1])
        self.assertEqual(assembler.add(chunks[1]), tuple(self.scan()))  # e.g. SCAN_RESEND
        self.assertEqual(assembler.missing(), [])
        self.assertEqual(assembler.dropped, 0)

    def test_out_of_order_and_duplicate_chunks(self):
        chunks = p.scan_to_chunks(3, self.scan())
        assembler, result = p.ScanAssembler(), None
        for chunk in (chunks[4], chunks[1], chunks[1], chunks[0], chunks[3], chunks[2]):
            result = assembler.add(chunk) or result
        self.assertEqual(result, tuple(self.scan()))
        self.assertIsNone(assembler.add(chunks[2]))     # late duplicate after completion
        self.assertEqual((assembler.missing(), assembler.dropped), ([], 0))

    def test_new_scan_drops_an_unfinished_one(self):
        first, second = p.scan_to_chunks(1, self.scan()), p.scan_to_chunks(2, self.scan())
        assembler, result = p.ScanAssembler(), None
        assembler.add(first[0])
        for chunk in second:
            result = assembler.add(chunk) or result
        self.assertEqual((result, assembler.dropped), (tuple(self.scan()), 1))

    def test_two_revolutions_never_stitched(self):
        first, second = p.scan_to_chunks(1, self.scan()), p.scan_to_chunks(2, self.scan())
        assembler = p.ScanAssembler()
        assembler.add(first[0])
        assembler.add(first[1])
        self.assertIsNone(assembler.add(second[2]))
        for chunk in first[2:]:
            self.assertIsNone(assembler.add(chunk))
        self.assertGreaterEqual(assembler.dropped, 1)

    def test_empty_scan_refused(self):
        with self.assertRaises(ValueError):
            p.scan_to_chunks(1, [])


class FirmwareVectorHeaderTests(unittest.TestCase):
    def test_header_is_up_to_date(self):
        self.assertTrue(VECTOR_HEADER.exists(), "run: python -m autonomy.protocol "
                        "--write-c-header firmware/tests/protocol_vectors.h")
        self.assertEqual(VECTOR_HEADER.read_text(encoding="ascii"), p.render_c_header(),
                         "protocol changed: regenerate firmware/tests/protocol_vectors.h")

    def test_header_lists_every_vector(self):
        header = p.render_c_header()
        for vector in p.golden_vectors():
            self.assertIn(f'"{vector.name}"', header)
        self.assertIn(f"kVectorCount = {len(p.golden_vectors())}", header)


if __name__ == "__main__":
    unittest.main()
