"""Tests of the firmware safety model and the mock robot link. No hardware.

    python -m unittest test_autonomy_link -v

Every harness operation also checks FirmwareModel.invariant_violations():
the driver is never enabled outside ARMED/MOVING, outputs are never
non-zero outside MOVING, and a latching fault always means FAULT.
"""
import math
import random
import subprocess
import sys
import threading
import unittest
from pathlib import Path

from autonomy import protocol as p
from dataclasses import replace

from autonomy.firmware_model import RX_BUFFER_BYTES, SERIAL_PEER, FirmwareModel
from autonomy.robot_link import (LinkError, MockRobotLink, MockTransport, SerialRobotLink,
                                 SerialTransport, UdpTransport, WiFiRobotLink, make_move)

SAMPLE_SCAN = tuple((i * 79 % 36000, 800 + i, 100) for i in range(456))


SESSION = 0x5E551011
OTHER_PEER = ("192.168.4.3", 47801)


class Harness:
    """Drive a FirmwareModel directly with frames and simulated time.

    The harness plays a well-behaved controller (session SESSION on
    SERIAL_PEER): HELLO at start, heartbeats echoing the robot's current
    uptime, and MOVEs stamped valid_until = uptime + COMMAND_TTL_MS
    (a Heartbeat / Move with a zero echo / deadline is stamped on send).
    """

    def __init__(self, test, hello=True, **options):
        self.test = test
        self.fw = FirmwareModel(**options)
        self.now = self.fw.now
        self.seq = p.SequenceCounter()
        self.decoder = p.FrameDecoder()
        self.messages = []
        if hello:
            self.send(p.Hello(SESSION))

    def check(self):
        self.test.assertEqual(self.fw.invariant_violations(), [], f"at {self.now} ms")

    def collect(self):
        for frame in self.fw.read_frames():
            for decoded in self.decoder.feed(frame):
                self.messages.append(p.decode_message(decoded))

    def raw(self, data, peer=SERIAL_PEER):
        self.fw.receive(data, self.now, peer)
        self.check()
        self.collect()

    def stamp(self, message):
        if isinstance(message, p.Heartbeat) and not message.echo_uptime_ms:
            return p.Heartbeat(self.fw.uptime())
        if isinstance(message, p.Move) and not message.valid_until_ms:
            return replace(message, valid_until_ms=(self.fw.uptime() + p.COMMAND_TTL_MS)
                           % p.UPTIME_MODULO)
        return message

    def send(self, message, seq=None, peer=SERIAL_PEER):
        seq = self.seq.next() if seq is None else seq
        self.raw(p.encode(self.stamp(message), seq), peer)
        return seq

    def run(self, duration_ms, heartbeat=True, tick=10, echo=None):
        """Advance time; heartbeats every 100 ms echo the current uptime, or
        echo(uptime) when given (to send stale / reordered echoes)."""
        end = self.now + duration_ms
        while self.now < end:
            self.now += tick
            if heartbeat and self.now % p.HEARTBEAT_PERIOD_MS == 0:
                uptime = (self.now - self.fw.boot_ms) % p.UPTIME_MODULO  # at receive time
                value = uptime if echo is None else echo(uptime)
                self.fw.receive(p.encode(p.Heartbeat(value % p.UPTIME_MODULO),
                                         self.seq.next()), self.now, SERIAL_PEER)
            self.fw.step(self.now)
            self.check()
            self.collect()

    def reply(self, seq):
        """The ACK / NACK answering `seq` (the last one)."""
        found = [m for m in self.messages if isinstance(m, (p.Ack, p.Nack)) and m.ref_seq == seq]
        self.test.assertTrue(found, f"no reply to seq {seq}")
        return found[-1]

    def nack_reason(self, seq):
        reply = self.reply(seq)
        self.test.assertIsInstance(reply, p.Nack)
        return p.NackReason(reply.reason)

    def of_type(self, cls):
        return [m for m in self.messages if isinstance(m, cls)]

    def armed(self):
        self.send(p.Hello(SESSION))
        self.send(p.Heartbeat())
        self.test.assertIsInstance(self.reply(self.send(p.Arm())), p.Ack)
        return self


MOVE = p.Move(forward_mm=200, turn_cdeg=4500, speed_mm_s=200, turn_rate_cdeg_s=9000,
              timeout_ms=2000)   # 500 ms turn, then 1000 ms forward


class BootAndArmingTests(unittest.TestCase):
    def test_boots_disarmed_with_driver_disabled(self):
        h = Harness(self)
        self.assertEqual(h.fw.state, p.State.DISARMED)
        self.assertFalse(h.fw.motor.enabled)
        h.run(1000, heartbeat=False)
        self.assertEqual(h.fw.state, p.State.DISARMED)   # no host: still safe, no fault
        self.assertEqual(h.fw.faults, p.Fault.NONE)

    def test_telemetry_every_200_ms(self):
        h = Harness(self)
        h.run(1000)
        telemetry = h.of_type(p.Telemetry)
        # first at the first loop pass (10 ms), then at 200, 400, ... 1000 ms
        self.assertEqual([t.uptime_ms for t in telemetry], [10, 200, 400, 600, 800, 1000])
        self.assertEqual(telemetry[-1].state, p.State.DISARMED)
        self.assertEqual(telemetry[-1].outputs_enabled, 0)

    def test_hello_reports_mock_driver(self):
        h = Harness(self, boot_id=77)
        h.send(p.Hello())
        reply = h.of_type(p.HelloReply)[0]
        self.assertEqual((reply.protocol_version, reply.boot_id, reply.motor_driver),
                         (p.VERSION, 77, p.MotorDriverKind.MOCK))

    def test_arm_needs_a_live_heartbeat(self):
        h = Harness(self)
        self.assertEqual(h.nack_reason(h.send(p.Arm())), p.NackReason.NO_HEARTBEAT)
        h.send(p.Heartbeat())
        h.run(310, heartbeat=False)
        self.assertEqual(h.nack_reason(h.send(p.Arm())), p.NackReason.NO_HEARTBEAT)
        h.armed()
        self.assertEqual(h.fw.state, p.State.ARMED)
        self.assertTrue(h.fw.motor.enabled)
        self.assertFalse(h.fw.motor.moving())

    def test_move_refused_unless_armed(self):
        h = Harness(self)
        h.send(p.Heartbeat())
        self.assertEqual(h.nack_reason(h.send(MOVE)), p.NackReason.NOT_ARMED)
        self.assertEqual(h.fw.motor.history[-1][1:], (False, 0, 0))


class WatchdogTests(unittest.TestCase):
    def test_heartbeat_timeout_boundary(self):
        h = Harness(self).armed()
        h.run(p.HEARTBEAT_TIMEOUT_MS, heartbeat=False)       # age exactly 300 ms
        self.assertEqual(h.fw.state, p.State.ARMED)
        h.run(10, heartbeat=False)                           # 310 ms
        self.assertEqual(h.fw.state, p.State.FAULT)
        self.assertTrue(h.fw.faults & p.Fault.HEARTBEAT_TIMEOUT)
        self.assertFalse(h.fw.motor.enabled)

    def test_comm_loss_during_move_stops_within_one_pass(self):
        h = Harness(self).armed()
        seq = h.send(MOVE)
        h.run(1000)
        self.assertTrue(h.fw.motor.moving())
        last_heartbeat = h.fw.last_heartbeat_ms
        h.run(p.HEARTBEAT_TIMEOUT_MS + 10, heartbeat=False)
        self.assertEqual(h.fw.state, p.State.FAULT)
        self.assertFalse(h.fw.motor.enabled or h.fw.motor.moving())
        done = h.of_type(p.MoveDone)
        self.assertEqual([(d.ref_seq, d.outcome) for d in done], [(seq, p.MoveOutcome.FAULT)])
        # the outputs went to zero at the first pass past the timeout
        off = [t for t, enabled, f, w in h.fw.motor.history if not enabled][-1]
        self.assertEqual(off - last_heartbeat, p.HEARTBEAT_TIMEOUT_MS + 10)

    def test_fault_needs_clear_then_arm(self):
        h = Harness(self).armed()
        h.run(400, heartbeat=False)
        self.assertEqual(h.fw.state, p.State.FAULT)
        h.send(p.Heartbeat())
        self.assertEqual(h.nack_reason(h.send(p.Arm())), p.NackReason.FAULT_ACTIVE)
        self.assertEqual(h.nack_reason(h.send(MOVE)), p.NackReason.FAULT_ACTIVE)
        self.assertIsInstance(h.reply(h.send(p.ClearFault())), p.Ack)
        self.assertEqual(h.fw.state, p.State.DISARMED)       # not straight back to ARMED
        self.assertFalse(h.fw.motor.enabled)
        h.armed()
        self.assertEqual(h.fw.state, p.State.ARMED)

    def test_clear_fault_needs_heartbeat(self):
        h = Harness(self).armed()
        h.run(400, heartbeat=False)
        self.assertEqual(h.nack_reason(h.send(p.ClearFault())), p.NackReason.NO_HEARTBEAT)
        self.assertEqual(h.fw.state, p.State.FAULT)

    def test_driver_fault_latches(self):
        h = Harness(self).armed()
        h.send(MOVE)
        h.run(100)
        h.fw.motor.faulted = True
        h.run(10)
        self.assertEqual(h.fw.state, p.State.FAULT)
        self.assertTrue(h.fw.faults & p.Fault.DRIVER_FAULT)
        self.assertEqual(h.of_type(p.MoveDone)[-1].outcome, p.MoveOutcome.FAULT)

    def test_motion_timeout_is_enforced(self):
        # White-box: a profile that would outlive its timeout (impossible
        # through validate_move) is still cut at timeout_ms.
        h = Harness(self).armed()
        h.send(MOVE)
        h.fw.move["total_ms"] = 10_000
        h.run(3100)
        self.assertEqual(h.fw.state, p.State.FAULT)
        self.assertTrue(h.fw.faults & p.Fault.MOTION_TIMEOUT)
        self.assertEqual(h.of_type(p.MoveDone)[-1].outcome, p.MoveOutcome.TIMEOUT)

    def test_rx_overflow_latches(self):
        h = Harness(self).armed()
        h.raw(b"\x00" * (RX_BUFFER_BYTES + 1))
        self.assertEqual(h.fw.state, p.State.FAULT)
        self.assertTrue(h.fw.faults & p.Fault.RX_OVERFLOW)


class MotionAndStopTests(unittest.TestCase):
    def test_move_profile_turn_then_forward(self):
        h = Harness(self).armed()
        start = h.now
        seq = h.send(MOVE)
        self.assertEqual(h.fw.state, p.State.MOVING)
        self.assertEqual((h.fw.motor.forward_mm_s, h.fw.motor.turn_cdeg_s), (0, 9000))
        h.run(2600)
        changes = [(t - start, f, w) for t, enabled, f, w in h.fw.motor.history if enabled]
        self.assertEqual(changes[-3:], [(0, 0, 9000), (500, 200, 0), (1500, 0, 0)])
        self.assertEqual(h.fw.state, p.State.ARMED)
        self.assertEqual([(d.ref_seq, d.outcome) for d in h.of_type(p.MoveDone)],
                         [(seq, p.MoveOutcome.COMPLETED)])

    def test_negative_move(self):
        h = Harness(self).armed()
        h.send(p.Move(-100, -9000, 100, 9000, 2000))
        self.assertEqual(h.fw.motor.turn_cdeg_s, -9000)
        h.run(1500)
        self.assertEqual(h.fw.motor.forward_mm_s, -100)

    def test_stop_zeroes_outputs_in_the_same_pass(self):
        h = Harness(self).armed()
        h.send(MOVE)
        h.run(800)
        self.assertTrue(h.fw.motor.moving())
        seq = h.send(p.Stop())                    # no step() in between
        self.assertFalse(h.fw.motor.moving())
        self.assertIsInstance(h.reply(seq), p.Ack)
        self.assertEqual(h.fw.state, p.State.ARMED)
        self.assertEqual(h.of_type(p.MoveDone)[-1].outcome, p.MoveOutcome.STOPPED)
        h.run(3000)
        self.assertFalse(h.fw.motor.moving())     # the move is not resumed

    def test_stop_is_idempotent_in_every_state(self):
        h = Harness(self)
        for _ in range(3):
            self.assertIsInstance(h.reply(h.send(p.Stop())), p.Ack)
        self.assertEqual(h.fw.state, p.State.DISARMED)
        h.armed()
        self.assertIsInstance(h.reply(h.send(p.Stop())), p.Ack)
        self.assertEqual(h.fw.state, p.State.ARMED)
        h.run(400, heartbeat=False)
        self.assertIsInstance(h.reply(h.send(p.Stop())), p.Ack)
        self.assertEqual(h.fw.state, p.State.FAULT)  # STOP does not clear a fault

    def test_disarm_during_move(self):
        h = Harness(self).armed()
        h.send(MOVE)
        h.run(200)
        h.send(p.Disarm())
        self.assertEqual(h.fw.state, p.State.DISARMED)
        self.assertFalse(h.fw.motor.enabled)
        self.assertEqual(h.of_type(p.MoveDone)[-1].outcome, p.MoveOutcome.STOPPED)

    def test_out_of_bounds_moves_never_reach_the_driver(self):
        # Forged frames that skip the host's validation.
        h = Harness(self).armed()
        bad = (p.Move(301, 0, 200, 9000, 2000), p.Move(0, 9001, 100, 9000, 2000),
               p.Move(100, 0, 0, 9000, 2000), p.Move(300, 0, 100, 9000, 2000),
               p.Move(0, 0, 100, 9000, 2000), p.Move(100, 0, 100, 9000, 0),
               p.Move(100, 0, 100, 9000, 2001))
        for move in bad:
            self.assertEqual(h.nack_reason(h.send(move)), p.NackReason.OUT_OF_BOUNDS)
        self.assertFalse(any(f or w for _, _, f, w in h.fw.motor.history))

    def test_move_while_moving_is_busy(self):
        h = Harness(self).armed()
        h.send(MOVE)
        self.assertEqual(h.nack_reason(h.send(MOVE)), p.NackReason.BUSY)

    def test_replayed_move_is_never_executed_twice(self):
        h = Harness(self).armed()
        seq = h.send(MOVE)
        h.run(2600)
        self.assertEqual(h.nack_reason(h.send(MOVE, seq)), p.NackReason.STALE_SEQ)
        self.assertEqual(h.nack_reason(h.send(MOVE, (seq - 5) % p.SEQ_MODULO)),
                         p.NackReason.STALE_SEQ)
        self.assertEqual(h.fw.state, p.State.ARMED)
        self.assertEqual(len(h.of_type(p.MoveDone)), 1)

    def test_move_seq_wraps(self):
        h = Harness(self).armed()
        h.send(p.Move(0, 100, 0, 9000, 1000), 0xFFFF)
        h.run(500)
        self.assertIsInstance(h.reply(h.send(p.Move(0, 100, 0, 9000, 1000), 0)), p.Ack)


class MalformedInputTests(unittest.TestCase):
    def test_bad_length_unknown_type_and_wrong_direction(self):
        h = Harness(self).armed()
        state = h.fw.state
        h.raw(p.encode_frame(p.MsgType.MOVE, 50, b"\x01\x02\x03"))
        self.assertEqual(h.nack_reason(50), p.NackReason.BAD_LENGTH)
        h.raw(p.encode_frame(0x42, 51))
        self.assertEqual(h.nack_reason(51), p.NackReason.UNKNOWN_TYPE)
        h.raw(p.encode(p.SAMPLE_TELEMETRY, 52))
        self.assertEqual(h.nack_reason(52), p.NackReason.WRONG_DIRECTION)
        self.assertEqual(h.fw.state, state)
        self.assertFalse(h.fw.motor.moving())

    def test_corrupt_frames_are_ignored_and_counted(self):
        h = Harness(self).armed()
        frame = bytearray(p.encode(MOVE, 60))
        frame[12] ^= 0x10
        h.raw(bytes(frame))
        self.assertFalse(h.fw.motor.moving())
        self.assertEqual([m for m in h.messages if getattr(m, "ref_seq", None) == 60], [])
        h.run(200)
        self.assertEqual(h.of_type(p.Telemetry)[-1].crc_errors, 1)

    def test_random_traffic_never_breaks_the_invariants(self):
        rng = random.Random(5)
        commands = (p.Hello(SESSION), p.Heartbeat(), p.Arm(), p.Disarm(), p.Stop(), p.ClearFault(),
                    p.ScanRequest(), MOVE, p.Move(-200, -9000, 200, 9000, 2000),
                    p.Move(301, 0, 200, 9000, 2000))
        h = Harness(self, scan_provider=lambda: SAMPLE_SCAN)
        for _ in range(3000):
            choice = rng.random()
            if choice < 0.5:
                h.send(rng.choice(commands), rng.randrange(0x10000))
            elif choice < 0.7:
                h.raw(bytes(rng.randrange(256) for _ in range(rng.randrange(1, 30))))
            else:
                h.run(rng.choice((10, 50, 200, 400)), heartbeat=rng.random() < 0.7)
            if h.fw.motor.enabled:   # enabled only after an ARM with a live heartbeat
                self.assertIsNotNone(h.fw.last_heartbeat_ms)


class ProtocolV2FirmwareTests(unittest.TestCase):
    """Heartbeat echo, command expiry, session binding, chunk resend."""

    def test_stale_echo_heartbeats_do_not_feed_the_watchdog(self):
        # Heartbeats keep ARRIVING, but they echo an uptime older than
        # HEARTBEAT_ECHO_MAX_MS (e.g. buffered during an outage): fault.
        h = Harness(self).armed()
        h.run(500)
        h.run(p.HEARTBEAT_TIMEOUT_MS + 20,
              echo=lambda uptime: uptime - p.HEARTBEAT_ECHO_MAX_MS - 1)
        self.assertEqual(h.fw.state, p.State.FAULT)
        self.assertTrue(h.fw.faults & p.Fault.HEARTBEAT_TIMEOUT)
        self.assertGreater(h.fw.rejected_heartbeats, 0)

    def test_echo_at_the_limit_is_accepted(self):
        # A controller whose newest telemetry is always exactly 800 ms old.
        # (Switching from fresh echoes to older ones would be rejected as
        # reordering: echoes may never go backwards.)
        h = Harness(self)
        h.run(1000, heartbeat=False)
        h.send(p.Heartbeat(h.fw.uptime() - p.HEARTBEAT_ECHO_MAX_MS))
        self.assertIsInstance(h.reply(h.send(p.Arm())), p.Ack)
        h.run(1000, echo=lambda uptime: uptime - p.HEARTBEAT_ECHO_MAX_MS)
        self.assertEqual(h.fw.state, p.State.ARMED)

    def test_reordered_and_future_echoes_rejected(self):
        h = Harness(self).armed()
        h.run(500)
        rejected = h.fw.rejected_heartbeats
        h.send(p.Heartbeat(h.fw.uptime() - 300))          # older than the last accepted
        h.send(p.Heartbeat(h.fw.uptime() + 50))           # from the "future"
        self.assertEqual(h.fw.rejected_heartbeats, rejected + 2)

    def test_expired_move_never_runs(self):
        h = Harness(self).armed()
        h.run(1000)
        late = replace(MOVE, valid_until_ms=h.fw.uptime() - 1)
        self.assertEqual(h.nack_reason(h.send(late)), p.NackReason.EXPIRED)
        self.assertFalse(h.fw.motor.moving())
        self.assertEqual(h.fw.state, p.State.ARMED)
        on_time = replace(MOVE, valid_until_ms=h.fw.uptime())   # "not after" = accepted
        self.assertIsInstance(h.reply(h.send(on_time)), p.Ack)

    def test_uptime_wraparound(self):
        h = Harness(self).armed()
        h.fw.boot_ms = h.now - (p.UPTIME_MODULO - 500)        # uptime wraps in 0.5 s
        h.fw.last_echo = h.fw.uptime()
        h.run(2000)                                           # across the wrap
        self.assertEqual(h.fw.state, p.State.ARMED)
        self.assertLess(h.fw.uptime(), 2000)
        self.assertIsInstance(h.reply(h.send(MOVE)), p.Ack)

    def test_second_controller_refused_but_its_stop_obeyed(self):
        h = Harness(self).armed()
        h.send(MOVE)
        h.run(300)
        for message in (p.Arm(), p.ClearFault(), MOVE, p.ScanRequest()):
            self.assertEqual(h.nack_reason(h.send(message, peer=OTHER_PEER)),
                             p.NackReason.NOT_CONTROLLER)
        self.assertEqual(h.nack_reason(h.send(p.Hello(SESSION + 1), peer=OTHER_PEER)),
                         p.NackReason.NOT_CONTROLLER)
        self.assertTrue(h.fw.motor.moving())
        self.assertIsInstance(h.reply(h.send(p.Stop(), peer=OTHER_PEER)), p.Ack)
        self.assertFalse(h.fw.motor.moving())
        self.assertIsInstance(h.reply(h.send(p.Disarm(), peer=OTHER_PEER)), p.Ack)
        self.assertEqual(h.fw.state, p.State.DISARMED)
        self.assertEqual(h.fw.controller, (SESSION, SERIAL_PEER))   # binding kept

    def test_foreign_heartbeats_do_not_feed_the_watchdog(self):
        h = Harness(self).armed()
        for _ in range(40):
            h.now += 10
            h.fw.receive(p.encode(p.Heartbeat(h.fw.uptime()), h.seq.next()), h.now, OTHER_PEER)
            h.fw.step(h.now)
            h.check()
        self.assertEqual(h.fw.state, p.State.FAULT)

    def test_same_session_rebinds_from_a_new_address(self):
        h = Harness(self).armed()
        h.send(p.Hello(SESSION), peer=OTHER_PEER)             # Wi-Fi reconnect, new IP
        self.assertEqual(h.fw.controller, (SESSION, OTHER_PEER))
        self.assertEqual(h.nack_reason(h.send(p.Arm())), p.NackReason.NOT_CONTROLLER)

    def test_telemetry_goes_to_the_controller_only(self):
        h = Harness(self)
        h.send(p.Hello(SESSION), peer=OTHER_PEER)
        h.fw.read_addressed()
        h.now += 300
        h.fw.step(h.now)
        self.assertEqual({peer for peer, _ in h.fw.read_addressed()}, {OTHER_PEER})

    def test_binding_released_after_silence_only_when_safe(self):
        h = Harness(self)
        h.run(p.SESSION_RELEASE_MS + 100, heartbeat=False)
        self.assertIsNone(h.fw.controller)                    # DISARMED and silent
        other = Harness(self).armed()
        other.run(p.SESSION_RELEASE_MS + 100, heartbeat=False)  # faults after 300 ms ...
        self.assertEqual(other.fw.state, p.State.FAULT)
        self.assertIsNone(other.fw.controller)                # ... then is released

    def test_disarm_by_the_controller_releases_the_robot(self):
        h = Harness(self).armed()
        h.send(p.Disarm())
        self.assertIsNone(h.fw.controller)
        self.assertEqual(h.nack_reason(h.send(p.Arm())), p.NackReason.NOT_CONTROLLER)

    def test_scan_resend(self):
        h = Harness(self, scan_provider=lambda: SAMPLE_SCAN)
        h.send(p.ScanRequest())
        chunks = h.of_type(p.ScanChunk)
        h.send(p.ScanResend(chunks[0].scan_id, 3))
        self.assertEqual(h.of_type(p.ScanChunk)[-1], chunks[3])
        self.assertEqual(h.nack_reason(h.send(p.ScanResend(chunks[0].scan_id, 9))),
                         p.NackReason.LIDAR_UNAVAILABLE)
        self.assertEqual(h.nack_reason(h.send(p.ScanResend(chunks[0].scan_id + 1, 0))),
                         p.NackReason.LIDAR_UNAVAILABLE)


class RestartAndScanTests(unittest.TestCase):
    def test_restart_mid_move_comes_back_disarmed(self):
        h = Harness(self, boot_id=1).armed()
        h.send(MOVE)
        h.run(300)
        h.fw.restart(h.now, boot_id=2)
        self.assertEqual(h.fw.state, p.State.DISARMED)
        self.assertFalse(h.fw.motor.enabled or h.fw.motor.moving())
        self.assertIsNone(h.fw.move)
        h.run(3000)
        self.assertFalse(h.fw.motor.moving())      # never resumes
        self.assertEqual(h.of_type(p.Telemetry)[-1].boot_id, 2)
        # sequence state is new: an old seq is accepted again after ARM
        h.armed()
        self.assertIsInstance(h.reply(h.send(MOVE, 1)), p.Ack)

    def test_scan_request(self):
        h = Harness(self, scan_provider=lambda: SAMPLE_SCAN)
        self.assertIsInstance(h.reply(h.send(p.ScanRequest())), p.Ack)  # allowed disarmed
        chunks = h.of_type(p.ScanChunk)
        self.assertEqual(len(chunks), 5)
        assembler, result = p.ScanAssembler(), None
        for chunk in chunks:
            result = assembler.add(chunk) or result
        self.assertEqual(result, SAMPLE_SCAN)

    def test_scan_refused_while_moving_or_without_lidar(self):
        h = Harness(self, scan_provider=lambda: SAMPLE_SCAN).armed()
        h.send(MOVE)
        self.assertEqual(h.nack_reason(h.send(p.ScanRequest())), p.NackReason.BUSY)
        h2 = Harness(self)
        self.assertEqual(h2.nack_reason(h2.send(p.ScanRequest())),
                         p.NackReason.LIDAR_UNAVAILABLE)


class RecordingPlant:
    """Integrates the requested body velocity (stand-in for SimRobot)."""

    def __init__(self):
        self.distance_mm = 0.0
        self.turn_cdeg = 0.0
        self.motion_ticks = 0

    def advance(self, forward_mm_s, turn_cdeg_s, enabled, dt_ms):
        assert enabled or not (forward_mm_s or turn_cdeg_s)
        self.distance_mm += forward_mm_s * dt_ms / 1000.0
        self.turn_cdeg += turn_cdeg_s * dt_ms / 1000.0
        self.motion_ticks += bool(forward_mm_s or turn_cdeg_s)


def make_link(seed=0, drop=0.0, corrupt=0.0, **options):
    plant = RecordingPlant()
    firmware = FirmwareModel(boot_id=11, scan_provider=lambda: SAMPLE_SCAN)
    link = MockRobotLink(firmware, MockTransport(drop_rate=drop, corrupt_rate=corrupt, seed=seed),
                         plant, **options)
    return link, plant


class MockLinkTests(unittest.TestCase):
    def test_connect_arm_move(self):
        link, plant = make_link()
        hello = link.connect()
        self.assertEqual(hello.motor_driver, p.MotorDriverKind.MOCK)
        self.assertTrue(link.arm().ok)
        start = link.now_ms
        result = link.move(0.2, math.radians(-30))
        self.assertEqual(result.outcome, "completed")
        self.assertAlmostEqual(plant.distance_mm, 200, delta=0.15 * 10 * 2)  # <= 2 ticks
        self.assertAlmostEqual(plant.turn_cdeg, -3000, delta=60 * 100 * 0.01 * 2)
        self.assertLess(link.now_ms - start, 3000)
        link.pump(p.TELEMETRY_PERIOD_MS + 50)
        self.assertEqual(link.telemetry.state, p.State.ARMED)

    def test_refused_move_never_sent(self):
        link, plant = make_link()
        link.connect()
        link.arm()
        sent = link.firmware.decoder.frames
        result = link.move(0.8, 0.0)     # 800 mm > 500 mm
        self.assertEqual(result.outcome, "refused")
        link.pump(50)
        self.assertLessEqual(link.firmware.decoder.frames - sent, 2)  # heartbeats only
        self.assertEqual(plant.motion_ticks, 0)

    def test_move_before_arm_is_refused_by_the_robot(self):
        link, plant = make_link()
        link.connect()
        result = link.move(0.2, 0.0)
        self.assertEqual((result.outcome, result.detail), ("refused", "NOT_ARMED"))
        self.assertEqual(plant.motion_ticks, 0)

    def test_requires_connect(self):
        link, _ = make_link()
        with self.assertRaises(LinkError):
            link.arm()

    def test_host_hang_triggers_the_watchdog(self):
        link, plant = make_link()
        link.connect()
        link.arm()
        link.heartbeat_enabled = False
        result = link.move(0.3, 0.0)
        self.assertEqual(result.outcome, "fault")
        self.assertTrue(result.faults & p.Fault.HEARTBEAT_TIMEOUT)
        self.assertFalse(link.firmware.motor.enabled)
        self.assertLess(plant.distance_mm, 0.15 * (p.HEARTBEAT_TIMEOUT_MS + 200))

    def test_blackout_during_move(self):
        link, plant = make_link()
        link.connect()
        link.arm()
        stop_at = {"t": None}

        class BlackoutPlant(RecordingPlant):
            def advance(inner, f, w, enabled, dt):
                RecordingPlant.advance(inner, f, w, enabled, dt)
                if inner.distance_mm > 100 and stop_at["t"] is None:
                    link.transport.blackout = True
                    stop_at["t"] = link.now_ms

        link.plant = BlackoutPlant()
        result = link.move(0.3, 0.0)
        self.assertIn(result.outcome, ("link_lost", "fault"))
        self.assertEqual(link.firmware.state, p.State.FAULT)
        self.assertFalse(link.firmware.motor.moving())
        off = [t for t, enabled, *_ in link.firmware.motor.history if not enabled][-1]
        self.assertLessEqual(off - stop_at["t"], p.HEARTBEAT_TIMEOUT_MS + p.HEARTBEAT_PERIOD_MS
                             + 2 * MockRobotLink.TICK_MS)

    def test_lossy_link_still_moves_exactly_once(self):
        for seed in range(15):
            with self.subTest(seed=seed):
                link, plant = make_link(seed=seed, drop=0.15, corrupt=0.05)
                for _ in range(3):
                    try:
                        link.connect()
                        break
                    except LinkError:
                        continue
                for _ in range(5):
                    armed = link.arm()
                    if armed.ok:
                        break
                    if armed.reason == "FAULT_ACTIVE":   # a lost-heartbeat streak
                        link.clear_fault()
                self.assertTrue(armed.ok, armed)
                result = link.move(0.2, 0.0, speed_m_s=0.2)
                if result.outcome == "completed":
                    self.assertAlmostEqual(plant.distance_mm, 200, delta=5)
                else:  # a heartbeat loss streak may fault: then it must be safe
                    self.assertIn(result.outcome, ("fault", "link_lost"))
                    self.assertFalse(link.firmware.motor.moving())
                    self.assertLessEqual(plant.distance_mm, 205)
                self.assertEqual(link.firmware.invariant_violations(), [])

    def test_one_way_loss_never_leaves_the_robot_moving(self):
        # Heartbeats still reach the robot, so its watchdog never fires; the
        # host hears nothing back and must stop the robot before returning.
        link, plant = make_link()
        link.connect()
        link.arm()

        class MuteAfterStart(RecordingPlant):
            def advance(inner, f, w, enabled, dt):
                RecordingPlant.advance(inner, f, w, enabled, dt)
                if inner.distance_mm > 30:
                    link.transport.drop_rate["to_host"] = 1.0

        link.plant = MuteAfterStart()
        result = link.move(0.3, 0.0)
        self.assertEqual(result.outcome, "link_lost")
        self.assertIn("watchdog starved", result.detail)
        self.assertFalse(link.firmware.motor.moving())
        self.assertEqual(link.firmware.state, p.State.FAULT)
        self.assertEqual(link.firmware.invariant_violations(), [])

    def test_lost_ack_is_recovered_by_seq_replay_protection(self):
        link, plant = make_link()
        link.connect()
        link.arm()
        original = link.transport.write
        dropped = {"done": False}

        def drop_first_ack(direction, frame):
            if direction == "to_host" and not dropped["done"] and frame[3] == p.MsgType.ACK:
                payload = frame[p.HEADER_SIZE:-2]
                if payload[2] == p.MsgType.MOVE:
                    dropped["done"] = True
                    return
            original(direction, frame)

        link.transport.write = drop_first_ack
        result = link.move(0.2, 0.0)
        self.assertTrue(dropped["done"])
        self.assertEqual(result.outcome, "completed")
        self.assertAlmostEqual(plant.distance_mm, 200, delta=3)   # once, not twice

    def test_stop_from_another_thread(self):
        link, plant = make_link()
        link.connect()
        link.arm()
        results = {}
        moving = threading.Event()

        class SignalPlant(RecordingPlant):
            def advance(inner, f, w, enabled, dt):
                RecordingPlant.advance(inner, f, w, enabled, dt)
                if inner.distance_mm > 50:
                    moving.set()

        link.plant = SignalPlant()
        link.realtime = 0.2   # slow enough for the other thread to act mid-move
        worker = threading.Thread(target=lambda: results.update(r=link.move(0.3, 0.0)))
        worker.start()
        self.assertTrue(moving.wait(10))
        with link.lock:
            stop_time = link.now_ms
        link.stop()
        worker.join(10)
        self.assertFalse(worker.is_alive())
        self.assertEqual(results["r"].outcome, "stopped")
        stopped = [t for t, enabled, f, w in link.firmware.motor.history if enabled and not f][-1]
        self.assertLessEqual(stopped - stop_time, 3 * MockRobotLink.TICK_MS)
        self.assertLess(link.plant.distance_mm, 300)

    def test_restart_detected_and_requires_reconnect(self):
        link, plant = make_link()
        link.connect()
        link.arm()

        class RestartPlant(RecordingPlant):
            def advance(inner, f, w, enabled, dt):
                RecordingPlant.advance(inner, f, w, enabled, dt)
                if inner.distance_mm > 60 and link.firmware.boot_id == 11:
                    link.firmware.restart(link.now_ms, boot_id=12)

        link.plant = RestartPlant()
        result = link.move(0.3, 0.0)
        self.assertEqual(result.outcome, "restarted")
        self.assertFalse(link.firmware.motor.moving())
        self.assertEqual(link.move(0.1, 0.0).outcome, "restarted")  # refused until reconnect
        link.connect()
        self.assertEqual(link.boot_id, 12)
        self.assertTrue(link.arm().ok)
        self.assertEqual(link.move(0.1, 0.0).outcome, "completed")

    def test_scan_over_115200_baud(self):
        link, _ = make_link()
        link.connect()
        start = link.now_ms
        result = link.request_scan()
        self.assertTrue(result.ok)
        self.assertEqual(result.points, SAMPLE_SCAN)
        self.assertGreaterEqual(link.now_ms - start, 180)   # ~2.3 kB at 11.5 B/ms
        self.assertLess(link.now_ms - start, 600)

    def test_scan_with_lost_chunk_recovered_by_resend(self):
        link, _ = make_link()
        link.connect()
        original = link.transport.write
        state = {"count": 0}

        def drop_second_chunk(direction, frame):
            if direction == "to_host" and frame[3] == p.MsgType.SCAN_CHUNK:
                state["count"] += 1
                if state["count"] == 2:
                    return
            original(direction, frame)

        link.transport.write = drop_second_chunk
        result = link.request_scan()
        self.assertTrue(result.ok)
        self.assertEqual((result.points, result.resent_chunks), (SAMPLE_SCAN, 1))

    def test_scan_gives_up_when_resends_are_lost_then_rescans(self):
        link, _ = make_link()
        link.connect()
        original = link.transport.write
        mute = {"on": True}

        def drop_chunks_after_first(direction, frame):
            if (mute["on"] and direction == "to_host" and frame[3] == p.MsgType.SCAN_CHUNK
                    and frame[p.HEADER_SIZE + 2] > 0):
                return                    # only chunk 0 of each scan arrives
            original(direction, frame)

        link.transport.write = drop_chunks_after_first
        result = link.request_scan(timeout_ms=1000)
        self.assertEqual(result.reason, "incomplete")
        self.assertGreater(result.resent_chunks, 0)
        mute["on"] = False
        self.assertEqual(link.request_scan().points, SAMPLE_SCAN)

    def test_second_session_cannot_connect(self):
        link, _ = make_link()
        link.connect()
        other = MockRobotLink(link.firmware, MockTransport(), session_id=link.session_id + 1)
        with self.assertRaises(LinkError):
            other.connect()

    def test_move_deadline_comes_from_the_robot_clock(self):
        link, _ = make_link()
        link.connect()
        link.arm()
        sent = []
        original = link.transport.write

        def capture(direction, frame):
            if direction == "to_robot" and frame[3] == p.MsgType.MOVE:
                sent.append(p.Move.from_payload(frame[p.HEADER_SIZE:-2]))
            original(direction, frame)

        link.transport.write = capture
        uptime_before = link.firmware.uptime()
        self.assertEqual(link.move(0.05, 0.0).outcome, "completed")
        deadline = sent[0].valid_until_ms
        self.assertGreaterEqual(deadline, uptime_before)
        self.assertLessEqual(deadline, uptime_before + p.COMMAND_TTL_MS + 20)

    def test_close_stops_and_disarms(self):
        link, _ = make_link()
        link.connect()
        link.arm()
        link.close()
        self.assertEqual(link.firmware.state, p.State.DISARMED)
        with self.assertRaises(LinkError):
            link.arm()

    def test_physical_driver_is_refused(self):
        link, _ = make_link()
        link.firmware.motor.KIND = p.MotorDriverKind.PHYSICAL
        with self.assertRaises(LinkError):
            link.connect()

    def test_make_move_units_and_timeout(self):
        move = make_move(0.3, math.radians(-45))
        self.assertEqual((move.forward_mm, move.turn_cdeg, move.speed_mm_s), (300, -4500, 150))
        self.assertEqual(move.timeout_ms, 2000)      # (750 + 2000) x 1.5 + 200, capped
        self.assertEqual(p.validate_move(move), p.NackReason.OUT_OF_BOUNDS)  # 2.75 s > 2 s
        self.assertEqual(make_move(0.1, 0.0).timeout_ms, 1200)       # 666.7 x 1.5 + 200
        self.assertIsNone(p.validate_move(make_move(0.0, math.radians(30))))


class NoHardwareTests(unittest.TestCase):
    def test_real_links_and_transports_refuse(self):
        for cls in (SerialRobotLink, WiFiRobotLink, SerialTransport, UdpTransport):
            with self.subTest(cls=cls.__name__), self.assertRaises(NotImplementedError):
                cls("COM7")

    def test_link_modules_never_import_pyserial(self):
        code = ("import sys, autonomy.robot_link, autonomy.firmware_model, autonomy.protocol;"
                "print('serial' in sys.modules)")
        out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True,
                             cwd=Path(__file__).resolve().parent, check=True)
        self.assertEqual(out.stdout.strip(), "False")


if __name__ == "__main__":
    unittest.main()
