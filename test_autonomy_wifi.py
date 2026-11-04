"""Protocol v2 over a simulated Wi-Fi network (autonomy/sim_network.py). No hardware.

    python -m unittest test_autonomy_wifi -v

The plant used here checks FirmwareModel.invariant_violations() on EVERY
simulated tick and raises at once, so no test can pass through a moment
where the driver is enabled outside ARMED/MOVING, outputs are non-zero
outside MOVING, or the robot is armed without a controller.

The soak test length (simulated minutes) is AUTONOMY_SOAK_MINUTES (default 30).
"""
import math
import os
import random
import unittest

from autonomy import protocol as p
from autonomy.firmware_model import FirmwareModel
from autonomy.robot_link import LinkError, MockRobotLink, ProtocolRobotLink
from autonomy.sim_network import (PROFILES, BurstLoss, LinkProfile, SimNetwork,
                                  SimNetworkTransport)

SAMPLE_SCAN = tuple((i * 79 % 36000, 800 + i, 100) for i in range(456))
SESSION = 0xA11CE


class SafetyError(AssertionError):
    pass


class MonitorPlant:
    """Integrates the requested body velocity; checks the invariants every tick."""

    def __init__(self, firmware):
        self.firmware = firmware
        self.distance_mm = 0.0
        self.turn_cdeg = 0.0
        self.motion_ticks = 0
        self.hook = None                    # optional per-tick callback (scenario events)

    def advance(self, forward_mm_s, turn_cdeg_s, enabled, dt_ms):
        problems = self.firmware.invariant_violations()
        if problems:
            raise SafetyError(f"at {self.firmware.now} ms: {problems}")
        if (forward_mm_s or turn_cdeg_s) and not enabled:
            raise SafetyError("motion requested while the driver is disabled")
        self.distance_mm += abs(forward_mm_s) * dt_ms / 1000.0
        self.turn_cdeg += turn_cdeg_s * dt_ms / 1000.0
        self.motion_ticks += bool(forward_mm_s or turn_cdeg_s)
        if self.hook:
            self.hook(self)


def make_wifi_link(profile="typical", seed=0, to_host=None, scan=True):
    profile = PROFILES[profile] if isinstance(profile, str) else profile
    firmware = FirmwareModel(boot_id=21, scan_provider=(lambda: SAMPLE_SCAN) if scan else None)
    network = SimNetwork(profile, to_host, seed=seed)
    plant = MonitorPlant(firmware)
    link = ProtocolRobotLink(SimNetworkTransport(firmware, network, plant), SESSION)
    return link, plant, network, firmware


def connect_and_arm(link, attempts=5):
    for _ in range(attempts):
        try:
            link.connect()
            break
        except LinkError:
            continue
    for _ in range(attempts):
        result = link.arm()
        if result.ok:
            return result
        if result.reason == "FAULT_ACTIVE":
            link.clear_fault()
    return result


class WifiBasicsTests(unittest.TestCase):
    def test_typical_wifi_moves_and_scans(self):
        outcomes = []
        for seed in range(10):
            link, plant, net, fw = make_wifi_link("typical", seed)
            self.assertTrue(connect_and_arm(link).ok, f"seed {seed}")
            result = link.move(0.2, math.radians(20))
            outcomes.append(result.outcome)
            self.assertFalse(fw.motor.moving())
            if result.completed:
                self.assertAlmostEqual(plant.distance_mm, 200, delta=4)
            self.assertTrue(link.request_scan().ok, f"seed {seed}")
        self.assertGreaterEqual(outcomes.count("completed"), 9, outcomes)

    def test_frames_fit_in_one_datagram(self):
        self.assertLess(p.MAX_FRAME, PROFILES["typical"].max_datagram)
        net = SimNetwork(PROFILES["ideal"])
        with self.assertRaises(ValueError):
            net.to_robot.send(bytes(1473), 0)

    def test_scan_rate_over_wifi_versus_serial(self):
        link, _, _, _ = make_wifi_link("typical", 1)
        link.connect()
        start, count = link.now_ms, 0
        while link.now_ms - start < 2000:
            count += link.request_scan().ok
        wifi_rate = count * 1000.0 / (link.now_ms - start)
        serial = MockRobotLink(FirmwareModel(scan_provider=lambda: SAMPLE_SCAN))
        serial.connect()
        start, count = serial.now_ms, 0
        while serial.now_ms - start < 2000:
            count += serial.request_scan().ok
        serial_rate = count * 1000.0 / (serial.now_ms - start)
        self.assertGreaterEqual(wifi_rate, 10.0)      # supports continuous 10 Hz (M4)
        self.assertLess(serial_rate, 5.0)             # 115200 baud cannot
        self.assertGreater(wifi_rate, 2 * serial_rate)


class ReorderDuplicateDelayTests(unittest.TestCase):
    def test_heavy_jitter_reorders_but_moves_once(self):
        jittery = LinkProfile(latency_ms=5.0, jitter_ms=80.0, loss=0.0)
        reordered = 0
        for seed in range(8):
            link, plant, net, fw = make_wifi_link(jittery, seed)
            self.assertTrue(connect_and_arm(link).ok)
            result = link.move(0.2, 0.0, speed_m_s=0.2)
            reordered += net.to_robot.stats["reordered"] + net.to_host.stats["reordered"]
            self.assertFalse(fw.motor.moving())
            self.assertLessEqual(plant.distance_mm, 204)          # never executed twice
            if result.completed:
                self.assertGreaterEqual(plant.distance_mm, 196)
        self.assertGreater(reordered, 0)

    def test_every_datagram_duplicated(self):
        doubled = LinkProfile(latency_ms=3.0, jitter_ms=5.0, duplicate=1.0)
        link, plant, net, fw = make_wifi_link(doubled, 2)
        self.assertTrue(connect_and_arm(link).ok)
        self.assertEqual(link.move(0.2, 0.0).outcome, "completed")
        self.assertAlmostEqual(plant.distance_mm, 200, delta=3)
        self.assertGreater(net.to_robot.stats["duplicated"], 10)
        self.assertEqual(link.request_scan().points, SAMPLE_SCAN)

    def test_older_move_arriving_late_is_never_executed(self):
        link, plant, net, fw = make_wifi_link(PROFILES["ideal"], 3)
        self.assertTrue(connect_and_arm(link).ok)
        seq_old = link._next_seq()
        old = p.Move(100, 0, 100, 9000, 2000, link._valid_until())
        net.to_robot.delay_next(600)                         # stuck in the network
        link.wire.send(p.encode(old, seq_old))
        result = link.move(0.2, 0.0)                         # newer seq, sent after it
        self.assertEqual(result.outcome, "completed")
        self.assertAlmostEqual(plant.distance_mm, 200, delta=3)   # only the newer move
        link.pump(500)
        self.assertFalse(fw.motor.moving())

    def test_move_delayed_past_its_deadline_is_refused(self):
        slow = LinkProfile(latency_ms=160.0, jitter_ms=0.0)   # RTT 320 ms > TTL 250 ms
        link, plant, net, fw = make_wifi_link(slow, 4)
        self.assertTrue(connect_and_arm(link).ok)             # heartbeats still accepted
        result = link.move(0.1, 0.0)
        self.assertEqual((result.outcome, result.detail), ("refused", "EXPIRED"))
        self.assertEqual(plant.motion_ticks, 0)

    def test_buffered_heartbeats_cannot_revive_the_watchdog(self):
        link, plant, net, fw = make_wifi_link(PROFILES["ideal"], 5)
        self.assertTrue(connect_and_arm(link).ok)
        held = {}

        def hold_mid_move(monitor):
            if monitor.distance_mm > 50 and not held:
                held["at"] = link.now_ms
                net.hold(link.now_ms, link.now_ms + 1500, "to_robot")

        plant.hook = hold_mid_move
        result = link.move(0.3, 0.0)
        plant.hook = None
        self.assertEqual(result.outcome, "fault")             # telemetry still reaches us
        self.assertTrue(result.faults & p.Fault.HEARTBEAT_TIMEOUT)
        rejected = fw.rejected_heartbeats
        link.pump(1600)                                       # the held burst arrives
        self.assertGreater(fw.rejected_heartbeats, rejected + 5)
        self.assertEqual(fw.state, p.State.FAULT)             # not revived
        self.assertFalse(fw.motor.enabled)


class OutageTests(unittest.TestCase):
    def cut_during_move(self, direction, profile="typical", seed=6):
        link, plant, net, fw = make_wifi_link(profile, seed)
        self.assertTrue(connect_and_arm(link).ok)
        cut = {}

        def cut_mid_move(monitor):
            if monitor.distance_mm > 60 and not cut:
                cut["at"] = link.now_ms
                net.cut(link.now_ms, link.now_ms + 2500, direction)

        plant.hook = cut_mid_move
        result = link.move(0.3, 0.0)
        plant.hook = None
        self.assertFalse(fw.motor.moving(), "move() returned while the robot moved")
        return link, plant, net, fw, result, cut["at"]

    def test_complete_cut_stops_within_the_watchdog_bound(self):
        link, plant, net, fw, result, cut_at = self.cut_during_move("both")
        self.assertEqual(result.outcome, "link_lost")
        self.assertEqual(fw.state, p.State.FAULT)
        off = [t for t, enabled, *_ in fw.motor.history if not enabled][-1]
        self.assertLessEqual(off - fw.last_heartbeat_ms, p.HEARTBEAT_TIMEOUT_MS + 10)
        self.assertLessEqual(off - cut_at, p.HEARTBEAT_TIMEOUT_MS + p.HEARTBEAT_PERIOD_MS + 40)
        # After the outage: reconnect, but no automatic resume.
        link.pump(2500)
        distance = plant.distance_mm
        link.connect()
        self.assertEqual(link.arm().reason, "FAULT_ACTIVE")
        self.assertTrue(link.clear_fault().ok)
        self.assertTrue(link.arm().ok)
        link.pump(1000)
        self.assertEqual(plant.distance_mm, distance)

    def test_cut_robot_to_host_only(self):
        # Heartbeats keep reaching the robot; the host hears nothing and
        # must stop it itself (STOP, then starving the watchdog).
        link, plant, net, fw, result, _ = self.cut_during_move("to_host")
        self.assertEqual(result.outcome, "link_lost")
        self.assertIn("watchdog starved", result.detail)

    def test_cut_host_to_robot_only(self):
        link, plant, net, fw, result, _ = self.cut_during_move("to_robot")
        self.assertEqual(result.outcome, "fault")              # the robot reports its fault
        self.assertTrue(result.faults & p.Fault.HEARTBEAT_TIMEOUT)

    def test_cut_during_scan_then_rescan(self):
        link, plant, net, fw = make_wifi_link("typical", 7)
        link.connect()
        cut_end = link.now_ms + 1200
        net.cut(link.now_ms + 25, cut_end, "to_host")
        first = link.request_scan(timeout_ms=1000)
        self.assertFalse(first.ok)                 # its ACK or chunks were cut
        link.pump(max(0, cut_end - link.now_ms) + 100)
        self.assertEqual(link.request_scan().points, SAMPLE_SCAN)

    def test_idle_armed_robot_faults_during_an_outage(self):
        link, plant, net, fw = make_wifi_link("typical", 8)
        self.assertTrue(connect_and_arm(link).ok)
        net.cut(link.now_ms, link.now_ms + 1000)
        link.pump(1000)
        self.assertEqual(fw.state, p.State.FAULT)
        self.assertEqual(plant.motion_ticks, 0)

    def test_restart_during_an_outage(self):
        link, plant, net, fw = make_wifi_link(PROFILES["ideal"], 9)
        self.assertTrue(connect_and_arm(link).ok)

        def cut_and_reboot(monitor):
            if monitor.distance_mm > 40 and fw.boot_id == 21:
                net.cut(link.now_ms, link.now_ms + 1500)
                fw.restart(link.now_ms, boot_id=22)

        plant.hook = cut_and_reboot
        result = link.move(0.3, 0.0)
        plant.hook = None
        self.assertEqual(result.outcome, "link_lost")          # unbound robot sends nothing
        self.assertFalse(fw.motor.moving())
        link.pump(1600)
        hello = link.connect()
        self.assertEqual(hello.boot_id, 22)
        self.assertTrue(link.arm().ok)
        self.assertEqual(link.move(0.1, 0.0).outcome, "completed")

    def test_burst_loss_is_always_safe(self):
        bursty = LinkProfile(latency_ms=8.0, jitter_ms=20.0,
                             burst=BurstLoss(p_enter_bad=0.05, p_leave_bad=0.2, loss_bad=0.9))
        outcomes = []
        for seed in range(10):
            link, plant, net, fw = make_wifi_link(bursty, seed)
            if not connect_and_arm(link).ok:
                outcomes.append("not armed")
                continue
            result = link.move(0.2, 0.0, speed_m_s=0.2)
            outcomes.append(result.outcome)
            self.assertFalse(fw.motor.moving())
            self.assertLessEqual(plant.distance_mm, 204)
        self.assertGreater(outcomes.count("completed"), 0, outcomes)


class SoakTest(unittest.TestCase):
    MINUTES = float(os.environ.get("AUTONOMY_SOAK_MINUTES", "30"))

    def test_long_random_operation_on_a_poor_network(self):
        rng = random.Random(2026)
        link, plant, net, fw = make_wifi_link("poor", 11)
        connect_and_arm(link)
        end = link.now_ms + self.MINUTES * 60_000
        stats = {}

        def count(key):
            stats[key] = stats.get(key, 0) + 1

        while link.now_ms < end:
            action = rng.random()
            if action < 0.03:                                  # a random outage
                length = rng.choice((200, 600, 1500, 4000))
                net.cut(link.now_ms + rng.randrange(0, 500), link.now_ms + length,
                        rng.choice(("both", "to_robot", "to_host")))
                count("cuts")
            if action < 0.55:
                result = link.move(rng.uniform(-0.3, 0.3), rng.uniform(-0.6, 0.6),
                                   speed_m_s=0.2, turn_rate_rad_s=1.2)
                count("move_" + result.outcome + (
                    "_" + result.detail.split(":")[0] if result.outcome == "refused" else ""))
                self.assertFalse(fw.motor.moving(), f"move() returned while moving: {result}")
                if result.outcome in ("fault", "link_lost", "restarted"):
                    connect_and_arm(link, attempts=3)
                elif result.outcome == "refused" and result.detail in (
                        "NOT_ARMED", "FAULT_ACTIVE", "NO_HEARTBEAT", "NOT_CONTROLLER"):
                    connect_and_arm(link, attempts=3)
            elif action < 0.80:
                count("scan_" + ("ok" if link.request_scan().ok else "failed"))
            elif action < 0.95:
                link.pump(rng.choice((100, 500, 2000)))
            else:
                link.stop()
                link.pump(100)
                count("stops")
        stats["rejected_heartbeats"] = fw.rejected_heartbeats
        stats["minutes"] = self.MINUTES
        print(f"\n  soak summary: {dict(sorted(stats.items()))}")
        self.assertGreater(stats.get("move_completed", 0), 10 * self.MINUTES)
        self.assertGreater(stats.get("scan_ok", 0), 5 * self.MINUTES)


if __name__ == "__main__":
    unittest.main()
