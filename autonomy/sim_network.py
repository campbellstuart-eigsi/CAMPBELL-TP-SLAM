"""Simulated Wi-Fi (UDP datagrams) between the host and the robot (step 2c).

SimNetwork holds two seeded, deterministic Channels (to_robot, to_host).
Each datagram sent on a Channel:
  - is refused if larger than max_datagram (1472 B: Ethernet MTU - IP/UDP);
  - is lost during a cut, by Bernoulli loss, or by the Gilbert-Elliott
    burst-loss model (good / bad states with their own loss rates);
  - waits for the channel (serialisation at bandwidth_bps), then travels
    for latency_ms + uniform(0, jitter_ms): independent delays REORDER it;
  - may be duplicated (the copy gets its own delay) or corrupted (one bit
    flipped; the CRCs reject it);
  - is held until the end of a hold window it was sent in (Wi-Fi buffering:
    released later as a burst);
  - is dropped if it would arrive inside a cut (complete disconnection).

SimNetworkTransport connects a ProtocolRobotLink to a FirmwareModel over a
SimNetwork, as WiFiRobotLink will over a real socket. Ground truth (the
plant) stays on the robot side.

Profiles are assumptions for testing, not measurements: real figures come
from hardware stage H1 (docs/research/esp32_wifi_transport.md, section 4).
"""
import heapq
import random
from collections import Counter
from dataclasses import dataclass

from autonomy.robot_link import Transport

ROBOT_ADDRESS = ("192.168.4.1", 47800)   # SoftAP default address, proposed port
HOST_PEER = ("192.168.4.2", 47801)       # the laptop as the robot sees it
MAX_DATAGRAM = 1472


@dataclass(frozen=True)
class BurstLoss:
    """Gilbert-Elliott model: a two-state Markov chain, one step per datagram."""
    p_enter_bad: float = 0.02
    p_leave_bad: float = 0.25
    loss_good: float = 0.0
    loss_bad: float = 0.7


@dataclass(frozen=True)
class LinkProfile:
    latency_ms: float = 5.0
    jitter_ms: float = 10.0
    loss: float = 0.0
    burst: BurstLoss = None
    duplicate: float = 0.0
    corrupt: float = 0.0
    bandwidth_bps: float = 2_000_000.0   # conservative vs Espressif's 20-30 Mbit/s
    max_datagram: int = MAX_DATAGRAM


PROFILES = {
    "ideal": LinkProfile(latency_ms=1.0, jitter_ms=0.0),
    "typical": LinkProfile(latency_ms=5.0, jitter_ms=15.0, loss=0.01),
    "poor": LinkProfile(latency_ms=20.0, jitter_ms=60.0, loss=0.03, burst=BurstLoss(),
                        duplicate=0.01, corrupt=0.005),
}


class Channel:
    """One direction of the network (see the module docstring)."""

    def __init__(self, profile, rng):
        self.profile = profile
        self.rng = rng
        self.queue = []              # heap of (deliver_ms, order, datagram)
        self.order = 0
        self.busy_until = 0.0
        self.bad_state = False
        self.cuts = []
        self.holds = []
        self.extra_delay = 0.0       # one-shot delay for the next datagram (tests)
        self.last_delivered = -1
        self.stats = Counter()

    def cut(self, start_ms, end_ms):
        self.cuts.append((start_ms, end_ms))

    def hold(self, start_ms, end_ms):
        self.holds.append((start_ms, end_ms))

    def delay_next(self, ms):
        self.extra_delay = float(ms)

    @staticmethod
    def _inside(windows, t):
        return any(start <= t < end for start, end in windows)

    def _lost(self):
        burst = self.profile.burst
        if burst is not None:
            flip = burst.p_leave_bad if self.bad_state else burst.p_enter_bad
            if self.rng.random() < flip:
                self.bad_state = not self.bad_state
            if self.rng.random() < (burst.loss_bad if self.bad_state else burst.loss_good):
                return True
        return self.rng.random() < self.profile.loss

    def send(self, datagram, now_ms):
        profile = self.profile
        if len(datagram) > profile.max_datagram:
            raise ValueError(f"datagram of {len(datagram)} B > {profile.max_datagram} B")
        self.stats["sent"] += 1
        extra, self.extra_delay = self.extra_delay, 0.0
        if self._inside(self.cuts, now_ms):
            self.stats["cut"] += 1
            return
        if self._lost():
            self.stats["lost"] += 1
            return
        start = max(float(now_ms), self.busy_until)
        self.busy_until = start + len(datagram) * 8 * 1000.0 / profile.bandwidth_bps
        copies = 2 if self.rng.random() < profile.duplicate else 1
        self.stats["duplicated"] += copies - 1
        for _ in range(copies):
            data = bytearray(datagram)
            if self.rng.random() < profile.corrupt:
                data[self.rng.randrange(len(data))] ^= 1 << self.rng.randrange(8)
                self.stats["corrupted"] += 1
            deliver = (self.busy_until + profile.latency_ms + extra
                       + self.rng.uniform(0.0, profile.jitter_ms))
            for hold_start, hold_end in self.holds:
                if hold_start <= now_ms < hold_end:
                    deliver = max(deliver, float(hold_end))
                    self.stats["held"] += 1
            heapq.heappush(self.queue, (deliver, self.order, bytes(data)))
        self.order += 1

    def receive(self, now_ms):
        """Datagrams arriving by now_ms, in arrival order."""
        arrived = []
        while self.queue and self.queue[0][0] <= now_ms:
            deliver, order, data = heapq.heappop(self.queue)
            if self._inside(self.cuts, deliver):
                self.stats["cut_in_flight"] += 1
                continue
            if order < self.last_delivered:
                self.stats["reordered"] += 1
            self.last_delivered = max(self.last_delivered, order)
            self.stats["delivered"] += 1
            arrived.append(data)
        return arrived


class SimNetwork:
    """Both directions; seeded, so every run with the same seed is identical."""

    def __init__(self, to_robot=None, to_host=None, seed=0):
        to_robot = to_robot or PROFILES["typical"]
        self.to_robot = Channel(to_robot, random.Random(seed * 2 + 1))
        self.to_host = Channel(to_host or to_robot, random.Random(seed * 2 + 2))

    def channels(self, direction):
        return {"to_robot": (self.to_robot,), "to_host": (self.to_host,),
                "both": (self.to_robot, self.to_host)}[direction]

    def cut(self, start_ms, end_ms, direction="both"):
        for channel in self.channels(direction):
            channel.cut(start_ms, end_ms)

    def hold(self, start_ms, end_ms, direction="both"):
        for channel in self.channels(direction):
            channel.hold(start_ms, end_ms)


class SimNetworkTransport(Transport):
    """A ProtocolRobotLink wire over SimNetwork to a FirmwareModel.

    The firmware sees the host as HOST_PEER and decodes each datagram on its
    own. Frames the firmware addresses to no one (unbound) or to another
    peer are dropped, as UDP would.
    """
    datagrams = True

    def __init__(self, firmware, network=None, plant=None, host_peer=HOST_PEER):
        self.firmware = firmware
        self.network = network or SimNetwork()
        self.plant = plant
        self.host_peer = host_peer
        self._now = firmware.now

    @property
    def now_ms(self):
        return self._now

    def send(self, frame):
        self.network.to_robot.send(frame, self._now)

    def tick(self, dt_ms):
        self._now += dt_ms
        for datagram in self.network.to_robot.receive(self._now):
            self.firmware.receive(datagram, self._now, self.host_peer, datagram=True)
        self.firmware.step(self._now)
        for peer, frame in self.firmware.read_addressed():
            if peer == self.host_peer:
                self.network.to_host.send(frame, self._now)
        received = self.network.to_host.receive(self._now)
        if self.plant is not None:
            motor = self.firmware.motor
            self.plant.advance(motor.forward_mm_s, motor.turn_cdeg_s, motor.enabled, dt_ms)
        return received
