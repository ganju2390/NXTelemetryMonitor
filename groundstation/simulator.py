"""Explicitly simulated NX. No serial port or physical actuator access."""

from __future__ import annotations
import argparse
from collections import OrderedDict
import heapq
import math
import random
import secrets
import threading
import time
from .models import Action, Axes, Mode, Result, newer
from .wire import UDPLink
from .generated import finsrov as mav


class SimulatedNX:
    def __init__(self, send, *, drop=0.0, delay=0.0, duplicate=0.0):
        self.send_raw = send
        self.session = secrets.randbits(63) or 1
        self.client = None
        self.mode = Mode.IDLE
        self.armed = False
        self.epoch = 0
        self.faults = 0
        self.sequence = self.aux_sequence = None
        self.last_control = -math.inf
        self.last_aux = -math.inf
        self.control_deadline = self.aux_deadline = -math.inf
        self.axes = Axes()
        self.gimbal = (0, 0)
        self.lights = [False, False]
        self.hold_depth = False
        self.depth_target = self.depth = 0.5
        self.x = self.y = self.yaw = self.vx = self.vy = self.yaw_rate = 0.0
        self.sample_seq = self.reset_id = 0
        self.valid = True
        self.reject_switch = False
        self.reset_requested = False
        self.drop, self.delay, self.duplicate = drop, delay, duplicate
        self.last_telemetry = self.last_status = self.last_heartbeat = -math.inf
        self.acks = OrderedDict()
        self.latest_action = {}
        self.delayed = []
        self.packet_id = 0

    def send(self, kind, now, **fields):
        if random.random() < self.drop:
            return
        copies = 2 if random.random() < self.duplicate else 1
        for _ in range(copies):
            self.packet_id += 1
            heapq.heappush(
                self.delayed, (now + self.delay, self.packet_id, kind, fields.copy())
            )

    def reset(self):
        self.x = self.y = self.yaw = 0.0
        self.vx = self.vy = self.yaw_rate = 0.0
        self.reset_id += 1
        self.mode = Mode.IDLE
        self.axes = Axes()
        self.hold_depth = False

    def receive(self, m, now):
        kind = m.get_type()
        if kind == "FINS_CAPABILITIES" and m.mode_mask == 0 and m.protocol_version == 1:
            if m.session != self.client:
                self.client = m.session
                self.mode, self.armed, self.epoch = Mode.IDLE, False, 0
                self.sequence = self.aux_sequence = None
                self.axes, self.gimbal = Axes(), (0, 0)
                self.hold_depth = False
                self.acks.clear()
                self.latest_action.clear()
            self.send(
                "FINS_CAPABILITIES",
                now,
                session=self.session,
                protocol_version=1,
                mode_mask=7,
                features=3,
                simulated=1,
            )
            return
        if kind == "TIMESYNC" and m.tc1 == 0:
            self.send("TIMESYNC", now, tc1=int(now * 1e9), ts1=m.ts1)
            return
        if (
            getattr(m, "session", None) != self.client
            or getattr(m, "target_session", None) != self.session
        ):
            return
        if kind == "FINS_ACTION_REQUEST":
            self.action(m, now)
        elif kind in ("FINS_CONTROL_SETPOINT", "FINS_AUX_SETPOINT"):
            age = now - m.source_us / 1e6
            previous = (
                self.sequence if kind == "FINS_CONTROL_SETPOINT" else self.aux_sequence
            )
            if (
                m.epoch != self.epoch
                or not newer(m.sequence, previous)
                or not 0 < m.valid_ms <= 200
                or not -0.05 <= age <= m.valid_ms / 1000
            ):
                return
            if kind == "FINS_CONTROL_SETPOINT":
                axes = Axes(m.yaw, m.forward, m.left, m.down, m.up)
                if (
                    not axes.valid()
                    or not math.isfinite(m.depth_target)
                    or m.depth_target < 0
                    or m.vertical_mode not in (0, 1)
                ):
                    return
                self.sequence, self.last_control = m.sequence, now
                self.control_deadline = min(
                    now + 0.2, m.source_us / 1e6 + m.valid_ms / 1000
                )
                self.axes = axes if self.armed and self.mode != Mode.IDLE else Axes()
                self.hold_depth = (
                    bool(m.vertical_mode)
                    and self.armed
                    and self.mode == Mode.TRAJECTORY
                )
                self.depth_target = m.depth_target
                self.faults = 0
            elif m.cam_yaw in (-1, 0, 1) and m.cam_pitch in (-1, 0, 1):
                self.aux_sequence, self.last_aux = m.sequence, now
                self.aux_deadline = min(
                    now + 0.2, m.source_us / 1e6 + m.valid_ms / 1000
                )
                self.gimbal = (
                    (m.cam_yaw, m.cam_pitch) if self.mode == Mode.MANUAL else (0, 0)
                )

    def action(self, m, now):
        key = (m.session, m.request_id)
        fingerprint = (m.action, m.value, m.epoch)
        if key in self.acks:
            previous, ack = self.acks[key]
            if previous == fingerprint:
                self.send("FINS_ACTION_ACK", now, **ack)
            return
        result = Result.ACCEPTED
        reason = mav.FINS_REASON_NONE
        if not newer(m.request_id, self.latest_action.get(m.action)):
            result = Result.INVALID
            reason = mav.FINS_REASON_STALE_REQUEST
        elif m.action == Action.SET_MODE:
            if m.value not in (0, 1, 2) or m.epoch <= self.epoch:
                result = Result.INVALID
                reason = mav.FINS_REASON_BAD_GENERATION
            elif self.reject_switch and m.value != Mode.IDLE:
                result = Result.REJECTED
                reason = mav.FINS_REASON_SWITCH_DENIED
            else:
                self.mode, self.epoch = Mode(m.value), m.epoch
                self.axes, self.gimbal = Axes(), (0, 0)
                self.hold_depth = False
                self.sequence = self.aux_sequence = None
                self.last_control = now
                self.control_deadline = now + 0.2
                self.faults = 0
        elif m.action == Action.SET_ARMED and m.value == 0:
            # Emergency disable remains valid even if a mode switch ACK is in flight.
            self.armed = False
            self.axes, self.gimbal = Axes(), (0, 0)
            self.hold_depth = False
        elif m.epoch != self.epoch:
            result = Result.INVALID
            reason = mav.FINS_REASON_BAD_GENERATION
        elif m.action == Action.SET_ARMED and m.value == 1:
            self.armed = True
        elif m.action == Action.RESET_ODOMETRY:
            self.reset()
        elif m.action in (
            Action.SET_CAMERA_LIGHT,
            Action.SET_FRAME_LIGHT,
        ) and m.value in (0, 1):
            self.lights[m.action - 4] = bool(m.value)
        else:
            result = Result.INVALID
            reason = mav.FINS_REASON_BAD_VALUE
        self.latest_action[m.action] = max(
            m.request_id, self.latest_action.get(m.action, 0)
        )
        ack = dict(
            session=self.session,
            target_session=m.session,
            request_id=m.request_id,
            epoch=m.epoch,
            action=m.action,
            result=int(result),
            reason=reason,
        )
        self.acks[key] = (fingerprint, ack)
        while len(self.acks) > 128:
            self.acks.popitem(last=False)
        self.send("FINS_ACTION_ACK", now, **ack)

    def tick(self, now, dt):
        if self.reset_requested:
            self.reset_requested = False
            self.reset()
        if now > self.control_deadline:
            self.axes = Axes()
            self.hold_depth = False
            if self.mode != Mode.IDLE:
                self.mode = Mode.IDLE
                self.faults = 1
        if now > self.aux_deadline:
            self.gimbal = (0, 0)
        dt = min(0.05, max(0.0, dt))
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.vx = c * self.axes.forward - s * self.axes.left
        self.vy = s * self.axes.forward + c * self.axes.left
        self.yaw_rate = 0.4 * self.axes.yaw
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.yaw = (self.yaw + self.yaw_rate * dt + math.pi) % (2 * math.pi) - math.pi
        vertical = (
            max(-0.3, min(0.3, self.depth_target - self.depth))
            if self.hold_depth
            else 0.3 * (self.axes.down - self.axes.up)
        )
        self.depth = max(0.0, self.depth + vertical * dt)
        if now - self.last_telemetry >= 0.019:
            self.sample_seq = (self.sample_seq + 1) & 0xFFFFFFFF
            self.send(
                "FINS_ODOMETRY",
                now,
                session=self.session,
                source_us=int(now * 1e6),
                sample_seq=self.sample_seq,
                reset_id=self.reset_id,
                x=self.x,
                y=self.y,
                yaw=self.yaw,
                vx=self.vx,
                vy=self.vy,
                yaw_rate=self.yaw_rate,
                depth=self.depth,
                valid=int(self.valid),
                depth_valid=1,
            )
            self.send(
                "FINS_TELEMETRY",
                now,
                session=self.session,
                source_us=int(now * 1e6),
                valid_mask=31,
                voltage=24.0,
                pressure=[101325 + 9806.65 * self.depth] * 4,
                rpm=[1200 * self.axes.forward] * 8,
                accel=[0.0, 0.0, 9.80665],
                gyro=[0.0, 0.0, self.yaw_rate],
                network=[self.axes.forward, self.axes.left, self.yaw_rate],
            )
            self.last_telemetry = now
        if now - self.last_status >= 0.1:
            self.send(
                "FINS_STATUS",
                now,
                session=self.session,
                client_session=self.client or 0,
                epoch=self.epoch,
                mode=int(self.mode),
                armed=int(self.armed),
                faults=self.faults,
                hold_depth=int(self.hold_depth),
                depth_target=self.depth_target,
                camera_light=int(self.lights[0]),
                frame_light=int(self.lights[1]),
            )
            self.last_status = now
        if now - self.last_heartbeat >= 1:
            self.send(
                "HEARTBEAT",
                now,
                type=12,
                autopilot=8,
                base_mode=128 if self.armed else 0,
                custom_mode=int(self.mode),
                system_status=4,
                mavlink_version=3,
            )
            self.last_heartbeat = now
        while self.delayed and self.delayed[0][0] <= now:
            _, _, kind, fields = heapq.heappop(self.delayed)
            self.send_raw(kind, **fields)


class SimulatorThread(threading.Thread):
    def __init__(self, bind=("127.0.0.1", 54322), peer=("127.0.0.1", 54321), **faults):
        super().__init__(name="SIMULATED-NX", daemon=True)
        self.link = UDPLink(bind, peer, system=1)
        self.model = SimulatedNX(self.link.send, **faults)
        self.stopping = threading.Event()
        self.error = ""

    def run(self):
        self.link.start()
        previous = time.monotonic()
        try:
            while not self.stopping.wait(0.005):
                now = time.monotonic()
                for _ in range(512):
                    try:
                        message, received = self.link.incoming.get_nowait()
                    except __import__("queue").Empty:
                        break
                    self.model.receive(message, received)
                self.model.tick(now, now - previous)
                previous = now
        except Exception as error:
            self.error = str(error)
        finally:
            self.link.close()

    def close(self):
        self.stopping.set()
        if self.is_alive():
            self.join(3)
        else:
            self.link.close()


def main():
    parser = argparse.ArgumentParser(description="SIMULATED NX — no hardware access")
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--pc", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=54322)
    parser.add_argument("--pc-port", type=int, default=54321)
    parser.add_argument("--delay-ms", type=float, default=0)
    parser.add_argument("--drop", type=float, default=0)
    parser.add_argument("--duplicate", type=float, default=0)
    parser.add_argument("--invalid-odometry", action="store_true")
    parser.add_argument("--reject-switch", action="store_true")
    parser.add_argument("--reset-after", type=float, default=0)
    args = parser.parse_args()
    if not 0 <= args.drop <= 1 or not 0 <= args.duplicate <= 1 or args.delay_ms < 0:
        parser.error("probabilities must be [0,1], delay nonnegative")
    worker = SimulatorThread(
        (args.bind, args.port),
        (args.pc, args.pc_port),
        drop=args.drop,
        delay=args.delay_ms / 1000,
        duplicate=args.duplicate,
    )
    worker.model.valid = not args.invalid_odometry
    worker.model.reject_switch = args.reject_switch
    worker.start()
    print("SIMULATED NX running; Ctrl+C to stop.")
    started = time.monotonic()
    try:
        while worker.is_alive():
            time.sleep(0.1)
            if args.reset_after and time.monotonic() - started >= args.reset_after:
                worker.model.reset_requested = True
                args.reset_after = 0
    except KeyboardInterrupt:
        pass
    finally:
        worker.close()
    if worker.error or worker.link.error:
        raise RuntimeError(worker.error or worker.link.error)


if __name__ == "__main__":
    main()
