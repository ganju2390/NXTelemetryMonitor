import unittest
from groundstation.controller import Controller
from groundstation.simulator import SimulatedNX
from groundstation.models import Action, Axes, InputSnapshot, Mode
from groundstation.inputs import map_gamepad, ButtonEdges
from groundstation.wire import Codec
from trajectory_tracking import Waypoint, TrackingSettings


class Harness:
    def __init__(self):
        self.pc_wire, self.nx_wire = Codec(), Codec(1)
        self.to_pc, self.to_nx = [], []
        self.pc = Controller(
            lambda k, **f: self.to_nx.append(self.pc_wire.encode(k, **f))
        )
        self.nx = SimulatedNX(
            lambda k, **f: self.to_pc.append(self.nx_wire.encode(k, **f))
        )
        self.now = 100.0
        self.step(3)

    def step(self, count=1, *, input_alive=True, incoming=True):
        for _ in range(count):
            self.now += 0.02
            if input_alive:
                self.pc.input = InputSnapshot(
                    self.now, True, self.pc.input.axes, source="keyboard"
                )
            self.pc.tick(self.now)
            packets, self.to_nx = self.to_nx, []
            for data in packets:
                for m in self.nx_wire.decode(data):
                    self.nx.receive(m, self.now)
            self.nx.tick(self.now, 0.02)
            packets, self.to_pc = self.to_pc, []
            if incoming:
                for data in packets:
                    for m in self.pc_wire.decode(data):
                        self.pc.receive(m, self.now)

    def action(self, action, value, **context):
        self.pc.request(action, value, self.now, context or None)
        self.step(2)

    def manual(self):
        self.action(Action.SET_MODE, Mode.MANUAL)
        self.action(Action.SET_ARMED, 1)


class InputTests(unittest.TestCase):
    def test_original_mapping(self):
        state = map_gamepad([0.25, 0, -0.5, 0.75, -1, 1], [0] * 15, (-1, 1), 5)
        self.assertEqual(state.axes, Axes(-0.25, -0.75, 0.5, 0, 1))
        self.assertEqual(state.hat, (-1, 1))

    def test_edges_are_not_replayed_on_connect(self):
        edges = ButtonEdges()
        b = [0] * 15
        b[6] = 1
        self.assertEqual(edges.update(b), [])
        b[6] = 0
        edges.update(b)
        b[6] = 1
        self.assertEqual(edges.update(b), [6])
        self.assertEqual(edges.update(b), [])
        edges.reset()
        self.assertEqual(edges.update(b), [])

    def test_light_edges_and_deadzone_preserve_scale(self):
        edges = ButtonEdges()
        edges.update([0] * 7)
        self.assertEqual([6, 0, 1], edges.update([1, 1, 0, 0, 0, 0, 1]))
        self.assertEqual([], edges.update([1, 1, 0, 0, 0, 0, 1]))
        state = map_gamepad([0.02, 1, -0.4, 0.5, -1, -1], [0] * 7, (0, 0), 1, 0.05)
        self.assertEqual(Axes(0, -0.5, 0.4, 0, 0), state.axes)


class ControlTests(unittest.TestCase):
    def test_failed_takeover_revokes_trajectory(self):
        h = Harness()
        h.manual()
        h.action(
            Action.SET_MODE,
            Mode.TRAJECTORY,
            points=(Waypoint(0, 0, 0, 1), Waypoint(1, 0, 0, 2)),
            settings=TrackingSettings(),
        )
        h.step(10)
        h.pc.input = InputSnapshot(h.now, True, Axes(forward=1), source="keyboard")
        with self.assertRaises(ValueError):
            h.pc.request(Action.SET_MODE, Mode.MANUAL, h.now)
        h.step(2)
        self.assertEqual(Mode.IDLE, h.pc.mode)
        self.assertEqual(Axes(), h.nx.axes)

    def test_old_session_sequence_and_remaining_validity(self):
        h = Harness()
        h.manual()

        def packet(session, sequence, stamp, forward):
            data = h.pc_wire.encode(
                "FINS_CONTROL_SETPOINT",
                session=session,
                target_session=h.nx.session,
                source_us=int(stamp * 1e6),
                epoch=h.pc.epoch,
                sequence=sequence,
                valid_ms=200,
                yaw=0.0,
                forward=forward,
                left=0.0,
                down=0.0,
                up=0.0,
                depth_target=0.0,
                vertical_mode=0,
            )
            h.nx.receive(h.nx_wire.decode(data)[0], h.now)

        packet(h.pc.session - 1, 900, h.now, 1.0)
        self.assertEqual(Axes(), h.nx.axes)
        packet(h.pc.session, 900, h.now - 0.18, 0.5)
        self.assertEqual(0.5, h.nx.axes.forward)
        packet(h.pc.session, 899, h.now, 1.0)
        self.assertEqual(0.5, h.nx.axes.forward)
        h.nx.tick(h.now + 0.04, 0.04)
        self.assertEqual(Axes(), h.nx.axes)

    def test_out_of_order_arm_does_not_override_disable(self):
        h = Harness()
        h.manual()
        for request, value in ((100, 0), (99, 1)):
            data = h.pc_wire.encode(
                "FINS_ACTION_REQUEST",
                session=h.pc.session,
                target_session=h.nx.session,
                request_id=request,
                epoch=h.pc.epoch,
                action=int(Action.SET_ARMED),
                value=value,
            )
            h.nx.receive(h.nx_wire.decode(data)[0], h.now)
        self.assertFalse(h.nx.armed)

    def test_handshake_manual_and_motor_enable(self):
        h = Harness()
        self.assertTrue(h.pc.ready(h.now))
        self.assertTrue(h.pc.simulated)
        h.manual()
        self.assertEqual(h.pc.mode, Mode.MANUAL)
        self.assertTrue(h.nx.armed)
        h.pc.input = InputSnapshot(h.now, True, Axes(forward=0.5), source="keyboard")
        h.step(10)
        self.assertGreater(h.nx.x, 0)

    def test_manual_disconnect_latches_stop(self):
        h = Harness()
        h.manual()
        h.pc.input = InputSnapshot(h.now, True, Axes(forward=0.5), source="keyboard")
        h.step(2)
        h.step(12, input_alive=False)
        self.assertEqual(h.pc.mode, Mode.IDLE)
        self.assertEqual(h.nx.axes, Axes())
        h.step(3)
        self.assertEqual(h.pc.mode, Mode.IDLE)

    def test_relative_trajectory_and_reset(self):
        h = Harness()
        h.manual()
        h.nx.x, h.nx.y, h.nx.yaw = 2.0, 3.0, 1.57079632679
        h.step(2)
        h.action(
            Action.SET_MODE,
            Mode.TRAJECTORY,
            points=(Waypoint(0, 0, 0, 1), Waypoint(0.5, 0, 0, 2)),
            settings=TrackingSettings(),
        )
        points = h.pc.modes[Mode.TRAJECTORY].points
        self.assertAlmostEqual(points[-1].x_m, 2, places=5)
        self.assertAlmostEqual(points[-1].y_m, 3.5, places=5)
        h.step(20)
        self.assertGreater(h.nx.y, 3)
        h.nx.reset()
        h.step(2)
        self.assertEqual(h.pc.mode, Mode.IDLE)

    def test_invalid_and_stale_odometry_stop_tracking(self):
        for stale in (False, True):
            h = Harness()
            h.manual()
            h.action(
                Action.SET_MODE,
                Mode.TRAJECTORY,
                points=(Waypoint(0, 0, 0, 1), Waypoint(0.5, 0, 0, 2)),
                settings=TrackingSettings(),
            )
            if not stale:
                h.nx.valid = False
            h.step(12, incoming=not stale)
            self.assertEqual(h.pc.mode, Mode.IDLE)
            h.nx.valid = True
            h.step(2)
            self.assertEqual(h.pc.mode, Mode.IDLE)

    def test_switch_rejection_does_not_activate(self):
        h = Harness()
        h.nx.reject_switch = True
        h.action(Action.SET_MODE, Mode.MANUAL)
        self.assertEqual(h.pc.mode, Mode.IDLE)
        self.assertTrue(h.pc.fault)

    def test_missing_ack_times_out_without_activating(self):
        h = Harness()
        h.pc.request(Action.SET_MODE, Mode.MANUAL, h.now)
        # Suppress only ACKs; state traffic must never stand in for an ACK.
        original = h.nx.send_raw
        h.nx.send_raw = lambda kind, **fields: (
            None if kind == "FINS_ACTION_ACK" else original(kind, **fields)
        )
        h.step(70)
        self.assertEqual(h.pc.mode, Mode.IDLE)
        self.assertFalse(h.pc.pending)

    def test_old_epoch_and_expired_packets_are_rejected(self):
        h = Harness()
        h.manual()

        def forged(epoch, stamp):
            data = h.pc_wire.encode(
                "FINS_CONTROL_SETPOINT",
                session=h.pc.session,
                target_session=h.nx.session,
                source_us=int(stamp * 1e6),
                epoch=epoch,
                sequence=10000,
                valid_ms=200,
                yaw=0.0,
                forward=1.0,
                left=0.0,
                down=0.0,
                up=0.0,
                depth_target=0.0,
                vertical_mode=0,
            )
            h.nx.receive(h.nx_wire.decode(data)[0], h.now)

        forged(h.pc.epoch - 1, h.now)
        forged(h.pc.epoch, h.now - 1)
        self.assertEqual(h.nx.axes, Axes())

    def test_duplicate_reset_applies_once(self):
        h = Harness()
        wire = h.pc_wire.encode(
            "FINS_ACTION_REQUEST",
            session=h.pc.session,
            target_session=h.nx.session,
            request_id=123,
            epoch=0,
            action=3,
            value=1,
        )
        for _ in range(2):
            h.nx.receive(h.nx_wire.decode(wire)[0], h.now)
        self.assertEqual(h.nx.reset_id, 1)

    def test_peer_restart_does_not_restore_mode(self):
        h = Harness()
        h.manual()
        h.nx = SimulatedNX(h.nx.send_raw)
        h.step(60)
        self.assertEqual(h.pc.mode, Mode.IDLE)
        self.assertFalse(h.pc.armed)

    def test_corrupt_mavlink_crc(self):
        codec = Codec()
        data = bytearray(codec.encode("TIMESYNC", tc1=0, ts1=123456))
        data[-1] ^= 1
        self.assertEqual(codec.decode(data), [])


if __name__ == "__main__":
    unittest.main()
