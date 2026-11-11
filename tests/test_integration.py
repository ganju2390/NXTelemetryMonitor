"""Real loopback UDP, real threads and generated MAVLink. No physical NX."""

import csv
from dataclasses import asdict
import json
from pathlib import Path
import socket
import tempfile
import time
import unittest
from apriltag_localization import VisionInterpolator, VisionSnapshot
from groundstation.models import Action, Axes, Config, InputSnapshot, Mode
from groundstation.queues import latest_get, latest_put
from groundstation.recording import SessionRecorder
from groundstation.runtime import ControlWorker
from groundstation.simulator import SimulatorThread
from trajectory_tracking import Waypoint, TrackingSettings


def free_ports():
    sockets = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(2)]
    try:
        for s in sockets:
            s.bind(("127.0.0.1", 0))
        return tuple(s.getsockname()[1] for s in sockets)
    finally:
        for s in sockets:
            s.close()


class UDPIntegrationTests(unittest.TestCase):
    def setUp(self):
        pc, nx = free_ports()
        self.nx = SimulatorThread(("127.0.0.1", nx), ("127.0.0.1", pc))
        self.pc = ControlWorker(
            Config(bind_ip="127.0.0.1", port=pc, nx_ip="127.0.0.1", nx_port=nx)
        )
        self.snapshot = None
        self.axes = Axes()
        self.nx.start()
        self.pc.start()
        self.wait(lambda s: s["ready"])

    def tearDown(self):
        self.pc.close()
        self.nx.close()
        self.assertFalse(self.pc.is_alive())
        self.assertFalse(self.nx.is_alive())
        self.assertFalse(self.pc.link.is_alive())
        self.assertFalse(self.nx.link.is_alive())
        self.assertEqual("", self.pc.error)
        self.assertEqual("", self.nx.error)

    def wait(self, predicate, timeout=4):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            latest_put(
                self.pc.inputs,
                InputSnapshot(time.monotonic(), True, self.axes, source="keyboard"),
            )
            self.snapshot = latest_get(self.pc.snapshots, self.snapshot)
            if self.snapshot and predicate(self.snapshot):
                return self.snapshot
            if self.pc.error:
                self.fail(self.pc.error)
            time.sleep(0.01)
        self.fail(f"timeout: {self.snapshot}")

    def manual(self):
        self.pc.command(Action.SET_MODE, Mode.MANUAL)
        self.wait(lambda s: s["mode"] == Mode.MANUAL and not s["pending"])
        self.pc.command(Action.SET_ARMED, 1)
        self.wait(lambda s: s["armed"] and not s["pending"])

    def trajectory(self):
        self.pc.command(
            Action.SET_MODE,
            Mode.TRAJECTORY,
            dict(
                points=(Waypoint(0, 0, 0, 0.2), Waypoint(0.4, 0, 0, 1)),
                settings=TrackingSettings(),
            ),
        )
        self.wait(lambda s: s["mode"] == Mode.TRAJECTORY)

    def test_manual_trajectory_stop_takeover_and_csv(self):
        with tempfile.TemporaryDirectory() as temp:
            recorder = SessionRecorder(
                Path(temp) / "session",
                dict(config=asdict(self.pc.config)),
                VisionInterpolator(),
            )
            recorder.start()
            self.pc.recorder = recorder
            try:
                self.manual()
                self.axes = Axes(forward=0.4)
                self.wait(lambda s: s["odometry"] and s["odometry"].pose.x_m > 0.05)
                self.axes = Axes()
                self.wait(lambda s: s["input"].axes.neutral())
                self.trajectory()
                self.wait(lambda s: s["output"].complete, 12)
                self.assertEqual(Axes(), self.snapshot["output"].axes)
                self.assertAlmostEqual(0.5, self.snapshot["output"].depth_target)
                self.pc.stop_motion()
                self.wait(lambda s: s["mode"] == Mode.IDLE and not s["pending"])
                self.axes = Axes(forward=0.4)
                self.wait(lambda s: s["input"].axes.forward > 0.3)
                self.pc.command(Action.SET_MODE, Mode.MANUAL)
                self.wait(lambda s: "回中" in s["message"])
                self.assertEqual(Mode.IDLE, self.snapshot["mode"])
                self.axes = Axes()
                self.wait(lambda s: s["input"].axes.neutral())
                self.pc.command(Action.SET_MODE, Mode.MANUAL)
                self.wait(lambda s: s["mode"] == Mode.MANUAL)
                self.pc.command(Action.SET_ARMED, 0)
                self.wait(lambda s: not s["armed"])
            finally:
                self.pc.recorder = None
                recorder.close()
            self.assertEqual("", recorder.error)
            self.assertFalse(recorder.is_alive())
            with (recorder.directory / "telemetry.csv").open(
                encoding="utf-8-sig"
            ) as stream:
                rows = list(csv.DictReader(stream))
            self.assertGreater(len(rows), 20)
            self.assertTrue(all(r["tag_x_m"] == "" for r in rows))
            self.assertTrue({"MANUAL", "TRAJECTORY"} <= {r["mode"] for r in rows})
            self.assertEqual(0, recorder.aligned)
            self.assertEqual(
                1,
                json.loads((recorder.directory / "session.json").read_text())[
                    "protocol_version"
                ],
            )

    def test_faults_and_reconnect_do_not_resume(self):
        self.manual()
        self.trajectory()
        self.nx.model.valid = False
        self.wait(lambda s: s["mode"] == Mode.IDLE and s["fault"])
        self.nx.model.valid = True
        self.wait(lambda s: s["odometry"].valid)
        self.assertEqual(Mode.IDLE, self.snapshot["mode"])
        self.trajectory()
        self.nx.model.reset_requested = True
        self.wait(lambda s: s["mode"] == Mode.IDLE and s["odometry"].reset_id == 1)
        self.trajectory()
        self.nx.model.delay = 0.35
        self.wait(lambda s: s["mode"] == Mode.IDLE and s["fault"])
        self.nx.model.delay = 0
        self.wait(
            lambda s: (
                s["odometry"].valid and time.monotonic() - s["odometry"].received < 0.1
            )
        )
        self.assertEqual(Mode.IDLE, self.snapshot["mode"])
        self.nx.model.drop = 1
        self.wait(lambda s: not s["ready"], 4)
        self.nx.model.drop = 0
        self.wait(lambda s: s["ready"])
        self.assertEqual(Mode.IDLE, self.snapshot["mode"])

    def test_camera_and_disk_failure_do_not_block_controller(self):
        from groundstation.camera import CameraLocalisationWorker
        import queue

        camera = CameraLocalisationWorker(
            99,
            queue.Queue(maxsize=1),
            VisionInterpolator(),
            Path("missing.yaml"),
            Path("missing.json"),
        )
        with tempfile.TemporaryDirectory() as temp:
            recorder = SessionRecorder(
                Path(temp), {}, VisionInterpolator()
            )  # directory collision: deliberate writer failure
            start = self.snapshot["ticks"]
            recorder.start()
            camera.start()
            self.pc.recorder = recorder
            self.wait(lambda s: s["ticks"] >= start + 25)
            camera.join(1)
            recorder.close()
            self.pc.recorder = None
            self.assertTrue(recorder.error)
            self.assertFalse(camera.is_alive())
            self.assertEqual("", self.pc.error)


class RecordingTests(unittest.TestCase):
    def test_strict_interpolation_and_missing_tag_rows(self):
        interp = VisionInterpolator()
        for t, x in ((10.0, 0.0), (10.2, 0.1)):
            interp.push(
                VisionSnapshot(True, x, 0.0, 0.0, (10,), 0.1, t, "2026-01-01", "ok")
            )
        with tempfile.TemporaryDirectory() as temp:
            recorder = SessionRecorder(Path(temp) / "session", {}, interp)
            recorder.start()
            for t in (10.1, 11.0):
                recorder.record(dict(received_utc="test", source_pc_monotonic=t))
            recorder.close()
            self.assertEqual(2, recorder.written)
            self.assertEqual(1, recorder.aligned)
            with (recorder.directory / "telemetry.csv").open(
                encoding="utf-8-sig"
            ) as stream:
                rows = list(csv.DictReader(stream))
            self.assertAlmostEqual(0.05, float(rows[0]["tag_x_m"]))
            self.assertEqual("", rows[1]["tag_x_m"])

    def test_local_camera_mp4_is_finalized(self):
        import cv2
        import numpy as np
        import queue
        from groundstation.camera import CameraLocalisationWorker

        camera = CameraLocalisationWorker(
            0,
            queue.Queue(maxsize=1),
            VisionInterpolator(),
            Path("missing"),
            Path("missing"),
        )
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "camera.mp4"
            camera.start_recording(path)
            writer = None
            for _ in range(4):
                writer = camera._write_video_frame(
                    cv2, writer, np.zeros((1080, 1920, 3), dtype=np.uint8)
                )
            self.assertIsNotNone(writer, camera.recording_status())
            camera.stop_recording()
            writer = camera._write_video_frame(
                cv2, writer, np.zeros((1080, 1920, 3), dtype=np.uint8)
            )
            self.assertIsNone(writer)
            cap = cv2.VideoCapture(str(path))
            try:
                self.assertEqual(4, int(cap.get(cv2.CAP_PROP_FRAME_COUNT)))
                self.assertTrue(cap.read()[0])
            finally:
                cap.release()


if __name__ == "__main__":
    unittest.main()
