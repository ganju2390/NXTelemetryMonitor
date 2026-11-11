import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from pathlib import Path
import shutil
import subprocess
import tempfile
import time
import unittest

from PySide6 import QtCore, QtWidgets, QtTest
from groundstation.app import MainWindow, configure_font
from groundstation.models import Config, Mode
from groundstation.video import VideoReceiver
from .test_integration import free_ports


class VideoTests(unittest.TestCase):
    def test_missing_runtime_is_reported(self):
        receiver = VideoReceiver(executable="definitely-missing-gst.exe")
        receiver.start()
        receiver.join(2)
        self.assertFalse(receiver.is_alive())
        self.assertIn("GStreamer", receiver.error)
        receiver.close()

    def test_rtp_decode_disconnect_recovery_and_process_exit(self):
        exe = os.environ.get("GST_LAUNCH") or shutil.which("gst-launch-1.0")
        if not exe:
            self.skipTest("Set GST_LAUNCH or install GStreamer to test real H264/RTP")
        port = free_ports()[0]
        receiver = VideoReceiver(
            executable=exe, port=port, width=320, height=240, latency=30
        )
        sender = None

        def start_sender():
            return subprocess.Popen(
                [
                    exe,
                    "-q",
                    "videotestsrc",
                    "is-live=true",
                    "!",
                    "video/x-raw,width=320,height=240,framerate=15/1",
                    "!",
                    "videoconvert",
                    "!",
                    "x264enc",
                    "tune=zerolatency",
                    "speed-preset=ultrafast",
                    "key-int-max=15",
                    "!",
                    "rtph264pay",
                    "pt=96",
                    "config-interval=1",
                    "!",
                    "udpsink",
                    "host=127.0.0.1",
                    f"port={port}",
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0,
            )

        try:
            receiver.start()
            sender = start_sender()
            self.assertEqual(
                320 * 240 * 3, len(receiver.frames.get(timeout=12)), receiver.error
            )
            sender.terminate()
            sender.wait(3)
            sender = None
            time.sleep(2.3)
            self.assertIn("断流", receiver.status)
            while not receiver.frames.empty():
                receiver.frames.get_nowait()
            sender = start_sender()
            self.assertEqual(320 * 240 * 3, len(receiver.frames.get(timeout=12)))
        finally:
            if sender:
                sender.terminate()
                sender.wait(3)
            receiver.close()
        self.assertFalse(receiver.is_alive())
        self.assertIsNotNone(receiver.process.poll())


class WindowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        configure_font(cls.app)

    def wait(self, predicate, timeout=4):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.app.processEvents()
            if predicate():
                return
            QtTest.QTest.qWait(10)
        self.fail(self.window.notice.text())

    def test_no_devices_simulation_controls_and_shutdown(self):
        pc, nx = free_ports()
        self.window = MainWindow(
            Config(port=pc, nx_port=nx), simulate=True, video_test=True
        )
        w = self.window
        w.show()
        w.activateWindow()
        with tempfile.TemporaryDirectory() as temp:
            try:
                self.wait(lambda: w.snapshot and w.snapshot["ready"])
                w.source.setCurrentIndex(1)
                w.manual_button.setFocus()
                self.wait(
                    lambda: (
                        w.snapshot["input"].connected
                        and w.snapshot["input"].source == "keyboard"
                    )
                )
                self.wait(lambda: not w.snapshot["pending"])
                w.manual_button.click()
                self.wait(lambda: w.snapshot["mode"] == Mode.MANUAL)
                button = next(
                    b
                    for b in w.findChildren(QtWidgets.QPushButton)
                    if b.text() == "启用电机"
                )
                button.click()
                self.wait(lambda: w.snapshot["armed"])
                w.record_root.setText(temp)
                w.start_recording()
                QtTest.QTest.keyPress(w, QtCore.Qt.Key.Key_W)
                self.wait(lambda: w.snapshot["odometry"].pose.x_m > 0.04)
                QtTest.QTest.keyRelease(w, QtCore.Qt.Key.Key_W)
                self.wait(lambda: w.snapshot["input"].axes.neutral())
                trajectory = Path(temp) / "path.csv"
                trajectory.write_text(
                    "x_m,y_m,yaw_rad,duration_s\n0,0,0,1\n0.4,0,0,1\n"
                )
                # Stop recording before changing the session's selected trajectory.
                w.stop_recording()
                w.load_trajectory(trajectory)
                w.start_trajectory()
                self.wait(lambda: w.snapshot["mode"] == Mode.TRAJECTORY)
                self.wait(lambda: w.snapshot["odometry"].pose.x_m > 0.1)
                exe = os.environ.get("GST_LAUNCH") or shutil.which("gst-launch-1.0")
                if exe:
                    w.gst_path.setText(exe)
                    w.video_width.setValue(640)
                    w.video_height.setValue(240)
                    w.start_video()
                    self.wait(lambda: w.video_image.original is not None, 12)
                if os.environ.get("UI_SCREENSHOT"):
                    w.grab().save(os.environ["UI_SCREENSHOT"])
                w.stop_button.click()
                self.wait(
                    lambda: (
                        w.snapshot["mode"] == Mode.IDLE and not w.snapshot["pending"]
                    )
                )
                w.manual_button.click()
                self.wait(lambda: w.snapshot["mode"] == Mode.MANUAL)
                w.disable_button.click()
                self.wait(lambda: not w.snapshot["armed"])
                w.calibration.setText(str(Path(temp) / "missing.yaml"))
                w.start_camera()
                self.wait(lambda: w.camera is not None and not w.camera.is_alive())
                self.assertTrue(w.worker.is_alive())
                worker = w.worker
                simulator = w.simulator
                input_worker = w.input_worker
                video = w.video
            finally:
                w.close()
                self.app.processEvents()
        self.assertFalse(worker.is_alive())
        self.assertFalse(simulator.is_alive())
        self.assertFalse(input_worker.is_alive())
        if video:
            self.assertFalse(video.is_alive())


if __name__ == "__main__":
    unittest.main()
