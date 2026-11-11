"""Independent local camera and AprilTag comparison worker, extracted from legacy app.

No camera measurement is supplied to the vehicle controller.
"""

import queue
import threading
import time
from pathlib import Path
from apriltag_localization import (
    AprilTagLocalizer,
    VisionInterpolator,
    VisionFrame,
    VisionSnapshot,
    load_camera_calibration,
    load_tag_layout,
)

CAMERA_CAPTURE_WIDTH = 1920
CAMERA_CAPTURE_HEIGHT = 1080
CAMERA_CAPTURE_FPS = 30.0
LOCALISATION_WIDTH = 1280
LOCALISATION_HEIGHT = 720


class CameraLocalisationWorker(threading.Thread):
    """Owns the OpenCV camera handle and publishes only the latest annotated frame."""

    def __init__(
        self,
        camera_index: int,
        output: queue.Queue[VisionFrame],
        interpolator: VisionInterpolator,
        calibration_path: Path,
        layout_path: Path,
    ) -> None:
        super().__init__(name="apriltag-camera", daemon=True)
        self._camera_index = camera_index
        self.calibration_path = calibration_path
        self.layout_path = layout_path
        self._output = output
        self._stop_event = threading.Event()
        self._snapshot_lock = threading.Lock()
        self._latest_snapshot = VisionSnapshot.unavailable("相机尚未启动")
        self._interpolator = interpolator
        self._recording_lock = threading.Lock()
        self._recording_request: Path | None = None
        self._recording_stop_requested = False
        self._active_recording_path: Path | None = None
        self._recording_status = "视频录制：未开始"

    def stop(self) -> None:
        self._stop_event.set()

    def start_recording(self, path: Path) -> None:
        """Request recording from the camera-owning thread on its next frame."""
        with self._recording_lock:
            self._recording_request = path
            self._recording_stop_requested = False
            self._recording_status = "视频录制：等待 1080P 相机帧…"

    def stop_recording(self) -> None:
        with self._recording_lock:
            self._recording_request = None
            self._recording_stop_requested = True
            if self._active_recording_path is None:
                self._recording_status = "视频录制：已停止"
            else:
                self._recording_status = "视频录制：正在停止并保存文件…"

    def recording_status(self) -> str:
        with self._recording_lock:
            return self._recording_status

    def _stop_video_writer(self, writer: object | None, message: str) -> None:
        if writer is not None:
            writer.release()
        with self._recording_lock:
            self._recording_request = None
            self._recording_stop_requested = False
            self._active_recording_path = None
            self._recording_status = message

    def _write_video_frame(
        self, cv2: object, writer: object | None, image_bgr: object
    ) -> object | None:
        """Open/write/release VideoWriter only from this camera thread."""
        with self._recording_lock:
            requested_path = self._recording_request
            stop_requested = self._recording_stop_requested

        if stop_requested:
            previous_name = (
                self._active_recording_path.name
                if self._active_recording_path is not None
                else ""
            )
            suffix = f"：{previous_name}" if previous_name else ""
            self._stop_video_writer(writer, f"视频录制已停止{suffix}")
            return None

        if writer is None and requested_path is not None:
            height, width = image_bgr.shape[:2]
            if (width, height) != (CAMERA_CAPTURE_WIDTH, CAMERA_CAPTURE_HEIGHT):
                with self._recording_lock:
                    self._recording_request = None
                    self._recording_status = (
                        f"视频录制未启动：相机实际输出 {width}×{height}，需要 "
                        f"{CAMERA_CAPTURE_WIDTH}×{CAMERA_CAPTURE_HEIGHT}"
                    )
                return None
            try:
                requested_path.parent.mkdir(parents=True, exist_ok=True)
                candidate = cv2.VideoWriter(
                    str(requested_path),
                    cv2.VideoWriter_fourcc(*"mp4v"),
                    CAMERA_CAPTURE_FPS,
                    (CAMERA_CAPTURE_WIDTH, CAMERA_CAPTURE_HEIGHT),
                )
            except Exception as error:
                with self._recording_lock:
                    self._recording_request = None
                    self._recording_status = f"视频录制未启动：无法创建文件（{error}）"
                return None
            if not candidate.isOpened():
                candidate.release()
                with self._recording_lock:
                    self._recording_request = None
                    self._recording_status = (
                        "视频录制未启动：OpenCV 无法创建 MP4 编码器"
                    )
                return None
            writer = candidate
            with self._recording_lock:
                self._recording_request = None
                self._active_recording_path = requested_path
                self._recording_status = (
                    f"视频录制中：{requested_path.name}（1920×1080 @ 30 FPS）"
                )

        if writer is not None:
            try:
                writer.write(image_bgr)
            except Exception as error:
                self._stop_video_writer(writer, f"视频录制已停止：写入失败（{error}）")
                return None
        return writer

    def latest_snapshot(self) -> VisionSnapshot:
        with self._snapshot_lock:
            return self._latest_snapshot

    def _publish(self, frame: VisionFrame) -> None:
        with self._snapshot_lock:
            self._latest_snapshot = frame.snapshot
        while True:
            try:
                self._output.put_nowait(frame)
                return
            except queue.Full:
                try:
                    self._output.get_nowait()
                except queue.Empty:
                    return

    def run(self) -> None:
        capture = None
        writer = None
        try:
            import cv2

            calibration = load_camera_calibration(self.calibration_path)
            layout = load_tag_layout(self.layout_path)
            localizer = AprilTagLocalizer(calibration, layout)
            backend = cv2.CAP_DSHOW if hasattr(cv2, "CAP_DSHOW") else cv2.CAP_ANY
            capture = cv2.VideoCapture(self._camera_index, backend)
            if not capture.isOpened() and backend != cv2.CAP_ANY:
                capture.release()
                capture = cv2.VideoCapture(self._camera_index, cv2.CAP_ANY)
            if not capture.isOpened():
                self._publish(
                    VisionFrame(
                        VisionSnapshot.unavailable(
                            f"无法打开相机索引 {self._camera_index}"
                        ),
                        None,
                    )
                )
                return
            # Record the original 1080P image.  AprilTag is processed from a
            # 1280x720 derivative so its existing calibration remains valid.
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_CAPTURE_WIDTH)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_CAPTURE_HEIGHT)
            capture.set(cv2.CAP_PROP_FPS, CAMERA_CAPTURE_FPS)

            while not self._stop_event.is_set():
                ok, image_bgr = capture.read()
                if not ok:
                    self._publish(
                        VisionFrame(VisionSnapshot.unavailable("相机读帧失败"), None)
                    )
                    time.sleep(0.05)
                    continue
                writer = self._write_video_frame(cv2, writer, image_bgr)
                localisation_image = cv2.resize(
                    image_bgr,
                    (LOCALISATION_WIDTH, LOCALISATION_HEIGHT),
                    interpolation=cv2.INTER_AREA,
                )
                frame = localizer.detect(localisation_image)
                accepted, reason = self._interpolator.push(frame.snapshot)
                if not accepted and frame.snapshot.valid:
                    frame = VisionFrame(
                        VisionSnapshot(
                            False,
                            None,
                            None,
                            None,
                            frame.snapshot.tag_ids,
                            frame.snapshot.reprojection_error_px,
                            frame.snapshot.captured_monotonic,
                            frame.snapshot.captured_utc,
                            f"定位帧已丢弃：{reason}",
                        ),
                        frame.image_bgr,
                    )
                self._publish(frame)
        except Exception as error:
            self._publish(
                VisionFrame(
                    VisionSnapshot.unavailable(f"视觉定位不可用：{error}"), None
                )
            )
        finally:
            if writer is not None:
                self._stop_video_writer(writer, "视频录制已停止：相机已关闭")
            if capture is not None:
                capture.release()
