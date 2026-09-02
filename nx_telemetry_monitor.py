"""FineSUB NX UDP telemetry monitor.

The wire contract mirrors the packed V5_SUB::UploadData structure:
<9f4B (40 bytes, little-endian).
"""

from __future__ import annotations

import csv
import queue
import socket
import struct
import threading
import time
import tkinter as tk
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Callable

from apriltag_localization import (
    AprilTagLocalizer,
    VisionInterpolator,
    VisionFrame,
    VisionSnapshot,
    load_camera_calibration,
    load_tag_layout,
)
from trajectory_tracking import (
    PoseEstimate,
    TrackingSettings,
    TrajectoryError,
    TrajectoryFollower,
    TrackingCommand,
    Waypoint,
    normalize_angle,
    load_waypoints,
)


PACKET_STRUCT = struct.Struct("<9f4B")
PACKET_SIZE = PACKET_STRUCT.size
PACKET_TAIL = (0x00, 0x00, 0x80, 0x7F)
CHART_HISTORY_SECONDS = 10.0
MAX_DRAW_SAMPLES = 400
CONTROL_HEADER = 0xAA
CONTROL_PAYLOAD_STRUCT = struct.Struct("<5f3B2b6f")
CONTROL_FRAME_SIZE = 1 + CONTROL_PAYLOAD_STRUCT.size + 2
CONTROL_PERIOD_MS = 20
CSV_INTERPOLATION_FLUSH_SECONDS = 0.55
PERCEPTION_ROOT = Path(r"D:\fins\tools\finsrov_perception")
CAMERA_CALIBRATION_PATH = PERCEPTION_ROOT / "calibration" / "rgb_camera.yaml"
TAG_LAYOUT_PATH = Path(__file__).with_name("apriltag_layout.json")
TRAJECTORY_ROOT = Path(__file__).with_name("training_trajectories")
DATASET_ROOT = Path(__file__).with_name("dataset")
TRAJECTORY_VERSIONS = ("v2", "v1")


def trajectory_directory_for(version: str) -> Path:
    if version not in TRAJECTORY_VERSIONS:
        raise ValueError(f"不支持的轨迹版本：{version}")
    return TRAJECTORY_ROOT / version


def dataset_directory_for(version: str) -> Path:
    if version not in TRAJECTORY_VERSIONS:
        raise ValueError(f"不支持的数据集版本：{version}")
    return DATASET_ROOT / version


class PacketError(ValueError):
    """Raised when a datagram cannot be interpreted as UploadData."""


def crc16_modbus(data: bytes) -> int:
    """CRC-16/MODBUS, matching etl::crc16_modbus in the A-board firmware."""
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def build_control_frame(
    yaw_rate: float = 0.0,
    forward_speed: float = 0.0,
    left_right_speed: float = 0.0,
    start_button: int = 0,
    enable_path_tracking: int = 0,
    global_x: float = 0.0,
    global_y: float = 0.0,
    global_yaw: float = 0.0,
    global_x_velocity: float = 0.0,
    global_y_velocity: float = 0.0,
    global_yaw_rate: float = 0.0,
) -> bytes:
    """Build the exact 52-byte Streamer frame accepted by the A-board."""
    payload = CONTROL_PAYLOAD_STRUCT.pack(
        yaw_rate,
        forward_speed,
        left_right_speed,
        0.0,  # downSpeed
        0.0,  # upSpeed
        start_button,
        0,  # fillLight
        enable_path_tracking,
        0,  # camYaw
        0,  # camPitch
        global_x,
        global_y,
        global_yaw,
        global_x_velocity,
        global_y_velocity,
        global_yaw_rate,
    )
    frame_without_crc = bytes((CONTROL_HEADER,)) + payload
    return frame_without_crc + struct.pack("<H", crc16_modbus(frame_without_crc))


def is_valid_control_frame(frame: bytes) -> bool:
    if len(frame) != CONTROL_FRAME_SIZE or frame[0] != CONTROL_HEADER:
        return False
    return struct.unpack_from("<H", frame, CONTROL_FRAME_SIZE - 2)[0] == crc16_modbus(frame[:-2])


@dataclass(frozen=True)
class TelemetryPacket:
    odom_global_x_velocity: float
    odom_global_y_velocity: float
    odom_global_yaw_rate: float
    odom_global_x: float
    odom_global_y: float
    odom_global_yaw: float
    network_body_x_velocity: float
    network_body_y_velocity: float
    network_body_yaw_rate: float
    source_ip: str
    source_port: int
    received_monotonic: float
    received_utc: str


@dataclass(frozen=True)
class PacketEvent:
    packet: TelemetryPacket


def decode_packet(data: bytes, source: tuple[str, int]) -> TelemetryPacket:
    """Decode and validate one raw 40-byte UploadData UDP payload."""
    if len(data) != PACKET_SIZE:
        raise PacketError(f"packet length is {len(data)}, expected {PACKET_SIZE}")

    unpacked = PACKET_STRUCT.unpack(data)
    if unpacked[-4:] != PACKET_TAIL:
        raise PacketError("packet tail is invalid")

    return TelemetryPacket(
        odom_global_x_velocity=unpacked[0],
        odom_global_y_velocity=unpacked[1],
        odom_global_yaw_rate=unpacked[2],
        odom_global_x=unpacked[3],
        odom_global_y=unpacked[4],
        odom_global_yaw=unpacked[5],
        network_body_x_velocity=unpacked[6],
        network_body_y_velocity=unpacked[7],
        network_body_yaw_rate=unpacked[8],
        source_ip=source[0],
        source_port=source[1],
        received_monotonic=time.monotonic(),
        received_utc=datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
    )


class ReceiverStats:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.accepted = 0
        self.invalid = 0
        self.source_filtered = 0
        self.ui_queue_dropped = 0
        self.last_invalid_detail = ""

    def add(self, name: str, amount: int = 1) -> None:
        with self._lock:
            setattr(self, name, getattr(self, name) + amount)

    def record_invalid(self, error: PacketError, data: bytes) -> None:
        preview = data[:64].hex(" ").upper()
        with self._lock:
            self.invalid += 1
            self.last_invalid_detail = f"{error}；实际尾部: {data[-8:].hex(' ').upper()}；前 64 字节: {preview}"

    def snapshot(self) -> tuple[int, int, int, int, str]:
        with self._lock:
            return self.accepted, self.invalid, self.source_filtered, self.ui_queue_dropped, self.last_invalid_detail


class UDPReceiver(threading.Thread):
    """Receives, validates and timestamps UDP packets on a background thread."""

    def __init__(
        self,
        port: int,
        expected_source_ip: str,
        events: queue.Queue[PacketEvent],
        on_event: Callable[[PacketEvent], None],
        stats: ReceiverStats,
    ) -> None:
        super().__init__(name="udp-receiver", daemon=True)
        self._port = port
        self._expected_source_ip = expected_source_ip.strip()
        self._events = events
        self._on_event = on_event
        self._stats = stats
        self._stop_event = threading.Event()
        self.bind_error: OSError | None = None

    def stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp_socket:
                udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                udp_socket.bind(("0.0.0.0", self._port))
                udp_socket.settimeout(0.2)

                while not self._stop_event.is_set():
                    try:
                        data, source = udp_socket.recvfrom(65535)
                    except TimeoutError:
                        continue
                    except OSError:
                        if not self._stop_event.is_set():
                            self.bind_error = OSError("UDP receive failed")
                        return

                    if self._expected_source_ip and source[0] != self._expected_source_ip:
                        self._stats.add("source_filtered")
                        continue

                    try:
                        packet = decode_packet(data, source)
                    except PacketError as error:
                        self._stats.record_invalid(error, data)
                        continue

                    event = PacketEvent(packet)
                    self._stats.add("accepted")
                    self._on_event(event)
                    try:
                        self._events.put_nowait(event)
                    except queue.Full:
                        self._stats.add("ui_queue_dropped")
        except OSError as error:
            self.bind_error = error


class ControlTransmitter:
    """Small UDP sender used by the GUI control loop."""

    def __init__(self, destination_ip: str = "192.168.0.2", destination_port: int = 54322) -> None:
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.destination = (destination_ip, destination_port)

    def configure(self, destination_ip: str, destination_port: int) -> None:
        self.destination = (destination_ip, destination_port)

    def send(self, frame: bytes) -> None:
        self.socket.sendto(frame, self.destination)

    def close(self) -> None:
        self.socket.close()


class CsvRecorder:
    """Writes valid packets asynchronously so disk I/O cannot block UDP reception."""

    FIELDNAMES = [
        "received_utc",
        "odom_global_x_velocity_m_s",
        "odom_global_y_velocity_m_s",
        "odom_global_yaw_rate_rad_s",
        "odom_global_x_m",
        "odom_global_y_m",
        "odom_global_yaw_rad",
        "network_body_x_velocity_m_s",
        "network_body_y_velocity_m_s",
        "network_body_yaw_rate_rad_s",
        "tag_timestamp_utc",
        "tag_x_m",
        "tag_y_m",
        "tag_yaw_rad",
    ]

    def __init__(self, interpolator: VisionInterpolator) -> None:
        self._lock = threading.Lock()
        self._queue: queue.Queue[PacketEvent] | None = None
        self._thread: threading.Thread | None = None
        self._stop_event: threading.Event | None = None
        self._interpolator = interpolator
        self.active = False
        self.records_written = 0
        self.queue_dropped = 0
        self.interpolation_dropped = 0

    def start(self, path: Path) -> None:
        with self._lock:
            if self.active:
                raise RuntimeError("recording is already active")
            self._queue = queue.Queue(maxsize=10000)
            self._stop_event = threading.Event()
            self.records_written = 0
            self.queue_dropped = 0
            self.interpolation_dropped = 0
            self.active = True
            self._thread = threading.Thread(
                target=self._write_loop,
                args=(path, self._queue, self._stop_event),
                name="csv-recorder",
                daemon=True,
            )
            self._thread.start()

    def record(self, event: PacketEvent) -> None:
        with self._lock:
            recorder_queue = self._queue if self.active else None
        if recorder_queue is None:
            return
        try:
            recorder_queue.put_nowait(event)
        except queue.Full:
            with self._lock:
                self.queue_dropped += 1

    def stop(self) -> None:
        with self._lock:
            if not self.active:
                return
            self.active = False
            stop_event = self._stop_event
            writer = self._thread
        if stop_event is not None:
            stop_event.set()
        if writer is not None:
            writer.join(timeout=10.0)

    def _write_loop(
        self,
        path: Path,
        events: queue.Queue[PacketEvent],
        stop_event: threading.Event,
    ) -> None:
        try:
            with path.open("w", newline="", encoding="utf-8-sig") as csv_file:
                writer = csv.DictWriter(csv_file, fieldnames=self.FIELDNAMES)
                writer.writeheader()
                pending: deque[PacketEvent] = deque()
                stop_deadline: float | None = None
                while True:
                    if stop_event.is_set() and stop_deadline is None:
                        # 停止采集后给相机一小段时间提供最后的插值右边界。
                        stop_deadline = time.monotonic() + CSV_INTERPOLATION_FLUSH_SECONDS
                    try:
                        pending.append(events.get(timeout=0.05))
                    except queue.Empty:
                        pass
                    while pending:
                        event = pending[0]
                        packet = event.packet
                        vision = self._interpolator.interpolate(packet.received_monotonic)
                        if vision is None:
                            oldest, newest = self._interpolator.bounds()
                            if oldest is not None and packet.received_monotonic < oldest:
                                pending.popleft()
                                with self._lock:
                                    self.interpolation_dropped += 1
                                continue
                            if newest is not None and packet.received_monotonic <= newest:
                                pending.popleft()
                                with self._lock:
                                    self.interpolation_dropped += 1
                                continue
                            break
                        pending.popleft()
                        writer.writerow(
                            {
                                "received_utc": packet.received_utc,
                                "odom_global_x_velocity_m_s": packet.odom_global_x_velocity,
                                "odom_global_y_velocity_m_s": packet.odom_global_y_velocity,
                                "odom_global_yaw_rate_rad_s": packet.odom_global_yaw_rate,
                                "odom_global_x_m": packet.odom_global_x,
                                "odom_global_y_m": packet.odom_global_y,
                                "odom_global_yaw_rad": packet.odom_global_yaw,
                                "network_body_x_velocity_m_s": packet.network_body_x_velocity,
                                "network_body_y_velocity_m_s": packet.network_body_y_velocity,
                                "network_body_yaw_rate_rad_s": packet.network_body_yaw_rate,
                                # 插值定位与此 NX 遥测同一主机时刻对齐。
                                "tag_timestamp_utc": packet.received_utc,
                                "tag_x_m": vision.x_m,
                                "tag_y_m": vision.y_m,
                                "tag_yaw_rad": vision.yaw_rad,
                            }
                        )
                        with self._lock:
                            self.records_written += 1
                    if stop_event.is_set() and events.empty():
                        if not pending:
                            break
                        if stop_deadline is not None and time.monotonic() >= stop_deadline:
                            break
                with self._lock:
                    self.interpolation_dropped += len(pending)
        finally:
            with self._lock:
                self.active = False


class CameraLocalisationWorker(threading.Thread):
    """Owns the OpenCV camera handle and publishes only the latest annotated frame."""

    def __init__(self, camera_index: int, output: queue.Queue[VisionFrame], interpolator: VisionInterpolator) -> None:
        super().__init__(name="apriltag-camera", daemon=True)
        self._camera_index = camera_index
        self._output = output
        self._stop_event = threading.Event()
        self._snapshot_lock = threading.Lock()
        self._latest_snapshot = VisionSnapshot.unavailable("相机尚未启动")
        self._interpolator = interpolator

    def stop(self) -> None:
        self._stop_event.set()

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
        try:
            import cv2

            calibration = load_camera_calibration(CAMERA_CALIBRATION_PATH)
            layout = load_tag_layout(TAG_LAYOUT_PATH)
            localizer = AprilTagLocalizer(calibration, layout)
            backend = cv2.CAP_DSHOW if hasattr(cv2, "CAP_DSHOW") else cv2.CAP_ANY
            capture = cv2.VideoCapture(self._camera_index, backend)
            if not capture.isOpened() and backend != cv2.CAP_ANY:
                capture.release()
                capture = cv2.VideoCapture(self._camera_index, cv2.CAP_ANY)
            if not capture.isOpened():
                self._publish(VisionFrame(VisionSnapshot.unavailable(f"无法打开相机索引 {self._camera_index}"), None))
                return
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            capture.set(cv2.CAP_PROP_FPS, 30)

            while not self._stop_event.is_set():
                ok, image_bgr = capture.read()
                if not ok:
                    self._publish(VisionFrame(VisionSnapshot.unavailable("相机读帧失败"), None))
                    time.sleep(0.05)
                    continue
                frame = localizer.detect(image_bgr)
                accepted, reason = self._interpolator.push(frame.snapshot)
                if not accepted and frame.snapshot.valid:
                    frame = VisionFrame(
                        VisionSnapshot(
                            False, None, None, None, frame.snapshot.tag_ids, frame.snapshot.reprojection_error_px,
                            frame.snapshot.captured_monotonic, frame.snapshot.captured_utc, f"定位帧已丢弃：{reason}",
                        ),
                        frame.image_bgr,
                    )
                self._publish(frame)
        except Exception as error:
            self._publish(VisionFrame(VisionSnapshot.unavailable(f"视觉定位不可用：{error}"), None))
        finally:
            if capture is not None:
                capture.release()


def scan_camera_indices(result_queue: queue.Queue[tuple[int, ...]], max_index: int = 8) -> None:
    """Probe cameras in a background thread so a missing device cannot freeze Tk."""
    try:
        import cv2

        found: list[int] = []
        backend = cv2.CAP_DSHOW if hasattr(cv2, "CAP_DSHOW") else cv2.CAP_ANY
        for index in range(max_index):
            capture = cv2.VideoCapture(index, backend)
            if not capture.isOpened() and backend != cv2.CAP_ANY:
                capture.release()
                capture = cv2.VideoCapture(index, cv2.CAP_ANY)
            try:
                if capture.isOpened():
                    ok, _image = capture.read()
                    if ok:
                        found.append(index)
            finally:
                capture.release()
        try:
            result_queue.put_nowait(tuple(found))
        except queue.Full:
            pass
    except Exception:
        try:
            result_queue.put_nowait(())
        except queue.Full:
            pass


class TelemetryMonitorApp:
    COLORS = ("#00a8ff", "#fbc531", "#4cd137", "#9c88ff", "#00cec9", "#e17055")
    VELOCITY_LABELS = ("Odom Vx", "Odom Vy", "Odom Wz", "Net Vx", "Net Vy", "Net Wz")

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("NX UDP Telemetry Monitor + AprilTag 定位")
        self.root.geometry("1420x1050")
        self.root.minsize(1000, 780)

        self.events: queue.Queue[PacketEvent] = queue.Queue(maxsize=5000)
        self.stats = ReceiverStats()
        self.receiver: UDPReceiver | None = None
        self.vision_interpolator = VisionInterpolator()
        self.recorder = CsvRecorder(self.vision_interpolator)
        self.control_transmitter = ControlTransmitter()
        self.trajectory_follower = TrajectoryFollower()
        self.loaded_waypoints: tuple[Waypoint, ...] | None = None
        self.loaded_trajectory_path: Path | None = None
        self.last_tracking_command: TrackingCommand | None = None
        self.vision_events: queue.Queue[VisionFrame] = queue.Queue(maxsize=2)
        self.camera_scan_events: queue.Queue[tuple[int, ...]] = queue.Queue(maxsize=1)
        self.vision_worker: CameraLocalisationWorker | None = None
        self.vision_snapshot = VisionSnapshot.unavailable("相机未启动")
        self.preview_image: object | None = None
        self.held_keys: set[str] = set()
        self.start_pulse_ticks = 0
        self.packet_times: deque[float] = deque()
        self.chart_samples: deque[tuple[float, tuple[float, ...]]] = deque(maxlen=2400)

        self.source_ip_var = tk.StringVar(value="192.168.0.2")
        self.port_var = tk.StringVar(value="54321")
        self.control_ip_var = tk.StringVar(value="192.168.0.2")
        self.control_port_var = tk.StringVar(value="54322")
        self.translation_scale_var = tk.DoubleVar(value=1.0)
        self.yaw_scale_var = tk.DoubleVar(value=1.0)
        self.trajectory_position_kp_var = tk.StringVar(value="0.70")
        self.trajectory_velocity_kp_var = tk.StringVar(value="1.00")
        self.trajectory_yaw_kp_var = tk.StringVar(value="0.50")
        self.trajectory_slew_var = tk.StringVar(value="1.00")
        self.trajectory_version_var = tk.StringVar(value="v2")
        self.y_axis_min_var = tk.StringVar(value="-1")
        self.y_axis_max_var = tk.StringVar(value="1")
        self.chart_y_min = -1.0
        self.chart_y_max = 1.0
        self.camera_index_var = tk.StringVar(value="4")
        self.listener_status_var = tk.StringVar(value="未监听")
        self.rate_var = tk.StringVar(value="0.0 Hz")
        self.timestamp_var = tk.StringVar(value="等待有效数据")
        self.packet_var = tk.StringVar(value="有效 0 / 无效 0 / 过滤 0 / UI 队列丢弃 0")
        self.invalid_detail_var = tk.StringVar(value="最近无效包：--")
        self.latest_var = tk.StringVar(value="里程计全局位置: X=--  Y=--  yaw=--")
        self.recording_var = tk.StringVar(value="未采集")
        self.control_status_var = tk.StringVar(value="控制目标：192.168.0.2:54322；键盘未按下")
        self.trajectory_status_var = tk.StringVar(value="轨迹：未加载；手动控制")
        self.vision_status_var = tk.StringVar(value="视觉定位：相机未启动")
        self.vision_pose_var = tk.StringVar(value="全局（相机坐标）X: --   Y: --   yaw: --")

        self._create_widgets()
        self.root.bind_all("<KeyPress>", self._on_key_press)
        self.root.bind_all("<KeyRelease>", self._on_key_release)
        self.root.bind_all("<FocusOut>", self._on_focus_out)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(40, self._refresh)
        self.root.after(CONTROL_PERIOD_MS, self._control_tick)

    def _create_widgets(self) -> None:
        root_frame = ttk.Frame(self.root, padding=10)
        root_frame.pack(fill=tk.BOTH, expand=True)
        root_frame.columnconfigure(0, weight=0, minsize=560)
        root_frame.columnconfigure(1, weight=1, minsize=640)
        root_frame.rowconfigure(0, weight=1, minsize=360)
        root_frame.rowconfigure(1, weight=1, minsize=300)

        configuration_panel = ttk.Frame(root_frame)
        configuration_panel.grid(row=0, column=0, rowspan=2, sticky=tk.NSEW, padx=(0, 8))

        config = ttk.LabelFrame(configuration_panel, text="UDP 监听配置", padding=8)
        config.pack(fill=tk.X)
        ttk.Label(config, text="NX 发送方 IP（留空不限）").grid(row=0, column=0, sticky=tk.W)
        ttk.Entry(config, textvariable=self.source_ip_var, width=20).grid(row=0, column=1, padx=(6, 18))
        ttk.Label(config, text="本机监听端口").grid(row=0, column=2, sticky=tk.W)
        ttk.Entry(config, textvariable=self.port_var, width=10).grid(row=0, column=3, padx=6)
        ttk.Button(config, text="应用监听配置", command=self.apply_listener).grid(row=0, column=4, padx=(12, 4))
        ttk.Button(config, text="停止监听", command=self.stop_listener).grid(row=0, column=5)
        ttk.Label(config, textvariable=self.listener_status_var).grid(row=1, column=0, columnspan=6, pady=(7, 0), sticky=tk.W)

        status = ttk.LabelFrame(configuration_panel, text="实时状态", padding=8)
        status.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(status, text="接收频率:").grid(row=0, column=0, sticky=tk.W)
        ttk.Label(status, textvariable=self.rate_var, font=("Segoe UI", 11, "bold")).grid(row=0, column=1, padx=(4, 20), sticky=tk.W)
        ttk.Label(status, textvariable=self.timestamp_var).grid(row=0, column=2, sticky=tk.W)
        ttk.Label(status, textvariable=self.packet_var).grid(row=1, column=0, columnspan=4, pady=(5, 0), sticky=tk.W)
        ttk.Label(status, textvariable=self.invalid_detail_var, foreground="#b33939", wraplength=530).grid(
            row=2, column=0, columnspan=4, pady=(5, 0), sticky=tk.W
        )
        ttk.Label(status, textvariable=self.latest_var).grid(row=3, column=0, columnspan=4, pady=(5, 0), sticky=tk.W)

        control = ttk.LabelFrame(configuration_panel, text="键盘运动控制（发送至 NX）", padding=8)
        control.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(control, text="NX IP").grid(row=0, column=0, sticky=tk.W)
        ttk.Entry(control, textvariable=self.control_ip_var, width=16).grid(row=0, column=1, padx=(6, 14))
        ttk.Label(control, text="控制端口").grid(row=0, column=2, sticky=tk.W)
        ttk.Entry(control, textvariable=self.control_port_var, width=8).grid(row=0, column=3, padx=6)
        ttk.Button(control, text="应用控制目标", command=self.apply_control_destination).grid(row=0, column=4, padx=(8, 18))
        ttk.Button(control, text="电机启停（单次脉冲）", command=self.request_start_button).grid(row=0, column=5)
        ttk.Label(control, text="平移幅值").grid(row=1, column=0, pady=(7, 0), sticky=tk.W)
        ttk.Scale(control, from_=0.0, to=1.0, variable=self.translation_scale_var, orient=tk.HORIZONTAL, length=160).grid(
            row=1, column=1, columnspan=2, pady=(7, 0), sticky=tk.W
        )
        ttk.Label(control, text="偏航幅值").grid(row=1, column=3, pady=(7, 0), sticky=tk.W)
        ttk.Scale(control, from_=0.0, to=1.0, variable=self.yaw_scale_var, orient=tk.HORIZONTAL, length=160).grid(
            row=1, column=4, pady=(7, 0), sticky=tk.W
        )
        ttk.Label(control, text="↑↓ 前后；←→ 左右；A/D 偏航；失焦或松键自动归零").grid(
            row=2, column=0, columnspan=6, pady=(7, 0), sticky=tk.W
        )
        ttk.Label(control, textvariable=self.control_status_var).grid(row=3, column=0, columnspan=6, pady=(5, 0), sticky=tk.W)

        trajectory = ttk.LabelFrame(configuration_panel, text="CSV 轨迹规划（AprilTag 位姿）", padding=8)
        trajectory.pack(fill=tk.X, pady=(8, 0))
        ttk.Button(trajectory, text="选择轨迹 CSV", command=self.choose_trajectory).grid(row=0, column=0, sticky=tk.W)
        ttk.Button(trajectory, text="启动轨迹", command=self.start_trajectory).grid(row=0, column=1, padx=(8, 0))
        ttk.Button(trajectory, text="停止轨迹", command=self.stop_trajectory).grid(row=0, column=2, padx=(6, 0))
        ttk.Label(trajectory, text="轨迹集").grid(row=0, column=3, padx=(12, 0), sticky=tk.E)
        ttk.Combobox(trajectory, textvariable=self.trajectory_version_var, values=TRAJECTORY_VERSIONS, state="readonly", width=5).grid(
            row=0, column=4, padx=(4, 0), sticky=tk.W
        )
        ttk.Label(trajectory, text="位置 Kp").grid(row=1, column=0, pady=(7, 0), sticky=tk.W)
        ttk.Entry(trajectory, textvariable=self.trajectory_position_kp_var, width=7).grid(row=1, column=1, pady=(7, 0), sticky=tk.W)
        ttk.Label(trajectory, text="速度→油门 Kp").grid(row=1, column=2, pady=(7, 0), padx=(8, 0), sticky=tk.W)
        ttk.Entry(trajectory, textvariable=self.trajectory_velocity_kp_var, width=7).grid(row=1, column=3, pady=(7, 0), sticky=tk.W)
        ttk.Label(trajectory, text="yaw Kp").grid(row=2, column=0, pady=(4, 0), sticky=tk.W)
        ttk.Entry(trajectory, textvariable=self.trajectory_yaw_kp_var, width=7).grid(row=2, column=1, pady=(4, 0), sticky=tk.W)
        ttk.Label(trajectory, text="油门斜率 /s").grid(row=2, column=2, pady=(4, 0), padx=(8, 0), sticky=tk.W)
        ttk.Entry(trajectory, textvariable=self.trajectory_slew_var, width=7).grid(row=2, column=3, pady=(4, 0), sticky=tk.W)
        ttk.Label(trajectory, text="CSV: x_m,y_m,yaw_rad,duration_s；每行 duration_s 是从上一点到该点的设定时间。", wraplength=520).grid(
            row=3, column=0, columnspan=4, pady=(6, 0), sticky=tk.W
        )
        ttk.Label(trajectory, textvariable=self.trajectory_status_var, wraplength=520).grid(
            row=4, column=0, columnspan=4, pady=(4, 0), sticky=tk.W
        )

        vision = ttk.LabelFrame(configuration_panel, text="AprilTag 25H9 相机配置", padding=8)
        vision.pack(fill=tk.X, pady=(8, 0))
        vision_controls = ttk.Frame(vision)
        vision_controls.pack(fill=tk.X)
        ttk.Label(vision_controls, text="相机索引").pack(side=tk.LEFT)
        ttk.Entry(vision_controls, textvariable=self.camera_index_var, width=8).pack(side=tk.LEFT, padx=(6, 8))
        ttk.Button(vision_controls, text="扫描相机", command=self.scan_cameras).pack(side=tk.LEFT)
        ttk.Button(vision_controls, text="启动定位", command=self.start_camera).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(vision_controls, text="停止定位", command=self.stop_camera).pack(side=tk.LEFT, padx=(6, 14))
        ttk.Label(vision, text="固定标定：RGB 1280×720；ID 10/11/12/13；黑边 93 mm；中心距 140 mm").pack(
            anchor=tk.W, pady=(5, 0)
        )

        recording = ttk.LabelFrame(configuration_panel, text="CSV 数据采集", padding=8)
        recording.pack(fill=tk.X, pady=(8, 0))
        ttk.Button(recording, text="开始采集", command=self.start_recording).pack(side=tk.LEFT)
        ttk.Button(recording, text="停止采集", command=self.stop_recording).pack(side=tk.LEFT, padx=(6, 14))
        ttk.Label(recording, textvariable=self.recording_var).pack(side=tk.LEFT)

        preview_frame = ttk.LabelFrame(root_frame, text="AprilTag 定位画面", padding=8)
        preview_frame.grid(row=0, column=1, sticky=tk.NSEW)
        preview_frame.rowconfigure(2, weight=1)
        preview_frame.columnconfigure(0, weight=1)
        ttk.Label(preview_frame, textvariable=self.vision_status_var, wraplength=760).grid(row=0, column=0, sticky=tk.W)
        ttk.Label(preview_frame, textvariable=self.vision_pose_var, font=("Segoe UI", 10, "bold")).grid(row=1, column=0, sticky=tk.W, pady=(4, 6))
        self.camera_preview = tk.Label(
            preview_frame,
            background="#101418",
            foreground="#bdc3c7",
            text="点击左侧“扫描相机”后选择索引，再启动定位",
        )
        self.camera_preview.grid(row=2, column=0, sticky=tk.NSEW)

        chart_frame = ttk.LabelFrame(root_frame, text="推进器转速（最近 10 秒）", padding=6)
        chart_frame.grid(row=1, column=1, sticky=tk.NSEW, pady=(8, 0))
        chart_frame.rowconfigure(1, weight=1)
        chart_frame.columnconfigure(0, weight=1)
        axis_controls = ttk.Frame(chart_frame)
        axis_controls.grid(row=0, column=0, sticky=tk.W, pady=(0, 5))
        ttk.Label(axis_controls, text="固定 Y 轴下界").pack(side=tk.LEFT)
        ttk.Entry(axis_controls, textvariable=self.y_axis_min_var, width=10).pack(side=tk.LEFT, padx=(5, 12))
        ttk.Label(axis_controls, text="固定 Y 轴上界").pack(side=tk.LEFT)
        ttk.Entry(axis_controls, textvariable=self.y_axis_max_var, width=10).pack(side=tk.LEFT, padx=(5, 10))
        ttk.Button(axis_controls, text="应用 Y 轴范围", command=self.apply_y_axis_limits).pack(side=tk.LEFT)
        self.chart = tk.Canvas(chart_frame, background="#101418", highlightthickness=0)
        self.chart.grid(row=1, column=0, sticky=tk.NSEW)
        self.chart.bind("<Configure>", lambda _event: self._draw_chart())

    def apply_listener(self) -> None:
        try:
            port = int(self.port_var.get())
            if not 1 <= port <= 65535:
                raise ValueError
        except ValueError:
            messagebox.showerror("端口错误", "监听端口必须是 1 到 65535 之间的整数。")
            return

        self.stop_listener()
        self.events = queue.Queue(maxsize=5000)
        self.packet_times.clear()
        self.chart_samples.clear()
        self.receiver = UDPReceiver(port, self.source_ip_var.get(), self.events, self._record_telemetry_event, self.stats)
        self.receiver.start()
        self.listener_status_var.set(f"监听中：0.0.0.0:{port}，发送方过滤：{self.source_ip_var.get().strip() or '关闭'}")

    def stop_listener(self) -> None:
        receiver = self.receiver
        self.receiver = None
        if receiver is not None:
            receiver.stop()
            receiver.join(timeout=1.0)
        if self.trajectory_follower.active:
            self.stop_trajectory("遥测监听已停止")
        self._clear_motion()
        self.listener_status_var.set("未监听")

    def _record_telemetry_event(self, event: PacketEvent) -> None:
        self.recorder.record(event)

    def scan_cameras(self) -> None:
        if any(thread.name == "camera-scan" and thread.is_alive() for thread in threading.enumerate()):
            return
        self.vision_status_var.set("正在扫描本机相机索引 0–7…")
        threading.Thread(
            target=scan_camera_indices,
            args=(self.camera_scan_events,),
            name="camera-scan",
            daemon=True,
        ).start()

    def start_camera(self) -> None:
        try:
            camera_index = int(self.camera_index_var.get())
            if not 0 <= camera_index <= 31:
                raise ValueError
        except ValueError:
            messagebox.showerror("相机索引错误", "相机索引必须是 0 到 31 之间的整数。")
            return
        self.stop_camera()
        while True:
            try:
                self.vision_events.get_nowait()
            except queue.Empty:
                break
        self.vision_worker = CameraLocalisationWorker(camera_index, self.vision_events, self.vision_interpolator)
        self.vision_worker.start()
        self.vision_status_var.set(f"正在打开相机索引 {camera_index}…")

    def stop_camera(self) -> None:
        if self.trajectory_follower.active:
            self.stop_trajectory("AprilTag 相机已停止")
        worker = self.vision_worker
        self.vision_worker = None
        if worker is not None:
            worker.stop()
            worker.join(timeout=1.5)
        self.vision_snapshot = VisionSnapshot.unavailable("相机已停止")
        self.vision_status_var.set("视觉定位：相机已停止")
        self.vision_pose_var.set("全局（相机坐标）X: --   Y: --   yaw: --")
        while True:
            try:
                self.vision_events.get_nowait()
            except queue.Empty:
                break
        self.preview_image = None
        if hasattr(self, "camera_preview"):
            self.camera_preview.configure(image="", text="相机已停止")

    def apply_control_destination(self) -> None:
        try:
            port = int(self.control_port_var.get())
            if not 1 <= port <= 65535:
                raise ValueError
            destination_ip = self.control_ip_var.get().strip()
            socket.inet_aton(destination_ip)
        except (OSError, ValueError):
            messagebox.showerror("控制目标错误", "请填写合法的 NX IPv4 地址和 1 到 65535 的端口。")
            return
        self.control_transmitter.configure(destination_ip, port)
        self.control_status_var.set(f"控制目标已应用：{destination_ip}:{port}")

    def request_start_button(self) -> None:
        self.start_pulse_ticks = 1
        self.control_status_var.set("将在下一控制周期发送 startButton 1→0 脉冲")

    def _tracking_settings(self) -> TrackingSettings:
        try:
            settings = TrackingSettings(
                position_kp_s=float(self.trajectory_position_kp_var.get()),
                velocity_to_throttle_kp=float(self.trajectory_velocity_kp_var.get()),
                yaw_kp_s=float(self.trajectory_yaw_kp_var.get()),
                output_slew_per_s=float(self.trajectory_slew_var.get()),
            )
            settings.validate()
            return settings
        except (TrajectoryError, ValueError) as error:
            raise TrajectoryError(f"轨迹控制参数无效：{error}") from error

    def _reliable_pose(self) -> PoseEstimate | None:
        kinematics = self.vision_interpolator.latest_kinematics()
        if kinematics is None:
            return None
        snapshot, x_velocity, y_velocity, yaw_rate = kinematics
        if None in (snapshot.x_m, snapshot.y_m, snapshot.yaw_rad):
            return None
        # 规划闭环全部使用质量门控后的 AprilTag 位姿；下传接口仍与键盘一致。
        return PoseEstimate(snapshot.x_m, snapshot.y_m, normalize_angle(snapshot.yaw_rad), x_velocity, y_velocity, yaw_rate)

    def _selected_trajectory_directory(self) -> Path:
        return trajectory_directory_for(self.trajectory_version_var.get())

    def _recording_dataset_directory(self) -> Path:
        if self.loaded_trajectory_path is not None:
            return dataset_directory_for(self.loaded_trajectory_path.parent.name)
        return dataset_directory_for(self.trajectory_version_var.get())

    def choose_trajectory(self) -> None:
        trajectory_directory = self._selected_trajectory_directory()
        trajectory_directory.mkdir(parents=True, exist_ok=True)
        filename = filedialog.askopenfilename(
            title="选择轨迹 CSV",
            initialdir=trajectory_directory,
            filetypes=(("轨迹 CSV", "*.csv"),),
        )
        if not filename:
            return
        path = Path(filename).resolve()
        try:
            path.relative_to(trajectory_directory.resolve())
        except ValueError:
            error = TrajectoryError(f"轨迹文件必须位于 training_trajectories/{self.trajectory_version_var.get()} 文件夹内")
            self.loaded_waypoints = None
            self.loaded_trajectory_path = None
            self.trajectory_status_var.set(f"轨迹加载失败：{error}")
            messagebox.showerror("轨迹 CSV 错误", str(error))
            return
        try:
            waypoints = load_waypoints(path)
        except TrajectoryError as error:
            self.loaded_waypoints = None
            self.loaded_trajectory_path = None
            self.trajectory_status_var.set(f"轨迹加载失败：{error}")
            messagebox.showerror("轨迹 CSV 错误", str(error))
            return
        self.loaded_waypoints = waypoints
        self.loaded_trajectory_path = path
        duration_s = sum(point.duration_s for point in waypoints)
        self.trajectory_status_var.set(f"已加载：{path.name}；{len(waypoints)} 个关键点；CSV 设定总时长 {duration_s:.1f} s；等待启动")

    def start_trajectory(self) -> None:
        if self.loaded_waypoints is None or self.loaded_trajectory_path is None:
            messagebox.showerror("无法启动轨迹", "请先从当前轨迹集文件夹选择合法 CSV。")
            return
        if self.vision_worker is None:
            messagebox.showerror("无法启动轨迹", "请先启动 AprilTag 定位。")
            return
        pose = self._reliable_pose()
        if pose is None:
            messagebox.showerror("无法启动轨迹", "需要新鲜、通过质量门控的 AprilTag XY 与 yaw 位姿。")
            return
        try:
            settings = self._tracking_settings()
            self.trajectory_follower.start(time.monotonic(), pose, self.loaded_waypoints, settings)
        except TrajectoryError as error:
            self.trajectory_status_var.set(f"轨迹启动失败：{error}")
            messagebox.showerror("轨迹错误", str(error))
            return
        self._clear_motion()
        self.last_tracking_command = None
        self.trajectory_status_var.set(f"轨迹执行中：{self.loaded_trajectory_path.name}；正在返回全局原点")

    def stop_trajectory(self, reason: str = "用户停止", transmit_immediately: bool = True) -> None:
        was_active = self.trajectory_follower.active
        self.trajectory_follower.stop()
        self.last_tracking_command = None
        self._clear_motion()
        self.start_pulse_ticks = 0
        if was_active or reason:
            self.trajectory_status_var.set(f"轨迹已停止：{reason}")
        if transmit_immediately:
            try:
                self.control_transmitter.send(build_control_frame())
            except OSError as error:
                self.control_status_var.set(f"控制 UDP 发送失败：{error}")

    def apply_y_axis_limits(self) -> None:
        try:
            minimum = float(self.y_axis_min_var.get())
            maximum = float(self.y_axis_max_var.get())
            if minimum >= maximum:
                raise ValueError
        except ValueError:
            messagebox.showerror("Y 轴范围错误", "Y 轴下界必须小于上界。")
            return
        self.chart_y_min = minimum
        self.chart_y_max = maximum
        self._draw_chart()

    def _on_key_press(self, event: tk.Event[tk.Misc]) -> str | None:
        key = event.keysym.lower()
        if key in {"up", "down", "left", "right", "a", "d"}:
            self.held_keys.add(key)
            return "break"
        return None

    def _on_key_release(self, event: tk.Event[tk.Misc]) -> str | None:
        key = event.keysym.lower()
        if key in {"up", "down", "left", "right", "a", "d"}:
            self.held_keys.discard(key)
            return "break"
        return None

    def _on_focus_out(self, _event: tk.Event[tk.Misc]) -> None:
        self._clear_motion()

    def _clear_motion(self) -> None:
        self.held_keys.clear()

    def _movement_values(self) -> tuple[float, float, float]:
        translation_scale = float(self.translation_scale_var.get())
        yaw_scale = float(self.yaw_scale_var.get())
        forward = translation_scale * (float("up" in self.held_keys) - float("down" in self.held_keys))
        left_right = translation_scale * (float("left" in self.held_keys) - float("right" in self.held_keys))
        yaw = yaw_scale * (float("a" in self.held_keys) - float("d" in self.held_keys))
        return yaw, forward, left_right

    def _control_tick(self) -> None:
        pose = self._reliable_pose()
        tracking = self.trajectory_follower.active
        enable_path_tracking = 0
        if tracking:
            if pose is None:
                self.stop_trajectory("AprilTag 可靠定位已失效", transmit_immediately=False)
                yaw, forward, left_right = 0.0, 0.0, 0.0
            else:
                try:
                    command = self.trajectory_follower.update(time.monotonic(), pose, self._tracking_settings())
                except TrajectoryError as error:
                    self.stop_trajectory(str(error), transmit_immediately=False)
                    yaw, forward, left_right = 0.0, 0.0, 0.0
                else:
                    self.last_tracking_command = command
                    yaw, forward, left_right = command.yaw_rate, command.forward_throttle, command.left_throttle
                    reference = command.reference
                    phase = "终点保持" if reference.complete else "轨迹执行中"
                    self.trajectory_status_var.set(
                        f"{phase}；参考 X={reference.x_m:+.2f} Y={reference.y_m:+.2f} yaw={reference.yaw_rad:+.2f}；"
                        f"实际 X={pose.x_m:+.2f} Y={pose.y_m:+.2f} yaw={normalize_angle(pose.yaw_rad):+.2f}；"
                        f"油门 F={forward:+.2f} L={left_right:+.2f} Y={yaw:+.2f}"
                    )
        else:
            yaw, forward, left_right = self._movement_values()
        start_button = int(self.start_pulse_ticks > 0)
        pose_fields = {}
        if pose is not None:
            # Only quality-gated, fresh AprilTag data is forwarded to the A board.
            pose_fields = {
                "global_x": pose.x_m,
                "global_y": pose.y_m,
                "global_yaw": pose.yaw_rad,
                "global_x_velocity": pose.x_velocity_m_s,
                "global_y_velocity": pose.y_velocity_m_s,
                "global_yaw_rate": pose.yaw_rate_rad_s,
            }
        frame = build_control_frame(
            yaw_rate=yaw,
            forward_speed=forward,
            left_right_speed=left_right,
            start_button=start_button,
            enable_path_tracking=enable_path_tracking,
            **pose_fields,
        )
        try:
            self.control_transmitter.send(frame)
            if start_button:
                self.start_pulse_ticks -= 1
            self.control_status_var.set(
                f"控制发送中：yaw={yaw:+.2f}，forward={forward:+.2f}，left={left_right:+.2f}，"
                f"目标 {self.control_transmitter.destination[0]}:{self.control_transmitter.destination[1]}"
            )
        except OSError as error:
            self.control_status_var.set(f"控制 UDP 发送失败：{error}")
        self.root.after(CONTROL_PERIOD_MS, self._control_tick)

    def start_recording(self) -> None:
        if self.recorder.active:
            return
        dataset_directory = self._recording_dataset_directory()
        dataset_directory.mkdir(parents=True, exist_ok=True)
        if self.loaded_trajectory_path is not None:
            path = dataset_directory / self.loaded_trajectory_path.name
            if path.exists() and not messagebox.askyesno(
                "覆盖已有采集文件？",
                f"采集文件已存在：\n{path}\n\n是否覆盖？",
            ):
                return
        else:
            filename = filedialog.asksaveasfilename(
                title="保存遥测 CSV",
                defaultextension=".csv",
                filetypes=(("CSV 文件", "*.csv"),),
                initialdir=dataset_directory,
                initialfile=f"nx_telemetry_{datetime.now():%Y%m%d_%H%M%S}.csv",
            )
            if not filename:
                return
            path = Path(filename)
        try:
            self.recorder.start(path)
        except OSError as error:
            messagebox.showerror("无法开始采集", str(error))
            return
        self.recording_var.set(f"采集中：{path}")

    def stop_recording(self) -> None:
        was_active = self.recorder.active
        self.recorder.stop()
        if was_active:
            self.recording_var.set(
                f"已停止：写入 {self.recorder.records_written} 条，定位未插值丢弃 {self.recorder.interpolation_dropped} 条，"
                f"队列丢弃 {self.recorder.queue_dropped} 条"
            )

    def _refresh(self) -> None:
        events_processed = 0
        while events_processed < 1000:
            try:
                event = self.events.get_nowait()
            except queue.Empty:
                break
            self._consume_event(event)
            events_processed += 1

        receiver = self.receiver
        if receiver is not None and receiver.bind_error is not None:
            self.listener_status_var.set(f"监听错误：{receiver.bind_error}")
            self.receiver = None

        now = time.monotonic()
        while self.packet_times and self.packet_times[0] < now - 1.0:
            self.packet_times.popleft()
        accepted, invalid, filtered, ui_dropped, last_invalid_detail = self.stats.snapshot()
        self.rate_var.set(f"{len(self.packet_times):.1f} Hz")
        self.packet_var.set(f"有效 {accepted} / 无效 {invalid} / 过滤 {filtered} / UI 队列丢弃 {ui_dropped}")
        self.invalid_detail_var.set(f"最近无效包：{last_invalid_detail or '--'}")
        if self.recorder.active:
            self.recording_var.set(
                f"采集中：已写入 {self.recorder.records_written} 条，定位未插值丢弃 {self.recorder.interpolation_dropped} 条，"
                f"写入队列丢弃 {self.recorder.queue_dropped} 条"
            )
        self._refresh_vision()
        self._draw_chart()
        self.root.after(40, self._refresh)

    def _refresh_vision(self) -> None:
        last_frame: VisionFrame | None = None
        while True:
            try:
                last_frame = self.vision_events.get_nowait()
            except queue.Empty:
                break
        if last_frame is not None:
            self.vision_snapshot = last_frame.snapshot
            self._update_vision_status(last_frame.snapshot)
            if last_frame.image_bgr is not None:
                self._show_vision_image(last_frame.image_bgr)

        try:
            camera_indices = self.camera_scan_events.get_nowait()
        except queue.Empty:
            camera_indices = None
        if camera_indices is not None:
            if camera_indices:
                self.camera_index_var.set(str(camera_indices[0]))
                self.vision_status_var.set(f"找到可读相机索引：{', '.join(str(index) for index in camera_indices)}；已选 {camera_indices[0]}")
            else:
                self.vision_status_var.set("未找到可读相机。请检查 USB 连接、相机占用和 OpenCV 依赖。")

        if self.vision_snapshot.valid and not self.vision_snapshot.is_fresh():
            self.vision_status_var.set(f"视觉定位已过期：{self.vision_snapshot.age_ms():.0f} ms 未更新")

    def _update_vision_status(self, snapshot: VisionSnapshot) -> None:
        tag_text = "、".join(str(tag_id) for tag_id in snapshot.tag_ids) or "--"
        self.vision_status_var.set(f"视觉定位：{snapshot.message}；Tag: {tag_text}；帧龄 {snapshot.age_ms():.0f} ms")
        if snapshot.valid and snapshot.x_m is not None and snapshot.y_m is not None and snapshot.yaw_rad is not None:
            self.vision_pose_var.set(
                f"全局（相机坐标）X: {snapshot.x_m:+.3f} m   Y: {snapshot.y_m:+.3f} m   "
                f"yaw: {snapshot.yaw_rad:+.3f} rad   重投影误差: {snapshot.reprojection_error_px:.2f} px"
            )
        else:
            self.vision_pose_var.set("全局（相机坐标）X: --   Y: --   yaw: --")

    def _show_vision_image(self, image_bgr: object) -> None:
        try:
            from PIL import Image, ImageTk

            image = Image.fromarray(image_bgr[:, :, ::-1])
            # 限制预览高度，保证右下角推进器转速图始终保留可见空间。
            image.thumbnail((720, 300))
            self.preview_image = ImageTk.PhotoImage(image=image)
            self.camera_preview.configure(image=self.preview_image, text="")
        except Exception as error:
            self.vision_status_var.set(f"定位图像显示失败：{error}")

    def _consume_event(self, event: PacketEvent) -> None:
        packet = event.packet
        self.packet_times.append(packet.received_monotonic)
        self.chart_samples.append((
            packet.received_monotonic,
            (
                packet.odom_global_x_velocity,
                packet.odom_global_y_velocity,
                packet.odom_global_yaw_rate,
                packet.network_body_x_velocity,
                packet.network_body_y_velocity,
                packet.network_body_yaw_rate,
            ),
        ))
        self.timestamp_var.set(
            f"上位机接收时间：{packet.received_utc}（回传协议不包含固件时间戳）"
        )
        self.latest_var.set(
            f"里程计全局位置: X={packet.odom_global_x:+.3f} m  Y={packet.odom_global_y:+.3f} m  "
            f"yaw={packet.odom_global_yaw:+.3f} rad"
        )

    def _draw_chart(self) -> None:
        canvas = self.chart
        width = canvas.winfo_width()
        height = canvas.winfo_height()
        if width < 80 or height < 80:
            return

        canvas.delete("all")
        margin_left, margin_right, margin_top, margin_bottom = 64, 18, 30, 40
        plot_width = max(1, width - margin_left - margin_right)
        plot_height = max(1, height - margin_top - margin_bottom)
        canvas.create_text(margin_left, 12, anchor=tk.W, fill="#dcdde1", text="里程计/网络预测速度（共享纵轴）")

        if not self.chart_samples:
            canvas.create_text(width / 2, height / 2, fill="#7f8c8d", text="等待有效 UDP 遥测数据")
            return

        latest_time = self.chart_samples[-1][0]
        visible = [sample for sample in self.chart_samples if sample[0] >= latest_time - CHART_HISTORY_SECONDS]
        minimum = self.chart_y_min
        maximum = self.chart_y_max

        for division in range(5):
            y = margin_top + plot_height * division / 4
            value = maximum - (maximum - minimum) * division / 4
            canvas.create_line(margin_left, y, width - margin_right, y, fill="#2f3640")
            canvas.create_text(margin_left - 6, y, anchor=tk.E, fill="#bdc3c7", text=f"{value:.0f}")

        for seconds_back in range(0, int(CHART_HISTORY_SECONDS) + 1, 2):
            x = width - margin_right - plot_width * seconds_back / CHART_HISTORY_SECONDS
            canvas.create_line(x, margin_top, x, height - margin_bottom, fill="#20252b")
            canvas.create_text(x, height - margin_bottom + 16, fill="#bdc3c7", text=f"-{seconds_back}s")

        stride = max(1, (len(visible) + MAX_DRAW_SAMPLES - 1) // MAX_DRAW_SAMPLES)
        sampled = visible[::stride]
        for velocity_index, color in enumerate(self.COLORS):
            points: list[float] = []
            for sample_time, velocities in sampled:
                x = margin_left + (sample_time - (latest_time - CHART_HISTORY_SECONDS)) / CHART_HISTORY_SECONDS * plot_width
                y = margin_top + (maximum - velocities[velocity_index]) / (maximum - minimum) * plot_height
                points.extend((x, y))
            if len(points) >= 4:
                canvas.create_line(*points, fill=color, width=1.5, smooth=False)
            legend_x = margin_left + velocity_index * 90
            canvas.create_rectangle(legend_x, height - 19, legend_x + 10, height - 9, fill=color, outline=color)
            canvas.create_text(legend_x + 14, height - 14, anchor=tk.W, fill="#dcdde1", text=self.VELOCITY_LABELS[velocity_index])

    def close(self) -> None:
        self._clear_motion()
        try:
            self.control_transmitter.send(build_control_frame())
        except OSError:
            pass
        self.control_transmitter.close()
        self.stop_camera()
        self.stop_listener()
        self.stop_recording()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    TelemetryMonitorApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
