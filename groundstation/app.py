"""PySide6 presentation layer. Worker snapshots are the only control state read by Qt."""

import argparse
import os
from collections import deque
from dataclasses import asdict
from datetime import datetime
from pathlib import Path
import queue
import time
import sys

from PySide6 import QtCore as C, QtGui as G, QtWidgets as W
import pyqtgraph as pg

from apriltag_localization import VisionInterpolator
from trajectory_tracking import load_waypoints, TrackingSettings
from .camera import CameraLocalisationWorker
from .models import Action, Axes, Config, InputSnapshot, Mode
from .queues import latest_get, latest_put
from .recording import SessionRecorder
from .runtime import ControlWorker, InputWorker
from .simulator import SimulatorThread
from .video import VideoReceiver

ROOT = Path(__file__).resolve().parents[1]
MODE_LABEL = {Mode.IDLE: "停止", Mode.MANUAL: "手动", Mode.TRAJECTORY: "轨迹追踪"}


def configure_font(app):
    """Load the Windows UI font explicitly for desktop and offscreen renderers."""
    path = Path(os.environ.get("WINDIR", "C:/Windows")) / "Fonts/msyh.ttc"
    if path.is_file():
        font_id = G.QFontDatabase.addApplicationFont(str(path))
        families = G.QFontDatabase.applicationFontFamilies(font_id)
        if families:
            app.setFont(G.QFont(families[0], 10))
            return
    app.setFont(G.QFont("sans-serif", 10))


def spin(value, minimum=0, maximum=100, step=0.05, decimals=2):
    widget = W.QDoubleSpinBox()
    widget.setRange(minimum, maximum)
    widget.setDecimals(decimals)
    widget.setSingleStep(step)
    widget.setValue(value)
    return widget


class ImagePanel(W.QLabel):
    def __init__(self, placeholder):
        super().__init__(placeholder)
        self.setAlignment(C.Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(260, 150)
        self.setStyleSheet("background:#070c12; color:#91a5b7; border-radius:8px;")
        self.original = None

    def show_rgb(self, data, width, height):
        self.original = G.QPixmap.fromImage(
            G.QImage(
                data, width, height, width * 3, G.QImage.Format.Format_RGB888
            ).copy()
        )
        self.redraw()

    def redraw(self):
        if self.original:
            self.setPixmap(
                self.original.scaled(
                    self.size(),
                    C.Qt.AspectRatioMode.KeepAspectRatio,
                    C.Qt.TransformationMode.SmoothTransformation,
                )
            )

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.redraw()


class MainWindow(W.QMainWindow):
    def __init__(self, config=None, simulate=False, video_test=False):
        super().__init__()
        self.config = config or Config()
        self.bind_ip = self.config.bind_ip
        self.worker = self.simulator = self.video = self.camera = self.recorder = None
        self.snapshot = None
        self.ui_error = ""
        self.input_worker = InputWorker()
        self.gamepad_state = InputSnapshot()
        self.device_list = []
        self.keys = set()
        self.points, self.trajectory_path = (), ""
        self.last_seq = None
        self.actual_xy = deque(maxlen=5000)
        self.history = deque(maxlen=1000)
        self.camera_frames = queue.Queue(maxsize=1)
        self.interpolator = VisionInterpolator()
        self.video_test = video_test
        self.setWindowTitle("FinsROV-Heavy · Ground Station")
        self.resize(1460, 940)
        self.build_ui()
        self.input_worker.start()
        W.QApplication.instance().installEventFilter(self)
        self.timer = C.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(20)
        if simulate:
            C.QTimer.singleShot(0, lambda: self.attempt(self.connect_simulation))

    def button(self, text, callback, style=""):
        button = W.QPushButton(text)
        button.setMinimumHeight(32)
        button.clicked.connect(lambda: self.attempt(callback))
        if style:
            button.setStyleSheet(style)
        return button

    def attempt(self, callback):
        self.ui_error = ""
        try:
            callback()
        except Exception as error:
            self.ui_error = str(error)
            self.notice.setText(str(error))

    def build_ui(self):
        pg.setConfigOptions(antialias=True, background="#101c28", foreground="#c7d6e2")
        self.setStyleSheet("""QMainWindow, QWidget {background:#101923; color:#d5e1ea; font-size:13px;}
            QGroupBox {border:1px solid #2d4053; border-radius:6px; margin-top:12px; padding-top:10px;}
            QGroupBox::title {subcontrol-origin:margin; left:10px; color:#7dd3fc;}
            QPushButton {background:#253b4e; border:1px solid #39566b; border-radius:5px; padding:5px 10px;}
            QPushButton:hover {background:#34536b;} QPushButton:disabled {color:#647484;}
            QLineEdit,QComboBox,QSpinBox,QDoubleSpinBox {background:#192a3a; padding:5px; border:1px solid #345064;}
            QTabBar::tab {padding:10px 18px;} QTabBar::tab:selected {background:#253b4e; color:#7dd3fc;}
            QScrollArea {border:0;}""")
        central = W.QWidget()
        self.setCentralWidget(central)
        layout = W.QVBoxLayout(central)
        title = W.QHBoxLayout()
        brand = W.QLabel("FinsROV-Heavy  /  GROUND STATION")
        brand.setStyleSheet("font-size:22px; font-weight:700;")
        title.addWidget(brand)
        title.addStretch()
        self.sim_badge = W.QLabel("未连接")
        self.sim_badge.setStyleSheet("color:#fbbf24; font-size:17px; font-weight:bold;")
        title.addWidget(self.sim_badge)
        layout.addLayout(title)
        bar = W.QHBoxLayout()
        self.status = W.QLabel(
            "连接：离线   |   模式：停止   |   电机：停用   |   NX 定位：无数据"
        )
        bar.addWidget(self.status, 1)
        self.stop_button = self.button(
            "停止运动", self.stop_motion, "background:#a33a33; font-weight:bold;"
        )
        self.manual_button = self.button(
            "手动接管 / 恢复", lambda: self.action(Action.SET_MODE, Mode.MANUAL)
        )
        self.disable_button = self.button(
            "电机停用", lambda: self.action(Action.SET_ARMED, 0), "color:#fca5a5;"
        )
        for button in (self.stop_button, self.manual_button, self.disable_button):
            bar.addWidget(button)
        layout.addLayout(bar)
        self.tabs = W.QTabWidget()
        layout.addWidget(self.tabs, 1)
        operation = W.QWidget()
        op = W.QHBoxLayout(operation)
        visual = W.QVBoxLayout()
        self.video_image = ImagePanel("GStreamer 视频\nH.264 / RTP · UDP 5600 · PT 96")
        self.video_status = W.QLabel("视频未启动")
        visual.addWidget(self.video_image, 1)
        visual.addWidget(self.video_status)
        self.xy = pg.PlotWidget(title="NX 世界坐标 · XY 轨迹")
        self.xy.setLabel("bottom", "X", units="m")
        self.xy.setLabel("left", "Y", units="m")
        self.xy.getAxis("bottom").enableAutoSIPrefix(False)
        self.xy.getAxis("left").enableAutoSIPrefix(False)
        self.xy.setAspectLocked()
        self.xy.showGrid(x=True, y=True, alpha=0.2)
        self.xy.addLegend()
        self.ref_curve = self.xy.plot(
            pen=pg.mkPen("#fbbf24", width=2, style=C.Qt.PenStyle.DashLine),
            name="参考路径",
        )
        self.actual_curve = self.xy.plot(
            pen=pg.mkPen("#38bdf8", width=2), name="NX 实际"
        )
        self.ref_dot = self.xy.plot(
            pen=None, symbol="o", symbolBrush="#fbbf24", symbolSize=8
        )
        visual.addWidget(self.xy, 1)
        op.addLayout(visual, 3)
        side_tabs = W.QTabWidget()
        side_tabs.setMinimumWidth(365)
        for title, builders in [
            ("控制", (self.build_manual, self.build_trajectory)),
            ("连接与视频", (self.build_connection, self.build_video)),
        ]:
            side = W.QWidget()
            form = W.QVBoxLayout(side)
            for builder in builders:
                builder(form)
            form.addStretch()
            scroll = W.QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setWidget(side)
            side_tabs.addTab(scroll, title)
        op.addWidget(side_tabs, 1)
        self.tabs.addTab(operation, "操控与轨迹")
        self.build_telemetry()
        self.build_experiment()
        self.notice = W.QLabel("选择连接与输入设备；确认模式后启用电机。")
        self.notice.setWordWrap(True)
        self.notice.setMinimumHeight(34)
        self.notice.setStyleSheet("color:#fbbf24; padding:4px;")
        layout.addWidget(self.notice)

    def group(self, parent, title):
        box = W.QGroupBox(title)
        form = W.QFormLayout(box)
        parent.addWidget(box)
        return form

    def build_connection(self, parent):
        form = self.group(parent, "连接 · MAVLink 2")
        self.host = W.QLineEdit(self.config.nx_ip)
        self.pc_port = W.QSpinBox()
        self.pc_port.setRange(1, 65535)
        self.pc_port.setValue(self.config.port)
        self.nx_port = W.QSpinBox()
        self.nx_port.setRange(1, 65535)
        self.nx_port.setValue(self.config.nx_port)
        form.addRow("NX 地址", self.host)
        form.addRow("PC 接收端口", self.pc_port)
        form.addRow("NX 接收端口", self.nx_port)
        form.addRow(
            self.button("连接 NX", self.connect_vehicle),
            self.button("断开连接", self.disconnect),
        )
        form.addRow(self.button("启动本机模拟 NX", self.connect_simulation))
        self.reset_button = self.button(
            "重置 NX 里程计", lambda: self.action(Action.RESET_ODOMETRY, 1)
        )
        form.addRow(self.reset_button)

    def build_manual(self, parent):
        form = self.group(parent, "手动输入")
        self.source = W.QComboBox()
        self.source.addItems(["手柄", "键盘备用"])
        self.source.currentIndexChanged.connect(self.input_source_changed)
        self.devices = W.QComboBox()
        self.devices.addItem("未选择手柄", -1)
        self.devices.activated.connect(self.select_gamepad)
        self.deadzone = spin(0, 0, 0.9, 0.01)
        form.addRow("输入来源", self.source)
        form.addRow("设备", self.devices)
        form.addRow("摇杆死区", self.deadzone)
        self.input_preview = W.QLabel("等待输入")
        self.input_preview.setWordWrap(True)
        form.addRow(self.input_preview)
        hint = W.QLabel(
            "键盘：W/S 前后 · A/D 左右 · Q/E 偏航\nR/F 上升/下降；失去窗口焦点立即撤销输入。"
        )
        hint.setWordWrap(True)
        form.addRow(hint)
        form.addRow(
            self.button(
                "进入手动模式", lambda: self.action(Action.SET_MODE, Mode.MANUAL)
            ),
            self.button("启用电机", lambda: self.action(Action.SET_ARMED, 1)),
        )
        form.addRow(
            self.button("相机灯目标开/关", lambda: self.toggle_light(0)),
            self.button("框架灯目标开/关", lambda: self.toggle_light(1)),
        )

    def build_trajectory(self, parent):
        form = self.group(parent, "水平轨迹 · NX 反馈")
        self.library = W.QComboBox()
        self.library.addItems(["v2", "v1"])
        form.addRow("轨迹库", self.library)
        self.choose_button = self.button("选择 / 导入轨迹 CSV", self.choose_trajectory)
        form.addRow(self.choose_button)
        self.trajectory_label = W.QLabel("未加载轨迹")
        self.trajectory_label.setWordWrap(True)
        form.addRow(self.trajectory_label)
        self.capture_depth = W.QCheckBox("捕获启动时深度")
        self.capture_depth.setChecked(True)
        self.depth = spin(0.5, 0, 250, 0.1)
        form.addRow(self.capture_depth)
        form.addRow("目标深度 m", self.depth)
        form.addRow(self.button("启动所选轨迹", self.start_trajectory))
        self.parameters = {}
        for name, label in [
            ("position_kp_s", "位置修正 /s"),
            ("velocity_to_throttle_kp", "速度→油门 Kp"),
            ("yaw_kp_s", "航向修正 /s"),
            ("yaw_full_scale_rad_s", "偏航满量程 rad/s"),
            ("output_slew_per_s", "输出变化率 /s"),
        ]:
            box = spin(getattr(self.config.settings, name), 0.001, 50, 0.05, 3)
            self.parameters[name] = box
            form.addRow(label, box)

    def build_video(self, parent):
        form = self.group(parent, "GStreamer 视频")
        self.gst_path = W.QLineEdit(self.config.gst_executable)
        self.video_port = W.QSpinBox()
        self.video_port.setRange(1, 65535)
        self.video_port.setValue(self.config.video_port)
        self.video_width = W.QSpinBox()
        self.video_width.setRange(4, 3840)
        self.video_width.setSingleStep(4)
        self.video_width.setValue(self.config.video_width)
        self.video_height = W.QSpinBox()
        self.video_height.setRange(4, 2160)
        self.video_height.setValue(self.config.video_height)
        self.video_latency = W.QSpinBox()
        self.video_latency.setRange(0, 2000)
        self.video_latency.setValue(self.config.video_latency_ms)
        form.addRow("gst-launch 路径", self.gst_path)
        form.addRow("UDP 端口", self.video_port)
        form.addRow("显示宽度 px", self.video_width)
        form.addRow("显示高度 px", self.video_height)
        form.addRow("缓存延迟 ms", self.video_latency)
        form.addRow(
            self.button("启动 / 重启视频", self.start_video),
            self.button("关闭视频", self.stop_video),
        )

    def build_telemetry(self):
        page = W.QWidget()
        layout = W.QVBoxLayout(page)
        self.chart_group = W.QComboBox()
        self.chart_group.addItems(
            [
                "世界速度 vx / vy / yaw rate",
                "世界位姿 x / y / yaw",
                "网络预测：机体 vx / vy / yaw rate",
            ]
        )
        layout.addWidget(self.chart_group)
        self.telemetry_plots = []
        self.telemetry_curves = []
        for i in range(3):
            plot = pg.PlotWidget()
            plot.showGrid(x=True, y=True, alpha=0.2)
            plot.setLabel("bottom", "PC 单调时钟", units="s")
            self.telemetry_curves.append(
                plot.plot(pen=pg.mkPen(["#38bdf8", "#fbbf24", "#4ade80"][i], width=2))
            )
            self.telemetry_plots.append(plot)
            layout.addWidget(plot)
        self.diagnostics = W.QLabel("等待遥测")
        self.diagnostics.setWordWrap(True)
        layout.addWidget(self.diagnostics)
        self.tabs.addTab(page, "遥测曲线")

    def build_experiment(self):
        page = W.QWidget()
        layout = W.QHBoxLayout(page)
        left = W.QVBoxLayout()
        self.camera_image = ImagePanel("本地 AprilTag 对照相机")
        left.addWidget(self.camera_image, 1)
        self.camera_status = W.QLabel("相机未启动")
        left.addWidget(self.camera_status)
        layout.addLayout(left, 2)
        right = W.QVBoxLayout()
        layout.addLayout(right, 1)
        form = self.group(right, "独立本地相机 · 对照定位与录像")
        self.camera_index = W.QSpinBox()
        self.camera_index.setRange(0, 20)
        self.calibration = W.QLineEdit(
            str(Path("D:/fins/tools/finsrov_perception/calibration/rgb_camera.yaml"))
        )
        self.tag_layout = W.QLineEdit(str(ROOT / "apriltag_layout.json"))
        form.addRow("设备索引", self.camera_index)
        form.addRow("相机标定 YAML", self.calibration)
        form.addRow("Tag 布局 JSON", self.tag_layout)
        form.addRow(self.button("选择标定文件", self.choose_calibration))
        form.addRow(
            self.button("打开相机", self.start_camera),
            self.button("关闭相机", self.stop_camera),
        )
        form.addRow(
            self.button("开始 MP4 录像", self.start_camera_recording),
            self.button("停止录像", self.stop_camera_recording),
        )
        form = self.group(right, "实验记录")
        self.record_root = W.QLineEdit(str(ROOT / "recordings"))
        form.addRow("保存目录", self.record_root)
        form.addRow(
            self.button("开始 CSV 会话", self.start_recording),
            self.button("停止并保存", self.stop_recording),
        )
        self.record_status = W.QLabel("未记录")
        self.record_status.setWordWrap(True)
        form.addRow(self.record_status)
        desc = W.QLabel(
            "telemetry.csv：保留所有 NX 采样，Tag 缺失留空。\naligned.csv：仅导出具有严格插值 Tag 的样本。\nsession.json：配置、协议版本和轨迹文件。"
        )
        desc.setWordWrap(True)
        form.addRow(desc)
        form = self.group(right, "模拟故障注入（仅本机模拟器）")
        self.invalid_odom = W.QCheckBox("定位无效")
        self.reject_mode = W.QCheckBox("拒绝模式切换")
        self.delay = spin(0, 0, 1000, 10, 0)
        self.drop = spin(0, 0, 1, 0.05)
        self.duplicate = spin(0, 0, 1, 0.05)
        form.addRow(self.invalid_odom)
        form.addRow(self.reject_mode)
        form.addRow("发送延迟 ms", self.delay)
        form.addRow("丢包概率", self.drop)
        form.addRow("重复包概率", self.duplicate)
        form.addRow(self.button("应用模拟故障", self.inject_faults))
        right.addStretch()
        self.tabs.addTab(page, "相机与实验")

    def connect_vehicle(self):
        self.disconnect()
        self.config.bind_ip = self.bind_ip
        self.config.nx_ip = self.host.text().strip()
        self.config.port = self.pc_port.value()
        self.config.nx_port = self.nx_port.value()
        self.worker = ControlWorker(self.config)
        self.worker.start()
        self.notice.setText("等待 NX 能力握手与时间同步")

    def connect_simulation(self):
        self.disconnect()
        self.host.setText("127.0.0.1")
        self.config.nx_ip = "127.0.0.1"
        self.config.port = self.pc_port.value()
        self.config.nx_port = self.nx_port.value()
        self.config.bind_ip = "127.0.0.1"
        self.simulator = SimulatorThread(
            ("127.0.0.1", self.config.nx_port), ("127.0.0.1", self.config.port)
        )
        try:
            self.worker = ControlWorker(self.config)
        except Exception:
            self.simulator.close()
            self.simulator = None
            raise
        self.simulator.start()
        self.worker.start()
        self.notice.setText("模拟 NX 已启动；选择输入来源，再进入手动模式")

    def disconnect(self):
        self.stop_recording()
        if self.worker:
            self.worker.close()
            self.worker = None
        if self.simulator:
            self.simulator.close()
            self.simulator = None
        self.snapshot = None
        self.last_seq = None
        self.keys.clear()
        self.actual_xy.clear()
        self.history.clear()

    def action(self, action, value, context=None):
        if not self.worker or not self.worker.is_alive():
            raise ValueError("先连接 NX 或启动模拟器")
        self.worker.command(action, value, context)

    def stop_motion(self):
        self.keys.clear()
        if self.worker:
            self.worker.stop_motion()
        self.notice.setText("已请求停止运动")

    def toggle_light(self, index):
        if self.snapshot:
            if self.snapshot["pending"]:
                raise ValueError("等待上一操作确认")
            self.action(
                Action.SET_CAMERA_LIGHT if index == 0 else Action.SET_FRAME_LIGHT,
                not self.snapshot["lights"][index],
            )

    def input_source_changed(self):
        self.stop_motion()
        self.gamepad_state = InputSnapshot()

    def select_gamepad(self, index):
        self.stop_motion()
        device = self.devices.itemData(index)
        if device is not None and device >= 0:
            latest_put(self.input_worker.commands, device)

    def choose_trajectory(self):
        path, _ = W.QFileDialog.getOpenFileName(
            self,
            "选择局部轨迹 CSV",
            str(ROOT / "training_trajectories" / self.library.currentText()),
            "CSV (*.csv)",
        )
        if path:
            self.load_trajectory(path)

    def load_trajectory(self, path):
        self.points = load_waypoints(Path(path))
        self.trajectory_path = str(path)
        self.trajectory_label.setText(
            f"{Path(path).name} · {len(self.points)} 点\n启动时固定 NX 当前位姿为原点"
        )
        self.ref_curve.setData(
            [p.x_m for p in self.points], [p.y_m for p in self.points]
        )

    def tracking_settings(self):
        settings = TrackingSettings(
            **{k: w.value() for k, w in self.parameters.items()}
        )
        settings.validate()
        return settings

    def start_trajectory(self):
        if not self.points:
            raise ValueError("请先选择轨迹 CSV")
        self.action(
            Action.SET_MODE,
            Mode.TRAJECTORY,
            dict(
                points=self.points,
                settings=self.tracking_settings(),
                depth=None if self.capture_depth.isChecked() else self.depth.value(),
            ),
        )

    def start_video(self):
        self.stop_video()
        self.video = VideoReceiver(
            executable=self.gst_path.text(),
            port=self.video_port.value(),
            width=self.video_width.value(),
            height=self.video_height.value(),
            latency=self.video_latency.value(),
            test_source=self.video_test,
        )
        self.video.start()

    def stop_video(self):
        if self.video:
            self.video.close()
            self.video = None
        self.video_status.setText("视频已停止")

    def choose_calibration(self):
        path, _ = W.QFileDialog.getOpenFileName(
            self, "相机标定", self.calibration.text(), "YAML (*.yaml *.yml)"
        )
        if path:
            self.calibration.setText(path)

    def start_camera(self):
        self.stop_camera()
        self.interpolator = VisionInterpolator()
        if self.recorder:
            self.recorder.interpolator = self.interpolator
        self.camera = CameraLocalisationWorker(
            self.camera_index.value(),
            self.camera_frames,
            self.interpolator,
            Path(self.calibration.text()),
            Path(self.tag_layout.text()),
        )
        self.camera.start()

    def stop_camera(self):
        if self.camera:
            self.camera.stop()
            self.camera.join(2)
            if self.camera.is_alive():
                raise ValueError("相机驱动尚未退出；控制循环仍可使用，请稍后重试关闭")
            self.camera = None

    def start_camera_recording(self):
        if not self.camera or not self.camera.is_alive():
            raise ValueError("先打开本地相机")
        self.camera.start_recording(
            Path(self.record_root.text())
            / ("camera_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".mp4")
        )

    def stop_camera_recording(self):
        if self.camera:
            self.camera.stop_recording()

    def start_recording(self):
        if self.recorder:
            raise ValueError("已在记录")
        if not self.worker:
            raise ValueError("先连接 NX")
        self.config.settings = self.tracking_settings()
        metadata = dict(
            config=asdict(self.config),
            trajectory_path=self.trajectory_path,
            depth_capture=self.capture_depth.isChecked(),
            depth_target=self.depth.value(),
            input_source=self.source.currentText(),
            deadzone=self.deadzone.value(),
            calibration=self.calibration.text(),
            tag_layout=self.tag_layout.text(),
            simulated=bool(self.snapshot and self.snapshot["simulated"]),
            video=dict(
                executable=self.gst_path.text(),
                port=self.video_port.value(),
                width=self.video_width.value(),
                height=self.video_height.value(),
                latency_ms=self.video_latency.value(),
            ),
        )
        directory = Path(self.record_root.text()) / datetime.now().strftime(
            "%Y%m%d_%H%M%S_%f"
        )
        self.recorder = SessionRecorder(directory, metadata, self.interpolator)
        self.recorder.start()
        self.worker.recorder = self.recorder
        for widget in [
            self.choose_button,
            self.deadzone,
            self.capture_depth,
            self.depth,
            *self.parameters.values(),
        ]:
            widget.setEnabled(False)

    def stop_recording(self):
        if self.worker:
            self.worker.recorder = None
        if self.recorder:
            self.recorder.close()
            self.record_status.setText(
                f"已保存 {self.recorder.written} 行 / 对齐 {self.recorder.aligned} 行\n{self.recorder.directory}\n{self.recorder.error}"
            )
            self.recorder = None
        for widget in [
            self.choose_button,
            self.deadzone,
            self.capture_depth,
            self.depth,
            *self.parameters.values(),
        ]:
            widget.setEnabled(True)

    def inject_faults(self):
        if not self.simulator:
            raise ValueError("请先启动本机模拟 NX")
        model = self.simulator.model
        model.valid = not self.invalid_odom.isChecked()
        model.reject_switch = self.reject_mode.isChecked()
        model.delay = self.delay.value() / 1000
        model.drop = self.drop.value()
        model.duplicate = self.duplicate.value()

    def eventFilter(self, watched, event):
        if event.type() == C.QEvent.Type.ApplicationDeactivate:
            self.keys.clear()
        if (
            event.type() in (C.QEvent.Type.KeyPress, C.QEvent.Type.KeyRelease)
            and self.source.currentIndex() == 1
        ):
            key = event.key()
            if event.type() == C.QEvent.Type.KeyRelease and not event.isAutoRepeat():
                self.keys.discard(key)
            editable = isinstance(
                W.QApplication.focusWidget(),
                (W.QLineEdit, W.QAbstractSpinBox, W.QComboBox),
            )
            if (
                key
                in [
                    C.Qt.Key.Key_W,
                    C.Qt.Key.Key_S,
                    C.Qt.Key.Key_A,
                    C.Qt.Key.Key_D,
                    C.Qt.Key.Key_Q,
                    C.Qt.Key.Key_E,
                    C.Qt.Key.Key_R,
                    C.Qt.Key.Key_F,
                ]
                and not editable
            ):
                if not event.isAutoRepeat():
                    if event.type() == C.QEvent.Type.KeyPress:
                        self.keys.add(key)
                    else:
                        self.keys.discard(key)
                return True
        return super().eventFilter(watched, event)

    def refresh(self):
        now = time.monotonic()
        self.input_worker.deadzone = self.deadzone.value()
        devices = latest_get(self.input_worker.devices, self.device_list)
        if devices != self.device_list:
            self.device_list = devices
            self.devices.clear()
            self.devices.addItem("选择手柄", -1)
            for i, name in devices:
                self.devices.addItem(f"{i}: {name}", i)
        self.gamepad_state = latest_get(self.input_worker.snapshots, self.gamepad_state)
        state = self.gamepad_state
        if self.source.currentIndex() == 1:
            if not self.isActiveWindow() or isinstance(
                W.QApplication.focusWidget(),
                (W.QLineEdit, W.QAbstractSpinBox, W.QComboBox),
            ):
                self.keys.clear()
            key = lambda letter: int(getattr(C.Qt.Key, "Key_" + letter) in self.keys)
            state = InputSnapshot(
                now,
                self.isActiveWindow(),
                Axes(
                    0.5 * (key("Q") - key("E")),
                    0.5 * (key("W") - key("S")),
                    0.5 * (key("A") - key("D")),
                    0.5 * key("F"),
                    0.5 * key("R"),
                ),
                source="keyboard",
            )
        for _ in range(32):
            try:
                stamp, button = self.input_worker.buttons.get_nowait()
            except queue.Empty:
                break
            if (
                self.source.currentIndex() == 0
                and now - stamp <= 0.2
                and self.snapshot
                and not self.snapshot["pending"]
            ):
                if button == 6:
                    self.attempt(
                        lambda: self.action(
                            Action.SET_ARMED, not self.snapshot["armed"]
                        )
                    )
                elif button in (0, 1):
                    self.attempt(lambda b=button: self.toggle_light(b))
        if self.worker:
            latest_put(self.worker.inputs, state)
            self.snapshot = latest_get(self.worker.snapshots, self.snapshot)
        a = state.axes
        self.input_preview.setText(
            f"{state.source} · {'已连接' if state.connected else '未连接'}\n前后 {a.forward:+.2f} · 左右 {a.left:+.2f} · 偏航 {a.yaw:+.2f}\n下降 {a.down:.2f} · 上升 {a.up:.2f} · 云台 {state.hat}"
        )
        self.render_snapshot(now)
        if self.video:
            frame = latest_get(self.video.frames)
            if frame is not None:
                self.video_image.show_rgb(frame, self.video.width, self.video.height)
            self.video_status.setText(self.video.status)
        frame = latest_get(self.camera_frames)
        if frame:
            if frame.image_bgr is not None:
                rgb = frame.image_bgr[:, :, ::-1].copy()
                self.camera_image.show_rgb(rgb.data, rgb.shape[1], rgb.shape[0])
            self.camera_status.setText(frame.snapshot.message)
        if self.camera:
            self.camera_status.setText(
                self.camera.latest_snapshot().message
                + " | "
                + self.camera.recording_status()
            )
            self.camera_status.setWordWrap(True)
        if self.recorder:
            self.record_status.setText(
                f"记录 {self.recorder.written} 行 / 对齐 {self.recorder.aligned} 行 / 丢弃 {self.recorder.dropped}\n{self.recorder.error or self.recorder.directory}"
            )

    def render_snapshot(self, now):
        s = self.snapshot
        if s is None:
            self.sim_badge.setText("未连接")
            self.status.setText(
                "连接：离线 | 模式：停止 | 电机：未知 | NX 定位：无数据"
            )
            return
        fresh = now - s["now"] <= 0.2
        odom = s["odometry"]
        valid = bool(
            fresh
            and odom
            and odom.valid
            and now - odom.received <= 0.2
            and now <= odom.expires
        )
        self.sim_badge.setText(
            "SIMULATED NX · 模拟运行" if s["simulated"] else "NX / 实机接口"
        )
        self.status.setText(
            f"连接：{'在线' if s['ready'] and fresh else '失联'} | 模式：{MODE_LABEL[s['mode']]} / NX {MODE_LABEL.get(s['actual_mode'], '未知')} | 电机：{'启用' if s['armed'] else '停用'} | 定位：{'有效' if valid else '无效'} | 代次 {s['epoch']}"
        )
        message = (
            self.worker.error if self.worker and self.worker.error else s["message"]
        )
        if s["output"].complete:
            message = "轨迹完成：水平和偏航输出为零，继续保持目标深度"
        self.notice.setText(self.ui_error or message)
        if odom and self.last_seq != (odom.reset_id, odom.sample_seq):
            if self.last_seq and self.last_seq[0] != odom.reset_id:
                self.actual_xy.clear()
            self.last_seq = (odom.reset_id, odom.sample_seq)
            p = odom.pose
            self.actual_xy.append((p.x_m, p.y_m))
            t = s["telemetry"]
            net = (
                list(t.network)
                if t and now - s["telemetry_received"] <= 0.2 and t.valid_mask & 16
                else [float("nan")] * 3
            )
            self.history.append(
                (
                    now,
                    p.x_velocity_m_s,
                    p.y_velocity_m_s,
                    p.yaw_rate_rad_s,
                    p.x_m,
                    p.y_m,
                    p.yaw_rad,
                    *net,
                )
            )
        if self.actual_xy:
            self.actual_curve.setData(*zip(*self.actual_xy))
        if s["points"] and s["mode"] == Mode.TRAJECTORY:
            self.ref_curve.setData(
                [p.x_m for p in s["points"]], [p.y_m for p in s["points"]]
            )
        if s["output"].reference:
            r = s["output"].reference
            self.ref_dot.setData([r.x_m], [r.y_m])
        if self.tabs.currentIndex() == 1 and self.history:
            columns = list(zip(*self.history))
            start = 1 + 3 * self.chart_group.currentIndex()
            labels = (
                [("vx", "m/s"), ("vy", "m/s"), ("yaw rate", "rad/s")]
                if start != 4
                else [("x", "m"), ("y", "m"), ("yaw", "rad")]
            )
            for i, curve in enumerate(self.telemetry_curves):
                curve.setData(columns[0], columns[start + i])
                self.telemetry_plots[i].setLabel("left", *labels[i])
        t = s["telemetry"]
        self.diagnostics.setText(
            f"控制周期：50 Hz · tick {s['ticks']} · 超期 {s['overruns']} · 最大间隔 {s['max_interval'] * 1000:.1f} ms\n电压：{t.voltage if t and t.valid_mask & 1 else '—'} V · 网络丢弃 {s['dropped_packets']} · {s['network_error']}"
        )

    def closeEvent(self, event):
        self.timer.stop()
        W.QApplication.instance().removeEventFilter(self)
        self.disconnect()
        self.stop_video()
        self.input_worker.close()
        try:
            self.stop_camera()
        except ValueError:
            pass
        event.accept()


def main():
    parser = argparse.ArgumentParser(
        description="FinsROV-Heavy multimode ground station"
    )
    parser.add_argument("--simulate", action="store_true")
    parser.add_argument("--nx", default="192.168.0.2")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=54321)
    parser.add_argument("--nx-port", type=int, default=54322)
    parser.add_argument("--gst", default="gst-launch-1.0")
    parser.add_argument("--video-test", action="store_true")
    args = parser.parse_args()
    app = W.QApplication(sys.argv[:1])
    configure_font(app)
    window = MainWindow(
        Config(
            bind_ip=args.bind,
            port=args.port,
            nx_ip=args.nx,
            nx_port=args.nx_port,
            gst_executable=args.gst,
        ),
        args.simulate,
        args.video_test,
    )
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
