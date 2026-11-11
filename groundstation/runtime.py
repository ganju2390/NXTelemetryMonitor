"""Thread adapters. Only ControlWorker touches the controller or sends commands."""

import queue
import threading
import time
from .controller import Controller
from .inputs import Gamepad
from .models import Action, Config, InputSnapshot, Mode
from .queues import latest_get, latest_put
from .recording import make_row
from .wire import UDPLink


class InputWorker(threading.Thread):
    def __init__(self):
        super().__init__(name="input-devices", daemon=True)
        self.commands = queue.Queue(maxsize=16)
        self.snapshots = queue.Queue(maxsize=1)
        self.buttons = queue.Queue(maxsize=32)
        self.devices = queue.Queue(maxsize=1)
        self.stopping = threading.Event()
        self.deadzone = 0.0
        self.error = ""

    def run(self):
        gamepad = Gamepad()
        last_scan = -1
        try:
            while not self.stopping.wait(0.01):
                now = time.monotonic()
                try:
                    if now - last_scan >= 1:
                        latest_put(self.devices, gamepad.devices())
                        last_scan = now
                    command = latest_get(self.commands)
                    if command is not None:
                        gamepad.select(command)
                    state, edges, self.error = gamepad.sample(now, self.deadzone)
                    latest_put(self.snapshots, state)
                    for button in edges:
                        latest_put(self.buttons, (now, button))
                except Exception as error:
                    self.error = str(error)
                    latest_put(self.snapshots, InputSnapshot(received=now))
        finally:
            gamepad.close()

    def close(self):
        self.stopping.set()
        if self.is_alive():
            self.join(2)


class ControlWorker(threading.Thread):
    def __init__(self, config=None):
        super().__init__(name="control-50hz", daemon=True)
        self.config = config or Config()
        self.link = UDPLink(
            (self.config.bind_ip, self.config.port),
            (self.config.nx_ip, self.config.nx_port),
        )
        self.controller = Controller(self.link.send, self.config)
        self.commands = queue.Queue(maxsize=32)
        self.inputs = queue.Queue(maxsize=1)
        self.snapshots = queue.Queue(maxsize=1)
        self.stopping = threading.Event()
        self.urgent_stop = threading.Event()
        self.recorder = None  # writer is created/stopped outside the control thread
        self.error = ""
        self.ticks = self.overruns = 0
        self.max_interval = 0.0
        self.last_sample = None

    def command(self, action, value, context=None):
        try:
            self.commands.put_nowait((action, value, context))
        except queue.Full:
            self.urgent_stop.set()
            raise ValueError("操作队列已满，已请求停止")

    def stop_motion(self):
        self.urgent_stop.set()

    def snapshot(self, now):
        c = self.controller
        return dict(
            now=now,
            ready=c.ready(now),
            simulated=c.simulated,
            mode=c.mode,
            armed=c.armed,
            actual_mode=c.actual_mode,
            lights=tuple(c.lights),
            epoch=c.epoch,
            pending=bool(c.pending),
            fault=c.fault,
            message=c.error,
            odometry=c.odometry,
            telemetry=c.telemetry,
            telemetry_received=c.telemetry_received,
            input=c.input,
            output=c.output,
            points=tuple(c.modes[Mode.TRAJECTORY].points),
            ticks=self.ticks,
            overruns=self.overruns,
            max_interval=self.max_interval,
            network_error=self.link.error,
            dropped_packets=self.link.dropped,
        )

    def run(self):
        self.link.start()
        previous = deadline = time.monotonic()
        try:
            while not self.stopping.is_set():
                now = time.monotonic()
                self.max_interval = max(self.max_interval, now - previous)
                previous = now
                c = self.controller
                c.input = latest_get(self.inputs, c.input)
                for _ in range(512):
                    try:
                        message, received = self.link.incoming.get_nowait()
                    except queue.Empty:
                        break
                    c.receive(message, received)
                    odom = c.odometry
                    if odom and (c.peer, odom.sample_seq) != self.last_sample:
                        self.last_sample = c.peer, odom.sample_seq
                        recorder = self.recorder
                        if recorder:
                            recorder.record(make_row(c, now))
                if self.link.error:
                    raise RuntimeError(self.link.error)
                if self.urgent_stop.is_set():
                    self.urgent_stop.clear()
                    while not self.commands.empty():
                        self.commands.get_nowait()
                    c.fail("用户停止；重新选择模式才会恢复")
                    if c.ready(now):
                        c.request(Action.SET_MODE, Mode.IDLE, now)
                for _ in range(32):
                    try:
                        action, value, context = self.commands.get_nowait()
                    except queue.Empty:
                        break
                    try:
                        c.request(action, value, now, context)
                    except ValueError as error:
                        c.error = str(error)
                c.tick(now)
                self.ticks += 1
                latest_put(self.snapshots, self.snapshot(now))
                deadline += self.config.period_s
                if deadline < time.monotonic():
                    self.overruns += 1
                    deadline = time.monotonic() + self.config.period_s
                self.stopping.wait(max(0.0, deadline - time.monotonic()))
        except Exception as error:
            self.error = str(error)
            self.controller.fail(self.error)
            latest_put(self.snapshots, self.snapshot(time.monotonic()))
        finally:
            # Best effort explicit zero and motor disable; receiver watchdog is authoritative.
            try:
                now = time.monotonic()
                self.controller.fail("连接关闭")
                self.controller._send_setpoint(now)
                if self.controller.ready(now):
                    self.controller.request(Action.SET_ARMED, 0, now)
                time.sleep(0.04)
            except Exception:
                pass
            self.link.close()

    def close(self):
        self.stopping.set()
        if self.is_alive():
            self.join(3)
        else:
            self.link.close()
