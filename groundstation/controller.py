"""Single-owner controller state machine, injectable clock and message sender."""

from dataclasses import dataclass
import math
import secrets
from trajectory_tracking import PoseEstimate
from .models import (
    Action,
    Config,
    InputSnapshot,
    Mode,
    Odometry,
    Result,
    Setpoint,
    newer,
)
from .modes import IdleMode, ManualMode, TrajectoryMode


@dataclass
class Pending:
    fields: dict
    sent: float
    retries: int = 0
    context: dict | None = None


class Controller:
    def __init__(self, send, config=None):
        self.send = send
        self.config = config or Config()
        self.session = secrets.randbits(63) or 1
        self.peer = None
        self.features = 0
        self.mode_mask = 0
        self.simulated = False
        self.last_peer = -math.inf
        self.last_status = -math.inf
        self.actual_mode = Mode.IDLE
        self.last_discovery = -math.inf
        self.offset = None  # NX monotonic seconds minus PC monotonic seconds
        self.last_sync = -math.inf
        self.sync_requests = {}
        self.mode = Mode.IDLE
        self.epoch = 0
        self.next_epoch = 0
        self.sequence = 0
        self.request_id = 0
        self.pending = {}
        self.armed = False
        self.lights = [False, False]
        self.input = InputSnapshot()
        self.odometry = None
        self.telemetry = None
        self.telemetry_received = -math.inf
        self.output = Setpoint()
        self.error = "未连接"
        self.fault = False
        self.modes = {
            Mode.IDLE: IdleMode(),
            Mode.MANUAL: ManualMode(),
            Mode.TRAJECTORY: TrajectoryMode(),
        }

    def ready(self, now):
        return (
            self.peer is not None
            and now - self.last_peer <= 2
            and self.offset is not None
            and now - self.last_sync <= 3
        )

    def receive(self, message, now):
        kind = message.get_type()
        if kind == "FINS_CAPABILITIES":
            if message.protocol_version != 1 or not message.mode_mask & 1:
                self.fail("NX 协议版本或能力不匹配")
                self.peer = None
                return
            if self.peer != message.session:
                self.fail("新连接，需明确选择模式及电机状态")
                self.peer = message.session
                self.epoch = self.next_epoch = 0
                self.offset = None
                self.sync_requests.clear()
                self.last_discovery = -math.inf
                self.odometry = None
                self.telemetry = None
                self.armed = False
            self.features = message.features
            self.mode_mask = message.mode_mask
            self.simulated = bool(message.simulated)
            self.last_peer = now
            return
        if kind == "TIMESYNC":
            sent = self.sync_requests.pop(message.ts1, None)
            if (
                sent is not None
                and message.tc1
                and 0 <= now - sent <= 0.2
                and self.peer
            ):
                self.offset = message.tc1 / 1e9 - (sent + now) / 2
                self.last_sync = now
            return
        if getattr(message, "session", None) != self.peer or self.peer is None:
            return
        self.last_peer = now
        if kind == "FINS_STATUS":
            if message.client_session != self.session:
                if self.mode != Mode.IDLE:
                    self.fail("控制权已变化")
                return
            if message.epoch < self.epoch:
                return
            self.last_status = now
            self.actual_mode = message.mode
            self.armed = bool(message.armed)
            self.lights = [bool(message.camera_light), bool(message.frame_light)]
            if message.faults and self.mode != Mode.IDLE:
                self.fail("NX 报告控制故障或指令过期")
            if (
                self.mode != Mode.IDLE
                and message.mode != self.mode
                and not self.pending
            ):
                self.fail("NX 实际模式已变化；需明确恢复")
            if message.epoch > self.epoch and not self.pending:
                self.fail("NX 控制代次已变化")
                self.epoch = message.epoch
                self.next_epoch = max(self.next_epoch, self.epoch)
        elif kind == "FINS_ACTION_ACK":
            self._ack(message, now)
        elif kind == "FINS_ODOMETRY":
            self._odometry(message, now)
        elif kind == "FINS_TELEMETRY":
            self.telemetry = message
            self.telemetry_received = now

    def _odometry(self, m, now):
        old = self.odometry
        if old and not newer(m.sample_seq, old.sample_seq):
            return
        if old and m.source_us < old.source_us:
            return
        values = [m.x, m.y, m.yaw, m.vx, m.vy, m.yaw_rate, m.depth]
        fresh = (
            self.offset is not None
            and -0.05 <= now + self.offset - m.source_us / 1e6 <= 0.2
        )
        valid = bool(m.valid) and all(math.isfinite(x) for x in values) and fresh
        pose = PoseEstimate(*values[:6])
        self.odometry = Odometry(
            pose,
            m.depth,
            valid,
            now,
            m.source_us,
            m.sample_seq,
            m.reset_id,
            bool(m.depth_valid) and fresh and math.isfinite(m.depth) and m.depth >= 0,
            m.source_us / 1e6 - (self.offset or 0) + 0.2,
        )
        if old and old.reset_id != m.reset_id and self.mode == Mode.TRAJECTORY:
            self.fail("里程计原点重置；重新启动轨迹以建立新原点")

    def request(self, action, value, now, context=None):
        if not self.ready(now):
            raise ValueError("等待 NX 能力握手与时间同步")
        if action == Action.SET_MODE:
            self._idle()
            self._send_setpoint(
                now
            )  # Every attempted handover revokes the old producer.
            self.pending = {
                k: p
                for k, p in self.pending.items()
                if p.fields["action"] != Action.SET_MODE
            }
            target = Mode(value)
            if not self.mode_mask & (1 << target):
                raise ValueError("NX 不支持所选模式")
            if target == Mode.MANUAL:
                ManualMode().enter(now, input_state=self.input)
                if now - self.input.received > 0.2:
                    raise ValueError("手动输入过期")
            if target == Mode.TRAJECTORY:
                if not self.armed or not self.features & 1:
                    raise ValueError("需先启用电机，且 NX 需支持定深")
                if (
                    not context
                    or not context.get("points")
                    or not context.get("settings")
                ):
                    raise ValueError("请先加载轨迹和控制参数")
                check = TrajectoryMode()
                check.enter(now, odometry=self.odometry, **(context or {}))
            self.next_epoch += 1
            epoch = self.next_epoch
        else:
            epoch = self.epoch
            if action == Action.SET_ARMED and value:
                ManualMode().enter(now, input_state=self.input)
                if self.mode != Mode.MANUAL or not self.input.axes.neutral():
                    raise ValueError("启用电机前请切到手动模式并回中")
            if (
                action == Action.SET_ARMED and not value
            ) or action == Action.RESET_ODOMETRY:
                self._idle()
                self._send_setpoint(now)
                # Cancel an outstanding mode activation so its late ACK cannot restart control.
                self.pending = {
                    k: p
                    for k, p in self.pending.items()
                    if p.fields["action"] != Action.SET_MODE
                }
        self.pending = {
            k: p for k, p in self.pending.items() if p.fields["action"] != action
        }
        self.request_id += 1
        fields = dict(
            session=self.session,
            target_session=self.peer,
            request_id=self.request_id,
            epoch=epoch,
            action=int(action),
            value=int(value),
        )
        self.pending[self.request_id] = Pending(fields, now, context=context)
        self.send("FINS_ACTION_REQUEST", **fields)
        self.fault = False
        self.error = "等待 NX 确认"

    def _ack(self, m, now):
        if m.target_session != self.session:
            return
        pending = self.pending.get(m.request_id)
        if (
            pending is None
            or m.action != pending.fields["action"]
            or m.epoch != pending.fields["epoch"]
        ):
            return
        del self.pending[m.request_id]
        if m.result != Result.ACCEPTED:
            self.fail(f"NX 拒绝操作 {m.action}，结果 {m.result}，原因 {m.reason}")
            return
        if m.action == Action.SET_MODE:
            self.epoch = m.epoch
            target = Mode(pending.fields["value"])
            try:
                self.modes[target].enter(
                    now,
                    input_state=self.input,
                    odometry=self.odometry,
                    **(pending.context or {}),
                )
                self.mode = target
            except ValueError as error:
                self.fail(str(error))
                return
        elif m.action == Action.SET_ARMED:
            self.armed = bool(pending.fields["value"])
        elif m.action in (Action.SET_CAMERA_LIGHT, Action.SET_FRAME_LIGHT):
            self.lights[int(m.action) - 4] = bool(pending.fields["value"])
        self.error = "已确认"

    def _idle(self):
        self.modes[self.mode].exit()
        self.mode = Mode.IDLE
        self.output = Setpoint()

    def fail(self, reason):
        self._idle()
        self.pending.clear()
        self.fault = True
        self.error = reason

    def tick(self, now):
        if now - self.last_discovery >= 1:
            self.send(
                "FINS_CAPABILITIES",
                session=self.session,
                protocol_version=1,
                mode_mask=0,
                features=3,
                simulated=0,
            )
            self.send(
                "HEARTBEAT",
                type=6,
                autopilot=8,
                base_mode=0,
                custom_mode=int(self.mode),
                system_status=4,
                mavlink_version=3,
            )
            stamp = int(now * 1e9)
            self.sync_requests = {stamp: now}
            self.send("TIMESYNC", tc1=0, ts1=stamp)
            self.last_discovery = now
        if not self.ready(now):
            if self.mode != Mode.IDLE or self.pending:
                self.fail("NX 连接或时间同步已失效；需手动恢复")
            return
        for key, pending in list(self.pending.items()):
            if now - pending.sent >= 0.3:
                if pending.retries >= 3:
                    self.fail("操作确认超时；未自动恢复运动")
                    break
                self.send("FINS_ACTION_REQUEST", **pending.fields)
                pending.sent, pending.retries = now, pending.retries + 1
        try:
            value = self.modes[self.mode].update(now, self.odometry, self.input)
            self.output = value if self.armed else Setpoint()
        except ValueError as error:
            self.fail(str(error))
        self._send_setpoint(now)

    def _send_setpoint(self, now):
        if not self.peer or self.offset is None:
            return
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        output = self.output
        axes = output.axes
        fields = dict(
            session=self.session,
            target_session=self.peer,
            source_us=max(0, int((now + self.offset) * 1e6)),
            epoch=self.epoch,
            sequence=self.sequence,
            valid_ms=200,
        )
        self.send(
            "FINS_CONTROL_SETPOINT",
            **fields,
            yaw=axes.yaw,
            forward=axes.forward,
            left=axes.left,
            down=axes.down,
            up=axes.up,
            depth_target=output.depth_target,
            vertical_mode=int(output.hold_depth),
        )
        hat = (
            self.input.hat
            if self.mode == Mode.MANUAL and self.input.connected
            else (0, 0)
        )
        self.send("FINS_AUX_SETPOINT", **fields, cam_yaw=hat[0], cam_pitch=hat[1])
