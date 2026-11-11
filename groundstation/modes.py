"""Mode implementations return commands; they never send packets."""

import math
from typing import Protocol
from trajectory_tracking import TrajectoryFollower, Waypoint, normalize_angle
from .models import Axes, Setpoint


class ControlMode(Protocol):
    """A mode owns its state, never transport, UI, camera, or recorder handles."""

    def enter(self, now: float, **context) -> None: ...
    def update(self, now: float, odometry, input_state) -> Setpoint: ...
    def exit(self) -> None: ...


def transform_waypoints(points, origin):
    c, s = math.cos(origin.yaw_rad), math.sin(origin.yaw_rad)
    return tuple(
        Waypoint(
            origin.x_m + c * p.x_m - s * p.y_m,
            origin.y_m + s * p.x_m + c * p.y_m,
            normalize_angle(origin.yaw_rad + p.yaw_rad),
            p.duration_s,
        )
        for p in points
    )


class IdleMode:
    def enter(self, now, **kwargs):
        pass

    def update(self, now, odometry, input_state):
        return Setpoint()

    def exit(self):
        pass


class ManualMode(IdleMode):
    def enter(self, now, input_state, **kwargs):
        if (
            not input_state.connected
            or not input_state.axes.valid()
            or not input_state.axes.neutral()
            or now - input_state.received > 0.2
        ):
            raise ValueError("先连接手动输入设备并回中，再启用手动模式")
        self.source = input_state.source

    def update(self, now, odometry, input_state):
        if (
            not input_state.connected
            or now - input_state.received > 0.2
            or input_state.source != self.source
            or not input_state.axes.valid()
        ):
            raise ValueError("手动输入已失效；回中后手动恢复")
        return Setpoint(input_state.axes)


class TrajectoryMode(IdleMode):
    def __init__(self):
        self.follower = TrajectoryFollower()
        self.depth = 0.0
        self.points = ()
        self.reset_id = None

    def enter(self, now, odometry, points, settings, depth=None, **kwargs):
        if (
            odometry is None
            or not odometry.valid
            or not odometry.depth_valid
            or now - odometry.received > 0.2
            or now > odometry.expires
        ):
            raise ValueError("需要有效且新鲜的 NX 位姿与深度")
        self.depth = odometry.depth if depth is None else depth
        if not math.isfinite(self.depth) or self.depth < 0:
            raise ValueError("定深目标须为非负有限值")
        self.reset_id = odometry.reset_id
        self.settings = settings
        self.points = transform_waypoints(points, odometry.pose)
        self.follower.start(now, odometry.pose, self.points, settings)

    def update(self, now, odometry, input_state):
        if (
            odometry is None
            or not odometry.valid
            or not odometry.depth_valid
            or now - odometry.received > 0.2
            or now > odometry.expires
            or odometry.reset_id != self.reset_id
        ):
            raise ValueError("NX 定位失效、过期或原点已重置；轨迹已停止")
        result = self.follower.update(now, odometry.pose, self.settings)
        return Setpoint(
            Axes(result.yaw_rate, result.forward_throttle, result.left_throttle),
            True,
            self.depth,
            result.reference,
            result.reference.complete,
        )

    def exit(self):
        self.follower.stop()
