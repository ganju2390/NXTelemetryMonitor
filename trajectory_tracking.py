"""CSV waypoint planning and AprilTag closed-loop tracking primitives."""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass, replace
from pathlib import Path


ORIGIN_TOLERANCE_M = 1e-4
WAYPOINT_ARRIVAL_RADIUS_M = 0.15
WAYPOINT_ARRIVAL_YAW_ERROR_RAD = math.radians(10.0)
EPSILON = 1e-8


class TrajectoryError(ValueError):
    """Raised when a trajectory file or trajectory configuration is invalid."""


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def angle_difference(target: float, current: float) -> float:
    return normalize_angle(target - current)


@dataclass(frozen=True)
class PoseEstimate:
    x_m: float
    y_m: float
    yaw_rad: float
    x_velocity_m_s: float = 0.0
    y_velocity_m_s: float = 0.0
    yaw_rate_rad_s: float = 0.0


@dataclass(frozen=True)
class Waypoint:
    x_m: float
    y_m: float
    yaw_rad: float
    duration_s: float


@dataclass(frozen=True)
class TrajectoryReference:
    x_m: float
    y_m: float
    yaw_rad: float
    x_velocity_m_s: float
    y_velocity_m_s: float
    yaw_rate_rad_s: float
    complete: bool


@dataclass(frozen=True)
class TrackingCommand:
    yaw_rate: float
    forward_throttle: float
    left_throttle: float
    reference: TrajectoryReference


@dataclass(frozen=True)
class TrackingSettings:
    position_kp_s: float = 0.70
    velocity_to_throttle_kp: float = 1.00
    yaw_kp_s: float = 0.50
    yaw_full_scale_rad_s: float = 0.40
    output_slew_per_s: float = 1.00

    def validate(self) -> None:
        values = tuple(self.__dict__.values())
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise TrajectoryError("轨迹控制参数必须是有限的正数")


def load_waypoints(path: Path) -> tuple[Waypoint, ...]:
    """Load the strict x_m,y_m,yaw_rad CSV waypoint contract."""
    try:
        with path.open("r", encoding="utf-8-sig", newline="") as file:
            reader = csv.DictReader(file)
            if reader.fieldnames != ["x_m", "y_m", "yaw_rad", "duration_s"]:
                raise TrajectoryError("CSV 表头必须严格为 x_m,y_m,yaw_rad,duration_s")
            points: list[Waypoint] = []
            for row_number, row in enumerate(reader, start=2):
                try:
                    point = Waypoint(float(row["x_m"]), float(row["y_m"]), float(row["yaw_rad"]), float(row["duration_s"]))
                except (KeyError, TypeError, ValueError) as error:
                    raise TrajectoryError(f"第 {row_number} 行不是有效数值") from error
                if not all(math.isfinite(value) for value in (point.x_m, point.y_m, point.yaw_rad, point.duration_s)):
                    raise TrajectoryError(f"第 {row_number} 行包含非有限数值")
                if point.duration_s <= 0.0:
                    raise TrajectoryError(f"第 {row_number} 行 duration_s 必须大于 0")
                points.append(point)
    except OSError as error:
        raise TrajectoryError(f"无法读取轨迹文件：{error}") from error

    if len(points) < 2:
        raise TrajectoryError("轨迹至少需要两个关键点")
    first = points[0]
    if abs(first.x_m) > ORIGIN_TOLERANCE_M or abs(first.y_m) > ORIGIN_TOLERANCE_M:
        raise TrajectoryError("CSV 首个关键点必须为相机全局原点 (0,0)")
    return tuple(points)


class NaturalCubicSpline:
    """One-dimensional natural C2 cubic spline with first derivative output."""

    def __init__(self, times: tuple[float, ...], values: tuple[float, ...]) -> None:
        if len(times) != len(values) or len(times) < 2:
            raise TrajectoryError("样条至少需要两个时间点")
        if any(right <= left for left, right in zip(times, times[1:])):
            raise TrajectoryError("样条时间必须严格递增")
        self.times = times
        self.values = values
        self._b, self._c, self._d = self._coefficients()

    def _coefficients(self) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
        point_count = len(self.times)
        segment_count = point_count - 1
        h = [self.times[index + 1] - self.times[index] for index in range(segment_count)]
        alpha = [0.0] * point_count
        for index in range(1, segment_count):
            alpha[index] = (
                3.0 / h[index] * (self.values[index + 1] - self.values[index])
                - 3.0 / h[index - 1] * (self.values[index] - self.values[index - 1])
            )

        lower = [0.0] * point_count
        mu = [0.0] * point_count
        z = [0.0] * point_count
        lower[0] = 1.0
        for index in range(1, segment_count):
            lower[index] = 2.0 * (self.times[index + 1] - self.times[index - 1]) - h[index - 1] * mu[index - 1]
            mu[index] = h[index] / lower[index]
            z[index] = (alpha[index] - h[index - 1] * z[index - 1]) / lower[index]
        lower[-1] = 1.0

        b = [0.0] * segment_count
        c = [0.0] * point_count
        d = [0.0] * segment_count
        for index in range(segment_count - 1, -1, -1):
            c[index] = z[index] - mu[index] * c[index + 1]
            b[index] = (self.values[index + 1] - self.values[index]) / h[index] - h[index] * (c[index + 1] + 2.0 * c[index]) / 3.0
            d[index] = (c[index + 1] - c[index]) / (3.0 * h[index])
        return tuple(b), tuple(c[:-1]), tuple(d)

    def sample(self, time_s: float) -> tuple[float, float, float]:
        if time_s <= self.times[0]:
            return self.values[0], self._b[0], 2.0 * self._c[0]
        if time_s >= self.times[-1]:
            return self.values[-1], 0.0, 0.0
        index = next(index for index in range(len(self.times) - 1) if time_s < self.times[index + 1])
        delta = time_s - self.times[index]
        value = self.values[index] + self._b[index] * delta + self._c[index] * delta * delta + self._d[index] * delta * delta * delta
        velocity = self._b[index] + 2.0 * self._c[index] * delta + 3.0 * self._d[index] * delta * delta
        acceleration = 2.0 * self._c[index] + 6.0 * self._d[index] * delta
        return value, velocity, acceleration


class PlannedTrajectory:
    """A C2 position/yaw trajectory that passes through all supplied waypoints."""

    def __init__(self, initial: PoseEstimate, waypoints: tuple[Waypoint, ...], settings: TrackingSettings) -> None:
        settings.validate()
        points = [Waypoint(initial.x_m, initial.y_m, initial.yaw_rad, 0.0), *waypoints]
        self._points = self._remove_duplicate_points(points)
        if len(self._points) < 2:
            raise TrajectoryError("当前位置与轨迹点完全重合，无法建立轨迹")
        self._settings = settings
        self._times = self._build_times()
        self._x_spline = NaturalCubicSpline(self._times, tuple(point.x_m for point in self._points))
        self._y_spline = NaturalCubicSpline(self._times, tuple(point.y_m for point in self._points))
        self._yaw_spline = NaturalCubicSpline(self._times, self._unwrapped_yaws())

    @staticmethod
    def _remove_duplicate_points(points: list[Waypoint]) -> tuple[Waypoint, ...]:
        result = [points[0]]
        for point in points[1:]:
            previous = result[-1]
            if math.hypot(point.x_m - previous.x_m, point.y_m - previous.y_m) <= ORIGIN_TOLERANCE_M and abs(angle_difference(point.yaw_rad, previous.yaw_rad)) <= 1e-4:
                continue
            result.append(point)
        return tuple(result)

    def _build_times(self) -> tuple[float, ...]:
        times = [0.0]
        for point in self._points[1:]:
            times.append(times[-1] + point.duration_s)
        return tuple(times)

    def _unwrapped_yaws(self) -> tuple[float, ...]:
        values = [self._points[0].yaw_rad]
        for point in self._points[1:]:
            values.append(values[-1] + angle_difference(point.yaw_rad, values[-1]))
        return tuple(values)

    @property
    def duration_s(self) -> float:
        return self._times[-1]

    def sample(self, elapsed_s: float) -> TrajectoryReference:
        complete = elapsed_s >= self.duration_s
        sample_time = min(max(elapsed_s, 0.0), self.duration_s)
        x_m, x_velocity, _ = self._x_spline.sample(sample_time)
        y_m, y_velocity, _ = self._y_spline.sample(sample_time)
        yaw, yaw_rate, _ = self._yaw_spline.sample(sample_time)
        return TrajectoryReference(
            x_m,
            y_m,
            normalize_angle(yaw),
            0.0 if complete else x_velocity,
            0.0 if complete else y_velocity,
            0.0 if complete else yaw_rate,
            complete,
        )


class TrajectoryFollower:
    """Plans one C2 segment at a time and advances only after pose arrival."""

    def __init__(self) -> None:
        self.trajectory: PlannedTrajectory | None = None
        self.started_monotonic: float | None = None
        self._waypoints: tuple[Waypoint, ...] = ()
        self.waypoint_index = 0
        self._previous_command = (0.0, 0.0, 0.0)
        self._last_update_monotonic: float | None = None

    @property
    def active(self) -> bool:
        return bool(self._waypoints)

    def start(self, now: float, pose: PoseEstimate, waypoints: tuple[Waypoint, ...], settings: TrackingSettings) -> None:
        self._waypoints = waypoints
        self.waypoint_index = 0
        self.trajectory = None
        self.started_monotonic = None
        self._previous_command = (0.0, 0.0, 0.0)
        self._last_update_monotonic = now
        self._begin_current_segment(now, pose, settings)

    def stop(self) -> None:
        self.trajectory = None
        self.started_monotonic = None
        self._waypoints = ()
        self.waypoint_index = 0
        self._previous_command = (0.0, 0.0, 0.0)
        self._last_update_monotonic = None

    @staticmethod
    def _slew(previous: float, target: float, maximum_delta: float) -> float:
        return max(previous - maximum_delta, min(previous + maximum_delta, target))

    @staticmethod
    def _has_arrived(target: Waypoint, pose: PoseEstimate) -> bool:
        position_arrived = math.hypot(target.x_m - pose.x_m, target.y_m - pose.y_m) <= WAYPOINT_ARRIVAL_RADIUS_M
        yaw_arrived = abs(angle_difference(target.yaw_rad, pose.yaw_rad)) <= WAYPOINT_ARRIVAL_YAW_ERROR_RAD
        return position_arrived and yaw_arrived

    def _begin_current_segment(self, now: float, pose: PoseEstimate, settings: TrackingSettings) -> None:
        if self.waypoint_index >= len(self._waypoints):
            self.trajectory = None
            self.started_monotonic = None
            return
        target = self._waypoints[self.waypoint_index]
        if self._has_arrived(target, pose):
            self.trajectory = None
            self.started_monotonic = None
            return
        self.trajectory = PlannedTrajectory(pose, (target,), settings)
        self.started_monotonic = now

    def update(self, now: float, pose: PoseEstimate, settings: TrackingSettings) -> TrackingCommand:
        if not self.active:
            raise TrajectoryError("轨迹未启动")
        settings.validate()
        while self.waypoint_index < len(self._waypoints):
            target = self._waypoints[self.waypoint_index]
            if not self._has_arrived(target, pose):
                break
            self.waypoint_index += 1
            self.trajectory = None
            self.started_monotonic = None

        if self.waypoint_index >= len(self._waypoints):
            self._previous_command = (0.0, 0.0, 0.0)
            final = self._waypoints[-1]
            return TrackingCommand(
                0.0,
                0.0,
                0.0,
                TrajectoryReference(final.x_m, final.y_m, normalize_angle(final.yaw_rad), 0.0, 0.0, 0.0, True),
            )

        if self.trajectory is None or self.started_monotonic is None:
            self._begin_current_segment(now, pose, settings)
        if self.trajectory is None or self.started_monotonic is None:
            raise TrajectoryError("无法建立当前轨迹段")
        reference = replace(self.trajectory.sample(now - self.started_monotonic), complete=False)

        global_x = settings.velocity_to_throttle_kp * (
            reference.x_velocity_m_s + settings.position_kp_s * (reference.x_m - pose.x_m) - pose.x_velocity_m_s
        )
        global_y = settings.velocity_to_throttle_kp * (
            reference.y_velocity_m_s + settings.position_kp_s * (reference.y_m - pose.y_m) - pose.y_velocity_m_s
        )
        horizontal_norm = math.hypot(global_x, global_y)
        if horizontal_norm > 1.0:
            global_x /= horizontal_norm
            global_y /= horizontal_norm

        forward_target = math.cos(pose.yaw_rad) * global_x + math.sin(pose.yaw_rad) * global_y
        left_target = -math.sin(pose.yaw_rad) * global_x + math.cos(pose.yaw_rad) * global_y
        yaw_target = (reference.yaw_rate_rad_s + settings.yaw_kp_s * angle_difference(reference.yaw_rad, pose.yaw_rad)) / settings.yaw_full_scale_rad_s
        forward_target = max(-1.0, min(1.0, forward_target))
        left_target = max(-1.0, min(1.0, left_target))
        yaw_target = max(-1.0, min(1.0, yaw_target))

        previous_update = self._last_update_monotonic if self._last_update_monotonic is not None else now
        delta = settings.output_slew_per_s * max(0.0, now - previous_update)
        self._last_update_monotonic = now
        yaw = self._slew(self._previous_command[0], yaw_target, delta)
        forward = self._slew(self._previous_command[1], forward_target, delta)
        left = self._slew(self._previous_command[2], left_target, delta)
        self._previous_command = (yaw, forward, left)
        return TrackingCommand(yaw, forward, left, reference)
