from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import IntEnum
from .generated import finsrov as mav
from trajectory_tracking import PoseEstimate, TrackingSettings, TrajectoryReference


class Mode(IntEnum):
    IDLE = mav.FINS_MODE_IDLE
    MANUAL = mav.FINS_MODE_MANUAL
    TRAJECTORY = mav.FINS_MODE_TRAJECTORY


class Action(IntEnum):
    SET_MODE = mav.FINS_ACTION_SET_MODE
    SET_ARMED = mav.FINS_ACTION_SET_ARMED
    RESET_ODOMETRY = mav.FINS_ACTION_RESET_ODOMETRY
    SET_CAMERA_LIGHT = mav.FINS_ACTION_SET_CAMERA_LIGHT
    SET_FRAME_LIGHT = mav.FINS_ACTION_SET_FRAME_LIGHT


class Result(IntEnum):
    ACCEPTED = mav.FINS_RESULT_ACCEPTED
    REJECTED = mav.FINS_RESULT_REJECTED
    INVALID = mav.FINS_RESULT_INVALID


@dataclass(frozen=True)
class Axes:
    yaw: float = 0.0
    forward: float = 0.0
    left: float = 0.0
    down: float = 0.0
    up: float = 0.0

    def valid(self):
        return all(
            math.isfinite(v) and -1 <= v <= 1
            for v in (self.yaw, self.forward, self.left)
        ) and all(math.isfinite(v) and 0 <= v <= 1 for v in (self.down, self.up))

    def neutral(self):
        return (
            max(
                abs(self.yaw),
                abs(self.forward),
                abs(self.left),
                abs(self.up - self.down),
            )
            <= 0.02
        )


@dataclass(frozen=True)
class InputSnapshot:
    received: float = 0.0
    connected: bool = False
    axes: Axes = field(default_factory=Axes)
    hat: tuple[int, int] = (0, 0)
    source: str = "gamepad"


@dataclass(frozen=True)
class Odometry:
    pose: PoseEstimate
    depth: float
    valid: bool
    received: float
    source_us: int
    sample_seq: int
    reset_id: int
    depth_valid: bool = True
    expires: float = math.inf


@dataclass(frozen=True)
class Setpoint:
    axes: Axes = field(default_factory=Axes)
    hold_depth: bool = False
    depth_target: float = 0.0
    reference: TrajectoryReference | None = None
    complete: bool = False


@dataclass
class Config:
    bind_ip: str = "0.0.0.0"
    port: int = 54321
    nx_ip: str = "192.168.0.2"
    nx_port: int = 54322
    timeout_s: float = 0.2
    period_s: float = 0.02
    settings: TrackingSettings = field(default_factory=TrackingSettings)
    video_port: int = 5600
    video_width: int = 1280
    video_height: int = 480
    video_latency_ms: int = 50
    gst_executable: str = "gst-launch-1.0"


def newer(sequence: int, previous: int | None) -> bool:
    return previous is None or 0 < ((sequence - previous) & 0xFFFFFFFF) < 0x80000000
