"""AprilTag 25H9 localisation primitives for the NX telemetry monitor.

The camera is fixed above the vehicle.  The four tags are attached to the
vehicle, so their known positions form one rigid PnP target.  This module has
no OpenCV import at module-load time: the rest of the telemetry monitor can
still run and report a useful error when the optional vision dependencies have
not been installed yet.
"""

from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from collections import deque
from pathlib import Path
from typing import Any


CALIBRATION_WIDTH = 1280
CALIBRATION_HEIGHT = 720
VISION_STALE_SECONDS = 0.5
MAX_CORNER_REPROJECTION_ERROR_PX = 6.0
MAX_POSITION_SPEED_M_S = 0.8
MAX_YAW_RATE_RAD_S = 3.0
# 可靠视觉帧可能因 AprilTag 求解而降至低于相机名义帧率。只要两端都通过
# 几何与运动门控，500 ms 内的间隔仍可安全地用于 200 Hz 遥测插值。
MAX_INTERPOLATION_GAP_SECONDS = 0.50


@dataclass(frozen=True)
class CameraCalibration:
    width: int
    height: int
    matrix: tuple[float, ...]
    distortion: tuple[float, ...]


@dataclass(frozen=True)
class TagPose:
    tag_id: int
    center_x_m: float
    center_y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class TagLayout:
    marker_length_m: float
    tags: tuple[TagPose, ...]

    def by_id(self) -> dict[int, TagPose]:
        return {tag.tag_id: tag for tag in self.tags}

    def object_corners(self, tag_id: int) -> tuple[tuple[float, float, float], ...]:
        """Return body-frame corners matching OpenCV's TL, TR, BR, BL order.

        Body +X is ROV forward and +Y is ROV left.  A tag yaw of zero means
        the printed top edge faces body +X.  The default layout therefore has
        IDs 10/13 on the forward side, as installed on the ROV.
        """
        tag = self.by_id().get(tag_id)
        if tag is None:
            raise KeyError(f"tag ID {tag_id} is not in the configured layout")

        half = self.marker_length_m / 2.0
        local_corners = ((half, half), (half, -half), (-half, -half), (-half, half))
        cosine = math.cos(tag.yaw_rad)
        sine = math.sin(tag.yaw_rad)
        return tuple(
            (
                tag.center_x_m + cosine * x - sine * y,
                tag.center_y_m + sine * x + cosine * y,
                0.0,
            )
            for x, y in local_corners
        )


@dataclass(frozen=True)
class VisionSnapshot:
    valid: bool
    x_m: float | None
    y_m: float | None
    yaw_rad: float | None
    tag_ids: tuple[int, ...]
    reprojection_error_px: float | None
    captured_monotonic: float
    captured_utc: str
    message: str

    @classmethod
    def unavailable(cls, message: str) -> "VisionSnapshot":
        now = time.monotonic()
        return cls(False, None, None, None, (), None, now, datetime.now(timezone.utc).isoformat(timespec="milliseconds"), message)

    def age_ms(self, now: float | None = None) -> float:
        return max(0.0, ((time.monotonic() if now is None else now) - self.captured_monotonic) * 1000.0)

    def is_fresh(self, now: float | None = None) -> bool:
        return self.valid and self.age_ms(now) <= VISION_STALE_SECONDS * 1000.0


@dataclass(frozen=True)
class VisionFrame:
    snapshot: VisionSnapshot
    image_bgr: Any | None


class VisionInterpolator:
    """Keeps only reliable camera poses and interpolates them at telemetry time."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._samples: deque[VisionSnapshot] = deque(maxlen=32)

    @staticmethod
    def _angle_difference(current: float, previous: float) -> float:
        return (current - previous + math.pi) % (2.0 * math.pi) - math.pi

    def push(self, snapshot: VisionSnapshot) -> tuple[bool, str]:
        """Accept one reliable vision frame or discard it without changing state."""
        if not snapshot.valid or None in (snapshot.x_m, snapshot.y_m, snapshot.yaw_rad):
            return False, snapshot.message
        with self._lock:
            if self._samples:
                previous = self._samples[-1]
                dt = snapshot.captured_monotonic - previous.captured_monotonic
                if dt <= 0.0:
                    return False, "视觉时间戳未递增"
                speed = math.hypot(snapshot.x_m - previous.x_m, snapshot.y_m - previous.y_m) / dt
                yaw_rate = abs(self._angle_difference(snapshot.yaw_rad, previous.yaw_rad)) / dt
                if speed > MAX_POSITION_SPEED_M_S:
                    return False, f"位置突变 {speed:.2f} m/s，已丢弃"
                if yaw_rate > MAX_YAW_RATE_RAD_S:
                    return False, f"航向突变 {yaw_rate:.2f} rad/s，已丢弃"
            self._samples.append(snapshot)
        return True, ""

    def bounds(self) -> tuple[float | None, float | None]:
        with self._lock:
            if not self._samples:
                return None, None
            return self._samples[0].captured_monotonic, self._samples[-1].captured_monotonic

    def latest_kinematics(self, now: float | None = None) -> tuple[VisionSnapshot, float, float, float] | None:
        """Return only fresh, accepted visual data plus a derivative from accepted samples.

        This is deliberately separate from CSV interpolation: real-time control must
        never extrapolate past the latest camera observation, while CSV can wait for
        a future bracketing frame.
        """
        with self._lock:
            samples = tuple(self._samples)
        if not samples:
            return None
        latest = samples[-1]
        if not latest.is_fresh(now):
            return None
        if len(samples) < 2:
            return latest, 0.0, 0.0, 0.0
        previous = samples[-2]
        duration = latest.captured_monotonic - previous.captured_monotonic
        if duration <= 0.0 or duration > MAX_INTERPOLATION_GAP_SECONDS:
            return latest, 0.0, 0.0, 0.0
        x_velocity = (latest.x_m - previous.x_m) / duration
        y_velocity = (latest.y_m - previous.y_m) / duration
        yaw_rate = self._angle_difference(latest.yaw_rad, previous.yaw_rad) / duration
        return latest, x_velocity, y_velocity, yaw_rate

    def interpolate(self, timestamp: float) -> VisionSnapshot | None:
        with self._lock:
            samples = tuple(self._samples)
        for before, after in zip(samples, samples[1:]):
            if before.captured_monotonic <= timestamp <= after.captured_monotonic:
                duration = after.captured_monotonic - before.captured_monotonic
                if duration <= 0.0 or duration > MAX_INTERPOLATION_GAP_SECONDS:
                    return None
                ratio = (timestamp - before.captured_monotonic) / duration
                yaw = before.yaw_rad + ratio * self._angle_difference(after.yaw_rad, before.yaw_rad)
                yaw = (yaw + math.pi) % (2.0 * math.pi) - math.pi
                return VisionSnapshot(
                    True,
                    before.x_m + ratio * (after.x_m - before.x_m),
                    before.y_m + ratio * (after.y_m - before.y_m),
                    yaw,
                    after.tag_ids,
                    max(before.reprojection_error_px or 0.0, after.reprojection_error_px or 0.0),
                    timestamp,
                    after.captured_utc,
                    "可靠视觉帧线性插值",
                )
        return None


def load_tag_layout(path: Path) -> TagLayout:
    raw = json.loads(path.read_text(encoding="utf-8"))
    marker_length_m = float(raw["marker_length_m"])
    if marker_length_m <= 0.0:
        raise ValueError("marker_length_m must be positive")
    tags = tuple(
        TagPose(
            tag_id=int(item["id"]),
            center_x_m=float(item["center_m"][0]),
            center_y_m=float(item["center_m"][1]),
            yaw_rad=math.radians(float(item.get("yaw_deg", 0.0))),
        )
        for item in raw["tags"]
    )
    if {tag.tag_id for tag in tags} != {10, 11, 12, 13} or len(tags) != 4:
        raise ValueError("the layout must contain IDs 10, 11, 12 and 13 exactly once")
    return TagLayout(marker_length_m=marker_length_m, tags=tags)


def load_camera_calibration(path: Path) -> CameraCalibration:
    import yaml

    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    matrix = tuple(float(value) for value in raw["camera_matrix"]["data"])
    distortion = tuple(float(value) for value in raw["distortion_coefficients"]["data"])
    if len(matrix) != 9 or not distortion:
        raise ValueError("camera calibration matrix or distortion coefficients are invalid")
    return CameraCalibration(
        width=int(raw["image_width"]),
        height=int(raw["image_height"]),
        matrix=matrix,
        distortion=distortion,
    )


class AprilTagLocalizer:
    """Detect configured tags and solve the ROV body pose in camera coordinates."""

    def __init__(self, calibration: CameraCalibration, layout: TagLayout) -> None:
        import cv2
        import numpy as np

        if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, "DICT_APRILTAG_25h9"):
            raise RuntimeError("installed OpenCV lacks cv2.aruco.DICT_APRILTAG_25h9; install opencv-contrib-python")
        self.cv2 = cv2
        self.np = np
        self.calibration = calibration
        self.layout = layout
        self._tag_by_id = layout.by_id()
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_25h9)
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        self._camera_matrix = np.asarray(calibration.matrix, dtype=np.float64).reshape(3, 3)
        self._distortion = np.asarray(calibration.distortion, dtype=np.float64).reshape(-1, 1)

    def _object_corners_with_uniform_yaw(self, tag_id: int, yaw_offset_rad: float) -> tuple[tuple[float, float], ...]:
        tag = self._tag_by_id[tag_id]
        half = self.layout.marker_length_m / 2.0
        local_corners = ((half, half), (half, -half), (-half, -half), (-half, half))
        yaw = tag.yaw_rad + yaw_offset_rad
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        return tuple(
            (tag.center_x_m + cosine * x - sine * y, tag.center_y_m + sine * x + cosine * y)
            for x, y in local_corners
        )

    def _solve_four_tag_centers(
        self,
        centers_by_id: dict[int, Any],
        corners_by_id: dict[int, Any],
    ) -> tuple[bool, Any, Any, float, str]:
        """Recover pose from the four known tag centres, independent of print yaw.

        A coplanar PnP based on all marker *corners* requires the printed
        orientation of every tag in the body frame.  The physical layout only
        specifies the centres, so using those four correspondences is the
        reliable estimator when all target IDs are visible.
        """
        cv2 = self.cv2
        np = self.np
        tag_ids = (10, 11, 12, 13)
        if any(tag_id not in centers_by_id for tag_id in tag_ids):
            return False, None, None, 0.0, "四 Tag 中心不完整"
        if any(tag_id not in corners_by_id for tag_id in tag_ids):
            return False, None, None, 0.0, "四 Tag 角点不完整"
        body_centers = np.asarray(
            [(self._tag_by_id[tag_id].center_x_m, self._tag_by_id[tag_id].center_y_m) for tag_id in tag_ids],
            dtype=np.float64,
        )
        image_centers = np.asarray([centers_by_id[tag_id] for tag_id in tag_ids], dtype=np.float64)
        undistorted = cv2.undistortPoints(
            image_centers.reshape(-1, 1, 2), self._camera_matrix, self._distortion, P=self._camera_matrix
        ).reshape(-1, 2)
        homography, _ = cv2.findHomography(body_centers, undistorted, method=0)
        if homography is None:
            return False, None, None, 0.0, "findHomography 失败"

        normalized = np.linalg.inv(self._camera_matrix) @ homography
        column_1, column_2, column_3 = normalized[:, 0], normalized[:, 1], normalized[:, 2]
        scale = 2.0 / (np.linalg.norm(column_1) + np.linalg.norm(column_2))
        rotation_approx = np.column_stack((scale * column_1, scale * column_2, np.cross(scale * column_1, scale * column_2)))
        translation = (scale * column_3).reshape(3, 1)
        if translation[2, 0] < 0.0:
            rotation_approx[:, :2] *= -1.0
            translation *= -1.0
        left, _singular, right = np.linalg.svd(rotation_approx)
        rotation = left @ right
        if np.linalg.det(rotation) < 0.0:
            left[:, -1] *= -1.0
            rotation = left @ right
        rvec, _ = cv2.Rodrigues(rotation)
        observed_corners = np.asarray([corner for tag_id in tag_ids for corner in corners_by_id[tag_id]], dtype=np.float64)
        observed_undistorted = cv2.undistortPoints(
            observed_corners.reshape(-1, 1, 2), self._camera_matrix, self._distortion, P=self._camera_matrix
        ).reshape(-1, 2)
        best_error = math.inf
        best_offset_deg = 0
        for offset_deg in (0, 90, 180, 270):
            body_corners = np.asarray(
                [corner for tag_id in tag_ids for corner in self._object_corners_with_uniform_yaw(tag_id, math.radians(offset_deg))],
                dtype=np.float64,
            )
            predicted_corners = cv2.perspectiveTransform(body_corners.reshape(-1, 1, 2), homography).reshape(-1, 2)
            candidate_error = float(np.mean(np.linalg.norm(predicted_corners - observed_undistorted, axis=1)))
            if candidate_error < best_error:
                best_error = candidate_error
                best_offset_deg = offset_deg
        error = best_error
        if error > MAX_CORNER_REPROJECTION_ERROR_PX:
            return (
                False,
                None,
                None,
                error,
                f"四 Tag 角点几何误差 {error:.2f}px（最佳统一朝向 {best_offset_deg}°）超过 {MAX_CORNER_REPROJECTION_ERROR_PX:.1f}px",
            )
        return True, rvec, translation, error, f"四 Tag 中心 Homography（16 角点校验通过；统一朝向 {best_offset_deg}°）"

    def _solve_partial_tags(
        self,
        tag_ids: tuple[int, ...],
        corners_by_id: dict[int, Any],
    ) -> tuple[bool, Any, Any, float, str]:
        """Solve one to three visible tags with all available marker corners."""
        cv2 = self.cv2
        np = self.np
        observed_corners = np.asarray([corner for tag_id in tag_ids for corner in corners_by_id[tag_id]], dtype=np.float64)
        best: tuple[float, Any, Any, int] | None = None
        for offset_deg in (0, 90, 180, 270):
            object_corners = np.asarray(
                [
                    (x, y, 0.0)
                    for tag_id in tag_ids
                    for x, y in self._object_corners_with_uniform_yaw(tag_id, math.radians(offset_deg))
                ],
                dtype=np.float64,
            )
            try:
                solved, rvec, tvec = cv2.solvePnP(
                    object_corners,
                    observed_corners,
                    self._camera_matrix,
                    self._distortion,
                    flags=cv2.SOLVEPNP_ITERATIVE,
                )
            except cv2.error:
                continue
            if not solved:
                continue
            projected, _ = cv2.projectPoints(object_corners, rvec, tvec, self._camera_matrix, self._distortion)
            error = float(np.mean(np.linalg.norm(projected.reshape(-1, 2) - observed_corners, axis=1)))
            if best is None or error < best[0]:
                best = (error, rvec, tvec, offset_deg)
        if best is None:
            return False, None, None, 0.0, f"{len(tag_ids)} Tag PnP 求解失败"
        error, rvec, tvec, offset_deg = best
        if error > MAX_CORNER_REPROJECTION_ERROR_PX:
            return (
                False,
                None,
                None,
                error,
                f"{len(tag_ids)} Tag PnP 角点误差 {error:.2f}px（最佳统一朝向 {offset_deg}°）超过 {MAX_CORNER_REPROJECTION_ERROR_PX:.1f}px",
            )
        return True, rvec, tvec, error, f"{len(tag_ids)} Tag 角点 PnP（统一朝向 {offset_deg}°）"

    def detect(self, image_bgr: Any) -> VisionFrame:
        cv2 = self.cv2
        np = self.np
        capture_monotonic = time.monotonic()
        capture_utc = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
        annotated = image_bgr.copy()
        image_height, image_width = image_bgr.shape[:2]
        if (image_width, image_height) != (self.calibration.width, self.calibration.height):
            snapshot = VisionSnapshot(
                False,
                None,
                None,
                None,
                (),
                None,
                capture_monotonic,
                capture_utc,
                f"图像为 {image_width}×{image_height}，标定要求 {self.calibration.width}×{self.calibration.height}",
            )
            return VisionFrame(snapshot, annotated)

        corners, ids, _rejected = self._detector.detectMarkers(image_bgr)
        if ids is None:
            return VisionFrame(
                VisionSnapshot(False, None, None, None, (), None, capture_monotonic, capture_utc, "未检测到 25H9 目标 Tag"),
                annotated,
            )

        found_ids: list[int] = []
        centers_by_id: dict[int, Any] = {}
        corners_by_id: dict[int, Any] = {}
        for detected_id, detected_corners in zip(ids.flatten().tolist(), corners):
            if detected_id not in self._tag_by_id:
                continue
            points = detected_corners.reshape(4, 2)
            found_ids.append(detected_id)
            polygon = points.astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(annotated, [polygon], True, (0, 220, 0), 2, cv2.LINE_AA)
            center = points.mean(axis=0).astype(int)
            centers_by_id[detected_id] = points.mean(axis=0)
            corners_by_id[detected_id] = points
            cv2.putText(annotated, f"ID {detected_id}", tuple(center), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 0), 2, cv2.LINE_AA)

        if not found_ids:
            return VisionFrame(
                VisionSnapshot(False, None, None, None, (), None, capture_monotonic, capture_utc, "检测到的 Tag 不在目标 ID 10/11/12/13 中"),
                annotated,
            )

        solve_method = ""
        try:
            if len(found_ids) == 4:
                solved, rvec, tvec, reprojection_error, solve_method = self._solve_four_tag_centers(centers_by_id, corners_by_id)
            else:
                solved, rvec, tvec, reprojection_error, solve_method = self._solve_partial_tags(
                    tuple(sorted(found_ids)), corners_by_id
                )
        except Exception as error:
            return VisionFrame(
                VisionSnapshot(False, None, None, None, tuple(sorted(found_ids)), None, capture_monotonic, capture_utc, f"PnP 异常：{error}"),
                annotated,
            )

        if not solved:
            return VisionFrame(
                VisionSnapshot(
                    False,
                    None,
                    None,
                    None,
                    tuple(sorted(found_ids)),
                    reprojection_error if len(found_ids) == 4 else None,
                    capture_monotonic,
                    capture_utc,
                    f"定位几何校验失败：{solve_method or 'solvePnP 未得到有效解'}",
                ),
                annotated,
            )

        rotation, _ = cv2.Rodrigues(rvec)
        forward_in_camera = rotation[:, 0]
        x_m = float(tvec.reshape(3)[0])
        y_m = -float(tvec.reshape(3)[1])
        yaw_rad = math.atan2(-float(forward_in_camera[1]), float(forward_in_camera[0]))
        cv2.drawFrameAxes(annotated, self._camera_matrix, self._distortion, rvec, tvec, self.layout.marker_length_m)
        cv2.putText(
            annotated,
            f"X={x_m:+.3f} m  Y={y_m:+.3f} m  yaw={math.degrees(yaw_rad):+.1f} deg  err={reprojection_error:.2f}px",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (30, 220, 255),
            2,
            cv2.LINE_AA,
        )
        snapshot = VisionSnapshot(
            True,
            x_m,
            y_m,
            yaw_rad,
            tuple(sorted(found_ids)),
            reprojection_error,
            capture_monotonic,
            capture_utc,
            f"定位有效（{solve_method}；相机坐标：+X 画面右、+Y 画面上、yaw 逆时针为正）",
        )
        return VisionFrame(snapshot, annotated)
