"""Generate reproducible ROV paper-demo trajectories (v2)."""

from __future__ import annotations

import argparse
import csv
import math
import random
from collections import Counter
from dataclasses import dataclass
from pathlib import Path


SEED = 20260830
ROOT = Path(__file__).with_name("training_trajectories") / "v2"
HEADER = ("x_m", "y_m", "yaw_rad", "duration_s")
X_MIN, X_MAX = -1.15, 1.0
Y_MIN, Y_MAX = -0.6, 0.6
DURATION_S = 6.0
POINT_COUNT = 21
RANDOM_MIN_STEP_M = 1.5
MAX_YAW_STEP_45_RAD = math.radians(45.0)
MAX_YAW_STEP_90_RAD = math.radians(90.0)
MIN_YAW_STEP_45_RAD = math.radians(45.0)


@dataclass(frozen=True)
class Point:
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class Spec:
    trajectory_id: int
    category: str
    yaw_mode: str
    points: tuple[Point, ...]
    duration_s: float = DURATION_S

    @property
    def filename(self) -> str:
        return f"{self.trajectory_id:03d}_{self.category}_{self.yaw_mode}.csv"


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def xy(x_m: float, y_m: float) -> tuple[float, float]:
    return round(x_m, 4), round(y_m, 4)


def closed_twice(base: tuple[tuple[float, float], ...]) -> tuple[tuple[float, float], ...]:
    if len(base) != 11 or base[0] != base[-1] or base[0] != (0.0, 0.0):
        raise ValueError("demo base path must have 11 points and close at the origin")
    points = base + base[1:]
    if len(points) != POINT_COUNT:
        raise AssertionError("unexpected repeated point count")
    return points


def tangent_yaws(path: tuple[tuple[float, float], ...]) -> tuple[float, ...]:
    values: list[float] = []
    for index, current in enumerate(path[:-1]):
        for candidate in path[index + 1 :]:
            delta_x, delta_y = candidate[0] - current[0], candidate[1] - current[1]
            if math.hypot(delta_x, delta_y) > 1e-6:
                values.append(wrap(math.atan2(delta_y, delta_x)))
                break
        else:
            values.append(values[0] if values else 0.0)
    values.append(values[0])
    return tuple(values)


def yaws_for(path: tuple[tuple[float, float], ...], mode: str, fixed_yaw: float) -> tuple[float, ...]:
    if mode == "fixed":
        return (wrap(fixed_yaw),) * len(path)
    if mode == "tangent":
        return tangent_yaws(path)
    if mode == "segmented":
        pattern = (0.0, 0.75, -0.75, 1.20, -1.20, 0.75, -0.75, 1.20, -1.20, 0.75)
        return tuple(pattern[index % len(pattern)] for index in range(len(path) - 1)) + (0.0,)
    if mode == "center_facing":
        values = [0.0 if math.hypot(x_m, y_m) < 1e-6 else wrap(math.atan2(-y_m, -x_m)) for x_m, y_m in path]
        values[-1] = values[0]
        return tuple(values)
    raise ValueError(f"unsupported yaw mode: {mode}")


def make_spec(trajectory_id: int, category: str, yaw_mode: str, base: tuple[tuple[float, float], ...], fixed_yaw: float = 0.0) -> Spec:
    path = closed_twice(base)
    yaws = yaws_for(path, yaw_mode, fixed_yaw)
    return Spec(trajectory_id, category, yaw_mode, tuple(Point(x_m, y_m, yaw_rad) for (x_m, y_m), yaw_rad in zip(path, yaws)))


def ellipse_base(center_x: float, axis_x: float, axis_y: float) -> tuple[tuple[float, float], ...]:
    ring = tuple(xy(center_x + axis_x * math.cos(index * math.pi / 4.0), axis_y * math.sin(index * math.pi / 4.0)) for index in range(8))
    return ((0.0, 0.0),) + ring + (ring[0], (0.0, 0.0))


def figure_eight_base(scale_x: float, scale_y: float) -> tuple[tuple[float, float], ...]:
    samples = tuple(xy(scale_x * math.sin(index * math.pi / 10.0), scale_y * math.sin(index * math.pi / 5.0)) for index in range(1, 10))
    return ((0.0, 0.0),) + samples + ((0.0, 0.0),)


def rectangle_base(right_x: float, left_x: float, top_y: float, bottom_y: float) -> tuple[tuple[float, float], ...]:
    return (
        (0.0, 0.0), xy(right_x, 0.0), xy(right_x, top_y), xy(0.0, top_y), xy(left_x, top_y),
        xy(left_x, 0.0), xy(left_x, bottom_y), xy(0.0, bottom_y), xy(right_x, bottom_y), xy(right_x, 0.0), (0.0, 0.0),
    )


def cross_base(arm_x: float, arm_y: float) -> tuple[tuple[float, float], ...]:
    return (
        (0.0, 0.0), xy(arm_x, 0.0), (0.0, 0.0), xy(-arm_x, 0.0), (0.0, 0.0),
        xy(0.0, arm_y), (0.0, 0.0), xy(0.0, -arm_y), (0.0, 0.0), xy(arm_x, 0.0), (0.0, 0.0),
    )


def crescent_base() -> tuple[tuple[float, float], ...]:
    return (
        (0.0, 0.0), xy(-0.15, 0.35), xy(-0.50, 0.50), xy(-0.85, 0.35), xy(-1.00, 0.0),
        xy(-0.85, -0.35), xy(-0.55, -0.22), xy(-0.45, 0.0), xy(-0.55, 0.22), xy(-0.15, 0.35), (0.0, 0.0),
    )


def random_base(rng: random.Random, scale_x: float, scale_y: float) -> tuple[tuple[float, float], ...]:
    interior = tuple(xy(rng.uniform(-scale_x, scale_x), rng.uniform(-scale_y, scale_y)) for _ in range(9))
    return ((0.0, 0.0),) + interior + ((0.0, 0.0),)


def random_long_step_base(rng: random.Random) -> tuple[tuple[float, float], ...]:
    """Create a random path with a short origin departure and long later steps.

    The workspace is narrower than 1.5 m from the origin, so only the first
    segment is exempt.  All following points alternate between two separated
    X bands.  Their X separation alone is at least 1.60 m.
    """
    points: list[tuple[float, float]] = [(0.0, 0.0)]
    for index in range(POINT_COUNT - 1):
        if index % 2 == 0:
            x_m = rng.uniform(-1.15, -0.90)
        else:
            x_m = rng.uniform(0.70, 1.00)
        points.append(xy(x_m, rng.uniform(Y_MIN, Y_MAX)))
    return tuple(points)


def random_limited_yaws(rng: random.Random, count: int, max_yaw_step_rad: float) -> tuple[float, ...]:
    """Generate a wrapped yaw random walk bounded by ``max_yaw_step_rad``."""
    values = [rng.uniform(-math.pi, math.pi)]
    for _ in range(1, count):
        values.append(wrap(values[-1] + rng.uniform(-max_yaw_step_rad, max_yaw_step_rad)))
    return tuple(values)


def random_yaws_with_step_range(
    rng: random.Random,
    count: int,
    min_yaw_step_rad: float,
    max_yaw_step_rad: float,
) -> tuple[float, ...]:
    """Generate a wrapped yaw walk whose every step is within a magnitude range."""
    if not 0.0 <= min_yaw_step_rad <= max_yaw_step_rad <= math.pi:
        raise ValueError("invalid yaw step range")
    values = [rng.uniform(-math.pi, math.pi)]
    for _ in range(1, count):
        # One RNG draw preserves the sequence used by later trajectory specs.
        sample = rng.random()
        magnitude = min_yaw_step_rad + (sample % 0.5) * 2.0 * (max_yaw_step_rad - min_yaw_step_rad)
        signed_step = magnitude if sample >= 0.5 else -magnitude
        values.append(wrap(values[-1] + signed_step))
    return tuple(values)


def make_random_long_step_spec(
    trajectory_id: int,
    rng: random.Random,
    max_yaw_step_rad: float,
    min_yaw_step_rad: float = 0.0,
    duration_s: float = DURATION_S,
) -> Spec:
    path = random_long_step_base(rng)
    yaws = (
        random_limited_yaws(rng, len(path), max_yaw_step_rad)
        if min_yaw_step_rad == 0.0
        else random_yaws_with_step_range(rng, len(path), min_yaw_step_rad, max_yaw_step_rad)
    )
    yaw_limit_deg = round(math.degrees(max_yaw_step_rad))
    yaw_mode = f"yaw_limited_{yaw_limit_deg}deg" if min_yaw_step_rad == 0.0 else f"yaw_step_{round(math.degrees(min_yaw_step_rad))}_to_{yaw_limit_deg}deg"
    return Spec(
        trajectory_id,
        "random_long_step",
        yaw_mode,
        tuple(Point(x_m, y_m, yaw_rad) for (x_m, y_m), yaw_rad in zip(path, yaws)),
        duration_s,
    )


def build_specs() -> tuple[Spec, ...]:
    rng = random.Random(SEED)
    specs = (
        make_spec(1, "ellipse", "fixed", ellipse_base(-0.10, 0.75, 0.32), 0.0),
        make_spec(2, "ellipse", "fixed", ellipse_base(-0.10, 0.55, 0.48), math.pi / 2.0),
        make_spec(3, "ellipse", "tangent", ellipse_base(-0.10, 0.85, 0.24)),
        make_spec(4, "figure_eight", "fixed", figure_eight_base(0.75, 0.35), 0.0),
        make_spec(5, "figure_eight", "tangent", figure_eight_base(0.85, 0.28)),
        make_spec(6, "figure_eight", "segmented", figure_eight_base(0.60, 0.45)),
        make_spec(7, "rectangle", "fixed", rectangle_base(0.75, -0.85, 0.45, -0.45), 0.0),
        make_spec(8, "rectangle", "fixed", rectangle_base(0.60, -0.95, 0.35, -0.35), math.pi / 2.0),
        make_spec(9, "rectangle", "segmented", rectangle_base(0.85, -0.75, 0.50, -0.30)),
        make_spec(10, "cross", "fixed", cross_base(0.90, 0.50), 0.0),
        make_spec(11, "cross", "fixed", cross_base(0.70, 0.55), -math.pi / 2.0),
        make_spec(12, "cross", "segmented", cross_base(1.00, 0.40)),
        make_spec(13, "crescent", "fixed", crescent_base(), 0.0),
        make_spec(14, "crescent", "tangent", crescent_base()),
        *(make_random_long_step_spec(trajectory_id, rng, MAX_YAW_STEP_45_RAD) for trajectory_id in range(15, 21)),
        *(make_random_long_step_spec(trajectory_id, rng, MAX_YAW_STEP_90_RAD, MIN_YAW_STEP_45_RAD, 7.5) for trajectory_id in range(21, 25)),
    )
    validate_specs(specs)
    return specs


def validate_specs(specs: tuple[Spec, ...]) -> None:
    if len(specs) != 24:
        raise ValueError("expected exactly 24 demo trajectories")
    expected_categories = {"ellipse": 3, "figure_eight": 3, "rectangle": 3, "cross": 3, "crescent": 2, "random_long_step": 10}
    if Counter(spec.category for spec in specs) != expected_categories:
        raise ValueError("unexpected demo category coverage")
    if sum(spec.yaw_mode == "fixed" for spec in specs) != 8:
        raise ValueError("expected 8 fixed-yaw trajectories")
    for spec in specs:
        if len(spec.points) != POINT_COUNT:
            raise ValueError(f"{spec.filename}: unexpected point count")
        first, last = spec.points[0], spec.points[-1]
        if (first.x_m, first.y_m) != (0.0, 0.0):
            raise ValueError(f"{spec.filename}: path must begin at the origin")
        if spec.trajectory_id <= 14 and (last.x_m, last.y_m) != (0.0, 0.0):
            raise ValueError(f"{spec.filename}: legacy demo path must end at the origin")
        for item in spec.points:
            if not (X_MIN <= item.x_m <= X_MAX and Y_MIN <= item.y_m <= Y_MAX):
                raise ValueError(f"{spec.filename}: point outside workspace")
            if not all(math.isfinite(value) for value in (item.x_m, item.y_m, item.yaw_rad)):
                raise ValueError(f"{spec.filename}: non-finite point")
        if spec.category == "random_long_step":
            max_yaw_step_rad = {
                "yaw_limited_45deg": MAX_YAW_STEP_45_RAD,
                "yaw_limited_90deg": MAX_YAW_STEP_90_RAD,
                "yaw_step_45_to_90deg": MAX_YAW_STEP_90_RAD,
            }.get(spec.yaw_mode)
            if max_yaw_step_rad is None:
                raise ValueError(f"{spec.filename}: unknown yaw limit")
            for previous, current in zip(spec.points[1:], spec.points[2:]):
                if math.hypot(current.x_m - previous.x_m, current.y_m - previous.y_m) < RANDOM_MIN_STEP_M:
                    raise ValueError(f"{spec.filename}: non-initial step shorter than {RANDOM_MIN_STEP_M} m")
            for previous, current in zip(spec.points, spec.points[1:]):
                yaw_step_rad = abs(wrap(current.yaw_rad - previous.yaw_rad))
                if yaw_step_rad > max_yaw_step_rad + 1e-9:
                    raise ValueError(f"{spec.filename}: yaw step exceeds its configured limit")
                if spec.yaw_mode == "yaw_step_45_to_90deg" and yaw_step_rad < MIN_YAW_STEP_45_RAD - 1e-9:
                    raise ValueError(f"{spec.filename}: yaw step is below 45 degrees")


def write_dataset(
    root: Path = ROOT,
    first_trajectory_id: int = 1,
    last_trajectory_id: int | None = None,
) -> tuple[Spec, ...]:
    specs = build_specs()
    if not 1 <= first_trajectory_id <= len(specs):
        raise ValueError(f"first trajectory id must be in [1, {len(specs)}]")
    if last_trajectory_id is None:
        last_trajectory_id = len(specs)
    if not first_trajectory_id <= last_trajectory_id <= len(specs):
        raise ValueError(f"last trajectory id must be in [{first_trajectory_id}, {len(specs)}]")
    root.mkdir(parents=True, exist_ok=True)
    for spec in specs:
        if not first_trajectory_id <= spec.trajectory_id <= last_trajectory_id:
            continue
        with (root / spec.filename).open("w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(HEADER)
            writer.writerows((item.x_m, item.y_m, item.yaw_rad, spec.duration_s) for item in spec.points)
    with (root / "manifest.csv").open("w", newline="", encoding="utf-8") as file:
        fields = ("trajectory_id", "filename", "category", "yaw_mode", "point_count", "duration_s", "expected_total_s", "x_min_m", "x_max_m", "y_min_m", "y_max_m", "yaw_min_rad", "yaw_max_rad")
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        for spec in specs:
            writer.writerow({
                "trajectory_id": f"{spec.trajectory_id:03d}", "filename": spec.filename, "category": spec.category,
                "yaw_mode": spec.yaw_mode, "point_count": len(spec.points), "duration_s": spec.duration_s,
                "expected_total_s": len(spec.points) * spec.duration_s,
                "x_min_m": min(item.x_m for item in spec.points), "x_max_m": max(item.x_m for item in spec.points),
                "y_min_m": min(item.y_m for item in spec.points), "y_max_m": max(item.y_m for item in spec.points),
                "yaw_min_rad": min(item.yaw_rad for item in spec.points), "yaw_max_rad": max(item.yaw_rad for item in spec.points),
            })
    return specs


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate ROV paper-demo trajectories v2")
    parser.add_argument("--check", action="store_true", help="validate the deterministic catalogue without writing files")
    parser.add_argument(
        "--from-id",
        type=int,
        default=1,
        help="write only trajectories at or after this ID; the manifest is always refreshed",
    )
    parser.add_argument("--to-id", type=int, help="last trajectory ID to write, inclusive")
    args = parser.parse_args()
    specs = build_specs() if args.check else write_dataset(first_trajectory_id=args.from_id, last_trajectory_id=args.to_id)
    print(f"validated {len(specs)} trajectories")


if __name__ == "__main__":
    main()
