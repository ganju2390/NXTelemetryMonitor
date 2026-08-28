"""Generate the deterministic ROV planar-dynamics training trajectory set v1."""

from __future__ import annotations

import argparse
import csv
import math
import random
from dataclasses import dataclass
from pathlib import Path


SEED = 20260828
ROOT = Path(__file__).with_name("training_trajectories") / "v1"
HEADER = ("x_m", "y_m", "yaw_rad", "duration_s")
X_MIN, X_MAX = -1.15, 1.0
Y_MIN, Y_MAX = -0.6, 0.6
# The already-collected X/Y families retain their 16-trajectory schedule.
DURATIONS = (6.0,) * 5 + (8.0,) * 3 + (4.0,) * 8
# The remaining three compact families have six trajectories each.  A 4 s
# trajectory is expanded to 35 points; the other five use 16 points.  This
# deterministic shuffled schedule totals 724 s (12.07 min) per family.
COMPACT_SCHEDULE = ((4.0, 35), (6.0, 16), (7.0, 16), (7.5, 16), (8.0, 16), (8.0, 16))
COMPACT_POINT_COUNT = 16


@dataclass(frozen=True)
class Point:
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class Spec:
    trajectory_id: int
    family: str
    primary_dof: str
    coupling: str
    duration_s: float
    points: tuple[Point, ...]

    @property
    def filename(self) -> str:
        return f"{self.trajectory_id:03d}_{self.family}.csv"


def point(x_m: float, y_m: float, yaw_rad: float = 0.0) -> Point:
    return Point(round(x_m, 4), round(y_m, 4), round(yaw_rad, 6))


def durations() -> tuple[float, ...]:
    return DURATIONS


def compact_schedule(rng: random.Random) -> tuple[tuple[float, int], ...]:
    values = list(COMPACT_SCHEDULE)
    rng.shuffle(values)
    return tuple(values)


def repeat_excitation_key_points(key_points: tuple[Point, ...], target_count: int = COMPACT_POINT_COUNT) -> tuple[Point, ...]:
    """Repeat large-amplitude excitation key points instead of linearly subdividing them."""
    if len(key_points) > target_count:
        raise ValueError("target point count is smaller than key-point count")
    if len(key_points) < 3 or key_points[0] != key_points[-1]:
        raise ValueError("excitation path must begin and end at the same point")
    excitation_points = key_points[1:-1]
    result = [key_points[0]]
    while len(result) < target_count - 1:
        for candidate in excitation_points:
            if len(result) >= target_count - 1:
                break
            if candidate != result[-1]:
                result.append(candidate)
    if len(result) != target_count:
        result.append(key_points[-1])
    if len(result) != target_count:
        raise AssertionError("unexpected repeated point count")
    return tuple(result)


def x_primary_specs(rng: random.Random, start_id: int) -> list[Spec]:
    amplitudes = (-1.10, -0.95, -0.85, -0.65, -0.45, 0.45, 0.65, 0.85, 0.95, -0.75, 0.75, -0.55, 0.55, -1.00, 0.90, -0.70)
    result: list[Spec] = []
    for index, (amplitude, duration) in enumerate(zip(amplitudes, durations())):
        coupled = index >= 12
        y_offset = rng.choice((-0.10, -0.08, 0.08, 0.10)) if coupled else 0.0
        yaw_offset = rng.choice((-0.25, -0.18, 0.18, 0.25)) if coupled else 0.0
        coupling = "small_y_yaw" if coupled else "x_only"
        points = (
            point(0.0, 0.0),
            point(amplitude, y_offset, yaw_offset),
            point(0.42 * amplitude, -y_offset, -yaw_offset),
            point(-0.35 * amplitude, y_offset, yaw_offset),
            point(0.0, 0.0),
        )
        result.append(Spec(start_id + index, "x_primary", "x", coupling, duration, points))
    return result


def y_primary_specs(rng: random.Random, start_id: int) -> list[Spec]:
    # With the ROV yawed +90 deg, a global-X command is a body-Y (left/right)
    # excitation.  Use a longer X excursion than the former global-Y tests.
    amplitudes = (-1.10, -1.00, -0.90, -0.75, -0.60, 0.60, 0.75, 0.90, 0.98, -0.85, 0.85, -0.65, -1.05, 0.95, -0.70, 0.70)
    heading = math.pi / 2.0
    result: list[Spec] = []
    for index, (amplitude, duration) in enumerate(zip(amplitudes, durations())):
        coupled = index >= 12
        y_offset = rng.choice((-0.10, -0.08, 0.08, 0.10)) if coupled else 0.0
        yaw_offset = rng.choice((-0.25, -0.18, 0.18, 0.25)) if coupled else 0.0
        coupling = "small_global_y_yaw" if coupled else "body_y_via_yaw_90"
        points = (
            point(0.0, 0.0, heading),
            point(amplitude, y_offset, heading + yaw_offset),
            point(0.42 * amplitude, -y_offset, heading - yaw_offset),
            point(-0.35 * amplitude, y_offset, heading + yaw_offset),
            point(0.0, 0.0, heading),
        )
        result.append(Spec(start_id + index, "y_primary", "y", coupling, duration, points))
    return result


def yaw_primary_specs(rng: random.Random, start_id: int) -> list[Spec]:
    amplitudes = (-3 * math.pi / 4, -math.pi / 2, -math.pi / 4, math.pi / 4, math.pi / 2, 3 * math.pi / 4)
    schedule = compact_schedule(rng)
    result: list[Spec] = []
    for index, (amplitude, (duration, point_count)) in enumerate(zip(amplitudes, schedule)):
        coupled = index >= 4
        x_offset = rng.choice((-0.15, -0.10, 0.10, 0.15)) if coupled else 0.0
        y_offset = rng.choice((-0.10, -0.08, 0.08, 0.10)) if coupled else 0.0
        coupling = "small_xy" if coupled else "yaw_only"
        points = repeat_excitation_key_points((
            point(0.0, 0.0),
            point(x_offset, y_offset, amplitude),
            point(-x_offset, -y_offset, -amplitude),
            point(x_offset, -y_offset, amplitude),
            point(0.0, 0.0),
        ), point_count)
        result.append(Spec(start_id + index, "yaw_primary", "yaw", coupling, duration, points))
    return result


def xy_coupled_specs(rng: random.Random, start_id: int) -> list[Spec]:
    x_values = (-1.05, -0.85, -0.65, -0.45, 0.45, 0.65)
    y_values = (-0.50, -0.40, -0.30, 0.30, 0.40, 0.50)
    schedule = compact_schedule(rng)
    result: list[Spec] = []
    for index, (x_value, y_value, (duration, point_count)) in enumerate(zip(x_values, y_values, schedule)):
        yaw = 0.0 if index < 4 else rng.choice((-0.20, -0.15, 0.15, 0.20))
        if index % 4 == 0:  # diagonal / bow shape
            points = (point(0, 0), point(x_value, y_value, yaw), point(-0.45 * x_value, -0.45 * y_value, -yaw), point(0.55 * x_value, -y_value, yaw), point(0, 0))
            coupling = "xy_diagonal"
        elif index % 4 == 1:  # rectangle
            points = (point(0, 0), point(x_value, 0, yaw), point(x_value, y_value, yaw), point(0, y_value, -yaw), point(-0.45 * x_value, -0.45 * y_value, 0), point(0, 0))
            coupling = "xy_rectangle"
        elif index % 4 == 2:  # S curve
            points = (point(0, 0), point(x_value, 0.5 * y_value, yaw), point(-0.35 * x_value, y_value, -yaw), point(0.55 * x_value, -y_value, yaw), point(-0.5 * x_value, -0.5 * y_value, -yaw), point(0, 0))
            coupling = "xy_s_curve"
        else:  # figure eight
            points = (point(0, 0), point(0.5 * x_value, 0.7 * y_value, yaw), point(x_value, 0, -yaw), point(0.5 * x_value, -0.7 * y_value, yaw), point(0, 0), point(-0.5 * x_value, 0.7 * y_value, -yaw), point(-x_value, 0, yaw), point(-0.5 * x_value, -0.7 * y_value, -yaw), point(0, 0))
            coupling = "xy_figure_eight"
        result.append(Spec(start_id + index, "xy_coupled", "xy", coupling, duration, repeat_excitation_key_points(points, point_count)))
    return result


def xyz_coupled_specs(rng: random.Random, start_id: int) -> list[Spec]:
    x_values = (-1.05, -0.95, -0.85, -0.65, -0.45, 0.45)
    y_values = (-0.50, 0.45, -0.40, 0.35, -0.30, 0.30)
    yaw_values = (-3 * math.pi / 4, -math.pi / 2, -math.pi / 4, math.pi / 4, math.pi / 2, 3 * math.pi / 4)
    schedule = compact_schedule(rng)
    result: list[Spec] = []
    for index, (x_value, y_value, yaw, (duration, point_count)) in enumerate(zip(x_values, y_values, yaw_values, schedule)):
        points = repeat_excitation_key_points((
            point(0, 0),
            point(0.70 * x_value, 0.40 * y_value, yaw),
            point(x_value, y_value, -0.50 * yaw),
            point(-0.50 * x_value, 0.80 * y_value, 0.75 * yaw),
            point(-0.80 * x_value, -0.60 * y_value, -0.75 * yaw),
            point(0.40 * x_value, -y_value, 0.40 * yaw),
            point(0, 0),
        ), point_count)
        result.append(Spec(start_id + index, "xyz_coupled", "xy_yaw", "full_planar", duration, points))
    return result


def build_specs() -> tuple[Spec, ...]:
    rng = random.Random(SEED)
    specs = (
        x_primary_specs(rng, 1)
        + y_primary_specs(rng, 17)
        + yaw_primary_specs(rng, 33)
        + xy_coupled_specs(rng, 39)
        + xyz_coupled_specs(rng, 45)
    )
    validate_specs(specs)
    return tuple(specs)


def validate_specs(specs: tuple[Spec, ...]) -> None:
    if len(specs) != 50:
        raise ValueError(f"expected 50 trajectories, got {len(specs)}")
    expected_families = {"x_primary": 16, "y_primary": 16, "yaw_primary": 6, "xy_coupled": 6, "xyz_coupled": 6}
    for family, expected_count in expected_families.items():
        if sum(spec.family == family for spec in specs) != expected_count:
            raise ValueError(f"{family} count is not {expected_count}")
    compact_families = {"yaw_primary", "xy_coupled", "xyz_coupled"}
    for spec in specs:
        if spec.family in compact_families:
            valid_point_count = len(spec.points) in {16, 35}
        else:
            valid_point_count = 4 <= len(spec.points) <= 16
        if not valid_point_count:
            raise ValueError(f"{spec.filename}: invalid point count")
        if not 3.0 <= spec.duration_s <= 8.0:
            raise ValueError(f"{spec.filename}: invalid duration")
        first = spec.points[0]
        if first.x_m != 0.0 or first.y_m != 0.0:
            raise ValueError(f"{spec.filename}: first point is not origin")
        for value in spec.points:
            if not (X_MIN <= value.x_m <= X_MAX and Y_MIN <= value.y_m <= Y_MAX):
                raise ValueError(f"{spec.filename}: point outside workspace")
            if not all(math.isfinite(number) for number in (value.x_m, value.y_m, value.yaw_rad)):
                raise ValueError(f"{spec.filename}: non-finite point")
    for family in compact_families:
        family_specs = [spec for spec in specs if spec.family == family]
        actual_schedule = sorted((spec.duration_s, len(spec.points)) for spec in family_specs)
        if actual_schedule != sorted(COMPACT_SCHEDULE):
            raise ValueError(f"{family}: compact duration/point schedule mismatch")
        family_total_seconds = sum(len(spec.points) * spec.duration_s for spec in family_specs)
        if family_total_seconds <= 12.0 * 60.0:
            raise ValueError(f"{family}: collection duration must exceed 12 min")


def write_dataset(root: Path = ROOT) -> tuple[Spec, ...]:
    specs = build_specs()
    root.mkdir(parents=True, exist_ok=True)
    expected_filenames = {spec.filename for spec in specs}
    generated_families = {"x_primary", "y_primary", "yaw_primary", "xy_coupled", "xyz_coupled"}
    for existing in root.glob("[0-9][0-9][0-9]_*.csv"):
        family = existing.stem.split("_", 1)[-1]
        if family in generated_families and existing.name not in expected_filenames:
            existing.unlink()
    for spec in specs:
        with (root / spec.filename).open("w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(HEADER)
            writer.writerows((item.x_m, item.y_m, item.yaw_rad, spec.duration_s) for item in spec.points)
    with (root / "manifest.csv").open("w", newline="", encoding="utf-8") as file:
        fields = ("trajectory_id", "filename", "family", "primary_dof", "coupling", "point_count", "duration_s", "expected_total_s", "x_min_m", "x_max_m", "y_min_m", "y_max_m", "yaw_min_rad", "yaw_max_rad")
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        for spec in specs:
            writer.writerow({
                "trajectory_id": f"{spec.trajectory_id:03d}",
                "filename": spec.filename,
                "family": spec.family,
                "primary_dof": spec.primary_dof,
                "coupling": spec.coupling,
                "point_count": len(spec.points),
                "duration_s": spec.duration_s,
                "expected_total_s": len(spec.points) * spec.duration_s,
                "x_min_m": min(item.x_m for item in spec.points),
                "x_max_m": max(item.x_m for item in spec.points),
                "y_min_m": min(item.y_m for item in spec.points),
                "y_max_m": max(item.y_m for item in spec.points),
                "yaw_min_rad": min(item.yaw_rad for item in spec.points),
                "yaw_max_rad": max(item.yaw_rad for item in spec.points),
            })
    return specs


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate ROV planar dynamics trajectories v1")
    parser.add_argument("--check", action="store_true", help="validate the deterministic catalogue without writing files")
    args = parser.parse_args()
    specs = build_specs() if args.check else write_dataset()
    total_seconds = sum(len(spec.points) * spec.duration_s for spec in specs)
    print(f"validated {len(specs)} trajectories; planned collection time {total_seconds / 60.0:.1f} min")


if __name__ == "__main__":
    main()
