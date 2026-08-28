from __future__ import annotations

import csv
import math
import unittest
from collections import Counter

from generate_demo_trajectories_v2 import (
    DURATION_S,
    MAX_YAW_STEP_45_RAD,
    MAX_YAW_STEP_90_RAD,
    MIN_YAW_STEP_45_RAD,
    POINT_COUNT,
    RANDOM_MIN_STEP_M,
    ROOT,
    build_specs,
    validate_specs,
    wrap,
)
from trajectory_tracking import load_waypoints


class DemoTrajectoryV2Tests(unittest.TestCase):
    def test_catalog_is_balanced_safe_and_two_minutes_long(self) -> None:
        specs = build_specs()
        validate_specs(specs)
        self.assertEqual(24, len(specs))
        self.assertEqual(
            {"ellipse": 3, "figure_eight": 3, "rectangle": 3, "cross": 3, "crescent": 2, "random_long_step": 10},
            Counter(spec.category for spec in specs),
        )
        self.assertEqual(8, sum(spec.yaw_mode == "fixed" for spec in specs))
        self.assertTrue(all(len(spec.points) == POINT_COUNT for spec in specs))
        self.assertGreater(POINT_COUNT * DURATION_S, 120.0)

    def test_written_csvs_use_the_existing_loader_contract(self) -> None:
        files = sorted(ROOT.glob("[0-9][0-9][0-9]_*.csv"))
        self.assertEqual(24, len(files))
        for path in files:
            waypoints = load_waypoints(path)
            self.assertGreaterEqual(len(waypoints), 4)
            self.assertEqual((0.0, 0.0), (waypoints[0].x_m, waypoints[0].y_m))
            self.assertTrue(all(waypoint.duration_s > 0.0 for waypoint in waypoints))
            if path.name.startswith(("015_", "016_", "017_", "018_", "019_", "020_", "021_", "022_", "023_", "024_")):
                self.assertEqual(POINT_COUNT, len(waypoints))
                for previous, current in zip(waypoints[1:], waypoints[2:]):
                    self.assertGreaterEqual(
                        math.hypot(current.x_m - previous.x_m, current.y_m - previous.y_m),
                        RANDOM_MIN_STEP_M,
                    )
                max_yaw_step_rad = MAX_YAW_STEP_45_RAD if path.name.startswith(("015_", "016_", "017_", "018_", "019_", "020_")) else MAX_YAW_STEP_90_RAD
                for previous, current in zip(waypoints, waypoints[1:]):
                    yaw_step_rad = abs(wrap(current.yaw_rad - previous.yaw_rad))
                    self.assertLessEqual(yaw_step_rad, max_yaw_step_rad)
                    if path.name.startswith(("021_", "022_", "023_", "024_")):
                        self.assertGreaterEqual(yaw_step_rad, MIN_YAW_STEP_45_RAD)
                expected_duration_s = 7.5 if path.name.startswith(("021_", "022_", "023_", "024_")) else DURATION_S
                self.assertTrue(all(waypoint.duration_s == expected_duration_s for waypoint in waypoints))
            else:
                self.assertEqual((0.0, 0.0), (waypoints[-1].x_m, waypoints[-1].y_m))
        with (ROOT / "manifest.csv").open(encoding="utf-8", newline="") as file:
            self.assertEqual(24, len(list(csv.DictReader(file))))


if __name__ == "__main__":
    unittest.main()
