from __future__ import annotations

import csv
import math
import unittest

from generate_training_trajectories import ROOT, build_specs, validate_specs
from trajectory_tracking import load_waypoints


class TrainingTrajectoryDatasetTests(unittest.TestCase):
    def test_catalog_has_balanced_valid_coverage(self) -> None:
        specs = build_specs()
        validate_specs(specs)
        self.assertEqual(50, len(specs))
        expected_counts = {"x_primary": 16, "y_primary": 16, "yaw_primary": 6, "xy_coupled": 6, "xyz_coupled": 6}
        for family, expected_count in expected_counts.items():
            self.assertEqual(expected_count, sum(spec.family == family for spec in specs))
        for spec in specs:
            self.assertGreaterEqual(spec.duration_s, 3.0)
            self.assertLessEqual(spec.duration_s, 8.0)
        for family in ("x_primary", "y_primary"):
            family_specs = [spec for spec in specs if spec.family == family]
            self.assertEqual([4.0] * 8, [spec.duration_s for spec in family_specs[-8:]])
            self.assertTrue(all(4 <= len(spec.points) <= 16 for spec in family_specs))
        for family in ("yaw_primary", "xy_coupled", "xyz_coupled"):
            family_specs = [spec for spec in specs if spec.family == family]
            self.assertEqual(sorted([(4.0, 35), (6.0, 16), (7.0, 16), (7.5, 16), (8.0, 16), (8.0, 16)]), sorted((spec.duration_s, len(spec.points)) for spec in family_specs))
            self.assertGreater(len({spec.duration_s for spec in family_specs}), 1)
            self.assertTrue(all(3.0 <= spec.duration_s <= 8.0 for spec in family_specs))
            self.assertGreater(sum(len(spec.points) * spec.duration_s for spec in family_specs), 12.0 * 60.0)
        yaw_primary_specs = [spec for spec in specs if spec.family == "yaw_primary"]
        self.assertTrue(all(max(abs(a.yaw_rad - b.yaw_rad) for a, b in zip(spec.points, spec.points[1:])) >= math.pi / 2.0 - 1e-6 for spec in yaw_primary_specs))
        y_primary_specs = [spec for spec in specs if spec.family == "y_primary"]
        self.assertTrue(all(math.isclose(spec.points[0].yaw_rad, math.pi / 2.0, abs_tol=1e-6) for spec in y_primary_specs))
        self.assertGreaterEqual(max(abs(point.x_m) for spec in y_primary_specs for point in spec.points), 1.0)

    def test_written_csvs_match_the_catalog_contract(self) -> None:
        files = sorted(ROOT.glob("[0-9][0-9][0-9]_*.csv"))
        self.assertEqual(50, len(files))
        for path in files:
            waypoints = load_waypoints(path)
            self.assertGreaterEqual(len(waypoints), 4)
            self.assertLessEqual(len(waypoints), 35)
            self.assertEqual((0.0, 0.0), (waypoints[0].x_m, waypoints[0].y_m))
        with (ROOT / "manifest.csv").open(encoding="utf-8", newline="") as file:
            manifest = list(csv.DictReader(file))
        self.assertEqual(50, len(manifest))
        self.assertEqual({"x_primary", "y_primary", "yaw_primary", "xy_coupled", "xyz_coupled"}, {row["family"] for row in manifest})


if __name__ == "__main__":
    unittest.main()
