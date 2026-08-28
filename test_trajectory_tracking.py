from __future__ import annotations

import tempfile
import unittest
import math
from pathlib import Path

from trajectory_tracking import (
    PoseEstimate,
    TrackingSettings,
    TrajectoryError,
    TrajectoryFollower,
    PlannedTrajectory,
    Waypoint,
    load_waypoints,
    normalize_angle,
)


class WaypointCsvTests(unittest.TestCase):
    def test_requires_origin_first_row(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.csv"
            path.write_text("x_m,y_m,yaw_rad,duration_s\n0.1,0,0,1\n1,0,0,1\n", encoding="utf-8")
            with self.assertRaises(TrajectoryError):
                load_waypoints(path)

    def test_loads_strict_csv_contract(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "valid.csv"
            path.write_text("x_m,y_m,yaw_rad,duration_s\n0,0,0,2\n1,0,1.5707963,3\n", encoding="utf-8")
            self.assertEqual(2, len(load_waypoints(path)))


class TrajectoryFollowerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.settings = TrackingSettings()
        self.pose = PoseEstimate(0.2, -0.1, 0.0)
        self.waypoints = (
            Waypoint(0.0, 0.0, 0.0, 3.0),
            Waypoint(1.0, 0.0, 0.0, 8.0),
            Waypoint(1.0, 1.0, 1.5707963, 10.0),
        )

    def test_current_segment_uses_csv_duration_and_ends_at_current_waypoint(self) -> None:
        follower = TrajectoryFollower()
        follower.start(10.0, self.pose, self.waypoints, self.settings)
        trajectory = follower.trajectory
        self.assertIsNotNone(trajectory)
        assert trajectory is not None
        for knot_time, point in zip(trajectory._times, trajectory._points):
            reference = trajectory.sample(knot_time)
            self.assertAlmostEqual(point.x_m, reference.x_m, places=5)
            self.assertAlmostEqual(point.y_m, reference.y_m, places=5)
        self.assertAlmostEqual(3.0, trajectory.duration_s)
        final = trajectory.sample(trajectory.duration_s)
        self.assertAlmostEqual(0.0, final.x_m, places=5)
        self.assertAlmostEqual(0.0, final.y_m, places=5)

    def test_does_not_plan_next_segment_until_arrival_radius(self) -> None:
        follower = TrajectoryFollower()
        follower.start(0.0, self.pose, self.waypoints, self.settings)
        self.assertEqual(0, follower.waypoint_index)
        first_segment = follower.trajectory
        self.assertIsNotNone(first_segment)
        follower.update(0.5, self.pose, self.settings)
        self.assertEqual(0, follower.waypoint_index)
        arrived = PoseEstimate(0.05, 0.02, 0.0)
        follower.update(0.6, arrived, self.settings)
        self.assertEqual(1, follower.waypoint_index)
        self.assertIsNotNone(follower.trajectory)
        assert follower.trajectory is not None
        next_target = follower.trajectory._points[-1]
        self.assertAlmostEqual(1.0, next_target.x_m)
        self.assertAlmostEqual(0.0, next_target.y_m)

    def test_arrival_radius_is_fifteen_centimetres(self) -> None:
        follower = TrajectoryFollower()
        follower.start(0.0, self.pose, self.waypoints, self.settings)
        follower.update(0.5, PoseEstimate(0.14, 0.0, 0.0), self.settings)
        self.assertEqual(1, follower.waypoint_index)

        follower = TrajectoryFollower()
        follower.start(0.0, self.pose, self.waypoints, self.settings)
        follower.update(0.5, PoseEstimate(0.16, 0.0, 0.0), self.settings)
        self.assertEqual(0, follower.waypoint_index)

    def test_does_not_advance_when_position_arrives_but_yaw_is_wrong(self) -> None:
        follower = TrajectoryFollower()
        follower.start(0.0, self.pose, self.waypoints, self.settings)
        position_arrived_with_wrong_yaw = PoseEstimate(0.05, 0.02, math.radians(20.0))
        follower.update(0.5, position_arrived_with_wrong_yaw, self.settings)
        self.assertEqual(0, follower.waypoint_index)
        position_and_yaw_arrived = PoseEstimate(0.05, 0.02, math.radians(5.0))
        follower.update(0.6, position_and_yaw_arrived, self.settings)
        self.assertEqual(1, follower.waypoint_index)

    def test_first_segment_targets_the_first_csv_waypoint(self) -> None:
        follower = TrajectoryFollower()
        follower.start(0.0, self.pose, self.waypoints, self.settings)
        assert follower.trajectory is not None
        target = follower.trajectory._points[-1]
        self.assertEqual(self.waypoints[0], target)

    def test_display_yaw_normalization_range(self) -> None:
        self.assertAlmostEqual(-math.pi, normalize_angle(3.0 * math.pi))
        self.assertGreaterEqual(normalize_angle(-5.0 * math.pi), -math.pi)
        self.assertLessEqual(normalize_angle(-5.0 * math.pi), math.pi)

    def test_spline_has_continuous_velocity_and_acceleration_at_joint(self) -> None:
        trajectory = PlannedTrajectory(self.pose, self.waypoints, self.settings)
        joint = trajectory._times[1]
        before = trajectory._x_spline.sample(joint - 1e-5)
        after = trajectory._x_spline.sample(joint + 1e-5)
        self.assertAlmostEqual(before[1], after[1], places=3)
        self.assertAlmostEqual(before[2], after[2], places=3)

    def test_output_is_bounded_and_slew_limited(self) -> None:
        follower = TrajectoryFollower()
        follower.start(0.0, self.pose, self.waypoints, self.settings)
        first = follower.update(0.02, self.pose, self.settings)
        second = follower.update(0.04, self.pose, self.settings)
        for command in (first, second):
            self.assertLessEqual(abs(command.forward_throttle), 1.0)
            self.assertLessEqual(abs(command.left_throttle), 1.0)
            self.assertLessEqual(abs(command.yaw_rate), 1.0)
        self.assertLessEqual(abs(second.forward_throttle - first.forward_throttle), 0.041)
        self.assertLessEqual(abs(second.left_throttle - first.left_throttle), 0.041)

    def test_complete_trajectory_holds_with_zero_motion(self) -> None:
        follower = TrajectoryFollower()
        follower.start(0.0, self.pose, self.waypoints, self.settings)
        follower.update(0.1, PoseEstimate(0.02, 0.01, 0.0), self.settings)
        follower.update(0.2, PoseEstimate(1.0, 0.02, 0.0), self.settings)
        command = follower.update(0.3, PoseEstimate(1.0, 1.0, 1.5707963), self.settings)
        self.assertTrue(command.reference.complete)
        self.assertEqual((0.0, 0.0, 0.0), (command.yaw_rate, command.forward_throttle, command.left_throttle))


if __name__ == "__main__":
    unittest.main()
