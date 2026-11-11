import math
import unittest
from groundstation.models import Axes, InputSnapshot, Odometry
from groundstation.modes import ManualMode, TrajectoryMode, transform_waypoints
from trajectory_tracking import PoseEstimate, Waypoint, TrackingSettings


class TrackingModeTests(unittest.TestCase):
    def test_delayed_source_expires_before_receive_timeout(self):
        mode = TrajectoryMode()
        odom = Odometry(PoseEstimate(0, 0, 0), 0.5, True, 1, 820000, 1, 0, True, 1.02)
        mode.enter(
            1, odom, (Waypoint(0, 0, 0, 1), Waypoint(1, 0, 0, 1)), TrackingSettings()
        )
        with self.assertRaises(ValueError):
            mode.update(1.04, odom, None)

    def test_all_existing_csv_files_remain_loadable(self):
        from pathlib import Path
        from trajectory_tracking import load_waypoints

        files = [
            p
            for p in (
                Path(__file__).resolve().parents[1] / "training_trajectories"
            ).rglob("*.csv")
            if p.name != "manifest.csv"
        ]
        self.assertGreaterEqual(len(files), 86)
        for path in files:
            with self.subTest(path=path):
                self.assertGreaterEqual(len(load_waypoints(path)), 2)

    def test_relative_rotation_and_yaw_wrap(self):
        points = transform_waypoints(
            (Waypoint(1, 0, 0.2, 2),), PoseEstimate(4, 5, math.pi - 0.1)
        )
        self.assertAlmostEqual(4 - math.cos(0.1), points[0].x_m)
        self.assertAlmostEqual(5 + math.sin(0.1), points[0].y_m)
        self.assertAlmostEqual(-math.pi + 0.1, points[0].yaw_rad)

    def test_world_velocity_mapping_to_body(self):
        mode = TrajectoryMode()
        pose = PoseEstimate(3, 4, math.pi / 2)
        odom = Odometry(pose, 0.7, True, 1, 1000000, 1, 0)
        mode.enter(
            1, odom, (Waypoint(0, 0, 0, 1), Waypoint(1, 0, 0, 2)), TrackingSettings()
        )
        mode.update(1, odom, None)
        out = mode.update(1.1, odom, None)
        self.assertGreater(out.axes.forward, 0)
        self.assertAlmostEqual(0, out.axes.left, places=6)
        self.assertLessEqual(abs(out.axes.forward), 0.1 + 1e-8)
        self.assertTrue(out.hold_depth)
        self.assertEqual(0.7, out.depth_target)

    def test_completion_does_not_restart_or_use_manual_input(self):
        mode = TrajectoryMode()
        pose = PoseEstimate(1, 2, 0)
        odom = Odometry(pose, 0.5, True, 1, 1, 1, 0)
        mode.enter(
            1, odom, (Waypoint(0, 0, 0, 1), Waypoint(0.1, 0, 0, 1)), TrackingSettings()
        )
        for now in (1, 1.1, 1.15):
            output = mode.update(now, odom, InputSnapshot(now, True, Axes(forward=1)))
        self.assertTrue(output.complete)
        self.assertEqual(Axes(), output.axes)

    def test_takeover_requires_neutral_and_current_input(self):
        for input_state in (
            InputSnapshot(1, True, Axes(forward=0.5)),
            InputSnapshot(0, True),
        ):
            with self.assertRaises(ValueError):
                ManualMode().enter(1, input_state)


if __name__ == "__main__":
    unittest.main()
