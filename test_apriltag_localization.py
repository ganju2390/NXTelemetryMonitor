from __future__ import annotations

import unittest
from pathlib import Path

from apriltag_localization import AprilTagLocalizer, VisionInterpolator, VisionSnapshot, load_camera_calibration, load_tag_layout


ROOT = Path(__file__).parent
CALIBRATION = Path(r"D:\fins\tools\finsrov_perception\calibration\rgb_camera.yaml")


class AprilTagDetectorTests(unittest.TestCase):
    def test_generated_25h9_tag_is_detected_and_localised(self) -> None:
        import cv2
        import numpy as np

        localizer = AprilTagLocalizer(load_camera_calibration(CALIBRATION), load_tag_layout(ROOT / "apriltag_layout.json"))
        image = np.full((720, 1280, 3), 255, dtype=np.uint8)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_25h9)
        marker = cv2.aruco.generateImageMarker(dictionary, 10, 180)
        image[270:450, 550:730] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)

        result = localizer.detect(image)

        self.assertTrue(result.snapshot.valid, result.snapshot.message)
        self.assertEqual(result.snapshot.tag_ids, (10,))
        self.assertIsNotNone(result.snapshot.x_m)
        self.assertIsNotNone(result.snapshot.yaw_rad)

    def test_four_tag_center_estimator_validates_consistent_marker_geometry(self) -> None:
        import cv2
        import numpy as np

        localizer = AprilTagLocalizer(load_camera_calibration(CALIBRATION), load_tag_layout(ROOT / "apriltag_layout.json"))
        image = np.full((720, 1280, 3), 255, dtype=np.uint8)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_25h9)
        for tag_id, x, y in ((10, 460, 180), (13, 700, 180), (11, 460, 420), (12, 700, 420)):
            marker = cv2.aruco.generateImageMarker(dictionary, tag_id, 160)
            image[y : y + 160, x : x + 160] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)

        result = localizer.detect(image)

        self.assertTrue(result.snapshot.valid, result.snapshot.message)
        self.assertEqual(result.snapshot.tag_ids, (10, 11, 12, 13))
        self.assertLess(result.snapshot.reprojection_error_px or 99.0, 1.0)
        self.assertIn("Homography", result.snapshot.message)

    def test_two_visible_tags_fall_back_to_corner_pnp(self) -> None:
        import cv2
        import numpy as np

        localizer = AprilTagLocalizer(load_camera_calibration(CALIBRATION), load_tag_layout(ROOT / "apriltag_layout.json"))
        image = np.full((720, 1280, 3), 255, dtype=np.uint8)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_25h9)
        for tag_id, x in ((10, 460), (13, 700)):
            marker = cv2.aruco.generateImageMarker(dictionary, tag_id, 160)
            image[180:340, x : x + 160] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)

        result = localizer.detect(image)

        self.assertTrue(result.snapshot.valid, result.snapshot.message)
        self.assertEqual(result.snapshot.tag_ids, (10, 13))
        self.assertIn("2 Tag", result.snapshot.message)


class VisionInterpolatorTests(unittest.TestCase):
    @staticmethod
    def snapshot(x: float, yaw: float, timestamp: float) -> VisionSnapshot:
        return VisionSnapshot(True, x, 0.0, yaw, (10, 11, 12, 13), 1.0, timestamp, "2026-08-28T00:00:00.000+00:00", "几何有效")

    def test_interpolator_fills_a_200hz_timestamp_and_discards_a_jump(self) -> None:
        interpolator = VisionInterpolator()
        self.assertTrue(interpolator.push(self.snapshot(0.0, 0.0, 1.0))[0])
        self.assertTrue(interpolator.push(self.snapshot(0.02, 0.1, 1.04))[0])
        interpolated = interpolator.interpolate(1.02)
        self.assertIsNotNone(interpolated)
        self.assertAlmostEqual(interpolated.x_m, 0.01)
        self.assertAlmostEqual(interpolated.yaw_rad, 0.05)
        accepted, reason = interpolator.push(self.snapshot(10.0, 2.0, 1.08))
        self.assertFalse(accepted)
        self.assertIn("突变", reason)


if __name__ == "__main__":
    unittest.main()
