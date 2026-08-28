from __future__ import annotations

import csv
import struct
import tempfile
import unittest
from pathlib import Path

from apriltag_localization import VisionInterpolator, VisionSnapshot, load_tag_layout

from nx_telemetry_monitor import (
    CONTROL_FRAME_SIZE,
    CONTROL_PAYLOAD_STRUCT,
    CsvRecorder,
    EXPECTED_TIMESTAMP_STEP,
    PACKET_SIZE,
    PACKET_STRUCT,
    PacketEvent,
    PacketError,
    TimestampValidator,
    TimestampResult,
    build_control_frame,
    dataset_directory_for,
    decode_packet,
    is_valid_control_frame,
    trajectory_directory_for,
)


def make_packet(timestamp: float = 1.0, tail: tuple[int, int, int, int] = (0, 0, 128, 127)) -> bytes:
    return PACKET_STRUCT.pack(
        1.2,
        0.1,
        0.2,
        0.3,
        0.4,
        0.5,
        100,
        200,
        300,
        400,
        500,
        600,
        700,
        800,
        timestamp,
        *tail,
    )


class TrajectoryVersionTests(unittest.TestCase):
    def test_v1_and_v2_map_to_matching_trajectory_and_dataset_directories(self) -> None:
        self.assertEqual("v1", trajectory_directory_for("v1").name)
        self.assertEqual("v2", trajectory_directory_for("v2").name)
        self.assertEqual("v1", dataset_directory_for("v1").name)
        self.assertEqual("v2", dataset_directory_for("v2").name)
        with self.assertRaises(ValueError):
            trajectory_directory_for("v3")


class DecoderTests(unittest.TestCase):
    def test_valid_packet_decodes_all_motor_speeds(self) -> None:
        packet = decode_packet(make_packet(), ("192.168.1.20", 6000))
        self.assertEqual(PACKET_SIZE, 48)
        self.assertEqual(packet.rpms, (100, 200, 300, 400, 500, 600, 700, 800))
        self.assertEqual(packet.source_ip, "192.168.1.20")

    def test_rejects_wrong_length_and_tail(self) -> None:
        with self.assertRaises(PacketError):
            decode_packet(b"\x00" * (PACKET_SIZE - 1), ("127.0.0.1", 1))
        with self.assertRaises(PacketError):
            decode_packet(make_packet(tail=(1, 2, 3, 4)), ("127.0.0.1", 1))


class TimestampTests(unittest.TestCase):
    def test_continuous_packet_and_missing_frame(self) -> None:
        validator = TimestampValidator()
        self.assertTrue(validator.check(1.0).continuous)
        next_result = validator.check(1.0 + EXPECTED_TIMESTAMP_STEP)
        self.assertTrue(next_result.continuous)
        missed_result = validator.check(1.0 + 3 * EXPECTED_TIMESTAMP_STEP)
        self.assertFalse(missed_result.continuous)
        self.assertEqual(missed_result.missing_frames, 1)


class ControlFrameTests(unittest.TestCase):
    def test_control_frame_matches_streamer_layout(self) -> None:
        frame = build_control_frame(yaw_rate=0.25, forward_speed=0.5, left_right_speed=-0.75, start_button=1)
        self.assertEqual(len(frame), CONTROL_FRAME_SIZE)
        self.assertTrue(is_valid_control_frame(frame))

        payload = CONTROL_PAYLOAD_STRUCT.unpack(frame[1:-2])
        self.assertAlmostEqual(payload[0], 0.25)
        self.assertAlmostEqual(payload[1], 0.5)
        self.assertAlmostEqual(payload[2], -0.75)
        self.assertEqual(payload[5], 1)
        self.assertEqual(payload[6:10], (0, 0, 0, 0))
        self.assertEqual(payload[10:], (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

    def test_control_frame_rejects_crc_damage(self) -> None:
        frame = bytearray(build_control_frame())
        frame[8] ^= 0x01
        self.assertFalse(is_valid_control_frame(bytes(frame)))

    def test_path_tracking_fields_keep_the_52_byte_contract(self) -> None:
        frame = build_control_frame(
            yaw_rate=-0.2,
            forward_speed=0.4,
            left_right_speed=-0.1,
            enable_path_tracking=1,
            global_x=1.0,
            global_y=-2.0,
            global_yaw=0.3,
            global_x_velocity=0.1,
            global_y_velocity=-0.2,
            global_yaw_rate=0.05,
        )
        self.assertEqual(52, len(frame))
        self.assertTrue(is_valid_control_frame(frame))
        payload = CONTROL_PAYLOAD_STRUCT.unpack(frame[1:-2])
        self.assertEqual(1, payload[7])
        for actual, expected in zip(payload[10:], (1.0, -2.0, 0.3, 0.1, -0.2, 0.05)):
            self.assertAlmostEqual(expected, actual, places=6)


class CsvRecorderTests(unittest.TestCase):
    def test_recorder_flushes_valid_packet_to_csv(self) -> None:
        packet = decode_packet(make_packet(), ("192.168.1.20", 6000))
        event = PacketEvent(packet, TimestampResult(delta=None, continuous=True, missing_frames=0))
        interpolator = VisionInterpolator()
        for offset, x_value in ((-0.05, 1.24), (0.05, 1.26)):
            accepted, reason = interpolator.push(
                VisionSnapshot(
                    valid=True,
                    x_m=x_value,
                    y_m=-0.5,
                    yaw_rad=0.75,
                    tag_ids=(10, 11),
                    reprojection_error_px=0.4,
                    captured_monotonic=packet.received_monotonic + offset,
                    captured_utc="2026-08-28T00:00:00.000+00:00",
                    message="定位有效",
                )
            )
            self.assertTrue(accepted, reason)
        recorder = CsvRecorder(interpolator)

        with tempfile.TemporaryDirectory() as temporary_directory:
            output_path = Path(temporary_directory) / "capture.csv"
            recorder.start(output_path)
            recorder.record(event)
            recorder.stop()

            with output_path.open(encoding="utf-8-sig", newline="") as csv_file:
                rows = list(csv.DictReader(csv_file))

        self.assertEqual(recorder.records_written, 1)
        self.assertEqual(rows[0]["int_rpm_8"], "800")
        self.assertEqual(rows[0]["firmware_timestamp"], "1.0")
        self.assertEqual(rows[0]["tag_x_m"], "1.25")
        self.assertEqual(rows[0]["tag_y_m"], "-0.5")
        self.assertEqual(rows[0]["tag_yaw_rad"], "0.75")
        self.assertEqual(rows[0]["tag_timestamp_utc"], rows[0]["received_utc"])
        self.assertNotIn("source_ip", rows[0])

    def test_recorder_accepts_reliable_slow_camera_interval(self) -> None:
        packet = decode_packet(make_packet(), ("192.168.1.20", 6000))
        event = PacketEvent(packet, TimestampResult(delta=None, continuous=True, missing_frames=0))
        interpolator = VisionInterpolator()
        for offset, x_value in ((-0.20, 2.0), (0.20, 2.2)):
            accepted, reason = interpolator.push(
                VisionSnapshot(
                    valid=True,
                    x_m=x_value,
                    y_m=0.0,
                    yaw_rad=0.0,
                    tag_ids=(10,),
                    reprojection_error_px=0.5,
                    captured_monotonic=packet.received_monotonic + offset,
                    captured_utc="2026-08-28T00:00:00.000+00:00",
                    message="定位有效",
                )
            )
            self.assertTrue(accepted, reason)
        recorder = CsvRecorder(interpolator)
        with tempfile.TemporaryDirectory() as temporary_directory:
            output_path = Path(temporary_directory) / "capture.csv"
            recorder.start(output_path)
            recorder.record(event)
            recorder.stop()
            with output_path.open(encoding="utf-8-sig", newline="") as csv_file:
                rows = list(csv.DictReader(csv_file))
        self.assertEqual(1, recorder.records_written)
        self.assertEqual("2.1", rows[0]["tag_x_m"])


class AprilTagLayoutTests(unittest.TestCase):
    def test_default_layout_matches_the_configured_square(self) -> None:
        layout = load_tag_layout(Path(__file__).with_name("apriltag_layout.json"))
        self.assertAlmostEqual(layout.marker_length_m, 0.093)
        self.assertEqual(set(layout.by_id()), {10, 11, 12, 13})
        top_left = layout.by_id()[10]
        bottom_right = layout.by_id()[12]
        self.assertAlmostEqual(top_left.center_x_m - bottom_right.center_x_m, 0.140)
        self.assertAlmostEqual(top_left.center_y_m - bottom_right.center_y_m, 0.140)
        corners = layout.object_corners(10)
        self.assertAlmostEqual(corners[0][0] - corners[2][0], 0.093)
        self.assertAlmostEqual(corners[0][1] - corners[2][1], 0.093)


if __name__ == "__main__":
    unittest.main()
