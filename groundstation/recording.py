"""Loss-accounted asynchronous session logger. NX rows survive absent AprilTags."""

import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import queue
import threading
import time

LEGACY_FIELDS = [
    "received_utc",
    "odom_global_x_velocity_m_s",
    "odom_global_y_velocity_m_s",
    "odom_global_yaw_rate_rad_s",
    "odom_global_x_m",
    "odom_global_y_m",
    "odom_global_yaw_rad",
    "network_body_x_velocity_m_s",
    "network_body_y_velocity_m_s",
    "network_body_yaw_rate_rad_s",
    "tag_timestamp_utc",
    "tag_x_m",
    "tag_y_m",
    "tag_yaw_rad",
]
FIELDS = LEGACY_FIELDS + [
    "source_us",
    "received_monotonic",
    "source_pc_monotonic",
    "sample_seq",
    "reset_id",
    "mode",
    "nx_mode",
    "input_source",
    "input_valid",
    "epoch",
    "simulated",
    "armed",
    "odom_valid",
    "odom_age_ms",
    "telemetry_valid_mask",
    "tag_valid",
    "depth_m",
    "ref_x_m",
    "ref_y_m",
    "ref_yaw_rad",
    "ref_vx_m_s",
    "ref_vy_m_s",
    "yaw_command",
    "forward_command",
    "left_command",
    "down_command",
    "up_command",
    "vertical_mode",
    "depth_target_m",
    "completed",
    "fault",
    "dropped_rows",
]


def make_row(controller, now):
    odom = controller.odometry
    if odom is None:
        return None
    pose, out, telem = odom.pose, controller.output, controller.telemetry
    telem_ok = telem is not None and now - controller.telemetry_received <= 0.2
    network_ok = telem_ok and telem.valid_mask & 16
    row = dict(
        zip(
            LEGACY_FIELDS[:10],
            [
                datetime.now(timezone.utc).isoformat(),
                pose.x_velocity_m_s,
                pose.y_velocity_m_s,
                pose.yaw_rate_rad_s,
                pose.x_m,
                pose.y_m,
                pose.yaw_rad,
                *(list(telem.network) if network_ok else ["", "", ""]),
            ],
        )
    )
    row.update(
        source_us=odom.source_us,
        received_monotonic=odom.received,
        source_pc_monotonic=odom.source_us / 1e6 - (controller.offset or 0),
        sample_seq=odom.sample_seq,
        reset_id=odom.reset_id,
        mode=controller.mode.name,
        nx_mode=int(controller.actual_mode),
        input_source=controller.input.source,
        input_valid=int(
            controller.input.connected and now - controller.input.received <= 0.2
        ),
        epoch=controller.epoch,
        simulated=int(controller.simulated),
        armed=int(controller.armed),
        odom_valid=int(odom.valid),
        odom_age_ms=max(
            0, (now + (controller.offset or 0) - odom.source_us / 1e6) * 1000
        ),
        telemetry_valid_mask=telem.valid_mask if telem_ok else 0,
        tag_valid=0,
        depth_m=odom.depth,
        yaw_command=out.axes.yaw,
        forward_command=out.axes.forward,
        left_command=out.axes.left,
        down_command=out.axes.down,
        up_command=out.axes.up,
        vertical_mode=int(out.hold_depth),
        depth_target_m=out.depth_target,
        completed=int(out.complete),
        fault=controller.error if controller.fault else "",
    )
    if out.reference:
        r = out.reference
        row.update(
            ref_x_m=r.x_m,
            ref_y_m=r.y_m,
            ref_yaw_rad=r.yaw_rad,
            ref_vx_m_s=r.x_velocity_m_s,
            ref_vy_m_s=r.y_velocity_m_s,
        )
    return row


class SessionRecorder(threading.Thread):
    def __init__(self, directory, metadata, interpolator):
        super().__init__(name="csv-writer", daemon=True)
        self.directory = Path(directory)
        self.metadata = metadata
        self.interpolator = interpolator
        self.rows = queue.Queue(maxsize=5000)
        self.stopping = threading.Event()
        self.error = ""
        self.written = self.aligned = self.dropped = 0

    def record(self, row):
        if row is None or self.stopping.is_set() or self.error:
            return
        try:
            self.rows.put_nowait(row)
        except queue.Full:
            self.dropped += 1

    def run(self):
        try:
            self.directory.mkdir(parents=True, exist_ok=False)
            metadata = dict(
                self.metadata,
                protocol_version=1,
                started_utc=datetime.now(timezone.utc).isoformat(),
            )
            trajectory = metadata.pop("trajectory_path", "")
            if trajectory:
                source = Path(trajectory)
                (self.directory / "trajectory.csv").write_bytes(source.read_bytes())
                metadata["trajectory_original"] = str(source)
            (self.directory / "session.json").write_text(
                json.dumps(metadata, indent=2, ensure_ascii=False), encoding="utf-8"
            )
            with (
                (self.directory / "telemetry.csv").open(
                    "w", newline="", encoding="utf-8-sig"
                ) as stream,
                (self.directory / "aligned.csv").open(
                    "w", newline="", encoding="utf-8-sig"
                ) as strict,
            ):
                writer = csv.DictWriter(stream, fieldnames=FIELDS)
                aligned = csv.DictWriter(
                    strict, fieldnames=LEGACY_FIELDS, extrasaction="ignore"
                )
                writer.writeheader()
                aligned.writeheader()
                while not self.stopping.is_set() or not self.rows.empty():
                    try:
                        row = self.rows.get(timeout=0.05)
                    except queue.Empty:
                        continue
                    # Wait only in this writer for a future bracketing camera pose.
                    stamp = row["source_pc_monotonic"]
                    deadline = time.monotonic() + 0.5
                    tag = self.interpolator.interpolate(stamp)
                    while (
                        tag is None
                        and not self.stopping.is_set()
                        and time.monotonic() < deadline
                    ):
                        first, last = self.interpolator.bounds()
                        if (
                            last is None
                            or (first is not None and stamp < first)
                            or last >= stamp
                        ):
                            break
                        self.stopping.wait(0.01)
                        tag = self.interpolator.interpolate(stamp)
                    if tag:
                        row.update(
                            tag_timestamp_utc=tag.captured_utc,
                            tag_x_m=tag.x_m,
                            tag_y_m=tag.y_m,
                            tag_yaw_rad=tag.yaw_rad,
                            tag_valid=1,
                        )
                        aligned.writerow(row)
                        self.aligned += 1
                    row["dropped_rows"] = self.dropped
                    writer.writerow(row)
                    self.written += 1
                    if self.written % 50 == 0:
                        stream.flush()
                        strict.flush()
            (self.directory / "result.json").write_text(
                json.dumps(
                    dict(
                        rows=self.written,
                        aligned=self.aligned,
                        dropped=self.dropped,
                        error=self.error,
                    ),
                    indent=2,
                ),
                encoding="utf-8",
            )
        except Exception as error:
            self.error = str(error)

    def close(self):
        self.stopping.set()
        if self.is_alive():
            self.join(3)
        if self.is_alive():
            self.error = "写盘未能在 3 秒内结束，请检查磁盘"
