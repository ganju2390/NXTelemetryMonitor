"""GStreamer owns RTP/decode in a child process; only latest RGB frame is kept."""

from collections import deque
import os
import queue
import shutil
import subprocess
import threading
import time
from .queues import latest_put


def pipeline(
    executable, port=5600, width=1280, height=480, latency=50, test_source=False
):
    if (
        not 1 <= port <= 65535
        or width % 4
        or not 4 <= width <= 3840
        or not 4 <= height <= 2160
        or not 0 <= latency <= 2000
    ):
        raise ValueError("视频参数无效（宽度须为 4 的倍数）")
    source = (
        ["videotestsrc", "is-live=true"]
        if test_source
        else [
            "udpsrc",
            f"port={port}",
            "caps=application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000",
            "!",
            "rtpjitterbuffer",
            f"latency={latency}",
            "drop-on-latency=true",
            "!",
            "rtph264depay",
            "!",
            "h264parse",
            "!",
            "avdec_h264",
        ]
    )
    return [
        executable,
        "-q",
        *source,
        "!",
        "videoconvert",
        "!",
        "videoscale",
        "!",
        f"video/x-raw,format=RGB,width={width},height={height}",
        "!",
        "queue",
        "max-size-buffers=1",
        "leaky=downstream",
        "!",
        "fdsink",
        "fd=1",
        "sync=false",
    ]


class VideoReceiver(threading.Thread):
    def __init__(
        self,
        *,
        executable="gst-launch-1.0",
        port=5600,
        width=1280,
        height=480,
        latency=50,
        test_source=False,
    ):
        super().__init__(name="gstreamer-reader", daemon=True)
        self.command = pipeline(executable, port, width, height, latency, test_source)
        self.width, self.height = width, height
        self.frames = queue.Queue(maxsize=1)
        self.last_frame = 0.0
        self.error = ""
        self.stderr = deque(maxlen=8)
        self.process = None
        self.stopping = threading.Event()
        self.test_source = test_source

    @property
    def status(self):
        if self.error:
            return self.error
        if self.stopping.is_set():
            return "视频已停止"
        if time.monotonic() - self.last_frame > 2:
            return "等待视频 / 视频断流（UDP H.264/RTP）"
        return "测试视频源" if self.test_source else "GStreamer 实时视频"

    def run(self):
        try:
            executable = shutil.which(self.command[0])
            if not executable and not os.path.isfile(self.command[0]):
                raise FileNotFoundError("未找到 GStreamer：请配置 gst-launch-1.0 路径")
            self.command[0] = executable or os.path.abspath(self.command[0])
            self.process = subprocess.Popen(
                self.command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
                creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0,
            )

            def drain():
                for line in iter(self.process.stderr.readline, b""):
                    self.stderr.append(line.decode(errors="replace").strip())

            errors = threading.Thread(
                target=drain, daemon=True, name="gstreamer-errors"
            )
            errors.start()
            size = self.width * self.height * 3
            while not self.stopping.is_set():
                buffer = bytearray()
                while len(buffer) < size and not self.stopping.is_set():
                    chunk = self.process.stdout.read(size - len(buffer))
                    if not chunk:
                        raise RuntimeError(
                            "GStreamer 已退出：" + " | ".join(self.stderr)
                        )
                    buffer.extend(chunk)
                if len(buffer) == size:
                    self.last_frame = time.monotonic()
                    latest_put(self.frames, bytes(buffer))
        except Exception as error:
            if not self.stopping.is_set():
                self.error = str(error)
        finally:
            self._terminate()

    def _terminate(self):
        process = self.process
        if process is not None:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(2)
            for pipe in (process.stdout, process.stderr):
                if pipe:
                    pipe.close()

    def close(self):
        self.stopping.set()
        process = self.process
        if process and process.poll() is None:
            process.terminate()  # unblock stdout.read even when RTP never arrived
        if self.is_alive():
            self.join(3)
