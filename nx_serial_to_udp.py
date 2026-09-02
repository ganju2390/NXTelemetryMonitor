#!/usr/bin/env python3
"""Forward raw fixed-size serial frames from an NVIDIA NX UART to UDP.

No payload decoding, validation, byte-order conversion, or modification is
performed.  Every consecutive frame_size bytes read from the UART becomes one
UDP datagram with the identical bytes.
"""

from __future__ import annotations

import argparse
import os
import select
import socket
import sys
import termios
import time


BAUD_RATES = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
    230400: termios.B230400,
    460800: termios.B460800,
    921600: termios.B921600,
}

FRAME_SIZE = 40
FRAME_TAIL = b"\x00\x00\x80\x7F"


def configure_serial(device: str, baud: int) -> int:
    if baud not in BAUD_RATES:
        supported = ", ".join(str(rate) for rate in BAUD_RATES)
        raise ValueError(f"unsupported baud rate {baud}; supported: {supported}")

    file_descriptor = os.open(device, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    attributes = termios.tcgetattr(file_descriptor)
    attributes[0] = termios.IGNPAR
    attributes[1] = 0
    attributes[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    attributes[3] = 0
    attributes[4] = BAUD_RATES[baud]
    attributes[5] = BAUD_RATES[baud]
    attributes[6][termios.VMIN] = 0
    attributes[6][termios.VTIME] = 0
    termios.tcsetattr(file_descriptor, termios.TCSANOW, attributes)
    termios.tcflush(file_descriptor, termios.TCIFLUSH)
    return file_descriptor


def is_valid_frame(frame: bytes, frame_size: int) -> bool:
    """A valid telemetry frame has the fixed size and the V5_SUB tail marker."""
    return len(frame) == frame_size and frame[-len(FRAME_TAIL):] == FRAME_TAIL


def forward_serial_to_udp(device: str, baud: int, destination_ip: str, destination_port: int, frame_size: int) -> None:
    serial_fd = configure_serial(device, baud)
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    buffer = bytearray()
    forwarded_frames = 0
    forwarded_bytes = 0
    rejected_candidates = 0
    last_report = time.monotonic()

    try:
        print(
            f"Forwarding {device} at {baud} baud to UDP {destination_ip}:{destination_port}; "
            f"frame size = {frame_size} bytes.",
            flush=True,
        )
        while True:
            readable, _, _ = select.select([serial_fd], [], [], 1.0)
            if readable:
                data = os.read(serial_fd, 4096)
                if data:
                    buffer.extend(data)

                while len(buffer) >= frame_size:
                    frame = bytes(buffer[:frame_size])
                    if is_valid_frame(frame, frame_size):
                        # This is intentionally the only transformation: serial stream
                        # segmentation into UDP datagrams. The frame bytes are unchanged.
                        del buffer[:frame_size]
                        udp_socket.sendto(frame, (destination_ip, destination_port))
                        forwarded_frames += 1
                        forwarded_bytes += frame_size
                    else:
                        # UART has no packet boundary. Drop one byte and try the next
                        # fixed-size window, which re-synchronizes on the fixed tail.
                        del buffer[0]
                        rejected_candidates += 1

            now = time.monotonic()
            if now - last_report >= 1.0:
                print(
                    f"forwarded {forwarded_frames} valid frames / {forwarded_bytes} bytes; "
                    f"rejected windows {rejected_candidates}; serial remainder {len(buffer)} bytes",
                    flush=True,
                )
                last_report = now
    finally:
        udp_socket.close()
        os.close(serial_fd)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Raw /dev/ttyTHS1 to UDP forwarder")
    parser.add_argument("--udp-ip", required=True, help="IP address of the PC running NXTelemetryMonitor")
    parser.add_argument("--udp-port", type=int, required=True, help="UDP listening port on the PC")
    parser.add_argument("--device", default="/dev/ttyTHS1", help="UART device path (default: /dev/ttyTHS1)")
    parser.add_argument("--baud", type=int, default=115200, help="UART baud rate (default: 115200)")
    arguments = parser.parse_args()
    if not 1 <= arguments.udp_port <= 65535:
        parser.error("--udp-port must be between 1 and 65535")
    return arguments


def main() -> None:
    arguments = parse_arguments()
    try:
        forward_serial_to_udp(
            device=arguments.device,
            baud=arguments.baud,
            destination_ip=arguments.udp_ip,
            destination_port=arguments.udp_port,
            frame_size=FRAME_SIZE,
        )
    except (OSError, ValueError) as error:
        print(f"serial-to-UDP forwarder failed: {error}", file=sys.stderr)
        raise SystemExit(1) from error
    except KeyboardInterrupt:
        print("\nStopped.", flush=True)


if __name__ == "__main__":
    main()
