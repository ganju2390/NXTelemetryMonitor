"""MAVLink framing and bounded, single-owner UDP transport."""

import queue
import socket
import threading
import time
from .generated import finsrov as mav


class Codec:
    def __init__(self, system=255, component=190):
        self.encoder = mav.MAVLink(None, srcSystem=system, srcComponent=component)
        self.decoder = mav.MAVLink(None)
        self.decoder.robust_parsing = True

    def encode(self, kind, **fields):
        packet = getattr(self.encoder, kind.lower() + "_encode")(**fields)
        data = packet.pack(self.encoder)
        self.encoder.seq = (self.encoder.seq + 1) & 255
        return data

    def decode(self, data):
        return [
            m
            for m in (self.decoder.parse_buffer(data) or [])
            if m.get_type() != "BAD_DATA"
        ]


class UDPLink(threading.Thread):
    def __init__(self, bind, peer, system=255):
        super().__init__(name="mavlink-udp", daemon=True)
        self.peer = (socket.gethostbyname(peer[0]), peer[1])
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.bind(bind)
            self.socket.settimeout(0.01)
        except Exception:
            self.socket.close()
            raise
        self.codec = Codec(system)
        self.incoming = queue.Queue(maxsize=512)
        self.outgoing = queue.Queue(maxsize=256)
        self.stop_event = threading.Event()
        self.error = ""
        self.dropped = 0

    def send(self, kind, **fields):
        try:
            self.outgoing.put_nowait((kind, fields))
        except queue.Full:
            raise RuntimeError("Network send queue full")

    def run(self):
        try:
            while not self.stop_event.is_set():
                for _ in range(64):
                    try:
                        kind, fields = self.outgoing.get_nowait()
                    except queue.Empty:
                        break
                    self.socket.sendto(self.codec.encode(kind, **fields), self.peer)
                try:
                    data, source = self.socket.recvfrom(65535)
                except socket.timeout:
                    continue
                if source != self.peer:
                    continue
                received = time.monotonic()
                try:
                    messages = self.codec.decode(data)
                except Exception:
                    self.dropped += 1
                    continue
                for message in messages:
                    try:
                        self.incoming.put_nowait((message, received))
                    except queue.Full:
                        self.dropped += 1
        except Exception as error:
            self.error = str(error)
        finally:
            self.socket.close()

    def close(self):
        self.stop_event.set()
        if self.is_alive():
            self.join(timeout=2)
        else:
            self.socket.close()
