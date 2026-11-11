"""Bounded latest-value exchange; producers never wait for consumers."""

import queue


def latest_put(channel, value):
    try:
        channel.put_nowait(value)
    except queue.Full:
        try:
            channel.get_nowait()
        except queue.Empty:
            pass
        try:
            channel.put_nowait(value)
        except queue.Full:
            pass


def latest_get(channel, default=None):
    result = default
    while True:
        try:
            result = channel.get_nowait()
        except queue.Empty:
            return result
