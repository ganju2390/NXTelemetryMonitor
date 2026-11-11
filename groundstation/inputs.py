"""Pure legacy mapping plus an optional SDL gamepad adapter."""

import math
from .models import Axes, InputSnapshot


def deadzone(value, threshold=0.0):
    if not math.isfinite(value):
        raise ValueError("Non-finite joystick axis")
    value = max(-1.0, min(1.0, value))
    return 0.0 if abs(value) <= threshold else value


def map_gamepad(axes, buttons, hat, now, threshold=0.0):
    if len(axes) < 6 or len(buttons) < 7 or not 0 <= threshold < 1:
        raise ValueError("手柄需至少 6 轴、7 键；死区须位于 [0,1)")
    if not all(math.isfinite(a) for a in axes):
        raise ValueError("Invalid joystick axes")
    normalize = lambda a: max(0.0, min(1.0, (a + 1) / 2))
    mapped = Axes(
        deadzone(-axes[0], threshold),
        deadzone(-axes[3], threshold),
        deadzone(-axes[2], threshold),
        normalize(axes[4]),
        normalize(axes[5]),
    )
    return InputSnapshot(now, True, mapped, (int(hat[0]), int(hat[1])))


class ButtonEdges:
    def __init__(self):
        self.previous = None

    def update(self, buttons):
        current = tuple(bool(x) for x in buttons)
        # A held button on initial connection/reconnection must not act as a press.
        edges = (
            []
            if self.previous is None
            else [i for i in (6, 0, 1) if current[i] and not self.previous[i]]
        )
        self.previous = current
        return edges

    def reset(self):
        self.previous = None


class Gamepad:
    def __init__(self):
        self.pygame = None
        self.device = None
        self.edges = ButtonEdges()

    def initialize(self):
        if self.pygame is None:
            import os

            os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
            import pygame

            pygame.display.init()  # SDL event subsystem; no window is created.
            pygame.joystick.init()
            self.pygame = pygame

    def devices(self):
        self.initialize()
        self.pygame.event.pump()
        return [
            (i, self.pygame.joystick.Joystick(i).get_name())
            for i in range(self.pygame.joystick.get_count())
        ]

    def select(self, index):
        self.close_device()
        self.initialize()
        self.device = self.pygame.joystick.Joystick(index)
        self.device.init()

    def close_device(self):
        if self.device is not None:
            self.device.quit()
        self.device = None
        self.edges.reset()

    def sample(self, now, threshold=0.0):
        try:
            self.initialize()
            self.pygame.event.pump()
            self.pygame.event.clear()
            if self.device is None or not self.device.get_attached():
                raise ValueError("手柄未连接")
            axes = [self.device.get_axis(i) for i in range(self.device.get_numaxes())]
            buttons = [
                self.device.get_button(i) for i in range(self.device.get_numbuttons())
            ]
            hat = self.device.get_hat(0) if self.device.get_numhats() else (0, 0)
            snapshot = map_gamepad(axes, buttons, hat, now, threshold)
            return snapshot, self.edges.update(buttons), ""
        except Exception as error:
            self.close_device()
            return InputSnapshot(received=now), [], str(error)

    def close(self):
        self.close_device()
        if self.pygame:
            self.pygame.quit()
