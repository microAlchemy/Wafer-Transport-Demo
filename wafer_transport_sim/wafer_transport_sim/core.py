"""ROS-independent control decisions, shared by nodes and tests."""
import math
from dataclasses import dataclass


def clamp(value, low, high):
    return max(low, min(high, value))


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def angle_error(target, actual):
    return math.atan2(math.sin(target - actual), math.cos(target - actual))


def placed(position, target, xy_tolerance=0.015, z_tolerance=0.006):
    return (all(math.isfinite(v) for v in (*position, *target)) and
            math.hypot(position[0] - target[0], position[1] - target[1])
            <= xy_tolerance and abs(position[2] - target[2]) <= z_tolerance)


class DetectionFilter:
    def __init__(self, count=3, valid_codes=()):
        self.required = count
        self.valid_codes = valid_codes
        self.last = ''
        self.count = 0

    def update(self, code):
        if code not in self.valid_codes:
            self.last, self.count = '', 0
            return ''
        self.count = self.count + 1 if code == self.last else 1
        self.last = code
        return code if self.count >= self.required else ''


@dataclass
class Sequence:
    """Feedback-gated steps; reaching a timeout can never count as success."""
    steps: tuple
    index: int = 0
    entered: float = 0.0

    @property
    def state(self):
        return self.steps[self.index]

    def advance(self, verified, now):
        if verified and self.index < len(self.steps) - 1:
            self.index += 1
            self.entered = now
            return True
        return False

    def expired(self, now, timeout):
        return now - self.entered > timeout
