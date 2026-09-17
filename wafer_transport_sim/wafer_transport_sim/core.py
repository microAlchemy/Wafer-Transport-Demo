"""ROS-independent control decisions, shared by nodes and tests."""
import math
from dataclasses import dataclass

STATIONS = ('STATION_A', 'STATION_B', 'STATION_C')
MOVING = {'START_TRANSPORT', 'FOLLOW_PATH', 'SCAN_QR', 'IS_TARGET_QR',
          'CONTINUE', 'SLOW_APPROACH', 'PRECISION_DOCK'}


def clamp(value, low, high):
    return max(low, min(high, value))


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def angle_error(target, actual):
    return math.atan2(math.sin(target - actual), math.cos(target - actual))


def destination_error(value, active):
    if value not in STATIONS:
        return 'destination_station must be STATION_A, STATION_B, or STATION_C'
    if active:
        return 'Destination is latched for this mission; relaunch to change it'
    return ''


def select_command(state, line, docking, fresh):
    if not fresh:
        return (0.0, 0.0)
    if state in {'SLOW_APPROACH', 'PRECISION_DOCK'}:
        return docking
    if state in MOVING:
        return line
    return (0.0, 0.0)


def docking_speed(distance, tolerance, max_speed, gain):
    if not math.isfinite(distance) or distance <= tolerance:
        return 0.0
    return min(max_speed, gain * distance)


def placed(position, target, xy_tolerance=0.015, z_tolerance=0.006):
    return (all(math.isfinite(v) for v in (*position, *target)) and
            math.hypot(position[0] - target[0], position[1] - target[1])
            <= xy_tolerance and abs(position[2] - target[2]) <= z_tolerance)


class DetectionFilter:
    def __init__(self, count=3, valid_codes=STATIONS):
        self.required = count
        self.valid_codes = valid_codes
        self.last = ''
        self.count = 0

    def update(self, station):
        if station not in self.valid_codes:
            self.last, self.count = '', 0
            return ''
        self.count = self.count + 1 if station == self.last else 1
        self.last = station
        return station if self.count >= self.required else ''


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
