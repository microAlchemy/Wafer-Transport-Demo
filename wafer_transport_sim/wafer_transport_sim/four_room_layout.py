"""Shared geometry for the four-room world and mission (metres, world frame)."""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Room:
    number: int
    x: float
    y: float
    direction: int

    @property
    def code(self):
        return f"ROOM_{self.number}"

    @property
    def heading(self):
        return 0.0 if self.direction == 1 else math.pi

    @property
    def work_heading(self):
        # The robot remains outside and turns inward to operate the wand.
        return -self.direction * math.pi / 2

    @property
    def route_y(self):
        return self.y + self.direction * (HALF_DEPTH + CORRIDOR_WIDTH / 2)

    @property
    def table(self):
        # Keep the table just inside the outside-facing service opening and
        # within the 0.25 m reach of the wand parked on the exterior tape.
        return self.x, self.y + self.direction * (HALF_DEPTH - 0.10), 0.155

    def door_x(self, side):
        return self.x + self.direction * (-HALF_LENGTH if side == "entry" else HALF_LENGTH)

    def door_joint(self, side):
        return f"room_{self.number}_{side}_z"


# Published facility dimensions.  Every room is 4 ft x 3 ft and adjacent
# room walls are separated by a clear 1 ft corridor.
ROOM_LENGTH = 1.2192
ROOM_DEPTH = 0.9144
ROOM_HEIGHT = 1.2192
CORRIDOR_WIDTH = 0.3048
HALF_LENGTH = ROOM_LENGTH / 2
HALF_DEPTH = ROOM_DEPTH / 2

ROOMS = (
    Room(1, -1.0,  0.8,  1),
    Room(2,  1.0,  0.8,  1),
    Room(3,  1.0, -0.8, -1),
    Room(4, -1.0, -0.8, -1),
)

ROOM_CODES = tuple(room.code for room in ROOMS)

START = (-1.9620, 0.8)
RADIUS = CORRIDOR_WIDTH / 2


def _fillet(p0, p1, p2, radius, samples=32):
    """Return a sampled circular fillet around waypoint p1."""

    in_dx = p1[0] - p0[0]
    in_dy = p1[1] - p0[1]
    out_dx = p2[0] - p1[0]
    out_dy = p2[1] - p1[1]

    len_in = math.hypot(in_dx, in_dy)
    len_out = math.hypot(out_dx, out_dy)

    if len_in < 1e-9 or len_out < 1e-9 or radius <= 0.0:
        return [p1]

    vin = (in_dx / len_in, in_dy / len_in)
    vout = (out_dx / len_out, out_dy / len_out)

    dot = max(-1.0, min(1.0, vin[0] * vout[0] + vin[1] * vout[1]))
    turn_angle = math.acos(dot)

    if turn_angle < 1e-5 or abs(turn_angle - math.pi) < 1e-5:
        return [p1]

    tangent = radius * math.tan(turn_angle / 2.0)

    # Do not let a corner radius consume most of a short segment.
    max_tangent = 0.45 * min(len_in, len_out)
    if tangent > max_tangent:
        tangent = max_tangent
        radius = tangent / math.tan(turn_angle / 2.0)

    t1 = (
        p1[0] - vin[0] * tangent,
        p1[1] - vin[1] * tangent,
    )
    t2 = (
        p1[0] + vout[0] * tangent,
        p1[1] + vout[1] * tangent,
    )

    cross = vin[0] * vout[1] - vin[1] * vout[0]

    if cross > 0.0:
        # Left turn.
        nx, ny = -vin[1], vin[0]
    else:
        # Right turn.
        nx, ny = vin[1], -vin[0]

    cx = t1[0] + nx * radius
    cy = t1[1] + ny * radius

    a1 = math.atan2(t1[1] - cy, t1[0] - cx)
    a2 = math.atan2(t2[1] - cy, t2[0] - cx)

    if cross > 0.0:
        while a2 < a1:
            a2 += 2.0 * math.pi
    else:
        while a2 > a1:
            a2 -= 2.0 * math.pi

    return [
        (
            cx + radius * math.cos(a1 + (a2 - a1) * i / samples),
            cy + radius * math.sin(a1 + (a2 - a1) * i / samples),
        )
        for i in range(samples + 1)
    ]


def route_points():
    """
    Clockwise tape path around the outside of every room.

    This restores the earlier detour topology: up and around each top room,
    down the right side, around each bottom room, then up the left side.  The
    tape is centred in a 1 ft lane outside the room walls.
    """

    left, middle_left = -1.7620, -0.2380
    middle_right, right = 0.2380, 1.7620
    return_x = -2.20
    top, bottom = 1.4096, -1.4096
    wps = [
        (START, 0.0),
        ((left, ROOMS[0].y), RADIUS),
        ((left, top), RADIUS),
        ((middle_left, top), RADIUS),
        ((middle_left, ROOMS[0].y), RADIUS),
        ((middle_right, ROOMS[1].y), RADIUS),
        ((middle_right, top), RADIUS),
        ((right, top), RADIUS),
        ((right, ROOMS[1].y), RADIUS),
        ((right, ROOMS[2].y), RADIUS),
        ((right, bottom), RADIUS),
        ((middle_right, bottom), RADIUS),
        ((middle_right, ROOMS[2].y), RADIUS),
        ((middle_left, ROOMS[3].y), RADIUS),
        ((middle_left, bottom), RADIUS),
        ((left, bottom), RADIUS),
        ((left, ROOMS[3].y), RADIUS),
        ((return_x, ROOMS[3].y), RADIUS),
        ((return_x, ROOMS[0].y), RADIUS),
        (START, 0.0),
    ]
    path = []
    for i, (point, radius) in enumerate(wps):
        if radius <= 0.0:
            if not path or math.dist(path[-1], point) > 1e-9:
                path.append(point)
            continue
        pts = _fillet(wps[i-1][0], point, wps[(i+1) % len(wps)][0],
                      radius, samples=24)
        path.extend(pts[1:] if path and math.dist(path[-1], pts[0]) < 1e-6 else pts)
    if math.dist(path[-1], path[0]) > 1e-9:
        path.append(path[0])
    return tuple(path)


PATH = route_points()

LENGTH = sum(
    math.dist(a, b)
    for a, b in zip(PATH, PATH[1:])
)


def project(x, y):
    """Return distance along tape and cross-track distance."""

    best = (float("inf"), 0.0)
    accumulated = 0.0

    for (ax, ay), (bx, by) in zip(PATH, PATH[1:]):
        dx = bx - ax
        dy = by - ay
        length = math.hypot(dx, dy)

        if length > 0.0:
            t = max(
                0.0,
                min(
                    1.0,
                    ((x - ax) * dx + (y - ay) * dy) / (length * length),
                ),
            )

            error = math.hypot(
                x - ax - t * dx,
                y - ay - t * dy,
            )

            if error < best[0]:
                best = (
                    error,
                    accumulated + t * length,
                )

        accumulated += length

    return best[1], best[0]


def point_at(distance):
    """Return a point on the closed tape loop at an along-route distance."""
    distance %= LENGTH
    accumulated = 0.0
    for (ax, ay), (bx, by) in zip(PATH, PATH[1:]):
        segment = math.hypot(bx-ax, by-ay)
        if segment and accumulated + segment >= distance:
            ratio = (distance-accumulated)/segment
            return ax+ratio*(bx-ax), ay+ratio*(by-ay)
        accumulated += segment
    return PATH[0]


def room_stop(room, phase):
    """
    Return a tape-distance stop near a room door/work location.

    Entry/exit targets remain outside the enclosure instead of placing
    the main transport robot directly on the wall.
    """
    offset = {
        # Stop before the closed panel with the custom forward camera clear.
        # In the 1 ft gap this is also the previous room's exit position.
        "entry": -(HALF_LENGTH + 0.20),
        "work": 0.0,
        "exit": HALF_LENGTH - 0.20,
    }[phase]

    distance = project(
        room.x + room.direction * offset,
        room.route_y if phase == 'exit' else room.y,
    )[0]
    # Cycle the side-mounted doors while stopped clear of their swept planes.
    return distance


def door_clear(x, y, heading, door_x, extension=0.0):
    """Check whether the robot body is clear of a door plane."""

    xs = [
        x + a * math.cos(heading) - b * math.sin(heading)
        for a in (-0.105, max(0.145, extension + 0.016))
        for b in (-0.090, 0.090)
    ]

    return (
        max(xs) < door_x - 0.025
        or min(xs) > door_x + 0.025
    )
