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
        return self.direction * math.pi / 2

    @property
    def table(self):
        return self.x, self.y + self.direction * 0.25, 0.155

    def door_x(self, side):
        return self.x + self.direction * (-0.5 if side == "entry" else 0.5)

    def door_joint(self, side):
        return f"room_{self.number}_{side}_z"


# Compact room placement.
ROOMS = (
    Room(1, -0.70,  0.65,  1),
    Room(2,  0.70,  0.65,  1),
    Room(3,  0.70, -0.65, -1),
    Room(4, -0.70, -0.65, -1),
)

ROOM_CODES = tuple(room.code for room in ROOMS)

START = (-1.55, 0.65)
RADIUS = 0.5


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
    Clockwise tape path around the four rooms.

    Room footprint assumptions from generate_four_rooms.py:
      width: 1.00 m
      top rooms:    y = 0.35 .. 1.30
      bottom rooms: y = -1.30 .. -0.35

    The bypass track stays approximately:
      0.15 m outside the room side walls
      0.25 m outside the top/bottom walls
    """

    r_main = 0.15
    r_detour = 0.12

    wps = [
        # Start / left outer lane.
        (START, 0.0),

        # Room 1 bypass.
        ((-1.35,  0.65), r_detour),
        ((-1.35,  1.55), r_detour),
        ((-0.05,  1.55), r_detour),
        ((-0.05,  0.65), r_detour),

        # Gap between Rooms 1 and 2.
        (( 0.05,  0.65), r_detour),

        # Room 2 bypass.
        (( 0.05,  1.55), r_detour),
        (( 1.35,  1.55), r_detour),
        (( 1.35,  0.65), r_detour),

        # Right outer side.
        (( 1.55,  0.65), r_main),
        (( 1.55, -0.65), r_main),

        # Room 3 bypass.
        (( 1.35, -0.65), r_detour),
        (( 1.35, -1.55), r_detour),
        (( 0.05, -1.55), r_detour),
        (( 0.05, -0.65), r_detour),

        # Gap between Rooms 3 and 4.
        ((-0.05, -0.65), r_detour),

        # Room 4 bypass.
        ((-0.05, -1.55), r_detour),
        ((-1.35, -1.55), r_detour),
        ((-1.35, -0.65), r_detour),

        # Left outer side / return.
        ((-1.55, -0.65), r_main),
        (START, 0.0),
    ]

    path = []

    for i, (point, radius) in enumerate(wps):
        if radius <= 0.0:
            if not path or math.dist(path[-1], point) > 1e-9:
                path.append(point)
            continue

        prev_point = wps[i - 1][0]
        next_point = wps[(i + 1) % len(wps)][0]

        pts = _fillet(
            prev_point,
            point,
            next_point,
            radius,
            samples=32,
        )

        if path and math.dist(path[-1], pts[0]) < 1e-6:
            path.extend(pts[1:])
        else:
            path.extend(pts)

    # Ensure the loop is explicitly closed.
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


def room_stop(room, phase):
    """
    Return a tape-distance stop near a room door/work location.

    Entry/exit targets remain outside the enclosure instead of placing
    the main transport robot directly on the wall.
    """
    offset = {
        "entry": -0.65,
        "work": 0.0,
        "exit": 0.65,
    }[phase]

    return project(
        room.x + room.direction * offset,
        room.y,
    )[0]


def door_clear(x, y, heading, door_x, extension=0.0):
    """Check whether the robot body is clear of a door plane."""

    xs = [
        x + a * math.cos(heading) - b * math.sin(heading)
        for a in (-0.105, max(0.105, extension + 0.016))
        for b in (-0.085, 0.085)
    ]

    return (
        max(xs) < door_x - 0.025
        or min(xs) > door_x + 0.025
    )
