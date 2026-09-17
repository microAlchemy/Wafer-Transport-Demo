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
        return f'ROOM_{self.number}'

    @property
    def heading(self):
        return 0. if self.direction == 1 else math.pi

    @property
    def work_heading(self):
        return self.direction * math.pi / 2

    @property
    def table(self):
        return self.x, self.y + self.direction * .25, .155

    def door_x(self, side):
        return self.x + self.direction * (-.5 if side == 'entry' else .5)

    def door_joint(self, side):
        return f'room_{self.number}_{side}_z'


ROOMS = (Room(1, -.95, .75, 1), Room(2, .95, .75, 1),
         Room(3, .95, -.75, -1), Room(4, -.95, -.75, -1))
ROOM_CODES = tuple(room.code for room in ROOMS)
START = (-1.7, .75)
RADIUS = .5


def route_points():
    """Clockwise rounded rectangle; coincident first/last point closes tape."""
    result = [START, (1.7, .75)]
    for cx, cy, start in [(1.7, .25, math.pi/2), (1.7, -.25, 0.),
                          (-1.7, -.25, -math.pi/2), (-1.7, .25, -math.pi)]:
        first = (cx + RADIUS*math.cos(start), cy + RADIUS*math.sin(start))
        if math.dist(result[-1], first) > 1e-9:
            result.append(first)
        result.extend((cx + RADIUS*math.cos(start - i*math.pi/64),
                       cy + RADIUS*math.sin(start - i*math.pi/64))
                      for i in range(1, 33))
    return tuple(result)


PATH = route_points()
LENGTH = sum(math.dist(a, b) for a, b in zip(PATH, PATH[1:]))


def project(x, y):
    """Distance along tape and cross-track distance, using actual base pose."""
    best = (float('inf'), 0.)
    accumulated = 0.
    for (ax, ay), (bx, by) in zip(PATH, PATH[1:]):
        dx, dy = bx-ax, by-ay
        length = math.hypot(dx, dy)
        t = max(0., min(1., ((x-ax)*dx + (y-ay)*dy)/(length*length)))
        error = math.hypot(x-ax-t*dx, y-ay-t*dy)
        if error < best[0]:
            best = error, accumulated + t*length
        accumulated += length
    return best[1], best[0]


def room_stop(room, phase):
    offset = {'entry': -.70, 'work': 0., 'exit': .70}[phase]
    return project(room.x + room.direction*offset, room.y)[0]


def door_clear(x, y, heading, door_x, extension=0.):
    xs = [x + a*math.cos(heading) - b*math.sin(heading)
          for a in (-.105, max(.105, extension+.016)) for b in (-.085, .085)]
    return max(xs) < door_x-.025 or min(xs) > door_x+.025
