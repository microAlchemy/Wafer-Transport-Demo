"""Shared geometry for the eleven-glovebox world and mission (metres, world frame).

The facility dimensions, the process-cell transfer constants and the black-tape
network live in one module so the world generator, the IR probe model, the
transport controller and the tests cannot drift apart.

The tape is a *network*, not a fixed all-room trajectory: two continuous
straight lanes joined by transverse rungs.  Track 1 is the main lane farthest
from the glovebox doors, Track 2 is the service lane beside them, and the
route FSM selects one finite edge at a time.
"""

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
        return f"GLOVEBOX_{self.number:02d}"

    @property
    def heading(self):
        return 0.0

    @property
    def work_heading(self):
        return math.pi / 2

    @property
    def route_y(self):
        return self.y - HALF_DEPTH - CORRIDOR_WIDTH / 2

    @property
    def table(self):
        """Blue process stage in the middle of the room."""
        return self.x, self.y - HALF_DEPTH + HANDOFF_SETBACK, SUPPORTED_WAFER_Z

    def handoff(self, side):
        """Wafer support just inside the entry or exit guillotine door."""
        offset = -HANDOFF_OFFSET if side == 'entry' else HANDOFF_OFFSET
        return self.x + offset, self.y - HALF_DEPTH + HANDOFF_SETBACK, SUPPORTED_WAFER_Z

    def handoff_heading(self, side):
        return math.pi / 2

    def process_joint(self, axis):
        return f"room_{self.number}_process_{axis}"

    def process_attachment(self):
        return f"process_{self.number}"

    def tag(self, side):
        return f"GLOVEBOX_{self.number:02d}_{side.upper()}"

    def door_x(self, side):
        return self.x + (-HANDOFF_OFFSET if side == 'entry' else HANDOFF_OFFSET)

    def door_position(self, side):
        return self.door_x(side), self.y-HALF_DEPTH

    def door_joint(self, side):
        return f"room_{self.number}_{side}_z"


# Process-cell transfer geometry.  The support, stage and wafer heights fix
# where an internal vacuum cup may touch down, so the rail shuttle, its joint
# travel and the controller are all derived from the values below instead of
# repeating a hand-tuned reach.
HANDOFF_OFFSET = .22
HANDOFF_SETBACK = .10  # keep shelves and wafer behind the closed door sweep
PROCESS_STAGE_DEPTH = .16
SUPPORT_TOP = .154
WAFER_THICKNESS = .002
WAFER_HALF_THICKNESS = WAFER_THICKNESS/2
SUPPORTED_WAFER_Z = SUPPORT_TOP + WAFER_HALF_THICKNESS

PROCESS_TRAVEL = 2*HANDOFF_OFFSET
PROCESS_RAIL_Z = .475
PROCESS_RAIL_SIZE = .055
PROCESS_CARRIAGE_SIZE = .12
PROCESS_ARM_LENGTH = .18
PROCESS_ARM_CENTER = .05
PROCESS_ARM_WIDTH = .045
PROCESS_CUP_RADIUS = .035
PROCESS_CUP_LENGTH = .012
PROCESS_CUP_OFFSET = .101
# Clearance between the parked cup and the top face of a supported wafer.
PROCESS_LIFT_STROKE = .03

PROCESS_CUP_REST_BOTTOM = round(SUPPORT_TOP + WAFER_THICKNESS + PROCESS_LIFT_STROKE, 6)
PROCESS_GRIPPER_Z = round(PROCESS_CUP_REST_BOTTOM + PROCESS_CUP_OFFSET + PROCESS_CUP_LENGTH/2, 6)
PROCESS_ARM_TOP = round(PROCESS_GRIPPER_Z + PROCESS_ARM_CENTER + PROCESS_ARM_LENGTH/2, 6)
PROCESS_RAIL_BOTTOM = round(PROCESS_RAIL_Z - PROCESS_RAIL_SIZE/2, 6)
# z joint travel: touch the supported wafer, never the support underneath it.
PROCESS_CONTACT_Z = round(-(PROCESS_CUP_REST_BOTTOM - (SUPPORT_TOP + WAFER_THICKNESS)), 6)
PROCESS_TRANSIT_Z = 0.
PROCESS_TRANSIT_WAFER_Z = round(SUPPORTED_WAFER_Z + PROCESS_LIFT_STROKE, 6)


# Published facility dimensions.  Every room is 4 ft x 3 ft and adjacent
# room walls are separated by a clear 1 ft corridor.
ROOM_LENGTH = 1.2192
ROOM_DEPTH = 0.9144
ROOM_HEIGHT = 1.2192
CORRIDOR_WIDTH = 0.3048
HALF_LENGTH = ROOM_LENGTH / 2
HALF_DEPTH = ROOM_DEPTH / 2

SPACING = ROOM_LENGTH + CORRIDOR_WIDTH
ROOMS = tuple(Room(i+1, (i-5)*SPACING, .70, 1) for i in range(11))

ROOM_CODES = tuple(room.code for room in ROOMS)

MAIN_Y = ROOMS[0].route_y-CORRIDOR_WIDTH
START = (ROOMS[0].x-HALF_LENGTH-.35, MAIN_Y)


# ---------------------------------------------------------------------------
# Two-track black-tape network.
#
# Track 1 (main) keeps the published far lane; Track 2 (service) keeps the
# corridor lane that used to be reached by a raised U detour.  The lanes are
# joined by straight transverse rungs in front of every glovebox but clear of
# the door opening: a rung sits JUNCTION_OFFSET either side of the room centre,
# which is beyond the 0.3724 m door envelope and below the box front plane, so
# the robot never sweeps a door leaf or a box corner while it turns.
# ---------------------------------------------------------------------------

TRACK_1_Y = MAIN_Y
TRACK_2_Y = ROOMS[0].route_y
LANE_PITCH = TRACK_2_Y - TRACK_1_Y
JUNCTION_OFFSET = .45
DECISION_OFFSET = .20
MARKER_LEAD = .25
MARKER_LATERAL = .105
MARKER_HEIGHT = .27
MARKER_SIZE = .20
TURN_X = ROOMS[-1].x + HALF_LENGTH + .35
# Both guillotine doors sit in the +y front wall, so the stop that faces a door
# is square to the wall.  Aligning to this heading instead of a slightly
# off-axis handoff keeps the extended wand envelope clear of the closed leaf.
DOOR_APPROACH_HEADING = math.pi/2


def entry_junction_x(room):
    return room.x - JUNCTION_OFFSET


def exit_junction_x(room):
    return room.x + JUNCTION_OFFSET


@dataclass(frozen=True)
class Edge:
    """One finite, monotone route leg the FSM can select."""

    name: str
    kind: str
    track: str
    start: str
    end: str
    points: tuple

    @property
    def length(self):
        return polyline_length(self.points)


def polyline_length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def polyline_project(points, x, y):
    """Return (distance along the polyline, perpendicular deviation)."""

    best = (float('inf'), 0.)
    accumulated = 0.

    for (ax, ay), (bx, by) in zip(points, points[1:]):
        dx = bx - ax
        dy = by - ay
        length = math.hypot(dx, dy)

        if length <= 0.:
            continue

        t = max(0., min(1., ((x-ax)*dx + (y-ay)*dy)/(length*length)))
        error = math.hypot(x-ax-t*dx, y-ay-t*dy)

        if error < best[0]:
            best = (error, accumulated + t*length)

        accumulated += length

    return best[1], best[0]


def polyline_point(points, distance):
    """Point at an along-polyline distance, clamped to the leg."""

    remaining = max(0., min(polyline_length(points), distance))

    for a, b in zip(points, points[1:]):
        segment = math.dist(a, b)
        if segment and remaining <= segment:
            ratio = remaining/segment
            return a[0]+ratio*(b[0]-a[0]), a[1]+ratio*(b[1]-a[1])
        remaining -= segment

    return points[-1]


def polyline_heading(points, distance):
    """Tangent heading of a leg at an along-polyline distance."""

    remaining = max(0., min(polyline_length(points), distance))

    for a, b in zip(points, points[1:]):
        segment = math.dist(a, b)
        if segment and remaining <= segment:
            return math.atan2(b[1]-a[1], b[0]-a[0])
        remaining -= segment

    a, b = points[-2], points[-1]
    return math.atan2(b[1]-a[1], b[0]-a[0])


def _edge(name, kind, track, start, end, *points):
    return Edge(name, kind, track, start, end, tuple(points))


EDGES = {}


def _register(edge):
    EDGES[edge.name] = edge
    return edge


MAIN_EDGES = []      # Track 1 approach legs, one per glovebox
SERVICE_EDGES = []   # Track 2 legs, one per glovebox
ENTRY_RUNGS = []     # main -> service
EXIT_RUNGS = []      # service -> main

_previous_x = START[0]
for _room in ROOMS:
    _entry_x, _exit_x = entry_junction_x(_room), exit_junction_x(_room)
    MAIN_EDGES.append(_register(_edge(
        f'main_{_room.number:02d}', 'main', 'TRACK_1', f'main_before_{_room.number:02d}',
        f'main_junction_{_room.number:02d}',
        (_previous_x, TRACK_1_Y), (_entry_x, TRACK_1_Y))))
    _previous_x = _exit_x
    ENTRY_RUNGS.append(_register(_edge(
        f'rung_in_{_room.number:02d}', 'rung_in', 'TRACK_2', f'main_junction_{_room.number:02d}',
        f'service_junction_{_room.number:02d}', (_entry_x, TRACK_1_Y), (_entry_x, TRACK_2_Y))))
    SERVICE_EDGES.append(_register(_edge(
        f'service_{_room.number:02d}', 'service', 'TRACK_2', f'service_junction_{_room.number:02d}',
        f'service_return_{_room.number:02d}', (_entry_x, TRACK_2_Y), (_exit_x, TRACK_2_Y))))
    EXIT_RUNGS.append(_register(_edge(
        f'rung_out_{_room.number:02d}', 'rung_out', 'TRACK_1', f'service_return_{_room.number:02d}',
        f'main_return_{_room.number:02d}', (_exit_x, TRACK_2_Y), (_exit_x, TRACK_1_Y))))

TAIL_EDGE = _register(_edge('main_tail', 'main', 'TRACK_1', 'main_return_11', 'turnaround',
                            (_previous_x, TRACK_1_Y), (TURN_X, TRACK_1_Y)))
HOME_EDGE = _register(_edge('main_home', 'main', 'TRACK_1', 'turnaround', 'home',
                            (TURN_X, TRACK_1_Y), (START[0], TRACK_1_Y)))

# Physical black tape, used by the world generator and the IR probe model.
TAPE_SEGMENTS = (
    ((START[0], TRACK_1_Y), (TURN_X, TRACK_1_Y)),
    ((entry_junction_x(ROOMS[0]), TRACK_2_Y), (exit_junction_x(ROOMS[-1]), TRACK_2_Y)),
    *[((entry_junction_x(room), TRACK_1_Y), (entry_junction_x(room), TRACK_2_Y))
      for room in ROOMS],
    *[((exit_junction_x(room), TRACK_1_Y), (exit_junction_x(room), TRACK_2_Y))
      for room in ROOMS],
)

# Legacy plot helper: the continuous Track 1 polyline.
PATH = ((START[0], TRACK_1_Y), (TURN_X, TRACK_1_Y))


def edge_project(edge, x, y):
    """Return (distance along the selected edge, deviation from it)."""

    return polyline_project(edge.points, x, y)


def edge_point(edge, distance):
    return polyline_point(edge.points, distance)


def edge_heading(edge, distance):
    return polyline_heading(edge.points, distance)


def tape_distance(x, y):
    """Distance from a point to the nearest physical black tape."""

    return min(polyline_project(segment, x, y)[1] for segment in TAPE_SEGMENTS)


def main_edge(room):
    return MAIN_EDGES[room.number-1]


def service_edge(room):
    return SERVICE_EDGES[room.number-1]


def entry_rung(room):
    return ENTRY_RUNGS[room.number-1]


def exit_rung(room):
    return EXIT_RUNGS[room.number-1]


def entry_decision_stop(room):
    """Track 1 stop where the entry tag is read *before* the junction."""

    return main_edge(room).length-DECISION_OFFSET


def room_stop(room, phase):
    """
    Return a service-lane stop distance near a room door location.

    Entry/exit targets remain outside the enclosure instead of placing
    the main transport robot directly on the wall.
    """

    if phase in ('entry', 'exit'):
        return room.door_x(phase)-entry_junction_x(room)
    return room.x-entry_junction_x(room)


def marker_pose(room, side):
    """World pose of the AprilTag sign the forward camera must read."""

    if side == 'entry':
        junction = entry_junction_x(room)
        across = TRACK_1_Y+MARKER_LATERAL
    else:
        junction = exit_junction_x(room)
        across = TRACK_2_Y+MARKER_LATERAL
    return junction+MARKER_LEAD, across, MARKER_HEIGHT, math.pi


def tag_decision_stop(room, side):
    """Pose where the FSM reads (entry/exit) tag: (x, y, heading)."""

    if side == 'entry':
        edge = main_edge(room)
        return (*edge_point(edge, entry_decision_stop(room)), 0.)
    edge = service_edge(room)
    return (*edge_point(edge, room_stop(room, 'exit')), 0.)


def route_plan():
    """Ordered selected edges of the full eleven-glovebox mission."""

    plan = []
    for room in ROOMS:
        plan.extend([main_edge(room).name, entry_rung(room).name,
                     service_edge(room).name, exit_rung(room).name])
    plan.extend([TAIL_EDGE.name, HOME_EDGE.name])
    return tuple(plan)


MISSION_LENGTH = sum(EDGES[name].length for name in route_plan())


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


def horizontal_door_clear(x, y, heading, door_y, extension=0.0):
    """Check the oriented Raspbot and extended wand against a horizontal door."""
    ys = [y + a*math.sin(heading) + b*math.cos(heading)
          for a in (-.105, max(.145, extension+.016)) for b in (-.09, .09)]
    return max(ys) < door_y-.005 or min(ys) > door_y+.005
