"""Shared geometry for the ten-glovebox U-shaped facility (metres).

The facility dimensions, the process-cell transfer constants and the black-tape
network live in one module so the world generator, the IR probe model, the
transport controller and the tests cannot drift apart.

Two continuous U-shaped black-tape lanes surround the boxes.  Track 1 is the
outer lane; Track 2 is the service lane.  AprilTags authorize each rung choice.
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Room:
    number: int
    x: float
    y: float
    yaw: float

    def local_to_world(self, dx, dy):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.x+c*dx-s*dy, self.y+s*dx+c*dy

    @property
    def code(self):
        return f"GLOVEBOX_{self.number:02d}"

    @property
    def heading(self):
        return self.yaw

    @property
    def work_heading(self):
        return self.yaw + math.pi / 2

    @property
    def route_y(self):
        return self.lane_point(0., 2)[1]

    @property
    def table(self):
        """Blue process stage in the middle of the room."""
        return (*self.local_to_world(0., -HALF_DEPTH+HANDOFF_SETBACK), SUPPORTED_WAFER_Z)

    def handoff(self, side):
        """Wafer support just inside the entry or exit guillotine door."""
        offset = -HANDOFF_OFFSET if side == 'entry' else HANDOFF_OFFSET
        return (*self.local_to_world(offset, -HALF_DEPTH+HANDOFF_SETBACK), SUPPORTED_WAFER_Z)

    def handoff_heading(self, side):
        return self.work_heading

    def process_joint(self, axis):
        return f"room_{self.number}_process_{axis}"

    def process_attachment(self):
        return f"process_{self.number}"

    def tag(self, side):
        return f"GLOVEBOX_{self.number:02d}_{side.upper()}"

    def door_x(self, side):
        return self.door_position(side)[0]

    def door_position(self, side):
        offset = -HANDOFF_OFFSET if side == 'entry' else HANDOFF_OFFSET
        return self.local_to_world(offset, -HALF_DEPTH)

    def lane_point(self, along, track):
        # Leave 8 mm beyond the nominal half corridor at the door: the robot's
        # front collision box then clears the 12 mm guillotine panel after an
        # in-place turn while wand reach stays below its 269 mm stroke.
        outward = CORRIDOR_WIDTH/2 + DOOR_CLEARANCE + (CORRIDOR_WIDTH if track == 1 else 0.)
        return self.local_to_world(along, -HALF_DEPTH-outward)

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
DOOR_CLEARANCE = .008
HALF_LENGTH = ROOM_LENGTH / 2
HALF_DEPTH = ROOM_DEPTH / 2

SPACING = ROOM_LENGTH + CORRIDOR_WIDTH
# Four along the top, two down the left, four along the bottom.  The local
# positive X axis is the direction of travel past each pair of doors.
ROOMS = tuple(
    [Room(i+1, x, 2.30, math.pi) for i, x in enumerate((2.286, .762, -.762, -2.286))] +
    [Room(5, -3.6576, .9144, -math.pi/2), Room(6, -3.6576, -.9144, -math.pi/2)] +
    [Room(i+7, x, -2.30, 0.) for i, x in enumerate((-2.286, -.762, .762, 2.286))]
)

ROOM_CODES = tuple(room.code for room in ROOMS)

TOP_MAIN_Y = ROOMS[0].lane_point(0., 1)[1]
TOP_SERVICE_Y = ROOMS[0].lane_point(0., 2)[1]
LEFT_MAIN_X = ROOMS[4].lane_point(0., 1)[0]
LEFT_SERVICE_X = ROOMS[4].lane_point(0., 2)[0]
BOTTOM_MAIN_Y = ROOMS[-1].lane_point(0., 1)[1]
BOTTOM_SERVICE_Y = ROOMS[-1].lane_point(0., 2)[1]
START = (ROOMS[0].x+HALF_LENGTH+.35, TOP_MAIN_Y)
FINISH = (ROOMS[-1].x+HALF_LENGTH+.35, BOTTOM_MAIN_Y)
MAIN_PATH = (START, (LEFT_MAIN_X, TOP_MAIN_Y), (LEFT_MAIN_X, BOTTOM_MAIN_Y), FINISH)
SERVICE_PATH = ((START[0], TOP_SERVICE_Y), (LEFT_SERVICE_X, TOP_SERVICE_Y),
                (LEFT_SERVICE_X, BOTTOM_SERVICE_Y), (FINISH[0], BOTTOM_SERVICE_Y))


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

TRACK_1_Y = TOP_MAIN_Y  # compatibility for the first (top) leg
TRACK_2_Y = TOP_SERVICE_Y
LANE_PITCH = CORRIDOR_WIDTH
JUNCTION_OFFSET = .45
DECISION_OFFSET = .20
MARKER_LEAD = .25
MARKER_LATERAL = .105
MARKER_HEIGHT = .27
MARKER_SIZE = .20
TURN_X = FINISH[0]
# Both guillotine doors sit in the +y front wall, so the stop that faces a door
# is square to the wall.  Aligning to this heading instead of a slightly
# off-axis handoff keeps the extended wand envelope clear of the closed leaf.
DOOR_APPROACH_HEADING = ROOMS[0].work_heading


def entry_junction_x(room):
    return room.lane_point(-JUNCTION_OFFSET, 1)[0]


def exit_junction_x(room):
    return room.lane_point(JUNCTION_OFFSET, 1)[0]


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


def polyline_slice(points, start, end):
    """Cut a monotone edge from a continuous track, retaining its corners."""
    assert 0 <= start < end <= polyline_length(points)+1e-8
    result = [polyline_point(points, start)]
    traversed = 0.
    for a, b in zip(points, points[1:]):
        traversed += math.dist(a, b)
        if start+1e-8 < traversed < end-1e-8:
            result.append(b)
    result.append(polyline_point(points, end))
    return tuple(result)


def lane_distance(path, point):
    along, deviation = polyline_project(path, *point)
    assert deviation < 1e-6, (point, deviation)
    return along


EDGES = {}


def _register(edge):
    EDGES[edge.name] = edge
    return edge


MAIN_EDGES = []
SERVICE_EDGES = []
ENTRY_RUNGS = []
EXIT_RUNGS = []
_previous = 0.
for _room in ROOMS:
    _in_main = _room.lane_point(-JUNCTION_OFFSET, 1)
    _out_main = _room.lane_point(JUNCTION_OFFSET, 1)
    _in_service = _room.lane_point(-JUNCTION_OFFSET, 2)
    _out_service = _room.lane_point(JUNCTION_OFFSET, 2)
    _entry_distance = lane_distance(MAIN_PATH, _in_main)
    _exit_distance = lane_distance(MAIN_PATH, _out_main)
    MAIN_EDGES.append(_register(Edge(
        f'main_{_room.number:02d}', 'main', 'TRACK_1',
        f'main_before_{_room.number:02d}', f'main_junction_{_room.number:02d}',
        polyline_slice(MAIN_PATH, _previous, _entry_distance))))
    _previous = _exit_distance
    ENTRY_RUNGS.append(_register(_edge(
        f'rung_in_{_room.number:02d}', 'rung_in', 'TRACK_2',
        f'main_junction_{_room.number:02d}', f'service_junction_{_room.number:02d}',
        _in_main, _in_service)))
    SERVICE_EDGES.append(_register(_edge(
        f'service_{_room.number:02d}', 'service', 'TRACK_2',
        f'service_junction_{_room.number:02d}', f'service_return_{_room.number:02d}',
        _in_service, _out_service)))
    EXIT_RUNGS.append(_register(_edge(
        f'rung_out_{_room.number:02d}', 'rung_out', 'TRACK_1',
        f'service_return_{_room.number:02d}', f'main_return_{_room.number:02d}',
        _out_service, _out_main)))

TAIL_EDGE = _register(Edge('main_tail', 'main', 'TRACK_1', 'main_return_10',
                           'turnaround', polyline_slice(MAIN_PATH, _previous,
                                                        polyline_length(MAIN_PATH))))
HOME_EDGE = _register(Edge('main_home', 'main', 'TRACK_1', 'turnaround',
                           'home', tuple(reversed(MAIN_PATH))))

# Every segment is real black tape. The return reuses Track 1.
TAPE_SEGMENTS = tuple(zip(MAIN_PATH, MAIN_PATH[1:])) + \
    tuple(zip(SERVICE_PATH, SERVICE_PATH[1:])) + \
    tuple((edge.points[0], edge.points[-1]) for edge in (*ENTRY_RUNGS, *EXIT_RUNGS))
PATH = MAIN_PATH


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
    """Along-service distance to the door; the robot remains outside."""
    if phase == 'entry':
        return JUNCTION_OFFSET-HANDOFF_OFFSET
    if phase == 'exit':
        return JUNCTION_OFFSET+HANDOFF_OFFSET
    return JUNCTION_OFFSET


def marker_pose(room, side):
    """AprilTag facing an approaching robot on its selected track."""
    along = (-JUNCTION_OFFSET if side == 'entry' else JUNCTION_OFFSET) + MARKER_LEAD
    track = 1 if side == 'entry' else 2
    # Positive local Y points into the room, toward the camera's left on
    # the top and bottom legs after applying the room rotation.
    x, y = room.lane_point(along, track)
    x -= MARKER_LATERAL*math.sin(room.yaw)
    y += MARKER_LATERAL*math.cos(room.yaw)
    return x, y, MARKER_HEIGHT, room.yaw+math.pi


def tag_decision_stop(room, side):
    edge = main_edge(room) if side == 'entry' else service_edge(room)
    distance = entry_decision_stop(room) if side == 'entry' else room_stop(room, 'exit')
    return (*edge_point(edge, distance), edge_heading(edge, distance))


def route_plan():
    """Ordered selected edges of the full ten-glovebox mission."""

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


def oriented_door_clear(x, y, heading, room, side, extension=0.0):
    """Check the whole robot and wand against a room's rotated door plane."""
    dx, dy = room.door_position(side)
    inward = (-math.sin(room.yaw), math.cos(room.yaw))
    signed = []
    for forward in (-.105, max(.145, extension+.016)):
        for lateral in (-.09, .09):
            px = x+forward*math.cos(heading)-lateral*math.sin(heading)
            py = y+forward*math.sin(heading)+lateral*math.cos(heading)
            signed.append((px-dx)*inward[0]+(py-dy)*inward[1])
    return max(signed) < -.001 or min(signed) > .001
