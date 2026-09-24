#!/usr/bin/env python3
"""Static overhead preview of the generated two-track tape network.

The preview is drawn from the *generated* four_rooms.sdf (the loop-floor tape
and underlay rectangles) plus the shared layout constants, so it shows the
committed world rather than a hand-drawn sketch.  It is a top view rendered
with PIL, not a Gazebo camera image: it says nothing about contact dynamics,
lighting or rendered AprilTag visibility.
"""
import argparse
import math
import sys
from pathlib import Path
import xml.etree.ElementTree as ET
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from wafer_transport_sim.four_room_layout import (ROOMS, ROOM_DEPTH, ROOM_LENGTH, START, PROCESS_STAGE_DEPTH,
                                                  TRACK_1_Y, TRACK_2_Y, TURN_X, marker_pose)

SCALE = 84            # pixels per metre
FLOOR = '#f2f5f7'
CORRIDOR = '#c8ced4'
TAPE = '#050505'
GLASS = '#8fb6d0'
STAGE = '#0f4d7a'
SIGN = '#d8d2c0'
DEFAULT = ROOT.parent / 'docs' / 'agent-work' / 'two-track-fsm' / 'two-track-overhead.png'


def drawn_rectangles(world_path=None):
    """Read the tape and underlay rectangles the world generator committed."""
    world = ET.parse(world_path or ROOT / 'worlds' / 'four_rooms.sdf')
    floor = world.find(".//model[@name='loop_floor']")
    drawn = {'tape': [], 'corridor': []}
    for visual in floor.findall('.//visual'):
        name = visual.get('name', '')
        kind = ('tape' if name.startswith('tape_') else
                'corridor' if name.startswith('corridor_') else None)
        if kind is None:
            continue
        pose = list(map(float, visual.findtext('pose').split()))
        size = list(map(float, visual.findtext('geometry/box/size').split()))
        drawn[kind].append(((pose[0], pose[1]), pose[5], size[0], size[1]))
    return drawn


def render(path=None, world_path=None, scale=SCALE):
    """Render the committed geometry; return the image transform."""
    path = Path(path or DEFAULT)
    rectangles = drawn_rectangles(world_path)
    margin = .45
    left, right = START[0]-margin, TURN_X+margin
    bottom = TRACK_1_Y-.55
    top = ROOMS[0].y+ROOM_DEPTH/2+margin
    width, height = int(round((right-left)*scale)), int(round((top-bottom)*scale))

    def point(x, y):
        return ((x-left)*scale, (top-y)*scale)

    def rectangle(centre, yaw, length, thickness, colour):
        cos, sin = math.cos(yaw), math.sin(yaw)
        corners = []
        for along, across in ((-length/2, -thickness/2), (length/2, -thickness/2),
                              (length/2, thickness/2), (-length/2, thickness/2)):
            corners.append(point(centre[0]+along*cos-across*sin,
                                 centre[1]+along*sin+across*cos))
        draw.polygon(corners, fill=colour)

    image = Image.new('RGB', (width, height), FLOOR)
    draw = ImageDraw.Draw(image)
    for centre, yaw, length, thickness in rectangles['corridor']:
        rectangle(centre, yaw, length, thickness, CORRIDOR)
    for room in ROOMS:
        rectangle((room.x, room.y), 0., ROOM_LENGTH, ROOM_DEPTH, GLASS)
    for room in ROOMS:
        rectangle((room.x, room.table[1]), 0., .22, PROCESS_STAGE_DEPTH, STAGE)
        for side in ('entry', 'exit'):
            sign_x, sign_y, _, _ = marker_pose(room, side)
            rectangle((sign_x, sign_y), 0., .06, .20, SIGN)
    for centre, yaw, length, thickness in rectangles['tape']:
        rectangle(centre, yaw, length, thickness, TAPE)

    draw.text(point(START[0]-.42, TRACK_1_Y+.04), 'START', fill='#12303f')
    draw.text(point(START[0]-.42, TRACK_2_Y+.20), 'TRACK 2  service lane', fill='#12303f')
    draw.text(point(START[0]+.2, TRACK_1_Y-.20), 'TRACK 1  main lane', fill='#12303f')
    draw.text(point(TURN_X-.60, TRACK_1_Y-.20), 'turnaround', fill='#12303f')
    for room in ROOMS:
        draw.text(point(room.x-.34, room.y), f'GLOVEBOX {room.number:02d}', fill='#0d2233')
    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path)
    return dict(size=image.size, left=left, top=top, scale=scale)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default=str(DEFAULT))
    options = parser.parse_args()
    info = render(options.output)
    print(f'wrote {options.output} ({info["size"][0]}x{info["size"][1]})')


if __name__ == '__main__':
    main()
