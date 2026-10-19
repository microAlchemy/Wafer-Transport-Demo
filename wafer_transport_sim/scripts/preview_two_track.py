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

from wafer_transport_sim.four_room_layout import (ROOMS, ROOM_DEPTH, ROOM_LENGTH, START, FINISH,
                                                  MAIN_PATH, SERVICE_PATH, CORRIDOR_WIDTH,
                                                  marker_pose)

SCALE = 84            # pixels per metre
FLOOR = '#f2f5f7'
OUTSIDE = '#e0e7ed'
TRANSPORT = '#b9d3ec'
JUNCTION = '#c8d4df'
TAPE = '#050505'
BOX = '#d6eafa'
STEEL = '#ff9800'
DOOR = '#2377dc'
SIGN = '#d8d2c0'
DEFAULT = ROOT.parent / 'docs' / 'ten-glovebox-u-layout.png'


def drawn_rectangles(world_path=None):
    """Read the tape and underlay rectangles the world generator committed."""
    world = ET.parse(world_path or ROOT / 'worlds' / 'four_rooms.sdf')
    floor = world.find(".//model[@name='loop_floor']")
    drawn = {'tape': [], 'corridor': [], 'divider': []}
    for visual in floor.findall('.//visual'):
        name = visual.get('name', '')
        kind = ('tape' if name.startswith('tape_') else
                'corridor' if name.startswith('corridor_') else
                'divider' if name.startswith('steel_divider_') else None)
        if kind is None:
            continue
        pose = list(map(float, visual.findtext('pose').split()))
        size = list(map(float, visual.findtext('geometry/box/size').split()))
        drawn[kind].append(((pose[0], pose[1]), pose[5], size[0], size[1], name))
    return drawn


def render(path=None, world_path=None, scale=SCALE):
    """Render the committed geometry; return the image transform."""
    path = Path(path or DEFAULT)
    rectangles = drawn_rectangles(world_path)
    margin = .45
    left = min(room.x-ROOM_LENGTH/2 for room in ROOMS)-1.
    right = max(room.x+ROOM_LENGTH/2 for room in ROOMS)+1.
    bottom = min(room.y-ROOM_LENGTH/2 for room in ROOMS)-1.
    top = max(room.y+ROOM_LENGTH/2 for room in ROOMS)+1.
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
    for centre, yaw, length, thickness, name in rectangles['corridor']:
        index = int(name.rsplit('_', 1)[1])
        colour = OUTSIDE if index < 3 else TRANSPORT if index < 6 else JUNCTION
        rectangle(centre, yaw, length, thickness, colour)
    for room in ROOMS:
        rectangle((room.x, room.y), room.yaw, ROOM_LENGTH, ROOM_DEPTH, BOX)
        corners = [room.local_to_world(dx, dy) for dx, dy in
                   ((-ROOM_LENGTH/2, -ROOM_DEPTH/2), (ROOM_LENGTH/2, -ROOM_DEPTH/2),
                    (ROOM_LENGTH/2, ROOM_DEPTH/2), (-ROOM_LENGTH/2, ROOM_DEPTH/2))]
        draw.line([point(*xy) for xy in corners+[corners[0]]], fill=STEEL, width=5)
        for side in ('entry', 'exit'):
            offset = -.22 if side == 'entry' else .22
            a = room.local_to_world(offset-CORRIDOR_WIDTH/2, -ROOM_DEPTH/2)
            b = room.local_to_world(offset+CORRIDOR_WIDTH/2, -ROOM_DEPTH/2)
            draw.line([point(*a), point(*b)], fill=DOOR, width=5)
        for side in ('entry', 'exit'):
            sign_x, sign_y, _, _ = marker_pose(room, side)
            rectangle((sign_x, sign_y), room.yaw, .06, .20, SIGN)
    for centre, yaw, length, thickness, _ in rectangles['tape']:
        rectangle(centre, yaw, length, thickness, TAPE)
    for centre, yaw, length, thickness, _ in rectangles['divider']:
        rectangle(centre, yaw, length, max(thickness, .035), STEEL)

    draw.text(point(START[0]-.40, START[1]-.22), 'START / HOME', fill='#12303f')
    draw.text(point(FINISH[0]-.60, FINISH[1]-.22), 'turnaround', fill='#12303f')
    draw.text(point(-1.45, .55), 'GRAY: OUTSIDE CORRIDOR / TRACK 1', fill='#12303f')
    draw.text(point(-1.45, .30), 'BLUE: TRANSPORT CORRIDOR / TRACK 2', fill='#12303f')
    draw.text(point(-1.45, .05), 'ORANGE: OPAQUE STEEL / BLUE: DOORS', fill='#12303f')
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
