#!/usr/bin/env python3
"""Build a continuous tape route with four smooth 45-degree turns."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET


WORLD = Path(__file__).resolve().parents[1] / 'worlds' / 'empty.sdf'
START = (-1.75, 0.0)
TAPE_WIDTH = 0.05
TURN_RADIUS = 0.45
TURN_STEPS = 10
# A left turn, two right turns, and a final left turn return to heading zero.
ROUTE = (
    ('straight', 0.70), ('turn', 45),
    ('straight', 0.40), ('turn', -45),
    ('straight', 0.40), ('turn', -45),
    ('straight', 0.40), ('turn', 45),
    ('straight', 0.40),
)


def path_segments():
    """Return connected (start, end) pairs along the tape centerline."""
    x, y = START
    heading = 0.0
    segments = []
    for kind, value in ROUTE:
        if kind == 'straight':
            end = (x + value * math.cos(heading), y + value * math.sin(heading))
            segments.append(((x, y), end))
            x, y = end
        else:
            turn = math.radians(value)
            sign = 1 if turn > 0 else -1
            origin_x, origin_y, initial_heading = x, y, heading
            for step in range(1, TURN_STEPS + 1):
                angle = turn * step / TURN_STEPS
                end = (
                    origin_x + TURN_RADIUS / sign *
                    (math.sin(initial_heading + angle) - math.sin(initial_heading)),
                    origin_y - TURN_RADIUS / sign *
                    (math.cos(initial_heading + angle) - math.cos(initial_heading)),
                )
                segments.append(((x, y), end))
                x, y = end
            heading += turn
    return segments


def generate(path=WORLD):
    tree = ET.parse(path)
    world = tree.getroot()
    line = world.find(".//model[@name='black_line']/link[@name='line_link']")
    for visual in line.findall('visual'):
        line.remove(visual)
    for index, (start, end) in enumerate(path_segments()):
        dx, dy = end[0] - start[0], end[1] - start[1]
        length = math.hypot(dx, dy) + 0.003  # overlap keeps the tape continuous
        visual = ET.SubElement(line, 'visual', name=f'tape_{index:02d}')
        ET.SubElement(visual, 'pose').text = (
            f'{(start[0] + end[0]) / 2} {(start[1] + end[1]) / 2} '
            f'0.001 0 0 {math.atan2(dy, dx)}'
        )
        box = ET.SubElement(ET.SubElement(visual, 'geometry'), 'box')
        ET.SubElement(box, 'size').text = f'{length} {TAPE_WIDTH} 0.001'
        material = ET.SubElement(visual, 'material')
        ET.SubElement(material, 'ambient').text = '0 0 0 1'
        ET.SubElement(material, 'diffuse').text = '0 0 0 1'
    tree.getroot().find(".//include[name='transport_robot']/pose").text = '-1.55 0 0 0 0 0'
    ET.indent(tree, space='  ')
    tree.write(path, encoding='unicode', xml_declaration=True)
    with Path(path).open('a') as stream:
        stream.write('\n')


if __name__ == '__main__':
    generate()
