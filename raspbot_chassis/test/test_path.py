import importlib.util
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('generate_path', ROOT / 'scripts' / 'generate_path.py')
path = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(path)


class PathTests(unittest.TestCase):
    def test_connected_four_45_degree_bends_inside_floor(self):
        segments = path.path_segments()
        self.assertEqual([value for kind, value in path.ROUTE if kind == 'turn'],
                         [45, -45, -45, 45])
        self.assertEqual(len(segments), 5 + 4 * path.TURN_STEPS)
        for (_, end), (next_start, _) in zip(segments, segments[1:]):
            self.assertAlmostEqual(math.dist(end, next_start), 0, places=9)
        points = [p for segment in segments for p in segment]
        self.assertTrue(all(abs(x) < 2 and abs(y) < 2 for x, y in points))
        self.assertAlmostEqual(segments[-1][1][1], 0, places=9)

    def test_world_has_each_tape_piece_and_robot_before_first_turn(self):
        world = ET.parse(ROOT / 'worlds' / 'empty.sdf').getroot()
        visuals = world.findall(".//model[@name='black_line']/link/visual")
        self.assertEqual(len(visuals), len(path.path_segments()))
        for visual in visuals:
            size = [float(v) for v in visual.findtext('geometry/box/size').split()]
            self.assertAlmostEqual(size[1], path.TAPE_WIDTH)
        robot_x, robot_y, *_ = [float(v) for v in
                                world.findtext(".//include[name='transport_robot']/pose").split()]
        self.assertLess(path.START[0], robot_x)
        self.assertEqual(robot_y, path.START[1])


if __name__ == '__main__':
    unittest.main()
