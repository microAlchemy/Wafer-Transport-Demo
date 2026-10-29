import importlib.util
import math
from pathlib import Path
import unittest

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'plot_frequency_response.py'
SPEC = importlib.util.spec_from_file_location('frequency_response', SCRIPT)
frequency = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(frequency)


class FrequencyResponseTests(unittest.TestCase):
    def test_camera_geometry_and_loop_are_finite(self):
        lookahead, scale = frequency.camera_geometry()
        self.assertGreater(lookahead, 0)
        self.assertGreater(scale, 0)
        values = frequency.loop_response(np.array([0.01, 0.1, 1.0]),
                                         lookahead=lookahead, camera_scale=scale,
                                         **frequency.DEFAULTS)
        self.assertTrue(all(math.isfinite(value.real) and math.isfinite(value.imag)
                            for value in values))
        self.assertGreater(abs(values[0]), abs(values[-1]))


if __name__ == '__main__':
    unittest.main()
