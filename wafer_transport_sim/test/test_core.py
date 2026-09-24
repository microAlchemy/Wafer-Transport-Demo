import math
from wafer_transport_sim.core import DetectionFilter, placed


def test_apriltag_filter_requires_consecutive_known_observations():
    valid = ('GLOVEBOX_01_ENTRY', 'GLOVEBOX_01_EXIT')
    detector = DetectionFilter(3, valid)
    assert detector.update(valid[0]) == ''
    assert detector.update(valid[0]) == ''
    assert detector.update(valid[0]) == valid[0]
    assert detector.update('UNKNOWN') == ''
    assert detector.update(valid[0]) == ''
    assert detector.update(valid[1]) == ''


def test_placement_checks_horizontal_and_vertical_distance():
    target = (1.145, 1.04, 0.134)
    assert placed(target, target)
    assert not placed((1.025, 1.04, 0.134), target)
    assert not placed((1.145, 1.04, 0.16), target)
    assert not placed((math.nan, 1.04, 0.134), target)
