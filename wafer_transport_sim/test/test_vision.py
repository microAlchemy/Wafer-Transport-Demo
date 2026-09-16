from pathlib import Path
import cv2
import numpy as np
import pytest
from wafer_transport_sim.vision import line_command, station_codes

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('letter', ['a', 'b', 'c'])
def test_committed_qr_decodes_with_opencv(letter):
    image = cv2.imread(str(ROOT / 'textures' / f'station_{letter}_qr.png'))
    assert image is not None
    assert 'STATION_' + letter.upper() in station_codes(cv2.QRCodeDetector(), image)
    small = cv2.resize(image, (145, 145), interpolation=cv2.INTER_AREA)
    canvas = np.full((480, 640, 3), 230, dtype=np.uint8)
    canvas[100:245, 350:495] = small
    assert 'STATION_' + letter.upper() in station_codes(cv2.QRCodeDetector(), canvas)


@pytest.mark.parametrize('x,sign', [(110, 1), (160, 0), (210, -1)])
def test_line_error_steers_toward_stripe(x, sign):
    image = np.full((240, 320, 3), 240, dtype=np.uint8)
    image[:, x-5:x+6] = 5
    result = line_command(image, .035, .004, .15, 55, .35)
    assert result is not None and result[0] == .035
    assert abs(result[1]) <= .15
    assert np.sign(result[1]) == sign


def test_blank_or_all_dark_image_is_lost_path():
    for brightness in (0, 255):
        image = np.full((240, 320, 3), brightness, dtype=np.uint8)
        assert line_command(image, .035, .004, .6, 55, .35) is None


@pytest.mark.parametrize('letter,robot_y,expected', [('a', .12, True),
                                                   ('b', .12, False),
                                                   ('c', .76, True)])
def test_qr_under_configured_camera_projection(letter, robot_y, expected):
    """Pinhole projection checks size/FOV feasibility, not Gazebo rendering."""
    import math
    import xml.etree.ElementTree as ET
    station = ET.parse(ROOT / 'models/stations/model.sdf')
    visual = station.find(f".//visual[@name='{letter}_qr']")
    x, y, z, roll, pitch, yaw = map(float, visual.findtext('pose').split())
    assert abs(roll - math.pi/2) < 1e-9 and pitch == 0
    normal_yaw = yaw - math.pi/2
    width, height = map(float, visual.findtext('geometry/plane/size').split())
    focal = 480 / math.tan(1.7 / 2)
    corners = []
    for dy, dz in [(-width/2, height/2), (width/2, height/2),
                   (width/2, -height/2), (-width/2, -height/2)]:
        world_x = x - math.sin(normal_yaw) * dy
        world_y = y + math.cos(normal_yaw) * dy
        depth = world_y - (robot_y + .095)
        corners.append([480 + focal * (world_x - 1.025) / depth,
                        360 - focal * (z + dz - .25) / depth])
    qr = cv2.imread(str(ROOT / 'textures' / f'station_{letter}_qr.png'))
    h, w = qr.shape[:2]
    matrix = cv2.getPerspectiveTransform(
        np.float32([[0, 0], [w-1, 0], [w-1, h-1], [0, h-1]]), np.float32(corners))
    projected = cv2.warpPerspective(qr, matrix, (960, 720), borderValue=(230, 230, 230))
    found = 'STATION_' + letter.upper() in station_codes(cv2.QRCodeDetector(), projected)
    assert found == expected
