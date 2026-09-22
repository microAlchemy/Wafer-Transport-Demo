"""Raspbot V2 standard-kit geometry in metres; provenance in docs/raspbot_v2.md.

Published envelope/wheelbase/mass/FOV are separated from estimated wheel and
component dimensions. This is a dimensioned reconstruction, not vendor CAD.
"""
import math

LENGTH = .20874
WIDTH = .16583
HEIGHT = .12705
WHEELBASE = .11720
STOCK_MASS = .885
CAMERA_FOV = math.radians(110)
# Estimated from the manufacturer drawing; measure the real kit to calibrate.
WHEEL_RADIUS = .030
WHEEL_WIDTH = .030
TRACK = WIDTH - WHEEL_WIDTH
MOTOR_MAX_RAD_S = 245 * 2 * math.pi / 60
PAN_LIMIT = math.pi / 2
TILT_LIMIT = math.radians(55)
CAMERA_PIVOT = (.082, 0., .10155)
CAMERA_OFFSET = .030
MISSION_CAMERA_TILT = -.35
# Conservative mounted envelope: the extra downward camera extends the nose.
REAR_CLEARANCE = .105
FRONT_CLEARANCE = .145
HALF_WIDTH_CLEARANCE = .090


def wheel_speeds(vx, vy, wz):
    """Gazebo MecanumDrive convention, order FL, FR, rear L, rear R."""
    turn = wz * (WHEELBASE + TRACK) / 2
    return tuple(v / WHEEL_RADIUS for v in
                 (vx-vy-turn, vx+vy+turn, vx+vy-turn, vx-vy+turn))
