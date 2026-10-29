import importlib
import math
import sys
import time
import types
import unittest
from unittest.mock import patch

from raspbot_chassis.pid import PID


class PIDTests(unittest.TestCase):
    def test_positive_camera_error_turns_right(self):
        pid = PID(kp=0.06, ki=0.0, kd=0.008)
        correction = pid.update(0.5, 0.05)
        self.assertGreater(correction, 0)
        left, right = 0.10 + correction, 0.10 - correction
        self.assertLess((right - left) / 0.13583, 0)

    def test_reset_and_zero_dt(self):
        pid = PID(0.06, 0.0, 0.008)
        pid.update(0.5, 0.0)
        pid.update(0.4, 0.0)
        pid.reset()
        self.assertIsNone(pid.prev_error)
        self.assertEqual(pid.integral, 0)
        self.assertTrue(math.isfinite(pid.update(0.2, 0.0)))


class HardwareTests(unittest.TestCase):
    def test_drive_mixing_and_stale_camera_stop(self):
        class Twist:
            def __init__(self):
                self.linear = types.SimpleNamespace(x=0.0)
                self.angular = types.SimpleNamespace(z=0.0)

        class Float32:
            def __init__(self, data=0.0):
                self.data = data

        geometry = types.ModuleType('geometry_msgs')
        geometry.msg = types.ModuleType('geometry_msgs.msg')
        geometry.msg.Twist = Twist
        standard = types.ModuleType('std_msgs')
        standard.msg = types.ModuleType('std_msgs.msg')
        standard.msg.Float32 = Float32
        navigation = types.ModuleType('nav_msgs')
        navigation.msg = types.ModuleType('nav_msgs.msg')
        navigation.msg.Odometry = type('Odometry', (), {})
        with patch.dict(sys.modules, {'geometry_msgs': geometry,
                                      'geometry_msgs.msg': geometry.msg,
                                      'std_msgs': standard,
                                      'std_msgs.msg': standard.msg,
                                      'nav_msgs': navigation,
                                      'nav_msgs.msg': navigation.msg}):
            hardware_module = importlib.import_module('raspbot_chassis.hardware')
            published = []
            node = types.SimpleNamespace(
                create_publisher=lambda *_: types.SimpleNamespace(publish=published.append),
                create_subscription=lambda *_: None)
            hardware = hardware_module.Hardware(node)
            self.assertIsNone(hardware.read_line_error())
            hardware._on_error(Float32(0.3))
            self.assertEqual(hardware.read_line_error(), 0.3)
            hardware.set_motors(0.13, 0.07)
            self.assertAlmostEqual(published[-1].linear.x, 0.10)
            self.assertLess(published[-1].angular.z, 0)
            hardware.received_at = time.monotonic() - 1
            self.assertIsNone(hardware.read_line_error())
            hardware._on_error(Float32(float('nan')))
            self.assertIsNone(hardware.read_line_error())
            hardware._on_odom(types.SimpleNamespace(twist=types.SimpleNamespace(
                twist=types.SimpleNamespace(angular=types.SimpleNamespace(z=0.1)))))
            self.assertAlmostEqual(hardware.odom.twist.twist.angular.z, 0.1)
            hardware.cleanup()
            self.assertEqual(published[-1].linear.x, 0)
            self.assertEqual(published[-1].angular.z, 0)
            sys.modules.pop('raspbot_chassis.hardware', None)


if __name__ == '__main__':
    unittest.main()
