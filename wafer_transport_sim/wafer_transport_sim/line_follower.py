"""Downward camera path tracking; never owns the robot command topic."""
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from rclpy.qos import qos_profile_sensor_data
from .common import DemoNode, run
from .vision import line_command


class LineFollower(DemoNode):
    def __init__(self):
        super().__init__('line_follower')
        for name, default in [('linear_speed', 0.06), ('kp', 0.004),
                              ('max_angular_speed', 0.6), ('threshold', 55),
                              ('camera_crop_ratio', 0.35)]:
            self.param(name, default)
        self.bridge = CvBridge()
        self.command = self.create_publisher(Twist, '/line/cmd_vel', 10)
        self.valid = self.create_publisher(Bool, '/line/valid', 10)
        self.create_subscription(Image, '/down_camera/image_raw', self.image,
                                 qos_profile_sensor_data)

    def image(self, msg):
        result = None
        healthy = False
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            healthy = True
            p = lambda name: self.get_parameter(name).value
            result = line_command(image, p('linear_speed'), p('kp'),
                                  p('max_angular_speed'), p('threshold'),
                                  p('camera_crop_ratio'))
        except (CvBridgeError, ValueError, cv2.error) as exc:
            healthy = False
            self.get_logger().error(str(exc))
        self.send('/line/healthy', Bool, healthy)
        twist = Twist()
        if result is not None:
            twist.linear.x, twist.angular.z = result
        self.command.publish(twist)
        self.valid.publish(Bool(data=result is not None))


def main(args=None):
    run(LineFollower, args)
