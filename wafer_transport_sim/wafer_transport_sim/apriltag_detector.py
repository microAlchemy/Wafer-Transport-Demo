"""Detect the rendered AprilTag 36h11 markers at process-cell doors."""
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from rclpy.qos import qos_profile_sensor_data

from .common import DemoNode, run
from .core import DetectionFilter


TAG_CODES = {2*(room-1): f'GLOVEBOX_{room:02d}_ENTRY' for room in range(1, 11)}
TAG_CODES.update({2*(room-1)+1: f'GLOVEBOX_{room:02d}_EXIT' for room in range(1, 11)})


def detected_codes(image):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    _, ids, _ = detector.detectMarkers(image)
    return [] if ids is None else [TAG_CODES[int(i)] for i in ids.flatten() if int(i) in TAG_CODES]


class AprilTagDetector(DemoNode):
    def __init__(self):
        super().__init__('apriltag_detector')
        self.bridge = CvBridge()
        count = self.param('confirmation_frames', 3)
        self.filters = {code: DetectionFilter(count, TAG_CODES.values()) for code in TAG_CODES.values()}
        self.create_subscription(Image, '/camera/image_raw', self.image, qos_profile_sensor_data)

    def image(self, msg):
        try:
            seen = detected_codes(self.bridge.imgmsg_to_cv2(msg, 'bgr8'))
        except (CvBridgeError, cv2.error, AttributeError) as exc:
            self.get_logger().error(str(exc))
            self.send('/apriltag/healthy', Bool, False)
            return
        self.send('/apriltag/healthy', Bool, True)
        for code, confirmation in self.filters.items():
            if confirmation.update(code if code in seen else ''):
                self.send('/detected_apriltag', String, code)


def main(args=None):
    run(AprilTagDetector, args)
