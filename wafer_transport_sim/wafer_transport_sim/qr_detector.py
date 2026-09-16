"""Decode real rendered QR pixels, with no station-position shortcut."""
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from rclpy.qos import qos_profile_sensor_data
from .common import DemoNode, run
from .core import DetectionFilter, STATIONS
from .vision import station_codes


class QrDetector(DemoNode):
    def __init__(self):
        super().__init__('qr_detector')
        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()
        count = self.param('confirmation_frames', 3)
        self.param('minimum_qr_side_pixels', 70.0)
        self.filters = {station: DetectionFilter(count) for station in STATIONS}
        self.create_subscription(Image, '/camera/image_raw', self.image,
                                 qos_profile_sensor_data)

    def image(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            codes = station_codes(self.detector, image, self.get_parameter('minimum_qr_side_pixels').value)
        except (CvBridgeError, cv2.error) as exc:
            self.get_logger().error(str(exc))
            self.send('/qr/healthy', Bool, False)
            return
        self.send('/qr/healthy', Bool, True)
        for station, confirmation in self.filters.items():
            result = confirmation.update(station if station in codes else '')
            if result:
                self.send('/detected_station', String, result)


def main(args=None):
    run(QrDetector, args)
