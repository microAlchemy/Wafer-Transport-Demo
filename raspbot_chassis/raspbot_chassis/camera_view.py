"""Live view and line offset topics from the simulated Gazebo camera."""

import os
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32

from .vision import detect


class CameraView(Node):
    def __init__(self):
        super().__init__('camera_view')
        self.declare_parameter('show_view', True)
        self.bridge = CvBridge()
        self.previous_center = None
        self.last_frame = None
        self.show_view = bool(self.get_parameter('show_view').value and os.environ.get('DISPLAY'))
        if self.get_parameter('show_view').value and not self.show_view:
            self.get_logger().warning('DISPLAY is unavailable; annotated images remain on /camera/annotated')
        self.annotated_pub = self.create_publisher(Image, '/camera/annotated', 10)
        self.visible_pub = self.create_publisher(Bool, '/line/visible', 10)
        self.offset_pub = self.create_publisher(Float32, '/line/offset_px', 10)
        self.normalized_pub = self.create_publisher(Float32, '/line/offset_normalized', 10)
        self.create_timer(0.5, self.check_camera)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, 10)

    def on_image(self, message):
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Camera image conversion failed: {exc}')
            return
        self.process(frame, message.header)

    def process(self, frame, header):
        self.last_frame = time.monotonic()
        result = detect(frame, self.previous_center)
        self.previous_center = result.center_px
        annotated = self.bridge.cv2_to_imgmsg(result.annotated, encoding='bgr8')
        annotated.header = header
        self.annotated_pub.publish(annotated)
        self.visible_pub.publish(Bool(data=result.visible))
        self.offset_pub.publish(Float32(data=result.offset_px))
        self.normalized_pub.publish(Float32(data=result.offset_normalized))
        if self.show_view:
            cv2.imshow('Gazebo Camera - Line Detection', result.annotated)
            cv2.imshow('Sobel Edge Detection', result.edges)
            cv2.imshow('Tape Mask', result.mask)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                path = Path.cwd() / 'line_result.png'
                cv2.imwrite(str(path), result.annotated)
                self.get_logger().info(f'Saved {path}')
            elif key == ord('q'):
                self.show_view = False
                cv2.destroyAllWindows()

    def publish_lost(self):
        self.previous_center = None
        self.visible_pub.publish(Bool(data=False))
        self.offset_pub.publish(Float32(data=float('nan')))
        self.normalized_pub.publish(Float32(data=float('nan')))

    def check_camera(self):
        if self.last_frame is None or time.monotonic() - self.last_frame > 1.0:
            self.publish_lost()

    def destroy_node(self):
        if self.show_view:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraView()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
