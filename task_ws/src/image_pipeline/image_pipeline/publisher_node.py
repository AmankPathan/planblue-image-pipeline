import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
from datetime import datetime

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        self.publisher_ = self.create_publisher(Image, '/image_stream', 10)
        self.bridge = CvBridge()
        self.frame_id = 0
        publish_rate_hz = 10.0
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_image)
        self.get_logger().info('Image publisher started')

    def publish_image(self):
        # Generate synthetic image (RGB)
        image = np.random.randint(
            0, 256, (480, 640, 3), dtype=np.uint8
        )

        # Get timestamp once
        now = self.get_clock().now()
        timestamp_msg = now.to_msg()

        # Convert to ROS Image message
        msg = self.bridge.cv2_to_imgmsg(image, encoding='rgb8')
        msg.header.stamp = timestamp_msg
        msg.header.frame_id = str(self.frame_id)

        self.publisher_.publish(msg)

        timestamp_sec = timestamp_msg.sec + timestamp_msg.nanosec * 1e-9
        self.get_logger().info(
            f'Published frame_id={self.frame_id} timestamp={timestamp_sec:.6f}'
        )

        self.frame_id += 1


def main():
    rclpy.init()
    node = ImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Image publisher stopped by user')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()