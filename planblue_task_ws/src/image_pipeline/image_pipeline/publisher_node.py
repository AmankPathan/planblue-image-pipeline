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
        # Synthetic image generation (RGB)
        image = np.random.randint(0, 256, (480, 640, 3), dtype=np.uint8)

        # Convert opencv to ROS image
        msg = self.bridge.cv2_to_imgmsg(image, encoding='rgb8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.frame_id)

        try:
            self.publisher_.publish(msg)
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.get_logger().info(f'Published frame {self.frame_id} at {timestamp:.6f}')
            self.frame_id += 1
        except Exception as e:
            self.get_logger().error(f'Failed to publish image: {e}')

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