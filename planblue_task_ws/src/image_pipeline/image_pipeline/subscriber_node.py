import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.subscription = self.create_subscription(Image, '/image_stream', self.image_callback, 10)
        self.bridge = CvBridge()
        self.get_logger().info('Image subscriber started')

    def image_callback(self, msg: Image):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            frame_id = msg.header.frame_id
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            self.get_logger().info(
                f'Received frame {frame_id} at {timestamp:.6f}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')

def main():
    rclpy.init()
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Image subscriber stopped by user')
    finally:
        node.destroy_node()