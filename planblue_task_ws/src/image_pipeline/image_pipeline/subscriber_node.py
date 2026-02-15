import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.subscription = self.create_subscription(Image, '/image_stream', self.image_callback, 10)
        self.bridge = CvBridge()
        self.get_logger().info('Image subscriber started')

    def image_callback(self, msg: Image):
        try:
            # Convert image (even though we don't use it yet)
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            # Parse frame ID safely
            try:
                frame_id = int(msg.header.frame_id)
            except ValueError:
                self.get_logger().error(
                    f'Invalid frame_id received: {msg.header.frame_id}'
                )
                return

            # Extract timestamp
            stamp = msg.header.stamp
            timestamp_sec = stamp.sec + stamp.nanosec * 1e-9

            # Prepare overlay text and put on image 
            overlay_text = f'Timestamp: {timestamp_sec:.6f}'
            cv2.putText(
                image,
                overlay_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
                cv2.LINE_AA
            )

            self.get_logger().info(
                f'Received frame_id={frame_id} timestamp={timestamp_sec:.6f}'
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
        if rclpy.ok():
            rclpy.shutdown()