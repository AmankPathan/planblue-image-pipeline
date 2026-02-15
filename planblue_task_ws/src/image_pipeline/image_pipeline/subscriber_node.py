import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.subscription = self.create_subscription(Image, '/image_stream', self.image_callback, 10)
        self.bridge = CvBridge()
        self.get_logger().info('Image subscriber started')
        self.output_dir = os.path.join(os.getcwd(), 'output', 'images')
        os.makedirs(self.output_dir, exist_ok=True)

    def image_callback(self, msg: Image):
        try:
            # Convert ROS Image to OpenCV image
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            # Parse frame ID safely
            try:
                frame_id = int(msg.header.frame_id)
            except ValueError:
                self.get_logger().error(f'Invalid frame_id received: {msg.header.frame_id}')
                return

            # Extract timestamp
            stamp = msg.header.stamp
            timestamp_sec = stamp.sec + stamp.nanosec * 1e-9

            # Prepare overlay text
            overlay_text = f'Timestamp: {timestamp_sec:.6f}'

            # Overlay timestamp on image
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

            # Build output filename
            filename = f'frame_{frame_id}_{timestamp_sec:.6f}.png'
            file_path = os.path.join(self.output_dir, filename)

            # Save image to disk
            success = cv2.imwrite(file_path, image)
            if not success:
                self.get_logger().error(f'Failed to write image to disk: {file_path}')
                return

            self.get_logger().info(f'Saved frame_id={frame_id} to {file_path}')

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