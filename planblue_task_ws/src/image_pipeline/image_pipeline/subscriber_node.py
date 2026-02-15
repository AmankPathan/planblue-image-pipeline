import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import json
from datetime import datetime, timezone

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.subscription = self.create_subscription(Image, '/image_stream', self.image_callback, 10)
        self.bridge = CvBridge()
        self.get_logger().info('Image subscriber started')

        # File handling for images
        self.output_dir = os.path.join(os.getcwd(), 'output', 'images')
        os.makedirs(self.output_dir, exist_ok=True)

        # File handling for metadata
        self.metadata_path = os.path.join(os.getcwd(), 'output', 'metadata.json')
        os.makedirs(os.path.dirname(self.metadata_path), exist_ok=True)

        # Reset output directory and metadata on startup
        self.get_logger().info('Resetting output images and metadata')

        # Clear images directory
        for filename in os.listdir(self.output_dir):
            file_path = os.path.join(self.output_dir, filename)
            if os.path.isfile(file_path):
                os.remove(file_path)

        # Reset metadata file
        with open(self.metadata_path, 'w') as f:
            json.dump([], f, indent=2)

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
            dt = datetime.fromtimestamp(timestamp_sec, tz=timezone.utc)
            timestamp_human = dt.strftime('%Y-%m-%d %H:%M:%S.%f')

            # Prepare overlay text
            overlay_text = f'Time: {timestamp_human} UTC'

            # Overlay timestamp on image
            cv2.putText(
                image,
                overlay_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 0),
                2,
                cv2.LINE_AA
            )

            # Build output filename
            filename = f'frame_{frame_id}.png'
            file_path = os.path.join(self.output_dir, filename)

            # Save image to disk
            success = cv2.imwrite(file_path, image)
            if not success:
                self.get_logger().error(f'Failed to write image to disk: {file_path}')
                return

            # Save metadata entry
            metadata_entry = {
                'frame_id': frame_id,
                'filename': filename,
                'timestamp': timestamp_sec,
                'timestamp_human_utc': timestamp_human
            }

            try:
                with open(self.metadata_path, 'r') as f:
                    try:
                        metadata = json.load(f)
                    except json.JSONDecodeError:
                        self.get_logger().warn('Metadata file corrupted, resetting')
                        metadata = []

                metadata.append(metadata_entry)

                with open(self.metadata_path, 'w') as f:
                    json.dump(metadata, f, indent=2)

            except Exception as e:
                self.get_logger().error(f'Failed to write metadata for frame_id={frame_id}: {e}')
                return

            self.get_logger().info(f'Saved frame_id={frame_id} image and metadata')

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