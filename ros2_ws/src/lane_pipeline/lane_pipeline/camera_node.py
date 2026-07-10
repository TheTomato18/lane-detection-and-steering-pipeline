"""Camera node (STAGE 1: capture).

Reads a video file frame by frame and publishes each frame as a
sensor_msgs/Image on /camera/image_raw, paced by a timer at the video's
native frame rate. Stands in for a real camera driver: swapping in
hardware later only means replacing this node, the rest of the graph is
unchanged.
"""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('video_path', '')
        self.declare_parameter('fps', 0.0)   # 0 = use the video's own rate
        self.declare_parameter('loop', False)

        video_path = self.get_parameter('video_path').value
        if not video_path:
            raise RuntimeError(
                'The "video_path" parameter is required, e.g. '
                'ros2 launch lane_pipeline pipeline.launch.py video_path:=/path/to/video.mp4')

        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            raise RuntimeError(f'Could not open video file: {video_path}')

        self.loop = self.get_parameter('loop').value
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            Image, 'camera/image_raw', qos_profile_sensor_data)

        fps = self.get_parameter('fps').value or self.cap.get(cv2.CAP_PROP_FPS) or 30.0
        self.timer = self.create_timer(1.0 / fps, self.publish_frame)
        self.get_logger().info(f'Publishing {video_path} at {fps:.1f} fps')

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            if self.loop:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self.cap.read()
            if not ret:
                self.get_logger().info('End of video, stopping publisher')
                self.timer.cancel()
                # Give the downstream nodes a moment to process the last
                # published frame before this node (and, via the launch
                # file's shutdown handler, the rest of the pipeline) exits.
                self.create_timer(1.0, self._shutdown)
                return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        self.publisher.publish(msg)

    def _shutdown(self):
        if rclpy.ok():
            rclpy.shutdown()

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
