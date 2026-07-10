"""Perception node (STAGE 2: lane detection).

Subscribes to /camera/image_raw, runs the lane detector on each frame,
and publishes:
  - /lane_offset (lane_interfaces/LaneOffset): where the lane center sits
    in the frame, consumed by the control node.
  - /lane_overlay (sensor_msgs/Image): the frame with lane lines and
    offset markers drawn on it, for the visualizer or rqt_image_view.
"""

import rclpy
from cv_bridge import CvBridge
from lane_interfaces.msg import LaneOffset
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from lane_pipeline.lane_detection_core import LaneDetector


class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception_node')

        self.declare_parameter('smoothing', 0.2)
        self.declare_parameter('max_missed_frames', 5)

        self.detector = LaneDetector(
            smoothing=self.get_parameter('smoothing').value,
            max_missed_frames=self.get_parameter('max_missed_frames').value,
        )
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image, 'camera/image_raw', self.on_frame, qos_profile_sensor_data)
        self.offset_pub = self.create_publisher(LaneOffset, 'lane_offset', 10)
        self.overlay_pub = self.create_publisher(
            Image, 'lane_overlay', qos_profile_sensor_data)

    def on_frame(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        overlay, lane_center_x = self.detector.detect(frame)

        offset = LaneOffset()
        offset.header = msg.header
        offset.detected = lane_center_x is not None
        offset.lane_center_x = float(lane_center_x) if lane_center_x is not None else 0.0
        offset.frame_width = msg.width
        offset.frame_height = msg.height
        self.offset_pub.publish(offset)

        overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        overlay_msg.header = msg.header
        self.overlay_pub.publish(overlay_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
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
