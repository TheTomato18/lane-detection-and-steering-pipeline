"""Visualizer node (optional).

Subscribes to /lane_overlay and /steering_cmd, draws the steering
readout onto each overlay frame, and optionally shows a live preview
window and/or records the result to an mp4 — the same output the
standalone script produced. Purely a debugging/demo aid; the pipeline
runs without it.
"""

import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class VisualizerNode(Node):

    def __init__(self):
        super().__init__('visualizer_node')

        self.declare_parameter('show_preview', True)
        self.declare_parameter('output_path', '')   # empty = don't record
        self.declare_parameter('fps', 30.0)         # frame rate of the recording

        self.show_preview = self.get_parameter('show_preview').value
        self.output_path = self.get_parameter('output_path').value
        self.bridge = CvBridge()
        self.writer = None
        self.steering_angle = 0.0

        self.create_subscription(
            Image, 'lane_overlay', self.on_overlay, qos_profile_sensor_data)
        self.create_subscription(
            Float32, 'steering_cmd', self.on_steering, 10)

    def on_steering(self, msg: Float32):
        self.steering_angle = msg.data

    def on_overlay(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        angle = self.steering_angle
        direction = 'LEFT' if angle > 0.5 else 'RIGHT' if angle < -0.5 else 'STRAIGHT'
        cv2.putText(
            frame, f'Steering: {angle:+.2f} deg ({direction})',
            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2,
        )

        if self.output_path:
            if self.writer is None:
                output_dir = os.path.dirname(self.output_path)
                if output_dir:
                    os.makedirs(output_dir, exist_ok=True)
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                fps = self.get_parameter('fps').value
                self.writer = cv2.VideoWriter(
                    self.output_path, fourcc, fps, (msg.width, msg.height))
                self.get_logger().info(f'Recording to {self.output_path}')
            self.writer.write(frame)

        if self.show_preview:
            cv2.imshow('Lane Detection Pipeline', frame)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.writer is not None:
            self.writer.release()
            self.get_logger().info(f'Saved recording to {self.output_path}')
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
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
