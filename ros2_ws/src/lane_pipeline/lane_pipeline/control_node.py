"""Control node (STAGE 3: steering).

Subscribes to /lane_offset and publishes the heading-correction angle in
degrees on /steering_cmd as a std_msgs/Float32. Positive = steer left,
negative = steer right. On a real vehicle this would map onto an
ackermann_msgs/AckermannDriveStamped command instead.
"""

import rclpy
from lane_interfaces.msg import LaneOffset
from rclpy.node import Node
from std_msgs.msg import Float32

from lane_pipeline.lane_detection_core import SteeringController


class ControlNode(Node):

    def __init__(self):
        super().__init__('control_node')

        self.declare_parameter('kp', 1.0)
        self.controller = SteeringController(kp=self.get_parameter('kp').value)

        self.subscription = self.create_subscription(
            LaneOffset, 'lane_offset', self.on_lane_offset, 10)
        self.steering_pub = self.create_publisher(Float32, 'steering_cmd', 10)

    def on_lane_offset(self, msg: LaneOffset):
        lane_center_x = msg.lane_center_x if msg.detected else None
        angle = self.controller.compute_steering(
            msg.frame_width, msg.frame_height, lane_center_x)

        cmd = Float32()
        cmd.data = float(angle)
        self.steering_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
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
