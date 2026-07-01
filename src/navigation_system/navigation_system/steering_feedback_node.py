#!/usr/bin/env python3

import math

import can
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class SteeringFeedbackNode(Node):
    def __init__(self):
        super().__init__('steering_feedback_node')

        self.declare_parameter('channel', 'can0')
        self.declare_parameter('can_id', 0x07000001)
        self.declare_parameter('positive_raw_per_deg', 180.0)
        self.declare_parameter('negative_raw_per_deg', 320.0)
        self.declare_parameter('invert_sign', False)

        self.channel = self.get_parameter('channel').value
        self.can_id = int(self.get_parameter('can_id').value)
        self.positive_raw_per_deg = float(
            self.get_parameter('positive_raw_per_deg').value
        )
        self.negative_raw_per_deg = float(
            self.get_parameter('negative_raw_per_deg').value
        )
        self.invert_sign = bool(self.get_parameter('invert_sign').value)

        self.angle_pub = self.create_publisher(
            Float64,
            '/steering/angle_feedback',
            10,
        )
        self.angle_deg_pub = self.create_publisher(
            Float64,
            '/steering/angle_feedback_deg',
            10,
        )

        self.bus = can.interface.Bus(channel=self.channel, bustype='socketcan')
        self.create_timer(0.01, self.read_can)
        self.get_logger().info(
            f'Steering feedback node started on {self.channel}, '
            f'id=0x{self.can_id:X}'
        )

    def read_can(self):
        try:
            msg = self.bus.recv(timeout=0.0)
        except Exception as exc:
            self.get_logger().warn(f'CAN receive error: {exc}', throttle_duration_sec=1.0)
            return

        if msg is None or msg.arbitration_id != self.can_id or len(msg.data) < 2:
            return

        raw = (msg.data[0] << 8) | msg.data[1]
        if raw >= 32768:
            raw -= 65536
        if self.invert_sign:
            raw = -raw

        scale = self.positive_raw_per_deg if raw >= 0 else self.negative_raw_per_deg
        if abs(scale) < 1e-6:
            return

        angle_deg = raw / scale
        angle_rad = math.radians(angle_deg)

        angle_msg = Float64()
        angle_msg.data = angle_rad
        self.angle_pub.publish(angle_msg)

        angle_deg_msg = Float64()
        angle_deg_msg.data = angle_deg
        self.angle_deg_pub.publish(angle_deg_msg)

        self.get_logger().info(
            f'Steering feedback raw={raw} angle={angle_deg:.2f} deg',
            throttle_duration_sec=0.5,
        )


def main(args=None):
    rclpy.init(args=args)
    node = SteeringFeedbackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
