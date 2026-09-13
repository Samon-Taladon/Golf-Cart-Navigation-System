#!/usr/bin/env python3
"""Dead-reckoning odometry from measured speed and GNSS heading only."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster


class OdomFusionNode(Node):
    """Integrate wheel speed in the current GNSS-heading direction from (0, 0)."""

    def __init__(self):
        super().__init__('odom_fusion_node')
        self.declare_parameter('gnss_odom_topic', '/gnss/odom')
        self.declare_parameter('speed_topic', '/speed_status')
        self.declare_parameter('fused_odom_topic', '/fused_odom')
        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('update_rate_hz', 50.0)
        self.declare_parameter('max_path_poses', 5000)
        # Calibration factor for the speed reported on /speed_status.
        # A 5 m measured run reported as 6 m gives 5 / 6 = 0.833333.
        self.declare_parameter('speed_scale', 5.0 / 6.0)

        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.max_path_poses = int(self.get_parameter('max_path_poses').value)
        self.speed_scale = float(self.get_parameter('speed_scale').value)
        update_rate = float(self.get_parameter('update_rate_hz').value)

        # Start at a local origin. GNSS position is deliberately never used.
        self.x = 0.0
        self.y = 0.0
        self.measured_speed = 0.0
        self.heading = Quaternion(w=1.0)
        self.yaw = 0.0
        self.have_heading = False
        self.last_update_time = self.get_clock().now()
        self.path = Path()
        self.path.header.frame_id = self.frame_id

        gnss_topic = self.get_parameter('gnss_odom_topic').value
        speed_topic = self.get_parameter('speed_topic').value
        fused_topic = self.get_parameter('fused_odom_topic').value
        path_topic = self.get_parameter('path_topic').value
        self.create_subscription(Odometry, gnss_topic, self.gnss_callback, 10)
        self.create_subscription(Float64, speed_topic, self.speed_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, fused_topic, 10)
        self.path_pub = self.create_publisher(Path, path_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(1.0 / update_rate, self.update_odometry)
        self.get_logger().info(
            'Dead reckoning ready: x=y=0 at startup; heading=%s, speed=%s, '
            'speed_scale=%.6f' % (gnss_topic, speed_topic, self.speed_scale))

    def speed_callback(self, message):
        self.measured_speed = float(message.data)

    def gnss_callback(self, message):
        self.heading = message.pose.pose.orientation
        self.yaw = self.quaternion_to_yaw(self.heading)
        self.have_heading = True

    def update_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now
        # Avoid a false large motion after a pause or before the first heading.
        if dt <= 0.0 or dt > 0.5 or not self.have_heading:
            return

        calibrated_speed = self.measured_speed * self.speed_scale
        self.x += calibrated_speed * math.cos(self.yaw) * dt
        self.y += calibrated_speed * math.sin(self.yaw) * dt
        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = self.heading
        odom.twist.twist.linear.x = calibrated_speed
        self.odom_pub.publish(odom)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id
        pose.pose = odom.pose.pose
        self.path.header.stamp = stamp
        self.path.poses.append(pose)
        if len(self.path.poses) > self.max_path_poses:
            del self.path.poses[0:len(self.path.poses) - self.max_path_poses]
        self.path_pub.publish(self.path)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.frame_id
        transform.child_frame_id = self.child_frame_id
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation = self.heading
        self.tf_broadcaster.sendTransform(transform)

        self.get_logger().info(
            'x=%.3f y=%.3f speed=%.3f m/s (raw=%.3f) yaw=%.2f deg' %
            (self.x, self.y, calibrated_speed, self.measured_speed,
             math.degrees(self.yaw)))

    @staticmethod
    def quaternion_to_yaw(quaternion):
        siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = OdomFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
