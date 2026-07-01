#!/usr/bin/env python3

import math
import re
import time

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Float64


class ImuYawNode(Node):
    def __init__(self):
        super().__init__('imu_yaw_node')

        self.declare_parameter('ports', ['/dev/ttyACM0', '/dev/ttyACM1'])
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('yaw_is_bearing', True)
        self.declare_parameter('yaw_offset_deg', 0.0)

        self.ports = list(self.get_parameter('ports').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.yaw_is_bearing = bool(self.get_parameter('yaw_is_bearing').value)
        self.yaw_offset = math.radians(
            float(self.get_parameter('yaw_offset_deg').value)
        )

        self.yaw_pub = self.create_publisher(Float64, '/imu/yaw', 10)
        self.yaw_deg_pub = self.create_publisher(Float64, '/imu/yaw_deg', 10)

        self.ser = self.open_serial()
        time.sleep(2.0)

        self.create_timer(0.01, self.read_imu)
        self.get_logger().info('IMU yaw node started: serial -> /imu/yaw')

    def open_serial(self):
        for port in self.ports:
            try:
                ser = serial.Serial(port, self.baudrate, timeout=0.02)
                self.get_logger().info(f'Connected IMU on {port}')
                return ser
            except Exception as exc:
                self.get_logger().warn(f'Cannot open {port}: {exc}')

        raise RuntimeError('Arduino/IMU serial port not found')

    def read_imu(self):
        try:
            line = self.ser.readline().decode(errors='replace').strip()
        except Exception as exc:
            self.get_logger().warn(f'IMU serial read error: {exc}')
            return

        if not line:
            return

        yaw_deg = self.extract_yaw_deg(line)
        if yaw_deg is None:
            return

        yaw_rad = math.radians(yaw_deg)
        if self.yaw_is_bearing:
            yaw_rad = math.pi / 2.0 - yaw_rad

        yaw_rad = self.normalize_angle(yaw_rad + self.yaw_offset)

        yaw_msg = Float64()
        yaw_msg.data = yaw_rad
        self.yaw_pub.publish(yaw_msg)

        yaw_deg_msg = Float64()
        yaw_deg_msg.data = math.degrees(yaw_rad)
        self.yaw_deg_pub.publish(yaw_deg_msg)

        self.get_logger().info(
            f'IMU yaw={math.degrees(yaw_rad):.2f} deg',
            throttle_duration_sec=1.0,
        )

    @staticmethod
    def extract_yaw_deg(line):
        match = re.search(r'YAW\s*:\s*(-?\d+(?:\.\d+)?)', line, re.IGNORECASE)
        if match:
            return float(match.group(1))

        try:
            return float(line)
        except ValueError:
            return None

    @staticmethod
    def normalize_angle(angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuYawNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
