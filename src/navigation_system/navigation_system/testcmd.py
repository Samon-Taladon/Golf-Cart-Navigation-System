#!/usr/bin/env python3

import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelTester(Node):

    def __init__(self):
        super().__init__("cmd_vel_tester")

        self.pub1 = self.create_publisher(Twist, "/cmd_vel_1", 10)
        self.pub2 = self.create_publisher(Twist, "/cmd_vel_2", 10)

        # เวลาที่ต้องหยุดส่งของแต่ละ topic
        self.stop1_until = 0.0
        self.stop2_until = 0.0

        self.timer = self.create_timer(0.1, self.publish_cmd)

        self.get_logger().info("Started CmdVel Tester")

    def publish_cmd(self):

        now = time.time()

        # ---------- cmd_vel_1 ----------
        msg1 = Twist()

        # linear.x
        if now < self.stop1_until:
            msg1.linear.x = 0.0
        else:
            msg1.linear.x = 1.0

        # angular.z ของ cmd_vel_1
        msg1.angular.z = 0.30

        self.pub1.publish(msg1)

        # ---------- cmd_vel_2 ----------
        msg2 = Twist()

        # linear.x
        if now < self.stop2_until:
            msg2.linear.x = 0.0
        else:
            msg2.linear.x = 1.0

        # angular.z ของ cmd_vel_2
        msg2.angular.z = -0.30

        self.pub2.publish(msg2)


def keyboard(node):

    while rclpy.ok():

        print("\n========================")
        print("1 -> STOP /cmd_vel_1 (10 sec)")
        print("2 -> STOP /cmd_vel_2 (10 sec)")
        print("q -> Quit")
        print("========================")

        key = input("Select : ")

        if key == "1":
            node.stop1_until = time.time() + 10
            print(">>> /cmd_vel_1 STOP 10 sec")

        elif key == "2":
            node.stop2_until = time.time() + 10
            print(">>> /cmd_vel_2 STOP 10 sec")

        elif key.lower() == "q":
            rclpy.shutdown()
            break


def main(args=None):

    rclpy.init(args=args)

    node = CmdVelTester()

    thread = threading.Thread(
        target=keyboard,
        args=(node,),
        daemon=True
    )
    thread.start()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()