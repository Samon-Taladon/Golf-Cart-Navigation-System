#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SteeringPublisher(Node):
    def __init__(self, rad):  # ✅ รับค่าผ่าน parameter แทน
        super().__init__('steering_publisher')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.steering_min = -0.873
        self.steering_max = 0.873
        self.rad = rad

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("✅ Publishing to /cmd_vel (RELIABLE)")

    def timer_callback(self):
        rad = max(min(self.rad, self.steering_max), self.steering_min)

        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = rad

        self.publisher_.publish(twist)
        self.get_logger().info(f"📤 cmd_vel → angular.z={rad:.3f}")


def main(args=None):
    # ✅ รับ input ก่อน rclpy.init()
    try:
        rad = float(input("กรอกค่าพวงมาลัย (rad): "))
    except:
        rad = 0.3

    rclpy.init(args=args)
    node = SteeringPublisher(rad)  # ✅ ส่งค่าเข้าไป
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# import time
# import random


# class SteeringPublisher(Node):
#     def __init__(self):
#         super().__init__('steering_publisher')
#         self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

#         # ช่วงพวงมาลัย (rad) ≈ -50° ถึง +50°
#         # self.steering_min = -0.873
#         # self.steering_max = 0.873
#         self.steering_min = -0.001
#         self.steering_max = 0.001

#         self.linear_speed = 0.5  # ความเร็วเดินหน้า (ปรับได้)

#     def publish_random_steering(self):
#         # สุ่มค่าพวงมาลัย
#         steering_angle_rad = random.uniform(
#             self.steering_min, self.steering_max
#         )

#         twist = Twist()
#         twist.linear.x = self.linear_speed
#         twist.angular.z = steering_angle_rad

#         self.publisher_.publish(twist)

#         self.get_logger().info(
#             f'Publishing cmd_vel | linear.x={twist.linear.x:.2f}, '
#             f'angular.z={twist.angular.z:.3f} rad'
#         )


# def main(args=None):
#     rclpy.init(args=args)
#     node = SteeringPublisher()

#     try:
#         while rclpy.ok():
#             node.publish_random_steering()
#             time.sleep(0.1)  # 10 Hz

#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()



# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist

# class SteeringPublisher(Node):
#     def __init__(self):
#         super().__init__('steering_publisher')

#         self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

#         self.rad = 0.8

#         # 🔥 ใช้ timer แทน loop
#         self.timer = self.create_timer(0.1, self.timer_callback)

#         self.get_logger().info("✅ Node Started")

#     def timer_callback(self):
#         twist = Twist()
#         twist.linear.x = 0.5
#         twist.angular.z = self.rad

#         self.publisher_.publish(twist)
#         self.get_logger().info("Publishing cmd_vel")

# def main(args=None):
#     rclpy.init(args=args)

#     node = SteeringPublisher()

#     rclpy.spin(node)   # 🔥 สำคัญที่สุด

#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()





# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# import random

# class SteeringPublisher(Node):
#     def __init__(self):
#         super().__init__('steering_publisher')

#         self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

#         self.steering_min = -0.873
#         self.steering_max = 0.873

#         # ✅ ค่า steering ปัจจุบัน
#         self.current_rad = 0.0

#         # ✅ เป้าหมายที่อยากเลี้ยวไป
#         self.target_rad = 0.0

#         # ✅ จำกัดความเร็วการหมุนพวงมาลัย (rad/sec)
#         self.max_steering_rate = 0.5  

#         # timer 0.1 sec
#         self.dt = 0.1
#         self.timer = self.create_timer(self.dt, self.timer_callback)

#         # เปลี่ยน target ทุก 2 วินาที
#         self.change_target_timer = self.create_timer(2.0, self.update_target)

#         self.get_logger().info("✅ Realistic steering simulation started")

#     def update_target(self):
#         # ✅ สุ่มเป้าหมายใหม่ (ไม่เปลี่ยนถี่เกิน)
#         self.target_rad = random.uniform(self.steering_min, self.steering_max)
#         self.get_logger().info(f"🎯 New target: {self.target_rad:.3f}")

#     def timer_callback(self):
#         # ✅ คำนวณ error
#         error = self.target_rad - self.current_rad

#         # ✅ จำกัดอัตราการเปลี่ยน (เหมือนหมุนพวงมาลัยจริง)
#         max_step = self.max_steering_rate * self.dt

#         if abs(error) > max_step:
#             step = max_step if error > 0 else -max_step
#         else:
#             step = error

#         self.current_rad += step

#         twist = Twist()
#         twist.linear.x = 1.5
#         twist.angular.z = self.current_rad

#         self.publisher_.publish(twist)
#         self.get_logger().info(
#             f"📤 steering={self.current_rad:.3f} → target={self.target_rad:.3f}"
#         )


# def main(args=None):
#     rclpy.init(args=args)
#     node = SteeringPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()