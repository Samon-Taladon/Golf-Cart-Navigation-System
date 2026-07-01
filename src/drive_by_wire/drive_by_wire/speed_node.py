import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SpeedNode(Node):
    def __init__(self):
        super().__init__('speed_node')

        # Publisher to /final_cmd_vel (Arduino subscribes to this)
        self.publisher_ = self.create_publisher(Twist, 'final_cmd_vel', 10)

        # Subscription to joystick or teleop cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        self.get_logger().info('SpeedNode initialized and relaying to /final_cmd_vel')

    def cmd_vel_callback(self, msg: Twist):
        linear_x = msg.linear.x

        # Clamp value between -1.0 and 1.0
        if abs(linear_x) < 0.01:
            linear_x = 0.0
        else:
            linear_x = max(min(linear_x, 1.0), -1.0)

        # Create new Twist message
        new_msg = Twist()
        new_msg.linear.x = linear_x
        new_msg.linear.y = 0.0
        new_msg.linear.z = 0.0
        new_msg.angular.x = 0.0
        new_msg.angular.y = 0.0
        new_msg.angular.z = 0.0

        self.publisher_.publish(new_msg)
        self.get_logger().info(f'Republished to /final_cmd_vel → linear.x = {linear_x:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = SpeedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
