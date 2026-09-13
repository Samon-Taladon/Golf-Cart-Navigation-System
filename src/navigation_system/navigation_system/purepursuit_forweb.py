#!/usr/bin/env python3
"""Pure pursuit controller gated by commands from the zoo web page."""

import json
import math
from typing import Any, Dict

import rclpy
from std_msgs.msg import String

from navigation_system.purepursuit import PurePursuitController as BasePurePursuitController


class PurePursuitForWeb(BasePurePursuitController):
    """Run the reference controller only after the web UI sends start_navigation."""

    def __init__(self):
        super().__init__()

        self.declare_parameter('web_command_topic', '/web/navigation_command')
        self.declare_parameter('web_status_topic', '/web/navigation_status')
        self.declare_parameter('web_status_period', 0.5)

        self.web_command_topic = self.get_parameter('web_command_topic').value
        self.web_status_topic = self.get_parameter('web_status_topic').value
        self.web_status_period = float(self.get_parameter('web_status_period').value)

        self.web_navigation_active = False
        self.web_arrived = False
        self.web_destination: Dict[str, Any] = {}
        self.web_last_status_time = 0.0
        self.web_last_wait_log_time = 0.0

        self.web_command_sub = self.create_subscription(
            String,
            self.web_command_topic,
            self.web_command_callback,
            10,
        )
        self.web_status_pub = self.create_publisher(
            String,
            self.web_status_topic,
            10,
        )

        self.get_logger().info(
            'Pure Pursuit for web started: waiting for start_navigation on '
            f'{self.web_command_topic}'
        )

    def web_command_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'Invalid web command JSON: {msg.data!r}')
            return

        command = data.get('type') or data.get('command')

        if command in ('set_destination', 'destination'):
            self.web_destination = {
                'name': data.get('name') or data.get('destination') or 'destination',
                'lat': data.get('lat'),
                'lon': data.get('lon'),
            }
            self.web_navigation_active = False
            self.web_arrived = False
            self.reset_for_new_run()
            self.stop_vehicle()
            self.publish_web_status('destination_set')
            self.get_logger().info(
                f"Web destination selected: {self.web_destination.get('name')}"
            )
            return

        if command in ('start_navigation', 'start'):
            self.web_navigation_active = True
            self.web_arrived = False
            if self.path_completed:
                self.reset_for_new_run()
            self.publish_web_status('navigation_started')
            self.get_logger().info('Navigation enabled from web page')
            return

        if command in ('stop_navigation', 'stop'):
            self.web_navigation_active = False
            self.web_arrived = False
            self.stop_vehicle()
            self.publish_web_status('navigation_stopped')
            self.get_logger().info('Navigation stopped from web page')
            return

        self.get_logger().warn(f'Unknown web command: {command!r}')

    def control_loop(self) -> None:
        if not self.web_navigation_active:
            self.stop_vehicle()
            self.publish_web_status_throttled('idle')
            self.log_waiting_for_web_throttled()
            return

        super().control_loop()

        if self.path_completed:
            self.web_navigation_active = False
            self.web_arrived = True
            self.stop_vehicle()
            self.publish_web_status('arrived')
            return

        self.publish_web_status_throttled('navigation_update')

    def reset_for_new_run(self) -> None:
        self.path_completed = False
        self.nearest_index = 0
        self.target_index = 0
        self.last_target_index = 0
        self.last_steering = 0.0
        self.last_commanded_speed = 0.0

    def publish_web_status_throttled(self, event: str) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.web_last_status_time < self.web_status_period:
            return
        self.web_last_status_time = now
        self.publish_web_status(event)

    def publish_web_status(self, event: str) -> None:
        status = {
            'type': 'status',
            'event': event,
            'navigation_active': self.web_navigation_active,
            'arrived': self.web_arrived,
            'path_loaded': self.path_loaded,
            'have_odom': self.have_odom,
            'rtk_state': self.rtk_state,
            'fix_quality': self.fix_quality,
            'destination': self.web_destination,
            'distance_m': self.distance_to_path_goal(),
            'nearest_index': self.nearest_index,
            'target_index': self.target_index,
            'path_points': len(self.path),
            'cmd_linear_x': self.last_commanded_speed,
        }
        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False)
        self.web_status_pub.publish(msg)

    def distance_to_path_goal(self):
        if not self.have_odom or not self.path:
            return None
        goal_x, goal_y = self.path[-1]
        return math.hypot(goal_x - self.current_x, goal_y - self.current_y)

    def log_waiting_for_web_throttled(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.web_last_wait_log_time < max(2.0, self.debug_period):
            return
        self.web_last_wait_log_time = now
        self.get_logger().info('Waiting for web start button; vehicle held stopped')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PurePursuitForWeb()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.web_navigation_active = False
            node.stop_vehicle()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
