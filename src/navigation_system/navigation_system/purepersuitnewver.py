# #!/usr/bin/env python3

# import csv
# from datetime import datetime
# import math
# import os
# from typing import List, Optional, Tuple

# import rclpy
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from rclpy.node import Node
# from tf_transformations import euler_from_quaternion
# from visualization_msgs.msg import Marker

# try:
#     from ackermann_msgs.msg import AckermannDriveStamped
# except ImportError:
#     AckermannDriveStamped = None


# class PurePursuitController(Node):
#     """Pure Pursuit controller tuned for a slow, heavy autonomous golf cart."""

#     def __init__(self):
#         super().__init__('pure_pursuit_controller')

#         self.declare_parameter('path_file', self.default_path_file())
#         self.declare_parameter('odom_topic', '/odom')
#         self.declare_parameter('drive_topic', '/drive')
#         self.declare_parameter('cmd_vel_topic', '/cmd_vel')
#         self.declare_parameter('frame_id', 'odom')
#         self.declare_parameter('wheelbase', 1.67)
#         self.declare_parameter('target_speed', 1.5)
#         self.declare_parameter('min_lookahead', 3.0)
#         self.declare_parameter('max_lookahead', 8.0)
#         self.declare_parameter('lookahead_base', 2.0)
#         self.declare_parameter('lookahead_speed_gain', 1.5)
#         self.declare_parameter('max_steering_deg', 48.0)
#         self.declare_parameter('steering_filter_alpha', 0.16)
#         self.declare_parameter('max_steering_rate_deg_s', 22.0)
#         self.declare_parameter('goal_tolerance', 2.0)
#         self.declare_parameter('forward_dot_threshold', 0.0)
#         self.declare_parameter('nearest_search_backtrack', 8)
#         self.declare_parameter('control_period', 0.05)
#         self.declare_parameter('debug_period', 1.0)
#         self.declare_parameter('rtk_float_timeout', 1.0)
#         self.declare_parameter('rtk_no_fix_timeout', 1.0)
#         self.declare_parameter('float_speed_limit', 0.4)
#         self.declare_parameter('fix_speed_limit', 1.0)
#         self.declare_parameter('float_min_lookahead', 2.5)
#         self.declare_parameter('yaw_hold_steering_gain', 0.8)
#         self.declare_parameter('yaw_hold_max_steering_deg', 12.0)
#         self.declare_parameter('position_jump_margin', 0.35)
#         self.declare_parameter('min_jump_threshold', 0.45)
#         self.declare_parameter('enable_output_log', True)
#         self.declare_parameter('output_log_dir', self.default_output_log_dir())
#         self.declare_parameter('output_log_filename', 'navigation_%Y%m%d_%H%M%S.csv')

#         self.path_file = self.get_parameter('path_file').value
#         self.odom_topic = self.get_parameter('odom_topic').value
#         self.drive_topic = self.get_parameter('drive_topic').value
#         self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
#         self.frame_id = self.get_parameter('frame_id').value

#         self.wheelbase = float(self.get_parameter('wheelbase').value)
#         self.target_speed = float(self.get_parameter('target_speed').value)
#         self.min_lookahead = float(self.get_parameter('min_lookahead').value)
#         self.max_lookahead = float(self.get_parameter('max_lookahead').value)
#         self.lookahead_base = float(self.get_parameter('lookahead_base').value)
#         self.lookahead_speed_gain = float(
#             self.get_parameter('lookahead_speed_gain').value
#         )
#         self.max_steering = math.radians(
#             float(self.get_parameter('max_steering_deg').value)
#         )
#         self.steering_filter_alpha = float(
#             self.get_parameter('steering_filter_alpha').value
#         )
#         self.max_steering_rate = math.radians(
#             float(self.get_parameter('max_steering_rate_deg_s').value)
#         )
#         self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
#         self.forward_dot_threshold = float(
#             self.get_parameter('forward_dot_threshold').value
#         )
#         self.nearest_search_backtrack = int(
#             self.get_parameter('nearest_search_backtrack').value
#         )
#         self.control_period = float(self.get_parameter('control_period').value)
#         self.debug_period = float(self.get_parameter('debug_period').value)
#         self.rtk_float_timeout = float(
#             self.get_parameter('rtk_float_timeout').value
#         )
#         self.rtk_no_fix_timeout = float(
#             self.get_parameter('rtk_no_fix_timeout').value
#         )
#         self.float_speed_limit = float(
#             self.get_parameter('float_speed_limit').value
#         )
#         self.fix_speed_limit = float(self.get_parameter('fix_speed_limit').value)
#         self.float_min_lookahead = float(
#             self.get_parameter('float_min_lookahead').value
#         )
#         self.yaw_hold_steering_gain = float(
#             self.get_parameter('yaw_hold_steering_gain').value
#         )
#         self.yaw_hold_max_steering = math.radians(
#             float(self.get_parameter('yaw_hold_max_steering_deg').value)
#         )
#         self.position_jump_margin = float(
#             self.get_parameter('position_jump_margin').value
#         )
#         self.min_jump_threshold = float(
#             self.get_parameter('min_jump_threshold').value
#         )
#         self.enable_output_log = bool(
#             self.get_parameter('enable_output_log').value
#         )
#         self.output_log_dir = self.get_parameter('output_log_dir').value
#         self.output_log_filename = self.get_parameter('output_log_filename').value

#         self.current_x = 0.0
#         self.current_y = 0.0
#         self.current_yaw = 0.0
#         self.current_speed = 0.0
#         self.have_odom = False
#         self.rtk_state = 'NO_FIX'
#         self.fix_quality = 0
#         self.allow_waypoint_progress = False
#         self.position_is_predicted = False
#         self.float_started_time = None
#         self.last_odom_time = None
#         self.last_fix_time = None
#         self.last_fix_yaw = None
#         self.float_hold_yaw = None
#         self.last_commanded_speed = 0.0

#         self.path: List[Tuple[float, float]] = []
#         self.path_loaded = False
#         self.path_completed = False

#         self.nearest_index = 0
#         self.target_index = 0
#         self.last_target_index = 0
#         self.last_steering = 0.0
#         self.last_control_time = None
#         self.last_debug_time = self.get_clock().now()
#         self.output_log_file = None
#         self.output_log_writer = None
#         self.output_log_path = None

#         self.load_path()
#         self.setup_output_logger()

#         self.odom_sub = self.create_subscription(
#             Odometry,
#             self.odom_topic,
#             self.odom_callback,
#             10,
#         )

#         self.drive_pub = None
#         if AckermannDriveStamped is not None:
#             self.drive_pub = self.create_publisher(
#                 AckermannDriveStamped,
#                 self.drive_topic,
#                 10,
#             )
#         else:
#             self.get_logger().warn(
#                 'ackermann_msgs is not installed; publishing /cmd_vel only'
#             )

#         self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
#         self.target_marker_pub = self.create_publisher(
#             Marker,
#             '/target_waypoint_marker',
#             10,
#         )
#         self.nearest_marker_pub = self.create_publisher(
#             Marker,
#             '/nearest_waypoint_marker',
#             10,
#         )

#         self.create_timer(self.control_period, self.control_loop)

#         self.get_logger().info(
#             'Pure Pursuit controller started: '
#             f'path={self.path_file}, odom={self.odom_topic}, '
#             f'drive={self.drive_topic}, cmd_vel={self.cmd_vel_topic}'
#         )

#     @staticmethod
#     def default_path_file() -> str:
#         return '/home/inc/ros2_ws/src/navigation_system/logs13/path_smoothlog13_resampled_1m.csv'

#     @staticmethod
#     def default_output_log_dir() -> str:
#         return os.path.join(os.getcwd(), 'output_logs')

#     def setup_output_logger(self) -> None:
#         if not self.enable_output_log:
#             return

#         try:
#             os.makedirs(self.output_log_dir, exist_ok=True)
#             filename = datetime.now().strftime(self.output_log_filename)
#             if not filename.lower().endswith('.csv'):
#                 filename = f'{filename}.csv'
#             self.output_log_path = os.path.join(self.output_log_dir, filename)
#             self.output_log_file = open(self.output_log_path, 'w', newline='')
#             self.output_log_writer = csv.writer(self.output_log_file)
#             self.output_log_writer.writerow([
#                 'x',
#                 'y',
#                 'speed',
#                 'heading',
#                 'fix_quality',
#                 'steering_angle',
#             ])
#             self.output_log_file.flush()
#             self.get_logger().info(f'Output CSV log: {self.output_log_path}')
#         except Exception as exc:
#             self.output_log_file = None
#             self.output_log_writer = None
#             self.get_logger().error(f'Failed to open output CSV log: {exc}')

#     def write_output_log(self, commanded_speed: float, steering: float) -> None:
#         if self.output_log_writer is None:
#             return

#         try:
#             self.output_log_writer.writerow([
#                 f'{self.current_x:.6f}',
#                 f'{self.current_y:.6f}',
#                 f'{self.current_speed:.6f}',
#                 f'{math.degrees(self.current_yaw):.6f}',
#                 self.fix_quality,
#                 f'{math.degrees(steering):.6f}',
#             ])
#             self.output_log_file.flush()
#         except Exception as exc:
#             self.get_logger().warn(f'Failed to write output CSV log: {exc}')

#     def close_output_logger(self) -> None:
#         if self.output_log_file is None:
#             return

#         try:
#             self.output_log_file.flush()
#             self.output_log_file.close()
#         except Exception as exc:
#             self.get_logger().warn(f'Failed to close output CSV log: {exc}')
#         finally:
#             self.output_log_file = None
#             self.output_log_writer = None

#     def load_path(self) -> None:
#         self.path = []

#         try:
#             with open(self.path_file, 'r', newline='') as file:
#                 reader = csv.DictReader(file)
#                 if reader.fieldnames is None:
#                     raise ValueError('CSV file has no header')

#                 required_columns = {'x', 'y'}
#                 if not required_columns.issubset(set(reader.fieldnames)):
#                     raise ValueError('CSV header must contain x,y columns')

#                 for row_number, row in enumerate(reader, start=2):
#                     try:
#                         self.path.append((float(row['x']), float(row['y'])))
#                     except (TypeError, ValueError) as exc:
#                         raise ValueError(
#                             f'invalid x,y at CSV row {row_number}: {row}'
#                         ) from exc

#             if len(self.path) < 2:
#                 raise ValueError('path must contain at least 2 waypoints')

#             self.path_loaded = True
#             self.get_logger().info(f'Loaded {len(self.path)} waypoints')

#         except Exception as exc:
#             self.path_loaded = False
#             self.get_logger().error(f'Failed to load path: {exc}')

#     def odom_callback(self, msg: Odometry) -> None:
#         now = self.get_clock().now().nanoseconds / 1e9
#         msg_x = msg.pose.pose.position.x
#         msg_y = msg.pose.pose.position.y
#         msg_speed = self.get_odom_speed(msg)
#         q = msg.pose.pose.orientation
#         _, _, msg_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

#         self.fix_quality = self.get_fix_quality(msg)
#         new_rtk_state = self.classify_rtk_state(self.fix_quality)
#         previous_rtk_state = self.rtk_state
#         self.current_speed = msg_speed
#         self.current_yaw = msg_yaw

#         if new_rtk_state != 'FLOAT':
#             self.float_started_time = None
#             self.float_hold_yaw = None
#         elif previous_rtk_state != 'FLOAT' or self.float_started_time is None:
#             self.float_started_time = now
#             self.float_hold_yaw = (
#                 self.last_fix_yaw if self.last_fix_yaw is not None else msg_yaw
#             )

#         self.rtk_state = new_rtk_state
#         self.allow_waypoint_progress = False
#         self.position_is_predicted = False

#         if self.rtk_state == 'FIX':
#             if self.is_position_outlier(msg_x, msg_y, now):
#                 self.throttled_warn('Rejected RTK FIX position outlier')
#                 return

#             self.current_x = msg_x
#             self.current_y = msg_y
#             self.last_fix_time = now
#             self.last_odom_time = now
#             self.last_fix_yaw = msg_yaw
#             self.allow_waypoint_progress = True
#             self.have_odom = True
#             return

#         if self.rtk_state == 'FLOAT':
#             float_time = 0.0 if self.float_started_time is None else now - self.float_started_time
#             dt = self.control_period if self.last_odom_time is None else max(
#                 now - self.last_odom_time,
#                 self.control_period,
#             )
#             self.predict_position(dt)
#             self.last_odom_time = now
#             self.position_is_predicted = True
#             self.have_odom = True
#             if float_time > self.rtk_float_timeout:
#                 self.throttled_warn('RTK FLOAT yaw hold active')
#             return

#         self.stop_vehicle()
#         self.throttled_warn('No valid RTK position; vehicle stopped')

#     @staticmethod
#     def get_odom_speed(msg: Odometry) -> float:
#         linear = msg.twist.twist.linear
#         return math.sqrt(
#             linear.x * linear.x
#             + linear.y * linear.y
#             + linear.z * linear.z
#         )

#     def get_fix_quality(self, msg: Odometry) -> int:
#         try:
#             return int(round(msg.pose.covariance[0]))
#         except (TypeError, ValueError, IndexError):
#             return 0

#     @staticmethod
#     def classify_rtk_state(fix_quality: int) -> str:
#         if fix_quality == 4:
#             return 'FIX'
#         if fix_quality == 5:
#             return 'FLOAT'
#         return 'NO_FIX'

#     def predict_position(self, dt: float) -> None:
#         speed = max(
#             0.0,
#             min(abs(self.current_speed or self.last_commanded_speed), self.fix_speed_limit),
#         )
#         self.current_x += speed * math.cos(self.current_yaw) * dt
#         self.current_y += speed * math.sin(self.current_yaw) * dt

#     def is_position_outlier(self, x: float, y: float, now: float) -> bool:
#         if not self.have_odom or self.last_odom_time is None:
#             return False

#         if self.last_fix_time is None or now - self.last_fix_time > 2.0:
#             return False

#         dt = max(now - self.last_odom_time, self.control_period)
#         distance = math.hypot(x - self.current_x, y - self.current_y)
#         speed = max(abs(self.current_speed), abs(self.last_commanded_speed))
#         max_possible_move = max(
#             self.min_jump_threshold,
#             speed * dt + self.position_jump_margin,
#         )
#         return distance > max_possible_move

#     def control_loop(self) -> None:
#         if not self.path_loaded or self.path_completed:
#             return

#         if not self.have_odom:
#             self.throttled_warn('Waiting for /odom')
#             return

#         now = self.get_clock().now().nanoseconds / 1e9
#         if (
#             self.last_odom_time is not None
#             and now - self.last_odom_time > self.rtk_no_fix_timeout
#         ):
#             self.rtk_state = 'NO_FIX'
#             self.allow_waypoint_progress = False
#             self.stop_vehicle()
#             self.throttled_warn('Odometry stale; vehicle stopped')
#             return

#         if self.is_float_yaw_hold_active(now):
#             steering_raw = self.compute_yaw_hold_steering()
#             steering = self.smooth_steering(steering_raw)
#             self.publish_drive_command(steering)
#             self.debug_yaw_hold(steering_raw, steering)
#             return

#         lookahead = self.compute_dynamic_lookahead()
#         if not self.allow_waypoint_progress and self.target_index < len(self.path):
#             nearest_index = self.nearest_index
#             target_index = self.target_index
#         else:
#             nearest_index = self.find_nearest_forward_waypoint()

#             if nearest_index is None:
#                 if self.is_near_final_goal():
#                     self.path_completed = True
#                     self.stop_vehicle()
#                     self.get_logger().info('Path completed; vehicle stopped')
#                     return

#                 self.stop_vehicle()
#                 self.throttled_warn('No forward waypoint found; vehicle stopped')
#                 return

#             target_index = self.find_target_waypoint(nearest_index, lookahead)

#             if target_index is None:
#                 self.stop_vehicle()
#                 self.throttled_warn('No forward waypoint found; vehicle stopped')
#                 return

#         self.nearest_index = nearest_index
#         self.target_index = target_index
#         self.last_target_index = max(self.last_target_index, target_index)

#         target_x, target_y = self.path[target_index]
#         steering_raw = self.compute_pure_pursuit(target_x, target_y)
#         steering = self.smooth_steering(steering_raw)

#         if self.is_path_completed(target_index):
#             self.path_completed = True
#             self.stop_vehicle()
#             self.get_logger().info('Path completed; vehicle stopped')
#             return

#         self.publish_drive_command(steering)
#         self.publish_marker(
#             target_x,
#             target_y,
#             self.target_marker_pub,
#             0,
#             1.0,
#             0.0,
#             0.0,
#         )

#         nearest_x, nearest_y = self.path[nearest_index]
#         self.publish_marker(
#             nearest_x,
#             nearest_y,
#             self.nearest_marker_pub,
#             1,
#             0.0,
#             1.0,
#             0.0,
#         )

#         self.debug_log(lookahead, steering_raw, steering)

#     def compute_dynamic_lookahead(self) -> float:
#         lookahead = self.lookahead_base + self.lookahead_speed_gain * abs(
#             self.current_speed
#         )
#         if self.rtk_state != 'FIX':
#             lookahead = max(lookahead, self.float_min_lookahead)
#         return self.clamp(lookahead, self.min_lookahead, self.max_lookahead)

#     def find_nearest_forward_waypoint(self) -> Optional[int]:
#         heading_x = math.cos(self.current_yaw)
#         heading_y = math.sin(self.current_yaw)

#         start_index = max(0, self.last_target_index - self.nearest_search_backtrack)
#         best_index = None
#         best_distance = float('inf')

#         for index in range(start_index, len(self.path)):
#             waypoint_x, waypoint_y = self.path[index]
#             dx = waypoint_x - self.current_x
#             dy = waypoint_y - self.current_y
#             dot = heading_x * dx + heading_y * dy

#             if dot < self.forward_dot_threshold:
#                 continue

#             distance = math.hypot(dx, dy)
#             if distance < best_distance:
#                 best_distance = distance
#                 best_index = index

#         return best_index

#     def find_target_waypoint(
#         self,
#         nearest_index: int,
#         lookahead: float,
#     ) -> Optional[int]:
#         heading_x = math.cos(self.current_yaw)
#         heading_y = math.sin(self.current_yaw)

#         start_index = max(nearest_index, self.last_target_index)

#         for index in range(start_index, len(self.path)):
#             waypoint_x, waypoint_y = self.path[index]
#             dx = waypoint_x - self.current_x
#             dy = waypoint_y - self.current_y
#             dot = heading_x * dx + heading_y * dy

#             if dot < self.forward_dot_threshold:
#                 continue

#             if math.hypot(dx, dy) >= lookahead:
#                 return index

#         if self.is_near_final_goal():
#             return len(self.path) - 1

#         return None

#     def compute_pure_pursuit(self, target_x: float, target_y: float) -> float:
#         dx = target_x - self.current_x
#         dy = target_y - self.current_y
#         lookahead_distance = math.hypot(dx, dy)

#         if lookahead_distance < 0.01:
#             return 0.0

#         target_angle = math.atan2(dy, dx)
#         alpha = self.normalize_angle(target_angle - self.current_yaw)

#         steering = math.atan2(
#             2.0 * self.wheelbase * math.sin(alpha),
#             lookahead_distance,
#         )
#         return self.clamp(steering, -self.max_steering, self.max_steering)

#     def is_float_yaw_hold_active(self, now: float) -> bool:
#         if self.rtk_state != 'FLOAT' or self.float_started_time is None:
#             return False
#         return now - self.float_started_time > self.rtk_float_timeout

#     def compute_yaw_hold_steering(self) -> float:
#         target_yaw = (
#             self.float_hold_yaw
#             if self.float_hold_yaw is not None
#             else self.current_yaw
#         )
#         yaw_error = self.normalize_angle(target_yaw - self.current_yaw)
#         steering = self.yaw_hold_steering_gain * yaw_error
#         return self.clamp(
#             steering,
#             -self.yaw_hold_max_steering,
#             self.yaw_hold_max_steering,
#         )

#     def smooth_steering(self, steering: float) -> float:
#         now = self.get_clock().now()

#         if self.last_control_time is None:
#             dt = self.control_period
#         else:
#             dt = max(
#                 (now - self.last_control_time).nanoseconds / 1e9,
#                 self.control_period,
#             )

#         self.last_control_time = now

#         filtered = (
#             self.steering_filter_alpha * steering
#             + (1.0 - self.steering_filter_alpha) * self.last_steering
#         )

#         rate_limit = self.max_steering_rate
#         if self.rtk_state != 'FIX':
#             rate_limit *= 0.5

#         max_delta = rate_limit * dt
#         filtered = self.clamp(
#             filtered,
#             self.last_steering - max_delta,
#             self.last_steering + max_delta,
#         )
#         filtered = self.clamp(filtered, -self.max_steering, self.max_steering)
#         self.last_steering = filtered

#         return filtered

#     def publish_drive_command(self, steering: float) -> None:
#         speed = self.get_command_speed()

#         if self.drive_pub is not None:
#             drive_msg = AckermannDriveStamped()
#             drive_msg.header.stamp = self.get_clock().now().to_msg()
#             drive_msg.header.frame_id = self.frame_id
#             drive_msg.drive.speed = speed
#             drive_msg.drive.steering_angle = steering
#             self.drive_pub.publish(drive_msg)

#         cmd_vel = Twist()
#         cmd_vel.linear.x = speed
#         # The drive_by_wire steering node interprets angular.z as steering
#         # angle in radians, not yaw rate.
#         cmd_vel.angular.z = steering
#         self.cmd_vel_pub.publish(cmd_vel)
#         self.last_commanded_speed = speed
#         self.write_output_log(speed, steering)

#     def get_command_speed(self) -> float:
#         if self.rtk_state == 'FIX':
#             return min(self.target_speed, self.fix_speed_limit)
#         if self.rtk_state == 'FLOAT':
#             return min(self.target_speed, self.float_speed_limit)
#         return 0.0

#     def stop_vehicle(self) -> None:
#         self.last_steering = 0.0
#         self.last_commanded_speed = 0.0

#         if self.drive_pub is not None:
#             drive_msg = AckermannDriveStamped()
#             drive_msg.header.stamp = self.get_clock().now().to_msg()
#             drive_msg.header.frame_id = self.frame_id
#             drive_msg.drive.speed = 0.0
#             drive_msg.drive.steering_angle = 0.0
#             self.drive_pub.publish(drive_msg)

#         cmd_vel = Twist()
#         cmd_vel.linear.x = 0.0
#         cmd_vel.angular.z = 0.0
#         self.cmd_vel_pub.publish(cmd_vel)

#     def destroy_node(self) -> None:
#         self.close_output_logger()
#         super().destroy_node()

#     def is_path_completed(self, target_index: int) -> bool:
#         if target_index < len(self.path) - 1:
#             return False

#         goal_x, goal_y = self.path[-1]
#         goal_distance = math.hypot(goal_x - self.current_x, goal_y - self.current_y)
#         return goal_distance <= self.goal_tolerance

#     def is_near_final_goal(self) -> bool:
#         goal_x, goal_y = self.path[-1]
#         goal_distance = math.hypot(goal_x - self.current_x, goal_y - self.current_y)
#         return goal_distance <= max(self.max_lookahead, self.goal_tolerance)

#     def publish_marker(
#         self,
#         x: float,
#         y: float,
#         publisher,
#         marker_id: int,
#         r: float,
#         g: float,
#         b: float,
#     ) -> None:
#         marker = Marker()
#         marker.header.frame_id = self.frame_id
#         marker.header.stamp = self.get_clock().now().to_msg()
#         marker.ns = 'pure_pursuit'
#         marker.id = marker_id
#         marker.type = Marker.SPHERE
#         marker.action = Marker.ADD
#         marker.pose.position.x = x
#         marker.pose.position.y = y
#         marker.pose.position.z = 0.0
#         marker.pose.orientation.w = 1.0
#         marker.scale.x = 1.2
#         marker.scale.y = 1.2
#         marker.scale.z = 1.2
#         marker.color.a = 1.0
#         marker.color.r = r
#         marker.color.g = g
#         marker.color.b = b
#         publisher.publish(marker)

#     def debug_log(
#         self,
#         lookahead: float,
#         steering_raw: float,
#         steering: float,
#     ) -> None:
#         now = self.get_clock().now()
#         if (now - self.last_debug_time).nanoseconds / 1e9 < self.debug_period:
#             return

#         self.last_debug_time = now
#         self.get_logger().info(
#             'PP debug: '
#             f'pos=({self.current_x:.2f}, {self.current_y:.2f}), '
#             f'yaw={math.degrees(self.current_yaw):.1f} deg, '
#             f'speed={self.current_speed:.2f} m/s, '
#             f'rtk={self.rtk_state}(q={self.fix_quality}), '
#             f'predicted={self.position_is_predicted}, '
#             f'lookahead={lookahead:.2f} m, '
#             f'nearest={self.nearest_index}, target={self.target_index}, '
#             f'steer_raw={math.degrees(steering_raw):.2f} deg, '
#             f'steer_cmd={math.degrees(steering):.2f} deg'
#         )

#     def debug_yaw_hold(self, steering_raw: float, steering: float) -> None:
#         now = self.get_clock().now()
#         if (now - self.last_debug_time).nanoseconds / 1e9 < self.debug_period:
#             return

#         self.last_debug_time = now
#         hold_yaw = (
#             self.float_hold_yaw
#             if self.float_hold_yaw is not None
#             else self.current_yaw
#         )
#         yaw_error = self.normalize_angle(hold_yaw - self.current_yaw)
#         self.get_logger().info(
#             'FLOAT yaw hold: '
#             f'hold_yaw={math.degrees(hold_yaw):.1f} deg, '
#             f'current_yaw={math.degrees(self.current_yaw):.1f} deg, '
#             f'error={math.degrees(yaw_error):.1f} deg, '
#             f'speed_cmd={self.last_commanded_speed:.2f} m/s, '
#             f'steer_raw={math.degrees(steering_raw):.2f} deg, '
#             f'steer_cmd={math.degrees(steering):.2f} deg'
#         )

#     def throttled_warn(self, message: str) -> None:
#         now = self.get_clock().now()
#         if (now - self.last_debug_time).nanoseconds / 1e9 < self.debug_period:
#             return

#         self.last_debug_time = now
#         self.get_logger().warn(message)

#     @staticmethod
#     def normalize_angle(angle: float) -> float:
#         while angle > math.pi:
#             angle -= 2.0 * math.pi
#         while angle < -math.pi:
#             angle += 2.0 * math.pi
#         return angle

#     @staticmethod
#     def clamp(value: float, minimum: float, maximum: float) -> float:
#         return max(minimum, min(maximum, value))


# def main(args=None):
#     rclpy.init(args=args)
#     node = None

#     try:
#         node = PurePursuitController()
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         if node is not None:
#             node.stop_vehicle()
#             node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()





#!/usr/bin/env python3

import csv
from datetime import datetime
import math
import os
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker

try:
    from ackermann_msgs.msg import AckermannDriveStamped
except ImportError:
    AckermannDriveStamped = None


class PurePursuitController(Node):
    """GNSS-only path follower with adaptive FLOAT filtering.

    This version keeps the original path format, but handles RTK FLOAT as a
    degraded measurement instead of ignoring its position completely.
    """

    def __init__(self):
        super().__init__('pure_pursuit_newver_controller')

        self.declare_parameter('path_file', self.default_path_file())
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('wheelbase', 1.67)
        self.declare_parameter('target_speed', 1.75)
        self.declare_parameter('min_speed', 1.5)
        self.declare_parameter('max_speed', 1.75)
        # self.declare_parameter('min_lookahead', 3.0)
        self.declare_parameter('min_lookahead', 1.5)
        # self.declare_parameter('max_lookahead', 8.0)
        self.declare_parameter('max_lookahead', 4.0)
        # self.declare_parameter('lookahead_base', 2.0)
        self.declare_parameter('lookahead_base', 1.5)
        # self.declare_parameter('lookahead_speed_gain', 1.5)
        self.declare_parameter('lookahead_speed_gain', 1.2)
        self.declare_parameter('max_steering_deg', 48.0)
        self.declare_parameter('steering_filter_alpha', 0.16)
        self.declare_parameter('max_steering_rate_deg_s', 22.0)
        self.declare_parameter('goal_tolerance', 2.0)
        self.declare_parameter('forward_dot_threshold', 0.0)
        self.declare_parameter('nearest_search_backtrack', 8)
        self.declare_parameter('control_period', 0.05)
        self.declare_parameter('debug_period', 1.0)
        self.declare_parameter('rtk_float_timeout', 1.0)
        self.declare_parameter('rtk_no_fix_timeout', 1.0)
        self.declare_parameter('float_speed_limit', 1.75)
        self.declare_parameter('fix_speed_limit', 1.75)
        self.declare_parameter('float_min_lookahead', 2.5)
        self.declare_parameter('float_alpha_min', 0.08)
        self.declare_parameter('float_alpha_max', 0.45)
        self.declare_parameter('float_jump_reject_margin', 0.75)
        self.declare_parameter('float_lateral_correction_limit', 0.45)
        self.declare_parameter('float_course_speed_threshold', 0.25)
        self.declare_parameter('enable_path_constraint', True)
        self.declare_parameter('path_constraint_gain', 0.35)
        self.declare_parameter('path_projection_search_window', 80)
        self.declare_parameter('stanley_in_degraded_mode', True)
        self.declare_parameter('stanley_gain', 0.9)
        self.declare_parameter('stanley_softening_speed', 0.6)
        self.declare_parameter('position_jump_margin', 0.35)
        self.declare_parameter('min_jump_threshold', 0.45)
        self.declare_parameter('enable_output_log', True)
        self.declare_parameter('output_log_dir', self.default_output_log_dir())
        self.declare_parameter('output_log_filename', 'navigation_%Y%m%d_%H%M%S.csv')

        self.path_file = self.get_parameter('path_file').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.drive_topic = self.get_parameter('drive_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.target_speed = float(self.get_parameter('target_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.min_lookahead = float(self.get_parameter('min_lookahead').value)
        self.max_lookahead = float(self.get_parameter('max_lookahead').value)
        self.lookahead_base = float(self.get_parameter('lookahead_base').value)
        self.lookahead_speed_gain = float(
            self.get_parameter('lookahead_speed_gain').value
        )
        self.max_steering = math.radians(
            float(self.get_parameter('max_steering_deg').value)
        )
        self.steering_filter_alpha = float(
            self.get_parameter('steering_filter_alpha').value
        )
        self.max_steering_rate = math.radians(
            float(self.get_parameter('max_steering_rate_deg_s').value)
        )
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.forward_dot_threshold = float(
            self.get_parameter('forward_dot_threshold').value
        )
        self.nearest_search_backtrack = int(
            self.get_parameter('nearest_search_backtrack').value
        )
        self.control_period = float(self.get_parameter('control_period').value)
        self.debug_period = float(self.get_parameter('debug_period').value)
        self.rtk_float_timeout = float(
            self.get_parameter('rtk_float_timeout').value
        )
        self.rtk_no_fix_timeout = float(
            self.get_parameter('rtk_no_fix_timeout').value
        )
        self.float_speed_limit = float(
            self.get_parameter('float_speed_limit').value
        )
        self.fix_speed_limit = float(self.get_parameter('fix_speed_limit').value)
        self.float_min_lookahead = float(
            self.get_parameter('float_min_lookahead').value
        )
        self.float_alpha_min = float(self.get_parameter('float_alpha_min').value)
        self.float_alpha_max = float(self.get_parameter('float_alpha_max').value)
        self.float_jump_reject_margin = float(
            self.get_parameter('float_jump_reject_margin').value
        )
        self.float_lateral_correction_limit = float(
            self.get_parameter('float_lateral_correction_limit').value
        )
        self.float_course_speed_threshold = float(
            self.get_parameter('float_course_speed_threshold').value
        )
        self.enable_path_constraint = bool(
            self.get_parameter('enable_path_constraint').value
        )
        self.path_constraint_gain = float(
            self.get_parameter('path_constraint_gain').value
        )
        self.path_projection_search_window = int(
            self.get_parameter('path_projection_search_window').value
        )
        self.stanley_in_degraded_mode = bool(
            self.get_parameter('stanley_in_degraded_mode').value
        )
        self.stanley_gain = float(self.get_parameter('stanley_gain').value)
        self.stanley_softening_speed = float(
            self.get_parameter('stanley_softening_speed').value
        )
        self.position_jump_margin = float(
            self.get_parameter('position_jump_margin').value
        )
        self.min_jump_threshold = float(
            self.get_parameter('min_jump_threshold').value
        )
        self.enable_output_log = bool(
            self.get_parameter('enable_output_log').value
        )
        self.output_log_dir = self.get_parameter('output_log_dir').value
        self.output_log_filename = self.get_parameter('output_log_filename').value

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_speed = 0.0
        self.have_odom = False
        self.rtk_state = 'NO_FIX'
        self.fix_quality = 0
        self.allow_waypoint_progress = False
        self.position_is_predicted = False
        self.float_started_time = None
        self.last_odom_time = None
        self.last_fix_time = None
        self.last_fix_yaw = None
        self.last_known_good_x: Optional[float] = None
        self.last_known_good_y: Optional[float] = None
        self.last_known_good_yaw: Optional[float] = None
        # Adaptive GNSS-only filter state for degraded RTK modes.
        self.filter_x = 0.0
        self.filter_y = 0.0
        self.filter_yaw = 0.0
        self.filter_active = False
        self.ublox_speed = 0.0
        self.ublox_yaw = 0.0
        self.filter_update_count = 0
        self.last_float_alpha = 0.0
        self.last_float_innovation = 0.0
        self.last_path_lateral_error = 0.0
        self.last_commanded_speed = 0.0

        self.path: List[Tuple[float, float]] = []
        self.path_loaded = False
        self.path_completed = False

        self.nearest_index = 0
        self.target_index = 0
        self.last_target_index = 0
        self.last_steering = 0.0
        self.last_control_time = None
        self.last_debug_time = self.get_clock().now()
        self.output_log_file = None
        self.output_log_writer = None
        self.output_log_path = None

        self.load_path()
        self.setup_output_logger()

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )

        self.drive_pub = None
        if AckermannDriveStamped is not None:
            self.drive_pub = self.create_publisher(
                AckermannDriveStamped,
                self.drive_topic,
                10,
            )
        else:
            self.get_logger().warn(
                'ackermann_msgs is not installed; publishing /cmd_vel only'
            )

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.target_marker_pub = self.create_publisher(
            Marker,
            '/target_waypoint_marker',
            10,
        )
        self.nearest_marker_pub = self.create_publisher(
            Marker,
            '/nearest_waypoint_marker',
            10,
        )

        self.create_timer(self.control_period, self.control_loop)

        self.get_logger().info(
            'Pure Pursuit controller started: '
            f'path={self.path_file}, odom={self.odom_topic}, '
            f'drive={self.drive_topic}, cmd_vel={self.cmd_vel_topic}'
        )

    @staticmethod
    def default_path_file() -> str:
        # return '/home/inc/ros2_ws/src/navigation_system/logs14/path_smoothlog14_resampled_1m.csv'
        return '/home/kr-zoo/ros2_ws/src/navigation_system/logs_zoo2/path_smoothlogs_zoo2_resampled_1m.csv'

    @staticmethod
    def default_output_log_dir() -> str:
        return os.path.join(os.getcwd(), 'output_logs')

    def setup_output_logger(self) -> None:
        if not self.enable_output_log:
            return

        try:
            os.makedirs(self.output_log_dir, exist_ok=True)
            filename = datetime.now().strftime(self.output_log_filename)
            if not filename.lower().endswith('.csv'):
                filename = f'{filename}.csv'
            self.output_log_path = os.path.join(self.output_log_dir, filename)
            self.output_log_file = open(self.output_log_path, 'w', newline='')
            self.output_log_writer = csv.writer(self.output_log_file)
            self.output_log_writer.writerow([
                'x',
                'y',
                'speed',
                'heading',
                'fix_quality',
                'steering_angle',
            ])
            self.output_log_file.flush()
            self.get_logger().info(f'Output CSV log: {self.output_log_path}')
        except Exception as exc:
            self.output_log_file = None
            self.output_log_writer = None
            self.get_logger().error(f'Failed to open output CSV log: {exc}')

    def write_output_log(self, commanded_speed: float, steering: float) -> None:
        if self.output_log_writer is None:
            return

        try:
            self.output_log_writer.writerow([
                f'{self.current_x:.6f}',
                f'{self.current_y:.6f}',
                f'{self.current_speed:.6f}',
                f'{math.degrees(self.current_yaw):.6f}',
                self.fix_quality,
                f'{math.degrees(steering):.6f}',
            ])
            self.output_log_file.flush()
        except Exception as exc:
            self.get_logger().warn(f'Failed to write output CSV log: {exc}')

    def close_output_logger(self) -> None:
        if self.output_log_file is None:
            return

        try:
            self.output_log_file.flush()
            self.output_log_file.close()
        except Exception as exc:
            self.get_logger().warn(f'Failed to close output CSV log: {exc}')
        finally:
            self.output_log_file = None
            self.output_log_writer = None

    def load_path(self) -> None:
        self.path = []

        try:
            with open(self.path_file, 'r', newline='') as file:
                reader = csv.DictReader(file)
                if reader.fieldnames is None:
                    raise ValueError('CSV file has no header')

                required_columns = {'x', 'y'}
                if not required_columns.issubset(set(reader.fieldnames)):
                    raise ValueError('CSV header must contain x,y columns')

                for row_number, row in enumerate(reader, start=2):
                    try:
                        self.path.append((float(row['x']), float(row['y'])))
                    except (TypeError, ValueError) as exc:
                        raise ValueError(
                            f'invalid x,y at CSV row {row_number}: {row}'
                        ) from exc

            if len(self.path) < 2:
                raise ValueError('path must contain at least 2 waypoints')

            self.path_loaded = True
            self.get_logger().info(f'Loaded {len(self.path)} waypoints')

        except Exception as exc:
            self.path_loaded = False
            self.get_logger().error(f'Failed to load path: {exc}')

    def odom_callback(self, msg: Odometry) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        msg_x     = msg.pose.pose.position.x
        msg_y     = msg.pose.pose.position.y
        msg_speed = self.get_odom_speed(msg)
        q = msg.pose.pose.orientation
        _, _, msg_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.fix_quality   = self.get_fix_quality(msg)
        new_rtk_state      = self.classify_rtk_state(self.fix_quality)
        previous_rtk_state = self.rtk_state

        # เก็บ raw ublox ไว้ให้ adaptive filter ใช้เสมอ ไม่ว่าจะเป็นสถานะใด
        self.ublox_speed   = msg_speed
        self.ublox_yaw     = msg_yaw
        self.current_speed = msg_speed

        # ── Transition: any state → FLOAT (เข้า FLOAT ครั้งแรก) ─────────────
        if new_rtk_state == 'FLOAT' and (
            previous_rtk_state != 'FLOAT' or self.float_started_time is None
        ):
            seed_x   = self.last_known_good_x   if self.last_known_good_x   is not None else self.current_x
            seed_y   = self.last_known_good_y   if self.last_known_good_y   is not None else self.current_y
            seed_yaw = self.last_known_good_yaw if self.last_known_good_yaw is not None else msg_yaw

            self.seed_adaptive_filter(seed_x, seed_y, seed_yaw)
            self.float_started_time = now

            self.get_logger().info(
                f'RTK FLOAT: adaptive GNSS filter seeded from last FIX → '
                f'x={seed_x:.3f}  y={seed_y:.3f}  yaw={math.degrees(seed_yaw):.1f}°'
            )

        # ── Transition: NO_FIX ครั้งแรก → seed adaptive filter ─────────────
        if new_rtk_state == 'NO_FIX' and not self.filter_active:
            if self.last_known_good_x is not None:
                self.seed_adaptive_filter(
                    self.last_known_good_x,
                    self.last_known_good_y,
                    self.last_known_good_yaw,
                )
                self.float_started_time = now
                self.get_logger().warn(
                    f'RTK NO_FIX: adaptive filter seeded from last_known_good → '
                    f'x={self.filter_x:.3f}  y={self.filter_y:.3f}  '
                    f'yaw={math.degrees(self.filter_yaw):.1f}°'
                )

        # ── Transition: FLOAT/NO_FIX → FIX หรือ DGPS ────────────────────────
        if new_rtk_state in ('FIX', 'DGPS') and self.filter_active:
            self.filter_active = False
            self.float_started_time = None
            self.filter_update_count = 0
            self.get_logger().info(
                f'RTK {new_rtk_state} (q={self.fix_quality}) at '
                f'x={msg_x:.3f}  y={msg_y:.3f}; adaptive filter deactivated'
            )

        self.rtk_state = new_rtk_state
        self.allow_waypoint_progress = False
        self.position_is_predicted   = False

        # ════════════════════════════════════════════════════════════════════
        #  FIX  — ใช้ตำแหน่งจาก ublox โดยตรง (ความแม่นยำสูงสุด)
        # ════════════════════════════════════════════════════════════════════
        if self.rtk_state == 'FIX':
            if self.is_position_outlier(msg_x, msg_y, now):
                self.throttled_warn('Rejected RTK FIX position outlier')
                return

            self.current_x   = msg_x
            self.current_y   = msg_y
            self.current_yaw = msg_yaw

            self.last_known_good_x   = msg_x
            self.last_known_good_y   = msg_y
            self.last_known_good_yaw = msg_yaw

            self.last_fix_time  = now
            self.last_fix_yaw   = msg_yaw
            self.last_odom_time = now
            self.allow_waypoint_progress = True
            self.have_odom = True
            return

        # ════════════════════════════════════════════════════════════════════
        #  DGPS — SPS / DGPS / SBAS / PPS / DR
        # ════════════════════════════════════════════════════════════════════
        if self.rtk_state == 'DGPS':
            self.current_x   = msg_x
            self.current_y   = msg_y
            self.current_yaw = msg_yaw
            self.last_odom_time = now
            self.allow_waypoint_progress = True
            self.have_odom = True
            return

        # ════════════════════════════════════════════════════════════════════
        #  FLOAT — adaptive GNSS filter จาก ublox speed/yaw + FLOAT position
        # ════════════════════════════════════════════════════════════════════
        if self.rtk_state == 'FLOAT':
            dt = self.control_period if self.last_odom_time is None else max(
                now - self.last_odom_time, self.control_period
            )

            self.update_adaptive_gnss_filter(msg_x, msg_y, msg_yaw, dt, True)
            self.last_odom_time = now
            self.position_is_predicted = True
            self.have_odom = True
            self.allow_waypoint_progress = self.filter_update_count >= 1

            float_elapsed = 0.0 if self.float_started_time is None else now - self.float_started_time
            if float_elapsed > self.rtk_float_timeout:
                self.throttled_warn(
                    f'FLOAT adaptive filter active ({float_elapsed:.1f}s) | '
                    f'est=({self.filter_x:.2f}, {self.filter_y:.2f}) | '
                    f'raw=({msg_x:.2f}, {msg_y:.2f}) | '
                    f'alpha={self.last_float_alpha:.2f}'
                )
            return

        # ════════════════════════════════════════════════════════════════════
        #  NO_FIX — predict ต่อช่วงสั้น ๆ ถ้า filter active / หยุดถ้าไม่มีฐาน
        # ════════════════════════════════════════════════════════════════════
        if self.rtk_state == 'NO_FIX':
            if self.filter_active:
                dt = self.control_period if self.last_odom_time is None else max(
                    now - self.last_odom_time, self.control_period
                )
                self.update_adaptive_gnss_filter(msg_x, msg_y, msg_yaw, dt, False)
                self.last_odom_time = now
                self.position_is_predicted = True
                self.have_odom = True
                self.allow_waypoint_progress = self.filter_update_count >= 1

                no_fix_elapsed = 0.0 if self.float_started_time is None else now - self.float_started_time
                self.throttled_warn(
                    f'NO_FIX adaptive prediction active ({no_fix_elapsed:.1f}s) | '
                    f'est=({self.filter_x:.2f}, {self.filter_y:.2f}) | '
                    f'raw=({msg_x:.2f}, {msg_y:.2f}) [not corrected]'
                )
                return

            # ไม่เคยมี FIX/FLOAT มาก่อน → หยุดรถรอ
            self.stop_vehicle()
            self.throttled_warn('No valid RTK position; vehicle stopped')

    @staticmethod
    def get_odom_speed(msg: Odometry) -> float:
        linear = msg.twist.twist.linear
        return math.sqrt(
            linear.x * linear.x
            + linear.y * linear.y
            + linear.z * linear.z
        )

    def get_fix_quality(self, msg: Odometry) -> int:
        try:
            return int(round(msg.pose.covariance[0]))
        except (TypeError, ValueError, IndexError):
            return 0

    @staticmethod
    def classify_rtk_state(fix_quality: int) -> str:
        """Map NMEA GGA fix quality to internal RTK state.

        NMEA / ublox quality codes:
          0 = Invalid
          1 = GPS SPS (autonomous)
          2 = DGPS / SBAS
          3 = PPS
          4 = RTK Fixed
          5 = RTK Float
          6 = Dead-reckoning (ublox internal DR)
          7 = Manual input
          8 = Simulation
        """
        if fix_quality == 4:
            return 'FIX'
        if fix_quality == 5:
            return 'FLOAT'
        if fix_quality in (1, 2, 3, 6, 7, 8):
            return 'DGPS'
        return 'NO_FIX'

    def seed_adaptive_filter(self, x: float, y: float, yaw: float) -> None:
        self.filter_x = x
        self.filter_y = y
        self.filter_yaw = yaw
        self.filter_active = True
        self.filter_update_count = 0
        self.last_float_alpha = 0.0
        self.last_float_innovation = 0.0
        self.last_path_lateral_error = 0.0

    def update_adaptive_gnss_filter(
        self,
        measured_x: float,
        measured_y: float,
        measured_yaw: float,
        dt: float,
        use_position_measurement: bool,
    ) -> None:
        """Predict from ublox speed/course and softly correct with FLOAT position."""
        if not self.filter_active:
            self.seed_adaptive_filter(measured_x, measured_y, measured_yaw)

        v = max(0.0, min(abs(self.ublox_speed), self.fix_speed_limit))
        if v >= self.float_course_speed_threshold:
            prediction_yaw = measured_yaw
        else:
            prediction_yaw = self.filter_yaw

        predicted_x = self.filter_x + v * math.cos(prediction_yaw) * dt
        predicted_y = self.filter_y + v * math.sin(prediction_yaw) * dt
        estimated_yaw = prediction_yaw

        alpha = 0.0
        innovation = 0.0
        corrected_x = predicted_x
        corrected_y = predicted_y

        if use_position_measurement:
            residual_x = measured_x - predicted_x
            residual_y = measured_y - predicted_y
            innovation = math.hypot(residual_x, residual_y)
            max_reasonable_jump = max(
                self.min_jump_threshold,
                v * dt + self.position_jump_margin + self.float_jump_reject_margin,
            )

            alpha = self.compute_float_alpha(innovation, max_reasonable_jump)
            correction_x = self.clamp(
                alpha * residual_x,
                -self.float_lateral_correction_limit,
                self.float_lateral_correction_limit,
            )
            correction_y = self.clamp(
                alpha * residual_y,
                -self.float_lateral_correction_limit,
                self.float_lateral_correction_limit,
            )
            corrected_x = predicted_x + correction_x
            corrected_y = predicted_y + correction_y

        if self.enable_path_constraint and self.path_loaded:
            corrected_x, corrected_y = self.apply_path_constraint(
                corrected_x,
                corrected_y,
            )

        self.filter_x = corrected_x
        self.filter_y = corrected_y
        self.filter_yaw = estimated_yaw
        self.filter_update_count += 1
        self.last_float_alpha = alpha
        self.last_float_innovation = innovation

        self.current_x = self.filter_x
        self.current_y = self.filter_y
        self.current_yaw = self.filter_yaw

    def compute_float_alpha(self, innovation: float, max_reasonable_jump: float) -> float:
        if max_reasonable_jump <= 0.0:
            return self.float_alpha_min

        quality = 1.0 - self.clamp(innovation / max_reasonable_jump, 0.0, 1.0)
        alpha = self.float_alpha_min + (
            self.float_alpha_max - self.float_alpha_min
        ) * quality
        return self.clamp(alpha, self.float_alpha_min, self.float_alpha_max)

    def apply_path_constraint(self, x: float, y: float) -> Tuple[float, float]:
        projection = self.project_point_to_path(x, y)
        if projection is None:
            self.last_path_lateral_error = 0.0
            return x, y

        path_x, path_y, lateral_error, _ = projection
        self.last_path_lateral_error = lateral_error

        gain = self.clamp(self.path_constraint_gain, 0.0, 1.0)
        constrained_x = x + gain * (path_x - x)
        constrained_y = y + gain * (path_y - y)
        return constrained_x, constrained_y

    def project_point_to_path(
        self,
        x: float,
        y: float,
    ) -> Optional[Tuple[float, float, float, int]]:
        if len(self.path) < 2:
            return None

        start_index = max(0, self.last_target_index - self.nearest_search_backtrack)
        end_index = min(
            len(self.path) - 1,
            max(start_index + 1, self.last_target_index + self.path_projection_search_window),
        )

        best_projection = None
        best_distance = float('inf')
        for index in range(start_index, end_index):
            x1, y1 = self.path[index]
            x2, y2 = self.path[index + 1]
            seg_x = x2 - x1
            seg_y = y2 - y1
            seg_len_sq = seg_x * seg_x + seg_y * seg_y
            if seg_len_sq <= 1e-9:
                continue

            t = self.clamp(((x - x1) * seg_x + (y - y1) * seg_y) / seg_len_sq, 0.0, 1.0)
            proj_x = x1 + t * seg_x
            proj_y = y1 + t * seg_y
            error_x = x - proj_x
            error_y = y - proj_y
            distance = math.hypot(error_x, error_y)
            if distance < best_distance:
                cross = seg_x * (y - proj_y) - seg_y * (x - proj_x)
                sign = 1.0 if cross >= 0.0 else -1.0
                best_distance = distance
                best_projection = (proj_x, proj_y, sign * distance, index)

        return best_projection

    def is_position_outlier(self, x: float, y: float, now: float) -> bool:
        if not self.have_odom or self.last_odom_time is None:
            return False

        if self.last_fix_time is None or now - self.last_fix_time > 2.0:
            return False

        ref_x = self.last_known_good_x if self.last_known_good_x is not None else self.current_x
        ref_y = self.last_known_good_y if self.last_known_good_y is not None else self.current_y

        dt = max(now - self.last_odom_time, self.control_period)
        distance = math.hypot(x - ref_x, y - ref_y)
        speed = max(abs(self.current_speed), abs(self.last_commanded_speed))
        max_possible_move = max(
            self.min_jump_threshold,
            speed * dt + self.position_jump_margin,
        )
        return distance > max_possible_move

    def control_loop(self) -> None:
        if not self.path_loaded or self.path_completed:
            return

        if not self.have_odom:
            self.throttled_warn('Waiting for /odom')
            return

        now = self.get_clock().now().nanoseconds / 1e9

        if (
            self.last_odom_time is not None
            and now - self.last_odom_time > self.rtk_no_fix_timeout
            and not self.filter_active
        ):
            self.rtk_state = 'NO_FIX'
            self.allow_waypoint_progress = False
            self.stop_vehicle()
            self.throttled_warn('Odometry stale; vehicle stopped')
            return

        if self.is_float_timeout_exceeded(now):
            self.throttled_warn(
                f'RTK {self.rtk_state} adaptive filter active: '
                f'pos=({self.current_x:.2f},{self.current_y:.2f}), '
                f'yaw={math.degrees(self.current_yaw):.1f} deg'
            )

        lookahead = self.compute_dynamic_lookahead()
        if not self.allow_waypoint_progress and self.target_index < len(self.path):
            nearest_index = self.nearest_index
            target_index = self.target_index
        else:
            nearest_index = self.find_nearest_forward_waypoint()

            if nearest_index is None:
                if self.is_near_final_goal():
                    self.path_completed = True
                    self.stop_vehicle()
                    self.get_logger().info('Path completed; vehicle stopped')
                    return

                self.stop_vehicle()
                self.throttled_warn('No forward waypoint found; vehicle stopped')
                return

            target_index = self.find_target_waypoint(nearest_index, lookahead)

            if target_index is None:
                self.stop_vehicle()
                self.throttled_warn('No forward waypoint found; vehicle stopped')
                return

        self.nearest_index = nearest_index
        self.target_index = target_index
        self.last_target_index = max(self.last_target_index, target_index)

        target_x, target_y = self.path[target_index]
        if self.should_use_stanley():
            steering_raw = self.compute_stanley(nearest_index)
        else:
            steering_raw = self.compute_pure_pursuit(target_x, target_y)
        steering = self.smooth_steering(steering_raw)

        if self.is_path_completed(target_index):
            self.path_completed = True
            self.stop_vehicle()
            self.get_logger().info('Path completed; vehicle stopped')
            return

        self.publish_drive_command(steering)
        self.publish_marker(
            target_x,
            target_y,
            self.target_marker_pub,
            0,
            1.0,
            0.0,
            0.0,
        )

        nearest_x, nearest_y = self.path[nearest_index]
        self.publish_marker(
            nearest_x,
            nearest_y,
            self.nearest_marker_pub,
            1,
            0.0,
            1.0,
            0.0,
        )

        if self.filter_active:
            self.debug_adaptive_filter(lookahead, steering_raw, steering)
        else:
            self.debug_log(lookahead, steering_raw, steering)
            if self.rtk_state == 'DGPS':
                self.throttled_warn(
                    f'DGPS mode (q={self.fix_quality}): '
                    f'pos=({self.current_x:.2f},{self.current_y:.2f}) '
                    f'— ความแม่นยำต่ำกว่า RTK FIX'
                )

    def compute_dynamic_lookahead(self) -> float:
        lookahead = self.lookahead_base + self.lookahead_speed_gain * abs(
            self.current_speed
        )
        if self.rtk_state in ('FLOAT', 'DGPS', 'NO_FIX'):
            lookahead = max(lookahead, self.float_min_lookahead)
        return self.clamp(lookahead, self.min_lookahead, self.max_lookahead)

    def find_nearest_forward_waypoint(self) -> Optional[int]:
        heading_x = math.cos(self.current_yaw)
        heading_y = math.sin(self.current_yaw)

        start_index = max(0, self.last_target_index - self.nearest_search_backtrack)
        best_index = None
        best_distance = float('inf')

        for index in range(start_index, len(self.path)):
            waypoint_x, waypoint_y = self.path[index]
            dx = waypoint_x - self.current_x
            dy = waypoint_y - self.current_y
            dot = heading_x * dx + heading_y * dy

            if dot < self.forward_dot_threshold:
                continue

            distance = math.hypot(dx, dy)
            if distance < best_distance:
                best_distance = distance
                best_index = index

        return best_index

    def find_target_waypoint(
        self,
        nearest_index: int,
        lookahead: float,
    ) -> Optional[int]:
        heading_x = math.cos(self.current_yaw)
        heading_y = math.sin(self.current_yaw)

        start_index = max(nearest_index, self.last_target_index)

        for index in range(start_index, len(self.path)):
            waypoint_x, waypoint_y = self.path[index]
            dx = waypoint_x - self.current_x
            dy = waypoint_y - self.current_y
            dot = heading_x * dx + heading_y * dy

            if dot < self.forward_dot_threshold:
                continue

            if math.hypot(dx, dy) >= lookahead:
                return index

        if self.is_near_final_goal():
            return len(self.path) - 1

        return None

    def compute_pure_pursuit(self, target_x: float, target_y: float) -> float:
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        lookahead_distance = math.hypot(dx, dy)

        if lookahead_distance < 0.01:
            return 0.0

        target_angle = math.atan2(dy, dx)
        alpha = self.normalize_angle(target_angle - self.current_yaw)

        steering = math.atan2(
            2.0 * self.wheelbase * math.sin(alpha),
            lookahead_distance,
        )
        return self.clamp(steering, -self.max_steering, self.max_steering)

    def should_use_stanley(self) -> bool:
        return (
            self.stanley_in_degraded_mode
            and self.rtk_state in ('FLOAT', 'NO_FIX')
            and self.filter_active
            and len(self.path) >= 2
        )

    def compute_stanley(self, nearest_index: int) -> float:
        segment_index = min(max(nearest_index, 0), len(self.path) - 2)
        x1, y1 = self.path[segment_index]
        x2, y2 = self.path[segment_index + 1]
        path_yaw = math.atan2(y2 - y1, x2 - x1)
        heading_error = self.normalize_angle(path_yaw - self.current_yaw)

        projection = self.project_point_to_path(self.current_x, self.current_y)
        if projection is None:
            cross_track_error = 0.0
        else:
            _, _, signed_lateral_error, _ = projection
            cross_track_error = -signed_lateral_error

        speed = max(abs(self.current_speed), self.stanley_softening_speed)
        cross_track_term = math.atan2(
            self.stanley_gain * cross_track_error,
            speed,
        )
        steering = heading_error + cross_track_term
        return self.clamp(steering, -self.max_steering, self.max_steering)

    def is_float_timeout_exceeded(self, now: float) -> bool:
        """True when we have been in FLOAT/NO_FIX filter mode longer than rtk_float_timeout."""
        if self.float_started_time is None or not self.filter_active:
            return False
        return now - self.float_started_time > self.rtk_float_timeout

    def debug_adaptive_filter(self, lookahead: float, steering_raw: float, steering: float) -> None:
        now = self.get_clock().now()
        if (now - self.last_debug_time).nanoseconds / 1e9 < self.debug_period:
            return
        self.last_debug_time = now

        elapsed = 0.0 if self.float_started_time is None else \
            now.nanoseconds / 1e9 - self.float_started_time

        self.get_logger().info(
            f'{self.rtk_state} adaptive filter ({elapsed:.1f}s): '
            f'est=({self.filter_x:.2f},{self.filter_y:.2f}), '
            f'yaw={math.degrees(self.filter_yaw):.1f} deg, '
            f'ublox_v={self.ublox_speed:.2f} m/s, '
            f'ublox_yaw={math.degrees(self.ublox_yaw):.1f} deg, '
            f'alpha={self.last_float_alpha:.2f}, '
            f'innovation={self.last_float_innovation:.2f} m, '
            f'path_lat={self.last_path_lateral_error:.2f} m, '
            f'lookahead={lookahead:.2f} m, '
            f'steer_raw={math.degrees(steering_raw):.2f} deg, '
            f'steer_cmd={math.degrees(steering):.2f} deg'
        )

    def smooth_steering(self, steering: float) -> float:
        now = self.get_clock().now()

        if self.last_control_time is None:
            dt = self.control_period
        else:
            dt = max(
                (now - self.last_control_time).nanoseconds / 1e9,
                self.control_period,
            )

        self.last_control_time = now

        filtered = (
            self.steering_filter_alpha * steering
            + (1.0 - self.steering_filter_alpha) * self.last_steering
        )

        rate_limit = self.max_steering_rate
        if self.rtk_state in ('FLOAT', 'DGPS', 'NO_FIX'):
            rate_limit *= 0.5

        max_delta = rate_limit * dt
        filtered = self.clamp(
            filtered,
            self.last_steering - max_delta,
            self.last_steering + max_delta,
        )
        filtered = self.clamp(filtered, -self.max_steering, self.max_steering)
        self.last_steering = filtered

        return filtered

    def publish_drive_command(self, steering: float) -> None:
        speed = self.get_command_speed()

        if self.drive_pub is not None:
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = self.get_clock().now().to_msg()
            drive_msg.header.frame_id = self.frame_id
            drive_msg.drive.speed = speed
            drive_msg.drive.steering_angle = steering
            self.drive_pub.publish(drive_msg)

        cmd_vel = Twist()
        cmd_vel.linear.x = speed
        cmd_vel.angular.z = steering
        self.cmd_vel_pub.publish(cmd_vel)
        self.last_commanded_speed = speed
        self.write_output_log(speed, steering)

    def get_command_speed(self) -> float:
        if self.rtk_state == 'FIX':
            speed = min(self.target_speed, self.fix_speed_limit)
            return self.clamp(speed, self.min_speed, self.max_speed)
        if self.rtk_state in ('FLOAT', 'NO_FIX') and self.filter_active:
            speed = min(self.target_speed, self.float_speed_limit)
            return self.clamp(speed, self.min_speed, self.max_speed)
        if self.rtk_state == 'DGPS':
            speed = min(self.target_speed, self.float_speed_limit)
            return self.clamp(speed, self.min_speed, self.max_speed)
        return 0.0

    def stop_vehicle(self) -> None:
        self.last_steering = 0.0
        self.last_commanded_speed = 0.0

        if self.drive_pub is not None:
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = self.get_clock().now().to_msg()
            drive_msg.header.frame_id = self.frame_id
            drive_msg.drive.speed = 0.0
            drive_msg.drive.steering_angle = 0.0
            self.drive_pub.publish(drive_msg)

        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

    def destroy_node(self) -> None:
        self.close_output_logger()
        super().destroy_node()

    def is_path_completed(self, target_index: int) -> bool:
        if target_index < len(self.path) - 1:
            return False

        goal_x, goal_y = self.path[-1]
        goal_distance = math.hypot(goal_x - self.current_x, goal_y - self.current_y)
        return goal_distance <= self.goal_tolerance

    def is_near_final_goal(self) -> bool:
        goal_x, goal_y = self.path[-1]
        goal_distance = math.hypot(goal_x - self.current_x, goal_y - self.current_y)
        return goal_distance <= max(self.max_lookahead, self.goal_tolerance)

    def publish_marker(
        self,
        x: float,
        y: float,
        publisher,
        marker_id: int,
        r: float,
        g: float,
        b: float,
    ) -> None:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'pure_pursuit'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.2
        marker.scale.y = 1.2
        marker.scale.z = 1.2
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        publisher.publish(marker)

    def debug_log(
        self,
        lookahead: float,
        steering_raw: float,
        steering: float,
    ) -> None:
        now = self.get_clock().now()
        if (now - self.last_debug_time).nanoseconds / 1e9 < self.debug_period:
            return

        self.last_debug_time = now
        self.get_logger().info(
            'PP debug: '
            f'pos=({self.current_x:.2f}, {self.current_y:.2f}), '
            f'yaw={math.degrees(self.current_yaw):.1f} deg, '
            f'speed={self.current_speed:.2f} m/s, '
            f'rtk={self.rtk_state}(q={self.fix_quality}), '
            f'predicted={self.position_is_predicted}, '
            f'lookahead={lookahead:.2f} m, '
            f'nearest={self.nearest_index}, target={self.target_index}, '
            f'steer_raw={math.degrees(steering_raw):.2f} deg, '
            f'steer_cmd={math.degrees(steering):.2f} deg'
        )

    def throttled_warn(self, message: str) -> None:
        now = self.get_clock().now()
        if (now - self.last_debug_time).nanoseconds / 1e9 < self.debug_period:
            return

        self.last_debug_time = now
        self.get_logger().warn(message)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = PurePursuitController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop_vehicle()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
