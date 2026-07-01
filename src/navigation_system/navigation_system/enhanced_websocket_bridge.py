# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32, String, Float64MultiArray
# from geometry_msgs.msg import Twist
# import math
# import csv
# from collections import deque

# class WaypointManager:
#     def __init__(self, csv_file_path, auto_start=True):
#         self.waypoints = []
#         self.current_waypoint_index = 0
#         self.auto_start = auto_start
#         self.navigation_started = False
#         self.starting_waypoint = 0
#         self.load_waypoints_from_csv(csv_file_path)
#         self.path_segments = [
#             {'start': 0, 'end': 240, 'type': 'straight', 'direction': 'S-N', 'target_heading': 0.0},
#             {'start': 241, 'end': 256, 'type': 'curve', 'direction': 'curve1', 'target_heading': None, 'turn': 'left'},
#             {'start': 257, 'end': 315, 'type': 'straight', 'direction': 'E-W', 'target_heading': 335.0, 'reverse_heading': True},
#             {'start': 316, 'end': 383, 'type': 'curve', 'direction': 'curve2', 'target_heading': None, 'turn': 'left'},
#             {'start': 384, 'end': 496, 'type': 'straight', 'direction': 'N-S', 'target_heading': 0.0, 'reverse_heading': True},
#             {'start': 497, 'end': 571, 'type': 'curve', 'direction': 'curve3', 'target_heading': None, 'turn': 'left'},
#             {'start': 572, 'end': 622, 'type': 'straight', 'direction': 'W-E', 'target_heading': 18.0, 'reverse_heading': True},
#             {'start': 623, 'end': 642, 'type': 'curve', 'direction': 'curve4', 'target_heading': None, 'turn': 'left'},
#             {'start': 643, 'end': 654, 'type': 'straight', 'direction': 'S-N', 'target_heading': 0.0}
#         ]
#         self.curve_radii = {
#             'curve1': 1.5,
#             'curve2': 30.0,
#             'curve3': 5.0,
#             'curve4': 5.0
#         }
#         self.previous_distance = None
#         self.distance_increasing_count = 0
#         self.max_distance_increasing = 3

#     def load_waypoints_from_csv(self, csv_file_path):
#         try:
#             with open(csv_file_path, 'r') as file:
#                 csv_reader = csv.DictReader(file)
#                 for row in csv_reader:
#                     waypoint = {
#                         'x': float(row['x_east']),
#                         'y': float(row['y_north'])
#                     }
#                     self.waypoints.append(waypoint)
#             print(f"✅ Loaded {len(self.waypoints)} waypoints from CSV")
#         except Exception as e:
#             print(f"❌ Error loading waypoints: {e}")
    
#     def find_nearest_forward_waypoint(self, current_position, current_heading, search_radius=50.0):
#         if not current_position or not self.waypoints:
#             return 0
#         current_x, current_y = current_position['x'], current_position['y']
#         candidates = []
#         for i, waypoint in enumerate(self.waypoints):
#             dx = waypoint['x'] - current_x
#             dy = waypoint['y'] - current_y
#             distance = math.sqrt(dx**2 + dy**2)
#             if distance > search_radius:
#                 continue
#             waypoint_bearing = math.degrees(math.atan2(dx, dy))
#             if waypoint_bearing < 0:
#                 waypoint_bearing += 360
#             heading_diff = abs(self.normalize_angle_difference(waypoint_bearing - current_heading))
#             if heading_diff <= 90.0:
#                 distance_score = distance
#                 angle_score = heading_diff / 90.0
#                 combined_score = distance_score + (angle_score * 10.0)
#                 candidates.append({'index': i, 'score': combined_score})
#         if candidates:
#             candidates.sort(key=lambda x: x['score'])
#             return candidates[0]['index']
#         else:
#             return self.find_nearest_waypoint(current_position)
    
#     def find_nearest_waypoint(self, current_position):
#         if not current_position or not self.waypoints:
#             return 0
#         current_x, current_y = current_position['x'], current_position['y']
#         min_distance = float('inf')
#         nearest_index = 0
#         for i, waypoint in enumerate(self.waypoints):
#             distance = math.sqrt((waypoint['x'] - current_x)**2 + (waypoint['y'] - current_y)**2)
#             if distance < min_distance:
#                 min_distance = distance
#                 nearest_index = i
#         return nearest_index
    
#     def initialize_starting_point(self, current_position, current_heading):
#         if self.navigation_started:
#             return False
#         if not current_position or current_heading is None:
#             return False
#         if self.auto_start:
#             self.current_waypoint_index = self.find_nearest_forward_waypoint(current_position, current_heading)
#         else:
#             self.current_waypoint_index = 0
#         self.starting_waypoint = self.current_waypoint_index
#         self.navigation_started = True
#         return True
    
#     def normalize_angle_difference(self, angle_diff):
#         return (angle_diff + 180) % 360 - 180
    
#     def should_advance_waypoint(self, distance_to_waypoint, waypoint_reached_threshold):
#         if distance_to_waypoint < waypoint_reached_threshold:
#             return True, "reached_threshold"
#         if self.previous_distance is not None:
#             if distance_to_waypoint > self.previous_distance:
#                 self.distance_increasing_count += 1
#             else:
#                 self.distance_increasing_count = 0
#         self.previous_distance = distance_to_waypoint
#         if (self.distance_increasing_count >= self.max_distance_increasing and 
#             distance_to_waypoint > waypoint_reached_threshold * 3):
#             return True, "passed_waypoint"
#         if distance_to_waypoint > 100.0:
#             return True, "too_far_skip"
#         return False, "continue"
    
#     def get_current_segment_info(self):
#         for segment in self.path_segments:
#             if segment['start'] <= self.current_waypoint_index <= segment['end']:
#                 return segment
#         return {'type': 'straight', 'direction': 'unknown', 'target_heading': 0.0}
    
#     def get_curve_radius(self, curve_direction):
#         return self.curve_radii.get(curve_direction, 15.0)
    
#     def get_current_waypoint(self):
#         if self.current_waypoint_index < len(self.waypoints):
#             return self.waypoints[self.current_waypoint_index]
#         return None
    
#     def advance_waypoint(self):
#         if self.current_waypoint_index < len(self.waypoints) - 1:
#             self.current_waypoint_index += 1
#             self.previous_distance = None
#             self.distance_increasing_count = 0
#             return True
#         return False
    
#     def get_progress_info(self):
#         return {
#             'current': self.current_waypoint_index + 1,
#             'total': len(self.waypoints),
#             'starting': self.starting_waypoint + 1,
#             'progress_percent': ((self.current_waypoint_index + 1) / len(self.waypoints)) * 100,
#             'from_start_count': self.current_waypoint_index - self.starting_waypoint + 1
#         }

# class BicycleKinematicModel:
#     def __init__(self, wheelbase):
#         self.wheelbase = wheelbase
#         self.current_target_heading = 0.0
#         self.heading_initialized = False
    
#     def set_target_heading(self, target_heading):
#         self.current_target_heading = target_heading
    
#     def get_current_target_heading(self):
#         return self.current_target_heading
    
#     def force_initial_heading(self, current_heading):
#         if not self.heading_initialized:
#             self.current_target_heading = 0.0
#             self.heading_initialized = True
#             return True
#         return False
    
#     def calculate_angular_velocity(self, linear_velocity, steering_angle):
#         if abs(steering_angle) < 1e-6:
#             return 0.0
#         return (linear_velocity * math.tan(steering_angle)) / self.wheelbase
    
#     def limit_steering_angle(self, steering_angle, max_steering_angle):
#         return max(-max_steering_angle, min(max_steering_angle, steering_angle))
    
#     def calculate_heading_correction_steering(self, current_compass_heading, target_compass_heading, kp=3.5):
#         heading_error = self.normalize_angle_difference(current_compass_heading - target_compass_heading)
#         steering_angle = kp * math.radians(heading_error)
#         return steering_angle, heading_error
    
#     def normalize_angle_difference(self, angle_diff):
#         return (angle_diff + 180) % 360 - 180

# class PurePursuitController:
#     def __init__(self, lookahead_distance=2.0):
#         self.lookahead_distance = lookahead_distance
    
#     def calculate_steering_angle(self, current_pos, current_compass_heading, target_pos, wheelbase):
#         dx = target_pos['x'] - current_pos['x']
#         dy = target_pos['y'] - current_pos['y']
#         target_compass_bearing = math.degrees(math.atan2(dx, dy))
#         if target_compass_bearing < 0:
#             target_compass_bearing += 360
#         heading_error = self.normalize_angle(target_compass_bearing - current_compass_heading)
#         distance = math.sqrt(dx**2 + dy**2)
#         alpha = math.radians(heading_error)
#         steering_angle = math.atan2(2 * wheelbase * math.sin(alpha), distance)
#         return steering_angle
    
#     def normalize_angle(self, angle):
#         return (angle + 180) % 360 - 180

# class ConstantCurvatureController:
#     def __init__(self):
#         self.curvature_history = deque(maxlen=5)
    
#     def calculate_steering_angle_with_radius(self, curve_radius, linear_velocity, wheelbase, segment_info=None):
#         curvature = 1.0 / abs(curve_radius)
#         steering_angle = math.atan(wheelbase * curvature)
#         steering_angle = abs(steering_angle)
#         return steering_angle

# class NavigationController(Node):
#     def __init__(self):
#         super().__init__('navigation_controller')
#         self.xy_subscriber = self.create_subscription(
#             Float64MultiArray, '/navigation/xy', self.xy_callback, 10)
#         self.heading_subscriber = self.create_subscription(
#             Float32, '/navigation/heading', self.heading_callback, 10)
#         self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.navigation_debug_publisher = self.create_publisher(String, '/navigation/debug', 10)
#         self.wheelbase = 1.67
#         self.max_steering_angle = 0.873  # ปรับเป็น 1.2 radian หรือค่าที่พวงมาลัยรองรับ
#         self.fixed_speed = 1.0
#         self.max_angular_velocity = 1.5
#         self.current_position = {'x': 0.0, 'y': 0.0}
#         self.current_heading = None
#         csv_file_path = '/home/inc/ros2_ws/waypointfile/waypoints3xy.csv'
#         self.waypoint_manager = WaypointManager(csv_file_path, auto_start=True)
#         self.pure_pursuit = PurePursuitController(lookahead_distance=3.0)
#         self.curvature_controller = ConstantCurvatureController()
#         self.bicycle_model = BicycleKinematicModel(wheelbase=self.wheelbase)
#         self.navigation_active = False
#         self.waypoint_reached_threshold = 3.0
#         self.heading_forced_to_zero = False
#         self.stats = {'navigation_commands': 0, 'heading_corrections': 0}
#         self.last_progress_log_time = 0
#         self.progress_log_interval = 2.0
#         self.create_timer(0.1, self.navigation_control)
#         self.get_logger().info('✅ Navigation Controller (เลี้ยวซ้ายทุกโค้ง + Smart Waypoint Advance) initialized')
#         self.get_logger().info(f'🚗 Vehicle: wheelbase={self.wheelbase}m, max_steering={math.degrees(self.max_steering_angle):.1f}°')
#         self.get_logger().info(f'🚀 Fixed Speed: {self.fixed_speed} m/s')
#         self.get_logger().info(f'🗺️ Loaded {len(self.waypoint_manager.waypoints)} waypoints')
#         self.get_logger().info(f'🎯 Waypoint threshold: {self.waypoint_reached_threshold}m (with smart advance)')
        
    
#     def xy_callback(self, msg):
#         if len(msg.data) >= 2:
#             self.current_position['x'] = msg.data[0]
#             self.current_position['y'] = msg.data[1]
    
#     def heading_callback(self, msg):
#         self.current_heading = msg.data

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Float64MultiArray
from geometry_msgs.msg import Twist
import math
import csv
from collections import deque
import os
from datetime import datetime

class DataLogger:
    def __init__(self, log_directory='/home/inc/ros2_ws/navigation_logs'):
        self.log_directory = log_directory
        self.csv_file_path = None
        self.csv_writer = None
        self.csv_file = None
        
        # สร้างโฟลเดอร์ถ้ายังไม่มี
        os.makedirs(log_directory, exist_ok=True)
        
        # สร้างไฟล์ CSV ใหม่สำหรับเซสชันนี้
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file_path = os.path.join(log_directory, f'navigation_data_{timestamp}.csv')
        
        # เปิดไฟล์และสร้าง writer
        self.csv_file = open(self.csv_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # เขียน header
        self.csv_writer.writerow([
            'timestamp', 'x', 'y', 'vx', 'yaw', 'steering_angle_rad', 
            'steering_angle_deg', 'current_waypoint', 'distance_to_waypoint',
            'segment_type', 'segment_direction', 'target_heading'
        ])
        self.csv_file.flush()  # บังคับเขียน header ลงดิสก์ทันที
        
        print(f"✅ Data logging initialized: {self.csv_file_path}")
    
    def log_data(self, timestamp, x, y, vx, yaw, steering_angle, 
                 current_waypoint, distance_to_waypoint, segment_type, 
                 segment_direction, target_heading):
        """บันทึกข้อมูลลงใน CSV"""
        try:
            self.csv_writer.writerow([
                timestamp, x, y, vx, yaw, steering_angle, 
                math.degrees(steering_angle), current_waypoint, distance_to_waypoint,
                segment_type, segment_direction, target_heading
            ])
            self.csv_file.flush()  # บังคับเขียนข้อมูลลงดิสก์ทันที
            # เพิ่ม debug log เพื่อตรวจสอบ
            print(f"[LOG] Data logged: WP={current_waypoint}, Dist={distance_to_waypoint:.2f}, Steer={math.degrees(steering_angle):.2f}°")
        except Exception as e:
            print(f"❌ Error logging data: {e}")
    
    def close(self):
        """ปิดไฟล์ CSV"""
        if self.csv_file:
            self.csv_file.close()
            print(f"📁 Data log saved: {self.csv_file_path}")

class WaypointManager:
    def __init__(self, csv_file_path, auto_start=True):
        self.waypoints = []
        self.current_waypoint_index = 0
        self.auto_start = auto_start
        self.navigation_started = False
        self.starting_waypoint = 0
        self.load_waypoints_from_csv(csv_file_path)
        self.path_segments = [
            {'start': 0, 'end': 240, 'type': 'straight', 'direction': 'S-N', 'target_heading': 0.0},
            {'start': 241, 'end': 256, 'type': 'curve', 'direction': 'curve1', 'target_heading': None, 'turn': 'left'},
            {'start': 257, 'end': 315, 'type': 'straight', 'direction': 'E-W', 'target_heading': 335.0, 'reverse_heading': True},
            {'start': 316, 'end': 383, 'type': 'curve', 'direction': 'curve2', 'target_heading': None, 'turn': 'left'},
            {'start': 384, 'end': 496, 'type': 'straight', 'direction': 'N-S', 'target_heading': 0.0, 'reverse_heading': True},
            {'start': 497, 'end': 571, 'type': 'curve', 'direction': 'curve3', 'target_heading': None, 'turn': 'left'},
            {'start': 572, 'end': 622, 'type': 'straight', 'direction': 'W-E', 'target_heading': 18.0, 'reverse_heading': True},
            {'start': 623, 'end': 642, 'type': 'curve', 'direction': 'curve4', 'target_heading': None, 'turn': 'left'},
            {'start': 643, 'end': 654, 'type': 'straight', 'direction': 'S-N', 'target_heading': 0.0}
        ]
        self.curve_radii = {
            'curve1': 1.5,
            'curve2': 30.0,
            'curve3': 5.0,
            'curve4': 5.0
        }
        self.previous_distance = None
        self.distance_increasing_count = 0
        self.max_distance_increasing = 3

    def load_waypoints_from_csv(self, csv_file_path):
        try:
            with open(csv_file_path, 'r') as file:
                csv_reader = csv.DictReader(file)
                for row in csv_reader:
                    waypoint = {
                        'x': float(row['x_east']),
                        'y': float(row['y_north'])
                    }
                    self.waypoints.append(waypoint)
            print(f"✅ Loaded {len(self.waypoints)} waypoints from CSV")
        except Exception as e:
            print(f"❌ Error loading waypoints: {e}")
    
    def find_nearest_forward_waypoint(self, current_position, current_heading, search_radius=50.0):
        if not current_position or not self.waypoints:
            return 0
        current_x, current_y = current_position['x'], current_position['y']
        candidates = []
        for i, waypoint in enumerate(self.waypoints):
            dx = waypoint['x'] - current_x
            dy = waypoint['y'] - current_y
            distance = math.sqrt(dx**2 + dy**2)
            if distance > search_radius:
                continue
            waypoint_bearing = math.degrees(math.atan2(dx, dy))
            if waypoint_bearing < 0:
                waypoint_bearing += 360
            heading_diff = abs(self.normalize_angle_difference(waypoint_bearing - current_heading))
            if heading_diff <= 90.0:
                distance_score = distance
                angle_score = heading_diff / 90.0
                combined_score = distance_score + (angle_score * 10.0)
                candidates.append({'index': i, 'score': combined_score})
        if candidates:
            candidates.sort(key=lambda x: x['score'])
            return candidates[0]['index']
        else:
            return self.find_nearest_waypoint(current_position)
    
    def find_nearest_waypoint(self, current_position):
        if not current_position or not self.waypoints:
            return 0
        current_x, current_y = current_position['x'], current_position['y']
        min_distance = float('inf')
        nearest_index = 0
        for i, waypoint in enumerate(self.waypoints):
            distance = math.sqrt((waypoint['x'] - current_x)**2 + (waypoint['y'] - current_y)**2)
            if distance < min_distance:
                min_distance = distance
                nearest_index = i
        return nearest_index
    
    def initialize_starting_point(self, current_position, current_heading):
        if self.navigation_started:
            return False
        if not current_position or current_heading is None:
            return False
        if self.auto_start:
            self.current_waypoint_index = self.find_nearest_forward_waypoint(current_position, current_heading)
        else:
            self.current_waypoint_index = 0
        self.starting_waypoint = self.current_waypoint_index
        self.navigation_started = True
        return True
    
    def normalize_angle_difference(self, angle_diff):
        return (angle_diff + 180) % 360 - 180
    
    def should_advance_waypoint(self, distance_to_waypoint, waypoint_reached_threshold):
        if distance_to_waypoint < waypoint_reached_threshold:
            return True, "reached_threshold"
        if self.previous_distance is not None:
            if distance_to_waypoint > self.previous_distance:
                self.distance_increasing_count += 1
            else:
                self.distance_increasing_count = 0
        self.previous_distance = distance_to_waypoint
        if (self.distance_increasing_count >= self.max_distance_increasing and 
            distance_to_waypoint > waypoint_reached_threshold * 3):
            return True, "passed_waypoint"
        if distance_to_waypoint > 100.0:
            return True, "too_far_skip"
        return False, "continue"
    
    def get_current_segment_info(self):
        for segment in self.path_segments:
            if segment['start'] <= self.current_waypoint_index <= segment['end']:
                return segment
        return {'type': 'straight', 'direction': 'unknown', 'target_heading': 0.0}
    
    def get_curve_radius(self, curve_direction):
        return self.curve_radii.get(curve_direction, 15.0)
    
    def get_current_waypoint(self):
        if self.current_waypoint_index < len(self.waypoints):
            return self.waypoints[self.current_waypoint_index]
        return None
    
    def advance_waypoint(self):
        if self.current_waypoint_index < len(self.waypoints) - 1:
            self.current_waypoint_index += 1
            self.previous_distance = None
            self.distance_increasing_count = 0
            return True
        return False
    
    def get_progress_info(self):
        return {
            'current': self.current_waypoint_index + 1,
            'total': len(self.waypoints),
            'starting': self.starting_waypoint + 1,
            'progress_percent': ((self.current_waypoint_index + 1) / len(self.waypoints)) * 100,
            'from_start_count': self.current_waypoint_index - self.starting_waypoint + 1
        }

class BicycleKinematicModel:
    def __init__(self, wheelbase):
        self.wheelbase = wheelbase
        self.current_target_heading = 0.0
        self.heading_initialized = False
    
    def set_target_heading(self, target_heading):
        self.current_target_heading = target_heading
    
    def get_current_target_heading(self):
        return self.current_target_heading
    
    def force_initial_heading(self, current_heading):
        if not self.heading_initialized:
            self.current_target_heading = 0.0
            self.heading_initialized = True
            return True
        return False
    
    def calculate_angular_velocity(self, linear_velocity, steering_angle):
        if abs(steering_angle) < 1e-6:
            return 0.0
        return (linear_velocity * math.tan(steering_angle)) / self.wheelbase
    
    def limit_steering_angle(self, steering_angle, max_steering_angle):
        return max(-max_steering_angle, min(max_steering_angle, steering_angle))
    
    def calculate_heading_correction_steering(self, current_compass_heading, target_compass_heading, kp=3.5):
        heading_error = self.normalize_angle_difference(current_compass_heading - target_compass_heading)
        steering_angle = kp * math.radians(heading_error)
        return steering_angle, heading_error
    
    def normalize_angle_difference(self, angle_diff):
        return (angle_diff + 180) % 360 - 180

class PurePursuitController:
    def __init__(self, lookahead_distance=2.0):
        self.lookahead_distance = lookahead_distance
    
    def calculate_steering_angle(self, current_pos, current_compass_heading, target_pos, wheelbase):
        dx = target_pos['x'] - current_pos['x']
        dy = target_pos['y'] - current_pos['y']
        target_compass_bearing = math.degrees(math.atan2(dx, dy))
        if target_compass_bearing < 0:
            target_compass_bearing += 360
        heading_error = self.normalize_angle(target_compass_bearing - current_compass_heading)
        distance = math.sqrt(dx**2 + dy**2)
        alpha = math.radians(heading_error)
        steering_angle = math.atan2(2 * wheelbase * math.sin(alpha), distance)
        return steering_angle
    
    def normalize_angle(self, angle):
        return (angle + 180) % 360 - 180

class ConstantCurvatureController:
    def __init__(self):
        self.curvature_history = deque(maxlen=5)
    
    def calculate_steering_angle_with_radius(self, curve_radius, linear_velocity, wheelbase, segment_info=None):
        curvature = 1.0 / abs(curve_radius)
        steering_angle = math.atan(wheelbase * curvature)
        steering_angle = abs(steering_angle)
        return steering_angle

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        self.xy_subscriber = self.create_subscription(
            Float64MultiArray, '/navigation/xy', self.xy_callback, 10)
        self.heading_subscriber = self.create_subscription(
            Float32, '/navigation/heading', self.heading_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.navigation_debug_publisher = self.create_publisher(String, '/navigation/debug', 10)
        
        # เพิ่ม DataLogger
        self.data_logger = DataLogger()
        
        self.wheelbase = 1.67
        self.max_steering_angle = 0.873
        self.fixed_speed = 1.0
        self.max_angular_velocity = 1.5
        self.current_position = {'x': 0.0, 'y': 0.0}
        self.current_heading = None
        self.previous_position = {'x': 0.0, 'y': 0.0}
        self.previous_time = None
        self.current_vx = 0.0  # คำนวณความเร็วในทิศทาง x
        
        csv_file_path = '/home/inc/ros2_ws/waypointfile/waypoints3xy.csv'
        self.waypoint_manager = WaypointManager(csv_file_path, auto_start=True)
        self.pure_pursuit = PurePursuitController(lookahead_distance=3.0)
        self.curvature_controller = ConstantCurvatureController()
        self.bicycle_model = BicycleKinematicModel(wheelbase=self.wheelbase)
        self.navigation_active = False
        self.waypoint_reached_threshold = 3.0
        self.heading_forced_to_zero = False
        self.stats = {'navigation_commands': 0, 'heading_corrections': 0}
        self.last_progress_log_time = 0
        self.progress_log_interval = 2.0
        
        # ตัวแปรสำหรับบันทึก steering angle
        self.current_steering_angle = 0.0
        
        self.create_timer(0.1, self.navigation_control)
        self.get_logger().info('✅ Navigation Controller with CSV Logging initialized')
        self.get_logger().info(f'🚗 Vehicle: wheelbase={self.wheelbase}m, max_steering={math.degrees(self.max_steering_angle):.1f}°')
        self.get_logger().info(f'🚀 Fixed Speed: {self.fixed_speed} m/s')
        self.get_logger().info(f'🗺️ Loaded {len(self.waypoint_manager.waypoints)} waypoints')
        self.get_logger().info(f'🎯 Waypoint threshold: {self.waypoint_reached_threshold}m (with smart advance)')
        self.get_logger().info(f'📁 Data logging to: {self.data_logger.csv_file_path}')
        
    def xy_callback(self, msg):
        if len(msg.data) >= 2:
            # คำนวณความเร็วในทิศทาง x
            current_time = self.get_clock().now().nanoseconds / 1e9
            if self.previous_time is not None:
                dt = current_time - self.previous_time
                if dt > 0:
                    dx = msg.data[0] - self.previous_position['x']
                    self.current_vx = dx / dt
            
            # อัปเดต position
            self.previous_position['x'] = self.current_position['x']
            self.previous_position['y'] = self.current_position['y']
            self.current_position['x'] = msg.data[0]
            self.current_position['y'] = msg.data[1]
            self.previous_time = current_time
    
    def heading_callback(self, msg):
        self.current_heading = msg.data

    def navigation_control(self):
        if not self.waypoint_manager.navigation_started:
            if self.current_heading is not None:
                if not self.heading_forced_to_zero:
                    self.bicycle_model.force_initial_heading(self.current_heading)
                    self.heading_forced_to_zero = True
                    self.get_logger().info(f'🔧 FORCING heading from {self.current_heading:.1f}° to 0.0°')
                if self.waypoint_manager.initialize_starting_point(self.current_position, self.current_heading):
                    self.navigation_active = True
                    progress = self.waypoint_manager.get_progress_info()
                    self.get_logger().info(f'🚀 Navigation started! Target heading: 0.0°')
                    self.get_logger().info(f'📍 Starting from waypoint {progress["starting"]} of {progress["total"]}')
                    self.get_logger().info(f'🎯 Current target: waypoint {progress["current"]}')
            return
        if not self.navigation_active or self.current_heading is None:
            return
        current_waypoint = self.waypoint_manager.get_current_waypoint()
        if current_waypoint is None:
            self.get_logger().info('🏁 Navigation completed - reached final waypoint')
            self.navigation_active = False
            self.send_stop_command()
            return
        distance_to_waypoint = math.sqrt(
            (self.current_position['x'] - current_waypoint['x'])**2 +
            (self.current_position['y'] - current_waypoint['y'])**2
        )
        should_advance, advance_reason = self.waypoint_manager.should_advance_waypoint(
            distance_to_waypoint, self.waypoint_reached_threshold
        )
        if should_advance:
            if self.waypoint_manager.advance_waypoint():
                progress = self.waypoint_manager.get_progress_info()
                segment_info = self.waypoint_manager.get_current_segment_info()
                reason_text = {
                    "reached_threshold": "REACHED (within threshold)",
                    "passed_waypoint": "PASSED (moving away)",
                    "too_far_skip": "SKIPPED (too far)"
                }.get(advance_reason, advance_reason)
                self.get_logger().info(
                    f'✅ {reason_text} waypoint {progress["current"]-1}! '
                    f'Progress: {progress["from_start_count"]-1}/{progress["total"] - progress["starting"]} '
                    f'({progress["progress_percent"]:.1f}% complete)'
                )
                self.get_logger().info(
                    f'🎯 NEXT TARGET: waypoint {progress["current"]} '
                    f'({segment_info["type"].upper()}: {segment_info["direction"]})'
                )
            else:
                self.get_logger().info('🏁 Navigation completed - ALL WAYPOINTS REACHED!')
                self.navigation_active = False
                self.send_stop_command()
                return
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_progress_log_time > self.progress_log_interval:
            self.log_current_progress(distance_to_waypoint)
            self.last_progress_log_time = current_time
        segment_info = self.waypoint_manager.get_current_segment_info()
        segment_type = segment_info['type']
        direction = segment_info['direction']
        target_heading = segment_info.get('target_heading')
        reverse = segment_info.get('reverse_heading', False)
        if segment_type == 'straight' and target_heading is not None:
            self.bicycle_model.set_target_heading(target_heading)
        linear_velocity = self.fixed_speed
        if segment_type == 'straight' and segment_info.get('radius') is not None:
            radius = segment_info['radius']
            steering_angle = self.curvature_controller.calculate_steering_angle_with_radius(
                radius, self.fixed_speed, self.wheelbase, segment_info
            )
            heading_error = 0.0
        elif segment_type == 'straight':
            # สลับทิศทาง heading correction ตาม reverse flag
            if reverse:
                steering_angle, heading_error = self.calculate_straight_path_control_reverse(
                    current_waypoint, self.bicycle_model.get_current_target_heading())
            else:
                steering_angle, heading_error = self.calculate_straight_path_control(
                    current_waypoint, self.bicycle_model.get_current_target_heading())
        else:
            steering_angle = self.calculate_curve_path_control(segment_info)
            heading_error = 0.0
        steering_angle = self.bicycle_model.limit_steering_angle(
            steering_angle, self.max_steering_angle
        )
        
        # *** จุดสำคัญ: เรียก log_data ที่นี่ ***
        if self.navigation_active and self.current_heading is not None:
            try:
                self.data_logger.log_data(
                    timestamp=current_time,
                    x=self.current_position['x'],
                    y=self.current_position['y'],
                    vx=self.current_vx,
                    yaw=self.current_heading,
                    steering_angle=steering_angle,
                    current_waypoint=self.waypoint_manager.current_waypoint_index,
                    distance_to_waypoint=distance_to_waypoint,
                    segment_type=segment_type,
                    segment_direction=direction,
                    target_heading=target_heading if target_heading is not None else 0.0
                )
            except Exception as e:
                self.get_logger().error(f"❌ Failed to log data: {e}")
        
        angular_velocity = self.bicycle_model.calculate_angular_velocity(
            linear_velocity, steering_angle
        )
        angular_velocity = max(-self.max_angular_velocity,
                             min(self.max_angular_velocity, angular_velocity))
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_velocity
        cmd_vel.angular.z = angular_velocity
        self.cmd_vel_publisher.publish(cmd_vel)
        self.stats['navigation_commands'] += 1
        self.publish_debug_info(segment_info, distance_to_waypoint, steering_angle,
                               linear_velocity, angular_velocity, heading_error)
    
    def log_current_progress(self, distance_to_waypoint):
        progress = self.waypoint_manager.get_progress_info()
        segment_info = self.waypoint_manager.get_current_segment_info()
        distance_trend = ""
        if self.waypoint_manager.previous_distance is not None:
            if distance_to_waypoint > self.waypoint_manager.previous_distance:
                distance_trend = f"↗️ (+{self.waypoint_manager.distance_increasing_count})"
            else:
                distance_trend = "↘️"
        self.get_logger().info(
            f'🚗 NAVIGATING: waypoint {progress["current"]}/{progress["total"]} '
            f'| Progress from start: {progress["from_start_count"]}/{progress["total"] - progress["starting"]} '
            f'| Distance: {distance_to_waypoint:.2f}m {distance_trend} '
            f'| Type: {segment_info["type"].upper()} ({segment_info["direction"]}) '
            f'| Heading: {self.current_heading:.1f}°'
        )
    
    def calculate_straight_path_control(self, target_waypoint, target_heading):
        heading_correction_steering, heading_error = self.bicycle_model.calculate_heading_correction_steering(
            self.current_heading,
            target_heading,
            3.5
        )
        pure_pursuit_steering = self.pure_pursuit.calculate_steering_angle(
            self.current_position,
            self.current_heading,
            target_waypoint,
            self.wheelbase
        )
        combined_steering = 0.95 * heading_correction_steering + 0.05 * pure_pursuit_steering
        if abs(heading_correction_steering) > 0.01:
            self.stats['heading_corrections'] += 1
        return combined_steering, heading_error

    def calculate_straight_path_control_reverse(self, target_waypoint, target_heading):
        # สลับลำดับเพื่อกลับทิศทาง correction
        heading_correction_steering, heading_error = self.bicycle_model.calculate_heading_correction_steering(
            target_heading,
            self.current_heading,
            3.5
        )
        pure_pursuit_steering = self.pure_pursuit.calculate_steering_angle(
            self.current_position,
            self.current_heading,
            target_waypoint,
            self.wheelbase
        )
        combined_steering = 0.95 * heading_correction_steering + 0.05 * pure_pursuit_steering
        if abs(heading_correction_steering) > 0.01:
            self.stats['heading_corrections'] += 1
        return combined_steering, heading_error
    
    def calculate_curve_path_control(self, segment_info):
        curve_direction = segment_info['direction']
        curve_radius = self.waypoint_manager.get_curve_radius(curve_direction)
        steering_angle = self.curvature_controller.calculate_steering_angle_with_radius(
            curve_radius,
            self.fixed_speed,
            self.wheelbase,
            segment_info
        )
        return steering_angle
    
    def publish_debug_info(self, segment_info, distance_to_waypoint, steering_angle,
                          linear_velocity, angular_velocity, heading_error):
        debug_msg = String()
        progress = self.waypoint_manager.get_progress_info()
        segment_type = segment_info['type']
        direction = segment_info['direction']
        target_heading = segment_info.get('target_heading')
        turn_direction = segment_info.get('turn', 'left')
        if segment_type == 'curve':
            curve_radius = self.waypoint_manager.get_curve_radius(direction)
            debug_msg.data = (
                f"🔄 CURVE ({direction}, R={curve_radius:.1f}m, Turn={turn_direction.upper()}) | "
                f"WP: {progress['current']}/{progress['total']} (Start:{progress['starting']}) | "
                f"Progress: {progress['from_start_count']} waypoints from start | "
                f"Dist: {distance_to_waypoint:.2f}m | "
                f"Steer: {math.degrees(steering_angle):.1f}° | "
                f"Speed: {linear_velocity:.2f}m/s | "
                f"Angular: {angular_velocity:.2f}rad/s | "
                f"Turn: {'LEFT(+)' if angular_velocity > 0 else 'RIGHT(-)' if angular_velocity < 0 else 'STRAIGHT'}"
            )
        else:
            target_heading_str = f"{target_heading:.0f}" if target_heading is not None else "None"
            current_heading_str = f"{self.current_heading:.1f}" if self.current_heading is not None else "None"
            debug_msg.data = (
                f"➡️ STRAIGHT ({direction}, Target={target_heading_str}°) | "
                f"WP: {progress['current']}/{progress['total']} (Start:{progress['starting']}) | "
                f"Progress: {progress['from_start_count']} waypoints from start | "
                f"Dist: {distance_to_waypoint:.2f}m | "
                f"Heading: {current_heading_str}° (Error: {heading_error:.1f}°) | "
                f"Steer: {math.degrees(steering_angle):.1f}° | "
                f"Speed: {linear_velocity:.2f}m/s | "
                f"Angular: {angular_velocity:.2f}rad/s | "
                f"Turn: {'LEFT(+)' if angular_velocity > 0 else 'RIGHT(-)' if angular_velocity < 0 else 'STRAIGHT'}"
            )
        self.navigation_debug_publisher.publish(debug_msg)
    
    def send_stop_command(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd_vel)
        self.get_logger().info('🛑 Stop command sent')
    
    def destroy_node(self):
        self.get_logger().info("🔄 Shutting down Navigation Controller...")
        if hasattr(self, 'navigation_active') and self.navigation_active:
            self.send_stop_command()
        self.print_final_statistics()
        super().destroy_node()
    
    def print_final_statistics(self):
        try:
            self.get_logger().info("📊 Final Navigation Statistics:")
            self.get_logger().info(f"  🚗 Navigation commands: {self.stats['navigation_commands']}")
            self.get_logger().info(f"  🔧 Heading corrections: {self.stats['heading_corrections']}")
            if hasattr(self, 'waypoint_manager') and self.waypoint_manager:
                progress = self.waypoint_manager.get_progress_info()
                self.get_logger().info(f"  🎯 Final waypoint: {progress['current']}/{progress['total']} ({progress['progress_percent']:.1f}%)")
                self.get_logger().info(f"  🚀 Started from waypoint: {progress['starting']}")
                self.get_logger().info(f"  📈 Completed waypoints: {progress['from_start_count']}")
        except Exception as e:
            self.get_logger().error(f"❌ Error printing statistics: {e}")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = NavigationController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n🛑 Navigation Controller stopped by user')
    except Exception as e:
        print(f'❌ Error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32, String, Float64MultiArray
# from geometry_msgs.msg import Twist
# import math
# import asyncio
# import threading
# import json
# import websockets
# import csv
# from collections import deque
# import logging

# # Set up logging for WebSocket
# logging.basicConfig(level=logging.INFO)
# logger = logging.getLogger(__name__)

# # --------- Haversine ---------
# def haversine(lat1, lon1, lat2, lon2):
#     R = 6371000  # meters
#     phi1 = math.radians(lat1)
#     phi2 = math.radians(lat2)
#     d_phi = math.radians(lat2 - lat1)
#     d_lambda = math.radians(lon2 - lon1)
#     a = (
#         math.sin(d_phi / 2) ** 2 +
#         math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2) ** 2
#     )
#     return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

# # --------- Enhanced WebSocket Server ---------
# class WebSocketDestinationServer:
#     def __init__(self, controller, port=5000):
#         self.destinations = []  # เก็บรายการจุดหมายปลายทาง (lat, lon)
#         self.port = port
#         self.controller = controller
#         self.active_websockets = set()
        
#         # Building destinations from HTML file
#         self.available_destinations = {
#             "S14": {"lat": 13.650940, "lon": 100.492070, "name": "King Mongkut's 190th Anniversary Building"},
#             "S13": {"lat": 13.650076, "lon": 100.492140, "name": "Classroom Building 3"},
#             "S12": {"lat": 13.649913, "lon": 100.492690, "name": "Classroom Building 4"},
#             "S11": {"lat": 13.649992, "lon": 100.493327, "name": "Classroom Building 5"},
#             "S15": {"lat": 13.650154, "lon": 100.492984, "name": "Department of Chemical Engineering Building"},
#             "N19": {"lat": 13.651010, "lon": 100.492980, "name": "Production Engineering Laboratory Building 5"},
#             "N20": {"lat": 13.6513894, "lon": 100.4929824, "name": "Classroom Building 1"}
#         }
        
#         self.current_destination = None
#         self.current_bearing = None

#     def add_destination(self, lat, lon, name=None):
#         """เพิ่มจุดหมายปลายทางใหม่"""
#         destination = {"lat": lat, "lon": lon}
#         if name:
#             destination["name"] = name
#         self.destinations.append(destination)
#         logger.info(f"✅ Destination added: ({lat}, {lon}) - {name}")

#     def set_destination_by_code(self, destination_code):
#         """ตั้งจุดหมายปลายทางตามรหัสอาคาร"""
#         if destination_code in self.available_destinations:
#             dest_info = self.available_destinations[destination_code]
#             self.current_destination = {
#                 "code": destination_code,
#                 "lat": dest_info["lat"],
#                 "lon": dest_info["lon"],
#                 "name": dest_info["name"]
#             }
#             logger.info(f"🎯 Destination set: {destination_code} - {dest_info['name']}")
#             return True
#         else:
#             logger.warning(f"❌ Unknown destination code: {destination_code}")
#             return False

#     def get_destinations(self):
#         """คืนค่ารายการจุดหมายปลายทางทั้งหมด"""
#         return self.destinations

#     def get_current_destination(self):
#         """คืนค่าจุดหมายปลายทางปัจจุบัน"""
#         return self.current_destination

#     def set_bearing(self, bearing):
#         """บันทึกทิศทางปัจจุบันจาก HTML"""
#         self.current_bearing = bearing

#     def get_bearing(self):
#         """คืนค่าทิศทางปัจจุบัน"""
#         return self.current_bearing

#     def remove_reached(self, index):
#         """ลบจุดหมายปลายทางที่ถึงแล้ว"""
#         if 0 <= index < len(self.destinations):
#             removed = self.destinations.pop(index)
#             logger.info(f"✅ Removed reached destination: {removed}")

#     async def handler(self, websocket, path):
#         """จัดการเมื่อมี client เชื่อมต่อ"""
#         client_address = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
#         logger.info(f"🔌 Client connected: {client_address}")
#         self.active_websockets.add(websocket)
        
#         try:
#             # Send welcome message with available destinations
#             welcome_msg = {
#                 "type": "welcome",
#                 "message": "Connected to GNSS Navigation Server",
#                 "available_destinations": self.available_destinations
#             }
#             await websocket.send(json.dumps(welcome_msg))
            
#             async for message in websocket:
#                 try:
#                     data = json.loads(message)
#                     await self.process_message(data, websocket)
#                 except json.JSONDecodeError as e:
#                     logger.error(f"❌ JSON decode error: {e}")
#                     error_msg = {"type": "error", "message": "Invalid JSON format"}
#                     await websocket.send(json.dumps(error_msg))
#                 except Exception as e:
#                     logger.error(f"❌ Error processing message: {e}")
#                     error_msg = {"type": "error", "message": str(e)}
#                     await websocket.send(json.dumps(error_msg))
                    
#         except websockets.exceptions.ConnectionClosed:
#             logger.info(f"🔌 Client disconnected: {client_address}")
#         except Exception as e:
#             logger.error(f"❌ WebSocket handler error: {e}")
#         finally:
#             if websocket in self.active_websockets:
#                 self.active_websockets.remove(websocket)

#     async def process_message(self, data, websocket):
#         """ประมวลผลข้อความที่ได้รับจาก client"""
        
#         # Handle destination setting
#         if "destination" in data:
#             destination_code = data["destination"]
#             if self.set_destination_by_code(destination_code):
#                 # Add to destinations list
#                 dest_info = self.available_destinations[destination_code]
#                 self.add_destination(dest_info["lat"], dest_info["lon"], dest_info["name"])
                
#                 # Send confirmation
#                 response = {
#                     "type": "destination_set",
#                     "destination": destination_code,
#                     "coordinates": {"lat": dest_info["lat"], "lon": dest_info["lon"]},
#                     "name": dest_info["name"]
#                 }
#                 await websocket.send(json.dumps(response))
#                 logger.info(f"📤 Sent destination confirmation: {destination_code}")
        
#         # Handle bearing update
#         elif "bearing" in data:
#             bearing = float(data["bearing"])
#             self.set_bearing(bearing)
#             logger.debug(f"🧭 Bearing updated: {bearing:.2f}°")
        
#         # Handle direct GPS coordinates
#         elif "lat" in data and "lon" in data:
#             lat = float(data["lat"])
#             lon = float(data["lon"])
#             name = data.get("name", "Custom Location")
#             self.add_destination(lat, lon, name)
            
#             response = {
#                 "type": "gps_destination_set",
#                 "coordinates": {"lat": lat, "lon": lon},
#                 "name": name
#             }
#             await websocket.send(json.dumps(response))
#             logger.info(f"📤 Sent GPS destination confirmation: ({lat}, {lon})")
        
#         # Handle status request
#         elif data.get("type") == "status_request":
#             status_msg = {
#                 "type": "status_response",
#                 "current_position": {
#                     "lat": self.controller.current_lat,
#                     "lon": self.controller.current_lon
#                 },
#                 "current_destination": self.current_destination,
#                 "navigation_active": self.controller.navigation_active,
#                 "heading": self.controller.current_heading
#             }
#             await websocket.send(json.dumps(status_msg))
        
#         else:
#             logger.warning(f"❓ Unknown message format: {data}")

#     async def broadcast_position(self):
#         """ส่งตำแหน่งปัจจุบัน (lat, lon) ไปยังทุก client ทุก ๆ 1 วินาที"""
#         while True:
#             if self.active_websockets:
#                 lat = self.controller.current_lat
#                 lon = self.controller.current_lon
#                 heading = self.controller.current_heading
                
#                 if lat is not None and lon is not None:
#                     message = {
#                         "type": "position_update",
#                         "latitude": lat,
#                         "longitude": lon,
#                         "heading": heading,
#                         "timestamp": asyncio.get_event_loop().time()
#                     }
                    
#                     # Add navigation status if available
#                     if hasattr(self.controller, 'waypoint_manager') and self.controller.waypoint_manager.navigation_started:
#                         progress = self.controller.waypoint_manager.get_progress_info()
#                         message["navigation_status"] = {
#                             "active": self.controller.navigation_active,
#                             "current_waypoint": progress["current"],
#                             "total_waypoints": progress["total"],
#                             "progress_percent": progress["progress_percent"]
#                         }
                    
#                     websockets_to_remove = set()
#                     for ws in list(self.active_websockets):  # Create a copy to avoid modification during iteration
#                         try:
#                             await ws.send(json.dumps(message))
#                         except websockets.exceptions.ConnectionClosed:
#                             websockets_to_remove.add(ws)
#                         except Exception as e:
#                             logger.error(f"❌ Error sending to client: {e}")
#                             websockets_to_remove.add(ws)
                    
#                     # Remove disconnected websockets
#                     self.active_websockets -= websockets_to_remove
                    
#             await asyncio.sleep(1.0)

#     def run_server(self):
#         """รัน WebSocket server"""
#         try:
#             asyncio.set_event_loop(asyncio.new_event_loop())
#             loop = asyncio.get_event_loop()
            
#             # Start server
#             start_server = websockets.serve(
#                 self.handler, 
#                 "0.0.0.0", 
#                 self.port,
#                 ping_interval=30,  # Send ping every 30 seconds
#                 ping_timeout=10,   # Wait 10 seconds for pong
#                 close_timeout=10   # Wait 10 seconds for close
#             )
            
#             loop.run_until_complete(start_server)
#             loop.create_task(self.broadcast_position())
            
#             logger.info(f"🌐 WebSocket server started on ws://0.0.0.0:{self.port}")
#             logger.info(f"📍 Available destinations: {', '.join(self.available_destinations.keys())}")
            
#             loop.run_forever()
            
#         except Exception as e:
#             logger.error(f"❌ WebSocket server error: {e}")

# # --------- WaypointManager ---------
# class WaypointManager:
#     def __init__(self, csv_file_path, auto_start=True):
#         self.waypoints = []
#         self.current_waypoint_index = 0
#         self.auto_start = auto_start
#         self.navigation_started = False
#         self.starting_waypoint = 0
#         self.load_waypoints_from_csv(csv_file_path)
#         self.path_segments = [
#             {'start': 0, 'end': 240, 'type': 'straight', 'direction': 'S-N', 'target_heading': 0.0},
#             {'start': 241, 'end': 256, 'type': 'curve', 'direction': 'curve1', 'target_heading': None, 'turn': 'left'},
#             {'start': 257, 'end': 315, 'type': 'straight', 'direction': 'E-W', 'target_heading': 335.0, 'reverse_heading': True},
#             {'start': 316, 'end': 383, 'type': 'curve', 'direction': 'curve2', 'target_heading': None, 'turn': 'left'},
#             {'start': 384, 'end': 496, 'type': 'straight', 'direction': 'N-S', 'target_heading': 0.0, 'reverse_heading': True},
#             {'start': 497, 'end': 571, 'type': 'curve', 'direction': 'curve3', 'target_heading': None, 'turn': 'left'},
#             {'start': 572, 'end': 622, 'type': 'straight', 'direction': 'W-E', 'target_heading': 18.0, 'reverse_heading': True},
#             {'start': 623, 'end': 642, 'type': 'curve', 'direction': 'curve4', 'target_heading': None, 'turn': 'left'},
#             {'start': 643, 'end': 654, 'type': 'straight', 'direction': 'S-N', 'target_heading': 0.0}
#         ]
#         self.curve_radii = {
#             'curve1': 1.5,
#             'curve2': 30.0,
#             'curve3': 5.0,
#             'curve4': 5.0
#         }
#         self.previous_distance = None
#         self.distance_increasing_count = 0
#         self.max_distance_increasing = 3

#     def load_waypoints_from_csv(self, csv_file_path):
#         try:
#             with open(csv_file_path, 'r') as file:
#                 csv_reader = csv.DictReader(file)
#                 for row in csv_reader:
#                     waypoint = {
#                         'x': float(row['x_east']),
#                         'y': float(row['y_north'])
#                     }
#                     self.waypoints.append(waypoint)
#             print(f"✅ Loaded {len(self.waypoints)} waypoints from CSV")
#         except Exception as e:
#             print(f"❌ Error loading waypoints: {e}")

#     def find_nearest_forward_waypoint(self, current_position, current_heading, search_radius=50.0):
#         if not current_position or not self.waypoints:
#             return 0
#         current_x, current_y = current_position['x'], current_position['y']
#         candidates = []
#         for i, waypoint in enumerate(self.waypoints):
#             dx = waypoint['x'] - current_x
#             dy = waypoint['y'] - current_y
#             distance = math.sqrt(dx**2 + dy**2)
#             if distance > search_radius:
#                 continue
#             waypoint_bearing = math.degrees(math.atan2(dx, dy))
#             if waypoint_bearing < 0:
#                 waypoint_bearing += 360
#             heading_diff = abs(self.normalize_angle_difference(waypoint_bearing - current_heading))
#             if heading_diff <= 90.0:
#                 distance_score = distance
#                 angle_score = heading_diff / 90.0
#                 combined_score = distance_score + (angle_score * 10.0)
#                 candidates.append({'index': i, 'score': combined_score})
#         if candidates:
#             candidates.sort(key=lambda x: x['score'])
#             return candidates[0]['index']
#         else:
#             return self.find_nearest_waypoint(current_position)

#     def find_nearest_waypoint(self, current_position):
#         if not current_position or not self.waypoints:
#             return 0
#         current_x, current_y = current_position['x'], current_position['y']
#         min_distance = float('inf')
#         nearest_index = 0
#         for i, waypoint in enumerate(self.waypoints):
#             distance = math.sqrt((waypoint['x'] - current_x)**2 + (waypoint['y'] - current_y)**2)
#             if distance < min_distance:
#                 min_distance = distance
#                 nearest_index = i
#         return nearest_index

#     def initialize_starting_point(self, current_position, current_heading):
#         if self.navigation_started:
#             return False
#         if not current_position or current_heading is None:
#             return False
#         if self.auto_start:
#             self.current_waypoint_index = self.find_nearest_forward_waypoint(current_position, current_heading)
#         else:
#             self.current_waypoint_index = 0
#         self.starting_waypoint = self.current_waypoint_index
#         self.navigation_started = True
#         return True

#     def normalize_angle_difference(self, angle_diff):
#         return (angle_diff + 180) % 360 - 180

#     def should_advance_waypoint(self, distance_to_waypoint, waypoint_reached_threshold):
#         if distance_to_waypoint < waypoint_reached_threshold:
#             return True, "reached_threshold"
#         if self.previous_distance is not None:
#             if distance_to_waypoint > self.previous_distance:
#                 self.distance_increasing_count += 1
#             else:
#                 self.distance_increasing_count = 0
#         self.previous_distance = distance_to_waypoint
#         if (self.distance_increasing_count >= self.max_distance_increasing and 
#             distance_to_waypoint > waypoint_reached_threshold * 3):
#             return True, "passed_waypoint"
#         if distance_to_waypoint > 100.0:
#             return True, "too_far_skip"
#         return False, "continue"

#     def get_current_segment_info(self):
#         for segment in self.path_segments:
#             if segment['start'] <= self.current_waypoint_index <= segment['end']:
#                 return segment
#         return {'type': 'straight', 'direction': 'unknown', 'target_heading': 0.0}

#     def get_curve_radius(self, curve_direction):
#         return self.curve_radii.get(curve_direction, 15.0)
    
#     def get_current_waypoint(self):
#         if self.current_waypoint_index < len(self.waypoints):
#             return self.waypoints[self.current_waypoint_index]
#         return None
    
#     def advance_waypoint(self):
#         if self.current_waypoint_index < len(self.waypoints) - 1:
#             self.current_waypoint_index += 1
#             self.previous_distance = None
#             self.distance_increasing_count = 0
#             return True
#         return False
    
#     def get_progress_info(self):
#         return {
#             'current': self.current_waypoint_index + 1,
#             'total': len(self.waypoints),
#             'starting': self.starting_waypoint + 1,
#             'progress_percent': ((self.current_waypoint_index + 1) / len(self.waypoints)) * 100,
#             'from_start_count': self.current_waypoint_index - self.starting_waypoint + 1
#         }

# # --------- Controller classes ---------
# class BicycleKinematicModel:
#     def __init__(self, wheelbase):
#         self.wheelbase = wheelbase
#         self.current_target_heading = 0.0
#         self.heading_initialized = False
    
#     def set_target_heading(self, target_heading):
#         self.current_target_heading = target_heading
    
#     def get_current_target_heading(self):
#         return self.current_target_heading
    
#     def force_initial_heading(self, current_heading):
#         if not self.heading_initialized:
#             self.current_target_heading = 0.0
#             self.heading_initialized = True
#             return True
#         return False
    
#     def calculate_angular_velocity(self, linear_velocity, steering_angle):
#         if abs(steering_angle) < 1e-6:
#             return 0.0
#         return (linear_velocity * math.tan(steering_angle)) / self.wheelbase
    
#     def limit_steering_angle(self, steering_angle, max_steering_angle):
#         return max(-max_steering_angle, min(max_steering_angle, steering_angle))
    
#     def calculate_heading_correction_steering(self, current_compass_heading, target_compass_heading, kp=3.5):
#         heading_error = self.normalize_angle_difference(current_compass_heading - target_compass_heading)
#         steering_angle = kp * math.radians(heading_error)
#         return steering_angle, heading_error
    
#     def normalize_angle_difference(self, angle_diff):
#         return (angle_diff + 180) % 360 - 180

# class PurePursuitController:
#     def __init__(self, lookahead_distance=2.0):
#         self.lookahead_distance = lookahead_distance
    
#     def calculate_steering_angle(self, current_pos, current_compass_heading, target_pos, wheelbase):
#         dx = target_pos['x'] - current_pos['x']
#         dy = target_pos['y'] - current_pos['y']
#         target_compass_bearing = math.degrees(math.atan2(dx, dy))
#         if target_compass_bearing < 0:
#             target_compass_bearing += 360
#         heading_error = self.normalize_angle(target_compass_bearing - current_compass_heading)
#         distance = math.sqrt(dx**2 + dy**2)
#         alpha = math.radians(heading_error)
#         steering_angle = math.atan2(2 * wheelbase * math.sin(alpha), distance)
#         return steering_angle
    
#     def normalize_angle(self, angle):
#         return (angle + 180) % 360 - 180

# class ConstantCurvatureController:
#     def __init__(self):
#         self.curvature_history = deque(maxlen=5)
    
#     def calculate_steering_angle_with_radius(self, curve_radius, linear_velocity, wheelbase, segment_info=None):
#         curvature = 1.0 / abs(curve_radius)
#         steering_angle = math.atan(wheelbase * curvature)
#         steering_angle = abs(steering_angle)
#         return steering_angle

# # --------- NavigationController ---------
# class NavigationController(Node):
#     def __init__(self):
#         super().__init__('navigation_controller')
        
#         # ROS2 subscribers and publishers
#         self.xy_subscriber = self.create_subscription(
#             Float64MultiArray, '/navigation/xy', self.xy_callback, 10)
#         self.heading_subscriber = self.create_subscription(
#             Float32, '/navigation/heading', self.heading_callback, 10)
#         self.gps_subscriber = self.create_subscription(
#             Float64MultiArray, '/navigation/raw_gps', self.gps_callback, 10)
#         self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.navigation_debug_publisher = self.create_publisher(String, '/navigation/debug', 10)
        
#         # Vehicle parameters
#         self.wheelbase = 1.67
#         self.max_steering_angle = 0.873
#         self.fixed_speed = 1.0
#         self.max_angular_velocity = 1.5
        
#         # Current state
#         self.current_position = {'x': 0.0, 'y': 0.0}
#         self.current_heading = None
#         self.current_lat = None
#         self.current_lon = None
        
#         # Load waypoints
#         csv_file_path = '/home/inc/ros2_ws/waypointfile/waypoints3xy.csv'
#         self.waypoint_manager = WaypointManager(csv_file_path, auto_start=True)
        
#         # Controllers
#         self.pure_pursuit = PurePursuitController(lookahead_distance=3.0)
#         self.curvature_controller = ConstantCurvatureController()
#         self.bicycle_model = BicycleKinematicModel(wheelbase=self.wheelbase)
        
#         # Navigation state
#         self.navigation_active = False
#         self.waypoint_reached_threshold = 3.0
#         self.heading_forced_to_zero = False
        
#         # Statistics
#         self.stats = {'navigation_commands': 0, 'heading_corrections': 0}
#         self.last_progress_log_time = 0
#         self.progress_log_interval = 2.0
        
#         # Main control timer
#         self.create_timer(0.1, self.navigation_control)
        
#         # Initialize WebSocket server
#         self.websocket_server = WebSocketDestinationServer(self, port=5000)
#         threading.Thread(target=self.websocket_server.run_server, daemon=True).start()
        
#         # Check destinations proximity
#         self.create_timer(0.2, self.check_destinations_proximity)
        
#         # Log initialization
#         self.get_logger().info('✅ Enhanced Navigation Controller initialized')
#         self.get_logger().info(f'🚗 Vehicle: wheelbase={self.wheelbase}m, max_steering={math.degrees(self.max_steering_angle):.1f}°')
#         self.get_logger().info(f'🚀 Fixed Speed: {self.fixed_speed} m/s')
#         self.get_logger().info(f'🗺️ Loaded {len(self.waypoint_manager.waypoints)} waypoints')
#         self.get_logger().info(f'🎯 Waypoint threshold: {self.waypoint_reached_threshold}m')
#         self.get_logger().info('🌐 WebSocket server starting for HTML client support')

#     def gps_callback(self, msg):
#         if len(msg.data) >= 2:
#             self.current_lat = msg.data[0]
#             self.current_lon = msg.data[1]

#     def check_destinations_proximity(self):
#         """ตรวจสอบว่าถึงจุดหมายปลายทางจาก GPS หรือยัง"""
#         if self.current_lat is None or self.current_lon is None:
#             return
        
#         reach_radius = 5.0  # meters
#         dests = self.websocket_server.get_destinations()
        
#         if not dests:
#             return
        
#         for idx, dest in enumerate(dests):
#             dist = haversine(self.current_lat, self.current_lon, dest['lat'], dest['lon'])
#             if dist <= reach_radius:
#                 dest_name = dest.get('name', f"Destination {idx+1}")
#                 self.get_logger().info(f"✅ Arrived at GPS destination: {dest_name} ({dist:.2f}m)")
                
#                 # Send arrival notification via WebSocket
#                 asyncio.run_coroutine_threadsafe(
#                     self.notify_arrival(dest),
#                     asyncio.get_event_loop()
#                 )
                
#                 # Stop the vehicle
#                 self.send_stop_command()
                
#                 # Remove reached destination
#                 self.websocket_server.remove_reached(idx)
#                 break

#     async def notify_arrival(self, destination):
#         """แจ้งเตือนการถึงจุดหมายปลายทางผ่าน WebSocket"""
#         if self.websocket_server.active_websockets:
#             arrival_msg = {
#                 "type": "arrival_notification",
#                 "destination": destination,
#                 "message": f"Arrived at {destination.get('name', 'destination')}!"
#             }
            
#             websockets_to_remove = set()
#             for ws in list(self.websocket_server.active_websockets):
#                 try:
#                     await ws.send(json.dumps(arrival_msg))
#                 except Exception:
#                     websockets_to_remove.add(ws)
            
#             self.websocket_server.active_websockets -= websockets_to_remove

#     def xy_callback(self, msg):
#         if len(msg.data) >= 2:
#             self.current_position['x'] = msg.data[0]
#             self.current_position['y'] = msg.data[1]

#     def heading_callback(self, msg):
#         self.current_heading = msg.data

#     def navigation_control(self):
#         if not self.waypoint_manager.navigation_started:
#             if self.current_heading is not None:
#                 if not self.heading_forced_to_zero:
#                     self.bicycle_model.force_initial_heading(self.current_heading)
#                     self.heading_forced_to_zero = True
#                     self.get_logger().info(f'🔧 FORCING heading from {self.current_heading:.1f}° to 0.0°')
#                 if self.waypoint_manager.initialize_starting_point(self.current_position, self.current_heading):
#                     self.navigation_active = True
#                     progress = self.waypoint_manager.get_progress_info()
#                     self.get_logger().info(f'🚀 Navigation started! Target heading: 0.0°')
#                     self.get_logger().info(f'📍 Starting from waypoint {progress["starting"]} of {progress["total"]}')
#                     self.get_logger().info(f'🎯 Current target: waypoint {progress["current"]}')
#             return

#         current_waypoint = self.waypoint_manager.get_current_waypoint()
#         if current_waypoint is None:
#             self.get_logger().info('🏁 Navigation completed - reached final waypoint')
#             self.navigation_active = False
#             self.send_stop_command()
#             return

#         distance_to_waypoint = math.sqrt(
#             (self.current_position['x'] - current_waypoint['x'])**2 +
#             (self.current_position['y'] - current_waypoint['y'])**2
#         )
#         should_advance, advance_reason = self.waypoint_manager.should_advance_waypoint(
#             distance_to_waypoint, self.waypoint_reached_threshold
#         )

#         # == แก้ปัญหาหลุด waypoint == #
#         if advance_reason == "too_far_skip":
#             nearest_index = self.waypoint_manager.find_nearest_waypoint(self.current_position)
#             self.waypoint_manager.current_waypoint_index = nearest_index
#             self.get_logger().info(
#                 f"🔄 Robot location far from all waypoints, reset to nearest waypoint {nearest_index+1}")
#             return  # รอรอบถัดไป

#         if should_advance:
#             if self.waypoint_manager.advance_waypoint():
#                 progress = self.waypoint_manager.get_progress_info()
#                 segment_info = self.waypoint_manager.get_current_segment_info()
#                 reason_text = {
#                     "reached_threshold": "REACHED (within threshold)",
#                     "passed_waypoint": "PASSED (moving away)",
#                     "too_far_skip": "SKIPPED (too far)"
#                 }.get(advance_reason, advance_reason)
#                 self.get_logger().info(
#                     f'✅ {reason_text} waypoint {progress["current"]-1}! '
#                     f'Progress: {progress["from_start_count"]-1}/{progress["total"] - progress["starting"]} '
#                     f'({progress["progress_percent"]:.1f}% complete)'
#                 )
#                 self.get_logger().info(
#                     f'🎯 NEXT TARGET: waypoint {progress["current"]} '
#                     f'({segment_info["type"].upper()}: {segment_info["direction"]})'
#                 )
#             else:
#                 self.get_logger().info('🏁 Navigation completed - ALL WAYPOINTS REACHED!')
#                 self.navigation_active = False
#                 self.send_stop_command()
#                 return

#         current_time = self.get_clock().now().nanoseconds / 1e9
#         if current_time - self.last_progress_log_time > self.progress_log_interval:
#             self.log_current_progress(distance_to_waypoint)
#             self.last_progress_log_time = current_time

#         segment_info = self.waypoint_manager.get_current_segment_info()
#         segment_type = segment_info['type']
#         direction = segment_info['direction']
#         target_heading = segment_info.get('target_heading')
#         reverse = segment_info.get('reverse_heading', False)
#         if segment_type == 'straight' and target_heading is not None:
#             self.bicycle_model.set_target_heading(target_heading)
#         linear_velocity = self.fixed_speed
#         if segment_type == 'straight' and segment_info.get('radius') is not None:
#             radius = segment_info['radius']
#             steering_angle = self.curvature_controller.calculate_steering_angle_with_radius(
#                 radius, self.fixed_speed, self.wheelbase, segment_info
#             )
#             heading_error = 0.0
#         elif segment_type == 'straight':
#             if reverse:
#                 steering_angle, heading_error = self.calculate_straight_path_control_reverse(
#                     current_waypoint, self.bicycle_model.get_current_target_heading())
#             else:
#                 steering_angle, heading_error = self.calculate_straight_path_control(
#                     current_waypoint, self.bicycle_model.get_current_target_heading())
#         else:
#             steering_angle = self.calculate_curve_path_control(segment_info)
#             heading_error = 0.0

#         steering_angle = self.bicycle_model.limit_steering_angle(
#             steering_angle, self.max_steering_angle
#         )
#         angular_velocity = self.bicycle_model.calculate_angular_velocity(
#             linear_velocity, steering_angle
#         )
#         angular_velocity = max(-self.max_angular_velocity,
#                                min(self.max_angular_velocity, angular_velocity))

#         cmd_vel = Twist()
#         cmd_vel.linear.x = linear_velocity
#         cmd_vel.angular.z = angular_velocity
#         self.cmd_vel_publisher.publish(cmd_vel)
#         self.stats['navigation_commands'] += 1
#         self.publish_debug_info(segment_info, distance_to_waypoint, steering_angle,
#                                linear_velocity, angular_velocity, heading_error)
#         self.get_logger().info(
#             f"[DEBUG] Heading: {self.current_heading:.2f}°, Target: {target_heading if target_heading is not None else '-'}°, "
#             f"Error: {heading_error:.2f}°, Steering(rad): {steering_angle:.3f}, Angular vel: {angular_velocity:.3f}"
#         )

#     def log_current_progress(self, distance_to_waypoint):
#         progress = self.waypoint_manager.get_progress_info()
#         segment_info = self.waypoint_manager.get_current_segment_info()
#         distance_trend = ""
#         if self.waypoint_manager.previous_distance is not None:
#             if distance_to_waypoint > self.waypoint_manager.previous_distance:
#                 distance_trend = f"↗️ (+{self.waypoint_manager.distance_increasing_count})"
#             else:
#                 distance_trend = "↘️"
#         self.get_logger().info(
#             f'🚗 NAVIGATING: waypoint {progress["current"]}/{progress["total"]} '
#             f'| Progress from start: {progress["from_start_count"]}/{progress["total"] - progress["starting"]} '
#             f'| Distance: {distance_to_waypoint:.2f}m {distance_trend} '
#             f'| Type: {segment_info["type"].upper()} ({segment_info["direction"]}) '
#             f'| Heading: {self.current_heading:.1f}°'
#         )

#     def calculate_straight_path_control(self, target_waypoint, target_heading):
#         heading_correction_steering, heading_error = self.bicycle_model.calculate_heading_correction_steering(
#             self.current_heading, target_heading, 3.5
#         )
#         pure_pursuit_steering = self.pure_pursuit.calculate_steering_angle(
#             self.current_position, self.current_heading, target_waypoint, self.wheelbase
#         )
#         combined_steering = 0.95 * heading_correction_steering + 0.05 * pure_pursuit_steering
#         if abs(heading_correction_steering) > 0.01:
#             self.stats['heading_corrections'] += 1
#         return combined_steering, heading_error
#     def calculate_straight_path_control_reverse(self, target_waypoint, target_heading):
#         heading_correction_steering, heading_error = self.bicycle_model.calculate_heading_correction_steering(
#             target_heading,
#             self.current_heading,
#             3.5
#         )
#         pure_pursuit_steering = self.pure_pursuit.calculate_steering_angle(
#             self.current_position,
#             self.current_heading,
#             target_waypoint,
#             self.wheelbase
#         )
#         combined_steering = 0.95 * heading_correction_steering + 0.05 * pure_pursuit_steering
#         if abs(heading_correction_steering) > 0.01:
#             self.stats['heading_corrections'] += 1
#         return combined_steering, heading_error
#     def calculate_curve_path_control(self, segment_info):
#         curve_direction = segment_info['direction']
#         curve_radius = self.waypoint_manager.get_curve_radius(curve_direction)
#         steering_angle = self.curvature_controller.calculate_steering_angle_with_radius(
#             curve_radius,
#             self.fixed_speed,
#             self.wheelbase,
#             segment_info
#         )
#         return steering_angle

#     def publish_debug_info(self, segment_info, distance_to_waypoint, steering_angle,
#                           linear_velocity, angular_velocity, heading_error):
#         debug_msg = String()
#         progress = self.waypoint_manager.get_progress_info()
#         segment_type = segment_info['type']
#         direction = segment_info['direction']
#         target_heading = segment_info.get('target_heading')
#         turn_direction = segment_info.get('turn', 'left')
#         if segment_type == 'curve':
#             curve_radius = self.waypoint_manager.get_curve_radius(direction)
#             debug_msg.data = (
#                 f"🔄 CURVE ({direction}, R={curve_radius:.1f}m, Turn={turn_direction.upper()}) | "
#                 f"WP: {progress['current']}/{progress['total']} (Start:{progress['starting']}) | "
#                 f"Progress: {progress['from_start_count']} waypoints from start | "
#                 f"Dist: {distance_to_waypoint:.2f}m | "
#                 f"Steer: {math.degrees(steering_angle):.1f}° | "
#                 f"Speed: {linear_velocity:.2f}m/s | "
#                 f"Angular: {angular_velocity:.2f}rad/s | "
#                 f"Turn: {'LEFT(+)' if angular_velocity > 0 else 'RIGHT(-)' if angular_velocity < 0 else 'STRAIGHT'}"
#             )
#         else:
#             target_heading_str = f"{target_heading:.0f}" if target_heading is not None else "None"
#             current_heading_str = f"{self.current_heading:.1f}" if self.current_heading is not None else "None"
#             debug_msg.data = (
#                 f"➡️ STRAIGHT ({direction}, Target={target_heading_str}°) | "
#                 f"WP: {progress['current']}/{progress['total']} (Start:{progress['starting']}) | "
#                 f"Progress: {progress['from_start_count']} waypoints from start | "
#                 f"Dist: {distance_to_waypoint:.2f}m | "
#                 f"Heading: {current_heading_str}° (Error: {heading_error:.1f}°) | "
#                 f"Steer: {math.degrees(steering_angle):.1f}° | "
#                 f"Speed: {linear_velocity:.2f}m/s | "
#                 f"Angular: {angular_velocity:.2f}rad/s | "
#                 f"Turn: {'LEFT(+)' if angular_velocity > 0 else 'RIGHT(-)' if angular_velocity < 0 else 'STRAIGHT'}"
#             )
#         self.navigation_debug_publisher.publish(debug_msg)

#     def send_stop_command(self):
#         cmd_vel = Twist()
#         cmd_vel.linear.x = 0.0
#         cmd_vel.angular.z = 0.0
#         self.cmd_vel_publisher.publish(cmd_vel)
#         self.get_logger().info('🛑 Stop command sent')

#     def destroy_node(self):
#         self.get_logger().info("🔄 Shutting down Navigation Controller...")
#         if hasattr(self, 'navigation_active') and self.navigation_active:
#             self.send_stop_command()
#         self.print_final_statistics()
#         super().destroy_node()
#     def print_final_statistics(self):
#         try:
#             self.get_logger().info("📊 Final Navigation Statistics:")
#             self.get_logger().info(f"  🚗 Navigation commands: {self.stats['navigation_commands']}")
#             self.get_logger().info(f"  🔧 Heading corrections: {self.stats['heading_corrections']}")
#             if hasattr(self, 'waypoint_manager') and self.waypoint_manager:
#                 progress = self.waypoint_manager.get_progress_info()
#                 self.get_logger().info(f"  🎯 Final waypoint: {progress['current']}/{progress['total']} ({progress['progress_percent']:.1f}%)")
#                 self.get_logger().info(f"  🚀 Started from waypoint: {progress['starting']}")
#                 self.get_logger().info(f"  📈 Completed waypoints: {progress['from_start_count']}")
#         except Exception as e:
#             self.get_logger().error(f"❌ Error printing statistics: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     try:
#         node = NavigationController()
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         print('\n🛑 Navigation Controller stopped by user')
#     except Exception as e:
#         print(f'❌ Error: {e}')
#     finally:
#         if 'node' in locals():
#             node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()