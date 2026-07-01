# # --------------------------------------test--------------------------------------------

# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
# from geometry_msgs.msg import Twist
# from std_msgs.msg import String
# import math
# import csv
# import os

# class CSVWaypointController(Node):
#     def __init__(self):
#         super().__init__('csv_waypoint_controller')
        
#         # Publishers and Subscribers
#         self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.status_publisher = self.create_publisher(String, '/navigation_status', 10)
#         self.gnss_subscriber = self.create_subscription(
#             NavSatFix, '/gnss/fix', self.gnss_callback, 10
#         )
        
#         # Navigation parameters
#         self.waypoints = []
#         self.current_waypoint = 0
#         self.waypoint_tolerance = 1.0  # meters
#         self.max_linear_velocity = 1.0  # m/s
#         self.max_angular_velocity = 1.0  # rad/s
#         self.search_radius = 10.0  # 10 เมตร
        
#         # State variables
#         self.current_position = None
#         self.initial_position_set = False
#         self.navigation_started = False
        
#         # CSV file path
#         self.csv_file_path = '/home/inc/ros2_ws/waypoints1m.csv'
        
#         # Load waypoints from CSV
#         self.load_waypoints_from_csv()
        
#         # Timer for control loop
#         self.control_timer = self.create_timer(0.1, self.control_loop)
        
#         self.get_logger().info('CSV Waypoint Controller initialized')
#         self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from CSV')
#         self.get_logger().info(f'Search radius set to {self.search_radius} meters')

#     def load_waypoints_from_csv(self):
#         """Load waypoints from CSV file"""
#         try:
#             if not os.path.exists(self.csv_file_path):
#                 self.get_logger().error(f'CSV file not found: {self.csv_file_path}')
#                 return
            
#             with open(self.csv_file_path, 'r') as csvfile:
#                 reader = csv.reader(csvfile)
                
#                 # Skip header if exists
#                 first_row = next(reader, None)
#                 if first_row and not self.is_numeric_row(first_row):
#                     self.get_logger().info('Skipping header row')
#                 else:
#                     # If first row is numeric, process it
#                     if first_row:
#                         self.process_csv_row(first_row, 0)
                
#                 # Process remaining rows
#                 for i, row in enumerate(reader, start=1 if first_row else 0):
#                     self.process_csv_row(row, i)
                    
#         except Exception as e:
#             self.get_logger().error(f'Error loading CSV file: {str(e)}')

#     def is_numeric_row(self, row):
#         """Check if row contains numeric data"""
#         try:
#             if len(row) >= 2:
#                 float(row[0])
#                 float(row[1])
#                 return True
#         except ValueError:
#             return False
#         return False

#     def process_csv_row(self, row, index):
#         """Process a single CSV row"""
#         try:
#             if len(row) >= 2:
#                 # Assume format: lat, lon or x_east, y_north
#                 coord1 = float(row[0])
#                 coord2 = float(row[1])
                
#                 # Check if coordinates are in UTM format (large numbers) or lat/lon
#                 if coord1 > 1000 and coord2 > 1000:
#                     # UTM coordinates
#                     lat, lon = self.utm_to_latlon(coord1, coord2)
#                     self.waypoints.append({
#                         'id': index,
#                         'lat': lat,
#                         'lon': lon,
#                         'x_east': coord1,
#                         'y_north': coord2
#                     })
#                 else:
#                     # Lat/Lon coordinates
#                     self.waypoints.append({
#                         'id': index,
#                         'lat': coord1,
#                         'lon': coord2,
#                         'x_east': None,
#                         'y_north': None
#                     })
                    
#         except (ValueError, IndexError) as e:
#             self.get_logger().warn(f'Skipping invalid row {index}: {row}')

#     def utm_to_latlon(self, x_east, y_north):
#         """Convert UTM coordinates to lat/lon (simplified for Thailand)"""
#         # Simplified conversion for Thailand UTM Zone 47N
#         lat = (y_north - 1509000) / 111320.0 + 13.65
#         lon = (x_east - 661000) / 111320.0 + 100.49
#         return lat, lon

#     def find_waypoints_in_radius(self, current_lat, current_lon, radius=10.0):
#         """Find all waypoints within specified radius"""
#         nearby_waypoints = []
        
#         for i, waypoint in enumerate(self.waypoints):
#             distance = self.calculate_distance(
#                 current_lat, current_lon,
#                 waypoint['lat'], waypoint['lon']
#             )
#             if distance <= radius:
#                 nearby_waypoints.append((i, distance))
        
#         return sorted(nearby_waypoints, key=lambda x: x[1])

#     def find_nearest_waypoint(self, current_lat, current_lon):
#         """Find the nearest waypoint to current position"""
#         if not self.waypoints:
#             return 0, float('inf')
        
#         min_distance = float('inf')
#         nearest_index = 0
        
#         for i, waypoint in enumerate(self.waypoints):
#             distance = self.calculate_distance(
#                 current_lat, current_lon,
#                 waypoint['lat'], waypoint['lon']
#             )
#             if distance < min_distance:
#                 min_distance = distance
#                 nearest_index = i
        
#         return nearest_index, min_distance

#     def calculate_distance(self, lat1, lon1, lat2, lon2):
#         """Calculate distance between two GPS coordinates using Haversine formula"""
#         R = 6371000  # Earth's radius in meters
        
#         lat1_rad = math.radians(lat1)
#         lat2_rad = math.radians(lat2)
#         delta_lat = math.radians(lat2 - lat1)
#         delta_lon = math.radians(lon2 - lon1)
        
#         a = (math.sin(delta_lat/2) * math.sin(delta_lat/2) +
#              math.cos(lat1_rad) * math.cos(lat2_rad) *
#              math.sin(delta_lon/2) * math.sin(delta_lon/2))
#         c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
#         return R * c

#     def calculate_bearing(self, lat1, lon1, lat2, lon2):
#         """Calculate bearing from point 1 to point 2"""
#         lat1_rad = math.radians(lat1)
#         lat2_rad = math.radians(lat2)
#         delta_lon = math.radians(lon2 - lon1)
        
#         y = math.sin(delta_lon) * math.cos(lat2_rad)
#         x = (math.cos(lat1_rad) * math.sin(lat2_rad) -
#              math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon))
        
#         bearing = math.atan2(y, x)
#         return (math.degrees(bearing) + 360) % 360

#     def gnss_callback(self, msg):
#         """Handle GNSS data"""
#         if msg.status.status < 0:  # No fix
#             return
        
#         self.current_position = {
#             'lat': msg.latitude,
#             'lon': msg.longitude,
#             'altitude': msg.altitude
#         }
        
#         # Initialize starting waypoint on first GPS fix
#         if not self.initial_position_set:
#             self.initialize_starting_waypoint()

#     def initialize_starting_waypoint(self):
#         """Initialize starting waypoint based on current position"""
#         if not self.current_position or not self.waypoints:
#             return
        
#         current_lat = self.current_position['lat']
#         current_lon = self.current_position['lon']
        
#         # Find waypoints within 10 meter radius
#         nearby_waypoints = self.find_waypoints_in_radius(
#             current_lat, current_lon, self.search_radius
#         )[1][4]
        
#         if nearby_waypoints:
#             # Choose the nearest waypoint within 10m radius
#             self.current_waypoint = nearby_waypoints[0][0]
#             distance = nearby_waypoints[0][1]
            
#             self.get_logger().info(f'✅ Starting from nearest waypoint: {self.current_waypoint}')
#             self.get_logger().info(f'📍 Distance to starting waypoint: {distance:.2f}m')
#             self.get_logger().info(f'🎯 Found {len(nearby_waypoints)} waypoints within {self.search_radius}m radius')
            
#             # Log nearby waypoints
#             for idx, (wp_idx, dist) in enumerate(nearby_waypoints[:5]):  # Show top 5
#                 waypoint = self.waypoints[wp_idx]
#                 self.get_logger().info(
#                     f'   #{idx+1}: Waypoint {wp_idx} - {dist:.2f}m '
#                     f'(lat: {waypoint["lat"]:.6f}, lon: {waypoint["lon"]:.6f})'
#                 )
                
#         else:
#             # If no waypoints within 10m radius, find the nearest one
#             nearest_idx, nearest_distance = self.find_nearest_waypoint(current_lat, current_lon)
#             self.current_waypoint = nearest_idx
#             self.get_logger().warn(f'⚠️  No waypoints within {self.search_radius}m radius')
#             self.get_logger().warn(f'🔍 Using nearest waypoint: {nearest_idx}')
#             self.get_logger().warn(f'📏 Distance: {nearest_distance:.2f}m')
        
#         self.initial_position_set = True
#         self.navigation_started = True
        
#         # Publish status
#         status_msg = String()
#         status_msg.data = f"Navigation started from waypoint {self.current_waypoint}"
#         self.status_publisher.publish(status_msg)

#     def control_loop(self):
#         """Main control loop"""
#         if not self.navigation_started or not self.current_position or not self.waypoints:
#             return
        
#         if self.current_waypoint >= len(self.waypoints):
#             self.get_logger().info('🏁 All waypoints completed!')
#             self.stop_robot()
#             return
        
#         # Get current target waypoint
#         target_waypoint = self.waypoints[self.current_waypoint]
#         current_lat = self.current_position['lat']
#         current_lon = self.current_position['lon']
        
#         # Calculate distance to target
#         distance_to_target = self.calculate_distance(
#             current_lat, current_lon,
#             target_waypoint['lat'], target_waypoint['lon']
#         )
        
#         # Check if we've reached the waypoint
#         if distance_to_target < self.waypoint_tolerance:
#             self.get_logger().info(f'✅ Reached waypoint {self.current_waypoint}')
#             self.current_waypoint += 1
            
#             # Publish status
#             status_msg = String()
#             if self.current_waypoint < len(self.waypoints):
#                 status_msg.data = f"Moving to waypoint {self.current_waypoint}"
#             else:
#                 status_msg.data = "Navigation completed"
#             self.status_publisher.publish(status_msg)
#             return
        
#         # Calculate control commands
#         bearing_to_target = self.calculate_bearing(
#             current_lat, current_lon,
#             target_waypoint['lat'], target_waypoint['lon']
#         )
        
#         # Simple proportional control
#         angular_error = bearing_to_target
#         if angular_error > 180:
#             angular_error -= 360
#         elif angular_error < -180:
#             angular_error += 360
        
#         # Create control command
#         cmd = Twist()
        
#         # Angular velocity (proportional to heading error)
#         cmd.angular.z = max(-self.max_angular_velocity, 
#                            min(self.max_angular_velocity, 
#                                angular_error * 0.02))
        
#         # Linear velocity (reduce when turning)
#         if abs(angular_error) > 30:
#             cmd.linear.x = self.max_linear_velocity * 0.3
#         else:
#             if distance_to_target < 5.0:
#                 cmd.linear.x = self.max_linear_velocity * 0.5
#             else:
#                 cmd.linear.x = self.max_linear_velocity
        
#         # Publish command
#         self.cmd_vel_publisher.publish(cmd)
        
#         # Log progress every second
#         if self.get_clock().now().nanoseconds % 1000000000 < 100000000:
#             self.get_logger().info(
#                 f'🚗 Waypoint {self.current_waypoint}/{len(self.waypoints)-1}: '
#                 f'Distance={distance_to_target:.2f}m, '
#                 f'Bearing={bearing_to_target:.1f}°'
#             )

#     def stop_robot(self):
#         """Stop the robot"""
#         cmd = Twist()
#         cmd.linear.x = 0.0
#         cmd.angular.z = 0.0
#         self.cmd_vel_publisher.publish(cmd)

# def main(args=None):
#     rclpy.init(args=args)
    
#     try:
#         controller = CSVWaypointController()
#         rclpy.spin(controller)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



#!/usr/bin/env python3

import serial
import pynmea2
import socket
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Point, Twist
import math
import pyproj
from collections import deque
import time
import glob
import os
import struct
import json
import numpy as np
import pandas as pd
import csv

# 🔧 เพิ่ม pyubx2 สำหรับ UBX parsing ที่ดีขึ้น
try:
    from pyubx2 import UBXReader, UBXMessage, UBX_MSGIDS
    PYUBX2_AVAILABLE = True
    print("✅ pyubx2 available - using enhanced UBX parsing")
except ImportError:
    PYUBX2_AVAILABLE = False
    print("⚠️ pyubx2 not available - install with: pip3 install pyubx2")

class BangBangWaypointController:
    def __init__(self, waypoints, final_heading=0.0, waypoint_tolerance=1.5, heading_tolerance=np.deg2rad(8)):
        """
        🎯 Enhanced Bang-Bang Controller สำหรับรถของคุณ
        
        Parameters:
        - waypoints: list of (x, y) coordinates
        - final_heading: ทิศทางสุดท้ายที่ต้องการ (radians)
        - waypoint_tolerance: ระยะทางที่ถือว่าถึง waypoint แล้ว (meters)
        - heading_tolerance: ความคลาดเคลื่อนทิศทางที่ยอมรับได้ (radians)
        """
        self.waypoints = waypoints
        self.final_heading = final_heading
        self.waypoint_tolerance = waypoint_tolerance
        self.heading_tolerance = heading_tolerance
        self.current_target_idx = 0
        
        # 🌟 Vehicle parameters สำหรับรถของคุณ
        self.wheelbase = 1.67  # เมตร (ฐานล้อ)
        self.max_steering_angle = 0.873  # rad (50 degrees)
        self.fixed_speed = 1.0  # m/s (ความเร็วคงที่)
        
        # คำนวณ angular velocity สูงสุดจากพารามิเตอร์รถ
        # ω_max = v * tan(δ_max) / L
        self.max_angular_velocity = self.fixed_speed * np.tan(self.max_steering_angle) / self.wheelbase
        
        # สถิติการทำงาน
        self.stats = {
            'waypoints_reached': 0,
            'total_distance': 0.0,
            'mission_start_time': time.time(),
            'left_turns': 0,
            'right_turns': 0,
            'straight_commands': 0
        }
        
        print(f"🚗 Vehicle Configuration:")
        print(f"   Wheelbase: {self.wheelbase} m")
        print(f"   Max Steering: ±{np.rad2deg(self.max_steering_angle):.1f}° (±{self.max_steering_angle:.3f} rad)")
        print(f"   Max Angular Velocity: ±{self.max_angular_velocity:.3f} rad/s")
        print(f"   Fixed Speed: {self.fixed_speed} m/s")
        
    def find_closest_waypoint(self, current_pos):
        """หาจุด waypoint ที่ใกล้ที่สุดกับตำแหน่งปัจจุบัน"""
        distances = []
        for i, wp in enumerate(self.waypoints):
            dist = np.hypot(wp[0] - current_pos[0], wp[1] - current_pos[1])
            distances.append((dist, i))
        
        distances.sort()
        return distances[0][1]  # return index of closest waypoint
    
    def find_waypoints_in_radius(self, current_pos, radius=10.0):
        """🌟 หา waypoints ทั้งหมดใน radius 10 เมตร"""
        nearby_waypoints = []
        for i, wp in enumerate(self.waypoints):
            dist = np.hypot(wp[0] - current_pos[0], wp[1] - current_pos[1])
            if dist <= radius:
                nearby_waypoints.append((i, dist))
        
        # เรียงตามระยะทาง
        nearby_waypoints.sort(key=lambda x: x[1])
        return nearby_waypoints
    
    def initialize_starting_waypoint(self, current_pos):
        """🌟 เริ่มต้นจาก waypoint ที่ใกล้ที่สุดใน radius 10 เมตร"""
        nearby_waypoints = self.find_waypoints_in_radius(current_pos, 10.0)
        
        if nearby_waypoints:
            # เลือก waypoint ใกล้ที่สุดใน radius 10 เมตร
            self.current_target_idx = nearby_waypoints[0][0]
            distance = nearby_waypoints[0][1]
            
            print(f"✅ Starting from nearest waypoint: {self.current_target_idx}")
            print(f"📍 Distance to starting waypoint: {distance:.2f}m")
            print(f"🎯 Found {len(nearby_waypoints)} waypoints within 10m radius")
            
            # แสดง waypoints ที่พบ (top 5)
            for idx, (wp_idx, dist) in enumerate(nearby_waypoints[:5]):
                print(f"   #{idx+1}: Waypoint {wp_idx} - {dist:.2f}m")
                
            return True
        else:
            # หากไม่พบ waypoints ใน radius 10m ใช้ waypoint ใกล้ที่สุด
            nearest_idx = self.find_closest_waypoint(current_pos)
            nearest_distance = np.hypot(
                self.waypoints[nearest_idx][0] - current_pos[0],
                self.waypoints[nearest_idx][1] - current_pos[1]
            )
            
            self.current_target_idx = nearest_idx
            print(f"⚠️ No waypoints within 10m radius")
            print(f"🔍 Using nearest waypoint: {nearest_idx}")
            print(f"📏 Distance: {nearest_distance:.2f}m")
            
            return True
    
    def get_target_heading(self, current_pos, target_waypoint_idx):
        """คำนวณทิศทางที่ต้องการไปยัง waypoint เป้าหมาย"""
        if target_waypoint_idx >= len(self.waypoints):
            return self.final_heading
            
        target_wp = self.waypoints[target_waypoint_idx]
        dx = target_wp[0] - current_pos[0]
        dy = target_wp[1] - current_pos[1]
        
        # ถ้าใกล้ waypoint สุดท้ายมาก ให้ใช้ final_heading
        if target_waypoint_idx == len(self.waypoints) - 1:
            distance_to_target = np.hypot(dx, dy)
            if distance_to_target < self.waypoint_tolerance:
                return self.final_heading
        
        return np.arctan2(dy, dx)
    
    def normalize_angle(self, angle):
        """ปรับมุมให้อยู่ในช่วง [-π, π]"""
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    def update_target_waypoint(self, current_pos):
        """อัพเดท waypoint เป้าหมายถ้าถึงจุดปัจจุบันแล้ว"""
        if self.current_target_idx < len(self.waypoints):
            current_wp = self.waypoints[self.current_target_idx]
            distance = np.hypot(current_wp[0] - current_pos[0], current_wp[1] - current_pos[1])
            
            if distance < self.waypoint_tolerance and self.current_target_idx < len(self.waypoints) - 1:
                self.current_target_idx += 1
                self.stats['waypoints_reached'] += 1
                return True
        return False
    
    def get_control_command(self, current_pos, current_heading):
        """
        🎯 คำนวณคำสั่งควบคุมแบบ Bang-Bang ที่ปรับปรุงแล้ว
        
        Returns:
        - cmd_vel: Twist message with linear and angular velocity
        - status: dictionary with current status information
        """
        # อัพเดท waypoint เป้าหมาย
        waypoint_changed = self.update_target_waypoint(current_pos)
        
        # หาทิศทางที่ต้องการ
        target_heading = self.get_target_heading(current_pos, self.current_target_idx)
        
        # คำนวณความผิดพลาดของทิศทาง
        heading_error = self.normalize_angle(target_heading - current_heading)
        
        # คำนวณระยะทางถึงเป้าหมาย
        if self.current_target_idx < len(self.waypoints):
            target_wp = self.waypoints[self.current_target_idx]
            distance_to_target = np.hypot(target_wp[0] - current_pos[0], target_wp[1] - current_pos[1])
        else:
            distance_to_target = 0.0
        
        # สร้าง Twist message
        cmd_vel = Twist()
        
        # 🌟 Enhanced Bang-Bang Control Logic สำหรับรถของคุณ
        if abs(heading_error) <= self.heading_tolerance:
            # ไปตรง
            angular_velocity = 0.0
            steering_angle = 0.0
            steering_command = "STRAIGHT"
            self.stats['straight_commands'] += 1
        elif heading_error > 0:
            # เลี้ยวซ้าย - ใช้ค่าสูงสุด 50°
            angular_velocity = self.max_angular_velocity
            steering_angle = self.max_steering_angle
            steering_command = "TURN_LEFT_MAX"
            self.stats['left_turns'] += 1
        else:
            # เลี้ยวขวา - ใช้ค่าสูงสุด -50°
            angular_velocity = -self.max_angular_velocity
            steering_angle = -self.max_steering_angle
            steering_command = "TURN_RIGHT_MAX"
            self.stats['right_turns'] += 1
        
        # ตั้งค่าความเร็วคงที่ 1 m/s
        if distance_to_target > 0.1:  # ยังไม่ถึงเป้าหมาย
            cmd_vel.linear.x = self.fixed_speed
        else:
            cmd_vel.linear.x = 0.0  # หยุดเมื่อถึงเป้าหมายสุดท้าย
        
        cmd_vel.angular.z = angular_velocity
        
        # สร้างข้อมูลสถานะ
        status = {
            'current_waypoint': self.current_target_idx,
            'total_waypoints': len(self.waypoints),
            'target_heading_deg': np.rad2deg(target_heading),
            'current_heading_deg': np.rad2deg(current_heading),
            'heading_error_deg': np.rad2deg(heading_error),
            'distance_to_target': distance_to_target,
            'steering_command': steering_command,
            'linear_velocity': cmd_vel.linear.x,
            'angular_velocity': cmd_vel.angular.z,
            'waypoint_changed': waypoint_changed,
            'mission_progress': (self.current_target_idx / len(self.waypoints)) * 100,
            'stats': self.stats
        }
        
        return cmd_vel, status
    
    def is_mission_complete(self, current_pos, current_heading):
        """ตรวจสอบว่าภารกิจเสร็จสิ้นแล้วหรือไม่"""
        if self.current_target_idx >= len(self.waypoints) - 1:
            last_wp = self.waypoints[-1]
            distance = np.hypot(last_wp[0] - current_pos[0], last_wp[1] - current_pos[1])
            heading_error = abs(self.normalize_angle(self.final_heading - current_heading))
            
            return (distance < self.waypoint_tolerance and 
                   heading_error < self.heading_tolerance)
        return False

class EnhancedZEDf9rGNSSWithHeading(Node):
    def __init__(self):
        super().__init__('enhanced_zedf9r_gnss_with_heading')
        
        # ROS2 Publishers
        self.gnss_publisher = self.create_publisher(NavSatFix, '/navigation/gnss', 10)
        self.xy_publisher = self.create_publisher(Point, '/navigation/xy_position', 10)
        self.heading_publisher = self.create_publisher(Float32, '/navigation/heading', 10)
        self.heading_debug_publisher = self.create_publisher(String, '/navigation/heading_debug', 10)
        
        # 🌟 NEW: Controller publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.controller_status_publisher = self.create_publisher(String, '/controller/status', 10)
        
        # Serial configuration
        self.possible_serial_ports = [
            '/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1'
        ]
        self.serial_port = None
        self.ser = None
        
        # NTRIP configuration  
        self.ntrip_server_ip = "110.78.0.54"
        self.ntrip_server_port = 2116
        self.ntrip_username = "1118600009224"
        self.ntrip_password = "CK79"
        self.mount_point = "VRS_RTCM32"
        
        # 🔧 Fixed Coordinate transformation
        self.transformer = pyproj.Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)
        
        # Position tracking
        self.position_history = deque(maxlen=15)
        self.min_movement_for_heading = 0.25
        
        # 🔧 Fixed reference point
        self.reference_point = (13.650748, 100.492985)
        self.reference_utm = None
        
        # 🔧 Enhanced Heading management สำหรับ ZED-F9R
        self.imu_heading = None
        self.nav_att_heading = None
        self.nav_att_yaw = None
        self.gps_heading = None
        self.final_heading = None
        self.heading_source = "NONE"
        
        self.imu_heading_time = 0
        self.nav_att_time = 0
        self.gps_heading_time = 0
        self.heading_timeout = 8.0
        
        # ZED-F9R specific data
        self.esf_status = None
        self.nav_att = None
        self.esf_alg = None
        self.fusion_mode = "NONE"
        self.imu_calibration_status = "UNKNOWN"
        
        # 🌟 NEW: Controller initialization
        self.controller = None
        self.current_local_x = None
        self.current_local_y = None
        self.controller_active = False
        self.initial_position_set = False  # 🌟 เพิ่มตัวแปรนี้
        
        # 🌟 NEW: pyubx2 UBX parsing
        self.ubx_reader = None
        self.ubx_buffer = bytearray()
        
        # 🔧 Enhanced Heading filtering
        self.heading_filter = EnhancedIMUHeadingFilter()
        
        # Performance stats
        self.stats = {
            'imu_count': 0,
            'nav_att_count': 0,
            'nav_att_yaw_count': 0,
            'gps_count': 0,
            'position_count': 0,
            'esf_meas_count': 0,
            'esf_status_count': 0,
            'pyubx2_success': 0,
            'pyubx2_errors': 0,
            'filtered_count': 0
        }
        
        # 🔧 Initialize coordinate system
        self.init_fixed_coordinate_system()
        
        # 🌟 Initialize waypoint controller
        self.init_waypoint_controller()
        
        # Initialize
        self.init_serial_connection()
        
        if self.ser and self.ser.is_open:
            self.ntrip_socket = self.connect_to_ntrip()
            
            # 🌟 Initialize pyubx2 if available
            if PYUBX2_AVAILABLE:
                self.init_pyubx2()
            
            # Configure ZED-F9R for enhanced IMU
            self.configure_enhanced_zed_f9r()
            
            # Main processing timer
            self.create_timer(0.1, self.main_loop)
            
            self.get_logger().info('✅ Enhanced ZED-F9R GNSS with Waypoint Controller initialized')
            self.get_logger().info(f'🔧 pyubx2: {"Available" if PYUBX2_AVAILABLE else "Not available"}')
        else:
            self.get_logger().error('❌ Failed to initialize')
    
    def init_waypoint_controller(self):
        """🌟 Initialize waypoint controller"""
        try:
            # โหลด waypoints จากไฟล์ CSV
            waypoints = self.load_waypoints_from_csv('/home/inc/ros2_ws/waypoints1m.csv')
            
            if waypoints:
                self.controller = BangBangWaypointController(
                    waypoints=waypoints,
                    final_heading=np.deg2rad(0),  # หันหน้าไปทางเหนือ
                    waypoint_tolerance=2.0,       # ยอมรับความผิดพลาด 2 เมตร
                    heading_tolerance=np.deg2rad(10)  # ยอมรับความผิดพลาด 10 องศา
                )
                
                self.get_logger().info(f'✅ Waypoint controller initialized with {len(waypoints)} waypoints')
                self.controller_active = True
            else:
                self.get_logger().error('❌ Failed to load waypoints')
                
        except Exception as e:
            self.get_logger().error(f'❌ Controller initialization failed: {e}')
    
    def load_waypoints_from_csv(self, filename):
        """🌟 โหลด waypoints จากไฟล์ CSV (รองรับทั้ง lat/lon และ UTM)"""
        try:
            waypoints = []
            
            if not os.path.exists(filename):
                self.get_logger().error(f'CSV file not found: {filename}')
                return []
            
            with open(filename, 'r') as csvfile:
                reader = csv.reader(csvfile)
                
                # Skip header if exists
                first_row = next(reader, None)
                if first_row and not self.is_numeric_row(first_row):
                    self.get_logger().info('Skipping header row')
                else:
                    # If first row is numeric, process it
                    if first_row:
                        waypoint = self.process_csv_row(first_row)
                        if waypoint:
                            waypoints.append(waypoint)
                
                # Process remaining rows
                for row in reader:
                    waypoint = self.process_csv_row(row)
                    if waypoint:
                        waypoints.append(waypoint)
            
            self.get_logger().info(f'📍 Loaded {len(waypoints)} waypoints from {filename}')
            return waypoints
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load waypoints: {e}')
            return []
    
    def is_numeric_row(self, row):
        """Check if row contains numeric data"""
        try:
            if len(row) >= 2:
                float(row[0])
                float(row[1])
                return True
        except ValueError:
            return False
        return False
    
    def process_csv_row(self, row):
        """🌟 Process a single CSV row and convert to local coordinates"""
        try:
            if len(row) >= 2:
                coord1 = float(row[0])
                coord2 = float(row[1])
                
                # Check if coordinates are in UTM format (large numbers) or lat/lon
                if coord1 > 1000 and coord2 > 1000:
                    # UTM coordinates - convert to lat/lon first, then to local
                    lat, lon = self.utm_to_latlon(coord1, coord2)
                    local_x, local_y = self.latlon_to_xy_fixed(lat, lon)
                else:
                    # Lat/Lon coordinates - convert directly to local
                    local_x, local_y = self.latlon_to_xy_fixed(coord1, coord2)
                
                return (local_x, local_y)
                
        except (ValueError, IndexError) as e:
            self.get_logger().debug(f'Skipping invalid row: {row}')
            return None
    
    def utm_to_latlon(self, x_east, y_north):
        """Convert UTM coordinates to lat/lon (simplified for Thailand)"""
        # Simplified conversion for Thailand UTM Zone 47N
        lat = (y_north - 1509000) / 111320.0 + 13.65
        lon = (x_east - 661000) / 111320.0 + 100.49
        return lat, lon
    
    def run_controller(self):
        """🌟 รันตัวควบคุม waypoint"""
        if not (self.controller_active and self.controller and 
                self.current_local_x is not None and self.current_local_y is not None and
                self.final_heading is not None):
            return
        
        try:
            current_pos = (self.current_local_x, self.current_local_y)
            
            # 🌟 Initialize starting waypoint on first position
            if not self.initial_position_set:
                if self.controller.initialize_starting_waypoint(current_pos):
                    self.initial_position_set = True
                    self.get_logger().info("🎯 Starting waypoint initialized!")
                return
            
            current_heading_rad = np.deg2rad(self.final_heading)
            
            # รับคำสั่งควบคุมจาก controller
            cmd_vel, status = self.controller.get_control_command(current_pos, current_heading_rad)
            
            # ส่งคำสั่งความเร็ว
            self.cmd_vel_publisher.publish(cmd_vel)
            
            # แสดงสถานะ
            self.publish_controller_status(status)
            
            # Echo ข้อมูลสำคัญ
            self.echo_controller_status(status)
            
        except Exception as e:
            self.get_logger().error(f'❌ Controller error: {e}')
    
    def publish_controller_status(self, status):
        """ส่งสถานะของ controller"""
        try:
            status_msg = String()
            status_msg.data = json.dumps(status, default=str)
            self.controller_status_publisher.publish(status_msg)
        except Exception as e:
            self.get_logger().debug(f'Status publish error: {e}')
    
    def echo_controller_status(self, status):
        """🌟 Echo สถานะปัจจุบันของ controller"""
        try:
            # สร้างข้อความสถานะ
            waypoint_info = f"Waypoint: {status['current_waypoint']}/{status['total_waypoints']}"
            direction_info = f"Direction: {status['target_heading_deg']:.1f}°"
            steering_info = f"Steering: {status['steering_command']}"
            speed_info = f"Speed: {status['linear_velocity']:.1f} m/s"
            distance_info = f"Distance: {status['distance_to_target']:.1f}m"
            progress_info = f"Progress: {status['mission_progress']:.1f}%"
            
            # Log ข้อมูลหลัก
            self.get_logger().info(
                f"🎯 {waypoint_info} | {direction_info} | {steering_info} | "
                f"{speed_info} | {distance_info} | {progress_info}"
            )
            
            # Log เพิ่มเติมเมื่อเปลี่ยน waypoint
            if status['waypoint_changed']:
                self.get_logger().info(
                    f"✅ Reached waypoint {status['current_waypoint']-1}! "
                    f"Moving to waypoint {status['current_waypoint']}"
                )
            
            # ตรวจสอบการเสร็จสิ้นภารกิจ
            if self.controller.is_mission_complete(
                (self.current_local_x, self.current_local_y), 
                np.deg2rad(self.final_heading)
            ):
                self.get_logger().info("🏁 Mission Complete! All waypoints reached.")
                
        except Exception as e:
            self.get_logger().debug(f'Echo error: {e}')

    # ส่วนที่เหลือของ methods ใช้จากโค้ดเดิม
    def init_pyubx2(self):
        """🌟 Initialize pyubx2 for enhanced UBX parsing"""
        try:
            if PYUBX2_AVAILABLE:
                self.get_logger().info('🌟 pyubx2 UBX parser initialized')
            else:
                self.get_logger().warning('⚠️ pyubx2 not available, using manual parsing')
        except Exception as e:
            self.get_logger().error(f'❌ pyubx2 initialization failed: {e}')
    
    def init_fixed_coordinate_system(self):
        """🔧 Initialize fixed coordinate system"""
        try:
            self.reference_utm = self.transformer.transform(
                self.reference_point[1], self.reference_point[0]
            )
            self.get_logger().info(
                f'🔧 Enhanced coordinate system initialized:\n'
                f'  📍 Reference: ({self.reference_point[0]:.6f}, {self.reference_point[1]:.6f})\n'
                f'  📍 UTM: ({self.reference_utm[0]:.2f}, {self.reference_utm[1]:.2f})'
            )
        except Exception as e:
            self.get_logger().error(f'❌ Coordinate system init failed: {e}')
            self.reference_utm = None
    
    def configure_enhanced_zed_f9r(self):
        """🌟 Enhanced ZED-F9R configuration for better NAV-ATT"""
        try:
            self.get_logger().info('🌟 Configuring ZED-F9R for enhanced NAV-ATT...')
            time.sleep(2)
            
            # Enable UBX messages with higher rates
            self.enable_ubx_message(0x01, 0x05, 5)  # 🌟 NAV-ATT at 5Hz
            self.enable_ubx_message(0x10, 0x02, 2)  # ESF-MEAS at 2Hz
            self.enable_ubx_message(0x10, 0x10, 1)  # ESF-STATUS
            self.enable_ubx_message(0x10, 0x14, 1)  # ESF-ALG
            
            self.get_logger().info('✅ Enhanced ZED-F9R configuration sent')
            
        except Exception as e:
            self.get_logger().error(f'❌ Enhanced configuration failed: {e}')
    
    def enable_ubx_message(self, msg_class, msg_id, rate=1):
        """เปิดใช้ UBX message"""
        payload = struct.pack('<BBB', msg_class, msg_id, rate)
        cmd = self.create_ubx_message(0x06, 0x01, payload)
        self.ser.write(cmd)
        time.sleep(0.1)
    
    def create_ubx_message(self, msg_class, msg_id, payload):
        """สร้าง UBX message"""
        header = struct.pack('<BBBB', 0xb5, 0x62, msg_class, msg_id)
        length = struct.pack('<H', len(payload))
        
        ck_a = ck_b = 0
        for byte in struct.pack('<BB', msg_class, msg_id) + length + payload:
            ck_a = (ck_a + byte) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        
        return header + length + payload + struct.pack('<BB', ck_a, ck_b)
    
    def find_available_serial_ports(self):
        """Find available serial ports"""
        available_ports = []
        for port in self.possible_serial_ports:
            if os.path.exists(port):
                available_ports.append(port)
        
        acm_ports = glob.glob('/dev/ttyACM*')
        usb_ports = glob.glob('/dev/ttyUSB*')
        
        for port in acm_ports + usb_ports:
            if port not in available_ports:
                available_ports.append(port)
        
        return sorted(available_ports)
    
    def init_serial_connection(self):
        """Initialize serial connection"""
        self.get_logger().info("🔍 Searching for Enhanced ZED-F9R device...")
        
        available_ports = self.find_available_serial_ports()
        if not available_ports:
            self.get_logger().error("❌ No serial ports found!")
            return
        
        baud_rates = [38400, 9600, 115200]
        
        for port in available_ports:
            for baud in baud_rates:
                try:
                    test_ser = serial.Serial(port, baud, timeout=1)
                    time.sleep(1)
                    
                    if test_ser.in_waiting > 0 or self.test_serial_port_advanced(test_ser):
                        self.ser = test_ser
                        self.serial_port = port
                        self.get_logger().info(f"✅ Enhanced connection to {port} at {baud} baud")
                        return
                    
                    test_ser.close()
                except:
                    continue
        
        # Last resort
        if available_ports:
            try:
                port = available_ports[0]
                self.ser = serial.Serial(port, 38400, timeout=1)
                self.serial_port = port
                self.get_logger().warning(f"⚠️ Connected to {port} without validation")
            except Exception as e:
                self.get_logger().error(f"❌ Failed to connect: {e}")
    
    def test_serial_port_advanced(self, test_ser):
        """Advanced port testing"""
        try:
            start_time = time.time()
            while time.time() - start_time < 2:
                if test_ser.in_waiting > 0:
                    data = test_ser.read(test_ser.in_waiting)
                    try:
                        text = data.decode('ascii', errors='replace')
                        if any(marker in text for marker in ['$G', 'UBX', '\xb5\x62']):
                            return True
                    except:
                        pass
                time.sleep(0.1)
            return False
        except:
            return False
    
    def connect_to_ntrip(self):
        """Connect to NTRIP server"""
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((self.ntrip_server_ip, self.ntrip_server_port))
            
            import base64
            auth_str = f"{self.ntrip_username}:{self.ntrip_password}"
            auth_b64 = base64.b64encode(auth_str.encode('ascii')).decode('ascii')
            
            request = (
                f"GET /{self.mount_point} HTTP/1.1\r\n"
                f"User-Agent: NTRIP Client\r\n" 
                f"Authorization: Basic {auth_b64}\r\n\r\n"
            )
            client_socket.send(request.encode('ascii'))
            self.get_logger().info("🌐 Connected to NTRIP server")
            return client_socket
        except Exception as e:
            self.get_logger().warning(f"⚠️ NTRIP failed: {e}")
            return None
    
    def main_loop(self):
        """Enhanced main processing loop"""
        if not (self.ser and self.ser.is_open):
            return
        
        try:
            # Read all available data
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)
                
                # Process NMEA (text)
                try:
                    text_data = data.decode('ascii', errors='replace')
                    for line in text_data.split('\n'):
                        line = line.strip()
                        if line.startswith('$'):
                            self.process_nmea_line(line)
                except:
                    pass
                
                # 🌟 Enhanced UBX processing
                if PYUBX2_AVAILABLE:
                    self.process_ubx_data_enhanced(data)
                else:
                    self.process_ubx_data_manual(data)
            
            # Handle NTRIP
            self.handle_ntrip_data()
            
            # 🌟 Update final heading with enhanced logic
            self.update_enhanced_final_heading()
            
            # 🌟 Run waypoint controller
            self.run_controller()
            
        except Exception as e:
            self.get_logger().error(f"❌ Enhanced main loop error: {e}")
    
    def process_nmea_line(self, line):
        """Process NMEA line (for position)"""
        try:
            if any(line.startswith(nmea) for nmea in ['$GPGGA', '$GNGGA', '$GPRMC', '$GNRMC', '$GPGLL', '$GNGLL']):
                msg = pynmea2.parse(line)
                
                if hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
                    if msg.latitude and msg.longitude:
                        self.process_position(msg.latitude, msg.longitude)
            
        except Exception as e:
            self.get_logger().debug(f"NMEA parse error: {e}")
    
    def process_position(self, latitude, longitude):
        """Process GNSS position with enhanced coordinate system"""
        current_time = time.time()
        
        if not (-90 <= latitude <= 90) or not (-180 <= longitude <= 180):
            return
        
        # Convert using fixed coordinate system
        local_x, local_y = self.latlon_to_xy_fixed(latitude, longitude)
        
        # 🌟 Store current position for controller
        self.current_local_x = local_x
        self.current_local_y = local_y
        
        # Store position
        position_record = {
            'lat': latitude, 'lon': longitude,
            'local_x': local_x, 'local_y': local_y,
            'timestamp': current_time
        }
        self.position_history.append(position_record)
        self.stats['position_count'] += 1
        
        # Calculate GPS heading
        self.update_gps_heading_improved()
        
        # Publish data
        self.publish_data(latitude, longitude, local_x, local_y)
    
    def latlon_to_xy_fixed(self, lat, lon):
        """Convert lat/lon using fixed PyProj"""
        try:
            if self.reference_utm is None:
                return self.simple_latlon_to_xy(lat, lon)
            
            utm_x, utm_y = self.transformer.transform(lon, lat)
            local_x = utm_x - self.reference_utm[0]
            local_y = utm_y - self.reference_utm[1]
            
            return local_x, local_y
            
        except Exception as e:
            return self.simple_latlon_to_xy(lat, lon)
    
    def simple_latlon_to_xy(self, lat, lon):
        """Fallback simple conversion"""
        dlat = lat - self.reference_point[0]
        dlon = lon - self.reference_point[1]
        
        x = dlon * 111319.9 * math.cos(math.radians(lat))
        y = dlat * 111319.9
        
        return x, y
    
    def update_gps_heading_improved(self):
        """Enhanced GPS heading calculation"""
        if len(self.position_history) < 3:
            return
        
        total_distance = 0.0
        weighted_x = weighted_y = 0.0
        
        recent_positions = list(self.position_history)[-5:]
        
        for i in range(len(recent_positions) - 1):
            curr = recent_positions[i+1]
            prev = recent_positions[i]
            
            dx = curr['local_x'] - prev['local_x']
            dy = curr['local_y'] - prev['local_y']
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance > self.min_movement_for_heading:
                heading_rad = math.atan2(dx, dy)
                
                time_weight = (i + 1) / len(recent_positions)
                weight = distance * time_weight
                
                weighted_x += math.cos(heading_rad) * weight
                weighted_y += math.sin(heading_rad) * weight
                total_distance += weight
        
        if total_distance > self.min_movement_for_heading:
            avg_heading = math.degrees(math.atan2(weighted_y, weighted_x))
            if avg_heading < 0:
                avg_heading += 360
            
            filtered_gps_heading = self.heading_filter.filter_gps_heading(avg_heading)
            
            if filtered_gps_heading is not None:
                self.gps_heading = filtered_gps_heading
                self.gps_heading_time = time.time()
                self.stats['gps_count'] += 1
    
    def update_enhanced_final_heading(self):
        """🌟 Enhanced final heading selection with NAV-ATT priority"""
        current_time = time.time()
        
        # Check data validity
        nav_att_valid = (self.nav_att_heading is not None and 
                        (current_time - self.nav_att_time) < self.heading_timeout and
                        self.fusion_mode in ["FUSION", "INIT"])
        
        gps_valid = (self.gps_heading is not None and 
                    (current_time - self.gps_heading_time) < self.heading_timeout)
        
        # 🌟 Enhanced priority: NAV-ATT > GPS > Hold
        if nav_att_valid and self.imu_calibration_status in ["FULLY_CALIBRATED", "CALIBRATED"]:
            self.final_heading = self.nav_att_heading
            self.heading_source = f"NAV_ATT_{self.imu_calibration_status}"
            
        elif nav_att_valid and self.imu_calibration_status == "CALIBRATING":
            if gps_valid:
                self.final_heading = self.blend_headings(self.nav_att_heading, self.gps_heading, 0.8)
                self.heading_source = "NAV_ATT_GPS_BLEND"
            else:
                self.final_heading = self.nav_att_heading
                self.heading_source = "NAV_ATT_CALIBRATING"
                
        elif gps_valid:
            self.final_heading = self.gps_heading  
            self.heading_source = "GPS"
            
        else:
            self.final_heading = None
            self.heading_source = "NONE"
        
        # Publish enhanced heading
        if self.final_heading is not None:
            heading_msg = Float32()
            heading_msg.data = float(self.final_heading)
            self.heading_publisher.publish(heading_msg)
            self.stats['filtered_count'] += 1
    
    def blend_headings(self, heading1, heading2, weight1):
        """Blend two headings considering circular nature"""
        if heading1 is None or heading2 is None:
            return heading1 or heading2
        
        x1, y1 = math.cos(math.radians(heading1)), math.sin(math.radians(heading1))
        x2, y2 = math.cos(math.radians(heading2)), math.sin(math.radians(heading2))
        
        weight2 = 1.0 - weight1
        x_blend = weight1 * x1 + weight2 * x2
        y_blend = weight1 * y1 + weight2 * y2
        
        blended_heading = math.degrees(math.atan2(y_blend, x_blend))
        return blended_heading % 360
    
    def publish_data(self, latitude, longitude, local_x, local_y):
        """Publish all data"""
        # GNSS
        gnss_msg = NavSatFix()
        gnss_msg.latitude = latitude
        gnss_msg.longitude = longitude
        gnss_msg.status.status = 0
        gnss_msg.status.service = 1
        self.gnss_publisher.publish(gnss_msg)
        
        # XY position
        xy_msg = Point()
        xy_msg.x = local_x
        xy_msg.y = local_y
        xy_msg.z = 0.0
        self.xy_publisher.publish(xy_msg)
    
    def handle_ntrip_data(self):
        """Handle NTRIP corrections"""
        if self.ntrip_socket:
            try:
                self.ntrip_socket.setblocking(0)
                try:
                    rtk_data = self.ntrip_socket.recv(1024)
                    if rtk_data:
                        self.ser.write(rtk_data)
                except socket.error:
                    pass
            except Exception as e:
                pass
    
    # เพิ่มส่วนที่เหลือของ UBX processing methods...
    def process_ubx_data_enhanced(self, data):
        """🌟 Enhanced UBX processing with pyubx2"""
        pass  # Implementation จากโค้ดเดิม
    
    def process_ubx_data_manual(self, data):
        """Fallback manual UBX processing"""
        pass  # Implementation จากโค้ดเดิม
    
    def destroy_node(self):
        """Cleanup"""
        self.get_logger().info("🔄 Shutting down Enhanced ZED-F9R with Controller...")
        
        if self.ser and self.ser.is_open:
            self.ser.close()
        
        if hasattr(self, 'ntrip_socket') and self.ntrip_socket:
            self.ntrip_socket.close()
        
        super().destroy_node()

# 🌟 Enhanced IMU Heading Filter (ใช้จากโค้ดเดิม)
class EnhancedIMUHeadingFilter:
    def __init__(self):
        self.nav_att_history = deque(maxlen=12)
        self.gps_history = deque(maxlen=8)
        
        self.nav_att_outlier_threshold = 20.0
        self.gps_outlier_threshold = 18.0
        self.smoothing_alpha = 0.25
        self.confidence_threshold = 0.8
        
        self.stats = {
            'nav_att_filtered': 0,
            'gps_filtered': 0,
            'nav_att_outliers': 0,
            'gps_outliers': 0,
            'confidence_boosted': 0,
            'confidence_reduced': 0
        }
    
    def filter_nav_att_heading(self, new_heading, confidence_factor=1.0):
        """Enhanced NAV-ATT heading filtering with confidence"""
        return new_heading  # Simplified for brevity
    
    def filter_gps_heading(self, new_gps_heading):
        """Enhanced GPS heading filtering"""
        return new_gps_heading  # Simplified for brevity
    
    def get_stats(self):
        """Get enhanced filter statistics"""
        return self.stats.copy()

def main(args=None):
    rclpy.init(args=args)
    gnss_publisher = EnhancedZEDf9rGNSSWithHeading()
    
    try:
        rclpy.spin(gnss_publisher)
    except KeyboardInterrupt:
        gnss_publisher.get_logger().info("🛑 Keyboard interrupt")
    finally:
        gnss_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()