#!/usr/bin/env python3
import asyncio
import websockets
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import threading
import signal
import sys
import socket
import math
import numpy as np
from geographiclib.geodesic import Geodesic
import time
from collections import deque

class LookaheadNavigationBridge(Node):
    def __init__(self):
        super().__init__('lookahead_navigation_bridge')
        
        # ROS2 subscribers and publishers
        self.gnss_sub = self.create_subscription(NavSatFix, '/navigation/gnss', self.gnss_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bearing_pub = self.create_publisher(Float32, '/navigation/bearing', 10)
        
        # WebSocket variables
        self.connections = set()
        self.latest_gnss = None
        self.websocket_loop = None
        self.server = None
        self.shutdown_requested = False
        
        # Navigation state
        self.current_position = None
        self.route_points = []
        self.current_waypoint_index = 0
        self.navigation_active = False
        self.destination_point = None
        self.lookahead_point = None
        
        # Enhanced Pure Pursuit parameters
        self.base_lookahead_distance = 10.0  # ตั้งค่าคงที่ 10 เมตร
        self.min_lookahead_distance = 10.0   # ตั้งค่าคงที่ 10 เมตร
        self.max_lookahead_distance = 10.0   # ตั้งค่าคงที่ 10 เมตร
        self.adaptive_lookahead = False      # ปิด adaptive ให้ใช้ 10 เมตรเสมอ
        
        # Vehicle parameters
        self.wheelbase = 2.2
        self.max_speed = 1.0  # ลดความเร็วให้เหมาะสม
        self.min_speed = 0.1
        self.arrival_threshold = 2.0
        self.waypoint_threshold = 2.0  # เพิ่มระยะ waypoint threshold ให้ลบเส้นได้เร็วขึ้น
        
        # Improved steering control
        self.max_steering_angle = math.radians(20)  # ลดมุมเลี้ยว
        self.steering_smoothing = 0.85  # เพิ่มการทำให้นุ่มนวล
        self.prev_steering = 0.0
        self.straight_line_threshold = math.radians(2)  # ลดเป็น 2 องศา สำหรับตรวจสอบทางตรง
        self.dead_zone_threshold = math.radians(0.5)  # dead zone 0.5 องศา ให้เป็น 0 เลย
        
        # Position tracking with improved heading calculation
        self.position_history = deque(maxlen=10)  # เพิ่มจำนวน history
        self.current_heading = 0.0
        self.heading_initialized = False
        self.min_movement_for_heading = 0.5  # ระยะขั้นต่ำสำหรับคำนวณ heading
        
        # Control timing
        self.last_control_time = time.time()
        self.control_frequency = 10.0  # Hz
        
        # Route processing improvements
        self.route_processed = False
        self.path_segments = []  # เก็บ segments ของเส้นทาง
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # Start WebSocket server
        self.server_thread = threading.Thread(target=self.start_websocket_server, daemon=True)
        self.server_thread.start()
        
        # Navigation timer at 10Hz
        self.create_timer(1.0/self.control_frequency, self.navigation_callback)
        
        self.get_logger().info('🎯 Enhanced Lookahead Point Navigation initialized')
    
    def signal_handler(self, signum, frame):
        self.get_logger().info(f'Received signal {signum}, shutting down...')
        self.shutdown_requested = True
        self.cleanup()
        sys.exit(0)
    
    def cleanup(self):
        self.shutdown_requested = True
        if self.websocket_loop and self.server:
            asyncio.run_coroutine_threadsafe(self.shutdown_server(), self.websocket_loop)
    
    async def shutdown_server(self):
        if self.server:
            self.get_logger().info('Shutting down WebSocket server...')
            self.server.close()
            await self.server.wait_closed()
    
    def find_available_port(self, start_port=5000, max_attempts=10):
        for port in range(start_port, start_port + max_attempts):
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                try:
                    s.bind(('0.0.0.0', port))
                    return port
                except OSError:
                    continue
        return None
    
    def start_websocket_server(self):
        self.websocket_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.websocket_loop)
        
        port = self.find_available_port(5000)
        if port is None:
            self.get_logger().error('No available ports found')
            return
        
        try:
            start_server = websockets.serve(self.websocket_handler, "0.0.0.0", port, ping_interval=30, ping_timeout=10)
            self.server = self.websocket_loop.run_until_complete(start_server)
            self.get_logger().info(f'🎯 WebSocket server running on port {port}')
            self.websocket_loop.run_forever()
        except Exception as e:
            self.get_logger().error(f'Failed to start WebSocket server: {e}')
    
    async def websocket_handler(self, websocket, path):
        self.connections.add(websocket)
        client_address = websocket.remote_address
        self.get_logger().info(f'New connection: {client_address}')
        
        try:
            async for message in websocket:
                if self.shutdown_requested:
                    break
                
                try:
                    data = json.loads(message)
                    
                    # รับเส้นทางจาก web
                    if 'path' in data and isinstance(data['path'], list):
                        self.process_route_from_web(data['path'])
                        
                        response = {
                            'route_received': True,
                            'waypoint_count': len(self.route_points),
                            'navigation_status': 'ready'
                        }
                        await websocket.send(json.dumps(response))
                    
                    # รับคำสั่งควบคุม
                    if 'navigation_command' in data:
                        cmd = data['navigation_command']
                        if cmd == 'start':
                            self.start_navigation()
                        elif cmd == 'stop':
                            self.stop_navigation()
                        elif cmd == 'pause':
                            self.pause_navigation()
                
                except json.JSONDecodeError:
                    self.get_logger().error(f'Invalid JSON: {message}')
                except Exception as e:
                    self.get_logger().error(f'Error processing message: {e}')
        
        except websockets.exceptions.ConnectionClosed:
            self.get_logger().info(f'Connection closed: {client_address}')
        finally:
            if websocket in self.connections:
                self.connections.remove(websocket)
    
    def process_route_from_web(self, path_data):
        """ประมวลผลเส้นทางที่ได้รับจาก web แบบปรับปรุง"""
        self.route_points = []
        self.path_segments = []
        
        for point in path_data:
            if 'lat' in point and 'lon' in point:
                self.route_points.append({
                    'lat': float(point['lat']),
                    'lon': float(point['lon'])
                })
        
        if len(self.route_points) > 0:
            self.destination_point = self.route_points[-1]
            self.create_path_segments()  # สร้าง segments
            self.get_logger().info(f'🎯 Route received: {len(self.route_points)} waypoints, {len(self.path_segments)} segments')
        
        self.current_waypoint_index = 0
        self.lookahead_point = None
        self.route_processed = True
    
    def create_path_segments(self):
        """สร้าง path segments เพื่อใช้ในการหา lookahead point"""
        self.path_segments = []
        
        for i in range(len(self.route_points) - 1):
            segment = {
                'start': self.route_points[i],
                'end': self.route_points[i + 1],
                'index': i
            }
            
            # คำนวณความยาวของ segment
            segment['length'] = self.calculate_distance(
                segment['start']['lat'], segment['start']['lon'],
                segment['end']['lat'], segment['end']['lon']
            )
            
            # คำนวณ bearing ของ segment
            segment['bearing'] = self.calculate_bearing(
                segment['start']['lat'], segment['start']['lon'],
                segment['end']['lat'], segment['end']['lon']
            )
            
            self.path_segments.append(segment)
    
    def start_navigation(self):
        if len(self.route_points) == 0:
            self.get_logger().warn('No route available')
            return
        
        self.navigation_active = True
        self.current_waypoint_index = 0
        self.route_processed = True
        self.get_logger().info('🚀 Navigation STARTED')
    
    def stop_navigation(self):
        self.navigation_active = False
        self.send_stop_command()
        self.get_logger().info('🛑 Navigation STOPPED')
    
    def pause_navigation(self):
        self.navigation_active = False
        self.send_stop_command()
        self.get_logger().info('⏸️ Navigation PAUSED')
    
    def send_stop_command(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
    
    def gnss_callback(self, msg):
        if self.shutdown_requested:
            return
        
        current_time = time.time()
        
        # ตรวจสอบความถูกต้องของข้อมูล GNSS
        if (msg.latitude == 0.0 and msg.longitude == 0.0) or \
           not (-90 <= msg.latitude <= 90) or \
           not (-180 <= msg.longitude <= 180):
            self.get_logger().debug(f'Invalid GNSS data: lat={msg.latitude}, lon={msg.longitude}')
            return
        
        # อัพเดทตำแหน่งปัจจุบัน
        new_position = {
            'lat': msg.latitude,
            'lon': msg.longitude,
            'timestamp': current_time,
            'status': msg.status.status,
            'service': msg.status.service
        }
        
        # Log การรับข้อมูล GNSS
        self.get_logger().info(
            f'🛰️ GNSS received: lat={msg.latitude:.7f}, lon={msg.longitude:.7f}, '
            f'status={msg.status.status}, service={msg.status.service}'
        )
        
        # ตรวจสอบความเปลี่ยนแปลงของตำแหน่ง
        if self.current_position:
            distance_moved = self.calculate_distance(
                self.current_position['lat'], self.current_position['lon'],
                new_position['lat'], new_position['lon']
            )
            
            self.get_logger().debug(f'Distance moved: {distance_moved:.3f}m')
            
            # อัพเดทเฉพาะเมื่อเคลื่อนที่จริง
            if distance_moved > 0.05:  # ลดเป็น 5 ซม. เพื่อการอัพเดทที่ดีขึ้น
                self.current_position = new_position
                self.position_history.append(new_position)
                self.update_heading_from_movement()
                self.get_logger().debug(f'Position updated: {distance_moved:.3f}m moved')
            else:
                # อัพเดท timestamp แม้ว่าตำแหน่งจะไม่เปลี่ยน
                self.current_position['timestamp'] = current_time
        else:
            self.current_position = new_position
            self.position_history.append(new_position)
            self.get_logger().info(f'🎯 Initial position set: lat={msg.latitude:.7f}, lon={msg.longitude:.7f}')
        
        # ส่งข้อมูลไปยัง web clients
        self.send_gnss_data_to_web()
    
    def update_heading_from_movement(self):
        """คำนวณทิศทางจากการเคลื่อนที่แบบปรับปรุง"""
        if len(self.position_history) < 3:
            return
        
        # ใช้หลายจุดเพื่อความแม่นยำ
        total_distance = 0.0
        weighted_bearings = []
        
        for i in range(len(self.position_history) - 1, max(0, len(self.position_history) - 5), -1):
            if i == 0:
                break
                
            current_pos = self.position_history[i]
            prev_pos = self.position_history[i-1]
            
            distance = self.calculate_distance(
                prev_pos['lat'], prev_pos['lon'],
                current_pos['lat'], current_pos['lon']
            )
            
            if distance > 0.2:  # พิจารณาเฉพาะการเคลื่อนที่ที่มีนัยสำคัญ
                bearing = self.calculate_bearing(
                    prev_pos['lat'], prev_pos['lon'],
                    current_pos['lat'], current_pos['lon']
                )
                
                weighted_bearings.append((bearing, distance))
                total_distance += distance
        
        if total_distance > self.min_movement_for_heading and weighted_bearings:
            # คำนวณ weighted average bearing
            total_x = 0.0
            total_y = 0.0
            
            for bearing, weight in weighted_bearings:
                total_x += math.cos(math.radians(bearing)) * weight
                total_y += math.sin(math.radians(bearing)) * weight
            
            if total_distance > 0:
                avg_bearing = math.degrees(math.atan2(total_y / total_distance, total_x / total_distance))
                if avg_bearing < 0:
                    avg_bearing += 360
                
                # Smooth heading transition
                if self.heading_initialized:
                    heading_diff = avg_bearing - self.current_heading
                    while heading_diff > 180:
                        heading_diff -= 360
                    while heading_diff < -180:
                        heading_diff += 360
                    
                    # อัพเดทเฉพาะเมื่อการเปลี่ยนแปลงสมเหตุสมผล
                    if abs(heading_diff) < 120:
                        self.current_heading = avg_bearing
                else:
                    self.current_heading = avg_bearing
                    self.heading_initialized = True
    
    def navigation_callback(self):
        """Main navigation control loop แบบปรับปรุง"""
        if not self.navigation_active or not self.current_position or not self.route_processed:
            return
        
        if len(self.route_points) == 0:
            return
        
        current_time = time.time()
        
        # ควบคุมความถี่
        if current_time - self.last_control_time < (1.0 / self.control_frequency):
            return
        
        self.last_control_time = current_time
        
        # ตรวจสอบการมาถึง
        if self.check_arrival():
            return
        
        # อัพเดท waypoint ปัจจุบัน
        self.update_current_waypoint()
        
        # หา lookahead point ใหม่
        self.lookahead_point = self.find_improved_lookahead_point()
        
        if self.lookahead_point is None:
            self.get_logger().warn('No lookahead point found, stopping navigation')
            self.stop_navigation()
            return
        
        # คำนวณและส่ง cmd_vel
        cmd_vel = self.calculate_improved_cmd_vel()
        if cmd_vel:
            self.cmd_vel_pub.publish(cmd_vel)
        
        # ส่ง bearing
        if self.lookahead_point:
            bearing = self.calculate_bearing(
                self.current_position['lat'], self.current_position['lon'],
                self.lookahead_point['lat'], self.lookahead_point['lon']
            )
            bearing_msg = Float32()
            bearing_msg.data = bearing
            self.bearing_pub.publish(bearing_msg)
    
    def find_improved_lookahead_point(self):
        """หา lookahead point แบบปรับปรุงใหม่"""
        if not self.current_position or len(self.route_points) == 0:
            return None
        
        # คำนวณ adaptive lookahead distance
        current_speed = self.get_current_speed()
        lookahead_distance = self.calculate_adaptive_lookahead_distance(current_speed)
        
        self.get_logger().debug(f'Lookahead distance: {lookahead_distance:.2f}m, Speed: {current_speed:.2f}m/s')
        
        # วิธีที่ 1: หาจุดบนเส้นทางที่อยู่ห่างจากตำแหน่งปัจจุบันตาม lookahead distance
        best_point = self.find_point_on_path(lookahead_distance)
        
        if best_point:
            return best_point
        
        # วิธีที่ 2: หา waypoint ที่เหมาะสมที่สุด
        return self.find_best_waypoint_lookahead(lookahead_distance)
    
    def calculate_adaptive_lookahead_distance(self, speed):
        """คำนวณ lookahead distance - ตั้งค่าคงที่ 10 เมตร"""
        return 10.0  # คงที่ 10 เมตรเสมอ
    
    def find_point_on_path(self, lookahead_distance):
        """หาจุดบนเส้นทางที่อยู่ห่างตาม lookahead distance"""
        if not self.path_segments:
            return None
        
        current_pos = self.current_position
        best_point = None
        min_distance_diff = float('inf')
        
        # ค้นหาใน segments ที่เหลือ
        for segment in self.path_segments[self.current_waypoint_index:]:
            # หาจุดที่ใกล้ที่สุดบน segment นี้
            closest_point = self.find_closest_point_on_segment(current_pos, segment)
            
            if closest_point:
                distance = self.calculate_distance(
                    current_pos['lat'], current_pos['lon'],
                    closest_point['lat'], closest_point['lon']
                )
                
                distance_diff = abs(distance - lookahead_distance)
                
                if distance_diff < min_distance_diff:
                    min_distance_diff = distance_diff
                    best_point = closest_point
                
                # หยุดหาถ้าผ่าน lookahead distance มากเกินไป
                if distance > lookahead_distance * 1.5:
                    break
        
        return best_point
    
    def find_closest_point_on_segment(self, current_pos, segment):
        """หาจุดที่ใกล้ที่สุดบน line segment"""
        start = segment['start']
        end = segment['end']
        
        # แปลงเป็น local coordinates
        start_x, start_y = self.lat_lon_to_meters(start['lat'], start['lon'], current_pos['lat'], current_pos['lon'])
        end_x, end_y = self.lat_lon_to_meters(end['lat'], end['lon'], current_pos['lat'], current_pos['lon'])
        current_x, current_y = 0, 0  # current position เป็น origin
        
        # คำนวณจุดที่ใกล้ที่สุดบน line segment
        A = end_x - start_x
        B = end_y - start_y
        C = current_x - start_x
        D = current_y - start_y
        
        dot = A * C + B * D
        len_sq = A * A + B * B
        
        if len_sq == 0:
            return start
        
        param = dot / len_sq
        
        if param < 0:
            closest_x, closest_y = start_x, start_y
        elif param > 1:
            closest_x, closest_y = end_x, end_y
        else:
            closest_x = start_x + param * A
            closest_y = start_y + param * B
        
        # แปลงกลับเป็น lat/lon
        closest_lat, closest_lon = self.meters_to_lat_lon(closest_x, closest_y, current_pos['lat'], current_pos['lon'])
        
        return {'lat': closest_lat, 'lon': closest_lon}
    
    def lat_lon_to_meters(self, lat, lon, ref_lat, ref_lon):
        """แปลง lat/lon เป็น local meters"""
        geod = Geodesic.WGS84
        
        # คำนวณ distance และ bearing
        g = geod.Inverse(ref_lat, ref_lon, lat, lon)
        distance = g['s12']
        bearing_rad = math.radians(g['azi1'])
        
        x = distance * math.sin(bearing_rad)
        y = distance * math.cos(bearing_rad)
        
        return x, y
    
    def meters_to_lat_lon(self, x, y, ref_lat, ref_lon):
        """แปลง local meters เป็น lat/lon"""
        geod = Geodesic.WGS84
        
        distance = math.sqrt(x*x + y*y)
        bearing = math.degrees(math.atan2(x, y))
        
        g = geod.Direct(ref_lat, ref_lon, bearing, distance)
        
        return g['lat2'], g['lon2']
    
    def find_best_waypoint_lookahead(self, lookahead_distance):
        """หา waypoint ที่เหมาะสมสำหรับ lookahead"""
        if self.current_waypoint_index >= len(self.route_points):
            return self.destination_point
        
        best_point = None
        min_distance_diff = float('inf')
        
        # ค้นหาใน waypoints ที่เหลือ
        for i in range(self.current_waypoint_index, len(self.route_points)):
            waypoint = self.route_points[i]
            distance = self.calculate_distance(
                self.current_position['lat'], self.current_position['lon'],
                waypoint['lat'], waypoint['lon']
            )
            
            distance_diff = abs(distance - lookahead_distance)
            
            if distance_diff < min_distance_diff:
                min_distance_diff = distance_diff
                best_point = waypoint
            
            # หยุดหาถ้าห่างเกินไป
            if distance > lookahead_distance * 1.8:
                break
        
        return best_point if best_point else self.route_points[self.current_waypoint_index]
    
    def calculate_improved_cmd_vel(self):
        """คำนวณ cmd_vel แบบปรับปรุง"""
        if not self.lookahead_point or not self.heading_initialized:
            # ส่งคำสั่งหยุดถ้าไม่มีข้อมูลเพียงพอ
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            return twist_msg
        
        # คำนวณ bearing ไปยัง lookahead point
        target_bearing = self.calculate_bearing(
            self.current_position['lat'], self.current_position['lon'],
            self.lookahead_point['lat'], self.lookahead_point['lon']
        )
        
        # คำนวณ heading error
        heading_error = target_bearing - self.current_heading
        while heading_error > 180:
            heading_error -= 360
        while heading_error < -180:
            heading_error += 360
        
        # คำนวณระยะทางไปยัง lookahead point
        distance_to_lookahead = self.calculate_distance(
            self.current_position['lat'], self.current_position['lon'],
            self.lookahead_point['lat'], self.lookahead_point['lon']
        )
        
        # Pure Pursuit algorithm
        if distance_to_lookahead > 0.1:
            alpha = math.radians(heading_error)
            steering_angle = math.atan2(2.0 * self.wheelbase * math.sin(alpha), distance_to_lookahead)
        else:
            steering_angle = 0.0
        
        # จำกัดมุมเลี้ยว
        steering_angle = max(min(steering_angle, self.max_steering_angle), -self.max_steering_angle)
        
        # Smooth steering transition
        steering_angle = self.steering_smoothing * self.prev_steering + (1.0 - self.steering_smoothing) * steering_angle
        self.prev_steering = steering_angle
        
        # คำนวณความเร็ว
        angular_velocity = steering_angle
        
        # ปรับความเร็วตามการเลี้ยว
        turn_severity = abs(angular_velocity) / self.max_steering_angle
        turn_factor = 1.0 - (turn_severity * 0.6)  # ลดความเร็วเมื่อเลี้ยวมาก
        
        # ปรับความเร็วตามระยะทางถึงจุดหมาย
        distance_to_destination = self.get_distance_to_destination()
        if distance_to_destination and distance_to_destination < 8.0:
            distance_factor = max(0.3, distance_to_destination / 8.0)
        else:
            distance_factor = 1.0
        
        # ปรับความเร็วตาม heading error
        heading_factor = max(0.5, 1.0 - abs(heading_error) / 90.0)
        
        # คำนวณความเร็วสุดท้าย
        linear_velocity = self.min_speed + (self.max_speed - self.min_speed) * turn_factor * distance_factor * heading_factor
        linear_velocity = max(self.min_speed, min(linear_velocity, self.max_speed))
        
        # สร้าง Twist message
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity
        
        # Debug logging
        self.get_logger().debug(f'Heading error: {heading_error:.1f}°, Steering: {math.degrees(steering_angle):.1f}°, Speed: {linear_velocity:.2f}m/s')
        
        return twist_msg
    
    def check_arrival(self):
        """ตรวจสอบการมาถึงจุดหมาย"""
        if not self.destination_point:
            return True
        
        distance = self.calculate_distance(
            self.current_position['lat'], self.current_position['lon'],
            self.destination_point['lat'], self.destination_point['lon']
        )
        
        if distance <= self.arrival_threshold:
            self.get_logger().info(f'🎉 ARRIVED! Distance: {distance:.2f}m')
            self.stop_navigation()
            
            # ส่งการแจ้งเตือน
            if self.connections:
                arrival_data = {'arrived': True, 'final_distance': distance}
                asyncio.run_coroutine_threadsafe(
                    self.send_to_all(json.dumps(arrival_data)),
                    self.websocket_loop
                )
            return True
        
        return False
    
    def update_current_waypoint(self):
        """อัพเดท waypoint ปัจจุบัน"""
        if self.current_waypoint_index >= len(self.route_points):
            return
        
        current_waypoint = self.route_points[self.current_waypoint_index]
        distance = self.calculate_distance(
            self.current_position['lat'], self.current_position['lon'],
            current_waypoint['lat'], current_waypoint['lon']
        )
        
        if distance <= self.waypoint_threshold:
            # ลบเส้นทางเก่าออกจากแผนที่
            self.remove_passed_route_segments()
            
            self.current_waypoint_index += 1
            self.get_logger().info(f'🎯 Passed waypoint {self.current_waypoint_index-1}, distance: {distance:.2f}m')
            
            # ตรวจสอบว่าถึง waypoint สุดท้ายหรือยัง
            if self.current_waypoint_index >= len(self.route_points):
                self.get_logger().info('Reached final waypoint')
    
    def remove_passed_route_segments(self):
        """ส่งคำสั่งให้ web ลบเส้นทางที่ผ่านมาแล้ว - เปิดกลับมาใหม่"""
        if self.connections and self.current_position:
            update_data = {
                'remove_passed_segments': True,
                'current_waypoint_index': self.current_waypoint_index,
                'current_position': {
                    'lat': self.current_position['lat'],
                    'lon': self.current_position['lon']
                }
            }
            
            try:
                asyncio.run_coroutine_threadsafe(
                    self.send_to_all(json.dumps(update_data)),
                    self.websocket_loop
                )
                self.get_logger().info(f'✂️ Removing passed route segments - waypoint {self.current_waypoint_index-1}')
            except Exception as e:
                self.get_logger().error(f'Error sending route update: {e}')
    
    def get_current_speed(self):
        """ประมาณความเร็วปัจจุบันจากประวัติตำแหน่ง"""
        if len(self.position_history) < 2:
            return 0.0
        
        # ใช้หลายจุดเพื่อความแม่นยำ
        total_distance = 0.0
        total_time = 0.0
        
        # คำนวณจาก 3 จุดล่าสุด
        for i in range(len(self.position_history) - 1, max(0, len(self.position_history) - 4), -1):
            if i == 0:
                break
                
            current_pos = self.position_history[i]
            prev_pos = self.position_history[i-1]
            
            distance = self.calculate_distance(
                prev_pos['lat'], prev_pos['lon'],
                current_pos['lat'], current_pos['lon']
            )
            
            time_diff = current_pos['timestamp'] - prev_pos['timestamp']
            
            if time_diff > 0 and distance > 0:
                total_distance += distance
                total_time += time_diff
        
        if total_time > 0:
            return total_distance / total_time
        else:
            return 0.0
    
    def get_distance_to_destination(self):
        """คำนวณระยะทางไปยังจุดหมาย"""
        if not self.current_position or not self.destination_point:
            return None
        
        return self.calculate_distance(
            self.current_position['lat'], self.current_position['lon'],
            self.destination_point['lat'], self.destination_point['lon']
        )
    
    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        """คำนวณ bearing"""
        geod = Geodesic.WGS84
        g = geod.Inverse(lat1, lon1, lat2, lon2)
        bearing = g['azi1']
        
        if bearing < 0:
            bearing += 360
        
        return bearing
    
    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """คำนวณระยะห่าง"""
        geod = Geodesic.WGS84
        g = geod.Inverse(lat1, lon1, lat2, lon2)
        return g['s12']
    
    def send_gnss_data_to_web(self):
        """ส่งข้อมูล GNSS ไปยัง web clients"""
        if not self.current_position:
            return
        
        # คำนวณระยะทางต่างๆ
        distance_to_destination = self.get_distance_to_destination()
        distance_to_lookahead = None
        
        if self.lookahead_point:
            distance_to_lookahead = self.calculate_distance(
                self.current_position['lat'], self.current_position['lon'],
                self.lookahead_point['lat'], self.lookahead_point['lon']
            )
        
        # คำนวณ progress
        route_progress = 0
        if len(self.route_points) > 0:
            route_progress = (self.current_waypoint_index / len(self.route_points) * 100)
        
        # สร้างข้อมูลส่ง
        gnss_data = {
            'latitude': self.current_position['lat'],
            'longitude': self.current_position['lon'],
            'navigation_active': self.navigation_active,
            'current_waypoint': self.current_waypoint_index,
            'total_waypoints': len(self.route_points),
            'distance_to_destination': distance_to_destination,
            'distance_to_lookahead': distance_to_lookahead,
            'route_progress': route_progress,
            'current_heading': self.current_heading,
            'heading_initialized': self.heading_initialized,
            'current_speed': self.get_current_speed(),
            'timestamp': time.time()
        }
        
        # เพิ่มข้อมูล lookahead point
        if self.lookahead_point:
            gnss_data['lookahead_lat'] = self.lookahead_point['lat']
            gnss_data['lookahead_lon'] = self.lookahead_point['lon']
        
        self.latest_gnss = gnss_data
        
        # ส่งข้อมูลไปยัง web clients
        if self.connections and self.websocket_loop:
            try:
                asyncio.run_coroutine_threadsafe(
                    self.send_to_all(json.dumps(gnss_data)),
                    self.websocket_loop
                )
            except Exception as e:
                self.get_logger().error(f'Error sending GNSS data: {e}')
    
    async def send_to_all(self, message):
        """ส่งข้อความไปยัง clients ทั้งหมด"""
        if not self.connections:
            return
        
        disconnected = set()
        
        for websocket in self.connections.copy():
            try:
                await websocket.send(message)
            except websockets.exceptions.ConnectionClosed:
                disconnected.add(websocket)
            except Exception as e:
                self.get_logger().error(f'Error sending to client: {e}')
                disconnected.add(websocket)
        
        for websocket in disconnected:
            if websocket in self.connections:
                self.connections.remove(websocket)

def main(args=None):
    rclpy.init(args=args)
    bridge = LookaheadNavigationBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.cleanup()
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()