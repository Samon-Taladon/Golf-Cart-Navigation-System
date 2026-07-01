# #!/usr/bin/env python3
# """
# 🚗 Smooth Pure Pursuit Golf Cart Navigation System
# ระบบนำทาง Pure Pursuit แบบเนียน (Continuous Path Following)
# Wheelbase: 1.67m, Steering: -50° to +50° (-0.873 to +0.873 rad)

# 🌟 NEW FEATURES:
# - Cubic Spline interpolation for smooth path
# - Continuous lookahead calculation
# - Curvature-based steering control
# - Smooth path following instead of point-to-point
# """

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String
import asyncio
import websockets
import json
import math
import time
import threading
import numpy as np
from scipy.interpolate import CubicSpline, interp1d
from scipy.optimize import minimize_scalar

class SmoothPurePursuitGolfCart(Node):
    def __init__(self):
        super().__init__('smooth_pure_pursuit_golf_cart')
        
        self.get_logger().info('🚗 Smooth Pure Pursuit Golf Cart Navigation Started!')
        
        # Vehicle Parameters
        self.WHEELBASE = 1.67  # meters
        self.MAX_STEERING_ANGLE = 0.873  # 50 degrees in radians
        self.MIN_STEERING_ANGLE = -0.873  # -50 degrees in radians
        
        # Smooth Pure Pursuit Parameters
        self.LOOKAHEAD_DISTANCE = 3.0  # meters
        self.MIN_LOOKAHEAD = 2.0
        self.MAX_LOOKAHEAD = 6.0
        self.SPEED_TO_LOOKAHEAD_RATIO = 2.5
        
        # 🌟 NEW: Smooth path parameters
        self.PATH_RESOLUTION = 0.5  # meters between interpolated points
        self.SMOOTHING_FACTOR = 0.1  # For spline smoothing
        self.CURVATURE_LOOKAHEAD = 5.0  # Distance to look ahead for curvature
        
        # ROS2 Publishers & Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribe to GNSS data
        self.gnss_sub = self.create_subscription(NavSatFix, '/navigation/gnss', self.gnss_callback, 10)
        self.xy_sub = self.create_subscription(Point, '/navigation/xy_position', self.xy_callback, 10)
        self.heading_sub = self.create_subscription(Float32, '/navigation/heading', self.heading_callback, 10)
        
        # Navigation State
        self.navigation_active = False
        self.raw_path_points = []  # Original waypoints
        self.smooth_path = None    # 🌟 NEW: Continuous smooth path
        self.path_length = 0.0     # Total path length
        self.current_pos = {'x': 0.0, 'y': 0.0}
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_heading = 0.0  # degrees
        self.target_speed = 1.0  # m/s
        
        # Reference point for coordinate conversion
        self.ref_lat = 13.650748
        self.ref_lon = 100.492985
        
        # 🌟 NEW: Smooth Path Following State
        self.current_path_s = 0.0  # Current position along path (arc length)
        self.lookahead_point = None
        self.path_curvature = 0.0  # Current path curvature
        self.cross_track_error = 0.0
        self.path_progress = 0.0
        
        # Control timing
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        
        # WebSocket Server
        self.connections = set()
        self.loop = None
        
        # Performance tracking
        self.start_time = 0
        self.total_distance = 0.0
        self.last_position = None
        
        # Debug
        self.debug_mode = True
        self.debug_counter = 0
        
        # Start WebSocket server
        self.websocket_thread = threading.Thread(target=self.run_websocket_server)
        self.websocket_thread.daemon = True
        self.websocket_thread.start()
        
        self.get_logger().info('✅ Smooth Pure Pursuit system ready!')
    
    def run_websocket_server(self):
        """Run WebSocket server in separate thread"""
        try:
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            
            start_server = websockets.serve(self.handle_websocket, 'localhost', 5001)
            self.loop.run_until_complete(start_server)
            self.get_logger().info('🌐 WebSocket server started on port 5001')
            self.loop.run_forever()
            
        except Exception as e:
            self.get_logger().error(f'❌ WebSocket server error: {e}')
    
    async def handle_websocket(self, websocket, path):
        """Handle WebSocket connections"""
        try:
            self.connections.add(websocket)
            self.get_logger().info('🔗 New client connected')
            
            async for message in websocket:
                try:
                    data = json.loads(message)
                    await self.handle_message(data)
                except json.JSONDecodeError:
                    self.get_logger().error('❌ Invalid JSON received')
                    
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.connections.discard(websocket)
            self.get_logger().info('🔌 Client disconnected')
    
    async def handle_message(self, data):
        """Handle incoming WebSocket messages"""
        command = data.get('command')
        
        if command == 'start_navigation':
            destination = data.get('destination')
            speed = data.get('speed', 1.0)
            if destination:
                self.start_navigation(destination['lat'], destination['lng'], speed)
            
        elif command == 'stop_navigation':
            self.stop_navigation()
            
        elif command == 'set_current_position':
            self.set_current_position(data['lat'], data['lng'], data.get('heading', 0))
        
        elif data.get('path'):
            # Received path from frontend
            await self.process_path_data(data)
        
        elif data.get('navigation_command'):
            cmd = data['navigation_command']
            if cmd == 'start':
                self.start_path_following()
            elif cmd == 'stop':
                self.stop_navigation()
            elif cmd == 'pause':
                self.pause_navigation()
    
    async def process_path_data(self, data):
        """🌟 NEW: Process path data and create smooth continuous path"""
        try:
            path_points = data.get('path', [])
            if len(path_points) < 2:
                self.get_logger().warn('⚠️ Path too short')
                return
            
            # Convert lat/lon to local coordinates
            self.raw_path_points = []
            for point in path_points:
                x, y = self.latlon_to_xy(point['lat'], point['lon'])
                self.raw_path_points.append((x, y))
            
            # 🌟 NEW: Create smooth continuous path
            self.create_smooth_path()
            
            # Reset navigation state
            self.current_path_s = 0.0
            self.path_progress = 0.0
            
            self.get_logger().info(
                f'🛣️ Smooth path created: {len(self.raw_path_points)} waypoints → '
                f'{len(self.smooth_path["x"])} smooth points, {self.path_length:.1f}m total'
            )
            
            # Send confirmation to frontend
            if self.connections:
                response = {
                    'route_received': True,
                    'smooth_points': len(self.smooth_path["x"]),
                    'original_points': len(self.raw_path_points),
                    'total_distance': self.path_length,
                    'algorithm': 'Smooth Pure Pursuit (Continuous)',
                    'wheelbase': self.WHEELBASE,
                    'max_steering': math.degrees(self.MAX_STEERING_ANGLE),
                    'path_resolution': self.PATH_RESOLUTION
                }
                await self.broadcast_message(response)
            
        except Exception as e:
            self.get_logger().error(f'❌ Smooth path creation error: {e}')
    
    def create_smooth_path(self):
        """🌟 NEW: Create smooth continuous path using cubic spline interpolation"""
        try:
            if len(self.raw_path_points) < 2:
                return
            
            # Extract x and y coordinates
            raw_x = [p[0] for p in self.raw_path_points]
            raw_y = [p[1] for p in self.raw_path_points]
            
            # Calculate cumulative distances for parameterization
            raw_distances = [0.0]
            for i in range(1, len(self.raw_path_points)):
                dx = raw_x[i] - raw_x[i-1]
                dy = raw_y[i] - raw_y[i-1]
                dist = math.sqrt(dx*dx + dy*dy)
                raw_distances.append(raw_distances[-1] + dist)
            
            # Create cubic splines for x(s) and y(s) where s is arc length
            if len(raw_distances) >= 3:
                # Use cubic spline for smooth interpolation
                spline_x = CubicSpline(raw_distances, raw_x, bc_type='natural')
                spline_y = CubicSpline(raw_distances, raw_y, bc_type='natural')
            else:
                # Fall back to linear interpolation for short paths
                spline_x = interp1d(raw_distances, raw_x, kind='linear', fill_value='extrapolate')
                spline_y = interp1d(raw_distances, raw_y, kind='linear', fill_value='extrapolate')
            
            # Generate smooth path points at regular intervals
            total_raw_length = raw_distances[-1]
            num_points = max(10, int(total_raw_length / self.PATH_RESOLUTION))
            s_values = np.linspace(0, total_raw_length, num_points)
            
            smooth_x = spline_x(s_values)
            smooth_y = spline_y(s_values)
            
            # Store smooth path with arc length parameterization
            self.smooth_path = {
                'x': smooth_x,
                'y': smooth_y,
                's': s_values,  # Arc length parameter
                'spline_x': spline_x,  # Keep splines for continuous queries
                'spline_y': spline_y
            }
            
            self.path_length = total_raw_length
            
            # 🌟 NEW: Calculate path curvature for each point
            self.calculate_path_curvature()
            
            if self.debug_mode:
                self.get_logger().info(
                    f"🌊 Smooth path created:\n"
                    f"  📊 Original: {len(self.raw_path_points)} points\n"
                    f"  🌟 Smooth: {len(smooth_x)} points\n"
                    f"  📏 Length: {self.path_length:.2f}m\n"
                    f"  🔧 Resolution: {self.PATH_RESOLUTION}m"
                )
            
        except Exception as e:
            self.get_logger().error(f'❌ Smooth path creation failed: {e}')
            # Fallback: use raw points
            self.smooth_path = {
                'x': np.array([p[0] for p in self.raw_path_points]),
                'y': np.array([p[1] for p in self.raw_path_points]),
                's': np.array([0] + [math.sqrt((self.raw_path_points[i][0]-self.raw_path_points[i-1][0])**2 + 
                                              (self.raw_path_points[i][1]-self.raw_path_points[i-1][1])**2) 
                                   for i in range(1, len(self.raw_path_points))]).cumsum()
            }
    
    def calculate_path_curvature(self):
        """🌟 NEW: Calculate curvature along the smooth path"""
        try:
            if 'spline_x' not in self.smooth_path:
                return
            
            s_values = self.smooth_path['s']
            spline_x = self.smooth_path['spline_x']
            spline_y = self.smooth_path['spline_y']
            
            # Calculate first and second derivatives
            dx_ds = spline_x.derivative(1)(s_values)  # dx/ds
            dy_ds = spline_y.derivative(1)(s_values)  # dy/ds
            d2x_ds2 = spline_x.derivative(2)(s_values)  # d²x/ds²
            d2y_ds2 = spline_y.derivative(2)(s_values)  # d²y/ds²
            
            # Curvature formula: κ = |x'y'' - y'x''| / (x'² + y'²)^(3/2)
            numerator = np.abs(dx_ds * d2y_ds2 - dy_ds * d2x_ds2)
            denominator = np.power(dx_ds**2 + dy_ds**2, 1.5)
            
            # Avoid division by zero
            denominator = np.maximum(denominator, 1e-10)
            curvatures = numerator / denominator
            
            self.smooth_path['curvature'] = curvatures
            
            if self.debug_mode:
                max_curv = np.max(curvatures)
                avg_curv = np.mean(curvatures)
                self.get_logger().info(f"📐 Path curvature: max={max_curv:.4f}, avg={avg_curv:.4f}")
            
        except Exception as e:
            self.get_logger().error(f'❌ Curvature calculation failed: {e}')
            # Fallback: zero curvature
            if self.smooth_path:
                self.smooth_path['curvature'] = np.zeros(len(self.smooth_path['s']))
    
    def find_closest_point_on_path(self):
        """🌟 NEW: Find closest point on smooth continuous path"""
        if not self.smooth_path:
            return 0.0, (0.0, 0.0)
        
        curr_x, curr_y = self.current_pos['x'], self.current_pos['y']
        
        # Find closest point using optimization
        def distance_to_path(s):
            if s < 0:
                s = 0
            elif s > self.path_length:
                s = self.path_length
            
            try:
                if 'spline_x' in self.smooth_path:
                    path_x = self.smooth_path['spline_x'](s)
                    path_y = self.smooth_path['spline_y'](s)
                else:
                    # Fallback: interpolate from discrete points
                    idx = np.searchsorted(self.smooth_path['s'], s)
                    if idx >= len(self.smooth_path['x']):
                        idx = len(self.smooth_path['x']) - 1
                    elif idx > 0 and s < self.smooth_path['s'][idx]:
                        # Linear interpolation
                        t = (s - self.smooth_path['s'][idx-1]) / (self.smooth_path['s'][idx] - self.smooth_path['s'][idx-1])
                        path_x = self.smooth_path['x'][idx-1] + t * (self.smooth_path['x'][idx] - self.smooth_path['x'][idx-1])
                        path_y = self.smooth_path['y'][idx-1] + t * (self.smooth_path['y'][idx] - self.smooth_path['y'][idx-1])
                    else:
                        path_x = self.smooth_path['x'][idx]
                        path_y = self.smooth_path['y'][idx]
                
                return math.sqrt((curr_x - path_x)**2 + (curr_y - path_y)**2)
            except:
                return float('inf')
        
        # Optimize to find closest point
        result = minimize_scalar(distance_to_path, bounds=(0, self.path_length), method='bounded')
        closest_s = result.x
        min_distance = result.fun
        
        # Get the closest point coordinates
        if 'spline_x' in self.smooth_path:
            closest_x = self.smooth_path['spline_x'](closest_s)
            closest_y = self.smooth_path['spline_y'](closest_s)
        else:
            # Fallback interpolation
            idx = np.searchsorted(self.smooth_path['s'], closest_s)
            if idx >= len(self.smooth_path['x']):
                closest_x = self.smooth_path['x'][-1]
                closest_y = self.smooth_path['y'][-1]
            else:
                closest_x = self.smooth_path['x'][idx]
                closest_y = self.smooth_path['y'][idx]
        
        self.current_path_s = closest_s
        self.cross_track_error = min_distance
        
        return closest_s, (closest_x, closest_y)
    
    def find_smooth_lookahead_point(self):
        """🌟 NEW: Find lookahead point on smooth continuous path"""
        if not self.smooth_path:
            return None
        
        # Get current position on path
        current_s, _ = self.find_closest_point_on_path()
        
        # Calculate adaptive lookahead distance
        current_speed = max(0.1, self.target_speed)
        lookahead_dist = max(self.MIN_LOOKAHEAD, 
                           min(self.MAX_LOOKAHEAD, 
                               current_speed * self.SPEED_TO_LOOKAHEAD_RATIO))
        
        # Find lookahead point along the path
        target_s = current_s + lookahead_dist
        
        # Clamp to path bounds
        target_s = max(0, min(self.path_length, target_s))
        
        # Get coordinates of lookahead point
        try:
            if 'spline_x' in self.smooth_path:
                lookahead_x = self.smooth_path['spline_x'](target_s)
                lookahead_y = self.smooth_path['spline_y'](target_s)
            else:
                # Fallback interpolation
                idx = np.searchsorted(self.smooth_path['s'], target_s)
                if idx >= len(self.smooth_path['x']):
                    lookahead_x = self.smooth_path['x'][-1]
                    lookahead_y = self.smooth_path['y'][-1]
                else:
                    lookahead_x = self.smooth_path['x'][idx]
                    lookahead_y = self.smooth_path['y'][idx]
            
            # Get path curvature at lookahead point for advanced control
            if 'curvature' in self.smooth_path:
                idx = np.searchsorted(self.smooth_path['s'], target_s)
                if idx < len(self.smooth_path['curvature']):
                    self.path_curvature = self.smooth_path['curvature'][idx]
                else:
                    self.path_curvature = 0.0
            
            return (lookahead_x, lookahead_y)
        
        except Exception as e:
            self.get_logger().error(f'❌ Lookahead calculation failed: {e}')
            return None
    
    def calculate_smooth_steering(self):
        """🌟 NEW: Calculate steering using smooth continuous path"""
        lookahead_point = self.find_smooth_lookahead_point()
        
        if not lookahead_point:
            return 0.0
        
        self.lookahead_point = lookahead_point
        
        # Current position and heading
        curr_x, curr_y = self.current_pos['x'], self.current_pos['y']
        target_x, target_y = lookahead_point
        
        # Vector from current position to lookahead point
        dx = target_x - curr_x
        dy = target_y - curr_y
        
        # Distance to lookahead point
        lookahead_distance = math.sqrt(dx*dx + dy*dy)
        
        if lookahead_distance < 0.1:
            return 0.0
        
        # Pure Pursuit calculation with smooth path
        heading_rad = math.radians(self.current_heading)
        target_global_angle = math.atan2(dy, dx)
        alpha = target_global_angle - heading_rad
        
        # Normalize alpha
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        
        # Basic Pure Pursuit steering
        steering_angle = math.atan2(2.0 * self.WHEELBASE * math.sin(alpha), lookahead_distance)
        
        # 🌟 NEW: Add curvature-based compensation for smoother control
        if hasattr(self, 'path_curvature') and self.path_curvature > 0:
            # Anticipate upcoming turns based on path curvature
            curvature_compensation = min(0.1, self.path_curvature * self.target_speed * 0.5)
            if alpha < 0:  # Turn right
                steering_angle -= curvature_compensation
            else:  # Turn left
                steering_angle += curvature_compensation
        
        # Limit steering angle
        steering_angle = max(self.MIN_STEERING_ANGLE, 
                           min(self.MAX_STEERING_ANGLE, steering_angle))
        
        # Debug logging
        if self.debug_mode:
            self.debug_counter += 1
            if self.debug_counter % 15 == 0:
                self.get_logger().info(
                    f"🌊 Smooth Pure Pursuit:\n"
                    f"  📍 Current s: {self.current_path_s:.2f}m / {self.path_length:.2f}m\n"
                    f"  🎯 Lookahead: ({target_x:.2f}, {target_y:.2f}), dist={lookahead_distance:.2f}m\n"
                    f"  📐 Alpha: {math.degrees(alpha):.1f}°, Curvature: {self.path_curvature:.4f}\n"
                    f"  🚗 Steering: {math.degrees(steering_angle):.2f}° ({steering_angle:+.4f} rad)\n"
                    f"  📊 Cross-track: {self.cross_track_error:.2f}m"
                )
        
        return steering_angle
    
    def calculate_path_progress(self):
        """Calculate progress along smooth path"""
        if not self.smooth_path or self.path_length == 0:
            return 0.0
        
        progress = (self.current_path_s / self.path_length) * 100
        self.path_progress = progress / 100.0
        return progress
    
    # [Keep all the existing callback methods unchanged]
    def gnss_callback(self, msg):
        """Handle GNSS position updates"""
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        
        # Convert to local coordinates
        x, y = self.latlon_to_xy(msg.latitude, msg.longitude)
        self.current_pos = {'x': x, 'y': y}
        
        # Update distance traveled
        if self.last_position:
            dx = x - self.last_position['x']
            dy = y - self.last_position['y']
            distance = math.sqrt(dx*dx + dy*dy)
            if distance < 10.0:  # Sanity check
                self.total_distance += distance
        
        self.last_position = {'x': x, 'y': y}
        
        # Send status to frontend
        asyncio.run_coroutine_threadsafe(self.send_status(), self.loop) if self.loop else None
    
    def xy_callback(self, msg):
        """Handle XY position updates"""
        self.current_pos = {'x': msg.x, 'y': msg.y}
    
    def heading_callback(self, msg):
        """Handle heading updates"""
        self.current_heading = msg.data
    
    def latlon_to_xy(self, lat, lon):
        """Convert lat/lon to local x,y coordinates"""
        dlat = lat - self.ref_lat
        dlon = lon - self.ref_lon
        
        x = dlon * 111319.9 * math.cos(math.radians(lat))
        y = dlat * 111319.9
        
        return x, y
    
    def start_path_following(self):
        """Start following the smooth path"""
        if not self.smooth_path:
            self.get_logger().error('❌ No smooth path available!')
            return
        
        self.navigation_active = True
        self.current_path_s = 0.0
        self.start_time = time.time()
        self.total_distance = 0.0
        
        self.get_logger().info('🚀 Smooth Pure Pursuit path following started!')
        
        # Notify frontend
        asyncio.run_coroutine_threadsafe(
            self.broadcast_message({'navigation_started': True}), 
            self.loop
        ) if self.loop else None
    
    def pause_navigation(self):
        """Pause navigation"""
        self.navigation_active = False
        self.stop_robot()
        self.get_logger().info('⏸️ Navigation paused')
    
    def stop_navigation(self):
        """Stop navigation completely"""
        self.navigation_active = False
        self.smooth_path = None
        self.current_path_s = 0.0
        self.stop_robot()
        self.get_logger().info('🛑 Navigation stopped')
    
    def stop_robot(self):
        """Send stop command to robot"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
    
    def control_loop(self):
        """🌟 NEW: Smooth control loop"""
        if not self.navigation_active or not self.smooth_path:
            return
        
        try:
            # Check if reached destination (near end of smooth path)
            progress_ratio = self.current_path_s / self.path_length if self.path_length > 0 else 0
            
            if progress_ratio > 0.95:  # Near end of path
                # Check actual distance to final point
                final_x = self.smooth_path['x'][-1]
                final_y = self.smooth_path['y'][-1]
                curr_x, curr_y = self.current_pos['x'], self.current_pos['y']
                distance_to_end = math.sqrt((curr_x - final_x)**2 + (curr_y - final_y)**2)
                
                if distance_to_end < 2.0:  # Reached destination
                    self.navigation_active = False
                    self.stop_robot()
                    
                    # Send arrival notification
                    asyncio.run_coroutine_threadsafe(
                        self.broadcast_message({
                            'arrived': True,
                            'total_time': time.time() - self.start_time,
                            'distance_traveled': self.total_distance,
                            'average_speed': self.total_distance / max(1, time.time() - self.start_time)
                        }), 
                        self.loop
                    ) if self.loop else None
                    
                    self.get_logger().info('🎉 Destination reached via smooth path!')
                    return
            
            # Calculate steering using smooth path
            steering_angle = self.calculate_smooth_steering()
            
            # 🌟 NEW: Advanced speed control based on path curvature
            speed = self.target_speed
            
            # Reduce speed for upcoming curves
            upcoming_curvature = self.get_upcoming_curvature()
            if upcoming_curvature > 0.1:  # High curvature ahead
                speed *= max(0.3, 1.0 - upcoming_curvature * 2.0)
            elif upcoming_curvature > 0.05:  # Medium curvature
                speed *= max(0.6, 1.0 - upcoming_curvature)
            
            # Reduce speed based on steering angle
            steering_deg = abs(math.degrees(steering_angle))
            if steering_deg > 20:
                speed *= 0.5
            elif steering_deg > 10:
                speed *= 0.7
            
            # Reduce speed based on cross track error
            if self.cross_track_error > 2.0:
                speed *= 0.4
            elif self.cross_track_error > 1.0:
                speed *= 0.7
            
            # Ensure reasonable speed range
            speed = max(0.1, min(self.target_speed, speed))
            
            # Publish control command
            twist = Twist()
            twist.linear.x = float(speed)
            twist.angular.z = float(steering_angle)
            self.cmd_vel_pub.publish(twist)
            
            # Regular logging
            current_time = time.time()
            if not hasattr(self, '_last_log_time') or current_time - self._last_log_time > 1.0:
                self._last_log_time = current_time
                progress = self.calculate_path_progress()
                
                self.get_logger().info(
                    f'🌊 Smooth Pure Pursuit Control:\n'
                    f'  🚗 Steering: {math.degrees(steering_angle):+6.2f}° | Speed: {speed:.2f}m/s\n'
                    f'  📊 Progress: {progress:.1f}% ({self.current_path_s:.1f}m / {self.path_length:.1f}m)\n'
                    f'  📏 Cross-track: {self.cross_track_error:.2f}m | Curvature: {self.path_curvature:.4f}\n'
                    f'  📍 Position: ({self.current_pos["x"]:.2f}, {self.current_pos["y"]:.2f})'
                )
        
        except Exception as e:
            self.get_logger().error(f'❌ Smooth control loop error: {e}')
            self.stop_robot()
    
    def get_upcoming_curvature(self):
        """🌟 NEW: Get curvature ahead of current position for speed planning"""
        if not self.smooth_path or 'curvature' not in self.smooth_path:
            return 0.0
        
        # Look ahead for curvature
        ahead_s = self.current_path_s + self.CURVATURE_LOOKAHEAD
        ahead_s = min(ahead_s, self.path_length)
        
        # Find curvature in the lookahead region
        s_values = self.smooth_path['s']
        curvatures = self.smooth_path['curvature']
        
        # Get max curvature in lookahead region
        mask = (s_values >= self.current_path_s) & (s_values <= ahead_s)
        if np.any(mask):
            return np.max(curvatures[mask])
        
        return 0.0
    
    async def send_status(self):
        """Send status to frontend"""
        if not self.connections:
            return
        
        try:
            progress = self.calculate_path_progress()
            
            # Calculate distance to destination
            distance_to_dest = None
            if self.smooth_path:
                final_x = self.smooth_path['x'][-1]
                final_y = self.smooth_path['y'][-1]
                curr_x, curr_y = self.current_pos['x'], self.current_pos['y']
                distance_to_dest = math.sqrt((curr_x - final_x)**2 + (curr_y - final_y)**2)
            
            status = {
                'latitude': self.current_lat,
                'longitude': self.current_lon,
                'current_speed': self.target_speed if self.navigation_active else 0.0,
                'navigation_active': self.navigation_active,
                'route_progress': progress,
                'distance_to_destination': distance_to_dest,
                'distance_traveled': self.total_distance,
                'cross_track_error': self.cross_track_error,
                'current_heading': self.current_heading,
                'algorithm': 'Smooth Pure Pursuit (Continuous Path)',
                'wheelbase': self.WHEELBASE,
                'lookahead_distance': self.LOOKAHEAD_DISTANCE,
                'path_curvature': self.path_curvature,
                'current_path_s': self.current_path_s,
                'path_length': self.path_length
            }
            
            # Add lookahead point if available
            if self.lookahead_point:
                lookahead_lat, lookahead_lon = self.xy_to_latlon(
                    self.lookahead_point[0], self.lookahead_point[1]
                )
                status['lookahead_lat'] = lookahead_lat
                status['lookahead_lon'] = lookahead_lon
            
            await self.broadcast_message(status)
            
        except Exception as e:
            self.get_logger().debug(f'Status send error: {e}')
    
    def xy_to_latlon(self, x, y):
        """Convert local x,y back to lat/lon"""
        dlat = y / 111319.9
        dlon = x / (111319.9 * math.cos(math.radians(self.ref_lat + dlat)))
        
        lat = self.ref_lat + dlat
        lon = self.ref_lon + dlon
        
        return lat, lon
    
    async def broadcast_message(self, message):
        """Broadcast message to all connected clients"""
        if not self.connections:
            return
        
        json_message = json.dumps(message)
        disconnected = set()
        
        for websocket in self.connections:
            try:
                await websocket.send(json_message)
            except websockets.exceptions.ConnectionClosed:
                disconnected.add(websocket)
        
        for websocket in disconnected:
            self.connections.discard(websocket)

def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    navigation = None
    
    try:
        navigation = SmoothPurePursuitGolfCart()
        print('🚀 Smooth Pure Pursuit Golf Cart Navigation running...')
        print('🌊 Using continuous path following with cubic spline interpolation!')
        rclpy.spin(navigation)
        
    except KeyboardInterrupt:
        print('🛑 Keyboard interrupt received')
    except Exception as e:
        print(f'❌ Error: {e}')
    finally:
        if navigation:
            navigation.destroy_node()
        rclpy.shutdown()
        print('🏁 Shutdown complete')

if __name__ == '__main__':
    main()




#!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, Point, PoseStamped
# from std_msgs.msg import Float32, String, Bool
# from nav_msgs.msg import Odometry
# import json
# import math
# import time
# import os
# import numpy as np
# from collections import deque

# class ImprovedPurePursuitController(Node):
#     def __init__(self):
#         super().__init__('pure_pursuit_controller')
        
#         # 🔧 IMPROVED: More tolerant control parameters
#         self.declare_parameters()
        
#         # Subscribers
#         self.position_sub = self.create_subscription(Point, '/navigation/xy_position', self.position_callback, 10)
#         self.heading_sub = self.create_subscription(Float32, '/navigation/heading', self.heading_callback, 10)
#         self.velocity_sub = self.create_subscription(Float32, '/navigation/velocity', self.velocity_callback, 10)
#         self.pose_sub = self.create_subscription(PoseStamped, '/navigation/pose', self.pose_callback, 10)
#         self.gnss_status_sub = self.create_subscription(String, '/navigation/gnss_status', self.gnss_status_callback, 10)
        
#         # Publishers
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.target_point_pub = self.create_publisher(Point, '/target_point', 10)
#         self.controller_status_pub = self.create_publisher(String, '/controller_status', 10)
#         self.control_enable_pub = self.create_publisher(Bool, '/control_enabled', 10)
        
#         # 🔧 IMPROVED: More tolerant control conditions
#         self.min_position_quality = ["FAIR", "GOOD", "EXCELLENT"]  # เพิ่ม FAIR
#         self.min_heading_quality = ["LOW", "MEDIUM", "HIGH"]  # เพิ่ม LOW (เงื่อนไขใจกว้าง)
#         self.max_cross_track_error = 200.0  # เพิ่มเป็น 200m
#         self.max_heading_error = 120.0  # เพิ่มเป็น 120 degrees
#         self.emergency_stop_cross_track = 500.0  # Emergency stop ที่ 500m
        
#         # Control parameters - ปรับให้ smooth มากขึ้น
#         self.lookahead_min = 2.0  # ระยะมองข้างหน้าต่ำสุด
#         self.lookahead_max = 15.0  # ระยะมองข้างหน้าสูงสุด
#         self.lookahead_gain = 2.0  # k * velocity
#         self.max_linear_velocity = 3.0  # m/s
#         self.max_angular_velocity = 1.5  # rad/s
#         self.min_velocity = 0.1  # m/s
        
#         # 🔧 NEW: Cross track error recovery
#         self.cross_track_recovery_mode = False
#         self.cross_track_recovery_speed = 0.5  # m/s for recovery
#         self.cross_track_tolerance = 50.0  # tolerance for normal operation
        
#         # 🔧 NEW: Heading fallback mechanism
#         self.use_position_heading = True  # ใช้ heading จากการเคลื่อนที่เมื่อ heading ไม่มี
#         self.position_heading_history = deque(maxlen=10)
        
#         # Control state
#         self.control_enabled = False
#         self.current_position = Point()
#         self.current_heading = None
#         self.current_velocity = 0.0
#         self.current_pose = None
        
#         # Position and heading quality
#         self.position_quality = "INVALID"
#         self.heading_quality = "INVALID"
#         self.heading_source = "NONE"
        
#         # Waypoints
#         self.waypoints = []
#         self.current_waypoint_index = 0
#         self.waypoint_file = "waypoints.json"
        
#         # 🔧 IMPROVED: Enhanced state tracking
#         self.last_position_time = 0
#         self.last_heading_time = 0
#         self.position_timeout = 2.0  # seconds
#         self.heading_timeout = 5.0  # seconds - เพิ่มเป็น 5 วินาที
        
#         # Control loop
#         self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz
#         self.status_timer = self.create_timer(1.0, self.publish_status)
        
#         # Statistics
#         self.stats = {
#             'control_cycles': 0,
#             'enabled_cycles': 0,
#             'waypoints_reached': 0,
#             'max_cross_track_error': 0.0,
#             'recovery_activations': 0
#         }
        
#         # Load waypoints
#         self.load_waypoints()
        
#         self.get_logger().info('🎯 Improved Pure Pursuit Controller initialized')
#         self.get_logger().info(f'📁 Loaded {len(self.waypoints)} waypoints')
#         self.get_logger().info(f'🔧 Tolerant mode: Cross track max={self.max_cross_track_error}m, Heading min={self.min_heading_quality}')

#     def declare_parameters(self):
#         """Declare ROS parameters"""
#         self.declare_parameter('lookahead_gain', 2.0)
#         self.declare_parameter('max_linear_velocity', 3.0) 
#         self.declare_parameter('max_angular_velocity', 1.5)
#         self.declare_parameter('cross_track_tolerance', 50.0)
#         self.declare_parameter('waypoint_tolerance', 2.0)

#     def load_waypoints(self):
#         """Load waypoints from JSON file"""
#         try:
#             if os.path.exists(self.waypoint_file):
#                 with open(self.waypoint_file, 'r') as f:
#                     data = json.load(f)
                    
#                 if 'waypoints' in data:
#                     self.waypoints = data['waypoints']
#                     self.get_logger().info(f"✅ Loaded {len(self.waypoints)} waypoints")
                    
#                     # Log first few waypoints for verification
#                     for i, wp in enumerate(self.waypoints[:3]):
#                         self.get_logger().info(f"  WP{i}: x={wp['x']:.1f}, y={wp['y']:.1f}, h={wp.get('heading', 'N/A')}")
                        
#                 else:
#                     self.get_logger().error("❌ No 'waypoints' key in JSON file")
#             else:
#                 self.get_logger().error(f"❌ Waypoint file not found: {self.waypoint_file}")
                
#         except Exception as e:
#             self.get_logger().error(f"❌ Failed to load waypoints: {e}")

#     def position_callback(self, msg):
#         """🔧 IMPROVED: Position callback with heading estimation"""
#         self.current_position = msg
#         current_time = time.time()
#         self.last_position_time = current_time
        
#         # 🔧 NEW: Calculate heading from position movement
#         if self.use_position_heading:
#             self.calculate_position_heading(msg, current_time)

#     def calculate_position_heading(self, position, timestamp):
#         """🔧 NEW: Calculate heading from position movement"""
#         self.position_heading_history.append({
#             'x': position.x,
#             'y': position.y,
#             'timestamp': timestamp
#         })
        
#         # ต้องมีอย่างน้อย 3 จุดเพื่อคำนวณ heading
#         if len(self.position_heading_history) >= 3:
#             recent = list(self.position_heading_history)[-3:]
            
#             # คำนวณ movement vector
#             total_dx = recent[-1]['x'] - recent[0]['x']
#             total_dy = recent[-1]['y'] - recent[0]['y']
#             total_distance = math.sqrt(total_dx*total_dx + total_dy*total_dy)
            
#             # ถ้าเคลื่อนที่เพียงพอ ให้คำนวณ heading
#             if total_distance > 0.5:  # เคลื่อนที่อย่างน้อย 0.5m
#                 position_heading_rad = math.atan2(total_dx, total_dy)
#                 position_heading_deg = math.degrees(position_heading_rad)
#                 if position_heading_deg < 0:
#                     position_heading_deg += 360
                
#                 # ใช้เป็น fallback heading ถ้าไม่มี heading หลัก
#                 if (self.current_heading is None or 
#                     (time.time() - self.last_heading_time) > self.heading_timeout):
#                     self.current_heading = position_heading_deg
#                     self.heading_quality = "ESTIMATED"  # แยกประเภท
#                     self.heading_source = "POSITION"

#     def heading_callback(self, msg):
#         """Heading callback"""
#         if not math.isnan(msg.data):
#             self.current_heading = msg.data
#             self.last_heading_time = time.time()

#     def velocity_callback(self, msg):
#         """Velocity callback"""
#         self.current_velocity = msg.data

#     def pose_callback(self, msg):
#         """Pose callback"""
#         self.current_pose = msg

#     def gnss_status_callback(self, msg):
#         """🔧 IMPROVED: GNSS status callback with better parsing"""
#         try:
#             status = json.loads(msg.data)
            
#             # Extract quality information
#             if 'position' in status:
#                 self.position_quality = status['position'].get('quality', 'INVALID')
            
#             if 'heading' in status:
#                 self.heading_quality = status['heading'].get('quality', 'INVALID')
#                 self.heading_source = status['heading'].get('source', 'NONE')
                
#         except Exception as e:
#             self.get_logger().debug(f"Failed to parse GNSS status: {e}")

#     def control_loop(self):
#         """🔧 IMPROVED: Main control loop with enhanced error handling"""
#         self.stats['control_cycles'] += 1
#         current_time = time.time()
        
#         # 🔧 IMPROVED: More tolerant state validation
#         position_valid = (current_time - self.last_position_time) < self.position_timeout
#         heading_valid = (self.current_heading is not None and 
#                         (current_time - self.last_heading_time) < self.heading_timeout)
        
#         # Position quality check
#         position_quality_ok = self.position_quality in self.min_position_quality
        
#         # 🔧 IMPROVED: More flexible heading quality check
#         heading_quality_ok = (self.heading_quality in self.min_heading_quality or 
#                              self.heading_quality == "ESTIMATED")  # อนุญาต ESTIMATED
        
#         # 🔧 NEW: Enhanced control enable logic
#         should_enable = (position_valid and position_quality_ok and 
#                         (heading_valid and heading_quality_ok))
        
#         # Special case: Enable with position-based heading if main heading fails
#         if not should_enable and position_valid and position_quality_ok:
#             if self.current_velocity > 0.2:  # เคลื่อนที่อยู่
#                 should_enable = True
#                 if self.current_heading is None:
#                     self.get_logger().info("🔄 Using position-based heading estimation")
        
#         if should_enable != self.control_enabled:
#             self.control_enabled = should_enable
#             status = "ENABLED" if should_enable else "DISABLED"
#             reason = f"Position: {self.position_quality}, Heading: {self.heading_quality}"
            
#             if should_enable:
#                 self.get_logger().info(f"🟢 Control {status} - {reason}")
#             else:
#                 self.get_logger().warn(f"🔴 Control {status} - {reason}")
        
#         # Publish control enable status
#         enable_msg = Bool()
#         enable_msg.data = self.control_enabled
#         self.control_enable_pub.publish(enable_msg)
        
#         # Execute control if enabled
#         if self.control_enabled:
#             self.execute_control()
#             self.stats['enabled_cycles'] += 1
#         else:
#             # Publish zero velocity
#             self.publish_zero_velocity()

#     def execute_control(self):
#         """🔧 IMPROVED: Execute pure pursuit control with enhanced error handling"""
#         if not self.waypoints or self.current_heading is None:
#             self.publish_zero_velocity()
#             return
        
#         try:
#             # Find target waypoint
#             target_waypoint, self.current_waypoint_index = self.find_target_waypoint()
            
#             if target_waypoint is None:
#                 self.get_logger().info("🏁 All waypoints completed!")
#                 self.publish_zero_velocity()
#                 return
            
#             # Publish target point for visualization
#             target_msg = Point()
#             target_msg.x = target_waypoint['x']
#             target_msg.y = target_waypoint['y']
#             self.target_point_pub.publish(target_msg)
            
#             # Calculate control commands
#             cmd_vel = self.calculate_pure_pursuit(target_waypoint)
            
#             # 🔧 NEW: Cross track error monitoring and recovery
#             cross_track_error = self.calculate_cross_track_error(target_waypoint)
#             self.stats['max_cross_track_error'] = max(self.stats['max_cross_track_error'], abs(cross_track_error))
            
#             # 🔧 NEW: Cross track error recovery mode
#             if abs(cross_track_error) > self.emergency_stop_cross_track:
#                 self.get_logger().error(f"🚨 EMERGENCY STOP - Cross track error: {cross_track_error:.1f}m")
#                 self.publish_zero_velocity()
#                 return
            
#             elif abs(cross_track_error) > self.cross_track_tolerance:
#                 if not self.cross_track_recovery_mode:
#                     self.cross_track_recovery_mode = True
#                     self.stats['recovery_activations'] += 1
#                     self.get_logger().warn(f"🔄 RECOVERY MODE - Cross track error: {cross_track_error:.1f}m")
                
#                 # In recovery mode, use slower speed and more direct approach
#                 cmd_vel = self.calculate_recovery_control(target_waypoint, cross_track_error)
                
#             else:
#                 if self.cross_track_recovery_mode:
#                     self.cross_track_recovery_mode = False
#                     self.get_logger().info("✅ Recovery mode OFF - back to normal operation")
            
#             # Publish command
#             self.cmd_vel_pub.publish(cmd_vel)
            
#             # Log periodic status
#             if self.stats['control_cycles'] % 50 == 0:  # ทุก 5 วินาที (10Hz * 50)
#                 self.get_logger().info(
#                     f"📊 Control: WP{self.current_waypoint_index}, "
#                     f"Cross track: {cross_track_error:.1f}m, "
#                     f"Target: [{target_waypoint['x']:.1f}, {target_waypoint['y']:.1f}], "
#                     f"Cmd: [{cmd_vel.linear.x:.2f}, {cmd_vel.angular.z:.2f}]"
#                 )
        
#         except Exception as e:
#             self.get_logger().error(f"❌ Control execution failed: {e}")
#             self.publish_zero_velocity()

#     def find_target_waypoint(self):
#         """🔧 IMPROVED: Find target waypoint with better logic"""
#         if not self.waypoints:
#             return None, 0
        
#         current_x = self.current_position.x
#         current_y = self.current_position.y
        
#         # Calculate lookahead distance based on velocity
#         lookahead_distance = self.lookahead_min + (self.lookahead_gain * self.current_velocity)
#         lookahead_distance = max(self.lookahead_min, min(self.lookahead_max, lookahead_distance))
        
#         # Start from current waypoint index
#         for i in range(self.current_waypoint_index, len(self.waypoints)):
#             waypoint = self.waypoints[i]
#             distance = math.sqrt((waypoint['x'] - current_x)**2 + (waypoint['y'] - current_y)**2)
            
#             # Check if we've reached current waypoint
#             if i == self.current_waypoint_index and distance < self.get_parameter('waypoint_tolerance').value:
#                 self.current_waypoint_index += 1
#                 self.stats['waypoints_reached'] += 1
#                 self.get_logger().info(f"✅ Waypoint {i} reached! Moving to waypoint {self.current_waypoint_index}")
#                 continue
            
#             # Find waypoint at lookahead distance
#             if distance >= lookahead_distance:
#                 return waypoint, i
        
#         # If no waypoint found at lookahead distance, use the last waypoint
#         if self.current_waypoint_index < len(self.waypoints):
#             return self.waypoints[-1], len(self.waypoints) - 1
        
#         return None, self.current_waypoint_index

#     def calculate_pure_pursuit(self, target_waypoint):
#         """🔧 IMPROVED: Calculate pure pursuit control commands"""
#         cmd_vel = Twist()
        
#         # Calculate relative position to target
#         dx = target_waypoint['x'] - self.current_position.x
#         dy = target_waypoint['y'] - self.current_position.y
#         distance_to_target = math.sqrt(dx*dx + dy*dy)
        
#         if distance_to_target < 0.1:
#             return cmd_vel  # Stop if very close
        
#         # Calculate target heading
#         target_heading_rad = math.atan2(dx, dy)
#         target_heading_deg = math.degrees(target_heading_rad)
#         if target_heading_deg < 0:
#             target_heading_deg += 360
        
#         # Calculate heading error
#         heading_error = target_heading_deg - self.current_heading
        
#         # Normalize heading error to [-180, 180]
#         while heading_error > 180:
#             heading_error -= 360
#         while heading_error < -180:
#             heading_error += 360
        
#         # 🔧 IMPROVED: More tolerant heading error check
#         if abs(heading_error) > self.max_heading_error:
#             # If heading error is too large, just rotate
#             cmd_vel.angular.z = math.copysign(self.max_angular_velocity * 0.5, heading_error)
#             cmd_vel.linear.x = 0.0
#             return cmd_vel
        
#         # Calculate lookahead distance
#         lookahead_distance = self.lookahead_min + (self.lookahead_gain * max(self.current_velocity, 0.1))
#         lookahead_distance = max(self.lookahead_min, min(self.lookahead_max, lookahead_distance))
        
#         # Pure pursuit angular velocity calculation
#         sin_alpha = dy / distance_to_target if distance_to_target > 0 else 0
#         angular_velocity = (2.0 * sin_alpha) / lookahead_distance
        
#         # Apply limits
#         angular_velocity = max(-self.max_angular_velocity, min(self.max_angular_velocity, angular_velocity))
        
#         # Calculate linear velocity based on heading error
#         heading_error_factor = max(0.0, 1.0 - (abs(heading_error) / 90.0))  # Reduce speed when turning
#         target_speed = target_waypoint.get('speed', 1.0)  # Default speed from waypoint
        
#         linear_velocity = target_speed * heading_error_factor
#         linear_velocity = max(self.min_velocity, min(self.max_linear_velocity, linear_velocity))
        
#         # Apply smoother turning reduction
#         if abs(angular_velocity) > 0.3:  # ลดความเร็วเมื่อเลี้ยว
#             linear_velocity *= 0.7
        
#         cmd_vel.linear.x = linear_velocity
#         cmd_vel.angular.z = angular_velocity
        
#         return cmd_vel

#     def calculate_recovery_control(self, target_waypoint, cross_track_error):
#         """🔧 NEW: Calculate control commands for cross track error recovery"""
#         cmd_vel = Twist()
        
#         # Calculate vector to target
#         dx = target_waypoint['x'] - self.current_position.x
#         dy = target_waypoint['y'] - self.current_position.y
#         distance_to_target = math.sqrt(dx*dx + dy*dy)
        
#         if distance_to_target < 0.1:
#             return cmd_vel
        
#         # In recovery mode, head directly toward the target
#         target_heading_rad = math.atan2(dx, dy)
#         target_heading_deg = math.degrees(target_heading_rad)
#         if target_heading_deg < 0:
#             target_heading_deg += 360
        
#         # Calculate heading error
#         heading_error = target_heading_deg - self.current_heading
#         while heading_error > 180:
#             heading_error -= 360
#         while heading_error < -180:
#             heading_error += 360
        
#         # Recovery control - more aggressive turning, slower speed
#         if abs(heading_error) > 30:  # Need significant turn
#             cmd_vel.angular.z = math.copysign(self.max_angular_velocity * 0.8, heading_error)
#             cmd_vel.linear.x = self.cross_track_recovery_speed * 0.3  # Very slow when turning
#         else:
#             # Head toward target with controlled speed
#             cmd_vel.angular.z = heading_error * 0.02  # Proportional control
#             cmd_vel.angular.z = max(-self.max_angular_velocity, min(self.max_angular_velocity, cmd_vel.angular.z))
#             cmd_vel.linear.x = self.cross_track_recovery_speed
        
#         return cmd_vel

#     def calculate_cross_track_error(self, target_waypoint):
#         """🔧 NEW: Calculate cross track error from path"""
#         # Simple distance to target point (can be enhanced with path-based calculation)
#         dx = target_waypoint['x'] - self.current_position.x
#         dy = target_waypoint['y'] - self.current_position.y
#         return math.sqrt(dx*dx + dy*dy)

#     def publish_zero_velocity(self):
#         """Publish zero velocity command"""
#         cmd_vel = Twist()
#         cmd_vel.linear.x = 0.0
#         cmd_vel.angular.z = 0.0
#         self.cmd_vel_pub.publish(cmd_vel)

#     def publish_status(self):
#         """🔧 IMPROVED: Publish comprehensive controller status"""
#         status = {
#             'timestamp': time.time(),
#             'control_enabled': self.control_enabled,
#             'control_mode': 'RECOVERY' if self.cross_track_recovery_mode else 'NORMAL',
#             'waypoint_progress': {
#                 'current_index': self.current_waypoint_index,
#                 'total_waypoints': len(self.waypoints),
#                 'progress_percent': (self.current_waypoint_index / len(self.waypoints) * 100) if self.waypoints else 0
#             },
#             'position': {
#                 'x': self.current_position.x,
#                 'y': self.current_position.y,
#                 'quality': self.position_quality
#             },
#             'heading': {
#                 'value': self.current_heading,
#                 'quality': self.heading_quality,
#                 'source': self.heading_source
#             },
#             'velocity': self.current_velocity,
#             'thresholds': {
#                 'max_cross_track_error': self.max_cross_track_error,
#                 'max_heading_error': self.max_heading_error,
#                 'emergency_stop_cross_track': self.emergency_stop_cross_track
#             },
#             'tolerances': {
#                 'position_timeout': self.position_timeout,
#                 'heading_timeout': self.heading_timeout,
#                 'min_position_quality': self.min_position_quality,
#                 'min_heading_quality': self.min_heading_quality
#             },
#             'statistics': self.stats
#         }
        
#         status_msg = String()
#         status_msg.data = json.dumps(status, default=str)
#         self.controller_status_pub.publish(status_msg)

#     def destroy_node(self):
#         """Cleanup"""
#         self.get_logger().info("🔄 Shutting down Pure Pursuit Controller...")
#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     controller = ImprovedPurePursuitController()
    
#     try:
#         rclpy.spin(controller)
#     except KeyboardInterrupt:
#         controller.get_logger().info("🛑 Keyboard interrupt")
#     finally:
#         controller.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()