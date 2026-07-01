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
from collections import deque
import pyproj

class FixedPathProgressionPurePursuitGolfCart(Node):
    def __init__(self):
        super().__init__('fixed_path_progression_pure_pursuit_golf_cart')
        
        self.get_logger().info('🚗 Fixed Path Progression Pure Pursuit Golf Cart Navigation Started!')
        
        # Vehicle Parameters
        self.WHEELBASE = 1.67  # meters
        self.MAX_STEERING_ANGLE = 0.873  # 50 degrees in radians
        self.MIN_STEERING_ANGLE = -0.873  # -50 degrees in radians
        
        # Pure Pursuit Parameters
        self.LOOKAHEAD_DISTANCE = 3.0  # meters
        self.MIN_LOOKAHEAD = 2.0
        self.MAX_LOOKAHEAD = 6.0
        self.SPEED_TO_LOOKAHEAD_RATIO = 2.5
        
        # 🔧 Fixed Path Parameters
        self.PATH_RESOLUTION = 0.3  # meters between interpolated points
        self.SMOOTHING_FACTOR = 0.05
        self.CURVATURE_LOOKAHEAD = 5.0
        self.USE_LINEAR_INTERPOLATION = True  # 🔧 Force linear to avoid derivative issues
        self.MAX_PATH_DEVIATION = 5.0
        
        # 🗺️ Path Progression Parameters (Google Maps Style)
        self.PROGRESS_DETECTION_DISTANCE = 8.0  # meters
        self.PROGRESS_CONFIRMATION_TIME = 2.0   # seconds
        self.MIN_FORWARD_PROGRESS = 0.5         # meters
        self.BACKWARD_PENALTY_DISTANCE = 10.0   # meters
        self.PATH_COMPLETION_THRESHOLD = 0.95   # 95%
        
        # 🔧 Fixed Coordinate System
        try:
            self.transformer = pyproj.Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)
        except Exception as e:
            self.get_logger().warning(f'⚠️ PyProj failed, using simple conversion: {e}')
            self.transformer = None
        
        self.ref_lat = 13.650748
        self.ref_lon = 100.492985
        self.reference_utm = None
        
        # ROS2 Publishers & Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gnss_sub = self.create_subscription(NavSatFix, '/navigation/gnss', self.gnss_callback, 10)
        self.xy_sub = self.create_subscription(Point, '/navigation/xy_position', self.xy_callback, 10)
        self.heading_sub = self.create_subscription(Float32, '/navigation/heading', self.heading_callback, 10)
        
        # Navigation State
        self.navigation_active = False
        self.raw_path_points = []
        self.smooth_path = None
        self.path_length = 0.0
        self.current_pos = {'x': 0.0, 'y': 0.0}
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_heading = 0.0
        self.target_speed = 1.0
        
        # 🔧 Fixed Path Following State
        self.current_path_s = 0.0
        self.lookahead_point = None
        self.path_curvature = 0.0
        self.cross_track_error = 0.0
        self.path_progress = 0.0
        
        # 🗺️ Path Progression State
        self.max_progress_s = 0.0
        self.progress_history = deque(maxlen=20)
        self.last_progress_time = 0.0
        self.completed_segments = set()
        self.path_completion_status = {}
        self.forward_progress_guarantee = True
        
        # 🔧 Fixed Heading Filter
        self.heading_filter = FixedHeadingFilter()
        self.final_heading_filtered = 0.0  # 🔧 Initialize with default value
        self.heading_source_filtered = "INITIALIZING"
        
        # Control timing
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
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
        
        # 🔧 Initialize coordinate system
        self.init_coordinate_system()
        
        # Start WebSocket server
        self.websocket_thread = threading.Thread(target=self.run_websocket_server)
        self.websocket_thread.daemon = True
        self.websocket_thread.start()
        
        self.get_logger().info('✅ Fixed Path Progression Pure Pursuit system ready!')
        self.get_logger().info('🗺️ Google Maps style path progression enabled!')
    
    def init_coordinate_system(self):
        """🔧 Fixed coordinate system initialization"""
        try:
            if self.transformer:
                self.reference_utm = self.transformer.transform(self.ref_lon, self.ref_lat)
                self.get_logger().info(
                    f'🔧 Coordinate system initialized:\n'
                    f'  📍 Reference: ({self.ref_lat:.6f}, {self.ref_lon:.6f})\n'
                    f'  📍 UTM: ({self.reference_utm[0]:.2f}, {self.reference_utm[1]:.2f})'
                )
            else:
                self.reference_utm = None
                self.get_logger().info('🔧 Using simple coordinate conversion (PyProj unavailable)')
        except Exception as e:
            self.get_logger().error(f'❌ Coordinate system init failed: {e}')
            self.reference_utm = None
    
    def run_websocket_server(self):
        """Run WebSocket server in separate thread"""
        try:
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            
            start_server = websockets.serve(self.handle_websocket, 'localhost', 5001)
            self.loop.run_until_complete(start_server)
            self.get_logger().info('🌐 Fixed Path Progression WebSocket server started on port 5001')
            self.loop.run_forever()
            
        except Exception as e:
            self.get_logger().error(f'❌ WebSocket server error: {e}')
    
    async def handle_websocket(self, websocket, path):
        """Handle WebSocket connections"""
        try:
            self.connections.add(websocket)
            self.get_logger().info('🔗 New client connected to Fixed Path Progression system')
            
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
        if data.get('path'):
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
        """🔧 Fixed path data processing"""
        try:
            path_points = data.get('path', [])
            if len(path_points) < 2:
                self.get_logger().warn('⚠️ Path too short')
                return
            
            self.get_logger().info(f'🗺️ Processing path with progression tracking: {len(path_points)} points')
            
            # Convert lat/lon to local coordinates
            self.raw_path_points = []
            for point in path_points:
                x, y = self.latlon_to_xy(point['lat'], point['lon'])
                self.raw_path_points.append((x, y))
            
            # 🔧 Create fixed smooth path
            self.create_fixed_smooth_path_with_progression()
            
            # 🗺️ Initialize path progression tracking
            self.init_path_progression()
            
            # Reset navigation state
            self.current_path_s = 0.0
            self.path_progress = 0.0
            self.max_progress_s = 0.0
            
            self.get_logger().info(
                f'🛣️ Fixed path with progression created:\n'
                f'  📊 Original: {len(self.raw_path_points)} waypoints\n'
                f'  🌟 Smooth: {len(self.smooth_path["x"]) if self.smooth_path else 0} points\n'
                f'  📏 Length: {self.path_length:.1f}m\n'
                f'  🗺️ Progression segments: {len(self.path_completion_status)}'
            )
            
            # Send confirmation to frontend
            if self.connections:
                response = {
                    'route_received': True,
                    'smooth_points': len(self.smooth_path["x"]) if self.smooth_path else 0,
                    'original_points': len(self.raw_path_points),
                    'total_distance': self.path_length,
                    'algorithm': 'Fixed Path Progression Pure Pursuit (Bug-free)',
                    'coordinate_system': 'UTM Zone 47N (Fixed)',
                    'wheelbase': self.WHEELBASE,
                    'max_steering': math.degrees(self.MAX_STEERING_ANGLE),
                    'path_resolution': self.PATH_RESOLUTION,
                    'progression_enabled': True,
                    'progression_segments': len(self.path_completion_status)
                }
                await self.broadcast_message(response)
            
        except Exception as e:
            self.get_logger().error(f'❌ Fixed path progression creation error: {e}')
    
    def create_fixed_smooth_path_with_progression(self):
        """🔧 Fixed smooth path creation (no derivative issues)"""
        try:
            if len(self.raw_path_points) < 2:
                return
            
            # Extract coordinates
            raw_x = [p[0] for p in self.raw_path_points]
            raw_y = [p[1] for p in self.raw_path_points]
            
            # Calculate cumulative distances
            raw_distances = [0.0]
            for i in range(1, len(self.raw_path_points)):
                dx = raw_x[i] - raw_x[i-1]
                dy = raw_y[i] - raw_y[i-1]
                dist = math.sqrt(dx*dx + dy*dy)
                raw_distances.append(raw_distances[-1] + dist)
            
            total_length = raw_distances[-1]
            
            # 🔧 Always use linear interpolation to avoid derivative issues
            spline_x = interp1d(raw_distances, raw_x, kind='linear', fill_value='extrapolate')
            spline_y = interp1d(raw_distances, raw_y, kind='linear', fill_value='extrapolate')
            
            # Generate path points
            num_points = max(15, int(total_length / self.PATH_RESOLUTION))
            s_values = np.linspace(0, total_length, num_points)
            
            smooth_x = spline_x(s_values)
            smooth_y = spline_y(s_values)
            
            # Store path
            self.smooth_path = {
                'x': smooth_x,
                'y': smooth_y,
                's': s_values,
                'spline_x': spline_x,  # Keep for consistency but won't use derivative
                'spline_y': spline_y,
                'raw_x': raw_x,
                'raw_y': raw_y,
                'interpolation_method': 'linear_fixed'
            }
            
            self.path_length = total_length
            
            # 🔧 Calculate curvature safely
            self.calculate_fixed_path_curvature()
            
            self.get_logger().info(f"🗺️ Fixed path with progression tracking created: {len(smooth_x)} points, {total_length:.2f}m")
            
        except Exception as e:
            self.get_logger().error(f'❌ Fixed path progression creation failed: {e}')
            self.create_fallback_path()
    
    def calculate_fixed_path_curvature(self):
        """🔧 Fixed curvature calculation (no derivative method calls)"""
        try:
            if not self.smooth_path or len(self.smooth_path['x']) < 3:
                # Default to zero curvature
                if self.smooth_path:
                    self.smooth_path['curvature'] = np.zeros(len(self.smooth_path['x']))
                return
            
            # 🔧 Use discrete curvature calculation only
            self.calculate_discrete_curvature_fixed()
            
        except Exception as e:
            self.get_logger().error(f'❌ Fixed curvature calculation failed: {e}')
            # Fallback to zero curvature
            if self.smooth_path:
                self.smooth_path['curvature'] = np.zeros(len(self.smooth_path['x']))
    
    def calculate_discrete_curvature_fixed(self):
        """🔧 Safe discrete curvature calculation"""
        try:
            x_points = self.smooth_path['x']
            y_points = self.smooth_path['y']
            curvatures = np.zeros(len(x_points))
            
            for i in range(1, len(x_points) - 1):
                try:
                    # Three point curvature approximation
                    x1, y1 = float(x_points[i-1]), float(y_points[i-1])
                    x2, y2 = float(x_points[i]), float(y_points[i])
                    x3, y3 = float(x_points[i+1]), float(y_points[i+1])
                    
                    # Calculate side lengths
                    a = math.sqrt((x2-x1)**2 + (y2-y1)**2)
                    b = math.sqrt((x3-x2)**2 + (y3-y2)**2)
                    c = math.sqrt((x3-x1)**2 + (y3-y1)**2)
                    
                    if a > 1e-6 and b > 1e-6 and c > 1e-6:
                        # Calculate area using Heron's formula
                        s = (a + b + c) / 2
                        area_squared = s * (s-a) * (s-b) * (s-c)
                        
                        if area_squared > 0:
                            area = math.sqrt(area_squared)
                            curvatures[i] = 4 * area / (a * b * c)
                        else:
                            curvatures[i] = 0.0
                    else:
                        curvatures[i] = 0.0
                        
                except Exception as e:
                    curvatures[i] = 0.0
            
            # Set endpoints
            curvatures[0] = curvatures[1] if len(curvatures) > 1 else 0.0
            curvatures[-1] = curvatures[-2] if len(curvatures) > 1 else 0.0
            
            self.smooth_path['curvature'] = curvatures
            
        except Exception as e:
            self.get_logger().error(f'❌ Discrete curvature calculation failed: {e}')
            # Ultimate fallback
            if self.smooth_path:
                self.smooth_path['curvature'] = np.zeros(len(self.smooth_path['x']))
    
    def create_fallback_path(self):
        """🔧 Safe fallback path creation"""
        try:
            raw_x = [p[0] for p in self.raw_path_points]
            raw_y = [p[1] for p in self.raw_path_points]
            
            # Calculate distances
            distances = [0.0]
            for i in range(1, len(self.raw_path_points)):
                dx = raw_x[i] - raw_x[i-1]
                dy = raw_y[i] - raw_y[i-1]
                dist = math.sqrt(dx*dx + dy*dy)
                distances.append(distances[-1] + dist)
            
            self.smooth_path = {
                'x': np.array(raw_x),
                'y': np.array(raw_y),
                's': np.array(distances),
                'curvature': np.zeros(len(raw_x)),  # Safe default
                'interpolation_method': 'raw_fallback'
            }
            self.path_length = distances[-1] if distances else 0.0
            
            self.get_logger().info("🔧 Using fixed fallback path (raw points)")
            
        except Exception as e:
            self.get_logger().error(f'❌ Fallback path creation failed: {e}')
    
    def init_path_progression(self):
        """🗺️ Initialize path progression tracking"""
        try:
            if not self.smooth_path:
                return
            
            # Create progression segments
            segment_length = 20.0  # meters per segment
            num_segments = max(5, int(self.path_length / segment_length))
            
            self.path_completion_status = {}
            
            for i in range(num_segments):
                start_s = (i / num_segments) * self.path_length
                end_s = ((i + 1) / num_segments) * self.path_length
                
                self.path_completion_status[i] = {
                    'start_s': start_s,
                    'end_s': end_s,
                    'completed': False,
                    'completion_time': None,
                    'status': 'pending'
                }
            
            self.get_logger().info(f'🗺️ Path progression initialized: {num_segments} segments')
            
        except Exception as e:
            self.get_logger().error(f'❌ Path progression init failed: {e}')
    
    def update_path_progression(self):
        """🗺️ Update path progression status"""
        try:
            if not self.smooth_path or not self.path_completion_status:
                return
            
            current_time = time.time()
            
            # Update maximum progress (forward guarantee)
            if self.current_path_s > self.max_progress_s:
                progress_made = self.current_path_s - self.max_progress_s
                
                if progress_made >= self.MIN_FORWARD_PROGRESS:
                    self.max_progress_s = self.current_path_s
                    self.last_progress_time = current_time
                    
                    self.progress_history.append({
                        'time': current_time,
                        's': self.current_path_s,
                        'progress': progress_made
                    })
            
            # Update segment completion status
            for segment_id, segment in self.path_completion_status.items():
                segment_center = (segment['start_s'] + segment['end_s']) / 2
                distance_to_segment = abs(self.current_path_s - segment_center)
                
                if distance_to_segment <= self.PROGRESS_DETECTION_DISTANCE and self.current_path_s >= (segment['start_s'] - 5.0):
                    if segment['status'] == 'pending':
                        segment['status'] = 'approaching'
                
                if self.current_path_s > segment['end_s'] + self.PROGRESS_DETECTION_DISTANCE:
                    if segment['status'] in ['pending', 'approaching']:
                        segment['status'] = 'completed'
                        segment['completed'] = True
                        segment['completion_time'] = current_time
                        self.completed_segments.add(segment_id)
                
                elif self.current_path_s > segment['end_s'] + 20.0:
                    if segment['status'] != 'passed':
                        segment['status'] = 'passed'
            
        except Exception as e:
            self.get_logger().error(f'❌ Path progression update failed: {e}')
    
    def find_closest_point_with_progression(self):
        """🗺️ Find closest point with forward progress guarantee"""
        try:
            if not self.smooth_path:
                return 0.0, (0.0, 0.0)
            
            curr_x, curr_y = self.current_pos['x'], self.current_pos['y']
            
            # Normal closest point calculation
            closest_s, closest_point = self.find_closest_point_normal(curr_x, curr_y)
            
            # Apply forward progress guarantee
            if self.forward_progress_guarantee:
                min_allowed_s = max(0, self.max_progress_s - self.BACKWARD_PENALTY_DISTANCE)
                
                if closest_s < min_allowed_s:
                    closest_s = min_allowed_s
                    try:
                        # Safe point calculation
                        idx = np.searchsorted(self.smooth_path['s'], closest_s)
                        if idx < len(self.smooth_path['x']):
                            closest_point = (float(self.smooth_path['x'][idx]), float(self.smooth_path['y'][idx]))
                    except:
                        closest_point = (curr_x, curr_y)  # Safe fallback
            
            self.current_path_s = closest_s
            
            # Calculate cross-track error
            try:
                self.cross_track_error = math.sqrt(
                    (curr_x - closest_point[0])**2 + (curr_y - closest_point[1])**2
                )
            except:
                self.cross_track_error = 0.0
            
            return closest_s, closest_point
            
        except Exception as e:
            self.get_logger().error(f'❌ Find closest point failed: {e}')
            return 0.0, (self.current_pos['x'], self.current_pos['y'])
    
    def find_closest_point_normal(self, curr_x, curr_y):
        """🔧 Safe normal closest point calculation"""
        try:
            if not self.smooth_path or len(self.smooth_path['x']) == 0:
                return 0.0, (curr_x, curr_y)
            
            # Use discrete search for safety
            min_distance = float('inf')
            closest_idx = 0
            
            for i, (px, py) in enumerate(zip(self.smooth_path['x'], self.smooth_path['y'])):
                try:
                    dist = math.sqrt((curr_x - float(px))**2 + (curr_y - float(py))**2)
                    if dist < min_distance:
                        min_distance = dist
                        closest_idx = i
                except:
                    continue
            
            closest_s = float(self.smooth_path['s'][closest_idx]) if 's' in self.smooth_path else 0.0
            closest_x = float(self.smooth_path['x'][closest_idx])
            closest_y = float(self.smooth_path['y'][closest_idx])
            
            return closest_s, (closest_x, closest_y)
            
        except Exception as e:
            self.get_logger().error(f'❌ Normal closest point calculation failed: {e}')
            return 0.0, (curr_x, curr_y)
    
    def find_lookahead_point_with_progression(self):
        """🗺️ Find lookahead point with progression awareness"""
        try:
            if not self.smooth_path:
                return None
            
            # Get current position on path
            current_s, _ = self.find_closest_point_with_progression()
            
            # Calculate lookahead distance
            current_speed = max(0.1, self.target_speed)
            base_lookahead = max(self.MIN_LOOKAHEAD, 
                               min(self.MAX_LOOKAHEAD, 
                                   current_speed * self.SPEED_TO_LOOKAHEAD_RATIO))
            
            # Get upcoming curvature safely
            try:
                upcoming_curvature = self.get_upcoming_curvature()
                curvature_factor = max(0.5, 1.0 - upcoming_curvature * 3.0)
                lookahead_dist = base_lookahead * curvature_factor
            except:
                lookahead_dist = base_lookahead
            
            # Find lookahead point
            target_s = current_s + lookahead_dist
            target_s = max(current_s, min(self.path_length, target_s))
            
            # Get coordinates safely
            try:
                idx = np.searchsorted(self.smooth_path['s'], target_s)
                if idx >= len(self.smooth_path['x']):
                    lookahead_x = float(self.smooth_path['x'][-1])
                    lookahead_y = float(self.smooth_path['y'][-1])
                elif idx == 0:
                    lookahead_x = float(self.smooth_path['x'][0])
                    lookahead_y = float(self.smooth_path['y'][0])
                else:
                    # Linear interpolation
                    t = (target_s - self.smooth_path['s'][idx-1]) / (self.smooth_path['s'][idx] - self.smooth_path['s'][idx-1])
                    lookahead_x = float(self.smooth_path['x'][idx-1] + t * (self.smooth_path['x'][idx] - self.smooth_path['x'][idx-1]))
                    lookahead_y = float(self.smooth_path['y'][idx-1] + t * (self.smooth_path['y'][idx] - self.smooth_path['y'][idx-1]))
                
                return (lookahead_x, lookahead_y)
                
            except Exception as e:
                self.get_logger().error(f'❌ Lookahead coordinate calculation failed: {e}')
                return None
        
        except Exception as e:
            self.get_logger().error(f'❌ Lookahead calculation failed: {e}')
            return None
    
    def get_upcoming_curvature(self):
        """🔧 Safe upcoming curvature calculation"""
        try:
            if not self.smooth_path or 'curvature' not in self.smooth_path:
                return 0.0
            
            ahead_s = self.current_path_s + self.CURVATURE_LOOKAHEAD
            ahead_s = min(ahead_s, self.path_length)
            
            s_values = self.smooth_path['s']
            curvatures = self.smooth_path['curvature']
            
            mask = (s_values >= self.current_path_s) & (s_values <= ahead_s)
            if np.any(mask):
                return float(np.max(curvatures[mask]))
            
            return 0.0
            
        except Exception as e:
            self.get_logger().debug(f'Upcoming curvature calculation failed: {e}')
            return 0.0
    
    def calculate_steering_with_progression(self):
        """🗺️ Calculate steering with progression awareness"""
        try:
            lookahead_point = self.find_lookahead_point_with_progression()
            
            if not lookahead_point:
                return 0.0
            
            self.lookahead_point = lookahead_point
            
            # Current position and heading
            curr_x, curr_y = self.current_pos['x'], self.current_pos['y']
            target_x, target_y = lookahead_point
            
            # Vector to lookahead point
            dx = target_x - curr_x
            dy = target_y - curr_y
            lookahead_distance = math.sqrt(dx*dx + dy*dy)
            
            if lookahead_distance < 0.1:
                return 0.0
            
            # 🔧 Safe heading usage
            heading_deg = self.final_heading_filtered if self.final_heading_filtered is not None else self.current_heading
            heading_rad = math.radians(float(heading_deg))
            
            target_global_angle = math.atan2(dy, dx)
            alpha = target_global_angle - heading_rad
            alpha = math.atan2(math.sin(alpha), math.cos(alpha))
            
            # Pure Pursuit steering
            steering_angle = math.atan2(2.0 * self.WHEELBASE * math.sin(alpha), lookahead_distance)
            
            # Safe curvature compensation
            try:
                if hasattr(self, 'path_curvature') and self.path_curvature > 0:
                    curvature_compensation = min(0.1, self.path_curvature * self.target_speed * 0.3)
                    if alpha < 0:
                        steering_angle -= curvature_compensation
                    else:
                        steering_angle += curvature_compensation
            except:
                pass  # Skip curvature compensation if error
            
            # Cross-track error compensation
            try:
                if abs(self.cross_track_error) > 1.0:
                    error_compensation = min(0.2, abs(self.cross_track_error) * 0.1)
                    if self.cross_track_error > 0:
                        steering_angle += error_compensation
                    else:
                        steering_angle -= error_compensation
            except:
                pass  # Skip error compensation if error
            
            # Limit steering angle
            steering_angle = max(self.MIN_STEERING_ANGLE, 
                               min(self.MAX_STEERING_ANGLE, steering_angle))
            
            return float(steering_angle)
            
        except Exception as e:
            self.get_logger().error(f'❌ Steering calculation failed: {e}')
            return 0.0
    
    def calculate_path_progress(self):
        """Calculate progress with safe handling"""
        try:
            if not self.smooth_path or self.path_length == 0:
                return 0.0
            
            progress_s = max(self.current_path_s, self.max_progress_s)
            progress = (progress_s / self.path_length) * 100
            self.path_progress = progress / 100.0
            return float(progress)
            
        except Exception as e:
            self.get_logger().debug(f'Progress calculation failed: {e}')
            return 0.0
    
    # 🔧 Fixed ROS callbacks
    def gnss_callback(self, msg):
        """Handle GNSS position updates"""
        try:
            self.current_lat = float(msg.latitude)
            self.current_lon = float(msg.longitude)
            
            x, y = self.latlon_to_xy(msg.latitude, msg.longitude)
            self.current_pos = {'x': float(x), 'y': float(y)}
            
            if self.last_position:
                dx = x - self.last_position['x']
                dy = y - self.last_position['y']
                distance = math.sqrt(dx*dx + dy*dy)
                if distance < 10.0:  # Sanity check
                    self.total_distance += distance
            
            self.last_position = {'x': float(x), 'y': float(y)}
            
            # Send status to frontend
            if self.loop:
                asyncio.run_coroutine_threadsafe(self.send_status(), self.loop)
                
        except Exception as e:
            self.get_logger().error(f'❌ GNSS callback failed: {e}')
    
    def xy_callback(self, msg):
        """Handle XY position updates"""
        try:
            self.current_pos = {'x': float(msg.x), 'y': float(msg.y)}
        except Exception as e:
            self.get_logger().debug(f'XY callback failed: {e}')
    
    def heading_callback(self, msg):
        """🔧 Fixed heading callback with safe filtering"""
        try:
            raw_heading = float(msg.data)
            self.current_heading = raw_heading
            
            current_speed = self.target_speed if self.navigation_active else 0.0
            filtered_result = self.heading_filter.filter_heading(raw_heading, current_speed)
            
            if filtered_result and len(filtered_result) >= 2:
                self.final_heading_filtered = filtered_result[0] if filtered_result[0] is not None else raw_heading
                self.heading_source_filtered = filtered_result[1] if filtered_result[1] is not None else "RAW"
            else:
                self.final_heading_filtered = raw_heading
                self.heading_source_filtered = "RAW"
                
        except Exception as e:
            self.get_logger().error(f'❌ Heading callback failed: {e}')
            # Safe fallback
            try:
                self.final_heading_filtered = float(msg.data)
                self.heading_source_filtered = "FALLBACK"
            except:
                self.final_heading_filtered = 0.0
                self.heading_source_filtered = "ERROR"
    
    def latlon_to_xy(self, lat, lon):
        """🔧 Safe coordinate conversion"""
        try:
            if self.transformer and self.reference_utm:
                utm_x, utm_y = self.transformer.transform(lon, lat)
                local_x = utm_x - self.reference_utm[0]
                local_y = utm_y - self.reference_utm[1]
                return local_x, local_y
            else:
                return self.simple_latlon_to_xy(lat, lon)
        except Exception as e:
            self.get_logger().debug(f'Coordinate conversion failed: {e}')
            return self.simple_latlon_to_xy(lat, lon)
    
    def simple_latlon_to_xy(self, lat, lon):
        """Safe simple conversion"""
        try:
            dlat = float(lat) - self.ref_lat
            dlon = float(lon) - self.ref_lon
            
            x = dlon * 111319.9 * math.cos(math.radians(lat))
            y = dlat * 111319.9
            
            return float(x), float(y)
        except Exception as e:
            self.get_logger().error(f'❌ Simple coordinate conversion failed: {e}')
            return 0.0, 0.0
    
    def xy_to_latlon(self, x, y):
        """🔧 Safe inverse coordinate conversion"""
        try:
            if self.transformer and self.reference_utm:
                utm_x = float(x) + self.reference_utm[0]
                utm_y = float(y) + self.reference_utm[1]
                
                lon, lat = self.transformer.transform(utm_x, utm_y, direction='INVERSE')
                return float(lat), float(lon)
            else:
                return self.simple_xy_to_latlon(x, y)
        except Exception as e:
            self.get_logger().debug(f'Inverse coordinate conversion failed: {e}')
            return self.simple_xy_to_latlon(x, y)
    
    def simple_xy_to_latlon(self, x, y):
        """Safe simple inverse conversion"""
        try:
            dlat = float(y) / 111319.9
            dlon = float(x) / (111319.9 * math.cos(math.radians(self.ref_lat + dlat)))
            
            lat = self.ref_lat + dlat
            lon = self.ref_lon + dlon
            
            return float(lat), float(lon)
        except Exception as e:
            self.get_logger().error(f'❌ Simple inverse coordinate conversion failed: {e}')
            return self.ref_lat, self.ref_lon
    
    def start_path_following(self):
        """Start path following with progression"""
        try:
            if not self.smooth_path:
                self.get_logger().error('❌ No path available!')
                return
            
            self.navigation_active = True
            self.current_path_s = 0.0
            self.max_progress_s = 0.0
            self.start_time = time.time()
            self.total_distance = 0.0
            
            self.get_logger().info('🚀 Fixed Path Progression Pure Pursuit started!')
            
            if self.loop:
                asyncio.run_coroutine_threadsafe(
                    self.broadcast_message({'navigation_started': True}), 
                    self.loop
                )
        except Exception as e:
            self.get_logger().error(f'❌ Start path following failed: {e}')
    
    def pause_navigation(self):
        """Pause navigation"""
        self.navigation_active = False
        self.stop_robot()
        self.get_logger().info('⏸️ Navigation paused')
    
    def stop_navigation(self):
        """Stop navigation"""
        self.navigation_active = False
        self.smooth_path = None
        self.current_path_s = 0.0
        self.max_progress_s = 0.0
        self.stop_robot()
        self.get_logger().info('🛑 Navigation stopped')
    
    def stop_robot(self):
        """Send stop command"""
        try:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
        except Exception as e:
            self.get_logger().error(f'❌ Stop robot failed: {e}')
    
    def control_loop(self):
        """🔧 Fixed control loop"""
        if not self.navigation_active or not self.smooth_path:
            return
        
        try:
            # Update path progression
            self.update_path_progression()
            
            # Check completion
            progress_ratio = self.max_progress_s / self.path_length if self.path_length > 0 else 0
            
            if progress_ratio > self.PATH_COMPLETION_THRESHOLD:
                final_x = float(self.smooth_path['x'][-1])
                final_y = float(self.smooth_path['y'][-1])
                curr_x, curr_y = self.current_pos['x'], self.current_pos['y']
                distance_to_end = math.sqrt((curr_x - final_x)**2 + (curr_y - final_y)**2)
                
                if distance_to_end < 2.0:
                    self.navigation_active = False
                    self.stop_robot()
                    
                    if self.loop:
                        asyncio.run_coroutine_threadsafe(
                            self.broadcast_message({
                                'arrived': True,
                                'total_time': time.time() - self.start_time,
                                'distance_traveled': self.total_distance,
                                'average_speed': self.total_distance / max(1, time.time() - self.start_time),
                                'algorithm': 'Fixed Path Progression Pure Pursuit',
                                'completed_segments': len(self.completed_segments),
                                'total_segments': len(self.path_completion_status)
                            }), 
                            self.loop
                        )
                    
                    self.get_logger().info('🎉 Destination reached via fixed path progression!')
                    return
            
            # Calculate steering
            steering_angle = self.calculate_steering_with_progression()
            
            # Speed control
            speed = self.target_speed
            
            try:
                upcoming_curvature = self.get_upcoming_curvature()
                if upcoming_curvature > 0.1:
                    speed *= max(0.3, 1.0 - upcoming_curvature * 2.0)
                elif upcoming_curvature > 0.05:
                    speed *= max(0.6, 1.0 - upcoming_curvature)
            except:
                pass  # Skip curvature speed adjustment if error
            
            try:
                steering_deg = abs(math.degrees(steering_angle))
                if steering_deg > 20:
                    speed *= 0.5
                elif steering_deg > 10:
                    speed *= 0.7
            except:
                pass  # Skip steering speed adjustment if error
            
            try:
                if self.cross_track_error > 2.0:
                    speed *= 0.4
                elif self.cross_track_error > 1.0:
                    speed *= 0.7
            except:
                pass  # Skip cross-track speed adjustment if error
            
            speed = max(0.1, min(self.target_speed, speed))
            
            # Publish control
            twist = Twist()
            twist.linear.x = float(speed)
            twist.angular.z = float(steering_angle)
            self.cmd_vel_pub.publish(twist)
            
            # 🔧 Fixed debug logging - safe string formatting
            current_time = time.time()
            if not hasattr(self, '_last_log_time') or current_time - self._last_log_time > 1.0:
                self._last_log_time = current_time
                progress = self.calculate_path_progress()
                
                # Safe formatting with fallbacks
                steering_deg = math.degrees(steering_angle) if steering_angle is not None else 0.0
                speed_str = f"{speed:.2f}" if speed is not None else "0.00"
                progress_str = f"{progress:.1f}" if progress is not None else "0.0"
                max_progress_str = f"{self.max_progress_s:.1f}" if self.max_progress_s is not None else "0.0"
                path_length_str = f"{self.path_length:.1f}" if self.path_length is not None else "0.0"
                cross_track_str = f"{self.cross_track_error:.2f}" if self.cross_track_error is not None else "0.00"
                completed_segments = len(self.completed_segments) if self.completed_segments is not None else 0
                total_segments = len(self.path_completion_status) if self.path_completion_status is not None else 0
                heading_str = f"{self.final_heading_filtered:.1f}" if self.final_heading_filtered is not None else "0.0"
                heading_source_str = self.heading_source_filtered if self.heading_source_filtered is not None else "UNKNOWN"
                
                self.get_logger().info(
                    f'🗺️ Fixed Path Progression Pure Pursuit:\n'
                    f'  🚗 Steering: {steering_deg:+6.2f}° | Speed: {speed_str}m/s\n'
                    f'  📊 Progress: {progress_str}% (Max: {max_progress_str}m / {path_length_str}m)\n'
                    f'  📏 Cross-track: {cross_track_str}m\n'
                    f'  🗺️ Completed segments: {completed_segments}/{total_segments}\n'
                    f'  🧭 Heading: {heading_str}° ({heading_source_str})'
                )
        
        except Exception as e:
            self.get_logger().error(f'❌ Fixed control loop error: {e}')
            self.stop_robot()
    
    async def send_status(self):
        """🔧 Send status with safe error handling"""
        if not self.connections:
            return
        
        try:
            progress = self.calculate_path_progress()
            
            distance_to_dest = None
            if self.smooth_path:
                try:
                    final_x = float(self.smooth_path['x'][-1])
                    final_y = float(self.smooth_path['y'][-1])
                    curr_x, curr_y = self.current_pos['x'], self.current_pos['y']
                    distance_to_dest = math.sqrt((curr_x - final_x)**2 + (curr_y - final_y)**2)
                except:
                    distance_to_dest = None
            
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
                'filtered_heading': self.final_heading_filtered,
                'heading_source': self.heading_source_filtered,
                'algorithm': 'Fixed Path Progression Pure Pursuit (Bug-free)',
                'coordinate_system': 'UTM Zone 47N (Fixed)',
                'wheelbase': self.WHEELBASE,
                'lookahead_distance': self.LOOKAHEAD_DISTANCE,
                'path_curvature': getattr(self, 'path_curvature', 0.0),
                'current_path_s': self.current_path_s,
                'max_progress_s': self.max_progress_s,
                'path_length': self.path_length,
                
                # Path progression status
                'progression': {
                    'completed_segments': len(self.completed_segments),
                    'total_segments': len(self.path_completion_status),
                    'forward_guarantee': self.forward_progress_guarantee,
                    'max_progress': self.max_progress_s,
                    'last_progress_time': self.last_progress_time,
                    'segment_status': self.path_completion_status
                }
            }
            
            # Add lookahead point safely
            if self.lookahead_point:
                try:
                    lookahead_lat, lookahead_lon = self.xy_to_latlon(
                        self.lookahead_point[0], self.lookahead_point[1]
                    )
                    status['lookahead_lat'] = lookahead_lat
                    status['lookahead_lon'] = lookahead_lon
                    status['lookahead_distance'] = math.sqrt(
                        (self.lookahead_point[0] - self.current_pos['x'])**2 + 
                        (self.lookahead_point[1] - self.current_pos['y'])**2
                    )
                except:
                    pass  # Skip lookahead if error
            
            await self.broadcast_message(status)
            
        except Exception as e:
            self.get_logger().debug(f'Status send error: {e}')
    
    async def broadcast_message(self, message):
        """Broadcast message to all connected clients"""
        if not self.connections:
            return
        
        try:
            json_message = json.dumps(message, default=str)
            disconnected = set()
            
            for websocket in self.connections:
                try:
                    await websocket.send(json_message)
                except websockets.exceptions.ConnectionClosed:
                    disconnected.add(websocket)
            
            for websocket in disconnected:
                self.connections.discard(websocket)
                
        except Exception as e:
            self.get_logger().debug(f'Broadcast error: {e}')


# 🔧 Fixed Heading Filter
class FixedHeadingFilter:
    def __init__(self):
        self.history = deque(maxlen=8)
        self.last_stable_heading = 0.0  # 🔧 Initialize with default
        self.stability_threshold = 8.0
        self.outlier_threshold = 30.0
        
    def filter_heading(self, new_heading, speed):
        """🔧 Safe heading filtering with error handling"""
        try:
            if new_heading is None:
                return self.last_stable_heading, "NONE"
            
            new_heading = float(new_heading)  # Ensure float
            
            if self.is_outlier(new_heading):
                return self.last_stable_heading, "OUTLIER_FILTERED"
            
            self.history.append(new_heading)
            
            if len(self.history) < 3:
                self.last_stable_heading = new_heading
                return new_heading, "INITIALIZING"
            
            if speed > 0.5:
                filtered_heading = self.weighted_moving_average()
                source = "MOVING_AVG"
            elif speed > 0.1:
                filtered_heading = self.median_filter()
                source = "MEDIAN"
            else:
                if self.is_stable():
                    filtered_heading = self.last_stable_heading
                    source = "HOLD_STABLE"
                else:
                    filtered_heading = self.median_filter()
                    source = "MEDIAN_STATIONARY"
            
            if filtered_heading is not None:
                if self.last_stable_heading is None:
                    self.last_stable_heading = filtered_heading
                else:
                    angle_diff = self.angle_difference(filtered_heading, self.last_stable_heading)
                    if abs(angle_diff) < self.stability_threshold:
                        alpha = 0.3
                        self.last_stable_heading = self.smooth_angle_transition(
                            self.last_stable_heading, filtered_heading, alpha
                        )
                    else:
                        self.last_stable_heading = filtered_heading
            
            return self.last_stable_heading, source
            
        except Exception as e:
            # Safe fallback
            return self.last_stable_heading if self.last_stable_heading is not None else 0.0, "ERROR"
    
    def is_outlier(self, new_heading):
        """Safe outlier detection"""
        try:
            if len(self.history) < 2:
                return False
            
            recent_avg = self.circular_mean(list(self.history)[-3:])
            angle_diff = abs(self.angle_difference(new_heading, recent_avg))
            
            return angle_diff > self.outlier_threshold
        except:
            return False
    
    def weighted_moving_average(self):
        """Safe weighted moving average"""
        try:
            if len(self.history) < 2:
                return self.history[-1]
            
            weights = np.array([0.1, 0.2, 0.3, 0.4])[-len(self.history):]
            weights = weights / weights.sum()
            
            angles = np.array(list(self.history))
            return self.circular_weighted_mean(angles, weights)
        except:
            return self.history[-1] if self.history else 0.0
    
    def median_filter(self):
        """Safe median filter"""
        try:
            if len(self.history) < 3:
                return self.history[-1]
            
            angles = list(self.history)
            angles.sort(key=lambda x: self.angle_difference(x, angles[0]))
            
            return angles[len(angles) // 2]
        except:
            return self.history[-1] if self.history else 0.0
    
    def is_stable(self):
        """Safe stability check"""
        try:
            if len(self.history) < 3 or self.last_stable_heading is None:
                return False
            
            recent_headings = list(self.history)[-3:]
            max_deviation = max(abs(self.angle_difference(h, self.last_stable_heading)) 
                              for h in recent_headings)
            
            return max_deviation < self.stability_threshold
        except:
            return False
    
    def angle_difference(self, angle1, angle2):
        """Safe angle difference calculation"""
        try:
            diff = float(angle1) - float(angle2)
            while diff > 180:
                diff -= 360
            while diff < -180:
                diff += 360
            return diff
        except:
            return 0.0
    
    def circular_mean(self, angles):
        """Safe circular mean calculation"""
        try:
            if not angles:
                return 0.0
            
            x = np.mean([math.cos(math.radians(float(a))) for a in angles])
            y = np.mean([math.sin(math.radians(float(a))) for a in angles])
            
            return math.degrees(math.atan2(y, x)) % 360
        except:
            return 0.0
    
    def circular_weighted_mean(self, angles, weights):
        """Safe circular weighted mean calculation"""
        try:
            x = np.sum(weights * np.cos(np.radians(angles)))
            y = np.sum(weights * np.sin(np.radians(angles)))
            
            return math.degrees(math.atan2(y, x)) % 360
        except:
            return 0.0
    
    def smooth_angle_transition(self, old_angle, new_angle, alpha):
        """Safe smooth angle transition"""
        try:
            diff = self.angle_difference(new_angle, old_angle)
            smoothed_diff = alpha * diff
            result = (float(old_angle) + smoothed_diff) % 360
            
            return result
        except:
            return float(old_angle) if old_angle is not None else 0.0
            

def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    navigation = None
    
    try:
        navigation = FixedPathProgressionPurePursuitGolfCart()
        print('🚀 Fixed Path Progression Pure Pursuit Golf Cart Navigation running...')
        print('🗺️ Google Maps style path progression enabled!')
        print('🔧 Bug fixes applied - no more derivative or format errors!')
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



# ---------------------------------now-----------------------
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, Point
# from sensor_msgs.msg import NavSatFix
# from std_msgs.msg import Float32, String
# import asyncio
# import websockets
# import json
# import math
# import time
# import threading
# import numpy as np
# from scipy.interpolate import interp1d
# from scipy.spatial.distance import cdist
# from collections import deque
# import pyproj

# class RoadAlignedNavigationSystem(Node):
#     def __init__(self):
#         super().__init__('road_aligned_navigation_system')
        
#         self.get_logger().info('🛣️ Road-Aligned Navigation System Started!')
        
#         # Vehicle Parameters
#         self.WHEELBASE = 1.67
#         self.MAX_STEERING_ANGLE = 0.873
#         self.MIN_STEERING_ANGLE = -0.873
        
#         # Pure Pursuit Parameters
#         self.LOOKAHEAD_DISTANCE = 3.0
#         self.MIN_LOOKAHEAD = 2.0
#         self.MAX_LOOKAHEAD = 6.0
#         self.SPEED_TO_LOOKAHEAD_RATIO = 2.5
        
#         # 🛣️ NEW: Road Alignment Parameters
#         self.GPS_TRACK_BUFFER_SIZE = 50         # จำนวนจุด GPS ที่เก็บไว้
#         self.ROAD_ALIGNMENT_DISTANCE = 5.0      # ระยะที่ใช้ปรับ path ให้ตรงถนน
#         self.CROSS_TRACK_TOLERANCE = 3.0        # ระยะที่ยอมรับได้จาก path
#         self.PATH_CORRECTION_ENABLED = True     # เปิดใช้การแก้ไข path
#         self.MIN_GPS_ACCURACY = 2.0             # GPS accuracy threshold
#         self.ROAD_WIDTH_ESTIMATE = 6.0          # ความกว้างถนนประมาณ
        
#         # 🛣️ Real-time Path Correction Parameters  
#         self.CORRECTION_SMOOTHING = 0.3         # ความนุ่มนวลในการปรับ
#         self.CORRECTION_MAX_DISTANCE = 15.0     # ระยะไกลสุดที่จะปรับ
#         self.GPS_CONFIDENCE_THRESHOLD = 0.8     # ความเชื่อมั่น GPS
#         self.REALTIME_CORRECTION = True         # เปิด real-time correction
        
#         # Coordinate System
#         try:
#             self.transformer = pyproj.Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)
#         except Exception as e:
#             self.get_logger().warning(f'⚠️ PyProj failed: {e}')
#             self.transformer = None
        
#         self.ref_lat = 13.650748
#         self.ref_lon = 100.492985
#         self.reference_utm = None
        
#         # ROS2 Publishers & Subscribers
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.gnss_sub = self.create_subscription(NavSatFix, '/navigation/gnss', self.gnss_callback, 10)
#         self.xy_sub = self.create_subscription(Point, '/navigation/xy_position', self.xy_callback, 10)
#         self.heading_sub = self.create_subscription(Float32, '/navigation/heading', self.heading_callback, 10)
        
#         # Navigation State
#         self.navigation_active = False
#         self.raw_path_points = []
#         self.original_path = None      # 🛣️ Path เดิมจาก routing
#         self.corrected_path = None     # 🛣️ Path ที่ปรับแล้ว
#         self.current_path = None       # 🛣️ Path ที่ใช้งานจริง
#         self.path_length = 0.0
#         self.current_pos = {'x': 0.0, 'y': 0.0}
#         self.current_lat = 0.0
#         self.current_lon = 0.0
#         self.current_heading = 0.0
#         self.target_speed = 1.0
        
#         # 🛣️ GPS Track Management
#         self.gps_track_history = deque(maxlen=self.GPS_TRACK_BUFFER_SIZE)
#         self.last_gps_time = 0.0
#         self.gps_accuracy = 0.0
#         self.gps_confidence = 0.0
        
#         # 🛣️ Road Alignment State
#         self.road_aligned = False
#         self.alignment_confidence = 0.0
#         self.last_alignment_time = 0.0
#         self.path_corrections = deque(maxlen=20)
#         self.original_cross_track_error = 0.0
#         self.corrected_cross_track_error = 0.0
        
#         # Path Following State
#         self.current_path_s = 0.0
#         self.lookahead_point = None
#         self.path_curvature = 0.0
#         self.cross_track_error = 0.0
#         self.path_progress = 0.0
        
#         # Heading Filter
#         self.heading_filter = RoadAlignedHeadingFilter()
#         self.final_heading_filtered = 0.0
#         self.heading_source_filtered = "INITIALIZING"
        
#         # Control timing
#         self.control_timer = self.create_timer(0.1, self.control_loop)
        
#         # WebSocket Server
#         self.connections = set()
#         self.loop = None
        
#         # Performance tracking
#         self.start_time = 0
#         self.total_distance = 0.0
#         self.last_position = None
        
#         # Debug
#         self.debug_mode = True
#         self.debug_counter = 0
        
#         # Initialize coordinate system
#         self.init_coordinate_system()
        
#         # Start WebSocket server
#         self.websocket_thread = threading.Thread(target=self.run_websocket_server)
#         self.websocket_thread.daemon = True
#         self.websocket_thread.start()
        
#         self.get_logger().info('✅ Road-Aligned Navigation System ready!')
#         self.get_logger().info('🛣️ Real-time path correction enabled!')
#         self.get_logger().info('📍 GPS track alignment active!')
    
#     def init_coordinate_system(self):
#         """Initialize coordinate system"""
#         try:
#             if self.transformer:
#                 self.reference_utm = self.transformer.transform(self.ref_lon, self.ref_lat)
#                 self.get_logger().info(
#                     f'🛣️ Road-aligned coordinate system initialized:\n'
#                     f'  📍 Reference: ({self.ref_lat:.6f}, {self.ref_lon:.6f})\n'
#                     f'  📍 UTM: ({self.reference_utm[0]:.2f}, {self.reference_utm[1]:.2f})'
#                 )
#             else:
#                 self.reference_utm = None
#         except Exception as e:
#             self.get_logger().error(f'❌ Coordinate system init failed: {e}')
#             self.reference_utm = None
    
#     def run_websocket_server(self):
#         """Run WebSocket server"""
#         try:
#             self.loop = asyncio.new_event_loop()
#             asyncio.set_event_loop(self.loop)
            
#             start_server = websockets.serve(self.handle_websocket, 'localhost', 5001)
#             self.loop.run_until_complete(start_server)
#             self.get_logger().info('🌐 Road-Aligned WebSocket server started on port 5001')
#             self.loop.run_forever()
            
#         except Exception as e:
#             self.get_logger().error(f'❌ WebSocket server error: {e}')
    
#     async def handle_websocket(self, websocket, path):
#         """Handle WebSocket connections"""
#         try:
#             self.connections.add(websocket)
#             self.get_logger().info('🔗 New client connected to Road-Aligned system')
            
#             async for message in websocket:
#                 try:
#                     data = json.loads(message)
#                     await self.handle_message(data)
#                 except json.JSONDecodeError:
#                     self.get_logger().error('❌ Invalid JSON received')
                    
#         except websockets.exceptions.ConnectionClosed:
#             pass
#         finally:
#             self.connections.discard(websocket)
#             self.get_logger().info('🔌 Client disconnected')
    
#     async def handle_message(self, data):
#         """Handle incoming WebSocket messages"""
#         if data.get('path'):
#             await self.process_path_data(data)
#         elif data.get('navigation_command'):
#             cmd = data['navigation_command']
#             if cmd == 'start':
#                 self.start_path_following()
#             elif cmd == 'stop':
#                 self.stop_navigation()
#             elif cmd == 'pause':
#                 self.pause_navigation()
    
#     async def process_path_data(self, data):
#         """🛣️ Process path data with road alignment"""
#         try:
#             path_points = data.get('path', [])
#             if len(path_points) < 2:
#                 self.get_logger().warn('⚠️ Path too short')
#                 return
            
#             self.get_logger().info(f'🛣️ Processing path with road alignment: {len(path_points)} points')
            
#             # Convert lat/lon to local coordinates
#             self.raw_path_points = []
#             for point in path_points:
#                 x, y = self.latlon_to_xy(point['lat'], point['lon'])
#                 self.raw_path_points.append((x, y))
            
#             # 🛣️ Create original path (from routing)
#             self.create_original_path()
            
#             # 🛣️ Initialize corrected path (same as original initially)
#             self.corrected_path = dict(self.original_path)
#             self.current_path = self.corrected_path
            
#             # Reset navigation state
#             self.current_path_s = 0.0
#             self.path_progress = 0.0
            
#             self.get_logger().info(
#                 f'🛣️ Road-aligned path created:\n'
#                 f'  📊 Original: {len(self.raw_path_points)} waypoints\n'
#                 f'  🌟 Path points: {len(self.current_path["x"]) if self.current_path else 0}\n'
#                 f'  📏 Length: {self.path_length:.1f}m\n'
#                 f'  🛣️ Road alignment: Ready for GPS tracking'
#             )
            
#             # Send confirmation to frontend
#             if self.connections:
#                 response = {
#                     'route_received': True,
#                     'smooth_points': len(self.current_path["x"]) if self.current_path else 0,
#                     'original_points': len(self.raw_path_points),
#                     'total_distance': self.path_length,
#                     'algorithm': 'Road-Aligned Navigation (GPS Track Corrected)',
#                     'coordinate_system': 'UTM Zone 47N (Road-Aligned)',
#                     'wheelbase': self.WHEELBASE,
#                     'max_steering': math.degrees(self.MAX_STEERING_ANGLE),
#                     'road_alignment_enabled': True,
#                     'gps_track_correction': True
#                 }
#                 await self.broadcast_message(response)
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Road-aligned path creation error: {e}')
    
#     def create_original_path(self):
#         """🛣️ Create original path from routing"""
#         try:
#             if len(self.raw_path_points) < 2:
#                 return
            
#             # Extract coordinates
#             raw_x = [p[0] for p in self.raw_path_points]
#             raw_y = [p[1] for p in self.raw_path_points]
            
#             # Calculate cumulative distances
#             raw_distances = [0.0]
#             for i in range(1, len(self.raw_path_points)):
#                 dx = raw_x[i] - raw_x[i-1]
#                 dy = raw_y[i] - raw_y[i-1]
#                 dist = math.sqrt(dx*dx + dy*dy)
#                 raw_distances.append(raw_distances[-1] + dist)
            
#             total_length = raw_distances[-1]
            
#             # Use linear interpolation for safety
#             spline_x = interp1d(raw_distances, raw_x, kind='linear', fill_value='extrapolate')
#             spline_y = interp1d(raw_distances, raw_y, kind='linear', fill_value='extrapolate')
            
#             # Generate path points
#             path_resolution = 0.5  # 50cm resolution for road alignment
#             num_points = max(20, int(total_length / path_resolution))
#             s_values = np.linspace(0, total_length, num_points)
            
#             smooth_x = spline_x(s_values)
#             smooth_y = spline_y(s_values)
            
#             # Store original path
#             self.original_path = {
#                 'x': smooth_x,
#                 'y': smooth_y,
#                 's': s_values,
#                 'raw_x': raw_x,
#                 'raw_y': raw_y,
#                 'interpolation_method': 'linear_road_aligned'
#             }
            
#             self.path_length = total_length
            
#             self.get_logger().info(f"🛣️ Original path created: {len(smooth_x)} points, {total_length:.2f}m")
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Original path creation failed: {e}')
    
#     def update_gps_track(self, lat, lon, accuracy=None):
#         """🛣️ Update GPS track history for road alignment"""
#         try:
#             current_time = time.time()
            
#             # Convert to local coordinates
#             x, y = self.latlon_to_xy(lat, lon)
            
#             # Add to GPS track history
#             gps_point = {
#                 'x': x, 'y': y,
#                 'lat': lat, 'lon': lon,
#                 'timestamp': current_time,
#                 'accuracy': accuracy if accuracy else 3.0  # Default 3m accuracy
#             }
            
#             self.gps_track_history.append(gps_point)
#             self.last_gps_time = current_time
            
#             # Update GPS accuracy metrics
#             if accuracy:
#                 self.gps_accuracy = accuracy
#                 self.gps_confidence = max(0.0, min(1.0, (5.0 - accuracy) / 5.0))  # Better accuracy = higher confidence
            
#             # 🛣️ Trigger path correction if conditions are met
#             if self.should_correct_path():
#                 self.perform_realtime_path_correction()
            
#         except Exception as e:
#             self.get_logger().error(f'❌ GPS track update failed: {e}')
    
#     def should_correct_path(self):
#         """🛣️ Determine if path correction should be performed"""
#         try:
#             if not self.PATH_CORRECTION_ENABLED or not self.original_path:
#                 return False
            
#             # Need sufficient GPS points
#             if len(self.gps_track_history) < 5:
#                 return False
            
#             # Check GPS accuracy
#             if self.gps_accuracy > self.MIN_GPS_ACCURACY:
#                 return False
            
#             # Check if we have significant cross-track error
#             if abs(self.cross_track_error) < 1.0:  # Path is already good
#                 return False
            
#             # Don't correct too frequently
#             if time.time() - self.last_alignment_time < 2.0:
#                 return False
            
#             return True
            
#         except Exception as e:
#             self.get_logger().debug(f'Should correct path check failed: {e}')
#             return False
    
#     def perform_realtime_path_correction(self):
#         """🛣️ Perform real-time path correction based on GPS track"""
#         try:
#             if not self.original_path or len(self.gps_track_history) < 5:
#                 return
            
#             self.get_logger().info('🛣️ Performing real-time path correction...')
            
#             # Get recent GPS track points
#             recent_gps = list(self.gps_track_history)[-10:]  # Last 10 points
            
#             # Create corrected path
#             corrected_x = []
#             corrected_y = []
#             corrected_s = []
            
#             original_x = self.original_path['x']
#             original_y = self.original_path['y']
#             original_s = self.original_path['s']
            
#             for i, (ox, oy, os) in enumerate(zip(original_x, original_y, original_s)):
#                 # Find the best GPS point to align with
#                 correction_x, correction_y = self.find_best_gps_alignment(ox, oy, recent_gps)
                
#                 # Apply weighted correction
#                 if correction_x is not None and correction_y is not None:
#                     # Calculate correction vector
#                     correction_dx = correction_x - ox
#                     correction_dy = correction_y - oy
#                     correction_distance = math.sqrt(correction_dx**2 + correction_dy**2)
                    
#                     # Limit correction distance
#                     if correction_distance > self.CORRECTION_MAX_DISTANCE:
#                         scale = self.CORRECTION_MAX_DISTANCE / correction_distance
#                         correction_dx *= scale
#                         correction_dy *= scale
                    
#                     # Apply smoothed correction
#                     corrected_x.append(ox + correction_dx * self.CORRECTION_SMOOTHING)
#                     corrected_y.append(oy + correction_dy * self.CORRECTION_SMOOTHING)
#                 else:
#                     # No correction available, use original
#                     corrected_x.append(ox)
#                     corrected_y.append(oy)
                
#                 corrected_s.append(os)
            
#             # Update corrected path
#             self.corrected_path = {
#                 'x': np.array(corrected_x),
#                 'y': np.array(corrected_y),
#                 's': np.array(corrected_s),
#                 'correction_applied': True,
#                 'correction_time': time.time()
#             }
            
#             # Switch to using corrected path
#             self.current_path = self.corrected_path
#             self.last_alignment_time = time.time()
#             self.road_aligned = True
            
#             # Calculate alignment confidence
#             self.alignment_confidence = self.calculate_alignment_confidence()
            
#             self.get_logger().info(
#                 f'✅ Path correction applied: '
#                 f'Confidence: {self.alignment_confidence:.2f}, '
#                 f'Points corrected: {len(corrected_x)}'
#             )
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Real-time path correction failed: {e}')
    
#     def find_best_gps_alignment(self, path_x, path_y, gps_points):
#         """🛣️ Find best GPS point to align path point with"""
#         try:
#             if not gps_points:
#                 return None, None
            
#             best_distance = float('inf')
#             best_x, best_y = None, None
            
#             for gps_point in gps_points:
#                 gps_x, gps_y = gps_point['x'], gps_point['y']
#                 distance = math.sqrt((path_x - gps_x)**2 + (path_y - gps_y)**2)
                
#                 # Only consider GPS points within reasonable range
#                 if distance < self.ROAD_ALIGNMENT_DISTANCE and distance < best_distance:
#                     best_distance = distance
#                     best_x, best_y = gps_x, gps_y
            
#             return best_x, best_y
            
#         except Exception as e:
#             self.get_logger().debug(f'GPS alignment search failed: {e}')
#             return None, None
    
#     def calculate_alignment_confidence(self):
#         """🛣️ Calculate confidence in road alignment"""
#         try:
#             if not self.gps_track_history or not self.current_path:
#                 return 0.0
            
#             # Factors affecting confidence:
#             # 1. GPS accuracy
#             # 2. Number of GPS points
#             # 3. Cross-track error improvement
#             # 4. Path consistency
            
#             gps_factor = self.gps_confidence
#             points_factor = min(1.0, len(self.gps_track_history) / 20.0)
            
#             # Cross-track improvement
#             if hasattr(self, 'original_cross_track_error') and self.original_cross_track_error > 0:
#                 improvement_factor = max(0.0, 1.0 - (self.cross_track_error / self.original_cross_track_error))
#             else:
#                 improvement_factor = 0.5
            
#             # Combined confidence
#             confidence = (gps_factor * 0.4 + points_factor * 0.3 + improvement_factor * 0.3)
            
#             return max(0.0, min(1.0, confidence))
            
#         except Exception as e:
#             self.get_logger().debug(f'Alignment confidence calculation failed: {e}')
#             return 0.0
    
#     def find_closest_point_on_road_aligned_path(self):
#         """🛣️ Find closest point on road-aligned path - ENHANCED"""
#         try:
#             if not self.current_path:
#                 return 0.0, (0.0, 0.0)
            
#             curr_x, curr_y = self.current_pos['x'], self.current_pos['y']
            
#             min_distance = float('inf')
#             closest_idx = 0
#             closest_x, closest_y = 0.0, 0.0
            
#             # หาจุดที่ใกล้ที่สุดบน path
#             for i, (px, py) in enumerate(zip(self.current_path['x'], self.current_path['y'])):
#                 try:
#                     px_float = float(px)
#                     py_float = float(py)
#                     dist = math.sqrt((curr_x - px_float)**2 + (curr_y - py_float)**2)
                    
#                     if dist < min_distance:
#                         min_distance = dist
#                         closest_idx = i
#                         closest_x, closest_y = px_float, py_float
#                 except Exception as e:
#                     continue
            
#             # Get s-coordinate
#             closest_s = float(self.current_path['s'][closest_idx]) if 's' in self.current_path else 0.0
#             self.current_path_s = closest_s
            
#             # 🔧 Calculate accurate cross-track error with direction
#             # หาเวกเตอร์จากจุดใกล้ที่สุดไปยังรถ
#             to_vehicle_x = curr_x - closest_x
#             to_vehicle_y = curr_y - closest_y
            
#             # หาทิศทางของ path ณ จุดใกล้ที่สุด
#             if closest_idx < len(self.current_path['x']) - 1:
#                 # ใช้จุดถัดไป
#                 next_x = float(self.current_path['x'][closest_idx + 1])
#                 next_y = float(self.current_path['y'][closest_idx + 1])
#                 path_dx = next_x - closest_x
#                 path_dy = next_y - closest_y
#             elif closest_idx > 0:
#                 # ใช้จุดก่อนหน้า
#                 prev_x = float(self.current_path['x'][closest_idx - 1])
#                 prev_y = float(self.current_path['y'][closest_idx - 1])
#                 path_dx = closest_x - prev_x
#                 path_dy = closest_y - prev_y
#             else:
#                 # ไม่สามารถคำนวณทิศทางได้
#                 path_dx, path_dy = 1.0, 0.0
            
#             # Normalize path direction
#             path_length = math.sqrt(path_dx**2 + path_dy**2)
#             if path_length > 0:
#                 path_dx /= path_length
#                 path_dy /= path_length
            
#             # คำนวณ cross product เพื่อหาทิศทางของ cross-track error
#             # Positive = ทางขวาของ path, Negative = ทางซ้ายของ path
#             cross_product = to_vehicle_x * path_dy - to_vehicle_y * path_dx
            
#             # Set cross-track error with proper sign
#             self.cross_track_error = cross_product
            
#             # Store original cross-track error for comparison
#             if not hasattr(self, 'original_cross_track_error') or self.original_cross_track_error == 0:
#                 self.original_cross_track_error = abs(cross_product)
            
#             # Enhanced logging
#             self.get_logger().debug(
#                 f"📍 Closest Point Analysis:\n"
#                 f"  🚗 Vehicle: ({curr_x:.2f}, {curr_y:.2f})\n"
#                 f"  📌 Closest: ({closest_x:.2f}, {closest_y:.2f}) [idx={closest_idx}]\n"
#                 f"  📏 Distance: {min_distance:.2f}m\n"
#                 f"  📐 Cross-track: {cross_product:+.2f}m\n"
#                 f"  📍 Path s: {closest_s:.2f}m"
#             )
            
#             return closest_s, (closest_x, closest_y)
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Enhanced closest point calculation failed: {e}')
#             return 0.0, (self.current_pos['x'], self.current_pos['y'])
    
#     def find_road_aligned_lookahead_point(self):
#         """🛣️ Find lookahead point on road-aligned path - ENHANCED"""
#         try:
#             if not self.current_path:
#                 self.get_logger().warning("❌ No current path available")
#                 return None
            
#             # Get current position on path
#             current_s, closest_point = self.find_closest_point_on_road_aligned_path()
            
#             # Calculate dynamic lookahead distance
#             current_speed = max(0.1, self.target_speed)
#             lookahead_dist = max(self.MIN_LOOKAHEAD, 
#                                min(self.MAX_LOOKAHEAD, 
#                                    current_speed * self.SPEED_TO_LOOKAHEAD_RATIO))
            
#             # 🔧 Ensure minimum effective lookahead
#             lookahead_dist = max(2.0, lookahead_dist)
            
#             # Find lookahead point
#             target_s = current_s + lookahead_dist
            
#             # 🔧 ตรวจสอบว่าไม่เกินจุดสิ้นสุดของ path
#             if target_s >= self.path_length:
#                 target_s = self.path_length - 0.1  # เหลือ 10cm จากจุดสิ้นสุด
                
#             target_s = max(current_s, target_s)
            
#             # Get coordinates with enhanced interpolation
#             try:
#                 s_array = self.current_path['s']
#                 x_array = self.current_path['x']
#                 y_array = self.current_path['y']
                
#                 # Find index for interpolation
#                 idx = np.searchsorted(s_array, target_s)
                
#                 if idx >= len(x_array):
#                     # ใช้จุดสุดท้าย
#                     lookahead_x = float(x_array[-1])
#                     lookahead_y = float(y_array[-1])
#                     self.get_logger().info(f"🎯 Using final point: ({lookahead_x:.2f}, {lookahead_y:.2f})")
#                 elif idx == 0:
#                     # ใช้จุดแรก
#                     lookahead_x = float(x_array[0])
#                     lookahead_y = float(y_array[0])
#                     self.get_logger().info(f"🎯 Using first point: ({lookahead_x:.2f}, {lookahead_y:.2f})")
#                 else:
#                     # Linear interpolation
#                     s_prev = s_array[idx-1]
#                     s_next = s_array[idx]
#                     t = (target_s - s_prev) / (s_next - s_prev) if s_next != s_prev else 0.0
                    
#                     lookahead_x = float(x_array[idx-1] + t * (x_array[idx] - x_array[idx-1]))
#                     lookahead_y = float(y_array[idx-1] + t * (y_array[idx] - y_array[idx-1]))
                    
#                     self.get_logger().debug(
#                         f"🎯 Interpolated lookahead: s={target_s:.2f}, "
#                         f"idx={idx}, t={t:.3f}, point=({lookahead_x:.2f}, {lookahead_y:.2f})"
#                     )
                
#                 # 🔧 Validate lookahead point
#                 curr_x, curr_y = self.current_pos['x'], self.current_pos['y']
#                 actual_distance = math.sqrt((lookahead_x - curr_x)**2 + (lookahead_y - curr_y)**2)
                
#                 if actual_distance < 0.5:  # ถ้าใกล้เกินไป
#                     # หาจุดที่ไกลขึ้น
#                     target_s = current_s + max(2.0, lookahead_dist * 1.5)
#                     target_s = min(target_s, self.path_length - 0.1)
                    
#                     idx = np.searchsorted(s_array, target_s)
#                     if idx >= len(x_array):
#                         lookahead_x = float(x_array[-1])
#                         lookahead_y = float(y_array[-1])
#                     else:
#                         # Re-interpolate with new target
#                         if idx > 0:
#                             s_prev = s_array[idx-1]
#                             s_next = s_array[idx] if idx < len(s_array) else s_array[-1]
#                             t = (target_s - s_prev) / (s_next - s_prev) if s_next != s_prev else 0.0
#                             lookahead_x = float(x_array[idx-1] + t * (x_array[idx] - x_array[idx-1]))
#                             lookahead_y = float(y_array[idx-1] + t * (y_array[idx] - y_array[idx-1]))
#                         else:
#                             lookahead_x = float(x_array[idx])
#                             lookahead_y = float(y_array[idx])
                    
#                     actual_distance = math.sqrt((lookahead_x - curr_x)**2 + (lookahead_y - curr_y)**2)
#                     self.get_logger().info(f"🔧 Adjusted lookahead distance: {actual_distance:.2f}m")
                
#                 self.get_logger().info(
#                     f"🎯 Lookahead Point: ({lookahead_x:.2f}, {lookahead_y:.2f}), "
#                     f"Distance: {actual_distance:.2f}m, Target_s: {target_s:.2f}"
#                 )
                
#                 return (lookahead_x, lookahead_y)
                
#             except Exception as e:
#                 self.get_logger().error(f'❌ Lookahead coordinate calculation failed: {e}')
#                 return None
        
#         except Exception as e:
#             self.get_logger().error(f'❌ Lookahead calculation failed: {e}')
#             return None
    
#     def calculate_road_aligned_steering(self):
#         """🛣️ Simple steering calculation - Point directly to lookahead"""
#         try:
#             lookahead_point = self.find_road_aligned_lookahead_point()
            
#             if not lookahead_point:
#                 return 0.0
            
#             self.lookahead_point = lookahead_point
            
#             # Current position
#             curr_x, curr_y = self.current_pos['x'], self.current_pos['y']
#             target_x, target_y = lookahead_point
            
#             # Vector to lookahead point
#             dx = target_x - curr_x
#             dy = target_y - curr_y
#             lookahead_distance = math.sqrt(dx*dx + dy*dy)
            
#             if lookahead_distance < 0.1:
#                 return 0.0
            
#             # 🔧 SIMPLE APPROACH: คำนวณมุมไปยัง lookahead point โดยตรง
#             # หามุมที่ต้องเลี้ยวเพื่อไปหา lookahead point
#             target_angle_rad = math.atan2(dy, dx)  # มุมในระบบ math coordinate
            
#             # แปลงเป็น degrees เพื่อเปรียบเทียบกับ heading
#             target_angle_deg = math.degrees(target_angle_rad)
#             if target_angle_deg < 0:
#                 target_angle_deg += 360
                
#             # Current heading (GPS format: 0°=North, 90°=East)
#             current_heading = self.final_heading_filtered if self.final_heading_filtered is not None else self.current_heading
            
#             # คำนวณความต่างของมุม
#             angle_diff = target_angle_deg - current_heading
            
#             # Normalize angle difference ให้อยู่ระหว่าง -180 ถึง 180
#             while angle_diff > 180:
#                 angle_diff -= 360
#             while angle_diff < -180:
#                 angle_diff += 360
            
#             # 🔧 SIMPLE STEERING: แปลงความต่างของมุมเป็น steering angle
#             # ใช้ gain ที่เหมาะสมเพื่อไม่ให้เลี้ยวแรงเกินไป
#             steering_gain = 0.02  # ปรับได้ตามต้องการ
#             max_steering_deg = math.degrees(self.MAX_STEERING_ANGLE)
            
#             steering_angle_deg = angle_diff * steering_gain
            
#             # จำกัดมุมเลี้ยว
#             steering_angle_deg = max(-max_steering_deg, min(max_steering_deg, steering_angle_deg))
            
#             # แปลงกลับเป็น radians
#             steering_angle = math.radians(steering_angle_deg)
            
#             # 🔧 Enhanced Debug logging
#             direction = "ตรง"
#             if abs(steering_angle_deg) > 3:  # มากกว่า 3 องศา
#                 direction = "เลี้ยวซ้าย" if steering_angle_deg > 0 else "เลี้ยวขวา"
            
#             self.get_logger().info(
#                 f'🎯 SIMPLE STEERING:\n'
#                 f'  📍 Vehicle: ({curr_x:.1f}, {curr_y:.1f})\n'
#                 f'  🎯 Target: ({target_x:.1f}, {target_y:.1f})\n'
#                 f'  📏 Distance: {lookahead_distance:.1f}m\n'
#                 f'  🧭 Current Heading: {current_heading:.1f}°\n'
#                 f'  🎯 Target Angle: {target_angle_deg:.1f}°\n'
#                 f'  📐 Angle Diff: {angle_diff:+.1f}°\n'
#                 f'  🎛️ Steering: {steering_angle_deg:+.1f}°\n'
#                 f'  🚗 Command: {direction}'
#             )
            
#             return float(steering_angle)
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Simple steering calculation failed: {e}')
#             return 0.0
    
#     def calculate_path_progress(self):
#         """Calculate progress with road alignment"""
#         try:
#             if not self.current_path or self.path_length == 0:
#                 return 0.0
            
#             progress = (self.current_path_s / self.path_length) * 100
#             self.path_progress = progress / 100.0
#             return float(progress)
            
#         except Exception as e:
#             self.get_logger().debug(f'Progress calculation failed: {e}')
#             return 0.0
    
#     # ROS callbacks (with road alignment)
#     def gnss_callback(self, msg):
#         """🛣️ Handle GNSS position updates with road alignment"""
#         try:
#             lat = float(msg.latitude)
#             lon = float(msg.longitude)
            
#             self.current_lat = lat
#             self.current_lon = lon
            
#             x, y = self.latlon_to_xy(lat, lon)
#             self.current_pos = {'x': float(x), 'y': float(y)}
            
#             # 🛣️ Update GPS track for road alignment
#             # Get accuracy from GNSS message if available
#             accuracy = getattr(msg, 'position_covariance', [3.0])[0] if hasattr(msg, 'position_covariance') else 3.0
#             self.update_gps_track(lat, lon, accuracy)
            
#             # Update distance traveled
#             if self.last_position:
#                 dx = x - self.last_position['x']
#                 dy = y - self.last_position['y']
#                 distance = math.sqrt(dx*dx + dy*dy)
#                 if distance < 10.0:
#                     self.total_distance += distance
            
#             self.last_position = {'x': float(x), 'y': float(y)}
            
#             # Send status to frontend
#             if self.loop:
#                 asyncio.run_coroutine_threadsafe(self.send_status(), self.loop)
                
#         except Exception as e:
#             self.get_logger().error(f'❌ Road-aligned GNSS callback failed: {e}')
    
#     def xy_callback(self, msg):
#         """Handle XY position updates"""
#         try:
#             self.current_pos = {'x': float(msg.x), 'y': float(msg.y)}
#         except Exception as e:
#             self.get_logger().debug(f'XY callback failed: {e}')
    
#     def heading_callback(self, msg):
#         """Handle heading updates with filtering"""
#         try:
#             raw_heading = float(msg.data)
#             self.current_heading = raw_heading
            
#             current_speed = self.target_speed if self.navigation_active else 0.0
#             filtered_result = self.heading_filter.filter_heading(raw_heading, current_speed)
            
#             if filtered_result and len(filtered_result) >= 2:
#                 self.final_heading_filtered = filtered_result[0] if filtered_result[0] is not None else raw_heading
#                 self.heading_source_filtered = filtered_result[1] if filtered_result[1] is not None else "RAW"
#             else:
#                 self.final_heading_filtered = raw_heading
#                 self.heading_source_filtered = "RAW"
                
#         except Exception as e:
#             self.get_logger().error(f'❌ Heading callback failed: {e}')
#             self.final_heading_filtered = 0.0
#             self.heading_source_filtered = "ERROR"
    
#     def latlon_to_xy(self, lat, lon):
#         """Coordinate conversion"""
#         try:
#             if self.transformer and self.reference_utm:
#                 utm_x, utm_y = self.transformer.transform(lon, lat)
#                 local_x = utm_x - self.reference_utm[0]
#                 local_y = utm_y - self.reference_utm[1]
#                 return local_x, local_y
#             else:
#                 return self.simple_latlon_to_xy(lat, lon)
#         except Exception as e:
#             return self.simple_latlon_to_xy(lat, lon)
    
#     def simple_latlon_to_xy(self, lat, lon):
#         """Simple conversion"""
#         try:
#             dlat = float(lat) - self.ref_lat
#             dlon = float(lon) - self.ref_lon
            
#             x = dlon * 111319.9 * math.cos(math.radians(lat))
#             y = dlat * 111319.9
            
#             return float(x), float(y)
#         except Exception as e:
#             return 0.0, 0.0
    
#     def xy_to_latlon(self, x, y):
#         """Inverse coordinate conversion"""
#         try:
#             if self.transformer and self.reference_utm:
#                 utm_x = float(x) + self.reference_utm[0]
#                 utm_y = float(y) + self.reference_utm[1]
                
#                 lon, lat = self.transformer.transform(utm_x, utm_y, direction='INVERSE')
#                 return float(lat), float(lon)
#             else:
#                 return self.simple_xy_to_latlon(x, y)
#         except Exception as e:
#             return self.simple_xy_to_latlon(x, y)
    
#     def simple_xy_to_latlon(self, x, y):
#         """Simple inverse conversion"""
#         try:
#             dlat = float(y) / 111319.9
#             dlon = float(x) / (111319.9 * math.cos(math.radians(self.ref_lat + dlat)))
            
#             lat = self.ref_lat + dlat
#             lon = self.ref_lon + dlon
            
#             return float(lat), float(lon)
#         except Exception as e:
#             return self.ref_lat, self.ref_lon
    
#     def start_path_following(self):
#         """Start road-aligned path following"""
#         try:
#             if not self.current_path:
#                 self.get_logger().error('❌ No road-aligned path available!')
#                 return
            
#             self.navigation_active = True
#             self.current_path_s = 0.0
#             self.start_time = time.time()
#             self.total_distance = 0.0
            
#             self.get_logger().info('🚀 Road-Aligned Navigation started!')
            
#             if self.loop:
#                 asyncio.run_coroutine_threadsafe(
#                     self.broadcast_message({'navigation_started': True}), 
#                     self.loop
#                 )
#         except Exception as e:
#             self.get_logger().error(f'❌ Start road-aligned path following failed: {e}')
    
#     def pause_navigation(self):
#         """Pause navigation"""
#         self.navigation_active = False
#         self.stop_robot()
#         self.get_logger().info('⏸️ Road-aligned navigation paused')
    
#     def stop_navigation(self):
#         """Stop navigation"""
#         self.navigation_active = False
#         self.current_path = None
#         self.current_path_s = 0.0
#         self.stop_robot()
#         self.get_logger().info('🛑 Road-aligned navigation stopped')
    
#     def stop_robot(self):
#         """Send stop command"""
#         try:
#             twist = Twist()
#             twist.linear.x = 0.0
#             twist.angular.z = 0.0
#             self.cmd_vel_pub.publish(twist)
#         except Exception as e:
#             self.get_logger().error(f'❌ Stop robot failed: {e}')
    
#     def control_loop(self):
#         """🛣️ Road-aligned control loop"""
#         if not self.navigation_active or not self.current_path:
#             return
        
#         try:
#             # Check completion
#             progress_ratio = self.current_path_s / self.path_length if self.path_length > 0 else 0
            
#             if progress_ratio > 0.95:
#                 final_x = float(self.current_path['x'][-1])
#                 final_y = float(self.current_path['y'][-1])
#                 curr_x, curr_y = self.current_pos['x'], self.current_pos['y']
#                 distance_to_end = math.sqrt((curr_x - final_x)**2 + (curr_y - final_y)**2)
                
#                 if distance_to_end < 2.0:
#                     self.navigation_active = False
#                     self.stop_robot()
                    
#                     if self.loop:
#                         asyncio.run_coroutine_threadsafe(
#                             self.broadcast_message({
#                                 'arrived': True,
#                                 'total_time': time.time() - self.start_time,
#                                 'distance_traveled': self.total_distance,
#                                 'average_speed': self.total_distance / max(1, time.time() - self.start_time),
#                                 'algorithm': 'Road-Aligned Navigation',
#                                 'road_aligned': self.road_aligned,
#                                 'alignment_confidence': self.alignment_confidence
#                             }), 
#                             self.loop
#                         )
                    
#                     self.get_logger().info('🎉 Destination reached via road-aligned path!')
#                     return
            
#             # Calculate steering
#             steering_angle = self.calculate_road_aligned_steering()
            
#             # Speed control
#             speed = self.target_speed
            
#             # Reduce speed based on cross-track error
#             if self.cross_track_error > self.CROSS_TRACK_TOLERANCE:
#                 speed *= 0.5  # Slow down when far from road
#             elif self.cross_track_error > 1.0:
#                 speed *= 0.7
            
#             # Reduce speed based on steering angle
#             try:
#                 steering_deg = abs(math.degrees(steering_angle))
#                 if steering_deg > 20:
#                     speed *= 0.6
#                 elif steering_deg > 10:
#                     speed *= 0.8
#             except:
#                 pass
            
#             speed = max(0.1, min(self.target_speed, speed))
            
#             # Publish control
#             twist = Twist()
#             twist.linear.x = float(speed)
#             twist.angular.z = float(steering_angle)
#             self.cmd_vel_pub.publish(twist)
            
#             # Debug logging
#             current_time = time.time()
#             if not hasattr(self, '_last_log_time') or current_time - self._last_log_time > 1.0:
#                 self._last_log_time = current_time
#                 progress = self.calculate_path_progress()
                
#                 steering_deg = math.degrees(steering_angle) if steering_angle is not None else 0.0
#                 speed_str = f"{speed:.2f}" if speed is not None else "0.00"
#                 progress_str = f"{progress:.1f}" if progress is not None else "0.0"
#                 cross_track_str = f"{self.cross_track_error:.2f}" if self.cross_track_error is not None else "0.00"
#                 alignment_str = f"{self.alignment_confidence:.2f}" if hasattr(self, 'alignment_confidence') else "0.00"
                
#                 self.get_logger().info(
#                     f'🛣️ Road-Aligned Navigation:\n'
#                     f'  🚗 Steering: {steering_deg:+6.2f}° | Speed: {speed_str}m/s\n'
#                     f'  📊 Progress: {progress_str}% | Cross-track: {cross_track_str}m\n'
#                     f'  🛣️ Road aligned: {self.road_aligned} | Confidence: {alignment_str}\n'
#                     f'  📍 GPS points: {len(self.gps_track_history)} | Accuracy: {self.gps_accuracy:.1f}m'
#                 )
        
#         except Exception as e:
#             self.get_logger().error(f'❌ Road-aligned control loop error: {e}')
#             self.stop_robot()
    
#     async def send_status(self):
#         """🛣️ Send status with road alignment info"""
#         if not self.connections:
#             return
        
#         try:
#             progress = self.calculate_path_progress()
            
#             distance_to_dest = None
#             if self.current_path:
#                 try:
#                     final_x = float(self.current_path['x'][-1])
#                     final_y = float(self.current_path['y'][-1])
#                     curr_x, curr_y = self.current_pos['x'], self.current_pos['y']
#                     distance_to_dest = math.sqrt((curr_x - final_x)**2 + (curr_y - final_y)**2)
#                 except:
#                     distance_to_dest = None
            
#             status = {
#                 'latitude': self.current_lat,
#                 'longitude': self.current_lon,
#                 'current_speed': self.target_speed if self.navigation_active else 0.0,
#                 'navigation_active': self.navigation_active,
#                 'route_progress': progress,
#                 'distance_to_destination': distance_to_dest,
#                 'distance_traveled': self.total_distance,
#                 'cross_track_error': self.cross_track_error,
#                 'current_heading': self.current_heading,
#                 'filtered_heading': self.final_heading_filtered,
#                 'heading_source': self.heading_source_filtered,
#                 'algorithm': 'Road-Aligned Navigation (GPS Track Corrected)',
#                 'coordinate_system': 'UTM Zone 47N (Road-Aligned)',
#                 'wheelbase': self.WHEELBASE,
#                 'lookahead_distance': self.LOOKAHEAD_DISTANCE,
#                 'current_path_s': self.current_path_s,
#                 'path_length': self.path_length,
                
#                 # 🛣️ Road alignment status
#                 'road_alignment': {
#                     'aligned': self.road_aligned,
#                     'confidence': getattr(self, 'alignment_confidence', 0.0),
#                     'gps_points': len(self.gps_track_history),
#                     'gps_accuracy': self.gps_accuracy,
#                     'gps_confidence': self.gps_confidence,
#                     'corrections_applied': hasattr(self, 'corrected_path') and self.corrected_path is not None,
#                     'original_cross_track': getattr(self, 'original_cross_track_error', 0.0),
#                     'corrected_cross_track': self.cross_track_error
#                 }
#             }
            
#             # Add lookahead point
#             if self.lookahead_point:
#                 try:
#                     lookahead_lat, lookahead_lon = self.xy_to_latlon(
#                         self.lookahead_point[0], self.lookahead_point[1]
#                     )
#                     status['lookahead_lat'] = lookahead_lat
#                     status['lookahead_lon'] = lookahead_lon
#                     status['lookahead_distance'] = math.sqrt(
#                         (self.lookahead_point[0] - self.current_pos['x'])**2 + 
#                         (self.lookahead_point[1] - self.current_pos['y'])**2
#                     )
#                 except:
#                     pass
            
#             await self.broadcast_message(status)
            
#         except Exception as e:
#             self.get_logger().debug(f'Status send error: {e}')
    
#     async def broadcast_message(self, message):
#         """Broadcast message to all connected clients"""
#         if not self.connections:
#             return
        
#         try:
#             json_message = json.dumps(message, default=str)
#             disconnected = set()
            
#             for websocket in self.connections:
#                 try:
#                     await websocket.send(json_message)
#                 except websockets.exceptions.ConnectionClosed:
#                     disconnected.add(websocket)
            
#             for websocket in disconnected:
#                 self.connections.discard(websocket)
                
#         except Exception as e:
#             self.get_logger().debug(f'Broadcast error: {e}')


# # Simple heading filter
# class RoadAlignedHeadingFilter:
#     def __init__(self):
#         self.history = deque(maxlen=8)
#         self.last_stable_heading = 0.0
#         self.stability_threshold = 8.0
#         self.outlier_threshold = 30.0
        
#     def filter_heading(self, new_heading, speed):
#         try:
#             if new_heading is None:
#                 return self.last_stable_heading, "NONE"
            
#             new_heading = float(new_heading)
            
#             if self.is_outlier(new_heading):
#                 return self.last_stable_heading, "OUTLIER_FILTERED"
            
#             self.history.append(new_heading)
            
#             if len(self.history) < 3:
#                 self.last_stable_heading = new_heading
#                 return new_heading, "INITIALIZING"
            
#             if speed > 0.5:
#                 filtered_heading = self.moving_average()
#                 source = "MOVING_AVG"
#             else:
#                 filtered_heading = self.median_filter()
#                 source = "MEDIAN"
            
#             if filtered_heading is not None:
#                 self.last_stable_heading = filtered_heading
            
#             return self.last_stable_heading, source
            
#         except Exception:
#             return self.last_stable_heading if self.last_stable_heading is not None else 0.0, "ERROR"
    
#     def is_outlier(self, new_heading):
#         try:
#             if len(self.history) < 2:
#                 return False
            
#             recent_avg = np.mean(list(self.history)[-3:])
#             angle_diff = abs(self.angle_difference(new_heading, recent_avg))
            
#             return angle_diff > self.outlier_threshold
#         except:
#             return False
    
#     def moving_average(self):
#         try:
#             return np.mean(list(self.history)[-3:])
#         except:
#             return self.history[-1] if self.history else 0.0
    
#     def median_filter(self):
#         try:
#             return np.median(list(self.history)[-3:])
#         except:
#             return self.history[-1] if self.history else 0.0
    
#     def angle_difference(self, angle1, angle2):
#         try:
#             diff = float(angle1) - float(angle2)
#             while diff > 180:
#                 diff -= 360
#             while diff < -180:
#                 diff += 360
#             return diff
#         except:
#             return 0.0


# def main(args=None):
#     """Main function"""
#     rclpy.init(args=args)
#     navigation = None
    
#     try:
#         navigation = RoadAlignedNavigationSystem()
#         print('🚀 Road-Aligned Navigation System running...')
#         print('🛣️ Real-time GPS track correction enabled!')
#         print('📍 Path automatically aligns with actual road position!')
#         print('🔧 FIXED: Steering direction and web redirect issues!')
#         rclpy.spin(navigation)
        
#     except KeyboardInterrupt:
#         print('🛑 Keyboard interrupt received')
#     except Exception as e:
#         print(f'❌ Error: {e}')
#     finally:
#         if navigation:
#             navigation.destroy_node()
#         rclpy.shutdown()
#         print('🏁 Shutdown complete')

# if __name__ == '__main__':
#     main()


# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, Point
# from sensor_msgs.msg import NavSatFix
# from std_msgs.msg import Float32, String
# import asyncio
# import websockets
# import json
# import math
# import time
# import threading
# import numpy as np
# import pandas as pd
# from scipy.interpolate import interp1d
# from scipy.spatial.distance import cdist
# from collections import deque
# import pyproj
# import os
# from pathlib import Path

# class HybridWaypointNavigationSystem(Node):
#     def __init__(self):
#         super().__init__('hybrid_waypoint_navigation_system')
        
#         self.get_logger().info('🛣️ Hybrid Waypoint Navigation System Started!')
        
#         # Vehicle Parameters
#         self.WHEELBASE = 1.67
#         self.MAX_STEERING_ANGLE = 0.873
#         self.MIN_STEERING_ANGLE = -0.873
        
#         # Pure Pursuit Parameters
#         self.LOOKAHEAD_DISTANCE = 3.0
#         self.MIN_LOOKAHEAD = 2.0
#         self.MAX_LOOKAHEAD = 6.0
#         self.SPEED_TO_LOOKAHEAD_RATIO = 2.5
        
#         # 🛣️ Hybrid Waypoint Parameters
#         self.CSV_WAYPOINT_PATH = "waypoints.csv"  # CSV file with recorded waypoints
#         self.WAYPOINT_MATCHING_DISTANCE = 10.0    # ระยะที่ใช้ค้นหา waypoint ที่ใกล้เคียง (เมตร)
#         self.OFF_TRACK_WARNING_DISTANCE = 5.0     # ระยะที่จะเตือนว่าออกนอกเส้นทาง
#         self.WAYPOINT_REACHED_DISTANCE = 3.0      # ระยะที่ถือว่าถึง waypoint แล้ว
#         self.PATH_SMOOTHING_RESOLUTION = 0.5      # ความละเอียดของ path
        
#         # 🛣️ Off-track Detection
#         self.consecutive_off_track_count = 0
#         self.MAX_OFF_TRACK_COUNT = 5
#         self.last_warning_time = 0
#         self.WARNING_INTERVAL = 3.0
        
#         # Coordinate System
#         try:
#             self.transformer = pyproj.Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)
#         except Exception as e:
#             self.get_logger().warning(f'⚠️ PyProj failed: {e}')
#             self.transformer = None
        
#         self.ref_lat = 13.650748
#         self.ref_lon = 100.492985
#         self.reference_utm = None
        
#         # ROS2 Publishers & Subscribers
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.gnss_sub = self.create_subscription(NavSatFix, '/navigation/gnss', self.gnss_callback, 10)
#         self.xy_sub = self.create_subscription(Point, '/navigation/xy_position', self.xy_callback, 10)
#         self.heading_sub = self.create_subscription(Float32, '/navigation/heading', self.heading_callback, 10)
        
#         # 🛣️ Hybrid Waypoint State
#         self.csv_waypoints_db = []         # All waypoints from CSV file (database)
#         self.selected_waypoints = []       # Selected waypoints for current route
#         self.route_waypoints = []          # Waypoints from web route
#         self.final_path = None             # Final interpolated path
#         self.current_waypoint_index = 0    # Current target waypoint
#         self.reached_waypoints = []        # List of reached waypoints
#         self.csv_loaded = False
#         self.route_matched = False
        
#         # Navigation State
#         self.navigation_active = False
#         self.current_pos = {'x': 0.0, 'y': 0.0}
#         self.current_lat = 0.0
#         self.current_lon = 0.0
#         self.current_heading = 0.0
#         self.target_speed = 1.0
        
#         # Path Following State
#         self.current_path_s = 0.0
#         self.lookahead_point = None
#         self.path_curvature = 0.0
#         self.cross_track_error = 0.0
#         self.path_progress = 0.0
#         self.distance_to_next_waypoint = 0.0
        
#         # Simplified heading
#         self.simple_heading = 0.0
#         self.heading_source = "GPS"
        
#         # Control timing
#         self.control_timer = self.create_timer(0.1, self.control_loop)
        
#         # WebSocket Server
#         self.connections = set()
#         self.loop = None
        
#         # Performance tracking
#         self.start_time = 0
#         self.total_distance = 0.0
#         self.last_position = None
        
#         # Debug
#         self.debug_mode = True
#         self.debug_counter = 0
        
#         # Initialize coordinate system
#         self.init_coordinate_system()
        
#         # 🛣️ Load CSV waypoints database
#         self.load_csv_waypoints_database()
        
#         # Start WebSocket server
#         self.websocket_thread = threading.Thread(target=self.run_websocket_server)
#         self.websocket_thread.daemon = True
#         self.websocket_thread.start()
        
#         self.get_logger().info('✅ Hybrid Waypoint Navigation System ready!')
#         if self.csv_loaded:
#             self.get_logger().info(f'🛣️ Loaded {len(self.csv_waypoints_db)} waypoints from CSV database')
#         else:
#             self.get_logger().warning('⚠️ No CSV waypoints database loaded')
    
#     def load_csv_waypoints_database(self):
#         """🛣️ Load all waypoints from CSV file as database"""
#         try:
#             # Try multiple possible CSV file locations
#             possible_paths = [
#                 self.CSV_WAYPOINT_PATH,
#                 "waypoints.csv",
#                 "data/waypoints.csv",
#                 os.path.expanduser("~/waypoints.csv"),
#                 "/tmp/waypoints.csv",
#                 "paste.txt"  # Add your file name
#             ]
            
#             csv_path = None
#             for path in possible_paths:
#                 if os.path.exists(path):
#                     csv_path = path
#                     break
            
#             if not csv_path:
#                 self.get_logger().error(f'❌ CSV waypoint file not found. Tried: {possible_paths}')
#                 return False
            
#             # Read CSV file
#             self.get_logger().info(f'📂 Loading waypoints database from: {csv_path}')
            
#             try:
#                 df = pd.read_csv(csv_path)
                
#                 self.get_logger().info(f'📊 CSV file loaded: {len(df)} rows, columns: {list(df.columns)}')
                
#                 # Print first few rows for debugging
#                 self.get_logger().info(f'📋 First 3 rows of data:')
#                 for i in range(min(3, len(df))):
#                     row_data = df.iloc[i].to_dict()
#                     self.get_logger().info(f'  Row {i}: {row_data}')
                
#                 # 🔧 Support different CSV formats
#                 if 'x_east' in df.columns and 'y_north' in df.columns:
#                     # UTM format from your file
#                     self.get_logger().info("📍 Detected UTM format (x_east, y_north)")
#                     for idx, row in df.iterrows():
#                         try:
#                             x_utm = float(row['x_east'])
#                             y_utm = float(row['y_north'])
                            
#                             # Skip invalid coordinates
#                             if x_utm == 0 or y_utm == 0 or pd.isna(x_utm) or pd.isna(y_utm):
#                                 continue
                            
#                             # Validate UTM coordinates for Thailand (Zone 47N)
#                             if not (600000 <= x_utm <= 800000 and 1400000 <= y_utm <= 1700000):
#                                 self.get_logger().debug(f'⚠️ Skipping invalid UTM: ({x_utm}, {y_utm})')
#                                 continue
                            
#                             # Convert UTM to lat/lon
#                             lat, lon = self.utm_to_latlon(x_utm, y_utm)
                            
#                             # Validate lat/lon for Thailand
#                             if not (5.0 <= lat <= 21.0 and 97.0 <= lon <= 106.0):
#                                 self.get_logger().debug(f'⚠️ Skipping invalid lat/lon: ({lat}, {lon})')
#                                 continue
                            
#                             # Convert to local coordinates
#                             local_x, local_y = self.latlon_to_xy(lat, lon)
                            
#                             self.csv_waypoints_db.append({
#                                 'lat': lat, 'lon': lon,
#                                 'x': local_x, 'y': local_y,
#                                 'utm_x': x_utm, 'utm_y': y_utm,
#                                 'index': len(self.csv_waypoints_db)
#                             })
                            
#                         except (ValueError, TypeError) as e:
#                             self.get_logger().debug(f'⚠️ Skipping invalid row {idx}: {e}')
#                             continue
                
#                 elif 'raw_x_east' in df.columns and 'raw_y_north' in df.columns:
#                     # Alternative UTM format
#                     self.get_logger().info("📍 Detected UTM format (raw_x_east, raw_y_north)")
#                     for idx, row in df.iterrows():
#                         try:
#                             x_utm = float(row['raw_x_east'])
#                             y_utm = float(row['raw_y_north'])
                            
#                             # Skip invalid coordinates
#                             if x_utm == 0 or y_utm == 0 or pd.isna(x_utm) or pd.isna(y_utm):
#                                 continue
                            
#                             # Validate UTM coordinates for Thailand
#                             if not (600000 <= x_utm <= 800000 and 1400000 <= y_utm <= 1700000):
#                                 self.get_logger().debug(f'⚠️ Skipping invalid UTM: ({x_utm}, {y_utm})')
#                                 continue
                            
#                             # Convert UTM to lat/lon
#                             lat, lon = self.utm_to_latlon(x_utm, y_utm)
                            
#                             # Validate lat/lon for Thailand
#                             if not (5.0 <= lat <= 21.0 and 97.0 <= lon <= 106.0):
#                                 self.get_logger().debug(f'⚠️ Skipping invalid lat/lon: ({lat}, {lon})')
#                                 continue
                            
#                             # Convert to local coordinates
#                             local_x, local_y = self.latlon_to_xy(lat, lon)
                            
#                             self.csv_waypoints_db.append({
#                                 'lat': lat, 'lon': lon,
#                                 'x': local_x, 'y': local_y,
#                                 'utm_x': x_utm, 'utm_y': y_utm,
#                                 'index': len(self.csv_waypoints_db)
#                             })
                            
#                         except (ValueError, TypeError) as e:
#                             self.get_logger().debug(f'⚠️ Skipping invalid row {idx}: {e}')
#                             continue
                
#                 elif 'lat' in df.columns and 'lon' in df.columns:
#                     # Lat/Lon format
#                     self.get_logger().info("📍 Detected Lat/Lon format")
#                     for idx, row in df.iterrows():
#                         try:
#                             lat = float(row['lat'])
#                             lon = float(row['lon'])
                            
#                             if -90 <= lat <= 90 and -180 <= lon <= 180:
#                                 x, y = self.latlon_to_xy(lat, lon)
#                                 self.csv_waypoints_db.append({
#                                     'lat': lat, 'lon': lon,
#                                     'x': x, 'y': y,
#                                     'index': len(self.csv_waypoints_db)
#                                 })
#                         except (ValueError, TypeError) as e:
#                             self.get_logger().debug(f'⚠️ Skipping invalid row {idx}: {e}')
#                             continue
                
#                 elif len(df.columns) >= 2:
#                     # Generic 2-column format - determine if UTM or Lat/Lon
#                     self.get_logger().info("📍 Detected generic 2-column format")
#                     col1_name = df.columns[0]
#                     col2_name = df.columns[1]
                    
#                     # Analyze data to determine format
#                     sample_val1 = df.iloc[0, 0] if len(df) > 0 else 0
#                     sample_val2 = df.iloc[0, 1] if len(df) > 0 else 0
                    
#                     self.get_logger().info(f'📊 Sample values: {col1_name}={sample_val1}, {col2_name}={sample_val2}')
                    
#                     # Check if values look like UTM (large numbers in Thailand range)
#                     if (600000 <= float(sample_val1) <= 800000 and 
#                         1400000 <= float(sample_val2) <= 1700000):
#                         self.get_logger().info("🎯 Detected as UTM coordinates!")
                        
#                         for idx, row in df.iterrows():
#                             try:
#                                 x_utm = float(row.iloc[0])
#                                 y_utm = float(row.iloc[1])
                                
#                                 # Skip invalid coordinates
#                                 if x_utm == 0 or y_utm == 0 or pd.isna(x_utm) or pd.isna(y_utm):
#                                     continue
                                
#                                 # Validate UTM coordinates
#                                 if not (600000 <= x_utm <= 800000 and 1400000 <= y_utm <= 1700000):
#                                     continue
                                
#                                 # Convert UTM to lat/lon
#                                 lat, lon = self.utm_to_latlon(x_utm, y_utm)
                                
#                                 # Validate lat/lon
#                                 if not (5.0 <= lat <= 21.0 and 97.0 <= lon <= 106.0):
#                                     continue
                                
#                                 # Convert to local coordinates
#                                 local_x, local_y = self.latlon_to_xy(lat, lon)
                                
#                                 self.csv_waypoints_db.append({
#                                     'lat': lat, 'lon': lon,
#                                     'x': local_x, 'y': local_y,
#                                     'utm_x': x_utm, 'utm_y': y_utm,
#                                     'index': len(self.csv_waypoints_db)
#                                 })
                                
#                             except (ValueError, TypeError) as e:
#                                 self.get_logger().debug(f'⚠️ Skipping invalid row {idx}: {e}')
#                                 continue
                    
#                     else:
#                         # Assume Lat/Lon
#                         self.get_logger().info("🎯 Detected as Lat/Lon coordinates!")
#                         for idx, row in df.iterrows():
#                             try:
#                                 lat = float(row.iloc[0])
#                                 lon = float(row.iloc[1])
                                
#                                 if -90 <= lat <= 90 and -180 <= lon <= 180:
#                                     x, y = self.latlon_to_xy(lat, lon)
#                                     self.csv_waypoints_db.append({
#                                         'lat': lat, 'lon': lon,
#                                         'x': x, 'y': y,
#                                         'index': len(self.csv_waypoints_db)
#                                     })
#                             except (ValueError, TypeError) as e:
#                                 self.get_logger().debug(f'⚠️ Skipping invalid row {idx}: {e}')
#                                 continue
                
#                 else:
#                     raise ValueError("Cannot identify coordinate columns")
            
#             except Exception as e:
#                 self.get_logger().error(f'❌ Error reading CSV: {e}')
#                 return False
            
#             if len(self.csv_waypoints_db) < 2:
#                 self.get_logger().error(f'❌ Need at least 2 waypoints in database, got {len(self.csv_waypoints_db)}')
#                 self.get_logger().error(f'❌ Check CSV file format and data validity')
#                 return False
            
#             self.csv_loaded = True
            
#             # Print sample of loaded waypoints
#             self.get_logger().info(f'📋 Sample waypoints loaded:')
#             for i in range(min(3, len(self.csv_waypoints_db))):
#                 wp = self.csv_waypoints_db[i]
#                 self.get_logger().info(
#                     f'  #{i}: Lat={wp["lat"]:.6f}, Lon={wp["lon"]:.6f}, '
#                     f'UTM=({wp.get("utm_x", "N/A")}, {wp.get("utm_y", "N/A")})'
#                 )
            
#             self.get_logger().info(
#                 f'✅ Successfully loaded {len(self.csv_waypoints_db)} waypoints into database\n'
#                 f'  📍 Coverage area: {self.get_waypoint_coverage()}'
#             )
            
#             return True
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Failed to load CSV waypoints database: {e}')
#             return False
    
#     def utm_to_latlon(self, utm_x, utm_y):
#         """Convert UTM coordinates to lat/lon"""
#         try:
#             if self.transformer:
#                 # For UTM Zone 47N (EPSG:32647)
#                 lon, lat = self.transformer.transform(utm_x, utm_y, direction='INVERSE')
#                 return float(lat), float(lon)
#             else:
#                 # Fallback: approximate conversion for Thailand area
#                 # This is a rough approximation - use real transformer when possible
#                 lat = self.ref_lat + (utm_y - 1509584) / 111319.9
#                 lon = self.ref_lon + (utm_x - 661487) / (111319.9 * math.cos(math.radians(lat)))
#                 return float(lat), float(lon)
#         except Exception as e:
#             self.get_logger().debug(f'UTM conversion error: {e}')
#             return self.ref_lat, self.ref_lon
    
#     def get_waypoint_coverage(self):
#         """Get coverage area of waypoints"""
#         if not self.csv_waypoints_db:
#             return "No waypoints"
        
#         lats = [wp['lat'] for wp in self.csv_waypoints_db]
#         lons = [wp['lon'] for wp in self.csv_waypoints_db]
        
#         return (f"Lat: {min(lats):.6f} to {max(lats):.6f}, "
#                 f"Lon: {min(lons):.6f} to {max(lons):.6f}")
    
#     def match_route_to_waypoints(self, route_points):
#         """🛣️ Match web route to CSV waypoints using sequential waypoint selection"""
#         try:
#             if not self.csv_waypoints_db or not route_points:
#                 return False
            
#             self.get_logger().info(f'🔍 Sequential waypoint matching: {len(route_points)} route points')
            
#             # Convert route points to XY coordinates
#             route_xy = []
#             for point in route_points:
#                 x, y = self.latlon_to_xy(point['lat'], point['lon'])
#                 route_xy.append({'lat': point['lat'], 'lon': point['lon'], 'x': x, 'y': y})
            
#             start_point = route_xy[0]
#             end_point = route_xy[-1]
            
#             # Find starting waypoint (closest to first route point)
#             start_waypoint_idx = self.find_closest_waypoint_index(start_point)
#             if start_waypoint_idx is None:
#                 self.get_logger().error('❌ Cannot find starting waypoint')
#                 return False
            
#             # Find ending waypoint (closest to last route point)
#             end_waypoint_idx = self.find_closest_waypoint_index(end_point)
#             if end_waypoint_idx is None:
#                 self.get_logger().error('❌ Cannot find ending waypoint')
#                 return False
            
#             # Ensure proper direction (start < end for forward movement)
#             if start_waypoint_idx > end_waypoint_idx:
#                 self.get_logger().warning('⚠️ Route appears to be in reverse, swapping start/end')
#                 start_waypoint_idx, end_waypoint_idx = end_waypoint_idx, start_waypoint_idx
            
#             # Select sequential waypoints from start to end
#             selected_waypoints = []
#             for i in range(start_waypoint_idx, end_waypoint_idx + 1):
#                 if i < len(self.csv_waypoints_db):
#                     waypoint = self.csv_waypoints_db[i].copy()
#                     waypoint['csv_sequence_index'] = i
#                     waypoint['route_distance_start'] = self.calculate_distance_to_point(waypoint, start_point)
#                     waypoint['route_distance_end'] = self.calculate_distance_to_point(waypoint, end_point)
#                     selected_waypoints.append(waypoint)
            
#             if len(selected_waypoints) < 2:
#                 self.get_logger().error('❌ Not enough sequential waypoints found')
#                 return False
            
#             # Store selected waypoints
#             self.selected_waypoints = selected_waypoints
#             self.route_waypoints = route_xy
            
#             # Create final path from sequential waypoints
#             self.create_final_path_from_waypoints()
            
#             self.route_matched = True
            
#             # Calculate total route distance for reporting
#             total_route_distance = 0.0
#             if len(selected_waypoints) > 1:
#                 for i in range(len(selected_waypoints) - 1):
#                     wp1 = selected_waypoints[i]
#                     wp2 = selected_waypoints[i + 1]
#                     dx = wp2['x'] - wp1['x']
#                     dy = wp2['y'] - wp1['y']
#                     total_route_distance += math.sqrt(dx*dx + dy*dy)
            
#             self.get_logger().info(
#                 f'✅ Sequential waypoint matching completed:\n'
#                 f'  🎯 Start waypoint: CSV#{start_waypoint_idx} (distance to route start: {selected_waypoints[0]["route_distance_start"]:.1f}m)\n'
#                 f'  🏁 End waypoint: CSV#{end_waypoint_idx} (distance to route end: {selected_waypoints[-1]["route_distance_end"]:.1f}m)\n'
#                 f'  📊 Sequential waypoints: {len(selected_waypoints)} (CSV#{start_waypoint_idx} to CSV#{end_waypoint_idx})\n'
#                 f'  📏 Total distance: {total_route_distance:.1f}m\n'
#                 f'  🛣️ Method: Sequential CSV Waypoint Following'
#             )
            
#             return True
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Sequential waypoint matching failed: {e}')
#             return False
    
#     def find_closest_waypoint_index(self, target_point):
#         """🎯 Find closest waypoint index to target point"""
#         try:
#             min_distance = float('inf')
#             closest_idx = None
            
#             for i, waypoint in enumerate(self.csv_waypoints_db):
#                 distance = self.calculate_distance_to_point(waypoint, target_point)
                
#                 if distance <= self.WAYPOINT_MATCHING_DISTANCE and distance < min_distance:
#                     min_distance = distance
#                     closest_idx = i
            
#             if closest_idx is not None:
#                 self.get_logger().info(
#                     f'🎯 Found closest waypoint: CSV#{closest_idx} '
#                     f'at ({self.csv_waypoints_db[closest_idx]["x"]:.1f}, {self.csv_waypoints_db[closest_idx]["y"]:.1f}) '
#                     f'distance: {min_distance:.1f}m'
#                 )
#             else:
#                 self.get_logger().warning(
#                     f'⚠️ No waypoint found within {self.WAYPOINT_MATCHING_DISTANCE}m of target '
#                     f'({target_point["x"]:.1f}, {target_point["y"]:.1f})'
#                 )
            
#             return closest_idx
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Find closest waypoint failed: {e}')
#             return None
    
#     def calculate_distance_to_point(self, waypoint, target_point):
#         """📏 Calculate distance between waypoint and target point"""
#         try:
#             dx = waypoint['x'] - target_point['x']
#             dy = waypoint['y'] - target_point['y']
#             return math.sqrt(dx*dx + dy*dy)
#         except Exception as e:
#             return float('inf')
    
#     def create_final_path_from_waypoints(self):
#         """🛣️ Create smooth final path from selected waypoints"""
#         try:
#             if len(self.selected_waypoints) < 2:
#                 return
            
#             # Extract coordinates from selected waypoints
#             x_coords = [wp['x'] for wp in self.selected_waypoints]
#             y_coords = [wp['y'] for wp in self.selected_waypoints]
            
#             # Calculate cumulative distances
#             distances = [0.0]
#             for i in range(1, len(x_coords)):
#                 dx = x_coords[i] - x_coords[i-1]
#                 dy = y_coords[i] - y_coords[i-1]
#                 dist = math.sqrt(dx*dx + dy*dy)
#                 distances.append(distances[-1] + dist)
            
#             total_distance = distances[-1]
            
#             # Create interpolation functions
#             if len(x_coords) >= 3:
#                 # Use cubic spline for smooth path
#                 spline_x = interp1d(distances, x_coords, kind='cubic', bounds_error=False, fill_value='extrapolate')
#                 spline_y = interp1d(distances, y_coords, kind='cubic', bounds_error=False, fill_value='extrapolate')
#             else:
#                 # Use linear interpolation for 2 points
#                 spline_x = interp1d(distances, x_coords, kind='linear', fill_value='extrapolate')
#                 spline_y = interp1d(distances, y_coords, kind='linear', fill_value='extrapolate')
            
#             # Generate smooth path points
#             num_points = max(50, int(total_distance / self.PATH_SMOOTHING_RESOLUTION))
#             s_values = np.linspace(0, total_distance, num_points)
            
#             smooth_x = spline_x(s_values)
#             smooth_y = spline_y(s_values)
            
#             # Store final path
#             self.final_path = {
#                 'x': smooth_x,
#                 'y': smooth_y,
#                 's': s_values,
#                 'total_distance': total_distance,
#                 'waypoint_indices': self.map_waypoints_to_path(distances, s_values),
#                 'source': 'matched_csv_waypoints'
#             }
            
#             self.path_length = total_distance
            
#             self.get_logger().info(f"🛣️ Final path created from matched waypoints: {len(smooth_x)} interpolated points")
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Final path creation failed: {e}')
    
#     def map_waypoints_to_path(self, waypoint_distances, path_s):
#         """Map waypoints to interpolated path indices"""
#         waypoint_path_indices = []
#         for dist in waypoint_distances:
#             closest_idx = np.argmin(np.abs(path_s - dist))
#             waypoint_path_indices.append(closest_idx)
#         return waypoint_path_indices
    
#     def find_current_waypoint_target(self):
#         """🛣️ Find the next waypoint to target (sequential)"""
#         if not self.selected_waypoints:
#             return None
        
#         curr_x, curr_y = self.current_pos['x'], self.current_pos['y']
        
#         # Check if we've reached the current target waypoint
#         if self.current_waypoint_index < len(self.selected_waypoints):
#             target_wp = self.selected_waypoints[self.current_waypoint_index]
#             distance_to_target = math.sqrt(
#                 (curr_x - target_wp['x'])**2 + (curr_y - target_wp['y'])**2
#             )
            
#             if distance_to_target <= self.WAYPOINT_REACHED_DISTANCE:
#                 # Reached current waypoint
#                 self.reached_waypoints.append(self.current_waypoint_index)
#                 self.current_waypoint_index += 1
                
#                 csv_idx = target_wp.get('csv_sequence_index', target_wp.get('index', '?'))
                
#                 self.get_logger().info(
#                     f'✅ Reached sequential waypoint {len(self.reached_waypoints)}/{len(self.selected_waypoints)}: '
#                     f'CSV#{csv_idx} ({target_wp["lat"]:.6f}, {target_wp["lon"]:.6f}) '
#                     f'[Sequential waypoint following]'
#                 )
                
#                 # Check if we've reached the final waypoint
#                 if self.current_waypoint_index >= len(self.selected_waypoints):
#                     final_csv_idx = self.selected_waypoints[-1].get('csv_sequence_index', 
#                                                                   self.selected_waypoints[-1].get('index', '?'))
#                     self.get_logger().info(f'🎉 All sequential waypoints reached! Final waypoint: CSV#{final_csv_idx}')
#                     return None
        
#         # Return current target waypoint
#         if self.current_waypoint_index < len(self.selected_waypoints):
#             target_wp = self.selected_waypoints[self.current_waypoint_index]
#             self.distance_to_next_waypoint = math.sqrt(
#                 (curr_x - target_wp['x'])**2 + (curr_y - target_wp['y'])**2
#             )
#             return target_wp
        
#         return None
    
#     def check_off_track_warning(self):
#         """🛣️ Check if vehicle is off track and issue warnings"""
#         if not self.final_path:
#             return 0.0
        
#         # Find closest point on path
#         _, closest_point = self.find_closest_point_on_final_path()
        
#         # Calculate distance to path
#         curr_x, curr_y = self.current_pos['x'], self.current_pos['y']
#         distance_to_path = math.sqrt(
#             (curr_x - closest_point[0])**2 + (curr_y - closest_point[1])**2
#         )
        
#         # Check if off track
#         if distance_to_path > self.OFF_TRACK_WARNING_DISTANCE:
#             self.consecutive_off_track_count += 1
            
#             # Issue warning if consistently off track
#             current_time = time.time()
#             if (self.consecutive_off_track_count >= self.MAX_OFF_TRACK_COUNT and 
#                 current_time - self.last_warning_time > self.WARNING_INTERVAL):
                
#                 self.get_logger().warning(
#                     f'⚠️ OFF TRACK WARNING! Distance to matched path: {distance_to_path:.1f}m '
#                     f'(Max allowed: {self.OFF_TRACK_WARNING_DISTANCE:.1f}m)'
#                 )
#                 self.last_warning_time = current_time
#         else:
#             # Reset counter when back on track
#             if self.consecutive_off_track_count > 0:
#                 self.get_logger().info(f'✅ Back on matched path! Distance: {distance_to_path:.1f}m')
#             self.consecutive_off_track_count = 0
        
#         return distance_to_path
    
#     def find_closest_point_on_final_path(self):
#         """🛣️ Find closest point on final path"""
#         if not self.final_path:
#             return 0.0, (self.current_pos['x'], self.current_pos['y'])
        
#         curr_x, curr_y = self.current_pos['x'], self.current_pos['y']
        
#         min_distance = float('inf')
#         closest_idx = 0
#         closest_x, closest_y = 0.0, 0.0
        
#         # Find closest point on path
#         for i, (px, py) in enumerate(zip(self.final_path['x'], self.final_path['y'])):
#             distance = math.sqrt((curr_x - px)**2 + (curr_y - py)**2)
#             if distance < min_distance:
#                 min_distance = distance
#                 closest_idx = i
#                 closest_x, closest_y = px, py
        
#         # Get s-coordinate
#         closest_s = self.final_path['s'][closest_idx]
#         self.current_path_s = closest_s
        
#         # Calculate cross-track error with sign
#         if closest_idx < len(self.final_path['x']) - 1:
#             # Path direction vector
#             next_x = self.final_path['x'][closest_idx + 1]
#             next_y = self.final_path['y'][closest_idx + 1]
#             path_dx = next_x - closest_x
#             path_dy = next_y - closest_y
#         elif closest_idx > 0:
#             # Use previous point
#             prev_x = self.final_path['x'][closest_idx - 1]
#             prev_y = self.final_path['y'][closest_idx - 1]
#             path_dx = closest_x - prev_x
#             path_dy = closest_y - prev_y
#         else:
#             path_dx, path_dy = 1.0, 0.0
        
#         # Normalize path direction
#         path_length = math.sqrt(path_dx**2 + path_dy**2)
#         if path_length > 0:
#             path_dx /= path_length
#             path_dy /= path_length
        
#         # Vector from closest point to vehicle
#         to_vehicle_x = curr_x - closest_x
#         to_vehicle_y = curr_y - closest_y
        
#         # Cross product for signed distance
#         cross_product = to_vehicle_x * path_dy - to_vehicle_y * path_dx
#         self.cross_track_error = cross_product
        
#         return closest_s, (closest_x, closest_y)
    
#     def find_final_path_lookahead_point(self):
#         """🛣️ Find lookahead point on final path"""
#         if not self.final_path:
#             return None
        
#         # Get current position on path
#         current_s, _ = self.find_closest_point_on_final_path()
        
#         # Calculate dynamic lookahead distance
#         current_speed = max(0.1, self.target_speed)
#         lookahead_dist = max(self.MIN_LOOKAHEAD, 
#                            min(self.MAX_LOOKAHEAD, 
#                                current_speed * self.SPEED_TO_LOOKAHEAD_RATIO))
        
#         # Find lookahead point
#         target_s = current_s + lookahead_dist
#         target_s = min(target_s, self.final_path['total_distance'])
        
#         # Interpolate position at target_s
#         s_array = self.final_path['s']
#         x_array = self.final_path['x']
#         y_array = self.final_path['y']
        
#         # Find interpolation index
#         idx = np.searchsorted(s_array, target_s)
        
#         if idx >= len(x_array):
#             # Use final point
#             lookahead_x = x_array[-1]
#             lookahead_y = y_array[-1]
#         elif idx == 0:
#             # Use first point
#             lookahead_x = x_array[0]
#             lookahead_y = y_array[0]
#         else:
#             # Linear interpolation
#             s_prev = s_array[idx-1]
#             s_next = s_array[idx]
#             t = (target_s - s_prev) / (s_next - s_prev) if s_next != s_prev else 0.0
            
#             lookahead_x = x_array[idx-1] + t * (x_array[idx] - x_array[idx-1])
#             lookahead_y = y_array[idx-1] + t * (y_array[idx] - y_array[idx-1])
        
#         return (lookahead_x, lookahead_y)
    
#     def calculate_simple_steering(self):
#         """🛣️ Simple steering calculation"""
#         lookahead_point = self.find_final_path_lookahead_point()
        
#         if not lookahead_point:
#             return 0.0
        
#         self.lookahead_point = lookahead_point
        
#         # Current position
#         curr_x, curr_y = self.current_pos['x'], self.current_pos['y']
#         target_x, target_y = lookahead_point
        
#         # Vector to lookahead point
#         dx = target_x - curr_x
#         dy = target_y - curr_y
#         lookahead_distance = math.sqrt(dx*dx + dy*dy)
        
#         if lookahead_distance < 0.1:
#             return 0.0
        
#         # Calculate target angle
#         target_angle_rad = math.atan2(dy, dx)
#         target_angle_deg = math.degrees(target_angle_rad)
#         if target_angle_deg < 0:
#             target_angle_deg += 360
        
#         # Current heading
#         current_heading = self.simple_heading
        
#         # Calculate angle difference
#         angle_diff = target_angle_deg - current_heading
        
#         # Normalize angle difference
#         while angle_diff > 180:
#             angle_diff -= 360
#         while angle_diff < -180:
#             angle_diff += 360
        
#         # Simple proportional steering
#         steering_gain = 0.03
#         max_steering_deg = math.degrees(self.MAX_STEERING_ANGLE)
        
#         steering_angle_deg = angle_diff * steering_gain
#         steering_angle_deg = max(-max_steering_deg, min(max_steering_deg, steering_angle_deg))
        
#         steering_angle = math.radians(steering_angle_deg)
        
#         # Direction description
#         direction = "straight"
#         if abs(steering_angle_deg) > 3:
#             direction = "left" if steering_angle_deg > 0 else "right"
        
#         self.get_logger().debug(
#             f'🛣️ Hybrid Navigation:\n'
#             f'  📍 Vehicle: ({curr_x:.1f}, {curr_y:.1f})\n'
#             f'  🎯 Target: ({target_x:.1f}, {target_y:.1f})\n'
#             f'  📏 Distance: {lookahead_distance:.1f}m\n'
#             f'  🧭 Target Angle: {target_angle_deg:.1f}°\n'
#             f'  📐 Angle Diff: {angle_diff:+.1f}°\n'
#             f'  🎛️ Steering: {steering_angle_deg:+.1f}°\n'
#             f'  🚗 Command: {direction}'
#         )
        
#         return steering_angle
    
#     def calculate_progress(self):
#         """Calculate progress along final path"""
#         if not self.final_path or self.final_path['total_distance'] == 0:
#             return 0.0
        
#         progress = (self.current_path_s / self.final_path['total_distance']) * 100
#         self.path_progress = progress / 100.0
#         return progress
    
#     # Coordinate system functions
#     def init_coordinate_system(self):
#         """Initialize coordinate system"""
#         try:
#             if self.transformer:
#                 self.reference_utm = self.transformer.transform(self.ref_lon, self.ref_lat)
#                 self.get_logger().info(
#                     f'🛣️ Hybrid coordinate system initialized:\n'
#                     f'  📍 Reference: ({self.ref_lat:.6f}, {self.ref_lon:.6f})\n'
#                     f'  📍 UTM: ({self.reference_utm[0]:.2f}, {self.reference_utm[1]:.2f})'
#                 )
#             else:
#                 self.reference_utm = None
#         except Exception as e:
#             self.get_logger().error(f'❌ Coordinate system init failed: {e}')
#             self.reference_utm = None
    
#     def latlon_to_xy(self, lat, lon):
#         """Convert lat/lon to local coordinates"""
#         try:
#             if self.transformer and self.reference_utm:
#                 utm_x, utm_y = self.transformer.transform(lon, lat)
#                 local_x = utm_x - self.reference_utm[0]
#                 local_y = utm_y - self.reference_utm[1]
#                 return local_x, local_y
#             else:
#                 return self.simple_latlon_to_xy(lat, lon)
#         except Exception as e:
#             return self.simple_latlon_to_xy(lat, lon)
    
#     def simple_latlon_to_xy(self, lat, lon):
#         """Simple conversion"""
#         try:
#             dlat = float(lat) - self.ref_lat
#             dlon = float(lon) - self.ref_lon
            
#             x = dlon * 111319.9 * math.cos(math.radians(lat))
#             y = dlat * 111319.9
            
#             return float(x), float(y)
#         except Exception as e:
#             return 0.0, 0.0
    
#     def xy_to_latlon(self, x, y):
#         """Convert local coordinates to lat/lon"""
#         try:
#             if self.transformer and self.reference_utm:
#                 utm_x = float(x) + self.reference_utm[0]
#                 utm_y = float(y) + self.reference_utm[1]
                
#                 lon, lat = self.transformer.transform(utm_x, utm_y, direction='INVERSE')
#                 return float(lat), float(lon)
#             else:
#                 return self.simple_xy_to_latlon(x, y)
#         except Exception as e:
#             return self.simple_xy_to_latlon(x, y)
    
#     def simple_xy_to_latlon(self, x, y):
#         """Simple inverse conversion"""
#         try:
#             dlat = float(y) / 111319.9
#             dlon = float(x) / (111319.9 * math.cos(math.radians(self.ref_lat + dlat)))
            
#             lat = self.ref_lat + dlat
#             lon = self.ref_lon + dlon
            
#             return float(lat), float(lon)
#         except Exception as e:
#             return self.ref_lat, self.ref_lon
    
#     # ROS callbacks
#     def gnss_callback(self, msg):
#         """Handle GNSS position updates"""
#         try:
#             lat = float(msg.latitude)
#             lon = float(msg.longitude)
            
#             self.current_lat = lat
#             self.current_lon = lon
            
#             x, y = self.latlon_to_xy(lat, lon)
#             self.current_pos = {'x': float(x), 'y': float(y)}
            
#             # Update distance traveled
#             if self.last_position:
#                 dx = x - self.last_position['x']
#                 dy = y - self.last_position['y']
#                 distance = math.sqrt(dx*dx + dy*dy)
#                 if distance < 10.0:
#                     self.total_distance += distance
            
#             self.last_position = {'x': float(x), 'y': float(y)}
            
#             # Send status to frontend
#             if self.loop:
#                 asyncio.run_coroutine_threadsafe(self.send_status(), self.loop)
                
#         except Exception as e:
#             self.get_logger().error(f'❌ GNSS callback failed: {e}')
    
#     def xy_callback(self, msg):
#         """Handle XY position updates"""
#         try:
#             self.current_pos = {'x': float(msg.x), 'y': float(msg.y)}
#         except Exception as e:
#             self.get_logger().debug(f'XY callback failed: {e}')
    
#     def heading_callback(self, msg):
#         """Handle heading updates (simplified)"""
#         try:
#             raw_heading = float(msg.data)
#             self.current_heading = raw_heading
#             self.simple_heading = raw_heading
#             self.heading_source = "GPS"
#         except Exception as e:
#             self.get_logger().error(f'❌ Heading callback failed: {e}')
#             self.simple_heading = 0.0
#             self.heading_source = "ERROR"
    
#     # Navigation control
#     def start_hybrid_navigation(self):
#         """🛣️ Start hybrid waypoint navigation"""
#         try:
#             if not self.route_matched or not self.final_path:
#                 self.get_logger().error('❌ No matched route available! Please send route first.')
#                 return False
            
#             self.navigation_active = True
#             self.current_waypoint_index = 0
#             self.reached_waypoints = []
#             self.current_path_s = 0.0
#             self.start_time = time.time()
#             self.total_distance = 0.0
#             self.consecutive_off_track_count = 0
            
#             self.get_logger().info(
#                 f'🚀 Sequential Waypoint Navigation started!\n'
#                 f'  📊 Sequential waypoints: {len(self.selected_waypoints)}\n'
#                 f'  🎯 Start: CSV#{self.selected_waypoints[0].get("csv_sequence_index", "?")}\n'
#                 f'  🏁 End: CSV#{self.selected_waypoints[-1].get("csv_sequence_index", "?")}\n'
#                 f'  📏 Total distance: {self.final_path["total_distance"]:.1f}m\n'
#                 f'  🛣️ Algorithm: Sequential CSV Waypoint Following'
#             )
            
#             if self.loop:
#                 asyncio.run_coroutine_threadsafe(
#                     self.broadcast_message({
#                         'navigation_started': True, 
#                         'sequential_navigation': True,
#                         'sequential_waypoints': len(self.selected_waypoints),
#                         'start_waypoint': self.selected_waypoints[0].get("csv_sequence_index", "?"),
#                         'end_waypoint': self.selected_waypoints[-1].get("csv_sequence_index", "?"),
#                         'algorithm': 'Sequential CSV Waypoint Following'
#                     }), 
#                     self.loop
#                 )
#             return True
#         except Exception as e:
#             self.get_logger().error(f'❌ Start hybrid navigation failed: {e}')
#             return False
    
#     def pause_navigation(self):
#         """Pause navigation"""
#         self.navigation_active = False
#         self.stop_robot()
#         self.get_logger().info('⏸️ Hybrid navigation paused')
    
#     def stop_navigation(self):
#         """Stop navigation"""
#         self.navigation_active = False
#         self.current_waypoint_index = 0
#         self.reached_waypoints = []
#         self.current_path_s = 0.0
#         self.stop_robot()
#         self.get_logger().info('🛑 Hybrid navigation stopped')
    
#     def stop_robot(self):
#         """Send stop command"""
#         try:
#             twist = Twist()
#             twist.linear.x = 0.0
#             twist.angular.z = 0.0
#             self.cmd_vel_pub.publish(twist)
#         except Exception as e:
#             self.get_logger().error(f'❌ Stop robot failed: {e}')
    
#     def control_loop(self):
#         """🛣️ Main control loop for hybrid navigation"""
#         if not self.navigation_active or not self.route_matched:
#             return
        
#         try:
#             # Check if all waypoints reached
#             if self.current_waypoint_index >= len(self.selected_waypoints):
#                 self.navigation_active = False
#                 self.stop_robot()
                
#                 if self.loop:
#                     asyncio.run_coroutine_threadsafe(
#                         self.broadcast_message({
#                             'arrived': True,
#                             'total_time': time.time() - self.start_time,
#                             'distance_traveled': self.total_distance,
#                             'average_speed': self.total_distance / max(1, time.time() - self.start_time),
#                             'algorithm': 'Hybrid Route-CSV Matching',
#                             'waypoints_reached': len(self.reached_waypoints),
#                             'total_waypoints': len(self.selected_waypoints),
#                             'csv_waypoints_used': len(self.selected_waypoints)
#                         }), 
#                         self.loop
#                     )
                
#                 self.get_logger().info(
#                     f'🎉 All matched waypoints reached!\n'
#                     f'  ✅ Completed: {len(self.reached_waypoints)}/{len(self.selected_waypoints)} waypoints\n'
#                     f'  📏 Distance: {self.total_distance:.1f}m\n'
#                     f'  ⏱️ Time: {time.time() - self.start_time:.1f}s\n'
#                     f'  🛣️ Method: Route matched to CSV waypoints'
#                 )
#                 return
            
#             # Find current target waypoint
#             target_waypoint = self.find_current_waypoint_target()
            
#             # Check for off-track warnings
#             distance_to_path = self.check_off_track_warning()
            
#             # Calculate steering
#             steering_angle = self.calculate_simple_steering()
            
#             # Speed control based on off-track distance
#             speed = self.target_speed
            
#             # Reduce speed if off track
#             if distance_to_path > 3.0:
#                 speed *= 0.6
#             elif distance_to_path > 1.5:
#                 speed *= 0.8
            
#             # Reduce speed based on steering angle
#             try:
#                 steering_deg = abs(math.degrees(steering_angle))
#                 if steering_deg > 20:
#                     speed *= 0.5
#                 elif steering_deg > 10:
#                     speed *= 0.7
#             except:
#                 pass
            
#             speed = max(0.1, min(self.target_speed, speed))
            
#             # Publish control commands
#             twist = Twist()
#             twist.linear.x = float(speed)
#             twist.angular.z = float(steering_angle)
#             self.cmd_vel_pub.publish(twist)
            
#             # Debug logging
#             current_time = time.time()
#             if not hasattr(self, '_last_log_time') or current_time - self._last_log_time > 1.0:
#                 self._last_log_time = current_time
#                 progress = self.calculate_progress()
                
#                 steering_deg = math.degrees(steering_angle) if steering_angle is not None else 0.0
                
#                 target_info = ""
#                 if target_waypoint:
#                     csv_seq_idx = target_waypoint.get('csv_sequence_index', target_waypoint.get('index', '?'))
#                     target_info = f"CSV#{csv_seq_idx} (sequential)"
                
#                 self.get_logger().info(
#                     f'🛣️ Sequential Navigation:\n'
#                     f'  🎯 Target: {self.current_waypoint_index + 1}/{len(self.selected_waypoints)} - {target_info}\n'
#                     f'  📏 Distance to waypoint: {self.distance_to_next_waypoint:.1f}m\n'
#                     f'  📐 Cross-track error: {self.cross_track_error:+.2f}m\n'
#                     f'  📏 Distance to path: {distance_to_path:.1f}m\n'
#                     f'  🎛️ Steering: {steering_deg:+6.2f}° | Speed: {speed:.2f}m/s\n'
#                     f'  📊 Progress: {progress:.1f}%\n'
#                     f'  ⚠️ Off-track count: {self.consecutive_off_track_count}'
#                 )
        
#         except Exception as e:
#             self.get_logger().error(f'❌ Hybrid control loop error: {e}')
#             self.stop_robot()
    
#     # WebSocket Server
#     def run_websocket_server(self):
#         """Run WebSocket server"""
#         try:
#             self.loop = asyncio.new_event_loop()
#             asyncio.set_event_loop(self.loop)
            
#             start_server = websockets.serve(self.handle_websocket, 'localhost', 5001)
#             self.loop.run_until_complete(start_server)
#             self.get_logger().info('🌐 Hybrid Waypoint WebSocket server started on port 5001')
#             self.loop.run_forever()
            
#         except Exception as e:
#             self.get_logger().error(f'❌ WebSocket server error: {e}')
    
#     async def handle_websocket(self, websocket, path):
#         """Handle WebSocket connections"""
#         try:
#             self.connections.add(websocket)
#             self.get_logger().info('🔗 New client connected to Hybrid Waypoint system')
            
#             async for message in websocket:
#                 try:
#                     data = json.loads(message)
#                     await self.handle_message(data)
#                 except json.JSONDecodeError:
#                     self.get_logger().error('❌ Invalid JSON received')
                    
#         except websockets.exceptions.ConnectionClosed:
#             pass
#         finally:
#             self.connections.discard(websocket)
#             self.get_logger().info('🔌 Client disconnected')
    
#     async def handle_message(self, data):
#         """Handle incoming WebSocket messages"""
#         if data.get('path'):
#             # Route data received from web
#             await self.process_route_data(data)
#         elif data.get('navigation_command'):
#             cmd = data['navigation_command']
#             if cmd == 'start':
#                 success = self.start_hybrid_navigation()
#                 await self.broadcast_message({
#                     'command_result': 'start',
#                     'success': success,
#                     'message': 'Hybrid navigation started' if success else 'Failed to start - no matched route'
#                 })
#             elif cmd == 'stop':
#                 self.stop_navigation()
#                 await self.broadcast_message({
#                     'command_result': 'stop',
#                     'success': True,
#                     'message': 'Navigation stopped'
#                 })
#             elif cmd == 'pause':
#                 self.pause_navigation()
#                 await self.broadcast_message({
#                     'command_result': 'pause',
#                     'success': True,
#                     'message': 'Navigation paused'
#                 })
    
#     async def process_route_data(self, data):
#         """🛣️ Process route data and match to CSV waypoints"""
#         try:
#             route_points = data.get('path', [])
#             if len(route_points) < 2:
#                 await self.broadcast_message({
#                     'route_received': False,
#                     'error': 'Route too short (need at least 2 points)'
#                 })
#                 return
            
#             self.get_logger().info(f'🛣️ Processing route with hybrid matching: {len(route_points)} points')
            
#             # Match route to CSV waypoints
#             matching_success = self.match_route_to_waypoints(route_points)
            
#             if matching_success:
#                 # Send confirmation to frontend
#                 response = {
#                     'route_received': True,
#                     'route_matched': True,
#                     'original_points': len(route_points),
#                     'matched_waypoints': len(self.selected_waypoints),
#                     'csv_database_size': len(self.csv_waypoints_db),
#                     'interpolated_points': len(self.final_path["x"]) if self.final_path else 0,
#                     'total_distance': self.final_path['total_distance'] if self.final_path else 0,
#                     'algorithm': 'Hybrid Route-to-CSV Waypoint Matching',
#                     'coordinate_system': 'UTM Zone 47N (Hybrid)',
#                     'matching_distance': self.WAYPOINT_MATCHING_DISTANCE,
#                     'coverage_percentage': len(self.selected_waypoints)/len(route_points)*100,
#                     'average_matching_distance': np.mean([wp["distance_to_route"] for wp in self.selected_waypoints]) if self.selected_waypoints else 0
#                 }
                
#                 await self.broadcast_message(response)
                
#                 self.get_logger().info(
#                     f'✅ Route matching completed and sent to frontend:\n'
#                     f'  📊 Matching success: {len(self.selected_waypoints)}/{len(route_points)} points\n'
#                     f'  📏 Total distance: {response["total_distance"]:.1f}m\n'
#                     f'  🎯 Ready for hybrid navigation'
#                 )
#             else:
#                 await self.broadcast_message({
#                     'route_received': False,
#                     'route_matched': False,
#                     'error': 'Failed to match route to CSV waypoints',
#                     'csv_database_size': len(self.csv_waypoints_db),
#                     'matching_distance': self.WAYPOINT_MATCHING_DISTANCE
#                 })
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Route processing error: {e}')
#             await self.broadcast_message({
#                 'route_received': False,
#                 'error': f'Route processing failed: {str(e)}'
#             })
    
#     async def send_status(self):
#         """🛣️ Send status with hybrid navigation info"""
#         if not self.connections:
#             return
        
#         try:
#             progress = self.calculate_progress()
            
#             # Calculate distance to destination (last waypoint)
#             distance_to_dest = None
#             if self.selected_waypoints and len(self.selected_waypoints) > 0:
#                 last_wp = self.selected_waypoints[-1]
#                 curr_x, curr_y = self.current_pos['x'], self.current_pos['y']
#                 distance_to_dest = math.sqrt((curr_x - last_wp['x'])**2 + (curr_y - last_wp['y'])**2)
            
#             status = {
#                 'latitude': self.current_lat,
#                 'longitude': self.current_lon,
#                 'current_speed': self.target_speed if self.navigation_active else 0.0,
#                 'navigation_active': self.navigation_active,
#                 'route_progress': progress,
#                 'distance_to_destination': distance_to_dest,
#                 'distance_traveled': self.total_distance,
#                 'cross_track_error': self.cross_track_error,
#                 'current_heading': self.current_heading,
#                 'simple_heading': self.simple_heading,
#                 'heading_source': self.heading_source,
#                 'algorithm': 'Sequential CSV Waypoint Following',
#                 'coordinate_system': 'UTM Zone 47N (Sequential)',
#                 'wheelbase': self.WHEELBASE,
#                 'lookahead_distance': self.LOOKAHEAD_DISTANCE,
#                 'current_path_s': self.current_path_s,
#                 'path_length': self.path_length if hasattr(self, 'path_length') else 0,
                
#                 # 🛣️ Hybrid-specific status
#                 'hybrid_navigation': {
#                     'csv_database_loaded': self.csv_loaded,
#                     'csv_database_size': len(self.csv_waypoints_db),
#                     'route_matched': self.route_matched,
#                     'selected_waypoints': len(self.selected_waypoints),
#                     'current_waypoint_index': self.current_waypoint_index,
#                     'reached_waypoints': len(self.reached_waypoints),
#                     'distance_to_next_waypoint': self.distance_to_next_waypoint,
#                     'off_track_warnings': self.consecutive_off_track_count,
#                     'matching_distance': self.WAYPOINT_MATCHING_DISTANCE,
#                     'off_track_threshold': self.OFF_TRACK_WARNING_DISTANCE
#                 }
#             }
            
#             # Add lookahead point
#             if self.lookahead_point:
#                 try:
#                     lookahead_lat, lookahead_lon = self.xy_to_latlon(
#                         self.lookahead_point[0], self.lookahead_point[1]
#                     )
#                     status['lookahead_lat'] = lookahead_lat
#                     status['lookahead_lon'] = lookahead_lon
#                     status['lookahead_distance'] = math.sqrt(
#                         (self.lookahead_point[0] - self.current_pos['x'])**2 + 
#                         (self.lookahead_point[1] - self.current_pos['y'])**2
#                     )
#                 except:
#                     pass
            
#             # Add current target waypoint info
#             if (self.current_waypoint_index < len(self.selected_waypoints) and 
#                 self.current_waypoint_index >= 0):
#                 target_wp = self.selected_waypoints[self.current_waypoint_index]
#                 status['current_target_waypoint'] = {
#                     'csv_index': target_wp['index'],
#                     'route_index': target_wp.get('route_index', -1),
#                     'lat': target_wp['lat'],
#                     'lon': target_wp['lon'],
#                     'distance': self.distance_to_next_waypoint,
#                     'matching_distance': target_wp.get('distance_to_route', 0)
#                 }
            
#             await self.broadcast_message(status)
            
#         except Exception as e:
#             self.get_logger().debug(f'Status send error: {e}')
    
#     async def broadcast_message(self, message):
#         """Broadcast message to all connected clients"""
#         if not self.connections:
#             return
        
#         try:
#             json_message = json.dumps(message, default=str)
#             disconnected = set()
            
#             for websocket in self.connections:
#                 try:
#                     await websocket.send(json_message)
#                 except websockets.exceptions.ConnectionClosed:
#                     disconnected.add(websocket)
            
#             for websocket in disconnected:
#                 self.connections.discard(websocket)
                
#         except Exception as e:
#             self.get_logger().debug(f'Broadcast error: {e}')


# def main(args=None):
#     """Main function"""
#     rclpy.init(args=args)
#     navigation = None
    
#     try:
#         navigation = HybridWaypointNavigationSystem()
#         print('🚀 Hybrid Waypoint Navigation System running...')
#         print('🛣️ Matches web routes to CSV waypoints for precision navigation!')
#         print('📍 Automatically selects best waypoints from database!')
#         print('🎯 Combines route planning flexibility with waypoint accuracy!')
#         rclpy.spin(navigation)
        
#     except KeyboardInterrupt:
#         print('🛑 Keyboard interrupt received')
#     except Exception as e:
#         print(f'❌ Error: {e}')
#     finally:
#         if navigation:
#             navigation.destroy_node()
#         rclpy.shutdown()
#         print('🏁 Shutdown complete')

# if __name__ == '__main__':
#     main()



# ----------------------------------------จับwaypointทั้งหมด-------------------------------------------------

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, Point
# from sensor_msgs.msg import NavSatFix
# from std_msgs.msg import Float32, String
# import asyncio
# import websockets
# import json
# import math
# import time
# import threading
# import numpy as np
# import pandas as pd
# from scipy.interpolate import interp1d
# from scipy.spatial.distance import cdist
# from collections import deque
# import pyproj
# import os
# from pathlib import Path

# class HybridWaypointNavigationSystem(Node):
#     def __init__(self):
#         super().__init__('hybrid_waypoint_navigation_system')
        
#         self.get_logger().info('🛣️ Hybrid Waypoint Navigation System Started!')
        
#         # Vehicle Parameters
#         self.WHEELBASE = 1.67
#         self.MAX_STEERING_ANGLE = 0.873
#         self.MIN_STEERING_ANGLE = -0.873
        
#         # Pure Pursuit Parameters
#         self.LOOKAHEAD_DISTANCE = 3.0
#         self.MIN_LOOKAHEAD = 2.0
#         self.MAX_LOOKAHEAD = 6.0
#         self.SPEED_TO_LOOKAHEAD_RATIO = 2.5
        
#         # 🛣️ Hybrid Waypoint Parameters
#         self.CSV_WAYPOINT_PATH = "waypoints.csv"
#         self.WAYPOINT_MATCHING_DISTANCE = 10.0
#         self.OFF_TRACK_WARNING_DISTANCE = 5.0
#         self.WAYPOINT_REACHED_DISTANCE = 3.0
#         self.PATH_SMOOTHING_RESOLUTION = 0.5
        
#         # 🛣️ Off-track Detection
#         self.consecutive_off_track_count = 0
#         self.MAX_OFF_TRACK_COUNT = 5
#         self.last_warning_time = 0
#         self.WARNING_INTERVAL = 3.0
        
#         # Coordinate System
#         try:
#             self.transformer = pyproj.Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)
#         except Exception as e:
#             self.get_logger().warning(f'⚠️ PyProj failed: {e}')
#             self.transformer = None
        
#         self.ref_lat = 13.650748
#         self.ref_lon = 100.492985
#         self.reference_utm = None
        
#         # ROS2 Publishers & Subscribers
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.gnss_sub = self.create_subscription(NavSatFix, '/navigation/gnss', self.gnss_callback, 10)
#         self.xy_sub = self.create_subscription(Point, '/navigation/xy_position', self.xy_callback, 10)
#         self.heading_sub = self.create_subscription(Float32, '/navigation/heading', self.heading_callback, 10)
        
#         # 🛣️ Hybrid Waypoint State
#         self.csv_waypoints_db = []
#         self.selected_waypoints = []
#         self.route_waypoints = []
#         self.final_path = None
#         self.current_waypoint_index = 0
#         self.reached_waypoints = []
#         self.csv_loaded = False
#         self.route_matched = False
        
#         # Navigation State
#         self.navigation_active = False
#         self.current_pos = {'x': 0.0, 'y': 0.0}
#         self.current_lat = 0.0
#         self.current_lon = 0.0
#         self.current_heading = 0.0
#         self.target_speed = 1.0
        
#         # Path Following State
#         self.current_path_s = 0.0
#         self.lookahead_point = None
#         self.path_curvature = 0.0
#         self.cross_track_error = 0.0
#         self.path_progress = 0.0
#         self.distance_to_next_waypoint = 0.0
        
#         # Simplified heading
#         self.simple_heading = 0.0
#         self.heading_source = "GPS"
        
#         # Control timing
#         self.control_timer = self.create_timer(0.1, self.control_loop)
        
#         # WebSocket Server
#         self.connections = set()
#         self.loop = None
        
#         # Performance tracking
#         self.start_time = 0
#         self.total_distance = 0.0
#         self.last_position = None
        
#         # Debug
#         self.debug_mode = True
#         self.debug_counter = 0
        
#         # Initialize coordinate system
#         self.init_coordinate_system()
        
#         # 🛣️ Load CSV waypoints database
#         self.load_csv_waypoints_database()
        
#         # Start WebSocket server
#         self.websocket_thread = threading.Thread(target=self.run_websocket_server)
#         self.websocket_thread.daemon = True
#         self.websocket_thread.start()
        
#         self.get_logger().info('✅ Hybrid Waypoint Navigation System ready!')
#         if self.csv_loaded:
#             self.get_logger().info(f'🛣️ Loaded {len(self.csv_waypoints_db)} waypoints from CSV database')
#         else:
#             self.get_logger().warning('⚠️ No CSV waypoints database loaded')
    
#     def load_csv_waypoints_database(self):
#         """🛣️ Load all waypoints from CSV file as database"""
#         try:
#             possible_paths = [
#                 self.CSV_WAYPOINT_PATH,
#                 "waypoints.csv",
#                 "data/waypoints.csv",
#                 os.path.expanduser("~/waypoints.csv"),
#                 "/tmp/waypoints.csv"
#             ]
            
#             csv_path = None
#             for path in possible_paths:
#                 if os.path.exists(path):
#                     csv_path = path
#                     break
            
#             if not csv_path:
#                 self.get_logger().error(f'❌ CSV waypoint file not found. Tried: {possible_paths}')
#                 return False
            
#             self.get_logger().info(f'📂 Loading waypoints database from: {csv_path}')
            
#             try:
#                 df = pd.read_csv(csv_path)
#                 self.get_logger().info(f'📊 CSV file loaded: {len(df)} rows, columns: {list(df.columns)}')
                
#                 # Show first 3 rows for debugging
#                 self.get_logger().info('📋 First 3 rows of data:')
#                 for i in range(min(3, len(df))):
#                     row_dict = df.iloc[i].to_dict()
#                     self.get_logger().info(f'  Row {i}: {row_dict}')
                
#                 # 🔧 Support different CSV formats
#                 if 'x_east' in df.columns and 'y_north' in df.columns:
#                     self.get_logger().info("📍 Detected UTM format (x_east, y_north)")
#                     for _, row in df.iterrows():
#                         x_utm = float(row['x_east'])
#                         y_utm = float(row['y_north'])
                        
#                         lat, lon = self.utm_to_latlon(x_utm, y_utm)
#                         local_x, local_y = self.latlon_to_xy(lat, lon)
                        
#                         self.csv_waypoints_db.append({
#                             'lat': lat, 'lon': lon,
#                             'x': local_x, 'y': local_y,
#                             'utm_x': x_utm, 'utm_y': y_utm,
#                             'index': len(self.csv_waypoints_db)
#                         })
                
#                 elif 'lat' in df.columns and 'lon' in df.columns:
#                     self.get_logger().info("📍 Detected Lat/Lon format")
#                     for _, row in df.iterrows():
#                         lat = float(row['lat'])
#                         lon = float(row['lon'])
                        
#                         if -90 <= lat <= 90 and -180 <= lon <= 180:
#                             x, y = self.latlon_to_xy(lat, lon)
#                             self.csv_waypoints_db.append({
#                                 'lat': lat, 'lon': lon,
#                                 'x': x, 'y': y,
#                                 'index': len(self.csv_waypoints_db)
#                             })
                
#                 elif len(df.columns) >= 2:
#                     self.get_logger().info("📍 Detected generic 2-column format")
#                     for _, row in df.iterrows():
#                         val1, val2 = float(row.iloc[0]), float(row.iloc[1])
                        
#                         if val1 > 180 or val2 > 180:
#                             lat, lon = self.utm_to_latlon(val1, val2)
#                             x, y = self.latlon_to_xy(lat, lon)
#                         else:
#                             lat, lon = val1, val2
#                             x, y = self.latlon_to_xy(lat, lon)
                        
#                         self.csv_waypoints_db.append({
#                             'lat': lat, 'lon': lon,
#                             'x': x, 'y': y,
#                             'index': len(self.csv_waypoints_db)
#                         })
                
#                 else:
#                     raise ValueError("Cannot identify coordinate columns")
            
#             except Exception as e:
#                 self.get_logger().error(f'❌ Error reading CSV: {e}')
#                 return False
            
#             if len(self.csv_waypoints_db) < 2:
#                 self.get_logger().error('❌ Need at least 2 waypoints in database')
#                 return False
            
#             self.csv_loaded = True
            
#             # Show sample waypoints
#             self.get_logger().info('📋 Sample waypoints loaded:')
#             for i in range(min(3, len(self.csv_waypoints_db))):
#                 wp = self.csv_waypoints_db[i]
#                 self.get_logger().info(
#                     f'  #{i}: Lat={wp["lat"]:.6f}, Lon={wp["lon"]:.6f}, '
#                     f'UTM=({wp.get("utm_x", wp["x"]):.4f}, {wp.get("utm_y", wp["y"]):.3f})'
#                 )
            
#             self.get_logger().info(
#                 f'✅ Successfully loaded {len(self.csv_waypoints_db)} waypoints into database\n'
#                 f'  📍 Coverage area: {self.get_waypoint_coverage()}'
#             )
            
#             return True
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Failed to load CSV waypoints database: {e}')
#             return False
    
#     def utm_to_latlon(self, utm_x, utm_y):
#         """Convert UTM coordinates to lat/lon"""
#         try:
#             if self.transformer:
#                 lon, lat = self.transformer.transform(utm_x, utm_y, direction='INVERSE')
#                 return float(lat), float(lon)
#             else:
#                 lat = self.ref_lat + (utm_y - 1509584) / 111319.9
#                 lon = self.ref_lon + (utm_x - 661487) / (111319.9 * math.cos(math.radians(lat)))
#                 return float(lat), float(lon)
#         except Exception as e:
#             self.get_logger().debug(f'UTM conversion error: {e}')
#             return self.ref_lat, self.ref_lon
    
#     def get_waypoint_coverage(self):
#         """Get coverage area of waypoints"""
#         if not self.csv_waypoints_db:
#             return "No waypoints"
        
#         lats = [wp['lat'] for wp in self.csv_waypoints_db]
#         lons = [wp['lon'] for wp in self.csv_waypoints_db]
        
#         return (f"Lat: {min(lats):.6f} to {max(lats):.6f}, "
#                 f"Lon: {min(lons):.6f} to {max(lons):.6f}")
    
#     def ensure_waypoint_fields(self, waypoints, route_points=None):
#         """🔧 Ensure all waypoints have required fields"""
#         for i, wp in enumerate(waypoints):
#             if 'distance_to_route' not in wp:
#                 if route_points and i < len(route_points):
#                     route_point = route_points[i]
#                     if isinstance(route_point, dict) and 'x' in route_point and 'y' in route_point:
#                         distance = math.sqrt(
#                             (wp['x'] - route_point['x'])**2 + (wp['y'] - route_point['y'])**2
#                         )
#                     else:
#                         distance = 0.0
#                     wp['distance_to_route'] = distance
#                 else:
#                     wp['distance_to_route'] = 0.0
            
#             if 'route_index' not in wp:
#                 wp['route_index'] = i
        
#         return waypoints
    
#     def match_route_to_waypoints(self, route_points):
#         """🛣️ Match web route to CSV waypoints"""
#         try:
#             if not self.csv_waypoints_db or not route_points:
#                 return False
            
#             self.get_logger().info(f'🔍 Matching route ({len(route_points)} points) to waypoint database...')
            
#             # Convert route points to XY coordinates
#             route_xy = []
#             for point in route_points:
#                 x, y = self.latlon_to_xy(point['lat'], point['lon'])
#                 route_xy.append({'lat': point['lat'], 'lon': point['lon'], 'x': x, 'y': y})
            
#             # Build waypoint coordinate arrays for efficient matching
#             waypoint_coords = np.array([[wp['x'], wp['y']] for wp in self.csv_waypoints_db])
            
#             # Find matching waypoints for each route point
#             matched_waypoints = []
#             used_waypoint_indices = set()
            
#             for i, route_point in enumerate(route_xy):
#                 # Calculate distances to all waypoints
#                 route_coord = np.array([[route_point['x'], route_point['y']]])
#                 distances = cdist(route_coord, waypoint_coords)[0]
                
#                 # Find closest waypoints within matching distance
#                 nearby_indices = np.where(distances <= self.WAYPOINT_MATCHING_DISTANCE)[0]
                
#                 if len(nearby_indices) > 0:
#                     # Choose closest unused waypoint, or closest if all used
#                     available_indices = [idx for idx in nearby_indices if idx not in used_waypoint_indices]
                    
#                     if available_indices:
#                         best_idx = available_indices[np.argmin(distances[available_indices])]
#                     else:
#                         best_idx = nearby_indices[np.argmin(distances[nearby_indices])]
                    
#                     matched_waypoint = self.csv_waypoints_db[best_idx].copy()
#                     matched_waypoint['route_index'] = i
#                     matched_waypoint['distance_to_route'] = distances[best_idx]
#                     matched_waypoints.append(matched_waypoint)
#                     used_waypoint_indices.add(best_idx)
                    
#                     self.get_logger().debug(
#                         f"  Route point {i}: matched to waypoint {best_idx} "
#                         f"(distance: {distances[best_idx]:.1f}m)"
#                     )
#                 else:
#                     self.get_logger().warning(
#                         f"  Route point {i}: no waypoint within {self.WAYPOINT_MATCHING_DISTANCE}m"
#                     )
            
#             if len(matched_waypoints) < 2:
#                 self.get_logger().error('❌ Not enough waypoints matched to route')
#                 return False
            
#             # Sort matched waypoints by route order
#             matched_waypoints.sort(key=lambda wp: wp['route_index'])
            
#             # Store selected waypoints
#             self.selected_waypoints = matched_waypoints
#             self.route_waypoints = route_xy
            
#             # Create final path from matched waypoints
#             self.create_final_path_from_waypoints()
            
#             self.route_matched = True
            
#             self.get_logger().info(
#                 f'✅ Route matching completed:\n'
#                 f'  📊 Original route: {len(route_points)} points\n'
#                 f'  🎯 Matched waypoints: {len(matched_waypoints)} points\n'
#                 f'  📏 Average matching distance: {np.mean([wp["distance_to_route"] for wp in matched_waypoints]):.1f}m\n'
#                 f'  📈 Coverage: {len(matched_waypoints)/len(route_points)*100:.1f}% of route'
#             )
            
#             return True
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Route matching failed: {e}')
#             return False
    
#     def match_route_to_sequential_waypoints(self, route_points):
#         """🛣️ Match web route to sequential CSV waypoints (FIXED VERSION)"""
#         try:
#             if not self.csv_waypoints_db or not route_points:
#                 return False
            
#             self.get_logger().info(f'🔍 Sequential waypoint matching: {len(route_points)} route points')
            
#             # Convert route points to XY coordinates
#             route_xy = []
#             for point in route_points:
#                 x, y = self.latlon_to_xy(point['lat'], point['lon'])
#                 route_xy.append({'lat': point['lat'], 'lon': point['lon'], 'x': x, 'y': y})
            
#             # Find closest waypoint to start point
#             start_route_point = route_xy[0]
#             min_start_distance = float('inf')
#             start_idx = 0
            
#             for i, wp in enumerate(self.csv_waypoints_db):
#                 distance = math.sqrt(
#                     (start_route_point['x'] - wp['x'])**2 + 
#                     (start_route_point['y'] - wp['y'])**2
#                 )
#                 if distance < min_start_distance:
#                     min_start_distance = distance
#                     start_idx = i
            
#             # Find closest waypoint to end point
#             end_route_point = route_xy[-1]
#             min_end_distance = float('inf')
#             end_idx = len(self.csv_waypoints_db) - 1
            
#             for i, wp in enumerate(self.csv_waypoints_db):
#                 distance = math.sqrt(
#                     (end_route_point['x'] - wp['x'])**2 + 
#                     (end_route_point['y'] - wp['y'])**2
#                 )
#                 if distance < min_end_distance:
#                     min_end_distance = distance
#                     end_idx = i
            
#             # Ensure proper order
#             if start_idx > end_idx:
#                 start_idx, end_idx = end_idx, start_idx
#                 min_start_distance, min_end_distance = min_end_distance, min_start_distance
            
#             self.get_logger().info(
#                 f'🎯 Found closest waypoint: CSV#{start_idx} at '
#                 f'({self.csv_waypoints_db[start_idx]["x"]:.1f}, {self.csv_waypoints_db[start_idx]["y"]:.1f}) '
#                 f'distance: {min_start_distance:.1f}m'
#             )
            
#             self.get_logger().info(
#                 f'🎯 Found closest waypoint: CSV#{end_idx} at '
#                 f'({self.csv_waypoints_db[end_idx]["x"]:.1f}, {self.csv_waypoints_db[end_idx]["y"]:.1f}) '
#                 f'distance: {min_end_distance:.1f}m'
#             )
            
#             # Select sequential waypoints and ADD distance_to_route field
#             self.selected_waypoints = []
#             for i in range(start_idx, end_idx + 1):
#                 waypoint = self.csv_waypoints_db[i].copy()
#                 waypoint['route_index'] = i - start_idx
                
#                 # 🔧 FIX: Calculate distance_to_route for consistency
#                 if i - start_idx < len(route_xy):
#                     route_point = route_xy[i - start_idx]
#                     distance_to_route = math.sqrt(
#                         (waypoint['x'] - route_point['x'])**2 + 
#                         (waypoint['y'] - route_point['y'])**2
#                     )
#                 else:
#                     # For waypoints beyond route points, calculate to nearest route point
#                     min_dist_to_route = float('inf')
#                     for rp in route_xy:
#                         dist = math.sqrt(
#                             (waypoint['x'] - rp['x'])**2 + 
#                             (waypoint['y'] - rp['y'])**2
#                         )
#                         if dist < min_dist_to_route:
#                             min_dist_to_route = dist
#                     distance_to_route = min_dist_to_route
                
#                 waypoint['distance_to_route'] = distance_to_route
#                 self.selected_waypoints.append(waypoint)
            
#             # Store route waypoints
#             self.route_waypoints = route_xy
            
#             # Create final path from selected waypoints
#             self.create_final_path_from_waypoints()
            
#             self.route_matched = True
            
#             # Calculate statistics
#             total_waypoints = len(self.selected_waypoints)
#             total_distance = sum([
#                 math.sqrt(
#                     (self.selected_waypoints[i]['x'] - self.selected_waypoints[i-1]['x'])**2 + 
#                     (self.selected_waypoints[i]['y'] - self.selected_waypoints[i-1]['y'])**2
#                 ) for i in range(1, total_waypoints)
#             ]) if total_waypoints > 1 else 0
            
#             # 🔧 FIX: Now we can safely calculate average matching distance
#             avg_matching_distance = np.mean([wp["distance_to_route"] for wp in self.selected_waypoints])
            
#             self.get_logger().info(
#                 f'✅ Sequential waypoint matching completed:\n'
#                 f'  🎯 Start waypoint: CSV#{start_idx} (distance to route start: {min_start_distance:.1f}m)\n'
#                 f'  🏁 End waypoint: CSV#{end_idx} (distance to route end: {min_end_distance:.1f}m)\n'
#                 f'  📊 Sequential waypoints: {total_waypoints} (CSV#{start_idx} to CSV#{end_idx})\n'
#                 f'  📏 Total distance: {total_distance:.1f}m\n'
#                 f'  📐 Average matching distance: {avg_matching_distance:.1f}m\n'
#                 f'  🛣️ Method: Sequential CSV Waypoint Following'
#             )
            
#             return True
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Sequential waypoint matching failed: {e}')
#             return False
    
#     def create_final_path_from_waypoints(self):
#         """🛣️ Create smooth final path from selected waypoints"""
#         try:
#             if len(self.selected_waypoints) < 2:
#                 return
            
#             # Extract coordinates from selected waypoints
#             x_coords = [wp['x'] for wp in self.selected_waypoints]
#             y_coords = [wp['y'] for wp in self.selected_waypoints]
            
#             # Calculate cumulative distances
#             distances = [0.0]
#             for i in range(1, len(x_coords)):
#                 dx = x_coords[i] - x_coords[i-1]
#                 dy = y_coords[i] - y_coords[i-1]
#                 dist = math.sqrt(dx*dx + dy*dy)
#                 distances.append(distances[-1] + dist)
            
#             total_distance = distances[-1]
            
#             # Create interpolation functions
#             if len(x_coords) >= 3:
#                 spline_x = interp1d(distances, x_coords, kind='cubic', bounds_error=False, fill_value='extrapolate')
#                 spline_y = interp1d(distances, y_coords, kind='cubic', bounds_error=False, fill_value='extrapolate')
#             else:
#                 spline_x = interp1d(distances, x_coords, kind='linear', fill_value='extrapolate')
#                 spline_y = interp1d(distances, y_coords, kind='linear', fill_value='extrapolate')
            
#             # Generate smooth path points
#             num_points = max(50, int(total_distance / self.PATH_SMOOTHING_RESOLUTION))
#             s_values = np.linspace(0, total_distance, num_points)
            
#             smooth_x = spline_x(s_values)
#             smooth_y = spline_y(s_values)
            
#             # Store final path
#             self.final_path = {
#                 'x': smooth_x,
#                 'y': smooth_y,
#                 's': s_values,
#                 'total_distance': total_distance,
#                 'waypoint_indices': self.map_waypoints_to_path(distances, s_values),
#                 'source': 'matched_csv_waypoints'
#             }
            
#             self.path_length = total_distance
            
#             self.get_logger().info(f"🛣️ Final path created from matched waypoints: {len(smooth_x)} interpolated points")
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Final path creation failed: {e}')
    
#     def map_waypoints_to_path(self, waypoint_distances, path_s):
#         """Map waypoints to interpolated path indices"""
#         waypoint_path_indices = []
#         for dist in waypoint_distances:
#             closest_idx = np.argmin(np.abs(path_s - dist))
#             waypoint_path_indices.append(closest_idx)
#         return waypoint_path_indices
    
#     def find_current_waypoint_target(self):
#         """🛣️ Find the next waypoint to target"""
#         if not self.selected_waypoints:
#             return None
        
#         curr_x, curr_y = self.current_pos['x'], self.current_pos['y']
        
#         # Check if we've reached the current target waypoint
#         if self.current_waypoint_index < len(self.selected_waypoints):
#             target_wp = self.selected_waypoints[self.current_waypoint_index]
#             distance_to_target = math.sqrt(
#                 (curr_x - target_wp['x'])**2 + (curr_y - target_wp['y'])**2
#             )
            
#             if distance_to_target <= self.WAYPOINT_REACHED_DISTANCE:
#                 # Reached current waypoint
#                 self.reached_waypoints.append(self.current_waypoint_index)
#                 self.current_waypoint_index += 1
                
#                 self.get_logger().info(
#                     f'✅ Reached waypoint {len(self.reached_waypoints)}/{len(self.selected_waypoints)}: '
#                     f'CSV#{target_wp["index"]} ({target_wp["lat"]:.6f}, {target_wp["lon"]:.6f}) '
#                     f'[matched from route point {target_wp.get("route_index", "?")}]'
#                 )
                
#                 # Check if we've reached the final waypoint
#                 if self.current_waypoint_index >= len(self.selected_waypoints):
#                     self.get_logger().info('🎉 All matched waypoints reached!')
#                     return None
        
#         # Return current target waypoint
#         if self.current_waypoint_index < len(self.selected_waypoints):
#             target_wp = self.selected_waypoints[self.current_waypoint_index]
#             self.distance_to_next_waypoint = math.sqrt(
#                 (curr_x - target_wp['x'])**2 + (curr_y - target_wp['y'])**2
#             )
#             return target_wp
        
#         return None
    
#     def check_off_track_warning(self):
#         """🛣️ Check if vehicle is off track and issue warnings"""
#         if not self.final_path:
#             return 0.0
        
#         # Find closest point on path
#         _, closest_point = self.find_closest_point_on_final_path()
        
#         # Calculate distance to path
#         curr_x, curr_y = self.current_pos['x'], self.current_pos['y']
#         distance_to_path = math.sqrt(
#             (curr_x - closest_point[0])**2 + (curr_y - closest_point[1])**2
#         )
        
#         # Check if off track
#         if distance_to_path > self.OFF_TRACK_WARNING_DISTANCE:
#             self.consecutive_off_track_count += 1
            
#             # Issue warning if consistently off track
#             current_time = time.time()
#             if (self.consecutive_off_track_count >= self.MAX_OFF_TRACK_COUNT and 
#                 current_time - self.last_warning_time > self.WARNING_INTERVAL):
                
#                 self.get_logger().warning(
#                     f'⚠️ OFF TRACK WARNING! Distance to matched path: {distance_to_path:.1f}m '
#                     f'(Max allowed: {self.OFF_TRACK_WARNING_DISTANCE:.1f}m)'
#                 )
#                 self.last_warning_time = current_time
#         else:
#             # Reset counter when back on track
#             if self.consecutive_off_track_count > 0:
#                 self.get_logger().info(f'✅ Back on matched path! Distance: {distance_to_path:.1f}m')
#             self.consecutive_off_track_count = 0
        
#         return distance_to_path
    
#     def find_closest_point_on_final_path(self):
#         """🛣️ Find closest point on final path"""
#         if not self.final_path:
#             return 0.0, (self.current_pos['x'], self.current_pos['y'])
        
#         curr_x, curr_y = self.current_pos['x'], self.current_pos['y']
        
#         min_distance = float('inf')
#         closest_idx = 0
#         closest_x, closest_y = 0.0, 0.0
        
#         # Find closest point on path
#         for i, (px, py) in enumerate(zip(self.final_path['x'], self.final_path['y'])):
#             distance = math.sqrt((curr_x - px)**2 + (curr_y - py)**2)
#             if distance < min_distance:
#                 min_distance = distance
#                 closest_idx = i
#                 closest_x, closest_y = px, py
        
#         # Get s-coordinate
#         closest_s = self.final_path['s'][closest_idx]
#         self.current_path_s = closest_s
        
#         # Calculate cross-track error with sign
#         if closest_idx < len(self.final_path['x']) - 1:
#             # Path direction vector
#             next_x = self.final_path['x'][closest_idx + 1]
#             next_y = self.final_path['y'][closest_idx + 1]
#             path_dx = next_x - closest_x
#             path_dy = next_y - closest_y
#         elif closest_idx > 0:
#             # Use previous point
#             prev_x = self.final_path['x'][closest_idx - 1]
#             prev_y = self.final_path['y'][closest_idx - 1]
#             path_dx = closest_x - prev_x
#             path_dy = closest_y - prev_y
#         else:
#             path_dx, path_dy = 1.0, 0.0
        
#         # Normalize path direction
#         path_length = math.sqrt(path_dx**2 + path_dy**2)
#         if path_length > 0:
#             path_dx /= path_length
#             path_dy /= path_length
        
#         # Vector from closest point to vehicle
#         to_vehicle_x = curr_x - closest_x
#         to_vehicle_y = curr_y - closest_y
        
#         # Cross product for signed distance
#         cross_product = to_vehicle_x * path_dy - to_vehicle_y * path_dx
#         self.cross_track_error = cross_product
        
#         return closest_s, (closest_x, closest_y)
    
#     def find_final_path_lookahead_point(self):
#         """🛣️ Find lookahead point on final path"""
#         if not self.final_path:
#             return None
        
#         # Get current position on path
#         current_s, _ = self.find_closest_point_on_final_path()
        
#         # Calculate dynamic lookahead distance
#         current_speed = max(0.1, self.target_speed)
#         lookahead_dist = max(self.MIN_LOOKAHEAD, 
#                            min(self.MAX_LOOKAHEAD, 
#                                current_speed * self.SPEED_TO_LOOKAHEAD_RATIO))
        
#         # Find lookahead point
#         target_s = current_s + lookahead_dist
#         target_s = min(target_s, self.final_path['total_distance'])
        
#         # Interpolate position at target_s
#         s_array = self.final_path['s']
#         x_array = self.final_path['x']
#         y_array = self.final_path['y']
        
#         # Find interpolation index
#         idx = np.searchsorted(s_array, target_s)
        
#         if idx >= len(x_array):
#             # Use final point
#             lookahead_x = x_array[-1]
#             lookahead_y = y_array[-1]
#         elif idx == 0:
#             # Use first point
#             lookahead_x = x_array[0]
#             lookahead_y = y_array[0]
#         else:
#             # Linear interpolation
#             s_prev = s_array[idx-1]
#             s_next = s_array[idx]
#             t = (target_s - s_prev) / (s_next - s_prev) if s_next != s_prev else 0.0
            
#             lookahead_x = x_array[idx-1] + t * (x_array[idx] - x_array[idx-1])
#             lookahead_y = y_array[idx-1] + t * (y_array[idx] - y_array[idx-1])
        
#         return (lookahead_x, lookahead_y)
    
#     def calculate_simple_steering(self):
#         """🛣️ Simple steering calculation"""
#         lookahead_point = self.find_final_path_lookahead_point()
        
#         if not lookahead_point:
#             return 0.0
        
#         self.lookahead_point = lookahead_point
        
#         # Current position
#         curr_x, curr_y = self.current_pos['x'], self.current_pos['y']
#         target_x, target_y = lookahead_point
        
#         # Vector to lookahead point
#         dx = target_x - curr_x
#         dy = target_y - curr_y
#         lookahead_distance = math.sqrt(dx*dx + dy*dy)
        
#         if lookahead_distance < 0.1:
#             return 0.0
        
#         # Calculate target angle
#         target_angle_rad = math.atan2(dy, dx)
#         target_angle_deg = math.degrees(target_angle_rad)
#         if target_angle_deg < 0:
#             target_angle_deg += 360
        
#         # Current heading
#         current_heading = self.simple_heading
        
#         # Calculate angle difference
#         angle_diff = target_angle_deg - current_heading
        
#         # Normalize angle difference
#         while angle_diff > 180:
#             angle_diff -= 360
#         while angle_diff < -180:
#             angle_diff += 360
        
#         # Simple proportional steering
#         steering_gain = 0.03
#         max_steering_deg = math.degrees(self.MAX_STEERING_ANGLE)
        
#         steering_angle_deg = angle_diff * steering_gain
#         steering_angle_deg = max(-max_steering_deg, min(max_steering_deg, steering_angle_deg))
        
#         steering_angle = math.radians(steering_angle_deg)
        
#         # Direction description
#         direction = "straight"
#         if abs(steering_angle_deg) > 3:
#             direction = "left" if steering_angle_deg > 0 else "right"
        
#         self.get_logger().debug(
#             f'🛣️ Hybrid Navigation:\n'
#             f'  📍 Vehicle: ({curr_x:.1f}, {curr_y:.1f})\n'
#             f'  🎯 Target: ({target_x:.1f}, {target_y:.1f})\n'
#             f'  📏 Distance: {lookahead_distance:.1f}m\n'
#             f'  🧭 Target Angle: {target_angle_deg:.1f}°\n'
#             f'  📐 Angle Diff: {angle_diff:+.1f}°\n'
#             f'  🎛️ Steering: {steering_angle_deg:+.1f}°\n'
#             f'  🚗 Command: {direction}'
#         )
        
#         return steering_angle
    
#     def calculate_progress(self):
#         """Calculate progress along final path"""
#         if not self.final_path or self.final_path['total_distance'] == 0:
#             return 0.0
        
#         progress = (self.current_path_s / self.final_path['total_distance']) * 100
#         self.path_progress = progress / 100.0
#         return progress
    
#     # Coordinate system functions
#     def init_coordinate_system(self):
#         """Initialize coordinate system"""
#         try:
#             if self.transformer:
#                 self.reference_utm = self.transformer.transform(self.ref_lon, self.ref_lat)
#                 self.get_logger().info(
#                     f'🛣️ Hybrid coordinate system initialized:\n'
#                     f'  📍 Reference: ({self.ref_lat:.6f}, {self.ref_lon:.6f})\n'
#                     f'  📍 UTM: ({self.reference_utm[0]:.2f}, {self.reference_utm[1]:.2f})'
#                 )
#             else:
#                 self.reference_utm = None
#         except Exception as e:
#             self.get_logger().error(f'❌ Coordinate system init failed: {e}')
#             self.reference_utm = None
    
#     def latlon_to_xy(self, lat, lon):
#         """Convert lat/lon to local coordinates"""
#         try:
#             if self.transformer and self.reference_utm:
#                 utm_x, utm_y = self.transformer.transform(lon, lat)
#                 local_x = utm_x - self.reference_utm[0]
#                 local_y = utm_y - self.reference_utm[1]
#                 return local_x, local_y
#             else:
#                 return self.simple_latlon_to_xy(lat, lon)
#         except Exception as e:
#             return self.simple_latlon_to_xy(lat, lon)
    
#     def simple_latlon_to_xy(self, lat, lon):
#         """Simple conversion"""
#         try:
#             dlat = float(lat) - self.ref_lat
#             dlon = float(lon) - self.ref_lon
            
#             x = dlon * 111319.9 * math.cos(math.radians(lat))
#             y = dlat * 111319.9
            
#             return float(x), float(y)
#         except Exception as e:
#             return 0.0, 0.0
    
#     def xy_to_latlon(self, x, y):
#         """Convert local coordinates to lat/lon"""
#         try:
#             if self.transformer and self.reference_utm:
#                 utm_x = float(x) + self.reference_utm[0]
#                 utm_y = float(y) + self.reference_utm[1]
                
#                 lon, lat = self.transformer.transform(utm_x, utm_y, direction='INVERSE')
#                 return float(lat), float(lon)
#             else:
#                 return self.simple_xy_to_latlon(x, y)
#         except Exception as e:
#             return self.simple_xy_to_latlon(x, y)
    
#     def simple_xy_to_latlon(self, x, y):
#         """Simple inverse conversion"""
#         try:
#             dlat = float(y) / 111319.9
#             dlon = float(x) / (111319.9 * math.cos(math.radians(self.ref_lat + dlat)))
            
#             lat = self.ref_lat + dlat
#             lon = self.ref_lon + dlon
            
#             return float(lat), float(lon)
#         except Exception as e:
#             return self.ref_lat, self.ref_lon
    
#     # ROS callbacks
#     def gnss_callback(self, msg):
#         """Handle GNSS position updates"""
#         try:
#             lat = float(msg.latitude)
#             lon = float(msg.longitude)
            
#             self.current_lat = lat
#             self.current_lon = lon
            
#             x, y = self.latlon_to_xy(lat, lon)
#             self.current_pos = {'x': float(x), 'y': float(y)}
            
#             # Update distance traveled
#             if self.last_position:
#                 dx = x - self.last_position['x']
#                 dy = y - self.last_position['y']
#                 distance = math.sqrt(dx*dx + dy*dy)
#                 if distance < 10.0:
#                     self.total_distance += distance
            
#             self.last_position = {'x': float(x), 'y': float(y)}
            
#             # Send status to frontend
#             if self.loop:
#                 asyncio.run_coroutine_threadsafe(self.send_status(), self.loop)
                
#         except Exception as e:
#             self.get_logger().error(f'❌ GNSS callback failed: {e}')
    
#     def xy_callback(self, msg):
#         """Handle XY position updates"""
#         try:
#             self.current_pos = {'x': float(msg.x), 'y': float(msg.y)}
#         except Exception as e:
#             self.get_logger().debug(f'XY callback failed: {e}')
    
#     def heading_callback(self, msg):
#         """Handle heading updates (simplified)"""
#         try:
#             raw_heading = float(msg.data)
#             self.current_heading = raw_heading
#             self.simple_heading = raw_heading
#             self.heading_source = "GPS"
#         except Exception as e:
#             self.get_logger().error(f'❌ Heading callback failed: {e}')
#             self.simple_heading = 0.0
#             self.heading_source = "ERROR"
    
#     # Navigation control
#     def start_hybrid_navigation(self):
#         """🛣️ Start hybrid waypoint navigation"""
#         try:
#             if not self.route_matched or not self.final_path:
#                 self.get_logger().error('❌ No matched route available! Please send route first.')
#                 return False
            
#             self.navigation_active = True
#             self.current_waypoint_index = 0
#             self.reached_waypoints = []
#             self.current_path_s = 0.0
#             self.start_time = time.time()
#             self.total_distance = 0.0
#             self.consecutive_off_track_count = 0
            
#             self.get_logger().info(
#                 f'🚀 Hybrid Waypoint Navigation started!\n'
#                 f'  📊 Selected waypoints: {len(self.selected_waypoints)}\n'
#                 f'  📏 Total distance: {self.final_path["total_distance"]:.1f}m\n'
#                 f'  🎯 Algorithm: Route-to-CSV-Waypoint Matching\n'
#                 f'  🛣️ First target: CSV#{self.selected_waypoints[0]["index"]}'
#             )
            
#             if self.loop:
#                 asyncio.run_coroutine_threadsafe(
#                     self.broadcast_message({
#                         'navigation_started': True, 
#                         'hybrid_navigation': True,
#                         'matched_waypoints': len(self.selected_waypoints),
#                         'algorithm': 'Hybrid Route-CSV Matching'
#                     }), 
#                     self.loop
#                 )
#             return True
#         except Exception as e:
#             self.get_logger().error(f'❌ Start hybrid navigation failed: {e}')
#             return False
    
#     def pause_navigation(self):
#         """Pause navigation"""
#         self.navigation_active = False
#         self.stop_robot()
#         self.get_logger().info('⏸️ Hybrid navigation paused')
    
#     def stop_navigation(self):
#         """Stop navigation"""
#         self.navigation_active = False
#         self.current_waypoint_index = 0
#         self.reached_waypoints = []
#         self.current_path_s = 0.0
#         self.stop_robot()
#         self.get_logger().info('🛑 Hybrid navigation stopped')
    
#     def stop_robot(self):
#         """Send stop command"""
#         try:
#             twist = Twist()
#             twist.linear.x = 0.0
#             twist.angular.z = 0.0
#             self.cmd_vel_pub.publish(twist)
#         except Exception as e:
#             self.get_logger().error(f'❌ Stop robot failed: {e}')
    
#     def control_loop(self):
#         """🛣️ Main control loop for hybrid navigation"""
#         if not self.navigation_active or not self.route_matched:
#             return
        
#         try:
#             # Check if all waypoints reached
#             if self.current_waypoint_index >= len(self.selected_waypoints):
#                 self.navigation_active = False
#                 self.stop_robot()
                
#                 if self.loop:
#                     asyncio.run_coroutine_threadsafe(
#                         self.broadcast_message({
#                             'arrived': True,
#                             'total_time': time.time() - self.start_time,
#                             'distance_traveled': self.total_distance,
#                             'average_speed': self.total_distance / max(1, time.time() - self.start_time),
#                             'algorithm': 'Hybrid Route-CSV Matching',
#                             'waypoints_reached': len(self.reached_waypoints),
#                             'total_waypoints': len(self.selected_waypoints),
#                             'csv_waypoints_used': len(self.selected_waypoints)
#                         }), 
#                         self.loop
#                     )
                
#                 self.get_logger().info(
#                     f'🎉 All matched waypoints reached!\n'
#                     f'  ✅ Completed: {len(self.reached_waypoints)}/{len(self.selected_waypoints)} waypoints\n'
#                     f'  📏 Distance: {self.total_distance:.1f}m\n'
#                     f'  ⏱️ Time: {time.time() - self.start_time:.1f}s\n'
#                     f'  🛣️ Method: Route matched to CSV waypoints'
#                 )
#                 return
            
#             # Find current target waypoint
#             target_waypoint = self.find_current_waypoint_target()
            
#             # Check for off-track warnings
#             distance_to_path = self.check_off_track_warning()
            
#             # Calculate steering
#             steering_angle = self.calculate_simple_steering()
            
#             # Speed control based on off-track distance
#             speed = self.target_speed
            
#             # Reduce speed if off track
#             if distance_to_path > 3.0:
#                 speed *= 0.6
#             elif distance_to_path > 1.5:
#                 speed *= 0.8
            
#             # Reduce speed based on steering angle
#             try:
#                 steering_deg = abs(math.degrees(steering_angle))
#                 if steering_deg > 20:
#                     speed *= 0.5
#                 elif steering_deg > 10:
#                     speed *= 0.7
#             except:
#                 pass
            
#             speed = max(0.1, min(self.target_speed, speed))
            
#             # Publish control commands
#             twist = Twist()
#             twist.linear.x = float(speed)
#             twist.angular.z = float(steering_angle)
#             self.cmd_vel_pub.publish(twist)
            
#             # Debug logging
#             current_time = time.time()
#             if not hasattr(self, '_last_log_time') or current_time - self._last_log_time > 1.0:
#                 self._last_log_time = current_time
#                 progress = self.calculate_progress()
                
#                 steering_deg = math.degrees(steering_angle) if steering_angle is not None else 0.0
                
#                 target_info = ""
#                 if target_waypoint:
#                     target_info = f"CSV#{target_waypoint['index']} (route pt {target_waypoint.get('route_index', '?')})"
                
#                 self.get_logger().info(
#                     f'🛣️ Hybrid Navigation:\n'
#                     f'  🎯 Target: {self.current_waypoint_index + 1}/{len(self.selected_waypoints)} - {target_info}\n'
#                     f'  📏 Distance to waypoint: {self.distance_to_next_waypoint:.1f}m\n'
#                     f'  📐 Cross-track error: {self.cross_track_error:+.2f}m\n'
#                     f'  📏 Distance to path: {distance_to_path:.1f}m\n'
#                     f'  🎛️ Steering: {steering_deg:+6.2f}° | Speed: {speed:.2f}m/s\n'
#                     f'  📊 Progress: {progress:.1f}%\n'
#                     f'  ⚠️ Off-track count: {self.consecutive_off_track_count}'
#                 )
        
#         except Exception as e:
#             self.get_logger().error(f'❌ Hybrid control loop error: {e}')
#             self.stop_robot()
    
#     # WebSocket Server
#     def run_websocket_server(self):
#         """Run WebSocket server"""
#         try:
#             self.loop = asyncio.new_event_loop()
#             asyncio.set_event_loop(self.loop)
            
#             start_server = websockets.serve(self.handle_websocket, 'localhost', 5001)
#             self.loop.run_until_complete(start_server)
#             self.get_logger().info('🌐 Hybrid Waypoint WebSocket server started on port 5001')
#             self.loop.run_forever()
            
#         except Exception as e:
#             self.get_logger().error(f'❌ WebSocket server error: {e}')
    
#     async def handle_websocket(self, websocket, path):
#         """Handle WebSocket connections"""
#         try:
#             self.connections.add(websocket)
#             self.get_logger().info('🔗 New client connected to Hybrid Waypoint system')
            
#             async for message in websocket:
#                 try:
#                     data = json.loads(message)
#                     await self.handle_message(data)
#                 except json.JSONDecodeError:
#                     self.get_logger().error('❌ Invalid JSON received')
                    
#         except websockets.exceptions.ConnectionClosed:
#             pass
#         finally:
#             self.connections.discard(websocket)
#             self.get_logger().info('🔌 Client disconnected')
    
#     async def handle_message(self, data):
#         """Handle incoming WebSocket messages"""
#         if data.get('path'):
#             # Route data received from web
#             await self.process_route_data(data)
#         elif data.get('navigation_command'):
#             cmd = data['navigation_command']
#             if cmd == 'start':
#                 success = self.start_hybrid_navigation()
#                 await self.broadcast_message({
#                     'command_result': 'start',
#                     'success': success,
#                     'message': 'Hybrid navigation started' if success else 'Failed to start - no matched route'
#                 })
#             elif cmd == 'stop':
#                 self.stop_navigation()
#                 await self.broadcast_message({
#                     'command_result': 'stop',
#                     'success': True,
#                     'message': 'Navigation stopped'
#                 })
#             elif cmd == 'pause':
#                 self.pause_navigation()
#                 await self.broadcast_message({
#                     'command_result': 'pause',
#                     'success': True,
#                     'message': 'Navigation paused'
#                 })
    
#     async def process_route_data(self, data):
#         """🛣️ Process route data and match to CSV waypoints"""
#         try:
#             route_points = data.get('path', [])
#             if len(route_points) < 2:
#                 await self.broadcast_message({
#                     'route_received': False,
#                     'error': 'Route too short (need at least 2 points)'
#                 })
#                 return
            
#             self.get_logger().info(f'🛣️ Processing route with hybrid matching: {len(route_points)} points')
            
#             # Try sequential matching first, then fallback to regular matching
#             matching_success = self.match_route_to_sequential_waypoints(route_points)
            
#             if not matching_success:
#                 self.get_logger().info('🔄 Sequential matching failed, trying regular matching...')
#                 matching_success = self.match_route_to_waypoints(route_points)
            
#             if matching_success:
#                 # Ensure all waypoints have required fields
#                 self.selected_waypoints = self.ensure_waypoint_fields(self.selected_waypoints, self.route_waypoints)
                
#                 # Send confirmation to frontend
#                 response = {
#                     'route_received': True,
#                     'route_matched': True,
#                     'original_points': len(route_points),
#                     'matched_waypoints': len(self.selected_waypoints),
#                     'csv_database_size': len(self.csv_waypoints_db),
#                     'interpolated_points': len(self.final_path["x"]) if self.final_path else 0,
#                     'total_distance': self.final_path['total_distance'] if self.final_path else 0,
#                     'algorithm': 'Hybrid Route-to-CSV Waypoint Matching',
#                     'coordinate_system': 'UTM Zone 47N (Hybrid)',
#                     'matching_distance': self.WAYPOINT_MATCHING_DISTANCE,
#                     'coverage_percentage': len(self.selected_waypoints)/len(route_points)*100,
#                     'average_matching_distance': np.mean([wp["distance_to_route"] for wp in self.selected_waypoints]) if self.selected_waypoints else 0
#                 }
                
#                 await self.broadcast_message(response)
                
#                 self.get_logger().info(
#                     f'✅ Route matching completed and sent to frontend:\n'
#                     f'  📊 Matching success: {len(self.selected_waypoints)}/{len(route_points)} points\n'
#                     f'  📏 Total distance: {response["total_distance"]:.1f}m\n'
#                     f'  🎯 Ready for hybrid navigation'
#                 )
#             else:
#                 await self.broadcast_message({
#                     'route_received': False,
#                     'route_matched': False,
#                     'error': 'Failed to match route to CSV waypoints',
#                     'csv_database_size': len(self.csv_waypoints_db),
#                     'matching_distance': self.WAYPOINT_MATCHING_DISTANCE
#                 })
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Route processing error: {e}')
#             await self.broadcast_message({
#                 'route_received': False,
#                 'error': f'Route processing failed: {str(e)}'
#             })
    
#     async def send_status(self):
#         """🛣️ Send status with hybrid navigation info"""
#         if not self.connections:
#             return
        
#         try:
#             progress = self.calculate_progress()
            
#             # Calculate distance to destination (last waypoint)
#             distance_to_dest = None
#             if self.selected_waypoints and len(self.selected_waypoints) > 0:
#                 last_wp = self.selected_waypoints[-1]
#                 curr_x, curr_y = self.current_pos['x'], self.current_pos['y']
#                 distance_to_dest = math.sqrt((curr_x - last_wp['x'])**2 + (curr_y - last_wp['y'])**2)
            
#             status = {
#                 'latitude': self.current_lat,
#                 'longitude': self.current_lon,
#                 'current_speed': self.target_speed if self.navigation_active else 0.0,
#                 'navigation_active': self.navigation_active,
#                 'route_progress': progress,
#                 'distance_to_destination': distance_to_dest,
#                 'distance_traveled': self.total_distance,
#                 'cross_track_error': self.cross_track_error,
#                 'current_heading': self.current_heading,
#                 'simple_heading': self.simple_heading,
#                 'heading_source': self.heading_source,
#                 'algorithm': 'Hybrid Route-CSV Matching',
#                 'coordinate_system': 'UTM Zone 47N (Hybrid)',
#                 'wheelbase': self.WHEELBASE,
#                 'lookahead_distance': self.LOOKAHEAD_DISTANCE,
#                 'current_path_s': self.current_path_s,
#                 'path_length': self.path_length if hasattr(self, 'path_length') else 0,
                
#                 # 🛣️ Hybrid-specific status
#                 'hybrid_navigation': {
#                     'csv_database_loaded': self.csv_loaded,
#                     'csv_database_size': len(self.csv_waypoints_db),
#                     'route_matched': self.route_matched,
#                     'selected_waypoints': len(self.selected_waypoints),
#                     'current_waypoint_index': self.current_waypoint_index,
#                     'reached_waypoints': len(self.reached_waypoints),
#                     'distance_to_next_waypoint': self.distance_to_next_waypoint,
#                     'off_track_warnings': self.consecutive_off_track_count,
#                     'matching_distance': self.WAYPOINT_MATCHING_DISTANCE,
#                     'off_track_threshold': self.OFF_TRACK_WARNING_DISTANCE
#                 }
#             }
            
#             # Add lookahead point
#             if self.lookahead_point:
#                 try:
#                     lookahead_lat, lookahead_lon = self.xy_to_latlon(
#                         self.lookahead_point[0], self.lookahead_point[1]
#                     )
#                     status['lookahead_lat'] = lookahead_lat
#                     status['lookahead_lon'] = lookahead_lon
#                     status['lookahead_distance'] = math.sqrt(
#                         (self.lookahead_point[0] - self.current_pos['x'])**2 + 
#                         (self.lookahead_point[1] - self.current_pos['y'])**2
#                     )
#                 except:
#                     pass
            
#             # Add current target waypoint info
#             if (self.current_waypoint_index < len(self.selected_waypoints) and 
#                 self.current_waypoint_index >= 0):
#                 target_wp = self.selected_waypoints[self.current_waypoint_index]
#                 status['current_target_waypoint'] = {
#                     'csv_index': target_wp['index'],
#                     'route_index': target_wp.get('route_index', -1),
#                     'lat': target_wp['lat'],
#                     'lon': target_wp['lon'],
#                     'distance': self.distance_to_next_waypoint,
#                     'matching_distance': target_wp.get('distance_to_route', 0)
#                 }
            
#             await self.broadcast_message(status)
            
#         except Exception as e:
#             self.get_logger().debug(f'Status send error: {e}')
    
#     async def broadcast_message(self, message):
#         """Broadcast message to all connected clients"""
#         if not self.connections:
#             return
        
#         try:
#             json_message = json.dumps(message, default=str)
#             disconnected = set()
            
#             for websocket in self.connections:
#                 try:
#                     await websocket.send(json_message)
#                 except websockets.exceptions.ConnectionClosed:
#                     disconnected.add(websocket)
            
#             for websocket in disconnected:
#                 self.connections.discard(websocket)
                
#         except Exception as e:
#             self.get_logger().debug(f'Broadcast error: {e}')


# def main(args=None):
#     """Main function"""
#     rclpy.init(args=args)
#     navigation = None
    
#     try:
#         # 🔧 แก้ไข: เปลี่ยนจาก FixedPathProgressionPurePursuitGolfCart เป็น HybridWaypointNavigationSystem
#         navigation = HybridWaypointNavigationSystem()
#         print('🚀 Hybrid Waypoint Navigation System running...')
#         print('🛣️ Matches web routes to CSV waypoints for precision navigation!')
#         print('📍 Automatically selects best waypoints from database!')
#         print('🎯 Combines route planning flexibility with waypoint accuracy!')
#         rclpy.spin(navigation)
        
#     except KeyboardInterrupt:
#         print('🛑 Keyboard interrupt received')
#     except Exception as e:
#         print(f'❌ Error: {e}')
#     finally:
#         if navigation:
#             navigation.destroy_node()
#         rclpy.shutdown()
#         print('🏁 Shutdown complete')

# if __name__ == '__main__':
#     main()



#!/usr/bin/env python3
#!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64MultiArray, Float64
# from geometry_msgs.msg import Twist  # เพิ่มสำหรับ cmd_vel
# import csv
# import math
# import time
# import json
# import asyncio
# import websockets
# import threading
# from datetime import datetime

# class NavigationController(Node):
#     def __init__(self):
#         super().__init__('enhanced_lateral_control')
        
#         # เก็บตัวแปรเดิมทั้งหมด
#         self.value = []
#         self.name = 'Data Column Name'
#         self.column = 0
#         self.csv_name = 0
#         self.Data = []
        
#         # Realtime XY callback Function
#         self.time = 0.0
#         self.x_east = 0.0
#         self.y_north = 0.0
#         self.xy = 0.0
#         self.start = 0
#         self.time_start = 0.0
#         self.time_operate = 0.0
        
#         # เพิ่มตัวแปรสำหรับ WebSocket
#         self.websocket_clients = set()
#         self.navigation_active = False
#         self.current_speed = 0.0
#         self.route_progress = 0.0
#         self.distance_to_destination = 0.0
#         self.distance_traveled = 0.0
#         self.cross_track_error = 0.0
#         self.lookahead_lat = 0.0
#         self.lookahead_lon = 0.0
#         self.lookahead_distance = 3.0
#         self.current_heading = 0.0
        
#         # เพิ่มตัวแปรสำหรับเส้นทาง
#         self.current_route = []
#         self.destination_lat = 0.0
#         self.destination_lon = 0.0
        
#         # Track parameters - แก้ไขให้เป็น float
#         self.Track = None
#         self.linear = 0
#         self.curve = 0
#         self.reference = 0.0
#         self.max_right = 0.0
#         self.max_left = 0.0
#         self.speed = 0.0
#         self.a, self.b, self.c = 1.0, 1.0, 1.0
#         self.aL, self.bL, self.cL = 0.0, 0.0, 0.0
#         self.aR, self.bR, self.cR = 0.0, 0.0, 0.0
        
#         # เก็บพิกัดเส้นทางเดิมทั้งหมด
#         self.init_track_parameters()
        
#         # เพิ่ม GPS conversion parameters
#         self.REFERENCE_LAT = 13.650748
#         self.REFERENCE_LON = 100.492985
        
#         # PID parameters - แก้ไขให้เป็น float ทั้งหมด
#         self.init_pid_parameters()
        
#         # ROS2 Publishers - เพิ่ม cmd_vel publisher
#         self.steering_angle_want = self.create_publisher(Float64, 'Steering_Angle_Want', 10)
#         self.cte_pub = self.create_publisher(Float64, 'CTE', 10)
#         self.speed_pub = self.create_publisher(Float64, 'Speed_Setpoint', 10)
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)  # ✅ เพิ่ม cmd_vel publisher
        
#         # ROS2 Subscriber
#         self.realtime_xy_sub = self.create_subscription(
#             Float64MultiArray,
#             'Realtime_XY',
#             self.realtime_xy_callback,
#             10
#         )
        
#         # เริ่ม WebSocket Server
#         self.start_websocket_server()
        
#         self.get_logger().info("Enhanced Navigation Controller with cmd_vel initialized")
    
#     def init_track_parameters(self):
#         """เก็บพารามิเตอร์เส้นทางเดิมทั้งหมด"""
#         # Linear S-N
#         self.SN1_start = 1509522.50
#         self.SN1_stop = 1509583.749
#         self.SN1_maxright = 661489.88
#         self.SN1_maxleft = 661484.173410818
#         self.SN1_ref = 661487.31726504
#         self.aSN1, self.bSN1, self.cSN1 = -78.53077, -1.0, 53456668.62524
        
#         self.SN2_start = 1509583.7491
#         self.SN2_stop = 1509685.06490994
#         self.SN2_maxright = 661489.88
#         self.SN2_maxleft = 661484.173410818
#         self.SN2_ref = 661486.8775
#         self.aSN2, self.bSN2, self.cSN2 = -272.2586713752698, -1.0, 181605145.8698626
        
#         # Curve1 E-W
#         self.curve1_s1_start = 1509685.065
#         self.curve1_s1_stop = 1509694.17
#         self.curve1_s1_maxright = 661488.735
#         self.curve1_s1_maxleft = 661482.46
#         self.aL1, self.bL1, self.cL1 = -2.0882352939222786, -1.0, 2891024.6175178257
#         self.a1, self.b1, self.c1 = -3.7748691098637175, -1.0, 4006714.14565129
#         self.aR1, self.bR1, self.cR1 = -4.341614906864185, -1.0, 4381619.400890658
        
#         # Linear E-W
#         self.EW_start = 661478.34
#         self.EW_stop = 661433.35
#         self.EW_maxright = 1509703.9
#         self.EW_maxleft = 1509696.5
#         self.EW_ref = 1509700.0
#         self.aEW, self.bEW, self.cEW = -0.06666666666666667, -1.0, 1553796.9466666665
        
#         # Linear N-S
#         self.NS_start = 1509653.566
#         self.NS_stop = 1509560.129
#         self.NS_maxright = 661375.26
#         self.NS_maxleft = 661383.26
#         self.NS_ref = 661379.26
#         self.aNS, self.bNS, self.cNS = 7786.41667453521, -1.0, -5148264891.408258
        
#         # Linear W-E
#         self.WE_start = 661436.79
#         self.WE_stop = 661466.07
#         self.WE_maxright = 1509506.53
#         self.WE_maxleft = 1509512.5
#         self.WE_ref = 1509509.63273
#         self.aWE, self.bWE, self.cWE = 0.012978142080676918, -1.0, 1500925.219361993
        
#         # ✅ เพิ่มเส้นทางสำหรับพิกัดปัจจุบัน
#         self.CURRENT_AREA_X_MIN = 661430.0
#         self.CURRENT_AREA_X_MAX = 661440.0
#         self.CURRENT_AREA_Y_MIN = 1509440.0
#         self.CURRENT_AREA_Y_MAX = 1509450.0
#         self.CURRENT_AREA_REF = 1509445.0
    
#     def init_pid_parameters(self):
#         """เริ่มต้นพารามิเตอร์ PID - แก้ไขให้เป็น float ทั้งหมด"""
#         self.cte_prev = 0.0
#         self.sum_error_cte = 0.0
#         self.error_cte_prev = 0.0
#         self.yaw_prev = 0.0
#         self.yaw_expect = 0.0
#         self.kp = 0.0
#         self.ki = 0.0
#         self.kd = 0.0
#         self.P = 0.0
#         self.I = 0.0
#         self.D = 0.0
        
#         # Control variables
#         self.yaw_control = 0.0
#         self.yaw = 0.0
#         self.cte = 0.0
#         self.distance = 0.0
#         self.error_track = 0.0
#         self.acceprtable_error = 0.0
    
#     def start_websocket_server(self):
#         """เริ่ม WebSocket Server ในเธรดแยก"""
#         def run_server():
#             loop = asyncio.new_event_loop()
#             asyncio.set_event_loop(loop)
#             start_server = websockets.serve(self.websocket_handler, "localhost", 5001)
#             loop.run_until_complete(start_server)
#             loop.run_forever()
        
#         server_thread = threading.Thread(target=run_server)
#         server_thread.daemon = True
#         server_thread.start()
#         self.get_logger().info("WebSocket Server started on ws://localhost:5001")
    
#     async def websocket_handler(self, websocket, path):
#         """จัดการ WebSocket connections"""
#         self.websocket_clients.add(websocket)
#         self.get_logger().info(f"WebSocket client connected: {websocket.remote_address}")
        
#         try:
#             async for message in websocket:
#                 await self.handle_websocket_message(websocket, message)
#         except websockets.exceptions.ConnectionClosed:
#             pass
#         finally:
#             self.websocket_clients.discard(websocket)
#             self.get_logger().info("WebSocket client disconnected")
    
#     async def handle_websocket_message(self, websocket, message):
#         """จัดการข้อความจาก WebSocket"""
#         try:
#             data = json.loads(message)
            
#             if 'path' in data:
#                 # รับเส้นทางจาก web interface
#                 self.current_route = data['path']
#                 if self.current_route:
#                     self.destination_lat = self.current_route[-1]['lat']
#                     self.destination_lon = self.current_route[-1]['lon']
                
#                 # ส่งการตอบกลับ
#                 response = {
#                     'route_received': True,
#                     'smooth_points': len(self.current_route),
#                     'algorithm': 'ROS2 Path Following',
#                     'progression_segments': len(self.current_route) // 10
#                 }
#                 await websocket.send(json.dumps(response))
#                 self.get_logger().info(f"Route received: {len(self.current_route)} points")
            
#             elif 'navigation_command' in data:
#                 command = data['navigation_command']
#                 if command == 'start':
#                     self.navigation_active = True
#                     await self.broadcast_message({'navigation_started': True})
#                     self.get_logger().info("Navigation started")
#                 elif command == 'pause':
#                     self.navigation_active = False
#                     self.get_logger().info("Navigation paused")
#                 elif command == 'stop':
#                     self.navigation_active = False
#                     await self.broadcast_message({'navigation_stopped': True})
#                     self.get_logger().info("Navigation stopped")
                    
#         except json.JSONDecodeError:
#             self.get_logger().warn("Invalid JSON received from WebSocket")
    
#     async def broadcast_message(self, message):
#         """ส่งข้อความไปยัง WebSocket clients ทั้งหมด"""
#         if self.websocket_clients:
#             disconnected = set()
#             for client in self.websocket_clients:
#                 try:
#                     await client.send(json.dumps(message))
#                 except websockets.exceptions.ConnectionClosed:
#                     disconnected.add(client)
            
#             # ลบ clients ที่ disconnect แล้ว
#             self.websocket_clients -= disconnected
    
#     def xy_to_latlon(self, x, y):
#         """แปลง local XY เป็น GPS coordinates"""
#         dlat = y / 111319.9
#         dlon = x / (111319.9 * math.cos((self.REFERENCE_LAT + dlat) * math.pi / 180))
        
#         return {
#             'lat': self.REFERENCE_LAT + dlat,
#             'lon': self.REFERENCE_LON + dlon
#         }
    
#     def latlon_to_xy(self, lat, lon):
#         """แปลง GPS coordinates เป็น local XY"""
#         dlat = lat - self.REFERENCE_LAT
#         dlon = lon - self.REFERENCE_LON
        
#         x = dlon * 111319.9 * math.cos(lat * math.pi / 180)
#         y = dlat * 111319.9
        
#         return {'x': x, 'y': y}
    
#     def calculate_lookahead_point(self):
#         """คำนวณ lookahead point"""
#         if not self.current_route:
#             return
        
#         # หา lookahead point จากเส้นทาง
#         current_pos = {'x': self.x_east, 'y': self.y_north}
        
#         # ค้นหาจุดที่ใกล้ที่สุดในเส้นทาง
#         min_distance = float('inf')
#         closest_index = 0
        
#         for i, point in enumerate(self.current_route):
#             local_xy = self.latlon_to_xy(point['lat'], point['lon'])
#             distance = math.sqrt((current_pos['x'] - local_xy['x'])**2 + 
#                                (current_pos['y'] - local_xy['y'])**2)
#             if distance < min_distance:
#                 min_distance = distance
#                 closest_index = i
        
#         # หา lookahead point
#         lookahead_index = min(closest_index + 5, len(self.current_route) - 1)
#         lookahead_point = self.current_route[lookahead_index]
        
#         self.lookahead_lat = lookahead_point['lat']
#         self.lookahead_lon = lookahead_point['lon']
    
#     def calculate_progress(self):
#         """คำนวณความก้าวหน้าของการเดินทาง"""
#         if not self.current_route:
#             return 0.0
        
#         # คำนวณระยะทางรวม
#         total_distance = 0.0
#         for i in range(len(self.current_route) - 1):
#             p1 = self.latlon_to_xy(self.current_route[i]['lat'], self.current_route[i]['lon'])
#             p2 = self.latlon_to_xy(self.current_route[i+1]['lat'], self.current_route[i+1]['lon'])
#             total_distance += math.sqrt((p2['x'] - p1['x'])**2 + (p2['y'] - p1['y'])**2)
        
#         # คำนวณระยะทางที่เดินทางแล้ว
#         current_pos = {'x': self.x_east, 'y': self.y_north}
#         start_pos = self.latlon_to_xy(self.current_route[0]['lat'], self.current_route[0]['lon'])
#         traveled_distance = math.sqrt((current_pos['x'] - start_pos['x'])**2 + 
#                                     (current_pos['y'] - start_pos['y'])**2)
        
#         if total_distance > 0:
#             return min(100.0, (traveled_distance / total_distance) * 100.0)
#         return 0.0
    
#     def realtime_xy_callback(self, msg):
#         """Callback function สำหรับรับข้อมูลตำแหน่ง"""
#         self.time = msg.data[0]
#         self.x_east = msg.data[1]
#         self.y_north = msg.data[2]
#         self.xy = msg.data
        
#         if self.start == 0:
#             self.time_start = self.time
#             self.start = 1
        
#         if self.start == 1:
#             self.time_operate = round(self.time - self.time_start, 2)
        
#         # คำนวณค่าต่างๆ
#         self.select_track()
#         self.cte_current()
#         self.pid_angle()
#         self.angle_control_motor()
        
#         # คำนวณ lookahead point และ progress
#         self.calculate_lookahead_point()
#         self.route_progress = self.calculate_progress()
        
#         # คำนวณระยะทางถึงจุดหมาย
#         if self.destination_lat != 0 and self.destination_lon != 0:
#             dest_xy = self.latlon_to_xy(self.destination_lat, self.destination_lon)
#             self.distance_to_destination = math.sqrt(
#                 (self.x_east - dest_xy['x'])**2 + (self.y_north - dest_xy['y'])**2
#             )
        
#         # ส่งข้อมูลไปยัง WebSocket clients
#         asyncio.run(self.send_navigation_data())
        
#         self.talker()
#         self.create_csv()
        
#         print(round(self.x_east, 2), '||', round(self.y_north, 2), '||', 
#               self.Track, '|| cte =', round(self.cte, 2), '|| Yaw =', round(self.yaw_control, 2))
    
#     async def send_navigation_data(self):
#         """ส่งข้อมูลการนำทางไปยัง WebSocket clients"""
#         if not self.websocket_clients:
#             return
        
#         # แปลงตำแหน่งปัจจุบันเป็น GPS
#         current_gps = self.xy_to_latlon(self.x_east, self.y_north)
        
#         data = {
#             'latitude': current_gps['lat'],
#             'longitude': current_gps['lon'],
#             'current_heading': self.current_heading,
#             'navigation_active': self.navigation_active,
#             'current_speed': self.current_speed,
#             'route_progress': self.route_progress,
#             'distance_to_destination': self.distance_to_destination,
#             'distance_traveled': self.distance_traveled,
#             'cross_track_error': abs(self.cte),
#             'lookahead_lat': self.lookahead_lat,
#             'lookahead_lon': self.lookahead_lon,
#             'lookahead_distance': self.lookahead_distance,
#             'heading_source': 'GPS',
#             'nav_att_status': 'CALIBRATED',
#             'fusion_mode': 'FUSION',
#             'imu_calibration': 'CALIBRATED',
#             'heading_accuracy': 2.0
#         }
        
#         await self.broadcast_message(data)
    
#     def select_track(self):
#         """เลือกเส้นทางตามตำแหน่งปัจจุบัน - เพิ่มเส้นทางสำหรับพิกัดปัจจุบัน"""
        
#         # ✅ เพิ่มเส้นทางสำหรับพิกัดปัจจุบัน
#         if (self.CURRENT_AREA_X_MIN <= self.x_east <= self.CURRENT_AREA_X_MAX) and \
#            (self.CURRENT_AREA_Y_MIN <= self.y_north <= self.CURRENT_AREA_Y_MAX):
#             self.Track = 'current_position_track'
#             self.linear = 8
#             self.curve = 0
#             self.speed = 1.5
#             self.a = 0.0
#             self.b = 1.0
#             self.c = -self.CURRENT_AREA_REF
#             self.reference = self.CURRENT_AREA_REF
#             self.max_right = self.CURRENT_AREA_Y_MAX
#             self.max_left = self.CURRENT_AREA_Y_MIN
#             self.kp = 20.0
#             self.ki = 0.05
#             self.kd = 15.0
#             return
        
#         # Linear SN1
#         elif (self.x_east <= self.SN1_maxright) & (self.x_east >= self.SN1_maxleft) & \
#            (self.y_north >= self.SN1_start) & (self.y_north <= self.SN1_stop):
#             self.Track = 'linear_SN1'
#             self.linear = 1
#             self.curve = 0
#             self.speed = 3.0
#             self.a = self.aSN1
#             self.b = self.bSN1
#             self.c = self.cSN1
#             self.reference = self.SN1_ref
#             self.max_right = self.SN1_maxright
#             self.max_left = self.SN1_maxleft
#             self.kp = 48.0
#             self.ki = 0.17
#             self.kd = 30.0
        
#         # Linear SN2
#         elif (self.x_east <= self.SN2_maxright) & (self.x_east >= self.SN2_maxleft) & \
#              (self.y_north >= self.SN2_start) & (self.y_north <= self.SN2_stop):
#             self.Track = 'linear_SN2'
#             self.linear = 1
#             self.curve = 0
#             self.speed = 3.0
#             self.a = self.aSN2
#             self.b = self.bSN2
#             self.c = self.cSN2
#             self.reference = self.SN2_ref
#             self.max_right = self.SN2_maxright
#             self.max_left = self.SN2_maxleft
#             self.kp = 45.0
#             self.ki = 0.02
#             self.kd = 26.0
        
#         # Linear EW
#         elif (self.x_east <= self.EW_start) & (self.x_east >= self.EW_stop) & \
#              (self.y_north <= self.EW_maxright) & (self.y_north >= self.EW_maxleft):
#             self.Track = 'linear_EW'
#             self.linear = 2
#             self.curve = 0
#             self.speed = 3.0
#             self.a = self.aEW
#             self.b = self.bEW
#             self.c = self.cEW
#             self.reference = self.EW_ref
#             self.max_right = self.EW_maxright
#             self.max_left = self.EW_maxleft
#             self.kp = 45.0
#             self.ki = 0.02
#             self.kd = 26.0
        
#         # Linear NS
#         elif (self.x_east < self.NS_maxleft) & (self.x_east > self.NS_maxright) & \
#              (self.y_north < self.NS_start) & (self.y_north > self.NS_stop):
#             self.Track = 'linear_NS'
#             self.linear = 3
#             self.curve = 0
#             self.speed = 1.0
#             self.a = self.aNS
#             self.b = self.bNS
#             self.c = self.cNS
#             self.reference = self.NS_ref
#             self.max_right = self.NS_maxright
#             self.max_left = self.NS_maxleft
#             self.kp = 40.0
#             self.ki = 0.02
#             self.kd = 23.0
        
#         # Linear WE
#         elif (self.x_east >= self.WE_start) & (self.x_east <= self.WE_stop) & \
#              (self.y_north <= self.WE_maxleft) & (self.y_north >= self.WE_maxright):
#             self.Track = 'linear_WE'
#             self.linear = 4
#             self.curve = 0
#             self.speed = 3.0
#             self.a = self.aWE
#             self.b = self.bWE
#             self.c = self.cWE
#             self.reference = self.WE_ref
#             self.max_right = self.WE_maxright
#             self.max_left = self.WE_maxleft
#             self.kp = 45.0
#             self.ki = 0.02
#             self.kd = 26.0
#         else:
#             # Default case
#             self.Track = 'unknown'
#             self.speed = 0.0
    
#     def cte_current(self):
#         """คำนวณ Cross Track Error - เพิ่มสำหรับเส้นทางใหม่"""
#         self.distance = abs((self.a * self.x_east) + (self.b * self.y_north) + self.c) / \
#                        math.sqrt(self.a**2 + self.b**2)
        
#         # ✅ เพิ่มสำหรับ current_position_track (linear = 8)
#         if (self.linear == 8) & (self.curve == 0):
#             if self.distance <= 0.5:
#                 self.cte = self.acceprtable_error
#             else:
#                 if self.y_north >= self.reference:
#                     self.cte = self.distance
#                 else:
#                     self.cte = (-1.0) * self.distance
        
#         elif (self.linear == 1) & (self.curve == 0):
#             # for linear S-N
#             if self.distance <= 0.15:
#                 self.cte = self.acceprtable_error
#             if self.distance > 0.15:
#                 if (self.x_east >= self.reference) & (self.x_east <= self.max_right):
#                     self.cte = self.distance
#                 if (self.x_east < self.reference) & (self.x_east >= self.max_left):
#                     self.cte = (-1.0) * self.distance
        
#         elif (self.linear == 2) & (self.curve == 0):
#             # for linear E-W
#             if self.distance <= 0.15:
#                 self.cte = self.acceprtable_error
#             if self.distance > 0.15:
#                 if (self.y_north >= self.reference) & (self.y_north <= self.max_right):
#                     self.cte = self.distance
#                 if (self.y_north < self.reference) & (self.y_north >= self.max_left):
#                     self.cte = (-1.0) * self.distance
        
#         elif (self.linear == 3) & (self.curve == 0):
#             # for linear N-S
#             if self.distance <= 0.15:
#                 self.cte = self.acceprtable_error
#             if self.distance > 0.15:
#                 if (self.x_east <= self.reference) & (self.x_east >= self.max_right):
#                     self.cte = self.distance
#                 if (self.x_east > self.reference) & (self.x_east <= self.max_left):
#                     self.cte = (-1.0) * self.distance
        
#         elif (self.linear == 4) & (self.curve == 0):
#             # for linear W-E
#             if self.distance <= 0.15:
#                 self.cte = self.acceprtable_error
#             if self.distance > 0.15:
#                 if (self.y_north < self.reference) & (self.y_north >= self.max_right):
#                     self.cte = self.distance
#                 if (self.y_north >= self.reference) & (self.y_north <= self.max_left):
#                     self.cte = (-1.0) * self.distance
#         else:
#             # Default case
#             self.cte = 0.0
    
#     def pid_angle(self):
#         """คำนวณมุมพวงมาลัยด้วย PID Controller"""
#         self.yaw_prev = self.yaw_expect
#         self.sum_error_cte += self.cte
#         self.P = self.kp * self.cte
#         self.I = self.ki * self.sum_error_cte
#         self.D = self.kd * ((self.cte - self.cte_prev))
#         self.yaw_expect = (-1.0) * (self.P + self.I + self.D)
#         self.cte_prev = self.cte
    
#     def angle_control_motor(self):
#         """ควบคุมมุมพวงมาลัย"""
#         if self.cte > 0.15:
#             self.yaw = abs(self.yaw_control - self.yaw_prev)
#             if self.yaw >= 0.25:
#                 self.yaw_control = abs(self.yaw_expect)
#         if self.cte < -0.15:
#             self.yaw = abs(self.yaw_control - self.yaw_prev)
#             if self.yaw >= 0.25:
#                 self.yaw_control = -1.0 * (self.yaw_expect)
#         if self.cte == 0:
#             self.yaw_control = 0.0
    
#     def publish_cmd_vel(self, linear_x, angular_z):
#         """✅ ฟังก์ชันสำหรับ publish cmd_vel"""
#         cmd_vel_msg = Twist()
#         cmd_vel_msg.linear.x = float(linear_x)
#         cmd_vel_msg.linear.y = 0.0
#         cmd_vel_msg.linear.z = 0.0
#         cmd_vel_msg.angular.x = 0.0
#         cmd_vel_msg.angular.y = 0.0
#         cmd_vel_msg.angular.z = float(angular_z)
        
#         self.cmd_vel_pub.publish(cmd_vel_msg)
    
#     def talker(self):
#         """ส่งข้อมูลผ่าน ROS2 Publishers - เพิ่ม cmd_vel"""
#         # แก้ไข: เพิ่ม float() conversion เพื่อป้องกัน AssertionError
#         angle_want = Float64()
#         angle_want.data = float(self.yaw_control)
#         self.steering_angle_want.publish(angle_want)
        
#         speed_sp = Float64()
#         speed_sp.data = float(self.speed)
#         self.speed_pub.publish(speed_sp)
        
#         cte_msg = Float64()
#         cte_msg.data = float(self.cte)
#         self.cte_pub.publish(cte_msg)
        
#         # ✅ เพิ่มการ publish cmd_vel
#         if self.navigation_active and self.Track != 'unknown':
#             # คำนวณ linear velocity จาก speed (แปลงจาก m/s เป็น velocity ที่เหมาะสม)
#             linear_velocity = self.speed * 0.1  # ปรับค่าตามต้องการ
            
#             # ใช้ yaw_control เป็น angular velocity (แปลงจาก degrees เป็น rad/s)
#             angular_velocity = self.yaw_control * 0.5  # ปรับค่าตามต้องการ
            
#             # จำกัดค่าสูงสุดเพื่อความปลอดภัย
#             linear_velocity = max(-2.0, min(2.0, linear_velocity))  # จำกัด ±2 m/s
#             angular_velocity = max(-1.0, min(1.0, angular_velocity))  # จำกัด ±1 rad/s
            
#             # Publish cmd_vel
#             self.publish_cmd_vel(linear_velocity, angular_velocity)
#         else:
#             # หยุดเมื่อไม่ได้ navigate หรือ track unknown
#             self.publish_cmd_vel(0.0, 0.0)
        
#         self.current_speed = float(self.speed)  # เก็บค่าสำหรับ WebSocket
    
#     def create_csv(self):
#         """สร้างไฟล์ CSV สำหรับบันทึกข้อมูล"""
#         self.value = self.time_operate, self.Track, self.x_east, self.y_north, self.cte, self.yaw_control
#         self.name = 'time_operate,Track,x_east,y_north,cte,yaw_control'
        
#         if self.time_operate <= 1000:
#             name = 'fullway_4.csv'
#             if self.csv_name == 0:
#                 for i in range(len(self.name.split(','))):
#                     self.Data.append(self.name.split(',')[i])
#                 with open(name, 'a', newline='') as f:
#                     writer = csv.writer(f, delimiter=",")
#                     writer.writerow(self.Data)
#                 self.csv_name = 1
            
#             if self.csv_name == 1:
#                 self.Data = self.value
#                 with open(name, 'a', newline='') as f:
#                     writer = csv.writer(f, delimiter=",")
#                     writer.writerow(self.Data)
        
#         if self.time_operate > 1000:
#             print("Stop...")

# def main(args=None):
#     """Main function สำหรับ ROS2"""
#     rclpy.init(args=args)
    
#     try:
#         controller = NavigationController()
#         rclpy.spin(controller)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         if 'controller' in locals():
#             controller.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# -----------------------------------------------------Bang Bang Controller---------------------------------------------------

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32, String
# from geometry_msgs.msg import Point, Twist
# import math
# import numpy as np
# import pandas as pd
# import time
# import json

# class BangBangWaypointController:
#     def __init__(self, waypoints, final_heading=0.0, waypoint_tolerance=1.5, 
#                  heading_tolerance=np.deg2rad(8), start_from_closest=True, 
#                  search_radius=5.0):
#         """
#         🎯 Enhanced Bang-Bang Controller สำหรับรถของคุณ
        
#         Parameters:
#         - waypoints: list of (x, y) coordinates
#         - final_heading: ทิศทางสุดท้ายที่ต้องการ (radians)
#         - waypoint_tolerance: ระยะทางที่ถือว่าถึง waypoint แล้ว (meters)
#         - heading_tolerance: ความคลาดเคลื่อนทิศทางที่ยอมรับได้ (radians)
#         - start_from_closest: เริ่มจากจุดใกล้ที่สุดหรือไม่
#         - search_radius: รัศมีการค้นหา waypoint (meters)
#         """
#         self.waypoints = waypoints
#         self.final_heading = final_heading
#         self.waypoint_tolerance = waypoint_tolerance
#         self.heading_tolerance = heading_tolerance
#         self.current_target_idx = 0
#         self.start_from_closest = start_from_closest
#         self.search_radius = search_radius
#         self.initial_position_set = False
        
#         # 🌟 Vehicle parameters สำหรับรถของคุณ
#         self.wheelbase = 1.67  # เมตร (ฐานล้อ)
#         self.max_steering_angle = 0.873  # rad (50 degrees)
#         self.fixed_speed = 1.0  # m/s (ความเร็วคงที่)
        
#         # คำนวณ angular velocity สูงสุดจากพารามิเตอร์รถ
#         self.max_angular_velocity = self.fixed_speed * np.tan(self.max_steering_angle) / self.wheelbase
        
#         # สถิติการทำงาน
#         self.stats = {
#             'waypoints_reached': 0,
#             'total_distance': 0.0,
#             'mission_start_time': time.time(),
#             'left_turns': 0,
#             'right_turns': 0,
#             'straight_commands': 0,
#             'waypoint_switches': 0
#         }
        
#         print(f"🚗 Enhanced Vehicle Configuration:")
#         print(f"   Wheelbase: {self.wheelbase} m")
#         print(f"   Max Steering: ±{np.rad2deg(self.max_steering_angle):.1f}°")
#         print(f"   Max Angular Velocity: ±{self.max_angular_velocity:.3f} rad/s")
#         print(f"   Fixed Speed: {self.fixed_speed} m/s")
#         print(f"   Search Radius: {self.search_radius} m")
#         print(f"   Start from closest: {self.start_from_closest}")
    
#     def find_closest_waypoint_in_front(self, current_pos, current_heading):
#         """
#         🎯 หาจุด waypoint ที่ใกล้ที่สุดในระยะ 5 เมตร 
#         และอยู่ข้างหน้ารถตามทิศทาง heading
#         """
#         candidates = []
        
#         for i, wp in enumerate(self.waypoints):
#             # คำนวณระยะทาง
#             distance = np.hypot(wp[0] - current_pos[0], wp[1] - current_pos[1])
            
#             # กรองเฉพาะจุดที่อยู่ในรัศมี search_radius
#             if distance <= self.search_radius:
#                 # คำนวณทิศทางจากรถไป waypoint
#                 dx = wp[0] - current_pos[0]
#                 dy = wp[1] - current_pos[1]
#                 waypoint_bearing = np.arctan2(dy, dx)
                
#                 # คำนวณความแตกต่างของทิศทาง
#                 heading_diff = self.normalize_angle(waypoint_bearing - current_heading)
                
#                 # เลือกเฉพาะจุดที่อยู่ข้างหน้า (±90 องศา)
#                 if abs(heading_diff) <= np.pi/2:  # ±90 degrees
#                     # คำนวณ score: ยิ่งใกล้และอยู่ตรงหน้ายิ่งดี
#                     distance_score = 1.0 / (distance + 0.1)  # ป้องกันหารด้วย 0
#                     heading_score = 1.0 - (abs(heading_diff) / (np.pi/2))  # 0-1 scale
#                     total_score = distance_score * 0.7 + heading_score * 0.3
                    
#                     candidates.append({
#                         'index': i,
#                         'distance': distance,
#                         'heading_diff': heading_diff,
#                         'score': total_score,
#                         'waypoint': wp
#                     })
        
#         if not candidates:
#             # ถ้าไม่มีจุดในรัศมี ให้หาจุดใกล้ที่สุดทั่วไป
#             return self.find_closest_waypoint(current_pos)
        
#         # เรียงตาม score (สูงสุดก่อน)
#         candidates.sort(key=lambda x: x['score'], reverse=True)
        
#         best_candidate = candidates[0]
        
#         print(f"🎯 Selected waypoint {best_candidate['index']}:")
#         print(f"   Distance: {best_candidate['distance']:.2f}m")
#         print(f"   Heading diff: {np.rad2deg(best_candidate['heading_diff']):.1f}°")
#         print(f"   Score: {best_candidate['score']:.3f}")
        
#         return best_candidate['index']
    
#     def find_closest_waypoint(self, current_pos):
#         """หาจุด waypoint ที่ใกล้ที่สุดแบบทั่วไป (fallback)"""
#         distances = []
#         for i, wp in enumerate(self.waypoints):
#             dist = np.hypot(wp[0] - current_pos[0], wp[1] - current_pos[1])
#             distances.append((dist, i))
        
#         distances.sort()
#         return distances[0][1]
    
#     def set_initial_position(self, current_pos, current_heading):
#         """🌟 กำหนดจุดเริ่มต้นจากตำแหน่งและทิศทางปัจจุบัน"""
#         if not self.initial_position_set and self.start_from_closest:
#             closest_idx = self.find_closest_waypoint_in_front(current_pos, current_heading)
#             self.current_target_idx = closest_idx
#             self.initial_position_set = True
            
#             print(f"🚀 Starting from optimal waypoint: {closest_idx}")
#             print(f"   Distance to start: {np.hypot(
#                 self.waypoints[closest_idx][0] - current_pos[0],
#                 self.waypoints[closest_idx][1] - current_pos[1]
#             ):.2f}m")
    
#     def set_start_waypoint(self, waypoint_index):
#         """กำหนดจุดเริ่มต้นแบบระบุ index"""
#         if 0 <= waypoint_index < len(self.waypoints):
#             self.current_target_idx = waypoint_index
#             self.initial_position_set = True
#             print(f"🎯 Manual start from waypoint: {waypoint_index}")
#         else:
#             print(f"❌ Invalid waypoint index: {waypoint_index}")
    
#     def get_target_heading(self, current_pos, target_waypoint_idx):
#         """คำนวณทิศทางที่ต้องการไปยัง waypoint เป้าหมาย"""
#         if target_waypoint_idx >= len(self.waypoints):
#             return self.final_heading
            
#         target_wp = self.waypoints[target_waypoint_idx]
#         dx = target_wp[0] - current_pos[0]
#         dy = target_wp[1] - current_pos[1]
        
#         # ถ้าใกล้ waypoint สุดท้ายมาก ให้ใช้ final_heading
#         if target_waypoint_idx == len(self.waypoints) - 1:
#             distance_to_target = np.hypot(dx, dy)
#             if distance_to_target < self.waypoint_tolerance:
#                 return self.final_heading
        
#         return np.arctan2(dy, dx)
    
#     def normalize_angle(self, angle):
#         """ปรับมุมให้อยู่ในช่วง [-π, π]"""
#         return (angle + np.pi) % (2 * np.pi) - np.pi
    
#     def update_target_waypoint(self, current_pos):
#         """อัพเดท waypoint เป้าหมายถ้าถึงจุดปัจจุบันแล้ว"""
#         if self.current_target_idx < len(self.waypoints):
#             current_wp = self.waypoints[self.current_target_idx]
#             distance = np.hypot(current_wp[0] - current_pos[0], current_wp[1] - current_pos[1])
            
#             if distance < self.waypoint_tolerance and self.current_target_idx < len(self.waypoints) - 1:
#                 self.current_target_idx += 1
#                 self.stats['waypoints_reached'] += 1
#                 self.stats['waypoint_switches'] += 1
#                 return True
#         return False
    
#     def get_control_command(self, current_pos, current_heading):
#         """
#         🎯 คำนวณคำสั่งควบคุมแบบ Bang-Bang ที่ปรับปรุงแล้ว
#         """
#         # 🌟 ตั้งค่าจุดเริ่มต้นครั้งแรก
#         if not self.initial_position_set:
#             self.set_initial_position(current_pos, current_heading)
        
#         # อัพเดท waypoint เป้าหมาย
#         waypoint_changed = self.update_target_waypoint(current_pos)
        
#         # หาทิศทางที่ต้องการ
#         target_heading = self.get_target_heading(current_pos, self.current_target_idx)
        
#         # คำนวณความผิดพลาดของทิศทาง
#         heading_error = self.normalize_angle(target_heading - current_heading)
        
#         # คำนวณระยะทางถึงเป้าหมาย
#         if self.current_target_idx < len(self.waypoints):
#             target_wp = self.waypoints[self.current_target_idx]
#             distance_to_target = np.hypot(target_wp[0] - current_pos[0], target_wp[1] - current_pos[1])
#         else:
#             distance_to_target = 0.0
        
#         # สร้าง Twist message
#         cmd_vel = Twist()
        
#         # 🌟 Enhanced Bang-Bang Control Logic
#         if abs(heading_error) <= self.heading_tolerance:
#             # ไปตรง
#             angular_velocity = 0.0
#             steering_angle = 0.0
#             steering_command = "STRAIGHT"
#             self.stats['straight_commands'] += 1
#         elif heading_error > 0:
#             # เลี้ยวซ้าย - ใช้ค่าสูงสุด 50°
#             angular_velocity = self.max_angular_velocity
#             steering_angle = self.max_steering_angle
#             steering_command = "TURN_LEFT_MAX"
#             self.stats['left_turns'] += 1
#         else:
#             # เลี้ยวขวา - ใช้ค่าสูงสุด -50°
#             angular_velocity = -self.max_angular_velocity
#             steering_angle = -self.max_steering_angle
#             steering_command = "TURN_RIGHT_MAX"
#             self.stats['right_turns'] += 1
        
#         # ตั้งค่าความเร็วคงที่ 1 m/s
#         if distance_to_target > 0.1:
#             cmd_vel.linear.x = self.fixed_speed
#         else:
#             cmd_vel.linear.x = 0.0
        
#         cmd_vel.angular.z = angular_velocity
        
#         # สร้างข้อมูลสถานะ
#         status = {
#             'current_waypoint': self.current_target_idx,
#             'total_waypoints': len(self.waypoints),
#             'target_heading_deg': np.rad2deg(target_heading),
#             'current_heading_deg': np.rad2deg(current_heading),
#             'heading_error_deg': np.rad2deg(heading_error),
#             'distance_to_target': distance_to_target,
#             'steering_command': steering_command,
#             'linear_velocity': cmd_vel.linear.x,
#             'angular_velocity': cmd_vel.angular.z,
#             'waypoint_changed': waypoint_changed,
#             'mission_progress': (self.current_target_idx / len(self.waypoints)) * 100,
#             'search_radius': self.search_radius,
#             'stats': self.stats
#         }
        
#         return cmd_vel, status
    
#     def is_mission_complete(self, current_pos, current_heading):
#         """ตรวจสอบว่าภารกิจเสร็จสิ้นแล้วหรือไม่"""
#         if self.current_target_idx >= len(self.waypoints) - 1:
#             last_wp = self.waypoints[-1]
#             distance = np.hypot(last_wp[0] - current_pos[0], last_wp[1] - current_pos[1])
#             heading_error = abs(self.normalize_angle(self.final_heading - current_heading))
            
#             return (distance < self.waypoint_tolerance and 
#                    heading_error < self.heading_tolerance)
#         return False

# class WaypointControllerNode(Node):
#     def __init__(self):
#         super().__init__('waypoint_controller_node')
        
#         # ROS2 Publishers
#         self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.controller_status_publisher = self.create_publisher(String, '/controller/status', 10)
        
#         # ROS2 Subscribers
#         self.xy_subscriber = self.create_subscription(
#             Point, '/navigation/xy_position', self.xy_position_callback, 10)
#         self.heading_subscriber = self.create_subscription(
#             Float32, '/navigation/heading', self.heading_callback, 10)
        
#         # Controller state
#         self.controller = None
#         self.current_local_x = None
#         self.current_local_y = None
#         self.current_heading = None
#         self.controller_active = False
#         self.mission_complete = False
        
#         # Data validation
#         self.last_position_time = 0
#         self.last_heading_time = 0
#         self.data_timeout = 2.0  # seconds
        
#         # Initialize waypoint controller
#         self.init_waypoint_controller()
        
#         # Control loop timer
#         self.create_timer(0.1, self.control_loop)  # 10Hz control loop
        
#         self.get_logger().info('✅ Waypoint Controller Node initialized')
    
#     def init_waypoint_controller(self):
#     """🌟 Initialize enhanced waypoint controller"""
#     try:
#         # โหลด waypoints จากไฟล์
#         waypoints = self.load_waypoints_from_csv('/home/inc/ros2_ws/waypoints1m.csv')
        
#         if waypoints:
#             self.controller = BangBangWaypointController(
#                 waypoints=waypoints,
#                 final_heading=np.deg2rad(0),        # หันหน้าไปทางเหนือ
#                 waypoint_tolerance=2.0,             # ยอมรับความผิดพลาด 2 เมตร
#                 heading_tolerance=np.deg2rad(10),   # ยอมรับความผิดพลาด 10 องศา
#                 start_from_closest=True,            # 🌟 เริ่มจากจุดใกล้ที่สุด
#                 search_radius=5.0                   # 🌟 ค้นหาในรัศมี 5 เมตร
#             )
            
#             self.get_logger().info(f'✅ Enhanced waypoint controller initialized')
#             self.get_logger().info(f'   📍 {len(waypoints)} waypoints loaded')
#             self.get_logger().info(f'   🔍 Search radius: 5.0m')
#             self.get_logger().info(f'   🎯 Auto-start from closest waypoint in front')
#             self.controller_active = True
#         else:
#             self.get_logger().error('❌ Failed to load waypoints')
            
#     except Exception as e:
#         self.get_logger().error(f'❌ Controller initialization failed: {e}')

# def run_controller(self):
#     """🌟 รันตัวควบคุม waypoint ที่ปรับปรุงแล้ว"""
#     if not (self.controller_active and self.controller and 
#             self.current_local_x is not None and self.current_local_y is not None and
#             self.final_heading is not None):
#         return
    
#     try:
#         current_pos = (self.current_local_x, self.current_local_y)
#         current_heading_rad = np.deg2rad(self.final_heading)
        
#         # รับคำสั่งควบคุมจาก enhanced controller
#         cmd_vel, status = self.controller.get_control_command(current_pos, current_heading_rad)
        
#         # ส่งคำสั่งความเร็ว
#         self.cmd_vel_publisher.publish(cmd_vel)
        
#         # แสดงสถานะ
#         self.publish_controller_status(status)
        
#         # Echo ข้อมูลสำคัญ
#         self.echo_enhanced_controller_status(status)
        
#     except Exception as e:
#         self.get_logger().error(f'❌ Enhanced controller error: {e}')

# def echo_enhanced_controller_status(self, status):
#     """🌟 Echo สถานะที่ปรับปรุงแล้ว"""
#     try:
#         # สร้างข้อความสถานะ
#         waypoint_info = f"WP: {status['current_waypoint']}/{status['total_waypoints']}"
#         direction_info = f"Dir: {status['target_heading_deg']:.1f}°"
#         steering_info = f"Steer: {status['steering_command']}"
#         speed_info = f"Speed: {status['linear_velocity']:.1f}m/s"
#         distance_info = f"Dist: {status['distance_to_target']:.1f}m"
#         progress_info = f"Prog: {status['mission_progress']:.1f}%"
#         radius_info = f"Radius: {status['search_radius']}m"
        
#         # Log ข้อมูลหลัก
#         self.get_logger().info(
#             f"🎯 {waypoint_info} | {direction_info} | {steering_info} | "
#             f"{speed_info} | {distance_info} | {progress_info} | {radius_info}"
#         )
        
#         # Log เพิ่มเติมเมื่อเปลี่ยน waypoint
#         if status['waypoint_changed']:
#             self.get_logger().info(
#                 f"✅ Reached waypoint {status['current_waypoint']-1}! "
#                 f"Moving to waypoint {status['current_waypoint']} "
#                 f"(Switches: {status['stats']['waypoint_switches']})"
#             )
        
#         # ตรวจสอบการเสร็จสิ้นภารกิจ
#         if self.controller.is_mission_complete(
#             (self.current_local_x, self.current_local_y), 
#             np.deg2rad(self.final_heading)
#         ):
#             self.get_logger().info("🏁 Mission Complete! All waypoints reached.")
            
#     except Exception as e:
#         self.get_logger().debug(f'Enhanced echo error: {e}')
    
#     def destroy_node(self):
#         """Cleanup"""
#         self.get_logger().info("🔄 Shutting down Waypoint Controller Node...")
        
#         # ส่งคำสั่งหยุดก่อนปิด
#         self.send_stop_command()
        
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     waypoint_controller = WaypointControllerNode()
    
#     try:
#         rclpy.spin(waypoint_controller)
#     except KeyboardInterrupt:
#         waypoint_controller.get_logger().info("🛑 Keyboard interrupt")
#     finally:
#         waypoint_controller.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

    # ------------------------------------------new controllor-------------------------------------------------
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
        
#         # กำหนด segment สำหรับเส้นทาง (ทุกโค้งเลี้ยวซ้าย)
#         self.path_segments = [
#             {'start': 0, 'end': 240, 'type': 'straight', 'direction': 'S-N', 'target_heading': 0.0},
#             {'start': 241, 'end': 256, 'type': 'curve', 'direction': 'curve1', 'target_heading': None, 'turn': 'left'},
#             {'start': 257, 'end': 315, 'type': 'straight', 'direction': 'E-W', 'target_heading': 335.0},
#             {'start': 316, 'end': 383, 'type': 'curve', 'direction': 'curve2', 'target_heading': None, 'turn': 'left'},
#             {'start': 384, 'end': 496, 'type': 'straight', 'direction': 'N-S', 'target_heading': 0.0},
#             {'start': 497, 'end': 571, 'type': 'curve', 'direction': 'curve3', 'target_heading': None, 'turn': 'left'},
#             {'start': 572, 'end': 622, 'type': 'straight', 'direction': 'W-E', 'target_heading': 18.0},
#             {'start': 623, 'end': 642, 'type': 'curve', 'direction': 'curve4', 'target_heading': None, 'turn': 'left'},
#             {'start': 643, 'end': 654, 'type': 'straight', 'direction': 'S-N', 'target_heading': 0.0}
#         ]
#         # ทุกโค้งเลี้ยวซ้าย
#         self.curve_radii = {
#             'curve1': 20.0,
#             'curve2': 10.0,
#             'curve3': 9.0,
#             'curve4': 7.0
#         }
        
#         # เพิ่มตัวแปรสำหรับติดตามระยะทาง
#         self.previous_distance = None
#         self.distance_increasing_count = 0
#         self.max_distance_increasing = 3  # ถ้าระยะทางเพิ่มขึ้น 3 ครั้งติดต่อกัน ให้ skip
    
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
#         while angle_diff > 180:
#             angle_diff -= 360
#         while angle_diff < -180:
#             angle_diff += 360
#         return angle_diff
    
#     def should_advance_waypoint(self, distance_to_waypoint, waypoint_reached_threshold):
#         """ตรวจสอบว่าควรเลื่อนไป waypoint ถัดไปหรือไม่"""
#         # วิธีที่ 1: ระยะทางใกล้พอ
#         if distance_to_waypoint < waypoint_reached_threshold:
#             return True, "reached_threshold"
        
#         # วิธีที่ 2: ระยะทางเพิ่มขึ้นเรื่อยๆ (รถขับผ่านไปแล้ว)
#         if self.previous_distance is not None:
#             if distance_to_waypoint > self.previous_distance:
#                 self.distance_increasing_count += 1
#             else:
#                 self.distance_increasing_count = 0
        
#         self.previous_distance = distance_to_waypoint
        
#         # ถ้าระยะทางเพิ่มขึ้นติดต่อกันและมากกว่า threshold ที่ใหญ่ขึ้น
#         if (self.distance_increasing_count >= self.max_distance_increasing and 
#             distance_to_waypoint > waypoint_reached_threshold * 3):
#             return True, "passed_waypoint"
        
#         # วิธีที่ 3: ระยะทางไกลเกินไป (skip waypoint ที่ไม่สามารถไปถึงได้)
#         if distance_to_waypoint > 100.0:  # 100 เมตร
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
#             # รีเซ็ตตัวแปรติดตาม
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
#         """บังคับให้ heading เริ่มต้นเป็น 0 องศา"""
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
    
#     def calculate_heading_correction_steering(self, current_compass_heading, target_compass_heading, kp=0.08):
#         heading_error = self.normalize_angle_difference(target_compass_heading - current_compass_heading)
#         steering_angle = kp * math.radians(heading_error)
#         return steering_angle, heading_error
    
#     def normalize_angle_difference(self, angle_diff):
#         while angle_diff > 180:
#             angle_diff -= 360
#         while angle_diff < -180:
#             angle_diff += 360
#         return angle_diff

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
#         while angle > 180:
#             angle -= 360
#         while angle < -180:
#             angle += 360
#         return angle

# class ConstantCurvatureController:
#     def __init__(self):
#         self.curvature_history = deque(maxlen=5)
    
#     def calculate_steering_angle_with_radius(self, curve_radius, linear_velocity, wheelbase, segment_info=None):
#         curvature = 1.0 / abs(curve_radius)
#         steering_angle = math.atan(wheelbase * curvature)
#         # ทุกโค้งเลี้ยวซ้าย
#         steering_angle = abs(steering_angle)
#         return steering_angle

# class NavigationController(Node):
#     def __init__(self):
#         super().__init__('navigation_controller')
        
#         # ROS2 Subscribers
#         self.xy_subscriber = self.create_subscription(
#             Float64MultiArray, '/navigation/xy', self.xy_callback, 10)
#         self.heading_subscriber = self.create_subscription(
#             Float32, '/navigation/heading', self.heading_callback, 10)
        
#         # ROS2 Publishers
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
        
#         # Navigation Components
#         csv_file_path = '/home/inc/ros2_ws/waypointfile/waypoints3xy.csv'
#         self.waypoint_manager = WaypointManager(csv_file_path, auto_start=True)
#         self.pure_pursuit = PurePursuitController(lookahead_distance=3.0)
#         self.curvature_controller = ConstantCurvatureController()
#         self.bicycle_model = BicycleKinematicModel(wheelbase=self.wheelbase)
        
#         # Navigation state
#         self.navigation_active = False
#         self.waypoint_reached_threshold = 3.0  # เพิ่มจาก 1.0 เป็น 3.0 เมตร
#         self.heading_forced_to_zero = False
        
#         # Statistics
#         self.stats = {'navigation_commands': 0, 'heading_corrections': 0}
        
#         # Timer for logging progress
#         self.last_progress_log_time = 0
#         self.progress_log_interval = 2.0  # แสดงความคืบหน้าทุก 2 วินาที
        
#         # Control timer
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
    
#     def navigation_control(self):
#         # ตรวจสอบการเริ่มต้นการนำทาง
#         if not self.waypoint_manager.navigation_started:
#             if self.current_heading is not None:
#                 # บังคับให้ heading เป็น 0 องศา
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
        
#         if not self.navigation_active or self.current_heading is None:
#             return
        
#         # ได้ตำแหน่งปัจจุบัน
#         current_waypoint = self.waypoint_manager.get_current_waypoint()
#         if current_waypoint is None:
#             self.get_logger().info('🏁 Navigation completed - reached final waypoint')
#             self.navigation_active = False
#             self.send_stop_command()
#             return
        
#         # ตรวจสอบว่าถึง waypoint แล้วหรือยัง (ใช้ smart advance)
#         distance_to_waypoint = math.sqrt(
#             (self.current_position['x'] - current_waypoint['x'])**2 +
#             (self.current_position['y'] - current_waypoint['y'])**2
#         )
        
#         should_advance, advance_reason = self.waypoint_manager.should_advance_waypoint(
#             distance_to_waypoint, self.waypoint_reached_threshold
#         )
        
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
        
#         # แสดงความคืบหน้าเป็นระยะ
#         current_time = self.get_clock().now().nanoseconds / 1e9
#         if current_time - self.last_progress_log_time > self.progress_log_interval:
#             self.log_current_progress(distance_to_waypoint)
#             self.last_progress_log_time = current_time
        
#         # ได้ข้อมูลเส้นทางปัจจุบัน
#         segment_info = self.waypoint_manager.get_current_segment_info()
#         segment_type = segment_info['type']
#         direction = segment_info['direction']
#         target_heading = segment_info.get('target_heading')
        
#         # อัพเดท target heading สำหรับเส้นทางตรง
#         if segment_type == 'straight' and target_heading is not None:
#             self.bicycle_model.set_target_heading(target_heading)
        
#         # ใช้ความเร็วคงที่
#         linear_velocity = self.fixed_speed
        
#         # คำนวณคำสั่งควบคุมตามประเภทเส้นทาง
#         if segment_type == 'straight':
#             steering_angle, heading_error = self.calculate_straight_path_control(
#                 current_waypoint, self.bicycle_model.get_current_target_heading()
#             )
#         else:  # curve
#             steering_angle = self.calculate_curve_path_control(segment_info)
#             heading_error = 0.0
        
#         # จำกัดมุมเลี้ยว
#         steering_angle = self.bicycle_model.limit_steering_angle(
#             steering_angle, self.max_steering_angle
#         )
        
#         # คำนวณ angular velocity
#         angular_velocity = self.bicycle_model.calculate_angular_velocity(
#             linear_velocity, steering_angle
#         )
        
#         # จำกัด angular velocity
#         angular_velocity = max(-self.max_angular_velocity,
#                              min(self.max_angular_velocity, angular_velocity))
        
#         # สร้างและส่งคำสั่ง cmd_vel
#         cmd_vel = Twist()
#         cmd_vel.linear.x = linear_velocity
#         cmd_vel.angular.z = angular_velocity
        
#         self.cmd_vel_publisher.publish(cmd_vel)
#         self.stats['navigation_commands'] += 1
        
#         # Debug information
#         self.publish_debug_info(segment_info, distance_to_waypoint, steering_angle,
#                                linear_velocity, angular_velocity, heading_error)
    
#     def log_current_progress(self, distance_to_waypoint):
#         """แสดงความคืบหน้าปัจจุบัน"""
#         progress = self.waypoint_manager.get_progress_info()
#         segment_info = self.waypoint_manager.get_current_segment_info()
        
#         # เพิ่มข้อมูลสถานะการติดตาม
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
#             self.current_heading,
#             target_heading,
#             0.08  # เพิ่ม kp เพื่อการแก้ไขที่เร็วขึ้น
#         )
#         pure_pursuit_steering = self.pure_pursuit.calculate_steering_angle(
#             self.current_position,
#             self.current_heading,
#             target_waypoint,
#             self.wheelbase
#         )
#         combined_steering = 0.8 * heading_correction_steering + 0.2 * pure_pursuit_steering * 0.5
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
#         # Print final statistics
#         self.print_final_statistics()
#         super().destroy_node()
    
#     def print_final_statistics(self):
#         """Print final performance statistics"""
#         try:
#             self.get_logger().info("📊 Final Navigation Statistics:")
#             self.get_logger().info(f"  🚗 Navigation commands: {self.stats['navigation_commands']}")
#             self.get_logger().info(f"  🔧 Heading corrections: {self.stats['heading_corrections']}")
            
#             # Waypoint progress
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



# --------------------------------------------------------------------------------------------

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
        
#         # เพิ่ม radius ใน segment ที่ต้องการให้วิ่งโค้งในทางตรง
#         self.path_segments = [
#             {'start': 0, 'end': 240, 'type': 'straight', 'direction': 'S-N', 'target_heading': 0.0, 'radius': 15.0},
#             {'start': 241, 'end': 256, 'type': 'curve', 'direction': 'curve1', 'target_heading': None, 'turn': 'left'},
#             {'start': 257, 'end': 315, 'type': 'straight', 'direction': 'E-W', 'target_heading': 335.0},
#             {'start': 316, 'end': 383, 'type': 'curve', 'direction': 'curve2', 'target_heading': None, 'turn': 'left'},
#             {'start': 384, 'end': 496, 'type': 'straight', 'direction': 'N-S', 'target_heading': 0.0},
#             {'start': 497, 'end': 571, 'type': 'curve', 'direction': 'curve3', 'target_heading': None, 'turn': 'left'},
#             {'start': 572, 'end': 622, 'type': 'straight', 'direction': 'W-E', 'target_heading': 18.0},
#             {'start': 623, 'end': 642, 'type': 'curve', 'direction': 'curve4', 'target_heading': None, 'turn': 'left'},
#             {'start': 643, 'end': 654, 'type': 'straight', 'direction': 'S-N', 'target_heading': 0.0}
#         ]
#         self.curve_radii = {
#             'curve1': 10.0,
#             'curve2': 10.0,
#             'curve3': 9.0,
#             'curve4': 7.0
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
#         while angle_diff > 180:
#             angle_diff -= 360
#         while angle_diff < -180:
#             angle_diff += 360
#         return angle_diff
    
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
    
#     def calculate_heading_correction_steering(self, current_compass_heading, target_compass_heading, kp=0.08):
#         heading_error = self.normalize_angle_difference(target_compass_heading - current_compass_heading)
#         steering_angle = kp * math.radians(heading_error)
#         return steering_angle, heading_error
    
#     def normalize_angle_difference(self, angle_diff):
#         while angle_diff > 180:
#             angle_diff -= 360
#         while angle_diff < -180:
#             angle_diff += 360
#         return angle_diff

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
#         while angle > 180:
#             angle -= 360
#         while angle < -180:
#             angle += 360
#         return angle

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
#         self.max_steering_angle = 0.873
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
#         if not self.navigation_active or self.current_heading is None:
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
#         if segment_type == 'straight' and target_heading is not None:
#             self.bicycle_model.set_target_heading(target_heading)
#         linear_velocity = self.fixed_speed
#         # ส่วนที่แก้ไขใหม่: สำหรับ straight ที่มี radius ให้ใช้รัศมีคงที่
#         if segment_type == 'straight' and segment_info.get('radius') is not None:
#             radius = segment_info['radius']
#             steering_angle = self.curvature_controller.calculate_steering_angle_with_radius(
#                 radius, self.fixed_speed, self.wheelbase, segment_info
#             )
#             heading_error = 0.0
#         elif segment_type == 'straight':
#             steering_angle, heading_error = self.calculate_straight_path_control(
#                 current_waypoint, self.bicycle_model.get_current_target_heading()
#             )
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
#                              min(self.max_angular_velocity, angular_velocity))
#         cmd_vel = Twist()
#         cmd_vel.linear.x = linear_velocity
#         cmd_vel.angular.z = angular_velocity
#         self.cmd_vel_publisher.publish(cmd_vel)
#         self.stats['navigation_commands'] += 1
#         self.publish_debug_info(segment_info, distance_to_waypoint, steering_angle,
#                                linear_velocity, angular_velocity, heading_error)
    
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
#             self.current_heading,
#             target_heading,
#             0.08
#         )
#         pure_pursuit_steering = self.pure_pursuit.calculate_steering_angle(
#             self.current_position,
#             self.current_heading,
#             target_waypoint,
#             self.wheelbase
#         )
#         combined_steering = 0.8 * heading_correction_steering + 0.2 * pure_pursuit_steering * 0.5
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




# -----------------------------------------------ดัที่สุเตินนี้ 14-7-68---------------------------------------
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
#             {'start': 0, 'end': 240, 'type': 'straight', 'direction': 'S-N', 'target_heading': 0.0,},
#             {'start': 241, 'end': 256, 'type': 'curve', 'direction': 'curve1', 'target_heading': None, 'turn': 'left'},
#             {'start': 257, 'end': 315, 'type': 'straight', 'direction': 'E-W', 'target_heading': 335.0},
#             {'start': 316, 'end': 383, 'type': 'curve', 'direction': 'curve2', 'target_heading': None, 'turn': 'left'},
#             {'start': 384, 'end': 496, 'type': 'straight', 'direction': 'N-S', 'target_heading': 0.0},
#             {'start': 497, 'end': 571, 'type': 'curve', 'direction': 'curve3', 'target_heading': None, 'turn': 'left'},
#             {'start': 572, 'end': 622, 'type': 'straight', 'direction': 'W-E', 'target_heading': 18.0},
#             {'start': 623, 'end': 642, 'type': 'curve', 'direction': 'curve4', 'target_heading': None, 'turn': 'left'},
#             {'start': 643, 'end': 654, 'type': 'straight', 'direction': 'S-N', 'target_heading': 0.0}
#         ]
#         self.curve_radii = {
#             'curve1': 30.0,
#             'curve2': 5.0,
#             'curve3': 20.0,
#             'curve4': 40.0
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
    
#     def calculate_heading_correction_steering(self, current_compass_heading, target_compass_heading, kp=1.2):
#         # kp สูงขึ้นมากเพื่อให้พวงมาลัยตอบสนองแรงขึ้น
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
#         self.max_steering_angle = 1.2  # ปรับเป็น 1.2 radian หรือค่าที่พวงมาลัยรองรับ
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
#         if not self.navigation_active or self.current_heading is None:
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
#             steering_angle, heading_error = self.calculate_straight_path_control(
#                 current_waypoint, self.bicycle_model.get_current_target_heading()
#             )
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
#                              min(self.max_angular_velocity, angular_velocity))
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
#             self.current_heading,
#             target_heading,
#             3.5  # kp สูงขึ้น
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