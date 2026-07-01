# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
# from std_msgs.msg import Float32, String
# from geometry_msgs.msg import Twist  # เพิ่มการใช้งาน Twist สำหรับ /cmd_vel
# import math
# import numpy as np
# from geographiclib.geodesic import Geodesic

# class PurePursuitNavigator(Node):
#     def __init__(self):
#         super().__init__('pure_pursuit_navigator')
        
#         # Subscribe to GNSS topic
#         self.gnss_sub = self.create_subscription(
#             NavSatFix,
#             '/navigation/gnss',
#             self.gnss_callback,
#             10)
            
#         # Subscribe to destination topic
#         self.destination_sub = self.create_subscription(
#             String,
#             '/navigation/destination',
#             self.destination_callback,
#             10)
            
#         # Publisher for bearing control
#         self.bearing_pub = self.create_publisher(
#             Float32,
#             '/navigation/bearing',
#             10)
        
#         # Publisher for velocity control (cmd_vel)
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)  # เพิ่ม publisher สำหรับ /cmd_vel
        
#         # Current state variables
#         self.current_pos = None      # Current position (lat, lon)
#         self.current_heading = 0.0   # Current heading in degrees
#         self.destination = None      # Destination (lat, lon)
#         self.path_points = []        # Path points between current and destination
#         self.lookahead_distance = 3.0  # Lookahead distance in meters
        
#         # Available destinations (same as in your web interface)
#         self.available_destinations = {
#             "S14": {"lat": 13.650940, "lon": 100.492070},
#             "S13": {"lat": 13.650076, "lon": 100.492140},
#             "S12": {"lat": 13.649913, "lon": 100.492690},
#             "S11": {"lat": 13.649992, "lon": 100.493327},
#             "S15": {"lat": 13.650154, "lon": 100.492984},
#             "N19": {"lat": 13.651010, "lon": 100.492980},
#             "N20": {"lat": 13.6513894, "lon": 100.4929824}
#         }
        
#         # Timer for path following at 10Hz
#         self.create_timer(0.1, self.path_following_callback)
        
#         self.get_logger().info('Pure Pursuit Navigator initialized')
    
#     def gnss_callback(self, msg):
#         self.current_pos = (msg.latitude, msg.longitude)
        
#         # In a real system, you might get heading from GNSS if available
#         # For now, we'll estimate it from consecutive positions
#         # This would ideally be replaced with compass or IMU data
        
#         self.get_logger().debug(f'Updated position: {self.current_pos[0]:.6f}, {self.current_pos[1]:.6f}')
    
#     def destination_callback(self, msg):
#         destination_code = msg.data
        
#         if destination_code in self.available_destinations:
#             dest_info = self.available_destinations[destination_code]
#             self.destination = (dest_info["lat"], dest_info["lon"])
            
#             # Generate path points between current position and destination
#             if self.current_pos:
#                 self.generate_path()
            
#             self.get_logger().info(f'New destination set: {destination_code} at {self.destination}')
#         else:
#             self.get_logger().warn(f'Unknown destination code: {destination_code}')
    
#     def generate_path(self):
#         """Generate a path from current position to destination"""
#         # For simplicity, we'll create a straight-line path
#         # In a real system, you might want to use a path planner
        
#         if not self.current_pos or not self.destination:
#             return
            
#         # Clear existing path
#         self.path_points = []
        
#         # Calculate distance
#         geod = Geodesic.WGS84
#         g = geod.Inverse(self.current_pos[0], self.current_pos[1], 
#                          self.destination[0], self.destination[1])
#         distance = g['s12']  # distance in meters
        
#         # For long distances, create waypoints along the path
#         if distance > 10:  # If more than 10 meters
#             num_points = int(distance / 5)  # One point every 5 meters
#             3
#             for i in range(num_points + 1):
#                 t = i / num_points
#                 g = geod.Direct(self.current_pos[0], self.current_pos[1], 
#                                g.azi1, t * distance)
#                 self.path_points.append((g['lat2'], g['lon2']))
        
#         # Add destination as final point
#         self.path_points.append(self.destination)
        
#         self.get_logger().info(f'Generated path with {len(self.path_points)} points')
    
#     def path_following_callback(self):
#         """Main Pure Pursuit algorithm implementation"""
#         if not self.current_pos or not self.path_points:
#             return
            
#         # Find the lookahead point on the path
#         target_point = self.find_lookahead_point()
        
#         if target_point:
#             # Calculate bearing to the target point
#             geod = Geodesic.WGS84
#             g = geod.Inverse(self.current_pos[0], self.current_pos[1], 
#                            target_point[0], target_point[1])
#             bearing = g.azi1  # Forward azimuth
            
#             # Normalize to 0-360 degrees
#             if bearing < 0:
#                 bearing += 360
            
#             # Create and publish bearing message
#             bearing_msg = Float32()
#             bearing_msg.data = float(bearing)
#             self.bearing_pub.publish(bearing_msg)
            
#             self.get_logger().debug(f'Pure Pursuit bearing: {bearing:.2f} degrees')

#             # Calculate speed and angular velocity for cmd_vel
#             twist_msg = Twist()
#             twist_msg.linear.x = 0.5  # Forward speed (m/s)
#             twist_msg.angular.z = self.calculate_angular_velocity(bearing)  # Angular velocity
            
#             # Publish the cmd_vel message to control the vehicle
#             self.cmd_vel_pub.publish(twist_msg)
    
#     def calculate_angular_velocity(self, bearing):
#         """Calculate angular velocity to steer the vehicle toward the target"""
#         # Simple approach: make angular velocity proportional to the difference in heading
#         angle_diff = bearing - self.current_heading
#         if angle_diff > 180:
#             angle_diff -= 360
#         elif angle_diff < -180:
#             angle_diff += 360
        
#         # Proportional control (adjust the constant for tuning)
#         angular_velocity = 0.5 * angle_diff
#         return angular_velocity
        
#     def find_lookahead_point(self):
#         """Find the point on the path that is lookahead_distance away"""
#         if not self.path_points:
#             return None
            
#         # Check if we've reached the final destination
#         geod = Geodesic.WGS84
#         g = geod.Inverse(self.current_pos[0], self.current_pos[1], 
#                        self.path_points[-1][0], self.path_points[-1][1])
#         distance = g['s12']   # If within 2 meters of destination
#             return self.path_points[-1]
        
#         # Find the closest point on the path
#         closest_point_idx = 0
#         min_distance = float('inf')
        
#         for i, point in enumerate(self.path_points):
#             g = geod.Inverse(self.current_pos[0], self.current_pos[1], 
#                            point[0], point[1])
#             distance = g['s12']
            
#             if distance < min_distance:
#                 min_distance = distance
#                 closest_point_idx = i
        
#         # Look ahead on the path
#         for i in range(closest_point_idx, len(self.path_points)):
#             point = self.path_points[i]
#             g = geod.Inverse(self.current_pos[0], self.current_pos[1], 
#                            point[0], point[1])
#             distance = g['s12']
            
#             if distance >= self.lookahead_distance:
#                 return point
        
#         # If no point is far enough, return the last point
#         return self.path_points[-1]

# def main(args=None):
#     rclpy.init(args=args)
#     navigator = PurePursuitNavigator()
#     rclpy.spin(navigator)
    
#     navigator.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



# -----------------------------ปัจจุบัน-----------------------------------
#!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
# from std_msgs.msg import Float32, String
# from geometry_msgs.msg import Twist
# import math
# import json
# import numpy as np
# from geographiclib.geodesic import Geodesic
# import websockets
# import threading
# import asyncio
# import concurrent.futures
# import time
# import matplotlib.pyplot as plt

# class PurePursuitNavigator(Node):
#     def __init__(self):
#         super().__init__('pure_pursuit_navigator')
        
#         # Subscribe to GNSS topic
#         self.gnss_sub = self.create_subscription(
#             NavSatFix,
#             '/navigation/gnss',
#             self.gnss_callback,
#             10)
            
#         # Subscribe to destination topic
#         self.destination_sub = self.create_subscription(
#             String,
#             '/navigation/destination',
#             self.destination_callback,
#             10)
            
#         # Publisher for bearing control
#         self.bearing_pub = self.create_publisher(
#             Float32,
#             '/navigation/bearing',
#             10)
        
#         # Publisher for velocity control (cmd_vel)
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
#         # Current state variables
#         self.current_pos = None      # Current position (lat, lon)
#         self.prev_pos = None         # Previous position for heading calculation
#         self.current_heading = 0.0   # Current heading in degrees
#         self.destination = None      # Destination (lat, lon)
#         self.path_points = []        # Path points between current and destination
#         self.lookahead_distance = 2.0  # Lookahead distance in meters
#         self.waypoint_spacing = 3.5  # สร้าง waypoint ทุกๆ 3.5 เมตร
#         self.arrival_threshold = 2.5  # ระยะห่างจากจุดหมายที่ถือว่าถึงแล้ว (เมตร)
        
#         # Vehicle parameters
#         self.wheelbase = 2.20  # ความยาวระหว่างล้อหน้าและล้อหลัง (เมตร)
#         self.max_steering_angle = 45.0  # มุมเลี้ยวสูงสุด (องศา)
#         self.max_velocity = 1.5  # ความเร็วสูงสุด (m/s)
        
#         # Available destinations
#         self.available_destinations = {
#             "S14": {"lat": 13.650940, "lon": 100.492070},
#             "S13": {"lat": 13.650076, "lon": 100.492140},
#             "S12": {"lat": 13.649913, "lon": 100.492690},
#             "S11": {"lat": 13.649992, "lon": 100.493327},
#             "S15": {"lat": 13.650154, "lon": 100.492984},
#             "N19": {"lat": 13.651010, "lon": 100.492980},
#             "N20": {"lat": 13.6513894, "lon": 100.4929824}
#         }
        
#         # Setup WebSocket server variables
#         self.ws_server = None
#         self._clients = set()  # Changed from self.clients to self._clients to avoid conflict
#         self.ws_lock = threading.Lock()  # Lock สำหรับป้องกัน race condition
        
#         # สร้าง event loop สำหรับ asyncio
#         self.loop = asyncio.new_event_loop()
        
#         # เริ่ม WebSocket server
#         self.start_websocket_server()
        
#         # Timer for path following at 10Hz
#         self.create_timer(0.1, self.path_following_callback)
        
#         self.get_logger().info('Pure Pursuit Navigator initialized')
    
#     def start_websocket_server(self):
#         """Start WebSocket server in a separate thread"""
#         def run_server():
#             asyncio.set_event_loop(self.loop)
            
#             async def handler(websocket, path):
#                 # Register new client
#                 with self.ws_lock:
#                     self._clients.add(websocket)  # Updated to use self._clients
#                 self.get_logger().info(f"New client connected. Total clients: {len(self._clients)}")
                
#                 try:
#                     async for message in websocket:
#                         try:
#                             data = json.loads(message)
                            
#                             # Handle destination
#                             if 'destination' in data:
#                                 dest_code = data['destination']
#                                 self.get_logger().info(f"Received destination from web: {dest_code}")
                                
#                                 # Process as if received from ROS topic
#                                 msg = String()
#                                 msg.data = dest_code
#                                 self.destination_callback(msg)
                            
#                             # Handle path data sent from frontend
#                             if 'path' in data and isinstance(data['path'], list) and len(data['path']) > 0:
#                                 # Convert the path points from web format to our format
#                                 new_path = []
#                                 for point in data['path']:
#                                     if 'lat' in point and 'lon' in point:
#                                         new_path.append((float(point['lat']), float(point['lon'])))
                                
#                                 if len(new_path) > 0:
#                                     self.get_logger().info(f"Received path from web with {len(new_path)} points")
#                                     # ทำการสร้าง waypoints ที่กระจายตัวสม่ำเสมอ
#                                     self.path_points = self.resample_path(new_path)
#                                     self.get_logger().info(f"Resampled path to {len(self.path_points)} waypoints")
                            
#                             # Handle bearing data for debugging
#                             if 'bearing' in data:
#                                 bearing = float(data['bearing'])
#                                 # สามารถใช้สำหรับ debug หรือกลยุทธ์การควบคุมอื่นๆ
                        
#                         except json.JSONDecodeError:
#                             self.get_logger().error(f"Failed to parse JSON message: {message}")
#                         except Exception as e:
#                             self.get_logger().error(f"Error processing message: {e}")
                
#                 except websockets.exceptions.ConnectionClosed:
#                     pass
#                 finally:
#                     # Unregister client
#                     with self.ws_lock:
#                         self._clients.discard(websocket)  # Updated to use self._clients
#                     self.get_logger().info(f"Client disconnected. Total clients: {len(self._clients)}")
            
#             # Start the WebSocket server
#             async def main():
#                 self.ws_server = await websockets.serve(handler, "0.0.0.0", 5000)
#                 self.get_logger().info("WebSocket server started on port 5000")
#                 await self.ws_server.wait_closed()
            
#             self.loop.run_until_complete(main())
#             self.loop.run_forever()
        
#         # Start the server in a separate thread
#         threading.Thread(target=run_server, daemon=True).start()
    
#     async def send_to_clients(self, data):
#         """Send data to all connected WebSocket clients"""
#         if not self._clients:  # Updated to use self._clients
#             return
            
#         message = json.dumps(data)
#         with self.ws_lock:
#             clients_copy = self._clients.copy()  # Updated to use self._clients
        
#         for client in clients_copy:
#             try:
#                 # ใช้ asyncio.wait_for เพื่อกำหนด timeout
#                 await asyncio.wait_for(client.send(message), timeout=0.5)
#             except asyncio.TimeoutError:
#                 self.get_logger().warn(f"Timeout sending message to client")
#                 # ไม่จำเป็นต้องลบ client เพราะจะตรวจจับได้จาก ConnectionClosed exception
#             except websockets.exceptions.ConnectionClosed:
#                 with self.ws_lock:
#                     self._clients.discard(client)  # Updated to use self._clients
#                 self.get_logger().info(f"Removed closed client connection. Total clients: {len(self._clients)}")
#             except Exception as e:
#                 self.get_logger().error(f"Error sending to client: {type(e).__name__}: {e}")
    
#     def send_update(self, data):
#         """Helper function to send updates via WebSocket from synchronous code"""
#         future = asyncio.run_coroutine_threadsafe(self.send_to_clients(data), self.loop)
#         try:
#             # Wait for a short time to avoid blocking
#             future.result(timeout=0.5)
#         except (asyncio.TimeoutError, concurrent.futures.TimeoutError):
#             self.get_logger().warn("Timeout when sending WebSocket update")
#         except Exception as e:
#             self.get_logger().error(f"Error in send_update: {type(e).__name__}: {e}")
    
#     def resample_path(self, path):
#         """Resample path points to create evenly spaced waypoints"""
#         if len(path) < 2:
#             return path
            
#         resampled_path = [path[0]]  # Start with the first point
#         geod = Geodesic.WGS84
        
#         # กำหนดระยะห่างระหว่าง waypoints ให้มีค่าเหมาะสม
#         target_spacing = self.waypoint_spacing  # ระยะห่างที่ต้องการระหว่าง waypoints
        
#         for i in range(len(path) - 1):
#             start_point = path[i]
#             end_point = path[i + 1]
            
#             # คำนวณระยะทางและทิศทางระหว่างจุด
#             g = geod.Inverse(start_point[0], start_point[1], end_point[0], end_point[1])
#             segment_distance = g['s12']  # ระยะทางในเมตร
#             initial_bearing = g['azi1']  # ทิศทางเริ่มต้น
            
#             # ถ้าระยะทางน้อยกว่าที่กำหนด ข้ามไปเลย
#             if segment_distance < target_spacing * 0.5:
#                 continue
                
#             # จำนวน waypoints ที่ต้องการระหว่างจุดต้นทางและปลายทาง
#             num_points = max(1, int(segment_distance / target_spacing))
            
#             # สร้าง waypoints ระหว่างจุดต้นทางและปลายทาง
#             for j in range(1, num_points):
#                 fraction = j / num_points
#                 point = geod.Direct(start_point[0], start_point[1], 
#                                   initial_bearing, fraction * segment_distance)
#                 resampled_path.append((point['lat2'], point['lon2']))
            
#             # เพิ่มจุดปลายทางของส่วนนี้
#             resampled_path.append(end_point)
        
#         return resampled_path
    
#     def gnss_callback(self, msg):
#         # Store previous position for heading calculation
#         if self.current_pos:
#             self.prev_pos = self.current_pos
            
#         self.current_pos = (msg.latitude, msg.longitude)
        
#         # Calculate heading if we have previous position
#         if self.prev_pos:
#             geod = Geodesic.WGS84
#             g = geod.Inverse(self.prev_pos[0], self.prev_pos[1], 
#                            self.current_pos[0], self.current_pos[1])
#             # Update heading only if moving (to avoid noise when stationary)
#             if g['s12'] > 0.2:  # If moved more than 20cm
#                 self.current_heading = g['azi1']
#                 if self.current_heading < 0:
#                     self.current_heading += 360.0
                
#                 self.get_logger().debug(f'Updated heading: {self.current_heading:.2f} degrees')
        
#         self.get_logger().debug(f'Updated position: {self.current_pos[0]:.6f}, {self.current_pos[1]:.6f}')
        
#         # Send position update to connected clients
#         try:
#             data = {
#                 'latitude': self.current_pos[0],
#                 'longitude': self.current_pos[1],
#                 'heading': self.current_heading
#             }
            
#             # ใช้ helper function ที่เราสร้างขึ้น
#             self.send_update(data)
#         except Exception as e:
#             self.get_logger().error(f"Error sending position update to clients: {type(e).__name__}: {e}")
        
#         # Check if we need to generate a path (if destination is set but no path)
#         if self.destination and not self.path_points:
#             self.generate_path()
    
#     def destination_callback(self, msg):
#         destination_code = msg.data
        
#         if destination_code in self.available_destinations:
#             dest_info = self.available_destinations[destination_code]
#             self.destination = (dest_info["lat"], dest_info["lon"])
            
#             # ล้างเส้นทางเก่าและตั้งค่า arrival state ใหม่
#             self.path_points = []
            
#             # Generate path only if we don't expect to receive one from the web interface
#             if self.current_pos and len(self._clients) == 0:  # Updated to use self._clients
#                 self.generate_path()
            
#             self.get_logger().info(f'New destination set: {destination_code} at {self.destination}')
#         else:
#             self.get_logger().warn(f'Unknown destination code: {destination_code}')
    
#     def generate_path(self):
#         """Generate a path from current position to destination with waypoints"""
#         if not self.current_pos or not self.destination:
#             return
            
#         # Clear existing path
#         self.path_points = []
        
#         # Get straight-line distance and bearing
#         geod = Geodesic.WGS84
#         g = geod.Inverse(self.current_pos[0], self.current_pos[1], 
#                          self.destination[0], self.destination[1])
#         total_distance = g['s12']  # distance in meters
#         bearing = g['azi1']  # initial bearing
        
#         # หา waypoints สำหรับเส้นทางที่มีทางเลี้ยว
#         custom_path = self.find_custom_path()
        
#         if custom_path:
#             # ใช้เส้นทางกำหนดเองที่มี waypoint ตามต้องการ
#             self.path_points = custom_path
#             self.get_logger().info(f'Using predefined path with {len(self.path_points)} waypoints')
#         else:
#             # สร้าง waypoints ด้วยการ resample
#             num_points = max(3, int(total_distance / self.waypoint_spacing))
            
#             straight_path = []
#             for i in range(num_points + 1):
#                 t = i / num_points
#                 point = geod.Direct(self.current_pos[0], self.current_pos[1], 
#                                   bearing, t * total_distance)
#                 straight_path.append((point['lat2'], point['lon2']))
            
#             # Add destination as final point
#             if straight_path[-1] != self.destination:
#                 straight_path.append(self.destination)
            
#             # Resample to get evenly spaced waypoints
#             self.path_points = self.resample_path(straight_path)
                
#             self.get_logger().info(f'Generated straight-line path with {len(self.path_points)} points')
        
#         # ส่งเส้นทางไปยัง web interface
#         try:
#             path_data = [{
#                 'lat': point[0], 
#                 'lon': point[1]
#             } for point in self.path_points]
            
#             data = {
#                 'path': path_data
#             }
            
#             self.send_update(data)
#             self.get_logger().info(f'Sent path with {len(path_data)} points to web clients')
#         except Exception as e:
#             self.get_logger().error(f"Error sending path to clients: {type(e).__name__}: {e}")
    
#     def find_custom_path(self):
#         """หาเส้นทางที่กำหนดเองตามตำแหน่งปัจจุบันและปลายทาง"""
#         if not self.current_pos or not self.destination:
#             return None
            
#         # Dictionary ของเส้นทางที่กำหนดเอง
#         # key คือ (start_building, end_building)
#         # value คือ list ของ waypoints [(lat1, lon1), (lat2, lon2), ...]
#         custom_paths = {
#             # ตัวอย่างเส้นทางจาก S14 ไป S11 ผ่านทางเดินที่มีอยู่จริง
#             ("S14", "S11"): [
#                 (13.650940, 100.492070),  # S14 starting point
#                 (13.650500, 100.492300),  # waypoint 1 - ทางแยกแรก
#                 (13.650300, 100.492800),  # waypoint 2 - ทางแยกที่สอง
#                 (13.649992, 100.493327)   # S11 destination
#             ],
#             # เส้นทางจาก S11 ไป S14 (เส้นทางกลับ)
#             ("S11", "S14"): [
#                 (13.649992, 100.493327),  # S11 starting point
#                 (13.650300, 100.492800),  # waypoint 1
#                 (13.650500, 100.492300),  # waypoint 2
#                 (13.650940, 100.492070)   # S14 destination
#             ],
#             # เพิ่มเส้นทางอื่นๆ ตามต้องการ...
#         }
        
#         # หาอาคารที่ใกล้ที่สุดกับตำแหน่งปัจจุบัน
#         start_building = self.find_nearest_building(self.current_pos)
#         # หาอาคารที่ใกล้ที่สุดกับจุดหมาย
#         end_building = self.find_nearest_building(self.destination)
        
#         path_key = (start_building, end_building)
#         if path_key in custom_paths:
#             path = custom_paths[path_key]
#             # ปรับปรุงจุดเริ่มต้นให้ตรงกับตำแหน่งปัจจุบัน
#             return [(self.current_pos[0], self.current_pos[1])] + path[1:]
        
#         # ถ้าไม่มีเส้นทางที่กำหนดเอง ให้ลองพลิกคู่ต้นทาง-ปลายทาง
#         # แล้วสร้างเส้นทางกลับ
#         reverse_key = (end_building, start_building)
#         if reverse_key in custom_paths:
#             # สร้างเส้นทางกลับโดยกลับลำดับ waypoints
#             reversed_path = list(reversed(custom_paths[reverse_key]))
#             # ปรับปรุงจุดเริ่มต้นให้ตรงกับตำแหน่งปัจจุบัน
#             return [(self.current_pos[0], self.current_pos[1])] + reversed_path[1:]
        
#         # ถ้าไม่มีเส้นทางที่กำหนดเอง
#         return None
    
#     def find_nearest_building(self, position):
#         """หาอาคารที่ใกล้ที่สุดกับตำแหน่งที่กำหนด"""
#         geod = Geodesic.WGS84
#         nearest_building = None
#         min_distance = float('inf')
        
#         for code, building in self.available_destinations.items():
#             g = geod.Inverse(position[0], position[1], building["lat"], building["lon"])
#             distance = g['s12']
            
#             if distance < min_distance:
#                 min_distance = distance
#                 nearest_building = code
        
#         return nearest_building
    
#     def path_following_callback(self):
#         """Main Pure Pursuit algorithm implementation"""
#         if not self.current_pos or not self.path_points:
#             return
            
#         # Check if we've reached the destination
#         geod = Geodesic.WGS84
#         g = geod.Inverse(self.current_pos[0], self.current_pos[1], 
#                        self.path_points[-1][0], self.path_points[-1][1])
#         distance_to_goal = g['s12']
        
#         if distance_to_goal <= self.arrival_threshold:
#             # We've arrived at the destination
#             self.get_logger().info(f'Arrived at destination! Distance: {distance_to_goal:.2f}m')
            
#             # Stop the vehicle
#             twist_msg = Twist()
#             twist_msg.linear.x = 0.0
#             twist_msg.angular.z = 0.0
#             self.cmd_vel_pub.publish(twist_msg)
            
#             # ส่งข้อมูลการมาถึงจุดหมายไปยัง web clients
#             try:
#                 arrival_data = {
#                     'arrived': True,
#                     'location': self.find_nearest_building(self.destination)
#                 }
#                 self.send_update(arrival_data)
#             except Exception as e:
#                 self.get_logger().error(f"Error sending arrival notification: {e}")
            
#             return
            
#         # Find the lookahead point on the path
#         target_point = self.find_lookahead_point()
        
#         if target_point:
#             # Calculate bearing to the target point
#             g = geod.Inverse(self.current_pos[0], self.current_pos[1], 
#                            target_point[0], target_point[1])
#             bearing = g['azi1']  # Forward azimuth
            
#             # Normalize to 0-360 degrees
#             if bearing < 0:
#                 bearing += 360
            
#             # Create and publish bearing message
#             bearing_msg = Float32()
#             bearing_msg.data = float(bearing)
#             self.bearing_pub.publish(bearing_msg)
            
#             # Calculate steering and velocity
#             twist_msg = self.calculate_control_commands(bearing, distance_to_goal)
            
#             # Publish the cmd_vel message to control the vehicle
#             self.cmd_vel_pub.publish(twist_msg)
            
#             self.get_logger().debug(f'Pure Pursuit: bearing={bearing:.2f}°, steering={twist_msg.angular.z:.2f}, speed={twist_msg.linear.x:.2f}')
    
#     def calculate_control_commands(self, target_bearing, distance_to_goal):
#         """Calculate steering angle and velocity based on Pure Pursuit geometry"""
#         twist_msg = Twist()
        
#         # Calculate heading error (difference between current heading and target bearing)
#         heading_error = target_bearing - self.current_heading
#         if heading_error > 180:
#             heading_error -= 360
#         elif heading_error < -180:
#             heading_error += 360
            
#         # Calculate steering angle using Pure Pursuit geometry
#         alpha = math.radians(heading_error)
#         steering_angle = math.atan2(2.0 * self.wheelbase * math.sin(alpha), self.lookahead_distance)
        
#         # Convert to angular velocity and apply limits
#         angular_velocity = steering_angle
#         angular_velocity = max(min(angular_velocity, math.radians(self.max_steering_angle)), -math.radians(self.max_steering_angle))
        
#         # เพิ่มการลดความเร็วในโค้งหรือเมื่อใกล้ถึงจุดหมาย
#         turn_factor = 1.0 - min(1.0, 1.5 * abs(angular_velocity) / math.radians(self.max_steering_angle))
#         distance_factor = min(1.0, distance_to_goal / 5.0)  # Slow down when closer than 5m
#         velocity = self.max_velocity * turn_factor * distance_factor
        
#         # Set the Twist message
#         twist_msg.linear.x = velocity
#         twist_msg.angular.z = angular_velocity
        
#         return twist_msg
        
#     def find_lookahead_point(self):
#         """Find the point on the path that is lookahead_distance away"""
#         if not self.path_points:
#             return None
            
#         # Find the closest point on the path
#         geod = Geodesic.WGS84
#         closest_point_idx = 0
#         min_distance = float('inf')
        
#         for i, point in enumerate(self.path_points):
#             g = geod.Inverse(self.current_pos[0], self.current_pos[1], 
#                            point[0], point[1])
#             distance = g['s12']
            
#             if distance < min_distance:
#                 min_distance = distance
#                 closest_point_idx = i
        
#         # หาจุดที่เหมาะสมบนเส้นทางโดยเริ่มจากจุดที่ใกล้ที่สุด
#         # และค้นหาจุดที่อยู่ถัดไปจนกว่าจะได้ระยะ lookahead
#         # ปรับปรุงจากเวอร์ชันเดิมโดยตัดจุดที่ผ่านไปแล้วออกจากเส้นทาง
        
#         # ตัดเส้นทางให้เริ่มต้นจากจุดที่อยู่ใกล้ที่สุด
#         # แต่ให้เหลือจุดอ้างอิงย้อนหลังหนึ่งจุดเพื่อการคำนวณที่ดีขึ้น
#         start_idx = max(0, closest_point_idx - 1)
        
#         # ค้นหาจุดที่อยู่ห่างออกไปตามระยะที่กำหนด
#         for i in range(start_idx, len(self.path_points)):
#             point = self.path_points[i]
#             g = geod.Inverse(self.current_pos[0], self.current_pos[1], 
#                            point[0], point[1])
#             distance = g['s12']
            
#             if distance >= self.lookahead_distance:
#                 # พบจุดที่เหมาะสม - ใช้จุดนี้เป็น target
#                 return point
        
#         # If no point is far enough, return the last point
#         return self.path_points[-1]
    

# def main(args=None):
#     rclpy.init(args=args)
#     navigator = PurePursuitNavigator()
#     rclpy.spin(navigator)
    
#     navigator.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()




#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist
import math
import json
import numpy as np
from geographiclib.geodesic import Geodesic
import utm  # เพิ่ม utm library
import websockets
import threading
import asyncio
import concurrent.futures
import time
import matplotlib.pyplot as plt

class PurePursuitNavigator(Node):
    def __init__(self):
        super().__init__('pure_pursuit_navigator')
        
        # Subscribe to GNSS topic
        self.gnss_sub = self.create_subscription(
            NavSatFix,
            '/navigation/gnss',
            self.gnss_callback,
            10)
            
        # Subscribe to destination topic
        self.destination_sub = self.create_subscription(
            String,
            '/navigation/destination',
            self.destination_callback,
            10)

        self.heading_sub = self.create_subscription(
            Float32,
            '/navigation/heading',
            self.heading_callback,
            10)
            
        # Publisher for bearing control
        self.bearing_pub = self.create_publisher(
            Float32,
            '/navigation/bearing',
            10)
        
        # Publisher for velocity control (cmd_vel)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Current state variables - ทั้ง lat/long และ UTM
        self.current_pos_latlon = None   # Current position (lat, lon)
        self.current_pos_utm = None      # Current position (x, y) in UTM
        self.prev_pos_utm = None         # Previous position in UTM for heading calculation
        self.current_heading = 0.0       # Current heading in degrees
        self.heading_source = 'INIT'
        self.destination_latlon = None   # Destination (lat, lon)
        self.destination_utm = None      # Destination (x, y) in UTM
        self.path_points_utm = []        # Path points in UTM coordinates
        self.lookahead_distance = 2.0    # Lookahead distance in meters
        self.base_lookahead_distance = 2.0
        self.waypoint_spacing = 3.5      # สร้าง waypoint ทุกๆ 3.5 เมตร
        self.arrival_threshold = 2.5     # ระยะห่างจากจุดหมายที่ถือว่าถึงแล้ว (เมตร)
        
        # UTM zone information (จะกำหนดเมื่อได้ตำแหน่งแรก)
        self.utm_zone_number = None
        self.utm_zone_letter = None
        
        # Vehicle parameters
        self.wheelbase = 1.67  # ความยาวระหว่างล้อหน้าและล้อหลัง (เมตร)
        self.max_steering_angle = 45.0  # มุมเลี้ยวสูงสุด (องศา)
        self.max_velocity = 1.5  # ความเร็วสูงสุด (m/s)
        self.min_turn_velocity = 0.25  # keep creeping while steering catches up
        self.current_speed = 0.0
        self.last_commanded_speed = 0.0

        # RTK quality gate and low-speed Pure Pursuit safety limits.
        self.rtk_state = 'NO_FIX'
        self.fix_quality = 0
        self.float_started_time = None
        self.last_gnss_time = None
        self.last_fix_time = None
        self.last_good_pos_utm = None
        self.last_good_pos_latlon = None
        self.allow_waypoint_progress = False
        self.position_is_predicted = False
        self.float_prediction_timeout = 1.0
        self.float_slow_speed = 0.4
        self.fix_max_speed = 1.0
        self.no_fix_stop_timeout = 1.0
        self.position_jump_margin = 0.35
        self.min_jump_threshold = 0.45
        self.outlier_check_timeout = 2.0
        self.max_index_step_float = 2
        self.last_closest_point_idx = 0
        self.last_target_point_idx = 0
        
        # Steering system parameters (ตามสเปคของพวงมาลัย)
        self.steering_max_angle_deg = 50.0    # มุมเลี้ยวสูงสุด ±50 องศา
        self.steering_max_angle_rad = math.radians(50.0)  # แปลงเป็น radian = ±0.873
        self.steering_max_dec = 15000         # ค่า DEC สูงสุด ±15000
        
        # Available destinations
        self.available_destinations = {
            "S14": {"lat": 13.650940, "lon": 100.492070},
            "S13": {"lat": 13.650076, "lon": 100.492140},
            "S12": {"lat": 13.649913, "lon": 100.492690},
            "S11": {"lat": 13.649992, "lon": 100.493327},
            "S15": {"lat": 13.650154, "lon": 100.492984},
            "N19": {"lat": 13.651010, "lon": 100.492980},
            "N20": {"lat": 13.6513894, "lon": 100.4929824}
        }
        
        # Setup WebSocket server variables
        self.ws_server = None
        self._clients = set()
        self.ws_lock = threading.Lock()
        
        # สร้าง event loop สำหรับ asyncio
        self.loop = asyncio.new_event_loop()
        
        # เริ่ม WebSocket server
        self.start_websocket_server()
        
        # Timer for path following at 10Hz
        self.create_timer(0.1, self.path_following_callback)
        
        self.get_logger().info('Pure Pursuit Navigator with UTM projection and steering correction initialized')

    def heading_callback(self, msg):
        self.current_heading = msg.data % 360.0
        self.heading_source = 'IMU'
    
    def steering_angle_to_angular_velocity(self, steering_angle_rad):
        """แปลงมุมเลี้ยว (radian) เป็นค่า angular velocity สำหรับ cmd_vel"""
        # จำกัดมุมเลี้ยวให้อยู่ในช่วงที่กำหนด
        steering_angle_rad = max(min(steering_angle_rad, self.steering_max_angle_rad), 
                                -self.steering_max_angle_rad)
        
        # แปลงมุมเลี้ยวเป็นค่า angular velocity
        # ใช้สัดส่วนตรงกับช่วงมุมเลี้ยวสูงสุด
        angular_velocity = steering_angle_rad
        
        self.get_logger().debug(f'Steering: {math.degrees(steering_angle_rad):.2f}° -> Angular velocity: {angular_velocity:.4f} rad/s')
        
        return angular_velocity
    
    def steering_angle_to_dec_value(self, steering_angle_rad):
        """แปลงมุมเลี้ยว (radian) เป็นค่า DEC สำหรับ debug/monitoring"""
        # จำกัดมุมเลี้ยวให้อยู่ในช่วงที่กำหนด
        steering_angle_rad = max(min(steering_angle_rad, self.steering_max_angle_rad), 
                                -self.steering_max_angle_rad)
        
        # แปลงเป็นค่า DEC โดยใช้อัตราส่วน
        dec_value = (steering_angle_rad / self.steering_max_angle_rad) * self.steering_max_dec
        
        return int(dec_value)

    def stop_vehicle(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.last_commanded_speed = 0.0
        self.current_speed = 0.0
        self.cmd_vel_pub.publish(twist_msg)
    
    def latlon_to_utm(self, lat, lon):
        """แปลง lat/lon เป็น UTM coordinates"""
        try:
            x, y, zone_number, zone_letter = utm.from_latlon(lat, lon)
            
            # กำหนด UTM zone ครั้งแรก
            if self.utm_zone_number is None:
                self.utm_zone_number = zone_number
                self.utm_zone_letter = zone_letter
                self.get_logger().info(f'UTM Zone set to: {zone_number}{zone_letter}')
            
            # ตรวจสอบว่าอยู่ใน zone เดียวกัน
            if zone_number != self.utm_zone_number or zone_letter != self.utm_zone_letter:
                self.get_logger().warn(f'UTM zone changed! Current: {zone_number}{zone_letter}, Expected: {self.utm_zone_number}{self.utm_zone_letter}')
            
            return x, y
        except Exception as e:
            self.get_logger().error(f'Error converting lat/lon to UTM: {e}')
            return None, None
    
    def utm_to_latlon(self, x, y):
        """แปลง UTM coordinates เป็น lat/lon"""
        try:
            if self.utm_zone_number is None or self.utm_zone_letter is None:
                self.get_logger().error('UTM zone not set')
                return None, None
                
            lat, lon = utm.to_latlon(x, y, self.utm_zone_number, self.utm_zone_letter)
            return lat, lon
        except Exception as e:
            self.get_logger().error(f'Error converting UTM to lat/lon: {e}')
            return None, None
    
    def calculate_heading_utm(self, from_utm, to_utm):
        """คำนวณ heading จาก UTM coordinates (เร็วกว่าการใช้ geodesic)"""
        if not from_utm or not to_utm:
            return None
            
        dx = to_utm[0] - from_utm[0]  # East direction
        dy = to_utm[1] - from_utm[1]  # North direction
        
        # คำนวณ bearing ในหน่วยเรเดียน
        bearing_rad = math.atan2(dx, dy)
        
        # แปลงเป็นองศาและปรับให้อยู่ในช่วง 0-360
        bearing_deg = math.degrees(bearing_rad)
        if bearing_deg < 0:
            bearing_deg += 360
            
        return bearing_deg
    
    def distance_utm(self, point1_utm, point2_utm):
        """คำนวณระยะทางระหว่างสองจุดใน UTM coordinates"""
        if not point1_utm or not point2_utm:
            return 0
            
        dx = point2_utm[0] - point1_utm[0]
        dy = point2_utm[1] - point1_utm[1]
        return math.sqrt(dx*dx + dy*dy)
    
    def start_websocket_server(self):
        """Start WebSocket server in a separate thread"""
        def run_server():
            asyncio.set_event_loop(self.loop)
            
            async def handler(websocket, path):
                # Register new client
                with self.ws_lock:
                    self._clients.add(websocket)
                self.get_logger().info(f"New client connected. Total clients: {len(self._clients)}")
                
                try:
                    async for message in websocket:
                        try:
                            data = json.loads(message)
                            
                            # Handle destination
                            if 'destination' in data:
                                dest_code = data['destination']
                                self.get_logger().info(f"Received destination from web: {dest_code}")
                                
                                # Process as if received from ROS topic
                                msg = String()
                                msg.data = dest_code
                                self.destination_callback(msg)
                            
                            # Handle path data sent from frontend
                            if 'path' in data and isinstance(data['path'], list) and len(data['path']) > 0:
                                # Convert the path points from web format to UTM
                                new_path_utm = []
                                for point in data['path']:
                                    if 'lat' in point and 'lon' in point:
                                        x, y = self.latlon_to_utm(float(point['lat']), float(point['lon']))
                                        if x is not None and y is not None:
                                            new_path_utm.append((x, y))
                                
                                if len(new_path_utm) > 0:
                                    self.get_logger().info(f"Received path from web with {len(new_path_utm)} points")
                                    # ทำการสร้าง waypoints ที่กระจายตัวสม่ำเสมอ
                                    self.path_points_utm = self.resample_path_utm(new_path_utm)
                                    self.last_closest_point_idx = 0
                                    self.last_target_point_idx = 0
                                    self.get_logger().info(f"Resampled path to {len(self.path_points_utm)} waypoints")
                            
                            # Handle bearing data for debugging
                            if 'bearing' in data:
                                bearing = float(data['bearing'])
                                # สามารถใช้สำหรับ debug หรือกลยุทธ์การควบคุมอื่นๆ
                        
                        except json.JSONDecodeError:
                            self.get_logger().error(f"Failed to parse JSON message: {message}")
                        except Exception as e:
                            self.get_logger().error(f"Error processing message: {e}")
                
                except websockets.exceptions.ConnectionClosed:
                    pass
                finally:
                    # Unregister client
                    with self.ws_lock:
                        self._clients.discard(websocket)
                    self.get_logger().info(f"Client disconnected. Total clients: {len(self._clients)}")
            
            # Start the WebSocket server
            async def main():
                self.ws_server = await websockets.serve(handler, "0.0.0.0", 5000)
                self.get_logger().info("WebSocket server started on port 5000")
                await self.ws_server.wait_closed()
            
            self.loop.run_until_complete(main())
            self.loop.run_forever()
        
        # Start the server in a separate thread
        threading.Thread(target=run_server, daemon=True).start()
    
    async def send_to_clients(self, data):
        """Send data to all connected WebSocket clients"""
        if not self._clients:
            return
            
        message = json.dumps(data)
        with self.ws_lock:
            clients_copy = self._clients.copy()
        
        for client in clients_copy:
            try:
                # ใช้ asyncio.wait_for เพื่อกำหนด timeout
                await asyncio.wait_for(client.send(message), timeout=0.5)
            except asyncio.TimeoutError:
                self.get_logger().warn(f"Timeout sending message to client")
            except websockets.exceptions.ConnectionClosed:
                with self.ws_lock:
                    self._clients.discard(client)
                self.get_logger().info(f"Removed closed client connection. Total clients: {len(self._clients)}")
            except Exception as e:
                self.get_logger().error(f"Error sending to client: {type(e).__name__}: {e}")
    
    def send_update(self, data):
        """Helper function to send updates via WebSocket from synchronous code"""
        future = asyncio.run_coroutine_threadsafe(self.send_to_clients(data), self.loop)
        try:
            # Wait for a short time to avoid blocking
            future.result(timeout=0.5)
        except (asyncio.TimeoutError, concurrent.futures.TimeoutError):
            self.get_logger().warn("Timeout when sending WebSocket update")
        except Exception as e:
            self.get_logger().error(f"Error in send_update: {type(e).__name__}: {e}")
    
    def resample_path_utm(self, path_utm):
        """Resample path points in UTM coordinates to create evenly spaced waypoints"""
        if len(path_utm) < 2:
            return path_utm
            
        resampled_path = [path_utm[0]]  # Start with the first point
        target_spacing = self.waypoint_spacing  # ระยะห่างที่ต้องการระหว่าง waypoints
        
        for i in range(len(path_utm) - 1):
            start_point = path_utm[i]
            end_point = path_utm[i + 1]
            
            # คำนวณระยะทางระหว่างจุด (ใช้ Euclidean distance ใน UTM)
            segment_distance = self.distance_utm(start_point, end_point)
            
            # ถ้าระยะทางน้อยกว่าที่กำหนด ข้ามไปเลย
            if segment_distance < target_spacing * 0.5:
                continue
                
            # จำนวน waypoints ที่ต้องการระหว่างจุดต้นทางและปลายทาง
            num_points = max(1, int(segment_distance / target_spacing))
            
            # สร้าง waypoints ระหว่างจุดต้นทางและปลายทาง
            for j in range(1, num_points):
                fraction = j / num_points
                x = start_point[0] + fraction * (end_point[0] - start_point[0])
                y = start_point[1] + fraction * (end_point[1] - start_point[1])
                resampled_path.append((x, y))
            
            # เพิ่มจุดปลายทางของส่วนนี้
            resampled_path.append(end_point)
        
        return resampled_path

    def get_fix_quality(self, msg):
        """Read GGA gps_qual carried in NavSatFix covariance[0]."""
        try:
            return int(round(msg.position_covariance[0]))
        except (TypeError, ValueError, IndexError):
            if msg.status.status > 0:
                return 4
            if msg.status.status == 0:
                return 1
            return 0

    def classify_rtk_state(self, fix_quality):
        if fix_quality == 4:
            return 'FIX'
        if fix_quality == 5:
            return 'FLOAT'
        return 'NO_FIX'

    def is_position_outlier(self, new_pos_utm, now):
        if self.last_gnss_time is None or self.current_pos_utm is None:
            return False

        dt = max(now - self.last_gnss_time, 0.01)
        distance = self.distance_utm(self.current_pos_utm, new_pos_utm)
        speed = max(abs(self.current_speed), abs(self.last_commanded_speed))
        max_possible_move = max(self.min_jump_threshold, speed * dt + self.position_jump_margin)
        return distance > max_possible_move

    def predict_position(self, dt):
        if self.current_pos_utm is None:
            return None

        speed = max(0.0, min(abs(self.current_speed or self.last_commanded_speed), self.fix_max_speed))
        yaw_rad = math.radians(self.current_heading)

        # current_heading is compass bearing: 0=N, 90=E.
        dx = speed * math.sin(yaw_rad) * dt
        dy = speed * math.cos(yaw_rad) * dt
        return (self.current_pos_utm[0] + dx, self.current_pos_utm[1] + dy)

    def apply_gnss_position_gate(self, gps_pos_utm, gps_pos_latlon, fix_quality, now):
        new_state = self.classify_rtk_state(fix_quality)
        self.fix_quality = fix_quality

        if new_state != 'FLOAT':
            self.float_started_time = None
        elif self.rtk_state != 'FLOAT' or self.float_started_time is None:
            self.float_started_time = now

        self.rtk_state = new_state
        self.allow_waypoint_progress = False
        self.position_is_predicted = False

        if new_state == 'FIX':
            recent_fix = self.last_fix_time is not None and now - self.last_fix_time <= self.outlier_check_timeout
            if recent_fix and self.is_position_outlier(gps_pos_utm, now):
                self.get_logger().warn('Rejected GNSS outlier while RTK FIX')
                return

            self.prev_pos_utm = self.current_pos_utm
            self.current_pos_utm = gps_pos_utm
            self.current_pos_latlon = gps_pos_latlon
            self.last_good_pos_utm = gps_pos_utm
            self.last_good_pos_latlon = gps_pos_latlon
            self.last_fix_time = now
            self.last_gnss_time = now
            self.allow_waypoint_progress = True
            self.lookahead_distance = self.base_lookahead_distance
            return

        if new_state == 'FLOAT':
            float_time = now - self.float_started_time
            if float_time <= self.float_prediction_timeout:
                dt = 0.1 if self.last_gnss_time is None else max(now - self.last_gnss_time, 0.01)
                predicted_pos = self.predict_position(dt)
                if predicted_pos is not None:
                    self.prev_pos_utm = self.current_pos_utm
                    self.current_pos_utm = predicted_pos
                    self.position_is_predicted = True
                    self.lookahead_distance = max(self.base_lookahead_distance, 2.5)
                    self.last_gnss_time = now
                return

            self.stop_vehicle()
            self.get_logger().warn('RTK FLOAT too long; vehicle stopped')
            return

        if self.last_gnss_time and now - self.last_gnss_time < self.no_fix_stop_timeout:
            dt = max(now - self.last_gnss_time, 0.01)
            predicted_pos = self.predict_position(dt)
            if predicted_pos is not None:
                self.prev_pos_utm = self.current_pos_utm
                self.current_pos_utm = predicted_pos
                self.position_is_predicted = True
                self.lookahead_distance = max(self.base_lookahead_distance, 2.5)
                self.last_gnss_time = now
                return

        self.stop_vehicle()
        self.get_logger().warn('No valid RTK position; vehicle stopped')
    
    def gnss_callback(self, msg):
        now = time.time()
        fix_quality = self.get_fix_quality(msg)
        gps_pos_latlon = (msg.latitude, msg.longitude)
        x, y = self.latlon_to_utm(msg.latitude, msg.longitude)
        if x is not None and y is not None:
            self.apply_gnss_position_gate((x, y), gps_pos_latlon, fix_quality, now)
            self.get_logger().debug(
                f'Position gate: state={self.rtk_state}, fix_q={fix_quality}, '
                f'predicted={self.position_is_predicted}, heading={self.current_heading:.2f} '
                f'({self.heading_source})'
            )
        
        # Send position update to connected clients (ส่งในรูป lat/lon)
        try:
            data = {
                'latitude': self.current_pos_latlon[0] if self.current_pos_latlon else msg.latitude,
                'longitude': self.current_pos_latlon[1] if self.current_pos_latlon else msg.longitude,
                'heading': self.current_heading,
                'rtk_state': self.rtk_state,
                'fix_quality': self.fix_quality,
                'position_predicted': self.position_is_predicted
            }
            
            self.send_update(data)
        except Exception as e:
            self.get_logger().error(f"Error sending position update to clients: {type(e).__name__}: {e}")
        
        # Check if we need to generate a path (if destination is set but no path)
        if self.destination_utm and not self.path_points_utm:
            self.generate_path()
    
    def destination_callback(self, msg):
        destination_code = msg.data
        
        if destination_code in self.available_destinations:
            dest_info = self.available_destinations[destination_code]
            self.destination_latlon = (dest_info["lat"], dest_info["lon"])
            
            # แปลงปลายทางเป็น UTM
            x, y = self.latlon_to_utm(dest_info["lat"], dest_info["lon"])
            if x is not None and y is not None:
                self.destination_utm = (x, y)
                
                # ล้างเส้นทางเก่า
                self.path_points_utm = []
                self.last_closest_point_idx = 0
                self.last_target_point_idx = 0
                
                # Generate path only if we don't expect to receive one from the web interface
                if self.current_pos_utm and len(self._clients) == 0:
                    self.generate_path()
                
                self.get_logger().info(f'New destination set: {destination_code} at {self.destination_latlon}, UTM: ({x:.2f}, {y:.2f})')
            else:
                self.get_logger().error(f'Failed to convert destination to UTM: {destination_code}')
        else:
            self.get_logger().warn(f'Unknown destination code: {destination_code}')
    
    def generate_path(self):
        """Generate a path from current position to destination with waypoints"""
        if not self.current_pos_utm or not self.destination_utm:
            return
            
        # Clear existing path
        self.path_points_utm = []
        self.last_closest_point_idx = 0
        self.last_target_point_idx = 0
        
        # Get straight-line distance and bearing (ใช้ UTM coordinates)
        total_distance = self.distance_utm(self.current_pos_utm, self.destination_utm)
        bearing = self.calculate_heading_utm(self.current_pos_utm, self.destination_utm)
        
        # หา waypoints สำหรับเส้นทางที่มีทางเลี้ยว
        custom_path = self.find_custom_path()
        
        if custom_path:
            # แปลงเส้นทางกำหนดเองเป็น UTM
            custom_path_utm = []
            for lat, lon in custom_path:
                x, y = self.latlon_to_utm(lat, lon)
                if x is not None and y is not None:
                    custom_path_utm.append((x, y))
            
            if custom_path_utm:
                self.path_points_utm = custom_path_utm
                self.last_closest_point_idx = 0
                self.last_target_point_idx = 0
                self.get_logger().info(f'Using predefined path with {len(self.path_points_utm)} waypoints')
        else:
            # สร้าง waypoints แบบเส้นตรง
            num_points = max(3, int(total_distance / self.waypoint_spacing))
            
            straight_path_utm = []
            for i in range(num_points + 1):
                t = i / num_points
                x = self.current_pos_utm[0] + t * (self.destination_utm[0] - self.current_pos_utm[0])
                y = self.current_pos_utm[1] + t * (self.destination_utm[1] - self.current_pos_utm[1])
                straight_path_utm.append((x, y))
            
            # Add destination as final point
            if straight_path_utm[-1] != self.destination_utm:
                straight_path_utm.append(self.destination_utm)
            
            # Resample to get evenly spaced waypoints
            self.path_points_utm = self.resample_path_utm(straight_path_utm)
            self.last_closest_point_idx = 0
            self.last_target_point_idx = 0
                
            self.get_logger().info(f'Generated straight-line path with {len(self.path_points_utm)} points')
        
        # ส่งเส้นทางไปยัง web interface (แปลงกลับเป็น lat/lon)
        try:
            path_data = []
            for utm_point in self.path_points_utm:
                lat, lon = self.utm_to_latlon(utm_point[0], utm_point[1])
                if lat is not None and lon is not None:
                    path_data.append({'lat': lat, 'lon': lon})
            
            if path_data:
                data = {'path': path_data}
                self.send_update(data)
                self.get_logger().info(f'Sent path with {len(path_data)} points to web clients')
        except Exception as e:
            self.get_logger().error(f"Error sending path to clients: {type(e).__name__}: {e}")
    
    def find_custom_path(self):
        """หาเส้นทางที่กำหนดเองตามตำแหน่งปัจจุบันและปลายทาง"""
        if not self.current_pos_latlon or not self.destination_latlon:
            return None
            
        # Dictionary ของเส้นทางที่กำหนดเอง (ยังคงใช้ lat/lon)
        custom_paths = {
            ("S14", "S11"): [
                (13.650940, 100.492070),  # S14 starting point
                (13.650500, 100.492300),  # waypoint 1 - ทางแยกแรก
                (13.650300, 100.492800),  # waypoint 2 - ทางแยกที่สอง
                (13.649992, 100.493327)   # S11 destination
            ],
            ("S11", "S14"): [
                (13.649992, 100.493327),  # S11 starting point
                (13.650300, 100.492800),  # waypoint 1
                (13.650500, 100.492300),  # waypoint 2
                (13.650940, 100.492070)   # S14 destination
            ],
        }
        
        # หาอาคารที่ใกล้ที่สุดกับตำแหน่งปัจจุบัน
        start_building = self.find_nearest_building(self.current_pos_latlon)
        # หาอาคารที่ใกล้ที่สุดกับจุดหมาย
        end_building = self.find_nearest_building(self.destination_latlon)
        
        path_key = (start_building, end_building)
        if path_key in custom_paths:
            path = custom_paths[path_key]
            # ปรับปรุงจุดเริ่มต้นให้ตรงกับตำแหน่งปัจจุบัน
            return [(self.current_pos_latlon[0], self.current_pos_latlon[1])] + path[1:]
        
        # ถ้าไม่มีเส้นทางที่กำหนดเอง ให้ลองพลิกคู่ต้นทาง-ปลายทาง
        reverse_key = (end_building, start_building)
        if reverse_key in custom_paths:
            reversed_path = list(reversed(custom_paths[reverse_key]))
            return [(self.current_pos_latlon[0], self.current_pos_latlon[1])] + reversed_path[1:]
        
        return None
    
    def find_nearest_building(self, position):
        """หาอาคารที่ใกล้ที่สุดกับตำแหน่งที่กำหนด"""
        geod = Geodesic.WGS84
        nearest_building = None
        min_distance = float('inf')
        
        for code, building in self.available_destinations.items():
            g = geod.Inverse(position[0], position[1], building["lat"], building["lon"])
            distance = g['s12']
            
            if distance < min_distance:
                min_distance = distance
                nearest_building = code
        
        return nearest_building
    
    def path_following_callback(self):
        """Main Pure Pursuit algorithm implementation using UTM coordinates"""
        if not self.current_pos_utm or not self.path_points_utm:
            return
            
        # Check if we've reached the destination (ใช้ UTM coordinates)
        distance_to_goal = self.distance_utm(self.current_pos_utm, self.path_points_utm[-1])
        
        if distance_to_goal <= self.arrival_threshold:
            # We've arrived at the destination
            self.get_logger().info(f'Arrived at destination! Distance: {distance_to_goal:.2f}m')
            
            # Stop the vehicle
            self.stop_vehicle()
            
            # ส่งข้อมูลการมาถึงจุดหมายไปยัง web clients
            try:
                arrival_data = {
                    'arrived': True,
                    'location': self.find_nearest_building(self.destination_latlon)
                }
                self.send_update(arrival_data)
            except Exception as e:
                self.get_logger().error(f"Error sending arrival notification: {e}")
            
            return
            
        # Find the lookahead point on the path (ใช้ UTM coordinates)
        target_point_utm = self.find_lookahead_point_utm()
        
        if target_point_utm:
            # Calculate bearing to the target point (ใช้ UTM coordinates)
            bearing = self.calculate_heading_utm(self.current_pos_utm, target_point_utm)
            
            # Create and publish bearing message
            bearing_msg = Float32()
            bearing_msg.data = float(bearing)
            self.bearing_pub.publish(bearing_msg)
            
            # Calculate steering and velocity
            twist_msg = self.calculate_control_commands(bearing, distance_to_goal)
            
            # Publish the cmd_vel message to control the vehicle
            self.cmd_vel_pub.publish(twist_msg)
            self.last_commanded_speed = twist_msg.linear.x
            self.current_speed = twist_msg.linear.x
            
            # คำนวณค่า DEC สำหรับ debug
            dec_value = self.steering_angle_to_dec_value(twist_msg.angular.z)
            
            self.get_logger().debug(
                f'Pure Pursuit: rtk={self.rtk_state}, fix_q={self.fix_quality}, '
                f'predicted={self.position_is_predicted}, bearing={bearing:.2f}°, '
                f'steering={math.degrees(twist_msg.angular.z):.2f}°, DEC={dec_value}, '
                f'speed={twist_msg.linear.x:.2f}'
            )
    
    def calculate_control_commands(self, target_bearing, distance_to_goal):
        """Calculate steering angle and velocity based on Pure Pursuit geometry"""
        twist_msg = Twist()
        
        # Calculate heading error (difference between current heading and target bearing)
        heading_error = target_bearing - self.current_heading
        if heading_error > 180:
            heading_error -= 360
        elif heading_error < -180:
            heading_error += 360
            
        # Calculate steering angle using Pure Pursuit geometry
        alpha = math.radians(heading_error)
        steering_angle = math.atan2(2.0 * self.wheelbase * math.sin(alpha), self.lookahead_distance)
        
        # จำกัดมุมเลี้ยวให้อยู่ในช่วงที่พวงมาลัยรองรับ (±50°)
        max_steering_rad = self.steering_max_angle_rad  # ±0.873 rad (±50°)
        steering_angle = max(min(steering_angle, max_steering_rad), -max_steering_rad)
        
        # แปลงมุมเลี้ยวเป็น angular velocity สำหรับ cmd_vel
        angular_velocity = self.steering_angle_to_angular_velocity(steering_angle)
        
        # เพิ่มการลดความเร็วในโค้งหรือเมื่อใกล้ถึงจุดหมาย
        turn_factor = 1.0 - min(0.85, 1.2 * abs(steering_angle) / max_steering_rad)
        distance_factor = min(1.0, distance_to_goal / 5.0)  # Slow down when closer than 5m
        speed_limit = self.fix_max_speed
        if self.rtk_state == 'FLOAT':
            speed_limit = self.float_slow_speed
        elif self.rtk_state != 'FIX':
            speed_limit = 0.0

        velocity = min(self.max_velocity, speed_limit) * turn_factor * distance_factor
        if distance_to_goal > self.arrival_threshold and abs(steering_angle) > math.radians(5.0):
            velocity = max(min(self.min_turn_velocity, speed_limit), velocity)

        if self.rtk_state != 'FIX':
            angular_velocity *= 0.5
        
        # Set the Twist message
        twist_msg.linear.x = velocity
        twist_msg.angular.z = angular_velocity
        
        return twist_msg
        
    def find_lookahead_point_utm(self):
        """Find the point on the path that is lookahead_distance away (ใช้ UTM coordinates)"""
        if not self.path_points_utm:
            return None

        if not self.allow_waypoint_progress and self.last_target_point_idx < len(self.path_points_utm):
            return self.path_points_utm[self.last_target_point_idx]

        # Find the closest point on the path
        closest_point_idx = self.last_closest_point_idx
        min_distance = float('inf')
        
        for i, point in enumerate(self.path_points_utm):
            distance = self.distance_utm(self.current_pos_utm, point)
            
            if distance < min_distance:
                min_distance = distance
                closest_point_idx = i

        if self.rtk_state != 'FIX':
            closest_point_idx = min(
                closest_point_idx,
                self.last_closest_point_idx + self.max_index_step_float
            )
        self.last_closest_point_idx = max(self.last_closest_point_idx, closest_point_idx)
        
        # หาจุดที่เหมาะสมบนเส้นทางโดยเริ่มจากจุดที่ใกล้ที่สุด
        start_idx = max(0, closest_point_idx - 1)
        
        # ค้นหาจุดที่อยู่ห่างออกไปตามระยะที่กำหนด
        for i in range(start_idx, len(self.path_points_utm)):
            point = self.path_points_utm[i]
            distance = self.distance_utm(self.current_pos_utm, point)
            
            if distance >= self.lookahead_distance:
                # พบจุดที่เหมาะสม - ใช้จุดนี้เป็น target
                if self.rtk_state != 'FIX':
                    i = min(i, self.last_target_point_idx + self.max_index_step_float)
                self.last_target_point_idx = max(self.last_target_point_idx, i)
                return self.path_points_utm[self.last_target_point_idx]
        
        # If no point is far enough, return the last point
        self.last_target_point_idx = len(self.path_points_utm) - 1
        return self.path_points_utm[-1]
    

def main(args=None):
    rclpy.init(args=args)
    navigator = PurePursuitNavigator()
    rclpy.spin(navigator)
    
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
