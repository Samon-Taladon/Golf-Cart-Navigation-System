# # #!/usr/bin/env python3

# # import serial
# # import pynmea2
# # import socket
# # import rclpy
# # from rclpy.node import Node
# # from sensor_msgs.msg import NavSatFix
# # from std_msgs.msg import Float32
# # from geometry_msgs.msg import Point
# # import math
# # from pyproj import Transformer
# # from collections import deque
# # import time

# # class EnhancedGNSSPublisher(Node):
# #     def __init__(self):
# #         super().__init__('enhanced_gnss_publisher')
        
# #         # ROS2 Publishers
# #         self.gnss_publisher = self.create_publisher(NavSatFix, '/navigation/gnss', 10)
# #         self.xy_publisher = self.create_publisher(Point, '/navigation/xy_position', 10)
# #         self.heading_publisher = self.create_publisher(Float32, '/navigation/heading', 10)
        
# #         # Serial configuration
# #         self.serial_port = '/dev/ttyACM0'
        
# #         # NTRIP configuration
# #         self.ntrip_server_ip = "110.78.0.54"
# #         self.ntrip_server_port = 2116
# #         self.ntrip_username = "1118600009224"
# #         self.ntrip_password = "CK79"
# #         self.mount_point = "VRS_RTCM32"
        
# #         # Coordinate transformation - UTM Zone 47N for Thailand
# #         self.transformer = Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)
        
# #         # Position tracking for heading calculation
# #         self.position_history = deque(maxlen=10)
# #         self.current_heading = 0.0
# #         self.heading_initialized = False
# #         self.min_movement_for_heading = 0.3  # minimum movement in meters
        
# #         # Reference point for local coordinate system (first valid GNSS position)
# #         self.reference_point = None  # (lat, lon)
# #         self.reference_utm = None    # (x, y) in UTM
        
# #         # Initialize serial connection
# #         try:
# #             self.ser = serial.Serial(self.serial_port, 9600, timeout=1)
# #             self.get_logger().info(f"Connected to {self.serial_port}")
# #         except Exception as e:
# #             self.get_logger().error(f"Error connecting to serial port {self.serial_port}: {e}")
# #             return
        
# #         # Connect to NTRIP server
# #         self.ntrip_socket = self.connect_to_ntrip()
        
# #         # Timer for reading GNSS data
# #         self.create_timer(0.1, self.read_gnss_data)
        
# #         self.get_logger().info('Enhanced GNSS Publisher initialized with XY conversion and heading calculation')
    
# #     def connect_to_ntrip(self):
# #         """Connect to NTRIP server for RTK corrections"""
# #         try:
# #             client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# #             client_socket.connect((self.ntrip_server_ip, self.ntrip_server_port))
            
# #             import base64
# #             auth_str = f"{self.ntrip_username}:{self.ntrip_password}"
# #             auth_bytes = auth_str.encode('ascii')
# #             auth_b64 = base64.b64encode(auth_bytes).decode('ascii')
            
# #             request = (
# #                 f"GET /{self.mount_point} HTTP/1.1\r\n"
# #                 f"User-Agent: NTRIP Client\r\n"
# #                 f"Authorization: Basic {auth_b64}\r\n"
# #                 f"\r\n"
# #             )
# #             client_socket.send(request.encode('ascii'))
# #             self.get_logger().info("Connected to NTRIP server")
# #             return client_socket
# #         except Exception as e:
# #             self.get_logger().error(f"Error connecting to NTRIP server: {e}")
# #             return None
    
# #     def read_gnss_data(self):
# #         """Read and process GNSS data"""
# #         if hasattr(self, 'ser') and self.ser.is_open:
# #             try:
# #                 data = self.ser.readline().decode('ascii', errors='replace')
                
# #                 if data.startswith('$GPGGA') or data.startswith('$GNGLL') or data.startswith('$GPRMC'):
# #                     try:
# #                         msg = pynmea2.parse(data)
                        
# #                         latitude = None
# #                         longitude = None
                        
# #                         # Extract coordinates from different NMEA message types
# #                         if isinstance(msg, pynmea2.types.talker.GGA):
# #                             latitude = msg.latitude
# #                             longitude = msg.longitude
# #                         elif isinstance(msg, pynmea2.types.talker.GLL):
# #                             latitude = msg.latitude
# #                             longitude = msg.longitude
# #                         elif isinstance(msg, pynmea2.types.talker.RMC):
# #                             latitude = msg.latitude
# #                             longitude = msg.longitude
                        
# #                         # Process valid coordinates
# #                         if latitude is not None and longitude is not None:
# #                             self.process_gnss_position(latitude, longitude)
                        
# #                         # Handle RTK corrections
# #                         self.handle_ntrip_data()
                    
# #                     except pynmea2.nmea.ChecksumError:
# #                         self.get_logger().warning("Checksum error in NMEA data")
            
# #             except Exception as e:
# #                 self.get_logger().error(f"Error reading serial data: {e}")
    
# #     def process_gnss_position(self, latitude, longitude):
# #         """Process GNSS position and convert to XY coordinates"""
# #         current_time = time.time()
        
# #         # Validate coordinates
# #         if not (-90 <= latitude <= 90) or not (-180 <= longitude <= 180):
# #             self.get_logger().warning(f"Invalid coordinates: lat={latitude}, lon={longitude}")
# #             return
        
# #         # Set reference point on first valid position
# #         if self.reference_point is None:
# #             self.reference_point = (latitude, longitude)
# #             self.reference_utm = self.transformer.transform(longitude, latitude)
# #             self.get_logger().info(f"Reference point set: lat={latitude:.7f}, lon={longitude:.7f}")
# #             self.get_logger().info(f"Reference UTM: x={self.reference_utm[0]:.2f}, y={self.reference_utm[1]:.2f}")
        
# #         # Convert to UTM coordinates
# #         utm_x, utm_y = self.transformer.transform(longitude, latitude)
        
# #         # Calculate local XY relative to reference point
# #         local_x = utm_x - self.reference_utm[0]
# #         local_y = utm_y - self.reference_utm[1]
        
# #         # Create position record
# #         position_record = {
# #             'lat': latitude,
# #             'lon': longitude,
# #             'utm_x': utm_x,
# #             'utm_y': utm_y,
# #             'local_x': local_x,
# #             'local_y': local_y,
# #             'timestamp': current_time
# #         }
        
# #         # Add to position history
# #         self.position_history.append(position_record)
        
# #         # Calculate and update heading
# #         self.update_heading()
        
# #         # Publish all data
# #         self.publish_data(latitude, longitude, local_x, local_y)
        
# #         # Log position data
# #         self.get_logger().info(
# #             f"GNSS: lat={latitude:.7f}, lon={longitude:.7f} | "
# #             f"Local XY: x={local_x:.2f}, y={local_y:.2f} | "
# #             f"Heading: {self.current_heading:.1f}°"
# #         )
    
# #     def update_heading(self):
# #         """Calculate heading from movement using XY coordinates"""
# #         if len(self.position_history) < 2:
# #             return
        
# #         # Use multiple points for better accuracy
# #         total_distance = 0.0
# #         weighted_headings = []
        
# #         # Calculate heading from recent movement
# #         for i in range(len(self.position_history) - 1, max(0, len(self.position_history) - 5), -1):
# #             if i == 0:
# #                 break
            
# #             current_pos = self.position_history[i]
# #             prev_pos = self.position_history[i-1]
            
# #             # Calculate movement in local XY
# #             dx = current_pos['local_x'] - prev_pos['local_x']
# #             dy = current_pos['local_y'] - prev_pos['local_y']
# #             distance = math.sqrt(dx*dx + dy*dy)
            
# #             if distance > self.min_movement_for_heading:
# #                 # Calculate heading in standard navigation format (0° = North, clockwise)
# #                 heading_rad = math.atan2(dx, dy)  # atan2(East, North)
# #                 heading_deg = math.degrees(heading_rad)
                
# #                 # Normalize to 0-360°
# #                 if heading_deg < 0:
# #                     heading_deg += 360
                
# #                 weighted_headings.append((heading_deg, distance))
# #                 total_distance += distance
        
# #         # Calculate weighted average heading if we have sufficient movement
# #         if total_distance > self.min_movement_for_heading and weighted_headings:
# #             # Use circular mean for angles
# #             total_x = 0.0
# #             total_y = 0.0
            
# #             for heading, weight in weighted_headings:
# #                 total_x += math.cos(math.radians(heading)) * weight
# #                 total_y += math.sin(math.radians(heading)) * weight
            
# #             if total_distance > 0:
# #                 avg_heading = math.degrees(math.atan2(total_y, total_x))
# #                 if avg_heading < 0:
# #                     avg_heading += 360
                
# #                 # Smooth heading transition for initialized heading
# #                 if self.heading_initialized:
# #                     heading_diff = avg_heading - self.current_heading
                    
# #                     # Handle angle wraparound
# #                     while heading_diff > 180:
# #                         heading_diff -= 360
# #                     while heading_diff < -180:
# #                         heading_diff += 360
                    
# #                     # Only update if change is reasonable (not a GPS jump)
# #                     if abs(heading_diff) < 90:  # Maximum 90° change per update
# #                         self.current_heading = avg_heading
# #                 else:
# #                     self.current_heading = avg_heading
# #                     self.heading_initialized = True
    
# #     def publish_data(self, latitude, longitude, local_x, local_y):
# #         """Publish GNSS, XY position, and heading data"""
# #         # Publish GNSS data
# #         gnss_msg = NavSatFix()
# #         gnss_msg.latitude = latitude
# #         gnss_msg.longitude = longitude
# #         gnss_msg.status.status = 0  # STATUS_FIX
# #         gnss_msg.status.service = 1  # SERVICE_GPS
# #         self.gnss_publisher.publish(gnss_msg)
        
# #         # Publish XY position
# #         xy_msg = Point()
# #         xy_msg.x = local_x
# #         xy_msg.y = local_y
# #         xy_msg.z = 0.0
# #         self.xy_publisher.publish(xy_msg)
        
# #         # Publish heading
# #         if self.heading_initialized:
# #             heading_msg = Float32()
# #             heading_msg.data = float(self.current_heading)
# #             self.heading_publisher.publish(heading_msg)
    
# #     def handle_ntrip_data(self):
# #         """Handle RTK corrections from NTRIP server"""
# #         if self.ntrip_socket:
# #             try:
# #                 self.ntrip_socket.setblocking(0)
# #                 try:
# #                     rtk_data = self.ntrip_socket.recv(1024)
# #                     if rtk_data:
# #                         self.ser.write(rtk_data)
# #                 except socket.error:
# #                     pass  # No data available
# #             except Exception as e:
# #                 self.get_logger().error(f"Error handling NTRIP data: {e}")
    
# #     def get_current_xy_position(self):
# #         """Get current XY position"""
# #         if len(self.position_history) > 0:
# #             latest = self.position_history[-1]
# #             return latest['local_x'], latest['local_y']
# #         return None, None
    
# #     def get_current_heading(self):
# #         """Get current heading in degrees"""
# #         return self.current_heading if self.heading_initialized else None
    
# #     def lat_lon_to_xy(self, lat, lon):
# #         """Convert any lat/lon to local XY coordinates"""
# #         if self.reference_point is None:
# #             return None, None
        
# #         # Convert to UTM
# #         utm_x, utm_y = self.transformer.transform(lon, lat)
        
# #         # Convert to local coordinates
# #         local_x = utm_x - self.reference_utm[0]
# #         local_y = utm_y - self.reference_utm[1]
        
# #         return local_x, local_y
    
# #     def xy_to_lat_lon(self, x, y):
# #         """Convert local XY coordinates back to lat/lon"""
# #         if self.reference_utm is None:
# #             return None, None
        
# #         # Convert to UTM
# #         utm_x = x + self.reference_utm[0]
# #         utm_y = y + self.reference_utm[1]
        
# #         # Convert back to lat/lon
# #         lon, lat = self.transformer.transform(utm_x, utm_y, direction='INVERSE')
        
# #         return lat, lon
    
# #     def calculate_bearing_xy(self, x1, y1, x2, y2):
# #         """Calculate bearing between two XY points"""
# #         dx = x2 - x1
# #         dy = y2 - y1
        
# #         # Calculate bearing (0° = North, clockwise)
# #         bearing_rad = math.atan2(dx, dy)
# #         bearing_deg = math.degrees(bearing_rad)
        
# #         # Normalize to 0-360°
# #         if bearing_deg < 0:
# #             bearing_deg += 360
        
# #         return bearing_deg
    
# #     def destroy_node(self):
# #         """Cleanup when node is destroyed"""
# #         if hasattr(self, 'ser') and self.ser.is_open:
# #             self.ser.close()
# #         if hasattr(self, 'ntrip_socket') and self.ntrip_socket:
# #             self.ntrip_socket.close()
# #         super().destroy_node()

# # def main(args=None):
# #     rclpy.init(args=args)
# #     gnss_publisher = EnhancedGNSSPublisher()
    
# #     try:
# #         rclpy.spin(gnss_publisher)
# #     except KeyboardInterrupt:
# #         pass
# #     finally:
# #         gnss_publisher.destroy_node()
# #         rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()

# # ///////////////////////////////////////////////////////////////

# #!/usr/bin/env python3

# # import serial
# # import pynmea2
# # import socket
# # import rclpy
# # from rclpy.node import Node
# # from sensor_msgs.msg import NavSatFix
# # from std_msgs.msg import Float32
# # from geometry_msgs.msg import Point
# # import math
# # from pyproj import Transformer
# # from collections import deque
# # import time
# # import glob
# # import os

# # class EnhancedGNSSPublisher(Node):
# #     def __init__(self):
# #         super().__init__('enhanced_gnss_publisher')
        
# #         # ROS2 Publishers
# #         self.gnss_publisher = self.create_publisher(NavSatFix, '/navigation/gnss', 10)
# #         self.xy_publisher = self.create_publisher(Point, '/navigation/xy_position', 10)
# #         self.heading_publisher = self.create_publisher(Float32, '/navigation/heading', 10)
        
# #         # Serial configuration - Multiple ports to try
# #         self.possible_serial_ports = [
# #             '/dev/ttyACM0',
# #             '/dev/ttyACM1',
# #             '/dev/ttyUSB0',
# #             '/dev/ttyUSB1'
# #         ]
# #         self.serial_port = None
# #         self.ser = None
        
# #         # NTRIP configuration
# #         self.ntrip_server_ip = "110.78.0.54"
# #         self.ntrip_server_port = 2116
# #         self.ntrip_username = "1118600009224"
# #         self.ntrip_password = "CK79"
# #         self.mount_point = "VRS_RTCM32"
        
# #         # Coordinate transformation - UTM Zone 47N for Thailand
# #         self.transformer = Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)
        
# #         # Position tracking for heading calculation
# #         self.position_history = deque(maxlen=10)
# #         self.current_heading = 0.0
# #         self.heading_initialized = False
# #         self.min_movement_for_heading = 0.3  # minimum movement in meters
        
# #         # Reference point for local coordinate system (first valid GNSS position)
# #         self.reference_point = None  # (lat, lon)
# #         self.reference_utm = None    # (x, y) in UTM
        
# #         # Initialize serial connection with auto-detection
# #         self.init_serial_connection()
        
# #         # Connect to NTRIP server only if serial connection successful
# #         if self.ser and self.ser.is_open:
# #             self.ntrip_socket = self.connect_to_ntrip()
            
# #             # Timer for reading GNSS data
# #             self.create_timer(0.1, self.read_gnss_data)
            
# #             self.get_logger().info('✅ Enhanced GNSS Publisher initialized with XY conversion and heading calculation')
# #         else:
# #             self.get_logger().error('❌ Failed to initialize GNSS Publisher - no serial connection')
    
# #     def find_available_serial_ports(self):
# #         """Find all available serial ports"""
# #         available_ports = []
        
# #         # Check our predefined ports
# #         for port in self.possible_serial_ports:
# #             if os.path.exists(port):
# #                 available_ports.append(port)
        
# #         # Also check for any ttyACM* or ttyUSB* devices
# #         acm_ports = glob.glob('/dev/ttyACM*')
# #         usb_ports = glob.glob('/dev/ttyUSB*')
        
# #         # Add any additional ports we found
# #         for port in acm_ports + usb_ports:
# #             if port not in available_ports:
# #                 available_ports.append(port)
        
# #         return sorted(available_ports)
    
# #     def test_serial_port(self, port, timeout=3):
# #         """Test if a serial port has GNSS data"""
# #         try:
# #             self.get_logger().info(f"🔍 Testing port: {port}")
# #             test_ser = serial.Serial(port, 9600, timeout=1)
            
# #             start_time = time.time()
# #             valid_data_found = False
            
# #             while time.time() - start_time < timeout:
# #                 try:
# #                     data = test_ser.readline().decode('ascii', errors='replace')
                    
# #                     # Check for GNSS NMEA sentences
# #                     if any(data.startswith(nmea) for nmea in ['$GPGGA', '$GNGLL', '$GPRMC', '$GNRMC', '$GNGGR']):
# #                         self.get_logger().info(f"📡 Found GNSS data on {port}: {data.strip()}")
# #                         valid_data_found = True
# #                         break
                        
# #                 except Exception as e:
# #                     continue
            
# #             test_ser.close()
# #             return valid_data_found
            
# #         except Exception as e:
# #             self.get_logger().debug(f"❌ Port {port} failed: {e}")
# #             return False
    
# #     def init_serial_connection(self):
# #         """Initialize serial connection with auto-detection"""
# #         self.get_logger().info("🔍 Searching for GNSS device...")
        
# #         # Find available ports
# #         available_ports = self.find_available_serial_ports()
        
# #         if not available_ports:
# #             self.get_logger().error("❌ No serial ports found!")
# #             return
        
# #         self.get_logger().info(f"📋 Available ports: {available_ports}")
        
# #         # Test each port for GNSS data
# #         for port in available_ports:
# #             if self.test_serial_port(port):
# #                 try:
# #                     self.ser = serial.Serial(port, 9600, timeout=1)
# #                     self.serial_port = port
# #                     self.get_logger().info(f"✅ Connected to GNSS device on {port}")
# #                     return
# #                 except Exception as e:
# #                     self.get_logger().error(f"❌ Failed to connect to {port}: {e}")
# #                     continue
        
# #         # If no port worked, try the first available one anyway
# #         if available_ports:
# #             try:
# #                 port = available_ports[0]
# #                 self.ser = serial.Serial(port, 9600, timeout=1)
# #                 self.serial_port = port
# #                 self.get_logger().warning(f"⚠️ No GNSS data detected, but connected to {port} anyway")
# #             except Exception as e:
# #                 self.get_logger().error(f"❌ Failed to connect to any port: {e}")
    
# #     def connect_to_ntrip(self):
# #         """Connect to NTRIP server for RTK corrections"""
# #         try:
# #             client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# #             client_socket.connect((self.ntrip_server_ip, self.ntrip_server_port))
            
# #             import base64
# #             auth_str = f"{self.ntrip_username}:{self.ntrip_password}"
# #             auth_bytes = auth_str.encode('ascii')
# #             auth_b64 = base64.b64encode(auth_bytes).decode('ascii')
            
# #             request = (
# #                 f"GET /{self.mount_point} HTTP/1.1\r\n"
# #                 f"User-Agent: NTRIP Client\r\n"
# #                 f"Authorization: Basic {auth_b64}\r\n"
# #                 f"\r\n"
# #             )
# #             client_socket.send(request.encode('ascii'))
# #             self.get_logger().info("🌐 Connected to NTRIP server for RTK corrections")
# #             return client_socket
# #         except Exception as e:
# #             self.get_logger().warning(f"⚠️ Could not connect to NTRIP server: {e}")
# #             return None
    
# #     def read_gnss_data(self):
# #         """Read and process GNSS data"""
# #         if not (hasattr(self, 'ser') and self.ser and self.ser.is_open):
# #             return
            
# #         try:
# #             data = self.ser.readline().decode('ascii', errors='replace')
            
# #             if data.startswith('$GPGGA') or data.startswith('$GNGLL') or data.startswith('$GPRMC') or data.startswith('$GNRMC'):
# #                 try:
# #                     msg = pynmea2.parse(data)
                    
# #                     latitude = None
# #                     longitude = None
                    
# #                     # Extract coordinates from different NMEA message types
# #                     if isinstance(msg, pynmea2.types.talker.GGA):
# #                         latitude = msg.latitude
# #                         longitude = msg.longitude
# #                     elif isinstance(msg, pynmea2.types.talker.GLL):
# #                         latitude = msg.latitude
# #                         longitude = msg.longitude
# #                     elif isinstance(msg, pynmea2.types.talker.RMC):
# #                         latitude = msg.latitude
# #                         longitude = msg.longitude
                    
# #                     # Process valid coordinates
# #                     if latitude is not None and longitude is not None:
# #                         self.process_gnss_position(latitude, longitude)
                    
# #                     # Handle RTK corrections
# #                     self.handle_ntrip_data()
                
# #                 except pynmea2.nmea.ChecksumError:
# #                     self.get_logger().warning("⚠️ Checksum error in NMEA data")
# #                 except Exception as e:
# #                     self.get_logger().debug(f"Parse error: {e}")
        
# #         except Exception as e:
# #             self.get_logger().error(f"❌ Error reading serial data: {e}")
# #             # Try to reconnect if serial connection fails
# #             self.reconnect_serial()
    
# #     def reconnect_serial(self):
# #         """Attempt to reconnect serial connection"""
# #         self.get_logger().warning("🔄 Attempting to reconnect serial connection...")
        
# #         if self.ser:
# #             try:
# #                 self.ser.close()
# #             except:
# #                 pass
        
# #         time.sleep(1)
# #         self.init_serial_connection()
    
# #     def process_gnss_position(self, latitude, longitude):
# #         """Process GNSS position and convert to XY coordinates"""
# #         current_time = time.time()
        
# #         # Validate coordinates
# #         if not (-90 <= latitude <= 90) or not (-180 <= longitude <= 180):
# #             self.get_logger().warning(f"⚠️ Invalid coordinates: lat={latitude}, lon={longitude}")
# #             return
        
# #         # Set reference point on first valid position
# #         if self.reference_point is None:
# #             self.reference_point = (latitude, longitude)
# #             self.reference_utm = self.transformer.transform(longitude, latitude)
# #             self.get_logger().info(f"🎯 Reference point set: lat={latitude:.7f}, lon={longitude:.7f}")
# #             self.get_logger().info(f"📍 Reference UTM: x={self.reference_utm[0]:.2f}, y={self.reference_utm[1]:.2f}")
        
# #         # Convert to UTM coordinates
# #         utm_x, utm_y = self.transformer.transform(longitude, latitude)
        
# #         # Calculate local XY relative to reference point
# #         local_x = utm_x - self.reference_utm[0]
# #         local_y = utm_y - self.reference_utm[1]
        
# #         # Create position record
# #         position_record = {
# #             'lat': latitude,
# #             'lon': longitude,
# #             'utm_x': utm_x,
# #             'utm_y': utm_y,
# #             'local_x': local_x,
# #             'local_y': local_y,
# #             'timestamp': current_time
# #         }
        
# #         # Add to position history
# #         self.position_history.append(position_record)
        
# #         # Calculate and update heading
# #         self.update_heading()
        
# #         # Publish all data
# #         self.publish_data(latitude, longitude, local_x, local_y)
        
# #         # Log position data (less frequent to reduce spam)
# #         if int(time.time()) % 2 == 0:  # Log every 2 seconds
# #             self.get_logger().info(
# #                 f"📡 GNSS [{self.serial_port}]: lat={latitude:.7f}, lon={longitude:.7f} | "
# #                 f"Local XY: x={local_x:.2f}, y={local_y:.2f} | "
# #                 f"Heading: {self.current_heading:.1f}°"
# #             )
    
# #     def update_heading(self):
# #         """Calculate heading from movement using XY coordinates"""
# #         if len(self.position_history) < 2:
# #             return
        
# #         # Use multiple points for better accuracy
# #         total_distance = 0.0
# #         weighted_headings = []
        
# #         # Calculate heading from recent movement
# #         for i in range(len(self.position_history) - 1, max(0, len(self.position_history) - 5), -1):
# #             if i == 0:
# #                 break
            
# #             current_pos = self.position_history[i]
# #             prev_pos = self.position_history[i-1]
            
# #             # Calculate movement in local XY
# #             dx = current_pos['local_x'] - prev_pos['local_x']
# #             dy = current_pos['local_y'] - prev_pos['local_y']
# #             distance = math.sqrt(dx*dx + dy*dy)
            
# #             if distance > self.min_movement_for_heading:
# #                 # Calculate heading in standard navigation format (0° = North, clockwise)
# #                 heading_rad = math.atan2(dx, dy)  # atan2(East, North)
# #                 heading_deg = math.degrees(heading_rad)
                
# #                 # Normalize to 0-360°
# #                 if heading_deg < 0:
# #                     heading_deg += 360
                
# #                 weighted_headings.append((heading_deg, distance))
# #                 total_distance += distance
        
# #         # Calculate weighted average heading if we have sufficient movement
# #         if total_distance > self.min_movement_for_heading and weighted_headings:
# #             # Use circular mean for angles
# #             total_x = 0.0
# #             total_y = 0.0
            
# #             for heading, weight in weighted_headings:
# #                 total_x += math.cos(math.radians(heading)) * weight
# #                 total_y += math.sin(math.radians(heading)) * weight
            
# #             if total_distance > 0:
# #                 avg_heading = math.degrees(math.atan2(total_y, total_x))
# #                 if avg_heading < 0:
# #                     avg_heading += 360
                
# #                 # Smooth heading transition for initialized heading
# #                 if self.heading_initialized:
# #                     heading_diff = avg_heading - self.current_heading
                    
# #                     # Handle angle wraparound
# #                     while heading_diff > 180:
# #                         heading_diff -= 360
# #                     while heading_diff < -180:
# #                         heading_diff += 360
                    
# #                     # Only update if change is reasonable (not a GPS jump)
# #                     if abs(heading_diff) < 90:  # Maximum 90° change per update
# #                         self.current_heading = avg_heading
# #                 else:
# #                     self.current_heading = avg_heading
# #                     self.heading_initialized = True
# #                     self.get_logger().info(f"🧭 Heading initialized: {self.current_heading:.1f}°")
    
# #     def publish_data(self, latitude, longitude, local_x, local_y):
# #         """Publish GNSS, XY position, and heading data"""
# #         # Publish GNSS data
# #         gnss_msg = NavSatFix()
# #         gnss_msg.latitude = latitude
# #         gnss_msg.longitude = longitude
# #         gnss_msg.status.status = 0  # STATUS_FIX
# #         gnss_msg.status.service = 1  # SERVICE_GPS
# #         self.gnss_publisher.publish(gnss_msg)
        
# #         # Publish XY position
# #         xy_msg = Point()
# #         xy_msg.x = local_x
# #         xy_msg.y = local_y
# #         xy_msg.z = 0.0
# #         self.xy_publisher.publish(xy_msg)
        
# #         # Publish heading
# #         if self.heading_initialized:
# #             heading_msg = Float32()
# #             heading_msg.data = float(self.current_heading)
# #             self.heading_publisher.publish(heading_msg)
    
# #     def handle_ntrip_data(self):
# #         """Handle RTK corrections from NTRIP server"""
# #         if self.ntrip_socket:
# #             try:
# #                 self.ntrip_socket.setblocking(0)
# #                 try:
# #                     rtk_data = self.ntrip_socket.recv(1024)
# #                     if rtk_data:
# #                         self.ser.write(rtk_data)
# #                 except socket.error:
# #                     pass  # No data available
# #             except Exception as e:
# #                 self.get_logger().debug(f"NTRIP error: {e}")
    
# #     def get_current_xy_position(self):
# #         """Get current XY position"""
# #         if len(self.position_history) > 0:
# #             latest = self.position_history[-1]
# #             return latest['local_x'], latest['local_y']
# #         return None, None
    
# #     def get_current_heading(self):
# #         """Get current heading in degrees"""
# #         return self.current_heading if self.heading_initialized else None
    
# #     def get_connected_port(self):
# #         """Get the currently connected serial port"""
# #         return self.serial_port
    
# #     def lat_lon_to_xy(self, lat, lon):
# #         """Convert any lat/lon to local XY coordinates"""
# #         if self.reference_point is None:
# #             return None, None
        
# #         # Convert to UTM
# #         utm_x, utm_y = self.transformer.transform(lon, lat)
        
# #         # Convert to local coordinates
# #         local_x = utm_x - self.reference_utm[0]
# #         local_y = utm_y - self.reference_utm[1]
        
# #         return local_x, local_y
    
# #     def xy_to_lat_lon(self, x, y):
# #         """Convert local XY coordinates back to lat/lon"""
# #         if self.reference_utm is None:
# #             return None, None
        
# #         # Convert to UTM
# #         utm_x = x + self.reference_utm[0]
# #         utm_y = y + self.reference_utm[1]
        
# #         # Convert back to lat/lon
# #         lon, lat = self.transformer.transform(utm_x, utm_y, direction='INVERSE')
        
# #         return lat, lon
    
# #     def calculate_bearing_xy(self, x1, y1, x2, y2):
# #         """Calculate bearing between two XY points"""
# #         dx = x2 - x1
# #         dy = y2 - y1
        
# #         # Calculate bearing (0° = North, clockwise)
# #         bearing_rad = math.atan2(dx, dy)
# #         bearing_deg = math.degrees(bearing_rad)
        
# #         # Normalize to 0-360°
# #         if bearing_deg < 0:
# #             bearing_deg += 360
        
# #         return bearing_deg
    
# #     def destroy_node(self):
# #         """Cleanup when node is destroyed"""
# #         self.get_logger().info("🔄 Shutting down GNSS Publisher...")
        
# #         if hasattr(self, 'ser') and self.ser and self.ser.is_open:
# #             self.ser.close()
# #             self.get_logger().info(f"📡 Closed serial connection to {self.serial_port}")
        
# #         if hasattr(self, 'ntrip_socket') and self.ntrip_socket:
# #             self.ntrip_socket.close()
# #             self.get_logger().info("🌐 Closed NTRIP connection")
        
# #         super().destroy_node()

# # def main(args=None):
# #     rclpy.init(args=args)
# #     gnss_publisher = EnhancedGNSSPublisher()
    
# #     try:
# #         rclpy.spin(gnss_publisher)
# #     except KeyboardInterrupt:
# #         gnss_publisher.get_logger().info("🛑 Keyboard interrupt received")
# #     finally:
# #         gnss_publisher.destroy_node()
# #         rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()


# -----------------------------------------heading yaw------------------------------------------------
#!/usr/bin/env python3
"""
อัปเดตโค้ด GNSS สำหรับ ZED-F9R IMU ที่ทำงานได้
"""
# #          # -------------------------------------------new controllor-------------------------------------------

# #!/usr/bin/env python3
# import serial
# import pynmea2
# import socket
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
# from std_msgs.msg import Float32, String, Float64MultiArray
# from geometry_msgs.msg import Point
# import math
# import pyproj
# from collections import deque
# import time
# import struct
# import base64
# import numpy as np

# # Enhanced IMU Heading Filter
# class EnhancedIMUHeadingFilter:
#     def __init__(self):
#         self.nav_att_history = deque(maxlen=12)
#         self.gps_history = deque(maxlen=8)
        
#         # Enhanced filter parameters
#         self.nav_att_outlier_threshold = 20.0  # degrees
#         self.gps_outlier_threshold = 18.0      # degrees
#         self.smoothing_alpha = 0.25
#         self.confidence_threshold = 0.8        # minimum confidence
        
#         # Statistics
#         self.stats = {
#             'nav_att_filtered': 0,
#             'gps_filtered': 0,
#             'nav_att_outliers': 0,
#             'gps_outliers': 0,
#             'confidence_boosted': 0,
#             'confidence_reduced': 0
#         }
    
#     def filter_nav_att_heading(self, new_heading, confidence_factor=1.0):
#         """Enhanced NAV-ATT heading filtering with confidence"""
#         if new_heading is None:
#             return None
        
#         # Check confidence threshold
#         if confidence_factor < self.confidence_threshold:
#             self.stats['confidence_reduced'] += 1
#             return self.get_last_stable_nav_att()
        
#         if confidence_factor > 1.0:
#             self.stats['confidence_boosted'] += 1
        
#         # Check for outliers
#         if self.is_nav_att_outlier(new_heading):
#             self.stats['nav_att_outliers'] += 1
#             return self.get_last_stable_nav_att()
        
#         # Add to history
#         self.nav_att_history.append((new_heading, confidence_factor))
        
#         # Apply enhanced smoothing
#         if len(self.nav_att_history) >= 3:
#             filtered = self.apply_nav_att_smoothing()
#             self.stats['nav_att_filtered'] += 1
#             return filtered
#         else:
#             return new_heading
    
#     def filter_gps_heading(self, new_gps_heading):
#         """Enhanced GPS heading filtering"""
#         if new_gps_heading is None:
#             return None
        
#         if self.is_gps_outlier(new_gps_heading):
#             self.stats['gps_outliers'] += 1
#             return self.get_last_stable_gps()
        
#         self.gps_history.append(new_gps_heading)
        
#         if len(self.gps_history) >= 2:
#             filtered = self.apply_gps_smoothing()
#             self.stats['gps_filtered'] += 1
#             return filtered
#         else:
#             return new_gps_heading
    
#     def is_nav_att_outlier(self, new_heading):
#         """Check if NAV-ATT heading is outlier"""
#         if len(self.nav_att_history) < 2:
#             return False
        
#         recent_headings = [h[0] for h in list(self.nav_att_history)[-3:]]
#         recent_avg = self.circular_mean(recent_headings)
#         diff = abs(self.angle_difference(new_heading, recent_avg))
        
#         return diff > self.nav_att_outlier_threshold
    
#     def is_gps_outlier(self, new_heading):
#         """Check if GPS heading is outlier"""
#         if len(self.gps_history) < 2:
#             return False
        
#         recent_avg = self.circular_mean(list(self.gps_history)[-2:])
#         diff = abs(self.angle_difference(new_heading, recent_avg))
        
#         return diff > self.gps_outlier_threshold
    
#     def apply_nav_att_smoothing(self):
#         """Enhanced NAV-ATT smoothing with confidence weighting"""
#         if len(self.nav_att_history) < 2:
#             return self.nav_att_history[-1][0]
        
#         # Get recent values with confidence
#         recent_data = list(self.nav_att_history)[-3:]
#         headings = [d[0] for d in recent_data]
#         confidences = [d[1] for d in recent_data]
        
#         # Weighted circular mean based on confidence
#         weights = np.array(confidences)
#         weights = weights / weights.sum()  # Normalize
        
#         return self.circular_weighted_mean(headings, weights)
    
#     def apply_gps_smoothing(self):
#         """GPS smoothing with moving average"""
#         if len(self.gps_history) < 2:
#             return self.gps_history[-1]
        
#         return self.circular_mean(list(self.gps_history)[-3:])
    
#     def get_last_stable_nav_att(self):
#         """Get last stable NAV-ATT heading"""
#         if len(self.nav_att_history) >= 2:
#             recent_headings = [h[0] for h in list(self.nav_att_history)[-2:]]
#             return self.circular_mean(recent_headings)
#         return None
    
#     def get_last_stable_gps(self):
#         """Get last stable GPS heading"""
#         if len(self.gps_history) >= 1:
#             return self.gps_history[-1]
#         return None
    
#     def angle_difference(self, angle1, angle2):
#         """Calculate shortest angular difference"""
#         diff = angle1 - angle2
#         while diff > 180:
#             diff -= 360
#         while diff < -180:
#             diff += 360
#         return diff
    
#     def circular_mean(self, angles):
#         """Calculate circular mean of angles"""
#         if not angles:
#             return 0
        
#         x = np.mean([math.cos(math.radians(a)) for a in angles])
#         y = np.mean([math.sin(math.radians(a)) for a in angles])
        
#         return math.degrees(math.atan2(y, x)) % 360
    
#     def circular_weighted_mean(self, angles, weights):
#         """Calculate weighted circular mean"""
#         if not angles or len(angles) != len(weights):
#             return 0
        
#         x = np.sum(weights * np.array([math.cos(math.radians(a)) for a in angles]))
#         y = np.sum(weights * np.array([math.sin(math.radians(a)) for a in angles]))
        
#         return math.degrees(math.atan2(y, x)) % 360
    
#     def get_stats(self):
#         """Get enhanced filter statistics"""
#         return self.stats.copy()

# class ZEDf9rGNSSNode(Node):
#     def __init__(self):
#         super().__init__('zed_f9r_gnss_node')
        
#         # ROS2 Publishers
#         self.gnss_publisher = self.create_publisher(NavSatFix, '/navigation/gnss', 10)
#         self.xy_publisher = self.create_publisher(Float64MultiArray, '/navigation/xy', 10)
#         self.raw_gps_publisher = self.create_publisher(Float64MultiArray, '/navigation/raw_gps', 10)
#         self.xy_debug_publisher = self.create_publisher(Point, '/navigation/xy_debug', 10)
#         # self.heading_publisher = self.create_publisher(Float32, '/navigation/heading', 10)
#         self.heading_debug_publisher = self.create_publisher(String, '/navigation/heading_debug', 10)
        
#         # Serial configuration
#         self.possible_serial_ports = [
#             '/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1'
#         ]
#         self.serial_port = None
#         self.ser = None
        
#         # NTRIP configuration
#         self.ntrip_server_ip = "110.78.0.54"
#         self.ntrip_server_port = 2116
#         self.ntrip_username = "1118600009224"
#         self.ntrip_password = "CK79"
#         self.mount_point = "VRS_RTCM32"
        
#         # Coordinate transformation
#         self.transformer = pyproj.Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)
        
#         # Position tracking
#         self.position_history = deque(maxlen=15)
#         self.min_movement_for_heading = 0.25
#         self.current_position = {'x': 0.0, 'y': 0.0}
        
#         # Reference point
#         self.reference_point = (13.650748, 100.492985)
#         self.reference_utm = None
        
#         # UTM coordinates
#         self.TARGET_UTM_X = 661452.0
#         self.TARGET_UTM_Y = 1509601.0
#         self.utm_offset_x = 0.0
#         self.utm_offset_y = 0.0
        
#         # Heading management
#         self.imu_heading = None
#         self.nav_att_heading = None
#         self.gps_heading = None
#         self.final_heading = None  # compass bearing (degrees)
#         self.heading_source = "NONE"
        
#         self.imu_heading_time = 0
#         self.nav_att_time = 0
#         self.gps_heading_time = 0
#         self.heading_timeout = 8.0
        
#         # ZED-F9R specific data
#         self.fusion_mode = "NONE"
#         self.imu_calibration_status = "UNKNOWN"
        
#         # Enhanced Heading filtering
#         self.heading_filter = EnhancedIMUHeadingFilter()
        
#         # NTRIP socket
#         self.ntrip_socket = None
        
#         # Performance stats
#         self.stats = {
#             'position_count': 0,
#             'nav_att_count': 0,
#             'gps_count': 0,
#             'filtered_count': 0
#         }
        
#         # Initialize coordinate system
#         self.init_track_coordinate_system()
        
#         # Initialize serial connection
#         self.init_serial_connection()
        
#         if self.ser and self.ser.is_open:
#             self.ntrip_socket = self.connect_to_ntrip()
#             self.configure_zed_f9r()
            
#             # Main processing timer
#             self.create_timer(0.1, self.main_loop)
            
#             self.get_logger().info('✅ ZED-F9R GNSS Node initialized')
#         else:
#             self.get_logger().error('❌ Failed to initialize GNSS')
    
#     def init_track_coordinate_system(self):
#         """Initialize coordinate system"""
#         if self.reference_utm is None:
#             self.reference_utm = self.transformer.transform(
#                 self.reference_point[1], self.reference_point[0]
#             )
        
#         self.utm_offset_x = self.TARGET_UTM_X - self.reference_utm[0]
#         self.utm_offset_y = self.TARGET_UTM_Y - self.reference_utm[1]
        
#         self.get_logger().info(f'🗺️ Coordinate system initialized')
    
#     def init_serial_connection(self):
#         """Initialize serial connection"""
#         for port in self.possible_serial_ports:
#             try:
#                 self.ser = serial.Serial(port, 38400, timeout=1)
#                 if self.ser.is_open:
#                     self.serial_port = port
#                     self.get_logger().info(f'✅ Connected to {port}')
#                     break
#             except Exception as e:
#                 self.get_logger().warn(f'⚠️ Failed to connect to {port}: {e}')
    
#     def connect_to_ntrip(self):
#         """Connect to NTRIP server"""
#         try:
#             ntrip_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#             ntrip_socket.connect((self.ntrip_server_ip, self.ntrip_server_port))
            
#             # NTRIP request
#             request = f"GET /{self.mount_point} HTTP/1.0\r\n"
#             request += f"User-Agent: NTRIP Client\r\n"
#             request += f"Authorization: Basic {self.encode_credentials()}\r\n"
#             request += "\r\n"
            
#             ntrip_socket.send(request.encode())
#             response = ntrip_socket.recv(1024).decode()
            
#             if "200 OK" in response:
#                 self.get_logger().info('✅ NTRIP connected successfully')
#                 return ntrip_socket
#             else:
#                 self.get_logger().error(f'❌ NTRIP connection failed: {response}')
#                 return None
                
#         except Exception as e:
#             self.get_logger().error(f'❌ NTRIP connection error: {e}')
#             return None
    
#     def encode_credentials(self):
#         """Encode NTRIP credentials"""
#         credentials = f"{self.ntrip_username}:{self.ntrip_password}"
#         return base64.b64encode(credentials.encode()).decode()
    
#     def configure_zed_f9r(self):
#         """Configure ZED-F9R for enhanced operation"""
#         if not self.ser or not self.ser.is_open:
#             return
        
#         # Configuration commands for ZED-F9R
#         config_commands = [
#             # Enable NAV-ATT messages
#             b'\xB5\x62\x06\x01\x08\x00\x01\x05\x00\x01\x00\x00\x00\x00\x16\x47',
#             # Enable ESF-STATUS messages  
#             b'\xB5\x62\x06\x01\x08\x00\x10\x10\x00\x01\x00\x00\x00\x00\x48\x3E',
#             # Enable high-rate positioning
#             b'\xB5\x62\x06\x08\x06\x00\x64\x00\x01\x00\x01\x00\x7A\x12'
#         ]
        
#         for cmd in config_commands:
#             try:
#                 self.ser.write(cmd)
#                 time.sleep(0.1)
#             except Exception as e:
#                 self.get_logger().warn(f'⚠️ Config command failed: {e}')
        
#         self.get_logger().info('🔧 ZED-F9R configuration completed')
    
#     def main_loop(self):
#         """Main processing loop"""
#         try:
#             # อ่านข้อมูลจาก GNSS
#             self.read_gnss_data()
            
#             # จัดการ NTRIP corrections
#             self.handle_ntrip_data()
            
#             # อัพเดท heading
#             self.update_enhanced_final_heading()
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Main loop error: {e}')
    
#     def handle_ntrip_data(self):
#         """Handle NTRIP corrections"""
#         if hasattr(self, 'ntrip_socket') and self.ntrip_socket:
#             try:
#                 self.ntrip_socket.setblocking(0)
#                 try:
#                     rtk_data = self.ntrip_socket.recv(1024)
#                     if rtk_data:
#                         self.ser.write(rtk_data)
#                 except socket.error:
#                     pass
#             except Exception as e:
#                 pass
    
#     def read_gnss_data(self):
#         """Read and process GNSS data"""
#         if not self.ser or not self.ser.is_open:
#             return
        
#         try:
#             if self.ser.in_waiting > 0:
#                 data = self.ser.read(self.ser.in_waiting)
#                 self.process_gnss_data(data)
#         except Exception as e:
#             self.get_logger().error(f'❌ GNSS data read error: {e}')
    
#     def process_gnss_data(self, data):
#         """Process incoming GNSS data"""
#         try:
#             # แยกข้อมูล NMEA และ UBX
#             data_str = data.decode('utf-8', errors='ignore')
#             lines = data_str.split('\n')
            
#             for line in lines:
#                 line = line.strip()
#                 if line.startswith('$'):
#                     # ประมวลผล NMEA
#                     self.process_nmea_sentence(line)
#                 elif b'\xb5\x62' in data:
#                     # ประมวลผล UBX
#                     self.process_ubx_data(data)
                        
#         except Exception as e:
#             self.get_logger().error(f'❌ GNSS data processing error: {e}')
    
#     def process_nmea_sentence(self, sentence):
#         """ประมวลผล NMEA sentence"""
#         try:
#             msg = pynmea2.parse(sentence)
            
#             if isinstance(msg, pynmea2.types.talker.GGA):
#                 # ประมวลผล GGA message สำหรับตำแหน่ง
#                 if msg.latitude and msg.longitude:
#                     lat = float(msg.latitude)
#                     lon = float(msg.longitude)
                    
#                     # แปลงเป็น UTM
#                     utm_x, utm_y = self.transformer.transform(lon, lat)
                    
#                     # อัพเดทตำแหน่งปัจจุบัน
#                     self.current_position['x'] = utm_x + self.utm_offset_x
#                     self.current_position['y'] = utm_y + self.utm_offset_y
                    
#                     # เก็บประวัติตำแหน่งสำหรับคำนวณ heading
#                     current_time = time.time()
#                     position_record = {
#                         'lat': lat, 'lon': lon,
#                         'utm_x': utm_x, 'utm_y': utm_y,
#                         'local_x': self.current_position['x'],
#                         'local_y': self.current_position['y'],
#                         'timestamp': current_time
#                     }
#                     self.position_history.append(position_record)
                    
#                     # คำนวณ GPS heading
#                     self.update_gps_heading_improved()
                    
#                     # Publish ข้อมูลตำแหน่ง
#                     xy_msg = Float64MultiArray()
#                     xy_msg.data = [self.current_position['x'], self.current_position['y']]
#                     self.xy_publisher.publish(xy_msg)
                    
#                     # Publish Raw GPS
#                     raw_gps_msg = Float64MultiArray()
#                     raw_gps_msg.data = [lat, lon]
#                     self.raw_gps_publisher.publish(raw_gps_msg)
                    
#                     # Publish XY Debug
#                     xy_debug_msg = Point()
#                     xy_debug_msg.x = self.current_position['x']
#                     xy_debug_msg.y = self.current_position['y']
#                     xy_debug_msg.z = 0.0
#                     self.xy_debug_publisher.publish(xy_debug_msg)
                    
#                     self.stats['position_count'] += 1
                    
#         except Exception as e:
#             pass  # ไม่แสดง error สำหรับ NMEA ที่ parse ไม่ได้
    
#     def process_ubx_data(self, data):
#         """Process UBX data for NAV-ATT"""
#         try:
#             # หา UBX NAV-ATT messages (Class 0x01, ID 0x05)
#             ubx_start = b'\xb5\x62'
#             if ubx_start in data:
#                 start_idx = data.find(ubx_start)
#                 if start_idx != -1 and len(data) > start_idx + 6:
#                     msg_class = data[start_idx + 2]
#                     msg_id = data[start_idx + 3]
                    
#                     # NAV-ATT message
#                     if msg_class == 0x01 and msg_id == 0x05:
#                         self.parse_nav_att_manual(data[start_idx:])
                        
#         except Exception as e:
#             self.get_logger().debug(f'UBX processing error: {e}')
    
#     def parse_nav_att_manual(self, ubx_data):
#         """Parse NAV-ATT message manually"""
#         try:
#             if len(ubx_data) >= 32:  # NAV-ATT minimum length
#                 # Extract heading from NAV-ATT (offset 16, 4 bytes, little endian)
#                 heading_raw = struct.unpack('<i', ubx_data[16:20])[0]
#                 heading_deg = heading_raw * 1e-5  # Convert from 1e-5 degrees
                
#                 # Convert to compass bearing (0-360°)
#                 if heading_deg < 0:
#                     heading_deg += 360
                
#                 # Apply filtering
#                 filtered_heading = self.heading_filter.filter_nav_att_heading(
#                     heading_deg, confidence_factor=1.0
#                 )
                
#                 if filtered_heading is not None:
#                     self.nav_att_heading = filtered_heading
#                     self.nav_att_time = time.time()
#                     self.stats['nav_att_count'] += 1
#                     self.imu_calibration_status = "CALIBRATED"
#                     self.fusion_mode = "FUSION"
                    
#         except Exception as e:
#             self.get_logger().debug(f'NAV-ATT parse error: {e}')
    
#     def update_gps_heading_improved(self):
#         """Enhanced GPS heading calculation using UTM coordinates"""
#         if len(self.position_history) < 3:
#             return
        
#         total_distance = 0.0
#         weighted_x = weighted_y = 0.0
        
#         recent_positions = list(self.position_history)[-5:]
        
#         for i in range(len(recent_positions) - 1):
#             curr = recent_positions[i+1]
#             prev = recent_positions[i]
            
#             # ใช้ UTM coordinates สำหรับการคำนวณ heading
#             dx = curr['utm_x'] - prev['utm_x']  # East
#             dy = curr['utm_y'] - prev['utm_y']  # North
#             distance = math.sqrt(dx*dx + dy*dy)
            
#             if distance > self.min_movement_for_heading:
#                 # คำนวณ compass bearing
#                 heading_rad = math.atan2(dx, dy)  # arctan2(East, North)
                
#                 time_weight = (i + 1) / len(recent_positions)
#                 weight = distance * time_weight
                
#                 weighted_x += math.cos(heading_rad) * weight
#                 weighted_y += math.sin(heading_rad) * weight
#                 total_distance += weight
        
#         if total_distance > self.min_movement_for_heading:
#             avg_heading_rad = math.atan2(weighted_y, weighted_x)
#             avg_heading_deg = math.degrees(avg_heading_rad)
#             if avg_heading_deg < 0:
#                 avg_heading_deg += 360
            
#             filtered_gps_heading = self.heading_filter.filter_gps_heading(avg_heading_deg)
            
#             if filtered_gps_heading is not None:
#                 self.gps_heading = filtered_gps_heading
#                 self.gps_heading_time = time.time()
#                 self.stats['gps_count'] += 1
    
#     def update_enhanced_final_heading(self):
#         """Enhanced final heading selection with NAV-ATT priority"""
#         current_time = time.time()
        
#         # Check data validity
#         nav_att_valid = (self.nav_att_heading is not None and 
#                         (current_time - self.nav_att_time) < self.heading_timeout)
        
#         gps_valid = (self.gps_heading is not None and 
#                     (current_time - self.gps_heading_time) < self.heading_timeout)
        
#         # Enhanced priority: NAV-ATT > GPS > Hold
#         if nav_att_valid and self.imu_calibration_status in ["FULLY_CALIBRATED", "CALIBRATED"]:
#             self.final_heading = self.nav_att_heading
#             self.heading_source = f"NAV_ATT_{self.imu_calibration_status}"
            
#         elif nav_att_valid and self.imu_calibration_status == "CALIBRATING":
#             if gps_valid:
#                 self.final_heading = self.blend_headings(self.nav_att_heading, self.gps_heading, 0.8)
#                 self.heading_source = "NAV_ATT_GPS_BLEND"
#             else:
#                 self.final_heading = self.nav_att_heading
#                 self.heading_source = "NAV_ATT_CALIBRATING"
                
#         elif gps_valid:
#             self.final_heading = self.gps_heading
#             self.heading_source = "GPS"
            
#         else:
#             # ถ้าไม่มีข้อมูลใหม่ ให้เก็บค่าเดิมไว้
#             if self.final_heading is not None:
#                 self.heading_source = "HOLD_LAST"
#             else:
#                 self.final_heading = None
#                 self.heading_source = "NONE"
        
#         # Publish heading
#         if self.final_heading is not None:
#             heading_msg = Float32()
#             heading_msg.data = float(self.final_heading)
#             self.heading_publisher.publish(heading_msg)
#             self.stats['filtered_count'] += 1
            
#             # Publish heading debug info
#             debug_msg = String()
#             nav_att_str = f"{self.nav_att_heading:.1f}" if self.nav_att_heading is not None else "None"
#             gps_str = f"{self.gps_heading:.1f}" if self.gps_heading is not None else "None"
            
#             debug_msg.data = (
#                 f"Heading: {self.final_heading:.1f}° | "
#                 f"Source: {self.heading_source} | "
#                 f"NAV-ATT: {nav_att_str}° | "
#                 f"GPS: {gps_str}° | "
#                 f"Fusion: {self.fusion_mode} | "
#                 f"IMU Cal: {self.imu_calibration_status}"
#             )
#             self.heading_debug_publisher.publish(debug_msg)
    
#     def blend_headings(self, heading1, heading2, weight1):
#         """Blend two headings considering circular nature"""
#         if heading1 is None or heading2 is None:
#             return heading1 or heading2
        
#         # Convert to radians for circular math
#         h1_rad = math.radians(heading1)
#         h2_rad = math.radians(heading2)
        
#         x1, y1 = math.cos(h1_rad), math.sin(h1_rad)
#         x2, y2 = math.cos(h2_rad), math.sin(h2_rad)
        
#         weight2 = 1.0 - weight1
#         x_blend = weight1 * x1 + weight2 * x2
#         y_blend = weight1 * y1 + weight2 * y2
        
#         blended_heading_rad = math.atan2(y_blend, x_blend)
#         blended_heading_deg = math.degrees(blended_heading_rad)
        
#         if blended_heading_deg < 0:
#             blended_heading_deg += 360
            
#         return blended_heading_deg

# def main(args=None):
#     rclpy.init(args=args)
    
#     try:
#         node = ZEDf9rGNSSNode()
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         print('\n🛑 GNSS Node stopped by user')
#     except Exception as e:
#         print(f'❌ Error: {e}')
#     finally:
#         if 'node' in locals():
#             node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Quaternion
# import serial
# import pynmea2
# import math
# import time
# import socket
# import base64
# import threading
# from pyproj import Transformer

# # -------- NTRIP RTK ----------
# NTRIP_SERVER_IP = "110.78.0.54"
# NTRIP_SERVER_PORT = 2116
# NTRIP_USERNAME = "1118600009224"
# NTRIP_PASSWORD = "CK79"
# NTRIP_MOUNTPOINT = "VRS_RTCM32"
# # -----------------------------


# class GnssPosePublisher(Node):

#     def __init__(self):
#         super().__init__('gnss_pose_publisher')

#         # ===== PARAMETERS =====
#         self.GNSS_PORT = '/dev/ttyACM0'
#         self.GNSS_BAUD = 9600

#         self.transformer = Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)

#         # state
#         self.prev_psi = None
#         self.prev_time = 0.0

#         # ===== ROS =====
#         self.pose_pub = self.create_publisher(Odometry, '/odom', 10)

#         try:
#             self.ser = serial.Serial(self.GNSS_PORT, self.GNSS_BAUD, timeout=1)
#             self.get_logger().info(f"✅ Opened {self.GNSS_PORT}")
#         except Exception as e:
#             self.get_logger().error(f"❌ Cannot open serial: {e}")
#             raise

#         # ===== START NTRIP =====
#         threading.Thread(target=self.start_ntrip_client, daemon=True).start()

#         self.create_timer(0.1, self.read_gps)
#         self.get_logger().info("📡 GNSS Pose Publisher READY")

#     # ================= NTRIP CLIENT =================
#     def start_ntrip_client(self):

#         while True:
#             try:

#                 self.get_logger().info("🌐 Connecting NTRIP...")

#                 sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#                 sock.connect((NTRIP_SERVER_IP, NTRIP_SERVER_PORT))

#                 auth = base64.b64encode(
#                     f"{NTRIP_USERNAME}:{NTRIP_PASSWORD}".encode()
#                 ).decode()

#                 request = (
#                     f"GET /{NTRIP_MOUNTPOINT} HTTP/1.0\r\n"
#                     "User-Agent: NTRIP ROS2Client\r\n"
#                     f"Authorization: Basic {auth}\r\n\r\n"
#                 )

#                 sock.send(request.encode())

#                 response = sock.recv(1024)

#                 if b"200 OK" not in response:
#                     self.get_logger().error("❌ NTRIP mountpoint error")
#                     sock.close()
#                     time.sleep(5)
#                     continue

#                 self.get_logger().info("✅ RTK Connected")

#                 while True:

#                     data = sock.recv(4096)

#                     if not data:
#                         break

#                     self.ser.write(data)

#             except Exception as e:

#                 self.get_logger().error(f"NTRIP Error: {e}")
#                 time.sleep(5)

#     # ================= GPS READ =================
#     def read_gps(self):

#         try:
#             line = self.ser.readline().decode('ascii', errors='replace').strip()
#         except Exception:
#             return

#         if not line.startswith(('$GPRMC', '$GNRMC')):
#             return

#         try:

#             msg = pynmea2.parse(line)

#             if msg.status != "A":
#                 return

#             now = time.time()

#             if now - self.prev_time < 0.1:
#                 return

#             self.prev_time = now

#             lat = self.nmea_to_decimal(msg.lat, msg.lat_dir)
#             lon = self.nmea_to_decimal(msg.lon, msg.lon_dir)

#             x, y = self.transformer.transform(lon, lat)

#             if msg.true_course is None or msg.true_course == '':
#                 psi = self.prev_psi if self.prev_psi is not None else 0.0
#             else:
#                 psi = self.normalize_angle(math.radians(float(msg.true_course)))
#                 self.prev_psi = psi

#             speed = float(msg.spd_over_grnd or 0) * 0.514444

#             odom = Odometry()

#             odom.header.stamp = self.get_clock().now().to_msg()
#             odom.header.frame_id = 'odom'

#             odom.pose.pose.position.x = x
#             odom.pose.pose.position.y = y
#             odom.pose.pose.position.z = 0.0

#             odom.pose.pose.orientation = self.yaw_to_quat(psi)

#             odom.twist.twist.linear.x = speed

#             self.pose_pub.publish(odom)

#             self.get_logger().info(
#                 f"📍 x={x:.2f} y={y:.2f} yaw={math.degrees(psi):.1f}° speed={speed:.2f}"
#             )

#         except Exception as e:
#             self.get_logger().warn(f"Parse error: {e}")

#     # ================= UTILS =================
#     def nmea_to_decimal(self, degree_str, direction):

#         if not degree_str or degree_str == '0':
#             return 0.0

#         raw = float(degree_str)

#         degrees = int(raw / 100)
#         minutes = raw - degrees * 100

#         decimal = degrees + minutes / 60

#         if direction in ['S', 'W']:
#             decimal = -decimal

#         return decimal

#     def yaw_to_quat(self, yaw):

#         q = Quaternion()

#         q.w = math.cos(yaw / 2)
#         q.x = 0.0
#         q.y = 0.0
#         q.z = math.sin(yaw / 2)

#         return q

#     def normalize_angle(self, a):

#         return (a + math.pi) % (2 * math.pi) - math.pi

#     def destroy_node(self):

#         self.ser.close()
#         super().destroy_node()


# def main():

#     rclpy.init()

#     node = GnssPosePublisher()

#     try:
#         rclpy.spin(node)

#     except KeyboardInterrupt:
#         pass

#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()


# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Quaternion
# import serial
# import pynmea2
# import math
# import time
# import socket
# import base64
# import threading
# from pyproj import Transformer

# # -------- NTRIP RTK ----------
# NTRIP_SERVER_IP   = "110.78.0.54"
# NTRIP_SERVER_PORT = 2116
# NTRIP_USERNAME    = "1118600009224"
# NTRIP_PASSWORD    = "CK79"
# NTRIP_MOUNTPOINT  = "VRS_RTCM32"
# # -----------------------------

# GGA_SEND_INTERVAL = 5.0   # ส่ง GGA กลับ caster ทุก 5 วิ (VRS requirement)


# class GnssPosePublisher(Node):

#     def __init__(self):
#         super().__init__('gnss_pose_publisher')

#         self.GNSS_PORT = '/dev/ttyACM0'
#         self.GNSS_BAUD = 9600

#         self.transformer = Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)

#         self.prev_psi  = None
#         self.prev_time = 0.0

#         # เก็บ GGA ล่าสุดไว้ส่งให้ NTRIP caster (VRS)
#         self._last_gga: str | None = None
#         self._ntrip_sock: socket.socket | None = None
#         self._gga_lock = threading.Lock()

#         self.pose_pub = self.create_publisher(Odometry, '/odom', 10)

#         try:
#             self.ser = serial.Serial(self.GNSS_PORT, self.GNSS_BAUD, timeout=1)
#             self.get_logger().info(f"✅ Opened {self.GNSS_PORT}")
#         except Exception as e:
#             self.get_logger().error(f"❌ Cannot open serial: {e}")
#             raise

#         threading.Thread(target=self._ntrip_thread, daemon=True).start()

#         self.create_timer(0.1, self.read_gps)
#         self.get_logger().info("📡 GNSS Pose Publisher READY")

#     # ══════════════════════════════════════════════════════════════════════════
#     #  NTRIP thread — รับ RTCM + ส่ง GGA กลับ (VRS)
#     # ══════════════════════════════════════════════════════════════════════════
#     def _ntrip_thread(self):
#         while True:
#             try:
#                 self.get_logger().info("🌐 Connecting NTRIP...")

#                 sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#                 sock.settimeout(10)
#                 sock.connect((NTRIP_SERVER_IP, NTRIP_SERVER_PORT))

#                 auth = base64.b64encode(
#                     f"{NTRIP_USERNAME}:{NTRIP_PASSWORD}".encode()
#                 ).decode()

#                 # ── ส่ง HTTP request พร้อม GGA ถ้ามี ──────────────────────────
#                 with self._gga_lock:
#                     gga_line = self._last_gga

#                 request = (
#                     f"GET /{NTRIP_MOUNTPOINT} HTTP/1.0\r\n"
#                     f"User-Agent: NTRIP ROS2Client/1.0\r\n"
#                     f"Authorization: Basic {auth}\r\n"
#                     f"Ntrip-Version: Ntrip/1.0\r\n"
#                 )
#                 if gga_line:
#                     request += f"Ntrip-GGA: {gga_line}\r\n"
#                 request += "\r\n"

#                 sock.sendall(request.encode())

#                 response = b""
#                 while b"\r\n\r\n" not in response:
#                     chunk = sock.recv(256)
#                     if not chunk:
#                         break
#                     response += chunk

#                 if b"200 OK" not in response and b"ICY 200 OK" not in response:
#                     self.get_logger().error(
#                         f"❌ NTRIP response: {response[:200]}"
#                     )
#                     sock.close()
#                     time.sleep(5)
#                     continue

#                 self.get_logger().info("✅ RTK Connected — waiting for RTCM...")
#                 sock.settimeout(5)

#                 with self._gga_lock:
#                     self._ntrip_sock = sock

#                 last_gga_sent = time.time()

#                 # ── main receive loop ──────────────────────────────────────────
#                 while True:
#                     # ส่ง GGA กลับให้ caster ทุก GGA_SEND_INTERVAL วิ (VRS)
#                     now = time.time()
#                     if now - last_gga_sent >= GGA_SEND_INTERVAL:
#                         with self._gga_lock:
#                             gga = self._last_gga
#                         if gga:
#                             try:
#                                 msg_out = gga if gga.endswith('\r\n') else gga + '\r\n'
#                                 sock.sendall(msg_out.encode())
#                                 self.get_logger().info(
#                                     f"📤 Sent GGA to caster: {gga.strip()}"
#                                 )
#                             except Exception as e:
#                                 self.get_logger().warn(f"GGA send error: {e}")
#                         last_gga_sent = now

#                     try:
#                         data = sock.recv(4096)
#                     except socket.timeout:
#                         continue

#                     if not data:
#                         break

#                     self.ser.write(data)

#             except Exception as e:
#                 self.get_logger().error(f"NTRIP Error: {e}")
#             finally:
#                 with self._gga_lock:
#                     self._ntrip_sock = None
#                 try:
#                     sock.close()
#                 except Exception:
#                     pass
#                 time.sleep(5)

#     # ══════════════════════════════════════════════════════════════════════════
#     #  GPS READ
#     # ══════════════════════════════════════════════════════════════════════════
#     def read_gps(self):
#         try:
#             line = self.ser.readline().decode('ascii', errors='replace').strip()
#         except Exception:
#             return

#         # ── เก็บ GGA ดิบไว้ส่ง VRS ──────────────────────────────────────────
#         if line.startswith(('$GPGGA', '$GNGGA')):
#             with self._gga_lock:
#                 self._last_gga = line
#             # แสดง fix quality ใน log
#             try:
#                 parts = line.split(',')
#                 fix_q = int(parts[6]) if len(parts) > 6 and parts[6] else 0
#                 labels = {0:"No fix", 1:"GPS only", 2:"DGPS",
#                           4:"RTK Fixed ✅", 5:"RTK Float 🔶"}
#                 label = labels.get(fix_q, f"Quality={fix_q}")
#                 self.get_logger().info(
#                     f"🛰️  Fix: {label}  (raw quality={fix_q})",
#                     throttle_duration_sec=2.0
#                 )
#             except Exception:
#                 pass
#             return  # GGA ไม่มี position ที่ต้องการ publish

#         if not line.startswith(('$GPRMC', '$GNRMC')):
#             return

#         try:
#             msg = pynmea2.parse(line)

#             if msg.status != "A":
#                 return

#             now = time.time()
#             if now - self.prev_time < 0.1:
#                 return
#             self.prev_time = now

#             lat = self.nmea_to_decimal(msg.lat, msg.lat_dir)
#             lon = self.nmea_to_decimal(msg.lon, msg.lon_dir)

#             x, y = self.transformer.transform(lon, lat)

#             if msg.true_course is None or msg.true_course == '':
#                 psi = self.prev_psi if self.prev_psi is not None else 0.0
#             else:
#                 psi = self.normalize_angle(math.radians(float(msg.true_course)))
#                 self.prev_psi = psi

#             speed = float(msg.spd_over_grnd or 0) * 0.514444

#             odom = Odometry()
#             odom.header.stamp    = self.get_clock().now().to_msg()
#             odom.header.frame_id = 'odom'
#             odom.pose.pose.position.x = x
#             odom.pose.pose.position.y = y
#             odom.pose.pose.position.z = 0.0
#             odom.pose.pose.orientation = self.yaw_to_quat(psi)
#             odom.twist.twist.linear.x  = speed
#             self.pose_pub.publish(odom)

#             self.get_logger().info(
#                 f"📍 x={x:.2f} y={y:.2f} "
#                 f"yaw={math.degrees(psi):.1f}° speed={speed:.2f}"
#             )

#         except Exception as e:
#             self.get_logger().warn(f"Parse error: {e}")

#     # ══════════════════════════════════════════════════════════════════════════
#     def nmea_to_decimal(self, degree_str, direction):
#         if not degree_str or degree_str == '0':
#             return 0.0
#         raw     = float(degree_str)
#         degrees = int(raw / 100)
#         minutes = raw - degrees * 100
#         decimal = degrees + minutes / 60
#         if direction in ['S', 'W']:
#             decimal = -decimal
#         return decimal

#     def yaw_to_quat(self, yaw):
#         q   = Quaternion()
#         q.w = math.cos(yaw / 2)
#         q.x = 0.0
#         q.y = 0.0
#         q.z = math.sin(yaw / 2)
#         return q

#     def normalize_angle(self, a):
#         return (a + math.pi) % (2 * math.pi) - math.pi

#     def destroy_node(self):
#         self.ser.close()
#         super().destroy_node()


# def main():
#     rclpy.init()
#     node = GnssPosePublisher()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()


# #!/usr/bin/env python3
# """
# gnss_pose_publisher.py  — Fixed Version
# ========================================
# การแก้ไขหลัก:
#   1. แปลง GPS bearing → ENU/ROS yaw อย่างถูกต้อง
#      yaw = pi/2 - bearing_rad  (0°=East, CCW positive)
#   2. ใช้ GGA เป็นแหล่ง position หลัก (มี fix quality)
#   3. บังคับ fix_quality >= 4 ก่อน publish
#   4. Heading จาก GGA consecutive positions (เสถียรกว่า true_course)
#   5. ENU frame ที่ถูกต้อง: x=East, y=North
# """
# #!/usr/bin/env python3
# """
# gnss_pose_publisher.py — UTM Absolute Version
# ===============================================
# การเปลี่ยนแปลงจากเวอร์ชันก่อน:
#   - ลบ origin normalization ออกทั้งหมด
#   - publish position เป็น UTM absolute ตรงๆ (เช่น x=700123.45, y=1500456.78)
#   - /odom และ waypoints.csv ใช้ coordinate เดียวกัน → ไม่ต้อง sync origin
#   - รถเริ่มจากจุดไหนก็ได้ ระบบหา nearest waypoint เองอัตโนมัติ
# """

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Quaternion
# import serial
# import pynmea2
# import math
# import time
# import socket
# import base64
# import threading
# from pyproj import Transformer

# # -------- NTRIP RTK ----------
# NTRIP_SERVER_IP   = "110.78.0.54"
# NTRIP_SERVER_PORT = 2116
# NTRIP_USERNAME    = "1118600009224"
# NTRIP_PASSWORD    = "CK79"
# NTRIP_MOUNTPOINT  = "VRS_RTCM32"
# # -----------------------------

# GGA_SEND_INTERVAL     = 0.2   # ส่ง GGA กลับ caster ทุก 5 วิ (VRS)
# MIN_FIX_QUALITY       = 4     # RTK Fixed เท่านั้น
# MIN_SPEED_FOR_HEADING = 0.3   # m/s — ต่ำกว่านี้ไม่อัปเดต heading จาก true_course


# class GnssPosePublisher(Node):

#     def __init__(self):
#         super().__init__('gnss_pose_publisher')

#         self.GNSS_PORT = '/dev/ttyACM0'
#         self.GNSS_BAUD = 9600

#         # UTM zone 47N (ประเทศไทย) — lon→x(East), lat→y(North)
#         self.transformer = Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)

#         self.prev_yaw    = None
#         self.prev_time   = 0.0
#         self.fix_quality = 0

#         # ✅ ไม่มี origin แล้ว — เก็บ prev UTM ไว้คำนวณ heading fallback เท่านั้น
#         self.prev_x_utm: float | None = None
#         self.prev_y_utm: float | None = None

#         # lat/lon ล่าสุดจาก GGA (แม่นกว่า RMC เมื่อใช้ RTK)
#         self._last_lat: float | None = None
#         self._last_lon: float | None = None

#         self._last_gga: str | None = None
#         self._ntrip_sock: socket.socket | None = None
#         self._gga_lock = threading.Lock()

#         self.pose_pub = self.create_publisher(Odometry, '/odom', 10)

#         try:
#             self.ser = serial.Serial(self.GNSS_PORT, self.GNSS_BAUD, timeout=1)
#             self.get_logger().info(f"✅ Opened {self.GNSS_PORT}")
#         except Exception as e:
#             self.get_logger().error(f"❌ Cannot open serial: {e}")
#             raise

#         threading.Thread(target=self._ntrip_thread, daemon=True).start()
#         self.create_timer(0.1, self.read_gps)
#         self.get_logger().info("📡 GNSS Pose Publisher READY — UTM Absolute Mode")

#     # ══════════════════════════════════════════════════════════════════════════
#     #  NTRIP thread — รับ RTCM + ส่ง GGA กลับ (VRS)
#     # ══════════════════════════════════════════════════════════════════════════
#     def _ntrip_thread(self):
#         while True:
#             try:
#                 self.get_logger().info("🌐 Connecting NTRIP...")
#                 sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#                 sock.settimeout(10)
#                 sock.connect((NTRIP_SERVER_IP, NTRIP_SERVER_PORT))

#                 auth = base64.b64encode(
#                     f"{NTRIP_USERNAME}:{NTRIP_PASSWORD}".encode()
#                 ).decode()

#                 with self._gga_lock:
#                     gga_line = self._last_gga

#                 request = (
#                     f"GET /{NTRIP_MOUNTPOINT} HTTP/1.0\r\n"
#                     f"User-Agent: NTRIP ROS2Client/1.0\r\n"
#                     f"Authorization: Basic {auth}\r\n"
#                     f"Ntrip-Version: Ntrip/1.0\r\n"
#                 )
#                 if gga_line:
#                     request += f"Ntrip-GGA: {gga_line}\r\n"
#                 request += "\r\n"

#                 sock.sendall(request.encode())

#                 response = b""
#                 while b"\r\n\r\n" not in response:
#                     chunk = sock.recv(256)
#                     if not chunk:
#                         break
#                     response += chunk

#                 if b"200 OK" not in response and b"ICY 200 OK" not in response:
#                     self.get_logger().error(f"❌ NTRIP response: {response[:200]}")
#                     sock.close()
#                     time.sleep(5)
#                     continue

#                 self.get_logger().info("✅ RTK Connected")
#                 sock.settimeout(5)

#                 with self._gga_lock:
#                     self._ntrip_sock = sock

#                 last_gga_sent = time.time()

#                 while True:
#                     now = time.time()
#                     if now - last_gga_sent >= GGA_SEND_INTERVAL:
#                         with self._gga_lock:
#                             gga = self._last_gga
#                         if gga:
#                             try:
#                                 msg_out = gga if gga.endswith('\r\n') else gga + '\r\n'
#                                 sock.sendall(msg_out.encode())
#                                 self.get_logger().info(f"📤 Sent GGA: {gga.strip()}")
#                             except Exception as e:
#                                 self.get_logger().warn(f"GGA send error: {e}")
#                         last_gga_sent = now

#                     try:
#                         data = sock.recv(4096)
#                     except socket.timeout:
#                         continue
#                     if not data:
#                         break
#                     self.ser.write(data)

#             except Exception as e:
#                 self.get_logger().error(f"NTRIP Error: {e}")
#             finally:
#                 with self._gga_lock:
#                     self._ntrip_sock = None
#                 try:
#                     sock.close()
#                 except Exception:
#                     pass
#                 time.sleep(5)

#     # ══════════════════════════════════════════════════════════════════════════
#     #  GPS READ
#     # ══════════════════════════════════════════════════════════════════════════
#     def read_gps(self):
#         try:
#             line = self.ser.readline().decode('ascii', errors='replace').strip()
#         except Exception:
#             return

#         # ── GGA: อัปเดต fix quality + เก็บ lat/lon ────────────────────────
#         if line.startswith(('$GPGGA', '$GNGGA')):
#             with self._gga_lock:
#                 self._last_gga = line
#             try:
#                 gga = pynmea2.parse(line)
#                 self.fix_quality = int(gga.gps_qual)

#                 labels = {0: "No fix ❌", 1: "GPS only ⚪", 2: "DGPS 🔵",
#                           4: "RTK Fixed ✅", 5: "RTK Float 🔶"}
#                 self.get_logger().info(
#                     f"🛰️  Fix: {labels.get(self.fix_quality, str(self.fix_quality))}",
#                     throttle_duration_sec=2.0
#                 )

#                 if self.fix_quality < MIN_FIX_QUALITY:
#                     return

#                 if gga.lat and gga.lon:
#                     # ✅ บันทึก lat/lon จาก GGA ไว้ใช้ใน RMC callback
#                     self._last_lat = self.nmea_to_decimal(gga.lat, gga.lat_dir)
#                     self._last_lon = self.nmea_to_decimal(gga.lon, gga.lon_dir)

#             except Exception as e:
#                 self.get_logger().warn(f"GGA parse error: {e}")
#             return

#         # ── RMC: speed + true_course ─────────────────────────────────────────
#         if not line.startswith(('$GPRMC', '$GNRMC')):
#             return

#         if self.fix_quality < MIN_FIX_QUALITY:
#             return

#         try:
#             msg = pynmea2.parse(line)
#             if msg.status != "A":
#                 return

#             now = time.time()
#             if now - self.prev_time < 0.1:
#                 return
#             self.prev_time = now

#             # ✅ ใช้ lat/lon จาก GGA (แม่นกว่า) ถ้ามี
#             if self._last_lat is not None:
#                 lat = self._last_lat
#                 lon = self._last_lon
#             else:
#                 lat = self.nmea_to_decimal(msg.lat, msg.lat_dir)
#                 lon = self.nmea_to_decimal(msg.lon, msg.lon_dir)

#             # ✅ แปลงเป็น UTM absolute — ไม่ลบ origin
#             x_utm, y_utm = self.transformer.transform(lon, lat)

#             speed = float(msg.spd_over_grnd or 0) * 0.514444  # knots → m/s

#             # ── คำนวณ yaw ──────────────────────────────────────────────────
#             # ✅ แปลง GPS bearing → ENU yaw
#             #    GPS bearing: 0° = North, หมุน CW
#             #    ENU yaw    : 0° = East,  หมุน CCW
#             #    สูตร: yaw = π/2 − bearing_rad
#             yaw = self.prev_yaw if self.prev_yaw is not None else 0.0

#             if msg.true_course and msg.true_course != '':
#                 bearing_deg = float(msg.true_course)
#                 if speed >= MIN_SPEED_FOR_HEADING:
#                     yaw = self.normalize_angle(
#                         math.pi / 2.0 - math.radians(bearing_deg)
#                     )
#                     self.prev_yaw = yaw

#             # fallback: คำนวณ heading จาก consecutive UTM positions
#             elif (self.prev_x_utm is not None and
#                   math.hypot(x_utm - self.prev_x_utm,
#                               y_utm - self.prev_y_utm) > 0.1):
#                 dx = x_utm - self.prev_x_utm
#                 dy = y_utm - self.prev_y_utm
#                 yaw = self.normalize_angle(math.atan2(dy, dx))
#                 self.prev_yaw = yaw

#             self.prev_x_utm = x_utm
#             self.prev_y_utm = y_utm

#             # ✅ publish UTM absolute ตรงๆ — ไม่ normalize
#             odom = Odometry()
#             odom.header.stamp    = self.get_clock().now().to_msg()
#             odom.header.frame_id = 'odom'
#             odom.child_frame_id  = 'base_link'
#             odom.pose.pose.position.x = x_utm   # ← UTM Easting  เช่น 700123.45
#             odom.pose.pose.position.y = y_utm   # ← UTM Northing เช่น 1500456.78
#             odom.pose.pose.position.z = 0.0
#             odom.pose.pose.orientation = self.yaw_to_quat(yaw)
#             odom.twist.twist.linear.x  = speed
#             self.pose_pub.publish(odom)

#             self.get_logger().info(
#                 f"📍 UTM x={x_utm:.3f} y={y_utm:.3f} "
#                 f"yaw={math.degrees(yaw):.1f}° speed={speed:.2f} fix={self.fix_quality}"
#             )

#         except Exception as e:
#             self.get_logger().warn(f"Parse error: {e}")

#     # ══════════════════════════════════════════════════════════════════════════
#     def nmea_to_decimal(self, degree_str, direction):
#         if not degree_str or degree_str == '0':
#             return 0.0
#         raw     = float(degree_str)
#         degrees = int(raw / 100)
#         minutes = raw - degrees * 100
#         decimal = degrees + minutes / 60
#         if direction in ['S', 'W']:
#             decimal = -decimal
#         return decimal

#     def yaw_to_quat(self, yaw):
#         q   = Quaternion()
#         q.w = math.cos(yaw / 2)
#         q.x = 0.0
#         q.y = 0.0
#         q.z = math.sin(yaw / 2)
#         return q

#     def normalize_angle(self, a):
#         return (a + math.pi) % (2 * math.pi) - math.pi

#     def destroy_node(self):
#         self.ser.close()
#         super().destroy_node()


# def main():
#     rclpy.init()
#     node = GnssPosePublisher()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()



# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Quaternion
# import serial
# import pynmea2
# import math
# import time
# import socket
# import base64
# import threading
# from pyproj import Transformer

# # -------- NTRIP RTK ----------
# NTRIP_SERVER_IP   = "110.78.0.54"
# NTRIP_SERVER_PORT = 2116
# NTRIP_USERNAME    = "1118600009224"
# NTRIP_PASSWORD    = "CK79"
# NTRIP_MOUNTPOINT  = "VRS_RTCM32"
# # -----------------------------

# # -------- ปรับให้ตรงกับ waypoint logger --------
# GGA_SEND_INTERVAL     = 0.2   # ⭐ ส่ง GGA บ่อย → VRS server แม่นยำ → RTK Fix เร็ว
# ALLOWED_FIX           = [4, 5]  # RTK Fixed ✅ + RTK Float 🔶 (เหมือน waypoint logger)
# MIN_SPEED_FOR_HEADING = 0.5   # m/s — ตรงกับ MIN_SPEED ของ waypoint logger
# # -----------------------------------------------

# FIX_LABELS = {
#     0: "No fix ❌",
#     1: "GPS only ⚪",
#     2: "DGPS 🔵",
#     4: "RTK Fixed ✅",
#     5: "RTK Float 🔶",
# }


# class GnssPosePublisher(Node):

#     def __init__(self):
#         super().__init__('gnss_pose_publisher')

#         # ⭐ แก้ baud rate ให้ตรงกับ waypoint logger (9600 → 115200)
#         self.GNSS_PORT = '/dev/ttyACM0'
#         self.GNSS_BAUD = 115200

#         # UTM zone 47N (ประเทศไทย)
#         self.transformer = Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)

#         self.prev_yaw    = None
#         self.prev_time   = 0.0
#         self.fix_quality = 0

#         self.prev_x_utm: float | None = None
#         self.prev_y_utm: float | None = None

#         # lat/lon ล่าสุดจาก GGA (แม่นกว่า RMC เมื่อใช้ RTK)
#         self._last_lat: float | None = None
#         self._last_lon: float | None = None

#         # ⭐ เก็บ raw GGA sentence ทั้งบรรทัด (เหมือน waypoint logger)
#         self._last_gga_sentence: str | None = None
#         self._gga_lock = threading.Lock()

#         self._ntrip_sock: socket.socket | None = None

#         self.pose_pub = self.create_publisher(Odometry, '/odom', 10)

#         try:
#             self.ser = serial.Serial(self.GNSS_PORT, self.GNSS_BAUD, timeout=1)
#             self.get_logger().info(f"✅ Opened {self.GNSS_PORT} @ {self.GNSS_BAUD} baud")
#         except Exception as e:
#             self.get_logger().error(f"❌ Cannot open serial: {e}")
#             raise

#         threading.Thread(target=self._ntrip_thread, daemon=True).start()
#         self.create_timer(0.1, self.read_gps)
#         self.get_logger().info("📡 GNSS Pose Publisher READY — UTM Absolute + RTK Float/Fixed Mode")

#     # ══════════════════════════════════════════════════════════════════════════
#     #  NTRIP thread
#     #  ⭐ ปรับ logic การส่ง GGA ให้ตรงกับ waypoint logger
#     #     - ส่ง raw GGA sentence (ทั้งบรรทัดที่ได้จาก serial ตรงๆ)
#     #     - ส่งทุก GGA_SEND_INTERVAL = 0.2 วิ
#     # ══════════════════════════════════════════════════════════════════════════
#     def _ntrip_thread(self):
#         while True:
#             try:
#                 self.get_logger().info("🛰 Connecting NTRIP...")
#                 sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#                 sock.connect((NTRIP_SERVER_IP, NTRIP_SERVER_PORT))

#                 auth = base64.b64encode(
#                     f"{NTRIP_USERNAME}:{NTRIP_PASSWORD}".encode()
#                 ).decode()

#                 # ⭐ ใช้ HTTP/1.0 แบบ waypoint logger (ไม่ส่ง Ntrip-GGA ใน header)
#                 request = (
#                     f"GET /{NTRIP_MOUNTPOINT} HTTP/1.0\r\n"
#                     f"Authorization: Basic {auth}\r\n\r\n"
#                 )
#                 sock.sendall(request.encode())
#                 self.get_logger().info("✅ NTRIP connected")

#                 sock.settimeout(0.1)
#                 last_send = 0

#                 while True:
#                     # -------- รับ RTCM → ส่งเข้า u-blox --------
#                     try:
#                         data = sock.recv(4096)
#                         if data:
#                             self.ser.write(data)
#                     except socket.timeout:
#                         pass

#                     # -------- ส่ง GGA กลับ caster (VRS) --------
#                     now = time.time()
#                     if now - last_send > GGA_SEND_INTERVAL:
#                         with self._gga_lock:
#                             gga = self._last_gga_sentence
#                         if gga:
#                             try:
#                                 sock.sendall((gga + "\r\n").encode())
#                                 last_send = now
#                             except Exception:
#                                 break

#             except Exception as e:
#                 self.get_logger().error(f"❌ NTRIP error: {e}")
#                 time.sleep(5)

#     # ══════════════════════════════════════════════════════════════════════════
#     #  GPS READ
#     # ══════════════════════════════════════════════════════════════════════════
#     def read_gps(self):
#         try:
#             line = self.ser.readline().decode('ascii', errors='replace').strip()
#         except Exception:
#             return

#         if not line.startswith('$'):
#             return

#         # ── GGA: fix quality + lat/lon + เก็บ raw sentence ──────────────────
#         if line.startswith(('$GPGGA', '$GNGGA')):
#             # ⭐ เก็บ raw sentence ทั้งบรรทัด (เหมือน waypoint logger)
#             with self._gga_lock:
#                 self._last_gga_sentence = line
#             try:
#                 gga = pynmea2.parse(line)
#                 self.fix_quality = int(gga.gps_qual)

#                 self.get_logger().info(
#                     f"🛰️  Fix: {FIX_LABELS.get(self.fix_quality, str(self.fix_quality))}",
#                     throttle_duration_sec=2.0
#                 )

#                 # ⭐ รับ lat/lon แม้ fix_quality=5 (Float) ได้เลย
#                 if self.fix_quality in ALLOWED_FIX and gga.lat and gga.lon:
#                     self._last_lat = self._nmea_to_decimal(gga.lat, gga.lat_dir)
#                     self._last_lon = self._nmea_to_decimal(gga.lon, gga.lon_dir)

#             except Exception as e:
#                 self.get_logger().warn(f"GGA parse error: {e}")
#             return

#         # ── RMC: speed + true_course (heading จาก u-blox ตรงๆ) ──────────────
#         if not line.startswith(('$GPRMC', '$GNRMC')):
#             return

#         # ⭐ กรอง fix quality ก่อน (เหมือน ALLOWED_FIX ของ waypoint logger)
#         if self.fix_quality not in ALLOWED_FIX:
#             return

#         try:
#             msg = pynmea2.parse(line)
#             if msg.status != 'A':
#                 return

#             now = time.time()
#             if now - self.prev_time < 0.1:
#                 return
#             self.prev_time = now

#             # ⭐ ใช้ lat/lon จาก GGA ก่อน (แม่นกว่า RMC ใน RTK mode)
#             if self._last_lat is not None:
#                 lat = self._last_lat
#                 lon = self._last_lon
#             else:
#                 lat = self._nmea_to_decimal(msg.lat, msg.lat_dir)
#                 lon = self._nmea_to_decimal(msg.lon, msg.lon_dir)

#             # แปลงเป็น UTM absolute
#             x_utm, y_utm = self.transformer.transform(lon, lat)

#             # knots → m/s
#             speed = float(msg.spd_over_grnd or 0) * 0.514444

#             # ── Heading จาก u-blox true_course (เหมือน waypoint logger) ──────
#             # ⭐ ใช้ true_course ตรงๆ จาก RMC โดยไม่แปลงเป็น ENU
#             #    เหมือนที่ waypoint logger ทำ: heading = float(msg.true_course)
#             #    แล้วค่อยแปลง bearing → ENU yaw เพื่อ publish quaternion
#             yaw = self.prev_yaw if self.prev_yaw is not None else 0.0

#             if msg.true_course and msg.true_course != '':
#                 bearing_deg = float(msg.true_course)
#                 if speed >= MIN_SPEED_FOR_HEADING:
#                     # GPS bearing (0°=North, CW) → ENU yaw (0°=East, CCW)
#                     yaw = self._normalize_angle(
#                         math.pi / 2.0 - math.radians(bearing_deg)
#                     )
#                     self.prev_yaw = yaw
#                     self.get_logger().info(
#                         f"🧭 Heading from u-blox true_course: {bearing_deg:.1f}°",
#                         throttle_duration_sec=1.0
#                     )

#             # ⭐ fallback จาก UTM consecutive (เหมือนเดิม แต่ใช้เฉพาะเมื่อไม่มี true_course)
#             elif (self.prev_x_utm is not None and
#                   math.hypot(x_utm - self.prev_x_utm,
#                               y_utm - self.prev_y_utm) > 0.1):
#                 dx = x_utm - self.prev_x_utm
#                 dy = y_utm - self.prev_y_utm
#                 yaw = self._normalize_angle(math.atan2(dy, dx))
#                 self.prev_yaw = yaw

#             self.prev_x_utm = x_utm
#             self.prev_y_utm = y_utm

#             # ── Publish Odometry ─────────────────────────────────────────────
#             odom = Odometry()
#             odom.header.stamp    = self.get_clock().now().to_msg()
#             odom.header.frame_id = 'odom'
#             odom.child_frame_id  = 'base_link'
#             odom.pose.pose.position.x = x_utm   # UTM Easting
#             odom.pose.pose.position.y = y_utm   # UTM Northing
#             odom.pose.pose.position.z = 0.0
#             odom.pose.pose.orientation = self._yaw_to_quat(yaw)
#             odom.twist.twist.linear.x  = speed
#             self.pose_pub.publish(odom)

#             self.get_logger().info(
#                 f"📍 UTM x={x_utm:.3f} y={y_utm:.3f} "
#                 f"yaw={math.degrees(yaw):.1f}° speed={speed:.2f} "
#                 f"fix={FIX_LABELS.get(self.fix_quality)}"
#             )

#         except Exception as e:
#             self.get_logger().warn(f"Parse error: {e}")

#     # ══════════════════════════════════════════════════════════════════════════
#     def _nmea_to_decimal(self, degree_str, direction):
#         if not degree_str or degree_str == '0':
#             return 0.0
#         raw     = float(degree_str)
#         degrees = int(raw / 100)
#         minutes = raw - degrees * 100
#         decimal = degrees + minutes / 60
#         if direction in ('S', 'W'):
#             decimal = -decimal
#         return decimal

#     def _yaw_to_quat(self, yaw):
#         q   = Quaternion()
#         q.w = math.cos(yaw / 2)
#         q.x = 0.0
#         q.y = 0.0
#         q.z = math.sin(yaw / 2)
#         return q

#     def _normalize_angle(self, a):
#         return (a + math.pi) % (2 * math.pi) - math.pi

#     def destroy_node(self):
#         self.ser.close()
#         super().destroy_node()


# def main():
#     rclpy.init()
#     node = GnssPosePublisher()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

