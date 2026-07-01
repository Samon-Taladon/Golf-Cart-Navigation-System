# # -------------------------------------------now--------------------------------------------

# import serial
# import pynmea2
# import socket
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
# from std_msgs.msg import Float32, String
# from geometry_msgs.msg import Point
# import math
# import pyproj
# from collections import deque
# import time
# import glob
# import os
# import struct
# import json
# import numpy as np

# # 🔧 เพิ่ม pyubx2 สำหรับ UBX parsing ที่ดีขึ้น
# try:
#     from pyubx2 import UBXReader, UBXMessage, UBX_MSGIDS
#     PYUBX2_AVAILABLE = True
#     print("✅ pyubx2 available - using enhanced UBX parsing")
# except ImportError:
#     PYUBX2_AVAILABLE = False
#     print("⚠️ pyubx2 not available - install with: pip3 install pyubx2")

# class EnhancedZEDf9rGNSSWithHeading(Node):
#     def __init__(self):
#         super().__init__('enhanced_zedf9r_gnss_with_heading')
        
#         # ROS2 Publishers
#         self.gnss_publisher = self.create_publisher(NavSatFix, '/navigation/gnss', 10)
#         self.xy_publisher = self.create_publisher(Point, '/navigation/xy_position', 10)
#         self.heading_publisher = self.create_publisher(Float32, '/navigation/heading', 10)
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
        
#         # 🔧 Fixed Coordinate transformation
#         self.transformer = pyproj.Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)
        
#         # Position tracking
#         self.position_history = deque(maxlen=15)
#         self.min_movement_for_heading = 0.25
        
#         # 🔧 Fixed reference point
#         self.reference_point = (13.650748, 100.492985)
#         self.reference_utm = None
        
#         # 🔧 Enhanced Heading management สำหรับ ZED-F9R
#         self.imu_heading = None
#         self.nav_att_heading = None  # 🌟 NEW: Direct from NAV-ATT
#         self.nav_att_yaw = None      # 🌟 NEW: Enhanced yaw from NAV-ATT
#         self.gps_heading = None
#         self.final_heading = None
#         self.heading_source = "NONE"
        
#         self.imu_heading_time = 0
#         self.nav_att_time = 0        # 🌟 NEW: NAV-ATT timestamp
#         self.gps_heading_time = 0
#         self.heading_timeout = 8.0
        
#         # ZED-F9R specific data
#         self.esf_status = None
#         self.nav_att = None
#         self.esf_alg = None
#         self.fusion_mode = "NONE"
#         self.imu_calibration_status = "UNKNOWN"
        
#         # 🌟 NEW: pyubx2 UBX parsing
#         self.ubx_reader = None
#         self.ubx_buffer = bytearray()
        
#         # 🔧 Enhanced Heading filtering
#         self.heading_filter = EnhancedIMUHeadingFilter()
        
#         # Performance stats
#         self.stats = {
#             'imu_count': 0,
#             'nav_att_count': 0,    # 🌟 NEW
#             'nav_att_yaw_count': 0, # 🌟 NEW
#             'gps_count': 0,
#             'position_count': 0,
#             'esf_meas_count': 0,
#             'esf_status_count': 0,
#             'pyubx2_success': 0,   # 🌟 NEW
#             'pyubx2_errors': 0,    # 🌟 NEW
#             'filtered_count': 0
#         }
        
#         # 🔧 Initialize coordinate system
#         self.init_fixed_coordinate_system()
        
#         # Initialize
#         self.init_serial_connection()
        
#         if self.ser and self.ser.is_open:
#             self.ntrip_socket = self.connect_to_ntrip()
            
#             # 🌟 Initialize pyubx2 if available
#             if PYUBX2_AVAILABLE:
#                 self.init_pyubx2()
            
#             # Configure ZED-F9R for enhanced IMU
#             self.configure_enhanced_zed_f9r()
            
#             # Main processing timer
#             self.create_timer(0.1, self.main_loop)
            
#             self.get_logger().info('✅ Enhanced ZED-F9R GNSS with NAV-ATT Yaw initialized')
#             self.get_logger().info(f'🔧 pyubx2: {"Available" if PYUBX2_AVAILABLE else "Not available"}')
#         else:
#             self.get_logger().error('❌ Failed to initialize')
    
#     def init_pyubx2(self):
#         """🌟 Initialize pyubx2 for enhanced UBX parsing"""
#         try:
#             if PYUBX2_AVAILABLE:
#                 # pyubx2 will be used for parsing, but we still need the serial stream
#                 self.get_logger().info('🌟 pyubx2 UBX parser initialized')
#             else:
#                 self.get_logger().warning('⚠️ pyubx2 not available, using manual parsing')
#         except Exception as e:
#             self.get_logger().error(f'❌ pyubx2 initialization failed: {e}')
    
#     def init_fixed_coordinate_system(self):
#         """🔧 Initialize fixed coordinate system"""
#         try:
#             self.reference_utm = self.transformer.transform(
#                 self.reference_point[1], self.reference_point[0]
#             )
#             self.get_logger().info(
#                 f'🔧 Enhanced coordinate system initialized:\n'
#                 f'  📍 Reference: ({self.reference_point[0]:.6f}, {self.reference_point[1]:.6f})\n'
#                 f'  📍 UTM: ({self.reference_utm[0]:.2f}, {self.reference_utm[1]:.2f})'
#             )
#         except Exception as e:
#             self.get_logger().error(f'❌ Coordinate system init failed: {e}')
#             self.reference_utm = None
    
#     def configure_enhanced_zed_f9r(self):
#         """🌟 Enhanced ZED-F9R configuration for better NAV-ATT"""
#         try:
#             self.get_logger().info('🌟 Configuring ZED-F9R for enhanced NAV-ATT...')
#             time.sleep(2)
            
#             # Enable UBX messages with higher rates
#             self.enable_ubx_message(0x01, 0x05, 5)  # 🌟 NAV-ATT at 5Hz (was 1Hz)
#             self.enable_ubx_message(0x10, 0x02, 2)  # ESF-MEAS at 2Hz
#             self.enable_ubx_message(0x10, 0x10, 1)  # ESF-STATUS
#             self.enable_ubx_message(0x10, 0x14, 1)  # ESF-ALG
            
#             # 🌟 Enhanced IMU configuration for better accuracy
#             # Enable high-precision attitude output
#             cfg_payload = struct.pack('<LLB', 0x20, 0x40270001, 1)  # High precision mode
#             cfg_cmd = self.create_ubx_message(0x06, 0x8A, cfg_payload)
#             self.ser.write(cfg_cmd)
#             time.sleep(0.1)
            
#             # Enable automatic IMU alignment with faster convergence
#             cfg_payload = struct.pack('<LLB', 0x20, 0x401A001A, 1)
#             cfg_cmd = self.create_ubx_message(0x06, 0x8A, cfg_payload)
#             self.ser.write(cfg_cmd)
#             time.sleep(0.1)
            
#             # Enable sensor fusion
#             cfg_payload = struct.pack('<LLB', 0x20, 0x40240001, 1)
#             cfg_cmd = self.create_ubx_message(0x06, 0x8A, cfg_payload)
#             self.ser.write(cfg_cmd)
#             time.sleep(0.1)
            
#             # 🌟 Set higher IMU output rate for better heading
#             cfg_payload = struct.pack('<LLB', 0x20, 0x40250002, 10)  # 10Hz IMU (was 5Hz)
#             cfg_cmd = self.create_ubx_message(0x06, 0x8A, cfg_payload)
#             self.ser.write(cfg_cmd)
#             time.sleep(0.1)
            
#             # Save configuration
#             save_payload = struct.pack('<LLLL', 0x00000000, 0xFFFFFFFF, 0x00000000, 0x00000000)
#             save_cmd = self.create_ubx_message(0x06, 0x09, save_payload)
#             self.ser.write(save_cmd)
#             time.sleep(0.5)
            
#             self.get_logger().info('✅ Enhanced ZED-F9R configuration sent (5Hz NAV-ATT, 10Hz IMU)')
#             self.get_logger().info('⏳ รอ IMU calibration สำหรับ high-precision heading...')
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Enhanced configuration failed: {e}')
    
#     def create_ubx_message(self, msg_class, msg_id, payload):
#         """สร้าง UBX message"""
#         header = struct.pack('<BBBB', 0xb5, 0x62, msg_class, msg_id)
#         length = struct.pack('<H', len(payload))
        
#         ck_a = ck_b = 0
#         for byte in struct.pack('<BB', msg_class, msg_id) + length + payload:
#             ck_a = (ck_a + byte) & 0xFF
#             ck_b = (ck_b + ck_a) & 0xFF
        
#         return header + length + payload + struct.pack('<BB', ck_a, ck_b)
    
#     def enable_ubx_message(self, msg_class, msg_id, rate=1):
#         """เปิดใช้ UBX message"""
#         payload = struct.pack('<BBB', msg_class, msg_id, rate)
#         cmd = self.create_ubx_message(0x06, 0x01, payload)
#         self.ser.write(cmd)
#         time.sleep(0.1)
    
#     def find_available_serial_ports(self):
#         """Find available serial ports"""
#         available_ports = []
#         for port in self.possible_serial_ports:
#             if os.path.exists(port):
#                 available_ports.append(port)
        
#         acm_ports = glob.glob('/dev/ttyACM*')
#         usb_ports = glob.glob('/dev/ttyUSB*')
        
#         for port in acm_ports + usb_ports:
#             if port not in available_ports:
#                 available_ports.append(port)
        
#         return sorted(available_ports)
    
#     def init_serial_connection(self):
#         """Initialize serial connection"""
#         self.get_logger().info("🔍 Searching for Enhanced ZED-F9R device...")
        
#         available_ports = self.find_available_serial_ports()
#         if not available_ports:
#             self.get_logger().error("❌ No serial ports found!")
#             return
        
#         baud_rates = [38400, 9600, 115200]
        
#         for port in available_ports:
#             for baud in baud_rates:
#                 try:
#                     test_ser = serial.Serial(port, baud, timeout=1)
#                     time.sleep(1)
                    
#                     if test_ser.in_waiting > 0 or self.test_serial_port_advanced(test_ser):
#                         self.ser = test_ser
#                         self.serial_port = port
#                         self.get_logger().info(f"✅ Enhanced connection to {port} at {baud} baud")
#                         return
                    
#                     test_ser.close()
#                 except:
#                     continue
        
#         # Last resort
#         if available_ports:
#             try:
#                 port = available_ports[0]
#                 self.ser = serial.Serial(port, 38400, timeout=1)
#                 self.serial_port = port
#                 self.get_logger().warning(f"⚠️ Connected to {port} without validation")
#             except Exception as e:
#                 self.get_logger().error(f"❌ Failed to connect: {e}")
    
#     def test_serial_port_advanced(self, test_ser):
#         """Advanced port testing"""
#         try:
#             start_time = time.time()
#             while time.time() - start_time < 2:
#                 if test_ser.in_waiting > 0:
#                     data = test_ser.read(test_ser.in_waiting)
#                     try:
#                         text = data.decode('ascii', errors='replace')
#                         if any(marker in text for marker in ['$G', 'UBX', '\xb5\x62']):
#                             return True
#                     except:
#                         pass
#                 time.sleep(0.1)
#             return False
#         except:
#             return False
    
#     def connect_to_ntrip(self):
#         """Connect to NTRIP server"""
#         try:
#             client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#             client_socket.connect((self.ntrip_server_ip, self.ntrip_server_port))
            
#             import base64
#             auth_str = f"{self.ntrip_username}:{self.ntrip_password}"
#             auth_b64 = base64.b64encode(auth_str.encode('ascii')).decode('ascii')
            
#             request = (
#                 f"GET /{self.mount_point} HTTP/1.1\r\n"
#                 f"User-Agent: NTRIP Client\r\n" 
#                 f"Authorization: Basic {auth_b64}\r\n\r\n"
#             )
#             client_socket.send(request.encode('ascii'))
#             self.get_logger().info("🌐 Connected to NTRIP server")
#             return client_socket
#         except Exception as e:
#             self.get_logger().warning(f"⚠️ NTRIP failed: {e}")
#             return None
    
#     def main_loop(self):
#         """Enhanced main processing loop"""
#         if not (self.ser and self.ser.is_open):
#             return
        
#         try:
#             # Read all available data
#             if self.ser.in_waiting > 0:
#                 data = self.ser.read(self.ser.in_waiting)
                
#                 # Process NMEA (text)
#                 try:
#                     text_data = data.decode('ascii', errors='replace')
#                     for line in text_data.split('\n'):
#                         line = line.strip()
#                         if line.startswith('$'):
#                             self.process_nmea_line(line)
#                 except:
#                     pass
                
#                 # 🌟 Enhanced UBX processing
#                 if PYUBX2_AVAILABLE:
#                     self.process_ubx_data_enhanced(data)
#                 else:
#                     self.process_ubx_data_manual(data)
            
#             # Handle NTRIP
#             self.handle_ntrip_data()
            
#             # 🌟 Update final heading with enhanced logic
#             self.update_enhanced_final_heading()
            
#             # Publish debug every 2 seconds
#             if int(time.time() * 0.5) % 1 == 0:
#                 self.publish_enhanced_debug()
        
#         except Exception as e:
#             self.get_logger().error(f"❌ Enhanced main loop error: {e}")
    
#     def process_ubx_data_enhanced(self, data):
#         """🌟 Enhanced UBX processing with pyubx2"""
#         self.ubx_buffer.extend(data)
        
#         while len(self.ubx_buffer) >= 8:
#             try:
#                 # Find UBX sync
#                 sync_pos = self.ubx_buffer.find(b'\xb5\x62')
#                 if sync_pos == -1:
#                     self.ubx_buffer.clear()
#                     break
                
#                 if sync_pos > 0:
#                     self.ubx_buffer = self.ubx_buffer[sync_pos:]
                
#                 if len(self.ubx_buffer) < 8:
#                     break
                
#                 # Try to parse with pyubx2
#                 try:
#                     # Get message length
#                     length = struct.unpack('<H', self.ubx_buffer[4:6])[0]
#                     total_len = 8 + length
                    
#                     if len(self.ubx_buffer) < total_len:
#                         break
                    
#                     # Extract complete message
#                     message_bytes = self.ubx_buffer[:total_len]
                    
#                     # Parse with pyubx2
#                     ubx_msg = UBXMessage.parse(message_bytes)
                    
#                     # Process specific messages
#                     if ubx_msg.identity == 'NAV-ATT':
#                         self.process_nav_att_enhanced(ubx_msg)
#                     elif ubx_msg.identity == 'ESF-STATUS':
#                         self.process_esf_status_enhanced(ubx_msg)
#                     elif ubx_msg.identity == 'ESF-MEAS':
#                         self.stats['esf_meas_count'] += 1
                    
#                     self.stats['pyubx2_success'] += 1
                    
#                     # Remove processed message
#                     self.ubx_buffer = self.ubx_buffer[total_len:]
                    
#                 except Exception as parse_error:
#                     # Fallback to manual parsing
#                     self.stats['pyubx2_errors'] += 1
#                     self.process_ubx_data_manual(message_bytes)
#                     self.ubx_buffer = self.ubx_buffer[1:]
                    
#             except Exception as e:
#                 self.ubx_buffer = self.ubx_buffer[1:]
    
#     def process_nav_att_enhanced(self, ubx_msg):
#         """🌟 Enhanced NAV-ATT processing with pyubx2"""
#         try:
#             # Extract attitude data
#             heading = ubx_msg.heading * 1e-5  # Convert to degrees
#             roll = ubx_msg.roll * 1e-5
#             pitch = ubx_msg.pitch * 1e-5
            
#             # 🌟 Get accuracy information if available
#             head_acc = getattr(ubx_msg, 'headAcc', 0) * 1e-5 if hasattr(ubx_msg, 'headAcc') else None
            
#             # Validate heading
#             if 0 <= heading <= 360:
#                 current_time = time.time()
                
#                 # 🌟 Enhanced filtering based on accuracy
#                 confidence_factor = 1.0
#                 if head_acc is not None and head_acc < 5.0:  # Good accuracy (< 5 degrees)
#                     confidence_factor = 1.2
#                 elif head_acc is not None and head_acc > 15.0:  # Poor accuracy (> 15 degrees)
#                     confidence_factor = 0.5
                
#                 # Apply enhanced heading filter
#                 filtered_heading = self.heading_filter.filter_nav_att_heading(
#                     heading, confidence_factor
#                 )
                
#                 if filtered_heading is not None:
#                     self.nav_att_heading = filtered_heading
#                     self.nav_att_yaw = heading  # Raw yaw for comparison
#                     self.nav_att_time = current_time
#                     self.stats['nav_att_count'] += 1
#                     self.stats['nav_att_yaw_count'] += 1
                    
#                     # Store enhanced attitude data
#                     self.nav_att = {
#                         'roll': roll,
#                         'pitch': pitch,
#                         'heading': heading,
#                         'filtered_heading': filtered_heading,
#                         'head_accuracy': head_acc,
#                         'confidence_factor': confidence_factor,
#                         'timestamp': current_time
#                     }
                    
#                     self.get_logger().debug(
#                         f"🌟 Enhanced NAV-ATT: Raw={heading:.1f}°, "
#                         f"Filtered={filtered_heading:.1f}°, "
#                         f"Acc={head_acc:.1f}° (CF:{confidence_factor:.1f})"
#                     )
        
#         except Exception as e:
#             self.get_logger().debug(f"Enhanced NAV-ATT parse error: {e}")
    
#     def process_ubx_data_manual(self, data):
#         """Fallback manual UBX processing"""
#         # Keep the original manual parsing as fallback
#         while len(self.ubx_buffer) >= 8:
#             sync_pos = self.ubx_buffer.find(b'\xb5\x62')
#             if sync_pos == -1:
#                 self.ubx_buffer.clear()
#                 break
            
#             if sync_pos > 0:
#                 self.ubx_buffer = self.ubx_buffer[sync_pos:]
            
#             if len(self.ubx_buffer) < 8:
#                 break
            
#             try:
#                 cls = self.ubx_buffer[2]
#                 id = self.ubx_buffer[3] 
#                 length = struct.unpack('<H', self.ubx_buffer[4:6])[0]
                
#                 total_len = 8 + length
#                 if len(self.ubx_buffer) < total_len:
#                     break
                
#                 payload = self.ubx_buffer[6:6+length]
                
#                 if cls == 0x01 and id == 0x05:  # NAV-ATT
#                     self.process_nav_att_manual(payload)
#                 elif cls == 0x10 and id == 0x10:  # ESF-STATUS
#                     self.process_esf_status_manual(payload)
                
#                 self.ubx_buffer = self.ubx_buffer[total_len:]
                
#             except Exception as e:
#                 self.ubx_buffer = self.ubx_buffer[1:]
    
#     def process_nav_att_manual(self, payload):
#         """Manual NAV-ATT processing (fallback)"""
#         try:
#             if len(payload) >= 32:
#                 data = struct.unpack('<LLLLLLLLLHHH', payload[:32])
#                 iTOW, version, reserved1, roll, pitch, heading = data[:6]
                
#                 heading_deg = (heading * 1e-5) % 360
                
#                 if 0 <= heading_deg <= 360:
#                     filtered_heading = self.heading_filter.filter_nav_att_heading(heading_deg, 1.0)
                    
#                     if filtered_heading is not None:
#                         self.nav_att_heading = filtered_heading
#                         self.nav_att_time = time.time()
#                         self.stats['nav_att_count'] += 1
        
#         except Exception as e:
#             self.get_logger().debug(f"Manual NAV-ATT parse error: {e}")
    
#     def process_esf_status_enhanced(self, ubx_msg):
#         """Enhanced ESF-STATUS processing"""
#         try:
#             fusion_mode = ubx_msg.fusionMode
#             init_status1 = ubx_msg.initStatus1
            
#             # Update status
#             fusion_modes = {0: "INIT", 1: "FUSION", 2: "SUSPENDED", 3: "DISABLED"}
#             self.fusion_mode = fusion_modes.get(fusion_mode, f"UNKNOWN({fusion_mode})")
            
#             # Enhanced calibration status
#             if init_status1 & 0x01:
#                 if init_status1 & 0x02:
#                     self.imu_calibration_status = "FULLY_CALIBRATED"
#                 else:
#                     self.imu_calibration_status = "CALIBRATED"
#             else:
#                 self.imu_calibration_status = "CALIBRATING"
            
#             self.stats['esf_status_count'] += 1
            
#         except Exception as e:
#             self.get_logger().debug(f"Enhanced ESF-STATUS parse error: {e}")
    
#     def process_esf_status_manual(self, payload):
#         """Manual ESF-STATUS processing (fallback)"""
#         try:
#             if len(payload) >= 16:
#                 data = struct.unpack('<LLLLBBBB', payload[:16])
#                 fusionMode = data[5]
                
#                 fusion_modes = {0: "INIT", 1: "FUSION", 2: "SUSPENDED", 3: "DISABLED"}
#                 self.fusion_mode = fusion_modes.get(fusionMode, f"UNKNOWN({fusionMode})")
#                 self.stats['esf_status_count'] += 1
        
#         except Exception as e:
#             pass
    
#     def process_nmea_line(self, line):
#         """Process NMEA line (for position)"""
#         try:
#             if any(line.startswith(nmea) for nmea in ['$GPGGA', '$GNGGA', '$GPRMC', '$GNRMC', '$GPGLL', '$GNGLL']):
#                 msg = pynmea2.parse(line)
                
#                 if hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
#                     if msg.latitude and msg.longitude:
#                         self.process_position(msg.latitude, msg.longitude)
            
#         except Exception as e:
#             self.get_logger().debug(f"NMEA parse error: {e}")
    
#     def process_position(self, latitude, longitude):
#         """Process GNSS position with enhanced coordinate system"""
#         current_time = time.time()
        
#         if not (-90 <= latitude <= 90) or not (-180 <= longitude <= 180):
#             return
        
#         # Convert using fixed coordinate system
#         local_x, local_y = self.latlon_to_xy_fixed(latitude, longitude)
        
#         # Store position
#         position_record = {
#             'lat': latitude, 'lon': longitude,
#             'local_x': local_x, 'local_y': local_y,
#             'timestamp': current_time
#         }
#         self.position_history.append(position_record)
#         self.stats['position_count'] += 1
        
#         # Calculate GPS heading
#         self.update_gps_heading_improved()
        
#         # Publish data
#         self.publish_data(latitude, longitude, local_x, local_y)
    
#     def latlon_to_xy_fixed(self, lat, lon):
#         """Convert lat/lon using fixed PyProj"""
#         try:
#             if self.reference_utm is None:
#                 return self.simple_latlon_to_xy(lat, lon)
            
#             utm_x, utm_y = self.transformer.transform(lon, lat)
#             local_x = utm_x - self.reference_utm[0]
#             local_y = utm_y - self.reference_utm[1]
            
#             return local_x, local_y
            
#         except Exception as e:
#             return self.simple_latlon_to_xy(lat, lon)
    
#     def simple_latlon_to_xy(self, lat, lon):
#         """Fallback simple conversion"""
#         dlat = lat - self.reference_point[0]
#         dlon = lon - self.reference_point[1]
        
#         x = dlon * 111319.9 * math.cos(math.radians(lat))
#         y = dlat * 111319.9
        
#         return x, y
    
#     def update_gps_heading_improved(self):
#         """Enhanced GPS heading calculation"""
#         if len(self.position_history) < 3:
#             return
        
#         total_distance = 0.0
#         weighted_x = weighted_y = 0.0
        
#         recent_positions = list(self.position_history)[-5:]
        
#         for i in range(len(recent_positions) - 1):
#             curr = recent_positions[i+1]
#             prev = recent_positions[i]
            
#             dx = curr['local_x'] - prev['local_x']
#             dy = curr['local_y'] - prev['local_y']
#             distance = math.sqrt(dx*dx + dy*dy)
            
#             if distance > self.min_movement_for_heading:
#                 heading_rad = math.atan2(dx, dy)
                
#                 time_weight = (i + 1) / len(recent_positions)
#                 weight = distance * time_weight
                
#                 weighted_x += math.cos(heading_rad) * weight
#                 weighted_y += math.sin(heading_rad) * weight
#                 total_distance += weight
        
#         if total_distance > self.min_movement_for_heading:
#             avg_heading = math.degrees(math.atan2(weighted_y, weighted_x))
#             if avg_heading < 0:
#                 avg_heading += 360
            
#             filtered_gps_heading = self.heading_filter.filter_gps_heading(avg_heading)
            
#             if filtered_gps_heading is not None:
#                 self.gps_heading = filtered_gps_heading
#                 self.gps_heading_time = time.time()
#                 self.stats['gps_count'] += 1
    
#     def update_enhanced_final_heading(self):
#         """🌟 Enhanced final heading selection with NAV-ATT priority"""
#         current_time = time.time()
        
#         # Check data validity
#         nav_att_valid = (self.nav_att_heading is not None and 
#                         (current_time - self.nav_att_time) < self.heading_timeout and
#                         self.fusion_mode in ["FUSION", "INIT"])
        
#         gps_valid = (self.gps_heading is not None and 
#                     (current_time - self.gps_heading_time) < self.heading_timeout)
        
#         # 🌟 Enhanced priority: NAV-ATT > GPS > Hold
#         if nav_att_valid and self.imu_calibration_status in ["FULLY_CALIBRATED", "CALIBRATED"]:
#             # Highest priority: Use enhanced NAV-ATT when calibrated
#             self.final_heading = self.nav_att_heading
#             self.heading_source = f"NAV_ATT_{self.imu_calibration_status}"
            
#         elif nav_att_valid and self.imu_calibration_status == "CALIBRATING":
#             # Blend NAV-ATT and GPS during calibration
#             if gps_valid:
#                 self.final_heading = self.blend_headings(self.nav_att_heading, self.gps_heading, 0.8)
#                 self.heading_source = "NAV_ATT_GPS_BLEND"
#             else:
#                 self.final_heading = self.nav_att_heading
#                 self.heading_source = "NAV_ATT_CALIBRATING"
                
#         elif gps_valid:
#             # Fallback to GPS
#             self.final_heading = self.gps_heading  
#             self.heading_source = "GPS"
            
#         else:
#             # No valid heading
#             self.final_heading = None
#             self.heading_source = "NONE"
        
#         # Publish enhanced heading
#         if self.final_heading is not None:
#             heading_msg = Float32()
#             heading_msg.data = float(self.final_heading)
#             self.heading_publisher.publish(heading_msg)
#             self.stats['filtered_count'] += 1
    
#     def blend_headings(self, heading1, heading2, weight1):
#         """Blend two headings considering circular nature"""
#         if heading1 is None or heading2 is None:
#             return heading1 or heading2
        
#         x1, y1 = math.cos(math.radians(heading1)), math.sin(math.radians(heading1))
#         x2, y2 = math.cos(math.radians(heading2)), math.sin(math.radians(heading2))
        
#         weight2 = 1.0 - weight1
#         x_blend = weight1 * x1 + weight2 * x2
#         y_blend = weight1 * y1 + weight2 * y2
        
#         blended_heading = math.degrees(math.atan2(y_blend, x_blend))
#         return blended_heading % 360
    
#     def publish_data(self, latitude, longitude, local_x, local_y):
#         """Publish all data"""
#         # GNSS
#         gnss_msg = NavSatFix()
#         gnss_msg.latitude = latitude
#         gnss_msg.longitude = longitude
#         gnss_msg.status.status = 0
#         gnss_msg.status.service = 1
#         self.gnss_publisher.publish(gnss_msg)
        
#         # XY position
#         xy_msg = Point()
#         xy_msg.x = local_x
#         xy_msg.y = local_y
#         xy_msg.z = 0.0
#         self.xy_publisher.publish(xy_msg)
    
#     def publish_enhanced_debug(self):
#         """🌟 Enhanced debug information with NAV-ATT details"""
#         debug_data = {
#             'final_heading': {
#                 'value': self.final_heading,
#                 'source': self.heading_source,
#                 'is_valid': self.final_heading is not None
#             },
#             'sources': {
#                 'NAV_ATT': {
#                     'raw_yaw': self.nav_att_yaw,
#                     'filtered': self.nav_att_heading,
#                     'age': time.time() - self.nav_att_time if self.nav_att_time > 0 else float('inf'),
#                     'accuracy': self.nav_att.get('head_accuracy') if self.nav_att else None,
#                     'confidence': self.nav_att.get('confidence_factor') if self.nav_att else None
#                 },
#                 'GPS': {
#                     'filtered': self.gps_heading,
#                     'age': time.time() - self.gps_heading_time if self.gps_heading_time > 0 else float('inf'),
#                     'movement_points': len(self.position_history)
#                 }
#             },
#             'zed_f9r_status': {
#                 'fusion_mode': self.fusion_mode,
#                 'calibration_status': self.imu_calibration_status,
#                 'nav_att_data': self.nav_att
#             },
#             'coordinate_system': {
#                 'type': 'Enhanced_UTM_Zone_47N_PyProj',
#                 'reference_lat': self.reference_point[0],
#                 'reference_lon': self.reference_point[1]
#             },
#             'enhanced_stats': {
#                 'pyubx2_available': PYUBX2_AVAILABLE,
#                 'pyubx2_success': self.stats['pyubx2_success'],
#                 'pyubx2_errors': self.stats['pyubx2_errors'],
#                 'nav_att_count': self.stats['nav_att_count'],
#                 'nav_att_yaw_count': self.stats['nav_att_yaw_count']
#             },
#             'filter_stats': self.heading_filter.get_stats(),
#             'stats': self.stats,
#             'port': self.serial_port
#         }
        
#         msg = String()
#         msg.data = json.dumps(debug_data, default=str)
#         self.heading_debug_publisher.publish(msg)
    
#     def handle_ntrip_data(self):
#         """Handle NTRIP corrections"""
#         if self.ntrip_socket:
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
    
#     def destroy_node(self):
#         """Cleanup"""
#         self.get_logger().info("🔄 Shutting down Enhanced ZED-F9R...")
        
#         if self.ser and self.ser.is_open:
#             self.ser.close()
        
#         if hasattr(self, 'ntrip_socket') and self.ntrip_socket:
#             self.ntrip_socket.close()
        
#         super().destroy_node()


# # 🌟 Enhanced IMU Heading Filter
# class EnhancedIMUHeadingFilter:
#     def __init__(self):
#         self.nav_att_history = deque(maxlen=12)  # เพิ่มจาก 10 เป็น 12
#         self.gps_history = deque(maxlen=8)
        
#         # Enhanced filter parameters
#         self.nav_att_outlier_threshold = 20.0  # degrees (ลดจาก 25)
#         self.gps_outlier_threshold = 18.0      # degrees (ลดจาก 20)
#         self.smoothing_alpha = 0.25            # (ลดจาก 0.3)
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
#         """🌟 Enhanced NAV-ATT heading filtering with confidence"""
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
#         """🌟 Enhanced NAV-ATT smoothing with confidence weighting"""
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
#         """🌟 Calculate weighted circular mean"""
#         if not angles or len(angles) != len(weights):
#             return 0
        
#         x = np.sum(weights * np.array([math.cos(math.radians(a)) for a in angles]))
#         y = np.sum(weights * np.array([math.sin(math.radians(a)) for a in angles]))
        
#         return math.degrees(math.atan2(y, x)) % 360
    
#     def get_stats(self):
#         """Get enhanced filter statistics"""
#         return self.stats.copy()


# def main(args=None):
#     rclpy.init(args=args)
#     gnss_publisher = EnhancedZEDf9rGNSSWithHeading()
    
#     try:
#         rclpy.spin(gnss_publisher)
#     except KeyboardInterrupt:
#         gnss_publisher.get_logger().info("🛑 Keyboard interrupt")
#     finally:
#         gnss_publisher.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()


# ------------------------------------------------รุ่นพี่-----------------------------------------------------------------

# # !/usr/bin/env python3
# import serial
# import pynmea2
# import socket
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
# from std_msgs.msg import Float32, String, Float64MultiArray
# from geometry_msgs.msg import Point, Twist
# from rcl_interfaces.msg import ParameterDescriptor  # เพิ่ม import สำหรับ parameter descriptor
# import math
# import pyproj
# from collections import deque
# import time
# import glob
# import os
# import struct
# import json
# import numpy as np

# # เพิ่ม pyubx2 สำหรับ UBX parsing ที่ดีขึ้น
# try:
#     from pyubx2 import UBXReader, UBXMessage, UBX_MSGIDS
#     PYUBX2_AVAILABLE = True
#     print("✅ pyubx2 available - using enhanced UBX parsing")
# except ImportError:
#     PYUBX2_AVAILABLE = False
#     print("⚠️ pyubx2 not available - install with: pip3 install pyubx2")

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


# class EnhancedZEDf9rGNSSWithHeading(Node):
#     def __init__(self):
#         super().__init__('enhanced_zedf9r_gnss_with_heading')
        
#         # ✅ 🌟 Vehicle parameters สำหรับรถของคุณ - ประกาศ ROS2 Parameters
#         self.declare_vehicle_parameters()
        
#         # ROS2 Publishers - เพิ่ม cmd_vel publisher
#         self.gnss_publisher = self.create_publisher(NavSatFix, '/navigation/gnss', 10)
#         self.xy_publisher = self.create_publisher(Float64MultiArray, 'Realtime_XY', 10)
#         self.raw_gps_publisher = self.create_publisher(Float64MultiArray, 'Raw_GPS', 10)
#         self.xy_debug_publisher = self.create_publisher(Point, '/navigation/xy_debug', 10)
#         self.heading_publisher = self.create_publisher(Float32, '/navigation/heading', 10)
#         self.heading_debug_publisher = self.create_publisher(String, '/navigation/heading_debug', 10)
#         self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)  # ✅ เพิ่ม cmd_vel publisher
        
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
        
#         # ✅ Fixed Coordinate transformation
#         self.transformer = pyproj.Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)
        
#         # Position tracking
#         self.position_history = deque(maxlen=15)
#         self.min_movement_for_heading = 0.25
        
#         # ✅ Fixed reference point - ปรับให้ตรงกับเส้นทาง
#         self.reference_point = (13.650748, 100.492985)
#         self.reference_utm = None
        
#         # ✅ กำหนด UTM coordinates เป้าหมายให้ตรงกับเส้นทาง
#         self.TARGET_UTM_X = 661452.0  # จุดกลาง X ของเส้นทางทั้งหมด
#         self.TARGET_UTM_Y = 1509601.0  # จุดกลาง Y ของเส้นทางทั้งหมด
#         self.utm_offset_x = 0.0
#         self.utm_offset_y = 0.0
        
#         # Enhanced Heading management สำหรับ ZED-F9R
#         self.imu_heading = None
#         self.nav_att_heading = None
#         self.nav_att_yaw = None
#         self.gps_heading = None
#         self.final_heading = None
#         self.heading_source = "NONE"
        
#         self.imu_heading_time = 0
#         self.nav_att_time = 0
#         self.gps_heading_time = 0
#         self.heading_timeout = 8.0
        
#         # ZED-F9R specific data
#         self.esf_status = None
#         self.nav_att = None
#         self.esf_alg = None
#         self.fusion_mode = "NONE"
#         self.imu_calibration_status = "UNKNOWN"
        
#         # pyubx2 UBX parsing
#         self.ubx_reader = None
#         self.ubx_buffer = bytearray()
        
#         # Enhanced Heading filtering
#         self.heading_filter = EnhancedIMUHeadingFilter()
        
#         # Performance stats
#         self.stats = {
#             'imu_count': 0,
#             'nav_att_count': 0,
#             'nav_att_yaw_count': 0,
#             'gps_count': 0,
#             'position_count': 0,
#             'esf_meas_count': 0,
#             'esf_status_count': 0,
#             'pyubx2_success': 0,
#             'pyubx2_errors': 0,
#             'filtered_count': 0
#         }
        
#         # ✅ Initialize coordinate system
#         self.init_track_coordinate_system()
        
#         # Initialize
#         self.init_serial_connection()
        
#         if self.ser and self.ser.is_open:
#             self.ntrip_socket = self.connect_to_ntrip()
            
#             # Initialize pyubx2 if available
#             if PYUBX2_AVAILABLE:
#                 self.init_pyubx2()
            
#             # Configure ZED-F9R for enhanced IMU
#             self.configure_enhanced_zed_f9r()
            
#             # Main processing timer
#             self.create_timer(0.1, self.main_loop)
            
#             self.get_logger().info('✅ Enhanced ZED-F9R GNSS with Track Coordinates initialized')
#             self.get_logger().info(f'🔧 pyubx2: {"Available" if PYUBX2_AVAILABLE else "Not available"}')
            
#             # ✅ Log vehicle parameters
#             self.log_vehicle_parameters()
#         else:
#             self.get_logger().error('❌ Failed to initialize')
    
#     def declare_vehicle_parameters(self):
#         """✅ ประกาศ Vehicle Parameters สำหรับรถของคุณ"""
        
#         # Wheelbase parameter
#         wheelbase_descriptor = ParameterDescriptor(
#             description='ฐานล้อของรถ (ระยะห่างระหว่างเพลาหน้าและหลัง) ในหน่วยเมตร'
#         )
#         self.declare_parameter('wheelbase', 1.67, wheelbase_descriptor)
        
#         # Max steering angle parameter
#         steering_descriptor = ParameterDescriptor(
#             description='มุมเลี้ยวสูงสุดของรถในหน่วย radians (50 degrees = 0.873 rad)'
#         )
#         self.declare_parameter('max_steering_angle', 0.873, steering_descriptor)
        
#         # Fixed speed parameter
#         speed_descriptor = ParameterDescriptor(
#             description='ความเร็วคงที่ของรถในหน่วย m/s'
#         )
#         self.declare_parameter('fixed_speed', 1.0, speed_descriptor)
        
#         # เพิ่ม parameters เสริมสำหรับการควบคุม
#         min_speed_descriptor = ParameterDescriptor(
#             description='ความเร็วต่ำสุดของรถในหน่วย m/s'
#         )
#         self.declare_parameter('min_speed', 0.1, min_speed_descriptor)
        
#         max_speed_descriptor = ParameterDescriptor(
#             description='ความเร็วสูงสุดของรถในหน่วย m/s'
#         )
#         self.declare_parameter('max_speed', 3.0, max_speed_descriptor)
        
#         # Angular velocity limits
#         max_angular_vel_descriptor = ParameterDescriptor(
#             description='ความเร็วเชิงมุมสูงสุดในหน่วย rad/s'
#         )
#         self.declare_parameter('max_angular_velocity', 1.0, max_angular_vel_descriptor)
        
#         self.get_logger().info('🌟 Vehicle parameters declared successfully!')
    
#     def get_vehicle_parameters(self):
#         """✅ ดึงค่า Vehicle Parameters ปัจจุบัน"""
#         try:
#             wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
#             max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value
#             fixed_speed = self.get_parameter('fixed_speed').get_parameter_value().double_value
#             min_speed = self.get_parameter('min_speed').get_parameter_value().double_value
#             max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
#             max_angular_velocity = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
            
#             return {
#                 'wheelbase': wheelbase,
#                 'max_steering_angle': max_steering_angle,
#                 'fixed_speed': fixed_speed,
#                 'min_speed': min_speed,
#                 'max_speed': max_speed,
#                 'max_angular_velocity': max_angular_velocity
#             }
#         except Exception as e:
#             self.get_logger().error(f'❌ Error getting vehicle parameters: {e}')
#             # Return default values
#             return {
#                 'wheelbase': 1.67,
#                 'max_steering_angle': 0.873,
#                 'fixed_speed': 1.0,
#                 'min_speed': 0.1,
#                 'max_speed': 3.0,
#                 'max_angular_velocity': 1.0
#             }
    
#     def log_vehicle_parameters(self):
#         """✅ แสดง Vehicle Parameters ปัจจุบัน"""
#         params = self.get_vehicle_parameters()
#         self.get_logger().info(
#             f'🚗 Vehicle Parameters:\n'
#             f'  🔧 Wheelbase: {params["wheelbase"]:.2f} m\n'
#             f'  🔧 Max Steering Angle: {params["max_steering_angle"]:.3f} rad ({math.degrees(params["max_steering_angle"]):.1f}°)\n'
#             f'  🔧 Fixed Speed: {params["fixed_speed"]:.2f} m/s\n'
#             f'  🔧 Speed Range: {params["min_speed"]:.2f} - {params["max_speed"]:.2f} m/s\n'
#             f'  🔧 Max Angular Velocity: {params["max_angular_velocity"]:.2f} rad/s'
#         )
    
#     def init_pyubx2(self):
#         """Initialize pyubx2 for enhanced UBX parsing"""
#         try:
#             if PYUBX2_AVAILABLE:
#                 self.get_logger().info('🌟 pyubx2 UBX parser initialized')
#             else:
#                 self.get_logger().warning('⚠️ pyubx2 not available, using manual parsing')
#         except Exception as e:
#             self.get_logger().error(f'❌ pyubx2 initialization failed: {e}')
    
#     def init_track_coordinate_system(self):
#         """✅ Initialize coordinate system ให้ตรงกับเส้นทาง"""
#         try:
#             # แปลง reference GPS เป็น UTM
#             ref_utm_x, ref_utm_y = self.transformer.transform(
#                 self.reference_point[1], self.reference_point[0]
#             )
            
#             # คำนวณ offset เพื่อให้ได้ค่า XY ที่ตรงกับเส้นทาง
#             self.utm_offset_x = self.TARGET_UTM_X - ref_utm_x
#             self.utm_offset_y = self.TARGET_UTM_Y - ref_utm_y
            
#             # เก็บ reference UTM สำหรับใช้งาน
#             self.reference_utm = (ref_utm_x, ref_utm_y)
            
#             self.get_logger().info(
#                 f'✅ Track coordinate system initialized:\n'
#                 f'  📍 Reference GPS: ({self.reference_point[0]:.6f}, {self.reference_point[1]:.6f})\n'
#                 f'  📍 Reference UTM: ({ref_utm_x:.2f}, {ref_utm_y:.2f})\n'
#                 f'  📍 Target Track: ({self.TARGET_UTM_X:.2f}, {self.TARGET_UTM_Y:.2f})\n'
#                 f'  📍 Offset: ({self.utm_offset_x:.2f}, {self.utm_offset_y:.2f})'
#             )
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Track coordinate system init failed: {e}')
#             # Fallback: ใช้ offset คงที่
#             self.utm_offset_x = 661470.0
#             self.utm_offset_y = 1509760.0
#             self.reference_utm = None
    
#     def configure_enhanced_zed_f9r(self):
#         """Enhanced ZED-F9R configuration for better NAV-ATT"""
#         try:
#             self.get_logger().info('🌟 Configuring ZED-F9R for enhanced NAV-ATT...')
#             time.sleep(2)
            
#             # Enable UBX messages with higher rates
#             self.enable_ubx_message(0x01, 0x05, 5)  # NAV-ATT at 5Hz
#             self.enable_ubx_message(0x10, 0x02, 2)  # ESF-MEAS at 2Hz
#             self.enable_ubx_message(0x10, 0x10, 1)  # ESF-STATUS
#             self.enable_ubx_message(0x10, 0x14, 1)  # ESF-ALG
            
#             # Enhanced IMU configuration for better accuracy
#             cfg_payload = struct.pack('<LLB', 0x20, 0x40270001, 1)  # High precision mode
#             cfg_cmd = self.create_ubx_message(0x06, 0x8A, cfg_payload)
#             self.ser.write(cfg_cmd)
#             time.sleep(0.1)
            
#             # Enable automatic IMU alignment with faster convergence
#             cfg_payload = struct.pack('<LLB', 0x20, 0x401A001A, 1)
#             cfg_cmd = self.create_ubx_message(0x06, 0x8A, cfg_payload)
#             self.ser.write(cfg_cmd)
#             time.sleep(0.1)
            
#             # Enable sensor fusion
#             cfg_payload = struct.pack('<LLB', 0x20, 0x40240001, 1)
#             cfg_cmd = self.create_ubx_message(0x06, 0x8A, cfg_payload)
#             self.ser.write(cfg_cmd)
#             time.sleep(0.1)
            
#             # Set higher IMU output rate for better heading
#             cfg_payload = struct.pack('<LLB', 0x20, 0x40250002, 10)  # 10Hz IMU
#             cfg_cmd = self.create_ubx_message(0x06, 0x8A, cfg_payload)
#             self.ser.write(cfg_cmd)
#             time.sleep(0.1)
            
#             # Save configuration
#             save_payload = struct.pack('<LLLL', 0x00000000, 0xFFFFFFFF, 0x00000000, 0x00000000)
#             save_cmd = self.create_ubx_message(0x06, 0x09, save_payload)
#             self.ser.write(save_cmd)
#             time.sleep(0.5)
            
#             self.get_logger().info('✅ Enhanced ZED-F9R configuration sent (5Hz NAV-ATT, 10Hz IMU)')
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Enhanced configuration failed: {e}')
    
#     def create_ubx_message(self, msg_class, msg_id, payload):
#         """สร้าง UBX message"""
#         header = struct.pack('<BBBB', 0xb5, 0x62, msg_class, msg_id)
#         length = struct.pack('<H', len(payload))
        
#         ck_a = ck_b = 0
#         for byte in struct.pack('<BB', msg_class, msg_id) + length + payload:
#             ck_a = (ck_a + byte) & 0xFF
#             ck_b = (ck_b + ck_a) & 0xFF
        
#         return header + length + payload + struct.pack('<BB', ck_a, ck_b)
    
#     def enable_ubx_message(self, msg_class, msg_id, rate=1):
#         """เปิดใช้ UBX message"""
#         payload = struct.pack('<BBB', msg_class, msg_id, rate)
#         cmd = self.create_ubx_message(0x06, 0x01, payload)
#         self.ser.write(cmd)
#         time.sleep(0.1)
    
#     def find_available_serial_ports(self):
#         """Find available serial ports"""
#         available_ports = []
#         for port in self.possible_serial_ports:
#             if os.path.exists(port):
#                 available_ports.append(port)
        
#         acm_ports = glob.glob('/dev/ttyACM*')
#         usb_ports = glob.glob('/dev/ttyUSB*')
        
#         for port in acm_ports + usb_ports:
#             if port not in available_ports:
#                 available_ports.append(port)
        
#         return sorted(available_ports)
    
#     def init_serial_connection(self):
#         """Initialize serial connection"""
#         self.get_logger().info("🔍 Searching for Enhanced ZED-F9R device...")
        
#         available_ports = self.find_available_serial_ports()
#         if not available_ports:
#             self.get_logger().error("❌ No serial ports found!")
#             return
        
#         baud_rates = [38400, 9600, 115200]
        
#         for port in available_ports:
#             for baud in baud_rates:
#                 try:
#                     test_ser = serial.Serial(port, baud, timeout=1)
#                     time.sleep(1)
                    
#                     if test_ser.in_waiting > 0 or self.test_serial_port_advanced(test_ser):
#                         self.ser = test_ser
#                         self.serial_port = port
#                         self.get_logger().info(f"✅ Enhanced connection to {port} at {baud} baud")
#                         return
                    
#                     test_ser.close()
#                 except:
#                     continue
        
#         # Last resort
#         if available_ports:
#             try:
#                 port = available_ports[0]
#                 self.ser = serial.Serial(port, 38400, timeout=1)
#                 self.serial_port = port
#                 self.get_logger().warning(f"⚠️ Connected to {port} without validation")
#             except Exception as e:
#                 self.get_logger().error(f"❌ Failed to connect: {e}")
    
#     def test_serial_port_advanced(self, test_ser):
#         """Advanced port testing"""
#         try:
#             start_time = time.time()
#             while time.time() - start_time < 2:
#                 if test_ser.in_waiting > 0:
#                     data = test_ser.read(test_ser.in_waiting)
#                     try:
#                         text = data.decode('ascii', errors='replace')
#                         if any(marker in text for marker in ['$G', 'UBX', '\xb5\x62']):
#                             return True
#                     except:
#                         pass
#                 time.sleep(0.1)
#             return False
#         except:
#             return False
    
#     def connect_to_ntrip(self):
#         """Connect to NTRIP server"""
#         try:
#             client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#             client_socket.connect((self.ntrip_server_ip, self.ntrip_server_port))
            
#             import base64
#             auth_str = f"{self.ntrip_username}:{self.ntrip_password}"
#             auth_b64 = base64.b64encode(auth_str.encode('ascii')).decode('ascii')
            
#             request = (
#                 f"GET /{self.mount_point} HTTP/1.1\r\n"
#                 f"User-Agent: NTRIP Client\r\n" 
#                 f"Authorization: Basic {auth_b64}\r\n\r\n"
#             )
#             client_socket.send(request.encode('ascii'))
#             self.get_logger().info("🌐 Connected to NTRIP server")
#             return client_socket
#         except Exception as e:
#             self.get_logger().warning(f"⚠️ NTRIP failed: {e}")
#             return None
    
#     def main_loop(self):
#         """Enhanced main processing loop"""
#         if not (self.ser and self.ser.is_open):
#             return
        
#         try:
#             # Read all available data
#             if self.ser.in_waiting > 0:
#                 data = self.ser.read(self.ser.in_waiting)
                
#                 # Process NMEA (text)
#                 try:
#                     text_data = data.decode('ascii', errors='replace')
#                     for line in text_data.split('\n'):
#                         line = line.strip()
#                         if line.startswith('$'):
#                             self.process_nmea_line(line)
#                 except:
#                     pass
                
#                 # Enhanced UBX processing
#                 if PYUBX2_AVAILABLE:
#                     self.process_ubx_data_enhanced(data)
#                 else:
#                     self.process_ubx_data_manual(data)
            
#             # Handle NTRIP
#             self.handle_ntrip_data()
            
#             # Update final heading with enhanced logic
#             self.update_enhanced_final_heading()
            
#             # ✅ Publish cmd_vel ทุกครั้งที่ main_loop ทำงาน (ใช้ vehicle parameters)
#             self.publish_cmd_vel_with_parameters()
            
#             # Publish debug every 2 seconds
#             if int(time.time() * 0.5) % 1 == 0:
#                 self.publish_enhanced_debug()
        
#         except Exception as e:
#             self.get_logger().error(f"❌ Enhanced main loop error: {e}")
    
#     def publish_cmd_vel_with_parameters(self):
#         """✅ Publish ข้อมูลไปยัง /cmd_vel โดยใช้ Vehicle Parameters"""
#         twist = Twist()
        
#         # ✅ ดึงค่า parameters ปัจจุบัน
#         params = self.get_vehicle_parameters()
        
#         # ตัวอย่างการควบคุมความเร็วโดยใช้ parameters
#         # คุณสามารถปรับ logic ตามต้องการ เช่น ใช้ fixed_speed หรือคำนวณจาก heading
        
#         # ใช้ fixed_speed จาก parameter
#         twist.linear.x = params['fixed_speed']  # ใช้ความเร็วจาก parameter
#         twist.linear.y = 0.0
#         twist.linear.z = 0.0
#         twist.angular.x = 0.0
#         twist.angular.y = 0.0
        
#         # คำนวณ angular velocity โดยจำกัดด้วย max_angular_velocity
#         desired_angular_vel = 0.2  # ตัวอย่าง (สามารถคำนวณจาก navigation logic)
#         twist.angular.z = max(-params['max_angular_velocity'], 
#                              min(params['max_angular_velocity'], desired_angular_vel))
        
#         # จำกัดความเร็วให้อยู่ในช่วงที่กำหนด
#         twist.linear.x = max(params['min_speed'], 
#                             min(params['max_speed'], twist.linear.x))
        
#         # Publish ข้อมูล
#         self.cmd_vel_publisher.publish(twist)
        
#         # Log ข้อมูลที่ส่ง พร้อม parameters
#         self.get_logger().info(
#             f"🚗 Published /cmd_vel: linear.x={twist.linear.x:.2f} m/s, angular.z={twist.angular.z:.2f} rad/s\n"
#             f"   📋 Using params: wheelbase={params['wheelbase']:.2f}m, max_steer={math.degrees(params['max_steering_angle']):.1f}°"
#         )
    
#     def process_ubx_data_enhanced(self, data):
#         """Enhanced UBX processing with pyubx2"""
#         self.ubx_buffer.extend(data)
        
#         while len(self.ubx_buffer) >= 8:
#             try:
#                 # Find UBX sync
#                 sync_pos = self.ubx_buffer.find(b'\xb5\x62')
#                 if sync_pos == -1:
#                     self.ubx_buffer.clear()
#                     break
                
#                 if sync_pos > 0:
#                     self.ubx_buffer = self.ubx_buffer[sync_pos:]
                
#                 if len(self.ubx_buffer) < 8:
#                     break
                
#                 # Try to parse with pyubx2
#                 try:
#                     # Get message length
#                     length = struct.unpack('<H', self.ubx_buffer[4:6])[0]
#                     total_len = 8 + length
                    
#                     if len(self.ubx_buffer) < total_len:
#                         break
                    
#                     # Extract complete message
#                     message_bytes = self.ubx_buffer[:total_len]
                    
#                     # Parse with pyubx2
#                     ubx_msg = UBXMessage.parse(message_bytes)
                    
#                     # Process specific messages
#                     if ubx_msg.identity == 'NAV-ATT':
#                         self.process_nav_att_enhanced(ubx_msg)
#                     elif ubx_msg.identity == 'ESF-STATUS':
#                         self.process_esf_status_enhanced(ubx_msg)
#                     elif ubx_msg.identity == 'ESF-MEAS':
#                         self.stats['esf_meas_count'] += 1
                    
#                     self.stats['pyubx2_success'] += 1
                    
#                     # Remove processed message
#                     self.ubx_buffer = self.ubx_buffer[total_len:]
                    
#                 except Exception as parse_error:
#                     # Fallback to manual parsing
#                     self.stats['pyubx2_errors'] += 1
#                     self.process_ubx_data_manual(message_bytes)
#                     self.ubx_buffer = self.ubx_buffer[1:]
                    
#             except Exception as e:
#                 self.ubx_buffer = self.ubx_buffer[1:]
    
#     def process_nav_att_enhanced(self, ubx_msg):
#         """Enhanced NAV-ATT processing with pyubx2"""
#         try:
#             # Extract attitude data
#             heading = ubx_msg.heading * 1e-5  # Convert to degrees
#             roll = ubx_msg.roll * 1e-5
#             pitch = ubx_msg.pitch * 1e-5
            
#             # Get accuracy information if available
#             head_acc = getattr(ubx_msg, 'headAcc', 0) * 1e-5 if hasattr(ubx_msg, 'headAcc') else None
            
#             # Validate heading
#             if 0 <= heading <= 360:
#                 current_time = time.time()
                
#                 # Enhanced filtering based on accuracy
#                 confidence_factor = 1.0
#                 if head_acc is not None and head_acc < 5.0:  # Good accuracy
#                     confidence_factor = 1.2
#                 elif head_acc is not None and head_acc > 15.0:  # Poor accuracy
#                     confidence_factor = 0.5
                
#                 # Apply enhanced heading filter
#                 filtered_heading = self.heading_filter.filter_nav_att_heading(
#                     heading, confidence_factor
#                 )
                
#                 if filtered_heading is not None:
#                     self.nav_att_heading = filtered_heading
#                     self.nav_att_yaw = heading  # Raw yaw for comparison
#                     self.nav_att_time = current_time
#                     self.stats['nav_att_count'] += 1
#                     self.stats['nav_att_yaw_count'] += 1
                    
#                     # Store enhanced attitude data
#                     self.nav_att = {
#                         'roll': roll,
#                         'pitch': pitch,
#                         'heading': heading,
#                         'filtered_heading': filtered_heading,
#                         'head_accuracy': head_acc,
#                         'confidence_factor': confidence_factor,
#                         'timestamp': current_time
#                     }
        
#         except Exception as e:
#             self.get_logger().debug(f"Enhanced NAV-ATT parse error: {e}")
    
#     def process_ubx_data_manual(self, data):
#         """Fallback manual UBX processing"""
#         self.ubx_buffer.extend(data)
        
#         while len(self.ubx_buffer) >= 8:
#             sync_pos = self.ubx_buffer.find(b'\xb5\x62')
#             if sync_pos == -1:
#                 self.ubx_buffer.clear()
#                 break
            
#             if sync_pos > 0:
#                 self.ubx_buffer = self.ubx_buffer[sync_pos:]
            
#             if len(self.ubx_buffer) < 8:
#                 break
            
#             try:
#                 cls = self.ubx_buffer[2]
#                 id = self.ubx_buffer[3] 
#                 length = struct.unpack('<H', self.ubx_buffer[4:6])[0]
                
#                 total_len = 8 + length
#                 if len(self.ubx_buffer) < total_len:
#                     break
                
#                 payload = self.ubx_buffer[6:6+length]
                
#                 if cls == 0x01 and id == 0x05:  # NAV-ATT
#                     self.process_nav_att_manual(payload)
#                 elif cls == 0x10 and id == 0x10:  # ESF-STATUS
#                     self.process_esf_status_manual(payload)
                
#                 self.ubx_buffer = self.ubx_buffer[total_len:]
                
#             except Exception as e:
#                 self.ubx_buffer = self.ubx_buffer[1:]
    
#     def process_nav_att_manual(self, payload):
#         """Manual NAV-ATT processing (fallback)"""
#         try:
#             if len(payload) >= 32:
#                 data = struct.unpack('<LLLLLLLLLHHH', payload[:32])
#                 iTOW, version, reserved1, roll, pitch, heading = data[:6]
                
#                 heading_deg = (heading * 1e-5) % 360
                
#                 if 0 <= heading_deg <= 360:
#                     filtered_heading = self.heading_filter.filter_nav_att_heading(heading_deg, 1.0)
                    
#                     if filtered_heading is not None:
#                         self.nav_att_heading = filtered_heading
#                         self.nav_att_time = time.time()
#                         self.stats['nav_att_count'] += 1
        
#         except Exception as e:
#             self.get_logger().debug(f"Manual NAV-ATT parse error: {e}")
    
#     def process_esf_status_enhanced(self, ubx_msg):
#         """Enhanced ESF-STATUS processing"""
#         try:
#             fusion_mode = ubx_msg.fusionMode
#             init_status1 = ubx_msg.initStatus1
            
#             # Update status
#             fusion_modes = {0: "INIT", 1: "FUSION", 2: "SUSPENDED", 3: "DISABLED"}
#             self.fusion_mode = fusion_modes.get(fusion_mode, f"UNKNOWN({fusion_mode})")
            
#             # Enhanced calibration status
#             if init_status1 & 0x01:
#                 if init_status1 & 0x02:
#                     self.imu_calibration_status = "FULLY_CALIBRATED"
#                 else:
#                     self.imu_calibration_status = "CALIBRATED"
#             else:
#                 self.imu_calibration_status = "CALIBRATING"
            
#             self.stats['esf_status_count'] += 1
            
#         except Exception as e:
#             self.get_logger().debug(f"Enhanced ESF-STATUS parse error: {e}")
    
#     def process_esf_status_manual(self, payload):
#         """Manual ESF-STATUS processing (fallback)"""
#         try:
#             if len(payload) >= 16:
#                 data = struct.unpack('<LLLLBBBB', payload[:16])
#                 fusionMode = data[5]
                
#                 fusion_modes = {0: "INIT", 1: "FUSION", 2: "SUSPENDED", 3: "DISABLED"}
#                 self.fusion_mode = fusion_modes.get(fusionMode, f"UNKNOWN({fusionMode})")
#                 self.stats['esf_status_count'] += 1
        
#         except Exception as e:
#             pass
    
#     def process_nmea_line(self, line):
#         """Process NMEA line (for position)"""
#         try:
#             if any(line.startswith(nmea) for nmea in ['$GPGGA', '$GNGGA', '$GPRMC', '$GNRMC', '$GPGLL', '$GNGLL']):
#                 msg = pynmea2.parse(line)
                
#                 if hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
#                     if msg.latitude and msg.longitude:
#                         self.process_position(msg.latitude, msg.longitude)
            
#         except Exception as e:
#             self.get_logger().debug(f"NMEA parse error: {e}")
    
#     def process_position(self, latitude, longitude):
#         """Process GNSS position with track coordinate system"""
#         current_time = time.time()
        
#         if not (-90 <= latitude <= 90) or not (-180 <= longitude <= 180):
#             return
        
#         # ✅ แปลงเป็น track coordinates
#         track_x, track_y = self.latlon_to_track_xy(latitude, longitude)
        
#         # ✅ ตรวจสอบว่าอยู่ในเส้นทางหรือไม่
#         track_match = self.validate_track_coordinates(track_x, track_y)
        
#         # Store position
#         position_record = {
#             'lat': latitude, 'lon': longitude,
#             'track_x': track_x, 'track_y': track_y,
#             'track_match': track_match,
#             'timestamp': current_time
#         }
#         self.position_history.append(position_record)
#         self.stats['position_count'] += 1
        
#         # Calculate GPS heading
#         self.update_gps_heading_improved()
        
#         # ✅ Publish track coordinates
#         self.publish_data(latitude, longitude, track_x, track_y)
    
#     def latlon_to_track_xy(self, lat, lon):
#         """✅ แปลง GPS เป็น Track Coordinates ที่ตรงกับเส้นทาง"""
#         try:
#             if self.reference_utm is None:
#                 return self.simple_latlon_to_xy_with_offset(lat, lon)
            
#             # แปลงเป็น UTM
#             utm_x, utm_y = self.transformer.transform(lon, lat)
            
#             # เพิ่ม offset เพื่อให้ตรงกับเส้นทาง
#             track_x = utm_x + self.utm_offset_x
#             track_y = utm_y + self.utm_offset_y
            
#             # ตรวจสอบว่าอยู่ในช่วงที่คาดหวัง
#             if (661300 <= track_x <= 661600) and (1509400 <= track_y <= 1509800):
#                 self.get_logger().debug(f"✅ Position in expected range: X={track_x:.2f}, Y={track_y:.2f}")
#             else:
#                 self.get_logger().warn(f"⚠️ Position outside expected range: X={track_x:.2f}, Y={track_y:.2f}")
            
#             return track_x, track_y
            
#         except Exception as e:
#             self.get_logger().error(f"❌ Track coordinate conversion failed: {e}")
#             return self.simple_latlon_to_xy_with_offset(lat, lon)
    
#     def simple_latlon_to_xy_with_offset(self, lat, lon):
#         """Fallback conversion with offset"""
#         dlat = lat - self.reference_point[0]
#         dlon = lon - self.reference_point[1]
        
#         x = dlon * 111319.9 * math.cos(math.radians(lat))
#         y = dlat * 111319.9
        
#         # เพิ่ม offset เพื่อให้ตรงกับเส้นทาง
#         track_x = x + 661470.0  # offset สำหรับ X
#         track_y = y + 1509760.0  # offset สำหรับ Y
        
#         return track_x, track_y
    
#     def validate_track_coordinates(self, track_x, track_y):
#         """ตรวจสอบว่าพิกัดอยู่ในช่วงเส้นทางหรือไม่"""
        
#         # กำหนดช่วงพิกัดของเส้นทาง
#         track_ranges = {
#             'SN1': {'x_min': 661484, 'x_max': 661490, 'y_min': 1509522, 'y_max': 1509584},
#             'SN2': {'x_min': 661484, 'x_max': 661490, 'y_min': 1509584, 'y_max': 1509685},
#             'EW': {'x_min': 661433, 'x_max': 661478, 'y_min': 1509696, 'y_max': 1509704},
#             'NS': {'x_min': 661375, 'x_max': 661383, 'y_min': 1509560, 'y_max': 1509654},
#             'WE': {'x_min': 661437, 'x_max': 661466, 'y_min': 1509506, 'y_max': 1509513}
#         }
        
#         # ตรวจสอบว่าอยู่ในเส้นทางไหน
#         for track_name, ranges in track_ranges.items():
#             if (ranges['x_min'] <= track_x <= ranges['x_max'] and 
#                 ranges['y_min'] <= track_y <= ranges['y_max']):
#                 self.get_logger().info(f"🎯 Position matches {track_name} track")
#                 return track_name
        
#         # ตรวจสอบว่าอยู่ใกล้เส้นทางหรือไม่ (ภายใน 10 เมตร)
#         for track_name, ranges in track_ranges.items():
#             if (ranges['x_min']-10 <= track_x <= ranges['x_max']+10 and 
#                 ranges['y_min']-10 <= track_y <= ranges['y_max']+10):
#                 self.get_logger().warn(f"⚠️ Position near {track_name} track (within 10m)")
#                 return f"near_{track_name}"
        
#         self.get_logger().warn(f"❌ Position outside all tracks: X={track_x:.2f}, Y={track_y:.2f}")
#         return None
    
#     def update_gps_heading_improved(self):
#         """Enhanced GPS heading calculation"""
#         if len(self.position_history) < 3:
#             return
        
#         total_distance = 0.0
#         weighted_x = weighted_y = 0.0
        
#         recent_positions = list(self.position_history)[-5:]
        
#         for i in range(len(recent_positions) - 1):
#             curr = recent_positions[i+1]
#             prev = recent_positions[i]
            
#             dx = curr['track_x'] - prev['track_x']  # ใช้ track coordinates
#             dy = curr['track_y'] - prev['track_y']
#             distance = math.sqrt(dx*dx + dy*dy)
            
#             if distance > self.min_movement_for_heading:
#                 heading_rad = math.atan2(dx, dy)
                
#                 time_weight = (i + 1) / len(recent_positions)
#                 weight = distance * time_weight
                
#                 weighted_x += math.cos(heading_rad) * weight
#                 weighted_y += math.sin(heading_rad) * weight
#                 total_distance += weight
        
#         if total_distance > self.min_movement_for_heading:
#             avg_heading = math.degrees(math.atan2(weighted_y, weighted_x))
#             if avg_heading < 0:
#                 avg_heading += 360
            
#             filtered_gps_heading = self.heading_filter.filter_gps_heading(avg_heading)
            
#             if filtered_gps_heading is not None:
#                 self.gps_heading = filtered_gps_heading
#                 self.gps_heading_time = time.time()
#                 self.stats['gps_count'] += 1
    
#     def update_enhanced_final_heading(self):
#         """Enhanced final heading selection with NAV-ATT priority"""
#         current_time = time.time()
        
#         # Check data validity
#         nav_att_valid = (self.nav_att_heading is not None and 
#                         (current_time - self.nav_att_time) < self.heading_timeout and
#                         self.fusion_mode in ["FUSION", "INIT"])
        
#         gps_valid = (self.gps_heading is not None and 
#                     (current_time - self.gps_heading_time) < self.heading_timeout)
        
#         # Enhanced priority: NAV-ATT > GPS > Hold
#         if nav_att_valid and self.imu_calibration_status in ["FULLY_CALIBRATED", "CALIBRATED"]:
#             # Highest priority: Use enhanced NAV-ATT when calibrated
#             self.final_heading = self.nav_att_heading
#             self.heading_source = f"NAV_ATT_{self.imu_calibration_status}"
            
#         elif nav_att_valid and self.imu_calibration_status == "CALIBRATING":
#             # Blend NAV-ATT and GPS during calibration
#             if gps_valid:
#                 self.final_heading = self.blend_headings(self.nav_att_heading, self.gps_heading, 0.8)
#                 self.heading_source = "NAV_ATT_GPS_BLEND"
#             else:
#                 self.final_heading = self.nav_att_heading
#                 self.heading_source = "NAV_ATT_CALIBRATING"
                
#         elif gps_valid:
#             # Fallback to GPS
#             self.final_heading = self.gps_heading  
#             self.heading_source = "GPS"
            
#         else:
#             # No valid heading
#             self.final_heading = None
#             self.heading_source = "NONE"
        
#         # Publish enhanced heading
#         if self.final_heading is not None:
#             heading_msg = Float32()
#             heading_msg.data = float(self.final_heading)
#             self.heading_publisher.publish(heading_msg)
#             self.stats['filtered_count'] += 1
    
#     def blend_headings(self, heading1, heading2, weight1):
#         """Blend two headings considering circular nature"""
#         if heading1 is None or heading2 is None:
#             return heading1 or heading2
        
#         x1, y1 = math.cos(math.radians(heading1)), math.sin(math.radians(heading1))
#         x2, y2 = math.cos(math.radians(heading2)), math.sin(math.radians(heading2))
        
#         weight2 = 1.0 - weight1
#         x_blend = weight1 * x1 + weight2 * x2
#         y_blend = weight1 * y1 + weight2 * y2
        
#         blended_heading = math.degrees(math.atan2(y_blend, x_blend))
#         return blended_heading % 360
    
#     def publish_data(self, latitude, longitude, track_x, track_y):
#         """✅ Publish ข้อมูลในรูปแบบที่ Navigation Controller ต้องการ"""
#         current_time = time.time()
        
#         # 1. GNSS NavSatFix (เก็บไว้เดิม)
#         gnss_msg = NavSatFix()
#         gnss_msg.header.stamp = self.get_clock().now().to_msg()
#         gnss_msg.header.frame_id = "gps"
#         gnss_msg.latitude = latitude
#         gnss_msg.longitude = longitude
#         gnss_msg.altitude = 0.0
#         gnss_msg.status.status = 0
#         gnss_msg.status.service = 1
#         self.gnss_publisher.publish(gnss_msg)
        
#         # 2. ✅ XY Array สำหรับ Navigation Controller (ค่า XY ที่ตรงกับเส้นทาง)
#         xy_array_msg = Float64MultiArray()
#         xy_array_msg.data = [current_time, track_x, track_y]
#         self.xy_publisher.publish(xy_array_msg)
        
#         # 3. Raw GPS Array
#         raw_gps_msg = Float64MultiArray()
#         raw_gps_msg.data = [current_time, latitude, longitude]
#         self.raw_gps_publisher.publish(raw_gps_msg)
        
#         # 4. XY Point สำหรับ debug
#         xy_point_msg = Point()
#         xy_point_msg.x = track_x
#         xy_point_msg.y = track_y
#         xy_point_msg.z = 0.0
#         self.xy_debug_publisher.publish(xy_point_msg)
        
#         # ✅ Log ที่แสดงค่าที่ถูกต้อง
#         self.get_logger().info(
#             f"📍 Published Track Coordinates: X={track_x:.2f}, Y={track_y:.2f} "
#             f"(GPS: {latitude:.6f}, {longitude:.6f})"
#         )
    
#     def publish_enhanced_debug(self):
#         """Enhanced debug information with NAV-ATT details and Vehicle Parameters"""
#         # ✅ เพิ่ม vehicle parameters ใน debug data
#         vehicle_params = self.get_vehicle_parameters()
        
#         debug_data = {
#             'final_heading': {
#                 'value': self.final_heading,
#                 'source': self.heading_source,
#                 'is_valid': self.final_heading is not None
#             },
#             'sources': {
#                 'NAV_ATT': {
#                     'raw_yaw': self.nav_att_yaw,
#                     'filtered': self.nav_att_heading,
#                     'age': time.time() - self.nav_att_time if self.nav_att_time > 0 else float('inf'),
#                     'accuracy': self.nav_att.get('head_accuracy') if self.nav_att else None,
#                     'confidence': self.nav_att.get('confidence_factor') if self.nav_att else None
#                 },
#                 'GPS': {
#                     'filtered': self.gps_heading,
#                     'age': time.time() - self.gps_heading_time if self.gps_heading_time > 0 else float('inf'),
#                     'movement_points': len(self.position_history)
#                 }
#             },
#             'zed_f9r_status': {
#                 'fusion_mode': self.fusion_mode,
#                 'calibration_status': self.imu_calibration_status,
#                 'nav_att_data': self.nav_att
#             },
#             'coordinate_system': {
#                 'type': 'Track_Coordinate_System',
#                 'reference_lat': self.reference_point[0],
#                 'reference_lon': self.reference_point[1],
#                 'target_utm_x': self.TARGET_UTM_X,
#                 'target_utm_y': self.TARGET_UTM_Y,
#                 'offset_x': self.utm_offset_x,
#                 'offset_y': self.utm_offset_y
#             },
#             # ✅ เพิ่ม vehicle parameters ใน debug
#             'vehicle_parameters': vehicle_params,
#             'enhanced_stats': {
#                 'pyubx2_available': PYUBX2_AVAILABLE,
#                 'pyubx2_success': self.stats['pyubx2_success'],
#                 'pyubx2_errors': self.stats['pyubx2_errors'],
#                 'nav_att_count': self.stats['nav_att_count'],
#                 'nav_att_yaw_count': self.stats['nav_att_yaw_count']
#             },
#             'filter_stats': self.heading_filter.get_stats(),
#             'stats': self.stats,
#             'port': self.serial_port
#         }
        
#         msg = String()
#         msg.data = json.dumps(debug_data, default=str)
#         self.heading_debug_publisher.publish(msg)
    
#     def handle_ntrip_data(self):
#         """Handle NTRIP corrections"""
#         if self.ntrip_socket:
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
    
#     def destroy_node(self):
#         """Cleanup"""
#         self.get_logger().info("🔄 Shutting down Enhanced ZED-F9R...")
        
#         if self.ser and self.ser.is_open:
#             self.ser.close()
        
#         if hasattr(self, 'ntrip_socket') and self.ntrip_socket:
#             self.ntrip_socket.close()
        
#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     gnss_publisher = EnhancedZEDf9rGNSSWithHeading()
    
#     try:
#         rclpy.spin(gnss_publisher)
#     except KeyboardInterrupt:
#         gnss_publisher.get_logger().info("🛑 Keyboard interrupt")
#     finally:
#         gnss_publisher.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()


# รอเทส----------------------------------------------------------------

# import serial
# import pynmea2
# import socket
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
# from std_msgs.msg import Float32, String
# from geometry_msgs.msg import Point, Twist
# import math
# import pyproj
# from collections import deque
# import time
# import glob
# import os
# import struct
# import json
# import numpy as np
# import pandas as pd

# # 🔧 เพิ่ม pyubx2 สำหรับ UBX parsing ที่ดีขึ้น
# try:
#     from pyubx2 import UBXReader, UBXMessage, UBX_MSGIDS
#     PYUBX2_AVAILABLE = True
#     print("✅ pyubx2 available - using enhanced UBX parsing")
# except ImportError:
#     PYUBX2_AVAILABLE = False
#     print("⚠️ pyubx2 not available - install with: pip3 install pyubx2")

# class BangBangWaypointController:
#     def __init__(self, waypoints, final_heading=0.0, waypoint_tolerance=1.5, heading_tolerance=np.deg2rad(8)):
#         """
#         🎯 Enhanced Bang-Bang Controller สำหรับรถของคุณ
        
#         Parameters:
#         - waypoints: list of (x, y) coordinates in UTM absolute coordinates
#         - final_heading: ทิศทางสุดท้ายที่ต้องการ (degrees, compass bearing)
#         - waypoint_tolerance: ระยะทางที่ถือว่าถึง waypoint แล้ว (meters)
#         - heading_tolerance: ความคลาดเคลื่อนทิศทางที่ยอมรับได้ (radians)
#         """
#         self.waypoints = waypoints
#         self.final_heading = final_heading
#         self.waypoint_tolerance = waypoint_tolerance
#         self.heading_tolerance = heading_tolerance
#         self.current_target_idx = 0
        
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
#             'straight_commands': 0
#         }
        
#         print(f"🚗 Vehicle Configuration:")
#         print(f"   Wheelbase: {self.wheelbase} m")
#         print(f"   Max Steering: ±{np.rad2deg(self.max_steering_angle):.1f}° (±{self.max_steering_angle:.3f} rad)")
#         print(f"   Max Angular Velocity: ±{self.max_angular_velocity:.3f} rad/s")
#         print(f"   Fixed Speed: {self.fixed_speed} m/s")
#         print(f"   Total Waypoints: {len(self.waypoints)}")
        
#         # แสดง waypoint แรกและสุดท้าย
#         if self.waypoints:
#             print(f"   First waypoint: ({self.waypoints[0][0]:.2f}, {self.waypoints[0][1]:.2f})")
#             print(f"   Last waypoint: ({self.waypoints[-1][0]:.2f}, {self.waypoints[-1][1]:.2f})")
    
#     def find_closest_waypoint(self, current_pos):
#         """หาจุด waypoint ที่ใกล้ที่สุดกับตำแหน่งปัจจุบัน"""
#         distances = []
#         for i, wp in enumerate(self.waypoints):
#             dist = np.hypot(wp[0] - current_pos[0], wp[1] - current_pos[1])
#             distances.append((dist, i))
        
#         distances.sort()
#         return distances[0][1]  # return index of closest waypoint
    
#     def get_target_heading(self, current_pos, target_waypoint_idx):
#         """🔧 คำนวณทิศทางที่ต้องการไปยัง waypoint เป้าหมาย (Compass Bearing)"""
#         if target_waypoint_idx >= len(self.waypoints):
#             return self.final_heading
            
#         target_wp = self.waypoints[target_waypoint_idx]
#         dx = target_wp[0] - current_pos[0]  # East (UTM X)
#         dy = target_wp[1] - current_pos[1]  # North (UTM Y)
        
#         # 🔧 ใช้ compass bearing: arctan2(East, North)
#         bearing_rad = np.arctan2(dx, dy)
        
#         # แปลงเป็นองศาและปรับให้เป็นค่าบวก (0-360°)
#         bearing_deg = np.rad2deg(bearing_rad)
#         if bearing_deg < 0:
#             bearing_deg += 360
        
#         # ถ้าใกล้ waypoint สุดท้ายมาก ให้ใช้ final_heading
#         if target_waypoint_idx == len(self.waypoints) - 1:
#             distance_to_target = np.hypot(dx, dy)
#             if distance_to_target < self.waypoint_tolerance:
#                 return self.final_heading
        
#         return bearing_deg  # คืนค่าเป็นองศา (compass bearing)
    
#     def normalize_angle_deg(self, angle_deg):
#         """ปรับมุมให้อยู่ในช่วง [-180, 180] องศา"""
#         while angle_deg > 180:
#             angle_deg -= 360
#         while angle_deg < -180:
#             angle_deg += 360
#         return angle_deg
    
#     def update_target_waypoint(self, current_pos):
#         """อัพเดท waypoint เป้าหมายถ้าถึงจุดปัจจุบันแล้ว"""
#         if self.current_target_idx < len(self.waypoints):
#             current_wp = self.waypoints[self.current_target_idx]
#             distance = np.hypot(current_wp[0] - current_pos[0], current_wp[1] - current_pos[1])
            
#             if distance < self.waypoint_tolerance and self.current_target_idx < len(self.waypoints) - 1:
#                 self.current_target_idx += 1
#                 self.stats['waypoints_reached'] += 1
#                 return True
#         return False
    
#     def get_control_command(self, current_pos, current_heading_deg):
#         """
#         🎯 คำนวณคำสั่งควบคุมแบบ Bang-Bang ที่ปรับปรุงแล้ว
        
#         Parameters:
#         - current_pos: ตำแหน่งปัจจุบัน (UTM)
#         - current_heading_deg: ทิศทางปัจจุบัน (องศา, compass bearing)
        
#         Returns:
#         - cmd_vel: Twist message with linear and angular velocity
#         - status: dictionary with current status information
#         """
#         # อัพเดท waypoint เป้าหมาย
#         waypoint_changed = self.update_target_waypoint(current_pos)
        
#         # หาทิศทางที่ต้องการ (compass bearing)
#         target_heading_deg = self.get_target_heading(current_pos, self.current_target_idx)
        
#         # 🔧 คำนวณความผิดพลาดของทิศทาง (compass bearing)
#         heading_error_deg = target_heading_deg - current_heading_deg
#         heading_error_deg = self.normalize_angle_deg(heading_error_deg)
        
#         # แปลงเป็น radians สำหรับการคำนวณ
#         heading_error_rad = np.deg2rad(heading_error_deg)
        
#         # คำนวณระยะทางถึงเป้าหมาย
#         if self.current_target_idx < len(self.waypoints):
#             target_wp = self.waypoints[self.current_target_idx]
#             distance_to_target = np.hypot(target_wp[0] - current_pos[0], target_wp[1] - current_pos[1])
#         else:
#             distance_to_target = 0.0
        
#         # สร้าง Twist message
#         cmd_vel = Twist()
        
#         # 🌟 Enhanced Bang-Bang Control Logic สำหรับรถของคุณ
#         if abs(heading_error_rad) <= self.heading_tolerance:
#             # ไปตรง
#             angular_velocity = 0.0
#             steering_angle = 0.0
#             steering_command = "STRAIGHT"
#             self.stats['straight_commands'] += 1
#         elif heading_error_deg > 0:
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
#         if distance_to_target > 0.1:  # ยังไม่ถึงเป้าหมาย
#             cmd_vel.linear.x = self.fixed_speed
#         else:
#             cmd_vel.linear.x = 0.0  # หยุดเมื่อถึงเป้าหมายสุดท้าย
        
#         cmd_vel.angular.z = angular_velocity
        
#         # สร้างข้อมูลสถานะ
#         status = {
#             'current_waypoint': self.current_target_idx,
#             'total_waypoints': len(self.waypoints),
#             'target_heading_deg': target_heading_deg,
#             'current_heading_deg': current_heading_deg,
#             'heading_error_deg': heading_error_deg,
#             'distance_to_target': distance_to_target,
#             'steering_command': steering_command,
#             'linear_velocity': cmd_vel.linear.x,
#             'angular_velocity': cmd_vel.angular.z,
#             'waypoint_changed': waypoint_changed,
#             'mission_progress': (self.current_target_idx / len(self.waypoints)) * 100,
#             'stats': self.stats
#         }
        
#         return cmd_vel, status
    
#     def is_mission_complete(self, current_pos, current_heading_deg):
#         """ตรวจสอบว่าภารกิจเสร็จสิ้นแล้วหรือไม่"""
#         if self.current_target_idx >= len(self.waypoints) - 1:
#             last_wp = self.waypoints[-1]
#             distance = np.hypot(last_wp[0] - current_pos[0], last_wp[1] - current_pos[1])
#             heading_error = abs(self.normalize_angle_deg(self.final_heading - current_heading_deg))
            
#             return (distance < self.waypoint_tolerance and 
#                    heading_error < np.rad2deg(self.heading_tolerance))
#         return False

# class EnhancedZEDf9rGNSSWithHeading(Node):
#     def __init__(self):
#         super().__init__('enhanced_zedf9r_gnss_with_heading')
        
#         # ROS2 Publishers
#         self.gnss_publisher = self.create_publisher(NavSatFix, '/navigation/gnss', 10)
#         self.xy_publisher = self.create_publisher(Point, '/navigation/xy_position', 10)
#         self.heading_publisher = self.create_publisher(Float32, '/navigation/heading', 10)
#         self.heading_debug_publisher = self.create_publisher(String, '/navigation/heading_debug', 10)
        
#         # 🌟 NEW: Controller publishers
#         self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.controller_status_publisher = self.create_publisher(String, '/controller/status', 10)
        
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
        
#         # 🔧 Fixed Coordinate transformation
#         self.transformer = pyproj.Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)
        
#         # Position tracking
#         self.position_history = deque(maxlen=15)
#         self.min_movement_for_heading = 0.25
        
#         # 🔧 Fixed reference point - ใช้เป็นจุดอ้างอิงเท่านั้น
#         self.reference_point = (13.650748, 100.492985)
#         self.reference_utm = None
        
#         # 🔧 Enhanced Heading management สำหรับ ZED-F9R (compass bearing)
#         self.imu_heading = None
#         self.nav_att_heading = None
#         self.nav_att_yaw = None
#         self.gps_heading = None
#         self.final_heading = None  # compass bearing (degrees)
#         self.heading_source = "NONE"
        
#         self.imu_heading_time = 0
#         self.nav_att_time = 0
#         self.gps_heading_time = 0
#         self.heading_timeout = 8.0
        
#         # ZED-F9R specific data
#         self.esf_status = None
#         self.nav_att = None
#         self.esf_alg = None
#         self.fusion_mode = "NONE"
#         self.imu_calibration_status = "UNKNOWN"
        
#         # 🌟 NEW: Controller initialization
#         self.controller = None
#         # 🔧 เปลี่ยนเป็น UTM สัมบูรณ์แทน local coordinates
#         self.current_utm_x = None
#         self.current_utm_y = None
#         self.controller_active = False
        
#         # 🌟 NEW: pyubx2 UBX parsing
#         self.ubx_reader = None
#         self.ubx_buffer = bytearray()
        
#         # 🔧 Enhanced Heading filtering
#         self.heading_filter = EnhancedIMUHeadingFilter()
        
#         # Performance stats
#         self.stats = {
#             'imu_count': 0,
#             'nav_att_count': 0,
#             'nav_att_yaw_count': 0,
#             'gps_count': 0,
#             'position_count': 0,
#             'esf_meas_count': 0,
#             'esf_status_count': 0,
#             'pyubx2_success': 0,
#             'pyubx2_errors': 0,
#             'filtered_count': 0
#         }
        
#         # 🔧 Initialize coordinate system
#         self.init_fixed_coordinate_system()
        
#         # 🌟 Initialize waypoint controller (หน่วงเวลา)
#         self.controller_init_delay = 5.0  # รอ 5 วินาที
#         self.controller_init_time = time.time()
        
#         # Initialize
#         self.init_serial_connection()
        
#         if self.ser and self.ser.is_open:
#             self.ntrip_socket = self.connect_to_ntrip()
            
#             # 🌟 Initialize pyubx2 if available
#             if PYUBX2_AVAILABLE:
#                 self.init_pyubx2()
            
#             # Configure ZED-F9R for enhanced IMU
#             self.configure_enhanced_zed_f9r()
            
#             # Main processing timer
#             self.create_timer(0.1, self.main_loop)
            
#             self.get_logger().info('✅ Enhanced ZED-F9R GNSS with Waypoint Controller initialized')
#             self.get_logger().info(f'🔧 pyubx2: {"Available" if PYUBX2_AVAILABLE else "Not available"}')
#         else:
#             self.get_logger().error('❌ Failed to initialize')
    
#     def find_closest_waypoint_index(self, current_pos, waypoints):
#         """🔧 หา index ของ waypoint ที่ใกล้ที่สุด"""
#         min_distance = float('inf')
#         closest_idx = 0
        
#         for i, wp in enumerate(waypoints):
#             distance = np.hypot(wp[0] - current_pos[0], wp[1] - current_pos[1])
#             if distance < min_distance:
#                 min_distance = distance
#                 closest_idx = i
        
#         return closest_idx
    
#     def init_waypoint_controller(self):
#         """🌟 Initialize waypoint controller with UTM absolute coordinates"""
#         try:
#             # ตรวจสอบตำแหน่งปัจจุบัน
#             if self.current_utm_x is None or self.current_utm_y is None:
#                 self.get_logger().warning('⚠️ Waiting for GPS position before initializing controller...')
#                 return
            
#             # แสดงตำแหน่งปัจจุบันในรูปแบบ UTM
#             self.get_logger().info(f'📍 Current UTM position: ({self.current_utm_x:.2f}, {self.current_utm_y:.2f})')
            
#             # โหลด waypoints จากไฟล์
#             waypoints = self.load_waypoints_from_csv('/home/inc/ros2_ws/waypointfile/waypoints3xy.csv')
            
#             if waypoints:
#                 # 🔧 หา waypoint ที่ใกล้ที่สุดเป็นจุดเริ่มต้น
#                 current_pos = (self.current_utm_x, self.current_utm_y)
#                 closest_idx = self.find_closest_waypoint_index(current_pos, waypoints)
                
#                 # ตรวจสอบระยะทางถึง waypoint ที่ใกล้ที่สุด
#                 closest_wp = waypoints[closest_idx]
#                 distance_to_closest = np.hypot(
#                     closest_wp[0] - self.current_utm_x, 
#                     closest_wp[1] - self.current_utm_y
#                 )
                
#                 self.get_logger().info(f'📍 Closest waypoint: {closest_idx} at ({closest_wp[0]:.2f}, {closest_wp[1]:.2f})')
#                 self.get_logger().info(f'📏 Distance to closest waypoint: {distance_to_closest:.2f}m')
                
#                 # ถ้าระยะทางมากเกินไป ให้แจ้งเตือน
#                 if distance_to_closest > 1000:
#                     self.get_logger().warning(f'⚠️ Distance to closest waypoint is large: {distance_to_closest:.2f}m')
#                     self.get_logger().warning('⚠️ Please verify waypoint coordinates')
                
#                 self.controller = BangBangWaypointController(
#                     waypoints=waypoints,
#                     final_heading=0.0,  # หันหน้าไปทางเหนือ (compass bearing)
#                     waypoint_tolerance=2.0,       # ยอมรับความผิดพลาด 2 เมตร
#                     heading_tolerance=np.deg2rad(5)  # ยอมรับความผิดพลาด 5 องศา
#                 )
                
#                 # 🔧 เริ่มจาก waypoint ที่ใกล้ที่สุด
#                 self.controller.current_target_idx = closest_idx
                
#                 self.get_logger().info(f'✅ Controller initialized, starting from waypoint {closest_idx}')
#                 self.controller_active = True
                
#             else:
#                 self.get_logger().error('❌ Failed to load waypoints')
                
#         except Exception as e:
#             self.get_logger().error(f'❌ Controller initialization failed: {e}')
    
#     def load_waypoints_from_csv(self, filename):
#         """โหลด waypoints จากไฟล์ CSV ในรูปแบบ UTM absolute coordinates"""
#         try:
#             df = pd.read_csv(filename)
            
#             # ตรวจสอบคอลัมน์ที่มีอยู่
#             if 'x_east' in df.columns and 'y_north' in df.columns:
#                 # ใช้ UTM coordinates โดยตรง
#                 waypoints = list(zip(df['x_east'], df['y_north']))
#                 self.get_logger().info(f'📍 Loaded {len(waypoints)} UTM waypoints from {filename}')
#                 self.get_logger().info(f'📍 Waypoint range: X({df["x_east"].min():.1f} to {df["x_east"].max():.1f}), Y({df["y_north"].min():.1f} to {df["y_north"].max():.1f})')
                
#             elif 'lat' in df.columns and 'lon' in df.columns:
#                 # แปลงจาก lat/lon เป็น UTM
#                 waypoints = []
#                 for _, row in df.iterrows():
#                     utm_x, utm_y = self.transformer.transform(row['lon'], row['lat'])
#                     waypoints.append((utm_x, utm_y))
#                 self.get_logger().info(f'📍 Converted {len(waypoints)} lat/lon waypoints to UTM from {filename}')
                
#             else:
#                 self.get_logger().error(f'❌ Invalid waypoint file format. Expected columns: x_east,y_north or lat,lon')
#                 return []
            
#             return waypoints
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Failed to load waypoints: {e}')
#             return []
    
#     def run_controller(self):
#         """🌟 รันตัวควบคุม waypoint ด้วย UTM coordinates และ compass bearing"""
#         if not (self.controller_active and self.controller and 
#                 self.current_utm_x is not None and self.current_utm_y is not None and
#                 self.final_heading is not None):
#             return
        
#         # 🔧 เพิ่มการตรวจสอบ heading source
#         if self.heading_source == "NONE":
#             self.get_logger().warning('⚠️ No valid heading available, controller paused')
#             return
        
#         try:
#             # ใช้ UTM coordinates โดยตรง
#             current_pos = (self.current_utm_x, self.current_utm_y)
#             current_heading_deg = self.final_heading  # compass bearing (degrees)
            
#             # ตรวจสอบความสมเหตุสมผลของตำแหน่ง UTM
#             if (self.current_utm_x < 100000 or self.current_utm_x > 1000000 or 
#                 self.current_utm_y < 1000000 or self.current_utm_y > 2000000):
#                 self.get_logger().warning(f'⚠️ UTM position may be invalid: ({self.current_utm_x:.2f}, {self.current_utm_y:.2f})')
#                 return
            
#             # รับคำสั่งควบคุมจาก controller
#             cmd_vel, status = self.controller.get_control_command(current_pos, current_heading_deg)
            
#             # ตรวจสอบระยะทางที่สมเหตุสมผล
#             if status['distance_to_target'] > 50000:  # มากกว่า 50 กิโลเมตร
#                 self.get_logger().error(f'❌ Unrealistic distance: {status["distance_to_target"]:.2f}m')
#                 return
            
#             # ส่งคำสั่งความเร็ว
#             self.cmd_vel_publisher.publish(cmd_vel)
            
#             # แสดงสถานะ
#             self.publish_controller_status(status)
            
#             # Echo ข้อมูลสำคัญ
#             self.echo_controller_status(status)
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Controller error: {e}')
    
#     def publish_controller_status(self, status):
#         """ส่งสถานะของ controller"""
#         try:
#             status_msg = String()
#             status_msg.data = json.dumps(status, default=str)
#             self.controller_status_publisher.publish(status_msg)
#         except Exception as e:
#             self.get_logger().debug(f'Status publish error: {e}')
    
#     def echo_controller_status(self, status):
#         """🌟 Echo สถานะปัจจุบันของ controller พร้อม compass bearing"""
#         try:
#             # สร้างข้อความสถานะ
#             waypoint_info = f"Waypoint: {status['current_waypoint']}/{status['total_waypoints']}"
#             direction_info = f"Direction: {status['target_heading_deg']:.1f}°"
#             steering_info = f"Steering: {status['steering_command']}"
#             speed_info = f"Speed: {status['linear_velocity']:.1f} m/s"
#             distance_info = f"Distance: {status['distance_to_target']:.1f}m"
#             progress_info = f"Progress: {status['mission_progress']:.1f}%"
            
#             # 🔧 เพิ่มข้อมูล heading error และ source
#             heading_error_info = f"HeadErr: {status['heading_error_deg']:.1f}°"
#             heading_info = f"Heading: {self.final_heading:.1f}° ({self.heading_source})"
#             utm_info = f"UTM: ({self.current_utm_x:.1f}, {self.current_utm_y:.1f})"
            
#             # Log ข้อมูลหลัก
#             self.get_logger().info(
#                 f"🎯 {waypoint_info} | {direction_info} | {steering_info} | "
#                 f"{speed_info} | {distance_info} | {progress_info} | {heading_error_info}"
#             )
            
#             # Log ข้อมูลเพิ่มเติม
#             self.get_logger().info(f"📍 {utm_info} | {heading_info}")
            
#             # Log เพิ่มเติมเมื่อเปลี่ยน waypoint
#             if status['waypoint_changed']:
#                 self.get_logger().info(
#                     f"✅ Reached waypoint {status['current_waypoint']-1}! "
#                     f"Moving to waypoint {status['current_waypoint']}"
#                 )
            
#             # 🔧 แสดงข้อมูลเพิ่มเติมเมื่อ heading error ใหญ่
#             if abs(status['heading_error_deg']) > 15:
#                 self.get_logger().warning(f'⚠️ Large heading error: {status["heading_error_deg"]:.1f}°')
            
#             # ตรวจสอบการเสร็จสิ้นภารกิจ
#             if self.controller.is_mission_complete(
#                 (self.current_utm_x, self.current_utm_y), 
#                 self.final_heading
#             ):
#                 self.get_logger().info("🏁 Mission Complete! All waypoints reached.")
                
#         except Exception as e:
#             self.get_logger().debug(f'Echo error: {e}')
    
#     def init_pyubx2(self):
#         """🌟 Initialize pyubx2 for enhanced UBX parsing"""
#         try:
#             if PYUBX2_AVAILABLE:
#                 self.get_logger().info('🌟 pyubx2 UBX parser initialized')
#             else:
#                 self.get_logger().warning('⚠️ pyubx2 not available, using manual parsing')
#         except Exception as e:
#             self.get_logger().error(f'❌ pyubx2 initialization failed: {e}')
    
#     def init_fixed_coordinate_system(self):
#         """🔧 Initialize fixed coordinate system"""
#         try:
#             self.reference_utm = self.transformer.transform(
#                 self.reference_point[1], self.reference_point[0]
#             )
#             self.get_logger().info(
#                 f'🔧 Coordinate system initialized:\n'
#                 f'  📍 Reference (lat,lon): ({self.reference_point[0]:.6f}, {self.reference_point[1]:.6f})\n'
#                 f'  📍 Reference UTM: ({self.reference_utm[0]:.2f}, {self.reference_utm[1]:.2f})'
#             )
#         except Exception as e:
#             self.get_logger().error(f'❌ Coordinate system init failed: {e}')
#             self.reference_utm = None
    
#     def configure_enhanced_zed_f9r(self):
#         """🌟 Enhanced ZED-F9R configuration for better NAV-ATT"""
#         try:
#             self.get_logger().info('🌟 Configuring ZED-F9R for enhanced NAV-ATT...')
#             time.sleep(2)
            
#             # Enable UBX messages with higher rates
#             self.enable_ubx_message(0x01, 0x05, 5)  # 🌟 NAV-ATT at 5Hz
#             self.enable_ubx_message(0x10, 0x02, 2)  # ESF-MEAS at 2Hz
#             self.enable_ubx_message(0x10, 0x10, 1)  # ESF-STATUS
#             self.enable_ubx_message(0x10, 0x14, 1)  # ESF-ALG
            
#             self.get_logger().info('✅ Enhanced ZED-F9R configuration sent')
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Enhanced configuration failed: {e}')
    
#     def enable_ubx_message(self, msg_class, msg_id, rate=1):
#         """เปิดใช้ UBX message"""
#         payload = struct.pack('<BBB', msg_class, msg_id, rate)
#         cmd = self.create_ubx_message(0x06, 0x01, payload)
#         self.ser.write(cmd)
#         time.sleep(0.1)
    
#     def create_ubx_message(self, msg_class, msg_id, payload):
#         """สร้าง UBX message"""
#         header = struct.pack('<BBBB', 0xb5, 0x62, msg_class, msg_id)
#         length = struct.pack('<H', len(payload))
        
#         ck_a = ck_b = 0
#         for byte in struct.pack('<BB', msg_class, msg_id) + length + payload:
#             ck_a = (ck_a + byte) & 0xFF
#             ck_b = (ck_b + ck_a) & 0xFF
        
#         return header + length + payload + struct.pack('<BB', ck_a, ck_b)
    
#     def find_available_serial_ports(self):
#         """Find available serial ports"""
#         available_ports = []
#         for port in self.possible_serial_ports:
#             if os.path.exists(port):
#                 available_ports.append(port)
        
#         acm_ports = glob.glob('/dev/ttyACM*')
#         usb_ports = glob.glob('/dev/ttyUSB*')
        
#         for port in acm_ports + usb_ports:
#             if port not in available_ports:
#                 available_ports.append(port)
        
#         return sorted(available_ports)
    
#     def init_serial_connection(self):
#         """Initialize serial connection"""
#         self.get_logger().info("🔍 Searching for Enhanced ZED-F9R device...")
        
#         available_ports = self.find_available_serial_ports()
#         if not available_ports:
#             self.get_logger().error("❌ No serial ports found!")
#             return
        
#         baud_rates = [38400, 9600, 115200]
        
#         for port in available_ports:
#             for baud in baud_rates:
#                 try:
#                     test_ser = serial.Serial(port, baud, timeout=1)
#                     time.sleep(1)
                    
#                     if test_ser.in_waiting > 0 or self.test_serial_port_advanced(test_ser):
#                         self.ser = test_ser
#                         self.serial_port = port
#                         self.get_logger().info(f"✅ Enhanced connection to {port} at {baud} baud")
#                         return
                    
#                     test_ser.close()
#                 except:
#                     continue
        
#         # Last resort
#         if available_ports:
#             try:
#                 port = available_ports[0]
#                 self.ser = serial.Serial(port, 38400, timeout=1)
#                 self.serial_port = port
#                 self.get_logger().warning(f"⚠️ Connected to {port} without validation")
#             except Exception as e:
#                 self.get_logger().error(f"❌ Failed to connect: {e}")
    
#     def test_serial_port_advanced(self, test_ser):
#         """Advanced port testing"""
#         try:
#             start_time = time.time()
#             while time.time() - start_time < 2:
#                 if test_ser.in_waiting > 0:
#                     data = test_ser.read(test_ser.in_waiting)
#                     try:
#                         text = data.decode('ascii', errors='replace')
#                         if any(marker in text for marker in ['$G', 'UBX', '\xb5\x62']):
#                             return True
#                     except:
#                         pass
#                 time.sleep(0.1)
#             return False
#         except:
#             return False
    
#     def connect_to_ntrip(self):
#         """Connect to NTRIP server"""
#         try:
#             client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#             client_socket.connect((self.ntrip_server_ip, self.ntrip_server_port))
            
#             import base64
#             auth_str = f"{self.ntrip_username}:{self.ntrip_password}"
#             auth_b64 = base64.b64encode(auth_str.encode('ascii')).decode('ascii')
            
#             request = (
#                 f"GET /{self.mount_point} HTTP/1.1\r\n"
#                 f"User-Agent: NTRIP Client\r\n" 
#                 f"Authorization: Basic {auth_b64}\r\n\r\n"
#             )
#             client_socket.send(request.encode('ascii'))
#             self.get_logger().info("🌐 Connected to NTRIP server")
#             return client_socket
#         except Exception as e:
#             self.get_logger().warning(f"⚠️ NTRIP failed: {e}")
#             return None
    
#     def main_loop(self):
#         """Enhanced main processing loop"""
#         if not (self.ser and self.ser.is_open):
#             return
        
#         try:
#             # Read all available data
#             if self.ser.in_waiting > 0:
#                 data = self.ser.read(self.ser.in_waiting)
                
#                 # Process NMEA (text)
#                 try:
#                     text_data = data.decode('ascii', errors='replace')
#                     for line in text_data.split('\n'):
#                         line = line.strip()
#                         if line.startswith('$'):
#                             self.process_nmea_line(line)
#                 except:
#                     pass
                
#                 # 🌟 Enhanced UBX processing
#                 if PYUBX2_AVAILABLE:
#                     self.process_ubx_data_enhanced(data)
#                 else:
#                     self.process_ubx_data_manual(data)
            
#             # Handle NTRIP
#             self.handle_ntrip_data()
            
#             # 🌟 Update final heading with enhanced logic
#             self.update_enhanced_final_heading()
            
#             # 🌟 ตรวจสอบการเริ่มต้น controller
#             if (not self.controller_active and 
#                 self.controller_init_time is not None and
#                 time.time() - self.controller_init_time > self.controller_init_delay and
#                 self.current_utm_x is not None and self.current_utm_y is not None):
                
#                 self.init_waypoint_controller()
#                 self.controller_init_time = None
            
#             # 🌟 Run waypoint controller
#             self.run_controller()
            
#         except Exception as e:
#             self.get_logger().error(f"❌ Enhanced main loop error: {e}")
    
#     def process_nmea_line(self, line):
#         """Process NMEA line (for position)"""
#         try:
#             if any(line.startswith(nmea) for nmea in ['$GPGGA', '$GNGGA', '$GPRMC', '$GNRMC', '$GPGLL', '$GNGLL']):
#                 msg = pynmea2.parse(line)
                
#                 if hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
#                     if msg.latitude and msg.longitude:
#                         self.process_position(msg.latitude, msg.longitude)
            
#         except Exception as e:
#             self.get_logger().debug(f"NMEA parse error: {e}")
    
#     def process_position(self, latitude, longitude):
#         """Process GNSS position with UTM coordinates"""
#         current_time = time.time()
        
#         if not (-90 <= latitude <= 90) or not (-180 <= longitude <= 180):
#             return
        
#         # แปลงเป็น UTM coordinates สัมบูรณ์
#         utm_x, utm_y = self.transformer.transform(longitude, latitude)
        
#         # 🌟 เก็บ UTM coordinates สัมบูรณ์สำหรับ controller
#         self.current_utm_x = utm_x
#         self.current_utm_y = utm_y
        
#         # คำนวณ local coordinates สำหรับการแสดงผล
#         if self.reference_utm is not None:
#             local_x = utm_x - self.reference_utm[0]
#             local_y = utm_y - self.reference_utm[1]
#         else:
#             local_x = utm_x
#             local_y = utm_y
        
#         # Store position for heading calculation
#         position_record = {
#             'lat': latitude, 'lon': longitude,
#             'utm_x': utm_x, 'utm_y': utm_y,
#             'local_x': local_x, 'local_y': local_y,
#             'timestamp': current_time
#         }
#         self.position_history.append(position_record)
#         self.stats['position_count'] += 1
        
#         # Calculate GPS heading
#         self.update_gps_heading_improved()
        
#         # Publish data
#         self.publish_data(latitude, longitude, local_x, local_y)
    
#     def update_gps_heading_improved(self):
#         """🔧 Enhanced GPS heading calculation using UTM coordinates (compass bearing)"""
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
#                 # 🔧 คำนวณ compass bearing
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
#         """🌟 Enhanced final heading selection with NAV-ATT priority (compass bearing)"""
#         current_time = time.time()
        
#         # Check data validity
#         nav_att_valid = (self.nav_att_heading is not None and 
#                         (current_time - self.nav_att_time) < self.heading_timeout and
#                         self.fusion_mode in ["FUSION", "INIT"])
        
#         gps_valid = (self.gps_heading is not None and 
#                     (current_time - self.gps_heading_time) < self.heading_timeout)
        
#         # 🌟 Enhanced priority: NAV-ATT > GPS > Hold
#         if nav_att_valid and self.imu_calibration_status in ["FULLY_CALIBRATED", "CALIBRATED"]:
#             self.final_heading = self.nav_att_heading  # compass bearing (degrees)
#             self.heading_source = f"NAV_ATT_{self.imu_calibration_status}"
            
#         elif nav_att_valid and self.imu_calibration_status == "CALIBRATING":
#             if gps_valid:
#                 self.final_heading = self.blend_headings(self.nav_att_heading, self.gps_heading, 0.8)
#                 self.heading_source = "NAV_ATT_GPS_BLEND"
#             else:
#                 self.final_heading = self.nav_att_heading
#                 self.heading_source = "NAV_ATT_CALIBRATING"
                
#         elif gps_valid:
#             self.final_heading = self.gps_heading  # compass bearing (degrees)
#             self.heading_source = "GPS"
            
#         else:
#             self.final_heading = None
#             self.heading_source = "NONE"
        
#         # Publish enhanced heading
#         if self.final_heading is not None:
#             heading_msg = Float32()
#             heading_msg.data = float(self.final_heading)
#             self.heading_publisher.publish(heading_msg)
#             self.stats['filtered_count'] += 1
    
#     def blend_headings(self, heading1, heading2, weight1):
#         """Blend two headings considering circular nature (compass bearing)"""
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
    
#     def publish_data(self, latitude, longitude, local_x, local_y):
#         """Publish all data"""
#         # GNSS
#         gnss_msg = NavSatFix()
#         gnss_msg.latitude = latitude
#         gnss_msg.longitude = longitude
#         gnss_msg.status.status = 0
#         gnss_msg.status.service = 1
#         self.gnss_publisher.publish(gnss_msg)
        
#         # XY position (local coordinates สำหรับการแสดงผล)
#         xy_msg = Point()
#         xy_msg.x = local_x
#         xy_msg.y = local_y
#         xy_msg.z = 0.0
#         self.xy_publisher.publish(xy_msg)
    
#     def handle_ntrip_data(self):
#         """Handle NTRIP corrections"""
#         if self.ntrip_socket:
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
    
#     def process_ubx_data_enhanced(self, data):
#         """🌟 Enhanced UBX processing with pyubx2"""
#         pass  # Implementation ตามต้องการ
    
#     def process_ubx_data_manual(self, data):
#         """Fallback manual UBX processing"""
#         pass  # Implementation ตามต้องการ
    
#     def destroy_node(self):
#         """Cleanup"""
#         self.get_logger().info("🔄 Shutting down Enhanced ZED-F9R with Controller...")
        
#         if self.ser and self.ser.is_open:
#             self.ser.close()
        
#         if hasattr(self, 'ntrip_socket') and self.ntrip_socket:
#             self.ntrip_socket.close()
        
#         super().destroy_node()

# class EnhancedIMUHeadingFilter:
#     def __init__(self):
#         self.nav_att_history = deque(maxlen=12)
#         self.gps_history = deque(maxlen=8)
        
#         self.nav_att_outlier_threshold = 20.0
#         self.gps_outlier_threshold = 18.0
#         self.smoothing_alpha = 0.25
#         self.confidence_threshold = 0.8
        
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
#         return new_heading
    
#     def filter_gps_heading(self, new_gps_heading):
#         """Enhanced GPS heading filtering"""
#         return new_gps_heading
    
#     def get_stats(self):
#         """Get enhanced filter statistics"""
#         return self.stats.copy()

# def main(args=None):
#     rclpy.init(args=args)
#     gnss_publisher = EnhancedZEDf9rGNSSWithHeading()
    
#     try:
#         rclpy.spin(gnss_publisher)
#     except KeyboardInterrupt:
#         gnss_publisher.get_logger().info("🛑 Keyboard interrupt")
#     finally:
#         gnss_publisher.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# --------------------------------------test--------------------------------------------

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




# ------------------------------------------------GMSS for Bang Bang-----------------------------------------------------------

# import serial
# import pynmea2
# import socket
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
# from std_msgs.msg import Float32, String
# from geometry_msgs.msg import Point
# import math
# import pyproj
# from collections import deque
# import time
# import glob
# import os
# import struct
# import numpy as np

# # 🔧 เพิ่ม pyubx2 สำหรับ UBX parsing ที่ดีขึ้น
# try:
#     from pyubx2 import UBXReader, UBXMessage, UBX_MSGIDS
#     PYUBX2_AVAILABLE = True
#     print("✅ pyubx2 available - using enhanced UBX parsing")
# except ImportError:
#     PYUBX2_AVAILABLE = False
#     print("⚠️ pyubx2 not available - install with: pip3 install pyubx2")

# class EnhancedIMUHeadingFilter:
#     def __init__(self):
#         self.nav_att_history = deque(maxlen=12)
#         self.gps_history = deque(maxlen=8)
        
#         self.nav_att_outlier_threshold = 20.0
#         self.gps_outlier_threshold = 18.0
#         self.smoothing_alpha = 0.25
#         self.confidence_threshold = 0.8
        
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
        
#         # Add to history
#         self.nav_att_history.append({
#             'heading': new_heading,
#             'timestamp': time.time(),
#             'confidence': confidence_factor
#         })
        
#         if len(self.nav_att_history) < 3:
#             return new_heading
        
#         # Calculate median for outlier detection
#         recent_headings = [h['heading'] for h in list(self.nav_att_history)[-5:]]
#         median_heading = np.median(recent_headings)
        
#         # Check for outliers
#         heading_diff = abs(self.angle_difference(new_heading, median_heading))
        
#         if heading_diff > self.nav_att_outlier_threshold:
#             self.stats['nav_att_outliers'] += 1
#             return None
        
#         # Apply smoothing
#         if len(self.nav_att_history) >= 2:
#             prev_heading = self.nav_att_history[-2]['heading']
#             smoothed_heading = self.smooth_heading_transition(prev_heading, new_heading, self.smoothing_alpha)
#         else:
#             smoothed_heading = new_heading
        
#         self.stats['nav_att_filtered'] += 1
#         return smoothed_heading
    
#     def filter_gps_heading(self, new_gps_heading):
#         """Enhanced GPS heading filtering"""
#         if new_gps_heading is None:
#             return None
        
#         # Add to history
#         self.gps_history.append({
#             'heading': new_gps_heading,
#             'timestamp': time.time()
#         })
        
#         if len(self.gps_history) < 2:
#             return new_gps_heading
        
#         # Calculate median for outlier detection
#         recent_headings = [h['heading'] for h in list(self.gps_history)[-3:]]
#         median_heading = np.median(recent_headings)
        
#         # Check for outliers
#         heading_diff = abs(self.angle_difference(new_gps_heading, median_heading))
        
#         if heading_diff > self.gps_outlier_threshold:
#             self.stats['gps_outliers'] += 1
#             return None
        
#         self.stats['gps_filtered'] += 1
#         return new_gps_heading
    
#     def angle_difference(self, angle1, angle2):
#         """Calculate the smallest difference between two angles"""
#         diff = angle1 - angle2
#         while diff > 180:
#             diff -= 360
#         while diff < -180:
#             diff += 360
#         return diff
    
#     def smooth_heading_transition(self, prev_heading, new_heading, alpha):
#         """Smooth heading transition considering circular nature"""
#         diff = self.angle_difference(new_heading, prev_heading)
#         smoothed_diff = alpha * diff
#         return (prev_heading + smoothed_diff) % 360
    
#     def get_stats(self):
#         """Get enhanced filter statistics"""
#         return self.stats.copy()

# class GNSSReceiverNode(Node):
#     def __init__(self):
#         super().__init__('gnss_receiver_node')
        
#         # ROS2 Publishers
#         self.gnss_publisher = self.create_publisher(NavSatFix, '/navigation/gnss', 10)
#         self.xy_publisher = self.create_publisher(Point, '/navigation/xy_position', 10)
#         self.heading_publisher = self.create_publisher(Float32, '/navigation/heading', 10)
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
        
#         # 🔧 Fixed Coordinate transformation
#         self.transformer = pyproj.Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)
        
#         # Position tracking
#         self.position_history = deque(maxlen=15)
#         self.min_movement_for_heading = 0.25
        
#         # 🔧 Fixed reference point
#         self.reference_point = (13.650748, 100.492985)
#         self.reference_utm = None
        
#         # 🔧 Enhanced Heading management สำหรับ ZED-F9R
#         self.imu_heading = None
#         self.nav_att_heading = None
#         self.nav_att_yaw = None
#         self.gps_heading = None
#         self.final_heading = None
#         self.heading_source = "NONE"
        
#         self.imu_heading_time = 0
#         self.nav_att_time = 0
#         self.gps_heading_time = 0
#         self.heading_timeout = 8.0
        
#         # ZED-F9R specific data
#         self.esf_status = None
#         self.nav_att = None
#         self.esf_alg = None
#         self.fusion_mode = "NONE"
#         self.imu_calibration_status = "UNKNOWN"
        
#         # 🌟 NEW: pyubx2 UBX parsing
#         self.ubx_reader = None
#         self.ubx_buffer = bytearray()
        
#         # 🔧 Enhanced Heading filtering
#         self.heading_filter = EnhancedIMUHeadingFilter()
        
#         # Performance stats
#         self.stats = {
#             'imu_count': 0,
#             'nav_att_count': 0,
#             'nav_att_yaw_count': 0,
#             'gps_count': 0,
#             'position_count': 0,
#             'esf_meas_count': 0,
#             'esf_status_count': 0,
#             'pyubx2_success': 0,
#             'pyubx2_errors': 0,
#             'filtered_count': 0
#         }
        
#         # 🔧 Initialize coordinate system
#         self.init_fixed_coordinate_system()
        
#         # Initialize
#         self.init_serial_connection()
        
#         if self.ser and self.ser.is_open:
#             self.ntrip_socket = self.connect_to_ntrip()
            
#             # 🌟 Initialize pyubx2 if available
#             if PYUBX2_AVAILABLE:
#                 self.init_pyubx2()
            
#             # Configure ZED-F9R for enhanced IMU
#             self.configure_enhanced_zed_f9r()
            
#             # Main processing timer
#             self.create_timer(0.1, self.main_loop)
            
#             self.get_logger().info('✅ GNSS Receiver Node initialized')
#             self.get_logger().info(f'🔧 pyubx2: {"Available" if PYUBX2_AVAILABLE else "Not available"}')
#         else:
#             self.get_logger().error('❌ Failed to initialize GNSS receiver')
    
#     def init_pyubx2(self):
#         """🌟 Initialize pyubx2 for enhanced UBX parsing"""
#         try:
#             if PYUBX2_AVAILABLE:
#                 self.get_logger().info('🌟 pyubx2 UBX parser initialized')
#             else:
#                 self.get_logger().warning('⚠️ pyubx2 not available, using manual parsing')
#         except Exception as e:
#             self.get_logger().error(f'❌ pyubx2 initialization failed: {e}')
    
#     def init_fixed_coordinate_system(self):
#         """🔧 Initialize fixed coordinate system"""
#         try:
#             self.reference_utm = self.transformer.transform(
#                 self.reference_point[1], self.reference_point[0]
#             )
#             self.get_logger().info(
#                 f'🔧 Enhanced coordinate system initialized:\n'
#                 f'  📍 Reference: ({self.reference_point[0]:.6f}, {self.reference_point[1]:.6f})\n'
#                 f'  📍 UTM: ({self.reference_utm[0]:.2f}, {self.reference_utm[1]:.2f})'
#             )
#         except Exception as e:
#             self.get_logger().error(f'❌ Coordinate system init failed: {e}')
#             self.reference_utm = None
    
#     def configure_enhanced_zed_f9r(self):
#         """🌟 Enhanced ZED-F9R configuration for better NAV-ATT"""
#         try:
#             self.get_logger().info('🌟 Configuring ZED-F9R for enhanced NAV-ATT...')
#             time.sleep(2)
            
#             # Enable UBX messages with higher rates
#             self.enable_ubx_message(0x01, 0x05, 5)  # 🌟 NAV-ATT at 5Hz
#             self.enable_ubx_message(0x10, 0x02, 2)  # ESF-MEAS at 2Hz
#             self.enable_ubx_message(0x10, 0x10, 1)  # ESF-STATUS
#             self.enable_ubx_message(0x10, 0x14, 1)  # ESF-ALG
            
#             self.get_logger().info('✅ Enhanced ZED-F9R configuration sent')
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Enhanced configuration failed: {e}')
    
#     def enable_ubx_message(self, msg_class, msg_id, rate=1):
#         """เปิดใช้ UBX message"""
#         payload = struct.pack('<BBB', msg_class, msg_id, rate)
#         cmd = self.create_ubx_message(0x06, 0x01, payload)
#         self.ser.write(cmd)
#         time.sleep(0.1)
    
#     def create_ubx_message(self, msg_class, msg_id, payload):
#         """สร้าง UBX message"""
#         header = struct.pack('<BBBB', 0xb5, 0x62, msg_class, msg_id)
#         length = struct.pack('<H', len(payload))
        
#         ck_a = ck_b = 0
#         for byte in struct.pack('<BB', msg_class, msg_id) + length + payload:
#             ck_a = (ck_a + byte) & 0xFF
#             ck_b = (ck_b + ck_a) & 0xFF
        
#         return header + length + payload + struct.pack('<BB', ck_a, ck_b)
    
#     def find_available_serial_ports(self):
#         """Find available serial ports"""
#         available_ports = []
#         for port in self.possible_serial_ports:
#             if os.path.exists(port):
#                 available_ports.append(port)
        
#         acm_ports = glob.glob('/dev/ttyACM*')
#         usb_ports = glob.glob('/dev/ttyUSB*')
        
#         for port in acm_ports + usb_ports:
#             if port not in available_ports:
#                 available_ports.append(port)
        
#         return sorted(available_ports)
    
#     def init_serial_connection(self):
#         """Initialize serial connection"""
#         self.get_logger().info("🔍 Searching for Enhanced ZED-F9R device...")
        
#         available_ports = self.find_available_serial_ports()
#         if not available_ports:
#             self.get_logger().error("❌ No serial ports found!")
#             return
        
#         baud_rates = [38400, 9600, 115200]
        
#         for port in available_ports:
#             for baud in baud_rates:
#                 try:
#                     test_ser = serial.Serial(port, baud, timeout=1)
#                     time.sleep(1)
                    
#                     if test_ser.in_waiting > 0 or self.test_serial_port_advanced(test_ser):
#                         self.ser = test_ser
#                         self.serial_port = port
#                         self.get_logger().info(f"✅ Enhanced connection to {port} at {baud} baud")
#                         return
                    
#                     test_ser.close()
#                 except:
#                     continue
        
#         # Last resort
#         if available_ports:
#             try:
#                 port = available_ports[0]
#                 self.ser = serial.Serial(port, 38400, timeout=1)
#                 self.serial_port = port
#                 self.get_logger().warning(f"⚠️ Connected to {port} without validation")
#             except Exception as e:
#                 self.get_logger().error(f"❌ Failed to connect: {e}")
    
#     def test_serial_port_advanced(self, test_ser):
#         """Advanced port testing"""
#         try:
#             start_time = time.time()
#             while time.time() - start_time < 2:
#                 if test_ser.in_waiting > 0:
#                     data = test_ser.read(test_ser.in_waiting)
#                     try:
#                         text = data.decode('ascii', errors='replace')
#                         if any(marker in text for marker in ['$G', 'UBX', '\xb5\x62']):
#                             return True
#                     except:
#                         pass
#                 time.sleep(0.1)
#             return False
#         except:
#             return False
    
#     def connect_to_ntrip(self):
#         """Connect to NTRIP server"""
#         try:
#             client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#             client_socket.connect((self.ntrip_server_ip, self.ntrip_server_port))
            
#             import base64
#             auth_str = f"{self.ntrip_username}:{self.ntrip_password}"
#             auth_b64 = base64.b64encode(auth_str.encode('ascii')).decode('ascii')
            
#             request = (
#                 f"GET /{self.mount_point} HTTP/1.1\r\n"
#                 f"User-Agent: NTRIP Client\r\n" 
#                 f"Authorization: Basic {auth_b64}\r\n\r\n"
#             )
#             client_socket.send(request.encode('ascii'))
#             self.get_logger().info("🌐 Connected to NTRIP server")
#             return client_socket
#         except Exception as e:
#             self.get_logger().warning(f"⚠️ NTRIP failed: {e}")
#             return None
    
#     def main_loop(self):
#         """Enhanced main processing loop"""
#         if not (self.ser and self.ser.is_open):
#             return
        
#         try:
#             # Read all available data
#             if self.ser.in_waiting > 0:
#                 data = self.ser.read(self.ser.in_waiting)
                
#                 # Process NMEA (text)
#                 try:
#                     text_data = data.decode('ascii', errors='replace')
#                     for line in text_data.split('\n'):
#                         line = line.strip()
#                         if line.startswith('$'):
#                             self.process_nmea_line(line)
#                 except:
#                     pass
                
#                 # 🌟 Enhanced UBX processing
#                 if PYUBX2_AVAILABLE:
#                     self.process_ubx_data_enhanced(data)
#                 else:
#                     self.process_ubx_data_manual(data)
            
#             # Handle NTRIP
#             self.handle_ntrip_data()
            
#             # 🌟 Update final heading with enhanced logic
#             self.update_enhanced_final_heading()
            
#         except Exception as e:
#             self.get_logger().error(f"❌ Enhanced main loop error: {e}")
    
#     def process_nmea_line(self, line):
#         """Process NMEA line (for position)"""
#         try:
#             if any(line.startswith(nmea) for nmea in ['$GPGGA', '$GNGGA', '$GPRMC', '$GNRMC', '$GPGLL', '$GNGLL']):
#                 msg = pynmea2.parse(line)
                
#                 if hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
#                     if msg.latitude and msg.longitude:
#                         self.process_position(msg.latitude, msg.longitude)
            
#         except Exception as e:
#             self.get_logger().debug(f"NMEA parse error: {e}")
    
#     def process_position(self, latitude, longitude):
#         """Process GNSS position with enhanced coordinate system"""
#         current_time = time.time()
        
#         if not (-90 <= latitude <= 90) or not (-180 <= longitude <= 180):
#             return
        
#         # Convert using fixed coordinate system
#         local_x, local_y = self.latlon_to_xy_fixed(latitude, longitude)
        
#         # Store position
#         position_record = {
#             'lat': latitude, 'lon': longitude,
#             'local_x': local_x, 'local_y': local_y,
#             'timestamp': current_time
#         }
#         self.position_history.append(position_record)
#         self.stats['position_count'] += 1
        
#         # Calculate GPS heading
#         self.update_gps_heading_improved()
        
#         # Publish data
#         self.publish_data(latitude, longitude, local_x, local_y)
    
#     def latlon_to_xy_fixed(self, lat, lon):
#         """Convert lat/lon using fixed PyProj"""
#         try:
#             if self.reference_utm is None:
#                 return self.simple_latlon_to_xy(lat, lon)
            
#             utm_x, utm_y = self.transformer.transform(lon, lat)
#             local_x = utm_x - self.reference_utm[0]
#             local_y = utm_y - self.reference_utm[1]
            
#             return local_x, local_y
            
#         except Exception as e:
#             return self.simple_latlon_to_xy(lat, lon)
    
#     def simple_latlon_to_xy(self, lat, lon):
#         """Fallback simple conversion"""
#         dlat = lat - self.reference_point[0]
#         dlon = lon - self.reference_point[1]
        
#         x = dlon * 111319.9 * math.cos(math.radians(lat))
#         y = dlat * 111319.9
        
#         return x, y
    
#     def update_gps_heading_improved(self):
#         """Enhanced GPS heading calculation"""
#         if len(self.position_history) < 3:
#             return
        
#         total_distance = 0.0
#         weighted_x = weighted_y = 0.0
        
#         recent_positions = list(self.position_history)[-5:]
        
#         for i in range(len(recent_positions) - 1):
#             curr = recent_positions[i+1]
#             prev = recent_positions[i]
            
#             dx = curr['local_x'] - prev['local_x']
#             dy = curr['local_y'] - prev['local_y']
#             distance = math.sqrt(dx*dx + dy*dy)
            
#             if distance > self.min_movement_for_heading:
#                 heading_rad = math.atan2(dx, dy)
                
#                 time_weight = (i + 1) / len(recent_positions)
#                 weight = distance * time_weight
                
#                 weighted_x += math.cos(heading_rad) * weight
#                 weighted_y += math.sin(heading_rad) * weight
#                 total_distance += weight
        
#         if total_distance > self.min_movement_for_heading:
#             avg_heading = math.degrees(math.atan2(weighted_y, weighted_x))
#             if avg_heading < 0:
#                 avg_heading += 360
            
#             filtered_gps_heading = self.heading_filter.filter_gps_heading(avg_heading)
            
#             if filtered_gps_heading is not None:
#                 self.gps_heading = filtered_gps_heading
#                 self.gps_heading_time = time.time()
#                 self.stats['gps_count'] += 1
    
#     def update_enhanced_final_heading(self):
#         """🌟 Enhanced final heading selection with NAV-ATT priority"""
#         current_time = time.time()
        
#         # Check data validity
#         nav_att_valid = (self.nav_att_heading is not None and 
#                         (current_time - self.nav_att_time) < self.heading_timeout and
#                         self.fusion_mode in ["FUSION", "INIT"])
        
#         gps_valid = (self.gps_heading is not None and 
#                     (current_time - self.gps_heading_time) < self.heading_timeout)
        
#         # 🌟 Enhanced priority: NAV-ATT > GPS > Hold
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
#             self.final_heading = None
#             self.heading_source = "NONE"
        
#         # Publish enhanced heading
#         if self.final_heading is not None:
#             heading_msg = Float32()
#             heading_msg.data = float(self.final_heading)
#             self.heading_publisher.publish(heading_msg)
#             self.stats['filtered_count'] += 1
            
#             # Publish debug info
#             debug_msg = String()
#             debug_msg.data = f"Source: {self.heading_source}, Heading: {self.final_heading:.1f}°"
#             self.heading_debug_publisher.publish(debug_msg)
    
#     def blend_headings(self, heading1, heading2, weight1):
#         """Blend two headings considering circular nature"""
#         if heading1 is None or heading2 is None:
#             return heading1 or heading2
        
#         x1, y1 = math.cos(math.radians(heading1)), math.sin(math.radians(heading1))
#         x2, y2 = math.cos(math.radians(heading2)), math.sin(math.radians(heading2))
        
#         weight2 = 1.0 - weight1
#         x_blend = weight1 * x1 + weight2 * x2
#         y_blend = weight1 * y1 + weight2 * y2
        
#         blended_heading = math.degrees(math.atan2(y_blend, x_blend))
#         return blended_heading % 360
    
#     def publish_data(self, latitude, longitude, local_x, local_y):
#         """Publish all data"""
#         # GNSS
#         gnss_msg = NavSatFix()
#         gnss_msg.latitude = latitude
#         gnss_msg.longitude = longitude
#         gnss_msg.status.status = 0
#         gnss_msg.status.service = 1
#         self.gnss_publisher.publish(gnss_msg)
        
#         # XY position
#         xy_msg = Point()
#         xy_msg.x = local_x
#         xy_msg.y = local_y
#         xy_msg.z = 0.0
#         self.xy_publisher.publish(xy_msg)
    
#     def handle_ntrip_data(self):
#         """Handle NTRIP corrections"""
#         if self.ntrip_socket:
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
    
#     def process_ubx_data_enhanced(self, data):
#         """🌟 Enhanced UBX processing with pyubx2"""
#         # Add data to buffer
#         self.ubx_buffer.extend(data)
        
#         # Process complete UBX messages
#         while len(self.ubx_buffer) >= 8:  # Minimum UBX message size
#             # Find UBX sync bytes
#             sync_pos = -1
#             for i in range(len(self.ubx_buffer) - 1):
#                 if self.ubx_buffer[i] == 0xb5 and self.ubx_buffer[i+1] == 0x62:
#                     sync_pos = i
#                     break
            
#             if sync_pos == -1:
#                 # No sync found, clear buffer
#                 self.ubx_buffer.clear()
#                 break
            
#             # Remove data before sync
#             if sync_pos > 0:
#                 self.ubx_buffer = self.ubx_buffer[sync_pos:]
            
#             # Check if we have enough data for length field
#             if len(self.ubx_buffer) < 6:
#                 break
            
#             # Get message length
#             msg_length = struct.unpack('<H', self.ubx_buffer[4:6])[0]
#             total_length = 8 + msg_length  # header(6) + payload + checksum(2)
            
#             if len(self.ubx_buffer) < total_length:
#                 break  # Wait for more data
            
#             # Extract complete message
#             msg_data = bytes(self.ubx_buffer[:total_length])
#             self.ubx_buffer = self.ubx_buffer[total_length:]
            
#             try:
#                 # Parse with pyubx2
#                 if PYUBX2_AVAILABLE:
#                     from pyubx2 import UBXReader
#                     ubx_reader = UBXReader(msg_data)
#                     parsed_msg = ubx_reader.read()
                    
#                     if parsed_msg:
#                         self.process_ubx_message(parsed_msg)
#                         self.stats['pyubx2_success'] += 1
#                 else:
#                     # Fallback to manual parsing
#                     self.process_ubx_message_manual(msg_data)
                    
#             except Exception as e:
#                 self.stats['pyubx2_errors'] += 1
#                 self.get_logger().debug(f"UBX parse error: {e}")
    
#     def process_ubx_data_manual(self, data):
#         """Fallback manual UBX processing"""
#         # Add data to buffer
#         self.ubx_buffer.extend(data)
        
#         # Process complete UBX messages
#         while len(self.ubx_buffer) >= 8:
#             # Find UBX sync bytes
#             sync_pos = -1
#             for i in range(len(self.ubx_buffer) - 1):
#                 if self.ubx_buffer[i] == 0xb5 and self.ubx_buffer[i+1] == 0x62:
#                     sync_pos = i
#                     break
            
#             if sync_pos == -1:
#                 self.ubx_buffer.clear()
#                 break
            
#             if sync_pos > 0:
#                 self.ubx_buffer = self.ubx_buffer[sync_pos:]
            
#             if len(self.ubx_buffer) < 8:
#                 break
            
#             msg_class = self.ubx_buffer[2]
#             msg_id = self.ubx_buffer[3]
#             msg_length = struct.unpack('<H', self.ubx_buffer[4:6])[0]
#             total_length = 8 + msg_length
            
#             if len(self.ubx_buffer) < total_length:
#                 break
            
#             msg_data = bytes(self.ubx_buffer[:total_length])
#             self.ubx_buffer = self.ubx_buffer[total_length:]
            
#             try:
#                 self.process_ubx_message_manual(msg_data)
#             except Exception as e:
#                 self.get_logger().debug(f"Manual UBX parse error: {e}")
    
#     def process_ubx_message(self, msg):
#         """Process parsed UBX message"""
#         try:
#             if hasattr(msg, 'identity'):
#                 msg_id = msg.identity
                
#                 if msg_id == 'NAV-ATT':
#                     self.process_nav_att_message(msg)
#                 elif msg_id == 'ESF-STATUS':
#                     self.process_esf_status_message(msg)
#                 elif msg_id == 'ESF-MEAS':
#                     self.process_esf_meas_message(msg)
                    
#         except Exception as e:
#             self.get_logger().debug(f"UBX message processing error: {e}")
    
#     def process_ubx_message_manual(self, msg_data):
#         """Manual UBX message processing"""
#         try:
#             if len(msg_data) < 8:
#                 return
            
#             msg_class = msg_data[2]
#             msg_id = msg_data[3]
#             payload = msg_data[6:-2]
            
#             if msg_class == 0x01 and msg_id == 0x05:  # NAV-ATT
#                 self.process_nav_att_manual(payload)
#             elif msg_class == 0x10 and msg_id == 0x10:  # ESF-STATUS
#                 self.process_esf_status_manual(payload)
                
#         except Exception as e:
#             self.get_logger().debug(f"Manual UBX processing error: {e}")
    
#     def process_nav_att_message(self, msg):
#         """Process NAV-ATT message"""
#         try:
#             if hasattr(msg, 'heading'):
#                 heading_deg = msg.heading / 100000.0  # Convert from 1e-5 degrees
                
#                 # Filter heading
#                 filtered_heading = self.heading_filter.filter_nav_att_heading(heading_deg, 1.0)
                
#                 if filtered_heading is not None:
#                     self.nav_att_heading = filtered_heading
#                     self.nav_att_time = time.time()
#                     self.stats['nav_att_count'] += 1
                    
#                     # Update calibration status
#                     if hasattr(msg, 'accH'):
#                         accuracy = msg.accH / 100000.0  # Convert from 1e-5 degrees
#                         if accuracy < 1.0:
#                             self.imu_calibration_status = "FULLY_CALIBRATED"
#                         elif accuracy < 5.0:
#                             self.imu_calibration_status = "CALIBRATED"
#                         else:
#                             self.imu_calibration_status = "CALIBRATING"
                    
#                     self.fusion_mode = "FUSION"
                    
#         except Exception as e:
#             self.get_logger().debug(f"NAV-ATT processing error: {e}")
    
#     def process_nav_att_manual(self, payload):
#         """Manual NAV-ATT processing"""
#         try:
#             if len(payload) >= 32:
#                 # Parse NAV-ATT payload manually
#                 heading_raw = struct.unpack('<i', payload[12:16])[0]
#                 heading_deg = heading_raw / 100000.0
                
#                 # Filter heading
#                 filtered_heading = self.heading_filter.filter_nav_att_heading(heading_deg, 1.0)
                
#                 if filtered_heading is not None:
#                     self.nav_att_heading = filtered_heading
#                     self.nav_att_time = time.time()
#                     self.stats['nav_att_count'] += 1
#                     self.fusion_mode = "FUSION"
#                     self.imu_calibration_status = "CALIBRATED"
                    
#         except Exception as e:
#             self.get_logger().debug(f"Manual NAV-ATT processing error: {e}")
    
#     def process_esf_status_message(self, msg):
#         """Process ESF-STATUS message"""
#         try:
#             if hasattr(msg, 'fusionMode'):
#                 fusion_mode_map = {0: "INIT", 1: "FUSION", 2: "SUSPENDED", 3: "DISABLED"}
#                 self.fusion_mode = fusion_mode_map.get(msg.fusionMode, "UNKNOWN")
#                 self.stats['esf_status_count'] += 1
                
#         except Exception as e:
#             self.get_logger().debug(f"ESF-STATUS processing error: {e}")
    
#     def process_esf_status_manual(self, payload):
#         """Manual ESF-STATUS processing"""
#         try:
#             if len(payload) >= 16:
#                 fusion_mode = payload[8]
#                 fusion_mode_map = {0: "INIT", 1: "FUSION", 2: "SUSPENDED", 3: "DISABLED"}
#                 self.fusion_mode = fusion_mode_map.get(fusion_mode, "UNKNOWN")
#                 self.stats['esf_status_count'] += 1
                
#         except Exception as e:
#             self.get_logger().debug(f"Manual ESF-STATUS processing error: {e}")
    
#     def process_esf_meas_message(self, msg):
#         """Process ESF-MEAS message"""
#         try:
#             self.stats['esf_meas_count'] += 1
#         except Exception as e:
#             self.get_logger().debug(f"ESF-MEAS processing error: {e}")
    
#     def destroy_node(self):
#         """Cleanup"""
#         self.get_logger().info("🔄 Shutting down GNSS Receiver Node...")
        
#         if self.ser and self.ser.is_open:
#             self.ser.close()
        
#         if hasattr(self, 'ntrip_socket') and self.ntrip_socket:
#             self.ntrip_socket.close()
        
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     gnss_receiver = GNSSReceiverNode()
    
#     try:
#         rclpy.spin(gnss_receiver)
#     except KeyboardInterrupt:
#         gnss_receiver.get_logger().info("🛑 Keyboard interrupt")
#     finally:
#         gnss_receiver.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

    # ------------------------------------------PP+waypoint-----------------------------------------------

#!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix, Imu
# from geometry_msgs.msg import Twist
# from std_msgs.msg import Float32
# import pyproj
# import numpy as np
# import math
# import csv
# import serial
# import pynmea2
# import socket
# import re
# from typing import List, Tuple, Optional
# from scipy.spatial.transform import Rotation as R

# class IntegratedPathController(Node):
#     def __init__(self):
#         super().__init__('integrated_path_controller')
        
#         # Vehicle parameters
#         self.wheelbase = 1.67  # เมตร (ฐานล้อ)
#         self.max_steering_angle = 0.873  # rad (50 degrees)
#         self.fixed_speed = 1.0  # m/s (ความเร็วคงที่)
        
#         # Pure Pursuit parameters
#         self.lookahead_distance = 2.5  # เมตร
#         self.k_gain = 2.0  # look ahead gain
        
#         # Coordinate transformer
#         self.transformer = pyproj.Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)
        
#         # Path and steering data
#         self.waypoints = []  # [(x, y), ...]
#         self.steering_commands = {}  # {(from_wp, to_wp): steering_angle}
#         self.current_waypoint_index = 0
#         self.target_index = 0
#         self.use_pure_pursuit = True  # เริ่มด้วย Pure Pursuit
        
#         # Current state - ใช้ compass bearing system (0-360°)
#         self.current_position = None  # (x, y) in UTM
#         self.current_heading_deg = 0.0  # degrees (compass bearing: 0° = North, 90° = East)
#         self.current_speed = 0.0
        
#         # IMU and GPS fusion parameters
#         self.gps_heading_deg = 0.0
#         self.imu_heading_deg = 0.0
#         self.fused_heading_deg = 0.0
#         self.previous_gps_position = None
#         self.gps_heading_valid = False
#         self.imu_initialized = False
        
#         # Fusion filter parameters
#         self.heading_filter_alpha = 0.98  # IMU weight (0.98 = 98% IMU, 2% GPS)
#         self.min_gps_speed = 0.5  # m/s minimum speed for GPS heading calculation
        
#         # GNSS configuration
#         self.serial_port = '/dev/ttyACM0'
#         self.ntrip_server_ip = "110.78.0.54"
#         self.ntrip_server_port = 2116
#         self.ntrip_username = "1118600009224"
#         self.ntrip_password = "CK79"
#         self.mount_point = "VRS_RTCM32"
        
#         # File paths
#         self.waypoint_file = '/home/inc/ros2_ws/waypointfile/waypoints1m.csv'
#         self.steering_file = '/home/inc/ros2_ws/waypointfile/waypoint_segment_result.csv'
        
#         # ROS2 publishers and subscribers
#         self.gnss_publisher = self.create_publisher(NavSatFix, '/navigation/gnss', 10)
#         self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.steering_publisher = self.create_publisher(Float32, '/steering_angle', 10)
#         self.heading_publisher = self.create_publisher(Float32, '/fused_heading', 10)
        
#         # IMU subscriber
#         self.imu_subscriber = self.create_subscription(
#             Imu,
#             '/imu/data',
#             self.imu_callback,
#             10
#         )
        
#         # Initialize GNSS
#         self.initialize_gnss()
        
#         # Control timer
#         self.create_timer(0.1, self.control_loop)  # 10 Hz
#         self.create_timer(0.1, self.read_gnss_data)  # 10 Hz GNSS reading
        
#         # Load path and steering data from CSV files
#         if not self.load_waypoints_from_csv():
#             self.get_logger().error('Failed to load waypoints. Shutting down.')
#             return
        
#         if not self.load_steering_from_csv():
#             self.get_logger().error('Failed to load steering data. Shutting down.')
#             return
        
#         self.get_logger().info('Integrated Path Controller with Compass Bearing System initialized')
    
#     def initialize_gnss(self):
#         """Initialize GNSS serial connection and NTRIP"""
#         try:
#             self.ser = serial.Serial(self.serial_port, 9600, timeout=1)
#             self.get_logger().info(f"Connected to GNSS at {self.serial_port}")
#         except Exception as e:
#             self.get_logger().error(f"Error connecting to GNSS: {e}")
#             self.ser = None
        
#         # Connect to NTRIP server
#         self.ntrip_socket = self.connect_to_ntrip()
    
#     def connect_to_ntrip(self):
#         """Connect to NTRIP server for RTK corrections"""
#         try:
#             client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#             client_socket.connect((self.ntrip_server_ip, self.ntrip_server_port))
#             auth_str = f"GET /{self.mount_point} HTTP/1.1\r\nUser-Agent: NTRIP Client\r\nAuthorization: Basic {self.ntrip_username}:{self.ntrip_password}\r\n\r\n"
#             client_socket.send(auth_str.encode('ascii'))
#             self.get_logger().info("Connected to NTRIP server")
#             return client_socket
#         except Exception as e:
#             self.get_logger().error(f"Error connecting to NTRIP server: {e}")
#             return None
    
#     def load_waypoints_from_csv(self) -> bool:
#         """โหลด waypoints จากไฟล์ CSV ตามที่อยู่ที่กำหนด"""
#         try:
#             self.waypoints = []
#             with open(self.waypoint_file, 'r') as csvfile:
#                 reader = csv.DictReader(csvfile)
#                 for row in reader:
#                     x = float(row['x_east'])
#                     y = float(row['y_north'])
#                     self.waypoints.append((x, y))
            
#             self.get_logger().info(f'Successfully loaded {len(self.waypoints)} waypoints from {self.waypoint_file}')
#             return True
#         except FileNotFoundError:
#             self.get_logger().error(f'Waypoint file not found: {self.waypoint_file}')
#             return False
#         except KeyError as e:
#             self.get_logger().error(f'Missing column in waypoint file: {e}')
#             return False
#         except Exception as e:
#             self.get_logger().error(f'Error loading waypoints from {self.waypoint_file}: {e}')
#             return False
    
#     def load_steering_from_csv(self) -> bool:
#         """โหลดคำสั่ง steering จากไฟล์ CSV ตามที่อยู่ที่กำหนด"""
#         try:
#             self.steering_commands = {}
#             with open(self.steering_file, 'r') as csvfile:
#                 reader = csv.DictReader(csvfile)
#                 for row in reader:
#                     from_wp = int(row['from_wp'])
#                     to_wp = int(row['to_wp'])
#                     segment_info = row['segment']
                    
#                     # Extract steering value using regex
#                     steering_match = re.search(r'steering=([-+]?\d*\.?\d+)', segment_info)
#                     if steering_match:
#                         steering_angle = float(steering_match.group(1))
#                         self.steering_commands[(from_wp, to_wp)] = steering_angle
            
#             self.get_logger().info(f'Successfully loaded {len(self.steering_commands)} steering commands from {self.steering_file}')
#             return True
#         except FileNotFoundError:
#             self.get_logger().error(f'Steering file not found: {self.steering_file}')
#             return False
#         except KeyError as e:
#             self.get_logger().error(f'Missing column in steering file: {e}')
#             return False
#         except Exception as e:
#             self.get_logger().error(f'Error loading steering data from {self.steering_file}: {e}')
#             return False
    
#     def normalize_angle_deg(self, angle_deg: float) -> float:
#         """ปรับมุมให้อยู่ในช่วง [0, 360) องศา (compass bearing)"""
#         while angle_deg >= 360.0:
#             angle_deg -= 360.0
#         while angle_deg < 0.0:
#             angle_deg += 360.0
#         return angle_deg
    
#     def angle_difference_deg(self, target_deg: float, current_deg: float) -> float:
#         """คำนวณความแตกต่างของมุมในช่วง [-180, 180] องศา"""
#         diff = target_deg - current_deg
#         while diff > 180.0:
#             diff -= 360.0
#         while diff < -180.0:
#             diff += 360.0
#         return diff
    
#     def imu_callback(self, msg):
#         """รับข้อมูล IMU และแปลงเป็น compass bearing"""
#         try:
#             # แปลง quaternion เป็น Euler angles
#             orientation_q = msg.orientation
#             orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            
#             # ใช้ scipy เพื่อแปลง quaternion เป็น Euler angles
#             r = R.from_quat(orientation_list)
#             roll, pitch, yaw = r.as_euler('xyz', degrees=False)
            
#             # แปลง yaw จาก ENU coordinate เป็น compass bearing
#             # ENU: yaw = 0 คือ East, compass bearing: 0° คือ North
#             compass_bearing_rad = (math.pi/2) - yaw  # หมุน 90° counterclockwise
#             compass_bearing_deg = math.degrees(compass_bearing_rad)
            
#             # ปรับให้อยู่ในช่วง [0, 360)
#             self.imu_heading_deg = self.normalize_angle_deg(compass_bearing_deg)
#             self.imu_initialized = True
            
#             # ทำ sensor fusion
#             self.fuse_heading()
            
#         except Exception as e:
#             self.get_logger().error(f"Error processing IMU data: {e}")
    
#     def calculate_gps_heading_deg(self, current_pos, previous_pos, dt):
#         """คำนวณ compass bearing จาก GPS movement"""
#         if previous_pos is None or dt <= 0:
#             return None
        
#         # คำนวณความเร็ว
#         dx = current_pos[0] - previous_pos[0]  # East (UTM X)
#         dy = current_pos[1] - previous_pos[1]  # North (UTM Y)
#         speed = math.sqrt(dx**2 + dy**2) / dt
        
#         # ใช้ GPS heading เฉพาะเมื่อความเร็วเพียงพอ
#         if speed >= self.min_gps_speed:
#             # คำนวณ compass bearing: arctan2(East, North)
#             bearing_rad = math.atan2(dx, dy)
#             bearing_deg = math.degrees(bearing_rad)
            
#             # ปรับให้เป็นค่าบวก (0-360°)
#             bearing_deg = self.normalize_angle_deg(bearing_deg)
#             return bearing_deg
        
#         return None
    
#     def fuse_heading(self):
#         """ผสมผสาน heading จาก IMU และ GPS (compass bearing system)"""
#         if not self.imu_initialized:
#             return
        
#         if self.gps_heading_valid:
#             # Complementary filter with angle wrapping for compass bearing
#             angle_diff = self.angle_difference_deg(self.gps_heading_deg, self.imu_heading_deg)
            
#             # Apply complementary filter
#             self.fused_heading_deg = self.imu_heading_deg + (1 - self.heading_filter_alpha) * angle_diff
            
#             # Normalize to [0, 360)
#             self.fused_heading_deg = self.normalize_angle_deg(self.fused_heading_deg)
#         else:
#             # ใช้ IMU เพียงอย่างเดียวถ้า GPS heading ไม่น่าเชื่อถือ
#             self.fused_heading_deg = self.imu_heading_deg
        
#         # อัปเดต current heading
#         self.current_heading_deg = self.fused_heading_deg
        
#         # Publish fused heading
#         heading_msg = Float32()
#         heading_msg.data = math.radians(self.fused_heading_deg)  # Convert to radians for ROS
#         self.heading_publisher.publish(heading_msg)
    
#     def calculate_target_heading_deg(self, current_pos: Tuple[float, float], target_pos: Tuple[float, float]) -> float:
#         """คำนวณ compass bearing ไปยังเป้าหมาย"""
#         dx = target_pos[0] - current_pos[0]  # East (UTM X)
#         dy = target_pos[1] - current_pos[1]  # North (UTM Y)
        
#         # คำนวณ compass bearing: arctan2(East, North)
#         bearing_rad = math.atan2(dx, dy)
#         bearing_deg = math.degrees(bearing_rad)
        
#         # ปรับให้เป็นค่าบวก (0-360°)
#         return self.normalize_angle_deg(bearing_deg)
    
#     def read_gnss_data(self):
#         """อ่านข้อมูล GNSS และประมวลผล"""
#         if not hasattr(self, 'ser') or not self.ser or not self.ser.is_open:
#             return
        
#         try:
#             data = self.ser.readline().decode('ascii', errors='replace')
            
#             if data.startswith('$GPGGA') or data.startswith('$GNGLL') or data.startswith('$GPRMC'):
#                 try:
#                     msg = pynmea2.parse(data)
                    
#                     latitude = None
#                     longitude = None
#                     timestamp = None
                    
#                     if isinstance(msg, pynmea2.types.talker.GGA):
#                         latitude = msg.latitude
#                         longitude = msg.longitude
#                         timestamp = msg.timestamp
#                     elif isinstance(msg, pynmea2.types.talker.GLL):
#                         latitude = msg.latitude
#                         longitude = msg.longitude
#                         timestamp = msg.timestamp
#                     elif isinstance(msg, pynmea2.types.talker.RMC):
#                         latitude = msg.latitude
#                         longitude = msg.longitude
#                         timestamp = msg.timestamp
                    
#                     if latitude is not None and longitude is not None:
#                         # Publish GNSS data
#                         nav_msg = NavSatFix()
#                         nav_msg.latitude = latitude
#                         nav_msg.longitude = longitude
#                         self.gnss_publisher.publish(nav_msg)
                        
#                         # Convert to UTM and update current position
#                         x_utm, y_utm = self.transformer.transform(longitude, latitude)
#                         self.update_position_with_heading(x_utm, y_utm, timestamp)
                        
#                         # Handle RTK corrections
#                         self.handle_rtk_corrections()
                        
#                 except pynmea2.nmea.ChecksumError:
#                     self.get_logger().warning("NMEA checksum error")
        
#         except Exception as e:
#             self.get_logger().error(f"Error reading GNSS data: {e}")
    
#     def handle_rtk_corrections(self):
#         """จัดการ RTK corrections จาก NTRIP"""
#         if self.ntrip_socket and self.ser:
#             try:
#                 self.ntrip_socket.setblocking(0)
#                 try:
#                     rtk_data = self.ntrip_socket.recv(1024)
#                     if rtk_data:
#                         self.ser.write(rtk_data)
#                 except socket.error:
#                     pass  # No data available
#             except Exception as e:
#                 self.get_logger().error(f"Error handling RTK data: {e}")
    
#     def update_position_with_heading(self, x_utm: float, y_utm: float, timestamp):
#         """อัปเดตตำแหน่งและคำนวณ GPS heading (compass bearing)"""
#         current_time = self.get_clock().now().nanoseconds / 1e9
        
#         if self.previous_gps_position and hasattr(self, 'previous_gps_time'):
#             dt = current_time - self.previous_gps_time
            
#             # คำนวณ GPS heading (compass bearing)
#             gps_heading_deg = self.calculate_gps_heading_deg(
#                 (x_utm, y_utm), 
#                 self.previous_gps_position, 
#                 dt
#             )
            
#             if gps_heading_deg is not None:
#                 self.gps_heading_deg = gps_heading_deg
#                 self.gps_heading_valid = True
#             else:
#                 self.gps_heading_valid = False
        
#         # อัปเดตตำแหน่งและเวลา
#         self.current_position = (x_utm, y_utm)
#         self.previous_gps_position = (x_utm, y_utm)
#         self.previous_gps_time = current_time
    
#     def calculate_distance(self, pos1: Tuple[float, float], pos2: Tuple[float, float]) -> float:
#         """คำนวณระยะทางระหว่างจุดสองจุด"""
#         return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    
#     def find_nearest_forward_waypoint(self) -> int:
#         """หา waypoint ที่ใกล้ที่สุดที่อยู่หน้ารถ (ใช้ compass bearing)"""
#         if not self.current_position or not self.waypoints:
#             return 0
        
#         min_distance = float('inf')
#         nearest_index = 0
        
#         for i, waypoint in enumerate(self.waypoints):
#             # คำนวณระยะทาง
#             distance = self.calculate_distance(self.current_position, waypoint)
            
#             # คำนวณ compass bearing ไปยัง waypoint
#             waypoint_bearing_deg = self.calculate_target_heading_deg(self.current_position, waypoint)
            
#             # คำนวณความแตกต่างของมุม
#             angle_diff_deg = abs(self.angle_difference_deg(waypoint_bearing_deg, self.current_heading_deg))
            
#             # ตรวจสอบว่า waypoint อยู่หน้ารถหรือไม่ (มุมไม่เกิน ±90°)
#             if angle_diff_deg <= 90.0:  # ±90 degrees
#                 if distance < min_distance:
#                     min_distance = distance
#                     nearest_index = i
        
#         return nearest_index
    
#     def find_lookahead_point(self) -> Tuple[Optional[Tuple[float, float]], int]:
#         """หา lookahead point สำหรับ Pure Pursuit (เฉพาะที่อยู่หน้ารถ)"""
#         if not self.current_position or not self.waypoints:
#             return None, 0
        
#         # เริ่มจาก waypoint ปัจจุบันหรือ waypoint ที่ใกล้ที่สุดที่อยู่หน้ารถ
#         start_index = max(self.current_waypoint_index, self.find_nearest_forward_waypoint())
        
#         for i in range(start_index, len(self.waypoints)):
#             waypoint = self.waypoints[i]
            
#             # คำนวณระยะทาง
#             distance = self.calculate_distance(self.current_position, waypoint)
            
#             # คำนวณ compass bearing ไปยัง waypoint
#             waypoint_bearing_deg = self.calculate_target_heading_deg(self.current_position, waypoint)
            
#             # คำนวณความแตกต่างของมุม
#             angle_diff_deg = abs(self.angle_difference_deg(waypoint_bearing_deg, self.current_heading_deg))
            
#             # ตรวจสอบว่าอยู่หน้ารถและระยะทางเหมาะสม
#             if angle_diff_deg <= 90.0 and distance >= self.lookahead_distance:
#                 return waypoint, i
        
#         # หากไม่เจอ ให้หา waypoint สุดท้ายที่อยู่หน้ารถ
#         for i in range(len(self.waypoints) - 1, -1, -1):
#             waypoint = self.waypoints[i]
#             waypoint_bearing_deg = self.calculate_target_heading_deg(self.current_position, waypoint)
#             angle_diff_deg = abs(self.angle_difference_deg(waypoint_bearing_deg, self.current_heading_deg))
            
#             if angle_diff_deg <= 90.0:
#                 return waypoint, i
        
#         return None, 0
    
#     def pure_pursuit_control(self, target_point: Tuple[float, float]) -> float:
#         """คำนวณ steering angle ด้วย Pure Pursuit Algorithm (ใช้ compass bearing)"""
#         if not self.current_position:
#             return 0.0
        
#         # คำนวณ compass bearing ไปยังเป้าหมาย
#         target_bearing_deg = self.calculate_target_heading_deg(self.current_position, target_point)
        
#         # คำนวณความแตกต่างของมุม (alpha)
#         alpha_deg = self.angle_difference_deg(target_bearing_deg, self.current_heading_deg)
#         alpha_rad = math.radians(alpha_deg)
        
#         # คำนวณ steering angle ด้วยสูตร Pure Pursuit
#         lookahead_distance = self.calculate_distance(self.current_position, target_point)
#         if lookahead_distance > 0.1:
#             steering_angle = math.atan2(2.0 * self.wheelbase * math.sin(alpha_rad), lookahead_distance)
#         else:
#             steering_angle = 0.0
        
#         # จำกัดมุมพวงมาลัย
#         steering_angle = max(-self.max_steering_angle, 
#                            min(self.max_steering_angle, steering_angle))
        
#         return steering_angle
    
#     def check_waypoint_reached(self) -> bool:
#         """ตรวจสอบว่าถึง waypoint แล้วหรือไม่"""
#         if not self.current_position or self.current_waypoint_index >= len(self.waypoints):
#             return False
        
#         current_waypoint = self.waypoints[self.current_waypoint_index]
#         distance = self.calculate_distance(self.current_position, current_waypoint)
        
#         return distance < 1.0  # ถือว่าถึงแล้วถ้าระยะทางน้อยกว่า 1 เมตร
    
#     def get_predefined_steering(self) -> Optional[float]:
#         """ดึงค่า steering ที่กำหนดไว้ในไฟล์"""
#         if self.current_waypoint_index >= len(self.waypoints) - 1:
#             return None
        
#         from_wp = self.current_waypoint_index
#         to_wp = self.current_waypoint_index + 1
        
#         return self.steering_commands.get((from_wp, to_wp))
    
#     def control_loop(self):
#         """Main control loop with forward waypoint selection"""
#         if not self.current_position or not self.waypoints:
#             return
        
#         # หา waypoint ที่ใกล้ที่สุดที่อยู่หน้ารถ
#         nearest_forward_waypoint = self.find_nearest_forward_waypoint()
        
#         # อัปเดต current_waypoint_index ถ้าจำเป็น
#         if nearest_forward_waypoint > self.current_waypoint_index:
#             self.current_waypoint_index = nearest_forward_waypoint
        
#         # ตรวจสอบว่าถึง waypoint แล้วหรือไม่
#         if self.check_waypoint_reached():
#             self.current_waypoint_index += 1
#             self.use_pure_pursuit = False
#             self.get_logger().info(f'Reached waypoint {self.current_waypoint_index}')
        
#         # ตรวจสอบว่าจบเส้นทางแล้วหรือไม่
#         if self.current_waypoint_index >= len(self.waypoints) - 1:
#             self.publish_stop_command()
#             self.get_logger().info('Path completed!')
#             return
        
#         steering_angle = 0.0
        
#         if self.use_pure_pursuit:
#             # ใช้ Pure Pursuit สำหรับการเข้าสู่เส้นทาง
#             lookahead_point, target_index = self.find_lookahead_point()
#             if lookahead_point:
#                 steering_angle = self.pure_pursuit_control(lookahead_point)
#                 self.target_index = target_index
#         else:
#             # ใช้ค่า steering ที่กำหนดไว้
#             predefined_steering = self.get_predefined_steering()
#             if predefined_steering is not None:
#                 steering_angle = predefined_steering
#             else:
#                 # ถ้าไม่มีค่ากำหนด กลับไปใช้ Pure Pursuit
#                 self.use_pure_pursuit = True
#                 lookahead_point, target_index = self.find_lookahead_point()
#                 if lookahead_point:
#                     steering_angle = self.pure_pursuit_control(lookahead_point)
        
#         # ส่งคำสั่งควบคุม
#         self.publish_control_commands(steering_angle)
    
#     def publish_control_commands(self, steering_angle: float):
#         """ส่งคำสั่งควบคุมรถ"""
#         # ส่งคำสั่งความเร็ว
#         twist_msg = Twist()
#         twist_msg.linear.x = self.fixed_speed
#         twist_msg.angular.z = steering_angle
#         self.cmd_publisher.publish(twist_msg)
        
#         # ส่งคำสั่ง steering angle แยกต่างหาก
#         steering_msg = Float32()
#         steering_msg.data = steering_angle
#         self.steering_publisher.publish(steering_msg)
        
#         # Log ข้อมูล
#         if self.current_position:
#             self.get_logger().info(
#                 f'Control: speed={self.fixed_speed:.2f} m/s, '
#                 f'steering={math.degrees(steering_angle):.1f}°, '
#                 f'waypoint={self.current_waypoint_index}/{len(self.waypoints)-1}, '
#                 f'mode={"Pure Pursuit" if self.use_pure_pursuit else "Predefined"}, '
#                 f'heading={self.current_heading_deg:.1f}° '
#                 f'(GPS: {self.gps_heading_deg:.1f}°, IMU: {self.imu_heading_deg:.1f}°), '
#                 f'pos=({self.current_position[0]:.2f}, {self.current_position[1]:.2f})'
#             )
    
#     def publish_stop_command(self):
#         """ส่งคำสั่งหยุดรถ"""
#         twist_msg = Twist()
#         twist_msg.linear.x = 0.0
#         twist_msg.angular.z = 0.0
#         self.cmd_publisher.publish(twist_msg)
        
#         steering_msg = Float32()
#         steering_msg.data = 0.0
#         self.steering_publisher.publish(steering_msg)
    
#     def __del__(self):
#         """Cleanup resources"""
#         if hasattr(self, 'ser') and self.ser and self.ser.is_open:
#             self.ser.close()
#         if hasattr(self, 'ntrip_socket') and self.ntrip_socket:
#             self.ntrip_socket.close()

# def main(args=None):
#     rclpy.init(args=args)
#     controller = IntegratedPathController()
    
#     try:
#         rclpy.spin(controller)
#     except KeyboardInterrupt:
#         controller.get_logger().info('Controller stopped by user')
#     finally:
#         controller.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# ----------------------------------------new controllor-------------------------------------------------

# #!/usr/bin/env python3
# import serial
# import pynmea2
# import socket
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
# from std_msgs.msg import Float32, String, Float64MultiArray
# from geometry_msgs.msg import Point, Twist
# from rcl_interfaces.msg import ParameterDescriptor
# import math
# import pyproj
# from collections import deque
# import time
# import glob
# import os
# import struct
# import json
# import numpy as np
# import csv

# # เพิ่ม pyubx2 สำหรับ UBX parsing ที่ดีขึ้น
# try:
#     from pyubx2 import UBXReader, UBXMessage, UBX_MSGIDS
#     PYUBX2_AVAILABLE = True
#     print("✅ pyubx2 available - using enhanced UBX parsing")
# except ImportError:
#     PYUBX2_AVAILABLE = False
#     print("⚠️ pyubx2 not available - install with: pip3 install pyubx2")

# # Enhanced IMU Heading Filter (เหมือนเดิม)
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

# # ✅ Enhanced Waypoint Manager with Starting Point Selection
# class WaypointManager:
#     def __init__(self, csv_file_path, start_waypoint_index=0):
#         self.waypoints = []
#         self.current_waypoint_index = start_waypoint_index  # ✅ เริ่มจากจุดที่กำหนด
#         self.load_waypoints_from_csv(csv_file_path)
        
#         # กำหนดช่วง waypoint และประเภทเส้นทาง
#         self.path_segments = [
#             {'start': 0, 'end': 240, 'type': 'straight', 'direction': 'S-N', 'target_heading': 0.0},      # waypoint 1-240 (North = 0°)
#             {'start': 241, 'end': 256, 'type': 'curve', 'direction': 'curve1', 'target_heading': None},    # waypoint 245-260
#             {'start': 257, 'end': 315, 'type': 'straight', 'direction': 'E-W', 'target_heading': 90.0},   # waypoint 261-315 (East = 90°)
#             {'start': 316, 'end': 383, 'type': 'curve', 'direction': 'curve2', 'target_heading': None},    # waypoint 316-390
#             {'start': 384, 'end': 496, 'type': 'straight', 'direction': 'N-S', 'target_heading': 180.0},  # waypoint 391-500 (South = 180°)
#             {'start': 497, 'end': 571, 'type': 'curve', 'direction': 'curve3', 'target_heading': None},    # waypoint 501-579
#             {'start': 572, 'end': 622, 'type': 'straight', 'direction': 'W-E', 'target_heading': 270.0},  # waypoint 580-630 (West = 270°)
#             {'start': 623, 'end': 642, 'type': 'curve', 'direction': 'curve4', 'target_heading': None},    # waypoint 631-646
#             {'start': 643, 'end': 654, 'type': 'straight', 'direction': 'S-N', 'target_heading': 0.0}     # waypoint 647-659 (North = 0°)
#         ]
        
#         # ✅ กำหนดรัศมีสำหรับแต่ละทางโค้ง (หน่วย: เมตร)
#         self.curve_radii = {
#             'curve1': 15.0,  # รัศมี 15 เมตร สำหรับโค้งที่ 1
#             'curve2': 20.0,  # รัศมี 20 เมตร สำหรับโค้งที่ 2
#             'curve3': 18.0,  # รัศมี 18 เมตร สำหรับโค้งที่ 3
#             'curve4': 12.0   # รัศมี 12 เมตร สำหรับโค้งที่ 4
#         }
        
#         # ตรวจสอบว่าจุดเริ่มต้นอยู่ในช่วงที่ถูกต้อง
#         if self.current_waypoint_index >= len(self.waypoints):
#             self.current_waypoint_index = 0
#             print(f"⚠️ Starting waypoint index out of range, reset to 0")
        
#         print(f"✅ Starting navigation from waypoint {self.current_waypoint_index + 1}")
    
#     def load_waypoints_from_csv(self, csv_file_path):
#         """โหลด waypoints จากไฟล์ CSV"""
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
    
#     def set_starting_waypoint(self, waypoint_index):
#         """✅ กำหนดจุดเริ่มต้นใหม่"""
#         if 0 <= waypoint_index < len(self.waypoints):
#             self.current_waypoint_index = waypoint_index
#             print(f"✅ Starting waypoint set to {waypoint_index + 1}")
#             return True
#         else:
#             print(f"❌ Invalid waypoint index: {waypoint_index}")
#             return False
    
#     def get_current_segment_type(self):
#         """ได้ประเภทของเส้นทางปัจจุบัน"""
#         for segment in self.path_segments:
#             if segment['start'] <= self.current_waypoint_index <= segment['end']:
#                 return segment['type'], segment['direction']
#         return 'straight', 'unknown'
    
#     def get_curve_radius(self, curve_direction):
#         """✅ ได้รัศมีของทางโค้ง"""
#         return self.curve_radii.get(curve_direction, 15.0)  # default 15 เมตร
    
#     def get_current_waypoint(self):
#         """ได้ waypoint ปัจจุบัน"""
#         if self.current_waypoint_index < len(self.waypoints):
#             return self.waypoints[self.current_waypoint_index]
#         return None
    
#     def get_next_waypoint(self):
#         """ได้ waypoint ถัดไป"""
#         next_index = self.current_waypoint_index + 1
#         if next_index < len(self.waypoints):
#             return self.waypoints[next_index]
#         return None
    
#     def advance_waypoint(self):
#         """เลื่อนไปยัง waypoint ถัดไป"""
#         if self.current_waypoint_index < len(self.waypoints) - 1:
#             self.current_waypoint_index += 1
#             return True
#         return False
    
#     def get_progress_info(self):
#         """✅ ได้ข้อมูลความคืบหน้า"""
#         return {
#             'current': self.current_waypoint_index + 1,
#             'total': len(self.waypoints),
#             'progress_percent': ((self.current_waypoint_index + 1) / len(self.waypoints)) * 100
#         }

# # Pure Pursuit Controller
# class PurePursuitController:
#     def __init__(self, lookahead_distance=2.0):
#         self.lookahead_distance = lookahead_distance
    
#     def calculate_steering_angle(self, current_pos, current_heading, target_pos, wheelbase):
#         """คำนวณมุมเลี้ยวสำหรับ Pure Pursuit"""
#         # คำนวณระยะทางและมุมไปยังเป้าหมาย
#         dx = target_pos['x'] - current_pos['x']
#         dy = target_pos['y'] - current_pos['y']
        
#         target_heading = math.degrees(math.atan2(dy, dx))
#         heading_error = self.normalize_angle(target_heading - current_heading)
        
#         distance = math.sqrt(dx**2 + dy**2)
        
#         # Pure Pursuit formula
#         alpha = math.radians(heading_error)
#         steering_angle = math.atan2(2 * wheelbase * math.sin(alpha), distance)
        
#         return steering_angle
    
#     def normalize_angle(self, angle):
#         """Normalize angle to [-180, 180]"""
#         while angle > 180:
#             angle -= 360
#         while angle < -180:
#             angle += 360
#         return angle

# # ✅ Enhanced Constant Curvature Controller with Radius Control
# class ConstantCurvatureController:
#     def __init__(self):
#         self.curvature_history = deque(maxlen=5)
    
#     def calculate_steering_angle_with_radius(self, curve_radius, linear_velocity, wheelbase):
#         """✅ คำนวณมุมเลี้ยวจากรัศมีที่กำหนด"""
#         # คำนวณ curvature จากรัศมี: κ = 1/R
#         curvature = 1.0 / curve_radius
        
#         # คำนวณมุมเลี้ยวจาก curvature และ bicycle model
#         # δ = atan(L * κ) โดย L = wheelbase, κ = curvature
#         steering_angle = math.atan(wheelbase * curvature)
        
#         return steering_angle
    
#     def calculate_steering_angle(self, waypoints_segment, current_pos, wheelbase):
#         """คำนวณมุมเลี้ยวสำหรับ Constant Curvature (วิธีเดิม)"""
#         if len(waypoints_segment) < 3:
#             return 0.0
        
#         # คำนวณ curvature จาก 3 จุด
#         curvature = self.calculate_curvature(waypoints_segment)
#         self.curvature_history.append(curvature)
        
#         # ใช้ค่าเฉลี่ยของ curvature
#         avg_curvature = np.mean(list(self.curvature_history))
        
#         # คำนวณมุมเลี้ยวจาก curvature
#         steering_angle = math.atan(wheelbase * avg_curvature)
        
#         return steering_angle
    
#     def calculate_curvature(self, points):
#         """คำนวณ curvature จาก 3 จุด"""
#         if len(points) < 3:
#             return 0.0
        
#         p1, p2, p3 = points[0], points[1], points[2]
        
#         # คำนวณ curvature โดยใช้สูตร
#         x1, y1 = p1['x'], p1['y']
#         x2, y2 = p2['x'], p2['y']
#         x3, y3 = p3['x'], p3['y']
        
#         # Area of triangle
#         area = 0.5 * abs((x2-x1)*(y3-y1) - (x3-x1)*(y2-y1))
        
#         # Side lengths
#         a = math.sqrt((x2-x3)**2 + (y2-y3)**2)
#         b = math.sqrt((x1-x3)**2 + (y1-y3)**2)
#         c = math.sqrt((x1-x2)**2 + (y1-y2)**2)
        
#         if a * b * c == 0:
#             return 0.0
        
#         # Curvature = 4 * Area / (a * b * c)
#         curvature = 4 * area / (a * b * c)
        
#         return curvature

# # Bicycle Kinematic Model
# class BicycleKinematicModel:
#     def __init__(self, wheelbase):
#         self.wheelbase = wheelbase
    
#     def calculate_angular_velocity(self, linear_velocity, steering_angle):
#         """คำนวณ angular velocity จาก bicycle model"""
#         if abs(steering_angle) < 1e-6:  # ป้องกันการหารด้วยศูนย์
#             return 0.0
        
#         angular_velocity = (linear_velocity * math.tan(steering_angle)) / self.wheelbase
#         return angular_velocity
    
#     def limit_steering_angle(self, steering_angle, max_steering_angle):
#         """จำกัดมุมเลี้ยวให้อยู่ในขอบเขต"""
#         return max(-max_steering_angle, min(max_steering_angle, steering_angle))

# # ✅ Main Enhanced GNSS Node with Fixed Speed Navigation
# class EnhancedZEDf9rGNSSWithNavigation(Node):
#     def __init__(self):
#         super().__init__('enhanced_zedf9r_gnss_with_navigation')
        
#         # ✅ Vehicle parameters
#         self.declare_vehicle_parameters()
        
#         # ROS2 Publishers
#         self.gnss_publisher = self.create_publisher(NavSatFix, '/navigation/gnss', 10)
#         self.xy_publisher = self.create_publisher(Float64MultiArray, 'Realtime_XY', 10)
#         self.raw_gps_publisher = self.create_publisher(Float64MultiArray, 'Raw_GPS', 10)
#         self.xy_debug_publisher = self.create_publisher(Point, '/navigation/xy_debug', 10)
#         self.heading_publisher = self.create_publisher(Float32, '/navigation/heading', 10)
#         self.heading_debug_publisher = self.create_publisher(String, '/navigation/heading_debug', 10)
#         self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.navigation_debug_publisher = self.create_publisher(String, '/navigation/debug', 10)
        
#         # Serial configuration (เหมือนเดิม)
#         self.possible_serial_ports = [
#             '/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1'
#         ]
#         self.serial_port = None
#         self.ser = None
        
#         # NTRIP configuration (เหมือนเดิม)
#         self.ntrip_server_ip = "110.78.0.54"
#         self.ntrip_server_port = 2116
#         self.ntrip_username = "1118600009224"
#         self.ntrip_password = "CK79"
#         self.mount_point = "VRS_RTCM32"
        
#         # Coordinate transformation (เหมือนเดิม)
#         self.transformer = pyproj.Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)
        
#         # Position tracking
#         self.position_history = deque(maxlen=15)
#         self.min_movement_for_heading = 0.25
#         self.current_position = {'x': 0.0, 'y': 0.0}
        
#         # Reference point (เหมือนเดิม)
#         self.reference_point = (13.650748, 100.492985)
#         self.reference_utm = None
        
#         # UTM coordinates (เหมือนเดิม)
#         self.TARGET_UTM_X = 661452.0
#         self.TARGET_UTM_Y = 1509601.0
#         self.utm_offset_x = 0.0
#         self.utm_offset_y = 0.0
        
#         # Enhanced Heading management (เหมือนเดิม)
#         self.imu_heading = None
#         self.nav_att_heading = None
#         self.nav_att_yaw = None
#         self.gps_heading = None
#         self.final_heading = None
#         self.heading_source = "NONE"
        
#         self.imu_heading_time = 0
#         self.nav_att_time = 0
#         self.gps_heading_time = 0
#         self.heading_timeout = 8.0
        
#         # ZED-F9R specific data (เหมือนเดิม)
#         self.esf_status = None
#         self.nav_att = None
#         self.esf_alg = None
#         self.fusion_mode = "NONE"
#         self.imu_calibration_status = "UNKNOWN"
        
#         # pyubx2 UBX parsing (เหมือนเดิม)
#         self.ubx_reader = None
#         self.ubx_buffer = bytearray()
        
#         # Enhanced Heading filtering (เหมือนเดิม)
#         self.heading_filter = EnhancedIMUHeadingFilter()
        
#         # ✅ Navigation Components with Enhanced Features
#         # เปลี่ยน path ให้ตรงกับไฟล์ของคุณ
#         csv_file_path = '/home/inc/ros2_ws/waypointfile/waypoints3xy.csv'  # แก้ไข path ให้ถูกต้อง
#         start_waypoint = 0  # ✅ เริ่มจาก waypoint ที่ 1 (index 0)
        
#         self.waypoint_manager = WaypointManager(csv_file_path, start_waypoint)
#         self.pure_pursuit = PurePursuitController(lookahead_distance=3.0)
#         self.curvature_controller = ConstantCurvatureController()
#         self.bicycle_model = BicycleKinematicModel(wheelbase=1.67)  # จะอัพเดทจาก parameter
        
#         # Navigation state
#         self.navigation_active = True
#         self.waypoint_reached_threshold = 1.0  # เมตร
        
#         # ✅ Fixed Speed Configuration
#         self.FIXED_SPEED = 1.0  # ความเร็วคงที่ 1 m/s
        
#         # Performance stats (เหมือนเดิม)
#         self.stats = {
#             'imu_count': 0,
#             'nav_att_count': 0,
#             'nav_att_yaw_count': 0,
#             'gps_count': 0,
#             'position_count': 0,
#             'esf_meas_count': 0,
#             'esf_status_count': 0,
#             'pyubx2_success': 0,
#             'pyubx2_errors': 0,
#             'filtered_count': 0,
#             'navigation_commands': 0
#         }
        
#         # Initialize coordinate system (เหมือนเดิม)
#         self.init_track_coordinate_system()
        
#         # Initialize serial connection (เหมือนเดิม)
#         self.init_serial_connection()
        
#         if self.ser and self.ser.is_open:
#             self.ntrip_socket = self.connect_to_ntrip()
            
#             # Initialize pyubx2 if available (เหมือนเดิม)
#             if PYUBX2_AVAILABLE:
#                 self.init_pyubx2()
            
#             # Configure ZED-F9R for enhanced IMU (เหมือนเดิม)
#             self.configure_enhanced_zed_f9r()
            
#             # Main processing timer
#             self.create_timer(0.1, self.main_loop)
            
#             self.get_logger().info('✅ Enhanced ZED-F9R GNSS with Fixed Speed Navigation initialized')
#             self.get_logger().info(f'🔧 pyubx2: {"Available" if PYUBX2_AVAILABLE else "Not available"}')
#             self.get_logger().info(f'🗺️ Loaded {len(self.waypoint_manager.waypoints)} waypoints')
#             self.get_logger().info(f'🚀 Fixed Speed: {self.FIXED_SPEED} m/s')
            
#             # Log vehicle parameters
#             self.log_vehicle_parameters()
            
#             # Log curve radii
#             self.log_curve_radii()
#         else:
#             self.get_logger().error('❌ Failed to initialize')
    
#     def declare_vehicle_parameters(self):
#         """✅ ประกาศ Vehicle Parameters"""
        
#         # Wheelbase parameter
#         wheelbase_descriptor = ParameterDescriptor(
#             description='ฐานล้อของรถ (ระยะห่างระหว่างเพลาหน้าและหลัง) ในหน่วยเมตร'
#         )
#         self.declare_parameter('wheelbase', 1.67, wheelbase_descriptor)
        
#         # Max steering angle parameter
#         steering_descriptor = ParameterDescriptor(
#             description='มุมเลี้ยวสูงสุดของรถในหน่วย radians (50 degrees = 0.873 rad)'
#         )
#         self.declare_parameter('max_steering_angle', 0.873, steering_descriptor)
        
#         # ✅ Fixed speed parameter - ตั้งค่าเป็น 1.0 m/s
#         speed_descriptor = ParameterDescriptor(
#             description='ความเร็วคงที่ของรถในหน่วย m/s (ตั้งค่าเป็น 1.0 m/s)'
#         )
#         self.declare_parameter('fixed_speed', 1.0, speed_descriptor)
        
#         # ✅ Starting waypoint parameter
#         start_waypoint_descriptor = ParameterDescriptor(
#             description='จุดเริ่มต้นของการนำทาง (waypoint index, เริ่มจาก 0)'
#         )
#         self.declare_parameter('start_waypoint_index', 0, start_waypoint_descriptor)
        
#         # ✅ Curve radius parameters
#         curve1_radius_descriptor = ParameterDescriptor(
#             description='รัศมีของทางโค้งที่ 1 ในหน่วยเมตร'
#         )
#         self.declare_parameter('curve1_radius', 15.0, curve1_radius_descriptor)
        
#         curve2_radius_descriptor = ParameterDescriptor(
#             description='รัศมีของทางโค้งที่ 2 ในหน่วยเมตร'
#         )
#         self.declare_parameter('curve2_radius', 20.0, curve2_radius_descriptor)
        
#         curve3_radius_descriptor = ParameterDescriptor(
#             description='รัศมีของทางโค้งที่ 3 ในหน่วยเมตร'
#         )
#         self.declare_parameter('curve3_radius', 18.0, curve3_radius_descriptor)
        
#         curve4_radius_descriptor = ParameterDescriptor(
#             description='รัศมีของทางโค้งที่ 4 ในหน่วยเมตร'
#         )
#         self.declare_parameter('curve4_radius', 12.0, curve4_radius_descriptor)
        
#         # Angular velocity limits
#         max_angular_vel_descriptor = ParameterDescriptor(
#             description='ความเร็วเชิงมุมสูงสุดในหน่วย rad/s'
#         )
#         self.declare_parameter('max_angular_velocity', 1.0, max_angular_vel_descriptor)
        
#         self.get_logger().info('🌟 Vehicle parameters declared successfully!')
    
#     def get_vehicle_parameters(self):
#         """✅ ดึงค่า Vehicle Parameters ปัจจุบัน"""
#         try:
#             wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
#             max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value
#             fixed_speed = self.get_parameter('fixed_speed').get_parameter_value().double_value
#             start_waypoint_index = self.get_parameter('start_waypoint_index').get_parameter_value().integer_value
#             max_angular_velocity = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
            
#             # ✅ ดึงค่ารัศมีของทางโค้ง
#             curve_radii = {
#                 'curve1': self.get_parameter('curve1_radius').get_parameter_value().double_value,
#                 'curve2': self.get_parameter('curve2_radius').get_parameter_value().double_value,
#                 'curve3': self.get_parameter('curve3_radius').get_parameter_value().double_value,
#                 'curve4': self.get_parameter('curve4_radius').get_parameter_value().double_value
#             }
            
#             return {
#                 'wheelbase': wheelbase,
#                 'max_steering_angle': max_steering_angle,
#                 'fixed_speed': fixed_speed,
#                 'start_waypoint_index': start_waypoint_index,
#                 'max_angular_velocity': max_angular_velocity,
#                 'curve_radii': curve_radii
#             }
#         except Exception as e:
#             self.get_logger().error(f'❌ Error getting vehicle parameters: {e}')
#             return {
#                 'wheelbase': 1.67,
#                 'max_steering_angle': 0.873,
#                 'fixed_speed': 1.0,
#                 'start_waypoint_index': 0,
#                 'max_angular_velocity': 1.0,
#                 'curve_radii': {
#                     'curve1': 15.0,
#                     'curve2': 20.0,
#                     'curve3': 18.0,
#                     'curve4': 12.0
#                 }
#             }
    
#     def log_vehicle_parameters(self):
#         """✅ แสดง Vehicle Parameters ปัจจุบัน"""
#         params = self.get_vehicle_parameters()
#         self.get_logger().info(
#             f'🚗 Vehicle Parameters:\n'
#             f'  🔧 Wheelbase: {params["wheelbase"]:.2f} m\n'
#             f'  🔧 Max Steering Angle: {params["max_steering_angle"]:.3f} rad ({math.degrees(params["max_steering_angle"]):.1f}°)\n'
#             f'  🔧 Fixed Speed: {params["fixed_speed"]:.2f} m/s\n'
#             f'  🔧 Start Waypoint: {params["start_waypoint_index"] + 1}\n'
#             f'  🔧 Max Angular Velocity: {params["max_angular_velocity"]:.2f} rad/s'
#         )
    
#     def log_curve_radii(self):
#         """✅ แสดงรัศมีของทางโค้ง"""
#         params = self.get_vehicle_parameters()
#         curve_radii = params['curve_radii']
        
#         self.get_logger().info(
#             f'🌀 Curve Radii Configuration:\n'
#             f'  🔄 Curve 1 (Waypoint 245-260): {curve_radii["curve1"]:.1f} m\n'
#             f'  🔄 Curve 2 (Waypoint 316-390): {curve_radii["curve2"]:.1f} m\n'
#             f'  🔄 Curve 3 (Waypoint 501-579): {curve_radii["curve3"]:.1f} m\n'
#             f'  🔄 Curve 4 (Waypoint 631-646): {curve_radii["curve4"]:.1f} m'
#         )
    
#     def navigation_control(self):
#         """✅ ระบบควบคุมการนำทางด้วยความเร็วคงที่และรัศมีที่กำหนด"""
#         if not self.navigation_active or self.final_heading is None:
#             return
        
#         # ได้ตำแหน่งปัจจุบัน
#         current_waypoint = self.waypoint_manager.get_current_waypoint()
#         if current_waypoint is None:
#             self.get_logger().info('🏁 Navigation completed - reached final waypoint')
#             self.navigation_active = False
#             self.send_stop_command()
#             return
        
#         # ตรวจสอบว่าถึง waypoint แล้วหรือยัง
#         distance_to_waypoint = math.sqrt(
#             (self.current_position['x'] - current_waypoint['x'])**2 + 
#             (self.current_position['y'] - current_waypoint['y'])**2
#         )
        
#         if distance_to_waypoint < self.waypoint_reached_threshold:
#             if self.waypoint_manager.advance_waypoint():
#                 progress = self.waypoint_manager.get_progress_info()
#                 self.get_logger().info(
#                     f'✅ Reached waypoint {progress["current"]-1} '
#                     f'({progress["progress_percent"]:.1f}% complete)'
#                 )
#             else:
#                 self.get_logger().info('🏁 Navigation completed')
#                 self.navigation_active = False
#                 self.send_stop_command()
#                 return
        
#         # ได้ประเภทเส้นทางปัจจุบัน
#         segment_type, direction = self.waypoint_manager.get_current_segment_type()
        
#         # ได้ parameters
#         params = self.get_vehicle_parameters()
        
#         # ✅ ใช้ความเร็วคงที่ 1 m/s
#         linear_velocity = self.FIXED_SPEED
        
#         # คำนวณคำสั่งควบคุมตามประเภทเส้นทาง
#         if segment_type == 'straight':
#             steering_angle = self.calculate_straight_path_control(current_waypoint, params)
#         else:  # curve
#             steering_angle = self.calculate_curve_path_control_with_radius(direction, params)
        
#         # จำกัดมุมเลี้ยว
#         steering_angle = self.bicycle_model.limit_steering_angle(
#             steering_angle, params['max_steering_angle']
#         )
        
#         # คำนวณ angular velocity ด้วย bicycle model
#         angular_velocity = self.bicycle_model.calculate_angular_velocity(
#             linear_velocity, steering_angle
#         )
        
#         # จำกัด angular velocity
#         angular_velocity = max(-params['max_angular_velocity'], 
#                              min(params['max_angular_velocity'], angular_velocity))
        
#         # สร้างและส่งคำสั่ง cmd_vel
#         cmd_vel = Twist()
#         cmd_vel.linear.x = linear_velocity
#         cmd_vel.angular.z = angular_velocity
        
#         self.cmd_vel_publisher.publish(cmd_vel)
#         self.stats['navigation_commands'] += 1
        
#         # Debug information
#         debug_msg = String()
#         progress = self.waypoint_manager.get_progress_info()
        
#         if segment_type == 'curve':
#             curve_radius = params['curve_radii'].get(direction, 15.0)
#             debug_msg.data = (
#                 f"Segment: {segment_type} ({direction}, R={curve_radius:.1f}m) | "
#                 f"Waypoint: {progress['current']}/{progress['total']} ({progress['progress_percent']:.1f}%) | "
#                 f"Distance: {distance_to_waypoint:.2f}m | "
#                 f"Steering: {math.degrees(steering_angle):.1f}° | "
#                 f"Speed: {linear_velocity:.2f}m/s | "
#                 f"Angular: {angular_velocity:.2f}rad/s"
#             )
#         else:
#             debug_msg.data = (
#                 f"Segment: {segment_type} ({direction}) | "
#                 f"Waypoint: {progress['current']}/{progress['total']} ({progress['progress_percent']:.1f}%) | "
#                 f"Distance: {distance_to_waypoint:.2f}m | "
#                 f"Steering: {math.degrees(steering_angle):.1f}° | "
#                 f"Speed: {linear_velocity:.2f}m/s | "
#                 f"Angular: {angular_velocity:.2f}rad/s"
#             )
        
#         self.navigation_debug_publisher.publish(debug_msg)
    
#     def calculate_straight_path_control(self, target_waypoint, params):
#         """คำนวณการควบคุมสำหรับเส้นทางตรงด้วย Pure Pursuit + Bicycle Model"""
#         steering_angle = self.pure_pursuit.calculate_steering_angle(
#             self.current_position,
#             self.final_heading,
#             target_waypoint,
#             params['wheelbase']
#         )
        
#         # ใช้ bicycle model เพื่อรักษาเส้นทางตรง
#         # ลดมุมเลี้ยวให้น้อยลงเพื่อให้เป็นเส้นตรงมากขึ้น
#         steering_angle *= 0.5  # ลดความแรงของการเลี้ยว
        
#         return steering_angle
    
#     def calculate_curve_path_control_with_radius(self, curve_direction, params):
#         """✅ คำนวณการควบคุมสำหรับเส้นทางโค้งด้วยรัศมีที่กำหนด"""
#         # ได้รัศมีของทางโค้งจาก parameters
#         curve_radius = params['curve_radii'].get(curve_direction, 15.0)
        
#         # คำนวณมุมเลี้ยวจากรัศมีที่กำหนด
#         steering_angle = self.curvature_controller.calculate_steering_angle_with_radius(
#             curve_radius,
#             self.FIXED_SPEED,
#             params['wheelbase']
#         )
        
#         return steering_angle
    
#     def send_stop_command(self):
#         """✅ ส่งคำสั่งหยุดรถ"""
#         cmd_vel = Twist()
#         cmd_vel.linear.x = 0.0
#         cmd_vel.angular.z = 0.0
#         self.cmd_vel_publisher.publish(cmd_vel)
#         self.get_logger().info('🛑 Stop command sent')
    
#     # ✅ เพิ่มฟังก์ชันสำหรับเปลี่ยนจุดเริ่มต้นระหว่างการทำงาน
#     def set_starting_waypoint_service(self, waypoint_index):
#         """✅ เปลี่ยนจุดเริ่มต้นระหว่างการทำงาน"""
#         if self.waypoint_manager.set_starting_waypoint(waypoint_index):
#             self.navigation_active = True
#             self.get_logger().info(f'🚀 Navigation restarted from waypoint {waypoint_index + 1}')
#             return True
#         return False
    
#     # เพิ่มฟังก์ชันที่ขาดหายไปจากโค้ดเดิม (init_track_coordinate_system, init_serial_connection, etc.)
#     def init_track_coordinate_system(self):
#         """Initialize coordinate system"""
#         if self.reference_utm is None:
#             self.reference_utm = self.transformer.transform(
#                 self.reference_point[1], self.reference_point[0]
#             )
        
#         self.utm_offset_x = self.TARGET_UTM_X - self.reference_utm[0]
#         self.utm_offset_y = self.TARGET_UTM_Y - self.reference_utm[1]
        
#         self.get_logger().info(f'🗺️ Track coordinate system initialized')
#         self.get_logger().info(f'📍 Reference UTM: {self.reference_utm}')
#         self.get_logger().info(f'🎯 Target UTM: ({self.TARGET_UTM_X}, {self.TARGET_UTM_Y})')
#         self.get_logger().info(f'📐 Offsets: ({self.utm_offset_x:.2f}, {self.utm_offset_y:.2f})')
    
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
        
#         if not self.ser or not self.ser.is_open:
#             self.get_logger().error('❌ No serial connection available')
    
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
#         import base64
#         credentials = f"{self.ntrip_username}:{self.ntrip_password}"
#         return base64.b64encode(credentials.encode()).decode()
    
#     def init_pyubx2(self):
#         """Initialize pyubx2 if available"""
#         if PYUBX2_AVAILABLE:
#             self.get_logger().info('✅ pyubx2 initialized for enhanced UBX parsing')
    
#     def configure_enhanced_zed_f9r(self):
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
            
#             # อัพเดทตำแหน่งปัจจุบัน
#             self.update_current_position()
            
#             # ควบคุมการนำทาง
#             self.navigation_control()
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Main loop error: {e}')
    
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
#         # ประมวลผลข้อมูล NMEA และ UBX
#         # (ใช้โค้ดเดิมที่มีอยู่แล้วในการประมวลผล GNSS data)
#         pass
    
#     def update_current_position(self):
#         """Update current position from GNSS data"""
#         # อัพเดทตำแหน่งปัจจุบันจากข้อมูล GNSS
#         # (ใช้โค้ดเดิมที่มีอยู่แล้วในการอัพเดทตำแหน่ง)
#         pass

# def main(args=None):
#     rclpy.init(args=args)
    
#     try:
#         node = EnhancedZEDf9rGNSSWithNavigation()
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         print('\n🛑 Navigation stopped by user')
#     except Exception as e:
#         print(f'❌ Error: {e}')
#     finally:
#         if 'node' in locals():
#             node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



# # ------------------------------------new controllerV2------------------------------
#!/usr/bin/env python3
# import serial
# import pynmea2
# import socket
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
# from std_msgs.msg import Float32, String, Float64MultiArray
# from geometry_msgs.msg import Point, Twist
# from rcl_interfaces.msg import ParameterDescriptor
# import math
# import pyproj
# from collections import deque
# import time
# import glob
# import os
# import struct
# import json
# import numpy as np
# import csv

# # เพิ่ม pyubx2 สำหรับ UBX parsing ที่ดีขึ้น
# try:
#     from pyubx2 import UBXReader, UBXMessage, UBX_MSGIDS
#     PYUBX2_AVAILABLE = True
#     print("✅ pyubx2 available - using enhanced UBX parsing")
# except ImportError:
#     PYUBX2_AVAILABLE = False
#     print("⚠️ pyubx2 not available - install with: pip3 install pyubx2")

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

# # ✅ Waypoint Manager with Starting Point Selection
# class WaypointManager:
#     def __init__(self, csv_file_path, start_waypoint_index=0):
#         self.waypoints = []
#         self.current_waypoint_index = start_waypoint_index
#         self.load_waypoints_from_csv(csv_file_path)
        
#         # กำหนดช่วง waypoint และประเภทเส้นทาง พร้อมทิศทางที่ต้องรักษา
#         self.path_segments = [
#             {'start': 0, 'end': 240, 'type': 'straight', 'direction': 'S-N', 'target_heading': 0.0},      # waypoint 1-240 (North = 0°)
#             {'start': 241, 'end': 256, 'type': 'curve', 'direction': 'curve1', 'target_heading': None},    # waypoint 245-260
#             {'start': 257, 'end': 315, 'type': 'straight', 'direction': 'E-W', 'target_heading': 90.0},   # waypoint 261-315 (East = 90°)
#             {'start': 316, 'end': 383, 'type': 'curve', 'direction': 'curve2', 'target_heading': None},    # waypoint 316-390
#             {'start': 384, 'end': 496, 'type': 'straight', 'direction': 'N-S', 'target_heading': 180.0},  # waypoint 391-500 (South = 180°)
#             {'start': 497, 'end': 571, 'type': 'curve', 'direction': 'curve3', 'target_heading': None},    # waypoint 501-579
#             {'start': 572, 'end': 622, 'type': 'straight', 'direction': 'W-E', 'target_heading': 270.0},  # waypoint 580-630 (West = 270°)
#             {'start': 623, 'end': 642, 'type': 'curve', 'direction': 'curve4', 'target_heading': None},    # waypoint 631-646
#             {'start': 643, 'end': 654, 'type': 'straight', 'direction': 'S-N', 'target_heading': 0.0}     # waypoint 647-659 (North = 0°)
#         ]
        
#         # กำหนดรัศมีสำหรับแต่ละทางโค้ง
#         self.curve_radii = {
#             'curve1': 15.0,  # รัศมี 15 เมตร สำหรับโค้งที่ 1
#             'curve2': 20.0,  # รัศมี 20 เมตร สำหรับโค้งที่ 2
#             'curve3': 18.0,  # รัศมี 18 เมตร สำหรับโค้งที่ 3
#             'curve4': 12.0   # รัศมี 12 เมตร สำหรับโค้งที่ 4
#         }
        
#         # ตรวจสอบว่าจุดเริ่มต้นอยู่ในช่วงที่ถูกต้อง
#         if self.current_waypoint_index >= len(self.waypoints):
#             self.current_waypoint_index = 0
#             print(f"⚠️ Starting waypoint index out of range, reset to 0")
        
#         print(f"✅ Starting navigation from waypoint {self.current_waypoint_index + 1}")
    
#     def load_waypoints_from_csv(self, csv_file_path):
#         """โหลด waypoints จากไฟล์ CSV"""
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
    
#     def set_starting_waypoint(self, waypoint_index):
#         """กำหนดจุดเริ่มต้นใหม่"""
#         if 0 <= waypoint_index < len(self.waypoints):
#             self.current_waypoint_index = waypoint_index
#             print(f"✅ Starting waypoint set to {waypoint_index + 1}")
#             return True
#         else:
#             print(f"❌ Invalid waypoint index: {waypoint_index}")
#             return False
    
#     def get_current_segment_info(self):
#         """ได้ข้อมูลของเส้นทางปัจจุบัน"""
#         for segment in self.path_segments:
#             if segment['start'] <= self.current_waypoint_index <= segment['end']:
#                 return segment
#         return {'type': 'straight', 'direction': 'unknown', 'target_heading': 0.0}
    
#     def get_curve_radius(self, curve_direction):
#         """ได้รัศมีของทางโค้ง"""
#         return self.curve_radii.get(curve_direction, 15.0)  # default 15 เมตร
    
#     def get_current_waypoint(self):
#         """ได้ waypoint ปัจจุบัน"""
#         if self.current_waypoint_index < len(self.waypoints):
#             return self.waypoints[self.current_waypoint_index]
#         return None
    
#     def get_next_waypoint(self):
#         """ได้ waypoint ถัดไป"""
#         next_index = self.current_waypoint_index + 1
#         if next_index < len(self.waypoints):
#             return self.waypoints[next_index]
#         return None
    
#     def advance_waypoint(self):
#         """เลื่อนไปยัง waypoint ถัดไป"""
#         if self.current_waypoint_index < len(self.waypoints) - 1:
#             self.current_waypoint_index += 1
#             return True
#         return False
    
#     def get_progress_info(self):
#         """ได้ข้อมูลความคืบหน้า"""
#         return {
#             'current': self.current_waypoint_index + 1,
#             'total': len(self.waypoints),
#             'progress_percent': ((self.current_waypoint_index + 1) / len(self.waypoints)) * 100
#         }

# # ✅ Bicycle Kinematic Model for Straight Path Control
# class BicycleKinematicModel:
#     def __init__(self, wheelbase):
#         self.wheelbase = wheelbase
    
#     def calculate_angular_velocity(self, linear_velocity, steering_angle):
#         """คำนวณ angular velocity จาก bicycle model"""
#         if abs(steering_angle) < 1e-6:  # ป้องกันการหารด้วยศูนย์
#             return 0.0
        
#         angular_velocity = (linear_velocity * math.tan(steering_angle)) / self.wheelbase
#         return angular_velocity
    
#     def limit_steering_angle(self, steering_angle, max_steering_angle):
#         """จำกัดมุมเลี้ยวให้อยู่ในขอบเขต"""
#         return max(-max_steering_angle, min(max_steering_angle, steering_angle))
    
#     def calculate_heading_correction_steering(self, current_heading, target_heading, kp=0.02):
#         """✅ คำนวณมุมเลี้ยวเพื่อแก้ไขทิศทางในทางตรง (bicycle model)"""
#         # คำนวณความผิดพลาดของทิศทาง
#         heading_error = self.normalize_angle_difference(target_heading - current_heading)
        
#         # ใช้ proportional control สำหรับการแก้ไขทิศทาง
#         # kp ต่ำ = การแก้ไขนุ่มนวล, kp สูง = การแก้ไขรวดเร็ว
#         steering_angle = kp * math.radians(heading_error)
        
#         return steering_angle
    
#     def normalize_angle_difference(self, angle_diff):
#         """ปรับความแตกต่างของมุมให้อยู่ในช่วง [-180, 180]"""
#         while angle_diff > 180:
#             angle_diff -= 360
#         while angle_diff < -180:
#             angle_diff += 360
#         return angle_diff

# # Pure Pursuit Controller
# class PurePursuitController:
#     def __init__(self, lookahead_distance=2.0):
#         self.lookahead_distance = lookahead_distance
    
#     def calculate_steering_angle(self, current_pos, current_heading, target_pos, wheelbase):
#         """คำนวณมุมเลี้ยวสำหรับ Pure Pursuit"""
#         # คำนวณระยะทางและมุมไปยังเป้าหมาย
#         dx = target_pos['x'] - current_pos['x']
#         dy = target_pos['y'] - current_pos['y']
        
#         target_heading = math.degrees(math.atan2(dx, dy))
#         heading_error = self.normalize_angle(target_heading - current_heading)
        
#         distance = math.sqrt(dx**2 + dy**2)
        
#         # Pure Pursuit formula
#         alpha = math.radians(heading_error)
#         steering_angle = math.atan2(2 * wheelbase * math.sin(alpha), distance)
        
#         return steering_angle
    
#     def normalize_angle(self, angle):
#         """Normalize angle to [-180, 180]"""
#         while angle > 180:
#             angle -= 360
#         while angle < -180:
#             angle += 360
#         return angle

# # Constant Curvature Controller
# class ConstantCurvatureController:
#     def __init__(self):
#         self.curvature_history = deque(maxlen=5)
    
#     def calculate_steering_angle_with_radius(self, curve_radius, linear_velocity, wheelbase):
#         """คำนวณมุมเลี้ยวจากรัศมีที่กำหนด"""
#         # คำนวณ curvature จากรัศมี: κ = 1/R
#         curvature = 1.0 / curve_radius
        
#         # คำนวณมุมเลี้ยวจาก curvature และ bicycle model
#         # δ = atan(L * κ) โดย L = wheelbase, κ = curvature
#         steering_angle = math.atan(wheelbase * curvature)
        
#         return steering_angle
    
#     def calculate_steering_angle(self, waypoints_segment, current_pos, wheelbase):
#         """คำนวณมุมเลี้ยวสำหรับ Constant Curvature (วิธีเดิม)"""
#         if len(waypoints_segment) < 3:
#             return 0.0
        
#         # คำนวณ curvature จาก 3 จุด
#         curvature = self.calculate_curvature(waypoints_segment)
#         self.curvature_history.append(curvature)
        
#         # ใช้ค่าเฉลี่ยของ curvature
#         avg_curvature = np.mean(list(self.curvature_history))
        
#         # คำนวณมุมเลี้ยวจาก curvature
#         steering_angle = math.atan(wheelbase * avg_curvature)
        
#         return steering_angle
    
#     def calculate_curvature(self, points):
#         """คำนวณ curvature จาก 3 จุด"""
#         if len(points) < 3:
#             return 0.0
        
#         p1, p2, p3 = points[0], points[1], points[2]
        
#         # คำนวณ curvature โดยใช้สูตร
#         x1, y1 = p1['x'], p1['y']
#         x2, y2 = p2['x'], p2['y']
#         x3, y3 = p3['x'], p3['y']
        
#         # Area of triangle
#         area = 0.5 * abs((x2-x1)*(y3-y1) - (x3-x1)*(y2-y1))
        
#         # Side lengths
#         a = math.sqrt((x2-x3)**2 + (y2-y3)**2)
#         b = math.sqrt((x1-x3)**2 + (y1-y3)**2)
#         c = math.sqrt((x1-x2)**2 + (y1-y2)**2)
        
#         if a * b * c == 0:
#             return 0.0
        
#         # Curvature = 4 * Area / (a * b * c)
#         curvature = 4 * area / (a * b * c)
        
#         return curvature

# # ✅ Main Enhanced GNSS Node with Bicycle Model Navigation
# class EnhancedZEDf9rGNSSWithBicycleNavigation(Node):
#     def __init__(self):
#         super().__init__('enhanced_zedf9r_gnss_with_bicycle_navigation')
        
#         # ✅ Vehicle parameters
#         self.declare_vehicle_parameters()
        
#         # ROS2 Publishers
#         self.gnss_publisher = self.create_publisher(NavSatFix, '/navigation/gnss', 10)
#         self.xy_publisher = self.create_publisher(Float64MultiArray, 'Realtime_XY', 10)
#         self.raw_gps_publisher = self.create_publisher(Float64MultiArray, 'Raw_GPS', 10)
#         self.xy_debug_publisher = self.create_publisher(Point, '/navigation/xy_debug', 10)
#         self.heading_publisher = self.create_publisher(Float32, '/navigation/heading', 10)
#         self.heading_debug_publisher = self.create_publisher(String, '/navigation/heading_debug', 10)
#         self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.navigation_debug_publisher = self.create_publisher(String, '/navigation/debug', 10)
        
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
        
#         # Enhanced Heading management
#         self.imu_heading = None
#         self.nav_att_heading = None
#         self.nav_att_yaw = None
#         self.gps_heading = None
#         self.final_heading = None
#         self.heading_source = "NONE"
        
#         self.imu_heading_time = 0
#         self.nav_att_time = 0
#         self.gps_heading_time = 0
#         self.heading_timeout = 8.0
        
#         # ZED-F9R specific data
#         self.esf_status = None
#         self.nav_att = None
#         self.esf_alg = None
#         self.fusion_mode = "NONE"
#         self.imu_calibration_status = "UNKNOWN"
        
#         # pyubx2 UBX parsing
#         self.ubx_reader = None
#         self.ubx_buffer = bytearray()
        
#         # Enhanced Heading filtering
#         self.heading_filter = EnhancedIMUHeadingFilter()
        
#         # ✅ Navigation Components with Bicycle Model
#         csv_file_path = '/home/inc/ros2_ws/waypointfile/waypoints3xy.csv'  # แก้ไข path ให้ถูกต้อง
#         start_waypoint = 0  # เริ่มจาก waypoint ที่ต้องการ
        
#         self.waypoint_manager = WaypointManager(csv_file_path, start_waypoint)
#         self.pure_pursuit = PurePursuitController(lookahead_distance=3.0)
#         self.curvature_controller = ConstantCurvatureController()
#         self.bicycle_model = BicycleKinematicModel(wheelbase=1.67)  # จะอัพเดทจาก parameter
        
#         # Navigation state
#         self.navigation_active = True
#         self.waypoint_reached_threshold = 1.0  # เมตร
        
#         # ✅ Fixed Speed Configuration
#         self.FIXED_SPEED = 1.0  # ความเร็วคงที่ 1 m/s
        
#         # Performance stats
#         self.stats = {
#             'imu_count': 0,
#             'nav_att_count': 0,
#             'nav_att_yaw_count': 0,
#             'gps_count': 0,
#             'position_count': 0,
#             'esf_meas_count': 0,
#             'esf_status_count': 0,
#             'pyubx2_success': 0,
#             'pyubx2_errors': 0,
#             'filtered_count': 0,
#             'navigation_commands': 0,
#             'heading_corrections': 0
#         }
        
#         # Initialize coordinate system
#         self.init_track_coordinate_system()
        
#         # Initialize serial connection
#         self.init_serial_connection()
        
#         if self.ser and self.ser.is_open:
#             self.ntrip_socket = self.connect_to_ntrip()
            
#             # Initialize pyubx2 if available
#             if PYUBX2_AVAILABLE:
#                 self.init_pyubx2()
            
#             # Configure ZED-F9R for enhanced IMU
#             self.configure_enhanced_zed_f9r()
            
#             # Main processing timer
#             self.create_timer(0.1, self.main_loop)
            
#             self.get_logger().info('✅ Enhanced ZED-F9R GNSS with Bicycle Model Navigation initialized')
#             self.get_logger().info(f'🔧 pyubx2: {"Available" if PYUBX2_AVAILABLE else "Not available"}')
#             self.get_logger().info(f'🗺️ Loaded {len(self.waypoint_manager.waypoints)} waypoints')
#             self.get_logger().info(f'🚀 Fixed Speed: {self.FIXED_SPEED} m/s')
#             self.get_logger().info('🚗 Bicycle Model: Enabled for straight path heading control')
            
#             # Log vehicle parameters
#             self.log_vehicle_parameters()
            
#             # Log curve radii
#             self.log_curve_radii()
#         else:
#             self.get_logger().error('❌ Failed to initialize')
    
#     def declare_vehicle_parameters(self):
#         """ประกาศ Vehicle Parameters"""
        
#         # Wheelbase parameter
#         wheelbase_descriptor = ParameterDescriptor(
#             description='ฐานล้อของรถ (ระยะห่างระหว่างเพลาหน้าและหลัง) ในหน่วยเมตร'
#         )
#         self.declare_parameter('wheelbase', 1.67, wheelbase_descriptor)
        
#         # Max steering angle parameter
#         steering_descriptor = ParameterDescriptor(
#             description='มุมเลี้ยวสูงสุดของรถในหน่วย radians (50 degrees = 0.873 rad)'
#         )
#         self.declare_parameter('max_steering_angle', 0.873, steering_descriptor)
        
#         # Fixed speed parameter
#         speed_descriptor = ParameterDescriptor(
#             description='ความเร็วคงที่ของรถในหน่วย m/s (ตั้งค่าเป็น 1.0 m/s)'
#         )
#         self.declare_parameter('fixed_speed', 1.0, speed_descriptor)
        
#         # Starting waypoint parameter
#         start_waypoint_descriptor = ParameterDescriptor(
#             description='จุดเริ่มต้นของการนำทาง (waypoint index, เริ่มจาก 0)'
#         )
#         self.declare_parameter('start_waypoint_index', 0, start_waypoint_descriptor)
        
#         # ✅ Bicycle model parameters สำหรับการควบคุมทิศทางในทางตรง
#         heading_kp_descriptor = ParameterDescriptor(
#             description='Proportional gain สำหรับการแก้ไขทิศทางในทางตรง (ค่าต่ำ = นุ่มนวล)'
#         )
#         self.declare_parameter('heading_correction_kp', 0.02, heading_kp_descriptor)
        
#         # Curve radius parameters
#         curve1_radius_descriptor = ParameterDescriptor(
#             description='รัศมีของทางโค้งที่ 1 ในหน่วยเมตร'
#         )
#         self.declare_parameter('curve1_radius', 15.0, curve1_radius_descriptor)
        
#         curve2_radius_descriptor = ParameterDescriptor(
#             description='รัศมีของทางโค้งที่ 2 ในหน่วยเมตร'
#         )
#         self.declare_parameter('curve2_radius', 20.0, curve2_radius_descriptor)
        
#         curve3_radius_descriptor = ParameterDescriptor(
#             description='รัศมีของทางโค้งที่ 3 ในหน่วยเมตร'
#         )
#         self.declare_parameter('curve3_radius', 18.0, curve3_radius_descriptor)
        
#         curve4_radius_descriptor = ParameterDescriptor(
#             description='รัศมีของทางโค้งที่ 4 ในหน่วยเมตร'
#         )
#         self.declare_parameter('curve4_radius', 12.0, curve4_radius_descriptor)
        
#         # Angular velocity limits
#         max_angular_vel_descriptor = ParameterDescriptor(
#             description='ความเร็วเชิงมุมสูงสุดในหน่วย rad/s'
#         )
#         self.declare_parameter('max_angular_velocity', 1.0, max_angular_vel_descriptor)
        
#         self.get_logger().info('🌟 Vehicle parameters declared successfully!')
    
#     def get_vehicle_parameters(self):
#         """ดึงค่า Vehicle Parameters ปัจจุบัน"""
#         try:
#             wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
#             max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value
#             fixed_speed = self.get_parameter('fixed_speed').get_parameter_value().double_value
#             start_waypoint_index = self.get_parameter('start_waypoint_index').get_parameter_value().integer_value
#             heading_correction_kp = self.get_parameter('heading_correction_kp').get_parameter_value().double_value
#             max_angular_velocity = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
            
#             # ดึงค่ารัศมีของทางโค้ง
#             curve_radii = {
#                 'curve1': self.get_parameter('curve1_radius').get_parameter_value().double_value,
#                 'curve2': self.get_parameter('curve2_radius').get_parameter_value().double_value,
#                 'curve3': self.get_parameter('curve3_radius').get_parameter_value().double_value,
#                 'curve4': self.get_parameter('curve4_radius').get_parameter_value().double_value
#             }
            
#             return {
#                 'wheelbase': wheelbase,
#                 'max_steering_angle': max_steering_angle,
#                 'fixed_speed': fixed_speed,
#                 'start_waypoint_index': start_waypoint_index,
#                 'heading_correction_kp': heading_correction_kp,
#                 'max_angular_velocity': max_angular_velocity,
#                 'curve_radii': curve_radii
#             }
#         except Exception as e:
#             self.get_logger().error(f'❌ Error getting vehicle parameters: {e}')
#             return {
#                 'wheelbase': 1.67,
#                 'max_steering_angle': 0.873,
#                 'fixed_speed': 1.0,
#                 'start_waypoint_index': 0,
#                 'heading_correction_kp': 0.02,
#                 'max_angular_velocity': 1.0,
#                 'curve_radii': {
#                     'curve1': 15.0,
#                     'curve2': 20.0,
#                     'curve3': 18.0,
#                     'curve4': 12.0
#                 }
#             }
    
#     def log_vehicle_parameters(self):
#         """แสดง Vehicle Parameters ปัจจุบัน"""
#         params = self.get_vehicle_parameters()
#         self.get_logger().info(
#             f'🚗 Vehicle Parameters:\n'
#             f'  🔧 Wheelbase: {params["wheelbase"]:.2f} m\n'
#             f'  🔧 Max Steering Angle: {params["max_steering_angle"]:.3f} rad ({math.degrees(params["max_steering_angle"]):.1f}°)\n'
#             f'  🔧 Fixed Speed: {params["fixed_speed"]:.2f} m/s\n'
#             f'  🔧 Start Waypoint: {params["start_waypoint_index"] + 1}\n'
#             f'  🔧 Heading Correction Kp: {params["heading_correction_kp"]:.3f}\n'
#             f'  🔧 Max Angular Velocity: {params["max_angular_velocity"]:.2f} rad/s'
#         )
    
#     def log_curve_radii(self):
#         """แสดงรัศมีของทางโค้ง"""
#         params = self.get_vehicle_parameters()
#         curve_radii = params['curve_radii']
        
#         self.get_logger().info(
#             f'🌀 Curve Radii Configuration:\n'
#             f'  🔄 Curve 1 (Waypoint 245-260): {curve_radii["curve1"]:.1f} m\n'
#             f'  🔄 Curve 2 (Waypoint 316-390): {curve_radii["curve2"]:.1f} m\n'
#             f'  🔄 Curve 3 (Waypoint 501-579): {curve_radii["curve3"]:.1f} m\n'
#             f'  🔄 Curve 4 (Waypoint 631-646): {curve_radii["curve4"]:.1f} m'
#         )
    
#     def navigation_control(self):
#         """✅ ระบบควบคุมการนำทางด้วย Bicycle Model สำหรับทางตรง"""
#         if not self.navigation_active or self.final_heading is None:
#             return
        
#         # ได้ตำแหน่งปัจจุบัน
#         current_waypoint = self.waypoint_manager.get_current_waypoint()
#         if current_waypoint is None:
#             self.get_logger().info('🏁 Navigation completed - reached final waypoint')
#             self.navigation_active = False
#             self.send_stop_command()
#             return
        
#         # ตรวจสอบว่าถึง waypoint แล้วหรือยัง
#         distance_to_waypoint = math.sqrt(
#             (self.current_position['x'] - current_waypoint['x'])**2 + 
#             (self.current_position['y'] - current_waypoint['y'])**2
#         )
        
#         if distance_to_waypoint < self.waypoint_reached_threshold:
#             if self.waypoint_manager.advance_waypoint():
#                 progress = self.waypoint_manager.get_progress_info()
#                 self.get_logger().info(
#                     f'✅ Reached waypoint {progress["current"]-1} '
#                     f'({progress["progress_percent"]:.1f}% complete)'
#                 )
#             else:
#                 self.get_logger().info('🏁 Navigation completed')
#                 self.navigation_active = False
#                 self.send_stop_command()
#                 return
        
#         # ได้ข้อมูลเส้นทางปัจจุบัน
#         segment_info = self.waypoint_manager.get_current_segment_info()
#         segment_type = segment_info['type']
#         direction = segment_info['direction']
#         target_heading = segment_info.get('target_heading')
        
#         # ได้ parameters
#         params = self.get_vehicle_parameters()
        
#         # ใช้ความเร็วคงที่ 1 m/s
#         linear_velocity = self.FIXED_SPEED
        
#         # คำนวณคำสั่งควบคุมตามประเภทเส้นทาง
#         if segment_type == 'straight':
#             steering_angle = self.calculate_straight_path_control_with_bicycle_model(
#                 current_waypoint, target_heading, params
#             )
#         else:  # curve
#             steering_angle = self.calculate_curve_path_control_with_radius(direction, params)
        
#         # จำกัดมุมเลี้ยว
#         steering_angle = self.bicycle_model.limit_steering_angle(
#             steering_angle, params['max_steering_angle']
#         )
        
#         # คำนวณ angular velocity ด้วย bicycle model
#         angular_velocity = self.bicycle_model.calculate_angular_velocity(
#             linear_velocity, steering_angle
#         )
        
#         # จำกัด angular velocity
#         angular_velocity = max(-params['max_angular_velocity'], 
#                              min(params['max_angular_velocity'], angular_velocity))
        
#         # สร้างและส่งคำสั่ง cmd_vel
#         cmd_vel = Twist()
#         cmd_vel.linear.x = linear_velocity
#         cmd_vel.angular.z = angular_velocity
        
#         self.cmd_vel_publisher.publish(cmd_vel)
#         self.stats['navigation_commands'] += 1
        
#         # Debug information
#         debug_msg = String()
#         progress = self.waypoint_manager.get_progress_info()
        
#         if segment_type == 'curve':
#             curve_radius = params['curve_radii'].get(direction, 15.0)
#             debug_msg.data = (
#                 f"Segment: {segment_type} ({direction}, R={curve_radius:.1f}m) | "
#                 f"Waypoint: {progress['current']}/{progress['total']} ({progress['progress_percent']:.1f}%) | "
#                 f"Distance: {distance_to_waypoint:.2f}m | "
#                 f"Steering: {math.degrees(steering_angle):.1f}° | "
#                 f"Speed: {linear_velocity:.2f}m/s | "
#                 f"Angular: {angular_velocity:.2f}rad/s"
#             )
#         else:
#             heading_error = self.bicycle_model.normalize_angle_difference(target_heading - self.final_heading) if target_heading is not None else 0.0
#             debug_msg.data = (
#                 f"Segment: {segment_type} ({direction}, Target={target_heading:.0f}°) | "
#                 f"Waypoint: {progress['current']}/{progress['total']} ({progress['progress_percent']:.1f}%) | "
#                 f"Distance: {distance_to_waypoint:.2f}m | "
#                 f"Heading: {self.final_heading:.1f}° (Error: {heading_error:.1f}°) | "
#                 f"Steering: {math.degrees(steering_angle):.1f}° | "
#                 f"Speed: {linear_velocity:.2f}m/s | "
#                 f"Angular: {angular_velocity:.2f}rad/s"
#             )
        
#         self.navigation_debug_publisher.publish(debug_msg)
    
#     def calculate_straight_path_control_with_bicycle_model(self, target_waypoint, target_heading, params):
#         """✅ คำนวณการควบคุมสำหรับเส้นทางตรงด้วย Bicycle Model + Heading Control"""
        
#         # ถ้าไม่มี target_heading ให้ใช้ Pure Pursuit
#         if target_heading is None:
#             steering_angle = self.pure_pursuit.calculate_steering_angle(
#                 self.current_position,
#                 self.final_heading,
#                 target_waypoint,
#                 params['wheelbase']
#             )
#             return steering_angle * 0.5  # ลดความแรงของการเลี้ยว
        
#         # ✅ ใช้ Bicycle Model เพื่อรักษาทิศทางคงที่
#         heading_correction_steering = self.bicycle_model.calculate_heading_correction_steering(
#             self.final_heading,
#             target_heading,
#             params['heading_correction_kp']
#         )
        
#         # เพิ่ม Pure Pursuit เล็กน้อยเพื่อนำทางไปยัง waypoint
#         pure_pursuit_steering = self.pure_pursuit.calculate_steering_angle(
#             self.current_position,
#             self.final_heading,
#             target_waypoint,
#             params['wheelbase']
#         )
        
#         # ผสมผสานระหว่าง heading correction และ pure pursuit
#         # ให้น้ำหนัก 80% กับ heading correction, 20% กับ pure pursuit
#         combined_steering = 0.8 * heading_correction_steering + 0.2 * pure_pursuit_steering * 0.3
        
#         # นับสถิติการแก้ไขทิศทาง
#         if abs(heading_correction_steering) > 0.01:  # มากกว่า ~0.6 องศา
#             self.stats['heading_corrections'] += 1
        
#         return combined_steering
    
#     def calculate_curve_path_control_with_radius(self, curve_direction, params):
#         """คำนวณการควบคุมสำหรับเส้นทางโค้งด้วยรัศมีที่กำหนด"""
#         # ได้รัศมีของทางโค้งจาก parameters
#         curve_radius = params['curve_radii'].get(curve_direction, 15.0)
        
#         # คำนวณมุมเลี้ยวจากรัศมีที่กำหนด
#         steering_angle = self.curvature_controller.calculate_steering_angle_with_radius(
#             curve_radius,
#             self.FIXED_SPEED,
#             params['wheelbase']
#         )
        
#         return steering_angle
    
#     def send_stop_command(self):
#         """ส่งคำสั่งหยุดรถ"""
#         cmd_vel = Twist()
#         cmd_vel.linear.x = 0.0
#         cmd_vel.angular.z = 0.0
#         self.cmd_vel_publisher.publish(cmd_vel)
#         self.get_logger().info('🛑 Stop command sent')
    
#     def set_starting_waypoint_service(self, waypoint_index):
#         """เปลี่ยนจุดเริ่มต้นระหว่างการทำงาน"""
#         if self.waypoint_manager.set_starting_waypoint(waypoint_index):
#             self.navigation_active = True
#             self.get_logger().info(f'🚀 Navigation restarted from waypoint {waypoint_index + 1}')
#             return True
#         return False
    
#     # เพิ่มฟังก์ชันที่ขาดหายไปจากโค้ดเดิม
#     def init_track_coordinate_system(self):
#         """Initialize coordinate system"""
#         if self.reference_utm is None:
#             self.reference_utm = self.transformer.transform(
#                 self.reference_point[1], self.reference_point[0]
#             )
        
#         self.utm_offset_x = self.TARGET_UTM_X - self.reference_utm[0]
#         self.utm_offset_y = self.TARGET_UTM_Y - self.reference_utm[1]
        
#         self.get_logger().info(f'🗺️ Track coordinate system initialized')
#         self.get_logger().info(f'📍 Reference UTM: {self.reference_utm}')
#         self.get_logger().info(f'🎯 Target UTM: ({self.TARGET_UTM_X}, {self.TARGET_UTM_Y})')
#         self.get_logger().info(f'📐 Offsets: ({self.utm_offset_x:.2f}, {self.utm_offset_y:.2f})')
    
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
        
#         if not self.ser or not self.ser.is_open:
#             self.get_logger().error('❌ No serial connection available')
    
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
#         import base64
#         credentials = f"{self.ntrip_username}:{self.ntrip_password}"
#         return base64.b64encode(credentials.encode()).decode()
    
#     def init_pyubx2(self):
#         """Initialize pyubx2 if available"""
#         if PYUBX2_AVAILABLE:
#             self.get_logger().info('✅ pyubx2 initialized for enhanced UBX parsing')
    
#     def configure_enhanced_zed_f9r(self):
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
            
#             # อัพเดทตำแหน่งปัจจุบัน
#             self.update_current_position()
            
#             # ควบคุมการนำทาง
#             self.navigation_control()
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Main loop error: {e}')
    
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
#         # ประมวลผลข้อมูล NMEA และ UBX
#         # (ใช้โค้ดเดิมที่มีอยู่แล้วในการประมวลผล GNSS data)
#         pass
    
#     def update_current_position(self):
#         """Update current position from GNSS data"""
#         # อัพเดทตำแหน่งปัจจุบันจากข้อมูล GNSS
#         # (ใช้โค้ดเดิมที่มีอยู่แล้วในการอัพเดทตำแหน่ง)
#         pass

# def main(args=None):
#     rclpy.init(args=args)
    
#     try:
#         node = EnhancedZEDf9rGNSSWithBicycleNavigation()
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         print('\n🛑 Navigation stopped by user')
#     except Exception as e:
#         print(f'❌ Error: {e}')
#     finally:
#         if 'node' in locals():
#             node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# --------------------------------------new controller V3---------------------------------------------


# #!/usr/bin/env python3
# import serial
# import pynmea2
# import socket
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
# from std_msgs.msg import Float32, String, Float64MultiArray
# from geometry_msgs.msg import Point, Twist
# from rcl_interfaces.msg import ParameterDescriptor
# import math
# import pyproj
# from collections import deque
# import time
# import glob
# import os
# import struct
# import json
# import numpy as np
# import csv

# # เพิ่ม pyubx2 สำหรับ UBX parsing ที่ดีขึ้น
# try:
#     from pyubx2 import UBXReader, UBXMessage, UBX_MSGIDS
#     PYUBX2_AVAILABLE = True
#     print("✅ pyubx2 available - using enhanced UBX parsing")
# except ImportError:
#     PYUBX2_AVAILABLE = False
#     print("⚠️ pyubx2 not available - install with: pip3 install pyubx2")

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

# # ✅ Enhanced Waypoint Manager with Smart Starting Point Selection
# class WaypointManager:
#     def __init__(self, csv_file_path, auto_start=True):
#         self.waypoints = []
#         self.current_waypoint_index = 0
#         self.auto_start = auto_start
#         self.navigation_started = False
#         self.load_waypoints_from_csv(csv_file_path)
        
#         # กำหนดช่วง waypoint และประเภทเส้นทาง พร้อมทิศทางที่ต้องรักษา
#         self.path_segments = [
#             {'start': 0, 'end': 240, 'type': 'straight', 'direction': 'S-N', 'target_heading': 0.0},      # waypoint 1-240 (North = 0°)
#             {'start': 241, 'end': 256, 'type': 'curve', 'direction': 'curve1', 'target_heading': None},    # waypoint 245-260
#             {'start': 257, 'end': 315, 'type': 'straight', 'direction': 'E-W', 'target_heading': 90.0},   # waypoint 261-315 (East = 90°)
#             {'start': 316, 'end': 383, 'type': 'curve', 'direction': 'curve2', 'target_heading': None},    # waypoint 316-390
#             {'start': 384, 'end': 496, 'type': 'straight', 'direction': 'N-S', 'target_heading': 180.0},  # waypoint 391-500 (South = 180°)
#             {'start': 497, 'end': 571, 'type': 'curve', 'direction': 'curve3', 'target_heading': None},    # waypoint 501-579
#             {'start': 572, 'end': 622, 'type': 'straight', 'direction': 'W-E', 'target_heading': 270.0},  # waypoint 580-630 (West = 270°)
#             {'start': 623, 'end': 642, 'type': 'curve', 'direction': 'curve4', 'target_heading': None},    # waypoint 631-646
#             {'start': 643, 'end': 654, 'type': 'straight', 'direction': 'S-N', 'target_heading': 0.0}     # waypoint 647-659 (North = 0°)
#         ]
        
#         # กำหนดรัศมีสำหรับแต่ละทางโค้ง
#         self.curve_radii = {
#             'curve1': 15.0,  # รัศมี 15 เมตร สำหรับโค้งที่ 1
#             'curve2': 20.0,  # รัศมี 20 เมตร สำหรับโค้งที่ 2
#             'curve3': 18.0,  # รัศมี 18 เมตร สำหรับโค้งที่ 3
#             'curve4': 12.0   # รัศมี 12 เมตร สำหรับโค้งที่ 4
#         }
        
#         print(f"✅ Loaded {len(self.waypoints)} waypoints from CSV")
#         if auto_start:
#             print("🚀 Auto-start mode enabled - will find nearest forward waypoint")
#         else:
#             print("🎯 Manual start mode - will start from waypoint 0")
    
#     def load_waypoints_from_csv(self, csv_file_path):
#         """โหลด waypoints จากไฟล์ CSV"""
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
#         """✅ หา waypoint ที่ใกล้ที่สุดที่อยู่หน้ารถ"""
#         if not current_position or not self.waypoints:
#             return 0
        
#         current_x, current_y = current_position['x'], current_position['y']
#         best_waypoint = None
#         best_index = 0
#         best_score = float('inf')
        
#         candidates = []
        
#         for i, waypoint in enumerate(self.waypoints):
#             # คำนวณระยะทาง
#             dx = waypoint['x'] - current_x
#             dy = waypoint['y'] - current_y
#             distance = math.sqrt(dx**2 + dy**2)
            
#             # ข้ามถ้าไกลเกินไป
#             if distance > search_radius:
#                 continue
            
#             # คำนวณทิศทางไปยัง waypoint (compass bearing)
#             waypoint_bearing = math.degrees(math.atan2(dx, dy))  # atan2(East, North)
#             if waypoint_bearing < 0:
#                 waypoint_bearing += 360
            
#             # คำนวณความแตกต่างของมุม
#             heading_diff = abs(self.normalize_angle_difference(waypoint_bearing - current_heading))
            
#             # ตรวจสอบว่าอยู่หน้ารถหรือไม่ (มุมไม่เกิน ±90°)
#             if heading_diff <= 90.0:
#                 # คำนวณคะแนน (ยิ่งใกล้และอยู่หน้าตรงยิ่งดี)
#                 distance_score = distance
#                 angle_score = heading_diff / 90.0  # normalize เป็น 0-1
#                 combined_score = distance_score + (angle_score * 10.0)  # ให้น้ำหนักกับมุม
                
#                 candidates.append({
#                     'index': i,
#                     'distance': distance,
#                     'heading_diff': heading_diff,
#                     'score': combined_score,
#                     'waypoint': waypoint
#                 })
        
#         if candidates:
#             # เรียงตามคะแนน
#             candidates.sort(key=lambda x: x['score'])
#             best_candidate = candidates[0]
            
#             print(f"🎯 Found {len(candidates)} forward waypoints within {search_radius}m:")
#             for i, candidate in enumerate(candidates[:5]):  # แสดง 5 อันดับแรก
#                 print(f"  #{i+1}: Waypoint {candidate['index']} - "
#                       f"Distance: {candidate['distance']:.2f}m, "
#                       f"Angle: {candidate['heading_diff']:.1f}°, "
#                       f"Score: {candidate['score']:.2f}")
            
#             print(f"✅ Selected waypoint {best_candidate['index']} as starting point")
#             return best_candidate['index']
#         else:
#             # ถ้าไม่เจอ waypoint ที่อยู่หน้ารถ ให้หาที่ใกล้ที่สุด
#             print(f"⚠️ No forward waypoints found within {search_radius}m")
#             return self.find_nearest_waypoint(current_position)
    
#     def find_nearest_waypoint(self, current_position):
#         """หา waypoint ที่ใกล้ที่สุด (ไม่สนใจทิศทาง)"""
#         if not current_position or not self.waypoints:
#             return 0
        
#         current_x, current_y = current_position['x'], current_position['y']
#         min_distance = float('inf')
#         nearest_index = 0
        
#         for i, waypoint in enumerate(self.waypoints):
#             distance = math.sqrt(
#                 (waypoint['x'] - current_x)**2 + 
#                 (waypoint['y'] - current_y)**2
#             )
#             if distance < min_distance:
#                 min_distance = distance
#                 nearest_index = i
        
#         print(f"🔍 Nearest waypoint: {nearest_index} (distance: {min_distance:.2f}m)")
#         return nearest_index
    
#     def initialize_starting_point(self, current_position, current_heading):
#         """✅ เริ่มต้นการนำทางจากจุดที่เหมาะสม"""
#         if self.navigation_started:
#             return False
        
#         if not current_position or current_heading is None:
#             print("⚠️ Waiting for position and heading data...")
#             return False
        
#         if self.auto_start:
#             # หา waypoint ที่ใกล้ที่สุดที่อยู่หน้ารถ
#             self.current_waypoint_index = self.find_nearest_forward_waypoint(
#                 current_position, current_heading
#             )
#         else:
#             # เริ่มจาก waypoint แรก
#             self.current_waypoint_index = 0
        
#         self.navigation_started = True
        
#         print(f"🚀 Navigation started from waypoint {self.current_waypoint_index + 1}")
#         print(f"📍 Target: ({self.waypoints[self.current_waypoint_index]['x']:.2f}, "
#               f"{self.waypoints[self.current_waypoint_index]['y']:.2f})")
        
#         return True
    
#     def normalize_angle_difference(self, angle_diff):
#         """ปรับความแตกต่างของมุมให้อยู่ในช่วง [-180, 180]"""
#         while angle_diff > 180:
#             angle_diff -= 360
#         while angle_diff < -180:
#             angle_diff += 360
#         return angle_diff
    
#     def set_starting_waypoint(self, waypoint_index):
#         """กำหนดจุดเริ่มต้นใหม่"""
#         if 0 <= waypoint_index < len(self.waypoints):
#             self.current_waypoint_index = waypoint_index
#             self.navigation_started = True
#             print(f"✅ Starting waypoint set to {waypoint_index + 1}")
#             return True
#         else:
#             print(f"❌ Invalid waypoint index: {waypoint_index}")
#             return False
    
#     def get_current_segment_info(self):
#         """ได้ข้อมูลของเส้นทางปัจจุบัน"""
#         for segment in self.path_segments:
#             if segment['start'] <= self.current_waypoint_index <= segment['end']:
#                 return segment
#         return {'type': 'straight', 'direction': 'unknown', 'target_heading': 0.0}
    
#     def get_curve_radius(self, curve_direction):
#         """ได้รัศมีของทางโค้ง"""
#         return self.curve_radii.get(curve_direction, 15.0)  # default 15 เมตร
    
#     def get_current_waypoint(self):
#         """ได้ waypoint ปัจจุบัน"""
#         if self.current_waypoint_index < len(self.waypoints):
#             return self.waypoints[self.current_waypoint_index]
#         return None
    
#     def get_next_waypoint(self):
#         """ได้ waypoint ถัดไป"""
#         next_index = self.current_waypoint_index + 1
#         if next_index < len(self.waypoints):
#             return self.waypoints[next_index]
#         return None
    
#     def advance_waypoint(self):
#         """เลื่อนไปยัง waypoint ถัดไป"""
#         if self.current_waypoint_index < len(self.waypoints) - 1:
#             self.current_waypoint_index += 1
#             return True
#         return False
    
#     def get_progress_info(self):
#         """ได้ข้อมูลความคืบหน้า"""
#         return {
#             'current': self.current_waypoint_index + 1,
#             'total': len(self.waypoints),
#             'progress_percent': ((self.current_waypoint_index + 1) / len(self.waypoints)) * 100
#         }

# # ✅ Bicycle Kinematic Model for Straight Path Control
# class BicycleKinematicModel:
#     def __init__(self, wheelbase):
#         self.wheelbase = wheelbase
    
#     def calculate_angular_velocity(self, linear_velocity, steering_angle):
#         """คำนวณ angular velocity จาก bicycle model"""
#         if abs(steering_angle) < 1e-6:  # ป้องกันการหารด้วยศูนย์
#             return 0.0
        
#         angular_velocity = (linear_velocity * math.tan(steering_angle)) / self.wheelbase
#         return angular_velocity
    
#     def limit_steering_angle(self, steering_angle, max_steering_angle):
#         """จำกัดมุมเลี้ยวให้อยู่ในขอบเขต"""
#         return max(-max_steering_angle, min(max_steering_angle, steering_angle))
    
#     def calculate_heading_correction_steering(self, current_heading, target_heading, kp=0.02):
#         """✅ คำนวณมุมเลี้ยวเพื่อแก้ไขทิศทางในทางตรง (bicycle model)"""
#         # คำนวณความผิดพลาดของทิศทาง
#         heading_error = self.normalize_angle_difference(target_heading - current_heading)
        
#         # ใช้ proportional control สำหรับการแก้ไขทิศทาง
#         # kp ต่ำ = การแก้ไขนุ่มนวล, kp สูง = การแก้ไขรวดเร็ว
#         steering_angle = kp * math.radians(heading_error)
        
#         return steering_angle
    
#     def normalize_angle_difference(self, angle_diff):
#         """ปรับความแตกต่างของมุมให้อยู่ในช่วง [-180, 180]"""
#         while angle_diff > 180:
#             angle_diff -= 360
#         while angle_diff < -180:
#             angle_diff += 360
#         return angle_diff

# # Pure Pursuit Controller
# class PurePursuitController:
#     def __init__(self, lookahead_distance=2.0):
#         self.lookahead_distance = lookahead_distance
    
#     def calculate_steering_angle(self, current_pos, current_heading, target_pos, wheelbase):
#         """คำนวณมุมเลี้ยวสำหรับ Pure Pursuit"""
#         # คำนวณระยะทางและมุมไปยังเป้าหมาย
#         dx = target_pos['x'] - current_pos['x']
#         dy = target_pos['y'] - current_pos['y']
        
#         target_heading = math.degrees(math.atan2(dx, dy))
#         heading_error = self.normalize_angle(target_heading - current_heading)
        
#         distance = math.sqrt(dx**2 + dy**2)
        
#         # Pure Pursuit formula
#         alpha = math.radians(heading_error)
#         steering_angle = math.atan2(2 * wheelbase * math.sin(alpha), distance)
        
#         return steering_angle
    
#     def normalize_angle(self, angle):
#         """Normalize angle to [-180, 180]"""
#         while angle > 180:
#             angle -= 360
#         while angle < -180:
#             angle += 360
#         return angle

# # Constant Curvature Controller
# class ConstantCurvatureController:
#     def __init__(self):
#         self.curvature_history = deque(maxlen=5)
    
#     def calculate_steering_angle_with_radius(self, curve_radius, linear_velocity, wheelbase):
#         """คำนวณมุมเลี้ยวจากรัศมีที่กำหนด"""
#         # คำนวณ curvature จากรัศมี: κ = 1/R
#         curvature = 1.0 / curve_radius
        
#         # คำนวณมุมเลี้ยวจาก curvature และ bicycle model
#         # δ = atan(L * κ) โดย L = wheelbase, κ = curvature
#         steering_angle = math.atan(wheelbase * curvature)
        
#         return steering_angle
    
#     def calculate_steering_angle(self, waypoints_segment, current_pos, wheelbase):
#         """คำนวณมุมเลี้ยวสำหรับ Constant Curvature (วิธีเดิม)"""
#         if len(waypoints_segment) < 3:
#             return 0.0
        
#         # คำนวณ curvature จาก 3 จุด
#         curvature = self.calculate_curvature(waypoints_segment)
#         self.curvature_history.append(curvature)
        
#         # ใช้ค่าเฉลี่ยของ curvature
#         avg_curvature = np.mean(list(self.curvature_history))
        
#         # คำนวณมุมเลี้ยวจาก curvature
#         steering_angle = math.atan(wheelbase * avg_curvature)
        
#         return steering_angle
    
#     def calculate_curvature(self, points):
#         """คำนวณ curvature จาก 3 จุด"""
#         if len(points) < 3:
#             return 0.0
        
#         p1, p2, p3 = points[0], points[1], points[2]
        
#         # คำนวณ curvature โดยใช้สูตร
#         x1, y1 = p1['x'], p1['y']
#         x2, y2 = p2['x'], p2['y']
#         x3, y3 = p3['x'], p3['y']
        
#         # Area of triangle
#         area = 0.5 * abs((x2-x1)*(y3-y1) - (x3-x1)*(y2-y1))
        
#         # Side lengths
#         a = math.sqrt((x2-x3)**2 + (y2-y3)**2)
#         b = math.sqrt((x1-x3)**2 + (y1-y3)**2)
#         c = math.sqrt((x1-x2)**2 + (y1-y2)**2)
        
#         if a * b * c == 0:
#             return 0.0
        
#         # Curvature = 4 * Area / (a * b * c)
#         curvature = 4 * area / (a * b * c)
        
#         return curvature

# # ✅ Main Enhanced GNSS Node with Smart Starting Point and Enhanced Heading
# class EnhancedZEDf9rGNSSWithSmartNavigation(Node):
#     def __init__(self):
#         super().__init__('enhanced_zedf9r_gnss_with_smart_navigation')
        
#         # ✅ Vehicle parameters
#         self.declare_vehicle_parameters()
        
#         # ROS2 Publishers
#         self.gnss_publisher = self.create_publisher(NavSatFix, '/navigation/gnss', 10)
#         self.xy_publisher = self.create_publisher(Float64MultiArray, 'Realtime_XY', 10)
#         self.raw_gps_publisher = self.create_publisher(Float64MultiArray, 'Raw_GPS', 10)
#         self.xy_debug_publisher = self.create_publisher(Point, '/navigation/xy_debug', 10)
        
#         # ✅ Enhanced Heading Publishers (เหมือนโค้ดที่ 2)
#         self.heading_publisher = self.create_publisher(Float32, '/navigation/heading', 10)
#         self.heading_debug_publisher = self.create_publisher(String, '/navigation/heading_debug', 10)
        
#         self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.navigation_debug_publisher = self.create_publisher(String, '/navigation/debug', 10)
        
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
        
#         # ✅ Enhanced Heading management (เหมือนโค้ดที่ 2)
#         self.imu_heading = None
#         self.nav_att_heading = None
#         self.nav_att_yaw = None
#         self.gps_heading = None
#         self.final_heading = None  # compass bearing (degrees)
#         self.heading_source = "NONE"
        
#         self.imu_heading_time = 0
#         self.nav_att_time = 0
#         self.gps_heading_time = 0
#         self.heading_timeout = 8.0
        
#         # ZED-F9R specific data
#         self.esf_status = None
#         self.nav_att = None
#         self.esf_alg = None
#         self.fusion_mode = "NONE"
#         self.imu_calibration_status = "UNKNOWN"
        
#         # ✅ UTM coordinates for controller (เหมือนโค้ดที่ 2)
#         self.current_utm_x = None
#         self.current_utm_y = None
        
#         # pyubx2 UBX parsing
#         self.ubx_reader = None
#         self.ubx_buffer = bytearray()
        
#         # Enhanced Heading filtering
#         self.heading_filter = EnhancedIMUHeadingFilter()
        
#         # ✅ Navigation Components with Smart Starting Point
#         csv_file_path = '/home/inc/ros2_ws/waypointfile/waypoints3xy.csv'  # แก้ไข path ให้ถูกต้อง
        
#         self.waypoint_manager = WaypointManager(csv_file_path, auto_start=True)  # ✅ เปิด auto-start
#         self.pure_pursuit = PurePursuitController(lookahead_distance=3.0)
#         self.curvature_controller = ConstantCurvatureController()
#         self.bicycle_model = BicycleKinematicModel(wheelbase=1.67)  # จะอัพเดทจาก parameter
        
#         # Navigation state
#         self.navigation_active = False  # ✅ เริ่มต้นเป็น False จนกว่าจะหาจุดเริ่มต้นได้
#         self.waypoint_reached_threshold = 1.0  # เมตร
        
#         # ✅ Fixed Speed Configuration
#         self.FIXED_SPEED = 1.0  # ความเร็วคงที่ 1 m/s
        
#         # Performance stats
#         self.stats = {
#             'imu_count': 0,
#             'nav_att_count': 0,
#             'nav_att_yaw_count': 0,
#             'gps_count': 0,
#             'position_count': 0,
#             'esf_meas_count': 0,
#             'esf_status_count': 0,
#             'pyubx2_success': 0,
#             'pyubx2_errors': 0,
#             'filtered_count': 0,
#             'navigation_commands': 0,
#             'heading_corrections': 0
#         }
        
#         # Initialize coordinate system
#         self.init_track_coordinate_system()
        
#         # Initialize serial connection
#         self.init_serial_connection()
        
#         if self.ser and self.ser.is_open:
#             self.ntrip_socket = self.connect_to_ntrip()
            
#             # Initialize pyubx2 if available
#             if PYUBX2_AVAILABLE:
#                 self.init_pyubx2()
            
#             # Configure ZED-F9R for enhanced IMU
#             self.configure_enhanced_zed_f9r()
            
#             # Main processing timer
#             self.create_timer(0.1, self.main_loop)
            
#             self.get_logger().info('✅ Enhanced ZED-F9R GNSS with Smart Navigation initialized')
#             self.get_logger().info(f'🔧 pyubx2: {"Available" if PYUBX2_AVAILABLE else "Not available"}')
#             self.get_logger().info(f'🗺️ Loaded {len(self.waypoint_manager.waypoints)} waypoints')
#             self.get_logger().info(f'🚀 Fixed Speed: {self.FIXED_SPEED} m/s')
#             self.get_logger().info('🎯 Smart starting point: Will find nearest forward waypoint')
#             self.get_logger().info('🧭 Enhanced heading calculation enabled')
            
#             # Log vehicle parameters
#             self.log_vehicle_parameters()
            
#             # Log curve radii
#             self.log_curve_radii()
#         else:
#             self.get_logger().error('❌ Failed to initialize')
    
#     def declare_vehicle_parameters(self):
#         """ประกาศ Vehicle Parameters"""
        
#         # Wheelbase parameter
#         wheelbase_descriptor = ParameterDescriptor(
#             description='ฐานล้อของรถ (ระยะห่างระหว่างเพลาหน้าและหลัง) ในหน่วยเมตร'
#         )
#         self.declare_parameter('wheelbase', 1.67, wheelbase_descriptor)
        
#         # Max steering angle parameter
#         steering_descriptor = ParameterDescriptor(
#             description='มุมเลี้ยวสูงสุดของรถในหน่วย radians (50 degrees = 0.873 rad)'
#         )
#         self.declare_parameter('max_steering_angle', 0.873, steering_descriptor)
        
#         # Fixed speed parameter
#         speed_descriptor = ParameterDescriptor(
#             description='ความเร็วคงที่ของรถในหน่วย m/s (ตั้งค่าเป็น 1.0 m/s)'
#         )
#         self.declare_parameter('fixed_speed', 1.0, speed_descriptor)
        
#         # ✅ Smart starting parameters
#         auto_start_descriptor = ParameterDescriptor(
#             description='เปิดใช้การหาจุดเริ่มต้นอัตโนมัติ (true = หาจุดที่ใกล้ที่สุดที่อยู่หน้ารถ)'
#         )
#         self.declare_parameter('auto_start', True, auto_start_descriptor)
        
#         search_radius_descriptor = ParameterDescriptor(
#             description='รัศมีการค้นหา waypoint ในหน่วยเมตร'
#         )
#         self.declare_parameter('search_radius', 50.0, search_radius_descriptor)
        
#         # ✅ Bicycle model parameters สำหรับการควบคุมทิศทางในทางตรง
#         heading_kp_descriptor = ParameterDescriptor(
#             description='Proportional gain สำหรับการแก้ไขทิศทางในทางตรง (ค่าต่ำ = นุ่มนวล)'
#         )
#         self.declare_parameter('heading_correction_kp', 0.02, heading_kp_descriptor)
        
#         # Curve radius parameters
#         curve1_radius_descriptor = ParameterDescriptor(
#             description='รัศมีของทางโค้งที่ 1 ในหน่วยเมตร'
#         )
#         self.declare_parameter('curve1_radius', 15.0, curve1_radius_descriptor)
        
#         curve2_radius_descriptor = ParameterDescriptor(
#             description='รัศมีของทางโค้งที่ 2 ในหน่วยเมตร'
#         )
#         self.declare_parameter('curve2_radius', 20.0, curve2_radius_descriptor)
        
#         curve3_radius_descriptor = ParameterDescriptor(
#             description='รัศมีของทางโค้งที่ 3 ในหน่วยเมตร'
#         )
#         self.declare_parameter('curve3_radius', 18.0, curve3_radius_descriptor)
        
#         curve4_radius_descriptor = ParameterDescriptor(
#             description='รัศมีของทางโค้งที่ 4 ในหน่วยเมตร'
#         )
#         self.declare_parameter('curve4_radius', 12.0, curve4_radius_descriptor)
        
#         # Angular velocity limits
#         max_angular_vel_descriptor = ParameterDescriptor(
#             description='ความเร็วเชิงมุมสูงสุดในหน่วย rad/s'
#         )
#         self.declare_parameter('max_angular_velocity', 1.0, max_angular_vel_descriptor)
        
#         self.get_logger().info('🌟 Vehicle parameters declared successfully!')
    
#     def get_vehicle_parameters(self):
#         """ดึงค่า Vehicle Parameters ปัจจุบัน"""
#         try:
#             wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
#             max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value
#             fixed_speed = self.get_parameter('fixed_speed').get_parameter_value().double_value
#             auto_start = self.get_parameter('auto_start').get_parameter_value().bool_value
#             search_radius = self.get_parameter('search_radius').get_parameter_value().double_value
#             heading_correction_kp = self.get_parameter('heading_correction_kp').get_parameter_value().double_value
#             max_angular_velocity = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
            
#             # ดึงค่ารัศมีของทางโค้ง
#             curve_radii = {
#                 'curve1': self.get_parameter('curve1_radius').get_parameter_value().double_value,
#                 'curve2': self.get_parameter('curve2_radius').get_parameter_value().double_value,
#                 'curve3': self.get_parameter('curve3_radius').get_parameter_value().double_value,
#                 'curve4': self.get_parameter('curve4_radius').get_parameter_value().double_value
#             }
            
#             return {
#                 'wheelbase': wheelbase,
#                 'max_steering_angle': max_steering_angle,
#                 'fixed_speed': fixed_speed,
#                 'auto_start': auto_start,
#                 'search_radius': search_radius,
#                 'heading_correction_kp': heading_correction_kp,
#                 'max_angular_velocity': max_angular_velocity,
#                 'curve_radii': curve_radii
#             }
#         except Exception as e:
#             self.get_logger().error(f'❌ Error getting vehicle parameters: {e}')
#             return {
#                 'wheelbase': 1.67,
#                 'max_steering_angle': 0.873,
#                 'fixed_speed': 1.0,
#                 'auto_start': True,
#                 'search_radius': 50.0,
#                 'heading_correction_kp': 0.02,
#                 'max_angular_velocity': 1.0,
#                 'curve_radii': {
#                     'curve1': 15.0,
#                     'curve2': 20.0,
#                     'curve3': 18.0,
#                     'curve4': 12.0
#                 }
#             }
    
#     def log_vehicle_parameters(self):
#         """แสดง Vehicle Parameters ปัจจุบัน"""
#         params = self.get_vehicle_parameters()
#         self.get_logger().info(
#             f'🚗 Vehicle Parameters:\n'
#             f'  🔧 Wheelbase: {params["wheelbase"]:.2f} m\n'
#             f'  🔧 Max Steering Angle: {params["max_steering_angle"]:.3f} rad ({math.degrees(params["max_steering_angle"]):.1f}°)\n'
#             f'  🔧 Fixed Speed: {params["fixed_speed"]:.2f} m/s\n'
#             f'  🔧 Auto Start: {params["auto_start"]}\n'
#             f'  🔧 Search Radius: {params["search_radius"]:.1f} m\n'
#             f'  🔧 Heading Correction Kp: {params["heading_correction_kp"]:.3f}\n'
#             f'  🔧 Max Angular Velocity: {params["max_angular_velocity"]:.2f} rad/s'
#         )
    
#     def log_curve_radii(self):
#         """แสดงรัศมีของทางโค้ง"""
#         params = self.get_vehicle_parameters()
#         curve_radii = params['curve_radii']
        
#         self.get_logger().info(
#             f'🌀 Curve Radii Configuration:\n'
#             f'  🔄 Curve 1 (Waypoint 245-260): {curve_radii["curve1"]:.1f} m\n'
#             f'  🔄 Curve 2 (Waypoint 316-390): {curve_radii["curve2"]:.1f} m\n'
#             f'  🔄 Curve 3 (Waypoint 501-579): {curve_radii["curve3"]:.1f} m\n'
#             f'  🔄 Curve 4 (Waypoint 631-646): {curve_radii["curve4"]:.1f} m'
#         )
    
#     def navigation_control(self):
#         """✅ ระบบควบคุมการนำทางด้วย Smart Starting Point"""
        
#         # ✅ ตรวจสอบการเริ่มต้นการนำทาง
#         if not self.waypoint_manager.navigation_started:
#             if self.final_heading is not None:
#                 # พยายามเริ่มต้นการนำทาง
#                 if self.waypoint_manager.initialize_starting_point(self.current_position, self.final_heading):
#                     self.navigation_active = True
#                     self.get_logger().info('🚀 Smart navigation started!')
#             return
        
#         if not self.navigation_active or self.final_heading is None:
#             return
        
#         # ได้ตำแหน่งปัจจุบัน
#         current_waypoint = self.waypoint_manager.get_current_waypoint()
#         if current_waypoint is None:
#             self.get_logger().info('🏁 Navigation completed - reached final waypoint')
#             self.navigation_active = False
#             self.send_stop_command()
#             return
        
#         # ตรวจสอบว่าถึง waypoint แล้วหรือยัง
#         distance_to_waypoint = math.sqrt(
#             (self.current_position['x'] - current_waypoint['x'])**2 + 
#             (self.current_position['y'] - current_waypoint['y'])**2
#         )
        
#         if distance_to_waypoint < self.waypoint_reached_threshold:
#             if self.waypoint_manager.advance_waypoint():
#                 progress = self.waypoint_manager.get_progress_info()
#                 self.get_logger().info(
#                     f'✅ Reached waypoint {progress["current"]-1} '
#                     f'({progress["progress_percent"]:.1f}% complete)'
#                 )
#             else:
#                 self.get_logger().info('🏁 Navigation completed')
#                 self.navigation_active = False
#                 self.send_stop_command()
#                 return
        
#         # ได้ข้อมูลเส้นทางปัจจุบัน
#         segment_info = self.waypoint_manager.get_current_segment_info()
#         segment_type = segment_info['type']
#         direction = segment_info['direction']
#         target_heading = segment_info.get('target_heading')
        
#         # ได้ parameters
#         params = self.get_vehicle_parameters()
        
#         # ใช้ความเร็วคงที่ 1 m/s
#         linear_velocity = self.FIXED_SPEED
        
#         # คำนวณคำสั่งควบคุมตามประเภทเส้นทาง
#         if segment_type == 'straight':
#             steering_angle = self.calculate_straight_path_control_with_bicycle_model(
#                 current_waypoint, target_heading, params
#             )
#         else:  # curve
#             steering_angle = self.calculate_curve_path_control_with_radius(direction, params)
        
#         # จำกัดมุมเลี้ยว
#         steering_angle = self.bicycle_model.limit_steering_angle(
#             steering_angle, params['max_steering_angle']
#         )
        
#         # คำนวณ angular velocity ด้วย bicycle model
#         angular_velocity = self.bicycle_model.calculate_angular_velocity(
#             linear_velocity, steering_angle
#         )
        
#         # จำกัด angular velocity
#         angular_velocity = max(-params['max_angular_velocity'], 
#                              min(params['max_angular_velocity'], angular_velocity))
        
#         # สร้างและส่งคำสั่ง cmd_vel
#         cmd_vel = Twist()
#         cmd_vel.linear.x = linear_velocity
#         cmd_vel.angular.z = angular_velocity
        
#         self.cmd_vel_publisher.publish(cmd_vel)
#         self.stats['navigation_commands'] += 1
        
#         # Debug information
#         debug_msg = String()
#         progress = self.waypoint_manager.get_progress_info()
        
#         if segment_type == 'curve':
#             curve_radius = params['curve_radii'].get(direction, 15.0)
#             debug_msg.data = (
#                 f"Segment: {segment_type} ({direction}, R={curve_radius:.1f}m) | "
#                 f"Waypoint: {progress['current']}/{progress['total']} ({progress['progress_percent']:.1f}%) | "
#                 f"Distance: {distance_to_waypoint:.2f}m | "
#                 f"Steering: {math.degrees(steering_angle):.1f}° | "
#                 f"Speed: {linear_velocity:.2f}m/s | "
#                 f"Angular: {angular_velocity:.2f}rad/s"
#             )
#         else:
#             heading_error = self.bicycle_model.normalize_angle_difference(target_heading - self.final_heading) if target_heading is not None else 0.0
#             debug_msg.data = (
#                 f"Segment: {segment_type} ({direction}, Target={target_heading:.0f}°) | "
#                 f"Waypoint: {progress['current']}/{progress['total']} ({progress['progress_percent']:.1f}%) | "
#                 f"Distance: {distance_to_waypoint:.2f}m | "
#                 f"Heading: {self.final_heading:.1f}° (Error: {heading_error:.1f}°) | "
#                 f"Steering: {math.degrees(steering_angle):.1f}° | "
#                 f"Speed: {linear_velocity:.2f}m/s | "
#                 f"Angular: {angular_velocity:.2f}rad/s"
#             )
        
#         self.navigation_debug_publisher.publish(debug_msg)
    
#     def calculate_straight_path_control_with_bicycle_model(self, target_waypoint, target_heading, params):
#         """✅ คำนวณการควบคุมสำหรับเส้นทางตรงด้วย Bicycle Model + Heading Control"""
        
#         # ถ้าไม่มี target_heading ให้ใช้ Pure Pursuit
#         if target_heading is None:
#             steering_angle = self.pure_pursuit.calculate_steering_angle(
#                 self.current_position,
#                 self.final_heading,
#                 target_waypoint,
#                 params['wheelbase']
#             )
#             return steering_angle * 0.5  # ลดความแรงของการเลี้ยว
        
#         # ✅ ใช้ Bicycle Model เพื่อรักษาทิศทางคงที่
#         heading_correction_steering = self.bicycle_model.calculate_heading_correction_steering(
#             self.final_heading,
#             target_heading,
#             params['heading_correction_kp']
#         )
        
#         # เพิ่ม Pure Pursuit เล็กน้อยเพื่อนำทางไปยัง waypoint
#         pure_pursuit_steering = self.pure_pursuit.calculate_steering_angle(
#             self.current_position,
#             self.final_heading,
#             target_waypoint,
#             params['wheelbase']
#         )
        
#         combined_steering = 0.8 * heading_correction_steering + 0.2 * pure_pursuit_steering * 0.3
        
#         # นับสถิติการแก้ไขทิศทาง
#         if abs(heading_correction_steering) > 0.01:  # มากกว่า ~0.6 องศา
#             self.stats['heading_corrections'] += 1
        
#         return combined_steering
    
#     def calculate_curve_path_control_with_radius(self, curve_direction, params):
#         """คำนวณการควบคุมสำหรับเส้นทางโค้งด้วยรัศมีที่กำหนด"""
#         # ได้รัศมีของทางโค้งจาก parameters
#         curve_radius = params['curve_radii'].get(curve_direction, 15.0)
        
#         # คำนวณมุมเลี้ยวจากรัศมีที่กำหนด
#         steering_angle = self.curvature_controller.calculate_steering_angle_with_radius(
#             curve_radius,
#             self.FIXED_SPEED,
#             params['wheelbase']
#         )
        
#         return steering_angle
    
#     def send_stop_command(self):
#         """ส่งคำสั่งหยุดรถ"""
#         cmd_vel = Twist()
#         cmd_vel.linear.x = 0.0
#         cmd_vel.angular.z = 0.0
#         self.cmd_vel_publisher.publish(cmd_vel)
#         self.get_logger().info('🛑 Stop command sent')
    
#     # ✅ Enhanced Heading Calculation Functions (from code 2)
#     def update_gps_heading_improved(self):
#         """🔧 Enhanced GPS heading calculation using UTM coordinates (compass bearing)"""
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
#                 # 🔧 คำนวณ compass bearing
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
#         """🌟 Enhanced final heading selection with NAV-ATT priority (compass bearing)"""
#         current_time = time.time()
        
#         # Check data validity
#         nav_att_valid = (self.nav_att_heading is not None and 
#                         (current_time - self.nav_att_time) < self.heading_timeout and
#                         self.fusion_mode in ["FUSION", "INIT"])
        
#         gps_valid = (self.gps_heading is not None and 
#                     (current_time - self.gps_heading_time) < self.heading_timeout)
        
#         # 🌟 Enhanced priority: NAV-ATT > GPS > Hold
#         if nav_att_valid and self.imu_calibration_status in ["FULLY_CALIBRATED", "CALIBRATED"]:
#             self.final_heading = self.nav_att_heading  # compass bearing (degrees)
#             self.heading_source = f"NAV_ATT_{self.imu_calibration_status}"
            
#         elif nav_att_valid and self.imu_calibration_status == "CALIBRATING":
#             if gps_valid:
#                 self.final_heading = self.blend_headings(self.nav_att_heading, self.gps_heading, 0.8)
#                 self.heading_source = "NAV_ATT_GPS_BLEND"
#             else:
#                 self.final_heading = self.nav_att_heading
#                 self.heading_source = "NAV_ATT_CALIBRATING"
                
#         elif gps_valid:
#             self.final_heading = self.gps_heading  # compass bearing (degrees)
#             self.heading_source = "GPS"
            
#         else:
#             self.final_heading = None
#             self.heading_source = "NONE"
        
#         # ✅ Publish enhanced heading (เหมือนโค้ดที่ 2)
#         if self.final_heading is not None:
#             heading_msg = Float32()
#             heading_msg.data = float(self.final_heading)
#             self.heading_publisher.publish(heading_msg)
#             self.stats['filtered_count'] += 1
            
#             # ✅ Publish heading debug info
#             debug_msg = String()
#             debug_msg.data = (
#                 f"Heading: {self.final_heading:.1f}° | "
#                 f"Source: {self.heading_source} | "
#                 f"NAV-ATT: {self.nav_att_heading:.1f}° | "
#                 f"GPS: {self.gps_heading:.1f}° | "
#                 f"Fusion: {self.fusion_mode} | "
#                 f"IMU Cal: {self.imu_calibration_status}"
#             )
#             self.heading_debug_publisher.publish(debug_msg)
    
#     def blend_headings(self, heading1, heading2, weight1):
#         """Blend two headings considering circular nature (compass bearing)"""
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
    
#     # เพิ่มฟังก์ชันที่ขาดหายไปจากโค้ดเดิม
#     def init_track_coordinate_system(self):
#         """Initialize coordinate system"""
#         if self.reference_utm is None:
#             self.reference_utm = self.transformer.transform(
#                 self.reference_point[1], self.reference_point[0]
#             )
        
#         self.utm_offset_x = self.TARGET_UTM_X - self.reference_utm[0]
#         self.utm_offset_y = self.TARGET_UTM_Y - self.reference_utm[1]
        
#         self.get_logger().info(f'🗺️ Track coordinate system initialized')
#         self.get_logger().info(f'📍 Reference UTM: {self.reference_utm}')
#         self.get_logger().info(f'🎯 Target UTM: ({self.TARGET_UTM_X}, {self.TARGET_UTM_Y})')
#         self.get_logger().info(f'📐 Offsets: ({self.utm_offset_x:.2f}, {self.utm_offset_y:.2f})')
    
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
        
#         if not self.ser or not self.ser.is_open:
#             self.get_logger().error('❌ No serial connection available')
    
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
#         import base64
#         credentials = f"{self.ntrip_username}:{self.ntrip_password}"
#         return base64.b64encode(credentials.encode()).decode()
    
#     def init_pyubx2(self):
#         """Initialize pyubx2 if available"""
#         if PYUBX2_AVAILABLE:
#             self.get_logger().info('✅ pyubx2 initialized for enhanced UBX parsing')
    
#     def configure_enhanced_zed_f9r(self):
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
#             # Process NTRIP corrections
#             if hasattr(self, 'ntrip_socket') and self.ntrip_socket:
#                 self.process_ntrip_corrections()
            
#             # Process serial data
#             if self.ser and self.ser.is_open:
#                 self.process_serial_data()
            
#             # Update GPS heading
#             self.update_gps_heading_improved()
            
#             # Update final heading
#             self.update_enhanced_final_heading()
            
#             # Navigation control
#             self.navigation_control()
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Error in main loop: {e}')
    
#     def process_ntrip_corrections(self):
#         """Process NTRIP correction data"""
#         try:
#             self.ntrip_socket.settimeout(0.01)
#             rtcm_data = self.ntrip_socket.recv(1024)
#             if rtcm_data:
#                 self.ser.write(rtcm_data)
#         except socket.timeout:
#             pass
#         except Exception as e:
#             self.get_logger().error(f'❌ NTRIP error: {e}')
    
#     def process_serial_data(self):
#         """Process serial data from ZED-F9R"""
#         try:
#             if self.ser.in_waiting > 0:
#                 data = self.ser.read(self.ser.in_waiting)
#                 self.ubx_buffer.extend(data)
                
#                 # Process UBX messages with pyubx2 if available
#                 if PYUBX2_AVAILABLE:
#                     self.process_ubx_messages_pyubx2()
                
#                 # Process NMEA messages
#                 self.process_nmea_messages(data)
                
#         except Exception as e:
#             self.get_logger().error(f'❌ Serial processing error: {e}')
    
#     def process_ubx_messages_pyubx2(self):
#         """Process UBX messages using pyubx2"""
#         try:
#             while len(self.ubx_buffer) >= 8:  # Minimum UBX message size
#                 # Find UBX sync bytes
#                 sync_pos = self.ubx_buffer.find(b'\xB5\x62')
#                 if sync_pos == -1:
#                     self.ubx_buffer.clear()
#                     break
                
#                 if sync_pos > 0:
#                     self.ubx_buffer = self.ubx_buffer[sync_pos:]
                
#                 if len(self.ubx_buffer) < 8:
#                     break
                
#                 # Get message length
#                 msg_length = struct.unpack('<H', self.ubx_buffer[4:6])[0] + 8
                
#                 if len(self.ubx_buffer) < msg_length:
#                     break
                
#                 # Extract message
#                 msg_data = bytes(self.ubx_buffer[:msg_length])
#                 self.ubx_buffer = self.ubx_buffer[msg_length:]
                
#                 # Parse with pyubx2
#                 try:
#                     parsed_msg = UBXMessage.parse(msg_data)
#                     self.process_parsed_ubx_message(parsed_msg)
#                     self.stats['pyubx2_success'] += 1
                    
#                 except Exception as parse_error:
#                     self.stats['pyubx2_errors'] += 1
#                     continue
                    
#         except Exception as e:
#             self.get_logger().error(f'❌ pyubx2 processing error: {e}')
    
#     def process_parsed_ubx_message(self, msg):
#         """Process parsed UBX message"""
#         try:
#             msg_class = msg.msg_cls
#             msg_id = msg.msg_id
            
#             # NAV-ATT: Attitude Solution
#             if msg_class == 1 and msg_id == 5:  # NAV-ATT
#                 self.process_nav_att_message(msg)
                
#             # ESF-STATUS: Sensor Fusion Status
#             elif msg_class == 16 and msg_id == 16:  # ESF-STATUS
#                 self.process_esf_status_message(msg)
                
#             # ESF-ALG: IMU Alignment Info
#             elif msg_class == 16 and msg_id == 20:  # ESF-ALG
#                 self.process_esf_alg_message(msg)
                
#             # ESF-MEAS: Sensor Measurements
#             elif msg_class == 16 and msg_id == 2:  # ESF-MEAS
#                 self.process_esf_meas_message(msg)
                
#         except Exception as e:
#             self.get_logger().error(f'❌ Error processing UBX message: {e}')
    
#     def process_nav_att_message(self, msg):
#         """Process NAV-ATT message for enhanced heading"""
#         try:
#             # Extract heading from NAV-ATT
#             heading_deg = msg.heading / 100000.0  # Convert from 1e-5 degrees
            
#             # Convert to compass bearing (0-360°)
#             compass_heading = heading_deg % 360
            
#             # Calculate confidence factor based on heading accuracy
#             heading_acc = getattr(msg, 'headAcc', 180000) / 100000.0  # Convert from 1e-5 degrees
#             confidence_factor = max(0.1, min(2.0, 30.0 / max(heading_acc, 1.0)))
            
#             # Apply enhanced filtering
#             filtered_heading = self.heading_filter.filter_nav_att_heading(
#                 compass_heading, confidence_factor
#             )
            
#             if filtered_heading is not None:
#                 self.nav_att_heading = filtered_heading
#                 self.nav_att_time = time.time()
#                 self.stats['nav_att_count'] += 1
                
#                 # Update fusion mode
#                 self.fusion_mode = "FUSION" if hasattr(msg, 'version') else "INIT"
                
#                 # Update calibration status based on accuracy
#                 if heading_acc < 2.0:
#                     self.imu_calibration_status = "FULLY_CALIBRATED"
#                 elif heading_acc < 5.0:
#                     self.imu_calibration_status = "CALIBRATED"
#                 elif heading_acc < 15.0:
#                     self.imu_calibration_status = "CALIBRATING"
#                 else:
#                     self.imu_calibration_status = "UNCALIBRATED"
                    
#         except Exception as e:
#             self.get_logger().error(f'❌ Error processing NAV-ATT: {e}')
    
#     def process_esf_status_message(self, msg):
#         """Process ESF-STATUS message"""
#         try:
#             self.esf_status = msg
#             self.stats['esf_status_count'] += 1
            
#             # Update fusion mode based on ESF status
#             if hasattr(msg, 'fusionMode'):
#                 fusion_mode_map = {
#                     0: "INIT",
#                     1: "FUSION",
#                     2: "SUSPENDED",
#                     3: "DISABLED"
#                 }
#                 self.fusion_mode = fusion_mode_map.get(msg.fusionMode, "UNKNOWN")
                
#         except Exception as e:
#             self.get_logger().error(f'❌ Error processing ESF-STATUS: {e}')
    
#     def process_esf_alg_message(self, msg):
#         """Process ESF-ALG message"""
#         try:
#             self.esf_alg = msg
            
#             # Update IMU calibration status
#             if hasattr(msg, 'status'):
#                 status_flags = msg.status
#                 if status_flags & 0x08:  # IMU-mount alignment confirmed
#                     self.imu_calibration_status = "FULLY_CALIBRATED"
#                 elif status_flags & 0x04:  # IMU-mount alignment ongoing
#                     self.imu_calibration_status = "CALIBRATING"
#                 else:
#                     self.imu_calibration_status = "UNCALIBRATED"
                    
#         except Exception as e:
#             self.get_logger().error(f'❌ Error processing ESF-ALG: {e}')
    
#     def process_esf_meas_message(self, msg):
#         """Process ESF-MEAS message"""
#         try:
#             self.stats['esf_meas_count'] += 1
#             # ESF-MEAS contains raw sensor measurements
#             # Can be used for advanced sensor fusion analysis
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Error processing ESF-MEAS: {e}')
    
#     def process_nmea_messages(self, data):
#         """Process NMEA messages"""
#         try:
#             data_str = data.decode('utf-8', errors='ignore')
#             lines = data_str.split('\n')
            
#             for line in lines:
#                 line = line.strip()
#                 if line.startswith('$'):
#                     try:
#                         msg = pynmea2.parse(line)
#                         self.process_nmea_message(msg)
#                     except Exception:
#                         continue
                        
#         except Exception as e:
#             self.get_logger().error(f'❌ NMEA processing error: {e}')
    
#     def process_nmea_message(self, msg):
#         """Process individual NMEA message"""
#         try:
#             # Process GGA messages for position
#             if hasattr(msg, 'sentence_type') and msg.sentence_type == 'GGA':
#                 self.process_gga_message(msg)
                
#         except Exception as e:
#             self.get_logger().error(f'❌ Error processing NMEA message: {e}')
    
#     def process_gga_message(self, msg):
#         """Process GGA message for position"""
#         try:
#             if msg.latitude and msg.longitude and msg.gps_qual > 0:
#                 lat = float(msg.latitude)
#                 lon = float(msg.longitude)
                
#                 # Convert to UTM
#                 utm_x, utm_y = self.transformer.transform(lon, lat)
                
#                 # Convert to track coordinates
#                 track_x = utm_x + self.utm_offset_x
#                 track_y = utm_y + self.utm_offset_y
                
#                 # Update current position
#                 self.current_position = {'x': track_x, 'y': track_y}
#                 self.current_utm_x = utm_x
#                 self.current_utm_y = utm_y
                
#                 # Add to position history
#                 position_data = {
#                     'x': track_x,
#                     'y': track_y,
#                     'utm_x': utm_x,
#                     'utm_y': utm_y,
#                     'timestamp': time.time()
#                 }
#                 self.position_history.append(position_data)
                
#                 # Publish position data
#                 self.publish_position_data(lat, lon, track_x, track_y)
                
#                 self.stats['position_count'] += 1
                
#         except Exception as e:
#             self.get_logger().error(f'❌ Error processing GGA: {e}')
    
#     def publish_position_data(self, lat, lon, track_x, track_y):
#         """Publish position data to ROS topics"""
#         try:
#             # Publish GNSS data
#             gnss_msg = NavSatFix()
#             gnss_msg.header.stamp = self.get_clock().now().to_msg()
#             gnss_msg.header.frame_id = "gnss"
#             gnss_msg.latitude = lat
#             gnss_msg.longitude = lon
#             gnss_msg.altitude = 0.0
#             self.gnss_publisher.publish(gnss_msg)
            
#             # Publish XY data
#             xy_msg = Float64MultiArray()
#             xy_msg.data = [track_x, track_y]
#             self.xy_publisher.publish(xy_msg)
            
#             # Publish raw GPS data
#             raw_gps_msg = Float64MultiArray()
#             raw_gps_msg.data = [lat, lon]
#             self.raw_gps_publisher.publish(raw_gps_msg)
            
#             # Publish XY debug
#             xy_debug_msg = Point()
#             xy_debug_msg.x = track_x
#             xy_debug_msg.y = track_y
#             xy_debug_msg.z = 0.0
#             self.xy_debug_publisher.publish(xy_debug_msg)
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Error publishing position data: {e}')

# def main(args=None):
#     """Main function"""
#     rclpy.init(args=args)
    
#     try:
#         node = EnhancedZEDf9rGNSSWithSmartNavigation()
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         print('\n🛑 Shutting down Enhanced ZED-F9R GNSS with Smart Navigation...')
#     except Exception as e:
#         print(f'❌ Error: {e}')
#     finally:
#         if 'node' in locals():
#             node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



    # -----------------------------------คลอส----------------------------------

#     import serial
# import pynmea2
# import socket
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
# from std_msgs.msg import Float32, String, Float64MultiArray
# from geometry_msgs.msg import Point, Twist
# from rcl_interfaces.msg import ParameterDescriptor
# import math
# import pyproj
# from collections import deque
# import time
# import glob
# import os
# import struct
# import json
# import numpy as np
# import csv

# # เพิ่ม pyubx2 สำหรับ UBX parsing ที่ดีขึ้น
# try:
#     from pyubx2 import UBXReader, UBXMessage, UBX_MSGIDS
#     PYUBX2_AVAILABLE = True
#     print("✅ pyubx2 available - using enhanced UBX parsing")
# except ImportError:
#     PYUBX2_AVAILABLE = False
#     print("⚠️ pyubx2 not available - install with: pip3 install pyubx2")

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

# # Enhanced Waypoint Manager with Smart Starting Point Selection
# class WaypointManager:
#     def __init__(self, csv_file_path, auto_start=True):
#         self.waypoints = []
#         self.current_waypoint_index = 0
#         self.auto_start = auto_start
#         self.navigation_started = False
#         self.load_waypoints_from_csv(csv_file_path)
        
#         # กำหนดช่วง waypoint และประเภทเส้นทาง พร้อมทิศทางที่ต้องรักษา
#         self.path_segments = [
#             {'start': 0, 'end': 240, 'type': 'straight', 'direction': 'S-N', 'target_heading': 0.0},      # waypoint 1-240 (North = 0°)
#             {'start': 241, 'end': 256, 'type': 'curve', 'direction': 'curve1', 'target_heading': None},    # waypoint 245-260
#             {'start': 257, 'end': 315, 'type': 'straight', 'direction': 'E-W', 'target_heading': 90.0},   # waypoint 261-315 (East = 90°)
#             {'start': 316, 'end': 383, 'type': 'curve', 'direction': 'curve2', 'target_heading': None},    # waypoint 316-390
#             {'start': 384, 'end': 496, 'type': 'straight', 'direction': 'N-S', 'target_heading': 180.0},  # waypoint 391-500 (South = 180°)
#             {'start': 497, 'end': 571, 'type': 'curve', 'direction': 'curve3', 'target_heading': None},    # waypoint 501-579
#             {'start': 572, 'end': 622, 'type': 'straight', 'direction': 'W-E', 'target_heading': 270.0},  # waypoint 580-630 (West = 270°)
#             {'start': 623, 'end': 642, 'type': 'curve', 'direction': 'curve4', 'target_heading': None},    # waypoint 631-646
#             {'start': 643, 'end': 654, 'type': 'straight', 'direction': 'S-N', 'target_heading': 0.0}     # waypoint 647-659 (North = 0°)
#         ]
        
#         # กำหนดรัศมีสำหรับแต่ละทางโค้ง
#         self.curve_radii = {
#             'curve1': 15.0,  # รัศมี 15 เมตร สำหรับโค้งที่ 1
#             'curve2': 20.0,  # รัศมี 20 เมตร สำหรับโค้งที่ 2
#             'curve3': 18.0,  # รัศมี 18 เมตร สำหรับโค้งที่ 3
#             'curve4': 12.0   # รัศมี 12 เมตร สำหรับโค้งที่ 4
#         }
        
#         print(f"✅ Loaded {len(self.waypoints)} waypoints from CSV")
#         if auto_start:
#             print("🚀 Auto-start mode enabled - will find nearest forward waypoint")
#         else:
#             print("🎯 Manual start mode - will start from waypoint 0")
    
#     def load_waypoints_from_csv(self, csv_file_path):
#         """โหลด waypoints จากไฟล์ CSV"""
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
#         """หา waypoint ที่ใกล้ที่สุดที่อยู่หน้ารถ"""
#         if not current_position or not self.waypoints:
#             return 0
        
#         current_x, current_y = current_position['x'], current_position['y']
#         best_waypoint = None
#         best_index = 0
#         best_score = float('inf')
        
#         candidates = []
        
#         for i, waypoint in enumerate(self.waypoints):
#             # คำนวณระยะทาง
#             dx = waypoint['x'] - current_x
#             dy = waypoint['y'] - current_y
#             distance = math.sqrt(dx**2 + dy**2)
            
#             # ข้ามถ้าไกลเกินไป
#             if distance > search_radius:
#                 continue
            
#             # คำนวณทิศทางไปยัง waypoint (compass bearing)
#             waypoint_bearing = math.degrees(math.atan2(dx, dy))  # atan2(East, North)
#             if waypoint_bearing < 0:
#                 waypoint_bearing += 360
            
#             # คำนวณความแตกต่างของมุม
#             heading_diff = abs(self.normalize_angle_difference(waypoint_bearing - current_heading))
            
#             # ตรวจสอบว่าอยู่หน้ารถหรือไม่ (มุมไม่เกิน ±90°)
#             if heading_diff <= 90.0:
#                 # คำนวณคะแนน (ยิ่งใกล้และอยู่หน้าตรงยิ่งดี)
#                 distance_score = distance
#                 angle_score = heading_diff / 90.0  # normalize เป็น 0-1
#                 combined_score = distance_score + (angle_score * 10.0)  # ให้น้ำหนักกับมุม
                
#                 candidates.append({
#                     'index': i,
#                     'distance': distance,
#                     'heading_diff': heading_diff,
#                     'score': combined_score,
#                     'waypoint': waypoint
#                 })
        
#         if candidates:
#             # เรียงตามคะแนน
#             candidates.sort(key=lambda x: x['score'])
#             best_candidate = candidates[0]
            
#             print(f"🎯 Found {len(candidates)} forward waypoints within {search_radius}m:")
#             for i, candidate in enumerate(candidates[:5]):  # แสดง 5 อันดับแรก
#                 print(f"  #{i+1}: Waypoint {candidate['index']} - "
#                       f"Distance: {candidate['distance']:.2f}m, "
#                       f"Angle: {candidate['heading_diff']:.1f}°, "
#                       f"Score: {candidate['score']:.2f}")
            
#             print(f"✅ Selected waypoint {best_candidate['index']} as starting point")
#             return best_candidate['index']
#         else:
#             # ถ้าไม่เจอ waypoint ที่อยู่หน้ารถ ให้หาที่ใกล้ที่สุด
#             print(f"⚠️ No forward waypoints found within {search_radius}m")
#             return self.find_nearest_waypoint(current_position)
    
#     def find_nearest_waypoint(self, current_position):
#         """หา waypoint ที่ใกล้ที่สุด (ไม่สนใจทิศทาง)"""
#         if not current_position or not self.waypoints:
#             return 0
        
#         current_x, current_y = current_position['x'], current_position['y']
#         min_distance = float('inf')
#         nearest_index = 0
        
#         for i, waypoint in enumerate(self.waypoints):
#             distance = math.sqrt(
#                 (waypoint['x'] - current_x)**2 + 
#                 (waypoint['y'] - current_y)**2
#             )
#             if distance < min_distance:
#                 min_distance = distance
#                 nearest_index = i
        
#         print(f"🔍 Nearest waypoint: {nearest_index} (distance: {min_distance:.2f}m)")
#         return nearest_index
    
#     def initialize_starting_point(self, current_position, current_heading):
#         """เริ่มต้นการนำทางจากจุดที่เหมาะสม"""
#         if self.navigation_started:
#             return False
        
#         if not current_position or current_heading is None:
#             print("⚠️ Waiting for position and heading data...")
#             return False
        
#         if self.auto_start:
#             # หา waypoint ที่ใกล้ที่สุดที่อยู่หน้ารถ
#             self.current_waypoint_index = self.find_nearest_forward_waypoint(
#                 current_position, current_heading
#             )
#         else:
#             # เริ่มจาก waypoint แรก
#             self.current_waypoint_index = 0
        
#         self.navigation_started = True
        
#         print(f"🚀 Navigation started from waypoint {self.current_waypoint_index + 1}")
#         print(f"📍 Target: ({self.waypoints[self.current_waypoint_index]['x']:.2f}, "
#               f"{self.waypoints[self.current_waypoint_index]['y']:.2f})")
        
#         return True
    
#     def normalize_angle_difference(self, angle_diff):
#         """ปรับความแตกต่างของมุมให้อยู่ในช่วง [-180, 180]"""
#         while angle_diff > 180:
#             angle_diff -= 360
#         while angle_diff < -180:
#             angle_diff += 360
#         return angle_diff
    
#     def set_starting_waypoint(self, waypoint_index):
#         """กำหนดจุดเริ่มต้นใหม่"""
#         if 0 <= waypoint_index < len(self.waypoints):
#             self.current_waypoint_index = waypoint_index
#             self.navigation_started = True
#             print(f"✅ Starting waypoint set to {waypoint_index + 1}")
#             return True
#         else:
#             print(f"❌ Invalid waypoint index: {waypoint_index}")
#             return False
    
#     def get_current_segment_info(self):
#         """ได้ข้อมูลของเส้นทางปัจจุบัน"""
#         for segment in self.path_segments:
#             if segment['start'] <= self.current_waypoint_index <= segment['end']:
#                 return segment
#         return {'type': 'straight', 'direction': 'unknown', 'target_heading': 0.0}
    
#     def get_curve_radius(self, curve_direction):
#         """ได้รัศมีของทางโค้ง"""
#         return self.curve_radii.get(curve_direction, 15.0)  # default 15 เมตร
    
#     def get_current_waypoint(self):
#         """ได้ waypoint ปัจจุบัน"""
#         if self.current_waypoint_index < len(self.waypoints):
#             return self.waypoints[self.current_waypoint_index]
#         return None
    
#     def get_next_waypoint(self):
#         """ได้ waypoint ถัดไป"""
#         next_index = self.current_waypoint_index + 1
#         if next_index < len(self.waypoints):
#             return self.waypoints[next_index]
#         return None
    
#     def advance_waypoint(self):
#         """เลื่อนไปยัง waypoint ถัดไป"""
#         if self.current_waypoint_index < len(self.waypoints) - 1:
#             self.current_waypoint_index += 1
#             return True
#         return False
    
#     def get_progress_info(self):
#         """ได้ข้อมูลความคืบหน้า"""
#         return {
#             'current': self.current_waypoint_index + 1,
#             'total': len(self.waypoints),
#             'progress_percent': ((self.current_waypoint_index + 1) / len(self.waypoints)) * 100
#         }

# # Bicycle Kinematic Model for Straight Path Control
# class BicycleKinematicModel:
#     def __init__(self, wheelbase):
#         self.wheelbase = wheelbase
    
#     def calculate_angular_velocity(self, linear_velocity, steering_angle):
#         """คำนวณ angular velocity จาก bicycle model"""
#         if abs(steering_angle) < 1e-6:  # ป้องกันการหารด้วยศูนย์
#             return 0.0
        
#         angular_velocity = (linear_velocity * math.tan(steering_angle)) / self.wheelbase
#         return angular_velocity
    
#     def limit_steering_angle(self, steering_angle, max_steering_angle):
#         """จำกัดมุมเลี้ยวให้อยู่ในขอบเขต"""
#         return max(-max_steering_angle, min(max_steering_angle, steering_angle))
    
#     def calculate_heading_correction_steering(self, current_heading, target_heading, kp=0.02):
#         """คำนวณมุมเลี้ยวเพื่อแก้ไขทิศทางในทางตรง (bicycle model)"""
#         # คำนวณความผิดพลาดของทิศทาง
#         heading_error = self.normalize_angle_difference(target_heading - current_heading)
        
#         # ใช้ proportional control สำหรับการแก้ไขทิศทาง
#         # kp ต่ำ = การแก้ไขนุ่มนวล, kp สูง = การแก้ไขรวดเร็ว
#         steering_angle = kp * math.radians(heading_error)
        
#         return steering_angle
    
#     def normalize_angle_difference(self, angle_diff):
#         """ปรับความแตกต่างของมุมให้อยู่ในช่วง [-180, 180]"""
#         while angle_diff > 180:
#             angle_diff -= 360
#         while angle_diff < -180:
#             angle_diff += 360
#         return angle_diff

# # Pure Pursuit Controller
# class PurePursuitController:
#     def __init__(self, lookahead_distance=2.0):
#         self.lookahead_distance = lookahead_distance
    
#     def calculate_steering_angle(self, current_pos, current_heading, target_pos, wheelbase):
#         """คำนวณมุมเลี้ยวสำหรับ Pure Pursuit"""
#         # คำนวณระยะทางและมุมไปยังเป้าหมาย
#         dx = target_pos['x'] - current_pos['x']
#         dy = target_pos['y'] - current_pos['y']
        
#         target_heading = math.degrees(math.atan2(dx, dy))
#         heading_error = self.normalize_angle(target_heading - current_heading)
        
#         distance = math.sqrt(dx**2 + dy**2)
        
#         # Pure Pursuit formula
#         alpha = math.radians(heading_error)
#         steering_angle = math.atan2(2 * wheelbase * math.sin(alpha), distance)
        
#         return steering_angle
    
#     def normalize_angle(self, angle):
#         """Normalize angle to [-180, 180]"""
#         while angle > 180:
#             angle -= 360
#         while angle < -180:
#             angle += 360
#         return angle

# # Constant Curvature Controller
# class ConstantCurvatureController:
#     def __init__(self):
#         self.curvature_history = deque(maxlen=5)
    
#     def calculate_steering_angle_with_radius(self, curve_radius, linear_velocity, wheelbase):
#         """คำนวณมุมเลี้ยวจากรัศมีที่กำหนด"""
#         # คำนวณ curvature จากรัศมี: κ = 1/R
#         curvature = 1.0 / curve_radius
        
#         # คำนวณมุมเลี้ยวจาก curvature และ bicycle model
#         # δ = atan(L * κ) โดย L = wheelbase, κ = curvature
#         steering_angle = math.atan(wheelbase * curvature)
        
#         return steering_angle
    
#     def calculate_steering_angle(self, waypoints_segment, current_pos, wheelbase):
#         """คำนวณมุมเลี้ยวสำหรับ Constant Curvature (วิธีเดิม)"""
#         if len(waypoints_segment) < 3:
#             return 0.0
        
#         # คำนวณ curvature จาก 3 จุด
#         curvature = self.calculate_curvature(waypoints_segment)
#         self.curvature_history.append(curvature)
        
#         # ใช้ค่าเฉลี่ยของ curvature
#         avg_curvature = np.mean(list(self.curvature_history))
        
#         # คำนวณมุมเลี้ยวจาก curvature
#         steering_angle = math.atan(wheelbase * avg_curvature)
        
#         return steering_angle
    
#     def calculate_curvature(self, points):
#         """คำนวณ curvature จาก 3 จุด"""
#         if len(points) < 3:
#             return 0.0
        
#         p1, p2, p3 = points[0], points[1], points[2]
        
#         # คำนวณ curvature โดยใช้สูตร
#         x1, y1 = p1['x'], p1['y']
#         x2, y2 = p2['x'], p2['y']
#         x3, y3 = p3['x'], p3['y']
        
#         # Area of triangle
#         area = 0.5 * abs((x2-x1)*(y3-y1) - (x3-x1)*(y2-y1))
        
#         # Side lengths
#         a = math.sqrt((x2-x3)**2 + (y2-y3)**2)
#         b = math.sqrt((x1-x3)**2 + (y1-y3)**2)
#         c = math.sqrt((x1-x2)**2 + (y1-y2)**2)
        
#         if a * b * c == 0:
#             return 0.0
        
#         # Curvature = 4 * Area / (a * b * c)
#         curvature = 4 * area / (a * b * c)
        
#         return curvature

# # Main Enhanced GNSS Node with Smart Starting Point
# class EnhancedZEDf9rGNSSWithSmartNavigation(Node):
#     def __init__(self):
#         super().__init__('enhanced_zedf9r_gnss_with_smart_navigation')
        
#         # Vehicle parameters
#         self.declare_vehicle_parameters()
        
#         # ROS2 Publishers
#         self.gnss_publisher = self.create_publisher(NavSatFix, '/navigation/gnss', 10)
#         self.xy_publisher = self.create_publisher(Float64MultiArray, 'Realtime_XY', 10)
#         self.raw_gps_publisher = self.create_publisher(Float64MultiArray, 'Raw_GPS', 10)
#         self.xy_debug_publisher = self.create_publisher(Point, '/navigation/xy_debug', 10)
#         self.heading_publisher = self.create_publisher(Float32, '/navigation/heading', 10)
#         self.heading_debug_publisher = self.create_publisher(String, '/navigation/heading_debug', 10)
#         self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.navigation_debug_publisher = self.create_publisher(String, '/navigation/debug', 10)
        
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
#         self.ntrip_socket = None
        
#         # Coordinate transformation
#         self.transformer = pyproj.Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)
        
#         # Position tracking
#         self.position_history = deque(maxlen=15)
#         self.min_movement_for_heading = 0.25
#         self.current_position = {'x': 0.0, 'y': 0.0}
#         self.last_position = None
#         self.last_position_time = 0
        
#         # Reference point
#         self.reference_point = (13.650748, 100.492985)
#         self.reference_utm = None
        
#         # UTM coordinates
#         self.TARGET_UTM_X = 661452.0
#         self.TARGET_UTM_Y = 1509601.0
#         self.utm_offset_x = 0.0
#         self.utm_offset_y = 0.0
        
#         # Enhanced Heading management
#         self.imu_heading = None
#         self.nav_att_heading = None
#         self.nav_att_yaw = None
#         self.gps_heading = None
#         self.final_heading = None
#         self.heading_source = "NONE"
        
#         self.imu_heading_time = 0
#         self.nav_att_time = 0
#         self.gps_heading_time = 0
#         self.heading_timeout = 8.0
        
#         # ZED-F9R specific data
#         self.esf_status = None
#         self.nav_att = None
#         self.esf_alg = None
#         self.fusion_mode = "NONE"
#         self.imu_calibration_status = "UNKNOWN"
        
#         # pyubx2 UBX parsing
#         self.ubx_reader = None
#         self.ubx_buffer = bytearray()
        
#         # Enhanced Heading filtering
#         self.heading_filter = EnhancedIMUHeadingFilter()
        
#         # ✅ Navigation Components with Smart Starting Point
#         csv_file_path = '/home/inc/ros2_ws/waypointfile/waypoints3xy.csv'  # แก้ไข path ให้ถูกต้อง
        
#         self.waypoint_manager = WaypointManager(csv_file_path, auto_start=True)  # ✅ เปิด auto-start
#         self.pure_pursuit = PurePursuitController(lookahead_distance=3.0)
#         self.curvature_controller = ConstantCurvatureController()
#         self.bicycle_model = BicycleKinematicModel(wheelbase=1.67)  # จะอัพเดทจาก parameter
        
#         # Navigation state
#         self.navigation_active = False  # ✅ เริ่มต้นเป็น False จนกว่าจะหาจุดเริ่มต้นได้
#         self.waypoint_reached_threshold = 1.0  # เมตร
        
#         # ✅ Fixed Speed Configuration
#         self.FIXED_SPEED = 1.0  # ความเร็วคงที่ 1 m/s
        
#         # Performance stats
#         self.stats = {
#             'imu_count': 0,
#             'nav_att_count': 0,
#             'nav_att_yaw_count': 0,
#             'gps_count': 0,
#             'position_count': 0,
#             'esf_meas_count': 0,
#             'esf_status_count': 0,
#             'pyubx2_success': 0,
#             'pyubx2_errors': 0,
#             'filtered_count': 0,
#             'navigation_commands': 0,
#             'heading_corrections': 0
#         }
        
#         # Initialize coordinate system
#         self.init_track_coordinate_system()
        
#         # Initialize serial connection
#         self.init_serial_connection()
        
#         if self.ser and self.ser.is_open:
#             self.ntrip_socket = self.connect_to_ntrip()
            
#             # Initialize pyubx2 if available
#             if PYUBX2_AVAILABLE:
#                 self.init_pyubx2()
            
#             # Configure ZED-F9R for enhanced IMU
#             self.configure_enhanced_zed_f9r()
            
#             # Main processing timer
#             self.create_timer(0.1, self.main_loop)
            
#             self.get_logger().info('✅ Enhanced ZED-F9R GNSS with Smart Navigation initialized')
#             self.get_logger().info(f'🔧 pyubx2: {"Available" if PYUBX2_AVAILABLE else "Not available"}')
#             self.get_logger().info(f'🗺️ Loaded {len(self.waypoint_manager.waypoints)} waypoints')
#             self.get_logger().info(f'🚀 Fixed Speed: {self.FIXED_SPEED} m/s')
#             self.get_logger().info('🎯 Smart starting point: Will find nearest forward waypoint')
            
#             # Log vehicle parameters
#             self.log_vehicle_parameters()
            
#             # Log curve radii
#             self.log_curve_radii()
#         else:
#             self.get_logger().error('❌ Failed to initialize')
    
#     def declare_vehicle_parameters(self):
#         """ประกาศ Vehicle Parameters"""
        
#         # Wheelbase parameter
#         wheelbase_descriptor = ParameterDescriptor(
#             description='ฐานล้อของรถ (ระยะห่างระหว่างเพลาหน้าและหลัง) ในหน่วยเมตร'
#         )
#         self.declare_parameter('wheelbase', 1.67, wheelbase_descriptor)
        
#         # Max steering angle parameter
#         steering_descriptor = ParameterDescriptor(
#             description='มุมเลี้ยวสูงสุดของรถในหน่วย radians (50 degrees = 0.873 rad)'
#         )
#         self.declare_parameter('max_steering_angle', 0.873, steering_descriptor)
        
#         # Fixed speed parameter
#         speed_descriptor = ParameterDescriptor(
#             description='ความเร็วคงที่ของรถในหน่วย m/s (ตั้งค่าเป็น 1.0 m/s)'
#         )
#         self.declare_parameter('fixed_speed', 1.0, speed_descriptor)
        
#         # ✅ Smart starting parameters
#         auto_start_descriptor = ParameterDescriptor(
#             description='เปิดใช้การหาจุดเริ่มต้นอัตโนมัติ (true = หาจุดที่ใกล้ที่สุดที่อยู่หน้ารถ)'
#         )
#         self.declare_parameter('auto_start', True, auto_start_descriptor)
        
#         search_radius_descriptor = ParameterDescriptor(
#             description='รัศมีการค้นหา waypoint ในหน่วยเมตร'
#         )
#         self.declare_parameter('search_radius', 50.0, search_radius_descriptor)
        
#         # ✅ Bicycle model parameters สำหรับการควบคุมทิศทางในทางตรง
#         heading_kp_descriptor = ParameterDescriptor(
#             description='Proportional gain สำหรับการแก้ไขทิศทางในทางตรง (ค่าต่ำ = นุ่มนวล)'
#         )
#         self.declare_parameter('heading_correction_kp', 0.02, heading_kp_descriptor)
        
#         # Curve radius parameters
#         curve1_radius_descriptor = ParameterDescriptor(
#             description='รัศมีของทางโค้งที่ 1 ในหน่วยเมตร'
#         )
#         self.declare_parameter('curve1_radius', 15.0, curve1_radius_descriptor)
        
#         curve2_radius_descriptor = ParameterDescriptor(
#             description='รัศมีของทางโค้งที่ 2 ในหน่วยเมตร'
#         )
#         self.declare_parameter('curve2_radius', 20.0, curve2_radius_descriptor)
        
#         curve3_radius_descriptor = ParameterDescriptor(
#             description='รัศมีของทางโค้งที่ 3 ในหน่วยเมตร'
#         )
#         self.declare_parameter('curve3_radius', 18.0, curve3_radius_descriptor)
        
#         curve4_radius_descriptor = ParameterDescriptor(
#             description='รัศมีของทางโค้งที่ 4 ในหน่วยเมตร'
#         )
#         self.declare_parameter('curve4_radius', 12.0, curve4_radius_descriptor)
        
#         # Angular velocity limits
#         max_angular_vel_descriptor = ParameterDescriptor(
#             description='ความเร็วเชิงมุมสูงสุดในหน่วย rad/s'
#         )
#         self.declare_parameter('max_angular_velocity', 1.0, max_angular_vel_descriptor)
        
#         self.get_logger().info('🌟 Vehicle parameters declared successfully!')
    
#     def get_vehicle_parameters(self):
#         """ดึงค่า Vehicle Parameters ปัจจุบัน"""
#         try:
#             wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
#             max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value
#             fixed_speed = self.get_parameter('fixed_speed').get_parameter_value().double_value
#             auto_start = self.get_parameter('auto_start').get_parameter_value().bool_value
#             search_radius = self.get_parameter('search_radius').get_parameter_value().double_value
#             heading_correction_kp = self.get_parameter('heading_correction_kp').get_parameter_value().double_value
#             max_angular_velocity = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
            
#             # ดึงค่ารัศมีของทางโค้ง
#             curve_radii = {
#                 'curve1': self.get_parameter('curve1_radius').get_parameter_value().double_value,
#                 'curve2': self.get_parameter('curve2_radius').get_parameter_value().double_value,
#                 'curve3': self.get_parameter('curve3_radius').get_parameter_value().double_value,
#                 'curve4': self.get_parameter('curve4_radius').get_parameter_value().double_value
#             }
            
#             return {
#                 'wheelbase': wheelbase,
#                 'max_steering_angle': max_steering_angle,
#                 'fixed_speed': fixed_speed,
#                 'auto_start': auto_start,
#                 'search_radius': search_radius,
#                 'heading_correction_kp': heading_correction_kp,
#                 'max_angular_velocity': max_angular_velocity,
#                 'curve_radii': curve_radii
#             }
#         except Exception as e:
#             self.get_logger().error(f'❌ Error getting vehicle parameters: {e}')
#             return {
#                 'wheelbase': 1.67,
#                 'max_steering_angle': 0.873,
#                 'fixed_speed': 1.0,
#                 'auto_start': True,
#                 'search_radius': 50.0,
#                 'heading_correction_kp': 0.02,
#                 'max_angular_velocity': 1.0,
#                 'curve_radii': {
#                     'curve1': 15.0,
#                     'curve2': 20.0,
#                     'curve3': 18.0,
#                     'curve4': 12.0
#                 }
#             }
    
#     def log_vehicle_parameters(self):
#         """แสดง Vehicle Parameters ปัจจุบัน"""
#         params = self.get_vehicle_parameters()
#         self.get_logger().info(
#             f'🚗 Vehicle Parameters:\n'
#             f'  🔧 Wheelbase: {params["wheelbase"]:.2f} m\n'
#             f'  🔧 Max Steering Angle: {params["max_steering_angle"]:.3f} rad ({math.degrees(params["max_steering_angle"]):.1f}°)\n'
#             f'  🔧 Fixed Speed: {params["fixed_speed"]:.2f} m/s\n'
#             f'  🔧 Auto Start: {params["auto_start"]}\n'
#             f'  🔧 Search Radius: {params["search_radius"]:.1f} m\n'
#             f'  🔧 Heading Correction Kp: {params["heading_correction_kp"]:.3f}\n'
#             f'  🔧 Max Angular Velocity: {params["max_angular_velocity"]:.2f} rad/s'
#         )
    
#     def log_curve_radii(self):
#         """แสดงรัศมีของทางโค้ง"""
#         params = self.get_vehicle_parameters()
#         curve_radii = params['curve_radii']
        
#         self.get_logger().info(
#             f'🌀 Curve Radii Configuration:\n'
#             f'  🔄 Curve 1 (Waypoint 245-260): {curve_radii["curve1"]:.1f} m\n'
#             f'  🔄 Curve 2 (Waypoint 316-390): {curve_radii["curve2"]:.1f} m\n'
#             f'  🔄 Curve 3 (Waypoint 501-579): {curve_radii["curve3"]:.1f} m\n'
#             f'  🔄 Curve 4 (Waypoint 631-646): {curve_radii["curve4"]:.1f} m'
#         )
    
#     def navigation_control(self):
#         """✅ ระบบควบคุมการนำทางด้วย Smart Starting Point"""
        
#         # ✅ ตรวจสอบการเริ่มต้นการนำทาง
#         if not self.waypoint_manager.navigation_started:
#             if self.final_heading is not None:
#                 # พยายามเริ่มต้นการนำทาง
#                 if self.waypoint_manager.initialize_starting_point(self.current_position, self.final_heading):
#                     self.navigation_active = True
#                     self.get_logger().info('🚀 Smart navigation started!')
#             return
        
#         if not self.navigation_active or self.final_heading is None:
#             return
        
#         # ได้ตำแหน่งปัจจุบัน
#         current_waypoint = self.waypoint_manager.get_current_waypoint()
#         if current_waypoint is None:
#             self.get_logger().info('🏁 Navigation completed - reached final waypoint')
#             self.navigation_active = False
#             self.send_stop_command()
#             return
        
#         # ตรวจสอบว่าถึง waypoint แล้วหรือยัง
#         distance_to_waypoint = math.sqrt(
#             (self.current_position['x'] - current_waypoint['x'])**2 + 
#             (self.current_position['y'] - current_waypoint['y'])**2
#         )
        
#         if distance_to_waypoint < self.waypoint_reached_threshold:
#             if self.waypoint_manager.advance_waypoint():
#                 progress = self.waypoint_manager.get_progress_info()
#                 self.get_logger().info(
#                     f'✅ Reached waypoint {progress["current"]-1} '
#                     f'({progress["progress_percent"]:.1f}% complete)'
#                 )
#             else:
#                 self.get_logger().info('🏁 Navigation completed')
#                 self.navigation_active = False
#                 self.send_stop_command()
#                 return
        
#         # ได้ข้อมูลเส้นทางปัจจุบัน
#         segment_info = self.waypoint_manager.get_current_segment_info()
#         segment_type = segment_info['type']
#         direction = segment_info['direction']
#         target_heading = segment_info.get('target_heading')
        
#         # ได้ parameters
#         params = self.get_vehicle_parameters()
        
#         # ใช้ความเร็วคงที่ 1 m/s
#         linear_velocity = self.FIXED_SPEED
        
#         # คำนวณคำสั่งควบคุมตามประเภทเส้นทาง
#         if segment_type == 'straight':
#             steering_angle = self.calculate_straight_path_control_with_bicycle_model(
#                 current_waypoint, target_heading, params
#             )
#         else:  # curve
#             steering_angle = self.calculate_curve_path_control_with_radius(direction, params)
        
#         # จำกัดมุมเลี้ยว
#         steering_angle = self.bicycle_model.limit_steering_angle(
#             steering_angle, params['max_steering_angle']
#         )
        
#         # คำนวณ angular velocity ด้วย bicycle model
#         angular_velocity = self.bicycle_model.calculate_angular_velocity(
#             linear_velocity, steering_angle
#         )
        
#         # จำกัด angular velocity
#         angular_velocity = max(-params['max_angular_velocity'], 
#                              min(params['max_angular_velocity'], angular_velocity))
        
#         # สร้างและส่งคำสั่ง cmd_vel
#         cmd_vel = Twist()
#         cmd_vel.linear.x = linear_velocity
#         cmd_vel.angular.z = angular_velocity
        
#         self.cmd_vel_publisher.publish(cmd_vel)
#         self.stats['navigation_commands'] += 1
        
#         # Debug information
#         debug_msg = String()
#         progress = self.waypoint_manager.get_progress_info()
        
#         if segment_type == 'curve':
#             curve_radius = params['curve_radii'].get(direction, 15.0)
#             debug_msg.data = (
#                 f"Segment: {segment_type} ({direction}, R={curve_radius:.1f}m) | "
#                 f"Waypoint: {progress['current']}/{progress['total']} ({progress['progress_percent']:.1f}%) | "
#                 f"Distance: {distance_to_waypoint:.2f}m | "
#                 f"Steering: {math.degrees(steering_angle):.1f}° | "
#                 f"Speed: {linear_velocity:.2f}m/s | "
#                 f"Angular: {angular_velocity:.2f}rad/s"
#             )
#         else:
#             heading_error = self.bicycle_model.normalize_angle_difference(target_heading - self.final_heading) if target_heading is not None else 0.0
#             debug_msg.data = (
#                 f"Segment: {segment_type} ({direction}, Target={target_heading:.0f}°) | "
#                 f"Waypoint: {progress['current']}/{progress['total']} ({progress['progress_percent']:.1f}%) | "
#                 f"Distance: {distance_to_waypoint:.2f}m | "
#                 f"Heading: {self.final_heading:.1f}° (Error: {heading_error:.1f}°) | "
#                 f"Steering: {math.degrees(steering_angle):.1f}° | "
#                 f"Speed: {linear_velocity:.2f}m/s | "
#                 f"Angular: {angular_velocity:.2f}rad/s"
#             )
        
#         self.navigation_debug_publisher.publish(debug_msg)
    
#     def calculate_straight_path_control_with_bicycle_model(self, target_waypoint, target_heading, params):
#         """✅ คำนวณการควบคุมสำหรับเส้นทางตรงด้วย Bicycle Model + Heading Control"""
        
#         # ถ้าไม่มี target_heading ให้ใช้ Pure Pursuit
#         if target_heading is None:
#             steering_angle = self.pure_pursuit.calculate_steering_angle(
#                 self.current_position,
#                 self.final_heading,
#                 target_waypoint,
#                 params['wheelbase']
#             )
#             return steering_angle * 0.5  # ลดความแรงของการเลี้ยว
        
#         # ✅ ใช้ Bicycle Model เพื่อรักษาทิศทางคงที่
#         heading_correction_steering = self.bicycle_model.calculate_heading_correction_steering(
#             self.final_heading,
#             target_heading,
#             params['heading_correction_kp']
#         )
        
#         # เพิ่ม Pure Pursuit เล็กน้อยเพื่อนำทางไปยัง waypoint
#         pure_pursuit_steering = self.pure_pursuit.calculate_steering_angle(
#             self.current_position,
#             self.final_heading,
#             target_waypoint,
#             params['wheelbase']
#         )
        
#         # ผสมผสานระหว่าง heading correction และ pure pursuit
#         # ให้น้ำหนัก 80% กับ heading correction, 20% กับ pure pursuit
#         combined_steering = 0.8 * heading_correction_steering + 0.2 * pure_pursuit_steering * 0.3
        
#         # นับสถิติการแก้ไขทิศทาง
#         if abs(heading_correction_steering) > 0.01:  # มากกว่า ~0.6 องศา
#             self.stats['heading_corrections'] += 1
        
#         return combined_steering
    
#     def calculate_curve_path_control_with_radius(self, curve_direction, params):
#         """คำนวณการควบคุมสำหรับเส้นทางโค้งด้วยรัศมีที่กำหนด"""
#         # ได้รัศมีของทางโค้งจาก parameters
#         curve_radius = params['curve_radii'].get(curve_direction, 15.0)
        
#         # คำนวณมุมเลี้ยวจากรัศมีที่กำหนด
#         steering_angle = self.curvature_controller.calculate_steering_angle_with_radius(
#             curve_radius,
#             self.FIXED_SPEED,
#             params['wheelbase']
#         )
        
#         return steering_angle
    
#     def send_stop_command(self):
#         """ส่งคำสั่งหยุดรถ"""
#         cmd_vel = Twist()
#         cmd_vel.linear.x = 0.0
#         cmd_vel.angular.z = 0.0
#         self.cmd_vel_publisher.publish(cmd_vel)
#         self.get_logger().info('🛑 Stop command sent')
    
#     # เพิ่มฟังก์ชันที่ขาดหายไปจากโค้ดเดิม
#     def init_track_coordinate_system(self):
#         """Initialize coordinate system"""
#         if self.reference_utm is None:
#             self.reference_utm = self.transformer.transform(
#                 self.reference_point[1], self.reference_point[0]
#             )
        
#         self.utm_offset_x = self.TARGET_UTM_X - self.reference_utm[0]
#         self.utm_offset_y = self.TARGET_UTM_Y - self.reference_utm[1]
        
#         self.get_logger().info(f'🗺️ Track coordinate system initialized')
#         self.get_logger().info(f'📍 Reference UTM: {self.reference_utm}')
#         self.get_logger().info(f'🎯 Target UTM: ({self.TARGET_UTM_X}, {self.TARGET_UTM_Y})')
#         self.get_logger().info(f'📐 Offsets: ({self.utm_offset_x:.2f}, {self.utm_offset_y:.2f})')
    
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
        
#         if not self.ser or not self.ser.is_open:
#             self.get_logger().error('❌ No serial connection available')
    
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
#         import base64
#         credentials = f"{self.ntrip_username}:{self.ntrip_password}"
#         return base64.b64encode(credentials.encode()).decode()
    
#     def init_pyubx2(self):
#         """Initialize pyubx2 if available"""
#         if PYUBX2_AVAILABLE:
#             self.get_logger().info('✅ pyubx2 initialized for enhanced UBX parsing')
    
#     def configure_enhanced_zed_f9r(self):
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
            
#             # อัพเดทตำแหน่งปัจจุบัน
#             self.update_current_position()
            
#             # ควบคุมการนำทาง
#             self.navigation_control()
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Main loop error: {e}')
    
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
#         # ประมวลผลข้อมูล NMEA และ UBX
#         # (ใช้โค้ดเดิมที่มีอยู่แล้วในการประมวลผล GNSS data)
#         pass
    
#     def update_current_position(self):
#         """Update current position from GNSS data"""
#         # อัพเดทตำแหน่งปัจจุบันจากข้อมูล GNSS
#         # (ใช้โค้ดเดิมที่มีอยู่แล้วในการอัพเดทตำแหน่ง)
#         pass

# def main(args=None):
#     rclpy.init(args=args)
    
#     try:
#         node = EnhancedZEDf9rGNSSWithSmartNavigation()
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         print('\n🛑 Navigation stopped by user')
#     except Exception as e:
#         print(f'❌ Error: {e}')
#     finally:
#         if 'node' in locals():
#             node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
# --------------------------------------new controller V4-----------------------------------------

# #!/usr/bin/env python3
# import serial
# import pynmea2
# import socket
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
# from std_msgs.msg import Float32, String, Float64MultiArray
# from geometry_msgs.msg import Point, Twist
# from rcl_interfaces.msg import ParameterDescriptor
# import math
# import pyproj
# from collections import deque
# import time
# import glob
# import os
# import struct
# import json
# import numpy as np
# import csv
# import base64

# # เพิ่ม pyubx2 สำหรับ UBX parsing ที่ดีขึ้น
# try:
#     from pyubx2 import UBXReader, UBXMessage, UBX_MSGIDS
#     PYUBX2_AVAILABLE = True
#     print("✅ pyubx2 available - using enhanced UBX parsing")
# except ImportError:
#     PYUBX2_AVAILABLE = False
#     print("⚠️ pyubx2 not available - install with: pip3 install pyubx2")

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

# # Enhanced Waypoint Manager with Smart Starting Point Selection
# class WaypointManager:
#     def __init__(self, csv_file_path, auto_start=True):
#         self.waypoints = []
#         self.current_waypoint_index = 0
#         self.auto_start = auto_start
#         self.navigation_started = False
#         self.load_waypoints_from_csv(csv_file_path)
        
#         # กำหนดช่วง waypoint และประเภทเส้นทาง พร้อมทิศทางที่ต้องรักษา
#         self.path_segments = [
#             {'start': 0, 'end': 240, 'type': 'straight', 'direction': 'S-N', 'target_heading': 0.0},      # waypoint 1-240 (North = 0°)
#             {'start': 241, 'end': 256, 'type': 'curve', 'direction': 'curve1', 'target_heading': None},    # waypoint 245-260
#             {'start': 257, 'end': 315, 'type': 'straight', 'direction': 'E-W', 'target_heading': 90.0},   # waypoint 261-315 (East = 90°)
#             {'start': 316, 'end': 383, 'type': 'curve', 'direction': 'curve2', 'target_heading': None},    # waypoint 316-390
#             {'start': 384, 'end': 496, 'type': 'straight', 'direction': 'N-S', 'target_heading': 180.0},  # waypoint 391-500 (South = 180°)
#             {'start': 497, 'end': 571, 'type': 'curve', 'direction': 'curve3', 'target_heading': None},    # waypoint 501-579
#             {'start': 572, 'end': 622, 'type': 'straight', 'direction': 'W-E', 'target_heading': 270.0},  # waypoint 580-630 (West = 270°)
#             {'start': 623, 'end': 642, 'type': 'curve', 'direction': 'curve4', 'target_heading': None},    # waypoint 631-646
#             {'start': 643, 'end': 654, 'type': 'straight', 'direction': 'S-N', 'target_heading': 0.0}     # waypoint 647-659 (North = 0°)
#         ]
        
#         # กำหนดรัศมีสำหรับแต่ละทางโค้ง
#         self.curve_radii = {
#             'curve1': 15.0,  # รัศมี 15 เมตร สำหรับโค้งที่ 1
#             'curve2': 20.0,  # รัศมี 20 เมตร สำหรับโค้งที่ 2
#             'curve3': 18.0,  # รัศมี 18 เมตร สำหรับโค้งที่ 3
#             'curve4': 12.0   # รัศมี 12 เมตร สำหรับโค้งที่ 4
#         }
        
#         print(f"✅ Loaded {len(self.waypoints)} waypoints from CSV")
#         if auto_start:
#             print("🚀 Auto-start mode enabled - will find nearest forward waypoint")
#         else:
#             print("🎯 Manual start mode - will start from waypoint 0")
    
#     def load_waypoints_from_csv(self, csv_file_path):
#         """โหลด waypoints จากไฟล์ CSV"""
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
#         """หา waypoint ที่ใกล้ที่สุดที่อยู่หน้ารถ"""
#         if not current_position or not self.waypoints:
#             return 0
        
#         current_x, current_y = current_position['x'], current_position['y']
#         candidates = []
        
#         for i, waypoint in enumerate(self.waypoints):
#             # คำนวณระยะทาง
#             dx = waypoint['x'] - current_x
#             dy = waypoint['y'] - current_y
#             distance = math.sqrt(dx**2 + dy**2)
            
#             # ข้ามถ้าไกลเกินไป
#             if distance > search_radius:
#                 continue
            
#             # คำนวณทิศทางไปยัง waypoint (compass bearing)
#             waypoint_bearing = math.degrees(math.atan2(dx, dy))  # atan2(East, North)
#             if waypoint_bearing < 0:
#                 waypoint_bearing += 360
            
#             # คำนวณความแตกต่างของมุม
#             heading_diff = abs(self.normalize_angle_difference(waypoint_bearing - current_heading))
            
#             # ตรวจสอบว่าอยู่หน้ารถหรือไม่ (มุมไม่เกิน ±90°)
#             if heading_diff <= 90.0:
#                 # คำนวณคะแนน (ยิ่งใกล้และอยู่หน้าตรงยิ่งดี)
#                 distance_score = distance
#                 angle_score = heading_diff / 90.0  # normalize เป็น 0-1
#                 combined_score = distance_score + (angle_score * 10.0)  # ให้น้ำหนักกับมุม
                
#                 candidates.append({
#                     'index': i,
#                     'distance': distance,
#                     'heading_diff': heading_diff,
#                     'score': combined_score,
#                     'waypoint': waypoint
#                 })
        
#         if candidates:
#             # เรียงตามคะแนน
#             candidates.sort(key=lambda x: x['score'])
#             best_candidate = candidates[0]
            
#             print(f"🎯 Found {len(candidates)} forward waypoints within {search_radius}m:")
#             for i, candidate in enumerate(candidates[:5]):  # แสดง 5 อันดับแรก
#                 print(f"  #{i+1}: Waypoint {candidate['index']} - "
#                       f"Distance: {candidate['distance']:.2f}m, "
#                       f"Angle: {candidate['heading_diff']:.1f}°, "
#                       f"Score: {candidate['score']:.2f}")
            
#             print(f"✅ Selected waypoint {best_candidate['index']} as starting point")
#             return best_candidate['index']
#         else:
#             # ถ้าไม่เจอ waypoint ที่อยู่หน้ารถ ให้หาที่ใกล้ที่สุด
#             print(f"⚠️ No forward waypoints found within {search_radius}m")
#             return self.find_nearest_waypoint(current_position)
    
#     def find_nearest_waypoint(self, current_position):
#         """หา waypoint ที่ใกล้ที่สุด (ไม่สนใจทิศทาง)"""
#         if not current_position or not self.waypoints:
#             return 0
        
#         current_x, current_y = current_position['x'], current_position['y']
#         min_distance = float('inf')
#         nearest_index = 0
        
#         for i, waypoint in enumerate(self.waypoints):
#             distance = math.sqrt(
#                 (waypoint['x'] - current_x)**2 + 
#                 (waypoint['y'] - current_y)**2
#             )
#             if distance < min_distance:
#                 min_distance = distance
#                 nearest_index = i
        
#         print(f"🔍 Nearest waypoint: {nearest_index} (distance: {min_distance:.2f}m)")
#         return nearest_index
    
#     def initialize_starting_point(self, current_position, current_heading):
#         """เริ่มต้นการนำทางจากจุดที่เหมาะสม"""
#         if self.navigation_started:
#             return False
        
#         if not current_position or current_heading is None:
#             print("⚠️ Waiting for position and heading data...")
#             return False
        
#         if self.auto_start:
#             # หา waypoint ที่ใกล้ที่สุดที่อยู่หน้ารถ
#             self.current_waypoint_index = self.find_nearest_forward_waypoint(
#                 current_position, current_heading
#             )
#         else:
#             # เริ่มจาก waypoint แรก
#             self.current_waypoint_index = 0
        
#         self.navigation_started = True
        
#         print(f"🚀 Navigation started from waypoint {self.current_waypoint_index + 1}")
#         print(f"📍 Target: ({self.waypoints[self.current_waypoint_index]['x']:.2f}, "
#               f"{self.waypoints[self.current_waypoint_index]['y']:.2f})")
        
#         return True
    
#     def normalize_angle_difference(self, angle_diff):
#         """ปรับความแตกต่างของมุมให้อยู่ในช่วง [-180, 180]"""
#         while angle_diff > 180:
#             angle_diff -= 360
#         while angle_diff < -180:
#             angle_diff += 360
#         return angle_diff
    
#     def set_starting_waypoint(self, waypoint_index):
#         """กำหนดจุดเริ่มต้นใหม่"""
#         if 0 <= waypoint_index < len(self.waypoints):
#             self.current_waypoint_index = waypoint_index
#             self.navigation_started = True
#             print(f"✅ Starting waypoint set to {waypoint_index + 1}")
#             return True
#         else:
#             print(f"❌ Invalid waypoint index: {waypoint_index}")
#             return False
    
#     def get_current_segment_info(self):
#         """ได้ข้อมูลของเส้นทางปัจจุบัน"""
#         for segment in self.path_segments:
#             if segment['start'] <= self.current_waypoint_index <= segment['end']:
#                 return segment
#         return {'type': 'straight', 'direction': 'unknown', 'target_heading': 0.0}
    
#     def get_curve_radius(self, curve_direction):
#         """ได้รัศมีของทางโค้ง"""
#         return self.curve_radii.get(curve_direction, 15.0)  # default 15 เมตร
    
#     def get_current_waypoint(self):
#         """ได้ waypoint ปัจจุบัน"""
#         if self.current_waypoint_index < len(self.waypoints):
#             return self.waypoints[self.current_waypoint_index]
#         return None
    
#     def get_next_waypoint(self):
#         """ได้ waypoint ถัดไป"""
#         next_index = self.current_waypoint_index + 1
#         if next_index < len(self.waypoints):
#             return self.waypoints[next_index]
#         return None
    
#     def advance_waypoint(self):
#         """เลื่อนไปยัง waypoint ถัดไป"""
#         if self.current_waypoint_index < len(self.waypoints) - 1:
#             self.current_waypoint_index += 1
#             return True
#         return False
    
#     def get_progress_info(self):
#         """ได้ข้อมูลความคืบหน้า"""
#         return {
#             'current': self.current_waypoint_index + 1,
#             'total': len(self.waypoints),
#             'progress_percent': ((self.current_waypoint_index + 1) / len(self.waypoints)) * 100
#         }

# # Bicycle Kinematic Model for Straight Path Control
# class BicycleKinematicModel:
#     def __init__(self, wheelbase):
#         self.wheelbase = wheelbase
    
#     def calculate_angular_velocity(self, linear_velocity, steering_angle):
#         """คำนวณ angular velocity จาก bicycle model"""
#         if abs(steering_angle) < 1e-6:  # ป้องกันการหารด้วยศูนย์
#             return 0.0
        
#         angular_velocity = (linear_velocity * math.tan(steering_angle)) / self.wheelbase
#         return angular_velocity
    
#     def limit_steering_angle(self, steering_angle, max_steering_angle):
#         """จำกัดมุมเลี้ยวให้อยู่ในขอบเขต"""
#         return max(-max_steering_angle, min(max_steering_angle, steering_angle))
    
#     def calculate_heading_correction_steering(self, current_heading, target_heading, kp=0.02):
#         """คำนวณมุมเลี้ยวเพื่อแก้ไขทิศทางในทางตรง (bicycle model)"""
#         # คำนวณความผิดพลาดของทิศทาง
#         heading_error = self.normalize_angle_difference(target_heading - current_heading)
        
#         # ใช้ proportional control สำหรับการแก้ไขทิศทาง
#         # kp ต่ำ = การแก้ไขนุ่มนวล, kp สูง = การแก้ไขรวดเร็ว
#         steering_angle = kp * math.radians(heading_error)
        
#         return steering_angle
    
#     def normalize_angle_difference(self, angle_diff):
#         """ปรับความแตกต่างของมุมให้อยู่ในช่วง [-180, 180]"""
#         while angle_diff > 180:
#             angle_diff -= 360
#         while angle_diff < -180:
#             angle_diff += 360
#         return angle_diff

# # Pure Pursuit Controller
# class PurePursuitController:
#     def __init__(self, lookahead_distance=2.0):
#         self.lookahead_distance = lookahead_distance
    
#     def calculate_steering_angle(self, current_pos, current_heading, target_pos, wheelbase):
#         """คำนวณมุมเลี้ยวสำหรับ Pure Pursuit"""
#         # คำนวณระยะทางและมุมไปยังเป้าหมาย
#         dx = target_pos['x'] - current_pos['x']
#         dy = target_pos['y'] - current_pos['y']
        
#         target_heading = math.degrees(math.atan2(dx, dy))
#         heading_error = self.normalize_angle(target_heading - current_heading)
        
#         distance = math.sqrt(dx**2 + dy**2)
        
#         # Pure Pursuit formula
#         alpha = math.radians(heading_error)
#         steering_angle = math.atan2(2 * wheelbase * math.sin(alpha), distance)
        
#         return steering_angle
    
#     def normalize_angle(self, angle):
#         """Normalize angle to [-180, 180]"""
#         while angle > 180:
#             angle -= 360
#         while angle < -180:
#             angle += 360
#         return angle

# # Constant Curvature Controller
# class ConstantCurvatureController:
#     def __init__(self):
#         self.curvature_history = deque(maxlen=5)
    
#     def calculate_steering_angle_with_radius(self, curve_radius, linear_velocity, wheelbase):
#         """คำนวณมุมเลี้ยวจากรัศมีที่กำหนด"""
#         # คำนวณ curvature จากรัศมี: κ = 1/R
#         curvature = 1.0 / curve_radius
        
#         # คำนวณมุมเลี้ยวจาก curvature และ bicycle model
#         # δ = atan(L * κ) โดย L = wheelbase, κ = curvature
#         steering_angle = math.atan(wheelbase * curvature)
        
#         return steering_angle
    
#     def calculate_steering_angle(self, waypoints_segment, current_pos, wheelbase):
#         """คำนวณมุมเลี้ยวสำหรับ Constant Curvature (วิธีเดิม)"""
#         if len(waypoints_segment) < 3:
#             return 0.0
        
#         # คำนวณ curvature จาก 3 จุด
#         curvature = self.calculate_curvature(waypoints_segment)
#         self.curvature_history.append(curvature)
        
#         # ใช้ค่าเฉลี่ยของ curvature
#         avg_curvature = np.mean(list(self.curvature_history))
        
#         # คำนวณมุมเลี้ยวจาก curvature
#         steering_angle = math.atan(wheelbase * avg_curvature)
        
#         return steering_angle
    
#     def calculate_curvature(self, points):
#         """คำนวณ curvature จาก 3 จุด"""
#         if len(points) < 3:
#             return 0.0
        
#         p1, p2, p3 = points[0], points[1], points[2]
        
#         # คำนวณ curvature โดยใช้สูตร
#         x1, y1 = p1['x'], p1['y']
#         x2, y2 = p2['x'], p2['y']
#         x3, y3 = p3['x'], p3['y']
        
#         # Area of triangle
#         area = 0.5 * abs((x2-x1)*(y3-y1) - (x3-x1)*(y2-y1))
        
#         # Side lengths
#         a = math.sqrt((x2-x3)**2 + (y2-y3)**2)
#         b = math.sqrt((x1-x3)**2 + (y1-y3)**2)
#         c = math.sqrt((x1-x2)**2 + (y1-y2)**2)
        
#         if a * b * c == 0:
#             return 0.0
        
#         # Curvature = 4 * Area / (a * b * c)
#         curvature = 4 * area / (a * b * c)
        
#         return curvature

# # Main Enhanced GNSS Node with Smart Starting Point and Enhanced Heading
# class EnhancedZEDf9rGNSSWithSmartNavigation(Node):
#     def __init__(self):
#         super().__init__('enhanced_zedf9r_gnss_with_smart_navigation')
        
#         # Vehicle parameters
#         self.declare_vehicle_parameters()
        
#         # ROS2 Publishers
#         self.gnss_publisher = self.create_publisher(NavSatFix, '/navigation/gnss', 10)
#         self.xy_publisher = self.create_publisher(Float64MultiArray, 'Realtime_XY', 10)
#         self.raw_gps_publisher = self.create_publisher(Float64MultiArray, 'Raw_GPS', 10)
#         self.xy_debug_publisher = self.create_publisher(Point, '/navigation/xy_debug', 10)
        
#         # Enhanced Heading Publishers (เหมือนโค้ดที่ 2)
#         self.heading_publisher = self.create_publisher(Float32, '/navigation/heading', 10)
#         self.heading_debug_publisher = self.create_publisher(String, '/navigation/heading_debug', 10)
        
#         self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.navigation_debug_publisher = self.create_publisher(String, '/navigation/debug', 10)
        
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
        
#         # Enhanced Heading management (เหมือนโค้ดที่ 2)
#         self.imu_heading = None
#         self.nav_att_heading = None
#         self.nav_att_yaw = None
#         self.gps_heading = None
#         self.final_heading = None  # compass bearing (degrees)
#         self.heading_source = "NONE"
        
#         self.imu_heading_time = 0
#         self.nav_att_time = 0
#         self.gps_heading_time = 0
#         self.heading_timeout = 8.0
        
#         # ZED-F9R specific data
#         self.esf_status = None
#         self.nav_att = None
#         self.esf_alg = None
#         self.fusion_mode = "NONE"
#         self.imu_calibration_status = "UNKNOWN"
        
#         # UTM coordinates for controller (เหมือนโค้ดที่ 2)
#         self.current_utm_x = None
#         self.current_utm_y = None
        
#         # pyubx2 UBX parsing
#         self.ubx_reader = None
#         self.ubx_buffer = bytearray()
        
#         # Enhanced Heading filtering
#         self.heading_filter = EnhancedIMUHeadingFilter()
        
#         # Navigation Components with Smart Starting Point
#         csv_file_path = '/home/inc/ros2_ws/waypointfile/waypoints3xy.csv'
        
#         self.waypoint_manager = WaypointManager(csv_file_path, auto_start=True)
#         self.pure_pursuit = PurePursuitController(lookahead_distance=3.0)
#         self.curvature_controller = ConstantCurvatureController()
#         self.bicycle_model = BicycleKinematicModel(wheelbase=1.67)
        
#         # Navigation state
#         self.navigation_active = False
#         self.waypoint_reached_threshold = 1.0  # เมตร
        
#         # Fixed Speed Configuration
#         self.FIXED_SPEED = 1.0  # ความเร็วคงที่ 1 m/s
        
#         # NTRIP socket
#         self.ntrip_socket = None
        
#         # Performance stats
#         self.stats = {
#             'imu_count': 0,
#             'nav_att_count': 0,
#             'nav_att_yaw_count': 0,
#             'gps_count': 0,
#             'position_count': 0,
#             'esf_meas_count': 0,
#             'esf_status_count': 0,
#             'pyubx2_success': 0,
#             'pyubx2_errors': 0,
#             'filtered_count': 0,
#             'navigation_commands': 0,
#             'heading_corrections': 0
#         }
        
#         # Initialize coordinate system
#         self.init_track_coordinate_system()
        
#         # Initialize serial connection
#         self.init_serial_connection()
        
#         if self.ser and self.ser.is_open:
#             self.ntrip_socket = self.connect_to_ntrip()
            
#             # Initialize pyubx2 if available
#             if PYUBX2_AVAILABLE:
#                 self.init_pyubx2()
            
#             # Configure ZED-F9R for enhanced IMU
#             self.configure_enhanced_zed_f9r()
            
#             # Main processing timer
#             self.create_timer(0.1, self.main_loop)
            
#             self.get_logger().info('✅ Enhanced ZED-F9R GNSS with Smart Navigation initialized')
#             self.get_logger().info(f'🔧 pyubx2: {"Available" if PYUBX2_AVAILABLE else "Not available"}')
#             self.get_logger().info(f'🗺️ Loaded {len(self.waypoint_manager.waypoints)} waypoints')
#             self.get_logger().info(f'🚀 Fixed Speed: {self.FIXED_SPEED} m/s')
#             self.get_logger().info('🎯 Smart starting point: Will find nearest forward waypoint')
#             self.get_logger().info('🧭 Enhanced heading calculation enabled')
            
#             # Log vehicle parameters
#             self.log_vehicle_parameters()
            
#             # Log curve radii
#             self.log_curve_radii()
#         else:
#             self.get_logger().error('❌ Failed to initialize')
    
#     def declare_vehicle_parameters(self):
#         """ประกาศ Vehicle Parameters"""
        
#         # Wheelbase parameter
#         wheelbase_descriptor = ParameterDescriptor(
#             description='ฐานล้อของรถ (ระยะห่างระหว่างเพลาหน้าและหลัง) ในหน่วยเมตร'
#         )
#         self.declare_parameter('wheelbase', 1.67, wheelbase_descriptor)
        
#         # Max steering angle parameter
#         steering_descriptor = ParameterDescriptor(
#             description='มุมเลี้ยวสูงสุดของรถในหน่วย radians (50 degrees = 0.873 rad)'
#         )
#         self.declare_parameter('max_steering_angle', 0.873, steering_descriptor)
        
#         # Fixed speed parameter
#         speed_descriptor = ParameterDescriptor(
#             description='ความเร็วคงที่ของรถในหน่วย m/s (ตั้งค่าเป็น 1.0 m/s)'
#         )
#         self.declare_parameter('fixed_speed', 1.0, speed_descriptor)
        
#         # Smart starting parameters
#         auto_start_descriptor = ParameterDescriptor(
#             description='เปิดใช้การหาจุดเริ่มต้นอัตโนมัติ (true = หาจุดที่ใกล้ที่สุดที่อยู่หน้ารถ)'
#         )
#         self.declare_parameter('auto_start', True, auto_start_descriptor)
        
#         search_radius_descriptor = ParameterDescriptor(
#             description='รัศมีการค้นหา waypoint ในหน่วยเมตร'
#         )
#         self.declare_parameter('search_radius', 50.0, search_radius_descriptor)
        
#         # Bicycle model parameters สำหรับการควบคุมทิศทางในทางตรง
#         heading_kp_descriptor = ParameterDescriptor(
#             description='Proportional gain สำหรับการแก้ไขทิศทางในทางตรง (ค่าต่ำ = นุ่มนวล)'
#         )
#         self.declare_parameter('heading_correction_kp', 0.02, heading_kp_descriptor)
        
#         # Curve radius parameters
#         curve1_radius_descriptor = ParameterDescriptor(
#             description='รัศมีของทางโค้งที่ 1 ในหน่วยเมตร'
#         )
#         self.declare_parameter('curve1_radius', 15.0, curve1_radius_descriptor)
        
#         curve2_radius_descriptor = ParameterDescriptor(
#             description='รัศมีของทางโค้งที่ 2 ในหน่วยเมตร'
#         )
#         self.declare_parameter('curve2_radius', 20.0, curve2_radius_descriptor)
        
#         curve3_radius_descriptor = ParameterDescriptor(
#             description='รัศมีของทางโค้งที่ 3 ในหน่วยเมตร'
#         )
#         self.declare_parameter('curve3_radius', 18.0, curve3_radius_descriptor)
        
#         curve4_radius_descriptor = ParameterDescriptor(
#             description='รัศมีของทางโค้งที่ 4 ในหน่วยเมตร'
#         )
#         self.declare_parameter('curve4_radius', 12.0, curve4_radius_descriptor)
        
#         # Angular velocity limits
#         max_angular_vel_descriptor = ParameterDescriptor(
#             description='ความเร็วเชิงมุมสูงสุดในหน่วย rad/s'
#         )
#         self.declare_parameter('max_angular_velocity', 1.0, max_angular_vel_descriptor)
        
#         self.get_logger().info('🌟 Vehicle parameters declared successfully!')
    
#     def get_vehicle_parameters(self):
#         """ดึงค่า Vehicle Parameters ปัจจุบัน"""
#         try:
#             wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
#             max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value
#             fixed_speed = self.get_parameter('fixed_speed').get_parameter_value().double_value
#             auto_start = self.get_parameter('auto_start').get_parameter_value().bool_value
#             search_radius = self.get_parameter('search_radius').get_parameter_value().double_value
#             heading_correction_kp = self.get_parameter('heading_correction_kp').get_parameter_value().double_value
#             max_angular_velocity = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
            
#             # ดึงค่ารัศมีของทางโค้ง
#             curve_radii = {
#                 'curve1': self.get_parameter('curve1_radius').get_parameter_value().double_value,
#                 'curve2': self.get_parameter('curve2_radius').get_parameter_value().double_value,
#                 'curve3': self.get_parameter('curve3_radius').get_parameter_value().double_value,
#                 'curve4': self.get_parameter('curve4_radius').get_parameter_value().double_value
#             }
            
#             return {
#                 'wheelbase': wheelbase,
#                 'max_steering_angle': max_steering_angle,
#                 'fixed_speed': fixed_speed,
#                 'auto_start': auto_start,
#                 'search_radius': search_radius,
#                 'heading_correction_kp': heading_correction_kp,
#                 'max_angular_velocity': max_angular_velocity,
#                 'curve_radii': curve_radii
#             }
#         except Exception as e:
#             self.get_logger().error(f'❌ Error getting vehicle parameters: {e}')
#             return {
#                 'wheelbase': 1.67,
#                 'max_steering_angle': 0.873,
#                 'fixed_speed': 1.0,
#                 'auto_start': True,
#                 'search_radius': 50.0,
#                 'heading_correction_kp': 0.02,
#                 'max_angular_velocity': 1.0,
#                 'curve_radii': {
#                     'curve1': 15.0,
#                     'curve2': 20.0,
#                     'curve3': 18.0,
#                     'curve4': 12.0
#                 }
#             }
    
#     def log_vehicle_parameters(self):
#         """แสดง Vehicle Parameters ปัจจุบัน"""
#         params = self.get_vehicle_parameters()
#         self.get_logger().info(
#             f'🚗 Vehicle Parameters:\n'
#             f'  🔧 Wheelbase: {params["wheelbase"]:.2f} m\n'
#             f'  🔧 Max Steering Angle: {params["max_steering_angle"]:.3f} rad ({math.degrees(params["max_steering_angle"]):.1f}°)\n'
#             f'  🔧 Fixed Speed: {params["fixed_speed"]:.2f} m/s\n'
#             f'  🔧 Auto Start: {params["auto_start"]}\n'
#             f'  🔧 Search Radius: {params["search_radius"]:.1f} m\n'
#             f'  🔧 Heading Correction Kp: {params["heading_correction_kp"]:.3f}\n'
#             f'  🔧 Max Angular Velocity: {params["max_angular_velocity"]:.2f} rad/s'
#         )
    
#     def log_curve_radii(self):
#         """แสดงรัศมีของทางโค้ง"""
#         params = self.get_vehicle_parameters()
#         curve_radii = params['curve_radii']
        
#         self.get_logger().info(
#             f'🌀 Curve Radii Configuration:\n'
#             f'  🔄 Curve 1 (Waypoint 245-260): {curve_radii["curve1"]:.1f} m\n'
#             f'  🔄 Curve 2 (Waypoint 316-390): {curve_radii["curve2"]:.1f} m\n'
#             f'  🔄 Curve 3 (Waypoint 501-579): {curve_radii["curve3"]:.1f} m\n'
#             f'  🔄 Curve 4 (Waypoint 631-646): {curve_radii["curve4"]:.1f} m'
#         )
    
#     def navigation_control(self):
#         """ระบบควบคุมการนำทางด้วย Smart Starting Point"""
        
#         # ตรวจสอบการเริ่มต้นการนำทาง
#         if not self.waypoint_manager.navigation_started:
#             if self.final_heading is not None:
#                 # พยายามเริ่มต้นการนำทาง
#                 if self.waypoint_manager.initialize_starting_point(self.current_position, self.final_heading):
#                     self.navigation_active = True
#                     self.get_logger().info('🚀 Smart navigation started!')
#             return
        
#         if not self.navigation_active or self.final_heading is None:
#             return
        
#         # ได้ตำแหน่งปัจจุบัน
#         current_waypoint = self.waypoint_manager.get_current_waypoint()
#         if current_waypoint is None:
#             self.get_logger().info('🏁 Navigation completed - reached final waypoint')
#             self.navigation_active = False
#             self.send_stop_command()
#             return
        
#         # ตรวจสอบว่าถึง waypoint แล้วหรือยัง
#         distance_to_waypoint = math.sqrt(
#             (self.current_position['x'] - current_waypoint['x'])**2 + 
#             (self.current_position['y'] - current_waypoint['y'])**2
#         )
        
#         if distance_to_waypoint < self.waypoint_reached_threshold:
#             if self.waypoint_manager.advance_waypoint():
#                 progress = self.waypoint_manager.get_progress_info()
#                 self.get_logger().info(
#                     f'✅ Reached waypoint {progress["current"]-1} '
#                     f'({progress["progress_percent"]:.1f}% complete)'
#                 )
#             else:
#                 self.get_logger().info('🏁 Navigation completed')
#                 self.navigation_active = False
#                 self.send_stop_command()
#                 return
        
#         # ได้ข้อมูลเส้นทางปัจจุบัน
#         segment_info = self.waypoint_manager.get_current_segment_info()
#         segment_type = segment_info['type']
#         direction = segment_info['direction']
#         target_heading = segment_info.get('target_heading')
        
#         # ได้ parameters
#         params = self.get_vehicle_parameters()
        
#         # ใช้ความเร็วคงที่ 1 m/s
#         linear_velocity = self.FIXED_SPEED
        
#         # คำนวณคำสั่งควบคุมตามประเภทเส้นทาง
#         if segment_type == 'straight':
#             steering_angle = self.calculate_straight_path_control_with_bicycle_model(
#                 current_waypoint, target_heading, params
#             )
#         else:  # curve
#             steering_angle = self.calculate_curve_path_control_with_radius(direction, params)
        
#         # จำกัดมุมเลี้ยว
#         steering_angle = self.bicycle_model.limit_steering_angle(
#             steering_angle, params['max_steering_angle']
#         )
        
#         # คำนวณ angular velocity ด้วย bicycle model
#         angular_velocity = self.bicycle_model.calculate_angular_velocity(
#             linear_velocity, steering_angle
#         )
        
#         # จำกัด angular velocity
#         angular_velocity = max(-params['max_angular_velocity'], 
#                              min(params['max_angular_velocity'], angular_velocity))
        
#         # สร้างและส่งคำสั่ง cmd_vel
#         cmd_vel = Twist()
#         cmd_vel.linear.x = linear_velocity
#         cmd_vel.angular.z = angular_velocity
        
#         self.cmd_vel_publisher.publish(cmd_vel)
#         self.stats['navigation_commands'] += 1
        
#         # Debug information - แก้ไข None formatting
#         debug_msg = String()
#         progress = self.waypoint_manager.get_progress_info()
        
#         if segment_type == 'curve':
#             curve_radius = params['curve_radii'].get(direction, 15.0)
#             debug_msg.data = (
#                 f"Segment: {segment_type} ({direction}, R={curve_radius:.1f}m) | "
#                 f"Waypoint: {progress['current']}/{progress['total']} ({progress['progress_percent']:.1f}%) | "
#                 f"Distance: {distance_to_waypoint:.2f}m | "
#                 f"Steering: {math.degrees(steering_angle):.1f}° | "
#                 f"Speed: {linear_velocity:.2f}m/s | "
#                 f"Angular: {angular_velocity:.2f}rad/s"
#             )
#         else:
#             # แก้ไข None handling สำหรับ target_heading และ final_heading
#             target_heading_str = f"{target_heading:.0f}" if target_heading is not None else "None"
#             final_heading_str = f"{self.final_heading:.1f}" if self.final_heading is not None else "None"
            
#             if target_heading is not None and self.final_heading is not None:
#                 heading_error = self.bicycle_model.normalize_angle_difference(target_heading - self.final_heading)
#             else:
#                 heading_error = 0.0
                
#             debug_msg.data = (
#                 f"Segment: {segment_type} ({direction}, Target={target_heading_str}°) | "
#                 f"Waypoint: {progress['current']}/{progress['total']} ({progress['progress_percent']:.1f}%) | "
#                 f"Distance: {distance_to_waypoint:.2f}m | "
#                 f"Heading: {final_heading_str}° (Error: {heading_error:.1f}°) | "
#                 f"Steering: {math.degrees(steering_angle):.1f}° | "
#                 f"Speed: {linear_velocity:.2f}m/s | "
#                 f"Angular: {angular_velocity:.2f}rad/s"
#             )
        
#         self.navigation_debug_publisher.publish(debug_msg)
    
#     def calculate_straight_path_control_with_bicycle_model(self, target_waypoint, target_heading, params):
#         """คำนวณการควบคุมสำหรับเส้นทางตรงด้วย Bicycle Model + Heading Control"""
        
#         # ถ้าไม่มี target_heading ให้ใช้ Pure Pursuit
#         if target_heading is None:
#             steering_angle = self.pure_pursuit.calculate_steering_angle(
#                 self.current_position,
#                 self.final_heading,
#                 target_waypoint,
#                 params['wheelbase']
#             )
#             return steering_angle * 0.5  # ลดความแรงของการเลี้ยว
        
#         # ใช้ Bicycle Model เพื่อรักษาทิศทางคงที่
#         heading_correction_steering = self.bicycle_model.calculate_heading_correction_steering(
#             self.final_heading,
#             target_heading,
#             params['heading_correction_kp']
#         )
        
#         # เพิ่ม Pure Pursuit เล็กน้อยเพื่อนำทางไปยัง waypoint
#         pure_pursuit_steering = self.pure_pursuit.calculate_steering_angle(
#             self.current_position,
#             self.final_heading,
#             target_waypoint,
#             params['wheelbase']
#         )
        
#         combined_steering = 0.8 * heading_correction_steering + 0.2 * pure_pursuit_steering * 0.3
        
#         # นับสถิติการแก้ไขทิศทาง
#         if abs(heading_correction_steering) > 0.01:  # มากกว่า ~0.6 องศา
#             self.stats['heading_corrections'] += 1
        
#         return combined_steering
    
#     def calculate_curve_path_control_with_radius(self, curve_direction, params):
#         """คำนวณการควบคุมสำหรับเส้นทางโค้งด้วยรัศมีที่กำหนด"""
#         # ได้รัศมีของทางโค้งจาก parameters
#         curve_radius = params['curve_radii'].get(curve_direction, 15.0)
        
#         # คำนวณมุมเลี้ยวจากรัศมีที่กำหนด
#         steering_angle = self.curvature_controller.calculate_steering_angle_with_radius(
#             curve_radius,
#             self.FIXED_SPEED,
#             params['wheelbase']
#         )
        
#         return steering_angle
    
#     def send_stop_command(self):
#         """ส่งคำสั่งหยุดรถ"""
#         cmd_vel = Twist()
#         cmd_vel.linear.x = 0.0
#         cmd_vel.angular.z = 0.0
#         self.cmd_vel_publisher.publish(cmd_vel)
#         self.get_logger().info('🛑 Stop command sent')
    
#     # Enhanced Heading Calculation Functions (from code 2)
#     def update_gps_heading_improved(self):
#         """Enhanced GPS heading calculation using UTM coordinates (compass bearing)"""
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
#         """Enhanced final heading selection with NAV-ATT priority (compass bearing)"""
#         current_time = time.time()
        
#         # Check data validity
#         nav_att_valid = (self.nav_att_heading is not None and 
#                         (current_time - self.nav_att_time) < self.heading_timeout)
        
#         gps_valid = (self.gps_heading is not None and 
#                     (current_time - self.gps_heading_time) < self.heading_timeout)
        
#         # Enhanced priority: NAV-ATT > GPS > Hold
#         if nav_att_valid and self.imu_calibration_status in ["FULLY_CALIBRATED", "CALIBRATED"]:
#             self.final_heading = self.nav_att_heading  # compass bearing (degrees)
#             self.heading_source = f"NAV_ATT_{self.imu_calibration_status}"
            
#         elif nav_att_valid and self.imu_calibration_status == "CALIBRATING":
#             if gps_valid:
#                 self.final_heading = self.blend_headings(self.nav_att_heading, self.gps_heading, 0.8)
#                 self.heading_source = "NAV_ATT_GPS_BLEND"
#             else:
#                 self.final_heading = self.nav_att_heading
#                 self.heading_source = "NAV_ATT_CALIBRATING"
                
#         elif gps_valid:
#             self.final_heading = self.gps_heading  # compass bearing (degrees)
#             self.heading_source = "GPS"
            
#         else:
#             # ถ้าไม่มีข้อมูลใหม่ ให้เก็บค่าเดิมไว้แทนที่จะเป็น None
#             if self.final_heading is not None:
#                 self.heading_source = "HOLD_LAST"
#             else:
#                 self.final_heading = None
#                 self.heading_source = "NONE"
        
#         # Publish enhanced heading - แก้ไข None handling
#         if self.final_heading is not None:
#             heading_msg = Float32()
#             heading_msg.data = float(self.final_heading)
#             self.heading_publisher.publish(heading_msg)
#             self.stats['filtered_count'] += 1
            
#             # Publish heading debug info - แก้ไข None formatting
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
#         """Blend two headings considering circular nature (compass bearing)"""
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
    
#     def init_track_coordinate_system(self):
#         """Initialize coordinate system"""
#         if self.reference_utm is None:
#             self.reference_utm = self.transformer.transform(
#                 self.reference_point[1], self.reference_point[0]
#             )
        
#         self.utm_offset_x = self.TARGET_UTM_X - self.reference_utm[0]
#         self.utm_offset_y = self.TARGET_UTM_Y - self.reference_utm[1]
        
#         self.get_logger().info(f'🗺️ Track coordinate system initialized')
#         self.get_logger().info(f'📍 Reference UTM: {self.reference_utm}')
#         self.get_logger().info(f'🎯 Target UTM: ({self.TARGET_UTM_X}, {self.TARGET_UTM_Y})')
#         self.get_logger().info(f'📐 Offsets: ({self.utm_offset_x:.2f}, {self.utm_offset_y:.2f})')
    
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
        
#         if not self.ser or not self.ser.is_open:
#             self.get_logger().error('❌ No serial connection available')
    
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
    
#     def init_pyubx2(self):
#         """Initialize pyubx2 if available"""
#         if PYUBX2_AVAILABLE:
#             self.get_logger().info('✅ pyubx2 initialized for enhanced UBX parsing')
    
#     def configure_enhanced_zed_f9r(self):
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
            
#             # อัพเดทตำแหน่งปัจจุบัน
#             self.update_current_position()
            
#             # อัพเดท heading
#             self.update_enhanced_final_heading()
            
#             # ควบคุมการนำทาง
#             self.navigation_control()
            
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
#         """Process incoming GNSS data - Enhanced version"""
#         try:
#             # แยกข้อมูล NMEA และ UBX
#             data_str = data.decode('utf-8', errors='ignore')
#             lines = data_str.split('\n')
            
#             for line in lines:
#                 line = line.strip()
#                 if line.startswith('$'):
#                     # ประมวลผล NMEA
#                     self.process_nmea_sentence(line)
#                 elif line.startswith('\xb5\x62') or b'\xb5\x62' in data:
#                     # ประมวลผล UBX
#                     if PYUBX2_AVAILABLE:
#                         self.process_ubx_data_enhanced(data)
#                     else:
#                         self.process_ubx_data_manual(data)
                        
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
                    
#                     # เก็บ UTM coordinates สำหรับ controller
#                     self.current_utm_x = utm_x
#                     self.current_utm_y = utm_y
                    
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
    
#     def process_ubx_data_enhanced(self, data):
#         """Enhanced UBX processing with pyubx2"""
#         try:
#             if PYUBX2_AVAILABLE:
#                 self.ubx_buffer.extend(data)
                
#                 while len(self.ubx_buffer) > 8:
#                     try:
#                         # หา UBX header
#                         sync_chars = b'\xb5\x62'
#                         if sync_chars not in self.ubx_buffer:
#                             self.ubx_buffer.clear()
#                             break
                        
#                         # ตัด buffer ให้เริ่มต้นด้วย sync chars
#                         start_idx = self.ubx_buffer.find(sync_chars)
#                         if start_idx > 0:
#                             self.ubx_buffer = self.ubx_buffer[start_idx:]
                        
#                         if len(self.ubx_buffer) < 8:
#                             break
                        
#                         # อ่าน length
#                         msg_len = struct.unpack('<H', self.ubx_buffer[4:6])[0]
#                         total_len = msg_len + 8  # header(2) + class(1) + id(1) + length(2) + payload + checksum(2)
                        
#                         if len(self.ubx_buffer) < total_len:
#                             break
                        
#                         # Extract complete message
#                         ubx_msg = bytes(self.ubx_buffer[:total_len])
#                         self.ubx_buffer = self.ubx_buffer[total_len:]
                        
#                         # Parse with pyubx2
#                         try:
#                             parsed_msg = UBXMessage.parse(ubx_msg)
#                             self.process_parsed_ubx_message(parsed_msg)
#                             self.stats['pyubx2_success'] += 1
#                         except Exception as parse_error:
#                             self.stats['pyubx2_errors'] += 1
#                             # Fallback to manual parsing
#                             self.process_ubx_data_manual(ubx_msg)
                        
#                     except Exception as e:
#                         self.ubx_buffer = self.ubx_buffer[1:]  # Remove first byte and try again
                        
#         except Exception as e:
#             self.get_logger().debug(f'UBX enhanced processing error: {e}')
    
#     def process_parsed_ubx_message(self, parsed_msg):
#         """Process parsed UBX message from pyubx2"""
#         try:
#             msg_class = parsed_msg.msg_cls
#             msg_id = parsed_msg.msg_id
            
#             if msg_class == 0x01 and msg_id == 0x05:  # NAV-ATT
#                 self.process_nav_att_pyubx2(parsed_msg)
#             elif msg_class == 0x10 and msg_id == 0x10:  # ESF-STATUS
#                 self.process_esf_status_pyubx2(parsed_msg)
#             elif msg_class == 0x10 and msg_id == 0x14:  # ESF-ALG
#                 self.process_esf_alg_pyubx2(parsed_msg)
                
#         except Exception as e:
#             self.get_logger().debug(f'Parsed UBX message processing error: {e}')
    
#     def process_nav_att_pyubx2(self, parsed_msg):
#         """Process NAV-ATT message from pyubx2"""
#         try:
#             if hasattr(parsed_msg, 'heading'):
#                 heading_deg = parsed_msg.heading * 1e-5  # Convert from 1e-5 degrees
                
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
#             self.get_logger().debug(f'NAV-ATT pyubx2 processing error: {e}')
    
#     def process_esf_status_pyubx2(self, parsed_msg):
#         """Process ESF-STATUS message from pyubx2"""
#         try:
#             if hasattr(parsed_msg, 'fusionMode'):
#                 fusion_mode = parsed_msg.fusionMode
#                 if fusion_mode == 0:
#                     self.fusion_mode = "INIT"
#                 elif fusion_mode == 1:
#                     self.fusion_mode = "FUSION"
#                 elif fusion_mode == 2:
#                     self.fusion_mode = "SUSPENDED"
#                 else:
#                     self.fusion_mode = "DISABLED"
                
#                 self.stats['esf_status_count'] += 1
                
#         except Exception as e:
#             self.get_logger().debug(f'ESF-STATUS pyubx2 processing error: {e}')
    
#     def process_esf_alg_pyubx2(self, parsed_msg):
#         """Process ESF-ALG message from pyubx2"""
#         try:
#             # Update IMU calibration status based on ESF-ALG
#             self.imu_calibration_status = "CALIBRATING"
            
#         except Exception as e:
#             self.get_logger().debug(f'ESF-ALG pyubx2 processing error: {e}')
    
#     def process_ubx_data_manual(self, data):
#         """Fallback manual UBX processing for NAV-ATT"""
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
    
#     def update_current_position(self):
#         """Update current position from GNSS data"""
#         # ฟังก์ชันนี้จะถูกเรียกจาก process_nmea_sentence
#         # เมื่อได้รับข้อมูลตำแหน่งใหม่
#         pass
    
#     def destroy_node(self):
#         """Cleanup resources"""
#         self.get_logger().info("🔄 Shutting down Enhanced ZED-F9R with Smart Navigation...")
        
#         # Stop navigation
#         if hasattr(self, 'navigation_active') and self.navigation_active:
#             self.send_stop_command()
        
#         # Close serial connection
#         if hasattr(self, 'ser') and self.ser and self.ser.is_open:
#             try:
#                 self.ser.close()
#                 self.get_logger().info("✅ Serial connection closed")
#             except Exception as e:
#                 self.get_logger().error(f"❌ Error closing serial: {e}")
        
#         # Close NTRIP connection
#         if hasattr(self, 'ntrip_socket') and self.ntrip_socket:
#             try:
#                 self.ntrip_socket.close()
#                 self.get_logger().info("✅ NTRIP connection closed")
#             except Exception as e:
#                 self.get_logger().error(f"❌ Error closing NTRIP: {e}")
        
#         # Print final statistics
#         self.print_final_statistics()
        
#         super().destroy_node()
    
#     def print_final_statistics(self):
#         """Print final performance statistics"""
#         try:
#             self.get_logger().info("📊 Final Performance Statistics:")
#             self.get_logger().info(f"  📍 Position updates: {self.stats['position_count']}")
#             self.get_logger().info(f"  🧭 GPS heading updates: {self.stats['gps_count']}")
#             self.get_logger().info(f"  🎯 NAV-ATT updates: {self.stats['nav_att_count']}")
#             self.get_logger().info(f"  🚗 Navigation commands: {self.stats['navigation_commands']}")
#             self.get_logger().info(f"  🔧 Heading corrections: {self.stats['heading_corrections']}")
            
#             if PYUBX2_AVAILABLE:
#                 self.get_logger().info(f"  ✅ pyubx2 successes: {self.stats['pyubx2_success']}")
#                 self.get_logger().info(f"  ❌ pyubx2 errors: {self.stats['pyubx2_errors']}")
            
#             # Waypoint progress
#             if hasattr(self, 'waypoint_manager') and self.waypoint_manager:
#                 progress = self.waypoint_manager.get_progress_info()
#                 self.get_logger().info(f"  🎯 Waypoint progress: {progress['current']}/{progress['total']} ({progress['progress_percent']:.1f}%)")
            
#             # Heading filter statistics
#             if hasattr(self, 'heading_filter'):
#                 filter_stats = self.heading_filter.get_stats()
#                 self.get_logger().info(f"  🔍 Filter statistics: {filter_stats}")
                
#         except Exception as e:
#             self.get_logger().error(f"❌ Error printing statistics: {e}")

# def main(args=None):
#     rclpy.init(args=args)
    
#     try:
#         node = EnhancedZEDf9rGNSSWithSmartNavigation()
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         print('\n🛑 Navigation stopped by user')
#     except Exception as e:
#         print(f'❌ Error: {e}')
#     finally:
#         if 'node' in locals():
#             node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# # ------------------------------------------เพียวPP--------------------------------------------------------------

# #!/usr/bin/env python3
# import serial
# import pynmea2
# import socket
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
# from std_msgs.msg import Float32, String, Float64MultiArray
# from geometry_msgs.msg import Point, Twist
# from rcl_interfaces.msg import ParameterDescriptor
# import math
# import pyproj
# from collections import deque
# import time
# import glob
# import os
# import struct
# import json
# import numpy as np
# import csv

# # เพิ่ม pyubx2 สำหรับ UBX parsing ที่ดีขึ้น
# try:
#     from pyubx2 import UBXReader, UBXMessage, UBX_MSGIDS
#     PYUBX2_AVAILABLE = True
#     print("✅ pyubx2 available - using enhanced UBX parsing")
# except ImportError:
#     PYUBX2_AVAILABLE = False
#     print("⚠️ pyubx2 not available - install with: pip3 install pyubx2")

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

# # Enhanced Waypoint Manager with Pure Pursuit Support
# class WaypointManager:
#     def __init__(self, csv_file_path, start_waypoint_index=0):
#         self.waypoints = []
#         self.current_waypoint_index = start_waypoint_index
#         self.target_waypoint_index = start_waypoint_index
#         self.load_waypoints_from_csv(csv_file_path)
        
#         # ตรวจสอบว่าจุดเริ่มต้นอยู่ในช่วงที่ถูกต้อง
#         if self.current_waypoint_index >= len(self.waypoints):
#             self.current_waypoint_index = 0
#             print(f"⚠️ Starting waypoint index out of range, reset to 0")
        
#         print(f"✅ Starting navigation from waypoint {self.current_waypoint_index + 1}")
#         print(f"📊 Total waypoints loaded: {len(self.waypoints)}")
    
#     def load_waypoints_from_csv(self, csv_file_path):
#         """โหลด waypoints จากไฟล์ CSV"""
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
    
#     def get_current_waypoint(self):
#         """ได้ waypoint ปัจจุบัน"""
#         if self.current_waypoint_index < len(self.waypoints):
#             return self.waypoints[self.current_waypoint_index]
#         return None
    
#     def get_target_waypoint(self):
#         """ได้ target waypoint สำหรับ Pure Pursuit"""
#         if self.target_waypoint_index < len(self.waypoints):
#             return self.waypoints[self.target_waypoint_index]
#         return None
    
#     def get_waypoints_ahead(self, count=10):
#         """ได้ waypoints ข้างหน้าสำหรับการหา lookahead point"""
#         start_idx = self.current_waypoint_index
#         end_idx = min(start_idx + count, len(self.waypoints))
#         return self.waypoints[start_idx:end_idx]
    
#     def update_current_waypoint(self, current_pos, threshold=1.0):
#         """อัพเดท current waypoint ตามตำแหน่งปัจจุบัน"""
#         current_waypoint = self.get_current_waypoint()
#         if current_waypoint is None:
#             return False
            
#         # ตรวจสอบว่าถึง waypoint แล้วหรือยัง
#         distance = math.sqrt(
#             (current_pos['x'] - current_waypoint['x'])**2 + 
#             (current_pos['y'] - current_waypoint['y'])**2
#         )
        
#         if distance < threshold:
#             if self.current_waypoint_index < len(self.waypoints) - 1:
#                 self.current_waypoint_index += 1
#                 # อัพเดท target waypoint ด้วย
#                 self.target_waypoint_index = max(
#                     self.target_waypoint_index, 
#                     self.current_waypoint_index
#                 )
#                 return True
        
#         return False
    
#     def set_target_waypoint_index(self, index):
#         """กำหนด target waypoint index"""
#         if 0 <= index < len(self.waypoints):
#             self.target_waypoint_index = index
    
#     def get_progress_info(self):
#         """ได้ข้อมูลความคืบหน้า"""
#         return {
#             'current': self.current_waypoint_index + 1,
#             'target': self.target_waypoint_index + 1,
#             'total': len(self.waypoints),
#             'progress_percent': ((self.current_waypoint_index + 1) / len(self.waypoints)) * 100
#         }

# # Pure Pursuit Controller
# class PurePursuitController:
#     def __init__(self, lookahead_distance=3.0, lookahead_gain=1.0, min_lookahead=1.0):
#         self.lookahead_distance = lookahead_distance
#         self.lookahead_gain = lookahead_gain
#         self.min_lookahead = min_lookahead
#         self.current_speed = 0.0
        
#     def update_lookahead_distance(self, current_speed):
#         """อัพเดทระยะ lookahead ตามความเร็วปัจจุบัน"""
#         self.current_speed = current_speed
#         # ระยะ lookahead = gain * speed + min_lookahead
#         self.lookahead_distance = max(
#             self.lookahead_gain * current_speed + self.min_lookahead,
#             self.min_lookahead
#         )
        
#     def find_lookahead_point(self, current_pos, waypoints, current_waypoint_index):
#         """หาจุด lookahead point จาก waypoints"""
#         if not waypoints or current_waypoint_index >= len(waypoints):
#             return None, current_waypoint_index
            
#         # เริ่มหาจากจุดปัจจุบัน
#         for i in range(current_waypoint_index, len(waypoints)):
#             waypoint = waypoints[i]
#             distance = math.sqrt(
#                 (waypoint['x'] - current_pos['x'])**2 + 
#                 (waypoint['y'] - current_pos['y'])**2
#             )
            
#             # ถ้าระยะทางมากกว่า lookahead distance ให้ใช้จุดนี้
#             if distance >= self.lookahead_distance:
#                 return waypoint, i
                
#         # ถ้าไม่เจอ ให้ใช้จุดสุดท้าย
#         if waypoints:
#             return waypoints[-1], len(waypoints) - 1
        
#         return None, current_waypoint_index
    
#     def calculate_steering_angle(self, current_pos, current_heading, target_pos, wheelbase):
#         """คำนวณมุมเลี้ยวด้วย Pure Pursuit Algorithm"""
#         if target_pos is None:
#             return 0.0
            
#         # คำนวณระยะทางและมุมไปยังเป้าหมาย
#         dx = target_pos['x'] - current_pos['x']
#         dy = target_pos['y'] - current_pos['y']
        
#         # ระยะทางไปยังเป้าหมาย
#         distance = math.sqrt(dx**2 + dy**2)
        
#         if distance < 0.1:  # ถ้าใกล้มาก
#             return 0.0
            
#         # คำนวณมุมไปยังเป้าหมาย
#         target_heading = math.degrees(math.atan2(dy, dx))
        
#         # คำนวณ heading error
#         heading_error = self.normalize_angle(target_heading - current_heading)
        
#         # Pure Pursuit Formula
#         # δ = atan(2 * L * sin(α) / ld)
#         # โดย α = heading_error, L = wheelbase, ld = lookahead distance
#         alpha = math.radians(heading_error)
        
#         # ใช้ระยะทางจริงแทน lookahead distance ถ้าใกล้กว่า
#         effective_distance = min(distance, self.lookahead_distance)
        
#         if effective_distance < 0.1:
#             return 0.0
            
#         steering_angle = math.atan(2 * wheelbase * math.sin(alpha) / effective_distance)
        
#         return steering_angle
    
#     def normalize_angle(self, angle):
#         """Normalize angle to [-180, 180]"""
#         while angle > 180:
#             angle -= 360
#         while angle < -180:
#             angle += 360
#         return angle

# # Bicycle Kinematic Model
# class BicycleKinematicModel:
#     def __init__(self, wheelbase):
#         self.wheelbase = wheelbase
    
#     def calculate_angular_velocity(self, linear_velocity, steering_angle):
#         """คำนวณ angular velocity จาก bicycle model"""
#         if abs(steering_angle) < 1e-6:  # ป้องกันการหารด้วยศูนย์
#             return 0.0
        
#         angular_velocity = (linear_velocity * math.tan(steering_angle)) / self.wheelbase
#         return angular_velocity
    
#     def limit_steering_angle(self, steering_angle, max_steering_angle):
#         """จำกัดมุมเลี้ยวให้อยู่ในขอบเขต"""
#         return max(-max_steering_angle, min(max_steering_angle, steering_angle))

# # Main Enhanced GNSS Node with Pure Pursuit Navigation
# class EnhancedZEDf9rGNSSWithPurePursuit(Node):
#     def __init__(self):
#         super().__init__('enhanced_zedf9r_gnss_with_pure_pursuit')
        
#         # ประกาศ parameters
#         self.declare_vehicle_parameters()
        
#         # ROS2 Publishers
#         self.gnss_publisher = self.create_publisher(NavSatFix, '/navigation/gnss', 10)
#         self.xy_publisher = self.create_publisher(Float64MultiArray, 'Realtime_XY', 10)
#         self.raw_gps_publisher = self.create_publisher(Float64MultiArray, 'Raw_GPS', 10)
#         self.xy_debug_publisher = self.create_publisher(Point, '/navigation/xy_debug', 10)
#         self.heading_publisher = self.create_publisher(Float32, '/navigation/heading', 10)
#         self.heading_debug_publisher = self.create_publisher(String, '/navigation/heading_debug', 10)
#         self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.navigation_debug_publisher = self.create_publisher(String, '/navigation/debug', 10)
        
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
        
#         # Enhanced Heading management
#         self.imu_heading = None
#         self.nav_att_heading = None
#         self.nav_att_yaw = None
#         self.gps_heading = None
#         self.final_heading = None
#         self.heading_source = "NONE"
        
#         self.imu_heading_time = 0
#         self.nav_att_time = 0
#         self.gps_heading_time = 0
#         self.heading_timeout = 8.0
        
#         # ZED-F9R specific data
#         self.esf_status = None
#         self.nav_att = None
#         self.esf_alg = None
#         self.fusion_mode = "NONE"
#         self.imu_calibration_status = "UNKNOWN"
        
#         # pyubx2 UBX parsing
#         self.ubx_reader = None
#         self.ubx_buffer = bytearray()
        
#         # Enhanced Heading filtering
#         self.heading_filter = EnhancedIMUHeadingFilter()
        
#         # Navigation Components with Pure Pursuit
#         csv_file_path = '/home/inc/ros2_ws/waypointfile/waypoints3xy.csv'
#         start_waypoint = 0
        
#         self.waypoint_manager = WaypointManager(csv_file_path, start_waypoint)
        
#         # สร้าง Pure Pursuit Controller
#         params = self.get_vehicle_parameters()
#         self.pure_pursuit = PurePursuitController(
#             lookahead_distance=params.get('lookahead_distance', 3.0),
#             lookahead_gain=params.get('lookahead_gain', 1.0),
#             min_lookahead=params.get('min_lookahead', 1.0)
#         )
        
#         self.bicycle_model = BicycleKinematicModel(wheelbase=params['wheelbase'])
        
#         # Navigation state
#         self.navigation_active = True
#         self.waypoint_reached_threshold = params.get('waypoint_threshold', 1.0)
        
#         # Fixed Speed Configuration
#         self.FIXED_SPEED = params['fixed_speed']
        
#         # Performance stats
#         self.stats = {
#             'imu_count': 0,
#             'nav_att_count': 0,
#             'nav_att_yaw_count': 0,
#             'gps_count': 0,
#             'position_count': 0,
#             'esf_meas_count': 0,
#             'esf_status_count': 0,
#             'pyubx2_success': 0,
#             'pyubx2_errors': 0,
#             'filtered_count': 0,
#             'navigation_commands': 0
#         }
        
#         # Initialize coordinate system
#         self.init_track_coordinate_system()
        
#         # Initialize serial connection
#         self.init_serial_connection()
        
#         if self.ser and self.ser.is_open:
#             self.ntrip_socket = self.connect_to_ntrip()
            
#             # Initialize pyubx2 if available
#             if PYUBX2_AVAILABLE:
#                 self.init_pyubx2()
            
#             # Configure ZED-F9R for enhanced IMU
#             self.configure_enhanced_zed_f9r()
            
#             # Main processing timer
#             self.create_timer(0.1, self.main_loop)
            
#             self.get_logger().info('✅ Enhanced ZED-F9R GNSS with Pure Pursuit Navigation initialized')
#             self.get_logger().info(f'🔧 pyubx2: {"Available" if PYUBX2_AVAILABLE else "Not available"}')
#             self.get_logger().info(f'🗺️ Loaded {len(self.waypoint_manager.waypoints)} waypoints')
#             self.get_logger().info(f'🚀 Fixed Speed: {self.FIXED_SPEED} m/s')
            
#             # Log vehicle parameters
#             self.log_vehicle_parameters()
#         else:
#             self.get_logger().error('❌ Failed to initialize')
    
#     def declare_vehicle_parameters(self):
#         """ประกาศ Vehicle Parameters สำหรับ Pure Pursuit"""
        
#         # Wheelbase parameter
#         wheelbase_descriptor = ParameterDescriptor(
#             description='ฐานล้อของรถ (ระยะห่างระหว่างเพลาหน้าและหลัง) ในหน่วยเมตร'
#         )
#         self.declare_parameter('wheelbase', 1.67, wheelbase_descriptor)
        
#         # Max steering angle parameter
#         steering_descriptor = ParameterDescriptor(
#             description='มุมเลี้ยวสูงสุดของรถในหน่วย radians (50 degrees = 0.873 rad)'
#         )
#         self.declare_parameter('max_steering_angle', 0.873, steering_descriptor)
        
#         # Fixed speed parameter
#         speed_descriptor = ParameterDescriptor(
#             description='ความเร็วคงที่ของรถในหน่วย m/s'
#         )
#         self.declare_parameter('fixed_speed', 1.0, speed_descriptor)
        
#         # Pure Pursuit parameters
#         lookahead_descriptor = ParameterDescriptor(
#             description='ระยะ lookahead พื้นฐานในหน่วยเมตร'
#         )
#         self.declare_parameter('lookahead_distance', 3.0, lookahead_descriptor)
        
#         lookahead_gain_descriptor = ParameterDescriptor(
#             description='ค่า gain สำหรับการคำนวณ lookahead distance ตามความเร็ว'
#         )
#         self.declare_parameter('lookahead_gain', 1.0, lookahead_gain_descriptor)
        
#         min_lookahead_descriptor = ParameterDescriptor(
#             description='ระยะ lookahead ขั้นต่ำในหน่วยเมตร'
#         )
#         self.declare_parameter('min_lookahead', 1.0, min_lookahead_descriptor)
        
#         # Waypoint threshold
#         waypoint_threshold_descriptor = ParameterDescriptor(
#             description='ระยะทางขั้นต่ำที่ถือว่าถึง waypoint แล้ว (เมตร)'
#         )
#         self.declare_parameter('waypoint_threshold', 1.0, waypoint_threshold_descriptor)
        
#         # Angular velocity limits
#         max_angular_vel_descriptor = ParameterDescriptor(
#             description='ความเร็วเชิงมุมสูงสุดในหน่วย rad/s'
#         )
#         self.declare_parameter('max_angular_velocity', 1.0, max_angular_vel_descriptor)
        
#         self.get_logger().info('🌟 Pure Pursuit parameters declared successfully!')
    
#     def get_vehicle_parameters(self):
#         """ดึงค่า Vehicle Parameters ปัจจุบัน"""
#         try:
#             wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
#             max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value
#             fixed_speed = self.get_parameter('fixed_speed').get_parameter_value().double_value
#             lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
#             lookahead_gain = self.get_parameter('lookahead_gain').get_parameter_value().double_value
#             min_lookahead = self.get_parameter('min_lookahead').get_parameter_value().double_value
#             waypoint_threshold = self.get_parameter('waypoint_threshold').get_parameter_value().double_value
#             max_angular_velocity = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
            
#             return {
#                 'wheelbase': wheelbase,
#                 'max_steering_angle': max_steering_angle,
#                 'fixed_speed': fixed_speed,
#                 'lookahead_distance': lookahead_distance,
#                 'lookahead_gain': lookahead_gain,
#                 'min_lookahead': min_lookahead,
#                 'waypoint_threshold': waypoint_threshold,
#                 'max_angular_velocity': max_angular_velocity
#             }
#         except Exception as e:
#             self.get_logger().error(f'❌ Error getting vehicle parameters: {e}')
#             return {
#                 'wheelbase': 1.67,
#                 'max_steering_angle': 0.873,
#                 'fixed_speed': 1.0,
#                 'lookahead_distance': 3.0,
#                 'lookahead_gain': 1.0,
#                 'min_lookahead': 1.0,
#                 'waypoint_threshold': 1.0,
#                 'max_angular_velocity': 1.0
#             }
    
#     def log_vehicle_parameters(self):
#         """แสดง Vehicle Parameters ปัจจุบัน"""
#         params = self.get_vehicle_parameters()
#         self.get_logger().info(
#             f'🚗 Vehicle Parameters:\n'
#             f'  🔧 Wheelbase: {params["wheelbase"]:.2f} m\n'
#             f'  🔧 Max Steering Angle: {params["max_steering_angle"]:.3f} rad ({math.degrees(params["max_steering_angle"]):.1f}°)\n'
#             f'  🔧 Fixed Speed: {params["fixed_speed"]:.2f} m/s\n'
#             f'  🔧 Max Angular Velocity: {params["max_angular_velocity"]:.2f} rad/s'
#         )
#         self.get_logger().info(
#             f'🎯 Pure Pursuit Parameters:\n'
#             f'  📏 Lookahead Distance: {params["lookahead_distance"]:.2f} m\n'
#             f'  📈 Lookahead Gain: {params["lookahead_gain"]:.2f}\n'
#             f'  📉 Min Lookahead: {params["min_lookahead"]:.2f} m\n'
#             f'  🎯 Waypoint Threshold: {params["waypoint_threshold"]:.2f} m'
#         )
    
#     def navigation_control(self):
#         """ระบบควบคุมการนำทางด้วย Pure Pursuit Algorithm"""
#         if not self.navigation_active or self.final_heading is None:
#             return
        
#         # อัพเดท current waypoint
#         waypoint_reached = self.waypoint_manager.update_current_waypoint(
#             self.current_position, 
#             self.waypoint_reached_threshold
#         )
        
#         if waypoint_reached:
#             progress = self.waypoint_manager.get_progress_info()
#             self.get_logger().info(
#                 f'✅ Reached waypoint {progress["current"]-1} '
#                 f'({progress["progress_percent"]:.1f}% complete)'
#             )
        
#         # ตรวจสอบว่าเสร็จสิ้นการนำทางแล้วหรือยัง
#         if self.waypoint_manager.current_waypoint_index >= len(self.waypoint_manager.waypoints) - 1:
#             current_waypoint = self.waypoint_manager.get_current_waypoint()
#             if current_waypoint:
#                 distance_to_final = math.sqrt(
#                     (self.current_position['x'] - current_waypoint['x'])**2 + 
#                     (self.current_position['y'] - current_waypoint['y'])**2
#                 )
                
#                 if distance_to_final < self.waypoint_reached_threshold:
#                     self.get_logger().info('🏁 Navigation completed')
#                     self.navigation_active = False
#                     self.send_stop_command()
#                     return
        
#         # ได้ parameters
#         params = self.get_vehicle_parameters()
        
#         # ใช้ความเร็วคงที่
#         linear_velocity = self.FIXED_SPEED
        
#         # อัพเดท lookahead distance ตามความเร็ว
#         self.pure_pursuit.update_lookahead_distance(linear_velocity)
        
#         # หา lookahead point
#         waypoints_ahead = self.waypoint_manager.get_waypoints_ahead(count=20)
#         lookahead_point, target_index = self.pure_pursuit.find_lookahead_point(
#             self.current_position,
#             waypoints_ahead,
#             0  # เริ่มจากจุดแรกใน waypoints_ahead
#         )
        
#         # อัพเดท target waypoint index
#         if target_index >= 0:
#             actual_target_index = self.waypoint_manager.current_waypoint_index + target_index
#             self.waypoint_manager.set_target_waypoint_index(actual_target_index)
        
#         # คำนวณมุมเลี้ยวด้วย Pure Pursuit
#         if lookahead_point:
#             steering_angle = self.pure_pursuit.calculate_steering_angle(
#                 self.current_position,
#                 self.final_heading,
#                 lookahead_point,
#                 params['wheelbase']
#             )
#         else:
#             steering_angle = 0.0
        
#         # จำกัดมุมเลี้ยว
#         steering_angle = self.bicycle_model.limit_steering_angle(
#             steering_angle, params['max_steering_angle']
#         )
        
#         # คำนวณ angular velocity ด้วย bicycle model
#         angular_velocity = self.bicycle_model.calculate_angular_velocity(
#             linear_velocity, steering_angle
#         )
        
#         # จำกัด angular velocity
#         angular_velocity = max(-params['max_angular_velocity'], 
#                              min(params['max_angular_velocity'], angular_velocity))
        
#         # สร้างและส่งคำสั่ง cmd_vel
#         cmd_vel = Twist()
#         cmd_vel.linear.x = linear_velocity
#         cmd_vel.angular.z = angular_velocity
        
#         self.cmd_vel_publisher.publish(cmd_vel)
#         self.stats['navigation_commands'] += 1
        
#         # Debug information
#         debug_msg = String()
#         progress = self.waypoint_manager.get_progress_info()
        
#         if lookahead_point:
#             lookahead_distance = math.sqrt(
#                 (lookahead_point['x'] - self.current_position['x'])**2 + 
#                 (lookahead_point['y'] - self.current_position['y'])**2
#             )
            
#             debug_msg.data = (
#                 f"Pure Pursuit Control | "
#                 f"Waypoint: {progress['current']}/{progress['total']} ({progress['progress_percent']:.1f}%) | "
#                 f"Target: {progress['target']} | "
#                 f"Lookahead: {self.pure_pursuit.lookahead_distance:.2f}m | "
#                 f"Distance: {lookahead_distance:.2f}m | "
#                 f"Steering: {math.degrees(steering_angle):.1f}° | "
#                 f"Speed: {linear_velocity:.2f}m/s | "
#                 f"Angular: {angular_velocity:.2f}rad/s"
#             )
#         else:
#             debug_msg.data = (
#                 f"Pure Pursuit Control | "
#                 f"Waypoint: {progress['current']}/{progress['total']} ({progress['progress_percent']:.1f}%) | "
#                 f"No lookahead point found | "
#                 f"Speed: {linear_velocity:.2f}m/s"
#             )
        
#         self.navigation_debug_publisher.publish(debug_msg)
    
#     def send_stop_command(self):
#         """ส่งคำสั่งหยุดรถ"""
#         cmd_vel = Twist()
#         cmd_vel.linear.x = 0.0
#         cmd_vel.angular.z = 0.0
#         self.cmd_vel_publisher.publish(cmd_vel)
#         self.get_logger().info('🛑 Stop command sent')
    
#     def init_track_coordinate_system(self):
#         """Initialize coordinate system"""
#         if self.reference_utm is None:
#             self.reference_utm = self.transformer.transform(
#                 self.reference_point[1], self.reference_point[0]
#             )
        
#         self.utm_offset_x = self.TARGET_UTM_X - self.reference_utm[0]
#         self.utm_offset_y = self.TARGET_UTM_Y - self.reference_utm[1]
        
#         self.get_logger().info(f'🗺️ Track coordinate system initialized')
#         self.get_logger().info(f'📍 Reference UTM: {self.reference_utm}')
#         self.get_logger().info(f'🎯 Target UTM: ({self.TARGET_UTM_X}, {self.TARGET_UTM_Y})')
#         self.get_logger().info(f'📐 Offsets: ({self.utm_offset_x:.2f}, {self.utm_offset_y:.2f})')
    
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
        
#         if not self.ser or not self.ser.is_open:
#             self.get_logger().error('❌ No serial connection available')
    
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
#         import base64
#         credentials = f"{self.ntrip_username}:{self.ntrip_password}"
#         return base64.b64encode(credentials.encode()).decode()
    
#     def init_pyubx2(self):
#         """Initialize pyubx2 if available"""
#         if PYUBX2_AVAILABLE:
#             self.get_logger().info('✅ pyubx2 initialized for enhanced UBX parsing')
    
#     def configure_enhanced_zed_f9r(self):
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
            
#             # อัพเดทตำแหน่งปัจจุบัน
#             self.update_current_position()
            
#             # ควบคุมการนำทาง
#             self.navigation_control()
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Main loop error: {e}')
    
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
#         # ประมวลผลข้อมูล NMEA และ UBX
#         # (ใช้โค้ดเดิมที่มีอยู่แล้วในการประมวลผล GNSS data)
#         lines = data.decode('utf-8', errors='ignore').split('\n')
        
#         for line in lines:
#             line = line.strip()
#             if line.startswith('$'):
#                 try:
#                     msg = pynmea2.parse(line)
#                     self.process_nmea_message(msg)
#                 except Exception as e:
#                     continue
    
#     def process_nmea_message(self, msg):
#         """Process NMEA messages"""
#         if hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
#             if msg.latitude and msg.longitude:
#                 # อัพเดทตำแหน่ง
#                 utm_x, utm_y = self.transformer.transform(msg.longitude, msg.latitude)
                
#                 # คำนวณตำแหน่งสัมพัทธ์
#                 relative_x = utm_x - self.reference_utm[0] + self.utm_offset_x
#                 relative_y = utm_y - self.reference_utm[1] + self.utm_offset_y
                
#                 self.current_position = {'x': relative_x, 'y': relative_y}
                
#                 # เพิ่มเข้า history
#                 self.position_history.append(self.current_position.copy())
                
#                 # คำนวณ GPS heading
#                 if len(self.position_history) >= 2:
#                     self.calculate_gps_heading()
                
#                 # Publish ข้อมูล
#                 self.publish_position_data(msg.latitude, msg.longitude, relative_x, relative_y)
    
#     def calculate_gps_heading(self):
#         """คำนวณ heading จาก GPS movement"""
#         if len(self.position_history) < 2:
#             return
        
#         current = self.position_history[-1]
#         previous = self.position_history[-2]
        
#         dx = current['x'] - previous['x']
#         dy = current['y'] - previous['y']
        
#         distance = math.sqrt(dx**2 + dy**2)
        
#         if distance > self.min_movement_for_heading:
#             heading = math.degrees(math.atan2(dy, dx))
#             if heading < 0:
#                 heading += 360
            
#             # Filter GPS heading
#             filtered_heading = self.heading_filter.filter_gps_heading(heading)
#             if filtered_heading is not None:
#                 self.gps_heading = filtered_heading
#                 self.gps_heading_time = time.time()
    
#     def update_current_position(self):
#         """Update current position and heading"""
#         current_time = time.time()
        
#         # อัพเดท final heading
#         if (self.nav_att_heading is not None and 
#             current_time - self.nav_att_time < self.heading_timeout):
#             self.final_heading = self.nav_att_heading
#             self.heading_source = "NAV-ATT"
#         elif (self.gps_heading is not None and 
#               current_time - self.gps_heading_time < self.heading_timeout):
#             self.final_heading = self.gps_heading
#             self.heading_source = "GPS"
#         else:
#             self.heading_source = "NONE"
        
#         # Publish heading
#         if self.final_heading is not None:
#             heading_msg = Float32()
#             heading_msg.data = self.final_heading
#             self.heading_publisher.publish(heading_msg)
    
#     def publish_position_data(self, lat, lon, x, y):
#         """Publish position data"""
#         # Publish XY coordinates
#         xy_msg = Float64MultiArray()
#         xy_msg.data = [x, y]
#         self.xy_publisher.publish(xy_msg)
        
#         # Publish raw GPS
#         raw_gps_msg = Float64MultiArray()
#         raw_gps_msg.data = [lat, lon]
#         self.raw_gps_publisher.publish(raw_gps_msg)
        
#         # Publish debug point
#         debug_point = Point()
#         debug_point.x = x
#         debug_point.y = y
#         debug_point.z = 0.0
#         self.xy_debug_publisher.publish(debug_point)

# def main(args=None):
#     rclpy.init(args=args)
    
#     try:
#         node = EnhancedZEDf9rGNSSWithPurePursuit()
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         print('\n🛑 Navigation stopped by user')
#     except Exception as e:
#         print(f'❌ Error: {e}')
#     finally:
#         if 'node' in locals():
#             node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



# #!/usr/bin/env python3
# import serial
# import pynmea2
# import socket
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
# from std_msgs.msg import Float32, String, Float64MultiArray
# from geometry_msgs.msg import Point, Twist
# from rcl_interfaces.msg import ParameterDescriptor
# import math
# import pyproj
# from collections import deque
# import time
# import glob
# import os
# import struct
# import json
# import numpy as np
# import csv

# # เพิ่ม pyubx2 สำหรับ UBX parsing ที่ดีขึ้น
# try:
#     from pyubx2 import UBXReader, UBXMessage, UBX_MSGIDS
#     PYUBX2_AVAILABLE = True
#     print("✅ pyubx2 available - using enhanced UBX parsing")
# except ImportError:
#     PYUBX2_AVAILABLE = False
#     print("⚠️ pyubx2 not available - install with: pip3 install pyubx2")

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

# # ✅ Enhanced Waypoint Manager with Flexible Starting Point
# class WaypointManager:
#     def __init__(self, csv_file_path, start_waypoint_index=0):
#         self.waypoints = []
#         self.current_waypoint_index = start_waypoint_index
#         self.target_waypoint_index = start_waypoint_index
#         self.load_waypoints_from_csv(csv_file_path)
        
#         # ตรวจสอบว่าจุดเริ่มต้นอยู่ในช่วงที่ถูกต้อง
#         if self.current_waypoint_index >= len(self.waypoints):
#             self.current_waypoint_index = 0
#             print(f"⚠️ Starting waypoint index out of range, reset to 0")
        
#         print(f"✅ Starting navigation from waypoint {self.current_waypoint_index + 1}")
#         print(f"📊 Total waypoints loaded: {len(self.waypoints)}")
    
#     def load_waypoints_from_csv(self, csv_file_path):
#         """โหลด waypoints จากไฟล์ CSV"""
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
    
#     def set_starting_waypoint(self, waypoint_index):
#         """✅ กำหนดจุดเริ่มต้นใหม่ - สามารถเริ่มจากจุดไหนก็ได้"""
#         if 0 <= waypoint_index < len(self.waypoints):
#             self.current_waypoint_index = waypoint_index
#             self.target_waypoint_index = waypoint_index
#             print(f"✅ Starting waypoint set to {waypoint_index + 1}")
#             return True
#         else:
#             print(f"❌ Invalid waypoint index: {waypoint_index}. Valid range: 1-{len(self.waypoints)}")
#             return False
    
#     def get_current_waypoint(self):
#         """ได้ waypoint ปัจจุบัน"""
#         if self.current_waypoint_index < len(self.waypoints):
#             return self.waypoints[self.current_waypoint_index]
#         return None
    
#     def get_target_waypoint(self):
#         """ได้ target waypoint สำหรับ Pure Pursuit"""
#         if self.target_waypoint_index < len(self.waypoints):
#             return self.waypoints[self.target_waypoint_index]
#         return None
    
#     def get_waypoints_ahead(self, count=10):
#         """ได้ waypoints ข้างหน้าสำหรับการหา lookahead point"""
#         start_idx = self.current_waypoint_index
#         end_idx = min(start_idx + count, len(self.waypoints))
#         return self.waypoints[start_idx:end_idx]
    
#     def update_current_waypoint(self, current_pos, threshold=1.0):
#         """อัพเดท current waypoint ตามตำแหน่งปัจจุบัน"""
#         current_waypoint = self.get_current_waypoint()
#         if current_waypoint is None:
#             return False
            
#         # ตรวจสอบว่าถึง waypoint แล้วหรือยัง
#         distance = math.sqrt(
#             (current_pos['x'] - current_waypoint['x'])**2 + 
#             (current_pos['y'] - current_waypoint['y'])**2
#         )
        
#         if distance < threshold:
#             if self.current_waypoint_index < len(self.waypoints) - 1:
#                 self.current_waypoint_index += 1
#                 # อัพเดท target waypoint ด้วย
#                 self.target_waypoint_index = max(
#                     self.target_waypoint_index, 
#                     self.current_waypoint_index
#                 )
#                 return True
        
#         return False
    
#     def set_target_waypoint_index(self, index):
#         """กำหนด target waypoint index"""
#         if 0 <= index < len(self.waypoints):
#             self.target_waypoint_index = index
    
#     def get_progress_info(self):
#         """ได้ข้อมูลความคืบหน้า"""
#         return {
#             'current': self.current_waypoint_index + 1,
#             'target': self.target_waypoint_index + 1,
#             'total': len(self.waypoints),
#             'progress_percent': ((self.current_waypoint_index + 1) / len(self.waypoints)) * 100
#         }

# # Pure Pursuit Controller
# class PurePursuitController:
#     def __init__(self, lookahead_distance=3.0, lookahead_gain=1.0, min_lookahead=1.0):
#         self.lookahead_distance = lookahead_distance
#         self.lookahead_gain = lookahead_gain
#         self.min_lookahead = min_lookahead
#         self.current_speed = 0.0
        
#     def update_lookahead_distance(self, current_speed):
#         """อัพเดทระยะ lookahead ตามความเร็วปัจจุบัน"""
#         self.current_speed = current_speed
#         # ระยะ lookahead = gain * speed + min_lookahead
#         self.lookahead_distance = max(
#             self.lookahead_gain * current_speed + self.min_lookahead,
#             self.min_lookahead
#         )
        
#     def find_lookahead_point(self, current_pos, waypoints, current_waypoint_index):
#         """หาจุด lookahead point จาก waypoints"""
#         if not waypoints or current_waypoint_index >= len(waypoints):
#             return None, current_waypoint_index
            
#         # เริ่มหาจากจุดปัจจุบัน
#         for i in range(current_waypoint_index, len(waypoints)):
#             waypoint = waypoints[i]
#             distance = math.sqrt(
#                 (waypoint['x'] - current_pos['x'])**2 + 
#                 (waypoint['y'] - current_pos['y'])**2
#             )
            
#             # ถ้าระยะทางมากกว่า lookahead distance ให้ใช้จุดนี้
#             if distance >= self.lookahead_distance:
#                 return waypoint, i
                
#         # ถ้าไม่เจอ ให้ใช้จุดสุดท้าย
#         if waypoints:
#             return waypoints[-1], len(waypoints) - 1
        
#         return None, current_waypoint_index
    
#     def calculate_steering_angle(self, current_pos, current_heading, target_pos, wheelbase):
#         """คำนวณมุมเลี้ยวด้วย Pure Pursuit Algorithm"""
#         if target_pos is None:
#             return 0.0
            
#         # คำนวณระยะทางและมุมไปยังเป้าหมาย
#         dx = target_pos['x'] - current_pos['x']
#         dy = target_pos['y'] - current_pos['y']
        
#         # ระยะทางไปยังเป้าหมาย
#         distance = math.sqrt(dx**2 + dy**2)
        
#         if distance < 0.1:  # ถ้าใกล้มาก
#             return 0.0
            
#         # คำนวณมุมไปยังเป้าหมาย
#         target_heading = math.degrees(math.atan2(dy, dx))
        
#         # คำนวณ heading error
#         heading_error = self.normalize_angle(target_heading - current_heading)
        
#         # Pure Pursuit Formula
#         # δ = atan(2 * L * sin(α) / ld)
#         # โดย α = heading_error, L = wheelbase, ld = lookahead distance
#         alpha = math.radians(heading_error)
        
#         # ใช้ระยะทางจริงแทน lookahead distance ถ้าใกล้กว่า
#         effective_distance = min(distance, self.lookahead_distance)
        
#         if effective_distance < 0.1:
#             return 0.0
            
#         steering_angle = math.atan(2 * wheelbase * math.sin(alpha) / effective_distance)
        
#         return steering_angle
    
#     def normalize_angle(self, angle):
#         """Normalize angle to [-180, 180]"""
#         while angle > 180:
#             angle -= 360
#         while angle < -180:
#             angle += 360
#         return angle

# # Bicycle Kinematic Model
# class BicycleKinematicModel:
#     def __init__(self, wheelbase):
#         self.wheelbase = wheelbase
    
#     def calculate_angular_velocity(self, linear_velocity, steering_angle):
#         """คำนวณ angular velocity จาก bicycle model"""
#         if abs(steering_angle) < 1e-6:  # ป้องกันการหารด้วยศูนย์
#             return 0.0
        
#         angular_velocity = (linear_velocity * math.tan(steering_angle)) / self.wheelbase
#         return angular_velocity
    
#     def limit_steering_angle(self, steering_angle, max_steering_angle):
#         """จำกัดมุมเลี้ยวให้อยู่ในขอบเขต"""
#         return max(-max_steering_angle, min(max_steering_angle, steering_angle))

# # ✅ Main Enhanced GNSS Node with Flexible Starting Point and cmd_vel Output
# class EnhancedZEDf9rGNSSWithPurePursuit(Node):
#     def __init__(self):
#         super().__init__('enhanced_zedf9r_gnss_with_pure_pursuit')
        
#         # ประกาศ parameters
#         self.declare_vehicle_parameters()
        
#         # ✅ ROS2 Publishers - ส่งออกเป็น cmd_vel
#         self.gnss_publisher = self.create_publisher(NavSatFix, '/navigation/gnss', 10)
#         self.xy_publisher = self.create_publisher(Float64MultiArray, 'Realtime_XY', 10)
#         self.raw_gps_publisher = self.create_publisher(Float64MultiArray, 'Raw_GPS', 10)
#         self.xy_debug_publisher = self.create_publisher(Point, '/navigation/xy_debug', 10)
#         self.heading_publisher = self.create_publisher(Float32, '/navigation/heading', 10)
#         self.heading_debug_publisher = self.create_publisher(String, '/navigation/heading_debug', 10)
        
#         # ✅ cmd_vel Publisher - หลักสำหรับการควบคุมรถ
#         self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.navigation_debug_publisher = self.create_publisher(String, '/navigation/debug', 10)
        
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
        
#         # Enhanced Heading management
#         self.imu_heading = None
#         self.nav_att_heading = None
#         self.nav_att_yaw = None
#         self.gps_heading = None
#         self.final_heading = None
#         self.heading_source = "NONE"
        
#         self.imu_heading_time = 0
#         self.nav_att_time = 0
#         self.gps_heading_time = 0
#         self.heading_timeout = 8.0
        
#         # ZED-F9R specific data
#         self.esf_status = None
#         self.nav_att = None
#         self.esf_alg = None
#         self.fusion_mode = "NONE"
#         self.imu_calibration_status = "UNKNOWN"
        
#         # pyubx2 UBX parsing
#         self.ubx_reader = None
#         self.ubx_buffer = bytearray()
        
#         # Enhanced Heading filtering
#         self.heading_filter = EnhancedIMUHeadingFilter()
        
#         # ✅ Navigation Components with Flexible Starting Point
#         csv_file_path = '/home/inc/ros2_ws/waypointfile/waypoints3xy.csv'
        
#         # ✅ ดึงค่า starting waypoint จาก parameter
#         params = self.get_vehicle_parameters()
#         start_waypoint = params.get('start_waypoint_index', 0)
        
#         self.waypoint_manager = WaypointManager(csv_file_path, start_waypoint)
        
#         # สร้าง Pure Pursuit Controller
#         self.pure_pursuit = PurePursuitController(
#             lookahead_distance=params.get('lookahead_distance', 3.0),
#             lookahead_gain=params.get('lookahead_gain', 1.0),
#             min_lookahead=params.get('min_lookahead', 1.0)
#         )
        
#         self.bicycle_model = BicycleKinematicModel(wheelbase=params['wheelbase'])
        
#         # Navigation state
#         self.navigation_active = True
#         self.waypoint_reached_threshold = params.get('waypoint_threshold', 1.0)
        
#         # Fixed Speed Configuration
#         self.FIXED_SPEED = params['fixed_speed']
        
#         # Performance stats
#         self.stats = {
#             'imu_count': 0,
#             'nav_att_count': 0,
#             'nav_att_yaw_count': 0,
#             'gps_count': 0,
#             'position_count': 0,
#             'esf_meas_count': 0,
#             'esf_status_count': 0,
#             'pyubx2_success': 0,
#             'pyubx2_errors': 0,
#             'filtered_count': 0,
#             'navigation_commands': 0
#         }
        
#         # Initialize coordinate system
#         self.init_track_coordinate_system()
        
#         # Initialize serial connection
#         self.init_serial_connection()
        
#         if self.ser and self.ser.is_open:
#             self.ntrip_socket = self.connect_to_ntrip()
            
#             # Initialize pyubx2 if available
#             if PYUBX2_AVAILABLE:
#                 self.init_pyubx2()
            
#             # Configure ZED-F9R for enhanced IMU
#             self.configure_enhanced_zed_f9r()
            
#             # Main processing timer
#             self.create_timer(0.1, self.main_loop)
            
#             self.get_logger().info('✅ Enhanced ZED-F9R GNSS with Pure Pursuit Navigation initialized')
#             self.get_logger().info(f'🔧 pyubx2: {"Available" if PYUBX2_AVAILABLE else "Not available"}')
#             self.get_logger().info(f'🗺️ Loaded {len(self.waypoint_manager.waypoints)} waypoints')
#             self.get_logger().info(f'🚀 Starting from waypoint: {start_waypoint + 1}')
#             self.get_logger().info(f'🚀 Fixed Speed: {self.FIXED_SPEED} m/s')
#             self.get_logger().info(f'📡 Publishing cmd_vel to /cmd_vel topic')
            
#             # Log vehicle parameters
#             self.log_vehicle_parameters()
#         else:
#             self.get_logger().error('❌ Failed to initialize')
    
#     def declare_vehicle_parameters(self):
#         """✅ ประกาศ Vehicle Parameters สำหรับ Pure Pursuit"""
        
#         # Wheelbase parameter
#         wheelbase_descriptor = ParameterDescriptor(
#             description='ฐานล้อของรถ (ระยะห่างระหว่างเพลาหน้าและหลัง) ในหน่วยเมตร'
#         )
#         self.declare_parameter('wheelbase', 1.67, wheelbase_descriptor)
        
#         # Max steering angle parameter
#         steering_descriptor = ParameterDescriptor(
#             description='มุมเลี้ยวสูงสุดของรถในหน่วย radians (50 degrees = 0.873 rad)'
#         )
#         self.declare_parameter('max_steering_angle', 0.873, steering_descriptor)
        
#         # Fixed speed parameter
#         speed_descriptor = ParameterDescriptor(
#             description='ความเร็วคงที่ของรถในหน่วย m/s'
#         )
#         self.declare_parameter('fixed_speed', 1.0, speed_descriptor)
        
#         # ✅ Starting waypoint parameter - สามารถเริ่มจากจุดไหนก็ได้
#         start_waypoint_descriptor = ParameterDescriptor(
#             description='จุดเริ่มต้นของการนำทาง (waypoint index, เริ่มจาก 0) - สามารถเริ่มจากจุดไหนก็ได้'
#         )
#         self.declare_parameter('start_waypoint_index', 0, start_waypoint_descriptor)
        
#         # Pure Pursuit parameters
#         lookahead_descriptor = ParameterDescriptor(
#             description='ระยะ lookahead พื้นฐานในหน่วยเมตร'
#         )
#         self.declare_parameter('lookahead_distance', 3.0, lookahead_descriptor)
        
#         lookahead_gain_descriptor = ParameterDescriptor(
#             description='ค่า gain สำหรับการคำนวณ lookahead distance ตามความเร็ว'
#         )
#         self.declare_parameter('lookahead_gain', 1.0, lookahead_gain_descriptor)
        
#         min_lookahead_descriptor = ParameterDescriptor(
#             description='ระยะ lookahead ขั้นต่ำในหน่วยเมตร'
#         )
#         self.declare_parameter('min_lookahead', 1.0, min_lookahead_descriptor)
        
#         # Waypoint threshold
#         waypoint_threshold_descriptor = ParameterDescriptor(
#             description='ระยะทางขั้นต่ำที่ถือว่าถึง waypoint แล้ว (เมตร)'
#         )
#         self.declare_parameter('waypoint_threshold', 1.0, waypoint_threshold_descriptor)
        
#         # Angular velocity limits
#         max_angular_vel_descriptor = ParameterDescriptor(
#             description='ความเร็วเชิงมุมสูงสุดในหน่วย rad/s'
#         )
#         self.declare_parameter('max_angular_velocity', 1.0, max_angular_vel_descriptor)
        
#         self.get_logger().info('🌟 Pure Pursuit parameters declared successfully!')
    
#     def get_vehicle_parameters(self):
#         """ดึงค่า Vehicle Parameters ปัจจุบัน"""
#         try:
#             wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
#             max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value
#             fixed_speed = self.get_parameter('fixed_speed').get_parameter_value().double_value
#             start_waypoint_index = self.get_parameter('start_waypoint_index').get_parameter_value().integer_value
#             lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
#             lookahead_gain = self.get_parameter('lookahead_gain').get_parameter_value().double_value
#             min_lookahead = self.get_parameter('min_lookahead').get_parameter_value().double_value
#             waypoint_threshold = self.get_parameter('waypoint_threshold').get_parameter_value().double_value
#             max_angular_velocity = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
            
#             return {
#                 'wheelbase': wheelbase,
#                 'max_steering_angle': max_steering_angle,
#                 'fixed_speed': fixed_speed,
#                 'start_waypoint_index': start_waypoint_index,
#                 'lookahead_distance': lookahead_distance,
#                 'lookahead_gain': lookahead_gain,
#                 'min_lookahead': min_lookahead,
#                 'waypoint_threshold': waypoint_threshold,
#                 'max_angular_velocity': max_angular_velocity
#             }
#         except Exception as e:
#             self.get_logger().error(f'❌ Error getting vehicle parameters: {e}')
#             return {
#                 'wheelbase': 1.67,
#                 'max_steering_angle': 0.873,
#                 'fixed_speed': 1.0,
#                 'start_waypoint_index': 0,
#                 'lookahead_distance': 3.0,
#                 'lookahead_gain': 1.0,
#                 'min_lookahead': 1.0,
#                 'waypoint_threshold': 1.0,
#                 'max_angular_velocity': 1.0
#             }
    
#     def log_vehicle_parameters(self):
#         """แสดง Vehicle Parameters ปัจจุบัน"""
#         params = self.get_vehicle_parameters()
#         self.get_logger().info(
#             f'🚗 Vehicle Parameters:\n'
#             f'  🔧 Wheelbase: {params["wheelbase"]:.2f} m\n'
#             f'  🔧 Max Steering Angle: {params["max_steering_angle"]:.3f} rad ({math.degrees(params["max_steering_angle"]):.1f}°)\n'
#             f'  🔧 Fixed Speed: {params["fixed_speed"]:.2f} m/s\n'
#             f'  🔧 Starting Waypoint: {params["start_waypoint_index"] + 1}\n'
#             f'  🔧 Max Angular Velocity: {params["max_angular_velocity"]:.2f} rad/s'
#         )
#         self.get_logger().info(
#             f'🎯 Pure Pursuit Parameters:\n'
#             f'  📏 Lookahead Distance: {params["lookahead_distance"]:.2f} m\n'
#             f'  📈 Lookahead Gain: {params["lookahead_gain"]:.2f}\n'
#             f'  📉 Min Lookahead: {params["min_lookahead"]:.2f} m\n'
#             f'  🎯 Waypoint Threshold: {params["waypoint_threshold"]:.2f} m'
#         )
    
#     def navigation_control(self):
#         """✅ ระบบควบคุมการนำทางด้วย Pure Pursuit Algorithm และส่งออกเป็น cmd_vel"""
#         if not self.navigation_active or self.final_heading is None:
#             return
        
#         # อัพเดท current waypoint
#         waypoint_reached = self.waypoint_manager.update_current_waypoint(
#             self.current_position, 
#             self.waypoint_reached_threshold
#         )
        
#         if waypoint_reached:
#             progress = self.waypoint_manager.get_progress_info()
#             self.get_logger().info(
#                 f'✅ Reached waypoint {progress["current"]-1} '
#                 f'({progress["progress_percent"]:.1f}% complete)'
#             )
        
#         # ตรวจสอบว่าเสร็จสิ้นการนำทางแล้วหรือยัง
#         if self.waypoint_manager.current_waypoint_index >= len(self.waypoint_manager.waypoints) - 1:
#             current_waypoint = self.waypoint_manager.get_current_waypoint()
#             if current_waypoint:
#                 distance_to_final = math.sqrt(
#                     (self.current_position['x'] - current_waypoint['x'])**2 + 
#                     (self.current_position['y'] - current_waypoint['y'])**2
#                 )
                
#                 if distance_to_final < self.waypoint_reached_threshold:
#                     self.get_logger().info('🏁 Navigation completed')
#                     self.navigation_active = False
#                     self.send_stop_command()
#                     return
        
#         # ได้ parameters
#         params = self.get_vehicle_parameters()
        
#         # ใช้ความเร็วคงที่
#         linear_velocity = self.FIXED_SPEED
        
#         # อัพเดท lookahead distance ตามความเร็ว
#         self.pure_pursuit.update_lookahead_distance(linear_velocity)
        
#         # หา lookahead point
#         waypoints_ahead = self.waypoint_manager.get_waypoints_ahead(count=20)
#         lookahead_point, target_index = self.pure_pursuit.find_lookahead_point(
#             self.current_position,
#             waypoints_ahead,
#             0  # เริ่มจากจุดแรกใน waypoints_ahead
#         )
        
#         # อัพเดท target waypoint index
#         if target_index >= 0:
#             actual_target_index = self.waypoint_manager.current_waypoint_index + target_index
#             self.waypoint_manager.set_target_waypoint_index(actual_target_index)
        
#         # คำนวณมุมเลี้ยวด้วย Pure Pursuit
#         if lookahead_point:
#             steering_angle = self.pure_pursuit.calculate_steering_angle(
#                 self.current_position,
#                 self.final_heading,
#                 lookahead_point,
#                 params['wheelbase']
#             )
#         else:
#             steering_angle = 0.0
        
#         # จำกัดมุมเลี้ยว
#         steering_angle = self.bicycle_model.limit_steering_angle(
#             steering_angle, params['max_steering_angle']
#         )
        
#         # คำนวณ angular velocity ด้วย bicycle model
#         angular_velocity = self.bicycle_model.calculate_angular_velocity(
#             linear_velocity, steering_angle
#         )
        
#         # จำกัด angular velocity
#         angular_velocity = max(-params['max_angular_velocity'], 
#                              min(params['max_angular_velocity'], angular_velocity))
        
#         # ✅ สร้างและส่งคำสั่ง cmd_vel
#         cmd_vel = Twist()
#         cmd_vel.linear.x = linear_velocity
#         cmd_vel.linear.y = 0.0
#         cmd_vel.linear.z = 0.0
#         cmd_vel.angular.x = 0.0
#         cmd_vel.angular.y = 0.0
#         cmd_vel.angular.z = angular_velocity
        
#         # ✅ ส่งออกเป็น cmd_vel
#         self.cmd_vel_publisher.publish(cmd_vel)
#         self.stats['navigation_commands'] += 1
        
#         # Debug information
#         debug_msg = String()
#         progress = self.waypoint_manager.get_progress_info()
        
#         if lookahead_point:
#             lookahead_distance = math.sqrt(
#                 (lookahead_point['x'] - self.current_position['x'])**2 + 
#                 (lookahead_point['y'] - self.current_position['y'])**2
#             )
            
#             debug_msg.data = (
#                 f"Pure Pursuit Control | "
#                 f"Waypoint: {progress['current']}/{progress['total']} ({progress['progress_percent']:.1f}%) | "
#                 f"Target: {progress['target']} | "
#                 f"Lookahead: {self.pure_pursuit.lookahead_distance:.2f}m | "
#                 f"Distance: {lookahead_distance:.2f}m | "
#                 f"Steering: {math.degrees(steering_angle):.1f}° | "
#                 f"Speed: {linear_velocity:.2f}m/s | "
#                 f"Angular: {angular_velocity:.2f}rad/s | "
#                 f"CMD_VEL: linear.x={linear_velocity:.2f}, angular.z={angular_velocity:.2f}"
#             )
#         else:
#             debug_msg.data = (
#                 f"Pure Pursuit Control | "
#                 f"Waypoint: {progress['current']}/{progress['total']} ({progress['progress_percent']:.1f}%) | "
#                 f"No lookahead point found | "
#                 f"Speed: {linear_velocity:.2f}m/s | "
#                 f"CMD_VEL: linear.x={linear_velocity:.2f}, angular.z=0.0"
#             )
        
#         self.navigation_debug_publisher.publish(debug_msg)
    
#     def send_stop_command(self):
#         """✅ ส่งคำสั่งหยุดรถผ่าน cmd_vel"""
#         cmd_vel = Twist()
#         cmd_vel.linear.x = 0.0
#         cmd_vel.linear.y = 0.0
#         cmd_vel.linear.z = 0.0
#         cmd_vel.angular.x = 0.0
#         cmd_vel.angular.y = 0.0
#         cmd_vel.angular.z = 0.0
        
#         self.cmd_vel_publisher.publish(cmd_vel)
#         self.get_logger().info('🛑 Stop command sent via cmd_vel')
    
#     # ✅ ฟังก์ชันสำหรับเปลี่ยนจุดเริ่มต้นระหว่างการทำงาน
#     def set_starting_waypoint_service(self, waypoint_index):
#         """✅ เปลี่ยนจุดเริ่มต้นระหว่างการทำงาน"""
#         if self.waypoint_manager.set_starting_waypoint(waypoint_index):
#             self.navigation_active = True
#             self.get_logger().info(f'🚀 Navigation restarted from waypoint {waypoint_index + 1}')
#             self.get_logger().info(f'📡 Publishing cmd_vel commands to /cmd_vel')
#             return True
#         return False
    
#     # เพิ่มฟังก์ชันที่จำเป็นสำหรับการทำงาน
#     def init_track_coordinate_system(self):
#         """Initialize coordinate system"""
#         if self.reference_utm is None:
#             self.reference_utm = self.transformer.transform(
#                 self.reference_point[1], self.reference_point[0]
#             )
        
#         self.utm_offset_x = self.TARGET_UTM_X - self.reference_utm[0]
#         self.utm_offset_y = self.TARGET_UTM_Y - self.reference_utm[1]
        
#         self.get_logger().info(f'🗺️ Track coordinate system initialized')
#         self.get_logger().info(f'📍 Reference UTM: {self.reference_utm}')
#         self.get_logger().info(f'🎯 Target UTM: ({self.TARGET_UTM_X}, {self.TARGET_UTM_Y})')
#         self.get_logger().info(f'📐 Offsets: ({self.utm_offset_x:.2f}, {self.utm_offset_y:.2f})')
    
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
        
#         if not self.ser or not self.ser.is_open:
#             self.get_logger().error('❌ No serial connection available')
    
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
#         import base64
#         credentials = f"{self.ntrip_username}:{self.ntrip_password}"
#         return base64.b64encode(credentials.encode()).decode()
    
#     def init_pyubx2(self):
#         """Initialize pyubx2 if available"""
#         if PYUBX2_AVAILABLE:
#             self.get_logger().info('✅ pyubx2 initialized for enhanced UBX parsing')
    
#     def configure_enhanced_zed_f9r(self):
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
            
#             # อัพเดทตำแหน่งปัจจุบัน
#             self.update_current_position()
            
#             # ควบคุมการนำทาง
#             self.navigation_control()
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Main loop error: {e}')
    
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
#         # ประมวลผลข้อมูล NMEA และ UBX
#         lines = data.decode('utf-8', errors='ignore').split('\n')
        
#         for line in lines:
#             line = line.strip()
#             if line.startswith('$'):
#                 try:
#                     msg = pynmea2.parse(line)
#                     self.process_nmea_message(msg)
#                 except Exception as e:
#                     continue
    
#     def process_nmea_message(self, msg):
#         """Process NMEA messages"""
#         if hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
#             if msg.latitude and msg.longitude:
#                 # อัพเดทตำแหน่ง
#                 utm_x, utm_y = self.transformer.transform(msg.longitude, msg.latitude)
                
#                 # คำนวณตำแหน่งสัมพัทธ์
#                 relative_x = utm_x - self.reference_utm[0] + self.utm_offset_x
#                 relative_y = utm_y - self.reference_utm[1] + self.utm_offset_y
                
#                 self.current_position = {'x': relative_x, 'y': relative_y}
                
#                 # เพิ่มเข้า history
#                 self.position_history.append(self.current_position.copy())
                
#                 # คำนวณ GPS heading
#                 if len(self.position_history) >= 2:
#                     self.calculate_gps_heading()
                
#                 # Publish ข้อมูล
#                 self.publish_position_data(msg.latitude, msg.longitude, relative_x, relative_y)
    
#     def calculate_gps_heading(self):
#         """คำนวณ heading จาก GPS movement"""
#         if len(self.position_history) < 2:
#             return
        
#         current = self.position_history[-1]
#         previous = self.position_history[-2]
        
#         dx = current['x'] - previous['x']
#         dy = current['y'] - previous['y']
        
#         distance = math.sqrt(dx**2 + dy**2)
        
#         if distance > self.min_movement_for_heading:
#             heading = math.degrees(math.atan2(dy, dx))
#             if heading < 0:
#                 heading += 360
            
#             # Filter GPS heading
#             filtered_heading = self.heading_filter.filter_gps_heading(heading)
#             if filtered_heading is not None:
#                 self.gps_heading = filtered_heading
#                 self.gps_heading_time = time.time()
    
#     def update_current_position(self):
#         """Update current position and heading"""
#         current_time = time.time()
        
#         # อัพเดท final heading
#         if (self.nav_att_heading is not None and 
#             current_time - self.nav_att_time < self.heading_timeout):
#             self.final_heading = self.nav_att_heading
#             self.heading_source = "NAV-ATT"
#         elif (self.gps_heading is not None and 
#               current_time - self.gps_heading_time < self.heading_timeout):
#             self.final_heading = self.gps_heading
#             self.heading_source = "GPS"
#         else:
#             self.heading_source = "NONE"
        
#         # Publish heading
#         if self.final_heading is not None:
#             heading_msg = Float32()
#             heading_msg.data = self.final_heading
#             self.heading_publisher.publish(heading_msg)
    
#     def publish_position_data(self, lat, lon, x, y):
#         """Publish position data"""
#         # Publish XY coordinates
#         xy_msg = Float64MultiArray()
#         xy_msg.data = [x, y]
#         self.xy_publisher.publish(xy_msg)
        
#         # Publish raw GPS
#         raw_gps_msg = Float64MultiArray()
#         raw_gps_msg.data = [lat, lon]
#         self.raw_gps_publisher.publish(raw_gps_msg)
        
#         # Publish debug point
#         debug_point = Point()
#         debug_point.x = x
#         debug_point.y = y
#         debug_point.z = 0.0
#         self.xy_debug_publisher.publish(debug_point)

# def main(args=None):
#     rclpy.init(args=args)
    
#     try:
#         node = EnhancedZEDf9rGNSSWithPurePursuit()
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         print('\n🛑 Navigation stopped by user')
#     except Exception as e:
#         print(f'❌ Error: {e}')
#     finally:
#         if 'node' in locals():
#             node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



# import serial
# import pynmea2
# import socket
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
# from std_msgs.msg import Float32, String, Float64MultiArray
# from geometry_msgs.msg import Point, Twist
# from rcl_interfaces.msg import ParameterDescriptor
# import math
# import pyproj
# from collections import deque
# import time
# import glob
# import os
# import struct
# import json
# import numpy as np
# import csv

# # เพิ่ม pyubx2 สำหรับ UBX parsing ที่ดีขึ้น
# try:
#     from pyubx2 import UBXReader, UBXMessage, UBX_MSGIDS
#     PYUBX2_AVAILABLE = True
#     print("✅ pyubx2 available - using enhanced UBX parsing")
# except ImportError:
#     PYUBX2_AVAILABLE = False
#     print("⚠️ pyubx2 not available - install with: pip3 install pyubx2")

# # Enhanced IMU Heading Filter (ปรับปรุงให้ใช้ compass bearing)
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
#         """Enhanced NAV-ATT heading filtering with confidence (compass bearing)"""
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
#         """Enhanced GPS heading filtering (compass bearing)"""
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
#         """Check if NAV-ATT heading is outlier (compass bearing)"""
#         if len(self.nav_att_history) < 2:
#             return False
        
#         recent_headings = [h[0] for h in list(self.nav_att_history)[-3:]]
#         recent_avg = self.circular_mean(recent_headings)
#         diff = abs(self.angle_difference(new_heading, recent_avg))
        
#         return diff > self.nav_att_outlier_threshold
    
#     def is_gps_outlier(self, new_heading):
#         """Check if GPS heading is outlier (compass bearing)"""
#         if len(self.gps_history) < 2:
#             return False
        
#         recent_avg = self.circular_mean(list(self.gps_history)[-2:])
#         diff = abs(self.angle_difference(new_heading, recent_avg))
        
#         return diff > self.gps_outlier_threshold
    
#     def apply_nav_att_smoothing(self):
#         """Enhanced NAV-ATT smoothing with confidence weighting (compass bearing)"""
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
#         """GPS smoothing with moving average (compass bearing)"""
#         if len(self.gps_history) < 2:
#             return self.gps_history[-1]
        
#         return self.circular_mean(list(self.gps_history)[-3:])
    
#     def get_last_stable_nav_att(self):
#         """Get last stable NAV-ATT heading (compass bearing)"""
#         if len(self.nav_att_history) >= 2:
#             recent_headings = [h[0] for h in list(self.nav_att_history)[-2:]]
#             return self.circular_mean(recent_headings)
#         return None
    
#     def get_last_stable_gps(self):
#         """Get last stable GPS heading (compass bearing)"""
#         if len(self.gps_history) >= 1:
#             return self.gps_history[-1]
#         return None
    
#     def angle_difference(self, angle1, angle2):
#         """Calculate shortest angular difference (compass bearing)"""
#         diff = angle1 - angle2
#         while diff > 180:
#             diff -= 360
#         while diff < -180:
#             diff += 360
#         return diff
    
#     def circular_mean(self, angles):
#         """Calculate circular mean of angles (compass bearing)"""
#         if not angles:
#             return 0
        
#         # 🔧 ใช้ compass bearing: sin สำหรับ East, cos สำหรับ North
#         x = np.mean([math.sin(math.radians(a)) for a in angles])  # East
#         y = np.mean([math.cos(math.radians(a)) for a in angles])  # North
        
#         # 🔧 ใช้ atan2(East, North) สำหรับ compass bearing
#         result = math.degrees(math.atan2(x, y))
#         if result < 0:
#             result += 360
#         return result
    
#     def circular_weighted_mean(self, angles, weights):
#         """Calculate weighted circular mean (compass bearing)"""
#         if not angles or len(angles) != len(weights):
#             return 0
        
#         # 🔧 ใช้ compass bearing: sin สำหรับ East, cos สำหรับ North
#         x = np.sum(weights * np.array([math.sin(math.radians(a)) for a in angles]))  # East
#         y = np.sum(weights * np.array([math.cos(math.radians(a)) for a in angles]))  # North
        
#         # 🔧 ใช้ atan2(East, North) สำหรับ compass bearing
#         result = math.degrees(math.atan2(x, y))
#         if result < 0:
#             result += 360
#         return result
    
#     def get_stats(self):
#         """Get enhanced filter statistics"""
#         return self.stats.copy()

# # ✅ Enhanced Waypoint Manager with Starting Point Selection
# class WaypointManager:
#     def __init__(self, csv_file_path, start_waypoint_index=0):
#         self.waypoints = []
#         self.current_waypoint_index = start_waypoint_index  # ✅ เริ่มจากจุดที่กำหนด
#         self.load_waypoints_from_csv(csv_file_path)
        
#         # กำหนดช่วง waypoint และประเภทเส้นทาง
#         self.path_segments = [
#             {'start': 0, 'end': 240, 'type': 'straight', 'direction': 'S-N', 'target_heading': 0.0},      # waypoint 1-240 (North = 0°)
#             {'start': 241, 'end': 256, 'type': 'curve', 'direction': 'curve1', 'target_heading': None},    # waypoint 245-260
#             {'start': 257, 'end': 315, 'type': 'straight', 'direction': 'E-W', 'target_heading': 90.0},   # waypoint 261-315 (East = 90°)
#             {'start': 316, 'end': 383, 'type': 'curve', 'direction': 'curve2', 'target_heading': None},    # waypoint 316-390
#             {'start': 384, 'end': 496, 'type': 'straight', 'direction': 'N-S', 'target_heading': 180.0},  # waypoint 391-500 (South = 180°)
#             {'start': 497, 'end': 571, 'type': 'curve', 'direction': 'curve3', 'target_heading': None},    # waypoint 501-579
#             {'start': 572, 'end': 622, 'type': 'straight', 'direction': 'W-E', 'target_heading': 270.0},  # waypoint 580-630 (West = 270°)
#             {'start': 623, 'end': 642, 'type': 'curve', 'direction': 'curve4', 'target_heading': None},    # waypoint 631-646
#             {'start': 643, 'end': 654, 'type': 'straight', 'direction': 'S-N', 'target_heading': 0.0}     # waypoint 647-659 (North = 0°)
#         ]
        
#         # ✅ กำหนดรัศมีสำหรับแต่ละทางโค้ง (หน่วย: เมตร)
#         self.curve_radii = {
#             'curve1': 15.0,  # รัศมี 15 เมตร สำหรับโค้งที่ 1
#             'curve2': 20.0,  # รัศมี 20 เมตร สำหรับโค้งที่ 2
#             'curve3': 18.0,  # รัศมี 18 เมตร สำหรับโค้งที่ 3
#             'curve4': 12.0   # รัศมี 12 เมตร สำหรับโค้งที่ 4
#         }
        
#         # ตรวจสอบว่าจุดเริ่มต้นอยู่ในช่วงที่ถูกต้อง
#         if self.current_waypoint_index >= len(self.waypoints):
#             self.current_waypoint_index = 0
#             print(f"⚠️ Starting waypoint index out of range, reset to 0")
        
#         print(f"✅ Starting navigation from waypoint {self.current_waypoint_index + 1}")
    
#     def load_waypoints_from_csv(self, csv_file_path):
#         """โหลด waypoints จากไฟล์ CSV"""
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
    
#     def set_starting_waypoint(self, waypoint_index):
#         """✅ กำหนดจุดเริ่มต้นใหม่"""
#         if 0 <= waypoint_index < len(self.waypoints):
#             self.current_waypoint_index = waypoint_index
#             print(f"✅ Starting waypoint set to {waypoint_index + 1}")
#             return True
#         else:
#             print(f"❌ Invalid waypoint index: {waypoint_index}")
#             return False
    
#     def get_current_segment_type(self):
#         """ได้ประเภทของเส้นทางปัจจุบัน"""
#         for segment in self.path_segments:
#             if segment['start'] <= self.current_waypoint_index <= segment['end']:
#                 return segment['type'], segment['direction']
#         return 'straight', 'unknown'
    
#     def get_curve_radius(self, curve_direction):
#         """✅ ได้รัศมีของทางโค้ง"""
#         return self.curve_radii.get(curve_direction, 15.0)  # default 15 เมตร
    
#     def get_current_waypoint(self):
#         """ได้ waypoint ปัจจุบัน"""
#         if self.current_waypoint_index < len(self.waypoints):
#             return self.waypoints[self.current_waypoint_index]
#         return None
    
#     def get_next_waypoint(self):
#         """ได้ waypoint ถัดไป"""
#         next_index = self.current_waypoint_index + 1
#         if next_index < len(self.waypoints):
#             return self.waypoints[next_index]
#         return None
    
#     def advance_waypoint(self):
#         """เลื่อนไปยัง waypoint ถัดไป"""
#         if self.current_waypoint_index < len(self.waypoints) - 1:
#             self.current_waypoint_index += 1
#             return True
#         return False
    
#     def get_progress_info(self):
#         """✅ ได้ข้อมูลความคืบหน้า"""
#         return {
#             'current': self.current_waypoint_index + 1,
#             'total': len(self.waypoints),
#             'progress_percent': ((self.current_waypoint_index + 1) / len(self.waypoints)) * 100
#         }

# # Pure Pursuit Controller (ปรับปรุงให้ใช้ compass bearing)
# class PurePursuitController:
#     def __init__(self, lookahead_distance=2.0):
#         self.lookahead_distance = lookahead_distance
    
#     def calculate_steering_angle(self, current_pos, current_heading, target_pos, wheelbase):
#         """คำนวณมุมเลี้ยวสำหรับ Pure Pursuit (compass bearing)"""
#         # คำนวณระยะทางและมุมไปยังเป้าหมาย
#         dx = target_pos['x'] - current_pos['x']  # East
#         dy = target_pos['y'] - current_pos['y']  # North
        
#         # 🔧 ใช้ compass bearing: atan2(East, North)
#         target_heading = math.degrees(math.atan2(dx, dy))
#         if target_heading < 0:
#             target_heading += 360
        
#         heading_error = self.normalize_angle(target_heading - current_heading)
        
#         distance = math.sqrt(dx**2 + dy**2)
        
#         # Pure Pursuit formula
#         alpha = math.radians(heading_error)
#         steering_angle = math.atan2(2 * wheelbase * math.sin(alpha), distance)
        
#         return steering_angle
    
#     def normalize_angle(self, angle):
#         """Normalize angle to [-180, 180] (compass bearing)"""
#         while angle > 180:
#             angle -= 360
#         while angle < -180:
#             angle += 360
#         return angle

# # ✅ Enhanced Constant Curvature Controller with Radius Control
# class ConstantCurvatureController:
#     def __init__(self):
#         self.curvature_history = deque(maxlen=5)
    
#     def calculate_steering_angle_with_radius(self, curve_radius, linear_velocity, wheelbase):
#         """✅ คำนวณมุมเลี้ยวจากรัศมีที่กำหนด"""
#         # คำนวณ curvature จากรัศมี: κ = 1/R
#         curvature = 1.0 / curve_radius
        
#         # คำนวณมุมเลี้ยวจาก curvature และ bicycle model
#         # δ = atan(L * κ) โดย L = wheelbase, κ = curvature
#         steering_angle = math.atan(wheelbase * curvature)
        
#         return steering_angle
    
#     def calculate_steering_angle(self, waypoints_segment, current_pos, wheelbase):
#         """คำนวณมุมเลี้ยวสำหรับ Constant Curvature (วิธีเดิม)"""
#         if len(waypoints_segment) < 3:
#             return 0.0
        
#         # คำนวณ curvature จาก 3 จุด
#         curvature = self.calculate_curvature(waypoints_segment)
#         self.curvature_history.append(curvature)
        
#         # ใช้ค่าเฉลี่ยของ curvature
#         avg_curvature = np.mean(list(self.curvature_history))
        
#         # คำนวณมุมเลี้ยวจาก curvature
#         steering_angle = math.atan(wheelbase * avg_curvature)
        
#         return steering_angle
    
#     def calculate_curvature(self, points):
#         """คำนวณ curvature จาก 3 จุด"""
#         if len(points) < 3:
#             return 0.0
        
#         p1, p2, p3 = points[0], points[1], points[2]
        
#         # คำนวณ curvature โดยใช้สูตร
#         x1, y1 = p1['x'], p1['y']
#         x2, y2 = p2['x'], p2['y']
#         x3, y3 = p3['x'], p3['y']
        
#         # Area of triangle
#         area = 0.5 * abs((x2-x1)*(y3-y1) - (x3-x1)*(y2-y1))
        
#         # Side lengths
#         a = math.sqrt((x2-x3)**2 + (y2-y3)**2)
#         b = math.sqrt((x1-x3)**2 + (y1-y3)**2)
#         c = math.sqrt((x1-x2)**2 + (y1-y2)**2)
        
#         if a * b * c == 0:
#             return 0.0
        
#         # Curvature = 4 * Area / (a * b * c)
#         curvature = 4 * area / (a * b * c)
        
#         return curvature

# # Bicycle Kinematic Model
# class BicycleKinematicModel:
#     def __init__(self, wheelbase):
#         self.wheelbase = wheelbase
    
#     def calculate_angular_velocity(self, linear_velocity, steering_angle):
#         """คำนวณ angular velocity จาก bicycle model"""
#         if abs(steering_angle) < 1e-6:  # ป้องกันการหารด้วยศูนย์
#             return 0.0
        
#         angular_velocity = (linear_velocity * math.tan(steering_angle)) / self.wheelbase
#         return angular_velocity
    
#     def limit_steering_angle(self, steering_angle, max_steering_angle):
#         """จำกัดมุมเลี้ยวให้อยู่ในขอบเขต"""
#         return max(-max_steering_angle, min(max_steering_angle, steering_angle))

# # ✅ Main Enhanced GNSS Node with Fixed Speed Navigation and Compass Bearing
# class EnhancedZEDf9rGNSSWithNavigation(Node):
#     def __init__(self):
#         super().__init__('enhanced_zedf9r_gnss_with_navigation')
        
#         # ✅ Vehicle parameters
#         self.declare_vehicle_parameters()
        
#         # ROS2 Publishers
#         self.gnss_publisher = self.create_publisher(NavSatFix, '/navigation/gnss', 10)
#         self.xy_publisher = self.create_publisher(Float64MultiArray, 'Realtime_XY', 10)
#         self.raw_gps_publisher = self.create_publisher(Float64MultiArray, 'Raw_GPS', 10)
#         self.xy_debug_publisher = self.create_publisher(Point, '/navigation/xy_debug', 10)
#         self.heading_publisher = self.create_publisher(Float32, '/navigation/heading', 10)
#         self.heading_debug_publisher = self.create_publisher(String, '/navigation/heading_debug', 10)
#         self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.navigation_debug_publisher = self.create_publisher(String, '/navigation/debug', 10)
        
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
        
#         # 🔧 Enhanced Heading management (compass bearing)
#         self.imu_heading = None
#         self.nav_att_heading = None  # compass bearing (degrees)
#         self.nav_att_yaw = None
#         self.gps_heading = None      # compass bearing (degrees)
#         self.final_heading = None    # compass bearing (degrees)
#         self.heading_source = "NONE"
        
#         self.imu_heading_time = 0
#         self.nav_att_time = 0
#         self.gps_heading_time = 0
#         self.heading_timeout = 8.0
        
#         # ZED-F9R specific data
#         self.esf_status = None
#         self.nav_att = None
#         self.esf_alg = None
#         self.fusion_mode = "NONE"
#         self.imu_calibration_status = "UNKNOWN"
        
#         # pyubx2 UBX parsing
#         self.ubx_reader = None
#         self.ubx_buffer = bytearray()
        
#         # 🔧 Enhanced Heading filtering with compass bearing support
#         self.heading_filter = EnhancedIMUHeadingFilter()
        
#         # ✅ Navigation Components with Enhanced Features
#         csv_file_path = '/home/inc/ros2_ws/waypointfile/waypoints3xy.csv'
#         start_waypoint = 0
        
#         self.waypoint_manager = WaypointManager(csv_file_path, start_waypoint)
#         self.pure_pursuit = PurePursuitController(lookahead_distance=3.0)
#         self.curvature_controller = ConstantCurvatureController()
#         self.bicycle_model = BicycleKinematicModel(wheelbase=1.67)
        
#         # Navigation state
#         self.navigation_active = True
#         self.waypoint_reached_threshold = 1.0  # เมตร
        
#         # ✅ Fixed Speed Configuration
#         self.FIXED_SPEED = 1.0  # ความเร็วคงที่ 1 m/s
        
#         # Performance stats
#         self.stats = {
#             'imu_count': 0,
#             'nav_att_count': 0,
#             'nav_att_yaw_count': 0,
#             'gps_count': 0,
#             'position_count': 0,
#             'esf_meas_count': 0,
#             'esf_status_count': 0,
#             'pyubx2_success': 0,
#             'pyubx2_errors': 0,
#             'filtered_count': 0,
#             'navigation_commands': 0
#         }
        
#         # Initialize coordinate system
#         self.init_track_coordinate_system()
        
#         # Initialize serial connection
#         self.init_serial_connection()
        
#         if self.ser and self.ser.is_open:
#             self.ntrip_socket = self.connect_to_ntrip()
            
#             # Initialize pyubx2 if available
#             if PYUBX2_AVAILABLE:
#                 self.init_pyubx2()
            
#             # Configure ZED-F9R for enhanced IMU
#             self.configure_enhanced_zed_f9r()
            
#             # Main processing timer
#             self.create_timer(0.1, self.main_loop)
            
#             self.get_logger().info('✅ Enhanced ZED-F9R GNSS with Fixed Speed Navigation and Compass Bearing initialized')
#             self.get_logger().info(f'🔧 pyubx2: {"Available" if PYUBX2_AVAILABLE else "Not available"}')
#             self.get_logger().info(f'🗺️ Loaded {len(self.waypoint_manager.waypoints)} waypoints')
#             self.get_logger().info(f'🚀 Fixed Speed: {self.FIXED_SPEED} m/s')
#             self.get_logger().info(f'🧭 Using Compass Bearing (0° = North, clockwise)')
            
#             # Log vehicle parameters
#             self.log_vehicle_parameters()
            
#             # Log curve radii
#             self.log_curve_radii()
#         else:
#             self.get_logger().error('❌ Failed to initialize')
    
#     def update_gps_heading_improved(self):
#         """🔧 Enhanced GPS heading calculation using UTM coordinates (compass bearing)"""
#         if len(self.position_history) < 3:
#             return
        
#         total_distance = 0.0
#         weighted_x = weighted_y = 0.0
        
#         recent_positions = list(self.position_history)[-5:]
        
#         for i in range(len(recent_positions) - 1):
#             curr = recent_positions[i+1]
#             prev = recent_positions[i]
            
#             # ใช้ UTM coordinates สำหรับการคำนวณ heading
#             dx = curr['x'] - prev['x']  # East
#             dy = curr['y'] - prev['y']  # North
#             distance = math.sqrt(dx*dx + dy*dy)
            
#             if distance > self.min_movement_for_heading:
#                 # 🔧 คำนวณ compass bearing: atan2(East, North)
#                 heading_rad = math.atan2(dx, dy)
                
#                 time_weight = (i + 1) / len(recent_positions)
#                 weight = distance * time_weight
                
#                 # 🔧 ใช้ compass bearing สำหรับ weighted calculation
#                 weighted_x += math.sin(heading_rad) * weight  # East component
#                 weighted_y += math.cos(heading_rad) * weight  # North component
#                 total_distance += weight
        
#         if total_distance > self.min_movement_for_heading:
#             # 🔧 คำนวณ compass bearing จาก weighted components
#             avg_heading_rad = math.atan2(weighted_x, weighted_y)
#             avg_heading_deg = math.degrees(avg_heading_rad)
            
#             # 🔧 แปลงให้เป็น compass bearing (0-360°)
#             if avg_heading_deg < 0:
#                 avg_heading_deg += 360
            
#             filtered_gps_heading = self.heading_filter.filter_gps_heading(avg_heading_deg)
            
#             if filtered_gps_heading is not None:
#                 self.gps_heading = filtered_gps_heading
#                 self.gps_heading_time = time.time()
#                 self.stats['gps_count'] += 1
    
#     def process_nav_att_message(self, nav_att_data):
#         """🔧 Process NAV-ATT message with compass bearing conversion"""
#         try:
#             # ดึงข้อมูล heading จาก NAV-ATT
#             heading_raw = nav_att_data.get('heading', 0)  # ในหน่วย 1e-5 degrees
            
#             # แปลงเป็นองศา
#             heading_deg = heading_raw / 1e5
            
#             # 🔧 ถ้า NAV-ATT ให้ mathematical angle, ต้องแปลงเป็น compass bearing
#             # สมมติว่า NAV-ATT ให้ mathematical angle (0° = East, counter-clockwise)
#             compass_bearing = (90 - heading_deg) % 360
            
#             # Filter และเก็บค่า
#             filtered_heading = self.heading_filter.filter_nav_att_heading(
#                 compass_bearing, 
#                 confidence_factor=1.0
#             )
            
#             if filtered_heading is not None:
#                 self.nav_att_heading = filtered_heading
#                 self.nav_att_time = time.time()
#                 self.stats['nav_att_count'] += 1
                
#         except Exception as e:
#             self.get_logger().debug(f'NAV-ATT processing error: {e}')
    
#     def update_enhanced_final_heading(self):
#         """🌟 Enhanced final heading selection with compass bearing consistency"""
#         current_time = time.time()
        
#         # ตรวจสอบความถูกต้องของข้อมูล
#         nav_att_valid = (self.nav_att_heading is not None and 
#                         (current_time - self.nav_att_time) < self.heading_timeout)
        
#         gps_valid = (self.gps_heading is not None and 
#                     (current_time - self.gps_heading_time) < self.heading_timeout)
        
#         # 🔧 ลำดับความสำคัญ: NAV-ATT > GPS > None
#         if nav_att_valid and self.imu_calibration_status in ["FULLY_CALIBRATED", "CALIBRATED"]:
#             self.final_heading = self.nav_att_heading  # compass bearing (degrees)
#             self.heading_source = f"NAV_ATT_{self.imu_calibration_status}"
            
#         elif nav_att_valid and self.imu_calibration_status == "CALIBRATING":
#             if gps_valid:
#                 # ผสมค่า heading แบบ circular
#                 self.final_heading = self.blend_headings_compass(
#                     self.nav_att_heading, self.gps_heading, 0.8
#                 )
#                 self.heading_source = "NAV_ATT_GPS_BLEND"
#             else:
#                 self.final_heading = self.nav_att_heading
#                 self.heading_source = "NAV_ATT_CALIBRATING"
                
#         elif gps_valid:
#             self.final_heading = self.gps_heading  # compass bearing (degrees)
#             self.heading_source = "GPS"
            
#         else:
#             self.final_heading = None
#             self.heading_source = "NONE"
        
#         # Publish heading
#         if self.final_heading is not None:
#             heading_msg = Float32()
#             heading_msg.data = float(self.final_heading)
#             self.heading_publisher.publish(heading_msg)
            
#             # Debug message
#             debug_msg = String()
#             debug_msg.data = (
#                 f"Heading: {self.final_heading:.1f}° ({self.heading_source}) | "
#                 f"NAV-ATT: {self.nav_att_heading:.1f}° | "
#                 f"GPS: {self.gps_heading:.1f}° | "
#                 f"Fusion: {self.fusion_mode}"
#             )
#             self.heading_debug_publisher.publish(debug_msg)
    
#     def blend_headings_compass(self, heading1, heading2, weight1):
#         """🔧 Blend two compass bearings considering circular nature"""
#         if heading1 is None or heading2 is None:
#             return heading1 or heading2
        
#         # แปลงเป็น radians สำหรับการคำนวณ circular
#         h1_rad = math.radians(heading1)
#         h2_rad = math.radians(heading2)
        
#         # แปลงเป็น unit vectors สำหรับ compass bearing
#         x1, y1 = math.sin(h1_rad), math.cos(h1_rad)  # East, North
#         x2, y2 = math.sin(h2_rad), math.cos(h2_rad)  # East, North
        
#         # ผสมค่า vectors
#         weight2 = 1.0 - weight1
#         x_blend = weight1 * x1 + weight2 * x2
#         y_blend = weight1 * y1 + weight2 * y2
        
#         # แปลงกลับเป็น compass bearing
#         blended_heading_rad = math.atan2(x_blend, y_blend)
#         blended_heading_deg = math.degrees(blended_heading_rad)
        
#         # ปรับให้เป็นค่าบวก (0-360°)
#         if blended_heading_deg < 0:
#             blended_heading_deg += 360
            
#         return blended_heading_deg
    
#     def calculate_target_heading_compass(self, current_pos, target_pos):
#         """🔧 คำนวณ target heading แบบ compass bearing"""
#         dx = target_pos['x'] - current_pos['x']  # East
#         dy = target_pos['y'] - current_pos['y']  # North
        
#         # คำนวณ compass bearing
#         bearing_rad = math.atan2(dx, dy)  # arctan2(East, North)
#         bearing_deg = math.degrees(bearing_rad)
        
#         # ปรับให้เป็น 0-360°
#         if bearing_deg < 0:
#             bearing_deg += 360
            
#         return bearing_deg
    
#     # เพิ่มฟังก์ชันที่จำเป็นอื่นๆ ตามโค้ดเดิม...
#     def declare_vehicle_parameters(self):
#         """✅ ประกาศ Vehicle Parameters"""
        
#         # Wheelbase parameter
#         wheelbase_descriptor = ParameterDescriptor(
#             description='ฐานล้อของรถ (ระยะห่างระหว่างเพลาหน้าและหลัง) ในหน่วยเมตร'
#         )
#         self.declare_parameter('wheelbase', 1.67, wheelbase_descriptor)
        
#         # Max steering angle parameter
#         steering_descriptor = ParameterDescriptor(
#             description='มุมเลี้ยวสูงสุดของรถในหน่วย radians (50 degrees = 0.873 rad)'
#         )
#         self.declare_parameter('max_steering_angle', 0.873, steering_descriptor)
        
#         # ✅ Fixed speed parameter - ตั้งค่าเป็น 1.0 m/s
#         speed_descriptor = ParameterDescriptor(
#             description='ความเร็วคงที่ของรถในหน่วย m/s (ตั้งค่าเป็น 1.0 m/s)'
#         )
#         self.declare_parameter('fixed_speed', 1.0, speed_descriptor)
        
#         # ✅ Starting waypoint parameter
#         start_waypoint_descriptor = ParameterDescriptor(
#             description='จุดเริ่มต้นของการนำทาง (waypoint index, เริ่มจาก 0)'
#         )
#         self.declare_parameter('start_waypoint_index', 0, start_waypoint_descriptor)
        
#         # ✅ Curve radius parameters
#         curve1_radius_descriptor = ParameterDescriptor(
#             description='รัศมีของทางโค้งที่ 1 ในหน่วยเมตร'
#         )
#         self.declare_parameter('curve1_radius', 15.0, curve1_radius_descriptor)
        
#         curve2_radius_descriptor = ParameterDescriptor(
#             description='รัศมีของทางโค้งที่ 2 ในหน่วยเมตร'
#         )
#         self.declare_parameter('curve2_radius', 20.0, curve2_radius_descriptor)
        
#         curve3_radius_descriptor = ParameterDescriptor(
#             description='รัศมีของทางโค้งที่ 3 ในหน่วยเมตร'
#         )
#         self.declare_parameter('curve3_radius', 18.0, curve3_radius_descriptor)
        
#         curve4_radius_descriptor = ParameterDescriptor(
#             description='รัศมีของทางโค้งที่ 4 ในหน่วยเมตร'
#         )
#         self.declare_parameter('curve4_radius', 12.0, curve4_radius_descriptor)
        
#         # Angular velocity limits
#         max_angular_vel_descriptor = ParameterDescriptor(
#             description='ความเร็วเชิงมุมสูงสุดในหน่วย rad/s'
#         )
#         self.declare_parameter('max_angular_velocity', 1.0, max_angular_vel_descriptor)
        
#         self.get_logger().info('🌟 Vehicle parameters declared successfully!')
    
#     def get_vehicle_parameters(self):
#         """✅ ดึงค่า Vehicle Parameters ปัจจุบัน"""
#         try:
#             wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
#             max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value
#             fixed_speed = self.get_parameter('fixed_speed').get_parameter_value().double_value
#             start_waypoint_index = self.get_parameter('start_waypoint_index').get_parameter_value().integer_value
#             max_angular_velocity = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
            
#             # ✅ ดึงค่ารัศมีของทางโค้ง
#             curve_radii = {
#                 'curve1': self.get_parameter('curve1_radius').get_parameter_value().double_value,
#                 'curve2': self.get_parameter('curve2_radius').get_parameter_value().double_value,
#                 'curve3': self.get_parameter('curve3_radius').get_parameter_value().double_value,
#                 'curve4': self.get_parameter('curve4_radius').get_parameter_value().double_value
#             }
            
#             return {
#                 'wheelbase': wheelbase,
#                 'max_steering_angle': max_steering_angle,
#                 'fixed_speed': fixed_speed,
#                 'start_waypoint_index': start_waypoint_index,
#                 'max_angular_velocity': max_angular_velocity,
#                 'curve_radii': curve_radii
#             }
#         except Exception as e:
#             self.get_logger().error(f'❌ Error getting vehicle parameters: {e}')
#             return {
#                 'wheelbase': 1.67,
#                 'max_steering_angle': 0.873,
#                 'fixed_speed': 1.0,
#                 'start_waypoint_index': 0,
#                 'max_angular_velocity': 1.0,
#                 'curve_radii': {
#                     'curve1': 15.0,
#                     'curve2': 20.0,
#                     'curve3': 18.0,
#                     'curve4': 12.0
#                 }
#             }
    
#     def log_vehicle_parameters(self):
#         """✅ แสดง Vehicle Parameters ปัจจุบัน"""
#         params = self.get_vehicle_parameters()
#         self.get_logger().info(
#             f'🚗 Vehicle Parameters:\n'
#             f'  🔧 Wheelbase: {params["wheelbase"]:.2f} m\n'
#             f'  🔧 Max Steering Angle: {params["max_steering_angle"]:.3f} rad ({math.degrees(params["max_steering_angle"]):.1f}°)\n'
#             f'  🔧 Fixed Speed: {params["fixed_speed"]:.2f} m/s\n'
#             f'  🔧 Start Waypoint: {params["start_waypoint_index"] + 1}\n'
#             f'  🔧 Max Angular Velocity: {params["max_angular_velocity"]:.2f} rad/s'
#         )
    
#     def log_curve_radii(self):
#         """✅ แสดงรัศมีของทางโค้ง"""
#         params = self.get_vehicle_parameters()
#         curve_radii = params['curve_radii']
        
#         self.get_logger().info(
#             f'🌀 Curve Radii Configuration:\n'
#             f'  🔄 Curve 1 (Waypoint 245-260): {curve_radii["curve1"]:.1f} m\n'
#             f'  🔄 Curve 2 (Waypoint 316-390): {curve_radii["curve2"]:.1f} m\n'
#             f'  🔄 Curve 3 (Waypoint 501-579): {curve_radii["curve3"]:.1f} m\n'
#             f'  🔄 Curve 4 (Waypoint 631-646): {curve_radii["curve4"]:.1f} m'
#         )
    
#     def navigation_control(self):
#         """✅ ระบบควบคุมการนำทางด้วยความเร็วคงที่และรัศมีที่กำหนด (compass bearing)"""
#         if not self.navigation_active or self.final_heading is None:
#             return
        
#         # ได้ตำแหน่งปัจจุบัน
#         current_waypoint = self.waypoint_manager.get_current_waypoint()
#         if current_waypoint is None:
#             self.get_logger().info('🏁 Navigation completed - reached final waypoint')
#             self.navigation_active = False
#             self.send_stop_command()
#             return
        
#         # ตรวจสอบว่าถึง waypoint แล้วหรือยัง
#         distance_to_waypoint = math.sqrt(
#             (self.current_position['x'] - current_waypoint['x'])**2 + 
#             (self.current_position['y'] - current_waypoint['y'])**2
#         )
        
#         if distance_to_waypoint < self.waypoint_reached_threshold:
#             if self.waypoint_manager.advance_waypoint():
#                 progress = self.waypoint_manager.get_progress_info()
#                 self.get_logger().info(
#                     f'✅ Reached waypoint {progress["current"]-1} '
#                     f'({progress["progress_percent"]:.1f}% complete)'
#                 )
#             else:
#                 self.get_logger().info('🏁 Navigation completed')
#                 self.navigation_active = False
#                 self.send_stop_command()
#                 return
        
#         # ได้ประเภทเส้นทางปัจจุบัน
#         segment_type, direction = self.waypoint_manager.get_current_segment_type()
        
#         # ได้ parameters
#         params = self.get_vehicle_parameters()
        
#         # ✅ ใช้ความเร็วคงที่ 1 m/s
#         linear_velocity = self.FIXED_SPEED
        
#         # คำนวณคำสั่งควบคุมตามประเภทเส้นทาง
#         if segment_type == 'straight':
#             steering_angle = self.calculate_straight_path_control(current_waypoint, params)
#         else:  # curve
#             steering_angle = self.calculate_curve_path_control_with_radius(direction, params)
        
#         # จำกัดมุมเลี้ยว
#         steering_angle = self.bicycle_model.limit_steering_angle(
#             steering_angle, params['max_steering_angle']
#         )
        
#         # คำนวณ angular velocity ด้วย bicycle model
#         angular_velocity = self.bicycle_model.calculate_angular_velocity(
#             linear_velocity, steering_angle
#         )
        
#         # จำกัด angular velocity
#         angular_velocity = max(-params['max_angular_velocity'], 
#                              min(params['max_angular_velocity'], angular_velocity))
        
#         # สร้างและส่งคำสั่ง cmd_vel
#         cmd_vel = Twist()
#         cmd_vel.linear.x = linear_velocity
#         cmd_vel.angular.z = angular_velocity
        
#         self.cmd_vel_publisher.publish(cmd_vel)
#         self.stats['navigation_commands'] += 1
        
#         # Debug information
#         debug_msg = String()
#         progress = self.waypoint_manager.get_progress_info()
        
#         if segment_type == 'curve':
#             curve_radius = params['curve_radii'].get(direction, 15.0)
#             debug_msg.data = (
#                 f"Segment: {segment_type} ({direction}, R={curve_radius:.1f}m) | "
#                 f"Waypoint: {progress['current']}/{progress['total']} ({progress['progress_percent']:.1f}%) | "
#                 f"Distance: {distance_to_waypoint:.2f}m | "
#                 f"Steering: {math.degrees(steering_angle):.1f}° | "
#                 f"Speed: {linear_velocity:.2f}m/s | "
#                 f"Angular: {angular_velocity:.2f}rad/s | "
#                 f"Heading: {self.final_heading:.1f}° (compass bearing)"
#             )
#         else:
#             debug_msg.data = (
#                 f"Segment: {segment_type} ({direction}) | "
#                 f"Waypoint: {progress['current']}/{progress['total']} ({progress['progress_percent']:.1f}%) | "
#                 f"Distance: {distance_to_waypoint:.2f}m | "
#                 f"Steering: {math.degrees(steering_angle):.1f}° | "
#                 f"Speed: {linear_velocity:.2f}m/s | "
#                 f"Angular: {angular_velocity:.2f}rad/s | "
#                 f"Heading: {self.final_heading:.1f}° (compass bearing)"
#             )
        
#         self.navigation_debug_publisher.publish(debug_msg)
    
#     def calculate_straight_path_control(self, target_waypoint, params):
#         """คำนวณการควบคุมสำหรับเส้นทางตรงด้วย Pure Pursuit + Bicycle Model (compass bearing)"""
#         steering_angle = self.pure_pursuit.calculate_steering_angle(
#             self.current_position,
#             self.final_heading,  # compass bearing
#             target_waypoint,
#             params['wheelbase']
#         )
        
#         # ใช้ bicycle model เพื่อรักษาเส้นทางตรง
#         # ลดมุมเลี้ยวให้น้อยลงเพื่อให้เป็นเส้นตรงมากขึ้น
#         steering_angle *= 0.5  # ลดความแรงของการเลี้ยว
        
#         return steering_angle
    
#     def calculate_curve_path_control_with_radius(self, curve_direction, params):
#         """✅ คำนวณการควบคุมสำหรับเส้นทางโค้งด้วยรัศมีที่กำหนด"""
#         # ได้รัศมีของทางโค้งจาก parameters
#         curve_radius = params['curve_radii'].get(curve_direction, 15.0)
        
#         # คำนวณมุมเลี้ยวจากรัศมีที่กำหนด
#         steering_angle = self.curvature_controller.calculate_steering_angle_with_radius(
#             curve_radius,
#             self.FIXED_SPEED,
#             params['wheelbase']
#         )
        
#         return steering_angle
    
#     def send_stop_command(self):
#         """✅ ส่งคำสั่งหยุดรถ"""
#         cmd_vel = Twist()
#         cmd_vel.linear.x = 0.0
#         cmd_vel.angular.z = 0.0
#         self.cmd_vel_publisher.publish(cmd_vel)
#         self.get_logger().info('🛑 Stop command sent')
    
#     # เพิ่มฟังก์ชันที่จำเป็นอื่นๆ ตามโค้ดเดิม...
#     def init_track_coordinate_system(self):
#         """Initialize coordinate system"""
#         if self.reference_utm is None:
#             self.reference_utm = self.transformer.transform(
#                 self.reference_point[1], self.reference_point[0]
#             )
        
#         self.utm_offset_x = self.TARGET_UTM_X - self.reference_utm[0]
#         self.utm_offset_y = self.TARGET_UTM_Y - self.reference_utm[1]
        
#         self.get_logger().info(f'🗺️ Track coordinate system initialized')
#         self.get_logger().info(f'📍 Reference UTM: {self.reference_utm}')
#         self.get_logger().info(f'🎯 Target UTM: ({self.TARGET_UTM_X}, {self.TARGET_UTM_Y})')
#         self.get_logger().info(f'📐 Offsets: ({self.utm_offset_x:.2f}, {self.utm_offset_y:.2f})')
    
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
        
#         if not self.ser or not self.ser.is_open:
#             self.get_logger().error('❌ No serial connection available')
    
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
#         import base64
#         credentials = f"{self.ntrip_username}:{self.ntrip_password}"
#         return base64.b64encode(credentials.encode()).decode()
    
#     def init_pyubx2(self):
#         """Initialize pyubx2 if available"""
#         if PYUBX2_AVAILABLE:
#             self.get_logger().info('✅ pyubx2 initialized for enhanced UBX parsing')
    
#     def configure_enhanced_zed_f9r(self):
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
            
#             # อัพเดทตำแหน่งปัจจุบัน
#             self.update_current_position()
            
#             # อัพเดท final heading
#             self.update_enhanced_final_heading()
            
#             # ควบคุมการนำทาง
#             self.navigation_control()
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Main loop error: {e}')
    
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
#         # ประมวลผลข้อมูล NMEA และ UBX
#         # (ใช้โค้ดเดิมที่มีอยู่แล้วในการประมวลผล GNSS data)
#         pass
    
#     def update_current_position(self):
#         """Update current position from GNSS data"""
#         # อัพเดทตำแหน่งปัจจุบันจากข้อมูล GNSS
#         # (ใช้โค้ดเดิมที่มีอยู่แล้วในการอัพเดทตำแหน่ง)
#         pass

# def main(args=None):
#     rclpy.init(args=args)
    
#     try:
#         node = EnhancedZEDf9rGNSSWithNavigation()
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         print('\n🛑 Navigation stopped by user')
#     except Exception as e:
#         print(f'❌ Error: {e}')
#     finally:
#         if 'node' in locals():
#             node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



#         def process_position(self, latitude, longitude):
#         """Process GNSS position with UTM coordinates"""
#         current_time = time.time()
        
#         if not (-90 <= latitude <= 90) or not (-180 <= longitude <= 180):
#             return
        
#         # แปลงเป็น UTM coordinates สัมบูรณ์
#         utm_x, utm_y = self.transformer.transform(longitude, latitude)
        
#         # 🌟 เก็บ UTM coordinates สัมบูรณ์สำหรับ controller
#         self.current_utm_x = utm_x
#         self.current_utm_y = utm_y
        
#         # คำนวณ local coordinates สำหรับการแสดงผล
#         if self.reference_utm is not None:
#             local_x = utm_x - self.reference_utm[0]
#             local_y = utm_y - self.reference_utm[1]
#         else:
#             local_x = utm_x
#             local_y = utm_y
        
#         # Store position for heading calculation
#         position_record = {
#             'lat': latitude, 'lon': longitude,
#             'utm_x': utm_x, 'utm_y': utm_y,
#             'local_x': local_x, 'local_y': local_y,
#             'timestamp': current_time
#         }
#         self.position_history.append(position_record)
#         self.stats['position_count'] += 1
        
#         # Calculate GPS heading
#         self.update_gps_heading_improved()
        
#         # Publish data
#         self.publish_data(latitude, longitude, local_x, local_y)
    
#     def update_gps_heading_improved(self):
#         """🔧 Enhanced GPS heading calculation using UTM coordinates (compass bearing)"""
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
#                 # 🔧 คำนวณ compass bearing
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
#         """🌟 Enhanced final heading selection with NAV-ATT priority (compass bearing)"""
#         current_time = time.time()
        
#         # Check data validity
#         nav_att_valid = (self.nav_att_heading is not None and 
#                         (current_time - self.nav_att_time) < self.heading_timeout and
#                         self.fusion_mode in ["FUSION", "INIT"])
        
#         gps_valid = (self.gps_heading is not None and 
#                     (current_time - self.gps_heading_time) < self.heading_timeout)
        
#         # 🌟 Enhanced priority: NAV-ATT > GPS > Hold
#         if nav_att_valid and self.imu_calibration_status in ["FULLY_CALIBRATED", "CALIBRATED"]:
#             self.final_heading = self.nav_att_heading  # compass bearing (degrees)
#             self.heading_source = f"NAV_ATT_{self.imu_calibration_status}"
            
#         elif nav_att_valid and self.imu_calibration_status == "CALIBRATING":
#             if gps_valid:
#                 self.final_heading = self.blend_headings(self.nav_att_heading, self.gps_heading, 0.8)
#                 self.heading_source = "NAV_ATT_GPS_BLEND"
#             else:
#                 self.final_heading = self.nav_att_heading
#                 self.heading_source = "NAV_ATT_CALIBRATING"
                
#         elif gps_valid:
#             self.final_heading = self.gps_heading  # compass bearing (degrees)
#             self.heading_source = "GPS"
            
#         else:
#             self.final_heading = None
#             self.heading_source = "NONE"
        
#         # Publish enhanced heading
#         if self.final_heading is not None:
#             heading_msg = Float32()
#             heading_msg.data = float(self.final_heading)
#             self.heading_publisher.publish(heading_msg)
#             self.stats['filtered_count'] += 1
    
#     def blend_headings(self, heading1, heading2, weight1):
#         """Blend two headings considering circular nature (compass bearing)"""
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
    
#     def publish_data(self, latitude, longitude, local_x, local_y):
#         """Publish all data"""
#         # GNSS
#         gnss_msg = NavSatFix()
#         gnss_msg.latitude = latitude
#         gnss_msg.longitude = longitude
#         gnss_msg.status.status = 0
#         gnss_msg.status.service = 1
#         self.gnss_publisher.publish(gnss_msg)
        
#         # XY position (local coordinates สำหรับการแสดงผล)
#         xy_msg = Point()
#         xy_msg.x = local_x
#         xy_msg.y = local_y
#         xy_msg.z = 0.0
#         self.xy_publisher.publish(xy_msg)
    
#     def handle_ntrip_data(self):
#         """Handle NTRIP corrections"""
#         if self.ntrip_socket:
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
    
#     def process_ubx_data_enhanced(self, data):
#         """🌟 Enhanced UBX processing with pyubx2"""
#         pass  # Implementation ตามต้องการ
    
#     def process_ubx_data_manual(self, data):
#         """Fallback manual UBX processing"""
#         pass  # Implementation ตามต้องการ
    
#     def destroy_node(self):
#         """Cleanup"""
#         self.get_logger().info("🔄 Shutting down Enhanced ZED-F9R with Controller...")
        
#         if self.ser and self.ser.is_open:
#             self.ser.close()
        
#         if hasattr(self, 'ntrip_socket') and self.ntrip_socket:
#             self.ntrip_socket.close()
        
#         super().destroy_node()

# class EnhancedIMUHeadingFilter:
#     def __init__(self):
#         self.nav_att_history = deque(maxlen=12)
#         self.gps_history = deque(maxlen=8)
        
#         self.nav_att_outlier_threshold = 20.0
#         self.gps_outlier_threshold = 18.0
#         self.smoothing_alpha = 0.25
#         self.confidence_threshold = 0.8
        
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
#         return new_heading
    
#     def filter_gps_heading(self, new_gps_heading):
#         """Enhanced GPS heading filtering"""
#         return new_gps_heading
    
#     def get_stats(self):
#         """Get enhanced filter statistics"""
#         return self.stats.copy()

# def main(args=None):
#     rclpy.init(args=args)
#     gnss_publisher = EnhancedZEDf9rGNSSWithHeading()
    
#     try:
#         rclpy.spin(gnss_publisher)
#     except KeyboardInterrupt:
#         gnss_publisher.get_logger().info("🛑 Keyboard interrupt")
#     finally:
#         gnss_publisher.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()