# #!/usr/bin/env python3

# import serial
# import pynmea2
# import json
# import socket
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
# from std_msgs.msg import Float32

# class GNSSPublisher(Node):
#     def __init__(self):
#         super().__init__('gnss_publisher')
        
#         # Create ROS2 publishers
#         self.gnss_publisher = self.create_publisher(
#             NavSatFix,
#             '/navigation/gnss',
#             10)
        
#         # Serial port configuration
#         self.serial_port = '/dev/ttyACM0'  # Change to your GNSS module port
        
#         # NTRIP configuration
#         self.ntrip_server_ip = "110.78.0.54"
#         self.ntrip_server_port = 2116
#         self.ntrip_username = "1118600009224"
#         self.ntrip_password = "CK79"
#         self.mount_point = "VRS_RTCM32"
        
#         # Initialize serial port
#         try:
#             self.ser = serial.Serial(self.serial_port, 9600, timeout=1)
#             self.get_logger().info(f"Connected to {self.serial_port}")
#         except Exception as e:
#             self.get_logger().error(f"Error connecting to serial port {self.serial_port}: {e}")
#             return
        
#         # Connect to NTRIP server
#         self.ntrip_socket = self.connect_to_ntrip()
        
#         # Create a timer for reading GNSS data
#         self.create_timer(0.1, self.read_gnss_data)  # 10 Hz
        
#         self.get_logger().info('GNSS Publisher initialized')
    
#     def connect_to_ntrip(self):
#         try:
#             # Create socket connection to NTRIP Server
#             client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#             client_socket.connect((self.ntrip_server_ip, self.ntrip_server_port))
#             auth_str = f"GET /{self.mount_point} HTTP/1.1\r\nUser-Agent: NTRIP Client\r\nAuthorization: Basic {self.ntrip_username}:{self.ntrip_password}\r\n\r\n"
#             client_socket.send(auth_str.encode('ascii'))
            
#             self.get_logger().info("Connected to NTRIP server")
#             return client_socket
#         except Exception as e:
#             self.get_logger().error(f"Error connecting to NTRIP server: {e}")
#             return None
    
#     def read_gnss_data(self):
#         # Read data from serial port
#         if hasattr(self, 'ser') and self.ser.is_open:
#             try:
#                 data = self.ser.readline().decode('ascii', errors='replace')
                
#                 # Parse NMEA data
#                 if data.startswith('$GPGGA') or data.startswith('$GNGLL') or data.startswith('$GPRMC'):
#                     try:
#                         msg = pynmea2.parse(data)
                        
#                         latitude = None
#                         longitude = None
                        
#                         if isinstance(msg, pynmea2.types.talker.GGA):  # GPGGA
#                             latitude = msg.latitude
#                             longitude = msg.longitude
#                         elif isinstance(msg, pynmea2.types.talker.GLL):  # GNGLL
#                             latitude = msg.latitude
#                             longitude = msg.longitude
#                         elif isinstance(msg, pynmea2.types.talker.RMC):  # GPRMC
#                             latitude = msg.latitude
#                             longitude = msg.longitude
                        
#                         # Check if valid coordinates were received
#                         if latitude is not None and longitude is not None:
#                             # Publish to ROS2 topic
#                             nav_msg = NavSatFix()
#                             nav_msg.latitude = latitude
#                             nav_msg.longitude = longitude
#                             self.gnss_publisher.publish(nav_msg)
                            
#                             self.get_logger().info(f"Published GNSS data: lat={latitude}, lon={longitude}")
                            
#                             # Forward RTK corrections from NTRIP to GNSS device
#                             if self.ntrip_socket:
#                                 try:
#                                     # Read RTK data from NTRIP server (non-blocking)
#                                     self.ntrip_socket.setblocking(0)
#                                     try:
#                                         rtk_data = self.ntrip_socket.recv(1024)
#                                         if rtk_data:
#                                             # Send RTK correction to the GNSS device
#                                             self.ser.write(rtk_data)
#                                     except socket.error:
#                                         pass  # No data available
#                                 except Exception as e:
#                                     self.get_logger().error(f"Error handling NTRIP data: {e}")
                    
#                     except pynmea2.nmea.ChecksumError:
#                         self.get_logger().warning("Checksum error in NMEA data")
            
#             except Exception as e:
#                 self.get_logger().error(f"Error reading serial data: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     gnss_publisher = GNSSPublisher()
#     rclpy.spin(gnss_publisher)
    
#     # Cleanup
#     if hasattr(gnss_publisher, 'ser') and gnss_publisher.ser.is_open:
#         gnss_publisher.ser.close()
#     if hasattr(gnss_publisher, 'ntrip_socket') and gnss_publisher.ntrip_socket:
#         gnss_publisher.ntrip_socket.close()
    
#     gnss_publisher.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# --------------------------------run web----------------------------------------------


# !/usr/bin/env python3

import serial
import pynmea2
import json
import socket
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32

class GNSSPublisher(Node):
    def __init__(self):
        super().__init__('gnss_publisher')
        
        # Create ROS2 publishers
        self.gnss_publisher = self.create_publisher(
            NavSatFix,
            '/navigation/gnss',
            10)
        
        # Serial port configuration
        self.serial_port = '/dev/ttyACM0'  # Change to your GNSS module port
        
        # NTRIP configuration
        self.ntrip_server_ip = "110.78.0.54"
        self.ntrip_server_port = 2116
        self.ntrip_username = "1118600009224"
        self.ntrip_password = "CK79"
        self.mount_point = "VRS_RTCM32"
        
        # Initialize serial port
        try:
            self.ser = serial.Serial(self.serial_port, 9600, timeout=1)
            self.get_logger().info(f"Connected to {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"Error connecting to serial port {self.serial_port}: {e}")
            return
        
        # Connect to NTRIP server
        self.ntrip_socket = self.connect_to_ntrip()
        self.last_fix_quality = 0
        
        # Create a timer for reading GNSS data
        self.create_timer(0.1, self.read_gnss_data)  # 10 Hz
        
        self.get_logger().info('GNSS Publisher initialized')
    
    def connect_to_ntrip(self):
        try:
            # Create socket connection to NTRIP Server
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((self.ntrip_server_ip, self.ntrip_server_port))
            auth_str = f"GET /{self.mount_point} HTTP/1.1\r\nUser-Agent: NTRIP Client\r\nAuthorization: Basic {self.ntrip_username}:{self.ntrip_password}\r\n\r\n"
            client_socket.send(auth_str.encode('ascii'))
            
            self.get_logger().info("Connected to NTRIP server")
            return client_socket
        except Exception as e:
            self.get_logger().error(f"Error connecting to NTRIP server: {e}")
            return None
    
    def read_gnss_data(self):
        # Read data from serial port
        if hasattr(self, 'ser') and self.ser.is_open:
            try:
                data = self.ser.readline().decode('ascii', errors='replace')
                
                # Parse NMEA data
                if data.startswith('$GPGGA') or data.startswith('$GNGGA') or data.startswith('$GNGLL') or data.startswith('$GPGLL') or data.startswith('$GPRMC') or data.startswith('$GNRMC'):
                    try:
                        msg = pynmea2.parse(data)
                        
                        latitude = None
                        longitude = None
                        
                        if isinstance(msg, pynmea2.types.talker.GGA):  # GPGGA
                            latitude = msg.latitude
                            longitude = msg.longitude
                            try:
                                self.last_fix_quality = int(msg.gps_qual)
                            except (TypeError, ValueError):
                                self.last_fix_quality = 0
                        elif isinstance(msg, pynmea2.types.talker.GLL):  # GNGLL
                            latitude = msg.latitude
                            longitude = msg.longitude
                        elif isinstance(msg, pynmea2.types.talker.RMC):  # GPRMC
                            latitude = msg.latitude
                            longitude = msg.longitude
                        
                        # Check if valid coordinates were received
                        if latitude is not None and longitude is not None:
                            # Publish to ROS2 topic
                            nav_msg = NavSatFix()
                            nav_msg.latitude = latitude
                            nav_msg.longitude = longitude
                            nav_msg.status.service = NavSatStatus.SERVICE_GPS
                            if self.last_fix_quality == 4:
                                nav_msg.status.status = NavSatStatus.STATUS_GBAS_FIX
                            elif self.last_fix_quality > 0:
                                nav_msg.status.status = NavSatStatus.STATUS_FIX
                            else:
                                nav_msg.status.status = NavSatStatus.STATUS_NO_FIX
                            nav_msg.position_covariance[0] = float(self.last_fix_quality)
                            nav_msg.position_covariance_type = (
                                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
                            )
                            self.gnss_publisher.publish(nav_msg)
                            
                            self.get_logger().info(f"Published GNSS data: lat={latitude}, lon={longitude}")
                            
                            # Forward RTK corrections from NTRIP to GNSS device
                            if self.ntrip_socket:
                                try:
                                    # Read RTK data from NTRIP server (non-blocking)
                                    self.ntrip_socket.setblocking(0)
                                    try:
                                        rtk_data = self.ntrip_socket.recv(1024)
                                        if rtk_data:
                                            # Send RTK correction to the GNSS device
                                            self.ser.write(rtk_data)
                                    except socket.error:
                                        pass  # No data available
                                except Exception as e:
                                    self.get_logger().error(f"Error handling NTRIP data: {e}")
                    
                    except pynmea2.nmea.ChecksumError:
                        self.get_logger().warning("Checksum error in NMEA data")
            
            except Exception as e:
                self.get_logger().error(f"Error reading serial data: {e}")

def main(args=None):
    rclpy.init(args=args)
    gnss_publisher = GNSSPublisher()
    rclpy.spin(gnss_publisher)
    
    # Cleanup
    if hasattr(gnss_publisher, 'ser') and gnss_publisher.ser.is_open:
        gnss_publisher.ser.close()
    if hasattr(gnss_publisher, 'ntrip_socket') and gnss_publisher.ntrip_socket:
        gnss_publisher.ntrip_socket.close()
    
    gnss_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




# -----------------------------------------fot plot xy------------------------------------------------
#!/usr/bin/env python3
# import serial
# import pynmea2
# import csv
# import math
# from pyproj import Transformer
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
# from std_msgs.msg import Float32

# class GNSSLogger(Node):
#     def __init__(self):
#         super().__init__('gnss_logger')

#         self.publisher_ = self.create_publisher(NavSatFix, '/navigation/gnss', 10)
#         self.heading_publisher = self.create_publisher(Float32, '/navigation/heading', 10)

#         # Serial config
#         self.serial_port = '/dev/ttyACM0'
#         self.ser = serial.Serial(self.serial_port, 9600, timeout=1)

#         # เปิดไฟล์ CSV
#         self.csv_file = open('gnss_log3.csv', mode='w', newline='')
#         self.csv_writer = csv.writer(self.csv_file)
#         self.csv_writer.writerow(['X (m)', 'Y (m)', 'Heading (deg)'])

#         # สร้าง transformer สำหรับแปลง lat/lon → UTM XY
#         self.transformer = Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)  # UTM zone 47N (ไทย)

#         # ตัวแปร heading
#         self.prev_x = None
#         self.prev_y = None

#         # timer call ทุก 0.1 วินาที
#         self.create_timer(0.1, self.read_data)

#         self.get_logger().info('GNSS Logger node started')

#     def read_data(self):
#         try:
#             data = self.ser.readline().decode('ascii', errors='replace')
#             print(f"RAW: {data}")

#             if data.startswith('$GNGLL'):
#                 msg = pynmea2.parse(data)
#                 lat = msg.latitude
#                 lon = msg.longitude

#                 if lat != 0.0 and lon != 0.0:
#                     x, y = self.transformer.transform(lon, lat)

#                     heading = 0.0
#                     if self.prev_x is not None and self.prev_y is not None:
#                         dx = x - self.prev_x
#                         dy = y - self.prev_y
#                         heading = math.degrees(math.atan2(dy, dx))
#                         if heading < 0:
#                             heading += 360.0

#                     self.prev_x = x
#                     self.prev_y = y

#                     # บันทึกลง csv
#                     self.csv_writer.writerow([x, y, heading])

#                     # publish GNSS
#                     gnss_msg = NavSatFix()
#                     gnss_msg.latitude = lat
#                     gnss_msg.longitude = lon
#                     self.publisher_.publish(gnss_msg)

#                     # publish heading
#                     heading_msg = Float32()
#                     heading_msg.data = heading
#                     self.heading_publisher.publish(heading_msg)

#                     self.get_logger().info(f"Lat: {lat}, Lon: {lon}, X: {x:.2f}, Y: {y:.2f}, Heading: {heading:.2f}")

#         except Exception as e:
#             self.get_logger().error(f"Error: {e}")

#     def destroy_node(self):
#         if self.ser.is_open:
#             self.ser.close()
#         self.csv_file.close()
#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     node = GNSSLogger()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()





# import serial
# import pynmea2
# import json
# import socket
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
# from std_msgs.msg import Float32

# class GNSSPublisher(Node):
#     def __init__(self):
#         super().__init__('gnss_publisher')
        
#         # Create ROS2 publishers
#         self.gnss_publisher = self.create_publisher(
#             NavSatFix,
#             '/navigation/gnss',
#             10)
        
#         # Publisher สำหรับ heading
#         self.heading_publisher = self.create_publisher(
#             Float32,
#             '/navigation/heading',
#             10)
        
#         # Serial port configuration
#         self.serial_port = '/dev/ttyACM0'  # Change to your GNSS module port
        
#         # NTRIP configuration
#         self.ntrip_server_ip = "110.78.0.54"
#         self.ntrip_server_port = 2116
#         self.ntrip_username = "1118600009224"
#         self.ntrip_password = "CK79"
#         self.mount_point = "VRS_RTCM32"
        
#         # Initialize serial port
#         try:
#             self.ser = serial.Serial(self.serial_port, 9600, timeout=1)
#             self.get_logger().info(f"Connected to {self.serial_port}")
#         except Exception as e:
#             self.get_logger().error(f"Error connecting to serial port {self.serial_port}: {e}")
#             return
        
#         # Connect to NTRIP server
#         self.ntrip_socket = self.connect_to_ntrip()
        
#         # Create a timer for reading GNSS data
#         self.create_timer(0.1, self.read_gnss_data)  # 10 Hz
        
#         self.get_logger().info('GNSS Publisher initialized')
    
#     def connect_to_ntrip(self):
#         try:
#             # Create socket connection to NTRIP Server
#             client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#             client_socket.connect((self.ntrip_server_ip, self.ntrip_server_port))
#             auth_str = f"GET /{self.mount_point} HTTP/1.1\r\nUser-Agent: NTRIP Client\r\nAuthorization: Basic {self.ntrip_username}:{self.ntrip_password}\r\n\r\n"
#             client_socket.send(auth_str.encode('ascii'))
            
#             self.get_logger().info("Connected to NTRIP server")
#             return client_socket
#         except Exception as e:
#             self.get_logger().error(f"Error connecting to NTRIP server: {e}")
#             return None
    
#     def read_gnss_data(self):
#         # Read data from serial port
#         if hasattr(self, 'ser') and self.ser.is_open:
#             try:
#                 data = self.ser.readline().decode('ascii', errors='replace')
                
#                 # Parse NMEA data
#                 if data.startswith('$GPGGA') or data.startswith('$GNGLL') or data.startswith('$GPRMC') or data.startswith('$GPHDT'):
#                     try:
#                         msg = pynmea2.parse(data)
                        
#                         latitude = None
#                         longitude = None
#                         heading = None
                        
#                         if isinstance(msg, pynmea2.types.talker.GGA):  # GPGGA
#                             latitude = msg.latitude
#                             longitude = msg.longitude
#                         elif isinstance(msg, pynmea2.types.talker.GLL):  # GNGLL
#                             latitude = msg.latitude
#                             longitude = msg.longitude
#                         elif isinstance(msg, pynmea2.types.talker.RMC):  # GPRMC
#                             latitude = msg.latitude
#                             longitude = msg.longitude
#                             # Heading ใน GPRMC บางทีจะอยู่ใน msg.true_course
#                             if hasattr(msg, 'true_course'):
#                                 heading = float(msg.true_course)
#                         elif isinstance(msg, pynmea2.types.talker.HDT):  # GPHDT คือ heading (Heading True)
#                             heading = float(msg.heading)
                        
#                         # Publish GNSS coordinates ถ้ามี
#                         if latitude is not None and longitude is not None:
#                             nav_msg = NavSatFix()
#                             nav_msg.latitude = latitude
#                             nav_msg.longitude = longitude
#                             self.gnss_publisher.publish(nav_msg)
#                             self.get_logger().info(f"Published GNSS data: lat={latitude}, lon={longitude}")
                        
#                         # Publish heading ถ้ามี
#                         if heading is not None:
#                             heading_msg = Float32()
#                             heading_msg.data = heading
#                             self.heading_publisher.publish(heading_msg)
#                             self.get_logger().info(f"Published heading: {heading}")
                            
#                         # Forward RTK corrections from NTRIP to GNSS device
#                         if self.ntrip_socket:
#                             try:
#                                 # Read RTK data from NTRIP server (non-blocking)
#                                 self.ntrip_socket.setblocking(0)
#                                 try:
#                                     rtk_data = self.ntrip_socket.recv(1024)
#                                     if rtk_data:
#                                         # Send RTK correction to the GNSS device
#                                         self.ser.write(rtk_data)
#                                 except socket.error:
#                                     pass  # No data available
#                             except Exception as e:
#                                 self.get_logger().error(f"Error handling NTRIP data: {e}")
                    
#                     except pynmea2.nmea.ChecksumError:
#                         self.get_logger().warning("Checksum error in NMEA data")
            
#             except Exception as e:
#                 self.get_logger().error(f"Error reading serial data: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     gnss_publisher = GNSSPublisher()
#     rclpy.spin(gnss_publisher)
    
#     # Cleanup
#     if hasattr(gnss_publisher, 'ser') and gnss_publisher.ser.is_open:
#         gnss_publisher.ser.close()
#     if hasattr(gnss_publisher, 'ntrip_socket') and gnss_publisher.ntrip_socket:
#         gnss_publisher.ntrip_socket.close()
    
#     gnss_publisher.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()




# import serial
# import pynmea2
# import json
# import socket
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
# from std_msgs.msg import Float32
# import math

# def calculate_bearing(lat1, lon1, lat2, lon2):
#     """
#     คำนวณ bearing ระหว่างสองจุดในหน่วยองศา (0-360)
#     0 = เหนือ, 90 = ตะวันออก
#     """
#     lat1_rad = math.radians(lat1)
#     lat2_rad = math.radians(lat2)
#     diff_long = math.radians(lon2 - lon1)

#     x = math.sin(diff_long) * math.cos(lat2_rad)
#     y = math.cos(lat1_rad) * math.sin(lat2_rad) - \
#         math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(diff_long)

#     initial_bearing = math.atan2(x, y)
#     initial_bearing_deg = (math.degrees(initial_bearing) + 360) % 360
#     return initial_bearing_deg

# class GNSSPublisher(Node):
#     def __init__(self):
#         super().__init__('gnss_publisher')
        
#         self.gnss_publisher = self.create_publisher(
#             NavSatFix,
#             '/navigation/gnss',
#             10)
        
#         self.heading_publisher = self.create_publisher(
#             Float32,
#             '/navigation/heading',
#             10)
        
#         self.serial_port = '/dev/ttyACM0'  # ปรับตามพอร์ตของ GNSS
        
#         self.ntrip_server_ip = "110.78.0.54"
#         self.ntrip_server_port = 2116
#         self.ntrip_username = "1118600009224"
#         self.ntrip_password = "CK79"
#         self.mount_point = "VRS_RTCM32"
        
#         # เก็บตำแหน่งก่อนหน้าไว้คำนวณ heading
#         self.prev_lat = None
#         self.prev_lon = None
        
#         try:
#             self.ser = serial.Serial(self.serial_port, 9600, timeout=1)
#             self.get_logger().info(f"Connected to {self.serial_port}")
#         except Exception as e:
#             self.get_logger().error(f"Error connecting to serial port {self.serial_port}: {e}")
#             return
        
#         self.ntrip_socket = self.connect_to_ntrip()
        
#         self.create_timer(0.1, self.read_gnss_data)  # 10 Hz
        
#         self.get_logger().info('GNSS Publisher initialized')
    
#     def connect_to_ntrip(self):
#         try:
#             client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#             client_socket.connect((self.ntrip_server_ip, self.ntrip_server_port))
            
#             import base64
#             auth_str = f"{self.ntrip_username}:{self.ntrip_password}"
#             auth_bytes = auth_str.encode('ascii')
#             auth_b64 = base64.b64encode(auth_bytes).decode('ascii')
            
#             request = (
#                 f"GET /{self.mount_point} HTTP/1.1\r\n"
#                 f"User-Agent: NTRIP Client\r\n"
#                 f"Authorization: Basic {auth_b64}\r\n"
#                 f"\r\n"
#             )
#             client_socket.send(request.encode('ascii'))
#             self.get_logger().info("Connected to NTRIP server")
#             return client_socket
#         except Exception as e:
#             self.get_logger().error(f"Error connecting to NTRIP server: {e}")
#             return None
    
#     def read_gnss_data(self):
#         if hasattr(self, 'ser') and self.ser.is_open:
#             try:
#                 data = self.ser.readline().decode('ascii', errors='replace')
                
#                 if data.startswith('$GPGGA') or data.startswith('$GNGLL') or data.startswith('$GPRMC') or data.startswith('$GPHDT'):
#                     try:
#                         msg = pynmea2.parse(data)
                        
#                         latitude = None
#                         longitude = None
#                         heading = None
                        
#                         if isinstance(msg, pynmea2.types.talker.GGA):
#                             latitude = msg.latitude
#                             longitude = msg.longitude
#                         elif isinstance(msg, pynmea2.types.talker.GLL):
#                             latitude = msg.latitude
#                             longitude = msg.longitude
#                         elif isinstance(msg, pynmea2.types.talker.RMC):
#                             latitude = msg.latitude
#                             longitude = msg.longitude
#                             if hasattr(msg, 'true_course'):
#                                 heading = float(msg.true_course)
#                         elif isinstance(msg, pynmea2.types.talker.HDT):
#                             heading = float(msg.heading)
                        
#                         if latitude is not None and longitude is not None:
#                             nav_msg = NavSatFix()
#                             nav_msg.latitude = latitude
#                             nav_msg.longitude = longitude
#                             self.gnss_publisher.publish(nav_msg)
#                             self.get_logger().info(f"Published GNSS data: lat={latitude}, lon={longitude}")
                            
#                             # ถ้า heading ไม่มี ให้คำนวณจากตำแหน่งก่อนหน้า
#                             if heading is None and self.prev_lat is not None and self.prev_lon is not None:
#                                 heading = calculate_bearing(self.prev_lat, self.prev_lon, latitude, longitude)
                            
#                             # Publish heading ถ้ามี
#                             if heading is not None:
#                                 heading_msg = Float32()
#                                 heading_msg.data = heading
#                                 self.heading_publisher.publish(heading_msg)
#                                 self.get_logger().info(f"Published heading: {heading}")
                            
#                             # เก็บตำแหน่งปัจจุบันสำหรับรอบถัดไป
#                             self.prev_lat = latitude
#                             self.prev_lon = longitude
                        
#                         # Forward RTK corrections from NTRIP
#                         if self.ntrip_socket:
#                             try:
#                                 self.ntrip_socket.setblocking(0)
#                                 try:
#                                     rtk_data = self.ntrip_socket.recv(1024)
#                                     if rtk_data:
#                                         self.ser.write(rtk_data)
#                                 except socket.error:
#                                     pass
#                             except Exception as e:
#                                 self.get_logger().error(f"Error handling NTRIP data: {e}")
                    
#                     except pynmea2.nmea.ChecksumError:
#                         self.get_logger().warning("Checksum error in NMEA data")
            
#             except Exception as e:
#                 self.get_logger().error(f"Error reading serial data: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     gnss_publisher = GNSSPublisher()
#     rclpy.spin(gnss_publisher)
    
#     if hasattr(gnss_publisher, 'ser') and gnss_publisher.ser.is_open:
#         gnss_publisher.ser.close()
#     if hasattr(gnss_publisher, 'ntrip_socket') and gnss_publisher.ntrip_socket:
#         gnss_publisher.ntrip_socket.close()
    
#     gnss_publisher.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



# import serial
# import pynmea2
# import json
# import socket
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
# import math

# class GNSSPublisher(Node):
#     def __init__(self):
#         super().__init__('gnss_publisher')
        
#         self.gnss_publisher = self.create_publisher(
#             NavSatFix,
#             '/navigation/gnss',
#             10)
        
#         self.serial_port = '/dev/ttyACM0'  # ปรับตามพอร์ตของ GNSS
        
#         self.ntrip_server_ip = "110.78.0.54"
#         self.ntrip_server_port = 2116
#         self.ntrip_username = "1118600009224"
#         self.ntrip_password = "CK79"
#         self.mount_point = "VRS_RTCM32"
        
#         try:
#             self.ser = serial.Serial(self.serial_port, 9600, timeout=1)
#             self.get_logger().info(f"Connected to {self.serial_port}")
#         except Exception as e:
#             self.get_logger().error(f"Error connecting to serial port {self.serial_port}: {e}")
#             return
        
#         self.ntrip_socket = self.connect_to_ntrip()
        
#         self.create_timer(0.1, self.read_gnss_data)  # 10 Hz
        
#         self.get_logger().info('GNSS Publisher initialized (Position only)')
    
#     def connect_to_ntrip(self):
#         try:
#             client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#             client_socket.connect((self.ntrip_server_ip, self.ntrip_server_port))
            
#             import base64
#             auth_str = f"{self.ntrip_username}:{self.ntrip_password}"
#             auth_bytes = auth_str.encode('ascii')
#             auth_b64 = base64.b64encode(auth_bytes).decode('ascii')
            
#             request = (
#                 f"GET /{self.mount_point} HTTP/1.1\r\n"
#                 f"User-Agent: NTRIP Client\r\n"
#                 f"Authorization: Basic {auth_b64}\r\n"
#                 f"\r\n"
#             )
#             client_socket.send(request.encode('ascii'))
#             self.get_logger().info("Connected to NTRIP server")
#             return client_socket
#         except Exception as e:
#             self.get_logger().error(f"Error connecting to NTRIP server: {e}")
#             return None
    
#     def read_gnss_data(self):
#         if hasattr(self, 'ser') and self.ser.is_open:
#             try:
#                 data = self.ser.readline().decode('ascii', errors='replace')
                
#                 # ใช้เฉพาะ NMEA messages ที่มีตำแหน่ง
#                 if data.startswith('$GPGGA') or data.startswith('$GNGLL') or data.startswith('$GPRMC'):
#                     try:
#                         msg = pynmea2.parse(data)
                        
#                         latitude = None
#                         longitude = None
                        
#                         if isinstance(msg, pynmea2.types.talker.GGA):
#                             latitude = msg.latitude
#                             longitude = msg.longitude
#                         elif isinstance(msg, pynmea2.types.talker.GLL):
#                             latitude = msg.latitude
#                             longitude = msg.longitude
#                         elif isinstance(msg, pynmea2.types.talker.RMC):
#                             latitude = msg.latitude
#                             longitude = msg.longitude
                        
#                         if latitude is not None and longitude is not None:
#                             # Publish ใน ROS2
#                             nav_msg = NavSatFix()
#                             nav_msg.latitude = latitude
#                             nav_msg.longitude = longitude
#                             self.gnss_publisher.publish(nav_msg)
#                             self.get_logger().info(f"Published GNSS data: lat={latitude}, lon={longitude}")
                        
#                         # Forward RTK corrections from NTRIP
#                         if self.ntrip_socket:
#                             try:
#                                 self.ntrip_socket.setblocking(0)
#                                 try:
#                                     rtk_data = self.ntrip_socket.recv(1024)
#                                     if rtk_data:
#                                         self.ser.write(rtk_data)
#                                 except socket.error:
#                                     pass
#                             except Exception as e:
#                                 self.get_logger().error(f"Error handling NTRIP data: {e}")
                    
#                     except pynmea2.nmea.ChecksumError:
#                         self.get_logger().warning("Checksum error in NMEA data")
            
#             except Exception as e:
#                 self.get_logger().error(f"Error reading serial data: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     gnss_publisher = GNSSPublisher()
#     rclpy.spin(gnss_publisher)
    
#     if hasattr(gnss_publisher, 'ser') and gnss_publisher.ser.is_open:
#         gnss_publisher.ser.close()
#     if hasattr(gnss_publisher, 'ntrip_socket') and gnss_publisher.ntrip_socket:
#         gnss_publisher.ntrip_socket.close()
    
#     gnss_publisher.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



# #!/usr/bin/env python3
# """
# GPS Waypoint Logger - เก็บ waypoints ในขณะเดินรอบสนามฟุตบอล
# """

# import json
# import time
# import math
# import threading
# import sys
# import select
# import termios
# import tty
# from datetime import datetime
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
# from std_msgs.msg import String
# import os

# class GPSWaypointLogger(Node):
#     def __init__(self):
#         super().__init__('gps_waypoint_logger')
        
#         # ROS2 subscriber
#         self.gnss_sub = self.create_subscription(
#             NavSatFix, '/navigation/gnss', self.gnss_callback, 10
#         )
        
#         # Publisher for status
#         self.status_pub = self.create_publisher(String, '/waypoint_logger/status', 10)
        
#         # Waypoint storage
#         self.waypoints = []
#         self.is_logging = False
#         self.last_position = None
#         self.last_log_time = None
        
#         # Configuration
#         self.min_distance_threshold = 0.5  # เมตร - ระยะทางขั้นต่ำระหว่างจุด
#         self.max_distance_threshold = 10.0  # เมตร - ระยะทางสูงสุดที่ยอมรับได้
#         self.min_time_interval = 1.0  # วินาที - เวลาขั้นต่ำระหว่างการเก็บ
#         self.max_speed_threshold = 2.0  # m/s - ความเร็วสูงสุดที่ยอมรับ (เดิน)
        
#         # Filtering parameters
#         self.position_buffer = []
#         self.buffer_size = 5  # เก็บ 5 ตำแหน่งล่าสุดเพื่อ smooth
        
#         # Boundary detection
#         self.boundary_start_position = None
#         self.boundary_completed = False
#         self.min_boundary_points = 8  # จุดขั้นต่ำสำหรับ boundary
        
#         # File handling
#         self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
#         self.output_dir = "waypoint_logs"
#         os.makedirs(self.output_dir, exist_ok=True)
        
#         # Keyboard handling
#         self.running = True
#         self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
#         self.keyboard_thread.start()
        
#         self.get_logger().info('🏈 GPS Waypoint Logger initialized')
#         self.get_logger().info(f'📁 Output directory: {self.output_dir}')
#         self.print_help()
        
#         # Auto-save timer
#         self.create_timer(30.0, self.auto_save_waypoints)
    
#     def print_help(self):
#         """แสดงคำสั่งทั้งหมด"""
#         self.get_logger().info('📱 Keyboard commands:')
#         self.get_logger().info('   s = START logging (เริ่มเก็บ waypoints)')
#         self.get_logger().info('   p = STOP/PAUSE logging (หยุดเก็บ waypoints)')
#         self.get_logger().info('   r = RESET waypoints (ลบ waypoints ทั้งหมด)')
#         self.get_logger().info('   d = DUMP waypoints (แสดง waypoints ปัจจุบัน)')
#         self.get_logger().info('   f = FORCE SAVE (บันทึกไฟล์ทันที)')
#         self.get_logger().info('   h = HELP (แสดงคำสั่งนี้)')
#         self.get_logger().info('   q = QUIT (ออกจากโปรแกรม)')
    
#     def keyboard_listener(self):
#         """รับคำสั่งจาก keyboard"""
#         # บันทึกการตั้งค่า terminal เดิม
#         old_settings = termios.tcgetattr(sys.stdin)
#         try:
#             # ตั้งค่า terminal เป็น raw mode
#             tty.cbreak(sys.stdin.fileno())
            
#             while self.running:
#                 if select.select([sys.stdin], [], [], 0.1) == ([sys.stdin], [], []):
#                     ch = sys.stdin.read(1).lower()
                    
#                     if ch == 's':
#                         self.start_logging()
#                     elif ch == 'p':
#                         self.stop_logging()
#                     elif ch == 'r':
#                         self.reset_waypoints()
#                     elif ch == 'd':
#                         self.dump_current_waypoints()
#                     elif ch == 'f':
#                         if len(self.waypoints) > 0:
#                             self.save_waypoints_to_file(auto_save=False)
#                         else:
#                             self.get_logger().warn('No waypoints to save')
#                     elif ch == 'h':
#                         self.print_help()
#                     elif ch == 'q':
#                         self.get_logger().info('🛑 Quit requested')
#                         self.running = False
#                         if self.is_logging:
#                             self.stop_logging()
#                         rclpy.shutdown()
#                         break
#         finally:
#             # คืนการตั้งค่า terminal เดิม
#             termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    
#     def gnss_callback(self, msg):
#         """Handle GNSS position updates"""
#         # ตรวจสอบความถูกต้องของข้อมูล GPS
#         if not self.is_valid_gnss_data(msg):
#             return
        
#         current_time = time.time()
#         current_position = {
#             'lat': msg.latitude,
#             'lon': msg.longitude,
#             'timestamp': current_time,
#             'altitude': msg.altitude if hasattr(msg, 'altitude') else 0.0
#         }
        
#         # แสดงข้อมูล GPS ที่ได้รับ (ไม่ว่าจะ logging หรือไม่)
#         if len(self.waypoints) == 0 or time.time() - getattr(self, 'last_display_time', 0) > 2.0:
#             status = "🟢 LOGGING" if self.is_logging else "🔴 PAUSED"
#             self.get_logger().info(f'{status} - GPS: {msg.latitude:.6f}, {msg.longitude:.6f}')
#             self.last_display_time = time.time()
        
#         # เพิ่มเข้า buffer สำหรับ smoothing
#         self.position_buffer.append(current_position)
#         if len(self.position_buffer) > self.buffer_size:
#             self.position_buffer.pop(0)
        
#         # ประมวลผลเฉพาะเมื่อ logging
#         if self.is_logging:
#             self.process_position_for_logging(current_position)
    
#     def is_valid_gnss_data(self, msg):
#         """ตรวจสอบความถูกต้องของข้อมูล GNSS"""
#         # ตรวจสอบค่าพื้นฐาน
#         if (msg.latitude == 0.0 and msg.longitude == 0.0) or \
#            not (-90 <= msg.latitude <= 90) or \
#            not (-180 <= msg.longitude <= 180):
#             return False
        
#         # ตรวจสอบสถานะ GPS
#         if hasattr(msg, 'status'):
#             if msg.status.status < 0:  # GPS ไม่ได้ fix
#                 return False
        
#         return True
    
#     def process_position_for_logging(self, current_position):
#         """ประมวลผลตำแหน่งสำหรับการเก็บ waypoint"""
#         current_time = time.time()
        
#         # ตรวจสอบเวลาขั้นต่ำ
#         if (self.last_log_time and 
#             current_time - self.last_log_time < self.min_time_interval):
#             return
        
#         # Smooth position ด้วย buffer
#         smoothed_position = self.get_smoothed_position()
#         if not smoothed_position:
#             return
        
#         # ตรวจสอบระยะทางจากจุดก่อนหน้า
#         if self.last_position:
#             distance = self.calculate_distance(
#                 self.last_position['lat'], self.last_position['lon'],
#                 smoothed_position['lat'], smoothed_position['lon']
#             )
            
#             # ข้ามถ้าใกล้เกินไป
#             if distance < self.min_distance_threshold:
#                 return
            
#             # ข้ามถ้าไกลเกินไป (อาจเป็น GPS error)
#             if distance > self.max_distance_threshold:
#                 self.get_logger().warn(f'⚠️ Distance too large: {distance:.1f}m, skipping')
#                 return
            
#             # ตรวจสอบความเร็ว
#             time_diff = current_time - self.last_log_time
#             speed = distance / time_diff if time_diff > 0 else 0
            
#             if speed > self.max_speed_threshold:
#                 self.get_logger().warn(f'⚠️ Speed too high: {speed:.1f}m/s, skipping')
#                 return
        
#         # เก็บ waypoint
#         self.add_waypoint(smoothed_position)
#         self.last_position = smoothed_position
#         self.last_log_time = current_time
    
#     def get_smoothed_position(self):
#         """ได้ตำแหน่งที่ smooth แล้วจาก buffer"""
#         if len(self.position_buffer) < 3:
#             return None
        
#         # ใช้ moving average
#         lat_sum = sum(pos['lat'] for pos in self.position_buffer)
#         lon_sum = sum(pos['lon'] for pos in self.position_buffer)
        
#         return {
#             'lat': lat_sum / len(self.position_buffer),
#             'lon': lon_sum / len(self.position_buffer),
#             'timestamp': time.time()
#         }
    
#     def add_waypoint(self, position):
#         """เพิ่ม waypoint ใหม่"""
#         waypoint = {
#             'id': len(self.waypoints) + 1,
#             'lat': position['lat'],
#             'lon': position['lon'],
#             'timestamp': position['timestamp'],
#             'datetime': datetime.fromtimestamp(position['timestamp']).isoformat()
#         }
        
#         self.waypoints.append(waypoint)
        
#         # Log การเพิ่ม waypoint
#         self.get_logger().info(
#             f'📍 Waypoint {waypoint["id"]}: '
#             f'{waypoint["lat"]:.6f}, {waypoint["lon"]:.6f}'
#         )
        
#         # ตรวจสอบการปิด boundary
#         self.check_boundary_completion(position)
        
#         # ส่งสถานะ
#         self.publish_status(f'Logged waypoint {len(self.waypoints)}')
    
#     def check_boundary_completion(self, current_position):
#         """ตรวจสอบว่าเดินรอบสนามครบหรือยัง"""
#         if len(self.waypoints) < self.min_boundary_points:
#             return
        
#         if not self.boundary_start_position:
#             self.boundary_start_position = self.waypoints[0]
#             return
        
#         # ตรวจสอบระยะทางกลับไปจุดเริ่มต้น
#         distance_to_start = self.calculate_distance(
#             current_position['lat'], current_position['lon'],
#             self.boundary_start_position['lat'], self.boundary_start_position['lon']
#         )
        
#         if distance_to_start < self.min_distance_threshold * 2:
#             if not self.boundary_completed:
#                 self.boundary_completed = True
#                 self.get_logger().info('🎉 Boundary completed! รอบสนามเสร็จแล้ว')
#                 self.publish_status('Boundary completed')
                
#                 # Auto-save เมื่อครบรอบ
#                 self.save_waypoints_to_file(auto_save=False)
    
#     def calculate_distance(self, lat1, lon1, lat2, lon2):
#         """คำนวณระยะทางระหว่าง 2 จุด (เมตร)"""
#         R = 6371e3  # รัศมีโลก
#         φ1 = math.radians(lat1)
#         φ2 = math.radians(lat2)
#         Δφ = math.radians(lat2 - lat1)
#         Δλ = math.radians(lon2 - lon1)

#         a = (math.sin(Δφ/2) * math.sin(Δφ/2) +
#              math.cos(φ1) * math.cos(φ2) *
#              math.sin(Δλ/2) * math.sin(Δλ/2))
#         c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

#         return R * c
    
#     def start_logging(self):
#         """เริ่มเก็บ waypoints"""
#         if not self.is_logging:
#             self.is_logging = True
#             self.waypoints = []
#             self.last_position = None
#             self.last_log_time = None
#             self.boundary_start_position = None
#             self.boundary_completed = False
            
#             self.get_logger().info('🟢 Started waypoint logging')
#             self.publish_status('Logging started')
#         else:
#             self.get_logger().info('Already logging!')
    
#     def stop_logging(self):
#         """หยุดเก็บ waypoints"""
#         if self.is_logging:
#             self.is_logging = False
#             self.get_logger().info(f'🔴 Stopped waypoint logging. Total: {len(self.waypoints)} points')
#             self.publish_status(f'Logging stopped. {len(self.waypoints)} waypoints collected')
            
#             # บันทึกไฟล์
#             if len(self.waypoints) > 0:
#                 self.save_waypoints_to_file()
#         else:
#             self.get_logger().info('Not currently logging!')
    
#     def reset_waypoints(self):
#         """รีเซ็ต waypoints ทั้งหมด"""
#         self.waypoints = []
#         self.last_position = None
#         self.boundary_start_position = None
#         self.boundary_completed = False
#         self.get_logger().info('🔄 Reset all waypoints')
#         self.publish_status('Waypoints reset')
    
#     def save_waypoints_to_file(self, auto_save=True):
#         """บันทึก waypoints ลงไฟล์"""
#         if len(self.waypoints) == 0:
#             self.get_logger().warn('No waypoints to save')
#             return
        
#         # สร้างข้อมูลสำหรับบันทึก
#         save_data = {
#             'session_id': self.session_id,
#             'timestamp': datetime.now().isoformat(),
#             'description': 'Football field boundary waypoints',
#             'total_waypoints': len(self.waypoints),
#             'boundary_completed': self.boundary_completed,
#             'collection_method': 'GPS logging',
#             'settings': {
#                 'min_distance_threshold': self.min_distance_threshold,
#                 'min_time_interval': self.min_time_interval,
#                 'max_speed_threshold': self.max_speed_threshold
#             },
#             'waypoints': self.waypoints,
#             'statistics': self.calculate_waypoint_statistics()
#         }
        
#         # กำหนดชื่อไฟล์
#         prefix = 'auto_' if auto_save else ''
#         filename = f'{prefix}football_field_waypoints_{self.session_id}.json'
#         filepath = os.path.join(self.output_dir, filename)
        
#         try:
#             with open(filepath, 'w', encoding='utf-8') as f:
#                 json.dump(save_data, f, indent=2, ensure_ascii=False)
            
#             self.get_logger().info(f'💾 Saved {len(self.waypoints)} waypoints to {filepath}')
            
#             # สร้าง JavaScript code file
#             self.generate_javascript_code(filepath.replace('.json', '.js'))
            
#         except Exception as e:
#             self.get_logger().error(f'Failed to save waypoints: {e}')
    
#     def generate_javascript_code(self, js_filepath):
#         """สร้างไฟล์ JavaScript code สำหรับใช้ในเว็บ"""
#         if len(self.waypoints) == 0:
#             return
        
#         js_code = f"""// ===== Football Field Boundary Waypoints =====
# // Generated on: {datetime.now().isoformat()}
# // Session ID: {self.session_id}
# // Total waypoints: {len(self.waypoints)}
# // Collection method: GPS logging

# const footballFieldBoundary = [
# """
        
#         for i, waypoint in enumerate(self.waypoints):
#             js_code += f'    {{ lat: {waypoint["lat"]:.6f}, lon: {waypoint["lon"]:.6f} }}'
#             if i < len(self.waypoints) - 1:
#                 js_code += ','
#             js_code += f'  // Point {waypoint["id"]}\n'
        
#         js_code += f"""];

# // Statistics
# const boundaryStats = {{
#     totalPoints: {len(self.waypoints)},
#     completed: {str(self.boundary_completed).lower()},
#     totalDistance: {self.calculate_total_distance():.1f}, // meters
#     area: calculateBoundaryArea(footballFieldBoundary) // will be calculated on load
# }};

# console.log("🏈 Loaded", footballFieldBoundary.length, "waypoints for football field boundary");
# console.log("🏈 Boundary completed:", boundaryStats.completed);
# console.log("🏈 Total distance:", boundaryStats.totalDistance, "meters");

# // Function to calculate boundary area (Shoelace formula)
# function calculateBoundaryArea(boundary) {{
#     if (boundary.length < 3) return 0;
    
#     let area = 0;
#     const n = boundary.length;
    
#     for (let i = 0; i < n; i++) {{
#         const j = (i + 1) % n;
#         area += (boundary[i].lon * boundary[j].lat - boundary[j].lon * boundary[i].lat);
#     }}
    
#     area = Math.abs(area) / 2;
#     const metersPerDegree = 111000; // approximate
#     return area * metersPerDegree * metersPerDegree;
# }}
# """
        
#         try:
#             with open(js_filepath, 'w', encoding='utf-8') as f:
#                 f.write(js_code)
            
#             self.get_logger().info(f'📋 Generated JavaScript code: {js_filepath}')
#         except Exception as e:
#             self.get_logger().error(f'Failed to generate JavaScript code: {e}')
    
#     def calculate_waypoint_statistics(self):
#         """คำนวณสถิติของ waypoints"""
#         if len(self.waypoints) < 2:
#             return {}
        
#         total_distance = self.calculate_total_distance()
        
#         # คำนวณพื้นที่ (Shoelace formula)
#         area = 0
#         n = len(self.waypoints)
#         for i in range(n):
#             j = (i + 1) % n
#             area += (self.waypoints[i]['lon'] * self.waypoints[j]['lat'] - 
#                     self.waypoints[j]['lon'] * self.waypoints[i]['lat'])
#         area = abs(area) / 2
#         area_sqm = area * 111000 * 111000  # ประมาณ
        
#         return {
#             'total_distance_meters': total_distance,
#             'estimated_area_sqm': area_sqm,
#             'average_point_spacing': total_distance / (len(self.waypoints) - 1) if len(self.waypoints) > 1 else 0,
#             'collection_duration_seconds': (self.waypoints[-1]['timestamp'] - self.waypoints[0]['timestamp']) if len(self.waypoints) > 1 else 0
#         }
    
#     def calculate_total_distance(self):
#         """คำนวณระยะทางรวมของเส้นทาง"""
#         if len(self.waypoints) < 2:
#             return 0
        
#         total_distance = 0
#         for i in range(len(self.waypoints) - 1):
#             distance = self.calculate_distance(
#                 self.waypoints[i]['lat'], self.waypoints[i]['lon'],
#                 self.waypoints[i + 1]['lat'], self.waypoints[i + 1]['lon']
#             )
#             total_distance += distance
        
#         return total_distance
    
#     def auto_save_waypoints(self):
#         """บันทึกอัตโนมัติทุก 30 วินาที"""
#         if self.is_logging and len(self.waypoints) > 0:
#             self.save_waypoints_to_file(auto_save=True)
    
#     def publish_status(self, message):
#         """ส่งสถานะผ่าน ROS topic"""
#         status_msg = String()
#         status_msg.data = message
#         self.status_pub.publish(status_msg)
    
#     def dump_current_waypoints(self):
#         """แสดง waypoints ปัจจุบัน"""
#         self.get_logger().info(f'📊 Current waypoints ({len(self.waypoints)}):')
#         if len(self.waypoints) == 0:
#             self.get_logger().info('   (No waypoints collected yet)')
#             return
            
#         for wp in self.waypoints[:10]:  # แสดงแค่ 10 ตัวแรก
#             self.get_logger().info(f'   {wp["id"]}: {wp["lat"]:.6f}, {wp["lon"]:.6f}')
        
#         if len(self.waypoints) > 10:
#             self.get_logger().info(f'   ... และอีก {len(self.waypoints) - 10} จุด')
        
#         if len(self.waypoints) > 0:
#             stats = self.calculate_waypoint_statistics()
#             self.get_logger().info(f'📏 Total distance: {stats.get("total_distance_meters", 0):.1f}m')
#             self.get_logger().info(f'📐 Estimated area: {stats.get("estimated_area_sqm", 0):.0f} sqm')
#             self.get_logger().info(f'🎯 Boundary completed: {self.boundary_completed}')

# def main(args=None):
#     rclpy.init(args=args)
#     logger = GPSWaypointLogger()
    
#     try:
#         logger.get_logger().info('🏈 GPS Waypoint Logger ready!')
#         logger.get_logger().info('⚠️  Make sure GNSS Publisher is running!')
#         logger.get_logger().info('📱 Press "s" to start logging, "h" for help')
        
#         rclpy.spin(logger)
#     except KeyboardInterrupt:
#         logger.get_logger().info('🛑 Shutting down GPS Waypoint Logger')
#         logger.running = False
#         if logger.is_logging:
#             logger.stop_logging()
#     finally:
#         logger.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()




# import struct
# import time

# class ZEDF9RConfig:
#     def __init__(self, serial_connection):
#         self.ser = serial_connection
        
#     def create_ubx_message(self, msg_class, msg_id, payload):
#         """สร้าง UBX message ที่ถูกต้อง"""
#         header = struct.pack('<BBBB', 0xb5, 0x62, msg_class, msg_id)
#         length = struct.pack('<H', len(payload))
        
#         # คำนวณ checksum
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
#         return cmd
    
#     def configure_zed_f9r_imu(self):
#         """Configuration ที่ถูกต้องสำหรับ ZED-F9R IMU"""
        
#         print("🔧 กำลัง configure ZED-F9R สำหรับ IMU...")
        
#         try:
#             # 1. เปิดใช้ UBX-ESF-MEAS (จำเป็นสำหรับ IMU data)
#             print("  📤 เปิด UBX-ESF-MEAS...")
#             self.enable_ubx_message(0x10, 0x02, 1)  # ESF-MEAS
            
#             # 2. เปิดใช้ UBX-ESF-STATUS (monitor calibration)
#             print("  📤 เปิด UBX-ESF-STATUS...")
#             self.enable_ubx_message(0x10, 0x10, 1)  # ESF-STATUS
            
#             # 3. เปิดใช้ UBX-NAV-ATT (attitude data)
#             print("  📤 เปิด UBX-NAV-ATT...")
#             self.enable_ubx_message(0x01, 0x05, 1)  # NAV-ATT
            
#             # 4. เปิดใช้ UBX-ESF-ALG (alignment data)
#             print("  📤 เปิด UBX-ESF-ALG...")
#             self.enable_ubx_message(0x10, 0x14, 1)  # ESF-ALG
            
#             # 5. Configure IMU mounting alignment (automatic)
#             print("  ⚙️  ตั้งค่า IMU alignment...")
#             # CFG-ESKALMAN-IMU_MNTALG_AUTO = 1 (automatic alignment)
#             cfg_payload = struct.pack('<LL', 0x401A001A, 1)  # Enable auto alignment
#             cfg_cmd = self.create_ubx_message(0x06, 0x8A, cfg_payload)
#             self.ser.write(cfg_cmd)
#             time.sleep(0.1)
            
#             # 6. Enable sensor fusion
#             print("  🔄 เปิด sensor fusion...")
#             cfg_payload = struct.pack('<LL', 0x40240001, 1)  # CFG-SFCORE-USE_SF
#             cfg_cmd = self.create_ubx_message(0x06, 0x8A, cfg_payload)
#             self.ser.write(cfg_cmd)
#             time.sleep(0.1)
            
#             # 7. Set navigation rate to 5Hz for better IMU performance
#             print("  📡 ตั้งค่า navigation rate...")
#             rate_payload = struct.pack('<HHH', 200, 1, 1)  # 200ms = 5Hz
#             rate_cmd = self.create_ubx_message(0x06, 0x08, rate_payload)
#             self.ser.write(rate_cmd)
#             time.sleep(0.1)
            
#             # 8. Save configuration to flash
#             print("  💾 บันทึกการตั้งค่า...")
#             save_payload = struct.pack('<LLLL', 0x00000000, 0xFFFFFFFF, 0x00000000, 0x00000000)
#             save_cmd = self.create_ubx_message(0x06, 0x09, save_payload)
#             self.ser.write(save_cmd)
#             time.sleep(0.5)
            
#             print("✅ Configuration เสร็จสิ้น!")
#             print("⏳ รอ IMU initialization (2-3 นาที)...")
#             print("🚶 เคลื่อนที่อุปกรณ์เบาๆ เพื่อ calibrate...")
            
#             return True
            
#         except Exception as e:
#             print(f"❌ เกิดข้อผิดพลาด: {e}")
#             return False

# def main():
#     """ตัวอย่างการใช้งาน"""
#     import serial
    
#     try:
#         # เชื่อมต่อกับ ZED-F9R
#         ser = serial.Serial('/dev/ttyACM0', 38400, timeout=1)
#         time.sleep(2)
        
#         # สร้าง configurator
#         config = ZEDF9RConfig(ser)
        
#         # Configure IMU
#         success = config.configure_zed_f9r_imu()
        
#         if success:
#             print("\n📋 ขั้นตอนต่อไป:")
#             print("  1. รอ 2-3 นาที เพื่อให้ IMU warm up")
#             print("  2. เคลื่อนที่อุปกรณ์ในทุกทิศทาง เบาๆ")
#             print("  3. รัน GNSS node ใหม่")
#             print("  4. ตรวจสอบด้วย helper script")
        
#         ser.close()
        
#     except Exception as e:
#         print(f"❌ ไม่สามารถเชื่อมต่อ: {e}")
#         print("💡 ลองตรวจสอบ:")
#         print("  - Port ที่ถูกต้อง (/dev/ttyACM0)")
#         print("  - Baud rate (ลอง 9600, 38400, 115200)")
#         print("  - การเชื่อมต่อ USB")

# if __name__ == "__main__":
#     main()



# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Point
# from std_msgs.msg import Float32, String
# import math
# import time
# import json
# import statistics
# import threading
# from collections import deque

# class AdvancedWaypointRecorder(Node):
#     def __init__(self):
#         super().__init__('advanced_waypoint_recorder')
        
#         # Subscribe to ZED-F9R data
#         self.xy_subscription = self.create_subscription(
#             Point, '/navigation/xy_position', self.xy_callback, 10
#         )
#         self.heading_subscription = self.create_subscription(
#             Float32, '/navigation/heading', self.heading_callback, 10
#         )
#         self.debug_subscription = self.create_subscription(
#             String, '/navigation/heading_debug', self.debug_callback, 10
#         )
        
#         # Current data
#         self.current_pos = {'x': 0.0, 'y': 0.0}
#         self.current_heading = None
#         self.heading_source = "NONE"
#         self.gps_quality = "UNKNOWN"
#         self.imu_status = "UNKNOWN"
        
#         # Manual heading input
#         self.manual_heading = None
#         self.heading_input_active = False
        
#         # Recording state
#         self.recording = False
#         self.recording_duration = 60.0      # เพิ่มเป็น 60 วินาที
#         self.min_samples = 100              # ขั้นต่ำ 100 ตัวอย่าง
#         self.quality_threshold = 0.05       # ±5cm สำหรับ excellent quality
        
#         # Data buffers
#         self.position_buffer = deque(maxlen=2000)  # เพิ่มขนาด buffer
#         self.heading_buffer = deque(maxlen=2000)
        
#         # Recording session
#         self.recording_start_time = 0
#         self.waypoint_name = ""
        
#         # Saved waypoints
#         self.waypoints = []
#         self.waypoint_file = "precision_waypoints.json"
        
#         # Heading management
#         self.heading_methods = {
#             'auto': 'Auto (IMU -> GPS -> Manual)',
#             'manual': 'Manual input only',
#             'movement': 'Movement calculation only',
#             'fixed': 'Fixed heading (0°)'
#         }
#         self.current_heading_method = 'auto'
#         self.fixed_heading_value = 0.0
        
#         # Load existing waypoints
#         self.load_waypoints()
        
#         # Status display timer
#         self.create_timer(1.0, self.display_status)
        
#         self.get_logger().info('🎯 Advanced Waypoint Recorder Ready!')
#         self.print_instructions()
        
#         # Start command input thread
#         self.input_thread = threading.Thread(target=self.command_input_loop, daemon=True)
#         self.input_thread.start()

#     def print_instructions(self):
#         """📋 Print detailed instructions"""
#         print("\n" + "="*70)
#         print("🎯 ADVANCED STATIC WAYPOINT RECORDER")
#         print("="*70)
#         print("📋 RECORDING COMMANDS:")
#         print("  r <name>     - Start recording waypoint (60s)")
#         print("  rq <name>    - Quick recording (30s)")
#         print("  s            - Stop recording early")
#         print("")
#         print("🧭 HEADING COMMANDS:")
#         print("  h auto       - Auto heading (IMU->GPS->Manual)")
#         print("  h manual     - Manual heading input mode")
#         print("  h <degrees>  - Set manual heading (0-360)")
#         print("  h movement   - Calculate from movement only")
#         print("  h fixed <deg>- Use fixed heading")
#         print("")
#         print("📊 MANAGEMENT COMMANDS:")
#         print("  l            - List all waypoints")
#         print("  d <index>    - Delete waypoint by index")
#         print("  c            - Clear all waypoints")
#         print("  e            - Export navigation code")
#         print("  q            - Quit")
#         print("")
#         print("💡 QUALITY TIPS:")
#         print("  • Record when GPS shows 'FUSION + CALIBRATED'")
#         print("  • Stay perfectly still for 60 seconds")
#         print("  • Avoid windy conditions")
#         print("  • Best time: early morning or late afternoon")
#         print("="*70)

#     def xy_callback(self, msg):
#         """📍 Receive position data"""
#         self.current_pos = {'x': msg.x, 'y': msg.y}
        
#         if self.recording:
#             timestamp = time.time()
#             self.position_buffer.append({
#                 'x': msg.x,
#                 'y': msg.y,
#                 'timestamp': timestamp
#             })

#     def heading_callback(self, msg):
#         """🧭 Receive heading data"""
#         if msg.data is not None and not math.isnan(msg.data):
#             self.current_heading = msg.data
            
#             if self.recording and self.current_heading_method == 'auto':
#                 timestamp = time.time()
#                 self.heading_buffer.append({
#                     'heading': msg.data,
#                     'timestamp': timestamp
#                 })

#     def debug_callback(self, msg):
#         """📊 Receive GPS quality info"""
#         try:
#             debug_data = json.loads(msg.data)
            
#             # Extract heading source
#             if 'final_heading' in debug_data:
#                 self.heading_source = debug_data['final_heading'].get('source', 'NONE')
            
#             # Extract GPS quality
#             if 'zed_f9r_status' in debug_data:
#                 status = debug_data['zed_f9r_status']
#                 self.gps_quality = status.get('fusion_mode', 'UNKNOWN')
#                 self.imu_status = status.get('calibration_status', 'UNKNOWN')
#         except:
#             pass

#     def set_heading_method(self, method, value=None):
#         """🧭 Set heading method"""
#         if method in self.heading_methods:
#             self.current_heading_method = method
            
#             if method == 'manual':
#                 self.heading_input_active = True
#                 self.get_logger().info('🧭 Manual heading mode - use "h <degrees>" to set')
            
#             elif method == 'fixed' and value is not None:
#                 try:
#                     self.fixed_heading_value = float(value) % 360
#                     self.get_logger().info(f'🧭 Fixed heading set to {self.fixed_heading_value:.1f}°')
#                 except:
#                     self.get_logger().error('❌ Invalid heading value')
#                     return
            
#             self.get_logger().info(f'🧭 Heading method: {self.heading_methods[method]}')
#         else:
#             self.get_logger().error('❌ Invalid heading method')

#     def set_manual_heading(self, heading):
#         """🧭 Set manual heading"""
#         try:
#             self.manual_heading = float(heading) % 360
#             self.get_logger().info(f'🧭 Manual heading set to {self.manual_heading:.1f}°')
            
#             if self.recording and self.current_heading_method == 'manual':
#                 timestamp = time.time()
#                 self.heading_buffer.append({
#                     'heading': self.manual_heading,
#                     'timestamp': timestamp
#                 })
#         except:
#             self.get_logger().error('❌ Invalid heading value (0-360)')

#     def get_effective_heading(self):
#         """🧭 Get heading based on current method"""
#         if self.current_heading_method == 'auto':
#             return self.current_heading
#         elif self.current_heading_method == 'manual':
#             return self.manual_heading
#         elif self.current_heading_method == 'fixed':
#             return self.fixed_heading_value
#         elif self.current_heading_method == 'movement':
#             return self.calculate_movement_heading()
#         return None

#     def calculate_movement_heading(self):
#         """📍 Calculate heading from recent movement"""
#         if len(self.position_buffer) < 5:
#             return None
        
#         # Use last 5 positions for stable calculation
#         recent_positions = list(self.position_buffer)[-5:]
        
#         total_dx = 0
#         total_dy = 0
#         total_weight = 0
        
#         for i in range(len(recent_positions) - 1):
#             curr = recent_positions[i + 1]
#             prev = recent_positions[i]
            
#             dx = curr['x'] - prev['x']
#             dy = curr['y'] - prev['y']
#             distance = math.sqrt(dx*dx + dy*dy)
            
#             if distance > 0.1:  # Only consider significant movement
#                 total_dx += dx * distance
#                 total_dy += dy * distance
#                 total_weight += distance
        
#         if total_weight > 0.5:  # At least 0.5m total movement
#             heading_rad = math.atan2(total_dx, total_dy)
#             heading_deg = math.degrees(heading_rad)
#             if heading_deg < 0:
#                 heading_deg += 360
#             return heading_deg
        
#         return None

#     def start_recording(self, waypoint_name, duration=None):
#         """🔴 Start recording waypoint"""
#         if self.recording:
#             self.get_logger().warn('⚠️ Already recording! Stop first.')
#             return
        
#         # Check GPS quality
#         if self.gps_quality not in ['FUSION', 'INIT']:
#             self.get_logger().warn(f'⚠️ GPS quality: {self.gps_quality} - may affect accuracy')
        
#         if self.imu_status != 'CALIBRATED':
#             self.get_logger().warn(f'⚠️ IMU status: {self.imu_status} - heading may be inaccurate')
        
#         # Set recording parameters
#         self.waypoint_name = waypoint_name
#         self.recording = True
#         self.recording_start_time = time.time()
#         self.recording_duration = duration or 60.0
        
#         # Clear buffers
#         self.position_buffer.clear()
#         self.heading_buffer.clear()
        
#         self.get_logger().info(f'🔴 Recording "{waypoint_name}" for {self.recording_duration:.0f}s...')
#         self.get_logger().info(f'🧭 Heading method: {self.heading_methods[self.current_heading_method]}')
#         self.get_logger().info('📍 STAY PERFECTLY STILL for best accuracy!')
        
#         # Pre-populate heading if using manual/fixed
#         effective_heading = self.get_effective_heading()
#         if effective_heading is not None and self.current_heading_method in ['manual', 'fixed']:
#             self.heading_buffer.append({
#                 'heading': effective_heading,
#                 'timestamp': time.time()
#             })

#     def stop_recording(self):
#         """⏹️ Stop recording and calculate waypoint"""
#         if not self.recording:
#             self.get_logger().warn('⚠️ Not recording!')
#             return
        
#         self.recording = False
#         recording_time = time.time() - self.recording_start_time
        
#         if len(self.position_buffer) < self.min_samples:
#             self.get_logger().error(f'❌ Not enough samples! Got {len(self.position_buffer)}, need {self.min_samples}')
#             self.get_logger().info('💡 Try recording for longer or check GPS signal')
#             return
        
#         # Calculate waypoint statistics
#         waypoint = self.calculate_waypoint_statistics(recording_time)
        
#         if waypoint:
#             self.waypoints.append(waypoint)
#             self.save_waypoints()
#             self.display_waypoint_result(waypoint)

#     def calculate_waypoint_statistics(self, recording_time):
#         """📊 Calculate comprehensive waypoint statistics"""
#         if not self.position_buffer:
#             return None
        
#         # Extract positions
#         x_values = [p['x'] for p in self.position_buffer]
#         y_values = [p['y'] for p in self.position_buffer]
        
#         # Position statistics
#         x_mean = statistics.mean(x_values)
#         y_mean = statistics.mean(y_values)
#         x_std = statistics.stdev(x_values) if len(x_values) > 1 else 0.0
#         y_std = statistics.stdev(y_values) if len(y_values) > 1 else 0.0
        
#         # Calculate confidence metrics
#         confidence_radius = 2.0 * math.sqrt(x_std**2 + y_std**2)  # 95% confidence
#         max_deviation = max(
#             max(abs(x - x_mean) for x in x_values),
#             max(abs(y - y_mean) for y in y_values)
#         )
        
#         # Quality assessment
#         quality = "EXCELLENT" if confidence_radius < self.quality_threshold else \
#                  "GOOD" if confidence_radius < 0.15 else \
#                  "FAIR" if confidence_radius < 0.5 else "POOR"
        
#         # Heading statistics
#         heading_mean = None
#         heading_std = 0.0
#         heading_count = 0
        
#         if self.heading_buffer:
#             heading_values = [h['heading'] for h in self.heading_buffer]
#             heading_values = self.unwrap_headings(heading_values)
            
#             if heading_values:
#                 heading_mean = statistics.mean(heading_values) % 360
#                 heading_std = statistics.stdev(heading_values) if len(heading_values) > 1 else 0.0
#                 heading_count = len(heading_values)
#         else:
#             # Use current effective heading if no buffer
#             effective_heading = self.get_effective_heading()
#             if effective_heading is not None:
#                 heading_mean = effective_heading
#                 heading_count = 1
        
#         waypoint = {
#             'name': self.waypoint_name,
#             'x': x_mean,
#             'y': y_mean,
#             'heading': heading_mean,
            
#             # Statistics
#             'x_std': x_std,
#             'y_std': y_std,
#             'heading_std': heading_std,
#             'confidence_radius': confidence_radius,
#             'max_deviation': max_deviation,
#             'quality': quality,
            
#             # Sample info
#             'position_samples': len(self.position_buffer),
#             'heading_samples': heading_count,
#             'recording_duration': recording_time,
            
#             # Metadata
#             'heading_method': self.current_heading_method,
#             'heading_source': self.heading_source,
#             'gps_quality': self.gps_quality,
#             'imu_status': self.imu_status,
#             'timestamp': time.time()
#         }
        
#         return waypoint

#     def display_waypoint_result(self, waypoint):
#         """📊 Display detailed waypoint results"""
#         print("\n" + "="*60)
#         print(f"✅ WAYPOINT RECORDED: {waypoint['name']}")
#         print("="*60)
        
#         # Position info
#         print(f"📍 Position: ({waypoint['x']:.4f}, {waypoint['y']:.4f})")
#         print(f"🎯 Accuracy: ±{waypoint['confidence_radius']*100:.1f}cm ({waypoint['quality']})")
#         print(f"📏 Max deviation: ±{waypoint['max_deviation']*100:.1f}cm")
        
#         # Heading info
#         if waypoint['heading'] is not None:
#             print(f"🧭 Heading: {waypoint['heading']:.1f}° (±{waypoint['heading_std']:.1f}°)")
#             print(f"🔧 Method: {waypoint['heading_method']} ({waypoint['heading_source']})")
#         else:
#             print("🧭 Heading: Not available")
        
#         # Sample info
#         print(f"📊 Samples: {waypoint['position_samples']} pos, {waypoint['heading_samples']} heading")
#         print(f"⏱️ Duration: {waypoint['recording_duration']:.1f}s")
        
#         # Quality info
#         print(f"📡 GPS: {waypoint['gps_quality']}, IMU: {waypoint['imu_status']}")
        
#         # Quality recommendations
#         if waypoint['quality'] == 'EXCELLENT':
#             print("🌟 EXCELLENT quality - ready for precision navigation!")
#         elif waypoint['quality'] == 'GOOD':
#             print("✅ GOOD quality - suitable for most navigation tasks")
#         elif waypoint['quality'] == 'FAIR':
#             print("⚠️ FAIR quality - consider re-recording for critical points")
#         else:
#             print("❌ POOR quality - recommend re-recording with better conditions")
        
#         print("="*60)

#     def unwrap_headings(self, headings):
#         """🔄 Handle heading wraparound for statistics"""
#         if not headings or len(headings) < 2:
#             return headings
        
#         unwrapped = [headings[0]]
        
#         for i in range(1, len(headings)):
#             diff = headings[i] - unwrapped[i-1]
            
#             if diff > 180:
#                 unwrapped.append(headings[i] - 360)
#             elif diff < -180:
#                 unwrapped.append(headings[i] + 360)
#             else:
#                 unwrapped.append(headings[i])
        
#         return unwrapped

#     def display_status(self):
#         """📺 Display current status"""
#         if self.recording:
#             elapsed = time.time() - self.recording_start_time
#             remaining = max(0, self.recording_duration - elapsed)
#             samples = len(self.position_buffer)
            
#             # Auto-stop after duration
#             if remaining <= 0 and samples >= self.min_samples:
#                 self.stop_recording()
#                 return
            
#             # Calculate current accuracy
#             if samples > 10:
#                 recent_positions = list(self.position_buffer)[-10:]
#                 x_vals = [p['x'] for p in recent_positions]
#                 y_vals = [p['y'] for p in recent_positions]
#                 current_std = math.sqrt(statistics.variance(x_vals) + statistics.variance(y_vals))
#                 accuracy_info = f", ~±{current_std*100:.1f}cm"
#             else:
#                 accuracy_info = ""
            
#             self.get_logger().info(
#                 f'🔴 Recording "{self.waypoint_name}": {remaining:.1f}s left, '
#                 f'{samples} samples{accuracy_info}'
#             )
#         else:
#             # Show current position and quality every 5 seconds
#             if int(time.time()) % 5 == 0:
#                 effective_heading = self.get_effective_heading()
#                 heading_info = f", H:{effective_heading:.1f}°" if effective_heading else ", H:N/A"
                
#                 self.get_logger().info(
#                     f'📍 Ready: ({self.current_pos["x"]:.3f}, {self.current_pos["y"]:.3f}){heading_info} '
#                     f'GPS:{self.gps_quality}, IMU:{self.imu_status}'
#                 )

#     def command_input_loop(self):
#         """⌨️ Handle user commands"""
#         while rclpy.ok():
#             try:
#                 command = input().strip()
#                 parts = command.split()
                
#                 if not parts:
#                     continue
                
#                 cmd = parts[0].lower()
                
#                 # Recording commands
#                 if cmd == 'r' and len(parts) > 1:
#                     self.start_recording(parts[1])
#                 elif cmd == 'rq' and len(parts) > 1:
#                     self.start_recording(parts[1], 30.0)  # Quick 30s recording
#                 elif cmd == 's':
#                     self.stop_recording()
                
#                 # Heading commands
#                 elif cmd == 'h' and len(parts) > 1:
#                     if parts[1] in self.heading_methods:
#                         value = parts[2] if len(parts) > 2 else None
#                         self.set_heading_method(parts[1], value)
#                     else:
#                         try:
#                             heading = float(parts[1])
#                             self.set_manual_heading(heading)
#                         except:
#                             print('❌ Invalid heading command')
                
#                 # Management commands
#                 elif cmd == 'l':
#                     self.list_waypoints()
#                 elif cmd == 'd' and len(parts) > 1:
#                     try:
#                         index = int(parts[1]) - 1
#                         self.delete_waypoint(index)
#                     except:
#                         print('❌ Invalid waypoint index')
#                 elif cmd == 'c':
#                     self.clear_waypoints()
#                 elif cmd == 'e':
#                     self.export_navigation_code()
#                 elif cmd == 'q':
#                     self.get_logger().info('👋 Quitting...')
#                     rclpy.shutdown()
#                     break
                
#                 else:
#                     print('❌ Unknown command. Type commands without prefix.')
            
#             except (EOFError, KeyboardInterrupt):
#                 break
#             except Exception as e:
#                 print(f'❌ Error: {e}')

#     def list_waypoints(self):
#         """📋 List all recorded waypoints"""
#         if not self.waypoints:
#             print('📭 No waypoints recorded yet.')
#             return
        
#         print(f'\n📋 Precision Waypoints ({len(self.waypoints)}):')
#         print('='*90)
#         print(f'{"#":>2} {"Name":<15} {"Position (X, Y)":<20} {"Heading":<8} {"Quality":<9} {"Accuracy":<10} {"Method":<10}')
#         print('-'*90)
        
#         for i, wp in enumerate(self.waypoints):
#             heading_str = f"{wp['heading']:.1f}°" if wp['heading'] is not None else "N/A"
#             accuracy_str = f"±{wp['confidence_radius']*100:.1f}cm"
            
#             print(f'{i+1:>2} {wp["name"]:<15} '
#                   f'({wp["x"]:7.3f}, {wp["y"]:7.3f})  '
#                   f'{heading_str:<8} {wp["quality"]:<9} '
#                   f'{accuracy_str:<10} {wp["heading_method"]:<10}')
#         print('='*90)

#     def delete_waypoint(self, index):
#         """🗑️ Delete waypoint by index"""
#         if 0 <= index < len(self.waypoints):
#             deleted = self.waypoints.pop(index)
#             self.save_waypoints()
#             print(f'✅ Deleted waypoint: {deleted["name"]}')
#         else:
#             print('❌ Invalid waypoint index')

#     def clear_waypoints(self):
#         """🗑️ Clear all waypoints"""
#         confirm = input('⚠️ Clear ALL waypoints? (y/N): ').strip().lower()
#         if confirm == 'y':
#             self.waypoints.clear()
#             self.save_waypoints()
#             print('✅ All waypoints cleared.')
#         else:
#             print('❌ Cancelled.')

#     def export_navigation_code(self):
#         """📤 Export waypoints for navigation code"""
#         if not self.waypoints:
#             print('📭 No waypoints to export.')
#             return
        
#         print('\n📤 Navigation Code:')
#         print('='*60)
#         print('# Precision waypoints from Static Recording')
#         print('waypoints = [')
        
#         for wp in self.waypoints:
#             comment = f"# {wp['name']} - {wp['quality']} ±{wp['confidence_radius']*100:.1f}cm"
#             print(f'    {{"x": {wp["x"]:.4f}, "y": {wp["y"]:.4f}}},  {comment}')
        
#         print(']')
#         print(f'# Total: {len(self.waypoints)} precision waypoints')
#         print('='*60)

#     def save_waypoints(self):
#         """💾 Save waypoints to file"""
#         try:
#             with open(self.waypoint_file, 'w') as f:
#                 json.dump(self.waypoints, f, indent=2)
#         except Exception as e:
#             self.get_logger().error(f'❌ Failed to save waypoints: {e}')

#     def load_waypoints(self):
#         """📂 Load waypoints from file"""
#         try:
#             with open(self.waypoint_file, 'r') as f:
#                 self.waypoints = json.load(f)
#             self.get_logger().info(f'📂 Loaded {len(self.waypoints)} waypoints')
#         except FileNotFoundError:
#             self.get_logger().info('📂 No existing waypoint file found')
#         except Exception as e:
#             self.get_logger().error(f'❌ Failed to load waypoints: {e}')

# def main(args=None):
#     rclpy.init(args=args)
    
#     try:
#         recorder = AdvancedWaypointRecorder()
#         rclpy.spin(recorder)
#     except KeyboardInterrupt:
#         print('\n🛑 Stopped by user')
#     finally:
#         if 'recorder' in locals():
#             recorder.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Point, PoseStamped
# from std_msgs.msg import Float32, String, Bool
# from nav_msgs.msg import Path
# import math
# import json
# import numpy as np
# import time
# import os
# from scipy.interpolate import interp1d
# from scipy.signal import savgol_filter

# class WaypointManager(Node):
#     def __init__(self):
#         super().__init__('waypoint_manager')
        
#         # Subscribers
#         self.position_sub = self.create_subscription(
#             Point, '/navigation/xy_position', self.position_callback, 10)
#         self.heading_sub = self.create_subscription(
#             Float32, '/navigation/heading', self.heading_callback, 10)
#         self.velocity_sub = self.create_subscription(
#             Float32, '/navigation/velocity', self.velocity_callback, 10)
#         self.gnss_status_sub = self.create_subscription(
#             String, '/navigation/gnss_status', self.gnss_status_callback, 10)
        
#         # Service-like subscribers for commands
#         self.command_sub = self.create_subscription(
#             String, '/waypoint/command', self.command_callback, 10)
        
#         # Publishers
#         self.waypoint_pub = self.create_publisher(Point, '/waypoint/current', 10)
#         self.path_pub = self.create_publisher(Path, '/waypoint/path', 10)
#         self.status_pub = self.create_publisher(String, '/waypoint/status', 10)
#         self.recording_status_pub = self.create_publisher(Bool, '/waypoint/recording', 10)
        
#         # Current state
#         self.current_position = Point()
#         self.current_heading = 0.0
#         self.current_velocity = 0.0
#         self.position_valid = False
#         self.heading_valid = False
#         self.position_quality = "POOR"
        
#         # Recording state
#         self.recording = False
#         self.waypoints = []
#         self.last_recorded_position = None
#         self.recording_start_time = 0.0
        
#         # Recording parameters
#         self.min_distance_threshold = 0.5  # minimum distance between waypoints (meters)
#         self.min_time_threshold = 1.0     # minimum time between waypoints (seconds)
#         self.min_heading_change = 5.0     # minimum heading change to force waypoint (degrees)
#         self.min_velocity_for_recording = 0.2  # minimum velocity to record waypoints
        
#         # Path processing parameters
#         self.smoothing_window = 5
#         self.smoothing_poly_order = 2
        
#         # File management
#         self.waypoint_file = "waypoints.json"
#         self.backup_folder = "waypoint_backups"
        
#         # Statistics
#         self.stats = {
#             'total_waypoints': 0,
#             'recording_sessions': 0,
#             'total_distance': 0.0,
#             'recording_time': 0.0,
#             'avg_waypoint_spacing': 0.0
#         }
        
#         # Initialize
#         self.create_backup_folder()
#         self.load_existing_waypoints()
        
#         # Timers
#         self.create_timer(0.1, self.recording_loop)    # 10Hz for recording
#         self.create_timer(1.0, self.publish_status)    # 1Hz for status
        
#         self.get_logger().info('📍 Waypoint Manager initialized')
#         self.print_commands()
    
#     def create_backup_folder(self):
#         """Create backup folder if it doesn't exist"""
#         if not os.path.exists(self.backup_folder):
#             os.makedirs(self.backup_folder)
    
#     def print_commands(self):
#         """Print available commands"""
#         commands = """
#         📍 Waypoint Manager Commands:
        
#         Recording:
#         - start_recording: Begin recording waypoints
#         - stop_recording: Stop recording and save
#         - pause_recording: Pause recording (resume with start_recording)
        
#         File Operations:
#         - save: Save current waypoints to file
#         - load: Load waypoints from file
#         - clear: Clear all waypoints
#         - backup: Create backup of current waypoints
        
#         Path Processing:
#         - smooth: Apply smoothing to waypoints
#         - analyze: Analyze path quality
#         - optimize: Optimize waypoint spacing
#         - reverse: Reverse waypoint order
        
#         Manual Operations:
#         - add_current: Add current position as waypoint
#         - remove_last: Remove last waypoint
#         - set_speed <speed>: Set speed for all waypoints
        
#         Utilities:
#         - status: Show detailed status
#         - export_csv: Export waypoints to CSV
#         - import_csv <filename>: Import waypoints from CSV
        
#         Publish commands to: /waypoint/command (String message)
#         Example: ros2 topic pub /waypoint/command std_msgs/String "data: 'start_recording'"
#         """
#         self.get_logger().info(commands)
    
#     def load_existing_waypoints(self):
#         """Load existing waypoints if file exists"""
#         try:
#             if os.path.exists(self.waypoint_file):
#                 with open(self.waypoint_file, 'r') as f:
#                     data = json.load(f)
#                     self.waypoints = data.get('waypoints', [])
#                     if 'stats' in data:
#                         self.stats.update(data['stats'])
                
#                 self.get_logger().info(f'📂 Loaded {len(self.waypoints)} existing waypoints')
#                 self.publish_path()
#             else:
#                 self.get_logger().info('📂 No existing waypoint file found')
#         except Exception as e:
#             self.get_logger().error(f'❌ Failed to load waypoints: {e}')
    
#     def position_callback(self, msg):
#         """Handle position updates"""
#         self.current_position = msg
#         self.position_valid = True
    
#     def heading_callback(self, msg):
#         """Handle heading updates"""
#         self.current_heading = msg.data
#         self.heading_valid = True
    
#     def velocity_callback(self, msg):
#         """Handle velocity updates"""
#         self.current_velocity = msg.data
    
#     def gnss_status_callback(self, msg):
#         """Handle GNSS status for quality assessment"""
#         try:
#             status = json.loads(msg.data)
#             self.position_quality = status.get('position', {}).get('quality', 'POOR')
#         except Exception:
#             pass
    
#     def command_callback(self, msg):
#         """Handle waypoint commands"""
#         command = msg.data.strip().lower()
#         self.get_logger().info(f'📨 Received command: {command}')
        
#         try:
#             # Recording commands
#             if command == 'start_recording':
#                 self.start_recording()
#             elif command == 'stop_recording':
#                 self.stop_recording()
#             elif command == 'pause_recording':
#                 self.pause_recording()
            
#             # File operations
#             elif command == 'save':
#                 self.save_waypoints()
#             elif command == 'load':
#                 self.load_waypoints()
#             elif command == 'clear':
#                 self.clear_waypoints()
#             elif command == 'backup':
#                 self.create_backup()
            
#             # Path processing
#             elif command == 'smooth':
#                 self.smooth_path()
#             elif command == 'analyze':
#                 self.analyze_path()
#             elif command == 'optimize':
#                 self.optimize_waypoints()
#             elif command == 'reverse':
#                 self.reverse_waypoints()
            
#             # Manual operations
#             elif command == 'add_current':
#                 self.add_current_position()
#             elif command == 'remove_last':
#                 self.remove_last_waypoint()
#             elif command.startswith('set_speed'):
#                 parts = command.split()
#                 if len(parts) == 2:
#                     try:
#                         speed = float(parts[1])
#                         self.set_speed_all_waypoints(speed)
#                     except ValueError:
#                         self.get_logger().error('❌ Invalid speed value')
#                 else:
#                     self.get_logger().error('❌ Usage: set_speed <speed_value>')
            
#             # Utilities
#             elif command == 'status':
#                 self.print_detailed_status()
#             elif command == 'export_csv':
#                 self.export_to_csv()
#             elif command.startswith('import_csv'):
#                 parts = command.split()
#                 if len(parts) == 2:
#                     self.import_from_csv(parts[1])
#                 else:
#                     self.get_logger().error('❌ Usage: import_csv <filename>')
            
#             else:
#                 self.get_logger().warning(f'❓ Unknown command: {command}')
#                 self.print_commands()
        
#         except Exception as e:
#             self.get_logger().error(f'❌ Command execution failed: {e}')
    
#     def start_recording(self):
#         """Start recording waypoints"""
#         if not self.position_valid:
#             self.get_logger().error('❌ Cannot start recording - no valid position')
#             return
        
#         if self.position_quality not in ['EXCELLENT', 'GOOD', 'FAIR']:
#             self.get_logger().warning(f'⚠️ Recording with poor position quality: {self.position_quality}')
        
#         self.recording = True
#         self.recording_start_time = time.time()
#         self.last_recorded_position = None
        
#         # Add current position as first waypoint
#         self.add_current_position()
        
#         self.get_logger().info('🔴 Started recording waypoints')
#         self.stats['recording_sessions'] += 1
    
#     def stop_recording(self):
#         """Stop recording and save waypoints"""
#         if not self.recording:
#             self.get_logger().warning('⚠️ Not currently recording')
#             return
        
#         self.recording = False
#         recording_duration = time.time() - self.recording_start_time
#         self.stats['recording_time'] += recording_duration
        
#         # Add final position
#         self.add_current_position()
        
#         # Auto-save after recording
#         self.save_waypoints()
        
#         self.get_logger().info(f'⏹️ Stopped recording. Duration: {recording_duration:.1f}s, Waypoints: {len(self.waypoints)}')
    
#     def pause_recording(self):
#         """Pause recording"""
#         if self.recording:
#             self.recording = False
#             self.get_logger().info('⏸️ Recording paused')
#         else:
#             self.get_logger().warning('⚠️ Not currently recording')
    
#     def recording_loop(self):
#         """Main recording loop"""
#         if not self.recording or not self.position_valid:
#             return
        
#         # Check if vehicle is moving
#         if self.current_velocity < self.min_velocity_for_recording:
#             return
        
#         # Check if we should record a new waypoint
#         if self.should_record_waypoint():
#             self.add_current_position()
    
#     def should_record_waypoint(self):
#         """Determine if we should record a new waypoint"""
#         if self.last_recorded_position is None:
#             return True
        
#         current_time = time.time()
        
#         # Calculate distance from last recorded position
#         dx = self.current_position.x - self.last_recorded_position['x']
#         dy = self.current_position.y - self.last_recorded_position['y']
#         distance = math.sqrt(dx*dx + dy*dy)
        
#         # Calculate time since last waypoint
#         time_diff = current_time - self.last_recorded_position['timestamp']
        
#         # Calculate heading change
#         heading_change = abs(self.current_heading - self.last_recorded_position['heading'])
#         if heading_change > 180:
#             heading_change = 360 - heading_change
        
#         # Record if any condition is met
#         return (distance >= self.min_distance_threshold or 
#                 time_diff >= self.min_time_threshold or
#                 heading_change >= self.min_heading_change)
    
#     def add_current_position(self):
#         """Add current position as waypoint"""
#         if not self.position_valid or not self.heading_valid:
#             self.get_logger().error('❌ Cannot add waypoint - invalid position or heading')
#             return False
        
#         waypoint = {
#             'x': self.current_position.x,
#             'y': self.current_position.y,
#             'heading': self.current_heading,
#             'speed': min(max(self.current_velocity, 0.5), 3.0),  # Default speed range
#             'timestamp': time.time(),
#             'quality': self.position_quality
#         }
        
#         self.waypoints.append(waypoint)
#         self.last_recorded_position = waypoint
#         self.stats['total_waypoints'] = len(self.waypoints)
        
#         # Calculate total distance
#         if len(self.waypoints) > 1:
#             prev_wp = self.waypoints[-2]
#             distance = self.distance_between_points(prev_wp, waypoint)
#             self.stats['total_distance'] += distance
#             self.stats['avg_waypoint_spacing'] = self.stats['total_distance'] / (len(self.waypoints) - 1)
        
#         self.get_logger().debug(f'➕ Added waypoint {len(self.waypoints)}: ({waypoint["x"]:.2f}, {waypoint["y"]:.2f})')
        
#         # Publish updated path
#         self.publish_path()
        
#         return True
    
#     def remove_last_waypoint(self):
#         """Remove the last waypoint"""
#         if self.waypoints:
#             removed = self.waypoints.pop()
#             self.stats['total_waypoints'] = len(self.waypoints)
#             self.get_logger().info(f'➖ Removed waypoint: ({removed["x"]:.2f}, {removed["y"]:.2f})')
#             self.publish_path()
#         else:
#             self.get_logger().warning('⚠️ No waypoints to remove')
    
#     def clear_waypoints(self):
#         """Clear all waypoints"""
#         count = len(self.waypoints)
#         self.waypoints.clear()
#         self.stats['total_waypoints'] = 0
#         self.stats['total_distance'] = 0.0
#         self.stats['avg_waypoint_spacing'] = 0.0
#         self.get_logger().info(f'🗑️ Cleared {count} waypoints')
#         self.publish_path()
    
#     def save_waypoints(self):
#         """Save waypoints to file"""
#         try:
#             data = {
#                 'metadata': {
#                     'created_time': time.time(),
#                     'total_waypoints': len(self.waypoints),
#                     'total_distance': self.stats['total_distance'],
#                     'recording_sessions': self.stats['recording_sessions']
#                 },
#                 'waypoints': self.waypoints,
#                 'stats': self.stats
#             }
            
#             with open(self.waypoint_file, 'w') as f:
#                 json.dump(data, f, indent=2)
            
#             self.get_logger().info(f'💾 Saved {len(self.waypoints)} waypoints to {self.waypoint_file}')
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Failed to save waypoints: {e}')
    
#     def load_waypoints(self):
#         """Load waypoints from file"""
#         try:
#             if os.path.exists(self.waypoint_file):
#                 with open(self.waypoint_file, 'r') as f:
#                     data = json.load(f)
#                     self.waypoints = data.get('waypoints', [])
#                     if 'stats' in data:
#                         self.stats.update(data['stats'])
                
#                 self.get_logger().info(f'📁 Loaded {len(self.waypoints)} waypoints from {self.waypoint_file}')
#                 self.publish_path()
#             else:
#                 self.get_logger().error(f'❌ File {self.waypoint_file} not found')
        
#         except Exception as e:
#             self.get_logger().error(f'❌ Failed to load waypoints: {e}')
    
#     def create_backup(self):
#         """Create backup of current waypoints"""
#         try:
#             timestamp = time.strftime("%Y%m%d_%H%M%S")
#             backup_file = os.path.join(self.backup_folder, f'waypoints_backup_{timestamp}.json')
            
#             data = {
#                 'metadata': {
#                     'backup_time': time.time(),
#                     'original_file': self.waypoint_file,
#                     'total_waypoints': len(self.waypoints)
#                 },
#                 'waypoints': self.waypoints,
#                 'stats': self.stats
#             }
            
#             with open(backup_file, 'w') as f:
#                 json.dump(data, f, indent=2)
            
#             self.get_logger().info(f'💾 Created backup: {backup_file}')
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Failed to create backup: {e}')
    
#     def smooth_path(self):
#         """Apply smoothing to waypoints"""
#         if len(self.waypoints) < 5:
#             self.get_logger().warning('⚠️ Need at least 5 waypoints for smoothing')
#             return
        
#         try:
#             # Create backup before smoothing
#             original_waypoints = self.waypoints.copy()
            
#             # Extract coordinates
#             x_coords = [wp['x'] for wp in self.waypoints]
#             y_coords = [wp['y'] for wp in self.waypoints]
            
#             # Apply Savitzky-Golay filter
#             if len(x_coords) >= self.smoothing_window:
#                 x_smooth = savgol_filter(x_coords, self.smoothing_window, self.smoothing_poly_order)
#                 y_smooth = savgol_filter(y_coords, self.smoothing_window, self.smoothing_poly_order)
                
#                 # Update waypoints
#                 for i, wp in enumerate(self.waypoints):
#                     wp['x'] = float(x_smooth[i])
#                     wp['y'] = float(y_smooth[i])
                
#                 # Recalculate headings
#                 self.recalculate_headings()
                
#                 self.get_logger().info(f'✨ Applied smoothing to {len(self.waypoints)} waypoints')
#                 self.publish_path()
                
#             else:
#                 self.get_logger().warning('⚠️ Not enough waypoints for smoothing window')
                
#         except Exception as e:
#             self.get_logger().error(f'❌ Smoothing failed: {e}')
#             self.waypoints = original_waypoints  # Restore original
    
#     def recalculate_headings(self):
#         """Recalculate headings for all waypoints"""
#         for i in range(len(self.waypoints)):
#             if i < len(self.waypoints) - 1:
#                 # Calculate heading to next waypoint
#                 dx = self.waypoints[i+1]['x'] - self.waypoints[i]['x']
#                 dy = self.waypoints[i+1]['y'] - self.waypoints[i]['y']
                
#                 if dx != 0 or dy != 0:
#                     heading = math.degrees(math.atan2(dx, dy))
#                     if heading < 0:
#                         heading += 360
#                     self.waypoints[i]['heading'] = heading
#             else:
#                 # For last waypoint, use heading of previous waypoint
#                 if i > 0:
#                     self.waypoints[i]['heading'] = self.waypoints[i-1]['heading']
    
#     def optimize_waypoints(self):
#         """Optimize waypoint spacing"""
#         if len(self.waypoints) < 3:
#             self.get_logger().warning('⚠️ Need at least 3 waypoints for optimization')
#             return
        
#         try:
#             optimized = [self.waypoints[0]]  # Keep first waypoint
            
#             for i in range(1, len(self.waypoints) - 1):
#                 prev_wp = optimized[-1]
#                 curr_wp = self.waypoints[i]
#                 next_wp = self.waypoints[i + 1]
                
#                 # Calculate angle change
#                 angle1 = math.atan2(curr_wp['y'] - prev_wp['y'], curr_wp['x'] - prev_wp['x'])
#                 angle2 = math.atan2(next_wp['y'] - curr_wp['y'], next_wp['x'] - curr_wp['x'])
#                 angle_diff = abs(angle2 - angle1)
                
#                 if angle_diff > math.pi:
#                     angle_diff = 2 * math.pi - angle_diff
                
#                 # Keep waypoint if significant direction change or sufficient distance
#                 distance = self.distance_between_points(prev_wp, curr_wp)
                
#                 if angle_diff > math.radians(10) or distance > 2.0:
#                     optimized.append(curr_wp)
            
#             optimized.append(self.waypoints[-1])  # Keep last waypoint
            
#             removed_count = len(self.waypoints) - len(optimized)
#             self.waypoints = optimized
#             self.stats['total_waypoints'] = len(self.waypoints)
            
#             self.get_logger().info(f'⚡ Optimized path: removed {removed_count} waypoints, {len(self.waypoints)} remaining')
#             self.publish_path()
            
#         except Exception as e:
#             self.get_logger().error(f'❌ Optimization failed: {e}')
    
#     def reverse_waypoints(self):
#         """Reverse the order of waypoints"""
#         if self.waypoints:
#             self.waypoints.reverse()
#             self.recalculate_headings()
#             self.get_logger().info(f'🔄 Reversed {len(self.waypoints)} waypoints')
#             self.publish_path()
#         else:
#             self.get_logger().warning('⚠️ No waypoints to reverse')
    
#     def set_speed_all_waypoints(self, speed):
#         """Set speed for all waypoints"""
#         if not self.waypoints:
#             self.get_logger().warning('⚠️ No waypoints to modify')
#             return
        
#         for wp in self.waypoints:
#             wp['speed'] = speed
        
#         self.get_logger().info(f'🏃 Set speed to {speed} m/s for all {len(self.waypoints)} waypoints')
    
#     def analyze_path(self):
#         """Analyze path quality and characteristics"""
#         if len(self.waypoints) < 2:
#             self.get_logger().warning('⚠️ Need at least 2 waypoints for analysis')
#             return
        
#         # Calculate statistics
#         distances = []
#         heading_changes = []
#         speeds = []
        
#         for i in range(len(self.waypoints) - 1):
#             # Distance between waypoints
#             dist = self.distance_between_points(self.waypoints[i], self.waypoints[i+1])
#             distances.append(dist)
            
#             # Heading change
#             h1 = self.waypoints[i]['heading']
#             h2 = self.waypoints[i+1]['heading']
#             heading_diff = abs(h2 - h1)
#             if heading_diff > 180:
#                 heading_diff = 360 - heading_diff
#             heading_changes.append(heading_diff)
            
#             # Speed
#             speeds.append(self.waypoints[i]['speed'])
        
#         speeds.append(self.waypoints[-1]['speed'])  # Add last waypoint speed
        
#         # Analysis results
#         analysis = {
#             'total_waypoints': len(self.waypoints),
#             'total_distance': sum(distances),
#             'avg_distance_between_waypoints': np.mean(distances),
#             'min_distance_between_waypoints': min(distances),
#             'max_distance_between_waypoints': max(distances),
#             'avg_heading_change': np.mean(heading_changes),
#             'max_heading_change': max(heading_changes),
#             'avg_speed': np.mean(speeds),
#             'min_speed': min(speeds),
#             'max_speed': max(speeds),
#             'estimated_travel_time': sum(d/s for d, s in zip(distances, speeds[:-1])) if all(s > 0 for s in speeds[:-1]) else 0
#         }
        
#         # Log analysis
#         self.get_logger().info('📊 Path Analysis:')
#         self.get_logger().info(f'   Total waypoints: {analysis["total_waypoints"]}')
#         self.get_logger().info(f'   Total distance: {analysis["total_distance"]:.2f} m')
#         self.get_logger().info(f'   Avg waypoint spacing: {analysis["avg_distance_between_waypoints"]:.2f} m')
#         self.get_logger().info(f'   Max heading change: {analysis["max_heading_change"]:.1f}°')
#         self.get_logger().info(f'   Avg speed: {analysis["avg_speed"]:.2f} m/s')
#         self.get_logger().info(f'   Estimated travel time: {analysis["estimated_travel_time"]:.1f} s')
        
#         return analysis
    
#     def export_to_csv(self):
#         """Export waypoints to CSV file"""
#         try:
#             import csv
            
#             csv_file = self.waypoint_file.replace('.json', '.csv')
            
#             with open(csv_file, 'w', newline='') as csvfile:
#                 fieldnames = ['waypoint_id', 'x', 'y', 'heading', 'speed', 'timestamp', 'quality']
#                 writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                
#                 writer.writeheader()
#                 for i, wp in enumerate(self.waypoints):
#                     writer.writerow({
#                         'waypoint_id': i,
#                         'x': wp['x'],
#                         'y': wp['y'],
#                         'heading': wp['heading'],
#                         'speed': wp['speed'],
#                         'timestamp': wp.get('timestamp', ''),
#                         'quality': wp.get('quality', '')
#                     })
            
#             self.get_logger().info(f'📤 Exported {len(self.waypoints)} waypoints to {csv_file}')
            
#         except Exception as e:
#             self.get_logger().error(f'❌ CSV export failed: {e}')
    
#     def import_from_csv(self, filename):
#         """Import waypoints from CSV file"""
#         try:
#             import csv
            
#             if not os.path.exists(filename):
#                 self.get_logger().error(f'❌ CSV file {filename} not found')
#                 return
            
#             new_waypoints = []
            
#             with open(filename, 'r') as csvfile:
#                 reader = csv.DictReader(csvfile)
#                 for row in reader:
#                     waypoint = {
#                         'x': float(row['x']),
#                         'y': float(row['y']),
#                         'heading': float(row['heading']),
#                         'speed': float(row['speed']),
#                         'timestamp': row.get('timestamp', time.time()),
#                         'quality': row.get('quality', 'UNKNOWN')
#                     }
#                     new_waypoints.append(waypoint)
            
#             self.waypoints = new_waypoints
#             self.stats['total_waypoints'] = len(self.waypoints)
            
#             self.get_logger().info(f'📥 Imported {len(self.waypoints)} waypoints from {filename}')
#             self.publish_path()
            
#         except Exception as e:
#             self.get_logger().error(f'❌ CSV import failed: {e}')
    
#     def distance_between_points(self, p1, p2):
#         """Calculate distance between two waypoints"""
#         return math.sqrt((p2['x'] - p1['x'])**2 + (p2['y'] - p1['y'])**2)
    
#     def publish_path(self):
#         """Publish waypoint path for visualization"""
#         if not self.waypoints:
#             return
        
#         path_msg = Path()
#         path_msg.header.stamp = self.get_clock().now().to_msg()
#         path_msg.header.frame_id = "map"
        
#         for i, wp in enumerate(self.waypoints):
#             pose = PoseStamped()
#             pose.header = path_msg.header
#             pose.pose.position.x = wp['x']
#             pose.pose.position.y = wp['y']
#             pose.pose.position.z = 0.0
            
#             # Set orientation from heading
#             heading_rad = math.radians(wp['heading'])
#             pose.pose.orientation.z = math.sin(heading_rad / 2)
#             pose.pose.orientation.w = math.cos(heading_rad / 2)
            
#             path_msg.poses.append(pose)
        
#         self.path_pub.publish(path_msg)
    
#     def print_detailed_status(self):
#         """Print detailed status information"""
#         status_text = f"""
#         📍 Waypoint Manager Status:
        
#         Current State:
#         - Position: ({self.current_position.x:.2f}, {self.current_position.y:.2f})
#         - Heading: {self.current_heading:.1f}°
#         - Velocity: {self.current_velocity:.2f} m/s
#         - Position Quality: {self.position_quality}
#         - Recording: {'🔴 ACTIVE' if self.recording else '⚪ INACTIVE'}
        
#         Waypoints:
#         - Total: {len(self.waypoints)}
#         - Total Distance: {self.stats['total_distance']:.2f} m
#         - Avg Spacing: {self.stats['avg_waypoint_spacing']:.2f} m
#         - Recording Sessions: {self.stats['recording_sessions']}
#         - Recording Time: {self.stats['recording_time']:.1f} s
        
#         Files:
#         - Main File: {self.waypoint_file}
#         - Backup Folder: {self.backup_folder}
#         """
        
#         self.get_logger().info(status_text)
    
#     def publish_status(self):
#         """Publish status message"""
#         status = {
#             'timestamp': time.time(),
#             'recording': self.recording,
#             'total_waypoints': len(self.waypoints),
#             'current_position': {
#                 'x': self.current_position.x,
#                 'y': self.current_position.y
#             },
#             'current_heading': self.current_heading,
#             'current_velocity': self.current_velocity,
#             'position_quality': self.position_quality,
#             'position_valid': self.position_valid,
#             'heading_valid': self.heading_valid,
#             'stats': self.stats,
#             'files': {
#                 'waypoint_file': self.waypoint_file,
#                 'backup_folder': self.backup_folder
#             }
#         }
        
#         status_msg = String()
#         status_msg.data = json.dumps(status, default=str)
#         self.status_pub.publish(status_msg)
        
#         # Publish recording status
#         recording_msg = Bool()
#         recording_msg.data = self.recording
#         self.recording_status_pub.publish(recording_msg)


# def main(args=None):
#     rclpy.init(args=args)
#     waypoint_manager = WaypointManager()
    
#     try:
#         rclpy.spin(waypoint_manager)
#     except KeyboardInterrupt:
#         waypoint_manager.get_logger().info("🛑 Keyboard interrupt")
#     finally:
#         waypoint_manager.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
