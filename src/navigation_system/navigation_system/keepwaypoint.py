# #!/usr/bin/env python3

# import serial
# import pynmea2
# import socket
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
# import pyproj
# import pandas as pd
# import os

# class GNSSPublisher(Node):
#     def __init__(self):
#         super().__init__('gnss_publisher')
        
#         # ROS2 Publisher
#         self.gnss_publisher = self.create_publisher(
#             NavSatFix,
#             '/navigation/gnss',
#             10)

#         # Serial port config
#         self.serial_port = '/dev/ttyACM0'  # ปรับพอร์ตให้ตรงกับ u-blox GNSS
#         try:
#             self.ser = serial.Serial(self.serial_port, 9600, timeout=1)
#             self.get_logger().info(f"✅ Connected to {self.serial_port}")
#         except Exception as e:
#             self.get_logger().error(f"❌ Error opening serial port: {e}")
#             return

#         # NTRIP config
#         self.ntrip_server_ip = "110.78.0.54"
#         self.ntrip_server_port = 2116
#         self.ntrip_username = "1118600009224"
#         self.ntrip_password = "CK79"
#         self.mount_point = "VRS_RTCM32"
#         self.ntrip_socket = self.connect_to_ntrip()

#         # UTM transformer (ใช้โซน 47N → EPSG:32647)
#         self.transformer = pyproj.Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)

#         # CSV file path
#         self.csv_file = '/home/inc/ros2_ws/waypointfile/waypoints3xy.csv'
#         if not os.path.isfile(self.csv_file):
#             df = pd.DataFrame(columns=['x_east', 'y_north'])
#             df.to_csv(self.csv_file, index=False)

#         # Timer for reading GNSS data
#         self.create_timer(0.01, self.read_gnss_data)  # 10 Hz
#         self.get_logger().info('🚀 GNSS Publisher initialized')

#     def connect_to_ntrip(self):
#         try:
#             client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#             client_socket.connect((self.ntrip_server_ip, self.ntrip_server_port))
#             auth_str = (
#                 f"GET /{self.mount_point} HTTP/1.1\r\n"
#                 f"User-Agent: NTRIP Client\r\n"
#                 f"Authorization: Basic {self.ntrip_username}:{self.ntrip_password}\r\n\r\n"
#             )
#             client_socket.send(auth_str.encode('ascii'))
#             self.get_logger().info("✅ Connected to NTRIP server")
#             return client_socket
#         except Exception as e:
#             self.get_logger().error(f"❌ Error connecting to NTRIP server: {e}")
#             return None

#     def read_gnss_data(self):
#         if hasattr(self, 'ser') and self.ser.is_open:
#             try:
#                 data = self.ser.readline().decode('ascii', errors='replace')

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
#                             # Publish to ROS2 topic
#                             nav_msg = NavSatFix()
#                             nav_msg.latitude = latitude
#                             nav_msg.longitude = longitude
#                             self.gnss_publisher.publish(nav_msg)
#                             self.get_logger().info(f"📡 GNSS: lat={latitude}, lon={longitude}")

#                             # Transform to UTM
#                             x_east, y_north = self.transformer.transform(longitude, latitude)

#                             # Append to CSV
#                             df = pd.DataFrame([[x_east, y_north]], columns=['x_east', 'y_north'])
#                             df.to_csv(self.csv_file, mode='a', header=False, index=False)
#                             self.get_logger().info(f"📝 Logged UTM: {x_east:.4f}, {y_north:.4f}")

#                             # Forward RTK corrections from NTRIP to GNSS device
#                             if self.ntrip_socket:
#                                 try:
#                                     self.ntrip_socket.setblocking(0)
#                                     try:
#                                         rtk_data = self.ntrip_socket.recv(1024)
#                                         if rtk_data:
#                                             self.ser.write(rtk_data)
#                                     except socket.error:
#                                         pass
#                                 except Exception as e:
#                                     self.get_logger().error(f"RTK data error: {e}")

#                     except pynmea2.nmea.ChecksumError:
#                         self.get_logger().warning("⚠️ Checksum error in NMEA data")

#             except Exception as e:
#                 self.get_logger().error(f"Serial read error: {e}")

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
# import pyproj
# import pandas as pd
# import os
# import time
# import math

# GNSS_PORT = '/dev/ttyACM0'
# GNSS_BAUD = 9600
# IMU_PORT = '/dev/ttyACM1'
# IMU_BAUD = 115200

# csv_file = 'waypoints_with_speed_heading2.csv'

# if not os.path.isfile(csv_file):
#     df = pd.DataFrame(columns=['timestamp', 'x_east', 'y_north', 'speed_mps', 'imu_heading'])
#     df.to_csv(csv_file, index=False)

# transformer = pyproj.Transformer.from_crs('epsg:4326', 'epsg:32647', always_xy=True)

# try:
#     gnss_ser = serial.Serial(GNSS_PORT, GNSS_BAUD, timeout=0.1)  # ลด timeout ให้เร็วขึ้น
#     imu_ser = serial.Serial(IMU_PORT, IMU_BAUD, timeout=0.1)
# except Exception as e:
#     print(f"Error opening serial ports: {e}")
#     exit(1)

# prev_x = None
# prev_y = None
# prev_time = None
# latest_heading = None

# def utm_distance(x1, y1, x2, y2):
#     return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# try:
#     while True:
#         # --- อ่านค่า IMU heading ให้เร็วที่สุด ---
#         try:
#             imu_line = imu_ser.readline().decode('utf-8').strip()
#             if imu_line:
#                 print(f"IMU raw line: {imu_line}")
#                 latest_heading = float(imu_line)
#         except Exception as e:
#             print(f"IMU Read error: {e}")

#         # --- อ่าน GNSS (ถี่ขึ้น เพราะ timeout ต่ำ) ---
#         try:
#             gnss_line = gnss_ser.readline().decode('ascii', errors='replace')
#             if gnss_line:
#                 print(f"GNSS raw line: {gnss_line.strip()}")
#             if gnss_line.startswith('$GPRMC') or gnss_line.startswith('$GNRMC'):
#                 msg = pynmea2.parse(gnss_line)
#                 latitude = msg.latitude
#                 longitude = msg.longitude
#                 speed_knots = float(msg.spd_over_grnd or 0)
#                 speed_mps = speed_knots * 0.514444

#                 if latitude == 0.0 or longitude == 0.0:
#                     continue

#                 x_east, y_north = transformer.transform(longitude, latitude)
#                 timestamp = time.time()

#                 if speed_mps == 0.0 and prev_x is not None and prev_y is not None and prev_time is not None:
#                     dt = timestamp - prev_time
#                     dist = utm_distance(prev_x, prev_y, x_east, y_north)
#                     if dt > 0:
#                         speed_mps = dist / dt

#                 # ----- เก็บทุกครั้งที่มีข้อมูลใหม่ -----
#                 if latest_heading is not None:
#                     df = pd.DataFrame([[timestamp, x_east, y_north, speed_mps, latest_heading]],
#                                       columns=['timestamp', 'x_east', 'y_north', 'speed_mps', 'imu_heading'])
#                     df.to_csv(csv_file, mode='a', header=False, index=False)
#                     print(f'Logged: x={x_east:.2f}, y={y_north:.2f}, speed={speed_mps:.2f}, heading={latest_heading:.2f}')

#                 prev_x = x_east
#                 prev_y = y_north
#                 prev_time = timestamp

#         except pynmea2.nmea.ParseError as e:
#             print(f"NMEA parse error: {e}")
#         except Exception as e:
#             print(f"GNSS Read error: {e}")

# except KeyboardInterrupt:
#     print('Exit')
# finally:
#     if gnss_ser.is_open:
#         gnss_ser.close()
#     if imu_ser.is_open:
#         imu_ser.close()




# import serial
# import pynmea2
# import pyproj
# import pandas as pd
# import os
# import time
# import math

# GNSS_PORT = '/dev/ttyACM0'
# GNSS_BAUD = 9600
# IMU_PORT = '/dev/ttyACM1'
# IMU_BAUD = 115200

# csv_file = 'waypoints_with_cumulative_time.csv'

# # สร้างไฟล์ CSV ถ้ายังไม่มี
# if not os.path.isfile(csv_file):
#     df = pd.DataFrame(columns=['cumulative_time', 'x_east', 'y_north', 'speed_mps', 'imu_heading'])
#     df.to_csv(csv_file, index=False)

# transformer = pyproj.Transformer.from_crs('epsg:4326', 'epsg:32647', always_xy=True)

# try:
#     gnss_ser = serial.Serial(GNSS_PORT, GNSS_BAUD, timeout=0.1)
#     imu_ser = serial.Serial(IMU_PORT, IMU_BAUD, timeout=0.1)
# except Exception as e:
#     print(f"Error opening serial ports: {e}")
#     exit(1)

# prev_x = None
# prev_y = None
# prev_time = None
# latest_heading = None
# cumulative_time = 0.0  # ตัวแปรเก็บ cumulative time

# def utm_distance(x1, y1, x2, y2):
#     return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# try:
#     while True:
#         # --- อ่านค่า IMU heading ---
#         try:
#             imu_line = imu_ser.readline().decode('utf-8').strip()
#             if imu_line:
#                 latest_heading = float(imu_line)
#         except Exception as e:
#             print(f"IMU Read error: {e}")

#         # --- อ่าน GNSS ---
#         try:
#             gnss_line = gnss_ser.readline().decode('ascii', errors='replace')
#             if gnss_line.startswith('$GPRMC') or gnss_line.startswith('$GNRMC'):
#                 msg = pynmea2.parse(gnss_line)
#                 latitude = msg.latitude
#                 longitude = msg.longitude
#                 speed_knots = float(msg.spd_over_grnd or 0)
#                 speed_mps = speed_knots * 0.514444

#                 if latitude == 0.0 or longitude == 0.0:
#                     continue

#                 x_east, y_north = transformer.transform(longitude, latitude)
#                 current_time = time.time()

#                 # --- คำนวณ dt และ cumulative_time ---
#                 if prev_time is None:
#                     dt = 0.0
#                 else:
#                     dt = current_time - prev_time

#                 cumulative_time += dt  # บวกเวลาที่ผ่านไป

#                 if speed_mps == 0.0 and prev_x is not None and prev_y is not None and dt > 0:
#                     dist = utm_distance(prev_x, prev_y, x_east, y_north)
#                     speed_mps = dist / dt

#                 # ----- เก็บข้อมูล -----
#                 if latest_heading is not None:
#                     df = pd.DataFrame([[cumulative_time, x_east, y_north, speed_mps, latest_heading]],
#                                       columns=['cumulative_time', 'x_east', 'y_north', 'speed_mps', 'imu_heading'])
#                     df.to_csv(csv_file, mode='a', header=False, index=False)
#                     print(f'Logged: t={cumulative_time:.2f}s, x={x_east:.2f}, y={y_north:.2f}, speed={speed_mps:.2f}, heading={latest_heading:.2f}')

#                 prev_x = x_east
#                 prev_y = y_north
#                 prev_time = current_time

#         except pynmea2.nmea.ParseError as e:
#             print(f"NMEA parse error: {e}")
#         except Exception as e:
#             print(f"GNSS Read error: {e}")

# except KeyboardInterrupt:
#     print('Exit')
# finally:
#     if gnss_ser.is_open:
#         gnss_ser.close()
#     if imu_ser.is_open:
#         imu_ser.close()

# ---------------------------------------XY UTM IMU + Arduino--------------------------------------------------

# import serial
# import pynmea2
# import pyproj
# import pandas as pd
# import os
# import time

# GNSS_PORT = '/dev/ttyACM0'
# GNSS_BAUD = 9600
# IMU_PORT = '/dev/ttyACM1'
# IMU_BAUD = 115200

# csv_file = 'waypoints_gnss_speed3.csv'

# # สร้างไฟล์ CSV ถ้ายังไม่มี
# if not os.path.isfile(csv_file):
#     df = pd.DataFrame(columns=['cumulative_time', 'x_east', 'y_north', 'speed_mps', 'imu_heading'])
#     df.to_csv(csv_file, index=False)

# transformer = pyproj.Transformer.from_crs('epsg:4326', 'epsg:32647', always_xy=True)

# try:
#     gnss_ser = serial.Serial(GNSS_PORT, GNSS_BAUD, timeout=0.1)
#     imu_ser = serial.Serial(IMU_PORT, IMU_BAUD, timeout=0.1)
# except Exception as e:
#     print(f"Error opening serial ports: {e}")
#     exit(1)

# prev_time = None
# latest_heading = None
# cumulative_time = 0.0

# try:
#     while True:
#         # อ่านค่า IMU heading
#         try:
#             imu_line = imu_ser.readline().decode('utf-8').strip()
#             if imu_line:
#                 latest_heading = float(imu_line)
#         except Exception as e:
#             print(f"IMU Read error: {e}")

#         # อ่าน GNSS
#         try:
#             gnss_line = gnss_ser.readline().decode('ascii', errors='replace')
#             if gnss_line.startswith('$GPRMC') or gnss_line.startswith('$GNRMC'):
#                 msg = pynmea2.parse(gnss_line)
#                 latitude = msg.latitude
#                 longitude = msg.longitude
#                 speed_knots = float(msg.spd_over_grnd or 0)
#                 speed_mps = speed_knots * 0.514444  # แปลงเป็น m/s

#                 if latitude == 0.0 or longitude == 0.0:
#                     continue

#                 x_east, y_north = transformer.transform(longitude, latitude)
#                 current_time = time.time()

#                 # คำนวณ cumulative_time
#                 if prev_time is None:
#                     dt = 0.0
#                 else:
#                     dt = current_time - prev_time

#                 cumulative_time += dt

#                 # ----- เก็บข้อมูลเฉพาะจาก GNSS -----
#                 if latest_heading is not None:
#                     df = pd.DataFrame([[cumulative_time, x_east, y_north, speed_mps, latest_heading]],
#                                       columns=['cumulative_time', 'x_east', 'y_north', 'speed_mps', 'imu_heading'])
#                     df.to_csv(csv_file, mode='a', header=False, index=False)
#                     print(f'Logged: t={cumulative_time:.2f}s, x={x_east:.2f}, y={y_north:.2f}, '
#                           f'speed={speed_mps:.2f} (GNSS), heading={latest_heading:.2f}')

#                 prev_time = current_time

#         except pynmea2.nmea.ParseError as e:
#             print(f"NMEA parse error: {e}")
#         except Exception as e:
#             print(f"GNSS Read error: {e}")

# except KeyboardInterrupt:
#     print('Exit')
# finally:
#     if gnss_ser.is_open:
#         gnss_ser.close()
#     if imu_ser.is_open:
#         imu_ser.close()



# -------------------------------XY mater IMU + Arduino-------------------------------------------------


# import serial
# import pynmea2
# import pyproj
# import pandas as pd
# import os
# import time

# GNSS_PORT = '/dev/ttyACM0'
# GNSS_BAUD = 9600
# IMU_PORT = '/dev/ttyACM1'
# IMU_BAUD = 115200

# csv_file = 'waypoints_gnss_speed3.csv'

# # สร้างไฟล์ CSV ถ้ายังไม่มี
# if not os.path.isfile(csv_file):
#     df = pd.DataFrame(columns=['cumulative_time', 'x_east', 'y_north', 'speed_mps', 'imu_heading'])
#     df.to_csv(csv_file, index=False)

# transformer = pyproj.Transformer.from_crs('epsg:4326', 'epsg:32647', always_xy=True)

# try:
#     gnss_ser = serial.Serial(GNSS_PORT, GNSS_BAUD, timeout=0.1)
#     imu_ser = serial.Serial(IMU_PORT, IMU_BAUD, timeout=0.1)
# except Exception as e:
#     print(f"Error opening serial ports: {e}")
#     exit(1)

# prev_time = None
# latest_heading = None
# cumulative_time = 0.0

# origin_set = False
# x_start, y_start = 0.0, 0.0

# try:
#     while True:
#         # อ่านค่า IMU heading
#         try:
#             imu_line = imu_ser.readline().decode('utf-8').strip()
#             if imu_line:
#                 latest_heading = float(imu_line)
#         except Exception as e:
#             print(f"IMU Read error: {e}")

#         # อ่าน GNSS
#         try:
#             gnss_line = gnss_ser.readline().decode('ascii', errors='replace')
#             if gnss_line.startswith('$GPRMC') or gnss_line.startswith('$GNRMC'):
#                 msg = pynmea2.parse(gnss_line)
#                 latitude = msg.latitude
#                 longitude = msg.longitude
#                 speed_knots = float(msg.spd_over_grnd or 0)
#                 speed_mps = speed_knots * 0.514444  # แปลงเป็น m/s

#                 if latitude == 0.0 or longitude == 0.0:
#                     continue

#                 x_east, y_north = transformer.transform(longitude, latitude)

#                 # กำหนดจุดเริ่มต้นของพิกัดเป็น 0
#                 if not origin_set:
#                     x_start = x_east
#                     y_start = y_north
#                     origin_set = True

#                 # แปลงพิกัดเป็น relative coordinate
#                 x_relative = x_east - x_start
#                 y_relative = y_north - y_start

#                 current_time = time.time()

#                 # คำนวณ cumulative_time
#                 if prev_time is None:
#                     dt = 0.0
#                 else:
#                     dt = current_time - prev_time

#                 cumulative_time += dt

#                 if latest_heading is not None:
#                     df = pd.DataFrame([[cumulative_time, x_relative, y_relative, speed_mps, latest_heading]],
#                                       columns=['cumulative_time', 'x_east', 'y_north', 'speed_mps', 'imu_heading'])
#                     df.to_csv(csv_file, mode='a', header=False, index=False)

#                     print(f'Logged: t={cumulative_time:.2f}s, x={x_relative:.2f}, y={y_relative:.2f}, '
#                           f'speed={speed_mps:.2f} (GNSS), heading={latest_heading:.2f}')

#                 prev_time = current_time

#         except pynmea2.nmea.ParseError as e:
#             print(f"NMEA parse error: {e}")
#         except Exception as e:
#             print(f"GNSS Read error: {e}")

# except KeyboardInterrupt:
#     print('Exit')

# finally:
#     if gnss_ser.is_open:
#         gnss_ser.close()
#     if imu_ser.is_open:
#         imu_ser.close()


# -----------------------------new model------------------------------------------

# import serial
# import pynmea2
# import pandas as pd
# import os
# import time
# import math

# # ----------------------------
# # พารามิเตอร์ระบบ
# GNSS_PORT = '/dev/ttyACM0'
# GNSS_BAUD = 9600
# csv_file = 'waypointsnewmodel3.csv'
# Ts = 0.5  # sampling [s]
# # ----------------------------

# # ถ้ายังไม่มีไฟล์ csv -> สร้าง header
# if not os.path.isfile(csv_file):
#     df = pd.DataFrame(columns=['time', 'x', 'y', 'psi', 'speed'])
#     df.to_csv(csv_file, index=False)

# # state เริ่มต้น
# x, y, psi = 0.0, 0.0, 0.0
# prev_time = time.time()
# start_time = prev_time   # เวลาที่เริ่มต้น log

# # เปิด serial GNSS
# gnss_ser = serial.Serial(GNSS_PORT, GNSS_BAUD, timeout=0.1)

# print("Start logging XY trajectory...")
# try:
#     while True:
#         line = gnss_ser.readline().decode('ascii', errors='replace').strip()
#         if not line.startswith(('$GPRMC', '$GNRMC')):
#             continue

#         try:
#             msg = pynmea2.parse(line)
#             if msg.status != "A":  # A = valid fix
#                 continue

#             # ความเร็ว (knots -> m/s)
#             v = float(msg.spd_over_grnd or 0) * 0.514444

#             # heading (deg -> rad)
#             if msg.true_course is None:
#                 continue
#             psi = math.radians(float(msg.true_course))

#             # เวลาปัจจุบัน
#             now = time.time()
#             dt = now - prev_time
#             if dt < Ts:
#                 continue
#             prev_time = now

#             # ----------------------------
#             # Update XY position
#             x = x + Ts * v * math.cos(psi)
#             y = y + Ts * v * math.sin(psi)
#             # ----------------------------

#             # เวลาสะสม (เริ่มจาก 0)
#             cumulative_time = now - start_time

#             # เซฟลง CSV
#             df = pd.DataFrame([[cumulative_time, x, y, psi, v]],
#                               columns=['time', 'x', 'y', 'psi', 'speed'])
#             df.to_csv(csv_file, mode='a', header=False, index=False)

#             print(f"t={cumulative_time:.1f}, x={x:.2f}, y={y:.2f}, v={v:.2f}, psi={psi:.2f}")

#         except Exception as e:
#             print(f"Parse error: {e}")

# except KeyboardInterrupt:
#     print("Stop logging")
# finally:
#     gnss_ser.close()

# ------------------------------new model V2--------------------------------------
# import serial
# import pynmea2
# import pandas as pd
# import os
# import time
# import math
# import numpy as np

# # ----------------------------
# # พารามิเตอร์ระบบ
# GNSS_PORT = '/dev/ttyACM0'
# GNSS_BAUD = 9600
# csv_file = 'waypointsnewmodel3.csv'
# Ts = 0.5        # sampling [s]
# V_MIN = 0.5     # [m/s] ถ้าช้ากว่านี้จะไม่อัปเดต heading
# ALPHA = 0.3     # smoothing heading
# # ----------------------------

# # ฟังก์ชัน wrap ให้อยู่ในช่วง -pi..pi
# def wrap_pi(a):
#     return (a + math.pi) % (2 * math.pi) - math.pi

# # ----------------------------
# # ถ้ายังไม่มีไฟล์ csv -> สร้าง header
# if not os.path.isfile(csv_file):
#     df = pd.DataFrame(columns=['time', 'x', 'y', 'psi', 'speed'])
#     df.to_csv(csv_file, index=False)

# # state เริ่มต้น
# x, y, psi = 0.0, 0.0, 0.0
# prev_time = time.time()
# start_time = prev_time   # เวลาที่เริ่มต้น log

# # เปิด serial GNSS
# gnss_ser = serial.Serial(GNSS_PORT, GNSS_BAUD, timeout=0.1)

# print("Start logging XY trajectory...")
# try:
#     while True:
#         line = gnss_ser.readline().decode('ascii', errors='replace').strip()
#         if not line.startswith(('$GPRMC', '$GNRMC')):
#             continue

#         try:
#             msg = pynmea2.parse(line)
#             if msg.status != "A":  # A = valid fix
#                 continue

#             # ความเร็ว (knots -> m/s)
#             v = float(msg.spd_over_grnd or 0) * 0.514444

#             # heading (deg -> rad)
#             if msg.true_course is None:
#                 continue
#             psi_meas = math.radians(float(msg.true_course))  # 0..2π
#             psi_meas = wrap_pi(psi_meas)                     # -π..π

#             # เวลาปัจจุบัน
#             now = time.time()
#             dt = now - prev_time
#             if dt < Ts:
#                 continue
#             prev_time = now

#             # ----------------------------
#             # อัปเดต heading (กรอง noise + velocity check)
#             if v >= V_MIN:
#                 d = wrap_pi(psi_meas - psi)    # ส่วนต่างมุมแบบสั้นที่สุด
#                 psi = wrap_pi(psi + ALPHA * d) # อัปเดตนิ่มๆ
#             # else: ใช้ psi เดิม

#             # Update XY position
#             x = x + Ts * v * math.cos(psi)
#             y = y + Ts * v * math.sin(psi)
#             # ----------------------------

#             # เวลาสะสม (เริ่มจาก 0)
#             cumulative_time = now - start_time

#             # เซฟลง CSV
#             df = pd.DataFrame([[cumulative_time, x, y, psi, v]],
#                               columns=['time', 'x', 'y', 'psi', 'speed'])
#             df.to_csv(csv_file, mode='a', header=False, index=False)

#             print(f"t={cumulative_time:.1f}, x={x:.2f}, y={y:.2f}, v={v:.2f}, psi={psi:.2f}")

#         except Exception as e:
#             print(f"Parse error: {e}")

# except KeyboardInterrupt:
#     print("Stop logging")
# finally:
#     gnss_ser.close()

# --------------------------------new model V3-----------------------------------------

# import serial
# import pynmea2
# import pandas as pd
# import os
# import time
# import math

# # ----------------------------
# # พารามิเตอร์ระบบ
# GNSS_PORT = '/dev/ttyACM0'
# GNSS_BAUD = 9600
# csv_file = 'waypointsnewmodel3.csv'
# Ts = 0.5  # sampling [s]
# # ----------------------------

# # ฟังก์ชัน wrap ให้อยู่ในช่วง -pi..pi
# def wrap_pi(a):
#     return (a + math.pi) % (2 * math.pi) - math.pi

# # ถ้ายังไม่มีไฟล์ csv -> สร้าง header
# if not os.path.isfile(csv_file):
#     df = pd.DataFrame(columns=['time', 'x', 'y', 'psi', 'speed'])
#     df.to_csv(csv_file, index=False)

# # state เริ่มต้น
# x, y, psi = 0.0, 0.0, 0.0
# prev_time = time.time()
# start_time = prev_time   # เวลาที่เริ่มต้น log

# # เปิด serial GNSS
# gnss_ser = serial.Serial(GNSS_PORT, GNSS_BAUD, timeout=0.1)

# print("Start logging XY trajectory...")
# try:
#     while True:
#         line = gnss_ser.readline().decode('ascii', errors='replace').strip()
#         if not line.startswith(('$GPRMC', '$GNRMC')):
#             continue

#         try:
#             msg = pynmea2.parse(line)
#             if msg.status != "A":  # A = valid fix
#                 continue

#             # ความเร็ว (knots -> m/s)
#             v = float(msg.spd_over_grnd or 0) * 0.514444

#             # heading (deg -> rad) และ wrap ให้อยู่ในช่วง -π ถึง π
#             if msg.true_course is None:
#                 continue
#             psi = math.radians(float(msg.true_course))
#             psi = wrap_pi(psi)

#             # เวลาปัจจุบัน
#             now = time.time()
#             dt = now - prev_time
#             if dt < Ts:
#                 continue
#             prev_time = now

#             # ----------------------------
#             # Update XY position
#             x = x + Ts * v * math.cos(psi)
#             y = y + Ts * v * math.sin(psi)
#             # ----------------------------

#             # เวลาสะสม (เริ่มจาก 0)
#             cumulative_time = now - start_time

#             # เซฟลง CSV
#             df = pd.DataFrame([[cumulative_time, x, y, psi, v]],
#                               columns=['time', 'x', 'y', 'psi', 'speed'])
#             df.to_csv(csv_file, mode='a', header=False, index=False)

#             print(f"t={cumulative_time:.1f}, x={x:.2f}, y={y:.2f}, v={v:.2f}, psi={psi:.2f}")

#         except Exception as e:
#             print(f"Parse error: {e}")

# except KeyboardInterrupt:
#     print("Stop logging")
# finally:
#     gnss_ser.close()




    # import serial
    # import pynmea2
    # import pandas as pd
    # import os
    # import time
    # import math
    # from pyproj import Transformer

    # # ----------------------------
    # GNSS_PORT = '/dev/ttyACM0'
    # GNSS_BAUD = 9600
    # csv_file = 'waypoints_utm.csv'
    # Ts = 0.5  # sampling [s]
    # # ----------------------------

    # # สร้าง transformer: WGS84 -> UTM zone 47N
    # transformer = Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)

    # # ถ้ายังไม่มีไฟล์ csv -> สร้าง header
    # if not os.path.isfile(csv_file):
    #     df = pd.DataFrame(columns=['time', 'x', 'y', 'psi', 'speed'])
    #     df.to_csv(csv_file, index=False)

    # prev_time = time.time()
    # start_time = prev_time

    # gnss_ser = serial.Serial(GNSS_PORT, GNSS_BAUD, timeout=0.1)

    # print("Start logging XY trajectory...")
    # try:
    #     while True:
    #         line = gnss_ser.readline().decode('ascii', errors='replace').strip()
    #         if not line.startswith(('$GPRMC', '$GNRMC')):
    #             continue

    #         try:
    #             msg = pynmea2.parse(line)
    #             if msg.status != "A":
    #                 continue

    #             # ความเร็ว (knots -> m/s)
    #             v = float(msg.spd_over_grnd or 0) * 0.514444

    #             # heading
    #             if msg.true_course is None:
    #                 continue
    #             psi = math.radians(float(msg.true_course))
    #             psi = (psi + math.pi) % (2*math.pi) - math.pi

    #             # เวลา
    #             now = time.time()
    #             if now - prev_time < Ts:
    #                 continue
    #             prev_time = now
    #             cumulative_time = now - start_time

    #             # ----------------------------
    #             # ใช้ lat/lon -> UTM
    #             lon = float(msg.lon) if msg.lon else 0.0
    #             lat = float(msg.lat) if msg.lat else 0.0
    #             x, y = transformer.transform(lon, lat)  # หน่วย: เมตร
    #             # ----------------------------

    #             # เซฟ CSV
    #             df = pd.DataFrame([[cumulative_time, x, y, psi, v]],
    #                             columns=['time', 'x', 'y', 'psi', 'speed'])
    #             df.to_csv(csv_file, mode='a', header=False, index=False)

    #             print(f"t={cumulative_time:.1f}, x={x:.2f}, y={y:.2f}, v={v:.2f}, psi={psi:.2f}")

    #         except Exception as e:
    #             print(f"Parse error: {e}")

    # except KeyboardInterrupt:
    #     print("Stop logging")
    # finally:
    #     gnss_ser.close()

        # -----------------------------------test-------------------------------------------

# import serial
# import pynmea2
# import pandas as pd
# import os
# import time
# import math
# from pyproj import Transformer

# # ----------------------------
# GNSS_PORT = '/dev/ttyACM0'
# GNSS_BAUD = 9600
# csv_file = 'waypoints_relative_xy.csv'
# Ts = 0.5  # sampling [s]
# # ----------------------------

# # ฟังก์ชันแปลง NMEA (ddmm.mmmm) -> Decimal Degrees
# def nmea_to_decimal(degree_str, direction):
#     """
#     degree_str: string ddmm.mmmm (เช่น '1345.6789')
#     direction: 'N','S','E','W'
#     """
#     if not degree_str or degree_str == '0':
#         return 0.0
#     raw = float(degree_str)
#     degrees = int(raw / 100)
#     minutes = raw - degrees * 100
#     decimal = degrees + minutes / 60
#     if direction in ['S', 'W']:
#         decimal = -decimal
#     return decimal

# # สร้าง transformer: WGS84 -> UTM zone 47N
# transformer = Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)

# # ตัวแปรสำหรับเก็บจุดอ้างอิง (จุดแรกที่ได้รับ)
# x0_utm, y0_utm = None, None
# first_fix = True

# # ถ้ายังไม่มีไฟล์ csv -> สร้าง header
# if not os.path.isfile(csv_file):
#     df = pd.DataFrame(columns=['time', 'x', 'y', 'psi', 'speed', 'xk+1', 'yk+1', 'utm_x', 'utm_y'])
#     df.to_csv(csv_file, index=False)

# prev_time = time.time()
# start_time = prev_time

# gnss_ser = serial.Serial(GNSS_PORT, GNSS_BAUD, timeout=0.1)

# print("Start logging XY trajectory with relative coordinates...")
# print("Waiting for first GPS fix to set origin (0,0)...")

# try:
#     while True:
#         line = gnss_ser.readline().decode('ascii', errors='replace').strip()
#         if not line.startswith(('$GPRMC', '$GNRMC')):
#             continue
        
#         try:
#             msg = pynmea2.parse(line)
#             if msg.status != "A":
#                 continue
            
#             # ความเร็ว (knots -> m/s)
#             v = float(msg.spd_over_grnd or 0) * 0.514444
            
#             # heading
#             if msg.true_course is None:
#                 continue
#             psi = math.radians(float(msg.true_course))
#             psi = (psi + math.pi) % (2 * math.pi) - math.pi
            
#             # เวลา
#             now = time.time()
#             if now - prev_time < Ts:
#                 continue
#             prev_time = now
#             cumulative_time = now - start_time
            
#             # ----------------------------
#             # แปลง NMEA -> decimal degrees
#             lat = nmea_to_decimal(msg.lat, msg.lat_dir)
#             lon = nmea_to_decimal(msg.lon, msg.lon_dir)
            
#             # ใช้ lat/lon -> UTM
#             utm_x, utm_y = transformer.transform(lon, lat)  # หน่วย: เมตร
            
#             # ตั้งจุดอ้างอิงครั้งแรก
#             if first_fix:
#                 x0_utm, y0_utm = utm_x, utm_y
#                 first_fix = False
#                 print(f"Origin set at UTM: ({x0_utm:.2f}, {y0_utm:.2f})")
#                 print("Now logging relative coordinates...")
            
#             # คำนวณพิกัดสัมพัทธ์ (x=0, y=0 ที่จุดเริ่มต้น)
#             x = utm_x - x0_utm
#             y = utm_y - y0_utm
#             # ----------------------------
            
#             # ----------------------------
#             # คำนวณ xk+1, yk+1 จาก model
#             x_next = x + Ts * v * math.cos(psi)
#             y_next = y + Ts * v * math.sin(psi)
#             # ----------------------------
            
#             # เซฟ CSV (เพิ่ม utm_x, utm_y สำหรับอ้างอิง)
#             df = pd.DataFrame([[cumulative_time, x, y, psi, v, x_next, y_next, utm_x, utm_y]],
#                             columns=['time', 'x', 'y', 'psi', 'speed', 'xk+1', 'yk+1', 'utm_x', 'utm_y'])
#             df.to_csv(csv_file, mode='a', header=False, index=False)
            
#             print(f"t={cumulative_time:.1f}, x={x:.2f}, y={y:.2f}, v={v:.2f}, psi={psi:.2f}, "
#                   f"xk+1={x_next:.2f}, yk+1={y_next:.2f}")
        
#         except Exception as e:
#             print(f"Parse error: {e}")

# except KeyboardInterrupt:
#     print("Stop logging")
# finally:
#     gnss_ser.close()
#     if not first_fix:
#         print(f"Origin was set at UTM: ({x0_utm:.2f}, {y0_utm:.2f})")



# import serial
# import pynmea2
# import pandas as pd
# import os
# import time
# import math
# from pyproj import Transformer

# # ----------------------------
# GNSS_PORT = '/dev/ttyACM0'
# GNSS_BAUD = 9600
# csv_file = 'waypoints_utmv2.csv'
# Ts = 0.5  # sampling [s]
# # ----------------------------

# # ฟังก์ชันแปลง NMEA (ddmm.mmmm) -> Decimal Degrees
# def nmea_to_decimal(degree_str, direction):
#     """
#     degree_str: string ddmm.mmmm (เช่น '1345.6789')
#     direction: 'N','S','E','W'
#     """
#     if not degree_str or degree_str == '0':
#         return 0.0
#     raw = float(degree_str)
#     degrees = int(raw / 100)
#     minutes = raw - degrees * 100
#     decimal = degrees + minutes / 60
#     if direction in ['S', 'W']:
#         decimal = -decimal
#     return decimal

# # สร้าง transformer: WGS84 -> UTM zone 47N
# transformer = Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)

# # ถ้ายังไม่มีไฟล์ csv -> สร้าง header
# if not os.path.isfile(csv_file):
#     df = pd.DataFrame(columns=['time', 'x_gps', 'y_gps', 'psi', 'speed', 'x_dr', 'y_dr'])
#     df.to_csv(csv_file, index=False)

# prev_time = time.time()
# start_time = prev_time

# gnss_ser = serial.Serial(GNSS_PORT, GNSS_BAUD, timeout=0.1)

# # ----------------------------
# # ค่าเริ่มต้นของ Dead Reckoning (เริ่มจาก GPS จุดแรก)
# x_dr, y_dr = None, None
# # ----------------------------

# print("Start logging XY trajectory...")#         except Exception as e:
# #             print(f"Parse error: {e}")

# # except KeyboardInterrupt:
# #     print("Stop logging")
# # finally:
# #     gnss_ser.close()
# #     if not first_fix:
# #         print(f"Origin was set at UTM: ({x0_utm:.2f}, {y0_utm:.2f})")



# import serial
# import pynmea2
# import pandas as pd
# import os
# import time
# import math
# from pyproj import Transformer

# # ----------------------------
# GNSS_PORT = '/dev/ttyACM0'
# GNSS_BAUD = 9600
# csv_file = 'waypoints_utmv2.csv'
# Ts = 0.5  # sampling [s]
# # ----------------------------

# # ฟังก์ชันแปลง NMEA (ddmm.mmmm) -> Decimal Degrees
# def nmea_to_decimal(degree_str, direction):
#     """
#     degree_str: string ddmm.mmmm (เช่น '1345.6789')
#     direction: 'N','S','E','W'
#     """
#     if not degree_str or degree_str == '0':
#         return 0.0
#     raw = float(degree_str)
#     degrees = int(raw / 100)
#     minutes = raw - degrees * 100
#     decimal = degrees + minutes / 60
#     if direction in ['S', 'W']:
#         decimal = -decimal
#     return decimal

# # สร้าง transformer: WGS84 -> UTM zone 47N
# transformer = Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)

# # ถ้ายังไม่มีไฟล์ csv -> สร้าง header
# if not os.path.isfile(csv_file):
#     df = pd.DataFrame(columns=['time', 'x_gps', 'y_gps', 'psi', 'speed', 'x_dr', 'y_dr'])
#     df.to_csv(csv_file, index=False)
# try:
#     while True:
#         line = gnss_ser.readline().decode('ascii', errors='replace').strip()
#         if not line.startswith(('$GPRMC', '$GNRMC')):
#             continue

#         try:
#             msg = pynmea2.parse(line)
#             if msg.status != "A":
#                 continue

#             # ความเร็ว (knots -> m/s)
#             v = float(msg.spd_over_grnd or 0) * 0.514444

#             # heading
#             if msg.true_course is None:
#                 continue
#             psi = math.radians(float(msg.true_course))
#             psi = (psi + math.pi) % (2 * math.pi) - math.pi

#             # เวลา
#             now = time.time()
#             if now - prev_time < Ts:
#                 continue
#             prev_time = now
#             cumulative_time = now - start_time

#             # ----------------------------
#             # แปลง NMEA -> decimal degrees (แก้ไขจากโค้ดเดิม)
#             lat = nmea_to_decimal(msg.lat, msg.lat_dir)
#             lon = nmea_to_decimal(msg.lon, msg.lon_dir)
            
#             # ใช้ lat/lon -> UTM (ค่าจริงจาก GPS)
#             x_gps, y_gps = transformer.transform(lon, lat)  # หน่วย: เมตร
#             # ----------------------------

#             # ----------------------------
#             # Dead Reckoning model
#             if x_dr is None or y_dr is None:
#                 # เริ่มต้นจาก GPS จุดแรก
#                 x_dr, y_dr = x_gps, y_gps
#             else:
#                 # คำนวณตำแหน่งถัดไปจาก model
#                 x_dr = x_dr + Ts * v * math.cos(psi)
#                 y_dr = y_dr + Ts * v * math.sin(psi)
#             # ----------------------------

#             # เซฟ CSV
#             df = pd.DataFrame([[cumulative_time, x_gps, y_gps, psi, v, x_dr, y_dr]],
#                               columns=['time', 'x_gps', 'y_gps', 'psi', 'speed', 'x_dr', 'y_dr'])
#             df.to_csv(csv_file, mode='a', header=False, index=False)

#             print(f"t={cumulative_time:.1f}, GPS=({x_gps:.2f}, {y_gps:.2f}), "
#                   f"v={v:.2f}, psi={psi:.2f}, DR=({x_dr:.2f}, {y_dr:.2f})")

#         except Exception as e:
#             print(f"Parse error: {e}")

# except KeyboardInterrupt:
#     print("Stop logging")
# finally:
#     gnss_ser.close()
#     if not first_fix:
#         print(f"Origin was set at UTM: ({x0_utm:.2f}, {y0_utm:.2f})")




# #!/usr/bin/env python3
# import serial
# import pynmea2
# import pandas as pd
# import os
# import time
# import math
# from pyproj import Transformer

# # ----------------------------
# GNSS_PORT  = '/dev/ttyACM0'
# GNSS_BAUD  = 9600
# csv_file   = 'waypoints2368.csv'
# MIN_SPEED  = 0.3   # m/s - ไม่เก็บตอนจอดนิ่ง
# MIN_DIST   = 1.0   # m   - ระยะห่างขั้นต่ำระหว่าง waypoint
# PSI_WINDOW = 5     # เฉลี่ย heading กี่ค่า
# # ----------------------------

# def nmea_to_decimal(degree_str, direction):
#     if not degree_str or degree_str == '0':
#         return 0.0
#     raw     = float(degree_str)
#     degrees = int(raw / 100)
#     minutes = raw - degrees * 100
#     decimal = degrees + minutes / 60
#     if direction in ['S', 'W']:
#         decimal = -decimal
#     return decimal

# def normalize_angle(a):
#     return (a + math.pi) % (2 * math.pi) - math.pi

# # WGS84 -> UTM zone 47N
# transformer = Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)

# # สร้าง CSV ถ้ายังไม่มี
# if not os.path.isfile(csv_file):
#     pd.DataFrame(columns=['time', 'x', 'y', 'psi', 'speed']).to_csv(csv_file, index=False)
#     print(f"✅ Created {csv_file}")

# start_time = time.time()
# prev_x     = None
# prev_y     = None
# prev_psi   = None
# psi_buffer = []
# count      = 0

# print(f"🔴 Recording to {csv_file} (Ctrl+C to stop)")
# print(f"   MIN_SPEED={MIN_SPEED} m/s | MIN_DIST={MIN_DIST} m")

# try:
#     gnss_ser = serial.Serial(GNSS_PORT, GNSS_BAUD, timeout=1)
#     print(f"✅ Opened {GNSS_PORT}\n")

#     while True:
#         line = gnss_ser.readline().decode('ascii', errors='replace').strip()

#         if not line.startswith(('$GPRMC', '$GNRMC')):
#             continue

#         try:
#             msg = pynmea2.parse(line)

#             if msg.status != "A":
#                 print("⚠️  Waiting for GPS fix...", end='\r')
#                 continue

#             t     = time.time() - start_time
#             speed = float(msg.spd_over_grnd or 0) * 0.514444

#             # กรอง 1: ไม่เก็บตอนจอดนิ่ง
#             if speed < MIN_SPEED:
#                 print(f"🅿️  Parked (speed={speed:.2f} m/s) - skip", end='\r')
#                 continue

#             # lat/lon → UTM ตรงๆ (ไม่ต้องลบ origin)
#             lat  = nmea_to_decimal(msg.lat, msg.lat_dir)
#             lon  = nmea_to_decimal(msg.lon, msg.lon_dir)
#             x, y = transformer.transform(lon, lat)

#             # กรอง 2: ต้องไกลจาก waypoint ก่อนพอ
#             if prev_x is not None:
#                 if math.hypot(x - prev_x, y - prev_y) < MIN_DIST:
#                     continue
#             prev_x, prev_y = x, y

#             # heading + smooth
#             if msg.true_course is None or msg.true_course == '':
#                 psi = prev_psi if prev_psi is not None else 0.0
#             else:
#                 psi      = normalize_angle(math.radians(float(msg.true_course)))
#                 prev_psi = psi

#             psi_buffer.append(psi)
#             if len(psi_buffer) > PSI_WINDOW:
#                 psi_buffer.pop(0)
#             psi_smooth = sum(psi_buffer) / len(psi_buffer)

#             # บันทึก
#             pd.DataFrame(
#                 [[t, x, y, psi_smooth, speed]],
#                 columns=['time', 'x', 'y', 'psi', 'speed']
#             ).to_csv(csv_file, mode='a', header=False, index=False)

#             count += 1
#             print(f"[{count:04d}] t={t:.1f}s | x={x:.2f} y={y:.2f} | psi={math.degrees(psi_smooth):.1f}° | speed={speed:.2f}m/s")

#         except Exception as e:
#             print(f"Parse error: {e}")

# except KeyboardInterrupt:
#     print(f"\n🛑 Stop recording — {count} waypoints saved")
# finally:
#     gnss_ser.close()
#     print(f"✅ Saved to {csv_file}")




# --------------------------------------------------waypoint saty-----------------------------------------

# #!/usr/bin/env python3
# import serial
# import pynmea2
# import csv
# import os
# import time
# import socket
# import base64
# import threading

# GNSS_PORT = '/dev/ttyACM0'
# GNSS_BAUD = 9600
# csv_file = "waypointssaty120368_mid.csv"

# # -------- NTRIP RTK ----------
# NTRIP_SERVER_IP = "110.78.0.54"
# NTRIP_SERVER_PORT = 2116
# NTRIP_USERNAME = "1118600009224"
# NTRIP_PASSWORD = "CK79"
# NTRIP_MOUNTPOINT = "VRS_RTCM32"
# # -----------------------------


# def nmea_to_decimal(degree_str, direction):

#     if not degree_str:
#         return 0.0

#     raw = float(degree_str)
#     degrees = int(raw / 100)
#     minutes = raw - degrees * 100

#     decimal = degrees + minutes / 60

#     if direction in ['S','W']:
#         decimal = -decimal

#     return decimal


# def connect_ntrip(serial_port):

#     auth = base64.b64encode(
#         f"{NTRIP_USERNAME}:{NTRIP_PASSWORD}".encode()
#     ).decode()

#     request = (
#         f"GET /{NTRIP_MOUNTPOINT} HTTP/1.0\r\n"
#         f"User-Agent: NTRIP PythonClient\r\n"
#         f"Authorization: Basic {auth}\r\n\r\n"
#     ).encode()

#     while True:

#         try:

#             sock = socket.create_connection(
#                 (NTRIP_SERVER_IP, NTRIP_SERVER_PORT)
#             )

#             sock.sendall(request)

#             print("✅ RTK Connected")

#             while True:

#                 data = sock.recv(4096)

#                 if not data:
#                     break

#                 serial_port.write(data)

#         except Exception as e:

#             print("NTRIP reconnect...", e)
#             time.sleep(5)


# # create csv
# if not os.path.isfile(csv_file):

#     with open(csv_file, 'w', newline='') as f:
#         writer = csv.writer(f)
#         writer.writerow(["lat", "lon", "speed", "heading"])


# try:

#     gnss_ser = serial.Serial(GNSS_PORT, GNSS_BAUD, timeout=1)

#     threading.Thread(
#         target=connect_ntrip,
#         args=(gnss_ser,),
#         daemon=True
#     ).start()

#     count = 0

#     print("🔴 Recording waypoints...")

#     while True:

#         line = gnss_ser.readline().decode('ascii', errors='replace').strip()

#         if not line.startswith(('$GPRMC', '$GNRMC')):
#             continue

#         try:

#             msg = pynmea2.parse(line)

#             if msg.status != "A":
#                 continue

#             lat = nmea_to_decimal(msg.lat, msg.lat_dir)
#             lon = nmea_to_decimal(msg.lon, msg.lon_dir)

#             speed = float(msg.spd_over_grnd or 0) * 0.514444
#             heading = float(msg.true_course or 0)

#             with open(csv_file, 'a', newline='') as f:

#                 writer = csv.writer(f)
#                 writer.writerow([lat, lon, speed, heading])

#             count += 1

#             print(
#                 f"[{count}] lat={lat:.8f} lon={lon:.8f} "
#                 f"speed={speed:.2f} heading={heading:.2f}"
#             )

#         except Exception as e:

#             print("parse error", e)

# except KeyboardInterrupt:

#     print("\n🛑 Stop recording")

# finally:

#     gnss_ser.close()


# #!/usr/bin/env python3
# import serial
# import pynmea2
# import csv
# import os
# import time
# import socket
# import base64
# import threading

# GNSS_PORT = '/dev/ttyACM0'
# GNSS_BAUD = 9600
# csv_file = "waypointssatyleft2.csv"

# # -------- NTRIP RTK ----------
# NTRIP_SERVER_IP = "110.78.0.54"
# NTRIP_SERVER_PORT = 2116
# NTRIP_USERNAME = "1118600009224"
# NTRIP_PASSWORD = "CK79"
# NTRIP_MOUNTPOINT = "VRS_RTCM32"
# # -----------------------------

# fix_status = 0


# def nmea_to_decimal(degree_str, direction):

#     if not degree_str:
#         return 0.0

#     raw = float(degree_str)
#     degrees = int(raw / 100)
#     minutes = raw - degrees * 100

#     decimal = degrees + minutes / 60

#     if direction in ['S','W']:
#         decimal = -decimal

#     return decimal


# def connect_ntrip(serial_port):

#     auth = base64.b64encode(
#         f"{NTRIP_USERNAME}:{NTRIP_PASSWORD}".encode()
#     ).decode()

#     request = (
#         f"GET /{NTRIP_MOUNTPOINT} HTTP/1.0\r\n"
#         f"User-Agent: NTRIP PythonClient\r\n"
#         f"Authorization: Basic {auth}\r\n\r\n"
#     ).encode()

#     while True:

#         try:

#             sock = socket.create_connection(
#                 (NTRIP_SERVER_IP, NTRIP_SERVER_PORT)
#             )

#             sock.sendall(request)

#             print("✅ RTK Connected")

#             while True:

#                 data = sock.recv(4096)

#                 if not data:
#                     break

#                 serial_port.write(data)

#         except Exception as e:

#             print("NTRIP reconnect...", e)
#             time.sleep(5)


# # create csv
# if not os.path.isfile(csv_file):

#     with open(csv_file, 'w', newline='') as f:
#         writer = csv.writer(f)
#         writer.writerow(["lat", "lon", "speed", "heading"])


# try:

#     gnss_ser = serial.Serial(GNSS_PORT, GNSS_BAUD, timeout=1)

#     threading.Thread(
#         target=connect_ntrip,
#         args=(gnss_ser,),
#         daemon=True
#     ).start()

#     count = 0

#     print("🔴 Recording waypoints (RTK Fixed only)...")

#     while True:

#         line = gnss_ser.readline().decode('ascii', errors='replace').strip()

#         if not line.startswith(('$GPRMC','$GNRMC','$GPGGA','$GNGGA')):
#             continue

#         try:

#             msg = pynmea2.parse(line)

#             # -------------------------
#             # อ่าน RTK status
#             # -------------------------
#             if isinstance(msg, pynmea2.types.talker.GGA):

#                 fix_status = int(msg.gps_qual)

#                 if fix_status == 4:
#                     print("🛰️ Fix: RTK Fixed ✅")

#                 elif fix_status == 5:
#                     print("🛰️ Fix: RTK Float 🔶")

#                 else:
#                     print("🛰️ Fix: No RTK ❌")


#             # -------------------------
#             # บันทึกตำแหน่ง
#             # -------------------------
#             if isinstance(msg, pynmea2.types.talker.RMC):

#                 if msg.status != "A":
#                     continue

#                 # เก็บเฉพาะ RTK Fixed
#                 if fix_status != 4:
#                     continue

#                 lat = nmea_to_decimal(msg.lat,msg.lat_dir)
#                 lon = nmea_to_decimal(msg.lon,msg.lon_dir)

#                 speed = float(msg.spd_over_grnd or 0) * 0.514444
#                 heading = float(msg.true_course or 0)

#                 with open(csv_file,'a',newline='') as f:

#                     writer = csv.writer(f)
#                     writer.writerow([lat,lon,speed,heading])

#                 count += 1

#                 print(
#                     f"[{count}] lat={lat:.8f} lon={lon:.8f} "
#                     f"speed={speed:.2f} heading={heading:.2f}"
#                 )

#         except Exception as e:

#             print("parse error",e)

# except KeyboardInterrupt:

#     print("\n🛑 Stop recording")

# finally:

#     gnss_ser.close()
# ----------------------------------------------------------------------------------#!/usr/bin/env python3
#!/usr/bin/env python3


# """
# waypoint_recorder.py
# =====================
# รอ RTK Fixed (quality=4) ก่อน แล้วค่อยเริ่มเก็บ waypoints
# พร้อมส่ง GGA กลับ NTRIP caster (VRS requirement)

# CSV format: index, x, y, yaw, speed
# """
# #!/usr/bin/env python3
# """
# waypoint_recorder.py — Fixed Version
# ======================================
# การแก้ไขหลัก:
#   1. แปลง GPS bearing → ENU yaw ก่อนบันทึก
#      yaw_enu = pi/2 - bearing_rad
#   2. บันทึก UTM absolute (ไม่ normalize ที่นี่)
#      เพราะ pure_pursuit_controller.py จะ normalize เอง
#      โดยใช้ waypoint[0] เป็น origin — สอดคล้องกัน
#   3. เพิ่ม speed smoothing (rolling average)
#   4. รอ RTK Fixed ก่อนเสมอ
# """

# import serial
# import pynmea2
# import csv
# import os
# import math
# import socket
# import base64
# import threading
# import time
# from pyproj import Transformer
 
# # ---------------- GNSS ----------------
# GNSS_PORT = '/dev/ttyACM0'
# GNSS_BAUD = 9600
 
# # ---------------- CSV ----------------
# CSV_FILE          = "/home/inc/ros2_ws/src/navigation_system/navigation_system/waypoints100368.csv"
# WAYPOINT_DISTANCE = 0.3   # ✅ เก็บทุก 0.3 ม. — แม่นขึ้นตอนโค้ง
 
# # ---------------- NTRIP RTK ----------------
# NTRIP_SERVER_IP   = "110.78.0.54"
# NTRIP_SERVER_PORT = 2116
# NTRIP_USERNAME    = "1118600009224"
# NTRIP_PASSWORD    = "CK79"
# NTRIP_MOUNTPOINT  = "VRS_RTCM32"
# GGA_SEND_INTERVAL = 5.0
# # -------------------------------------------
 
# MIN_RECORD_FIX    = 4     # บันทึกเฉพาะ RTK Fixed (quality=4) เท่านั้น
# MIN_SPEED_FOR_HDG = 0.3   # m/s — ต่ำกว่านี้ใช้ heading จาก position diff แทน
# SPEED_WINDOW      = 5     # rolling average window สำหรับ speed
 
# # ✅ ชดเชยตำแหน่ง antenna ที่ไม่ได้อยู่กึ่งกลางรถ
# # วัดระยะจากกึ่งกลางรถถึง antenna ในแนวซ้าย-ขวา
# # + = antenna อยู่ทางขวาของกึ่งกลาง
# # - = antenna อยู่ทางซ้ายของกึ่งกลาง
# # ถ้า antenna อยู่กึ่งกลางพอดี ใส่ 0.0
# ANTENNA_LATERAL_OFFSET = 0.0  # เมตร
 
# FIX_LABELS = {
#     0: "No fix ❌",
#     1: "GPS only ⚪",
#     2: "DGPS 🔵",
#     4: "RTK Fixed ✅",
#     5: "RTK Float 🔶",
# }
 
# transformer = Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)
 
# # ── shared state ──────────────────────────────────────────────────────────────
# last_x     = None   # UTM x ของ waypoint ล่าสุดที่บันทึก
# last_y     = None   # UTM y ของ waypoint ล่าสุดที่บันทึก
# prev_x_utm = None   # UTM x ก่อนหน้า (ใช้คำนวณ heading fallback)
# prev_y_utm = None
# index      = 0
# fix_qual   = 0
# last_gga   = None
# gga_lock   = threading.Lock()
# gnss_ser   = None
# speed_buf  = []
# last_yaw   = 0.0
 
# # สถิติคุณภาพ
# quality_counts = {}
 
 
# # ══════════════════════════════════════════════════════════════════════════════
# #  UTILS
# # ══════════════════════════════════════════════════════════════════════════════
# def nmea_to_decimal(degree_str, direction):
#     raw     = float(degree_str)
#     degrees = int(raw / 100)
#     minutes = raw - degrees * 100
#     decimal = degrees + minutes / 60
#     if direction in ('S', 'W'):
#         decimal = -decimal
#     return decimal
 
 
# def bearing_to_enu_yaw(bearing_deg: float) -> float:
#     """
#     แปลง GPS bearing → ENU/ROS yaw
#     GPS bearing : 0° = North, clockwise positive
#     ENU yaw     : 0° = East,  counter-clockwise positive
#     สูตร: yaw = π/2 − bearing_rad  แล้ว normalize ให้อยู่ใน [-π, π]
#     """
#     raw = math.pi / 2.0 - math.radians(bearing_deg)
#     return (raw + math.pi) % (2 * math.pi) - math.pi
 
 
# def apply_antenna_offset(x_utm: float, y_utm: float,
#                          yaw: float, lateral_offset: float):
#     """
#     ชดเชยตำแหน่ง GPS antenna ที่ไม่ได้อยู่กึ่งกลางรถ
#     lateral_offset: + = ขวา, - = ซ้าย (relative to vehicle heading)
#     """
#     if lateral_offset == 0.0:
#         return x_utm, y_utm
#     # เวกเตอร์ตั้งฉากกับทิศวิ่ง (perpendicular)
#     perp_yaw   = yaw + math.pi / 2.0
#     x_fixed    = x_utm - lateral_offset * math.cos(perp_yaw)
#     y_fixed    = y_utm - lateral_offset * math.sin(perp_yaw)
#     return x_fixed, y_fixed
 
 
# # ══════════════════════════════════════════════════════════════════════════════
# #  NTRIP thread — รับ RTCM + ส่ง GGA กลับ (VRS)
# # ══════════════════════════════════════════════════════════════════════════════
# def ntrip_client():
#     global gnss_ser
#     while True:
#         try:
#             print("🛰  Connecting to NTRIP...")
#             sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#             sock.settimeout(10)
#             sock.connect((NTRIP_SERVER_IP, NTRIP_SERVER_PORT))
 
#             auth = base64.b64encode(
#                 f"{NTRIP_USERNAME}:{NTRIP_PASSWORD}".encode()
#             ).decode()
 
#             with gga_lock:
#                 gga_now = last_gga
 
#             request = (
#                 f"GET /{NTRIP_MOUNTPOINT} HTTP/1.0\r\n"
#                 f"User-Agent: NTRIP PythonClient\r\n"
#                 f"Authorization: Basic {auth}\r\n"
#                 f"Ntrip-Version: Ntrip/1.0\r\n"
#             )
#             if gga_now:
#                 request += f"Ntrip-GGA: {gga_now}\r\n"
#             request += "\r\n"
 
#             sock.sendall(request.encode())
 
#             response = b""
#             while b"\r\n\r\n" not in response:
#                 chunk = sock.recv(256)
#                 if not chunk:
#                     break
#                 response += chunk
 
#             if b"200 OK" not in response and b"ICY 200 OK" not in response:
#                 print(f"❌ NTRIP error: {response[:200]}")
#                 sock.close()
#                 time.sleep(5)
#                 continue
 
#             print("✅ NTRIP connected — receiving RTCM...")
#             sock.settimeout(5)
#             last_sent = time.time()
 
#             while True:
#                 now = time.time()
#                 if now - last_sent >= GGA_SEND_INTERVAL:
#                     with gga_lock:
#                         gga = last_gga
#                     if gga:
#                         try:
#                             out = gga if gga.endswith('\r\n') else gga + '\r\n'
#                             sock.sendall(out.encode())
#                             print(f"📤 Sent GGA: {gga.strip()}")
#                         except Exception as e:
#                             print(f"GGA send error: {e}")
#                     last_sent = now
 
#                 try:
#                     data = sock.recv(4096)
#                 except socket.timeout:
#                     continue
#                 if not data:
#                     break
#                 gnss_ser.write(data)
 
#         except Exception as e:
#             print(f"❌ NTRIP error: {e}")
#         time.sleep(5)
 
 
# # ══════════════════════════════════════════════════════════════════════════════
# #  MAIN
# # ══════════════════════════════════════════════════════════════════════════════
# # สร้าง CSV ถ้ายังไม่มี
# if not os.path.isfile(CSV_FILE):
#     with open(CSV_FILE, 'w', newline='') as f:
#         # ✅ เพิ่ม fix_quality column เพื่อ QA ภายหลัง
#         csv.writer(f).writerow(["index", "x", "y", "yaw", "speed", "fix_quality"])
#     print(f"📄 Created new CSV: {CSV_FILE}")
# else:
#     print(f"📄 Appending to existing CSV: {CSV_FILE}")
 
# gnss_ser = serial.Serial(GNSS_PORT, GNSS_BAUD, timeout=1)
# print(f"✅ Opened {GNSS_PORT} @ {GNSS_BAUD} baud")
# print(f"📐 Mode: UTM Absolute (zone 47N)")
# print(f"📏 Waypoint spacing: {WAYPOINT_DISTANCE} m")
# if ANTENNA_LATERAL_OFFSET != 0.0:
#     side = "ขวา" if ANTENNA_LATERAL_OFFSET > 0 else "ซ้าย"
#     print(f"📡 Antenna offset: {abs(ANTENNA_LATERAL_OFFSET):.3f} ม. ทาง{side}")
 
# threading.Thread(target=ntrip_client, daemon=True).start()
# print("\n⏳ Waiting for RTK Fixed (quality=4)...\n")
 
# recording = False
 
# while True:
#     try:
#         line = gnss_ser.readline().decode('ascii', errors='replace').strip()
 
#         # ── GGA: อัปเดต fix quality ────────────────────────────────────────
#         if line.startswith(('$GPGGA', '$GNGGA')):
#             try:
#                 gga      = pynmea2.parse(line)
#                 fix_qual = int(gga.gps_qual)
 
#                 with gga_lock:
#                     last_gga = line
 
#                 quality_counts[fix_qual] = quality_counts.get(fix_qual, 0) + 1
#                 label = FIX_LABELS.get(fix_qual, f"quality={fix_qual}")
 
#                 # แสดงสถานะแบบ inline
#                 print(f"\r🛰️  {label}  | waypoints บันทึกแล้ว: {index}   ",
#                       end='', flush=True)
 
#                 if fix_qual >= MIN_RECORD_FIX and not recording:
#                     print(f"\n\n🟢 RTK Fixed! เริ่มบันทึก waypoints...\n")
#                     recording = True
 
#                 if fix_qual < MIN_RECORD_FIX and recording:
#                     print(f"\n\n⚠️  Fix หลุดเป็น {label} — หยุดบันทึกชั่วคราว\n")
#                     recording = False
 
#             except Exception:
#                 pass
#             continue
 
#         # ── RMC: บันทึก waypoint ───────────────────────────────────────────
#         if not line.startswith(('$GPRMC', '$GNRMC')):
#             continue
#         if not recording or fix_qual < MIN_RECORD_FIX:
#             continue
 
#         msg = pynmea2.parse(line)
#         if msg.status != 'A':
#             continue
 
#         lat     = nmea_to_decimal(msg.lat, msg.lat_dir)
#         lon     = nmea_to_decimal(msg.lon, msg.lon_dir)
 
#         # ✅ UTM absolute — ไม่ลบ origin ใดๆ
#         x_utm, y_utm = transformer.transform(lon, lat)
 
#         # ── Speed smoothing ────────────────────────────────────────────────
#         raw_speed = float(msg.spd_over_grnd or 0) * 0.514444  # knots → m/s
#         speed_buf.append(raw_speed)
#         if len(speed_buf) > SPEED_WINDOW:
#             speed_buf.pop(0)
#         speed = sum(speed_buf) / len(speed_buf)
 
#         # ── Heading calculation ────────────────────────────────────────────
#         if msg.true_course and msg.true_course != '' and speed >= MIN_SPEED_FOR_HDG:
#             # ✅ แปลง GPS bearing → ENU yaw
#             yaw      = bearing_to_enu_yaw(float(msg.true_course))
#             last_yaw = yaw
#         elif prev_x_utm is not None:
#             dx = x_utm - prev_x_utm
#             dy = y_utm - prev_y_utm
#             d  = math.hypot(dx, dy)
#             if d > 0.05:
#                 # ✅ atan2(dy, dx) ใน UTM ENU frame ถูกต้องโดยตรง
#                 yaw      = math.atan2(dy, dx)
#                 last_yaw = yaw
#             else:
#                 yaw = last_yaw
#         else:
#             yaw = last_yaw
 
#         # ── Antenna offset compensation ────────────────────────────────────
#         x_final, y_final = apply_antenna_offset(x_utm, y_utm, yaw,
#                                                  ANTENNA_LATERAL_OFFSET)
 
#         # ── กรองระยะขั้นต่ำ ────────────────────────────────────────────────
#         if last_x is not None:
#             d_from_last = math.hypot(x_final - last_x, y_final - last_y)
#             if d_from_last < WAYPOINT_DISTANCE:
#                 prev_x_utm = x_utm
#                 prev_y_utm = y_utm
#                 continue
 
#         # ── บันทึก UTM absolute ────────────────────────────────────────────
#         with open(CSV_FILE, 'a', newline='') as f:
#             csv.writer(f).writerow([
#                 index,
#                 f"{x_final:.4f}",   # ← UTM Easting  เช่น 700123.4567
#                 f"{y_final:.4f}",   # ← UTM Northing เช่น 1500456.7890
#                 f"{yaw:.6f}",       # ← ENU yaw (radians)
#                 f"{speed:.4f}",     # ← m/s (smoothed)
#                 fix_qual            # ← fix quality สำหรับ QA
#             ])
 
#         print(f"\n  [{index:4d}]  "
#               f"x={x_final:.3f}  y={y_final:.3f}  "
#               f"yaw={math.degrees(yaw):6.1f}°  "
#               f"speed={speed:.2f} m/s  "
#               f"fix={fix_qual}")
 
#         last_x     = x_final
#         last_y     = y_final
#         prev_x_utm = x_utm
#         prev_y_utm = y_utm
#         index     += 1
 
#     except KeyboardInterrupt:
#         total = sum(quality_counts.values()) or 1
#         print(f"\n\n{'='*55}")
#         print(f"🛑 หยุดบันทึก — {index} waypoints → {CSV_FILE}")
#         print(f"\n📊 สถิติคุณภาพ Fix :")
#         for q, label in [(4, "RTK Fixed ✅"), (5, "RTK Float 🔶"),
#                          (2, "DGPS 🔵"), (1, "GPS only ⚪"), (0, "No fix ❌")]:
#             n = quality_counts.get(q, 0)
#             if n > 0:
#                 print(f"   {label}: {n} ครั้ง ({100 * n // total}%)")
 
#         rtk_pct = 100 * quality_counts.get(4, 0) // total
#         print()
#         if rtk_pct >= 95:
#             print(f"✅ คุณภาพดีมาก ({rtk_pct}% RTK Fixed) — พร้อมใช้งาน")
#         elif rtk_pct >= 80:
#             print(f"⚠️  คุณภาพพอใช้ ({rtk_pct}% RTK Fixed) — ควรบันทึกใหม่บางช่วง")
#         else:
#             print(f"❌ คุณภาพต่ำ ({rtk_pct}% RTK Fixed) — แนะนำบันทึกใหม่ทั้งหมด")
#         print('='*55)
#         break
 
#     except Exception as e:
#         print(f"\nError: {e}")
 
# gnss_ser.close()
 



# #  ----------------------------------saty update------------------------------------------------
# import serial
# import pynmea2
# import csv
# import os
# import math
# import socket
# import base64
# import threading
# import time
# from pyproj import Transformer
 
# # ---------------- GNSS ----------------
# GNSS_PORT = '/dev/ttyACM0'
# GNSS_BAUD = 9600
 
# # ---------------- CSV ----------------
# CSV_FILE          = "/home/inc/ros2_ws/src/navigation_system/navigation_system/waypointaroundfootballground5.csv"
# WAYPOINT_DISTANCE = 0.3   # ✅ เก็บทุก 0.3 ม. — แม่นขึ้นตอนโค้ง
 
# # ---------------- NTRIP RTK ----------------
# NTRIP_SERVER_IP   = "110.78.0.54"
# NTRIP_SERVER_PORT = 2116
# NTRIP_USERNAME    = "1118600009224"
# NTRIP_PASSWORD    = "CK79"
# NTRIP_MOUNTPOINT  = "VRS_RTCM32"
# GGA_SEND_INTERVAL = 1.0
# # -------------------------------------------
 
# MIN_RECORD_FIX    = 4     # บันทึกเฉพาะ RTK Fixed (quality=4) เท่านั้น
# MIN_SPEED_FOR_HDG = 0.3   # m/s — ต่ำกว่านี้ใช้ heading จาก position diff แทน
# SPEED_WINDOW      = 5     # rolling average window สำหรับ speed
 
# # ✅ ชดเชยตำแหน่ง antenna ที่ไม่ได้อยู่กึ่งกลางรถ
# # วัดระยะจากกึ่งกลางรถถึง antenna ในแนวซ้าย-ขวา
# # + = antenna อยู่ทางขวาของกึ่งกลาง
# # - = antenna อยู่ทางซ้ายของกึ่งกลาง
# # ถ้า antenna อยู่กึ่งกลางพอดี ใส่ 0.0
# ANTENNA_LATERAL_OFFSET = 0.0  # เมตร
 
# FIX_LABELS = {
#     0: "No fix ❌",
#     1: "GPS only ⚪",
#     2: "DGPS 🔵",
#     4: "RTK Fixed ✅",
#     5: "RTK Float 🔶",
# }
 
# # ✅ ใช้ transformer ทั้งสองทิศทาง
# transformer_to_utm    = Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)
# transformer_to_latlon = Transformer.from_crs("epsg:32647", "epsg:4326", always_xy=True)
 
# # ── shared state ──────────────────────────────────────────────────────────────
# last_lat   = None   # lat ของ waypoint ล่าสุดที่บันทึก
# last_lon   = None   # lon ของ waypoint ล่าสุดที่บันทึก
# last_x_utm = None   # UTM x ของ waypoint ล่าสุด (ใช้คำนวณระยะ)
# last_y_utm = None   # UTM y ของ waypoint ล่าสุด
# prev_x_utm = None   # UTM x ก่อนหน้า (ใช้คำนวณ heading fallback)
# prev_y_utm = None
# index      = 0
# fix_qual   = 0
# last_gga   = None
# gga_lock   = threading.Lock()
# gnss_ser   = None
# speed_buf  = []
# last_yaw   = 0.0
 
# # สถิติคุณภาพ
# quality_counts = {}
 
 
# # ══════════════════════════════════════════════════════════════════════════════
# #  UTILS
# # ══════════════════════════════════════════════════════════════════════════════
# def nmea_to_decimal(degree_str, direction):
#     raw     = float(degree_str)
#     degrees = int(raw / 100)
#     minutes = raw - degrees * 100
#     decimal = degrees + minutes / 60
#     if direction in ('S', 'W'):
#         decimal = -decimal
#     return decimal
 
 
# def bearing_to_enu_yaw(bearing_deg: float) -> float:
#     """
#     แปลง GPS bearing → ENU/ROS yaw
#     GPS bearing : 0° = North, clockwise positive
#     ENU yaw     : 0° = East,  counter-clockwise positive
#     สูตร: yaw = π/2 − bearing_rad  แล้ว normalize ให้อยู่ใน [-π, π]
#     """
#     raw = math.pi / 2.0 - math.radians(bearing_deg)
#     return (raw + math.pi) % (2 * math.pi) - math.pi
 
 
# def apply_antenna_offset_utm(x_utm: float, y_utm: float,
#                               yaw: float, lateral_offset: float):
#     """
#     ชดเชยตำแหน่ง GPS antenna ที่ไม่ได้อยู่กึ่งกลางรถ (ทำงานใน UTM)
#     lateral_offset: + = ขวา, - = ซ้าย (relative to vehicle heading)
#     คืนค่า (x_utm_fixed, y_utm_fixed)
#     """
#     if lateral_offset == 0.0:
#         return x_utm, y_utm
#     perp_yaw = yaw + math.pi / 2.0
#     x_fixed  = x_utm - lateral_offset * math.cos(perp_yaw)
#     y_fixed  = y_utm - lateral_offset * math.sin(perp_yaw)
#     return x_fixed, y_fixed
 
 
# # ══════════════════════════════════════════════════════════════════════════════
# #  NTRIP thread — รับ RTCM + ส่ง GGA กลับ (VRS)
# # ══════════════════════════════════════════════════════════════════════════════
# def ntrip_client():
#     global gnss_ser
#     while True:
#         try:
#             print("🛰  Connecting to NTRIP...")
#             sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#             sock.settimeout(10)
#             sock.connect((NTRIP_SERVER_IP, NTRIP_SERVER_PORT))
 
#             auth = base64.b64encode(
#                 f"{NTRIP_USERNAME}:{NTRIP_PASSWORD}".encode()
#             ).decode()
 
#             with gga_lock:
#                 gga_now = last_gga
 
#             request = (
#                 f"GET /{NTRIP_MOUNTPOINT} HTTP/1.0\r\n"
#                 f"User-Agent: NTRIP PythonClient\r\n"
#                 f"Authorization: Basic {auth}\r\n"
#                 f"Ntrip-Version: Ntrip/1.0\r\n"
#             )
#             if gga_now:
#                 request += f"Ntrip-GGA: {gga_now}\r\n"
#             request += "\r\n"
 
#             sock.sendall(request.encode())
 
#             response = b""
#             while b"\r\n\r\n" not in response:
#                 chunk = sock.recv(256)
#                 if not chunk:
#                     break
#                 response += chunk
 
#             if b"200 OK" not in response and b"ICY 200 OK" not in response:
#                 print(f"❌ NTRIP error: {response[:200]}")
#                 sock.close()
#                 time.sleep(5)
#                 continue
 
#             print("✅ NTRIP connected — receiving RTCM...")
#             sock.settimeout(5)
#             last_sent = time.time()
 
#             while True:
#                 now = time.time()
#                 if now - last_sent >= GGA_SEND_INTERVAL:
#                     with gga_lock:
#                         gga = last_gga
#                     if gga:
#                         try:
#                             out = gga if gga.endswith('\r\n') else gga + '\r\n'
#                             sock.sendall(out.encode())
#                             print(f"📤 Sent GGA: {gga.strip()}")
#                         except Exception as e:
#                             print(f"GGA send error: {e}")
#                     last_sent = now
 
#                 try:
#                     data = sock.recv(4096)
#                 except socket.timeout:
#                     continue
#                 if not data:
#                     break
#                 gnss_ser.write(data)
 
#         except Exception as e:
#             print(f"❌ NTRIP error: {e}")
#         time.sleep(5)
 
 
# # ══════════════════════════════════════════════════════════════════════════════
# #  MAIN
# # ══════════════════════════════════════════════════════════════════════════════
# # สร้าง CSV ถ้ายังไม่มี
# if not os.path.isfile(CSV_FILE):
#     with open(CSV_FILE, 'w', newline='') as f:
#         # ✅ เปลี่ยน x, y → latitude, longitude (Decimal Degrees WGS84)
#         csv.writer(f).writerow(["index", "latitude", "longitude", "yaw", "speed", "fix_quality"])
#     print(f"📄 Created new CSV: {CSV_FILE}")
# else:
#     print(f"📄 Appending to existing CSV: {CSV_FILE}")
 
# gnss_ser = serial.Serial(GNSS_PORT, GNSS_BAUD, timeout=1)
# print(f"✅ Opened {GNSS_PORT} @ {GNSS_BAUD} baud")
# print(f"📐 Mode: Lat/Lon Decimal Degrees (WGS84)")
# print(f"📏 Waypoint spacing: {WAYPOINT_DISTANCE} m")
# if ANTENNA_LATERAL_OFFSET != 0.0:
#     side = "ขวา" if ANTENNA_LATERAL_OFFSET > 0 else "ซ้าย"
#     print(f"📡 Antenna offset: {abs(ANTENNA_LATERAL_OFFSET):.3f} ม. ทาง{side}")
 
# threading.Thread(target=ntrip_client, daemon=True).start()
# print("\n⏳ Waiting for RTK Fixed (quality=4)...\n")
 
# recording = False
 
# while True:
#     try:
#         line = gnss_ser.readline().decode('ascii', errors='replace').strip()
 
#         # ── GGA: อัปเดต fix quality ────────────────────────────────────────
#         if line.startswith(('$GPGGA', '$GNGGA')):
#             try:
#                 gga      = pynmea2.parse(line)
#                 fix_qual = int(gga.gps_qual)
 
#                 with gga_lock:
#                     last_gga = line
 
#                 quality_counts[fix_qual] = quality_counts.get(fix_qual, 0) + 1
#                 label = FIX_LABELS.get(fix_qual, f"quality={fix_qual}")
 
#                 # แสดงสถานะแบบ inline
#                 print(f"\r🛰️  {label}  | waypoints บันทึกแล้ว: {index}   ",
#                       end='', flush=True)
 
#                 if fix_qual >= MIN_RECORD_FIX and not recording:
#                     print(f"\n\n🟢 RTK Fixed! เริ่มบันทึก waypoints...\n")
#                     recording = True
 
#                 if fix_qual < MIN_RECORD_FIX and recording:
#                     print(f"\n\n⚠️  Fix หลุดเป็น {label} — หยุดบันทึกชั่วคราว\n")
#                     recording = False
 
#             except Exception:
#                 pass
#             continue
 
#         # ── RMC: บันทึก waypoint ───────────────────────────────────────────
#         if not line.startswith(('$GPRMC', '$GNRMC')):
#             continue
#         if not recording or fix_qual < MIN_RECORD_FIX:
#             continue
 
#         msg = pynmea2.parse(line)
#         if msg.status != 'A':
#             continue
 
#         lat = nmea_to_decimal(msg.lat, msg.lat_dir)
#         lon = nmea_to_decimal(msg.lon, msg.lon_dir)
 
#         # ✅ แปลงเป็น UTM เพื่อใช้คำนวณระยะ + heading (เมตร-based)
#         x_utm, y_utm = transformer_to_utm.transform(lon, lat)
 
#         # ── Speed smoothing ────────────────────────────────────────────────
#         raw_speed = float(msg.spd_over_grnd or 0) * 0.514444  # knots → m/s
#         speed_buf.append(raw_speed)
#         if len(speed_buf) > SPEED_WINDOW:
#             speed_buf.pop(0)
#         speed = sum(speed_buf) / len(speed_buf)
 
#         # ── Heading calculation ────────────────────────────────────────────
#         if msg.true_course and msg.true_course != '' and speed >= MIN_SPEED_FOR_HDG:
#             # ✅ แปลง GPS bearing → ENU yaw
#             yaw      = bearing_to_enu_yaw(float(msg.true_course))
#             last_yaw = yaw
#         elif prev_x_utm is not None:
#             dx = x_utm - prev_x_utm
#             dy = y_utm - prev_y_utm
#             d  = math.hypot(dx, dy)
#             if d > 0.05:
#                 # ✅ atan2(dy, dx) ใน UTM ENU frame ถูกต้องโดยตรง
#                 yaw      = math.atan2(dy, dx)
#                 last_yaw = yaw
#             else:
#                 yaw = last_yaw
#         else:
#             yaw = last_yaw
 
#         # ── Antenna offset compensation (ทำงานใน UTM แล้วแปลงกลับ) ────────
#         x_fixed_utm, y_fixed_utm = apply_antenna_offset_utm(
#             x_utm, y_utm, yaw, ANTENNA_LATERAL_OFFSET
#         )
 
#         # ✅ แปลง UTM (ที่ชดเชย offset แล้ว) → lat/lon
#         lon_final, lat_final = transformer_to_latlon.transform(x_fixed_utm, y_fixed_utm)
 
#         # ── กรองระยะขั้นต่ำ (ใช้ UTM เพื่อความแม่นยำ) ─────────────────────
#         if last_x_utm is not None:
#             d_from_last = math.hypot(x_fixed_utm - last_x_utm, y_fixed_utm - last_y_utm)
#             if d_from_last < WAYPOINT_DISTANCE:
#                 prev_x_utm = x_utm
#                 prev_y_utm = y_utm
#                 continue
 
#         # ── บันทึก lat/lon ─────────────────────────────────────────────────
#         with open(CSV_FILE, 'a', newline='') as f:
#             csv.writer(f).writerow([
#                 index,
#                 f"{lat_final:.8f}",   # ← Latitude  เช่น 13.84512345
#                 f"{lon_final:.8f}",   # ← Longitude เช่น 100.56789012
#                 f"{yaw:.6f}",         # ← ENU yaw (radians)
#                 f"{speed:.4f}",       # ← m/s (smoothed)
#                 fix_qual              # ← fix quality สำหรับ QA
#             ])
 
#         print(f"\n  [{index:4d}]  "
#               f"lat={lat_final:.8f}  lon={lon_final:.8f}  "
#               f"yaw={math.degrees(yaw):6.1f}°  "
#               f"speed={speed:.2f} m/s  "
#               f"fix={fix_qual}")
 
#         last_lat   = lat_final
#         last_lon   = lon_final
#         last_x_utm = x_fixed_utm
#         last_y_utm = y_fixed_utm
#         prev_x_utm = x_utm
#         prev_y_utm = y_utm
#         index     += 1
 
#     except KeyboardInterrupt:
#         total = sum(quality_counts.values()) or 1
#         print(f"\n\n{'='*55}")
#         print(f"🛑 หยุดบันทึก — {index} waypoints → {CSV_FILE}")
#         print(f"\n📊 สถิติคุณภาพ Fix :")
#         for q, label in [(4, "RTK Fixed ✅"), (5, "RTK Float 🔶"),
#                          (2, "DGPS 🔵"), (1, "GPS only ⚪"), (0, "No fix ❌")]:
#             n = quality_counts.get(q, 0)
#             if n > 0:
#                 print(f"   {label}: {n} ครั้ง ({100 * n // total}%)")
 
#         rtk_pct = 100 * quality_counts.get(4, 0) // total
#         print()
#         if rtk_pct >= 95:
#             print(f"✅ คุณภาพดีมาก ({rtk_pct}% RTK Fixed) — พร้อมใช้งาน")
#         elif rtk_pct >= 80:
#             print(f"⚠️  คุณภาพพอใช้ ({rtk_pct}% RTK Fixed) — ควรบันทึกใหม่บางช่วง")
#         else:
#             print(f"❌ คุณภาพต่ำ ({rtk_pct}% RTK Fixed) — แนะนำบันทึกใหม่ทั้งหมด")
#         print('='*55)
#         break
 
#     except Exception as e:
#         print(f"\nError: {e}")
 
# gnss_ser.close()



#  ----------------------------------------------------------------------------------
# import serial
# import pynmea2
# import csv
# import os
# import math
# import socket
# import base64
# import threading
# import time
# import datetime
# from pyproj import Transformer

# # ---------------- LOG PATH ----------------
# LOG_DIR  = "/home/inc/ros2_ws/src/navigation_system/logs"
# os.makedirs(LOG_DIR, exist_ok=True)

# TS       = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
# NMEA_LOG = os.path.join(LOG_DIR, f"nmea_{TS}.log")
# SNR_CSV  = os.path.join(LOG_DIR, f"snr_{TS}.csv")

# with open(SNR_CSV, 'w', newline='') as f:
#     csv.writer(f).writerow(["timestamp","sentence","sat_id","elevation","snr_db"])

# print(f"📝 NMEA log: {NMEA_LOG}")
# print(f"📊 SNR  log: {SNR_CSV}")

# # ---------------- GNSS ----------------
# GNSS_PORT = '/dev/ttyACM0'
# GNSS_BAUD = 9600

# # ---------------- CSV ----------------
# CSV_FILE          = "/home/inc/ros2_ws/src/navigation_system/navigation_system/waypointsnownownow.csv"
# WAYPOINT_DISTANCE = 0.3

# # ---------------- NTRIP ----------------
# NTRIP_SERVER_IP   = "110.78.0.54"
# NTRIP_SERVER_PORT = 2116
# NTRIP_USERNAME    = "1118600009224"
# NTRIP_PASSWORD    = "CK79"
# NTRIP_MOUNTPOINT  = "VRS_RTCM32"
# GGA_SEND_INTERVAL = 1.0

# MIN_RECORD_FIX    = 4
# MIN_SPEED_FOR_HDG = 0.3
# SPEED_WINDOW      = 5
# ANTENNA_LATERAL_OFFSET = 0.0

# FIX_LABELS = {
#     0: "No fix ❌",
#     1: "GPS only ⚪",
#     2: "DGPS 🔵",
#     4: "RTK Fixed ✅",
#     5: "RTK Float 🔶",
# }

# transformer_to_utm    = Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)
# transformer_to_latlon = Transformer.from_crs("epsg:32647", "epsg:4326", always_xy=True)

# # ── shared state ─────────────────────
# last_lat = last_lon = None
# last_x_utm = last_y_utm = None
# prev_x_utm = prev_y_utm = None
# index = 0
# fix_qual = 0
# last_gga = None
# gga_lock = threading.Lock()
# gnss_ser = None
# speed_buf = []
# last_yaw = 0.0
# quality_counts = {}

# # ═════════════════════════════════════
# # UTILS
# # ═════════════════════════════════════
# def nmea_to_decimal(degree_str, direction):
#     raw = float(degree_str)
#     degrees = int(raw / 100)
#     minutes = raw - degrees * 100
#     decimal = degrees + minutes / 60
#     if direction in ('S', 'W'):
#         decimal = -decimal
#     return decimal

# def bearing_to_enu_yaw(bearing_deg):
#     raw = math.pi/2 - math.radians(bearing_deg)
#     return (raw + math.pi) % (2*math.pi) - math.pi

# def apply_antenna_offset_utm(x, y, yaw, offset):
#     if offset == 0.0:
#         return x, y
#     perp = yaw + math.pi/2
#     return x - offset*math.cos(perp), y - offset*math.sin(perp)

# # ═════════════════════════════════════
# # NTRIP THREAD
# # ═════════════════════════════════════
# def ntrip_client():
#     global gnss_ser
#     while True:
#         try:
#             print("🛰 Connecting NTRIP...")
#             sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#             sock.connect((NTRIP_SERVER_IP, NTRIP_SERVER_PORT))

#             auth = base64.b64encode(f"{NTRIP_USERNAME}:{NTRIP_PASSWORD}".encode()).decode()

#             request = (
#                 f"GET /{NTRIP_MOUNTPOINT} HTTP/1.0\r\n"
#                 f"Authorization: Basic {auth}\r\n\r\n"
#             )
#             sock.sendall(request.encode())

#             print("✅ NTRIP connected")

#             while True:
#                 data = sock.recv(4096)
#                 if not data:
#                     break
#                 gnss_ser.write(data)

#         except Exception as e:
#             print("❌ NTRIP error:", e)
#             time.sleep(5)

# # ═════════════════════════════════════
# # MAIN
# # ═════════════════════════════════════
# gnss_ser = serial.Serial(GNSS_PORT, GNSS_BAUD, timeout=1)
# threading.Thread(target=ntrip_client, daemon=True).start()

# print("⏳ Waiting RTK...\n")

# while True:
#     try:
#         line = gnss_ser.readline().decode('ascii', errors='replace').strip()

#         # ✅ บันทึก NMEA ทุกบรรทัด
#         if line.startswith('$'):
#             with open(NMEA_LOG, 'a') as f:
#                 f.write(f"{time.time():.3f},{line}\n")

#         # ✅ เก็บ SNR จาก GSV
#         if line.startswith(('$GPGSV','$GNGSV','$GLGSV','$GAGSV','$GBGSV')):
#             try:
#                 gsv = pynmea2.parse(line)
#                 t_now = datetime.datetime.now().isoformat()
#                 with open(SNR_CSV, 'a', newline='') as f:
#                     w = csv.writer(f)
#                     for i in range(1,5):
#                         sat = getattr(gsv, f'sv_prn_num_{i}', None)
#                         ele = getattr(gsv, f'elevation_{i}', None)
#                         snr = getattr(gsv, f'snr_{i}', None)
#                         if sat and snr:
#                             w.writerow([t_now, line[:6], sat, ele, snr])
#             except:
#                 pass
#             continue

#         # GGA
#         if line.startswith(('$GPGGA','$GNGGA')):
#             gga = pynmea2.parse(line)
#             fix_qual = int(gga.gps_qual)
#             print(f"\rFix: {FIX_LABELS.get(fix_qual)}", end='')
#             continue

#         # RMC
#         if not line.startswith(('$GPRMC','$GNRMC')):
#             continue

#         msg = pynmea2.parse(line)
#         if msg.status != 'A':
#             continue

#         lat = nmea_to_decimal(msg.lat, msg.lat_dir)
#         lon = nmea_to_decimal(msg.lon, msg.lon_dir)

#         x, y = transformer_to_utm.transform(lon, lat)

#         speed = float(msg.spd_over_grnd or 0) * 0.514

#         yaw = last_yaw
#         if msg.true_course:
#             yaw = bearing_to_enu_yaw(float(msg.true_course))
#             last_yaw = yaw

#         x, y = apply_antenna_offset_utm(x, y, yaw, ANTENNA_LATERAL_OFFSET)

#         print(f"\n📍 {lat:.6f}, {lon:.6f} | speed={speed:.2f}")

#     except KeyboardInterrupt:
#         print("\n🛑 Stop")
#         break
#     except Exception as e:
#         print("\nError:", e)

# gnss_ser.close()


# import serial
# import pynmea2
# import csv
# import os
# import math
# import socket
# import base64
# import threading
# import time
# import datetime

# import rclpy
# from std_msgs.msg import Float64

# # ---------------- PATH ----------------
# LOG_DIR  = "/home/inc/ros2_ws/src/navigation_system/logs13"
# os.makedirs(LOG_DIR, exist_ok=True)

# TS       = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

# NMEA_LOG = os.path.join(LOG_DIR, f"nmea_{TS}.log")
# SNR_CSV  = os.path.join(LOG_DIR, f"snr_{TS}.csv")
# WAYPOINT_CSV = os.path.join(LOG_DIR, "waypoints_clean.csv")

# print(f"📝 NMEA log: {NMEA_LOG}")
# print(f"📊 SNR log: {SNR_CSV}")
# print(f"📁 Waypoint file: {WAYPOINT_CSV}")

# # ---------------- GNSS ----------------
# GNSS_PORT = '/dev/ttyACM0'
# GNSS_BAUD = 115200

# # ---------------- NTRIP ----------------
# NTRIP_SERVER_IP   = "110.78.0.54"
# NTRIP_SERVER_PORT = 2116
# NTRIP_USERNAME    = "1118600009224"
# NTRIP_PASSWORD    = "CK79"
# NTRIP_MOUNTPOINT  = "VRS_RTCM32"
# GGA_SEND_INTERVAL = 0.2   # ⭐ สำคัญมาก

# # ---------------- FIX LABEL ----------------
# FIX_LABELS = {
#     0: "No fix ❌",
#     1: "GPS only ⚪",
#     2: "DGPS 🔵",
#     4: "RTK Fixed ✅",
#     5: "RTK Float 🔶",
# }

# # ---------------- FILTER PARAM ----------------
# MIN_SPEED = 0.5        # m/s
# ALLOWED_FIX = [4, 5]   # RTK FIX + FLOAT

# last_lat = None
# last_lon = None
# DIST_THRESHOLD = 0.5   # เมตร

# # ---------------- STATE ----------------
# fix_qual = 0
# last_heading = 0.0
# last_imu_yaw = None
# last_imu_yaw_time = 0.0
# IMU_YAW_TIMEOUT = 1.0
# last_gga_sentence = None
# gnss_ser = None

# rclpy.init(args=None)
# imu_node = rclpy.create_node('waypoint_imu_yaw_listener')

# def imu_yaw_callback(msg):
#     global last_imu_yaw, last_imu_yaw_time
#     last_imu_yaw = msg.data
#     last_imu_yaw_time = time.time()

# imu_node.create_subscription(Float64, '/imu/yaw', imu_yaw_callback, 10)

# # ---------------- CREATE FILES ----------------
# with open(SNR_CSV, 'w', newline='') as f:
#     csv.writer(f).writerow(["timestamp","sentence","sat_id","elevation","snr_db"])

# with open(WAYPOINT_CSV, 'w', newline='') as f:
#     csv.writer(f).writerow(["lat","lon","speed","heading","fix_quality"])

# # ---------------- UTILS ----------------
# def nmea_to_decimal(degree_str, direction):
#     raw = float(degree_str)
#     degrees = int(raw / 100)
#     minutes = raw - degrees * 100
#     decimal = degrees + minutes / 60
#     if direction in ('S', 'W'):
#         decimal = -decimal
#     return decimal

# def haversine(lat1, lon1, lat2, lon2):
#     R = 6371000  # meters
#     phi1, phi2 = math.radians(lat1), math.radians(lat2)
#     dphi = phi2 - phi1
#     dlambda = math.radians(lon2 - lon1)

#     a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
#     return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1-a))

# # ---------------- NTRIP THREAD ----------------
# def ntrip_client():
#     global gnss_ser, last_gga_sentence

#     while True:
#         try:
#             print("🛰 Connecting NTRIP...")
#             sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#             sock.connect((NTRIP_SERVER_IP, NTRIP_SERVER_PORT))

#             auth = base64.b64encode(f"{NTRIP_USERNAME}:{NTRIP_PASSWORD}".encode()).decode()

#             request = (
#                 f"GET /{NTRIP_MOUNTPOINT} HTTP/1.0\r\n"
#                 f"Authorization: Basic {auth}\r\n\r\n"
#             )
#             sock.sendall(request.encode())

#             print("✅ NTRIP connected")

#             sock.settimeout(0.1)
#             last_send = 0

#             while True:
#                 # -------- รับ RTCM --------
#                 try:
#                     data = sock.recv(4096)
#                     if data:
#                         gnss_ser.write(data)
#                 except socket.timeout:
#                     pass

#                 # -------- ส่ง GGA --------
#                 now = time.time()
#                 if last_gga_sentence and (now - last_send > GGA_SEND_INTERVAL):
#                     try:
#                         sock.sendall((last_gga_sentence + "\r\n").encode())
#                         last_send = now
#                     except:
#                         break

#         except Exception as e:
#             print("❌ NTRIP error:", e)
#             time.sleep(5)

# # ---------------- START ----------------
# gnss_ser = serial.Serial(GNSS_PORT, GNSS_BAUD, timeout=1)
# threading.Thread(target=ntrip_client, daemon=True).start()

# print("⏳ Waiting GNSS...\n")

# # ---------------- MAIN LOOP ----------------
# while True:
#     try:
#         line = gnss_ser.readline().decode('ascii', errors='replace').strip()

#         # -------- LOG NMEA --------
#         if line.startswith('$'):
#             with open(NMEA_LOG, 'a') as f:
#                 f.write(f"{time.time():.3f},{line}\n")

#         # -------- SNR --------
#         if line.startswith(('$GPGSV','$GLGSV','$GAGSV','$GBGSV','$GNGSV')):
#             try:
#                 gsv = pynmea2.parse(line)
#                 t_now = datetime.datetime.now().isoformat()

#                 with open(SNR_CSV, 'a', newline='') as f:
#                     w = csv.writer(f)
#                     for i in range(1,5):
#                         sat = getattr(gsv, f'sv_prn_num_{i}', None)
#                         ele = getattr(gsv, f'elevation_{i}', None)
#                         snr = getattr(gsv, f'snr_{i}', None)

#                         if sat and snr:
#                             w.writerow([t_now, line[:6], sat, ele, snr])
#             except:
#                 pass
#             continue

#         # -------- GGA --------
#         if line.startswith(('$GPGGA','$GNGGA')):
#             try:
#                 gga = pynmea2.parse(line)
#                 fix_qual = int(gga.gps_qual)

#                 # ⭐ เก็บไว้ส่งเข้า NTRIP
#                 last_gga_sentence = line

#                 print(f"\rFix: {FIX_LABELS.get(fix_qual)}", end='')
#             except:
#                 pass
#             continue

#         # -------- RMC --------
#         if not line.startswith(('$GPRMC','$GNRMC')):
#             continue

#         msg = pynmea2.parse(line)
#         if msg.status != 'A':
#             continue

#         rclpy.spin_once(imu_node, timeout_sec=0.0)

#         lat = nmea_to_decimal(msg.lat, msg.lat_dir)
#         lon = nmea_to_decimal(msg.lon, msg.lon_dir)

#         speed = float(msg.spd_over_grnd or 0) * 0.514

#         heading = last_heading
#         if last_imu_yaw is not None and time.time() - last_imu_yaw_time <= IMU_YAW_TIMEOUT:
#             heading = math.degrees(last_imu_yaw)
#             last_heading = heading

#         print(f"\n📍 {lat:.6f}, {lon:.6f} | speed={speed:.2f} | yaw_imu={heading:.2f} | {FIX_LABELS.get(fix_qual)}")

#         # -------- SAVE WAYPOINT --------
#         if fix_qual in ALLOWED_FIX and speed > MIN_SPEED:

#             if last_lat is not None:
#                 dist = haversine(lat, lon, last_lat, last_lon)
#                 if dist < DIST_THRESHOLD:
#                     print("  ⛔ TOO CLOSE")
#                     continue

#             last_lat, last_lon = lat, lon

#             with open(WAYPOINT_CSV, 'a', newline='') as f:
#                 writer = csv.writer(f)
#                 writer.writerow([lat, lon, speed, heading, fix_qual])
#                 f.flush()

#             print("  ✅ SAVED")

#         else:
#             print("  ⛔ SKIPPED")

#     except KeyboardInterrupt:
#         print("\n🛑 Stop")
#         break

#     except Exception as e:
#         print("\nError:", e)

# imu_node.destroy_node()
# rclpy.shutdown()

# gnss_ser.close()


#------------------------------------------------ keepwaypointsaty-------------------------------------------------------------y


import serial
import pynmea2
import csv
import os
import math
import socket
import base64
import threading
import time
import datetime

# ---------------- PATH ----------------
LOG_DIR  = "/home/inc/ros2_ws/src/navigation_system/logs14"
os.makedirs(LOG_DIR, exist_ok=True)

TS       = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

NMEA_LOG = os.path.join(LOG_DIR, f"nmea_{TS}.log")
SNR_CSV  = os.path.join(LOG_DIR, f"snr_{TS}.csv")
WAYPOINT_CSV = os.path.join(LOG_DIR, "waypoints_clean.csv")

print(f"📝 NMEA log: {NMEA_LOG}")
print(f"📊 SNR log: {SNR_CSV}")
print(f"📁 Waypoint file: {WAYPOINT_CSV}")

# ---------------- GNSS ----------------
GNSS_PORT = '/dev/ttyACM0'
GNSS_BAUD = 115200

# ---------------- NTRIP ----------------
NTRIP_SERVER_IP   = "110.78.0.54"
NTRIP_SERVER_PORT = 2116
NTRIP_USERNAME    = "1118600009224"
NTRIP_PASSWORD    = "CK79"
NTRIP_MOUNTPOINT  = "VRS_RTCM32"
GGA_SEND_INTERVAL = 0.2   # ⭐ สำคัญมาก

# ---------------- FIX LABEL ----------------
FIX_LABELS = {
    0: "No fix ❌",
    1: "GPS only ⚪",
    2: "DGPS 🔵",
    4: "RTK Fixed ✅",
    5: "RTK Float 🔶",
}

# ---------------- FILTER PARAM ----------------
MIN_SPEED = 0.2        # m/s
ALLOWED_FIX = [4, 5]   # RTK FIX + FLOAT

last_lat = None
last_lon = None
DIST_THRESHOLD = 0.5   # เมตร

# ---------------- STATE ----------------
fix_qual = 0
last_heading = 0.0
last_ublox_heading = None
last_ublox_heading_time = 0.0
UBLOX_HEADING_TIMEOUT = 2.0
last_gga_sentence = None
gnss_ser = None

# ---------------- CREATE FILES ----------------
with open(SNR_CSV, 'w', newline='') as f:
    csv.writer(f).writerow(["timestamp","sentence","sat_id","elevation","snr_db"])

with open(WAYPOINT_CSV, 'w', newline='') as f:
    csv.writer(f).writerow(["lat","lon","speed","heading","fix_quality"])

# ---------------- UTILS ----------------
def nmea_to_decimal(degree_str, direction):
    raw = float(degree_str)
    degrees = int(raw / 100)
    minutes = raw - degrees * 100
    decimal = degrees + minutes / 60
    if direction in ('S', 'W'):
        decimal = -decimal
    return decimal

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000  # meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = phi2 - phi1
    dlambda = math.radians(lon2 - lon1)

    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1-a))

def normalize_heading_deg(heading):
    return float(heading) % 360.0

def update_ublox_heading(heading):
    global last_heading, last_ublox_heading, last_ublox_heading_time
    if heading is None:
        return
    last_ublox_heading = normalize_heading_deg(heading)
    last_ublox_heading_time = time.time()
    last_heading = last_ublox_heading

def get_nmea_field(line, field_index):
    body = line.split('*', 1)[0]
    parts = body.split(',')
    if len(parts) <= field_index or parts[field_index] == '':
        return None
    return parts[field_index]

def parse_ublox_heading_sentence(line):
    try:
        msg = pynmea2.parse(line)
    except pynmea2.ParseError:
        msg = None

    if msg is not None and isinstance(msg, pynmea2.types.talker.HDT):
        return msg.heading

    if msg is not None and line.startswith(('$GNTHS', '$GPTHS')):
        return getattr(msg, 'heading', None)

    if msg is not None and isinstance(msg, pynmea2.types.talker.VTG):
        return getattr(msg, 'true_track', None)

    if line.startswith(('$GPHDT', '$GNHDT', '$GPTHS', '$GNTHS', '$GPVTG', '$GNVTG')):
        return get_nmea_field(line, 1)

    return None

# ---------------- NTRIP THREAD ----------------
def ntrip_client():
    global gnss_ser, last_gga_sentence

    while True:
        try:
            print("🛰 Connecting NTRIP...")
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((NTRIP_SERVER_IP, NTRIP_SERVER_PORT))

            auth = base64.b64encode(f"{NTRIP_USERNAME}:{NTRIP_PASSWORD}".encode()).decode()

            request = (
                f"GET /{NTRIP_MOUNTPOINT} HTTP/1.0\r\n"
                f"Authorization: Basic {auth}\r\n\r\n"
            )
            sock.sendall(request.encode())

            print("✅ NTRIP connected")

            sock.settimeout(0.1)
            last_send = 0

            while True:
                # -------- รับ RTCM --------
                try:
                    data = sock.recv(4096)
                    if data:
                        gnss_ser.write(data)
                except socket.timeout:
                    pass

                # -------- ส่ง GGA --------
                now = time.time()
                if last_gga_sentence and (now - last_send > GGA_SEND_INTERVAL):
                    try:
                        sock.sendall((last_gga_sentence + "\r\n").encode())
                        last_send = now
                    except:
                        break

        except Exception as e:
            print("❌ NTRIP error:", e)
            time.sleep(5)

# ---------------- START ----------------
gnss_ser = serial.Serial(GNSS_PORT, GNSS_BAUD, timeout=1)
threading.Thread(target=ntrip_client, daemon=True).start()

print("⏳ Waiting GNSS...\n")

# ---------------- MAIN LOOP ----------------
while True:
    try:
        line = gnss_ser.readline().decode('ascii', errors='replace').strip()

        # -------- LOG NMEA --------
        if line.startswith('$'):
            with open(NMEA_LOG, 'a') as f:
                f.write(f"{time.time():.3f},{line}\n")

        # -------- SNR --------
        if line.startswith(('$GPGSV','$GLGSV','$GAGSV','$GBGSV','$GNGSV')):
            try:
                gsv = pynmea2.parse(line)
                t_now = datetime.datetime.now().isoformat()

                with open(SNR_CSV, 'a', newline='') as f:
                    w = csv.writer(f)
                    for i in range(1,5):
                        sat = getattr(gsv, f'sv_prn_num_{i}', None)
                        ele = getattr(gsv, f'elevation_{i}', None)
                        snr = getattr(gsv, f'snr_{i}', None)

                        if sat and snr:
                            w.writerow([t_now, line[:6], sat, ele, snr])
            except:
                pass
            continue

        # -------- UBLOX HEADING --------
        if line.startswith(('$GPHDT', '$GNHDT', '$GPTHS', '$GNTHS', '$GPVTG', '$GNVTG')):
            heading_from_ublox = parse_ublox_heading_sentence(line)
            if heading_from_ublox is not None:
                update_ublox_heading(heading_from_ublox)
            continue

        # -------- GGA --------
        if line.startswith(('$GPGGA','$GNGGA')):
            try:
                gga = pynmea2.parse(line)
                fix_qual = int(gga.gps_qual)

                # ⭐ เก็บไว้ส่งเข้า NTRIP
                last_gga_sentence = line

                print(f"\rFix: {FIX_LABELS.get(fix_qual)}", end='')
            except:
                pass
            continue

        # -------- RMC --------
        if not line.startswith(('$GPRMC','$GNRMC')):
            continue

        msg = pynmea2.parse(line)
        if msg.status != 'A':
            continue

        lat = nmea_to_decimal(msg.lat, msg.lat_dir)
        lon = nmea_to_decimal(msg.lon, msg.lon_dir)

        speed = float(msg.spd_over_grnd or 0) * 0.514

        if msg.true_course not in (None, ''):
            update_ublox_heading(msg.true_course)

        heading = last_heading
        if (
            last_ublox_heading is not None
            and time.time() - last_ublox_heading_time <= UBLOX_HEADING_TIMEOUT
        ):
            heading = last_ublox_heading

        print(f"\n📍 {lat:.6f}, {lon:.6f} | speed={speed:.2f} | heading_ublox={heading:.2f} | {FIX_LABELS.get(fix_qual)}")

        # -------- SAVE WAYPOINT --------
        if fix_qual in ALLOWED_FIX and speed > MIN_SPEED:

            if last_lat is not None:
                dist = haversine(lat, lon, last_lat, last_lon)
                if dist < DIST_THRESHOLD:
                    print("  ⛔ TOO CLOSE")
                    continue

            last_lat, last_lon = lat, lon

            with open(WAYPOINT_CSV, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([lat, lon, speed, heading, fix_qual])
                f.flush()

            print("  ✅ SAVED")

        else:
            print("  ⛔ SKIPPED")

    except KeyboardInterrupt:
        print("\n🛑 Stop")
        break

    except Exception as e:
        print("\nError:", e)

gnss_ser.close()
