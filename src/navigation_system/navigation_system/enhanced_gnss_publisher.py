import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64
import serial
import pynmea2
import math
import time
import socket
import base64
import threading
from pyproj import Transformer

# -------- NTRIP RTK ----------
NTRIP_SERVER_IP   = "110.78.0.54"
NTRIP_SERVER_PORT = 2116
NTRIP_USERNAME    = "1118600009224"
NTRIP_PASSWORD    = "CK79"
NTRIP_MOUNTPOINT  = "VRS_RTCM32"
NTRIP_CONNECT_TIMEOUT = 10.0
NTRIP_RETRY_INTERVAL  = 5.0
# -----------------------------

# -------- ปรับให้ตรงกับ waypoint logger --------
GGA_SEND_INTERVAL     = 0.2   # ⭐ ส่ง GGA บ่อย → VRS server แม่นยำ → RTK Fix เร็ว
ALLOWED_FIX           = [4, 5]  # RTK Fixed ✅ + RTK Float 🔶 (เหมือน waypoint logger)
MIN_SPEED_FOR_HEADING = 0.5   # m/s — ตรงกับ MIN_SPEED ของ waypoint logger
# -----------------------------------------------

FIX_LABELS = {
    0: "No fix ❌",
    1: "GPS only ⚪",
    2: "DGPS 🔵",
    4: "RTK Fixed ✅",
    5: "RTK Float 🔶",
}


class GnssPosePublisher(Node):

    def __init__(self):
        super().__init__('gnss_pose_publisher')

        # ⭐ แก้ baud rate ให้ตรงกับ waypoint logger (9600 → 115200)
        self.GNSS_PORT = '/dev/ttyACM0'
        self.GNSS_BAUD = 115200

        # UTM zone 47N (ประเทศไทย)
        self.transformer = Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)

        self.prev_yaw    = None
        self.prev_time   = 0.0
        self.fix_quality = 0
        self.imu_yaw = None
        self.imu_yaw_time = 0.0
        self.imu_yaw_timeout = 1.0

        self.prev_x_utm: float | None = None
        self.prev_y_utm: float | None = None

        # lat/lon ล่าสุดจาก GGA (แม่นกว่า RMC เมื่อใช้ RTK)
        self._last_lat: float | None = None
        self._last_lon: float | None = None

        # ⭐ เก็บ raw GGA sentence ทั้งบรรทัด (เหมือน waypoint logger)
        self._last_gga_sentence: str | None = None
        self._gga_lock = threading.Lock()

        self._ntrip_sock: socket.socket | None = None
        self._ntrip_host = NTRIP_SERVER_IP
        self._ntrip_port = NTRIP_SERVER_PORT

        self.pose_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_yaw_sub = self.create_subscription(
            Float64,
            '/imu/yaw',
            self.imu_yaw_callback,
            10,
        )

        try:
            self.ser = serial.Serial(self.GNSS_PORT, self.GNSS_BAUD, timeout=1)
            self.get_logger().info(f"✅ Opened {self.GNSS_PORT} @ {self.GNSS_BAUD} baud")
        except Exception as e:
            self.get_logger().error(f"❌ Cannot open serial: {e}")
            raise

        threading.Thread(target=self._ntrip_thread, daemon=True).start()
        self.create_timer(0.1, self.read_gps)
        self.get_logger().info("📡 GNSS Pose Publisher READY — UTM Absolute + RTK Float/Fixed Mode")

    def imu_yaw_callback(self, msg):
        self.imu_yaw = self._normalize_angle(msg.data)
        self.imu_yaw_time = time.time()

    # ══════════════════════════════════════════════════════════════════════════
    #  NTRIP thread
    #  ⭐ ปรับ logic การส่ง GGA ให้ตรงกับ waypoint logger
    #     - ส่ง raw GGA sentence (ทั้งบรรทัดที่ได้จาก serial ตรงๆ)
    #     - ส่งทุก GGA_SEND_INTERVAL = 0.2 วิ
    # ══════════════════════════════════════════════════════════════════════════
    def _ntrip_thread(self):
        while True:
            sock = None
            try:
                local_addr = self._wait_for_network()
                self.get_logger().info(
                    f"🛰 Connecting NTRIP via active network ({local_addr})..."
                )
                sock = self._open_ntrip_socket()
                self._ntrip_sock = sock

                auth = base64.b64encode(
                    f"{NTRIP_USERNAME}:{NTRIP_PASSWORD}".encode()
                ).decode()

                # ⭐ ใช้ HTTP/1.0 แบบ waypoint logger (ไม่ส่ง Ntrip-GGA ใน header)
                request = (
                    f"GET /{NTRIP_MOUNTPOINT} HTTP/1.0\r\n"
                    f"Authorization: Basic {auth}\r\n\r\n"
                )
                sock.sendall(request.encode())
                self.get_logger().info("✅ NTRIP connected")

                sock.settimeout(0.1)
                last_send = 0

                while True:
                    # -------- รับ RTCM → ส่งเข้า u-blox --------
                    try:
                        data = sock.recv(4096)
                        if data:
                            self.ser.write(data)
                    except socket.timeout:
                        pass

                    # -------- ส่ง GGA กลับ caster (VRS) --------
                    now = time.time()
                    if now - last_send > GGA_SEND_INTERVAL:
                        with self._gga_lock:
                            gga = self._last_gga_sentence
                        if gga:
                            try:
                                sock.sendall((gga + "\r\n").encode())
                                last_send = now
                            except Exception:
                                break

            except Exception as e:
                self.get_logger().error(f"❌ NTRIP error: {e}")
            finally:
                self._ntrip_sock = None
                if sock:
                    try:
                        sock.close()
                    except Exception:
                        pass
                time.sleep(NTRIP_RETRY_INTERVAL)

    def _wait_for_network(self):
        while rclpy.ok():
            try:
                return self._active_network_address()
            except OSError:
                self.get_logger().warn(
                    "No active network route for NTRIP yet; retrying...",
                    throttle_duration_sec=10.0
                )
                time.sleep(NTRIP_RETRY_INTERVAL)
        raise RuntimeError("ROS shutdown before network became available")

    def _active_network_address(self):
        # UDP connect does not send packets; it only lets the OS choose the
        # route/interface that would be used for this caster.
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.connect((self._ntrip_host, self._ntrip_port))
            return probe.getsockname()[0]
        finally:
            probe.close()

    def _open_ntrip_socket(self):
        last_error = None
        addr_infos = socket.getaddrinfo(
            self._ntrip_host,
            self._ntrip_port,
            type=socket.SOCK_STREAM,
        )

        for family, socktype, proto, _canonname, sockaddr in addr_infos:
            sock = socket.socket(family, socktype, proto)
            sock.settimeout(NTRIP_CONNECT_TIMEOUT)
            try:
                sock.connect(sockaddr)
                return sock
            except OSError as e:
                last_error = e
                sock.close()

        if last_error:
            raise last_error
        raise OSError(f"Cannot resolve NTRIP host {self._ntrip_host}")

    # ══════════════════════════════════════════════════════════════════════════
    #  GPS READ
    # ══════════════════════════════════════════════════════════════════════════
    def read_gps(self):
        for _ in range(50):
            try:
                line = self.ser.readline().decode('ascii', errors='replace').strip()
            except Exception:
                return

            if line:
                self.process_nmea_line(line)

            if getattr(self.ser, 'in_waiting', 0) <= 0:
                break

    def process_nmea_line(self, line):
        try:
            if not line.startswith('$'):
                return

            self.handle_nmea_line(line)
        except Exception as e:
            self.get_logger().warn(f"Parse error: {e}")

    def handle_nmea_line(self, line):
        if not line.startswith('$'):
            return

        # ── GGA: fix quality + lat/lon + เก็บ raw sentence ──────────────────
        if line.startswith(('$GPGGA', '$GNGGA')):
            # ⭐ เก็บ raw sentence ทั้งบรรทัด (เหมือน waypoint logger)
            with self._gga_lock:
                self._last_gga_sentence = line
            try:
                gga = pynmea2.parse(line)
                self.fix_quality = int(gga.gps_qual)

                self.get_logger().info(
                    f"🛰️  Fix: {FIX_LABELS.get(self.fix_quality, str(self.fix_quality))}",
                    throttle_duration_sec=2.0
                )

                # ⭐ รับ lat/lon แม้ fix_quality=5 (Float) ได้เลย
                if self.fix_quality in ALLOWED_FIX and gga.lat and gga.lon:
                    self._last_lat = self._nmea_to_decimal(gga.lat, gga.lat_dir)
                    self._last_lon = self._nmea_to_decimal(gga.lon, gga.lon_dir)

            except Exception as e:
                self.get_logger().warn(f"GGA parse error: {e}")
            return

        # ── RMC: speed + true_course (heading จาก u-blox ตรงๆ) ──────────────
        if not line.startswith(('$GPRMC', '$GNRMC')):
            return

        # ⭐ กรอง fix quality ก่อน (เหมือน ALLOWED_FIX ของ waypoint logger)
        if self.fix_quality not in ALLOWED_FIX:
            return

        try:
            msg = pynmea2.parse(line)
            if msg.status != 'A':
                return

            now = time.time()
            if now - self.prev_time < 0.1:
                return
            self.prev_time = now

            # ⭐ ใช้ lat/lon จาก GGA ก่อน (แม่นกว่า RMC ใน RTK mode)
            if self._last_lat is not None:
                lat = self._last_lat
                lon = self._last_lon
            else:
                lat = self._nmea_to_decimal(msg.lat, msg.lat_dir)
                lon = self._nmea_to_decimal(msg.lon, msg.lon_dir)

            # แปลงเป็น UTM absolute
            x_utm, y_utm = self.transformer.transform(lon, lat)

            # knots → m/s
            speed = float(msg.spd_over_grnd or 0) * 0.514444

            # ── Heading จาก IMU เป็นหลัก ───────────────────────────────────
            yaw = self.prev_yaw if self.prev_yaw is not None else 0.0
            yaw_source = 'fallback'

            if self.imu_yaw is not None and time.time() - self.imu_yaw_time <= self.imu_yaw_timeout:
                yaw = self.imu_yaw
                self.prev_yaw = yaw
                yaw_source = 'imu'

            elif msg.true_course and msg.true_course != '':
                bearing_deg = float(msg.true_course)
                if speed >= MIN_SPEED_FOR_HEADING:
                    # GPS bearing (0°=North, CW) → ENU yaw (0°=East, CCW)
                    yaw = self._normalize_angle(
                        math.pi / 2.0 - math.radians(bearing_deg)
                    )
                    self.prev_yaw = yaw
                    yaw_source = 'gnss_true_course'
                    self.get_logger().info(
                        f"🧭 Heading from u-blox true_course: {bearing_deg:.1f}°",
                        throttle_duration_sec=1.0
                    )

            # ⭐ fallback จาก UTM consecutive (เหมือนเดิม แต่ใช้เฉพาะเมื่อไม่มี true_course)
            elif (self.prev_x_utm is not None and
                  math.hypot(x_utm - self.prev_x_utm,
                              y_utm - self.prev_y_utm) > 0.1):
                dx = x_utm - self.prev_x_utm
                dy = y_utm - self.prev_y_utm
                yaw = self._normalize_angle(math.atan2(dy, dx))
                self.prev_yaw = yaw
                yaw_source = 'utm_delta'

            self.prev_x_utm = x_utm
            self.prev_y_utm = y_utm

            # ── Publish Odometry ─────────────────────────────────────────────
            odom = Odometry()
            odom.header.stamp    = self.get_clock().now().to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id  = 'base_link'
            odom.pose.pose.position.x = x_utm   # UTM Easting
            odom.pose.pose.position.y = y_utm   # UTM Northing
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = self._yaw_to_quat(yaw)
            odom.pose.covariance[0] = float(self.fix_quality)
            odom.twist.twist.linear.x  = speed
            self.pose_pub.publish(odom)

            self.get_logger().info(
                f"📍 UTM x={x_utm:.3f} y={y_utm:.3f} "
                f"yaw={math.degrees(yaw):.1f}° yaw_source={yaw_source} speed={speed:.2f} "
                f"fix={FIX_LABELS.get(self.fix_quality)}"
            )

        except Exception as e:
            self.get_logger().warn(f"Parse error: {e}")

    # ══════════════════════════════════════════════════════════════════════════
    def _nmea_to_decimal(self, degree_str, direction):
        if not degree_str or degree_str == '0':
            return 0.0
        raw     = float(degree_str)
        degrees = int(raw / 100)
        minutes = raw - degrees * 100
        decimal = degrees + minutes / 60
        if direction in ('S', 'W'):
            decimal = -decimal
        return decimal

    def _yaw_to_quat(self, yaw):
        q   = Quaternion()
        q.w = math.cos(yaw / 2)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2)
        return q

    def _normalize_angle(self, a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    def destroy_node(self):
        if self._ntrip_sock:
            try:
                self._ntrip_sock.close()
            except Exception:
                pass
        self.ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = GnssPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
