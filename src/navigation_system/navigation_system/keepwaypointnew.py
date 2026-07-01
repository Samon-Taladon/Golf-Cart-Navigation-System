#!/usr/bin/env python3

import base64
import csv
import datetime
import math
import os
import socket
import threading
import time

import pynmea2
import rclpy
import serial
from pyproj import Transformer
from rclpy.node import Node
from std_msgs.msg import Float64


LOG_DIR = "/home/inc/ros2_ws/src/navigation_system/logs14"
GNSS_PORT = "/dev/ttyACM0"
GNSS_BAUD = 115200

NTRIP_SERVER_IP = "110.78.0.54"
NTRIP_SERVER_PORT = 2116
NTRIP_USERNAME = "1118600009224"
NTRIP_PASSWORD = "CK79"
NTRIP_MOUNTPOINT = "VRS_RTCM32"
GGA_SEND_INTERVAL = 0.2

WHEELBASE = 1.67
MIN_SPEED = 0.2
MIN_SPEED_FOR_COURSE = 0.5
DIST_THRESHOLD = 0.5
MAX_FIX_PREDICT_ERROR = 1.0
FIX_STABLE_SECONDS = 1.5
FIX_STABLE_SAMPLES = 10
MAX_PREDICT_DT = 0.5

FIX_LABELS = {
    0: "No fix",
    1: "GPS only",
    2: "DGPS",
    4: "RTK Fixed",
    5: "RTK Float",
}


class KeepWaypointNew(Node):
    def __init__(self):
        super().__init__("keepwaypointnew")

        os.makedirs(LOG_DIR, exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.nmea_log = os.path.join(LOG_DIR, f"nmea_{ts}.log")
        self.snr_csv = os.path.join(LOG_DIR, f"snr_{ts}.csv")
        self.waypoint_csv = os.path.join(LOG_DIR, "waypoints_clean.csv")

        self.to_utm = Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)
        self.to_latlon = Transformer.from_crs("epsg:32647", "epsg:4326", always_xy=True)

        self.fix_qual = 0
        self.last_gga_sentence = None
        self.gga_lock = threading.Lock()

        self.last_heading_deg = 0.0
        self.last_ublox_heading_deg = None
        self.last_ublox_heading_time = 0.0

        self.steering_rad = 0.0
        self.steering_time = 0.0
        self.steering_timeout = 1.0

        self.pred_x = None
        self.pred_y = None
        self.pred_yaw = None
        self.pred_time = None

        self.last_fix_x = None
        self.last_fix_y = None
        self.last_fix_yaw = None

        self.last_output_x = None
        self.last_output_y = None

        self.fix_started_time = None
        self.fix_sample_count = 0
        self.was_non_fix = True

        self.create_subscription(
            Float64,
            "/steering/angle_feedback",
            self.steering_callback,
            10,
        )

        self.init_files()
        self.ser = serial.Serial(GNSS_PORT, GNSS_BAUD, timeout=0.02)
        threading.Thread(target=self.ntrip_client, daemon=True).start()
        self.create_timer(0.01, self.read_gnss)

        self.get_logger().info(f"NMEA log: {self.nmea_log}")
        self.get_logger().info(f"SNR log: {self.snr_csv}")
        self.get_logger().info(f"Waypoint file: {self.waypoint_csv}")

    def init_files(self):
        with open(self.snr_csv, "w", newline="") as f:
            csv.writer(f).writerow(["timestamp", "sentence", "sat_id", "elevation", "snr_db"])

        with open(self.waypoint_csv, "w", newline="") as f:
            csv.writer(f).writerow(
                ["lat", "lon", "speed", "heading", "fix_quality", "source"]
            )

    def steering_callback(self, msg):
        self.steering_rad = float(msg.data)
        self.steering_time = time.time()

    def ntrip_client(self):
        while rclpy.ok():
            sock = None
            try:
                self.get_logger().info("Connecting NTRIP...")
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(10.0)
                sock.connect((NTRIP_SERVER_IP, NTRIP_SERVER_PORT))

                auth = base64.b64encode(
                    f"{NTRIP_USERNAME}:{NTRIP_PASSWORD}".encode()
                ).decode()
                request = (
                    f"GET /{NTRIP_MOUNTPOINT} HTTP/1.0\r\n"
                    f"Authorization: Basic {auth}\r\n\r\n"
                )
                sock.sendall(request.encode())
                sock.settimeout(0.1)
                self.get_logger().info("NTRIP connected")

                last_send = 0.0
                while rclpy.ok():
                    try:
                        data = sock.recv(4096)
                        if data:
                            self.ser.write(data)
                    except socket.timeout:
                        pass

                    now = time.time()
                    if now - last_send > GGA_SEND_INTERVAL:
                        with self.gga_lock:
                            gga = self.last_gga_sentence
                        if gga:
                            sock.sendall((gga + "\r\n").encode())
                            last_send = now
            except Exception as exc:
                self.get_logger().warn(f"NTRIP error: {exc}")
                time.sleep(5.0)
            finally:
                if sock is not None:
                    try:
                        sock.close()
                    except Exception:
                        pass

    def read_gnss(self):
        for _ in range(50):
            try:
                line = self.ser.readline().decode("ascii", errors="replace").strip()
            except Exception as exc:
                self.get_logger().warn(f"GNSS serial read error: {exc}")
                return

            if not line:
                break

            self.handle_nmea(line)

            if getattr(self.ser, "in_waiting", 0) <= 0:
                break

    def handle_nmea(self, line):
        if not line.startswith("$"):
            return

        with open(self.nmea_log, "a") as f:
            f.write(f"{time.time():.3f},{line}\n")

        if line.startswith(("$GPGSV", "$GLGSV", "$GAGSV", "$GBGSV", "$GNGSV")):
            self.log_snr(line)
            return

        if line.startswith(("$GPHDT", "$GNHDT", "$GPTHS", "$GNTHS", "$GPVTG", "$GNVTG")):
            heading = self.parse_ublox_heading_sentence(line)
            if heading is not None:
                self.update_ublox_heading(heading)
            return

        if line.startswith(("$GPGGA", "$GNGGA")):
            self.handle_gga(line)
            return

        if line.startswith(("$GPRMC", "$GNRMC")):
            self.handle_rmc(line)

    def log_snr(self, line):
        try:
            gsv = pynmea2.parse(line)
            timestamp = datetime.datetime.now().isoformat()
            with open(self.snr_csv, "a", newline="") as f:
                writer = csv.writer(f)
                for i in range(1, 5):
                    sat = getattr(gsv, f"sv_prn_num_{i}", None)
                    ele = getattr(gsv, f"elevation_{i}", None)
                    snr = getattr(gsv, f"snr_{i}", None)
                    if sat and snr:
                        writer.writerow([timestamp, line[:6], sat, ele, snr])
        except Exception:
            return

    def handle_gga(self, line):
        try:
            gga = pynmea2.parse(line)
            previous = self.fix_qual
            self.fix_qual = int(gga.gps_qual)
            with self.gga_lock:
                self.last_gga_sentence = line

            if self.fix_qual == 4:
                now = time.time()
                if previous != 4:
                    self.fix_started_time = now
                    self.fix_sample_count = 0
                self.fix_sample_count += 1
            else:
                self.fix_started_time = None
                self.fix_sample_count = 0
                self.was_non_fix = True

            self.get_logger().info(
                f"Fix: {FIX_LABELS.get(self.fix_qual, str(self.fix_qual))}",
                throttle_duration_sec=1.0,
            )
        except Exception as exc:
            self.get_logger().warn(f"GGA parse error: {exc}")

    def handle_rmc(self, line):
        try:
            msg = pynmea2.parse(line)
        except Exception as exc:
            self.get_logger().warn(f"RMC parse error: {exc}")
            return

        if msg.status != "A" or not msg.lat or not msg.lon:
            return

        lat = self.nmea_to_decimal(msg.lat, msg.lat_dir)
        lon = self.nmea_to_decimal(msg.lon, msg.lon_dir)
        speed = float(msg.spd_over_grnd or 0.0) * 0.514444
        now = time.time()

        if msg.true_course not in (None, "") and speed >= MIN_SPEED_FOR_COURSE:
            self.update_ublox_heading(msg.true_course)

        heading_deg, heading_is_fresh = self.get_current_heading_deg()
        yaw = self.heading_deg_to_enu_yaw(heading_deg)
        x, y = self.to_utm.transform(lon, lat)

        self.update_prediction(now, speed, yaw, heading_is_fresh, seed_x=x, seed_y=y)

        if self.fix_qual == 4:
            self.handle_fixed_position(lat, lon, x, y, yaw, speed, heading_deg, now)
        else:
            self.handle_predicted_position(speed, heading_deg)

    def handle_fixed_position(self, lat, lon, x, y, yaw, speed, heading_deg, now):
        self.last_fix_x = x
        self.last_fix_y = y
        self.last_fix_yaw = yaw

        if not self.fix_is_stable(now):
            self.get_logger().info(
                "RTK Fix returned; waiting for stable samples before saving",
                throttle_duration_sec=0.5,
            )
            return

        if self.was_non_fix and self.pred_x is not None:
            error = math.hypot(x - self.pred_x, y - self.pred_y)
            if error > MAX_FIX_PREDICT_ERROR:
                self.get_logger().warn(
                    f"Rejected new Fix: {error:.2f} m from predicted path"
                )
                return

        if speed <= MIN_SPEED:
            return

        if not self.output_spacing_ok(x, y):
            return

        self.write_waypoint(lat, lon, speed, heading_deg, self.fix_qual, "fix")
        self.last_output_x = x
        self.last_output_y = y
        self.was_non_fix = False

        self.pred_x = x
        self.pred_y = y
        self.pred_yaw = yaw
        self.pred_time = now

    def handle_predicted_position(self, speed, heading_deg):
        if speed <= MIN_SPEED or self.pred_x is None or self.pred_y is None:
            return

        if not self.output_spacing_ok(self.pred_x, self.pred_y):
            return

        lon, lat = self.to_latlon.transform(self.pred_x, self.pred_y)
        self.write_waypoint(
            lat,
            lon,
            speed,
            heading_deg,
            self.fix_qual,
            "predicted",
        )
        self.last_output_x = self.pred_x
        self.last_output_y = self.pred_y

    def update_prediction(self, now, speed, ublox_yaw, heading_is_fresh, seed_x, seed_y):
        if self.pred_x is None or self.pred_y is None or self.pred_yaw is None:
            self.pred_x = seed_x
            self.pred_y = seed_y
            self.pred_yaw = ublox_yaw
            self.pred_time = now
            return

        if self.pred_time is None:
            self.pred_time = now
            return

        dt = max(0.0, min(now - self.pred_time, MAX_PREDICT_DT))
        self.pred_time = now

        if dt <= 0.0:
            return

        steering = self.get_current_steering()
        v = max(0.0, speed)
        yaw_rate = 0.0
        if WHEELBASE > 0.0:
            yaw_rate = (v / WHEELBASE) * math.tan(steering)

        yaw_ref = ublox_yaw if heading_is_fresh else self.pred_yaw

        # Use fresh u-blox yaw when available. If u-blox heading is stale,
        # keep integrating yaw from the bicycle model and steering feedback.
        mid_yaw = self.normalize_angle(yaw_ref + yaw_rate * dt * 0.5)
        self.pred_x += v * math.cos(mid_yaw) * dt
        self.pred_y += v * math.sin(mid_yaw) * dt
        self.pred_yaw = self.normalize_angle(yaw_ref + yaw_rate * dt)

    def get_current_steering(self):
        if time.time() - self.steering_time <= self.steering_timeout:
            return self.steering_rad
        return 0.0

    def fix_is_stable(self, now):
        if self.fix_started_time is None:
            return False
        stable_time = now - self.fix_started_time >= FIX_STABLE_SECONDS
        stable_samples = self.fix_sample_count >= FIX_STABLE_SAMPLES
        return stable_time or stable_samples

    def output_spacing_ok(self, x, y):
        if self.last_output_x is None:
            return True
        return math.hypot(x - self.last_output_x, y - self.last_output_y) >= DIST_THRESHOLD

    def write_waypoint(self, lat, lon, speed, heading_deg, fix_quality, source):
        with open(self.waypoint_csv, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    f"{lat:.9f}",
                    f"{lon:.9f}",
                    f"{speed:.3f}",
                    f"{heading_deg:.3f}",
                    int(fix_quality),
                    source,
                ]
            )
            f.flush()

        self.get_logger().info(
            f"Saved {source}: lat={lat:.9f} lon={lon:.9f} "
            f"speed={speed:.2f} heading={heading_deg:.1f} fix={fix_quality}"
        )

    def update_ublox_heading(self, heading):
        try:
            heading_deg = float(heading) % 360.0
        except (TypeError, ValueError):
            return
        self.last_ublox_heading_deg = heading_deg
        self.last_ublox_heading_time = time.time()
        self.last_heading_deg = heading_deg

    def get_current_heading_deg(self):
        if (
            self.last_ublox_heading_deg is not None
            and time.time() - self.last_ublox_heading_time <= 2.0
        ):
            return self.last_ublox_heading_deg, True
        return self.last_heading_deg, False

    @staticmethod
    def parse_ublox_heading_sentence(line):
        try:
            msg = pynmea2.parse(line)
        except pynmea2.ParseError:
            msg = None

        if msg is not None and isinstance(msg, pynmea2.types.talker.HDT):
            return msg.heading

        if msg is not None and line.startswith(("$GNTHS", "$GPTHS")):
            return getattr(msg, "heading", None)

        if msg is not None and isinstance(msg, pynmea2.types.talker.VTG):
            return getattr(msg, "true_track", None)

        if line.startswith(("$GPHDT", "$GNHDT", "$GPTHS", "$GNTHS", "$GPVTG", "$GNVTG")):
            return KeepWaypointNew.get_nmea_field(line, 1)

        return None

    @staticmethod
    def get_nmea_field(line, field_index):
        body = line.split("*", 1)[0]
        parts = body.split(",")
        if len(parts) <= field_index or parts[field_index] == "":
            return None
        return parts[field_index]

    @staticmethod
    def nmea_to_decimal(degree_str, direction):
        raw = float(degree_str)
        degrees = int(raw / 100)
        minutes = raw - degrees * 100
        decimal = degrees + minutes / 60.0
        if direction in ("S", "W"):
            decimal = -decimal
        return decimal

    @staticmethod
    def heading_deg_to_enu_yaw(heading_deg):
        return KeepWaypointNew.normalize_angle(math.pi / 2.0 - math.radians(heading_deg))

    @staticmethod
    def normalize_angle(angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def destroy_node(self):
        if hasattr(self, "ser") and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeepWaypointNew()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
