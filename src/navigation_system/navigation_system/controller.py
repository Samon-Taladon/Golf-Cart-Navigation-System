# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64MultiArray
# from geometry_msgs.msg import Twist
# import math
# import time

# class EnhancedNavigationWithStop(Node):
#     def __init__(self):
#         super().__init__('enhanced_navigation_with_stop')

#         self.current_latlon = None
#         self.destination_latlon = None
#         self.in_stop_zone = False
#         self.stop_start_time = None

#         # Subscriptions
#         self.gps_sub = self.create_subscription(
#             Float64MultiArray,
#             '/navigation/raw_gps',
#             self.gps_callback,
#             10
#         )

#         self.dest_sub = self.create_subscription(
#             Float64MultiArray,
#             '/web/selected_destination',
#             self.destination_callback,
#             10
#         )

#         # Publisher to stop the vehicle
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

#         # Timer to check periodically
#         self.create_timer(0.1, self.monitor_destination_proximity)

#         self.get_logger().info("🛰️ Enhanced Navigation with Stop Initialized")

#     def gps_callback(self, msg):
#         self.current_latlon = (msg.data[0], msg.data[1])

#     def destination_callback(self, msg):
#         self.destination_latlon = (msg.data[0], msg.data[1])
#         self.get_logger().info(f"🎯 New destination set to: {self.destination_latlon}")
#         # Reset stop flag in case it's re-used
#         self.in_stop_zone = False
#         self.stop_start_time = None

#     def monitor_destination_proximity(self):
#         if not self.current_latlon or not self.destination_latlon:
#             return

#         distance = self.haversine(self.current_latlon, self.destination_latlon)

#         if distance < 5.0 and not self.in_stop_zone:
#             self.get_logger().info(f"🛑 Reached near destination ({distance:.2f} m), stopping for 15s...")
#             self.send_stop_command()
#             self.in_stop_zone = True
#             self.stop_start_time = time.time()

#         elif self.in_stop_zone:
#             elapsed = time.time() - self.stop_start_time
#             if elapsed >= 15.0:
#                 self.get_logger().info("▶️ 15 seconds passed. Proceeding to continue...")
#                 self.in_stop_zone = False
#                 self.stop_start_time = None
#             else:
#                 # Still waiting
#                 self.send_stop_command()

#     def send_stop_command(self):
#         twist = Twist()
#         twist.linear.x = 0.0
#         twist.angular.z = 0.0
#         self.cmd_vel_pub.publish(twist)

#     def haversine(self, pos1, pos2):
#         R = 6371000  # radius of Earth in meters
#         lat1, lon1 = map(math.radians, pos1)
#         lat2, lon2 = map(math.radians, pos2)
#         dlat = lat2 - lat1
#         dlon = lon2 - lon1
#         a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
#         c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
#         return R * c

# def main(args=None):
#     rclpy.init(args=args)
#     node = EnhancedNavigationWithStop()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()




# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# import numpy as np
# import math


# class PurePursuitController(Node):
#     def __init__(self):
#         super().__init__('pure_pursuit_controller')

#         # ===== PARAMETERS =====
#         self.L          = 1.67    # wheelbase (m)
#         self.Ld         = 2.5     # lookahead distance (m)
#         self.steer_min  = -0.873  # rad
#         self.steer_max  =  0.873  # rad
#         self.dt         = 0.05    # 20 Hz

#         # ===== LOAD WAYPOINTS =====
#         wp = np.loadtxt(
#             "/home/inc/ros2_ws/src/navigation_system/navigation_system/waypoints100368.csv",
#             delimiter=",",
#             skiprows=1
#         )
#         # CSV: time, x, y, psi, speed  (UTM ตรงๆ)
#         self.path_x     = wp[:, 1]
#         self.path_y     = wp[:, 2]
#         self.path_speed = wp[:, 4]
#         self.N          = len(self.path_x)
#         self.get_logger().info(f"✅ Loaded {self.N} waypoints")

#         # ===== STATE =====
#         self.x    = None
#         self.y    = None
#         self.yaw  = None
#         self.speed = 0.0

#         self.target_idx        = 0
#         self.navigation_started = False

#         # ===== LOG =====
#         self.cte_log = []

#         # ===== ROS =====
#         self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.odom_sub = self.create_subscription(
#             Odometry, '/odom', self.odom_callback, 10
#         )
#         self.create_timer(self.dt, self.control_loop)
#         self.get_logger().info("🚗 Pure Pursuit Controller READY")

#     # ================= ODOM CALLBACK =================
#     def odom_callback(self, msg):
#         self.x     = msg.pose.pose.position.x
#         self.y     = msg.pose.pose.position.y
#         self.yaw   = self.quat_to_yaw(msg.pose.pose.orientation)
#         self.speed = msg.twist.twist.linear.x

#     # ================= CONTROL LOOP =================
#     def control_loop(self):
#         if self.x is None or self.yaw is None:
#             self.get_logger().warn("⚠️ Waiting for /odom...", throttle_duration_sec=2.0)
#             return

#         # --- หา waypoint เริ่มต้นที่ใกล้สุด (ครั้งแรกครั้งเดียว) ---
#         if not self.navigation_started:
#             d = np.hypot(self.path_x - self.x, self.path_y - self.y)
#             nearest = int(np.argmin(d))

#             # ✅ ไม่มองข้างหลัง: หา waypoint ที่ใกล้สุด
#             # แต่ต้องอยู่ "ข้างหน้า" ตามทิศทางการขับ
#             # กรองเฉพาะ waypoint ที่อยู่ในทิศหน้า (dot product > 0)
#             forward_idx = nearest
#             for i in range(nearest, self.N):
#                 dx = self.path_x[i] - self.x
#                 dy = self.path_y[i] - self.y
#                 # เวกเตอร์ทิศหน้าของรถ
#                 forward_dot = dx * math.cos(self.yaw) + dy * math.sin(self.yaw)
#                 if forward_dot > 0:
#                     forward_idx = i
#                     break

#             self.target_idx = forward_idx
#             self.navigation_started = True
#             self.get_logger().info(
#                 f"🚀 Start at waypoint {self.target_idx}/{self.N} "
#                 f"(x={self.path_x[self.target_idx]:.2f}, y={self.path_y[self.target_idx]:.2f})"
#             )

#         # --- ถึงจุดหมายแล้ว ---
#         dist_to_end = math.hypot(
#             self.path_x[-1] - self.x,
#             self.path_y[-1] - self.y
#         )
#         if dist_to_end < 1.5:
#             self.get_logger().info("🏁 Reached final waypoint!")
#             self.cmd_pub.publish(Twist())
#             return

#         # --- คำนวณ steering ---
#         steering = self.compute_pure_pursuit()
#         steering = float(np.clip(steering, self.steer_min, self.steer_max))

#         # --- speed จาก waypoint CSV ---
#         spd = float(self.path_speed[self.target_idx])
#         if spd < 0.1:
#             spd = 1.0  # fallback ถ้า CSV ไม่มีความเร็ว

#         # --- publish ---
#         cmd = Twist()
#         cmd.linear.x  = spd
#         cmd.angular.z = steering
#         self.cmd_pub.publish(cmd)

#         # --- log ---
#         cte  = self.compute_cte()
#         self.cte_log.append(cte)
#         rmse = math.sqrt(np.mean(np.square(self.cte_log)))

#         self.get_logger().info(
#             f"idx={self.target_idx}/{self.N} | "
#             f"steer={math.degrees(steering):.1f}° | "
#             f"CTE={cte:.3f}m RMSE={rmse:.3f}m"
#         )

#     # ================= PURE PURSUIT =================
#     def compute_pure_pursuit(self):
#         # ✅ วนจาก target_idx ไปข้างหน้าเท่านั้น (ไม่ย้อนกลับ)
#         found = False
#         for i in range(self.target_idx, self.N):
#             dx = self.path_x[i] - self.x
#             dy = self.path_y[i] - self.y
#             if math.hypot(dx, dy) >= self.Ld:
#                 self.target_idx = i
#                 found = True
#                 break

#         if not found:
#             self.target_idx = self.N - 1

#         tx = self.path_x[self.target_idx]
#         ty = self.path_y[self.target_idx]

#         alpha = math.atan2(ty - self.y, tx - self.x) - self.yaw
#         alpha = self.normalize_angle(alpha)

#         delta = math.atan2(2.0 * self.L * math.sin(alpha), self.Ld)
#         return delta

#     # ================= CTE =================
#     def compute_cte(self):
#         P        = np.array([self.x, self.y])
#         min_dist = float('inf')
#         # คำนวณ CTE เฉพาะ segment ที่ผ่านมาแล้ว (ไม่ต้องคำนวณทั้งเส้น)
#         start = max(0, self.target_idx - 10)
#         end   = min(self.N - 1, self.target_idx + 5)
#         for i in range(start, end):
#             A    = np.array([self.path_x[i],     self.path_y[i]])
#             B    = np.array([self.path_x[i + 1], self.path_y[i + 1]])
#             AB   = B - A
#             AP   = P - A
#             ab2  = np.dot(AB, AB)
#             if ab2 == 0:
#                 dist = np.linalg.norm(AP)
#             else:
#                 t    = np.clip(np.dot(AP, AB) / ab2, 0.0, 1.0)
#                 proj = A + t * AB
#                 dist = np.linalg.norm(P - proj)
#             min_dist = min(min_dist, dist)
#         return min_dist

#     # ================= UTILS =================
#     def quat_to_yaw(self, q):
#         siny = 2.0 * (q.w * q.z + q.x * q.y)
#         cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#         return math.atan2(siny, cosy)

#     def normalize_angle(self, a):
#         return (a + math.pi) % (2 * math.pi) - math.pi


# def main():
#     rclpy.init()
#     node = PurePursuitController()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
"""
pure_pursuit_controller.py — UTM Absolute Version
===================================================
การเปลี่ยนแปลงจากเวอร์ชันก่อน:
  - ลบ origin normalization ออกทั้งหมด
  - ใช้ UTM absolute จาก CSV ตรงๆ — ตรงกับ /odom ที่ publisher ส่งมา
  - รถเริ่มจากจุดไหนก็ได้ — หา nearest waypoint ใน forward direction อัตโนมัติ
  - Pure Pursuit calculation ทำงานได้ปกติกับ UTM absolute
    เพราะใช้แค่ผลต่างพิกัด (dx, dy) ไม่ใช่ absolute value
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import numpy as np
import math


class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # ===== PARAMETERS =====
        self.L         = 1.67    # wheelbase (m) — วัดจากรถจริง
        self.Ld_min    = 1.5     # lookahead distance ขั้นต่ำ (m)
        self.Ld_max    = 5.0     # lookahead distance สูงสุด (m)
        self.Ld_k      = 0.8     # dynamic: Ld = Ld_k * speed + Ld_min
        self.steer_min = -0.873  # rad (−50°)
        self.steer_max =  0.873  # rad (+50°)
        self.dt        = 0.05    # control loop 20 Hz
        self.goal_dist = 1.5     # ระยะที่ถือว่าถึงจุดหมาย (m)

        # ===== HEADING LOW-PASS FILTER =====
        # yaw_filtered = alpha*yaw_new + (1-alpha)*yaw_old
        # alpha น้อย = smooth มาก (ตอบสนองช้า)
        # alpha มาก = ตอบสนองเร็ว (noise มากขึ้น)
        self.yaw_filter_alpha  = 0.3
        self.yaw_filtered: float | None = None

        # ===== STUCK DETECTION =====
        self.stuck_dist_thresh  = 0.3   # m — ถ้าขยับน้อยกว่านี้ใน stuck_time_thresh วิ
        self.stuck_time_thresh  = 5.0   # วิ
        self.last_x_check       = None
        self.last_y_check       = None
        self.last_progress_time = None

        # ===== LOAD WAYPOINTS (UTM Absolute) =====
        wp = np.loadtxt(
            "/home/inc/ros2_ws/src/navigation_system/navigation_system/waypoints100368.csv",
            delimiter=",",
            skiprows=1
        )
        # CSV columns: index, x_utm, y_utm, yaw_enu, speed, [fix_quality]
        # ✅ ใช้ UTM absolute ตรงๆ — ไม่ normalize
        self.path_x     = wp[:, 1]   # UTM Easting  (เช่น 700123.45)
        self.path_y     = wp[:, 2]   # UTM Northing (เช่น 1500456.78)
        self.path_yaw   = wp[:, 3]   # ENU yaw (radians) — reference
        self.path_speed = wp[:, 4]   # m/s
        self.N          = len(self.path_x)

        self.get_logger().info(
            f"✅ Loaded {self.N} waypoints (UTM Absolute)\n"
            f"   First: ({self.path_x[0]:.2f}, {self.path_y[0]:.2f})\n"
            f"   Last : ({self.path_x[-1]:.2f}, {self.path_y[-1]:.2f})"
        )

        # ===== VEHICLE STATE =====
        self.x     = None   # UTM Easting (จาก /odom)
        self.y     = None   # UTM Northing (จาก /odom)
        self.yaw   = None   # ENU yaw (radians)
        self.imu_yaw = None
        self.imu_yaw_time = None
        self.imu_yaw_timeout = 1.0
        self.steer_feedback = None
        self.steer_feedback_time = None
        self.speed = 0.0    # m/s

        self.target_idx         = 0
        self.navigation_started = False

        # ===== LOG =====
        self.cte_log = []

        # ===== ROS =====
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.imu_yaw_sub = self.create_subscription(
            Float64, '/imu/yaw', self.imu_yaw_callback, 10
        )
        self.steer_feedback_sub = self.create_subscription(
            Float64, '/steering/angle_feedback', self.steer_feedback_callback, 10
        )
        self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("🚗 Pure Pursuit Controller READY — UTM Absolute Mode")

    # ══════════════════════════════════════════════════════════════════════════
    #  ODOM CALLBACK
    # ══════════════════════════════════════════════════════════════════════════
    def odom_callback(self, msg):
        # ✅ รับ UTM absolute ตรงจาก publisher
        self.x     = msg.pose.pose.position.x   # UTM Easting
        self.y     = msg.pose.pose.position.y   # UTM Northing
        self.speed = msg.twist.twist.linear.x

    def imu_yaw_callback(self, msg):
        raw_yaw = self.normalize_angle(msg.data)
        self.imu_yaw = raw_yaw
        self.imu_yaw_time = self.get_clock().now()

        # ✅ Low-pass filter บน yaw (handle angle wrap-around ด้วย)
        if self.yaw_filtered is None:
            self.yaw_filtered = raw_yaw
        else:
            diff = self.normalize_angle(raw_yaw - self.yaw_filtered)
            self.yaw_filtered = self.normalize_angle(
                self.yaw_filtered + self.yaw_filter_alpha * diff
            )

        self.yaw = self.yaw_filtered

    def steering_feedback_callback(self, msg):
        self.steer_feedback = msg.data
        self.steer_feedback_time = self.get_clock().now()

    # ══════════════════════════════════════════════════════════════════════════
    #  CONTROL LOOP
    # ══════════════════════════════════════════════════════════════════════════
    def control_loop(self):
        if self.x is None or self.yaw is None:
            self.get_logger().warn(
                "⚠️ Waiting for /odom and /imu/yaw...", throttle_duration_sec=2.0
            )
            return

        if self.imu_yaw_time is None or (
            self.get_clock().now() - self.imu_yaw_time
        ).nanoseconds / 1e9 > self.imu_yaw_timeout:
            self.get_logger().warn(
                "⚠️ /imu/yaw timeout; stopping vehicle",
                throttle_duration_sec=1.0,
            )
            self.cmd_pub.publish(Twist())
            return

        # ── หา waypoint เริ่มต้น (ทำครั้งเดียวตอนเริ่ม) ──────────────────
        if not self.navigation_started:
            self._find_start_waypoint()

        # ── ตรวจสอบถึงจุดหมายแล้ว ────────────────────────────────────────
        dist_to_end = math.hypot(
            self.path_x[-1] - self.x,
            self.path_y[-1] - self.y
        )
        if dist_to_end < self.goal_dist:
            self.get_logger().info("🏁 Reached final waypoint! Stopping.")
            self.cmd_pub.publish(Twist())
            return

        # ── Stuck detection ───────────────────────────────────────────────
        self._check_stuck()

        # ── Dynamic lookahead ─────────────────────────────────────────────
        # ✅ Ld เพิ่มตาม speed — โค้งแคบใช้ Ld น้อย, ทางตรงใช้ Ld มาก
        Ld = float(np.clip(
            self.Ld_k * self.speed + self.Ld_min,
            self.Ld_min, self.Ld_max
        ))

        # ── คำนวณ steering ───────────────────────────────────────────────
        steering = self.compute_pure_pursuit(Ld)
        steering = float(np.clip(steering, self.steer_min, self.steer_max))

        # ── speed จาก waypoint CSV ───────────────────────────────────────
        spd = float(self.path_speed[self.target_idx])
        if spd < 0.1:
            spd = 1.0   # ค่า default ถ้า speed ใน CSV ผิดปกติ

        # ── Publish ──────────────────────────────────────────────────────
        cmd = Twist()
        cmd.linear.x  = spd
        cmd.angular.z = steering
        self.cmd_pub.publish(cmd)

        # ── Log ──────────────────────────────────────────────────────────
        cte  = self.compute_cte()
        self.cte_log.append(cte)
        rmse = math.sqrt(np.mean(np.square(self.cte_log)))

        self.get_logger().info(
            f"idx={self.target_idx}/{self.N} | "
            f"Ld={Ld:.2f}m | "
            f"yaw_imu={math.degrees(self.yaw):.1f}° | "
            f"steer_cmd={math.degrees(steering):.1f}° | "
            f"steer_feedback={self._format_angle_deg(self.steer_feedback)} | "
            f"spd={spd:.2f}m/s | "
            f"CTE={cte:.3f}m RMSE={rmse:.3f}m"
        )

    # ══════════════════════════════════════════════════════════════════════════
    #  FIND START WAYPOINT
    # ══════════════════════════════════════════════════════════════════════════
    def _find_start_waypoint(self):
        """
        หา waypoint เริ่มต้นที่ใกล้สุดและอยู่ข้างหน้า (forward direction)
        รองรับการเริ่มจากจุดไหนก็ได้บนเส้นทาง
        """
        # หา nearest waypoint ก่อน
        dists   = np.hypot(self.path_x - self.x, self.path_y - self.y)
        nearest = int(np.argmin(dists))

        # ✅ หา waypoint แรกที่อยู่ข้างหน้าในทิศที่รถหัน
        forward_idx = nearest
        search_end  = min(nearest + 30, self.N)  # search ไปข้างหน้า 30 จุด

        for i in range(nearest, search_end):
            dx  = self.path_x[i] - self.x
            dy  = self.path_y[i] - self.y
            # dot product กับ heading vector
            dot = dx * math.cos(self.yaw) + dy * math.sin(self.yaw)
            if dot > 0:
                forward_idx = i
                break

        self.target_idx         = forward_idx
        self.navigation_started = True
        self.last_x_check       = self.x
        self.last_y_check       = self.y
        self.last_progress_time = self.get_clock().now().nanoseconds * 1e-9

        nearest_dist = float(dists[nearest])
        self.get_logger().info(
            f"🚀 Navigation started!\n"
            f"   Vehicle pos : ({self.x:.2f}, {self.y:.2f})\n"
            f"   Nearest wp  : idx={nearest} dist={nearest_dist:.2f}m\n"
            f"   Start wp    : idx={self.target_idx} "
            f"({self.path_x[self.target_idx]:.2f}, {self.path_y[self.target_idx]:.2f})"
        )

        if nearest_dist > 5.0:
            self.get_logger().warn(
                f"⚠️ รถอยู่ห่างจาก waypoint ใกล้สุด {nearest_dist:.1f} ม. "
                f"— ตรวจสอบว่ารถอยู่ถูกเส้นทางหรือไม่"
            )

    # ══════════════════════════════════════════════════════════════════════════
    #  STUCK DETECTION
    # ══════════════════════════════════════════════════════════════════════════
    def _check_stuck(self):
        if self.last_x_check is None:
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        moved   = math.hypot(
            self.x - self.last_x_check,
            self.y - self.last_y_check
        )

        if moved > self.stuck_dist_thresh:
            # รถขยับแล้ว — reset timer
            self.last_x_check       = self.x
            self.last_y_check       = self.y
            self.last_progress_time = now_sec
        elif now_sec - self.last_progress_time > self.stuck_time_thresh:
            # รถไม่ขยับนานเกินไป — ข้าม waypoint
            skip = 3
            self.get_logger().warn(
                f"⚠️ Stuck! ข้าม {skip} waypoints "
                f"(idx {self.target_idx} → {self.target_idx + skip})"
            )
            self.target_idx         = min(self.target_idx + skip, self.N - 1)
            self.last_x_check       = self.x
            self.last_y_check       = self.y
            self.last_progress_time = now_sec

    # ══════════════════════════════════════════════════════════════════════════
    #  PURE PURSUIT
    # ══════════════════════════════════════════════════════════════════════════
    def compute_pure_pursuit(self, Ld: float) -> float:
        """
        Pure Pursuit ใน ENU frame กับ UTM absolute coordinates
        ทำงานได้ปกติเพราะใช้แค่ผลต่าง (dx, dy) ไม่ใช่ absolute value
        """
        # หา lookahead point — waypoint แรกที่ห่างจากรถ >= Ld
        found = False
        for i in range(self.target_idx, self.N):
            dx = self.path_x[i] - self.x
            dy = self.path_y[i] - self.y
            if math.hypot(dx, dy) >= Ld:
                self.target_idx = i
                found = True
                break

        if not found:
            self.target_idx = self.N - 1

        tx = self.path_x[self.target_idx]
        ty = self.path_y[self.target_idx]

        # ✅ alpha = มุมของ lookahead point ใน vehicle frame
        #    atan2(dy, dx) → absolute angle ใน ENU
        #    ลบ self.yaw (ENU) → มุมใน vehicle frame
        alpha = self.normalize_angle(
            math.atan2(ty - self.y, tx - self.x) - self.yaw
        )

        # ✅ Pure Pursuit steering: δ = atan(2L·sin(α) / Ld)
        delta = math.atan2(2.0 * self.L * math.sin(alpha), Ld)
        return delta

    # ══════════════════════════════════════════════════════════════════════════
    #  CROSS TRACK ERROR
    # ══════════════════════════════════════════════════════════════════════════
    def compute_cte(self) -> float:
        """
        คำนวณระยะตั้งฉากจากรถถึงเส้นทาง (Cross Track Error)
        ทำงานได้กับ UTM absolute เพราะใช้แค่ผลต่างพิกัด
        """
        P        = np.array([self.x, self.y])
        min_dist = float('inf')

        start = max(0, self.target_idx - 10)
        end   = min(self.N - 1, self.target_idx + 5)

        for i in range(start, end):
            A  = np.array([self.path_x[i],     self.path_y[i]])
            B  = np.array([self.path_x[i + 1], self.path_y[i + 1]])
            AB = B - A
            AP = P - A
            ab2 = np.dot(AB, AB)
            if ab2 < 1e-9:
                d = np.linalg.norm(AP)
            else:
                t    = float(np.clip(np.dot(AP, AB) / ab2, 0.0, 1.0))
                proj = A + t * AB
                d    = np.linalg.norm(P - proj)
            min_dist = min(min_dist, d)

        return min_dist

    # ══════════════════════════════════════════════════════════════════════════
    #  UTILS
    # ══════════════════════════════════════════════════════════════════════════
    def quat_to_yaw(self, q) -> float:
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def normalize_angle(self, a: float) -> float:
        return (a + math.pi) % (2 * math.pi) - math.pi

    @staticmethod
    def _format_angle_deg(angle):
        if angle is None:
            return "None"
        return f"{math.degrees(angle):.1f}°"


def main():
    rclpy.init()
    node = PurePursuitController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
