# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32
# import serial

# class IMUHeadingPublisher(Node):
#     def __init__(self):
#         super().__init__('imu_heading_publisher')
#         self.publisher_ = self.create_publisher(Float32, '/navigation/heading', 10)
#         # ปรับพอร์ตให้ตรงกับ MKR Zero + ICM-20948
#         self.ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
#         self.create_timer(0.1, self.timer_callback)

#     def timer_callback(self):
#         try:
#             line = self.ser.readline().decode('utf-8').strip()      
#             if line:
#                 yaw = float(line)
#                 msg = Float32()
#                 msg.data = yaw
#                 self.publisher_.publish(msg)
#         except Exception as e:
#             self.get_logger().warn(f"Read error: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = IMUHeadingPublisher()
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
# from nav_msgs.msg import Path, Odometry
# from geometry_msgs.msg import PoseStamped
# import numpy as np


# class PathVisualizer(Node):

#     def __init__(self):

#         super().__init__('path_visualizer')

#         # โหลด waypoint
#         wp = np.loadtxt(
#             "/home/inc/ros2_ws/src/navigation_system/navigation_system/waypoints100368.csv",
#             delimiter=",",
#             skiprows=1
#         )

#         self.x = wp[:,1]
#         self.y = wp[:,2]

#         # publishers
#         self.path_pub = self.create_publisher(Path, "/global_path", 10)
#         self.traj_pub = self.create_publisher(Path, "/trajectory", 10)
#         self.pose_pub = self.create_publisher(PoseStamped, "/vehicle_pose", 10)

#         # subscriber
#         self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

#         # global path
#         self.global_path = Path()
#         self.global_path.header.frame_id = "odom"

#         for x,y in zip(self.x,self.y):

#             pose = PoseStamped()

#             pose.header.frame_id = "odom"
#             pose.pose.position.x = float(x)
#             pose.pose.position.y = float(y)
#             pose.pose.orientation.w = 1.0

#             self.global_path.poses.append(pose)

#         # trajectory
#         self.traj = Path()
#         self.traj.header.frame_id = "odom"

#         # timer publish path
#         self.timer = self.create_timer(0.5,self.publish_path)

#         self.get_logger().info("Path visualizer started")


#     def publish_path(self):

#         now = self.get_clock().now().to_msg()

#         self.global_path.header.stamp = now

#         for p in self.global_path.poses:
#             p.header.stamp = now

#         self.path_pub.publish(self.global_path)


#     def odom_callback(self,msg):

#         pose = PoseStamped()

#         pose.header.frame_id = "odom"
#         pose.header.stamp = self.get_clock().now().to_msg()

#         pose.pose = msg.pose.pose

#         self.pose_pub.publish(pose)

#         # add trajectory point
#         traj_pose = PoseStamped()

#         traj_pose.header = pose.header
#         traj_pose.pose = pose.pose

#         self.traj.header.stamp = pose.header.stamp
#         self.traj.poses.append(traj_pose)

#         self.traj_pub.publish(self.traj)


# def main():

#     rclpy.init()

#     node = PathVisualizer()

#     rclpy.spin(node)

#     node.destroy_node()

#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()



# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Path, Odometry
# from geometry_msgs.msg import PoseStamped
# import numpy as np


# class PathVisualizer(Node):

#     def __init__(self):
#         super().__init__('path_visualizer')

#         # โหลด waypoint
#         wp = np.loadtxt(
#             "/home/inc/ros2_ws/src/navigation_system/navigation_system/waypoints100368.csv",
#             delimiter=",",
#             skiprows=1
#         )

#         raw_x = wp[:, 1]
#         raw_y = wp[:, 2]

#         # ✅ normalize waypoints ให้ origin เป็น 0,0 (ตรงกับ gnss_pose_publisher)
#         self.origin_x = raw_x[0]
#         self.origin_y = raw_y[0]
#         self.x = raw_x - self.origin_x
#         self.y = raw_y - self.origin_y

#         self.get_logger().info(
#             f"✅ Loaded {len(self.x)} waypoints | "
#             f"UTM origin=({self.origin_x:.2f}, {self.origin_y:.2f})"
#         )

#         # publishers
#         self.path_pub = self.create_publisher(Path, "/global_path", 10)
#         self.traj_pub = self.create_publisher(Path, "/trajectory", 10)
#         self.pose_pub = self.create_publisher(PoseStamped, "/vehicle_pose", 10)

#         # subscriber
#         self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

#         # global path
#         self.global_path = Path()
#         self.global_path.header.frame_id = "odom"

#         for x, y in zip(self.x, self.y):
#             pose = PoseStamped()
#             pose.header.frame_id = "odom"
#             pose.pose.position.x = float(x)
#             pose.pose.position.y = float(y)
#             pose.pose.orientation.w = 1.0
#             self.global_path.poses.append(pose)

#         # trajectory
#         self.traj = Path()
#         self.traj.header.frame_id = "odom"

#         # timer publish path
#         self.timer = self.create_timer(0.5, self.publish_path)

#         self.get_logger().info("✅ Path visualizer started")

#     def publish_path(self):
#         now = self.get_clock().now().to_msg()
#         self.global_path.header.stamp = now
#         for p in self.global_path.poses:
#             p.header.stamp = now
#         self.path_pub.publish(self.global_path)

#     def odom_callback(self, msg):
#         now = self.get_clock().now().to_msg()

#         # publish vehicle pose
#         pose = PoseStamped()
#         pose.header.frame_id = "odom"
#         pose.header.stamp = now
#         pose.pose = msg.pose.pose
#         self.pose_pub.publish(pose)

#         # add trajectory point
#         traj_pose = PoseStamped()
#         traj_pose.header.frame_id = "odom"
#         traj_pose.header.stamp = now
#         traj_pose.pose = msg.pose.pose
#         self.traj.header.stamp = now
#         self.traj.poses.append(traj_pose)
#         self.traj_pub.publish(self.traj)


# def main():
#     rclpy.init()
#     node = PathVisualizer()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()


#!/usr/bin/env python3
"""
debug_visualizer.py
──────────────────────────────────────────────────────────────────────────────
แสดงข้อมูล debug แบบ real-time:
  • ตำแหน่งรถ vs waypoint ปัจจุบัน
  • หัวรถ (yaw) vs มุมที่ควรไป (bearing to target)
  • CTE และ steering angle
  • publish Marker ให้ RViz2 แสดงลูกศรหัวรถ + เส้นจาก รถ → target
──────────────────────────────────────────────────────────────────────────────
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
import math


WAYPOINTS_CSV = "/home/inc/ros2_ws/src/navigation_system/navigation_system/waypoints100368.csv"
LOOKAHEAD     = 2.5   # ต้องตรงกับ pure_pursuit_controller.py


class DebugVisualizer(Node):

    def __init__(self):
        super().__init__('debug_visualizer')

        # โหลด waypoints + normalize
        wp = np.loadtxt(WAYPOINTS_CSV, delimiter=",", skiprows=1)
        raw_x = wp[:, 1]
        raw_y = wp[:, 2]
        self.origin_x = raw_x[0]
        self.origin_y = raw_y[0]
        self.path_x   = raw_x - self.origin_x
        self.path_y   = raw_y - self.origin_y
        self.N        = len(self.path_x)

        self.x   = None
        self.y   = None
        self.yaw = None

        # publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/debug_markers', 10)

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_timer(0.2, self.update)   # 5 Hz

        self.get_logger().info("🔍 Debug Visualizer READY — watch /debug_markers in RViz2")

    # ─────────────────────────────────────────────────────────────────────────
    def odom_cb(self, msg):
        self.x   = msg.pose.pose.position.x
        self.y   = msg.pose.pose.position.y
        self.yaw = self.quat_to_yaw(msg.pose.pose.orientation)

    # ─────────────────────────────────────────────────────────────────────────
    def update(self):
        if self.x is None:
            return

        # หา nearest waypoint
        d       = np.hypot(self.path_x - self.x, self.path_y - self.y)
        nearest = int(np.argmin(d))

        # หา lookahead target (เหมือน pure pursuit)
        target_idx = nearest
        for i in range(nearest, self.N):
            if math.hypot(self.path_x[i] - self.x,
                          self.path_y[i] - self.y) >= LOOKAHEAD:
                target_idx = i
                break

        tx = self.path_x[target_idx]
        ty = self.path_y[target_idx]

        # มุมที่ควรไป
        bearing = math.atan2(ty - self.y, tx - self.x)
        alpha   = self.normalize_angle(bearing - self.yaw)
        delta   = math.atan2(2.0 * 1.67 * math.sin(alpha), LOOKAHEAD)

        # CTE (ระยะจากรถถึง nearest segment)
        cte = self._cte(nearest)

        # ══ LOG ══════════════════════════════════════════════════════════════
        self.get_logger().info(
            f"\n"
            f"┌─── DEBUG ──────────────────────────────────────────\n"
            f"│ 🚗 Vehicle  : x={self.x:+.2f}  y={self.y:+.2f}\n"
            f"│ 🎯 Target   : idx={target_idx}/{self.N}  "
            f"x={tx:+.2f}  y={ty:+.2f}\n"
            f"│ 📐 Yaw (รถ): {math.degrees(self.yaw):+.1f}°\n"
            f"│ 📐 Bearing  : {math.degrees(bearing):+.1f}°\n"
            f"│ 📐 Alpha    : {math.degrees(alpha):+.1f}°  "
            f"(ผิดทิศ {'⚠️ YES' if abs(math.degrees(alpha)) > 90 else '✅ NO'})\n"
            f"│ 🔄 Steer    : {math.degrees(delta):+.1f}°\n"
            f"│ 📏 CTE      : {cte:.3f} m\n"
            f"│ 📏 Dist nearest: {d[nearest]:.2f} m  "
            f"Dist target: {math.hypot(tx-self.x, ty-self.y):.2f} m\n"
            f"└────────────────────────────────────────────────────"
        )

        # ══ MARKERS ══════════════════════════════════════════════════════════
        now  = self.get_clock().now().to_msg()
        arr  = MarkerArray()

        # 1) ลูกศรหัวรถ (สีน้ำเงิน)
        m1 = Marker()
        m1.header.frame_id = 'odom'
        m1.header.stamp    = now
        m1.ns, m1.id       = 'debug', 0
        m1.type            = Marker.ARROW
        m1.action          = Marker.ADD
        m1.scale.x = 2.0; m1.scale.y = 0.3; m1.scale.z = 0.3
        m1.color.r = 0.0; m1.color.g = 0.4; m1.color.b = 1.0; m1.color.a = 1.0
        m1.pose.position.x = self.x
        m1.pose.position.y = self.y
        m1.pose.orientation = self.yaw_to_quat(self.yaw)
        arr.markers.append(m1)

        # 2) เส้นจากรถไป target (สีเหลือง)
        m2 = Marker()
        m2.header.frame_id = 'odom'
        m2.header.stamp    = now
        m2.ns, m2.id       = 'debug', 1
        m2.type            = Marker.LINE_STRIP
        m2.action          = Marker.ADD
        m2.scale.x         = 0.15
        m2.color.r = 1.0; m2.color.g = 1.0; m2.color.b = 0.0; m2.color.a = 1.0
        p1 = Point(); p1.x = self.x;  p1.y = self.y;  p1.z = 0.0
        p2 = Point(); p2.x = tx;      p2.y = ty;       p2.z = 0.0
        m2.points = [p1, p2]
        arr.markers.append(m2)

        # 3) sphere ที่ target waypoint (สีแดง)
        m3 = Marker()
        m3.header.frame_id = 'odom'
        m3.header.stamp    = now
        m3.ns, m3.id       = 'debug', 2
        m3.type            = Marker.SPHERE
        m3.action          = Marker.ADD
        m3.scale.x = m3.scale.y = m3.scale.z = 0.5
        m3.color.r = 1.0; m3.color.g = 0.0; m3.color.b = 0.0; m3.color.a = 1.0
        m3.pose.position.x = tx
        m3.pose.position.y = ty
        arr.markers.append(m3)

        # 4) sphere ที่ nearest waypoint (สีเขียว)
        m4 = Marker()
        m4.header.frame_id = 'odom'
        m4.header.stamp    = now
        m4.ns, m4.id       = 'debug', 3
        m4.type            = Marker.SPHERE
        m4.action          = Marker.ADD
        m4.scale.x = m4.scale.y = m4.scale.z = 0.4
        m4.color.r = 0.0; m4.color.g = 1.0; m4.color.b = 0.0; m4.color.a = 1.0
        m4.pose.position.x = self.path_x[nearest]
        m4.pose.position.y = self.path_y[nearest]
        arr.markers.append(m4)

        # 5) ข้อความ CTE (สีขาว)
        m5 = Marker()
        m5.header.frame_id = 'odom'
        m5.header.stamp    = now
        m5.ns, m5.id       = 'debug', 4
        m5.type            = Marker.TEXT_VIEW_FACING
        m5.action          = Marker.ADD
        m5.scale.z         = 0.5
        m5.color.r = m5.color.g = m5.color.b = m5.color.a = 1.0
        m5.pose.position.x = self.x
        m5.pose.position.y = self.y + 1.0
        m5.pose.position.z = 1.5
        m5.text = (
            f"yaw={math.degrees(self.yaw):+.0f}° "
            f"bear={math.degrees(bearing):+.0f}° "
            f"α={math.degrees(alpha):+.0f}° "
            f"CTE={cte:.2f}m"
        )
        arr.markers.append(m5)

        self.marker_pub.publish(arr)

    # ─────────────────────────────────────────────────────────────────────────
    def _cte(self, nearest):
        P        = np.array([self.x, self.y])
        min_dist = float('inf')
        start    = max(0, nearest - 5)
        end      = min(self.N - 1, nearest + 5)
        for i in range(start, end):
            A  = np.array([self.path_x[i],     self.path_y[i]])
            B  = np.array([self.path_x[i + 1], self.path_y[i + 1]])
            AB = B - A
            AP = P - A
            ab2 = np.dot(AB, AB)
            if ab2 == 0:
                dist = np.linalg.norm(AP)
            else:
                t    = np.clip(np.dot(AP, AB) / ab2, 0.0, 1.0)
                dist = np.linalg.norm(P - (A + t * AB))
            min_dist = min(min_dist, dist)
        return min_dist

    # ─────────────────────────────────────────────────────────────────────────
    def quat_to_yaw(self, q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def yaw_to_quat(self, yaw):
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.w = math.cos(yaw / 2)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2)
        return q

    def normalize_angle(self, a):
        return (a + math.pi) % (2 * math.pi) - math.pi


def main():
    rclpy.init()
    node = DebugVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()