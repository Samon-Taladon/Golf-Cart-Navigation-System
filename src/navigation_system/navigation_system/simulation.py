
# import numpy as np
# import matplotlib.pyplot as plt
# import pandas as pd
# import math
# import os

# class BangBangWaypointSimulation:
#     def __init__(self, csv_file_path, final_heading=0.0, waypoint_tolerance=2.0, heading_tolerance=np.deg2rad(10)):
#         """
#         🎯 Simulation ของ Bang-Bang Controller ที่รับค่าจากไฟล์ CSV
        
#         Parameters:
#         - csv_file_path: path ไปยังไฟล์ CSV ที่มี waypoints
#         - final_heading: ทิศทางสุดท้ายที่ต้องการ (radians)
#         - waypoint_tolerance: ระยะทางที่ถือว่าถึง waypoint แล้ว (meters)
#         - heading_tolerance: ความคลาดเคลื่อนทิศทางที่ยอมรับได้ (radians)
#         """
#         # โหลด waypoints จากไฟล์ CSV
#         self.waypoints = self.load_waypoints_from_csv(csv_file_path)
        
#         self.final_heading = final_heading
#         self.waypoint_tolerance = waypoint_tolerance
#         self.heading_tolerance = heading_tolerance
#         self.current_target_idx = 0
        
#         # Vehicle parameters (ตามโค้ดจริงของคุณ)
#         self.wheelbase = 1.67  # เมตร
#         self.max_steering_angle = 0.873  # rad (50 degrees)
#         self.fixed_speed = 1.0  # m/s
#         self.max_angular_velocity = self.fixed_speed * np.tan(self.max_steering_angle) / self.wheelbase
        
#         # สถานะปัจจุบัน - เริ่มที่ waypoint แรก
#         if self.waypoints:
#             self.current_pos = np.array(self.waypoints[0], dtype=float)
#         else:
#             self.current_pos = np.array([0.0, 0.0])
        
#         self.current_heading = 0.0  # radians
        
#         # เก็บประวัติการเคลื่อนที่
#         self.path = [self.current_pos.copy()]
#         self.heading_history = [self.current_heading]
#         self.steering_commands = []
#         self.distances_to_target = []
#         self.heading_errors = []
#         self.waypoint_reached_times = []
        
#         # สถิติ
#         self.stats = {
#             'left_turns': 0,
#             'right_turns': 0,
#             'straight_commands': 0,
#             'waypoints_reached': 0,
#             'total_distance_traveled': 0.0,
#             'simulation_time': 0.0
#         }
        
#         print(f"🚗 Vehicle Configuration:")
#         print(f"   Max Steering: ±{np.rad2deg(self.max_steering_angle):.1f}°")
#         print(f"   Max Angular Velocity: ±{self.max_angular_velocity:.3f} rad/s")
#         print(f"   Fixed Speed: {self.fixed_speed} m/s")
#         print(f"   Waypoint Tolerance: {self.waypoint_tolerance} m")
#         print(f"   Heading Tolerance: ±{np.rad2deg(self.heading_tolerance):.1f}°")
#         print(f"   Total Waypoints: {len(self.waypoints)}")
        
#         if self.waypoints:
#             print(f"   First waypoint: ({self.waypoints[0][0]:.2f}, {self.waypoints[0][1]:.2f})")
#             print(f"   Last waypoint: ({self.waypoints[-1][0]:.2f}, {self.waypoints[-1][1]:.2f})")
    
#     def load_waypoints_from_csv(self, csv_file_path):
#         """โหลด waypoints จากไฟล์ CSV"""
#         try:
#             print(f"📂 Loading waypoints from: {csv_file_path}")
#             df = pd.read_csv(csv_file_path)
#             print(f"📊 CSV file has {len(df)} rows")
#             print(f"📊 Available columns: {list(df.columns)}")
            
#             # ตรวจสอบคอลัมน์ที่มีอยู่
#             if 'x_east' in df.columns and 'y_north' in df.columns:
#                 # ใช้ UTM coordinates โดยตรง
#                 waypoints = list(zip(df['x_east'], df['y_north']))
#                 print(f'📍 Loaded {len(waypoints)} UTM waypoints from {csv_file_path}')
#                 print(f'📍 X range: {df["x_east"].min():.1f} to {df["x_east"].max():.1f}')
#                 print(f'📍 Y range: {df["y_north"].min():.1f} to {df["y_north"].max():.1f}')
                
#             elif 'lat' in df.columns and 'lon' in df.columns:
#                 # ใช้ lat/lon coordinates
#                 waypoints = list(zip(df['lon'], df['lat']))
#                 print(f'📍 Loaded {len(waypoints)} lat/lon waypoints from {csv_file_path}')
                
#             elif 'x' in df.columns and 'y' in df.columns:
#                 # ใช้ x,y coordinates ทั่วไป
#                 waypoints = list(zip(df['x'], df['y']))
#                 print(f'📍 Loaded {len(waypoints)} x,y waypoints from {csv_file_path}')
                
#             else:
#                 print(f'❌ Invalid CSV format. Expected columns: x_east,y_north or lat,lon or x,y')
#                 print(f'Available columns: {list(df.columns)}')
#                 return []
            
#             return waypoints
            
#         except Exception as e:
#             print(f'❌ Failed to load waypoints from {csv_file_path}: {e}')
#             return []
    
#     def normalize_angle(self, angle):
#         """ปรับมุมให้อยู่ในช่วง [-π, π]"""
#         return (angle + np.pi) % (2 * np.pi) - np.pi
    
#     def get_target_heading(self, current_pos, target_waypoint_idx):
#         """คำนวณทิศทางที่ต้องการไปยัง waypoint เป้าหมาย"""
#         if target_waypoint_idx >= len(self.waypoints):
#             return self.final_heading
            
#         target_wp = self.waypoints[target_waypoint_idx]
#         dx = target_wp[0] - current_pos[0]
#         dy = target_wp[1] - current_pos[1]
        
#         return np.arctan2(dy, dx)
    
#     def update_target_waypoint(self):
#         """อัพเดท waypoint เป้าหมายถ้าถึงจุดปัจจุบันแล้ว"""
#         if self.current_target_idx < len(self.waypoints):
#             current_wp = self.waypoints[self.current_target_idx]
#             distance = np.hypot(current_wp[0] - self.current_pos[0], 
#                               current_wp[1] - self.current_pos[1])
            
#             if distance < self.waypoint_tolerance and self.current_target_idx < len(self.waypoints) - 1:
#                 self.waypoint_reached_times.append(len(self.path))
#                 self.current_target_idx += 1
#                 self.stats['waypoints_reached'] += 1
#                 print(f"✅ Reached waypoint {self.current_target_idx-1}, moving to waypoint {self.current_target_idx}")
#                 return True
#         return False
    
#     def step(self, dt=0.1):
#         """จำลองการทำงาน 1 step (ตามโค้ดจริงของคุณ)"""
#         if self.current_target_idx >= len(self.waypoints):
#             return False  # Mission complete
        
#         # อัพเดท waypoint เป้าหมาย
#         waypoint_changed = self.update_target_waypoint()
        
#         # หาทิศทางที่ต้องการ
#         target_heading = self.get_target_heading(self.current_pos, self.current_target_idx)
        
#         # คำนวณความผิดพลาดของทิศทาง
#         heading_error = self.normalize_angle(target_heading - self.current_heading)
        
#         # คำนวณระยะทางถึงเป้าหมาย
#         if self.current_target_idx < len(self.waypoints):
#             target_wp = self.waypoints[self.current_target_idx]
#             distance_to_target = np.hypot(target_wp[0] - self.current_pos[0], 
#                                         target_wp[1] - self.current_pos[1])
#         else:
#             distance_to_target = 0.0
        
#         # 🌟 Bang-Bang Control Logic (ตามโค้ดจริงของคุณ)
#         if abs(heading_error) <= self.heading_tolerance:
#             # ไปตรง
#             angular_velocity = 0.0
#             steering_command = "STRAIGHT"
#             self.stats['straight_commands'] += 1
#         elif heading_error > 0:
#             # เลี้ยวซ้าย - ใช้ค่าสูงสุด
#             angular_velocity = self.max_angular_velocity
#             steering_command = "TURN_LEFT_MAX"
#             self.stats['left_turns'] += 1
#         else:
#             # เลี้ยวขวา - ใช้ค่าสูงสุด
#             angular_velocity = -self.max_angular_velocity
#             steering_command = "TURN_RIGHT_MAX"
#             self.stats['right_turns'] += 1
        
#         # อัพเดททิศทาง
#         self.current_heading += angular_velocity * dt
#         self.current_heading = self.normalize_angle(self.current_heading)
        
#         # เคลื่อนที่ไปข้างหน้า
#         if distance_to_target > 0.1:
#             velocity = self.fixed_speed
#         else:
#             velocity = 0.0
        
#         # อัพเดทตำแหน่ง
#         old_pos = self.current_pos.copy()
#         self.current_pos[0] += velocity * np.cos(self.current_heading) * dt
#         self.current_pos[1] += velocity * np.sin(self.current_heading) * dt
        
#         # คำนวณระยะทางที่เดินทาง
#         distance_traveled = np.hypot(self.current_pos[0] - old_pos[0], 
#                                    self.current_pos[1] - old_pos[1])
#         self.stats['total_distance_traveled'] += distance_traveled
#         self.stats['simulation_time'] += dt
        
#         # เก็บประวัติ
#         self.path.append(self.current_pos.copy())
#         self.heading_history.append(self.current_heading)
#         self.steering_commands.append(steering_command)
#         self.distances_to_target.append(distance_to_target)
#         self.heading_errors.append(np.rad2deg(heading_error))
        
#         return True
    
#     def run_simulation(self, max_steps=15000):
#         """รัน simulation จนจบ"""
#         print("🚀 Starting Bang-Bang Controller simulation...")
#         print(f"📊 Max simulation steps: {max_steps}")
        
#         for step in range(max_steps):
#             if not self.step():
#                 print(f"✅ Mission completed in {step} steps ({self.stats['simulation_time']:.1f} seconds)!")
#                 break
            
#             if step % 500 == 0 and step > 0:
#                 progress = (self.current_target_idx / len(self.waypoints)) * 100
#                 print(f"Step {step}: Waypoint {self.current_target_idx}/{len(self.waypoints)} ({progress:.1f}%), "
#                       f"Distance: {self.distances_to_target[-1]:.1f}m")
        
#         self.print_final_statistics()
    
#     def print_final_statistics(self):
#         """แสดงสถิติสุดท้าย"""
#         print(f"\n📊 Final Simulation Statistics:")
#         print(f"   ✅ Waypoints reached: {self.stats['waypoints_reached']}/{len(self.waypoints)-1}")
#         print(f"   📏 Total distance traveled: {self.stats['total_distance_traveled']:.1f} m")
#         print(f"   ⏱️  Total simulation time: {self.stats['simulation_time']:.1f} s")
#         print(f"   🔄 Left turns: {self.stats['left_turns']}")
#         print(f"   🔄 Right turns: {self.stats['right_turns']}")
#         print(f"   ➡️  Straight commands: {self.stats['straight_commands']}")
        
#         total_commands = self.stats['left_turns'] + self.stats['right_turns'] + self.stats['straight_commands']
#         if total_commands > 0:
#             straight_percentage = (self.stats['straight_commands'] / total_commands) * 100
#             print(f"   📈 Straight driving: {straight_percentage:.1f}%")
    
#     def plot_results(self):
#         """แสดงผลการจำลองที่ปรับปรุงสำหรับ waypoints จำนวนมาก"""
#         fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(18, 14))
        
#         path = np.array(self.path)
#         waypoints = np.array(self.waypoints)
        
#         print(f"🎯 Plotting results for {len(waypoints)} waypoints")
        
#         # 1. Path และ Waypoints - ปรับปรุงสำหรับจำนวนจุดมาก
#         # ปรับขนาดจุดและการแสดงผลตามจำนวน waypoints
#         if len(waypoints) > 300:
#             marker_size = 5
#             show_every_nth = max(1, len(waypoints) // 20)  # แสดงหมายเลขทุก 20 จุด
#             path_alpha = 0.8
#             waypoint_alpha = 0.6
#         elif len(waypoints) > 100:
#             marker_size = 8
#             show_every_nth = max(1, len(waypoints) // 15)  # แสดงหมายเลขทุก 15 จุด
#             path_alpha = 0.8
#             waypoint_alpha = 0.7
#         elif len(waypoints) > 50:
#             marker_size = 15
#             show_every_nth = max(1, len(waypoints) // 10)  # แสดงหมายเลขทุก 10 จุด
#             path_alpha = 0.8
#             waypoint_alpha = 0.8
#         else:
#             marker_size = 50
#             show_every_nth = 1
#             path_alpha = 0.7
#             waypoint_alpha = 1.0
        
#         # Plot path และ waypoints ทั้งหมด
#         ax1.plot(path[:, 0], path[:, 1], 'b-', linewidth=2, 
#                 label='Vehicle Path', alpha=path_alpha)
        
#         # 🔧 แสดง waypoints ทั้งหมด
#         ax1.scatter(waypoints[:, 0], waypoints[:, 1], c='red', s=marker_size, 
#                    marker='o', label=f'All Waypoints ({len(waypoints)})', 
#                    zorder=5, alpha=waypoint_alpha)
        
#         # แสดงหมายเลขเฉพาะบางจุดเพื่อไม่ให้รกรุงรัง
#         for i in range(0, len(waypoints), show_every_nth):
#             wp = waypoints[i]
#             ax1.annotate(f'{i}', (wp[0], wp[1]), xytext=(3, 3), 
#                         textcoords='offset points', fontsize=6, fontweight='bold',
#                         bbox=dict(boxstyle="round,pad=0.1", facecolor="white", alpha=0.7))
        
#         # แสดงจุดเริ่มต้นและจุดสิ้นสุดพิเศษ
#         if len(waypoints) > 0:
#             ax1.scatter(waypoints[0, 0], waypoints[0, 1], c='green', s=marker_size*3, 
#                        marker='s', label='Start', zorder=6)
#             ax1.scatter(waypoints[-1, 0], waypoints[-1, 1], c='purple', s=marker_size*3, 
#                        marker='s', label='End', zorder=6)
        
#         # แสดงจุดที่ถึง waypoint ด้วยดาวสีเขียว
#         for reach_time in self.waypoint_reached_times:
#             if reach_time < len(path):
#                 ax1.scatter(path[reach_time, 0], path[reach_time, 1], 
#                            c='lime', s=marker_size*2, marker='*', zorder=6, alpha=0.8)
        
#         ax1.set_title(f'🎯 Bang-Bang Controller Path ({len(waypoints)} waypoints)')
#         ax1.set_xlabel('X East (meters)')
#         ax1.set_ylabel('Y North (meters)')
#         ax1.legend()
#         ax1.grid(True, alpha=0.3)
#         ax1.axis('equal')
        
#         # 2. Steering Commands
#         time_steps = np.arange(len(self.steering_commands))
#         steering_values = []
#         colors = []
        
#         for cmd in self.steering_commands:
#             if cmd == "TURN_LEFT_MAX":
#                 steering_values.append(1)
#                 colors.append('red')
#             elif cmd == "TURN_RIGHT_MAX":
#                 steering_values.append(-1)
#                 colors.append('blue')
#             else:
#                 steering_values.append(0)
#                 colors.append('green')
        
#         ax2.scatter(time_steps, steering_values, c=colors, alpha=0.6, s=1)
#         ax2.set_xlabel('Time Steps')
#         ax2.set_ylabel('Steering Command')
#         ax2.set_title('🎮 Steering Commands Over Time')
#         ax2.set_ylim(-1.5, 1.5)
#         ax2.grid(True, alpha=0.3)
        
#         # Legend for steering
#         ax2.scatter([], [], c='red', label='Turn Left Max', s=20)
#         ax2.scatter([], [], c='blue', label='Turn Right Max', s=20)
#         ax2.scatter([], [], c='green', label='Straight', s=20)
#         ax2.legend()
        
#         # 3. Distance to Target
#         ax3.plot(self.distances_to_target, 'purple', linewidth=2, alpha=0.8)
#         ax3.set_xlabel('Time Steps')
#         ax3.set_ylabel('Distance to Target (m)')
#         ax3.set_title('📏 Distance to Current Waypoint')
#         ax3.grid(True, alpha=0.3)
        
#         # เพิ่มเส้นแสดงการถึง waypoint
#         for reach_time in self.waypoint_reached_times:
#             if reach_time < len(self.distances_to_target):
#                 ax3.axvline(x=reach_time, color='green', linestyle='--', alpha=0.7, linewidth=1)
        
#         # 4. Heading Error
#         ax4.plot(self.heading_errors, 'orange', linewidth=2, alpha=0.8)
#         ax4.axhline(y=np.rad2deg(self.heading_tolerance), color='red', linestyle='--', 
#                    label=f'Tolerance (±{np.rad2deg(self.heading_tolerance):.1f}°)', alpha=0.7)
#         ax4.axhline(y=-np.rad2deg(self.heading_tolerance), color='red', linestyle='--', alpha=0.7)
#         ax4.set_xlabel('Time Steps')
#         ax4.set_ylabel('Heading Error (degrees)')
#         ax4.set_title('🧭 Heading Error Over Time')
#         ax4.legend()
#         ax4.grid(True, alpha=0.3)
        
#         plt.tight_layout()
#         plt.show()
        
#         # เพิ่มสถิติในกราฟ
#         self.plot_statistics()
    
#     def plot_statistics(self):
#         """แสดงกราฟสถิติเพิ่มเติม"""
#         fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
        
#         # 1. Steering Command Distribution
#         commands = ['Straight', 'Turn Left', 'Turn Right']
#         counts = [self.stats['straight_commands'], 
#                  self.stats['left_turns'], 
#                  self.stats['right_turns']]
#         colors = ['green', 'red', 'blue']
        
#         ax1.pie(counts, labels=commands, colors=colors, autopct='%1.1f%%', startangle=90)
#         ax1.set_title('🎮 Steering Command Distribution')
        
#         # 2. Performance Metrics
#         metrics = ['Waypoints\nReached', 'Total Distance\n(m)', 'Simulation Time\n(s)']
#         values = [self.stats['waypoints_reached'], 
#                  self.stats['total_distance_traveled'], 
#                  self.stats['simulation_time']]
        
#         bars = ax2.bar(metrics, values, color=['lightblue', 'lightgreen', 'lightyellow'])
#         ax2.set_title('📊 Performance Metrics')
#         ax2.set_ylabel('Values')
        
#         # เพิ่มค่าบนแท่งกราฟ
#         for bar, value in zip(bars, values):
#             height = bar.get_height()
#             ax2.text(bar.get_x() + bar.get_width()/2., height + height*0.01,
#                     f'{value:.1f}', ha='center', va='bottom')
        
#         plt.tight_layout()
#         plt.show()

# # 🌟 การใช้งานหลัก
# if __name__ == "__main__":
#     # ใช้ไฟล์ waypoints จริงของคุณ
#     csv_file = "/home/inc/ros2_ws/waypointfile/waypoints1m.csv"
    
#     # ตรวจสอบว่าไฟล์มีอยู่หรือไม่
#     if not os.path.exists(csv_file):
#         print(f"❌ File not found: {csv_file}")
#         print("Please check the file path and try again.")
#         exit(1)
    
#     # สร้าง simulation
#     sim = BangBangWaypointSimulation(
#         csv_file_path=csv_file,
#         final_heading=np.deg2rad(0),        # หันหน้าไปทางเหนือ
#         waypoint_tolerance=2.0,             # ยอมรับความผิดพลาด 2 เมตร
#         heading_tolerance=np.deg2rad(10)    # ยอมรับความผิดพลาด 10 องศา
#     )
    
#     # รัน simulation
#     sim.run_simulation(max_steps=20000)  # เพิ่ม max_steps สำหรับ waypoints จำนวนมาก
    
#     # แสดงผล
#     sim.plot_results()



# #!/usr/bin/env python3
# import numpy as np
# import matplotlib.pyplot as plt
# import pandas as pd
# import math
# from typing import List, Tuple
# import time
# import csv

# class CompleteNavigationSimulation:
#     def __init__(self):
#         # Vehicle parameters (ตรงกับโค้ดจริง)
#         self.wheelbase = 1.67  # meters
#         self.max_steering_angle = 0.873  # radians (50 degrees)
#         self.fixed_speed = 1.0  # m/s
        
#         # Simulation parameters
#         self.dt = 0.1  # time step (seconds)
#         self.waypoint_tolerance = 1.0  # meters
        
#         # Current state (เริ่มต้นด้วย heading 0 องศา)
#         self.current_x = 0.0
#         self.current_y = 0.0
#         self.current_heading = 0.0  # radians (0° = North)
        
#         # Path data
#         self.waypoints = []
#         self.path_segments = []
#         self.trajectory = []  # recorded path
        
#         # Statistics
#         self.total_distance = 0.0
#         self.total_time = 0.0
#         self.current_waypoint_index = 0
#         self.navigation_started = False
        
#         # ✅ กำหนด path segments ตามโค้ดจริง
#         self.define_path_segments()
        
#         # ✅ กำหนดรัศมีสำหรับแต่ละทางโค้ง
#         self.curve_radii = {
#             'curve1': 15.0,  # รัศมี 15 เมตร สำหรับโค้งที่ 1
#             'curve2': 20.0,  # รัศมี 20 เมตร สำหรับโค้งที่ 2
#             'curve3': 18.0,  # รัศมี 18 เมตร สำหรับโค้งที่ 3
#             'curve4': 12.0   # รัศมี 12 เมตร สำหรับโค้งที่ 4
#         }
        
#         # Navigation parameters
#         self.heading_correction_kp = 0.02
#         self.lookahead_distance = 3.0
    
#     def define_path_segments(self):
#         """✅ กำหนด path segments ตามโค้ดจริง"""
#         self.path_segments = [
#             {'start': 0, 'end': 243, 'type': 'straight', 'direction': 'S-N', 'target_heading': 0.0},      # waypoint 1-244 (North = 0°)
#             {'start': 244, 'end': 259, 'type': 'curve', 'direction': 'curve1', 'target_heading': None},    # waypoint 245-260
#             {'start': 260, 'end': 314, 'type': 'straight', 'direction': 'E-W', 'target_heading': 90.0},   # waypoint 261-315 (East = 90°)
#             {'start': 315, 'end': 389, 'type': 'curve', 'direction': 'curve2', 'target_heading': None},    # waypoint 316-390
#             {'start': 390, 'end': 499, 'type': 'straight', 'direction': 'N-S', 'target_heading': 180.0},  # waypoint 391-500 (South = 180°)
#             {'start': 500, 'end': 578, 'type': 'curve', 'direction': 'curve3', 'target_heading': None},    # waypoint 501-579
#             {'start': 579, 'end': 629, 'type': 'straight', 'direction': 'W-E', 'target_heading': 270.0},  # waypoint 580-630 (West = 270°)
#             {'start': 630, 'end': 645, 'type': 'curve', 'direction': 'curve4', 'target_heading': None},    # waypoint 631-646
#             {'start': 646, 'end': 658, 'type': 'straight', 'direction': 'S-N', 'target_heading': 0.0}     # waypoint 647-659 (North = 0°)
#         ]
    
#     def load_waypoints_from_csv(self, csv_file_path):
#         """✅ โหลด waypoints จากไฟล์ CSV จริง"""
#         try:
#             waypoints = []
#             with open(csv_file_path, 'r') as file:
#                 csv_reader = csv.DictReader(file)
#                 for row in csv_reader:
#                     waypoint = {
#                         'x': float(row['x_east']),
#                         'y': float(row['y_north'])
#                     }
#                     waypoints.append(waypoint)
            
#             self.waypoints = waypoints
#             print(f"✅ Loaded {len(waypoints)} waypoints from {csv_file_path}")
            
#             # แสดงข้อมูลพื้นฐาน
#             if waypoints:
#                 x_coords = [wp['x'] for wp in waypoints]
#                 y_coords = [wp['y'] for wp in waypoints]
#                 print(f"📍 X range: {min(x_coords):.1f} to {max(x_coords):.1f}")
#                 print(f"📍 Y range: {min(y_coords):.1f} to {max(y_coords):.1f}")
#                 print(f"📍 First waypoint: ({waypoints[0]['x']:.2f}, {waypoints[0]['y']:.2f})")
#                 print(f"📍 Last waypoint: ({waypoints[-1]['x']:.2f}, {waypoints[-1]['y']:.2f})")
            
#             return True
            
#         except Exception as e:
#             print(f"❌ Error loading waypoints: {e}")
#             return False
    
#     def find_nearest_forward_waypoint(self, current_position, current_heading, search_radius=50.0):
#         """✅ หา waypoint ที่ใกล้ที่สุดที่อยู่หน้ารถ (ตามโค้ดจริง)"""
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
#         """✅ เริ่มต้นการนำทางจากจุดที่เหมาะสม (ตามโค้ดจริง)"""
#         if self.navigation_started:
#             return False
        
#         if not current_position or current_heading is None:
#             print("⚠️ Waiting for position and heading data...")
#             return False
        
#         # หา waypoint ที่ใกล้ที่สุดที่อยู่หน้ารถ
#         self.current_waypoint_index = self.find_nearest_forward_waypoint(
#             current_position, current_heading
#         )
        
#         self.navigation_started = True
        
#         print(f"🚀 Navigation started from waypoint {self.current_waypoint_index + 1}")
#         print(f"📍 Target: ({self.waypoints[self.current_waypoint_index]['x']:.2f}, "
#               f"{self.waypoints[self.current_waypoint_index]['y']:.2f})")
        
#         return True
    
#     def get_current_segment_info(self):
#         """ได้ข้อมูลของเส้นทางปัจจุบัน"""
#         for segment in self.path_segments:
#             if segment['start'] <= self.current_waypoint_index <= segment['end']:
#                 return segment
#         return {'type': 'straight', 'direction': 'unknown', 'target_heading': 0.0}
    
#     def normalize_angle(self, angle):
#         """ปรับมุมให้อยู่ในช่วง [-π, π]"""
#         while angle > math.pi:
#             angle -= 2 * math.pi
#         while angle < -math.pi:
#             angle += 2 * math.pi
#         return angle
    
#     def normalize_angle_difference(self, angle_diff):
#         """ปรับความแตกต่างของมุมให้อยู่ในช่วง [-180, 180]"""
#         while angle_diff > 180:
#             angle_diff -= 360
#         while angle_diff < -180:
#             angle_diff += 360
#         return angle_diff
    
#     def bicycle_model_update(self, steering_angle, dt):
#         """✅ อัพเดทตำแหน่งด้วย bicycle kinematic model (ตามโค้ดจริง)"""
#         # จำกัดมุมเลี้ยว
#         steering_angle = max(-self.max_steering_angle, 
#                            min(self.max_steering_angle, steering_angle))
        
#         # คำนวณ angular velocity
#         if abs(steering_angle) > 1e-6:
#             angular_velocity = (self.fixed_speed * math.tan(steering_angle)) / self.wheelbase
#         else:
#             angular_velocity = 0.0
        
#         # อัพเดทตำแหน่งและทิศทาง
#         self.current_x += self.fixed_speed * math.cos(self.current_heading) * dt
#         self.current_y += self.fixed_speed * math.sin(self.current_heading) * dt
#         self.current_heading += angular_velocity * dt
#         self.current_heading = self.normalize_angle(self.current_heading)
        
#         return angular_velocity
    
#     def pure_pursuit_control(self, target_waypoint):
#         """✅ Pure Pursuit controller (ตามโค้ดจริง)"""
#         dx = target_waypoint['x'] - self.current_x
#         dy = target_waypoint['y'] - self.current_y
        
#         # คำนวณทิศทางไปยังเป้าหมาย
#         target_heading = math.atan2(dy, dx)
#         heading_error = self.normalize_angle(target_heading - self.current_heading)
        
#         # คำนวณระยะทาง
#         distance = math.sqrt(dx**2 + dy**2)
        
#         # Pure Pursuit formula
#         if distance > 0.1:
#             alpha = heading_error
#             steering_angle = math.atan2(2 * self.wheelbase * math.sin(alpha), distance)
#         else:
#             steering_angle = 0.0
        
#         return steering_angle
    
#     def heading_correction_control(self, target_heading_deg):
#         """✅ Bicycle model heading correction สำหรับทางตรง (ตามโค้ดจริง)"""
#         target_heading_rad = math.radians(target_heading_deg)
#         current_heading_deg = math.degrees(self.current_heading) % 360
#         heading_error = self.normalize_angle_difference(target_heading_deg - current_heading_deg)
#         steering_angle = self.heading_correction_kp * math.radians(heading_error)
#         return steering_angle
    
#     def curvature_control(self, radius):
#         """✅ Constant curvature controller สำหรับทางโค้ง (ตามโค้ดจริง)"""
#         curvature = 1.0 / radius
#         steering_angle = math.atan(self.wheelbase * curvature)
#         return steering_angle
    
#     def calculate_control(self):
#         """✅ คำนวณคำสั่งควบคุมตามประเภทเส้นทาง (ตามโค้ดจริง)"""
#         if self.current_waypoint_index >= len(self.waypoints):
#             return 0.0
        
#         current_waypoint = self.waypoints[self.current_waypoint_index]
#         segment_info = self.get_current_segment_info()
        
#         if segment_info['type'] == 'straight':
#             # ✅ ใช้ bicycle model สำหรับรักษาทิศทาง
#             target_heading = segment_info.get('target_heading', 0.0)
#             if target_heading is not None:
#                 heading_correction = self.heading_correction_control(target_heading)
                
#                 # เพิ่ม Pure Pursuit เล็กน้อย
#                 pure_pursuit_steering = self.pure_pursuit_control(current_waypoint)
                
#                 # ผสมผสาน: 80% heading correction + 20% pure pursuit (ตามโค้ดจริง)
#                 steering_angle = 0.8 * heading_correction + 0.2 * pure_pursuit_steering * 0.3
#             else:
#                 steering_angle = self.pure_pursuit_control(current_waypoint) * 0.5
            
#         else:  # curve
#             # ✅ ใช้ constant curvature สำหรับทางโค้ง
#             radius = self.curve_radii.get(segment_info['direction'], 15.0)
#             steering_angle = self.curvature_control(radius)
        
#         return steering_angle
    
#     def check_waypoint_reached(self):
#         """ตรวจสอบว่าถึง waypoint แล้วหรือไม่"""
#         if self.current_waypoint_index >= len(self.waypoints):
#             return False
        
#         current_waypoint = self.waypoints[self.current_waypoint_index]
#         distance = math.sqrt(
#             (self.current_x - current_waypoint['x'])**2 + 
#             (self.current_y - current_waypoint['y'])**2
#         )
        
#         return distance < self.waypoint_tolerance
    
#     def run_simulation(self, csv_file_path, max_time=800):
#         """✅ รัน simulation ด้วยไฟล์ waypoints จริง"""
#         print("🚀 Starting complete navigation simulation...")
        
#         # โหลด waypoints จากไฟล์จริง
#         if not self.load_waypoints_from_csv(csv_file_path):
#             print("❌ Failed to load waypoints")
#             return []
        
#         # ✅ ตั้งค่าจุดเริ่มต้น (ใกล้ waypoint แรก)
#         if self.waypoints:
#             start_waypoint = self.waypoints[0]
#             self.current_x = start_waypoint['x']
#             self.current_y = start_waypoint['y']
#             self.current_heading = 0.0  # ✅ เริ่มต้นที่ 0 องศา (หันหน้าไปทางเหนือ)
            
#             # เริ่มต้นการนำทาง
#             current_position = {'x': self.current_x, 'y': self.current_y}
#             current_heading_deg = math.degrees(self.current_heading) % 360
            
#             if self.initialize_starting_point(current_position, current_heading_deg):
#                 print(f"✅ Starting from waypoint {self.current_waypoint_index + 1}")
#                 print(f"📍 Initial position: ({self.current_x:.2f}, {self.current_y:.2f})")
#                 print(f"🧭 Initial heading: {math.degrees(self.current_heading):.1f}°")
#             else:
#                 print("❌ Failed to initialize starting point")
#                 return []
#         else:
#             print("❌ No waypoints loaded")
#             return []
        
#         simulation_time = 0.0
#         step_count = 0
        
#         while simulation_time < max_time and self.current_waypoint_index < len(self.waypoints):
#             # คำนวณคำสั่งควบคุม
#             steering_angle = self.calculate_control()
            
#             # อัพเดทตำแหน่งด้วย bicycle model
#             angular_velocity = self.bicycle_model_update(steering_angle, self.dt)
            
#             # บันทึกเส้นทาง
#             segment_info = self.get_current_segment_info()
#             self.trajectory.append({
#                 'time': simulation_time,
#                 'x': self.current_x,
#                 'y': self.current_y,
#                 'heading': self.current_heading,
#                 'heading_deg': math.degrees(self.current_heading) % 360,
#                 'steering_angle': steering_angle,
#                 'angular_velocity': angular_velocity,
#                 'waypoint_index': self.current_waypoint_index,
#                 'segment_type': segment_info['type'],
#                 'segment_direction': segment_info['direction'],
#                 'target_heading': segment_info.get('target_heading')
#             })
            
#             # ตรวจสอบว่าถึง waypoint แล้วหรือไม่
#             if self.check_waypoint_reached():
#                 print(f"✅ Reached waypoint {self.current_waypoint_index + 1}")
#                 self.current_waypoint_index += 1
            
#             # อัพเดทเวลา
#             simulation_time += self.dt
#             step_count += 1
            
#             # แสดงสถานะทุกๆ 5 วินาที
#             if step_count % 50 == 0:
#                 segment_info = self.get_current_segment_info()
#                 target_heading = segment_info.get('target_heading', 'N/A')
#                 print(f"Time: {simulation_time:.1f}s | "
#                       f"Waypoint: {self.current_waypoint_index + 1}/{len(self.waypoints)} | "
#                       f"Segment: {segment_info['type']} ({segment_info['direction']}) | "
#                       f"Target heading: {target_heading}° | "
#                       f"Current heading: {math.degrees(self.current_heading) % 360:.1f}° | "
#                       f"Position: ({self.current_x:.1f}, {self.current_y:.1f}) | "
#                       f"Steering: {math.degrees(steering_angle):.1f}°")
        
#         self.total_time = simulation_time
#         print(f"🏁 Simulation completed in {simulation_time:.1f} seconds")
#         print(f"📊 Reached {self.current_waypoint_index}/{len(self.waypoints)} waypoints")
        
#         return self.trajectory
    
#     def plot_results(self):
#         """✅ แสดงผลลัพธ์การ simulation"""
#         if not self.trajectory:
#             print("❌ No trajectory data to plot")
#             return
        
#         # แปลงข้อมูลเป็น arrays
#         trajectory_data = pd.DataFrame(self.trajectory)
#         waypoints_array = np.array([[wp['x'], wp['y']] for wp in self.waypoints])
        
#         # สร้าง plots
#         fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
        
#         # Plot 1: เส้นทางการเดินทาง
#         ax1.plot(trajectory_data['x'], trajectory_data['y'], 'b-', linewidth=2, label='Actual Path')
#         ax1.plot(waypoints_array[:, 0], waypoints_array[:, 1], 'ro', markersize=1, alpha=0.6, label='Waypoints')
#         ax1.plot(trajectory_data['x'].iloc[0], trajectory_data['y'].iloc[0], 'go', markersize=10, label='Start')
#         ax1.plot(trajectory_data['x'].iloc[-1], trajectory_data['y'].iloc[-1], 'rs', markersize=10, label='End')
        
#         # เพิ่มการแสดง segment types
#         straight_mask = trajectory_data['segment_type'] == 'straight'
#         curve_mask = trajectory_data['segment_type'] == 'curve'
        
#         ax1.scatter(trajectory_data[straight_mask]['x'], trajectory_data[straight_mask]['y'], 
#                    c='blue', s=0.5, alpha=0.3, label='Straight segments')
#         ax1.scatter(trajectory_data[curve_mask]['x'], trajectory_data[curve_mask]['y'], 
#                    c='red', s=0.5, alpha=0.3, label='Curve segments')
        
#         ax1.set_xlabel('X (UTM East) [m]')
#         ax1.set_ylabel('Y (UTM North) [m]')
#         ax1.set_title('Complete Navigation Path with Smart Starting Point')
#         ax1.legend()
#         ax1.grid(True, alpha=0.3)
#         ax1.axis('equal')
        
#         # Plot 2: Heading vs Time
#         ax2.plot(trajectory_data['time'], trajectory_data['heading_deg'], 'g-', linewidth=2, label='Current Heading')
        
#         # เพิ่มเส้น target heading สำหรับ straight segments
#         for segment in self.path_segments:
#             if segment['type'] == 'straight' and segment['target_heading'] is not None:
#                 mask = (trajectory_data['waypoint_index'] >= segment['start']) & \
#                        (trajectory_data['waypoint_index'] <= segment['end'])
#                 if mask.any():
#                     ax2.axhline(y=segment['target_heading'], color='r', linestyle='--', alpha=0.7, 
#                                label=f'Target {segment["direction"]} ({segment["target_heading"]}°)')
        
#         ax2.set_xlabel('Time (s)')
#         ax2.set_ylabel('Heading (degrees)')
#         ax2.set_title('Vehicle Heading vs Time (with Bicycle Model Control)')
#         ax2.legend()
#         ax2.grid(True, alpha=0.3)
#         ax2.set_ylim(-10, 370)
        
#         # Plot 3: Steering Angle vs Time
#         ax3.plot(trajectory_data['time'], np.degrees(trajectory_data['steering_angle']), 'r-', linewidth=2)
#         ax3.axhline(y=50, color='r', linestyle='--', alpha=0.5, label='Max Steering (±50°)')
#         ax3.axhline(y=-50, color='r', linestyle='--', alpha=0.5)
#         ax3.set_xlabel('Time (s)')
#         ax3.set_ylabel('Steering Angle (degrees)')
#         ax3.set_title('Steering Commands (Bicycle Model + Smart Control)')
#         ax3.legend()
#         ax3.grid(True, alpha=0.3)
        
#         # Plot 4: Angular Velocity vs Time
#         ax4.plot(trajectory_data['time'], trajectory_data['angular_velocity'], 'm-', linewidth=2)
#         ax4.set_xlabel('Time (s)')
#         ax4.set_ylabel('Angular Velocity (rad/s)')
#         ax4.set_title('Angular Velocity (from Bicycle Model)')
#         ax4.grid(True, alpha=0.3)
        
#         plt.tight_layout()
#         plt.show()
        
#         # แสดงสถิติ
#         self.print_statistics()
    
#     def print_statistics(self):
#         """✅ แสดงสถิติการทำงาน"""
#         if not self.trajectory:
#             return
        
#         trajectory_data = pd.DataFrame(self.trajectory)
        
#         # คำนวณระยะทางรวม
#         total_distance = 0.0
#         for i in range(1, len(trajectory_data)):
#             dx = trajectory_data['x'].iloc[i] - trajectory_data['x'].iloc[i-1]
#             dy = trajectory_data['y'].iloc[i] - trajectory_data['y'].iloc[i-1]
#             total_distance += math.sqrt(dx**2 + dy**2)
        
#         # สถิติ steering
#         max_steering = np.degrees(trajectory_data['steering_angle'].abs().max())
#         avg_steering = np.degrees(trajectory_data['steering_angle'].abs().mean())
        
#         # สถิติ angular velocity
#         max_angular_vel = trajectory_data['angular_velocity'].abs().max()
#         avg_angular_vel = trajectory_data['angular_velocity'].abs().mean()
        
#         # สถิติ heading error สำหรับ straight segments
#         straight_data = trajectory_data[trajectory_data['segment_type'] == 'straight']
#         if not straight_data.empty:
#             heading_errors = []
#             for _, row in straight_data.iterrows():
#                 if row['target_heading'] is not None:
#                     error = self.normalize_angle_difference(row['target_heading'] - row['heading_deg'])
#                     heading_errors.append(abs(error))
            
#             if heading_errors:
#                 avg_heading_error = np.mean(heading_errors)
#                 max_heading_error = np.max(heading_errors)
#             else:
#                 avg_heading_error = 0
#                 max_heading_error = 0
#         else:
#             avg_heading_error = 0
#             max_heading_error = 0
        
#         print("\n📊 Complete Navigation Simulation Statistics:")
#         print(f"⏱️  Total Time: {self.total_time:.1f} seconds")
#         print(f"📏 Total Distance: {total_distance:.1f} meters")
#         print(f"🚗 Average Speed: {total_distance/self.total_time:.2f} m/s")
#         print(f"🎯 Waypoints Reached: {self.current_waypoint_index}/{len(self.waypoints)}")
#         print(f"📐 Max Steering Angle: {max_steering:.1f}°")
#         print(f"📐 Avg Steering Angle: {avg_steering:.1f}°")
#         print(f"🔄 Max Angular Velocity: {max_angular_vel:.3f} rad/s")
#         print(f"🔄 Avg Angular Velocity: {avg_angular_vel:.3f} rad/s")
#         print(f"🧭 Avg Heading Error (Straight): {avg_heading_error:.1f}°")
#         print(f"🧭 Max Heading Error (Straight): {max_heading_error:.1f}°")
        
#         # ตรวจสอบการทำงานของแต่ละ segment
#         segment_stats = trajectory_data.groupby(['segment_type', 'segment_direction']).agg({
#             'time': 'count',
#             'steering_angle': lambda x: np.degrees(x.abs().mean()),
#             'angular_velocity': lambda x: x.abs().mean()
#         }).round(2)
        
#         print(f"\n📈 Segment Performance:")
#         for (seg_type, seg_dir), stats in segment_stats.iterrows():
#             print(f"  {seg_type} ({seg_dir}): {stats['time']} steps, "
#                   f"avg steering: {stats['steering_angle']:.1f}°, "
#                   f"avg angular vel: {stats['angular_velocity']:.3f} rad/s")

# def main():
#     """✅ รัน simulation ด้วยโค้ดที่แก้ไขแล้ว"""
#     print("🎮 Complete Navigation System Simulation")
#     print("=" * 60)
    
#     # สร้าง simulation
#     sim = CompleteNavigationSimulation()
    
#     # ✅ โหลด waypoints จากไฟล์จริง
#     csv_file_path = '/home/inc/ros2_ws/waypointfile/waypoints3xy.csv'
    
#     # ✅ รัน simulation ด้วยระบบนำทางที่สมบูรณ์
#     trajectory = sim.run_simulation(csv_file_path, max_time=1000)
    
#     if trajectory:
#         # แสดงผลลัพธ์
#         sim.plot_results()
        
#         print("\n✅ Complete simulation finished successfully!")
#         print("📝 Check the plots to see:")
#         print("   🎯 Smart starting point selection")
#         print("   🚗 Bicycle model heading control")
#         print("   📐 Fixed speed navigation (1 m/s)")
#         print("   🌀 Curve radius control")
#         print("   📊 Real waypoint data integration")
#     else:
#         print("❌ Simulation failed to generate trajectory")

# if __name__ == '__main__':
#     main()



#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import time

class SteeringFeedbackSim(Node):
    def __init__(self):
        super().__init__('steering_feedback_sim')

        # Publisher: ส่ง feedback กลับไป
        self.publisher = self.create_publisher(Float32, '/steering_angle_feedback', 10)

        # Subscriber: ฟังคำสั่งมุมที่ PID node ส่งออกมา
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Timer สำหรับ update loop (20 Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)

        # States
        self.current_angle = 50.0   # rad (มุมจริงจำลอง)
        self.target_angle = 20.0    # rad (ค่าที่ PID อยากให้ไป)
        self.response_rate = 0.30  # ยิ่งมาก → ล้อตอบสนองเร็ว

        self.get_logger().info("Steering Feedback Simulator started.")

    def cmd_vel_callback(self, msg: Twist):
        """ รับ desired steering angle จาก /cmd_vel """
        self.target_angle = msg.angular.z
        self.get_logger().info(f"Received desired angle: {self.target_angle:.3f} rad")

    def timer_callback(self):
        """ จำลอง dynamics: current_angle วิ่งเข้าใกล้ target_angle แบบมี inertia """
        error = self.target_angle - self.current_angle
        self.current_angle += error * self.response_rate  # update dynamics

        # publish ค่ามุมจริงจำลองออกไป
        msg = Float32()
        msg.data = self.current_angle
        self.publisher.publish(msg)

        self.get_logger().info(f"Sim feedback angle: {self.current_angle:.3f} rad (target {self.target_angle:.3f})")

def main(args=None):
    rclpy.init(args=args)
    node = SteeringFeedbackSim()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
