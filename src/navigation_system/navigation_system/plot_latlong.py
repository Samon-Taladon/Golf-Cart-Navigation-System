# # # ------------------------see waypoint--------------------------------------
# import pandas as pd
# import matplotlib.pyplot as plt

# # อ่านไฟล์ CSV
# df = pd.read_csv('/home/inc/ros2_ws/waypointfile/waypoints3latlong.csv')

# # ตรวจสอบว่ามีคอลัมน์ Latitude และ Longitude หรือไม่
# if 'Latitude' not in df.columns or 'Longitude' not in df.columns:
#     raise ValueError("CSV ต้องมีคอลัมน์ 'Latitude' และ 'Longitude'")

# # สร้าง scatter plot
# plt.figure(figsize=(8, 6))
# plt.scatter(df['Latitude'], df['Longitude'], c='blue', marker='o')

# # ตั้งชื่อแกนและหัวข้อ
# plt.title('Plot of Latitude and Longitude')
# plt.xlabel('Longitude')
# plt.ylabel('Latitude')

# # เปิด grid
# plt.grid(True)

# # แสดงกราฟ
# plt.show()








# # import numpy as np
# # import csv
# # import matplotlib.pyplot as plt

# # # เตรียม list เก็บค่าจาก csv
# # x_vals = []
# # y_vals = []
# # headings = []

# # # อ่านข้อมูลจาก csv
# # with open('gnss_log.csv', mode='r') as csvfile:
# #     reader = csv.DictReader(csvfile)
# #     for row in reader:
# #         x_vals.append(float(row['X (m)']))
# #         y_vals.append(float(row['Y (m)']))
# #         headings.append(float(row['Heading (deg)']))

# # # Plot XY Path
# # plt.figure(figsize=(8, 8))
# # plt.plot(x_vals, y_vals, marker='o', color='blue', label='Path')

# # # Plot heading vector
# # for i in range(len(x_vals)):
# #     dx = 1.0 * np.cos(np.radians(headings[i]))
# #     dy = 1.0 * np.sin(np.radians(headings[i]))
# #     plt.arrow(x_vals[i], y_vals[i], dx, dy, head_width=0.5, head_length=0.5, fc='red', ec='red')

# # plt.xlabel('X (m)')
# # plt.ylabel('Y (m)')
# # plt.title('GNSS Path with Heading')
# # plt.grid(True)
# # plt.axis('equal')
# # plt.legend()
# # plt.show()


# import json
# import numpy as np
# import pandas as pd
# import matplotlib.pyplot as plt

# # อ่านไฟล์ JSON
# with open('waypoints.json') as f:
#     data = json.load(f)

# # แปลง waypoint เป็น DataFrame
# df = pd.DataFrame(data['waypoints'])

# # เช็กว่ามี heading มั้ย
# if 'heading' not in df.columns:
#     print("❌ ไม่มีค่า heading อยู่ใน waypoint JSON")
# else:
#     # 1️⃣ Plot Waypoint Path + Heading Vector
#     plt.figure(figsize=(10, 8))
#     plt.plot(df['x'], df['y'], 'bo-', markersize=3, label='Path')

#     # วาด heading vector ทุก n จุด (กำหนด n ได้)
#     n = 10
#     for i in range(0, len(df), n):
#         x = df['x'][i]
#         y = df['y'][i]
#         heading_rad = np.deg2rad(df['heading'][i])
#         dx = np.cos(heading_rad)
#         dy = np.sin(heading_rad)
#         plt.arrow(x, y, dx, dy, head_width=0.5, head_length=0.5, fc='r', ec='r')

#     plt.xlabel('X Position')
#     plt.ylabel('Y Position')
#     plt.title('Waypoint Path with Heading Vectors')
#     plt.legend()
#     plt.grid(True)
#     plt.axis('equal')
#     plt.show()

#     # 2️⃣ Plot Speed vs Time
#     plt.figure(figsize=(8, 4))
#     plt.plot(df['timestamp'], df['speed'], color='green')
#     plt.xlabel('Timestamp')
#     plt.ylabel('Speed (m/s)')
#     plt.title('Speed vs Time')
#     plt.grid(True)
#     plt.show()

#     # 3️⃣ Plot Heading vs Time
#     plt.figure(figsize=(8, 4))
#     plt.plot(df['timestamp'], df['heading'], color='orange')
#     plt.xlabel('Timestamp')
#     plt.ylabel('Heading (°)')
#     plt.title('Heading vs Time')
#     plt.grid(True)
#     plt.show()

#     # 4️⃣ Plot histogram distribution ของ heading
#     plt.figure(figsize=(6, 4))
#     plt.hist(df['heading'], bins=36, color='purple')
#     plt.xlabel('Heading (°)')
#     plt.ylabel('Count')
#     plt.title('Heading Distribution')
#     plt.grid(axis='y')
#     plt.show()



# ---------------------------------------plot waypoints--------------------------
# import pandas as pd
# import matplotlib.pyplot as plt

# # อ่านไฟล์ CSV
# df = pd.read_csv('/home/inc/ros2_ws/waypointfile/waypoints3xy.csv')

# # ดูข้อมูลก่อน (optional)
# print(df)

# # สร้างกราฟ scatter
# plt.plot(df['x_east'], df['y_north'], 'bo-')  # จุดสีน้ำเงินเชื่อมเส้น
# plt.xlabel('X East (m)')
# plt.ylabel('Y North (m)')
# plt.title('Waypoint Plot')
# plt.grid(True)
# plt.axis('equal')  # ให้ scale แกนเท่ากัน
# plt.show()






# import pandas as pd
# import numpy as np

# # อ่านไฟล์ waypoint
# df = pd.read_csv("/home/inc/ros2_ws/waypoints1m.csv")

# # คำนวณ heading ระหว่าง waypoint (หน่วยเป็น radian)
# def get_heading_rad(x1, y1, x2, y2):
#     return np.arctan2(y2 - y1, x2 - x1)  # np.arctan2 คืนค่าเป็น radian อยู่แล้ว

# # ฟังก์ชันแปลง delta heading (rad) เป็น steering angle (rad) ในช่วง [-0.873, 0.873]
# def heading_to_steering(delta_rad):
#     # องศา max steering = 50°
#     max_steering_deg = 50
#     max_steering_rad = np.deg2rad(max_steering_deg)  # 0.873 rad

#     # แปลง delta_rad (หน่วย rad) เป็นองศาก่อน เพื่อ normalize
#     delta_deg = np.rad2deg(delta_rad)

#     # จำกัด delta_deg ให้อยู่ในช่วง -50 ถึง 50 องศา
#     if delta_deg > max_steering_deg:
#         delta_deg = max_steering_deg
#     elif delta_deg < -max_steering_deg:
#         delta_deg = -max_steering_deg

#     # แปลงกลับเป็น rad และ normalizeเป็น steering angle
#     steering_rad = (delta_deg / max_steering_deg) * max_steering_rad

#     return steering_rad

# # เก็บ heading เป็น radian
# headings = []

# for i in range(len(df) - 1):
#     x1, y1 = df.iloc[i]['x_east'], df.iloc[i]['y_north']
#     x2, y2 = df.iloc[i+1]['x_east'], df.iloc[i+1]['y_north']
#     heading = get_heading_rad(x1, y1, x2, y2)
#     headings.append(heading)

# # หาค่าความต่าง heading ระหว่างจุด (delta heading)
# delta_headings = np.diff(headings)

# # กำหนดช่วงประเภททางใน radian
# tolerance_straight_deg = 5  # 5 องศา
# tolerance_straight_rad = np.deg2rad(tolerance_straight_deg)

# segment_type = []

# for delta in delta_headings:
#     steering_rad = heading_to_steering(delta)
#     if abs(delta) <= tolerance_straight_rad:
#         segment_type.append(f"straight (steering={steering_rad:.3f} rad)")
#     elif delta > tolerance_straight_rad:
#         segment_type.append(f"right turn (steering={steering_rad:.3f} rad)")
#     else:  # delta < -tolerance_straight_rad
#         segment_type.append(f"left turn (steering={steering_rad:.3f} rad)")

# # แสดงผลลัพธ์
# for i in range(len(segment_type)):
#     print(f"Waypoint {i} to {i+1}: {segment_type[i]}")

# # เซฟออกไฟล์ CSV
# result_df = pd.DataFrame({
#     'from_wp': range(len(segment_type)),
#     'to_wp': range(1, len(segment_type)+1),
#     'segment': segment_type
# })

# result_df.to_csv("waypoint_segment_result.csv", index=False)



# import pandas as pd
# import numpy as np
# import matplotlib.pyplot as plt

# # โหลดไฟล์ผลลัพธ์
# df = pd.read_csv('/home/inc/ros2_ws/waypointfile/waypoint_segments3xy.csv')

# # สร้าง figure
# plt.figure(figsize=(12, 8))

# # Plot segment ตามประเภท
# for i in range(len(df)-1):
#     x = [df.loc[i, 'x_east'], df.loc[i+1, 'x_east']]
#     y = [df.loc[i, 'y_north'], df.loc[i+1, 'y_north']]
    
#     if df.loc[i, 'segment_type'] == 'straight':
#         plt.plot(x, y, color='blue', linewidth=1.5)
#     else:
#         plt.plot(x, y, color='red', linewidth=2.5)

# # Plot จุด waypoint
# plt.scatter(df['x_east'], df['y_north'], c='black', s=10, label='Waypoint')

# # ใส่รัศมีที่ตำแหน่งโค้ง
# for i in range(len(df)):
#     if df.loc[i, 'segment_type'] == 'curve' and np.isfinite(df.loc[i, 'radius']):
#         plt.text(df.loc[i, 'x_east'], df.loc[i, 'y_north'],
#                  f"R={df.loc[i, 'radius']:.1f}m",
#                  fontsize=8, color='green')

# # ตั้งค่าแกน
# plt.xlabel('X East (m)')
# plt.ylabel('Y North (m)')
# plt.title('Waypoint Segments: Straight vs Curve with Radius')
# plt.grid(True)
# plt.axis('equal')
# plt.legend()
# plt.tight_layout()
# plt.show()



# import pandas as pd
# import numpy as np
# import math

# # อ่านไฟล์ waypoints
# df = pd.read_csv('/home/inc/ros2_ws/waypointfile/waypoints3xy.csv')

# # เพิ่มคอลัมน์ heading
# def calculate_heading(x1, y1, x2, y2):
#     return math.atan2(y2 - y1, x2 - x1)

# headings = []
# for i in range(len(df)-1):
#     h = calculate_heading(df.loc[i, 'x_east'], df.loc[i, 'y_north'],
#                           df.loc[i+1, 'x_east'], df.loc[i+1, 'y_north'])
#     headings.append(h)
# headings.append(headings[-1])  # ให้ตัวสุดท้ายเท่ากับตัวก่อนหน้า
# df['heading'] = headings

# # คำนวณ delta heading
# df['delta_heading'] = np.abs(np.ediff1d(df['heading'], to_end=0))

# # Threshold การเปลี่ยน heading ที่ถือว่าเป็นโค้ง (rad)
# curve_threshold = np.deg2rad(3)

# # แบ่ง segment
# df['segment_type'] = df['delta_heading'].apply(lambda x: 'straight' if x < curve_threshold else 'curve')

# # คำนวณรัศมีโค้ง (จาก 3 จุด)
# def radius_from_3points(x1, y1, x2, y2, x3, y3):
#     a = np.hypot(x2-x1, y2-y1)
#     b = np.hypot(x3-x2, y3-y2)
#     c = np.hypot(x3-x1, y3-y1)
#     s = (a + b + c) / 2
#     area = np.sqrt(s*(s-a)*(s-b)*(s-c))
#     if area == 0:
#         return np.inf
#     return (a*b*c) / (4*area)

# # เพิ่มคอลัมน์ radius
# radius_list = []
# for i in range(1, len(df)-1):
#     if df.loc[i, 'segment_type'] == 'curve':
#         R = radius_from_3points(
#             df.loc[i-1, 'x_east'], df.loc[i-1, 'y_north'],
#             df.loc[i, 'x_east'], df.loc[i, 'y_north'],
#             df.loc[i+1, 'x_east'], df.loc[i+1, 'y_north'])
#     else:
#         R = np.inf
#     radius_list.append(R)

# # เติมค่า NaN ให้หัวท้าย
# radius_list = [np.inf] + radius_list + [np.inf]
# df['radius'] = radius_list

# # ดูผลลัพธ์
# print(df[['x_east', 'y_north', 'heading', 'delta_heading', 'segment_type', 'radius']])

# # export เป็น csv ถ้าอยากดูเต็มๆ
# df.to_csv('/home/inc/ros2_ws/waypointfile/waypoint_segments3xy.csv', index=False)




# import pandas as pd
# import matplotlib.pyplot as plt

# # อ่านไฟล์ waypoint
# df = pd.read_csv('/home/inc/ros2_ws/waypointfile/waypoint_segments3xy.csv')  # หรือไฟล์ waypoint_segments.csv ก็ได้

# # สร้างกราฟ
# plt.figure(figsize=(12, 8))

# # Plot path
# plt.plot(df['x_east'], df['y_north'], 'k.-', label='Path')

# # Plot จุด waypoint ทุกจุดพร้อมหมายเลข
# for i in range(len(df)):
#     plt.plot(df.loc[i, 'x_east'], df.loc[i, 'y_north'], 'ro')  # จุดสีแดง
#     plt.text(df.loc[i, 'x_east']+0.3, df.loc[i, 'y_north']+0.3, f'{i}', fontsize=8, color='blue')  # หมายเลข

# # ตั้งค่าแกนและชื่อ
# plt.xlabel('X East (m)')
# plt.ylabel('Y North (m)')
# plt.title('Waypoint Position with Index')
# plt.grid(True)
# plt.axis('equal')
# plt.legend()
# plt.tight_layout()
# plt.show()



# import pandas as pd
# import matplotlib.pyplot as plt

# # อ่านไฟล์ CSV
# file_path = '/home/inc/ros2_ws/navigation_logs/navigation_data_20250722_195409_copy.csv'
# df = pd.read_csv(file_path)

# # พล็อตกราฟแยกตามคอลัมน์ที่ต้องการ
# plt.figure(figsize=(10, 6))

# # พล็อต x กับ y
# plt.subplot(2, 2, 1)
# plt.plot(df['x'], df['y'], marker='o', linestyle='-', color='b')
# plt.title('Plot of x and y')
# plt.xlabel('x')
# plt.ylabel('y')

# # พล็อต yaw
# plt.subplot(2, 2, 2)
# plt.plot(df.index, df['yaw'], marker='o', linestyle='-', color='r')
# plt.title('Plot of yaw')
# plt.xlabel('Index')
# plt.ylabel('Yaw')

# # พล็อต steering angle
# plt.subplot(2, 2, 3)
# plt.plot(df.index, df['steering_angle_deg'], marker='o', linestyle='-', color='g')
# plt.title('Plot of Steering Angle')
# plt.xlabel('Index')
# plt.ylabel('Steering Angle (degrees)')



# # ปรับปรุงเลเบล
# plt.tight_layout()

# # แสดงกราฟ
# plt.show()


# import pandas as pd
# import matplotlib.pyplot as plt

# # โหลดไฟล์ CSV
# df = pd.read_csv('/home/inc/ros2_ws/navigation_logs/navigation_data_20250722_195409_copy.csv')

# # เลือกบางจุดมา plot (เลือกทุกๆ 5 แถว)
# df_sampled = df.iloc[::5]

# # พล็อต x, y
# plt.figure(figsize=(10, 6))
# plt.subplot(2, 2, 1)
# plt.plot(df_sampled['x'], df_sampled['y'], marker='o')
# plt.title('x vs y')

# # พล็อต yaw
# plt.subplot(2, 2, 2)
# plt.plot(df_sampled.index, df_sampled['yaw'], marker='o')
# plt.title('Yaw')

# # พล็อต Steering Angle
# plt.subplot(2, 2, 3)
# plt.plot(df_sampled.index, df_sampled['steering_angle_deg'], marker='o')
# plt.title('Steering Angle (deg)')

# plt.tight_layout()
# plt.show()



# import pandas as pd
# import matplotlib.pyplot as plt

# # โหลด CSV
# df = pd.read_csv('/home/inc/ros2_ws/navigation_logs1/navigation_data_20250722_190706.csv')

# # เลือกช่วงแถว 0-1000
# df_sampled = df.iloc[0:500]

# # พล็อต
# plt.figure(figsize=(10, 6))

# plt.subplot(2, 2, 1)
# plt.plot(df_sampled['x'], df_sampled['y'], marker='o')
# plt.title('x vs y')

# plt.subplot(2, 2, 2)
# plt.plot(df_sampled.index, df_sampled['yaw'], marker='o')
# plt.title('Yaw')

# plt.subplot(2, 2, 3)
# plt.plot(df_sampled.index, df_sampled['steering_angle_deg'], marker='o')
# plt.title('Steering Angle (deg)')

# plt.tight_layout()
# plt.show()




# import pandas as pd
# import matplotlib.pyplot as plt

# # อ่านข้อมูลจาก CSV
# df = pd.read_csv('waypoints_gnss_speed2.csv')

# # สร้าง subplot 3 แถว (Path, Speed, Heading)
# fig, axes = plt.subplots(3, 1, figsize=(10, 12))

# # --- กราฟตำแหน่ง XY ---
# axes[0].plot(df['x_east'], df['y_north'], marker='.', color='b')
# axes[0].set_xlabel('X East (meters)')
# axes[0].set_ylabel('Y North (meters)')
# axes[0].set_title('Path: X-Y Plot (UTM)')
# axes[0].grid()

# # --- กราฟความเร็ว ---
# axes[1].plot(df['cumulative_time'], df['speed_mps'], color='g')
# axes[1].set_xlabel('Cumulative Time (s)')
# axes[1].set_ylabel('Speed (m/s)')
# axes[1].set_title('Speed over Time')
# axes[1].grid()

# # --- กราฟมุม IMU ---
# axes[2].plot(df['cumulative_time'], df['imu_heading'], color='r')
# axes[2].set_xlabel('Cumulative Time (s)')
# axes[2].set_ylabel('IMU Heading (deg)')
# axes[2].set_title('IMU Heading over Time')
# axes[2].grid()

# plt.tight_layout()
# plt.show()




# import pandas as pd
# import matplotlib.pyplot as plt

# # อ่านข้อมูลจาก CSV
# df = pd.read_csv("waypointsnewmodel3.csv")   # <- ใส่ชื่อไฟล์ของคุณ

# # สร้าง subplot 3 แถว (Path, Speed, Heading)
# fig, axes = plt.subplots(3, 1, figsize=(10, 12))

# # --- กราฟตำแหน่ง XY ---
# axes[0].plot(df['x'], df['y'], marker='.', color='b')
# axes[0].set_xlabel('X (meters)')
# axes[0].set_ylabel('Y (meters)')
# axes[0].set_title('Path: X-Y Plot')
# axes[0].grid()

# # --- กราฟความเร็ว ---
# axes[1].plot(df['time'], df['speed'], color='g')
# axes[1].set_xlabel('Time (s)')
# axes[1].set_ylabel('Speed (m/s)')
# axes[1].set_title('Speed over Time')
# axes[1].grid()

# # --- กราฟมุม psi ---
# axes[2].plot(df['time'], df['psi'], color='r')
# axes[2].set_xlabel('Time (s)')
# axes[2].set_ylabel('Psi (rad)')
# axes[2].set_title('Heading (Psi) over Time')
# axes[2].grid()

# plt.tight_layout()
# plt.show()




# import pandas as pd
# import matplotlib.pyplot as plt
# import numpy as np

# # โหลดข้อมูลจาก CSV
# csv_file = 'waypointsnewmodel3.csv'
# df = pd.read_csv(csv_file)

# # แก้ heading ให้นิ่ม (unwrap)
# psi_unwrap = np.unwrap(df['psi'].values)

# # --- Plot ---
# fig, axes = plt.subplots(3, 1, figsize=(10, 12))

# # 1) Path XY
# axes[0].plot(df['x'], df['y'], marker='.', color='b')
# axes[0].set_xlabel("X (meters)")
# axes[0].set_ylabel("Y (meters)")
# axes[0].set_title("Path: X-Y Plot")
# axes[0].grid(True)

# # 2) Speed vs Time
# axes[1].plot(df['time'], df['speed'], color='g')
# axes[1].set_xlabel("Time (s)")
# axes[1].set_ylabel("Speed (m/s)")
# axes[1].set_title("Speed over Time")
# axes[1].grid(True)

# # 3) Heading vs Time (unwrap แล้ว)
# axes[2].plot(df['time'], psi_unwrap, color='r')
# axes[2].set_xlabel("Time (s)")
# axes[2].set_ylabel("Psi (rad)")
# axes[2].set_title("Heading (Psi) over Time")
# axes[2].grid(True)

# plt.tight_layout()
# plt.show()




# import pandas as pd
# import matplotlib.pyplot as plt
# import numpy as np

# # อ่านข้อมูลจาก CSV
# df = pd.read_csv("waypointsnewmodel3.csv")   # <- ใส่ชื่อไฟล์ของคุณ

# # แปลง psi ให้อยู่ในช่วง -pi ถึง pi
# df['psi_wrapped'] = df['psi'].apply(lambda x: x - 2 * np.pi if x > np.pi else x)

# # สร้าง subplot 3 แถว (Path, Speed, Heading)
# fig, axes = plt.subplots(3, 1, figsize=(10, 12))

# # --- กราฟตำแหน่ง XY ---
# axes[0].plot(df['x'], df['y'], marker='.', color='b')
# axes[0].set_xlabel('X (meters)')
# axes[0].set_ylabel('Y (meters)')
# axes[0].set_title('Path: X-Y Plot')
# axes[0].grid()

# # --- กราฟความเร็ว ---
# axes[1].plot(df['time'], df['speed'], color='g')
# axes[1].set_xlabel('Time (s)')
# axes[1].set_ylabel('Speed (m/s)')
# axes[1].set_title('Speed over Time')
# axes[1].grid()

# # --- กราฟมุม psi (แบบ wrap -pi ถึง pi) ---
# axes[2].plot(df['time'], df['psi_wrapped'], color='r')
# axes[2].set_xlabel('Time (s)')
# axes[2].set_ylabel('Psi (rad)')
# axes[2].set_title('Heading (Psi) over Time (Wrapped -π to π)')
# axes[2].set_ylim([-3.5, 3.5])
# axes[2].grid()

# plt.tight_layout()
# plt.show()



# import pandas as pd
# import matplotlib.pyplot as plt
# import numpy as np

# # โหลดข้อมูลจาก CSV
# csv_file = 'waypointsnewmodel3.csv'
# df = pd.read_csv(csv_file)

# # แก้ heading ให้นิ่ม (unwrap)
# psi_unwrap = np.unwrap(df['psi'].values)

# # --- Plot ---
# fig, axes = plt.subplots(3, 1, figsize=(10, 12))

# # 1) Path XY
# axes[0].plot(df['x'], df['y'], marker='.', color='b')
# axes[0].set_xlabel("X (meters)")
# axes[0].set_ylabel("Y (meters)")
# axes[0].set_title("Path: X-Y Plot")
# axes[0].grid(True)

# # 2) Speed vs Time
# axes[1].plot(df['time'], df['speed'], color='g')
# axes[1].set_xlabel("Time (s)")
# axes[1].set_ylabel("Speed (m/s)")
# axes[1].set_title("Speed over Time")
# axes[1].grid(True)

# # 3) Heading vs Time (unwrap แล้ว)
# axes[2].plot(df['time'], psi_unwrap, color='r')
# axes[2].set_xlabel("Time (s)")
# axes[2].set_ylabel("Psi (rad)")
# axes[2].set_title("Heading (Psi) over Time")
# axes[2].grid(True)

# plt.tight_layout()
# plt.show()



# import pandas as pd
# import matplotlib.pyplot as plt

# # อ่านไฟล์ CSV
# df = pd.read_csv('waypoints_relative_xy.csv')  # เปลี่ยนเป็นชื่อไฟล์จริง

# # สร้าง figure และ axes 3 แถว
# fig, axes = plt.subplots(3, 1, figsize=(10, 15))

# # --- กราฟ 1: Trajectory GPS vs Dead Reckoning ---
# axes[0].plot(df['x'], df['y'], marker='o', linestyle='-', color='blue', label='GPS (x, y)')
# axes[0].plot(df['xk+1'], df['yk+1'], marker='x', linestyle='--', color='red', label='Dead Reckoning (xk+1, yk+1)')
# axes[0].set_title('Vehicle Trajectory: GPS vs Dead Reckoning')
# axes[0].set_xlabel('X [m]')
# axes[0].set_ylabel('Y [m]')
# axes[0].grid(True)
# axes[0].legend()
# axes[0].axis('equal')  # สัดส่วนแกน x,y เท่ากัน

# # --- กราฟ 2: Speed vs Time ---
# axes[1].plot(df['time'], df['speed'], marker='x', linestyle='-', color='green')
# axes[1].set_title('Speed over Time')
# axes[1].set_xlabel('Time [s]')
# axes[1].set_ylabel('Speed [m/s]')
# axes[1].grid(True)

# # --- กราฟ 3: Heading (psi) vs Time ---
# axes[2].plot(df['time'], df['psi'], marker='.', linestyle='-', color='red')
# axes[2].set_title('Heading (psi) over Time')
# axes[2].set_xlabel('Time [s]')
# axes[2].set_ylabel('Psi [rad]')
# axes[2].grid(True)

# plt.tight_layout()
# plt.show()





# import pandas as pd
# import matplotlib.pyplot as plt
# import numpy as np

# # อ่านไฟล์ CSV
# df = pd.read_csv('waypoints_relative_xy.csv')

# # =========================
# # คำนวณ CTE และ RMSE
# # =========================
# df['CTE'] = np.sqrt(
#     (df['x'] - df['xk+1'])**2 +
#     (df['y'] - df['yk+1'])**2
# )

# rmse = np.sqrt(np.mean(df['CTE']**2))

# # =========================
# # ตั้งค่า global font
# # =========================
# plt.rcParams.update({
#     'font.size': 12,
#     'axes.titlesize': 14,
#     'axes.labelsize': 13,
#     'legend.fontsize': 12,
#     'xtick.labelsize': 11,
#     'ytick.labelsize': 11
# })

# # =========================
# # สร้าง figure
# # =========================
# fig, axes = plt.subplots(4, 1, figsize=(12, 20))

# # ---------- 1. Trajectory ----------
# axes[0].plot(df['x'], df['y'],
#              linewidth=2.5,
#              label='GPS trajectory')

# axes[0].plot(df['xk+1'], df['yk+1'],
#              linestyle='--',
#              linewidth=2.5,
#              label='Dead Reckoning')

# axes[0].set_title('Vehicle Trajectory: GPS vs Dead Reckoning')
# axes[0].set_xlabel('X [m]')
# axes[0].set_ylabel('Y [m]')
# axes[0].legend()
# axes[0].grid(True, linestyle='--', alpha=0.4)
# axes[0].axis('equal')

# # ---------- 2. Speed ----------
# axes[1].plot(df['time'], df['speed'],
#              linewidth=2.0)

# axes[1].set_title('Speed over Time')
# axes[1].set_xlabel('Time [s]')
# axes[1].set_ylabel('Speed [m/s]')
# axes[1].grid(True, linestyle='--', alpha=0.4)

# # ---------- 3. Heading ----------
# axes[2].plot(df['time'], df['psi'],
#              linewidth=2.0)

# axes[2].set_title('Heading (ψ) over Time')
# axes[2].set_xlabel('Time [s]')
# axes[2].set_ylabel('ψ [rad]')
# axes[2].grid(True, linestyle='--', alpha=0.4)

# # ---------- 4. CTE ----------
# axes[3].plot(df['time'], df['CTE'],
#              linewidth=2.5)

# axes[3].set_title(f'Cross-Track Error (CTE) over Time\nRMSE = {rmse:.3f} m')
# axes[3].set_xlabel('Time [s]')
# axes[3].set_ylabel('CTE [m]')
# axes[3].grid(True, linestyle='--', alpha=0.4)

# plt.tight_layout(pad=2.0)
# plt.show()




# import pandas as pd
# import numpy as np
# import matplotlib.pyplot as plt

# # อ่านไฟล์
# df = pd.read_csv("/home/inc/ros2_ws/src/navigation_system/navigation_system/waypointssaty3.csv")

# # normalize heading (-180 ถึง 180)
# df["heading_fixed"] = ((df["heading"] + 180) % 360) - 180

# # สร้าง index
# t = np.arange(len(df))

# plt.figure(figsize=(10,8))

# # ---------- Trajectory ----------
# plt.subplot(3,1,1)
# plt.plot(df["x"], df["y"], marker='o')
# plt.title("Trajectory (X-Y)")
# plt.xlabel("X")
# plt.ylabel("Y")
# plt.axis("equal")
# plt.grid()

# # ---------- Heading ----------
# plt.subplot(3,1,2)
# plt.plot(t, df["heading_fixed"])
# plt.title("Heading (-180 to 180)")
# plt.ylabel("Degree")
# plt.grid()

# # ---------- Speed ----------
# plt.subplot(3,1,3)
# plt.plot(t, df["speed"])
# plt.title("Speed")
# plt.xlabel("Index")
# plt.ylabel("Speed")
# plt.grid()

# plt.tight_layout()
# plt.show()

# import numpy as np
# import pandas as pd
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation

# # ---------- settings ----------
# csv_path = "/home/inc/ros2_ws/src/navigation_system/navigation_system/waypointssaty3.csv"
# upsample = 5        # ยิ่งมาก ยิ่งเนียน (แต่หนักขึ้น)
# interval_ms = 100   # ระยะเวลาระหว่างเฟรม (ms) ของ animation
# arrow_scale = 0.08  # ขนาดลูกศร (ปรับให้เหมาะกับข้อมูลของคุณ)
# # ------------------------------

# # อ่านไฟล์
# df = pd.read_csv(csv_path)

# # ถ้า heading ยังไม่ได้ normalize ให้ทำก่อน
# df["heading_fixed"] = ((df["heading"] + 180) % 360) - 180

# # ข้อมูลต้นฉบับ
# n = len(df)
# t_orig = np.arange(n)

# # --- upsample/interpolate เพื่อให้เคลื่อนไหวเนียนขึ้น ---
# t_new = np.linspace(0, n-1, n * upsample)

# x_new = np.interp(t_new, t_orig, df["x"].values)
# y_new = np.interp(t_new, t_orig, df["y"].values)
# speed_new = np.interp(t_new, t_orig, df["speed"].values)

# # สำหรับ heading: แปลงเป็น rad, unwrap, interpolate แล้วกลับเป็น deg (แล้ว normalize)
# heading_rad = np.radians(df["heading_fixed"].values)
# heading_unwrapped = np.unwrap(heading_rad)                    # ป้องกันการกระโดดเมื่อ 359->-1
# heading_interp_rad = np.interp(t_new, t_orig, heading_unwrapped)
# heading_new = np.degrees(heading_interp_rad)
# heading_new = ((heading_new + 180) % 360) - 180               # normalize กลับเพื่อโชว์ -180..180

# # --- เตรียม figure / axes ---
# fig, axs = plt.subplots(3, 1, figsize=(10, 10), gridspec_kw={'height_ratios':[2,1,1]})
# ax_xy, ax_head, ax_speed = axs

# # limits with margin
# margin = 0.1
# xmin, xmax = np.min(x_new), np.max(x_new)
# ymin, ymax = np.min(y_new), np.max(y_new)
# dx = (xmax - xmin) if (xmax - xmin) != 0 else 1.0
# dy = (ymax - ymin) if (ymax - ymin) != 0 else 1.0
# ax_xy.set_xlim(xmin - margin*dx, xmax + margin*dx)
# ax_xy.set_ylim(ymin - margin*dy, ymax + margin*dy)
# ax_xy.set_aspect('equal', adjustable='box')
# ax_xy.set_title("Trajectory (X-Y)")
# ax_xy.set_xlabel("X")
# ax_xy.set_ylabel("Y")
# ax_xy.grid()

# # plot background (full path faint)
# ax_xy.plot(x_new, y_new, lw=1, color='gray', alpha=0.4)

# # trail line, moving point, and heading arrow (quiver)
# trail_line, = ax_xy.plot([], [], lw=2, color='C0')
# current_point, = ax_xy.plot([], [], 'o', color='C1', markersize=6)
# # initial quiver (one arrow)
# u0 = np.cos(np.radians(heading_new[0])) * arrow_scale
# v0 = np.sin(np.radians(heading_new[0])) * arrow_scale
# quiv = ax_xy.quiver([x_new[0]], [y_new[0]], [u0], [v0], angles='xy', scale_units='xy', scale=1)

# # --- Heading axis ---
# ax_head.plot(t_new, heading_new, color='gray', alpha=0.4, lw=1)
# ax_head.set_title("Heading (-180..180)")
# ax_head.set_ylabel("deg")
# ax_head.set_xlim(t_new[0], t_new[-1])
# ax_head.grid()
# head_marker, = ax_head.plot([], [], 'ro')

# # --- Speed axis ---
# ax_speed.plot(t_new, speed_new, color='gray', alpha=0.4, lw=1)
# ax_speed.set_title("Speed")
# ax_speed.set_ylabel("m/s (or units)")
# ax_speed.set_xlabel("time index (interpolated)")
# ax_speed.set_xlim(t_new[0], t_new[-1])
# ax_speed.grid()
# speed_marker, = ax_speed.plot([], [], 'ro')

# # --- init / update functions ---
# def init():
#     trail_line.set_data([], [])
#     current_point.set_data([], [])
#     quiv.set_offsets([[x_new[0], y_new[0]]])
#     quiv.set_UVC(np.cos(np.radians(heading_new[0])) * arrow_scale,
#                   np.sin(np.radians(heading_new[0])) * arrow_scale)
#     head_marker.set_data([], [])
#     speed_marker.set_data([], [])
#     return trail_line, current_point, quiv, head_marker, speed_marker

# def update(frame):
#     # trail
#     trail_line.set_data(x_new[:frame+1], y_new[:frame+1])
#     # current point
#     current_point.set_data(x_new[frame], y_new[frame])
#     # update arrow position & direction
#     u = np.cos(np.radians(heading_new[frame])) * arrow_scale
#     v = np.sin(np.radians(heading_new[frame])) * arrow_scale
#     quiv.set_offsets([[x_new[frame], y_new[frame]]])
#     quiv.set_UVC(u, v)
#     # update heading & speed markers
#     head_marker.set_data(t_new[frame], heading_new[frame])
#     speed_marker.set_data(t_new[frame], speed_new[frame])
#     return trail_line, current_point, quiv, head_marker, speed_marker

# # --- run animation ---
# ani = animation.FuncAnimation(fig, update, frames=len(t_new), init_func=init,
#                               interval=interval_ms, blit=False, repeat=False)

# plt.tight_layout()
# plt.show()

# # --- (optionally) save to mp4 (uncomment to use; requires ffmpeg installed) ---
# # writer = animation.FFMpegWriter(fps=1000/interval_ms)
# # ani.save("trajectory_animation.mp4", writer=writer)







# import pandas as pd
# import numpy as np
# import matplotlib.pyplot as plt

# # อ่านไฟล์
# df = pd.read_csv("/home/inc/ros2_ws/src/navigation_system/navigation_system/waypointssaty3.csv")

# # CTE เทียบเส้น x = 0
# df["cte"] = df["x"]

# # time index
# t = np.arange(len(df))

# plt.figure(figsize=(12,10))

# # ===============================
# # Trajectory XY
# # ===============================
# plt.subplot(2,1,1)

# plt.plot(df["x"], df["y"], linewidth=1)

# # ลูกศรแสดงทิศทาง
# step = 5
# plt.quiver(
#     df["x"][::step],
#     df["y"][::step],
#     np.diff(df["x"], prepend=df["x"][0])[::step],
#     np.diff(df["y"], prepend=df["y"][0])[::step],
#     angles='xy',
#     scale_units='xy',
#     scale=1
# )

# # จุดเริ่มต้น / จุดจบ
# plt.scatter(df["x"].iloc[0], df["y"].iloc[0], s=80, label="Start")
# plt.scatter(df["x"].iloc[-1], df["y"].iloc[-1], s=80, label="End")

# # เส้น reference x=0
# plt.axvline(0, linestyle="--", label="Reference x=0")

# plt.title("Trajectory (X-Y)")
# plt.xlabel("X (m)")
# plt.ylabel("Y (m)")
# plt.axis("equal")
# plt.grid()
# plt.legend()

# # ===============================
# # CTE Graph
# # ===============================
# plt.subplot(2,1,2)

# plt.plot(t, df["cte"], linewidth=2)

# # เส้นกลาง
# plt.axhline(0, linestyle="--")

# plt.title("Cross Track Error (CTE)")
# plt.xlabel("Time Index")
# plt.ylabel("CTE (m)")
# plt.grid()

# plt.tight_layout()
# plt.show()





import pandas as pd
import matplotlib.pyplot as plt

df1 = pd.read_csv("/home/inc/ros2_ws/src/navigation_system/navigation_system/waypointssaty120368_left2.csv")
df2 = pd.read_csv("/home/inc/ros2_ws/src/navigation_system/navigation_system/waypointssaty120368_mid2.csv")
df3 = pd.read_csv("/home/inc/ros2_ws/src/navigation_system/navigation_system/waypointssaty120368_right2.csv")
car = pd.read_csv("/home/inc/ros2_ws/src/navigation_system/navigation_system/waypointssaty120368_3.csv")

# ทำให้ column ชื่อเหมือนกัน
df1 = df1.rename(columns={"latitude":"lat","longitude":"lon"})
df2 = df2.rename(columns={"latitude":"lat","longitude":"lon"})
df3 = df3.rename(columns={"latitude":"lat","longitude":"lon"})
car = car.rename(columns={"latitude":"lat","longitude":"lon"})

plt.figure(figsize=(10,8))

# lanes
plt.plot(df1["lon"], df1["lat"], 'r', label="left lane")
plt.plot(df2["lon"], df2["lat"], 'g', label="mid lane")
plt.plot(df3["lon"], df3["lat"], 'b', label="right lane")

# car trajectory (เส้นปะเล็ก)
plt.plot(car["lon"], car["lat"], linestyle="--", linewidth=1.5, color="black", label="car")

plt.xlabel("Longitude")
plt.ylabel("Latitude")

plt.title("Vehicle Trajectory Comparison")

plt.legend()
plt.grid(True)
plt.axis("equal")

plt.show()






# import pandas as pd
# import numpy as np
# import matplotlib.pyplot as plt

# # อ่านไฟล์
# df = pd.read_csv("/home/inc/ros2_ws/src/navigation_system/navigation_system/waypointssaty120368_2.csv")

# # normalize heading (-180 ถึง 180)
# df["heading_fixed"] = ((df["heading"] + 180) % 360) - 180

# # สร้าง index สำหรับแกน x
# t = np.arange(len(df))

# plt.figure(figsize=(10,6))

# # ---------- Heading ----------
# plt.subplot(2,1,1)
# plt.plot(t, df["heading_fixed"])
# plt.title("Heading (-180 to 180)")
# plt.ylabel("Degree")
# plt.grid()

# # ---------- Speed ----------
# plt.subplot(2,1,2)
# plt.plot(t, df["speed"])
# plt.title("Speed")
# plt.xlabel("Index")
# plt.ylabel("Speed")
# plt.grid()

# plt.tight_layout()
# plt.show()