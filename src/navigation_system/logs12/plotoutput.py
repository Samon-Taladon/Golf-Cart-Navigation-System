import pandas as pd
import matplotlib.pyplot as plt

# อ่านไฟล์ CSV (เปลี่ยนชื่อไฟล์ตามจริง)
df = pd.read_csv("/home/inc/ros2_ws/output_logs/navigation_20260525_221557.csv")

# ===== Plot trajectory x-y =====
plt.figure(figsize=(6,6))
plt.plot(df['x'], df['y'], marker='o')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Trajectory (x vs y)')
plt.grid(True)
plt.axis('equal')
plt.show()


# ===== Plot speed =====
plt.figure(figsize=(8,4))
plt.plot(df.index, df['speed'])
plt.xlabel('Index')
plt.ylabel('Speed')
plt.title('Speed over Time')
plt.grid(True)
plt.show()


# ===== Plot heading =====
plt.figure(figsize=(8,4))
plt.plot(df.index, df['heading'])
plt.xlabel('Index')
plt.ylabel('Heading')
plt.title('Heading over Time')
plt.grid(True)
plt.show()


# ===== Plot steering angle =====
plt.figure(figsize=(8,4))
plt.plot(df.index, df['steering_angle'])
plt.xlabel('Index')
plt.ylabel('Steering Angle')
plt.title('Steering Angle over Time')
plt.grid(True)
plt.show()