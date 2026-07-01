# import pandas as pd
# import numpy as np
# import matplotlib.pyplot as plt
# from scipy.spatial import cKDTree

# # ============================================================
# # CONFIG
# # ============================================================

# # ไฟล์ผลวิ่งจริง
# result_csv = "/home/inc/ros2_ws/output_logs/navigation_20260528_232453.csv"

# # ไฟล์ waypoints
# waypoint_csv = "/home/inc/ros2_ws/src/navigation_system/logs13/path_smoothlog13_resampled_1m.csv"

# # ============================================================
# # LOAD RESULT CSV (ROBUST)
# # ============================================================

# column_names = [
#     "x",
#     "y",
#     "speed",
#     "heading",
#     "fix_quality",
#     "steering_angle"
# ]

# # อ่านไฟล์แบบข้ามบรรทัดเสีย
# df = pd.read_csv(
#     result_csv,
#     on_bad_lines="skip",
#     engine="python"
# )

# # เอาแค่ 6 columns แรก
# df = df.iloc[:, :6]
# df.columns = column_names

# # แปลงเป็น numeric
# for col in column_names:
#     df[col] = pd.to_numeric(df[col], errors="coerce")

# # ลบแถวเสีย
# df = df.dropna()

# print(f"Loaded valid rows: {len(df)}")

# # ============================================================
# # LOAD WAYPOINT CSV
# # ============================================================

# waypoints = pd.read_csv(waypoint_csv)

# waypoints["x"] = pd.to_numeric(
#     waypoints["x"],
#     errors="coerce"
# )

# waypoints["y"] = pd.to_numeric(
#     waypoints["y"],
#     errors="coerce"
# )

# waypoints = waypoints.dropna()

# # ============================================================
# # EXTRACT DATA
# # ============================================================

# x_real = df["x"].values
# y_real = df["y"].values

# speed = df["speed"].values
# heading = df["heading"].values
# fix_quality = df["fix_quality"].values
# steering = df["steering_angle"].values

# x_wp = waypoints["x"].values
# y_wp = waypoints["y"].values

# # ============================================================
# # COMPUTE CTE
# # nearest waypoint distance
# # ============================================================

# waypoint_points = np.column_stack((x_wp, y_wp))
# real_points = np.column_stack((x_real, y_real))

# tree = cKDTree(waypoint_points)

# distances, nearest_idx = tree.query(real_points)

# cte = distances

# # ============================================================
# # STATISTICS
# # ============================================================

# speed_avg = np.mean(speed)
# speed_min = np.min(speed)
# speed_max = np.max(speed)

# cte_avg = np.mean(cte)
# cte_min = np.min(cte)
# cte_max = np.max(cte)

# # ============================================================
# # GRAPH 1 : TRAJECTORY
# # ============================================================

# plt.figure(figsize=(10, 8))

# # waypoints (เล็กมาก)
# plt.scatter(
#     x_wp,
#     y_wp,
#     color="black",
#     s=3,
#     label="Waypoints"
# )

# # RTK FIX
# fix_mask = fix_quality == 4

# plt.scatter(
#     x_real[fix_mask],
#     y_real[fix_mask],
#     color="#00FF00",   # เขียวสดมาก
#     s=1,
#     label="RTK Fix"
# )

# # RTK FLOAT
# float_mask = fix_quality == 5

# plt.scatter(
#     x_real[float_mask],
#     y_real[float_mask],
#     color="orange",
#     s=1,   # จุดเล็กมาก
#     label="RTK Float"
# )

# plt.xlabel("X (m)")
# plt.ylabel("Y (m)")
# plt.title("Waypoint vs Real Trajectory")

# plt.axis("equal")
# plt.grid(True)
# plt.legend(loc="upper right")

# plt.tight_layout()
# plt.savefig(
#     "graph_1_trajectory.png",
#     dpi=300
# )
# plt.close()

# # ============================================================
# # GRAPH 2 : CTE
# # ============================================================

# plt.figure(figsize=(10, 5))

# plt.plot(cte, linewidth=1)

# cte_text = (
#     f"avg = {cte_avg:.3f} m\n"
#     f"min = {cte_min:.3f} m\n"
#     f"max = {cte_max:.3f} m"
# )

# plt.text(
#     0.98,
#     0.98,
#     cte_text,
#     transform=plt.gca().transAxes,
#     verticalalignment="top",
#     horizontalalignment="right",
#     bbox=dict(
#         facecolor="white",
#         alpha=0.8
#     )
# )

# plt.xlabel("Sample Index")
# plt.ylabel("CTE (m)")
# plt.title("Cross Track Error (CTE)")
# plt.grid(True)

# plt.tight_layout()
# plt.savefig(
#     "graph_2_cte.png",
#     dpi=300
# )
# plt.close()

# # ============================================================
# # GRAPH 3 : HEADING / YAW
# # ============================================================

# plt.figure(figsize=(10, 5))

# plt.plot(
#     heading,
#     linewidth=1
# )

# plt.xlabel("Sample Index")
# plt.ylabel("Yaw (deg)")
# plt.title("Yaw / Heading")

# plt.ylim(-180, 180)

# plt.grid(True)

# plt.tight_layout()
# plt.savefig(
#     "graph_3_heading.png",
#     dpi=300
# )
# plt.close()

# # ============================================================
# # GRAPH 4 : STEERING ANGLE
# # ============================================================

# plt.figure(figsize=(10, 5))

# plt.plot(
#     steering,
#     linewidth=1
# )

# plt.xlabel("Sample Index")
# plt.ylabel("Steering Angle (deg)")
# plt.title("Steering Angle")

# plt.grid(True)

# plt.tight_layout()
# plt.savefig(
#     "graph_4_steering.png",
#     dpi=300
# )
# plt.close()

# # ============================================================
# # GRAPH 5 : SPEED
# # ============================================================

# plt.figure(figsize=(10, 5))

# plt.plot(
#     speed,
#     linewidth=1
# )

# stats_text = (
#     f"avg = {speed_avg:.3f} m/s\n"
#     f"min = {speed_min:.3f} m/s\n"
#     f"max = {speed_max:.3f} m/s"
# )

# plt.text(
#     0.98,
#     0.98,
#     stats_text,
#     transform=plt.gca().transAxes,
#     verticalalignment="top",
#     horizontalalignment="right",
#     bbox=dict(
#         facecolor="white",
#         alpha=0.8
#     )
# )

# plt.xlabel("Sample Index")
# plt.ylabel("Speed (m/s)")
# plt.title("Speed")

# plt.grid(True)

# plt.tight_layout()
# plt.savefig(
#     "graph_5_speed.png",
#     dpi=300
# )
# plt.close()

# # ============================================================
# # DONE
# # ============================================================

# print("\nSaved:")
# print("graph_1_trajectory.png")
# print("graph_2_cte.png")
# print("graph_3_heading.png")
# print("graph_4_steering.png")
# print("graph_5_speed.png")

# print("\nSpeed Statistics")
# print(f"AVG : {speed_avg:.3f}")
# print(f"MIN : {speed_min:.3f}")
# print(f"MAX : {speed_max:.3f}")

# print("\nCTE Statistics")
# print(f"AVG : {cte_avg:.3f}")
# print(f"MIN : {cte_min:.3f}")
# print(f"MAX : {cte_max:.3f}")


import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree

# ============================================================
# CONFIG
# ============================================================

# ไฟล์ผลวิ่งจริง
result_csv = "/home/inc/ros2_ws/output_logs/navigation_20260601_195132.csv"

# ไฟล์ waypoints
waypoint_csv = "/home/inc/ros2_ws/src/navigation_system/logs13/path_smoothlog13_resampled_1m.csv"

# Sampling rate ของ logger
# 10 Hz = 10 samples/sec
sample_rate = 10.0

# ============================================================
# LOAD RESULT CSV
# ============================================================

column_names = [
    "x",
    "y",
    "speed",
    "heading",
    "fix_quality",
    "steering_angle"
]

df = pd.read_csv(
    result_csv,
    on_bad_lines="skip",
    engine="python"
)

# เอา 6 columns แรก
df = df.iloc[:, :6]
df.columns = column_names

# แปลงเป็น numeric
for col in column_names:
    df[col] = pd.to_numeric(
        df[col],
        errors="coerce"
    )

# ลบแถวเสีย
df = df.dropna()

print(f"Loaded valid rows: {len(df)}")

# ============================================================
# LOAD WAYPOINT CSV
# ============================================================

waypoints = pd.read_csv(
    waypoint_csv
)

waypoints["x"] = pd.to_numeric(
    waypoints["x"],
    errors="coerce"
)

waypoints["y"] = pd.to_numeric(
    waypoints["y"],
    errors="coerce"
)

waypoints = waypoints.dropna()

# ============================================================
# EXTRACT DATA
# ============================================================

x_real = df["x"].values
y_real = df["y"].values

speed = df["speed"].values
heading = df["heading"].values
fix_quality = df["fix_quality"].values
steering = df["steering_angle"].values

x_wp = waypoints["x"].values
y_wp = waypoints["y"].values

# ============================================================
# TIME AXIS (SECOND)
# ============================================================

time_sec = np.arange(
    len(df)
) / sample_rate

# ============================================================
# YAW START FROM 0°
# ============================================================

heading_zero = (
    heading - heading[0]
)

# wrap angle -> [-180,180]
heading_zero = (
    (heading_zero + 180) % 360
) - 180

# ============================================================
# COMPUTE SIGNED CTE
# ============================================================

waypoint_points = np.column_stack(
    (x_wp, y_wp)
)

real_points = np.column_stack(
    (x_real, y_real)
)

tree = cKDTree(
    waypoint_points
)

# nearest waypoint
distances, nearest_idx = tree.query(
    real_points
)

cte_signed = []

for i, idx in enumerate(
    nearest_idx
):

    # ป้องกัน index หลุด
    idx2 = min(
        idx + 1,
        len(x_wp) - 1
    )

    # path vector
    path_vec = np.array([
        x_wp[idx2] - x_wp[idx],
        y_wp[idx2] - y_wp[idx]
    ])

    # waypoint -> vehicle vector
    car_vec = np.array([
        x_real[i] - x_wp[idx],
        y_real[i] - y_wp[idx]
    ])

    # cross product
    cross = (
        path_vec[0] * car_vec[1]
        - path_vec[1] * car_vec[0]
    )

    sign = np.sign(
        cross
    )

    signed_distance = (
        distances[i] * sign
    )

    cte_signed.append(
        signed_distance
    )

cte_signed = np.array(
    cte_signed
)

# ============================================================
# STATISTICS
# ใช้ค่า CTE เดิม (absolute)
# ============================================================

speed_avg = np.mean(
    speed
)
speed_min = np.min(
    speed
)
speed_max = np.max(
    speed
)

cte_avg = np.mean(
    distances
)
cte_min = np.min(
    distances
)
cte_max = np.max(
    distances
)

# ============================================================
# GRAPH 1 : TRAJECTORY
# ============================================================

plt.figure(
    figsize=(10, 8)
)

# Waypoints
plt.scatter(
    x_wp,
    y_wp,
    color="black",
    s=3,
    label="Waypoints"
)

# RTK Fixed
fix_mask = (
    fix_quality == 4
)

plt.scatter(
    x_real[fix_mask],
    y_real[fix_mask],
    color="#00FF00",
    s=1,
    label="RTK Fixed"
)

# RTK Float
float_mask = (
    fix_quality == 5
)

plt.scatter(
    x_real[float_mask],
    y_real[float_mask],
    color="orange",
    s=1,
    label="RTK Float"
)

plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title(
    "Waypoint vs Real Trajectory"
)

plt.axis("equal")
plt.grid(True)
plt.legend(
    loc="upper right"
)

plt.tight_layout()

plt.savefig(
    "graph_1_trajectory.png",
    dpi=300
)

plt.close()

# ============================================================
# GRAPH 2 : SIGNED CTE
# ============================================================

plt.figure(
    figsize=(10, 5)
)

plt.plot(
    time_sec,
    cte_signed,
    linewidth=1
)

plt.axhline(
    0,
    linestyle="--"
)

cte_text = (
    f"avg = {cte_avg:.3f} m\n"
    f"min = {cte_min:.3f} m\n"
    f"max = {cte_max:.3f} m"
)

plt.text(
    0.98,
    0.98,
    cte_text,
    transform=plt.gca().transAxes,
    verticalalignment="top",
    horizontalalignment="right",
    bbox=dict(
        facecolor="white",
        alpha=0.8
    )
)

plt.xlabel(
    "Time (s)"
)

plt.ylabel(
    "Signed CTE (m)"
)

plt.title(
    "Cross Track Error"
)

plt.grid(True)

plt.tight_layout()

plt.savefig(
    "graph_2_cte.png",
    dpi=300
)

plt.close()

# ============================================================
# GRAPH 3 : YAW
# ============================================================

plt.figure(
    figsize=(10, 5)
)

plt.plot(
    time_sec,
    heading_zero,
    linewidth=1
)

plt.xlabel(
    "Time (s)"
)

plt.ylabel(
    "Yaw (deg)"
)

plt.title(
    "Yaw / Heading"
)

plt.ylim(
    -180,
    180
)

plt.grid(True)

plt.tight_layout()

plt.savefig(
    "graph_3_heading.png",
    dpi=300
)

plt.close()

# ============================================================
# GRAPH 4 : STEERING ANGLE
# ============================================================

plt.figure(
    figsize=(10, 5)
)

plt.plot(
    time_sec,
    steering,
    linewidth=1
)

plt.xlabel(
    "Time (s)"
)

plt.ylabel(
    "Steering Angle (deg)"
)

plt.title(
    "Steering Angle"
)

plt.grid(True)

plt.tight_layout()

plt.savefig(
    "graph_4_steering.png",
    dpi=300
)

plt.close()

# ============================================================
# GRAPH 5 : SPEED
# ============================================================

plt.figure(
    figsize=(10, 5)
)

plt.plot(
    time_sec,
    speed,
    linewidth=1
)

stats_text = (
    f"avg = {speed_avg:.3f} m/s\n"
    f"min = {speed_min:.3f} m/s\n"
    f"max = {speed_max:.3f} m/s"
)

plt.text(
    0.98,
    0.98,
    stats_text,
    transform=plt.gca().transAxes,
    verticalalignment="top",
    horizontalalignment="right",
    bbox=dict(
        facecolor="white",
        alpha=0.8
    )
)

plt.xlabel(
    "Time (s)"
)

plt.ylabel(
    "Speed (m/s)"
)

plt.title(
    "Speed"
)

plt.grid(True)

plt.tight_layout()

plt.savefig(
    "graph_5_speed.png",
    dpi=300
)

plt.close()

# ============================================================
# DONE
# ============================================================

print("\nSaved:")
print("graph_1_trajectory.png")
print("graph_2_cte.png")
print("graph_3_heading.png")
print("graph_4_steering.png")
print("graph_5_speed.png")

print("\nSpeed Statistics")
print(f"AVG : {speed_avg:.3f}")
print(f"MIN : {speed_min:.3f}")
print(f"MAX : {speed_max:.3f}")

print("\nCTE Statistics")
print(f"AVG : {cte_avg:.3f}")
print(f"MIN : {cte_min:.3f}")
print(f"MAX : {cte_max:.3f}")