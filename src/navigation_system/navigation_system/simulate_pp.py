# import numpy as np
# import pandas as pd
# import matplotlib.pyplot as plt
# import math

# # ---------------- Parameters ----------------
# L = 1.67                    # Wheelbase (m)
# dt = 0.02                   # Time step (s)
# look_ahead_base = 3.0       # Base lookahead distance
# look_ahead_gain = 0.2       # Gain (depends on speed)
# max_steer_deg = 50          # Steering angle limit
# waypoint_reach_threshold = 1.0  # Distance threshold to count as "passed"
# goal_tolerance = 0.5        # Distance to final goal

# # ---------------- Load Waypoints ----------------
# waypoint_file = "waypoints_relative_xy.csv"
# data = pd.read_csv(waypoint_file)

# # ตรวจชื่อคอลัมน์ (รองรับทั้ง x,y หรือ utm_x,utm_y)
# if "x" in data.columns and "y" in data.columns:
#     waypoints = data[["x", "y"]].values
# elif "utm_x" in data.columns and "utm_y" in data.columns:
#     waypoints = data[["utm_x", "utm_y"]].values
# else:
#     raise ValueError("CSV ต้องมีคอลัมน์ x,y หรือ utm_x,utm_y")

# # Normalize จุดเริ่มต้นเป็น (0,0)
# waypoints[:, 0] -= waypoints[0, 0]
# waypoints[:, 1] -= waypoints[0, 1]

# # ---------------- Pure Pursuit Controller ----------------
# def find_target_index(state, waypoints, Ld):
#     # หา waypoint ที่ไกลกว่าระยะ Ld จากตำแหน่งปัจจุบัน
#     d = np.hypot(waypoints[:, 0] - state[0], waypoints[:, 1] - state[1])
#     for i in range(len(waypoints)):
#         if d[i] >= Ld:
#             return i
#     return len(waypoints) - 1

# def pure_pursuit_control(state, waypoints, v, last_target_idx):
#     Ld = np.clip(look_ahead_base + look_ahead_gain * v, 1.0, 10.0)
#     target_idx = find_target_index(state, waypoints[last_target_idx:], Ld) + last_target_idx
#     target_idx = min(target_idx, len(waypoints) - 1)
#     tx, ty = waypoints[target_idx]

#     # มุม yaw ถึงเป้าหมาย
#     alpha = math.atan2(ty - state[1], tx - state[0]) - state[2]
#     delta = math.atan2(2 * L * math.sin(alpha) / Ld, 1.0)
#     delta = np.clip(delta, -math.radians(max_steer_deg), math.radians(max_steer_deg))
#     return delta, target_idx

# def update_state(state, v, delta):
#     x, y, yaw = state
#     x += v * math.cos(yaw) * dt
#     y += v * math.sin(yaw) * dt
#     yaw += v / L * math.tan(delta) * dt
#     yaw = (yaw + np.pi) % (2 * np.pi) - np.pi  # normalize
#     return [x, y, yaw]

# # ---------------- Simulation ----------------
# state = [0.0, 0.0, 0.0]  # [x, y, yaw]
# path_x, path_y = [state[0]], [state[1]]
# target_idx = 0
# v = 1.5  # ความเร็วคงที่ (m/s)

# for step in range(20000):
#     delta, target_idx = pure_pursuit_control(state, waypoints, v, target_idx)
#     state = update_state(state, v, delta)
#     path_x.append(state[0])
#     path_y.append(state[1])

#     # ตรวจว่า waypoint ปัจจุบันถูกผ่านหรือยัง
#     dist_to_target = np.hypot(waypoints[target_idx, 0] - state[0],
#                               waypoints[target_idx, 1] - state[1])
#     if dist_to_target < waypoint_reach_threshold and target_idx < len(waypoints) - 1:
#         target_idx += 1

#     # หยุดเมื่อถึง waypoint สุดท้าย
#     if target_idx >= len(waypoints) - 1:
#         goal_dist = np.hypot(waypoints[-1, 0] - state[0], waypoints[-1, 1] - state[1])
#         if goal_dist < goal_tolerance:
#             print(f"✅ Reached goal at step {step}, total path points: {len(path_x)}")
#             break

# # ---------------- Plot ----------------
# plt.figure(figsize=(10, 6))
# plt.plot(waypoints[:, 0], waypoints[:, 1], 'r--', label='Desired Path (Waypoints)')
# plt.plot(path_x, path_y, 'b-', label='Actual Path (Pure Pursuit)')
# plt.plot(waypoints[0, 0], waypoints[0, 1], 'go', label='Start')
# plt.plot(waypoints[-1, 0], waypoints[-1, 1], 'm*', label='Goal')
# plt.xlabel("X Position (m)")
# plt.ylabel("Y Position (m)")
# plt.title("Path Tracking Performance (Pure Pursuit Controller)")
# plt.legend()
# plt.grid(True)
# plt.axis("equal")
# plt.show()




# import numpy as np
# import pandas as pd
# import matplotlib.pyplot as plt

# # ================= PARAMETERS =================
# L = 1.67
# v_max = 5.0
# dt = 0.05
# delta_max = np.deg2rad(50)
# GOAL_THRESHOLD = 0.3

# # ================= LOAD WAYPOINTS =================
# data = pd.read_csv("waypoints_relative_xy (copy).csv")
# path_x = data['x'].values
# path_y = data['y'].values
# path_len = len(path_x)

# print(f"Loaded {path_len} waypoints")

# # ================= INITIAL STATE =================
# x, y, yaw = path_x[0], path_y[0], 0.0
# v = 0.0
# target_idx = 0

# # ================= LOGS =================
# history_x, history_y = [], []
# history_v, history_delta = [], []   # delta = steering angle (deg)

# # ================= HELPER FUNCTIONS =================
# def normalize_angle(a):
#     return (a + np.pi) % (2 * np.pi) - np.pi

# def compute_curvature(i, px, py, w=3):
#     if i < w or i >= len(px) - w:
#         return 0.0
#     dx1, dy1 = px[i] - px[i-w], py[i] - py[i-w]
#     dx2, dy2 = px[i+w] - px[i], py[i+w] - py[i]
#     a1, a2 = np.arctan2(dy1, dx1), np.arctan2(dy2, dx2)
#     da = normalize_angle(a2 - a1)
#     d = (np.hypot(dx1, dy1) + np.hypot(dx2, dy2)) / 2.0
#     return 0.0 if d < 0.01 else abs(da / d)

# def update_target_idx(x, y, idx, px, py, r=10):
#     end = min(idx + r, len(px))
#     d = np.hypot(px[idx:end] - x, py[idx:end] - y)
#     return idx + np.argmin(d)

# def find_lookahead(x, y, Ld, px, py, idx):
#     d = np.hypot(px[idx:] - x, py[idx:] - y)
#     ids = np.where(d >= Ld)[0]
#     tidx = idx + ids[0] if len(ids) > 0 else min(idx+1, len(px)-1)
#     return px[tidx], py[tidx], tidx

# def compute_cte(x, y, px, py):
#     return np.min(np.hypot(px - x, py - y))

# # ================= SIMULATION =================
# for step in range(15000):
#     target_idx = update_target_idx(x, y, target_idx, path_x, path_y)

#     Ld = 0.5 * v + 0.8
#     tx, ty, look_idx = find_lookahead(x, y, Ld, path_x, path_y, target_idx)

#     alpha = normalize_angle(np.arctan2(ty - y, tx - x) - yaw)
#     delta = np.arctan2(2 * L * np.sin(alpha), Ld)
#     delta = np.clip(delta, -delta_max, delta_max)

#     curvature = compute_curvature(look_idx, path_x, path_y)
#     v_target = v_max * (1.0 - 3.5 * curvature)

#     dist_goal = np.hypot(x - path_x[-1], y - path_y[-1])
#     if dist_goal < 15:
#         v_target *= dist_goal / 15.0

#     v_target = np.clip(v_target, 0.5, v_max)
#     v += (v_target - v) * 0.3 * dt
#     v = np.clip(v, 0.0, v_max)

#     x += v * np.cos(yaw) * dt
#     y += v * np.sin(yaw) * dt
#     yaw += v / L * np.tan(delta) * dt
#     yaw = normalize_angle(yaw)

#     history_x.append(x)
#     history_y.append(y)
#     history_v.append(v)
#     history_delta.append(np.rad2deg(delta))

#     if dist_goal < GOAL_THRESHOLD:
#         print(f"✓ Goal reached at step {step}")
#         break

# # ================= CTE & RMSE =================
# history_cte = np.array([
#     compute_cte(x, y, path_x, path_y)
#     for x, y in zip(history_x, history_y)
# ])
# rmse_cte = np.sqrt(np.mean(history_cte ** 2))
# time = np.arange(len(history_cte)) * dt

# # ================= PLOTS =================
# plt.figure(figsize=(14, 18))

# # 1. Pure Pursuit Path
# plt.subplot(4, 1, 1)
# plt.plot(path_x, path_y, 'r--', lw=2.5, label='Reference Path')
# plt.plot(history_x, history_y, 'b-', lw=2.5, label='Vehicle Path')
# plt.scatter(path_x[0], path_y[0], c='g', s=100, label='Start')
# plt.scatter(path_x[-1], path_y[-1], c='r', s=100, label='Goal')
# plt.title("Pure Pursuit Path Tracking")
# plt.xlabel("X [m]")
# plt.ylabel("Y [m]")
# plt.axis('equal')
# plt.legend()
# plt.grid(alpha=0.3)

# # 2. Speed over Time
# plt.subplot(4, 1, 2)
# plt.plot(time, history_v, lw=2.5)
# plt.title("Speed over Time")
# plt.xlabel("Time [s]")
# plt.ylabel("Speed [m/s]")
# plt.grid(alpha=0.3)

# # 3. Steering Angle over Time
# plt.subplot(4, 1, 3)
# plt.plot(time, history_delta, lw=2.5)
# plt.axhline(np.rad2deg(delta_max), color='r', ls='--', alpha=0.5)
# plt.axhline(-np.rad2deg(delta_max), color='r', ls='--', alpha=0.5)
# plt.title("Steering Angle over Time")
# plt.xlabel("Time [s]")
# plt.ylabel("Steering Angle [deg]")
# plt.grid(alpha=0.3)

# # 4. CTE + RMSE
# plt.subplot(4, 1, 4)
# plt.plot(time, history_cte, lw=2.5)
# plt.axhline(rmse_cte, color='r', ls='--',
#             label=f'RMSE = {rmse_cte:.3f} m')
# plt.title("Cross-Track Error (CTE) over Time")
# plt.xlabel("Time [s]")
# plt.ylabel("CTE [m]")
# plt.legend()
# plt.grid(alpha=0.3)

# plt.tight_layout()
# plt.show()

# # ================= SUMMARY =================
# print("\n===== PURE PURSUIT PERFORMANCE =====")
# print(f"CTE RMSE : {rmse_cte:.3f} m")
# print(f"Max CTE  : {np.max(history_cte):.3f} m")
# print(f"Mean CTE : {np.mean(history_cte):.3f} m")
# print("===================================")








import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ================= PARAMETERS =================
L = 1.67                     # wheelbase (m)
v_max = 5.0                  # max speed (m/s)
dt = 0.05                    # timestep (s)
delta_max = np.deg2rad(50)   # max steering angle
GOAL_THRESHOLD = 0.3         # stop condition (m)

# ================= LOAD WAYPOINTS =================
data = pd.read_csv("waypoints_relative_xy (copy).csv")
path_x = data['x'].values
path_y = data['y'].values
path_len = len(path_x)

print(f"Loaded {path_len} waypoints")

# ================= INITIAL STATE =================
x, y, yaw = path_x[0], path_y[0], 0.0
v = 0.0
delta = 0.0
target_idx = 0

# ================= LOGS =================
history_x, history_y = [], []
history_v, history_delta = [], []

# ================= HELPER FUNCTIONS =================
def normalize_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def compute_curvature(i, px, py, w=3):
    if i < w or i >= len(px) - w:
        return 0.0
    dx1, dy1 = px[i] - px[i-w], py[i] - py[i-w]
    dx2, dy2 = px[i+w] - px[i], py[i+w] - py[i]
    a1 = np.arctan2(dy1, dx1)
    a2 = np.arctan2(dy2, dx2)
    da = normalize_angle(a2 - a1)
    d = (np.hypot(dx1, dy1) + np.hypot(dx2, dy2)) / 2.0
    return 0.0 if d < 0.01 else abs(da / d)


def update_target_idx(x, y, idx, px, py, r=10):
    end = min(idx + r, len(px))
    d = np.hypot(px[idx:end] - x, py[idx:end] - y)
    return idx + np.argmin(d)


def find_lookahead(x, y, Ld, px, py, idx):
    d = np.hypot(px[idx:] - x, py[idx:] - y)
    ids = np.where(d >= Ld)[0]
    tidx = idx + ids[0] if len(ids) > 0 else min(idx + 1, len(px) - 1)
    return px[tidx], py[tidx], tidx


def compute_cte_segment(x, y, px, py):
    """
    Compute CTE as perpendicular distance to the nearest path segment
    """
    min_dist = np.inf
    P = np.array([x, y])

    for i in range(len(px) - 1):
        A = np.array([px[i], py[i]])
        B = np.array([px[i+1], py[i+1]])

        AB = B - A
        AP = P - A

        ab_len2 = np.dot(AB, AB)

        if ab_len2 == 0:
            dist = np.linalg.norm(AP)
        else:
            t = np.dot(AP, AB) / ab_len2
            t = np.clip(t, 0.0, 1.0)

            proj = A + t * AB
            dist = np.linalg.norm(P - proj)

        if dist < min_dist:
            min_dist = dist

    return min_dist



# ===== Predict future state using kinematic bicycle model =====
def predict_state(x, y, yaw, v, delta, L, dt, steps=5):
    """
    Predict future vehicle state using kinematic bicycle model
    """
    x_p, y_p, yaw_p = x, y, yaw

    for _ in range(steps):
        x_p += v * np.cos(yaw_p) * dt
        y_p += v * np.sin(yaw_p) * dt
        yaw_p += v / L * np.tan(delta) * dt
        yaw_p = normalize_angle(yaw_p)

    return x_p, y_p, yaw_p


# ================= SIMULATION =================
for step in range(15000):

    # ===== Predict future position =====
    x_pred, y_pred, yaw_pred = predict_state(
        x, y, yaw,
        v, delta,
        L, dt,
        steps=5
    )

    # ===== Update target index using predicted position =====
    target_idx = update_target_idx(
        x_pred, y_pred,
        target_idx,
        path_x, path_y
    )

    # ===== Lookahead distance =====
    Ld = 0.5 * v + 0.8

    # ===== Find lookahead point ON WAYPOINTS =====
    tx, ty, look_idx = find_lookahead(
        x_pred, y_pred,
        Ld,
        path_x, path_y,
        target_idx
    )

    # ===== Pure Pursuit Steering (using predicted state) =====
    alpha = normalize_angle(
        np.arctan2(ty - y_pred, tx - x_pred) - yaw_pred
    )

    delta = np.arctan2(2 * L * np.sin(alpha), Ld)
    delta = np.clip(delta, -delta_max, delta_max)

    # ===== Speed control using curvature =====
    curvature = compute_curvature(look_idx, path_x, path_y)
    v_target = v_max * (1.0 - 3.5 * curvature)

    # Slow down near goal
    dist_goal = np.hypot(x - path_x[-1], y - path_y[-1])
    if dist_goal < 15:
        v_target *= dist_goal / 15.0

    v_target = np.clip(v_target, 0.5, v_max)
    v += (v_target - v) * 0.3 * dt
    v = np.clip(v, 0.0, v_max)

    # ===== Kinematic Bicycle Model (real motion) =====
    x += v * np.cos(yaw) * dt
    y += v * np.sin(yaw) * dt
    yaw += v / L * np.tan(delta) * dt
    yaw = normalize_angle(yaw)

    # ===== Log =====
    history_x.append(x)
    history_y.append(y)
    history_v.append(v)
    history_delta.append(np.rad2deg(delta))

    # ===== Stop condition =====
    if dist_goal < GOAL_THRESHOLD:
        print(f"✓ Goal reached at step {step}")
        break


# ================= CTE & RMSE =================
history_cte = np.array([
    compute_cte_segment(x, y, path_x, path_y)
    for x, y in zip(history_x, history_y)
])


rmse_cte = np.sqrt(np.mean(history_cte ** 2))
time = np.arange(len(history_cte)) * dt


# ================= PLOTS =================
plt.figure(figsize=(14, 18))

# 1. Path tracking
plt.subplot(4, 1, 1)
plt.plot(path_x, path_y, 'r--', lw=2.5, label='Reference Path')
plt.plot(history_x, history_y, 'b-', lw=2.5, label='Vehicle Path')
plt.scatter(path_x[0], path_y[0], c='g', s=100, label='Start')
plt.scatter(path_x[-1], path_y[-1], c='r', s=100, label='Goal')
plt.title("Predictive Pure Pursuit Path Tracking")
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.axis('equal')
plt.legend()
plt.grid(alpha=0.3)

# 2. Speed
plt.subplot(4, 1, 2)
plt.plot(time, history_v, lw=2.5)
plt.title("Speed over Time")
plt.xlabel("Time [s]")
plt.ylabel("Speed [m/s]")
plt.grid(alpha=0.3)

# 3. Steering angle
plt.subplot(4, 1, 3)
plt.plot(time, history_delta, lw=2.5)
plt.axhline(np.rad2deg(delta_max), color='r', ls='--', alpha=0.5)
plt.axhline(-np.rad2deg(delta_max), color='r', ls='--', alpha=0.5)
plt.title("Steering Angle over Time")
plt.xlabel("Time [s]")
plt.ylabel("Steering Angle [deg]")
plt.grid(alpha=0.3)

# 4. CTE + RMSE
plt.subplot(4, 1, 4)
plt.plot(time, history_cte, lw=2.5)
plt.axhline(rmse_cte, color='r', ls='--',
            label=f'RMSE = {rmse_cte:.3f} m')
plt.title("Cross-Track Error (CTE)")
plt.xlabel("Time [s]")
plt.ylabel("CTE [m]")
plt.legend()
plt.grid(alpha=0.3)

plt.tight_layout()
plt.show()

# ================= SUMMARY =================
print("\n===== PREDICTIVE PURE PURSUIT PERFORMANCE =====")
print(f"CTE RMSE : {rmse_cte:.3f} m")
print(f"Max CTE  : {np.max(history_cte):.3f} m")
print(f"Mean CTE : {np.mean(history_cte):.3f} m")
print("==============================================")
