import pandas as pd
import numpy as np

# =========================
# FILE
# =========================
INPUT_FILE = "waypoints_utmlog6.csv"
OUTPUT_FILE = "path_smoothlog6.csv"

# =========================
# LOAD CSV
# =========================
df = pd.read_csv(INPUT_FILE)

# =========================
# MOVING AVERAGE SMOOTH
# =========================
WINDOW_SIZE = 5

df['x_smooth'] = (
    df['x']
    .rolling(window=WINDOW_SIZE, center=True)
    .mean()
)

df['y_smooth'] = (
    df['y']
    .rolling(window=WINDOW_SIZE, center=True)
    .mean()
)

# เติมค่าขอบที่เป็น NaN
df['x_smooth'] = df['x_smooth'].fillna(df['x'])
df['y_smooth'] = df['y_smooth'].fillna(df['y'])

# =========================
# DOWNSAMPLE
# เก็บจุดห่างกัน >= 0.5 m
# =========================
DIST_THRESHOLD = 0.5

smooth_points = []

last_x = None
last_y = None

for _, row in df.iterrows():

    x = row['x_smooth']
    y = row['y_smooth']

    if last_x is None:
        smooth_points.append([x, y])
        last_x = x
        last_y = y
        continue

    dist = np.sqrt((x - last_x)**2 + (y - last_y)**2)

    if dist >= DIST_THRESHOLD:
        smooth_points.append([x, y])
        last_x = x
        last_y = y

# =========================
# SAVE
# =========================
smooth_df = pd.DataFrame(smooth_points, columns=['x', 'y'])

smooth_df.to_csv(OUTPUT_FILE, index=False)

print("✅ Saved:", OUTPUT_FILE)
print("Total points:", len(smooth_df))