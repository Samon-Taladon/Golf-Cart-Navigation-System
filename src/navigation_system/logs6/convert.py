import pandas as pd
from pyproj import Transformer

# อ่านไฟล์ CSV
df = pd.read_csv("/home/inc/ros2_ws/src/navigation_system/logs6/waypoints_clean.csv")

# สร้างตัวแปลงพิกัด
# UTM Zone 47N (ประเทศไทย)
transformer = Transformer.from_crs(
    "epsg:4326",   # WGS84 lat/lon
    "epsg:32647",  # UTM Zone 47N
    always_xy=True
)

# แปลง lon, lat -> UTM x, y
x, y = transformer.transform(df["lon"].values, df["lat"].values)

# แทนที่ lat/lon ด้วย x/y
df["x"] = x
df["y"] = y

# ลบคอลัมน์เดิม
df = df.drop(columns=["lat", "lon"])

# จัดลำดับคอลัมน์ใหม่
df = df[["x", "y", "speed", "heading", "fix_quality"]]

# บันทึกไฟล์ใหม่
df.to_csv("/home/inc/ros2_ws/src/navigation_system/logs6/waypoints_utmlog6.csv", index=False)

print(df.head())