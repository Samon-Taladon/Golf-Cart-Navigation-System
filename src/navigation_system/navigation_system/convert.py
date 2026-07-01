# ----------------------------------latlongtoxy-------------------------------------
# import csv
# import math

# # จุดอ้างอิง (Reference Point) - ปรับได้ตามต้องการ
# REFERENCE_POINT = (13.64998983, 100.4927375)  # ใช้จุดแรกสุดในไฟล์

# def simple_latlon_to_xy(lat, lon, ref_point):
#     dlat = lat - ref_point[0]
#     dlon = lon - ref_point[1]
#     x = dlon * 111319.9 * math.cos(math.radians(lat))
#     y = dlat * 111319.9
#     return x, y

# # อ่านข้อมูลจากไฟล์ CSV
# input_filename = "/home/inc/ros2_ws/waypointfile/waypointfrommap.csv"
# output_filename = "/home/inc/ros2_ws/waypointfile/waypointfrommapxy.csv"

# with open(input_filename, newline='', encoding='utf-8') as csvfile:
#     reader = csv.DictReader(csvfile)
#     output_rows = []

#     for row in reader:
#         lat = float(row['latitude'])
#         lon = float(row['longitude'])
#         x, y = simple_latlon_to_xy(lat, lon, REFERENCE_POINT)
#         output_rows.append({'X': x, 'Y': y})

# # เขียนผลลัพธ์ลงไฟล์ใหม่
# with open(output_filename, mode='w', newline='', encoding='utf-8') as csvfile:
#     fieldnames = ['X', 'Y']
#     writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
#     writer.writeheader()
#     writer.writerows(output_rows)

# print("แปลงสำเร็จ! บันทึกไฟล์เป็น", output_filename)


# import csv
# from pyproj import Transformer

# # สร้างตัวแปลงพิกัดจาก WGS84 (lat/lon) ไปเป็น UTM zone 47N (EPSG:32647)
# transformer = Transformer.from_crs("EPSG:4326", "EPSG:32647", always_xy=True)

# # ชื่อไฟล์
# input_filename = "/home/inc/ros2_ws/waypointfile/waypointfrommap.csv"
# output_filename = "/home/inc/ros2_ws/waypointfile/waypointfrommapxy.csv"

# with open(input_filename, newline='', encoding='utf-8') as csvfile:
#     reader = csv.DictReader(csvfile)
#     output_rows = []

#     for row in reader:
#         lat = float(row['latitude'])
#         lon = float(row['longitude'])
#         x, y = transformer.transform(lon, lat)  # ลำดับ: lon, lat
#         output_rows.append({'x_east': x, 'y_north': y})

# # เขียนผลลัพธ์
# with open(output_filename, mode='w', newline='', encoding='utf-8') as csvfile:
#     fieldnames = ['x_east', 'y_north']
#     writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
#     writer.writeheader()
#     writer.writerows(output_rows)

# print("✅ แปลงพิกัดสำเร็จเป็นระบบ UTM (EPSG:32647) ->", output_filename)


# --------------------------xytolatlong------------------------------
import pandas as pd
import pyproj

class UTMToLatLonConverter:
    def __init__(self):
        # ตั้งค่าตัวแปลง UTM Zone 47N ไป WGS84
        self.transformer = pyproj.Transformer.from_crs("epsg:32647", "epsg:4326", always_xy=True)

    def convert(self, x, y):
        lon, lat = self.transformer.transform(x, y)
        return lat, lon

    def convert_csv(self, input_csv, output_csv):
        # อ่านข้อมูล CSV
        df = pd.read_csv(input_csv)

        # เตรียม list สำหรับเก็บผลลัพธ์
        latitudes = []
        longitudes = []

        # แปลงค่าทุกแถว
        for x, y in zip(df['x_east'], df['y_north']):
            lat, lon = self.convert(x, y)
            latitudes.append(lat)
            longitudes.append(lon)

        # สร้าง DataFrame ใหม่เฉพาะ latitude, longitude
        df_result = pd.DataFrame({
            'latitude': latitudes,
            'longitude': longitudes
        })

        # เซฟเป็นไฟล์ CSV ใหม่
        df_result.to_csv(output_csv, index=False)
        print(f"✅ แปลงเสร็จแล้ว บันทึกเป็น {output_csv}")

if __name__ == "__main__":
    # ตั้งชื่อไฟล์ที่ต้องการแปลง และไฟล์ผลลัพธ์
    input_csv = '/home/inc/ros2_ws/waypointfile/waypoints3xy.csv'
    output_csv = '/home/inc/ros2_ws/waypointfile/waypoints3latlong.csv'

    # สร้างอ็อบเจกต์ตัวแปลง และแปลงไฟล์ CSV
    converter = UTMToLatLonConverter()
    converter.convert_csv(input_csv, output_csv)
