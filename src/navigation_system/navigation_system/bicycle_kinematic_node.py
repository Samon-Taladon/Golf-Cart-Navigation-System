# #!/usr/bin/env python3
# """
# GNSS Reference Point Manager
# จัดการ reference point สำหรับ Enhanced GNSS Node
# """

# import json
# import os
# import sys
# import argparse
# from datetime import datetime

# class ReferenceManager:
#     def __init__(self):
#         self.reference_file = "gnss_reference.json"
    
#     def show_current(self):
#         """แสดง reference point ปัจจุบัน"""
#         if not os.path.exists(self.reference_file):
#             print("❌ No reference file found")
#             return False
        
#         try:
#             with open(self.reference_file, 'r') as f:
#                 data = json.load(f)
            
#             timestamp = datetime.fromtimestamp(data['timestamp'])
            
#             print("📍 Current Reference Point:")
#             print(f"   Latitude:  {data['latitude']:.8f}")
#             print(f"   Longitude: {data['longitude']:.8f}")
#             print(f"   UTM X:     {data['utm_x']:.2f}")
#             print(f"   UTM Y:     {data['utm_y']:.2f}")
#             print(f"   Created:   {timestamp.strftime('%Y-%m-%d %H:%M:%S')}")
#             print(f"   Compatible: {data.get('waypoints_compatible', 'Unknown')}")
            
#             return True
            
#         except Exception as e:
#             print(f"❌ Error reading reference file: {e}")
#             return False
    
#     def set_reference(self, lat, lon):
#         """ตั้ง reference point ใหม่"""
#         try:
#             # Validate coordinates
#             if not (-90 <= lat <= 90):
#                 print(f"❌ Invalid latitude: {lat}")
#                 return False
            
#             if not (-180 <= lon <= 180):
#                 print(f"❌ Invalid longitude: {lon}")
#                 return False
            
#             # Calculate UTM (simplified - in real use, import pyproj)
#             utm_x = lon * 111320  # Approximate
#             utm_y = lat * 110540  # Approximate
            
#             data = {
#                 'latitude': lat,
#                 'longitude': lon,
#                 'utm_x': utm_x,
#                 'utm_y': utm_y,
#                 'timestamp': datetime.now().timestamp(),
#                 'waypoints_compatible': True,
#                 'set_manually': True
#             }
            
#             with open(self.reference_file, 'w') as f:
#                 json.dump(data, f, indent=2)
            
#             print(f"✅ Reference point set to: {lat:.8f}, {lon:.8f}")
#             return True
            
#         except Exception as e:
#             print(f"❌ Error setting reference: {e}")
#             return False
    
#     def reset_reference(self):
#         """ลบ reference point (จะใช้ตำแหน่งแรกที่ได้รับ)"""
#         try:
#             if os.path.exists(self.reference_file):
#                 # Backup before delete
#                 backup_name = f"{self.reference_file}.backup.{int(datetime.now().timestamp())}"
#                 os.rename(self.reference_file, backup_name)
#                 print(f"💾 Backed up to: {backup_name}")
            
#             print("🔄 Reference point reset")
#             print("   Next GNSS fix will be used as new reference")
#             return True
            
#         except Exception as e:
#             print(f"❌ Error resetting reference: {e}")
#             return False
    
#     def check_waypoints_compatibility(self):
#         """ตรวจสอบความเข้ากันได้กับ waypoints.json"""
#         if not os.path.exists("waypoints.json"):
#             print("⚠️ No waypoints.json found")
#             return
        
#         if not os.path.exists(self.reference_file):
#             print("⚠️ No reference file found")
#             return
        
#         try:
#             # Read reference
#             with open(self.reference_file, 'r') as f:
#                 ref_data = json.load(f)
            
#             # Read waypoints
#             with open("waypoints.json", 'r') as f:
#                 wp_data = json.load(f)
            
#             print("🔍 Compatibility Check:")
#             print(f"   Reference created: {datetime.fromtimestamp(ref_data['timestamp'])}")
#             print(f"   Waypoints created: {datetime.fromtimestamp(wp_data['metadata']['created_time'])}")
            
#             time_diff = abs(ref_data['timestamp'] - wp_data['metadata']['created_time'])
            
#             if time_diff < 3600:  # 1 hour
#                 print("✅ Reference and waypoints are compatible (same session)")
#             elif time_diff < 86400:  # 1 day
#                 print("⚠️ Reference and waypoints may be compatible (same day)")
#             else:
#                 print("❌ Reference and waypoints are likely incompatible")
#                 print("   Consider recording waypoints again or resetting reference")
            
#         except Exception as e:
#             print(f"❌ Error checking compatibility: {e}")

# def main():
#     parser = argparse.ArgumentParser(description='GNSS Reference Point Manager')
#     parser.add_argument('action', choices=['show', 'set', 'reset', 'check'], 
#                        help='Action to perform')
#     parser.add_argument('--lat', type=float, help='Latitude for set action')
#     parser.add_argument('--lon', type=float, help='Longitude for set action')
    
#     args = parser.parse_args()
    
#     manager = ReferenceManager()
    
#     if args.action == 'show':
#         manager.show_current()
    
#     elif args.action == 'set':
#         if args.lat is None or args.lon is None:
#             print("❌ --lat and --lon required for set action")
#             print("Example: python3 reference_manager.py set --lat 13.6500330 --lon 100.4929967")
#             sys.exit(1)
#         manager.set_reference(args.lat, args.lon)
    
#     elif args.action == 'reset':
#         confirm = input("⚠️ Reset reference point? (y/N): ")
#         if confirm.lower() == 'y':
#             manager.reset_reference()
#         else:
#             print("❌ Reset cancelled")
    
#     elif args.action == 'check':
#         manager.check_waypoints_compatibility()

# if __name__ == '__main__':
#     main()


import pandas as pd
import numpy as np

# อ่านไฟล์ waypoints ที่ผู้ใช้ส่งมา
file_path = "/home/inc/ros2_ws/waypoints.csv"
df = pd.read_csv(file_path)

# ดูค่าเริ่มต้นและสิ้นสุด
start_point = df.iloc[0]
end_point = df.iloc[-1]

# คำนวณระยะทางทั้งหมดระหว่างจุดเริ่มและจุดสิ้นสุด
from math import sqrt

distance_total = sqrt((end_point['x_east'] - start_point['x_east'])**2 + 
                      (end_point['y_north'] - start_point['y_north'])**2)

# สร้างจำนวนจุดที่ต้องการ (spacing 0.5 m)
spacing = 0.5  # เมตร
num_points = int(distance_total // spacing) + 1

# สร้าง waypoint ใหม่ที่ห่างกัน 0.5 เมตร
x_new = np.linspace(start_point['x_east'], end_point['x_east'], num_points)
y_new = np.linspace(start_point['y_north'], end_point['y_north'], num_points)

# รวมเป็น DataFrame ใหม่
waypoints_generated = pd.DataFrame({'x_east': x_new, 'y_north': y_new})

# บันทึกไฟล์
output_path = "/home/inc/ros2_ws/generated_waypoints_0_5m.csv"
waypoints_generated.to_csv(output_path, index=False)

# ส่ง path กลับไปให้โหลด
output_path
