build 
cd ~/ros2_ws
colcon build
source install/setup.bash
----------------------------------------
ros2 run ublox_gps ublox_gps_node --ros-args -p serial_port:=/dev/ttyACM0
-------------------------------------------
python3 gnss.py
ros2 topic echo /gps
ros2 topic echo /gps/fix

-------------------------------------------
ให้สิทธิ์การเข้าถึงพอร์ต
sudo usermod -a -G dialout $USER

ดูค่าgnss
sudo screen /dev/ttyACM0 9600
ลบหน้าจอที่รันอยู่
screen -ls | grep -o '[0-9]*\.' | xargs -I {} screen -X -S {} quit


python3 gnss_server.py
python3 gnss_receiver.py
python3 gps_marker_node.py
ros2 topic info /gps

------------เช็คพอร์ต-------------------------------
ls /dev/ttyACM*0
sudo lsof /dev/ttyACM0
sudo kill -9 1234		

-----------------------------------------------------
ros2 topic pub /gps/navsat_fix sensor_msgs/msg/NavSatFix '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, status: {status: 0, service: 0}, latitude: 13.649963833333333, longitude: 100.492796, altitude: 0.0, position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], position_covariance_type: 0}'


ros2 run nav2_map_server map_server
ros2 topic echo /rosout

---------------------------see rviz----------------------------------
python3 gnss_receiver.py
ros2 topic echo /gps/fix
python3 marker.py
colcon build --symlink-install --packages-select rviz_satellite
source install/setup.bash
rviz2 --ros-args --log-level debug
https://a.tile.openstreetmap.org/{z}/{x}/{y}.png



------------------------------------web---------------------------------------
colcon build --symlink-install
source install/setup.bash
ros2 run navigation_system gnss_publisher



colcon build --packages-select navigation_system
source install/setup.bash
--------------------------------------------------------------------
ros2 run navigation_system gnss_publisher
ros2 run navigation_system websocket_bridge
ros2 run navigation_system bearing_subscriber
ros2 run navigation_system pure_pursuit_navigator
ros2 run navigation_system enhanced_websocket_bridge
ros2 run navigation_system bicycle_kinematic_node
ros2 run navigation_system enhanced_gnss_publisher
ros2 run navigation_system path_visualizer
ros2 run navigation_system controller
ros2 run navigation_system websocket

ros2 topic echo /navigation/heading
ros2 topic echo /navigation/yaw
ros2 topic pub /waypoint/command std_msgs/String "data: 'start_recording'"
ros2 topic pub /waypoint/command std_msgs/String "data: 'stop_recording'"


ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.5}}"
google-chrome ~/ros2_ws/src/navigation_system/web/maptest.html &

python3 -m http.server 5500

ros2 topic echo /cmd_vel
sudo apt remove --purge chromium-browser
sudo apt install chromium-browser

sudo kill -9 8995


-----------------steering----------------------
source install/setup.bash
sudo ip link set can0 up type can bitrate 250000
ros2 run drive_by_wire steering_node
cansend can0 06000001#230C200100000000
-------------------speed------------------------
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
ros2 run drive_by_wire speed_node

keep waypoint
cd /home/inc/project/seemap
python3 waypoint.py

# ดูข้อมูล Position
ros2 topic echo /navigation/xy_position

# ดูข้อมูล Heading
ros2 topic echo /navigation/heading

# ดูข้อมูล Velocity
ros2 topic echo /navigation/velocity



pin imu 

VCC to VIN
GND to GND
SCL to CL
SDA to DA


# ดูข้อมูล สัญลีกษณ์ดาวเทียม
$GPGSV → GPS
$GLGSV → GLONASS
$GAGSV → Galileo
$GBGSV → Compass

cat /dev/ttyACM0 | grep -E "GPGSV|GLGSV|GAGSV|GBGSV"


# run pp controller
cd /home/inc/ros2_ws
  source /opt/ros/foxy/setup.bash
  colcon build --packages-select navigation_system
  source install/setup.bash
x
  จากนั้นเปิด terminal รัน GNSS odom node:

  ros2 run navigation_system imu_node --ros-args -p ports:="['/dev/ttyACM1']"

  ros2 run navigation_system enhanced_gnss_publisher


  รัน Pure Pursuit controller:

  source /opt/ros/foxy/setup.bash
  source /home/inc/ros2_ws/install/setup.bash
  ros2 run navigation_system purepursuit --ros-args -p rtk_no_fix_timeout:=2.0

  ros2 run navigation_system purepursuit --ros-args -p enable_output_log:=true -p output_log_dir:=/
  home/inc/ros2_ws/output_logs -p rtk_no_fix_timeout:=2.0 -p rtk_float_timeout:=2.0 -p
  float_speed_limit:=0.3 -p yaw_hold_steering_gain:=0.8 -p yaw_hold_max_steering_deg:=12.0


# เก็บlogs
ros2 run navigation_system purepursuit --ros-args -p enable_output_log:=true -p output_log_dir:=/home/inc/ros2_ws/output_logs -p rtk_no_fix_timeout:=2.0


ros2 run navigation_system purepersuitnewver --ros-args -p enable_output_log:=true -p output_log_dir:=/home/inc/ros2_ws/output_logs -p rtk_no_fix_timeout:=2.0




  เช็คว่าข้อมูลเข้าครบ:

  ros2 topic echo /imu/yaw
  ros2 topic echo /odom
  ros2 topic echo /cmd_vel

# Rviz show
  source /home/inc/ros2_ws/install/setup.bash
  ROS_LOG_DIR=/tmp/ros2_logs ros2 launch navigation_system controller.launch.py

# No Rviz show
  ROS_LOG_DIR=/tmp/ros2_logs ros2 launch navigation_system controller.launch.py rviz:=false

# Configure_f9r

  python3 configure_f9r.py



 สิ่งที่ใส่ในเวอร์ชันใหม่:

  - FLOAT ไม่ใช้ bicycle model แบบเดิมแล้ว
  - ใช้ adaptive GNSS filter:
      - predict จาก ublox_speed + ublox_yaw
      - correct ด้วยตำแหน่ง ublox ตอน FLOAT แบบถ่วงน้ำหนัก
      - clamp การกระโดดของตำแหน่ง
      - ใช้ path constraint ช่วยดึง estimate กลับเข้าแนว waypoint
  - ตอน FLOAT/NO_FIX ใช้ Stanley Controller ถ้าเปิด stanley_in_degraded_mode=True
  - FIX ยังใช้ตำแหน่ง ublox ตรง ๆ เหมือนเดิม
  - เปลี่ยน node name เป็น pure_pursuit_newver_controller เพื่อไม่ชนกับตัวเดิม

  จุดสำคัญสำหรับ tuning อยู่แถว purepersuitnewver.py:849:

  float_alpha_min
  float_alpha_max
  float_lateral_correction_limit
  enable_path_constraint
  stanley_gain
  stanley_softening_speed

  ตรวจแล้วด้วย:

  python3 -m py_compile purepersuitnewver.py

  ผ่าน ไม่มี syntax error

  ยังไม่ได้รันกับ ROS จริงหรือ bag จริง ดังนั้นแนะนำให้ทดสอบที่ความเร็วต่ำก่อน โดยเฉพาะ stanley_gain, path_constraint_gain, และ
  float_lateral_correction_limit เพราะ 3 ตัวนี้มีผลกับอาการหักไว/ส่ายมากที่สุด.














รันแบบแยก terminal zoo

  Terminal 1: build/source

  cd /home/inc/ros2_ws
  source /opt/ros/foxy/setup.bash
  colcon build --packages-select navigation_system
  source install/setup.bash

  Terminal 2: อ่าน u-blox แล้ว publish /navigation/gnss

  cd /home/inc/ros2_ws
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 run navigation_system gnss_publisher

  ตัวนี้ใช้ serial /dev/ttyACM0 จากไฟล์ gnss_publisher.py

  Terminal 3: เปิด WebSocket ที่หน้าเว็บใช้

  cd /home/inc/ros2_ws
  source /opt/ros/foxy/setup.bash
  source install/setup.bash
  python3 src/navigation_system/navigation_system/ublox_destination_bridge.py

  หน้านี้ต่อไปที่:

  ws://localhost:5055

  Terminal 4: เปิดเว็บ

  cd /home/inc/ros2_ws
  python3 -m http.server 8080 --directory src/navigation_system/web

  แล้วเปิด browser:

  http://localhost:8080/mapzoo_ublox.html

  ถ้าใช้ ros2 launch navigation_system navigation_system.launch.py ได้เหมือนกันบางส่วน แต่มันยังไม่ได้เปิด
  ublox_destination_bridge.py พอร์ต 5055 ให้หน้า mapzoo_ublox.html ดังนั้นยังต้องรัน bridge เพิ่มเองตาม Terminal 3

  ตรวจ topic:

  ros2 topic echo /navigation/gnss
  ros2 topic echo /cmd_vel

  ถ้า u-blox ต่อไม่ได้ ให้เช็กพอร์ต:

  ls /dev/ttyACM*

  และ permission:

  sudo usermod -a -G dialout $USER

  แล้ว logout/login ใหม่ หรือ reboot.
