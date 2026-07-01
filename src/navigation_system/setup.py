# from setuptools import find_packages, setup
# import os
# import glob

# package_name = 'navigation_system'

# setup(
#     name=package_name,
#     version='0.0.1',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#         ('share/' + package_name + '/web', glob.glob('web/**/*', recursive=True)),  # รวมไฟล์ทั้งหมดในโฟลเดอร์ web และ subdirectories
#         ('share/' + package_name + '/launch', glob.glob('launch/*')),  # รวบรวมไฟล์ทั้งหมดในโฟลเดอร์ launch
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='Tapanasamon',  
#     maintainer_email='tapanasamontaladon@gmail.com', 
#     description='ระบบนำทางด้วย GNSS และอินเตอร์เฟซเว็บ',
#     license='TODO: License declaration',  
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#             'gnss_publisher = navigation_system.gnss_publisher:main',
#             'bearing_subscriber = navigation_system.bearing_subscriber:main',
#         ],
#     },
# )


# from setuptools import find_packages, setup
# import os
# from glob import glob

# package_name = 'navigation_system'

# setup(
#     name=package_name,
#     version='0.0.1',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#         # Include launch files
#         (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
#         # Include web files - use recursive glob to include all files
#         (os.path.join('share', package_name, 'web'), glob('web/*.*')),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='Tapanasamon',
#     maintainer_email='tapanasamontaladon@gmail.com',
#     description='ระบบนำทางด้วย GNSS และอินเตอร์เฟซเว็บ',
#     license='TODO: License declaration',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#             'pure_pursuit_navigator = navigation_system.pure_pursuit_navigator:main',
#             'gnss_publisher = navigation_system.gnss_publisher:main',
#             'websocket_bridge = navigation_system.websocket_bridge:main',
#             'bearing_subscriber = navigation_system.bearing_subscriber:main',
#             'bicycle_kinematic_node = navigation_system.bicycle_kinematic_node:main',
#         ],
#     },
# )



from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'navigation_system'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include RViz configs
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        # Include web files - use recursive glob to include all files
        (os.path.join('share', package_name, 'web'), glob('web/*.*')),
        (os.path.join('share', package_name, 'logs6'), glob('logs6/*.csv')),
        (os.path.join('share', package_name, 'logs12'), glob('logs12/*.csv')),
    ],
    install_requires=[
        'setuptools',
        'pyproj',  # เพิ่มสำหรับการแปลงพิกัด UTM
        'geographiclib',  # สำหรับการคำนวณ geodetic
        'websockets',  # สำหรับ WebSocket server
        'pynmea2',  # สำหรับ parsing NMEA messages
        'pyserial',  # สำหรับการเชื่อมต่อ serial port
        'numpy',  # สำหรับการคำนวณทางคณิตศาสตร์
        'python-can',
    ],
    zip_safe=True,
    maintainer='Tapanasamon',
    maintainer_email='tapanasamontaladon@gmail.com',
    description='ระบบนำทางด้วย GNSS และอินเตอร์เฟซเว็บ พร้อมการแปลงพิกัด XY',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # เก็บ entry points เดิมไว้
            'pure_pursuit_navigator = navigation_system.pure_pursuit_navigator:main',
            'gnss_publisher = navigation_system.gnss_publisher:main',
            'websocket_bridge = navigation_system.websocket_bridge:main',
            'bearing_subscriber = navigation_system.bearing_subscriber:main',
            'bicycle_kinematic_node = navigation_system.bicycle_kinematic_node:main',
            
            # เพิ่ม entry points สำหรับไฟล์ใหม่
            'enhanced_gnss_publisher = navigation_system.enhanced_gnss_publisher:main',
            'enhanced_websocket_bridge = navigation_system.enhanced_websocket_bridge:main',
            'enhanced_websocket_bridgenow = navigation_system.enhanced_websocket_bridgenow:main',
            'enhanced_gnss_publishernow = navigation_system.enhanced_gnss_publishernow:main',
            'controller = navigation_system.controller:main',
            'websocket = navigation_system.websocket:main',
            'path_visualizer = navigation_system.path_visualizer:main',
            'teststeering = navigation_system.teststeering:main',
            'purepursuit = navigation_system.purepursuit:main',
            'purepersuitnewver = navigation_system.purepersuitnewver:main',
            'imu = navigation_system.imu:main',
            'imu_node = navigation_system.imu:main',
            'steering_feedback_node = navigation_system.steering_feedback_node:main',
            'keepwaypointnew = navigation_system.keepwaypointnew:main',
        ],
    },
)
