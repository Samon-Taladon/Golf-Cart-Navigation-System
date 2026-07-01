# #!/usr/bin/env python3

# import asyncio
# import websockets
# import json
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
# from std_msgs.msg import Float32, String
# import threading

# class WebSocketBridge(Node):
#     def __init__(self):
#         super().__init__('websocket_bridge')
        
#         # ROS2 subscribers
#         self.gnss_sub = self.create_subscription(
#             NavSatFix,
#             '/navigation/gnss',
#             self.gnss_callback,
#             10)
        
#         # ROS2 publishers
#         self.bearing_pub = self.create_publisher(
#             Float32,
#             '/navigation/bearing',
#             10)
        
#         self.destination_pub = self.create_publisher(
#             String,
#             '/navigation/destination',
#             10)
        
#         # WebSocket server
#         self.connections = set()
#         self.latest_gnss = None
        
#         # Start WebSocket server in a separate thread
#         threading.Thread(target=self.start_websocket_server, daemon=True).start()
        
#         self.get_logger().info('WebSocket bridge initialized')
    
#     def start_websocket_server(self):
#         loop = asyncio.new_event_loop()
#         asyncio.set_event_loop(loop)
        
#         start_server = websockets.serve(self.websocket_handler, "0.0.0.0", 5000)
#         loop.run_until_complete(start_server)
#         self.get_logger().info('WebSocket server running on ws://0.0.0.0:5000')
#         loop.run_forever()
    
#     async def websocket_handler(self, websocket, path):
#         # Register new connection
#         self.connections.add(websocket)
#         self.get_logger().info(f'New WebSocket connection: {websocket.remote_address}')
        
#         # Send latest GNSS data if available
#         if self.latest_gnss:
#             await websocket.send(json.dumps(self.latest_gnss))
        
#         try:
#             async for message in websocket:
#                 try:
#                     data = json.loads(message)
                    
#                     # Handle bearing/motor control message (from the web client)
#                     if 'motorHex' in data or 'bearing' in data:
#                         bearing_msg = Float32()
                        
#                         if 'bearing' in data:
#                             bearing_msg.data = float(data['bearing'])
#                         elif 'motorHex' in data:
#                             # Convert hex back to bearing if needed
#                             hex_val = int(data['motorHex'].replace('0x', ''), 16)
#                             if hex_val <= 15:
#                                 bearing_msg.data = (hex_val / 255) * 360
#                             else:
#                                 bearing_msg.data = 360 - ((255 - hex_val) / 255) * 360
                        
#                         self.bearing_pub.publish(bearing_msg)
#                         self.get_logger().info(f'Published bearing: {bearing_msg.data}')
                    
#                     # Handle destination selection
#                     if 'destination' in data:
#                         dest_msg = String()
#                         dest_msg.data = data['destination']
#                         self.destination_pub.publish(dest_msg)
#                         self.get_logger().info(f'Published destination: {dest_msg.data}')
                
#                 except json.JSONDecodeError:
#                     self.get_logger().error(f'Invalid JSON received: {message}')
#                 except Exception as e:
#                     self.get_logger().error(f'Error processing message: {e}')
        
#         finally:
#             # Remove connection when closed
#             self.connections.remove(websocket)
#             self.get_logger().info(f'WebSocket connection closed: {websocket.remote_address}')
    
#     def gnss_callback(self, msg):
#         # Convert NavSatFix message to JSON
#         gnss_data = {
#             'latitude': msg.latitude,
#             'longitude': msg.longitude
#         }
        
#         # Store latest GNSS data
#         self.latest_gnss = gnss_data
        
#         # Send to all connected clients
#         if self.connections:
#             asyncio.run_coroutine_threadsafe(
#                 self.send_to_all(json.dumps(gnss_data)),
#                 asyncio.get_event_loop()
#             )
    
#     async def send_to_all(self, message):
#         # Send message to all connected WebSocket clients
#         disconnected = set()
        
#         for websocket in self.connections:
#             try:
#                 await websocket.send(message)
#             except websockets.exceptions.ConnectionClosed:
#                 disconnected.add(websocket)
        
#         # Remove disconnected clients
#         for websocket in disconnected:
#             self.connections.remove(websocket)

# def main(args=None):
#     rclpy.init(args=args)
#     bridge = WebSocketBridge()
#     rclpy.spin(bridge)
    
#     bridge.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3

#!/usr/bin/env python3

# ---------------------------------------------------------------------------------------

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32, String
# import math

# class BearingSubscriber(Node):
#     def __init__(self):
#         super().__init__('bearing_subscriber')
        
#         # Subscribe to bearing topic
#         self.bearing_sub = self.create_subscription(
#             Float32,
#             '/navigation/bearing',
#             self.bearing_callback,
#             10)
        
#         # Subscribe to destination topic
#         self.destination_sub = self.create_subscription(
#             String,
#             '/navigation/destination',
#             self.destination_callback,
#             10)
        
#         self.current_bearing = None
#         self.current_destination = None
        
#         self.get_logger().info('Bearing Subscriber initialized')
    
#     def bearing_callback(self, msg):
#         self.current_bearing = msg.data
#         self.get_logger().info(f'Received bearing: {self.current_bearing:.2f} degrees')
        
#         # Here you would add code to control your hardware based on the bearing
#         # For example, sending commands to a motor controller
#         self.control_motor(self.current_bearing)
    
#     def destination_callback(self, msg):
#         self.current_destination = msg.data
#         self.get_logger().info(f'New destination set: {self.current_destination}')
    
#     def control_motor(self, bearing):
#         # Example motor control code - replace with your actual hardware control
#         # This is a placeholder function that converts bearing to motor commands
        
#         # Example: Convert bearing to a hex value for motor control
#         # Normalize bearing to 0-255 range for servo or motor control
#         normalized_value = int((bearing % 360) / 360 * 255)
#         hex_value = f"0x{normalized_value:02X}"
        
#         self.get_logger().info(f'Motor control value: {hex_value}')
        
#         # Add your motor control code here
#         # Example: Use serial connection, GPIO, etc. to control motors
#         pass

# def main(args=None):
#     rclpy.init(args=args)
#     bearing_subscriber = BearingSubscriber()
#     rclpy.spin(bearing_subscriber)
    
#     bearing_subscriber.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import math

class BearingSubscriber(Node):
    def __init__(self):
        super().__init__('bearing_subscriber')
        
        # Subscribe to bearing topic
        self.bearing_sub = self.create_subscription(
            Float32,
            '/navigation/bearing',
            self.bearing_callback,
            10)
        
        # Subscribe to destination topic
        self.destination_sub = self.create_subscription(
            String,
            '/navigation/destination',
            self.destination_callback,
            10)
        
        self.current_bearing = None
        self.current_destination = None
        
        self.get_logger().info('Bearing Subscriber initialized')
    
    def bearing_callback(self, msg):
        self.current_bearing = msg.data
        self.get_logger().info(f'Received bearing: {self.current_bearing:.2f} degrees')
        
        # แปลงจากองศาเป็นเรเดียน
        bearing_radian = math.radians(self.current_bearing)
        self.get_logger().info(f'Converted bearing: {bearing_radian:.2f} radians')
        
        # ส่งค่า bearing ที่แปลงแล้วไปควบคุมมอเตอร์
        self.control_motor(bearing_radian)
    
    def destination_callback(self, msg):
        self.current_destination = msg.data
        self.get_logger().info(f'New destination set: {self.current_destination}')
    
    def control_motor(self, bearing_radian):
        # แปลงจากเรเดียนเป็นค่าควบคุมมอเตอร์
        # Normalize bearing to 0-255 range for servo or motor control
        normalized_value = int((bearing_radian % (2 * math.pi)) / (2 * math.pi) * 255)
        hex_value = f"0x{normalized_value:02X}"
        
        self.get_logger().info(f'Motor control value: {hex_value}')
        
        # เพิ่มโค้ดควบคุมมอเตอร์ที่นี่ เช่น ใช้ serial connection, GPIO ฯลฯ
        pass

def main(args=None):
    rclpy.init(args=args)
    bearing_subscriber = BearingSubscriber()
    rclpy.spin(bearing_subscriber)
    
    bearing_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
