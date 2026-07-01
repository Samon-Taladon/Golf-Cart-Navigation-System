#!/usr/bin/env python3
import asyncio
import websockets
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class WebSocketBridge(Node):
    def __init__(self):
        super().__init__('websocket_bridge')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/web/selected_destination', 10)
        self.get_logger().info("🛰️ WebSocketBridge node started.")

    async def handler(self, websocket, path):
        async for message in websocket:
            try:
                data = json.loads(message)
                if 'lat' in data and 'lon' in data:
                    msg = Float64MultiArray()
                    msg.data = [float(data['lat']), float(data['lon'])]
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"✅ Destination received via WebSocket: {msg.data}")
                else:
                    self.get_logger().warn("⚠️ Invalid data received via WebSocket.")
            except Exception as e:
                self.get_logger().error(f"❌ Error processing WebSocket message: {e}")


def main():
    rclpy.init()
    node = WebSocketBridge()

    loop = asyncio.get_event_loop()
    start_server = websockets.serve(node.handler, "0.0.0.0", 8765)  # port 8765 for WebSocket

    loop.run_until_complete(start_server)
    node.get_logger().info("🌐 WebSocket server listening on ws://0.0.0.0:8765")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 WebSocketBridge stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
