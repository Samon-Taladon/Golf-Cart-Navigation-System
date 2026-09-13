#!/usr/bin/env python3
"""WebSocket bridge for mapzoo_ublox_forweb.html.

The bridge only forwards web commands to ROS and broadcasts controller status.
It never publishes /cmd_vel, so purepursuit_forweb remains the single driving
authority.
"""

import asyncio
import json
import signal
import socket
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import websockets


class WebNavigationBridgeForWeb(Node):
    def __init__(self):
        super().__init__('web_navigation_bridge_forweb')

        self.declare_parameter('websocket_host', '0.0.0.0')
        self.declare_parameter('websocket_port', 5055)
        self.declare_parameter('web_command_topic', '/web/navigation_command')
        self.declare_parameter('web_status_topic', '/web/navigation_status')

        self.websocket_host = self.get_parameter('websocket_host').value
        self.websocket_port = int(self.get_parameter('websocket_port').value)
        self.web_command_topic = self.get_parameter('web_command_topic').value
        self.web_status_topic = self.get_parameter('web_status_topic').value

        self.command_pub = self.create_publisher(String, self.web_command_topic, 10)
        self.status_sub = self.create_subscription(
            String,
            self.web_status_topic,
            self.status_callback,
            10,
        )

        self.latest_status = self.default_status('starting')
        self.connections = set()
        self.websocket_loop = None
        self.websocket_server = None
        self.last_status_at = 0.0

        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        self.server_thread = threading.Thread(
            target=self.start_websocket_server,
            daemon=True,
        )
        self.server_thread.start()

        self.get_logger().info(
            f'Web bridge for web started on ws://{self.websocket_host}:'
            f'{self.websocket_port}; commands -> {self.web_command_topic}'
        )

    def signal_handler(self, signum, frame):
        self.get_logger().info(f'Received signal {signum}, shutting down')
        self.publish_command({'type': 'stop_navigation'})
        self.stop_websocket_server()
        sys.exit(0)

    def start_websocket_server(self):
        self.websocket_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.websocket_loop)

        port = self.find_available_port(self.websocket_port)
        if port != self.websocket_port:
            self.get_logger().warn(
                f'Port {self.websocket_port} is busy, using {port} instead'
            )
            self.websocket_port = port

        try:
            server = websockets.serve(
                self.websocket_handler,
                self.websocket_host,
                self.websocket_port,
                ping_interval=30,
                ping_timeout=10,
            )
            self.websocket_server = self.websocket_loop.run_until_complete(server)
            self.get_logger().info(
                f'WebSocket server running on ws://{self.websocket_host}:'
                f'{self.websocket_port}'
            )
            self.websocket_loop.run_forever()
        except Exception as exc:
            self.get_logger().error(f'Failed to start WebSocket server: {exc}')

    def stop_websocket_server(self):
        if not self.websocket_loop or not self.websocket_server:
            return
        asyncio.run_coroutine_threadsafe(
            self.close_websocket_server(),
            self.websocket_loop,
        )

    async def close_websocket_server(self):
        self.websocket_server.close()
        await self.websocket_server.wait_closed()
        self.websocket_loop.stop()

    def find_available_port(self, start_port, max_attempts=20):
        for port in range(start_port, start_port + max_attempts):
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                try:
                    sock.bind((self.websocket_host, port))
                    return port
                except OSError:
                    continue
        return start_port

    async def websocket_handler(self, websocket, path=None):
        del path
        self.connections.add(websocket)
        await self.send_json(websocket, self.status_with_port('connected'))

        try:
            async for message in websocket:
                await self.handle_web_message(websocket, message)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.connections.discard(websocket)

    async def handle_web_message(self, websocket, message):
        try:
            data = json.loads(message)
        except json.JSONDecodeError:
            await self.send_json(websocket, {'type': 'error', 'message': 'invalid_json'})
            return

        msg_type = data.get('type') or data.get('command')
        if msg_type in (
            'set_destination',
            'destination',
            'start_navigation',
            'start',
            'stop_navigation',
            'stop',
        ):
            self.publish_command(data)
            await self.broadcast_json(self.status_with_port(f'web_{msg_type}'))
            return

        await self.send_json(websocket, {'type': 'error', 'message': 'unknown_command'})

    def publish_command(self, payload):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.command_pub.publish(msg)
        self.get_logger().info(f"Web command forwarded: {payload.get('type')}")

    def status_callback(self, msg):
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'Invalid status JSON: {msg.data!r}')
            return

        status.setdefault('type', 'status')
        status['websocket_port'] = self.websocket_port
        self.latest_status = status
        self.last_status_at = time.time()
        self.schedule_broadcast(status)

    def default_status(self, event):
        return {
            'type': 'status',
            'event': event,
            'navigation_active': False,
            'arrived': False,
            'distance_m': None,
            'websocket_port': self.websocket_port,
        }

    def status_with_port(self, event):
        status = dict(self.latest_status or self.default_status(event))
        status['type'] = 'status'
        status['event'] = event
        status['websocket_port'] = self.websocket_port
        return status

    def schedule_broadcast(self, payload):
        if not self.websocket_loop or not self.connections:
            return
        asyncio.run_coroutine_threadsafe(
            self.broadcast_json(payload),
            self.websocket_loop,
        )

    async def send_json(self, websocket, payload):
        await websocket.send(json.dumps(payload, ensure_ascii=False))

    async def broadcast_json(self, payload):
        if not self.connections:
            return

        message = json.dumps(payload, ensure_ascii=False)
        disconnected = set()
        for websocket in list(self.connections):
            try:
                await websocket.send(message)
            except websockets.exceptions.ConnectionClosed:
                disconnected.add(websocket)
            except Exception as exc:
                self.get_logger().warn(f'WebSocket send failed: {exc}')
                disconnected.add(websocket)

        self.connections.difference_update(disconnected)


def main(args=None):
    rclpy.init(args=args)
    node = WebNavigationBridgeForWeb()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_command({'type': 'stop_navigation'})
        node.stop_websocket_server()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
