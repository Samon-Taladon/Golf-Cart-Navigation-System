#!/usr/bin/env python3
import asyncio
import json
import math
import signal
import socket
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import websockets


class UbloxDestinationBridge(Node):
    """Small WebSocket bridge for one lat/lon destination from the zoo web page."""

    def __init__(self):
        super().__init__('ublox_destination_bridge')

        self.declare_parameter('gnss_topic', '/navigation/gnss')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('websocket_host', '0.0.0.0')
        self.declare_parameter('websocket_port', 5055)
        self.declare_parameter('arrival_threshold_m', 10.0)
        self.declare_parameter('drive_speed', 1.0)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.gnss_topic = self.get_parameter('gnss_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.websocket_host = self.get_parameter('websocket_host').value
        self.websocket_port = int(self.get_parameter('websocket_port').value)
        self.arrival_threshold_m = float(self.get_parameter('arrival_threshold_m').value)
        self.drive_speed = float(self.get_parameter('drive_speed').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.gnss_sub = self.create_subscription(
            NavSatFix, self.gnss_topic, self.gnss_callback, 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.current_position = None
        self.destination = None
        self.navigation_active = False
        self.arrived = False
        self.last_distance_m = None
        self.last_status_sent_at = 0.0

        self.connections = set()
        self.websocket_loop = None
        self.websocket_server = None
        self.shutdown_requested = False

        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        self.server_thread = threading.Thread(
            target=self.start_websocket_server, daemon=True
        )
        self.server_thread.start()

        self.create_timer(1.0 / self.publish_rate_hz, self.control_loop)

        self.get_logger().info(
            'u-blox destination bridge started: '
            f'{self.gnss_topic} -> {self.cmd_vel_topic}, '
            f'ws://localhost:{self.websocket_port}, '
            f'threshold={self.arrival_threshold_m:.2f}m'
        )

    def signal_handler(self, signum, frame):
        self.get_logger().info(f'Received signal {signum}, shutting down')
        self.shutdown_requested = True
        self.publish_stop()
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
                f'WebSocket server running on ws://{self.websocket_host}:{self.websocket_port}'
            )
            self.websocket_loop.run_forever()
        except Exception as exc:
            self.get_logger().error(f'Failed to start WebSocket server: {exc}')

    def stop_websocket_server(self):
        if not self.websocket_loop or not self.websocket_server:
            return
        asyncio.run_coroutine_threadsafe(
            self.close_websocket_server(), self.websocket_loop
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
        await self.send_json(websocket, self.build_status('connected'))

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

        if msg_type in ('set_destination', 'destination'):
            if not self.set_destination_from_payload(data):
                await self.send_json(
                    websocket,
                    {'type': 'error', 'message': 'destination_requires_lat_lon'},
                )
                return
            await self.broadcast_status('destination_set')
            return

        if msg_type in ('start_navigation', 'start'):
            if not self.destination:
                await self.send_json(
                    websocket,
                    {'type': 'error', 'message': 'set_destination_first'},
                )
                return
            self.arrived = False
            self.navigation_active = True
            self.publish_drive()
            await self.broadcast_status('navigation_started')
            return

        if msg_type in ('stop_navigation', 'stop'):
            self.navigation_active = False
            self.arrived = False
            self.publish_stop()
            await self.broadcast_status('navigation_stopped')
            return

        await self.send_json(websocket, {'type': 'error', 'message': 'unknown_command'})

    def set_destination_from_payload(self, data):
        try:
            lat = float(data['lat'])
            lon = float(data['lon'])
        except (KeyError, TypeError, ValueError):
            return False

        if not self.is_valid_coordinate(lat, lon):
            return False

        self.destination = {
            'name': str(data.get('name') or data.get('destination') or 'destination'),
            'lat': lat,
            'lon': lon,
        }
        self.navigation_active = False
        self.arrived = False
        self.last_distance_m = self.calculate_current_distance()
        self.publish_stop()
        self.get_logger().info(
            f"Destination set: {self.destination['name']} "
            f"lat={lat:.7f}, lon={lon:.7f}"
        )
        return True

    def gnss_callback(self, msg):
        if not self.is_valid_coordinate(msg.latitude, msg.longitude):
            return

        self.current_position = {
            'lat': float(msg.latitude),
            'lon': float(msg.longitude),
            'stamp': time.time(),
            'status': int(msg.status.status),
            'service': int(msg.status.service),
        }
        self.last_distance_m = self.calculate_current_distance()

        now = time.time()
        if now - self.last_status_sent_at >= 0.5:
            self.last_status_sent_at = now
            self.schedule_broadcast_status('gnss_update')

    def control_loop(self):
        if not self.navigation_active or not self.current_position or not self.destination:
            return

        distance_m = self.calculate_current_distance()
        self.last_distance_m = distance_m

        if distance_m is not None and distance_m <= self.arrival_threshold_m:
            self.navigation_active = False
            self.arrived = True
            self.publish_stop()
            self.get_logger().info(f'Arrived at destination: distance={distance_m:.2f}m')
            self.schedule_broadcast_status('arrived')
            return

        self.publish_drive()

    def publish_drive(self):
        twist = Twist()
        twist.linear.x = self.drive_speed
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def publish_stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def calculate_current_distance(self):
        if not self.current_position or not self.destination:
            return None
        return self.haversine_m(
            self.current_position['lat'],
            self.current_position['lon'],
            self.destination['lat'],
            self.destination['lon'],
        )

    @staticmethod
    def haversine_m(lat1, lon1, lat2, lon2):
        earth_radius_m = 6371000.0
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        d_phi = math.radians(lat2 - lat1)
        d_lambda = math.radians(lon2 - lon1)

        a = (
            math.sin(d_phi / 2.0) ** 2
            + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2.0) ** 2
        )
        c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
        return earth_radius_m * c

    @staticmethod
    def is_valid_coordinate(lat, lon):
        return (
            lat is not None
            and lon is not None
            and -90.0 <= float(lat) <= 90.0
            and -180.0 <= float(lon) <= 180.0
            and not (float(lat) == 0.0 and float(lon) == 0.0)
        )

    def build_status(self, event):
        return {
            'type': 'status',
            'event': event,
            'navigation_active': self.navigation_active,
            'arrived': self.arrived,
            'arrival_threshold_m': self.arrival_threshold_m,
            'drive_speed': self.drive_speed,
            'current_position': self.current_position,
            'destination': self.destination,
            'distance_m': self.last_distance_m,
            'cmd_linear_x': 1.0 if self.navigation_active else 0.0,
            'websocket_port': self.websocket_port,
        }

    def schedule_broadcast_status(self, event):
        if not self.websocket_loop or not self.connections:
            return
        asyncio.run_coroutine_threadsafe(
            self.broadcast_status(event), self.websocket_loop
        )

    async def broadcast_status(self, event):
        await self.broadcast_json(self.build_status(event))

    async def send_json(self, websocket, payload):
        await websocket.send(json.dumps(payload, ensure_ascii=False))

    async def broadcast_json(self, payload):
        if not self.connections:
            return

        disconnected = set()
        message = json.dumps(payload, ensure_ascii=False)
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
    node = UbloxDestinationBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.navigation_active = False
        node.publish_stop()
        node.stop_websocket_server()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
