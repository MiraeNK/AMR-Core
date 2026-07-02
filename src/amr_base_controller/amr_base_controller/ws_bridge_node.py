#!/usr/bin/env python3
"""
ws_bridge_node.py — WebSocket ↔ ROS2 Bridge
============================================
Menjalankan WebSocket server di port 9090.

Perubahan dari versi sebelumnya:
- _on_status sekarang juga mem-broadcast robot_pose dari data /agv/status
  sehingga digital twin di UI bergerak tanpa perlu AMCL.
- /amcl_pose tetap di-subscribe untuk kompatibilitas saat AMCL aktif.

Protokol UI → Bridge:
  { "type": "set_path",    "points": [{x, y}, ...] }    → /agv/path (nav_msgs/Path)
  { "type": "rerun" }                                    → /agv/cmd "rerun"
  { "type": "pose_estimate", x, y, yaw }                → /initialpose
  { "cmd": "pause" }                                     → /agv/cmd "pause"
  { "cmd": "resume" }                                    → /agv/cmd "resume"
  { "cmd": "stop" }                                      → /agv/cmd "stop"
  { "cmd": "set_speed", "value": float }                 → /agv/cmd "set_speed:0.40"

Protokol Bridge → UI:
  { "state": ..., "x": ..., "y": ..., "yaw_deg": ...,
    "waypoint": ..., "total_wp": ..., "obstacle": ... }  (from /agv/status)
  { "type": "robot_pose", "x": ..., "y": ..., "yaw_deg": ... }
  { "type": "scan", "points": [{x, y}, ...] }
  { "type": "path_ack", "data": { "status": "ok", "message": "..." } }
"""

import asyncio
import json
import math
import threading

import rclpy
import websockets
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

WS_HOST        = '0.0.0.0'
WS_PORT        = 9090
SCAN_DOWNSAMPLE = 4

# Offset mounting lidar relatif base_footprint (lidar.xacro: xyz="0.25 0 0.175")
LASER_OFFSET_X  = 0.25
LASER_OFFSET_Y  = 0.0
LASER_MOUNT_YAW = 0.0


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class WsBridgeNode(Node):

    def __init__(self):
        super().__init__('ws_bridge_node')

        self._clients: set = set()
        self._loop: asyncio.AbstractEventLoop | None = None

        path_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # ── Publishers ────────────────────────────────────────────────
        self._path_pub = self.create_publisher(Path,   '/agv/path', path_qos)
        self._cmd_pub  = self.create_publisher(String, '/agv/cmd',  10)
        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # ── Subscribers ───────────────────────────────────────────────
        self.create_subscription(String,    '/agv/status', self._on_status, 10)
        self.create_subscription(LaserScan, '/scan',       self._on_scan,   10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, 10)

        self.get_logger().info(
            f'ws_bridge_node ready — WebSocket on ws://*:{WS_PORT}')

    # ================================================================
    # ROS CALLBACKS
    # ================================================================

    def _broadcast(self, payload: dict):
        if not self._clients or self._loop is None:
            return
        msg = json.dumps(payload)
        asyncio.run_coroutine_threadsafe(
            self._async_broadcast(msg), self._loop)

    async def _async_broadcast(self, msg: str):
        dead = set()
        for ws in list(self._clients):
            try:
                await ws.send(msg)
            except Exception:
                dead.add(ws)
        self._clients -= dead

    def _on_status(self, msg: String):
        """
        Terima JSON dari agv_line_follower_node atau path_follower_node.
        Broadcast langsung ke UI sebagai status update.
        Sekaligus forward posisi sebagai robot_pose agar digital twin
        bergerak tanpa perlu AMCL.
        """
        try:
            data = json.loads(msg.data)

            # Broadcast status ke UI (state, waypoint, obstacle, CTE, dll)
            self._broadcast(data)

            # Forward posisi sebagai robot_pose → digital twin bergerak
            if all(k in data for k in ('x', 'y', 'yaw_deg')):
                self._broadcast({
                    'type'    : 'robot_pose',
                    'x'       : data['x'],
                    'y'       : data['y'],
                    'yaw_deg' : data['yaw_deg'],
                })

        except Exception as e:
            self.get_logger().warn(f'Status parse error: {e}')

    def _on_scan(self, msg: LaserScan):
        """
        Konversi LaserScan ke robot-frame (base_footprint) XY points.
        UI mentransformasi ke map-frame menggunakan pose robot terkini.
        """
        pts    = []
        angle  = msg.angle_min
        cos_m  = math.cos(LASER_MOUNT_YAW)
        sin_m  = math.sin(LASER_MOUNT_YAW)

        for i, r in enumerate(msg.ranges):
            if i % SCAN_DOWNSAMPLE == 0:
                if msg.range_min < r < msg.range_max:
                    lx = r * math.cos(angle)
                    ly = r * math.sin(angle)
                    bx = LASER_OFFSET_X + (lx * cos_m - ly * sin_m)
                    by = LASER_OFFSET_Y + (lx * sin_m + ly * cos_m)
                    pts.append({'x': round(bx, 3), 'y': round(by, 3)})
            angle += msg.angle_increment

        self._broadcast({'type': 'scan', 'points': pts})

    def _on_amcl(self, msg: PoseWithCovarianceStamped):
        """
        Dipakai saat AMCL aktif (agv_navigation.launch.py).
        Di agv_virtual_line.launch.py AMCL tidak jalan, tapi subscriber
        ini tetap ada untuk kompatibilitas.
        """
        p   = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self._broadcast({
            'type'    : 'robot_pose',
            'x'       : round(p.x, 4),
            'y'       : round(p.y, 4),
            'yaw_deg' : round(math.degrees(yaw), 2),
        })

    # ================================================================
    # INCOMING WS MESSAGES
    # ================================================================

    async def _handle_message(self, ws, raw: str):
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            await ws.send(json.dumps({
                'type': 'path_ack',
                'data': {'status': 'err', 'message': 'JSON parse error'},
            }))
            return

        msg_type = data.get('type')
        cmd      = data.get('cmd')

        if msg_type == 'set_path':
            pts = data.get('points', [])
            if len(pts) < 2:
                await ws.send(json.dumps({'type': 'path_ack', 'data': {
                    'status': 'err', 'message': 'Minimal 2 waypoint'}}))
                return

            path_msg             = Path()
            path_msg.header.frame_id = 'map'
            path_msg.header.stamp    = self.get_clock().now().to_msg()
            for p in pts:
                ps            = PoseStamped()
                ps.header     = path_msg.header
                ps.pose.position.x    = float(p['x'])
                ps.pose.position.y    = float(p['y'])
                ps.pose.orientation.w = 1.0
                path_msg.poses.append(ps)

            self._path_pub.publish(path_msg)
            self.get_logger().info(f'Path published: {len(pts)} waypoints')
            await ws.send(json.dumps({'type': 'path_ack', 'data': {
                'status': 'ok',
                'message': f'{len(pts)} waypoint dikirim'}}))

        elif msg_type == 'rerun':
            m = String(); m.data = 'rerun'
            self._cmd_pub.publish(m)

        elif msg_type == 'pose_estimate':
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.frame_id = 'map'
            pose_msg.header.stamp    = self.get_clock().now().to_msg()
            pose_msg.pose.pose.position.x    = float(data['x'])
            pose_msg.pose.pose.position.y    = float(data['y'])
            pose_msg.pose.pose.orientation   = yaw_to_quaternion(float(data['yaw']))
            pose_msg.pose.covariance[0]  = 0.25
            pose_msg.pose.covariance[7]  = 0.25
            pose_msg.pose.covariance[35] = 0.0685
            self._pose_pub.publish(pose_msg)
            self.get_logger().info(
                f'Initial pose set: ({data["x"]:.2f}, {data["y"]:.2f}) '
                f'yaw={math.degrees(data["yaw"]):.1f}°')

        elif cmd in ('pause', 'resume', 'stop'):
            m = String(); m.data = cmd
            self._cmd_pub.publish(m)

        elif cmd == 'set_speed':
            val = float(data.get('value', 0.4))
            m   = String(); m.data = f'set_speed:{val:.3f}'
            self._cmd_pub.publish(m)

        else:
            self.get_logger().warn(f'Unknown message: {data}')

    # ================================================================
    # WEBSOCKET SERVER
    # ================================================================

    async def _ws_handler(self, ws):
        self._clients.add(ws)
        self.get_logger().info(f'WS client connected: {ws.remote_address}')
        try:
            async for raw in ws:
                await self._handle_message(ws, raw)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self._clients.discard(ws)
            self.get_logger().info(
                f'WS client disconnected: {ws.remote_address}')

    async def _run_server(self):
        self._loop = asyncio.get_event_loop()
        async with websockets.serve(self._ws_handler, WS_HOST, WS_PORT):
            self.get_logger().info(
                f'WebSocket server listening on {WS_HOST}:{WS_PORT}')
            await asyncio.Future()

    def start_ws_server(self):
        asyncio.run(self._run_server())


def main(args=None):
    rclpy.init(args=args)
    node = WsBridgeNode()

    ws_thread = threading.Thread(
        target=node.start_ws_server, daemon=True)
    ws_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
