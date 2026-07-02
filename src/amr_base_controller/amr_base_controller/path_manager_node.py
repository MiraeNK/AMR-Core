#!/usr/bin/env python3
"""
path_manager_node.py — AGV Path Manager
========================================
Maintainer : Hafizh Husaini <miraenk7@gmail.com>

Menerima jalur dari ws_bridge_node (via /agv_path_raw JSON),
mengkonversinya ke nav_msgs/Path, dan mem-publish ke /agv_path.
Juga menyimpan jalur terakhir agar robot bisa dijalankan ulang
tanpa harus kirim jalur baru dari UI.

Topic:
  /agv_path_raw  (std_msgs/String JSON) ← dari ws_bridge_node
  /agv_path      (nav_msgs/Path)        → ke path_follower_node
  /agv_command   (std_msgs/String JSON) ← dari ws_bridge_node
"""

import json
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from builtin_interfaces.msg import Time as RosTime


class PathManagerNode(Node):

    def __init__(self):
        super().__init__('path_manager_node')

        qos_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1)

        self._sub_raw = self.create_subscription(
            String, '/agv_path_raw', self._raw_callback, 10)

        self._pub_path = self.create_publisher(Path, '/agv_path', qos_latched)
        self._pub_ack  = self.create_publisher(String, '/agv_path_ack', 10)

        self._last_path_points = []

        self.get_logger().info('[ path_manager ] Siap menerima jalur dari ws_bridge.')

    def _raw_callback(self, msg: String):
        """
        Format JSON yang diterima dari UI via ws_bridge:
        {
          "type": "set_path",
          "points": [
            {"x": 1.0, "y": 2.0},
            {"x": 3.5, "y": 2.0},
            ...
          ]
        }
        Atau type "rerun" untuk jalankan jalur terakhir lagi.
        """
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'[ path_manager ] JSON parse error: {e}')
            return

        msg_type = data.get('type', '')

        if msg_type == 'set_path':
            points = data.get('points', [])
            if len(points) < 2:
                self.get_logger().warn(
                    '[ path_manager ] Jalur butuh minimal 2 titik.')
                self._send_ack('error', 'Minimal 2 titik diperlukan')
                return
            self._last_path_points = points
            self._publish_path(points)
            self._send_ack('ok', f'{len(points)} titik diterima')

        elif msg_type == 'rerun':
            if not self._last_path_points:
                self._send_ack('error', 'Tidak ada jalur tersimpan')
                return
            self._publish_path(self._last_path_points)
            self._send_ack('ok', 'Jalur terakhir dijalankan ulang')

        elif msg_type == 'clear':
            self._last_path_points = []
            # Publish jalur kosong untuk stop follower
            self._publish_path([])
            self._send_ack('ok', 'Jalur dihapus')

        else:
            self.get_logger().warn(f'[ path_manager ] Tipe tidak dikenal: {msg_type}')

    def _publish_path(self, points):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp    = self.get_clock().now().to_msg()

        for pt in points:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp    = path_msg.header.stamp
            pose.pose.position.x = float(pt['x'])
            pose.pose.position.y = float(pt['y'])
            pose.pose.position.z = 0.0
            # Orientasi default (tidak dipakai Pure Pursuit tapi wajib valid)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self._pub_path.publish(path_msg)
        if points:
            self.get_logger().info(
                f'[ path_manager ] Published {len(points)} titik ke /agv_path')

    def _send_ack(self, status, message):
        ack = String()
        ack.data = json.dumps({"status": status, "message": message})
        self._pub_ack.publish(ack)


def main(args=None):
    rclpy.init(args=args)
    node = PathManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
