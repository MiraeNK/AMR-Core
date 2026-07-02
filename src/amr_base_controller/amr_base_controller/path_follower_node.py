#!/usr/bin/env python3
"""
path_follower_node.py — AGV Waypoint Follower (Odometry-Only)
================================================================
ARSITEKTUR FINAL: navigasi 100% berbasis ODOMETRY dengan P-Controller.
AMCL TIDAK dipakai untuk kontrol sama sekali.
"""

import json
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

# ── Parameter perilaku ──────────────────────────────────────────────────
DEFAULT_CRUISE_SPEED   = 0.40   # m/s — kecepatan maju lurus
DEFAULT_PIVOT_SPEED    = 0.60   # rad/s — kecepatan maksimal saat pivot
DEFAULT_GOAL_TOL       = 0.10   # m — jarak dianggap waypoint tercapai
DEFAULT_YAW_TOL_DEG    = 9.0    # derajat — toleransi heading sebelum berhenti pivot
                                #   DILONGGARKAN dari 4° ke 9°. Toleransi terlalu
                                #   ketat (4°) membuat robot "stuck" pivot pelan tak
                                #   pernah tuntas karena drift odometry beberapa derajat
                                #   sudah cukup membuat yaw_err tak pernah turun < 4°.
DEFAULT_OBS_DIST       = 0.30   # m — stop jika obstacle lebih dekat dari ini
OBS_SECTOR_HALF_DEG    = 20     # ± derajat sektor depan untuk obstacle detection
OBS_MIN_RANGE          = 0.25   # m — abaikan reading < ini (noise body robot)
CTRL_HZ                = 20     # Hz — frekuensi control loop
REPIVOT_THRESHOLD_DEG  = 20.0   # derajat — batas melenceng saat FORWARD sebelum repivot.
                                #   DINAIKKAN dari 12° ke 20°. Harus lebih besar dari
                                #   yaw_tol (9°) dengan margin cukup, kalau tidak robot
                                #   akan bolak-balik PIVOT<->FORWARD (chattering) tepat
                                #   di sekitar ambang toleransi.
FINAL_GOAL_TOL         = 0.20   # m — toleransi LEBIH LONGGAR khusus waypoint terakhir.
                                #   Waypoint terakhir tidak butuh presisi sudut (tidak ada
                                #   segmen lanjutan), jadi cukup "sampai sekitar sini" =
                                #   selesai. Mencegah robot berputar-putar mengejar titik
                                #   presisi yang tak pernah tercapai akibat drift odometry.
NO_PIVOT_ZONE_FACTOR   = 2.0    # kelipatan goal_tol — di dalam zona ini (dekat waypoint),
                                #   PIVOT besar dilarang. Robot hanya boleh maju lurus /
                                #   koreksi halus, supaya tidak "muter di tempat" saat
                                #   sudah dekat target (target_yaw bisa berbalik drastis
                                #   ketika robot melewati titik dari jarak sangat dekat).

class PathFollowerNode(Node):
    def __init__(self):
        super().__init__('path_follower_node')

        # ── Parameter ROS ──
        self.declare_parameter('cruise_speed',      DEFAULT_CRUISE_SPEED)
        self.declare_parameter('pivot_speed',       DEFAULT_PIVOT_SPEED)
        self.declare_parameter('goal_tolerance',    DEFAULT_GOAL_TOL)
        self.declare_parameter('yaw_tolerance_deg', DEFAULT_YAW_TOL_DEG)
        self.declare_parameter('obstacle_dist',     DEFAULT_OBS_DIST)

        self._cruise_speed = self.get_parameter('cruise_speed').value
        self._pivot_speed  = self.get_parameter('pivot_speed').value
        self._goal_tol     = self.get_parameter('goal_tolerance').value
        self._yaw_tol      = math.radians(self.get_parameter('yaw_tolerance_deg').value)
        self._obs_dist     = self.get_parameter('obstacle_dist').value

        # ── State navigasi ──
        self._state      = 'IDLE'
        self._nav_phase  = 'PIVOT'
        self._waypoints  = []
        self._last_path  = []
        self._wp_idx     = 0

        # ── Pose ODOMETRY ──
        self._odom_x   = 0.0
        self._odom_y   = 0.0
        self._odom_yaw = 0.0
        self._odom_received = False
        self._obstacle = False

        self._last_log_time_odom = 0.0
        self._last_log_time_obs = 0.0

        path_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # ── Pub & Sub ──
        self._vel_pub    = self.create_publisher(Twist,  '/cmd_vel',    10)
        self._status_pub = self.create_publisher(String, '/agv/status', 10)

        self.create_subscription(Path,     '/agv/path', self._on_path, path_qos)
        self.create_subscription(String,   '/agv/cmd',  self._on_cmd,  10)
        self.create_subscription(Odometry, '/odom',     self._on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)

        self.create_timer(1.0 / CTRL_HZ, self._control_loop)

        self.get_logger().info(
            f'path_follower_node started — P-Controller, '
            f'yaw_tol={math.degrees(self._yaw_tol):.0f}° repivot={REPIVOT_THRESHOLD_DEG:.0f}°'
        )

    @staticmethod
    def _wrap_angle(a: float) -> float:
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    def _on_path(self, msg: Path):
        wps = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(wps) < 2:
            self.get_logger().warn('Path rejected: fewer than 2 waypoints')
            return
        self._waypoints = wps
        self._last_path = wps[:]
        self._wp_idx    = 1
        self._nav_phase = 'PIVOT'
        self._state     = 'RUNNING'
        self.get_logger().info(f'New path: {len(wps)} waypoints, state → RUNNING')
        self._publish_status()

    def _on_cmd(self, msg: String):
        cmd = msg.data.strip()
        if cmd == 'pause' and self._state == 'RUNNING':
            self._state = 'PAUSED'
            self._stop_motors()
        elif cmd == 'resume' and self._state == 'PAUSED':
            self._state = 'RUNNING'
        elif cmd == 'stop':
            self._state = 'STOPPED'
            self._waypoints = []
            self._stop_motors()
        elif cmd == 'rerun':
            if self._last_path:
                self._waypoints = self._last_path[:]
                self._wp_idx    = 1
                self._nav_phase = 'PIVOT'
                self._state     = 'RUNNING'
        elif cmd.startswith('set_speed:'):
            try:
                val = float(cmd.split(':')[1])
                self._cruise_speed = max(0.05, min(0.8, val))
            except ValueError:
                pass
        self._publish_status()

    def _on_odom(self, msg: Odometry):
        self._odom_x   = msg.pose.pose.position.x
        self._odom_y   = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._odom_yaw = math.atan2(siny, cosy)
        self._odom_received = True

    def _on_scan(self, msg: LaserScan):
        self._obs_dist = self.get_parameter('obstacle_dist').value
        half_rad = math.radians(OBS_SECTOR_HALF_DEG)
        detected = False
        for i, r in enumerate(msg.ranges):
            if not (OBS_MIN_RANGE < r < self._obs_dist):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) < half_rad:
                detected = True
                break
        self._obstacle = detected

    def _control_loop(self):
        current_time = self.get_clock().now().nanoseconds / 1e9

        if self._state != 'RUNNING':
            # PENTING: kirim perintah berhenti SETIAP cycle, bukan sekali saja.
            # kinematics_node mempertahankan perintah cmd_vel terakhir jika tidak
            # ada pesan baru yang masuk — jadi kalau hanya kirim Twist() kosong
            # sekali lalu diam, motor akan "membeku" di perintah maju terakhir dan
            # robot terus berjalan. Dengan terus mem-publish nol, motor dijamin
            # menerima perintah berhenti secara kontinu di state DONE/STOPPED/
            # PAUSED/IDLE.
            self._stop_motors()
            self._publish_status()
            return

        if not self._odom_received:
            self._stop_motors()
            if current_time - self._last_log_time_odom > 2.0:
                self.get_logger().warn('Menunggu data /odom...')
                self._last_log_time_odom = current_time
            self._publish_status()
            return

        if not self._waypoints or self._wp_idx >= len(self._waypoints):
            self._state = 'DONE'
            self._stop_motors()
            self._publish_status()
            return

        if self._obstacle and self._obs_dist > 0:
            self._stop_motors()
            if current_time - self._last_log_time_obs > 2.0:
                self.get_logger().warn('Obstacle terdeteksi! Menghentikan motor sementara.')
                self._last_log_time_obs = current_time
            self._publish_status()
            return

        x, y, yaw = self._odom_x, self._odom_y, self._odom_yaw
        gx, gy = self._waypoints[self._wp_idx]

        dist_to_goal = math.hypot(gx - x, gy - y)
        target_yaw   = math.atan2(gy - y, gx - x)
        yaw_err      = self._wrap_angle(target_yaw - yaw)

        # Waypoint terakhir pakai toleransi lebih longgar — "sampai sekitar sini"
        # sudah cukup, tidak perlu presisi sudut yang membuat robot berputar-putar.
        is_final_wp = (self._wp_idx == len(self._waypoints) - 1)
        effective_goal_tol = FINAL_GOAL_TOL if is_final_wp else self._goal_tol

        if dist_to_goal < effective_goal_tol:
            self._wp_idx += 1
            if self._wp_idx >= len(self._waypoints):
                self._state = 'DONE'
                self._stop_motors()
                self.get_logger().info('Tujuan akhir tercapai → DONE')
                self._publish_status()
                return
            self._nav_phase = 'PIVOT'
            self._publish_status()
            return

        # Zona "approach": ketika sudah dekat waypoint tapi belum dalam toleransi,
        # LARANG pivot di tempat. Dari jarak sangat dekat, target_yaw bisa berbalik
        # tajam dan memicu pivot 180° bolak-balik (osilasi mengelilingi titik).
        # Di zona ini robot hanya maju lurus pelan; jika memang sudah cukup dekat
        # akan segera masuk toleransi di cycle berikutnya dan dianggap selesai.
        in_no_pivot_zone = dist_to_goal < (effective_goal_tol * NO_PIVOT_ZONE_FACTOR)

        twist = Twist()

        if self._nav_phase == 'PIVOT':
            # Di zona dekat waypoint, jangan pivot — paksa maju pelan saja.
            # Ini memutus siklus osilasi (pivot 180° bolak-balik) di titik akhir.
            if in_no_pivot_zone:
                self._nav_phase = 'FORWARD'
                twist.linear.x  = min(self._cruise_speed, 0.15)  # merayap pelan ke titik
                twist.angular.z = 0.0
            elif abs(yaw_err) <= self._yaw_tol:
                self._nav_phase = 'FORWARD'
                twist.linear.x  = 0.0
                twist.angular.z = 0.0
            else:
                twist.linear.x  = 0.0

                # --- P-CONTROLLER UNTUK PIVOT (Anti-Bablas) ---
                Kp_pivot = 1.0
                target_w = Kp_pivot * yaw_err

                # min_w = batas bawah agar motor tidak stall/mendengung saat pelan,
                #         tapi cukup rendah agar tidak overshoot melewati toleransi.
                min_w = 0.18
                max_w = self._pivot_speed

                if target_w > 0:
                    twist.angular.z = max(min_w, min(max_w, target_w))
                else:
                    twist.angular.z = min(-min_w, max(-max_w, target_w))

        elif self._nav_phase == 'FORWARD':
            if in_no_pivot_zone:
                # Sudah dekat target: merayap lurus tanpa koreksi sudut agresif,
                # biarkan masuk toleransi secara alami. Tidak boleh repivot di sini.
                twist.linear.x  = min(self._cruise_speed, 0.15)
                twist.angular.z = 0.0
            elif abs(yaw_err) > math.radians(REPIVOT_THRESHOLD_DEG):
                self._nav_phase = 'PIVOT'
                twist.linear.x  = 0.0
                twist.angular.z = 0.0
            else:
                twist.linear.x  = self._cruise_speed

                # --- P-CONTROLLER UNTUK FORWARD (Anti-Osilasi/Zig-zag) ---
                Kp_yaw = 0.6
                koreksi_steer = Kp_yaw * yaw_err
                twist.angular.z = max(-0.15, min(0.15, koreksi_steer))

        self._vel_pub.publish(twist)
        self._publish_status()

    def _stop_motors(self):
        self._vel_pub.publish(Twist())

    def _publish_status(self):
        payload = {
            'state':    self._state,
            'phase':    self._nav_phase if self._state == 'RUNNING' else '',
            'x':        round(self._odom_x, 4),
            'y':        round(self._odom_y, 4),
            'yaw_deg':  round(math.degrees(self._odom_yaw), 2),
            'waypoint': self._wp_idx,
            'total_wp': len(self._waypoints),
            'obstacle': self._obstacle
        }
        m = String()
        m.data = json.dumps(payload)
        self._status_pub.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
