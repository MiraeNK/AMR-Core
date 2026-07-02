#!/usr/bin/env python3
"""
agv_line_follower_node.py — Virtual Magnetic Line Follower
==========================================================
Design And Develop : Engineering - Eqdev - AISIN Indonesia
Maintainer : Hafizh Husaini - Intern <miraenk7@gmail.com>

Logic: 
1. Command Mirroring
2. Anti Corner-Cutting
3. PLC Safety Clamp
4. Curvature Look-Ahead Speed Scaling
5. EWMA CTE Smoothing + Deadband Logic (Transient suppression)
6. Continuous Patrol Mode (Looping Abadi)
"""

import json
import math
import threading
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Float64MultiArray, String
from tf_transformations import euler_from_quaternion


# =============================================================================
# GEOMETRI & KALKULASI
# =============================================================================

def cte_to_segment(rx, ry, ax, ay, bx, by) -> float:
    dx = bx - ax
    dy = by - ay
    seg_len = math.hypot(dx, dy)
    if seg_len < 1e-6:
        return math.hypot(rx - ax, ry - ay)
    nx = dy / seg_len
    ny = -dx / seg_len
    return (rx - ax) * nx + (ry - ay) * ny


def dist_to_point(rx, ry, px, py) -> float:
    return math.hypot(rx - px, ry - py)


def scale_cte(cte_m: float, cte_scale: float, cte_max_m: float) -> int:
    """
    Rumus normal agar sinkron dengan Web GUI.
    """
    cte_clamp = max(-cte_max_m, min(cte_max_m, cte_m))
    scaled = 63 + int(cte_clamp * cte_scale)
    return max(0, min(126, scaled))


def path_heading_at(path: List[dict], idx: int) -> float:
    a = path[idx]
    b = path[min(idx + 1, len(path) - 1)]
    return math.atan2(b['y'] - a['y'], b['x'] - a['x'])


def angle_diff(a: float, b: float) -> float:
    """Selisih sudut terpendek (rad), hasil di rentang [-pi, pi]."""
    d = a - b
    while d > math.pi: 
        d -= 2 * math.pi
    while d < -math.pi: 
        d += 2 * math.pi
    return d


def lookahead_curvature_deg(path: List[dict], seg_idx: int, lookahead_n: int) -> float:
    max_idx = len(path) - 2
    if max_idx <= seg_idx:
        return 0.0
    
    start_heading = path_heading_at(path, seg_idx)
    end_idx = min(seg_idx + lookahead_n, max_idx)
    end_heading = path_heading_at(path, end_idx)
    
    return abs(math.degrees(angle_diff(end_heading, start_heading)))


def curvature_to_speed_scale(curvature_deg: float, soft_deg: float, hard_deg: float, min_scale: float) -> float:
    if curvature_deg <= soft_deg:
        return 1.0
    if curvature_deg >= hard_deg:
        return min_scale
    
    frac = (curvature_deg - soft_deg) / (hard_deg - soft_deg)
    return 1.0 - frac * (1.0 - min_scale)


# =============================================================================
# NODE UTAMA
# =============================================================================

class AgvLineFollowerNode(Node):

    def __init__(self):
        super().__init__('agv_line_follower_node')
        self._declare_params()
        self._load_params()
        self._init_state()
        self._init_ros()
        self._timer = self.create_timer(
            1.0 / self._p.cycle_hz, 
            self._cycle_cb
        )
        self.get_logger().info(
            '[ agv_line_follower ] Init selesai. Anti Corner-Cutting, PLC Clamp, EWMA Filtering, & Continuous Loop ACTIVE.'
        )

    def _declare_params(self):
        self.declare_parameter('cycle_hz', 20.0)
        self.declare_parameter('goal_tolerance_m', 0.30)
        self.declare_parameter('sensor_offset_m', 0.25)
        self.declare_parameter('cte_max_m', 0.20)
        self.declare_parameter('cte_scale_override', -1.0)
        self.declare_parameter('cte_smoothing_alpha', 0.8) 
        self.declare_parameter('cte_deadband_m', 0.015)
        self.declare_parameter('obstacle_dist_m', 0.45)
        self.declare_parameter('obstacle_min_dist_m', 0.20)
        self.declare_parameter('obstacle_angle_deg', 45.0)
        self.declare_parameter('obstacle_confirm_n', 3)
        
        self.declare_parameter('curvature_lookahead_n', 6)     
        self.declare_parameter('curvature_soft_deg', 15.0)     
        self.declare_parameter('curvature_hard_deg', 60.0)     
        self.declare_parameter('curvature_min_speed_scale', 0.35) 

    def _load_params(self):
        p = type('P', (), {})()
        p.cycle_hz = float(self.get_parameter('cycle_hz').value)
        p.goal_tolerance_m = float(self.get_parameter('goal_tolerance_m').value)
        p.sensor_offset_m = float(self.get_parameter('sensor_offset_m').value)
        p.cte_max_m = float(self.get_parameter('cte_max_m').value)
        p.cte_smoothing_alpha = float(self.get_parameter('cte_smoothing_alpha').value)
        p.cte_deadband_m = float(self.get_parameter('cte_deadband_m').value)

        override = float(self.get_parameter('cte_scale_override').value)
        if override > 0:
            p.cte_scale = override
        else:
            p.cte_scale = (126 - 63) / max(p.cte_max_m, 1e-6)

        p.obstacle_dist_m = float(self.get_parameter('obstacle_dist_m').value)
        p.obstacle_min_dist_m = float(self.get_parameter('obstacle_min_dist_m').value)
        p.obstacle_angle_deg = float(self.get_parameter('obstacle_angle_deg').value)
        p.obstacle_confirm_n = int(self.get_parameter('obstacle_confirm_n').value)

        p.curvature_lookahead_n = int(self.get_parameter('curvature_lookahead_n').value)
        p.curvature_soft_deg = float(self.get_parameter('curvature_soft_deg').value)
        p.curvature_hard_deg = float(self.get_parameter('curvature_hard_deg').value)
        p.curvature_min_speed_scale = float(self.get_parameter('curvature_min_speed_scale').value)
        
        self._p = p

    def _init_state(self):
        self._lock = threading.Lock()
        self._rx = 0.0
        self._ry = 0.0
        self._ryaw = 0.0
        self._odom_ok = False
        self._path: List[dict] = []
        self._seg_idx = 0
        self._state = 'IDLE'
        self._hb_bit = 0
        self._obstacle = False
        self._obs_counter = 0
        self._filtered_cte_m: Optional[float] = None
        self._fb_counter = 0
        
        # --- Fitur Looping ---
        self._is_looping = False

    def _init_ros(self):
        qos = QoSProfile(depth=10)
        path_qos = QoSProfile(
            depth=1, 
            durability=DurabilityPolicy.TRANSIENT_LOCAL, 
            reliability=ReliabilityPolicy.RELIABLE
        )
        scan_qos = QoSProfile(
            depth=5, 
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self._sub_odom = self.create_subscription(Odometry, '/odom', self._odom_cb, qos)
        self._sub_path = self.create_subscription(Path, '/agv/path', self._path_cb, path_qos)
        self._sub_cmd = self.create_subscription(String, '/agv/cmd', self._cmd_cb, qos)
        self._sub_scan = self.create_subscription(LaserScan, '/scan', self._scan_cb, scan_qos)

        self._pub_vl_cmd = self.create_publisher(Float64MultiArray, '/agv/vl_cmd', qos)
        self._pub_status = self.create_publisher(String, '/agv/status', qos)
        self._pub_cte    = self.create_publisher(Float32, '/agv/cte', qos)

    def _publish_vl_cmd(self, cte_raw: int, mode: int, gate: int, speed_scale: float = 1.0):
        self._hb_bit = 1 - self._hb_bit
        cte_to_plc = 126 - cte_raw
        cte_to_plc = max(5, min(121, cte_to_plc))

        msg = Float64MultiArray()
        msg.data = [
            float(cte_to_plc), 
            float(mode), 
            float(self._hb_bit), 
            float(gate), 
            float(speed_scale)
        ]
        self._pub_vl_cmd.publish(msg)

    def _odom_cb(self, msg: Odometry):
        with self._lock:
            self._rx = msg.pose.pose.position.x
            self._ry = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            self._ryaw = yaw
            self._odom_ok = True

    def _path_cb(self, msg: Path):
        pts = [{'x': ps.pose.position.x, 'y': ps.pose.position.y} for ps in msg.poses]
        with self._lock: 
            self._path = pts
            self._seg_idx = 0
            self._state = 'RUNNING'

    def _cmd_cb(self, msg: String):
        try: 
            d = json.loads(msg.data)
            cmd = d.get('cmd', '')
        except Exception: 
            cmd = msg.data.strip()
            
        with self._lock:
            if cmd == 'stop': 
                self._state = 'IDLE'
                self._path = []
                self._is_looping = False  # Matikan loop jika di-stop
            elif cmd == 'pause': 
                self._state = 'PAUSED'
            elif cmd == 'resume': 
                self._state = 'RUNNING'
            elif cmd == 'rerun': 
                self._seg_idx = 0
                self._state = 'RUNNING'
            # --- Perintah Loop dari Web GUI ---
            elif cmd == 'enable_loop':
                self._is_looping = True
                self.get_logger().info('[ CMD ] Continuous Patrol Mode: ENABLED')
            elif cmd == 'disable_loop':
                self._is_looping = False
                self.get_logger().info('[ CMD ] Continuous Patrol Mode: DISABLED')

    def _scan_cb(self, msg: LaserScan):
        half_angle = math.radians(self._p.obstacle_angle_deg)
        raw_hit = False
        angle = msg.angle_min
        
        for r in msg.ranges:
            if self._p.obstacle_min_dist_m < r < self._p.obstacle_dist_m:
                a = angle
                while a > math.pi: a -= 2 * math.pi
                while a < -math.pi: a += 2 * math.pi
                if abs(a) <= half_angle: 
                    raw_hit = True
                    break
            angle += msg.angle_increment
            
        with self._lock:
            if raw_hit: 
                self._obs_counter = min(self._obs_counter + 1, self._p.obstacle_confirm_n * 2)
            else: 
                self._obs_counter = max(self._obs_counter - 1, -self._p.obstacle_confirm_n * 2)
                
            self._obstacle = (self._obs_counter >= self._p.obstacle_confirm_n)

    def _cycle_cb(self):
        with self._lock: 
            state = self._state
            path = self._path
            seg_idx = self._seg_idx
            rx = self._rx
            ry = self._ry
            ryaw = self._ryaw
            odom_ok = self._odom_ok
            obstacle = self._obstacle
            
        sx = rx + math.cos(ryaw) * self._p.sensor_offset_m
        sy = ry + math.sin(ryaw) * self._p.sensor_offset_m
        
        if not (state == 'RUNNING' and len(path) >= 2 and odom_ok):
            self._publish_vl_cmd(63, 0, 1, 1.0)
            self._publish_status(state, 0, len(path), 0.0, 63, False, 1.0)
            return
            
        if obstacle:
            self._publish_vl_cmd(63, 1, 1, 1.0)
            self._publish_status(state, seg_idx, len(path), 0.0, 63, True, 1.0)
            return

        seg_idx = self._advance_segment(path, seg_idx, sx, sy)
        with self._lock: 
            self._seg_idx = seg_idx

        last_wp = path[-1]
        dist_to_goal = dist_to_point(rx, ry, last_wp['x'], last_wp['y'])
        
        # --- PERBAIKAN BUG LOOP & KONDISI DONE ---
        # Syarat DONE atau RESET LOOP: Jarak dekat tujuan DAN robot sudah berada di segmen titik-titik terakhir.
        is_near_end_of_path = (seg_idx >= len(path) - 10) or (len(path) < 10)
        
        if is_near_end_of_path and (dist_to_goal < self._p.goal_tolerance_m):
            if self._is_looping:
                with self._lock:
                    self._seg_idx = 0  # Instan reset indeks kembali ke titik nol (Continuous Mode)
            else:
                with self._lock: 
                    self._state = 'DONE'
                self._publish_vl_cmd(63, 0, 1, 1.0)
                self._publish_status('DONE', seg_idx, len(path), 0.0, 63, False, 1.0)
                return
        # ----------------------------------------

        a = path[seg_idx]
        b = path[min(seg_idx + 1, len(path) - 1)]
        
        # 1. Hitung CTE
        cte_m = cte_to_segment(sx, sy, a['x'], a['y'], b['x'], b['y'])

        # 2. EWMA Filter (Smoothing)
        if self._filtered_cte_m is None:
            self._filtered_cte_m = cte_m
        
        alpha = self._p.cte_smoothing_alpha
        self._filtered_cte_m = (alpha * cte_m) + ((1.0 - alpha) * self._filtered_cte_m)

        # 3. DEAD-BAND Logic (Menahan koreksi kecil agar tidak hunting)
        if abs(self._filtered_cte_m) < self._p.cte_deadband_m:
            self._filtered_cte_m = 0.0

        # 4. Scale dengan nilai filter
        cte_raw = scale_cte(self._filtered_cte_m, self._p.cte_scale, self._p.cte_max_m)

        # 5. Look-ahead speed scaling
        curvature_deg = lookahead_curvature_deg(path, seg_idx, self._p.curvature_lookahead_n)
        speed_scale = curvature_to_speed_scale(
            curvature_deg, 
            self._p.curvature_soft_deg,
            self._p.curvature_hard_deg, 
            self._p.curvature_min_speed_scale
        )

        self._publish_vl_cmd(cte_raw, 1, 0, speed_scale)
        self._publish_status(state, seg_idx, len(path), self._filtered_cte_m, cte_raw, False, speed_scale)

        cte_msg = Float32()
        cte_msg.data = float(self._filtered_cte_m)
        self._pub_cte.publish(cte_msg)

        self._fb_counter += 1
        if self._fb_counter % 10 == 0:
            self.get_logger().info(
                f'[ CTE ] seg={seg_idx}/{len(path)-1} cte_filt={self._filtered_cte_m:+.4f} raw={cte_raw}'
            )

    def _advance_segment(self, path, seg_idx, sx, sy) -> int:
        max_seg = len(path) - 2
        if seg_idx >= max_seg: 
            return max_seg

        best_idx = seg_idx
        min_dist = dist_to_point(sx, sy, path[seg_idx]['x'], path[seg_idx]['y'])

        for i in range(seg_idx + 1, min(seg_idx + 15, max_seg + 1)):
            dist = dist_to_point(sx, sy, path[i]['x'], path[i]['y'])
            if dist <= min_dist:
                min_dist = dist
                best_idx = i
            else:
                break
        return best_idx

    def _publish_status(self, state, seg_idx, total_pts, cte_m, cte_raw, obstacle, speed_scale):
        with self._lock:
            rx = self._rx
            ry = self._ry
            ryaw = self._ryaw

        status = {
            'state': state, 
            'x': round(rx, 3), 
            'y': round(ry, 3),
            'yaw_deg': round(math.degrees(ryaw), 1), 
            'waypoint': seg_idx,
            'total_wp': max(0, total_pts - 1), 
            'cte_m': round(cte_m, 4),
            'cte_raw': cte_raw, 
            'obstacle': obstacle, 
            'speed_scale': round(speed_scale, 3)
        }
        msg = String()
        msg.data = json.dumps(status)
        self._pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AgvLineFollowerNode()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally: 
        try:
            node._publish_vl_cmd(63, 0, 1, 1.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': 
    main()
