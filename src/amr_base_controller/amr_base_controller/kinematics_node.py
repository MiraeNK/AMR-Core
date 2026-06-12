#!/usr/bin/env python3

"""
kinematics_node.py — AMR Base Controller Layer
===============================================
Maintainer : Hafizh Husaini <miraenk7@gmail.com>

Tanggung jawab node ini:
  1. Konversi /wheel_ticks  -> /odom + TF broadcast (odom -> base_footprint)
  2. Konversi /cmd_vel      -> /wheel_cmd_vel
  3. Logger output ke PLC

ARSITEKTUR OUTPUT PLC
---------------------
Unit PLC (0-350) adalah unit frekuensi/duty cycle driver motor langsung,
bukan m/s. Konversi dibagi tiga jalur:

  Zona Transit  (|vx| >= nav2_cruise_threshold):
    Roda dominan dijamin antara plc_cruise_min .. plc_cruise_max.

  Zona Presisi  (|vx| < nav2_cruise_threshold):
    Output 0 .. plc_precision_max, deadband afin.

  Pivot Anti-Stall (terdeteksi roda berlawanan arah saat vx kecil):
    Tenaga dinaikkan ke plc_pivot_min agar bodi benar-benar berputar
    menembus gesekan statis lantai — mencegah motor mendengung (stall)
    yang memicu trip pada driver PLC.

  Zero-snap: jika Nav2 praktis menyuruh diam total, output di-ramp ke 0.
  Ramping: semua perubahan dibatasi plc_ramp_rate per tick (anti-trip).
"""

import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Int64MultiArray
from tf2_ros import TransformBroadcaster


# =============================================================================
# HELPER
# =============================================================================

def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def u32_to_signed(u32: int) -> int:
    return u32 if u32 < 0x80000000 else u32 - 0x100000000


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


# =============================================================================
# NODE UTAMA
# =============================================================================

class KinematicsNode(Node):

    def __init__(self):
        super().__init__('kinematics_node')

        # -- Mekanikal --
        self.declare_parameter('wheel_diameter_m',   0.110)
        self.declare_parameter('wheel_separation_m', 0.240)
        self.declare_parameter('ticks_per_rev',      360000.0)
        self.declare_parameter('gear_ratio',         1.0)

        # -- Frame --
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')

        # -- Referensi Nav2 --
        self.declare_parameter('max_linear_vel',  3.0)
        self.declare_parameter('max_angular_vel', 1.5)

        # -- Batas register PLC --
        self.declare_parameter('speed_max', 350)
        self.declare_parameter('speed_min', -350)

        # -- Filter odometri --
        self.declare_parameter('jump_filter_m', 5.0)

        # -- Zona Transit --
        self.declare_parameter('nav2_cruise_threshold', 0.10)  # m/s
        self.declare_parameter('plc_cruise_min', 120.0)
        self.declare_parameter('plc_cruise_max', 350.0)

        # -- Zona Presisi --
        self.declare_parameter('plc_precision_max', 80.0)
        self.declare_parameter('plc_deadband',      35.0)

        # -- Pivot Anti-Stall --
        # Saat robot harus berputar di tempat (roda berlawanan arah), tenaga
        # 35 tidak cukup menembus gesekan statis lantai -> motor mendengung ->
        # trip. plc_pivot_min memberi torsi cukup agar pivot tuntas & decisive.
        self.declare_parameter('plc_pivot_min', 55.0)
        # vx di bawah ini dianggap "tidak maju" -> kandidat pivot.
        self.declare_parameter('pivot_vx_eps', 0.05)   # m/s

        # -- Ramping anti-trip --
        self.declare_parameter('plc_ramp_rate', 40.0)

        # -- Zero-snap --
        self.declare_parameter('stop_lin_eps', 0.03)   # m/s
        self.declare_parameter('stop_ang_eps', 0.15)   # rad/s

        self._load_parameters()
        self._init_state()
        self._init_ros_interfaces()

        self.get_logger().info(
            '[ kinematics_node ] Init selesai. '
            'Mode: transit/presisi + pivot anti-stall + ramping + zero-snap.'
        )

    # =========================================================================
    # INISIALISASI
    # =========================================================================

    def _load_parameters(self):
        p = type('Params', (), {})()

        p.wheel_diameter_m   = float(self.get_parameter('wheel_diameter_m').value)
        p.wheel_separation_m = float(self.get_parameter('wheel_separation_m').value)
        p.ticks_per_rev      = float(self.get_parameter('ticks_per_rev').value)
        p.gear_ratio         = float(self.get_parameter('gear_ratio').value)
        p.circ               = math.pi * p.wheel_diameter_m

        p.odom_frame_id = self.get_parameter('odom_frame_id').value
        p.base_frame_id = self.get_parameter('base_frame_id').value

        p.max_linear_vel  = float(self.get_parameter('max_linear_vel').value)
        p.max_angular_vel = float(self.get_parameter('max_angular_vel').value)
        p.speed_max       = int(self.get_parameter('speed_max').value)
        p.speed_min       = int(self.get_parameter('speed_min').value)
        p.jump_filter_m   = float(self.get_parameter('jump_filter_m').value)

        p.nav2_cruise_threshold = float(self.get_parameter('nav2_cruise_threshold').value)
        p.plc_cruise_min        = float(self.get_parameter('plc_cruise_min').value)
        p.plc_cruise_max        = float(self.get_parameter('plc_cruise_max').value)

        p.plc_precision_max = float(self.get_parameter('plc_precision_max').value)
        p.plc_deadband      = float(self.get_parameter('plc_deadband').value)

        p.plc_pivot_min = float(self.get_parameter('plc_pivot_min').value)
        p.pivot_vx_eps  = float(self.get_parameter('pivot_vx_eps').value)

        p.plc_ramp_rate = float(self.get_parameter('plc_ramp_rate').value)

        p.stop_lin_eps = float(self.get_parameter('stop_lin_eps').value)
        p.stop_ang_eps = float(self.get_parameter('stop_ang_eps').value)

        self._p = p

    def _init_state(self):
        self._lock = threading.Lock()

        self._x   = 0.0
        self._y   = 0.0
        self._yaw = 0.0
        self._vx  = 0.0
        self._wz  = 0.0

        self._last_left_ticks:  Optional[int]   = None
        self._last_right_ticks: Optional[int]   = None
        self._jump_count       = 0
        self._last_tick_time:   Optional[float] = None

        self._plc_right = 0.0
        self._plc_left  = 0.0
        self._last_cmd_time = time.time()

    def _init_ros_interfaces(self):
        self._pub_odom = self.create_publisher(Odometry,          '/odom',          10)
        self._pub_cmd  = self.create_publisher(Float64MultiArray, '/wheel_cmd_vel', 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._sub_ticks   = self.create_subscription(
            Int64MultiArray, '/wheel_ticks', self._ticks_callback,   10)
        self._sub_cmd_vel = self.create_subscription(
            Twist,           '/cmd_vel',     self._cmd_vel_callback, 10)

    # =========================================================================
    # KONVERSI cmd_vel -> PLC
    # =========================================================================

    def _ramp(self, current: float, target: float, max_delta: float) -> float:
        diff = target - current
        if abs(diff) <= max_delta:
            return target
        return current + math.copysign(max_delta, diff)

    def _deadband_affine(self, raw: float, hi: float) -> float:
        p = self._p
        a = abs(raw)
        if a < 1.0:
            return 0.0
        if hi <= p.plc_deadband:
            return math.copysign(p.plc_deadband, raw)
        comp = p.plc_deadband + (a - 1.0) / (hi - 1.0) * (hi - p.plc_deadband)
        return math.copysign(clamp(comp, p.plc_deadband, hi), raw)

    def _apply_pivot_floor(self, tgt_r: float, tgt_l: float, vx: float):
        """
        Jika robot harus pivot (vx kecil, roda berlawanan arah), naikkan
        magnitudo kedua roda minimal ke plc_pivot_min agar bodi benar-benar
        berputar menembus gesekan statis. Mencegah stall/trip.
        """
        p = self._p
        if abs(vx) >= p.pivot_vx_eps:
            return tgt_r, tgt_l
        # Pivot = tanda berlawanan dan keduanya non-nol.
        if tgt_r * tgt_l >= 0:
            return tgt_r, tgt_l
        if tgt_r != 0.0 and abs(tgt_r) < p.plc_pivot_min:
            tgt_r = math.copysign(p.plc_pivot_min, tgt_r)
        if tgt_l != 0.0 and abs(tgt_l) < p.plc_pivot_min:
            tgt_l = math.copysign(p.plc_pivot_min, tgt_l)
        return tgt_r, tgt_l

    def _cmd_vel_callback(self, msg: Twist):
        p   = self._p
        now = time.time()
        dt  = clamp(now - self._last_cmd_time, 0.005, 0.2)
        self._last_cmd_time = now

        vx = clamp(msg.linear.x,  -p.max_linear_vel,  p.max_linear_vel)
        wz = clamp(msg.angular.z, -p.max_angular_vel,  p.max_angular_vel)

        # -- ZERO-SNAP: Nav2 praktis diam total --
        if abs(vx) < p.stop_lin_eps and abs(wz) < p.stop_ang_eps:
            self._plc_right = self._ramp(self._plc_right, 0.0, p.plc_ramp_rate)
            self._plc_left  = self._ramp(self._plc_left,  0.0, p.plc_ramp_rate)
            self._publish_plc(self._plc_right, self._plc_left, msg)
            return

        v_r_norm = (vx + wz * (p.wheel_separation_m / 2.0)) / p.max_linear_vel
        v_l_norm = (vx - wz * (p.wheel_separation_m / 2.0)) / p.max_linear_vel

        if abs(vx) >= p.nav2_cruise_threshold:
            # == ZONA TRANSIT ==
            raw_r = v_r_norm * p.plc_cruise_max
            raw_l = v_l_norm * p.plc_cruise_max
            max_abs = max(abs(raw_r), abs(raw_l))
            if max_abs > 0 and max_abs < p.plc_cruise_min:
                boost = p.plc_cruise_min / max_abs
                raw_r *= boost
                raw_l *= boost
            tgt_r = clamp(raw_r, float(p.speed_min), float(p.speed_max))
            tgt_l = clamp(raw_l, float(p.speed_min), float(p.speed_max))
        else:
            # == ZONA PRESISI ==
            tgt_r = self._deadband_affine(v_r_norm * p.plc_precision_max, p.plc_precision_max)
            tgt_l = self._deadband_affine(v_l_norm * p.plc_precision_max, p.plc_precision_max)

        # == PIVOT ANTI-STALL (berlaku di kedua zona) ==
        tgt_r, tgt_l = self._apply_pivot_floor(tgt_r, tgt_l, vx)

        tgt_r = clamp(tgt_r, float(p.speed_min), float(p.speed_max))
        tgt_l = clamp(tgt_l, float(p.speed_min), float(p.speed_max))

        # -- RAMPING anti-trip --
        self._plc_right = self._ramp(self._plc_right, tgt_r, p.plc_ramp_rate)
        self._plc_left  = self._ramp(self._plc_left,  tgt_l, p.plc_ramp_rate)

        self._publish_plc(self._plc_right, self._plc_left, msg)

    def _publish_plc(self, speed_right: float, speed_left: float, msg: Twist):
        p = self._p
        r = clamp(speed_right, float(p.speed_min), float(p.speed_max))
        l = clamp(speed_left,  float(p.speed_min), float(p.speed_max))

        if r != 0.0 or l != 0.0:
            self.get_logger().info(
                f"[ SPEED DEBUG ] Out to PLC -> Kanan: {r:.1f} | Kiri: {l:.1f} "
                f"(Input Nav2: V={msg.linear.x:.2f}m/s, W={msg.angular.z:.2f}rad/s)",
                throttle_duration_sec=0.5
            )

        cmd_msg = Float64MultiArray()
        cmd_msg.data = [float(r), float(l)]
        self._pub_cmd.publish(cmd_msg)

    # =========================================================================
    # ODOMETRI
    # =========================================================================

    def _ticks_callback(self, msg: Int64MultiArray):
        if len(msg.data) < 2:
            return

        now       = time.time()
        left_u32  = int(msg.data[0]) & 0xFFFFFFFF
        right_u32 = int(msg.data[1]) & 0xFFFFFFFF
        p         = self._p

        with self._lock:
            if self._last_left_ticks is None:
                self._last_left_ticks  = left_u32
                self._last_right_ticks = right_u32
                self._last_tick_time   = now
                return

            dt = now - self._last_tick_time
            if dt <= 0.0:
                return

            dl_u32 = (left_u32  - self._last_left_ticks)  & 0xFFFFFFFF
            dr_u32 = (right_u32 - self._last_right_ticks) & 0xFFFFFFFF

            dl_ticks = u32_to_signed(dl_u32)
            dr_ticks = u32_to_signed(dr_u32)

            dl_dist = (float(dl_ticks) / p.ticks_per_rev / p.gear_ratio) * p.circ
            dr_dist = (float(dr_ticks) / p.ticks_per_rev / p.gear_ratio) * p.circ

            if abs(dl_dist) >= p.jump_filter_m or abs(dr_dist) >= p.jump_filter_m:
                self._jump_count += 1
                self._last_tick_time   = now
                self._last_left_ticks  = left_u32
                self._last_right_ticks = right_u32
                if self._jump_count >= 15:
                    self._jump_count = 0
                self._publish_odom(self._x, self._y, self._yaw, 0.0, 0.0)
                return

            self._jump_count = 0

            delta_s     = (dl_dist + dr_dist) * 0.5
            delta_theta = (dr_dist - dl_dist) / max(1e-6, p.wheel_separation_m)
            mid_yaw     = self._yaw + 0.5 * delta_theta

            self._x   += delta_s * math.cos(mid_yaw)
            self._y   += delta_s * math.sin(mid_yaw)
            self._yaw += delta_theta
            self._vx   = delta_s / dt
            self._wz   = delta_theta / dt

            x, y, yaw, vx, wz = self._x, self._y, self._yaw, self._vx, self._wz

            self._last_left_ticks  = left_u32
            self._last_right_ticks = right_u32
            self._last_tick_time   = now

        self._publish_odom(x, y, yaw, vx, wz)

    def _publish_odom(self, x, y, yaw, vx, wz):
        p     = self._p
        stamp = self.get_clock().now().to_msg()
        q     = yaw_to_quaternion(yaw)

        odom                         = Odometry()
        odom.header.stamp            = stamp
        odom.header.frame_id         = p.odom_frame_id
        odom.child_frame_id          = p.base_frame_id
        odom.pose.pose.position.x    = x
        odom.pose.pose.position.y    = y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation   = q
        odom.twist.twist.linear.x    = vx
        odom.twist.twist.angular.z   = wz
        self._pub_odom.publish(odom)

        tf                         = TransformStamped()
        tf.header.stamp            = stamp
        tf.header.frame_id         = p.odom_frame_id
        tf.child_frame_id          = p.base_frame_id
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = 0.0
        tf.transform.rotation      = q
        self._tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
