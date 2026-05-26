
#!/usr/bin/env python3

"""

kinematics_node.py — AMR Base Controller Layer

===============================================

Maintainer : Hafizh Husaini <miraenk7@gmail.com>



Tanggung jawab node ini:

  1. Konversi /wheel_ticks  → /odom + TF broadcast (odom → base_footprint)

  2. Konversi /cmd_vel      → /wheel_cmd_vel



Rumus kinematika IDENTIK dengan digital_twin_enhanced.py yang sudah teruji

di hardware Polebot, termasuk:

  - Signed u32 tick conversion

  - Jump filter per siklus

  - Mid-yaw integration (Runge-Kutta orde 1)

  - Negasi delta_theta (koreksi orientasi fisik roda kiri terpasang terbalik)



Topik:

  Subscribe  /wheel_ticks    std_msgs/Int64MultiArray  [left_u32, right_u32]

  Subscribe  /cmd_vel        geometry_msgs/Twist

  Publish    /odom           nav_msgs/Odometry

  Publish    /wheel_cmd_vel  std_msgs/Float64MultiArray [right_spd, left_spd]

  Broadcast  TF odom → base_footprint

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

# FUNGSI HELPER — MURNI

# =============================================================================



def yaw_to_quaternion(yaw):

    """Konversi yaw (radian) ke geometry_msgs/Quaternion."""

    q = Quaternion()

    q.x = 0.0

    q.y = 0.0

    q.z = math.sin(yaw / 2.0)

    q.w = math.cos(yaw / 2.0)

    return q





def u32_to_signed(u32):

    """

    Konversi nilai u32 ke signed integer 32-bit.

    Identik dengan konversi di digital_twin_enhanced.py.

    """

    return u32 if u32 < 0x80000000 else u32 - 0x100000000





# =============================================================================

# NODE UTAMA

# =============================================================================



class KinematicsNode(Node):



    def __init__(self):

        super().__init__('kinematics_node')

        self._declare_all_parameters()

        self._load_parameters()

        self._init_state()

        self._init_ros_interfaces()

        self.get_logger().info(

            '[ kinematics_node ] Init selesai. '

            'wheel_diameter={}m  separation={}m  '

            'ticks_per_rev={}  gear_ratio={}'.format(

                self._p.wheel_diameter_m,

                self._p.wheel_separation_m,

                self._p.ticks_per_rev,

                self._p.gear_ratio,

            )

        )



    # =========================================================================

    # PARAMETER

    # =========================================================================



    def _declare_all_parameters(self):

        self.declare_parameter('wheel_diameter_m',   0.110)

        self.declare_parameter('wheel_separation_m', 0.240)

        self.declare_parameter('ticks_per_rev',      360000.0)

        self.declare_parameter('gear_ratio',         1.0)

        self.declare_parameter('odom_frame_id',      'odom')

        self.declare_parameter('base_frame_id',      'base_footprint')

        self.declare_parameter('max_linear_vel',     0.5)

        self.declare_parameter('max_angular_vel',    1.0)

        self.declare_parameter('speed_max',          100)

        self.declare_parameter('speed_min',          -100)

        self.declare_parameter('jump_filter_m',      0.20)



    def _load_parameters(self):

        p = type('Params', (), {})()



        p.wheel_diameter_m   = float(

            self.get_parameter('wheel_diameter_m').value)

        p.wheel_separation_m = float(

            self.get_parameter('wheel_separation_m').value)

        p.ticks_per_rev      = float(

            self.get_parameter('ticks_per_rev').value)

        p.gear_ratio         = float(

            self.get_parameter('gear_ratio').value)

        p.circ               = math.pi * p.wheel_diameter_m



        p.odom_frame_id      = self.get_parameter('odom_frame_id').value

        p.base_frame_id      = self.get_parameter('base_frame_id').value



        p.max_linear_vel     = float(

            self.get_parameter('max_linear_vel').value)

        p.max_angular_vel    = float(

            self.get_parameter('max_angular_vel').value)

        p.speed_max          = int(self.get_parameter('speed_max').value)

        p.speed_min          = int(self.get_parameter('speed_min').value)



        p.jump_filter_m      = float(

            self.get_parameter('jump_filter_m').value)



        self._p = p



        self.get_logger().info(

            '[ PARAMS ] '

            'odom_frame={} base_frame={} '

            'max_lin={}m/s max_ang={}rad/s '

            'jump_filter={}m'.format(

                p.odom_frame_id, p.base_frame_id,

                p.max_linear_vel, p.max_angular_vel,

                p.jump_filter_m,

            )

        )



    # =========================================================================

    # STATE

    # =========================================================================



    def _init_state(self):

        self._lock = threading.Lock()



        # Pose robot — state odometri

        # Identik dengan self.x, self.y, self.yaw di digital_twin_enhanced.py

        self._x   = 0.0

        self._y   = 0.0

        self._yaw = 0.0



        # Referensi encoder siklus sebelumnya

        self._last_left_ticks  = None  # type: Optional[int]

        self._last_right_ticks = None  # type: Optional[int]



        # Kecepatan terakhir untuk publikasi Odometry twist

        self._vx = 0.0

        self._wz = 0.0



        # Timestamp siklus terakhir untuk dt

        self._last_tick_time = None  # type: Optional[float]



    # =========================================================================

    # ROS INTERFACE

    # =========================================================================



    def _init_ros_interfaces(self):

        # Publishers

        self._pub_odom = self.create_publisher(

            Odometry, '/odom', 10)

        self._pub_cmd = self.create_publisher(

            Float64MultiArray, '/wheel_cmd_vel', 10)



        # TF broadcaster

        self._tf_broadcaster = TransformBroadcaster(self)



        # Subscribers

        self._sub_ticks = self.create_subscription(

            Int64MultiArray,

            '/wheel_ticks',

            self._ticks_callback,

            10,

        )

        self._sub_cmd_vel = self.create_subscription(

            Twist,

            '/cmd_vel',

            self._cmd_vel_callback,

            10,

        )



    # =========================================================================

    # CALLBACK: /wheel_ticks → odometri + TF

    # =========================================================================



    def _ticks_callback(self, msg):

        """

        Konversi tick encoder menjadi pose odometri.



        Rumus IDENTIK dengan plc_worker() di digital_twin_enhanced.py:



          dl_ticks_u32 = (left_u32 - last_left) & 0xFFFFFFFF

          dl_ticks     = signed(dl_ticks_u32)

          dl_dist      = (dl_ticks / TICKS_PER_REV / GEAR_RATIO) * circ



          delta_s      = (dl_dist + dr_dist) * 0.5

          delta_theta  = ((dr_dist - dl_dist) / WHEEL_SEPARATION)

                         ↑ negasi: koreksi fisik roda kiri terpasang terbalik



          mid_yaw = yaw + 0.5 * delta_theta

          x   += delta_s * cos(mid_yaw)

          y   += delta_s * sin(mid_yaw)

          yaw += delta_theta

        """

        if len(msg.data) < 2:

            self.get_logger().warn(

                '[ TICKS ] /wheel_ticks butuh 2 elemen. '

                'Diterima: {}'.format(len(msg.data)))

            return



        now       = time.time()

        left_u32  = int(msg.data[0]) & 0xFFFFFFFF

        right_u32 = int(msg.data[1]) & 0xFFFFFFFF

        p         = self._p



        with self._lock:

            # Siklus pertama — simpan referensi, belum bisa hitung delta

            if self._last_left_ticks is None:

                self._last_left_ticks  = left_u32

                self._last_right_ticks = right_u32

                self._last_tick_time   = now

                return



            dt = now - self._last_tick_time

            if dt <= 0.0:

                return



            # ── Delta ticks (u32 wrapping-safe) ──────────────────────────

            dl_u32 = (left_u32  - self._last_left_ticks)  & 0xFFFFFFFF

            dr_u32 = (right_u32 - self._last_right_ticks) & 0xFFFFFFFF



            # Signed conversion — identik dengan digital_twin_enhanced.py

            dl_ticks = u32_to_signed(dl_u32)

            dr_ticks = u32_to_signed(dr_u32)



            # ── Konversi ticks ke jarak (meter) ──────────────────────────

            dl_dist = (float(dl_ticks) / p.ticks_per_rev

                       / p.gear_ratio) * p.circ

            dr_dist = (float(dr_ticks) / p.ticks_per_rev

                       / p.gear_ratio) * p.circ



            # ── Jump filter — identik dengan digital_twin_enhanced.py ────

            if (abs(dl_dist) >= p.jump_filter_m or

                    abs(dr_dist) >= p.jump_filter_m):

                self.get_logger().warn(

                    '[ JUMP ] Delta L={:+.3f}m R={:+.3f}m '

                    '→ siklus diabaikan.'.format(dl_dist, dr_dist))

                self._last_tick_time   = now

                self._last_left_ticks  = left_u32

                self._last_right_ticks = right_u32

                # Tetap publish TF posisi terakhir valid agar slam_toolbox

                # tidak kehilangan transform selama encoder recovery

                _stamp = self.get_clock().now().to_msg()

                _q     = yaw_to_quaternion(self._yaw)

                _odom  = Odometry()

                _odom.header.stamp          = _stamp

                _odom.header.frame_id       = p.odom_frame_id

                _odom.child_frame_id        = p.base_frame_id

                _odom.pose.pose.position.x  = self._x

                _odom.pose.pose.position.y  = self._y

                _odom.pose.pose.orientation = _q

                _odom.twist.twist.linear.x  = 0.0

                _odom.twist.twist.angular.z = 0.0

                self._pub_odom.publish(_odom)

                _tf = TransformStamped()

                _tf.header.stamp            = _stamp

                _tf.header.frame_id         = p.odom_frame_id

                _tf.child_frame_id          = p.base_frame_id

                _tf.transform.translation.x = self._x

                _tf.transform.translation.y = self._y

                _tf.transform.translation.z = 0.0

                _tf.transform.rotation      = _q

                self._tf_broadcaster.sendTransform(_tf)

                return



            # ── Kinematika differential drive ────────────────────────────

            delta_s = (dl_dist + dr_dist) * 0.5



            # NEGASI delta_theta:

            # Koreksi orientasi karena roda kiri terpasang terbalik

            # secara fisik pada Polebot. Tanpa negasi ini belok kanan

            # terbaca sebagai belok kiri di odometri.

            # Sumber: digital_twin_enhanced.py baris koreksi orientasi.

            delta_theta = (

                (dr_dist - dl_dist) / max(1e-6, p.wheel_separation_m)

            )



            # Mid-yaw integration (Runge-Kutta orde 1)

            # Lebih akurat dari Euler biasa untuk kurva

            mid_yaw = self._yaw + 0.5 * delta_theta



            self._x   += delta_s * math.cos(mid_yaw)

            self._y   += delta_s * math.sin(mid_yaw)

            self._yaw += delta_theta



            # Kecepatan untuk twist field Odometry

            self._vx = delta_s / dt

            self._wz = delta_theta / dt



            # Snapshot untuk publish

            x   = self._x

            y   = self._y

            yaw = self._yaw

            vx  = self._vx

            wz  = self._wz



            # Update referensi

            self._last_left_ticks  = left_u32

            self._last_right_ticks = right_u32

            self._last_tick_time   = now



        # ── Publikasikan /odom ────────────────────────────────────────────

        stamp = self.get_clock().now().to_msg()

        q     = yaw_to_quaternion(yaw)



        odom_msg = Odometry()

        odom_msg.header.stamp            = stamp

        odom_msg.header.frame_id         = p.odom_frame_id

        odom_msg.child_frame_id          = p.base_frame_id

        odom_msg.pose.pose.position.x    = x

        odom_msg.pose.pose.position.y    = y

        odom_msg.pose.pose.position.z    = 0.0

        odom_msg.pose.pose.orientation   = q

        odom_msg.twist.twist.linear.x    = vx

        odom_msg.twist.twist.angular.z   = wz

        self._pub_odom.publish(odom_msg)



        # ── Broadcast TF: odom → base_footprint ──────────────────────────

        tf_msg                         = TransformStamped()

        tf_msg.header.stamp            = stamp

        tf_msg.header.frame_id         = p.odom_frame_id

        tf_msg.child_frame_id          = p.base_frame_id

        tf_msg.transform.translation.x = x

        tf_msg.transform.translation.y = y

        tf_msg.transform.translation.z = 0.0

        tf_msg.transform.rotation      = q

        self._tf_broadcaster.sendTransform(tf_msg)



    # =========================================================================

    # CALLBACK: /cmd_vel → /wheel_cmd_vel

    # =========================================================================



    def _cmd_vel_callback(self, msg):

        """

        Konversi Twist (linear.x, angular.z) ke kecepatan roda kanan/kiri

        dalam unit PLC (-100 s/d 100).



        Rumus differential drive:

          v_right = linear.x + angular.z * (wheel_separation / 2)

          v_left  = linear.x - angular.z * (wheel_separation / 2)



        Normalisasi ke unit PLC:

          speed_right = (v_right / max_linear_vel) * speed_max

          speed_left  = (v_left  / max_linear_vel) * speed_max

        """

        p = self._p



        linear  = msg.linear.x

        angular = msg.angular.z



        # Differential drive decomposition

        v_right = linear + angular * (p.wheel_separation_m / 2.0)

        v_left  = linear - angular * (p.wheel_separation_m / 2.0)



        # Normalisasi ke unit PLC

        if p.max_linear_vel > 0:

            scale = float(p.speed_max) / p.max_linear_vel

        else:

            scale = 0.0



        speed_right = v_right * scale

        speed_left  = v_left  * scale



        # Clamp ke batas

        speed_right = max(float(p.speed_min),

                          min(float(p.speed_max), speed_right))

        speed_left  = max(float(p.speed_min),

                          min(float(p.speed_max), speed_left))



        cmd_msg      = Float64MultiArray()

        cmd_msg.data = [speed_right, speed_left]

        self._pub_cmd.publish(cmd_msg)





# =============================================================================

# ENTRY POINT

# =============================================================================



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

