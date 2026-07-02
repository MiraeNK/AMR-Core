#!/usr/bin/env python3
"""
plc_driver_node.py — AMR Hardware Abstraction Layer
====================================================
Design And Develop : Engineering - Eqdev - AISIN Indonesia
Maintainer : Hafizh Husaini - Intern <miraenk7@gmail.com>

Satu-satunya node yang berkomunikasi dengan PLC via MC Protocol.
Update: Fix Heartbeat Sync (M231 = M230) & Parsing Speed Scale.
"""

import threading
import time
from typing import Optional

import pymcprotocol
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, Int64MultiArray


# =============================================================================
# KONVERSI
# =============================================================================

def u16pair_to_u32(lo_u16, hi_u16):
    return ((hi_u16 & 0xFFFF) << 16) | (lo_u16 & 0xFFFF)


def speed_to_mc_words(speed):
    val_u32 = int(speed) & 0xFFFFFFFF
    lo_u16  = val_u32 & 0xFFFF
    hi_u16  = (val_u32 >> 16) & 0xFFFF
    lo_s16  = lo_u16 if lo_u16 < 32768 else lo_u16 - 65536
    hi_s16  = hi_u16 if hi_u16 < 32768 else hi_u16 - 65536
    return [lo_s16, hi_s16]


# =============================================================================
# NODE UTAMA
# =============================================================================

class PlcDriverNode(Node):

    def __init__(self):
        super().__init__('plc_driver_node')
        self._declare_all_parameters()
        self._load_parameters()
        self._init_state()
        self._init_ros_interfaces()
        self._client = pymcprotocol.Type3E(plctype='Q')
        self._try_connect()
        self._timer = self.create_timer(
            1.0 / self._p.cycle_hz,
            self._cycle_callback,
        )
        self.get_logger().info(
            '[ plc_driver_node ] Init selesai. '
            '{}:{} @ {} Hz | M231 Static Heartbeat Sync ACTIVE'.format(
                self._p.plc_host,
                self._p.plc_port,
                self._p.cycle_hz,
            )
        )

    # =========================================================================
    # PARAMETER
    # =========================================================================

    def _declare_all_parameters(self):
        self.declare_parameter('plc_host',                 '192.168.3.250')
        self.declare_parameter('plc_port',                 5007)
        self.declare_parameter('cycle_hz',                 20.0)
        self.declare_parameter('reconnect_s',              2.0)
        self.declare_parameter('wheel_diameter_m',         0.110)
        self.declare_parameter('wheel_separation_m',       0.240)
        self.declare_parameter('ticks_per_rev',            360000.0)
        self.declare_parameter('gear_ratio',               1.0)
        self.declare_parameter('speed_max',                100)
        self.declare_parameter('speed_min',                -100)
        self.declare_parameter('zero_crossing_cooldown_s', 0.50)
        self.declare_parameter('jump_filter_m',            0.20)
        self.declare_parameter('enc_head',                 'D40')
        self.declare_parameter('enc_read_size',            102)
        self.declare_parameter('left_lo_idx',              0)
        self.declare_parameter('left_hi_idx',              1)
        self.declare_parameter('right_lo_idx',             100)
        self.declare_parameter('right_hi_idx',             101)
        self.declare_parameter('cmd_right_head',           'D61')
        self.declare_parameter('cmd_left_head',            'D161')
        self.declare_parameter('heartbeat_head',           'M211')
        self.declare_parameter('servo_lock_head',          'M220')
        self.declare_parameter('vl_cte_head',              'D310')
        self.declare_parameter('vl_mode_bit',              'M230')
        self.declare_parameter('vl_heartbeat_bit',         'M231')
        self.declare_parameter('vl_gate_bit',              'M236')

    def _load_parameters(self):
        p = type('Params', (), {})()
        p.plc_host           = self.get_parameter('plc_host').value
        p.plc_port           = int(self.get_parameter('plc_port').value)
        p.cycle_hz           = float(self.get_parameter('cycle_hz').value)
        p.reconnect_s        = float(self.get_parameter('reconnect_s').value)
        p.wheel_diameter_m   = float(self.get_parameter('wheel_diameter_m').value)
        p.wheel_separation_m = float(self.get_parameter('wheel_separation_m').value)
        p.ticks_per_rev      = float(self.get_parameter('ticks_per_rev').value)
        p.gear_ratio         = float(self.get_parameter('gear_ratio').value)
        p.circ               = 3.141592653589793 * p.wheel_diameter_m
        p.speed_max          = int(self.get_parameter('speed_max').value)
        p.speed_min          = int(self.get_parameter('speed_min').value)
        p.jump_filter_m      = float(self.get_parameter('jump_filter_m').value)
        p.enc_head           = self.get_parameter('enc_head').value
        p.enc_read_size      = int(self.get_parameter('enc_read_size').value)
        p.left_lo_idx        = int(self.get_parameter('left_lo_idx').value)
        p.left_hi_idx        = int(self.get_parameter('left_hi_idx').value)
        p.right_lo_idx       = int(self.get_parameter('right_lo_idx').value)
        p.right_hi_idx       = int(self.get_parameter('right_hi_idx').value)
        p.cmd_right_head     = self.get_parameter('cmd_right_head').value
        p.cmd_left_head      = self.get_parameter('cmd_left_head').value
        p.heartbeat_head     = self.get_parameter('heartbeat_head').value
        p.servo_lock_head    = self.get_parameter('servo_lock_head').value
        p.vl_cte_head        = self.get_parameter('vl_cte_head').value
        p.vl_mode_bit        = self.get_parameter('vl_mode_bit').value
        p.vl_heartbeat_bit   = self.get_parameter('vl_heartbeat_bit').value
        p.vl_gate_bit        = self.get_parameter('vl_gate_bit').value
        self._p = p

    # =========================================================================
    # STATE & ROS
    # =========================================================================

    def _init_state(self):
        self._lock             = threading.Lock()
        self._cmd_right        = 0
        self._cmd_left         = 0
        self._vl_cte_raw       = 63
        self._vl_mode          = 0
        self._vl_heartbeat     = 0
        self._vl_gate          = 1   
        self._vl_speed_scale   = 1.0 # Placeholder jika nanti PID di ladder butuh baca D-register ini
        self._last_left_ticks  = None
        self._last_right_ticks = None
        self._plc_connected    = False
        self._next_reconnect   = 0.0
        self._hb_bit           = 0

    def _init_ros_interfaces(self):
        self._pub_ticks  = self.create_publisher(Int64MultiArray, '/wheel_ticks', 10)
        self._pub_status = self.create_publisher(Bool, '/plc_status', 10)
        self._sub_cmd = self.create_subscription(
            Float64MultiArray,
            '/wheel_cmd_vel',
            self._cmd_callback,
            10,
        )
        self._sub_vl = self.create_subscription(
            Float64MultiArray,
            '/agv/vl_cmd',
            self._vl_callback,
            10,
        )

    # =========================================================================
    # CALLBACKS
    # =========================================================================

    def _cmd_callback(self, msg):
        if len(msg.data) < 2:
            return
        p = self._p
        with self._lock:
            self._cmd_right = int(max(p.speed_min, min(p.speed_max, msg.data[0])))
            self._cmd_left  = int(max(p.speed_min, min(p.speed_max, msg.data[1])))

    def _vl_callback(self, msg):
        """
        Terima [cte_raw, mode, hb, gate, speed_scale] dari agv_line_follower_node.
        """
        if len(msg.data) < 4:
            return
        with self._lock:
            self._vl_cte_raw   = int(max(0, min(126, msg.data[0])))
            self._vl_mode      = int(msg.data[1])
            
            # KUNCI PERBAIKAN: Abaikan heartbeat toggle dari ROS (msg.data[2]).
            # M231 (vl_heartbeat) harus selalu SAMA dengan M230 (vl_mode) 
            # agar rangkaian Ladder tidak terputus di tengah tikungan.
            self._vl_heartbeat = self._vl_mode
            
            self._vl_gate      = int(msg.data[3])
            
            # Baca speed_scale jika elemen ke-5 tersedia (Future proofing PLC Ladder)
            if len(msg.data) >= 5:
                self._vl_speed_scale = float(msg.data[4])

    # =========================================================================
    # KONEKSI
    # =========================================================================

    def _try_connect(self):
        try:
            self._client.connect(self._p.plc_host, self._p.plc_port)
            self._plc_connected  = True
            self._next_reconnect = 0.0
            self.get_logger().info('[ PLC ] Terhubung ke {}:{}'.format(
                self._p.plc_host, self._p.plc_port))
        except Exception as exc:
            self._plc_connected  = False
            self._next_reconnect = time.time() + self._p.reconnect_s
            self.get_logger().error('[ PLC ] Gagal terhubung: {}'.format(exc))

    def _try_reconnect_if_needed(self, now):
        if not self._plc_connected and now >= self._next_reconnect:
            try:
                self._client.close()
            except Exception:
                pass
            self._client = pymcprotocol.Type3E(plctype='Q')
            self._try_connect()

    # =========================================================================
    # CYCLE — satu-satunya tempat I/O PLC
    # =========================================================================

    def _cycle_callback(self):
        now = time.time()
        p   = self._p

        self._try_reconnect_if_needed(now)

        status_msg      = Bool()
        status_msg.data = self._plc_connected
        self._pub_status.publish(status_msg)

        if not self._plc_connected:
            return

        with self._lock:
            out_r  = self._cmd_right
            out_l  = self._cmd_left
            vl_cte = self._vl_cte_raw
            vl_mod = self._vl_mode
            vl_hb  = self._vl_heartbeat
            vl_gt  = self._vl_gate

        try:
            # ── Baca encoder ──────────────────────────────────────────────
            d_all     = self._client.batchread_wordunits(
                headdevice=p.enc_head, readsize=p.enc_read_size)
            left_u32  = u16pair_to_u32(
                d_all[p.left_lo_idx],  d_all[p.left_hi_idx])
            right_u32 = u16pair_to_u32(
                d_all[p.right_lo_idx], d_all[p.right_hi_idx])

            if self._last_left_ticks is not None:
                dl_u32 = (left_u32  - self._last_left_ticks)  & 0xFFFFFFFF
                dr_u32 = (right_u32 - self._last_right_ticks) & 0xFFFFFFFF
                dl = dl_u32 if dl_u32 < 0x80000000 else dl_u32 - 0x100000000
                dr = dr_u32 if dr_u32 < 0x80000000 else dr_u32 - 0x100000000
                dl_m = (float(dl) / p.ticks_per_rev / p.gear_ratio) * p.circ
                dr_m = (float(dr) / p.ticks_per_rev / p.gear_ratio) * p.circ
                if abs(dl_m) >= p.jump_filter_m or abs(dr_m) >= p.jump_filter_m:
                    self.get_logger().warn(
                        '[ JUMP ] L={:+.3f}m R={:+.3f}m → diabaikan.'.format(
                            dl_m, dr_m))

            self._last_left_ticks  = left_u32
            self._last_right_ticks = right_u32
            ticks_msg      = Int64MultiArray()
            ticks_msg.data = [int(left_u32), int(right_u32)]
            self._pub_ticks.publish(ticks_msg)

            # ── Tulis motor cmd (dari /wheel_cmd_vel) ─────────────────────
            self._client.batchwrite_wordunits(
                headdevice=p.cmd_right_head,
                values=speed_to_mc_words(out_r))
            self._client.batchwrite_wordunits(
                headdevice=p.cmd_left_head,
                values=speed_to_mc_words(out_l))

            # ── Tulis virtual line registers ───────────────────────────────
            self._client.batchwrite_wordunits(
                headdevice=p.vl_cte_head,
                values=[vl_cte])
            
            # M230 dan M231 sekarang nilainya selalu sama agar stabil
            self._client.batchwrite_bitunits(
                headdevice=p.vl_mode_bit,
                values=[vl_mod, vl_hb])
            
            self._client.batchwrite_bitunits(
                headdevice=p.vl_gate_bit,
                values=[vl_gt])

            # ── Heartbeat sistem utama (M211) & servo lock ─────────────────
            # M211 dibiarkan tetap toggle karena ini watchdog node general (bukan spesifik VL)
            self._client.batchwrite_bitunits(
                headdevice=p.heartbeat_head,
                values=[self._hb_bit, 1])
            self._client.batchwrite_bitunits(
                headdevice=p.servo_lock_head,
                values=[1])

            self._hb_bit        = 1 - self._hb_bit
            self._plc_connected = True

        except Exception as exc:
            self._plc_connected  = False
            self._next_reconnect = now + p.reconnect_s
            self.get_logger().error(
                '[ PLC ] Error siklus: {}'.format(exc))

    # =========================================================================
    # SHUTDOWN
    # =========================================================================

    def destroy_node(self):
        self.get_logger().info('[ plc_driver_node ] Shutdown — safe exit...')
        try:
            # Tutup gate
            self._client.batchwrite_bitunits(headdevice=self._p.vl_gate_bit, values=[1])
            
            # Matikan M230 dan M231 (Mode dan VL Heartbeat) sekaligus
            self._client.batchwrite_bitunits(headdevice=self._p.vl_mode_bit, values=[0, 0])
            
            self._client.batchwrite_wordunits(headdevice=self._p.vl_cte_head, values=[63])
            self._client.batchwrite_wordunits(headdevice=self._p.cmd_right_head, values=[0, 0])
            self._client.batchwrite_wordunits(headdevice=self._p.cmd_left_head,  values=[0, 0])
            self._client.batchwrite_bitunits(headdevice=self._p.servo_lock_head, values=[0])
            
            self._client.close()
            self.get_logger().info('[ PLC ] Koneksi ditutup dengan aman.')
        except Exception:
            pass
        super().destroy_node()


# =============================================================================
# ENTRY POINT
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = PlcDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
