#!/usr/bin/env python3
"""
amr_mqtt_bridge_node.py — AMR Polebot MQTT Bridge
Broadcast semua data robot ke MQTT Broker di PC Server (FMR App)

Install: pip install paho-mqtt --break-system-packages
Run    : ros2 run amr_mqtt_bridge amr_mqtt_bridge
"""

import json, math, time, threading, socket
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int64MultiArray, Float32

import paho.mqtt.client as mqtt

# ══════════════════════════════════════════════════════════
#  KONFIGURASI — sesuaikan sebelum deploy
# ══════════════════════════════════════════════════════════
ROBOT_ID      = "AMR-01"
ROBOT_NAME    = "AMR-01"
BROKER_HOST   = "192.168.137.1"   # IP PC hotspot gateway
BROKER_PORT   = 1883
KEEPALIVE_S   = 30

# Frekuensi publish (Hz)
POSE_HZ       = 2.0
HEALTH_HZ     = 1.0
SCAN_HZ       = 1.0
ENCODER_HZ    = 2.0

# Subsample scan — kirim 1 dari N rays (hemat bandwidth)
SCAN_STEP     = 5

# QoS
QOS_BE = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
QOS_RL = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,    depth=10)


class AmrMqttBridge(Node):

    def __init__(self):
        super().__init__("amr_mqtt_bridge")
        self.get_logger().info(f"[BRIDGE] Robot={ROBOT_ID}  Broker={BROKER_HOST}:{BROKER_PORT}")

        # ── State (dilindungi lock) ───────────────────────────────
        self._lk             = threading.Lock()
        self._pose_x         = 0.0
        self._pose_y         = 0.0
        self._pose_yaw       = 0.0
        self._linear_vel     = 0.0
        self._angular_vel    = 0.0
        self._odom_x         = 0.0
        self._odom_y         = 0.0
        self._odom_yaw       = 0.0
        self._battery        = 100.0
        self._plc_ok         = False
        self._nav_state      = "Idle"
        self._heartbeat      = 0
        self._enc_left       = 0
        self._enc_right      = 0
        self._scan_ranges    = []
        self._scan_angle_min = 0.0
        self._scan_angle_inc = 0.0
        self._scan_frame     = "laser_frame"
        self._connected      = False
        self._connect_done   = False

        # ── MQTT ─────────────────────────────────────────────────
        self._mqtt = mqtt.Client(client_id=ROBOT_ID, clean_session=True)
        self._mqtt.on_connect    = self._on_connect
        self._mqtt.on_disconnect = self._on_disconnect
        self._mqtt.on_message    = self._on_message

        # Last will — robot dinyatakan offline jika koneksi putus paksa
        self._mqtt.will_set(
            f"amr/{ROBOT_ID}/identity",
            json.dumps({
                "robot_id": ROBOT_ID,
                "online"  : False,
                "timestamp": int(time.time())
            }),
            qos=1, retain=True
        )

        # ── ROS 2 Subscribers ────────────────────────────────────
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",    self._cb_amcl,    QOS_BE)
        self.create_subscription(Odometry,                  "/odom",          self._cb_odom,    QOS_BE)
        self.create_subscription(LaserScan,                 "/scan",          self._cb_scan,    QOS_BE)
        self.create_subscription(Bool,                      "/plc_status",    self._cb_plc,     QOS_RL)
        self.create_subscription(Int64MultiArray,           "/wheel_ticks",   self._cb_encoder, QOS_BE)
        self.create_subscription(Twist,                     "/cmd_vel",       self._cb_cmdvel,  QOS_BE)

        # ── ROS 2 Publishers ─────────────────────────────────────
        self._pub_goal   = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self._pub_cmdvel = self.create_publisher(Twist, "/cmd_vel", 10)

        # ── Timers ───────────────────────────────────────────────
        self.create_timer(2.0,             self._connect_once)
        self.create_timer(1.0 / POSE_HZ,   self._pub_pose)
        self.create_timer(1.0 / HEALTH_HZ, self._pub_health)
        self.create_timer(1.0 / SCAN_HZ,   self._pub_scan)
        self.create_timer(1.0 / ENCODER_HZ,self._pub_encoder)

        self.get_logger().info("[BRIDGE] Ready — waiting for MQTT connection...")

    # ══════════════════════════════════════════════════════
    #  MQTT lifecycle
    # ══════════════════════════════════════════════════════

    def _connect_once(self):
        if self._connect_done:
            return
        self._connect_done = True
        try:
            self._mqtt.connect_async(BROKER_HOST, BROKER_PORT, KEEPALIVE_S)
            self._mqtt.loop_start()
            self.get_logger().info(f"[BRIDGE] Connecting to {BROKER_HOST}:{BROKER_PORT}...")
        except Exception as e:
            self.get_logger().warn(f"[BRIDGE] Connect error: {e}")
            self._connect_done = False  # retry next cycle

    def _on_connect(self, client, userdata, flags, rc):
        if rc != 0:
            self.get_logger().error(f"[BRIDGE] MQTT connect failed rc={rc}")
            self._connect_done = False
            return

        self._connected = True
        self.get_logger().info("[BRIDGE] MQTT connected!")

        # Umumkan identitas robot (retain=True → PC tahu robot ini pernah connect)
        identity = {
            "robot_id"    : ROBOT_ID,
            "name"        : ROBOT_NAME,
            "type"        : "AMR_Differential_Drive",
            "hw"          : "NVIDIA_Jetson",
            "ros"         : "ROS2_Foxy",
            "ip"          : self._get_ip(),
            "capabilities": ["amcl_nav", "slam_mapping", "lidar_scan",
                             "encoder_odom", "plc_blvd", "mqtt_bridge"],
            "topics": {
                "publish"  : [f"amr/{ROBOT_ID}/status/pose",
                              f"amr/{ROBOT_ID}/status/health",
                              f"amr/{ROBOT_ID}/status/scan",
                              f"amr/{ROBOT_ID}/status/encoder"],
                "subscribe": [f"amr/{ROBOT_ID}/cmd/goal",
                              f"amr/{ROBOT_ID}/cmd/cancel",
                              f"amr/{ROBOT_ID}/cmd/estop"]
            },
            "online"    : True,
            "timestamp" : int(time.time())
        }
        client.publish(f"amr/{ROBOT_ID}/identity",
                       json.dumps(identity), qos=1, retain=True)
        self.get_logger().info("[BRIDGE] Identity broadcast OK")

        # Subscribe command topics
        client.subscribe([
            (f"amr/{ROBOT_ID}/cmd/goal",   1),
            (f"amr/{ROBOT_ID}/cmd/cancel", 1),
            (f"amr/{ROBOT_ID}/cmd/estop",  1),
            (f"amr/{ROBOT_ID}/cmd/mode",   0),
        ])

    def _on_disconnect(self, client, userdata, rc):
        self._connected = False
        self._connect_done = False
        self.get_logger().warn(f"[BRIDGE] Disconnected rc={rc} — will reconnect...")

    # ══════════════════════════════════════════════════════
    #  MQTT → ROS 2 (command handler)
    # ══════════════════════════════════════════════════════

    def _on_message(self, client, userdata, msg):
        topic = msg.topic
        try:
            data = json.loads(msg.payload.decode())
        except Exception:
            return

        if topic.endswith("/cmd/goal"):
            self._handle_goal(data)
        elif topic.endswith("/cmd/cancel"):
            self._handle_cancel()
        elif topic.endswith("/cmd/estop"):
            self._handle_estop()

    def _handle_goal(self, data):
        x   = float(data.get("x",   0.0))
        y   = float(data.get("y",   0.0))
        yaw = float(data.get("yaw", 0.0))
        tid = data.get("task_id", "fmr")

        self.get_logger().info(f"[BRIDGE] NAV GOAL task={tid} x={x:.3f} y={y:.3f} yaw={yaw:.3f}")

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        self._pub_goal.publish(goal)

        with self._lk:
            self._nav_state = "Navigating"

        self._publish(f"amr/{ROBOT_ID}/event/goal_accepted",
                      {"task_id": tid, "x": x, "y": y, "timestamp": int(time.time())})

    def _handle_cancel(self):
        self.get_logger().info("[BRIDGE] CANCEL navigation")
        self._zero_vel(times=3)
        with self._lk:
            self._nav_state = "Idle"
        self._publish(f"amr/{ROBOT_ID}/event/cancelled",
                      {"timestamp": int(time.time())})

    def _handle_estop(self):
        self.get_logger().warn("[BRIDGE] *** E-STOP ***")
        self._zero_vel(times=10)
        with self._lk:
            self._nav_state = "EStop"
        self._publish(f"amr/{ROBOT_ID}/event/estop",
                      {"timestamp": int(time.time()), "source": "fmr_server"})

    def _zero_vel(self, times=5):
        t = Twist()
        for _ in range(times):
            self._pub_cmdvel.publish(t)

    # ══════════════════════════════════════════════════════
    #  ROS 2 callbacks
    # ══════════════════════════════════════════════════════

    def _cb_amcl(self, msg: PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        with self._lk:
            self._pose_x   = msg.pose.pose.position.x
            self._pose_y   = msg.pose.pose.position.y
            self._pose_yaw = yaw

    def _cb_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        with self._lk:
            self._odom_x       = msg.pose.pose.position.x
            self._odom_y       = msg.pose.pose.position.y
            self._odom_yaw     = yaw
            self._linear_vel   = msg.twist.twist.linear.x
            self._angular_vel  = msg.twist.twist.angular.z

    def _cb_scan(self, msg: LaserScan):
        with self._lk:
            self._scan_ranges    = list(msg.ranges[::SCAN_STEP])
            self._scan_angle_min = msg.angle_min
            self._scan_angle_inc = msg.angle_increment * SCAN_STEP
            self._scan_frame     = msg.header.frame_id

    def _cb_plc(self, msg: Bool):
        with self._lk:
            self._plc_ok     = msg.data
            self._heartbeat += 1

    def _cb_encoder(self, msg: Int64MultiArray):
        if len(msg.data) >= 2:
            with self._lk:
                self._enc_left  = msg.data[0]
                self._enc_right = msg.data[1]

    def _cb_cmdvel(self, msg: Twist):
        with self._lk:
            self._linear_vel  = msg.linear.x
            self._angular_vel = msg.angular.z

    # ══════════════════════════════════════════════════════
    #  Periodic publishers → MQTT
    # ══════════════════════════════════════════════════════

    def _pub_pose(self):
        with self._lk:
            payload = {
                "robot_id"    : ROBOT_ID,
                "timestamp"   : int(time.time()),
                # AMCL pose (frame: map)
                "x"           : round(self._pose_x,   3),
                "y"           : round(self._pose_y,   3),
                "yaw"         : round(self._pose_yaw, 4),
                "yaw_deg"     : round(math.degrees(self._pose_yaw), 2),
                "frame"       : "map",
                # Odometry (frame: odom)
                "odom_x"      : round(self._odom_x,   3),
                "odom_y"      : round(self._odom_y,   3),
                "odom_yaw"    : round(self._odom_yaw, 4),
                # Velocity
                "linear_vel"  : round(self._linear_vel,  3),
                "angular_vel" : round(self._angular_vel, 3),
            }
        self._publish(f"amr/{ROBOT_ID}/status/pose", payload)

    def _pub_health(self):
        with self._lk:
            payload = {
                "robot_id"  : ROBOT_ID,
                "timestamp" : int(time.time()),
                "online"    : True,
                "plc_ok"    : self._plc_ok,
                "battery"   : self._battery,
                "nav_state" : self._nav_state,
                "heartbeat" : self._heartbeat,
                "ip"        : self._get_ip(),
            }
        self._publish(f"amr/{ROBOT_ID}/status/health", payload, qos=1)

    def _pub_scan(self):
        with self._lk:
            if not self._scan_ranges:
                return
            payload = {
                "robot_id"   : ROBOT_ID,
                "timestamp"  : int(time.time()),
                "angle_min"  : round(self._scan_angle_min, 4),
                "angle_inc"  : round(self._scan_angle_inc, 4),
                "ranges"     : [round(r, 2) if not math.isinf(r) else 0.0
                                for r in self._scan_ranges],
                "frame"      : self._scan_frame,
            }
        self._publish(f"amr/{ROBOT_ID}/status/scan", payload)

    def _pub_encoder(self):
        with self._lk:
            payload = {
                "robot_id"   : ROBOT_ID,
                "timestamp"  : int(time.time()),
                "enc_left"   : self._enc_left,
                "enc_right"  : self._enc_right,
                "linear_vel" : round(self._linear_vel,  3),
                "angular_vel": round(self._angular_vel, 3),
            }
        self._publish(f"amr/{ROBOT_ID}/status/encoder", payload)

    # ══════════════════════════════════════════════════════
    #  Helpers
    # ══════════════════════════════════════════════════════

    def _publish(self, topic: str, payload: dict, qos: int = 0, retain: bool = False):
        if not self._connected:
            return
        try:
            self._mqtt.publish(topic, json.dumps(payload), qos=qos, retain=retain)
        except Exception as e:
            self.get_logger().warn(f"[BRIDGE] Publish error: {e}")

    @staticmethod
    def _get_ip() -> str:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("192.168.137.1", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "unknown"

    def destroy_node(self):
        if self._connected:
            self._publish(f"amr/{ROBOT_ID}/identity",
                         {"robot_id": ROBOT_ID, "online": False,
                          "timestamp": int(time.time())},
                         qos=1, retain=True)
            self._mqtt.disconnect()
            self._mqtt.loop_stop()
        super().destroy_node()


# ══════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = AmrMqttBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
