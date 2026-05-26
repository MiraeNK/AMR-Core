#!/usr/bin/env python3

import sys

import select

import termios

import tty

import threading

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist



KEY_MAP = {

    'w': 'FORWARD',

    's': 'BACKWARD',

    'a': 'LEFT',

    'd': 'RIGHT',

    'q': 'QUIT',

}



BANNER = """

╔══════════════════════════════════════╗

║   AMR Polebot — Manual Control       ║

╠══════════════════════════════════════╣

║  W : Maju    S : Mundur              ║

║  A : Kiri    D : Kanan               ║

║  Q : Keluar                          ║

╚══════════════════════════════════════╝

"""



def get_key(timeout=0.05):

    fd = sys.stdin.fileno()

    old = termios.tcgetattr(fd)

    try:

        tty.setraw(fd)

        r, _, _ = select.select([fd], [], [], timeout)

        return sys.stdin.read(1) if r else None

    finally:

        termios.tcsetattr(fd, termios.TCSADRAIN, old)





class ManualControlNode(Node):

    def __init__(self):

        super().__init__('manual_control_node')

        self.declare_parameter('linear_speed',  0.3)

        self.declare_parameter('angular_speed', 0.8)

        self.declare_parameter('publish_hz',    10.0)



        self._lin  = float(self.get_parameter('linear_speed').value)

        self._ang  = float(self.get_parameter('angular_speed').value)

        self._hz   = float(self.get_parameter('publish_hz').value)



        self._pub  = self.create_publisher(Twist, '/cmd_vel', 10)

        self._key  = None

        self._run  = True

        self._lock = threading.Lock()



        self._timer = self.create_timer(1.0 / self._hz, self._publish)

        self._thread = threading.Thread(target=self._keyboard, daemon=True)

        self._thread.start()



        print(BANNER)

        self.get_logger().info(

            'manual_control_node aktif. lin={}m/s ang={}rad/s'.format(

                self._lin, self._ang))



    def _keyboard(self):

        while self._run:

            key = get_key()

            with self._lock:

                if key in KEY_MAP:

                    if KEY_MAP[key] == 'QUIT':

                        self._run = False

                        rclpy.shutdown()

                        break

                    self._key = KEY_MAP[key]

                else:

                    self._key = None



    def _publish(self):

        if not self._run:

            return

        msg = Twist()

        with self._lock:

            k = self._key

        if k == 'FORWARD':

            msg.linear.x =  self._lin

        elif k == 'BACKWARD':

            msg.linear.x = -self._lin

        elif k == 'LEFT':

            msg.angular.z =  self._ang

        elif k == 'RIGHT':

            msg.angular.z = -self._ang



        self._pub.publish(msg)

        label = {

            'FORWARD': 'MAJU  ',

            'BACKWARD': 'MUNDUR',

            'LEFT': 'KIRI  ',

            'RIGHT': 'KANAN ',

        }.get(k, 'STOP  ')

        sys.stdout.write('\r  [ {} ]  lin={:+.2f}  ang={:+.2f}  '.format(

            label, msg.linear.x, msg.angular.z))

        sys.stdout.flush()



    def destroy_node(self):

        self._run = False

        stop = Twist()

        self._pub.publish(stop)

        super().destroy_node()





def main(args=None):

    rclpy.init(args=args)

    node = ManualControlNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        try:

            rclpy.shutdown()

        except Exception:

            pass

    print()





if __name__ == '__main__':

    main()

