#!/usr/bin/env python3
"""
wasd_teleop_node.py - Game-style keyboard teleop for the QBot.

Controls (terminal must have keyboard focus):
    W       forward
    A       turn left
    D       turn right
    S       backward
    SPACE   stop
    +       increase speed
    -       decrease speed
    Q       quit
"""

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# (linear_factor, angular_factor)
KEY_BINDINGS = {
    'w': ( 1.0,  0.0),
    's': (-1.0,  0.0),
    'a': ( 0.0,  1.0),
    'd': ( 0.0, -1.0),
    ' ': ( 0.0,  0.0),
}


class WASDTeleop(Node):
    def __init__(self):
        super().__init__('wasd_teleop')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_speed',  0.2)
        self.declare_parameter('angular_speed', 0.5)

        cmd_vel_topic       = self.get_parameter('cmd_vel_topic').value
        self.linear_speed   = self.get_parameter('linear_speed').value
        self.angular_speed  = self.get_parameter('angular_speed').value

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.print_instructions()

    def print_instructions(self):
        print()
        print('  +-----------------------------+')
        print('  |  WASD TELEOP                |')
        print('  |                             |')
        print('  |       W = forward           |')
        print('  |   A = left   D = right      |')
        print('  |       S = back              |')
        print('  |                             |')
        print('  |  SPACE  = stop              |')
        print('  |  +/-    = adjust speed      |')
        print('  |  Q      = quit              |')
        print('  |                             |')
        print(f'  |  linear  = {self.linear_speed:.2f} m/s         |')
        print(f'  |  angular = {self.angular_speed:.2f} rad/s      |')
        print('  +-----------------------------+')
        print()

    def publish(self, lin_factor, ang_factor):
        t = Twist()
        t.linear.x  = lin_factor * self.linear_speed
        t.angular.z = ang_factor * self.angular_speed
        self.cmd_pub.publish(t)

    def stop(self):
        self.cmd_pub.publish(Twist())


def get_key(timeout=0.1):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1).lower()
        return ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main(args=None):
    rclpy.init(args=args)
    node = WASDTeleop()

    try:
        while rclpy.ok():
            key = get_key()
            if key == 'q':
                break
            if key in KEY_BINDINGS:
                lin, ang = KEY_BINDINGS[key]
                node.publish(lin, ang)
            elif key == '+':
                node.linear_speed  = min(node.linear_speed  + 0.05, 1.0)
                node.angular_speed = min(node.angular_speed + 0.1, 2.0)
                print(f'  speed up: linear={node.linear_speed:.2f} angular={node.angular_speed:.2f}')
            elif key == '-':
                node.linear_speed  = max(node.linear_speed  - 0.05, 0.05)
                node.angular_speed = max(node.angular_speed - 0.1, 0.1)
                print(f'  speed down: linear={node.linear_speed:.2f} angular={node.angular_speed:.2f}')
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
