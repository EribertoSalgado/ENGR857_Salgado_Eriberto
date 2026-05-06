#!/usr/bin/env python3
"""
intent_communication_node.py — Visual + signal feedback per robot state.

Subscribes:
    /robot_state  (std_msgs/String)

Publishes:
    /intent/color    (std_msgs/ColorRGBA)  — RGB intent for LEDs
    /intent/display  (std_msgs/String)     — short text for display screens

Also prints a colored banner in the terminal so you can see it work
without any hardware.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ColorRGBA


# State -> (R, G, B, A), display text, ANSI color, emoji
STATE_INTENT = {
    'WAITING':         (1.0, 1.0, 1.0, 1.0, 'READY',          '\033[97m', 'O'),
    'SET_DESTINATION': (1.0, 0.8, 0.0, 1.0, 'PREPARING',      '\033[93m', '*'),
    'SIGNAL_MOVE':     (1.0, 0.8, 0.0, 1.0, 'ABOUT TO MOVE',  '\033[93m', '!'),
    'MOVING':          (0.0, 1.0, 0.0, 1.0, 'MOVING',         '\033[92m', '>'),
    'YIELD_STOP':      (1.0, 0.0, 0.0, 1.0, 'YIELDING',       '\033[91m', 'X'),
    'DELIVERY_READY':  (0.0, 0.5, 1.0, 1.0, 'DELIVERY READY', '\033[94m', '#'),
    'DELIVER':         (1.0, 0.0, 1.0, 1.0, 'DELIVERING',     '\033[95m', '$'),
    'RETURN_HOME':     (0.0, 1.0, 0.5, 1.0, 'RETURNING HOME', '\033[96m', '<'),
    'ERROR':           (1.0, 0.0, 0.0, 1.0, 'ERROR',          '\033[91m', '!'),
}

ANSI_RESET = '\033[0m'


class IntentCommunication(Node):
    def __init__(self):
        super().__init__('intent_communication')

        self.declare_parameter('state_topic',   '/robot_state')
        self.declare_parameter('color_topic',   '/intent/color')
        self.declare_parameter('display_topic', '/intent/display')

        state_topic   = self.get_parameter('state_topic').value
        color_topic   = self.get_parameter('color_topic').value
        display_topic = self.get_parameter('display_topic').value

        self.create_subscription(String, state_topic, self.state_callback, 10)
        self.color_pub   = self.create_publisher(ColorRGBA, color_topic, 10)
        self.display_pub = self.create_publisher(String,    display_topic, 10)

        self.last_state = None

        self.get_logger().info('Intent Communication started.')

    def state_callback(self, msg):
        state = msg.data
        if state == self.last_state:
            return  # only react on state change
        self.last_state = state

        intent = STATE_INTENT.get(state)
        if intent is None:
            self.get_logger().warn(f'Unknown state: {state}')
            return

        r, g, b, a, display_text, ansi, marker = intent

        # Publish color (for hardware LED driver later)
        color = ColorRGBA()
        color.r = r
        color.g = g
        color.b = b
        color.a = a
        self.color_pub.publish(color)

        # Publish display text (for screen driver later)
        disp = String()
        disp.data = display_text
        self.display_pub.publish(disp)

        # Visual feedback in terminal
        print()
        print(f'  {ansi}+----------------------------------------+{ANSI_RESET}')
        print(f'  {ansi}|  [{marker}]  STATE: {state:<22s} |{ANSI_RESET}')
        print(f'  {ansi}|       -> {display_text:<29s} |{ANSI_RESET}')
        print(f'  {ansi}+----------------------------------------+{ANSI_RESET}')
        print()


def main(args=None):
    rclpy.init(args=args)
    node = IntentCommunication()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()