#!/usr/bin/env python3
"""
state_machine_node.py — Delivery Bot state machine with full transitions.

Cycle: WAITING -> SET_DESTINATION -> SIGNAL_MOVE -> MOVING ->
       DELIVERY_READY -> DELIVER -> RETURN_HOME -> WAITING
With YIELD_STOP if an obstacle appears during MOVING or RETURN_HOME.

Send a destination:
    ros2 topic pub --once /destination geometry_msgs/msg/Point \
        "{x: 2.0, y: 0.0, z: 0.0}"
"""

import math
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class RobotState(Enum):
    WAITING = "WAITING"
    SET_DESTINATION = "SET_DESTINATION"
    SIGNAL_MOVE = "SIGNAL_MOVE"
    MOVING = "MOVING"
    YIELD_STOP = "YIELD_STOP"
    DELIVERY_READY = "DELIVERY_READY"
    DELIVER = "DELIVER"
    RETURN_HOME = "RETURN_HOME"
    ERROR = "ERROR"


class DeliveryBotStateMachine(Node):
    def __init__(self):
        super().__init__('delivery_bot_state_machine')

        # --- Parameters (no hardcoding) -------------------------------
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('state_topic', '/robot_state')
        self.declare_parameter('destination_topic', '/destination')

        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('obstacle_distance', 0.6)
        self.declare_parameter('signal_duration', 2.0)
        self.declare_parameter('handoff_duration', 3.0)
        self.declare_parameter('deliver_duration', 2.0)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        odom_topic    = self.get_parameter('odom_topic').value
        scan_topic    = self.get_parameter('scan_topic').value
        state_topic   = self.get_parameter('state_topic').value
        dest_topic    = self.get_parameter('destination_topic').value

        self.linear_speed     = self.get_parameter('linear_speed').value
        self.angular_speed    = self.get_parameter('angular_speed').value
        self.goal_tolerance   = self.get_parameter('goal_tolerance').value
        self.obstacle_distance= self.get_parameter('obstacle_distance').value
        self.signal_duration  = self.get_parameter('signal_duration').value
        self.handoff_duration = self.get_parameter('handoff_duration').value
        self.deliver_duration = self.get_parameter('deliver_duration').value

        # --- Pubs / Subs ---------------------------------------------
        self.cmd_pub   = self.create_publisher(Twist,  cmd_vel_topic, 10)
        self.state_pub = self.create_publisher(String, state_topic,  10)
        self.create_subscription(Odometry,  odom_topic, self.odom_callback,        10)
        self.create_subscription(LaserScan, scan_topic, self.scan_callback,        10)
        self.create_subscription(Point,     dest_topic, self.destination_callback, 10)

        # --- Internal state ------------------------------------------
        self.state = RobotState.WAITING
        self.previous_state = None
        self.latest_odom = None
        self.latest_scan = None
        self.goal = None
        self.home = Point()
        self.timer_start = None

        self.create_timer(0.1, self.tick)
        self.get_logger().info(f'Started. State = {self.state.value}')

    # --- Callbacks ---------------------------------------------------
    def odom_callback(self, msg): self.latest_odom = msg
    def scan_callback(self, msg): self.latest_scan = msg

    def destination_callback(self, msg):
        if self.state == RobotState.WAITING:
            self.goal = Point(x=msg.x, y=msg.y, z=msg.z)
            self.get_logger().info(f'Destination: ({msg.x:.2f}, {msg.y:.2f})')
        else:
            self.get_logger().warn(f'Busy ({self.state.value}), ignoring destination')

    # --- Main loop ---------------------------------------------------
    def tick(self):
        handlers = {
            RobotState.WAITING:         self.handle_waiting,
            RobotState.SET_DESTINATION: self.handle_set_destination,
            RobotState.SIGNAL_MOVE:     self.handle_signal_move,
            RobotState.MOVING:          self.handle_moving,
            RobotState.YIELD_STOP:      self.handle_yield_stop,
            RobotState.DELIVERY_READY:  self.handle_delivery_ready,
            RobotState.DELIVER:         self.handle_deliver,
            RobotState.RETURN_HOME:     self.handle_return_home,
            RobotState.ERROR:           self.handle_error,
        }
        handlers[self.state]()
        self.publish_state()

    # --- Helpers -----------------------------------------------------
    def transition_to(self, new_state):
        if new_state != self.state:
            self.get_logger().info(f'{self.state.value} -> {new_state.value}')
            if self.state in (RobotState.MOVING, RobotState.RETURN_HOME):
                self.previous_state = self.state
            self.state = new_state
            self.timer_start = self.get_clock().now()

    def publish_state(self):
        msg = String(); msg.data = self.state.value
        self.state_pub.publish(msg)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def drive(self, linear, angular):
        t = Twist(); t.linear.x = float(linear); t.angular.z = float(angular)
        self.cmd_pub.publish(t)

    def elapsed_in_state(self):
        if self.timer_start is None: return 0.0
        return (self.get_clock().now() - self.timer_start).nanoseconds / 1e9

    def distance_to(self, target):
        if self.latest_odom is None: return float('inf')
        dx = target.x - self.latest_odom.pose.pose.position.x
        dy = target.y - self.latest_odom.pose.pose.position.y
        return math.sqrt(dx*dx + dy*dy)

    def heading(self):
        if self.latest_odom is None: return 0.0
        q = self.latest_odom.pose.pose.orientation
        return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def drive_toward(self, target):
        if self.latest_odom is None: return
        cx = self.latest_odom.pose.pose.position.x
        cy = self.latest_odom.pose.pose.position.y
        target_heading = math.atan2(target.y - cy, target.x - cx)
        err = target_heading - self.heading()
        while err >  math.pi: err -= 2*math.pi
        while err < -math.pi: err += 2*math.pi
        if abs(err) > 0.3:
            self.drive(0.0, self.angular_speed * (1 if err > 0 else -1))
        else:
            self.drive(self.linear_speed, err)

    def obstacle_ahead(self):
        if self.latest_scan is None: return False
        r = self.latest_scan.ranges
        n = len(r)
        if n == 0: return False
        front = n // 2
        window = n // 12
        valid = [x for x in r[front-window:front+window] if x > 0.05]
        return bool(valid) and min(valid) < self.obstacle_distance

    # --- State handlers ---------------------------------------------
    def handle_waiting(self):
        self.stop_robot()
        if self.goal is not None:
            self.transition_to(RobotState.SET_DESTINATION)

    def handle_set_destination(self):
        self.stop_robot()
        self.transition_to(RobotState.SIGNAL_MOVE)

    def handle_signal_move(self):
        self.stop_robot()
        if self.elapsed_in_state() >= self.signal_duration:
            self.transition_to(RobotState.MOVING)

    def handle_moving(self):
        if self.obstacle_ahead():
            self.transition_to(RobotState.YIELD_STOP); return
        if self.distance_to(self.goal) < self.goal_tolerance:
            self.transition_to(RobotState.DELIVERY_READY); return
        self.drive_toward(self.goal)

    def handle_yield_stop(self):
        self.stop_robot()
        if not self.obstacle_ahead():
            resume = self.previous_state or RobotState.MOVING
            self.transition_to(resume)

    def handle_delivery_ready(self):
        self.stop_robot()
        if self.elapsed_in_state() >= self.handoff_duration:
            self.transition_to(RobotState.DELIVER)

    def handle_deliver(self):
        self.stop_robot()
        if self.elapsed_in_state() >= self.deliver_duration:
            self.get_logger().info('Delivered. Returning home.')
            self.goal = self.home
            self.transition_to(RobotState.RETURN_HOME)

    def handle_return_home(self):
        if self.obstacle_ahead():
            self.transition_to(RobotState.YIELD_STOP); return
        if self.distance_to(self.home) < self.goal_tolerance:
            self.get_logger().info('Home.')
            self.goal = None
            self.transition_to(RobotState.WAITING); return
        self.drive_toward(self.home)

    def handle_error(self):
        self.stop_robot()


def main(args=None):
    rclpy.init(args=args)
    node = DeliveryBotStateMachine()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
