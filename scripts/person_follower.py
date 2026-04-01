#!/usr/bin/env python3
"""
Person follower node — subscribes to /vision/nearest_person and drives
the robot to maintain a target distance from the nearest detected person.

Control:
  Linear  speed  ∝ (distance − TARGET_DIST)   — approach / back off
  Angular speed  ∝ −angle                      — turn to centre person

Stops automatically when:
  - No person detected for > LOST_TIMEOUT seconds
  - Person is closer than STOP_DIST metres

Topics:
  Sub: /vision/nearest_person  std_msgs/String  JSON
  Pub: /cmd_vel                geometry_msgs/Twist
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
import math
import time


# ── Tunable parameters ────────────────────────────────────────────────────────
TARGET_DIST      = 1.0    # metres — desired follow distance
STOP_DIST        = 0.5    # metres — stop if closer than this
LINEAR_DEADBAND  = 0.5    # metres — tolerate ±0.5 m around TARGET_DIST (0.5–1.5 m ok)
ANGULAR_DEADBAND = math.radians(10)  # rad — don't turn if within ±10° of centre
MAX_LINEAR       = 0.2    # m/s
MAX_ANGULAR      = 0.2    # rad/s
KP_LINEAR        = 0.4    # proportional gain for distance error
KP_ANGULAR       = 0.3    # proportional gain for angle error
LOST_TIMEOUT     = 2.0    # seconds before stopping when person disappears


class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')

        self._last_distance  = None
        self._last_angle     = None
        self._last_seen      = None   # time of last valid detection

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String, '/vision/nearest_person', self._cb, 10)

        # Control loop at 10 Hz — re-publishes last known velocity between
        # slow YOLO updates so the robot moves smoothly instead of jerkily.
        self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            f'Person follower started '
            f'(target={TARGET_DIST}m ±{LINEAR_DEADBAND}m, '
            f'stop={STOP_DIST}m, angle deadband=±{math.degrees(ANGULAR_DEADBAND):.0f}°)')

    def _cb(self, msg: String):
        """Store the latest detection; actual control is in _control_loop."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        if data is None:
            return

        self._last_distance  = data.get('distance', 0.0)
        self._last_angle     = data.get('angle',    0.0)
        self._last_seen      = time.time()

    def _control_loop(self):
        """10 Hz control loop — runs independently of YOLO update rate."""
        twist = Twist()

        # No detection yet
        if self._last_seen is None:
            return

        # Person lost — stop and reset
        if time.time() - self._last_seen > LOST_TIMEOUT:
            self.cmd_pub.publish(twist)
            self._last_seen      = None
            self._last_distance  = None
            self._last_angle     = None
            self.get_logger().info('Person lost — stopped')
            return

        distance = self._last_distance
        angle    = self._last_angle

        if distance < STOP_DIST:
            self.cmd_pub.publish(twist)
            return

        dist_error = distance - TARGET_DIST

        if abs(angle) > ANGULAR_DEADBAND:
            twist.angular.z = max(-MAX_ANGULAR, min(-KP_ANGULAR * angle, MAX_ANGULAR))

        if dist_error > LINEAR_DEADBAND:
            twist.linear.x = min(KP_LINEAR * dist_error, MAX_LINEAR)
        elif dist_error < -LINEAR_DEADBAND:
            twist.linear.x = max(KP_LINEAR * dist_error, -MAX_LINEAR * 0.5)

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
