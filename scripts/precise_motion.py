#!/usr/bin/env python3
"""Precise distance or rotation movement using EKF odometry (/odometry/filtered).

Parameters
----------
mode       : 'move' | 'rotate'  (required)
distance   : metres   — used when mode=move  (+forward, -backward)
angle      : degrees  — used when mode=rotate (+CCW / anti-clockwise, -CW / clockwise)
speed      : m/s for move (default 0.15), °/s for rotate (default 30.0)

Terminal usage
--------------
# Move forward 1 m
ros2 run my_bot precise_motion --ros-args -p mode:=move -p distance:=1.0

# Move backward 0.5 m at 0.1 m/s
ros2 run my_bot precise_motion --ros-args -p mode:=move -p distance:=-0.5 -p speed:=0.1

# Rotate 90° counterclockwise (anti-clockwise)
ros2 run my_bot precise_motion --ros-args -p mode:=rotate -p angle:=90.0

# Rotate 90° clockwise
ros2 run my_bot precise_motion --ros-args -p mode:=rotate -p angle:=-90.0

# Rotate 45° CW at 20 °/s
ros2 run my_bot precise_motion --ros-args -p mode:=rotate -p angle:=-45.0 -p speed:=20.0
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# ── helpers ──────────────────────────────────────────────────────────────────

def _quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap(angle: float) -> float:
    """Normalise angle to (−π, π]."""
    while angle >  math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle


# ── node ─────────────────────────────────────────────────────────────────────

class PreciseMotion(Node):
    def __init__(self):
        super().__init__('precise_motion')

        self.declare_parameter('mode',     'move')
        self.declare_parameter('distance', 1.0)    # metres
        self.declare_parameter('angle',    90.0)   # degrees
        self.declare_parameter('speed',    0.0)    # 0 = use default per mode

        self._mode  = self.get_parameter('mode').value
        self._done  = False
        self._start = None   # (x, y) for move | yaw for rotate

        raw_speed = self.get_parameter('speed').value

        if self._mode == 'move':
            self._target = self.get_parameter('distance').value
            self._speed  = abs(raw_speed) if raw_speed != 0.0 else 0.4
            direction = 'forward' if self._target >= 0 else 'backward'
            self.get_logger().info(
                f'[move] {abs(self._target):.3f} m {direction} @ {self._speed:.2f} m/s'
            )

        elif self._mode == 'rotate':
            angle_deg    = self.get_parameter('angle').value
            self._target = math.radians(angle_deg)
            self._speed  = math.radians(abs(raw_speed)) if raw_speed != 0.0 \
                           else math.radians(45.0)
            direction = 'CCW (anti-clockwise)' if angle_deg >= 0 else 'CW (clockwise)'
            self.get_logger().info(
                f'[rotate] {abs(angle_deg):.1f}° {direction} '
                f'@ {math.degrees(self._speed):.1f} °/s'
            )

        else:
            self.get_logger().fatal(f"Unknown mode '{self._mode}'. Use 'move' or 'rotate'.")
            raise SystemExit(1)

        self._cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self._odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self._odom_cb, 10)

    # ── callbacks ────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        if self._done:
            return

        if self._mode == 'move':
            self._handle_move(msg)
        else:
            self._handle_rotate(msg)

    def _handle_move(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self._start is None:
            self._start = (x, y)
            self.get_logger().info('Start pose latched.')
            return

        traveled = math.sqrt((x - self._start[0]) ** 2 + (y - self._start[1]) ** 2)

        if traveled >= abs(self._target):
            self._stop(f'Done. Target {abs(self._target):.3f} m, actual {traveled:.3f} m')
        else:
            twist = Twist()
            twist.linear.x = self._speed if self._target >= 0 else -self._speed
            self._cmd_pub.publish(twist)

    def _handle_rotate(self, msg: Odometry):
        yaw = _quat_to_yaw(msg.pose.pose.orientation)

        if self._start is None:
            self._start = yaw
            self.get_logger().info(f'Start yaw latched: {math.degrees(yaw):.1f}°')
            return

        rotated = _wrap(yaw - self._start)
        reached = (self._target >= 0 and rotated >= self._target) or \
                  (self._target <  0 and rotated <= self._target)

        if reached:
            self._stop(
                f'Done. Target {math.degrees(self._target):.1f}°, '
                f'actual {math.degrees(rotated):.1f}°'
            )
        else:
            twist = Twist()
            twist.angular.z = self._speed if self._target >= 0 else -self._speed
            self._cmd_pub.publish(twist)

    def _stop(self, msg: str):
        self._cmd_pub.publish(Twist())
        self._done = True
        self.get_logger().info(msg)


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = PreciseMotion()

    while rclpy.ok() and not node._done:
        rclpy.spin_once(node, timeout_sec=0.05)

    node._cmd_pub.publish(Twist())   # guarantee stop on exit
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
