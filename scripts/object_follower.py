#!/usr/bin/env python3
"""
Object follower node — follows any YOLO-detected object class.

ROS parameters (set via --ros-args -p name:=value):
  target_class    string  default: 'person'  — YOLO class name to follow
  target_width_m  float   default: 0.45      — real-world width of target (metres)
                                               used for distance estimation via bbox width

Topics:
  Sub: /vision/yolo_detections  std_msgs/String     JSON list of detections
  Sub: /camera/camera_info      sensor_msgs/CameraInfo
  Pub: /cmd_vel                 geometry_msgs/Twist
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Twist
import json
import math
import time


# ── Tunable parameters ────────────────────────────────────────────────────────
TARGET_DIST      = 1.0    # metres — desired follow distance
STOP_DIST        = 0.5    # metres — stop if closer than this
LINEAR_DEADBAND  = 0.5    # metres — tolerate ±0.5 m around TARGET_DIST (0.5–1.5 m ok)
ANGULAR_DEADBAND = math.radians(7)   # rotate if angle exceeds ±7°
MAX_LINEAR       = 0.2    # m/s
MAX_ANGULAR      = 0.2    # rad/s
KP_LINEAR        = 0.4    # proportional gain for distance error
KP_ANGULAR       = 0.3    # proportional gain for angle error
LOST_TIMEOUT     = 2.0    # seconds before stopping when target disappears


class ObjectFollower(Node):
    def __init__(self):
        super().__init__('object_follower')

        self.declare_parameter('target_class',   'person')
        self.declare_parameter('target_width_m', 0.45)

        self.target_class   = self.get_parameter('target_class').get_parameter_value().string_value
        self.target_width_m = self.get_parameter('target_width_m').get_parameter_value().double_value

        self._last_distance = None
        self._last_angle    = None
        self._last_seen     = None
        self._fx            = None
        self._img_w         = None

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String,     '/vision/yolo_detections', self._detection_cb, 10)
        self.create_subscription(CameraInfo, '/camera/camera_info',     self._camera_info_cb, 10)

        # 10 Hz control loop — runs independently of YOLO update rate
        self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            f'Object follower started '
            f'(class="{self.target_class}", width={self.target_width_m}m, '
            f'target={TARGET_DIST}m ±{LINEAR_DEADBAND}m, stop={STOP_DIST}m)')

    def _camera_info_cb(self, msg: CameraInfo):
        if self._fx is None:
            self._img_w = msg.width
            fx = msg.k[0]   # focal length in pixels
            self._fx = fx if fx > 10.0 else msg.width * 1.1   # fallback for ~90° FOV

    def _detection_cb(self, msg: String):
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        if not detections or self._fx is None:
            return

        img_w = self._img_w or 640
        fx    = self._fx

        targets = [d for d in detections if d['class'] == self.target_class]
        nearest = None
        for d in targets:
            bw_px = d['bbox'][2] * img_w   # normalised bbox width → pixels
            if bw_px < 1:
                continue
            dist      = round(self.target_width_m * fx / bw_px, 2)
            cx_offset = (d['bbox'][0] - 0.5) * img_w   # px from image centre
            angle     = round(math.atan2(cx_offset, fx), 4)
            if nearest is None or dist < nearest[0]:
                nearest = (dist, angle)

        if nearest:
            self._last_distance = nearest[0]
            self._last_angle    = nearest[1]
            self._last_seen     = time.time()

    def _control_loop(self):
        """10 Hz control loop — runs independently of YOLO update rate."""
        twist = Twist()

        if self._last_seen is None:
            return

        if time.time() - self._last_seen > LOST_TIMEOUT:
            self.cmd_pub.publish(twist)
            self._last_seen = self._last_distance = self._last_angle = None
            self.get_logger().info(f'"{self.target_class}" lost — stopped')
            return

        distance = self._last_distance
        angle    = self._last_angle

        if distance < STOP_DIST:
            self.cmd_pub.publish(twist)
            return

        dist_error = distance - TARGET_DIST

        if abs(angle) > ANGULAR_DEADBAND:
            # Not facing target — rotate only
            twist.angular.z = max(-MAX_ANGULAR, min(-KP_ANGULAR * angle, MAX_ANGULAR))
        else:
            # Facing target — adjust distance
            if dist_error > LINEAR_DEADBAND:
                twist.linear.x = min(KP_LINEAR * dist_error, MAX_LINEAR)
            elif dist_error < -LINEAR_DEADBAND:
                twist.linear.x = max(KP_LINEAR * dist_error, -MAX_LINEAR * 0.5)

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
