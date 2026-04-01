#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String, Bool, Float32
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, OccupancyGrid
import subprocess
import signal
import os
import math
import json
import threading
import time
import tf2_ros

class RobotManager(Node):
    def __init__(self):
        super().__init__('robot_manager')
        
        # Get use_sim_time parameter (automatically declared by ROS2)
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        
        # State variables
        self.mapping_active = False
        self.navigation_active = False
        self.camera_active = False
        self.explore_active = False
        self.vision_active = False
        self.precise_move_active = False
        self.follow_person_active = False
        self.slam_process = None
        self.nav2_process = None
        self.camera_process = None
        self.explore_process = None
        self.vision_process = None
        self.precise_move_process = None
        self.follow_person_process = None

        # Per-detector enabled flags (only meaningful when vision_active)
        self.yolo_enabled      = False
        self.pose_enabled      = False
        self.face_enabled      = False
        self.gesture_enabled   = False
        self.aruco_enabled     = False
        
        # Navigation state
        self.nav_status = 'idle'  # idle, navigating, goal_reached, goal_failed, cancelled
        self.current_goal = None
        self.current_pose = None
        self.distance_to_goal = 0.0

        # Named locations
        self.locations_file = os.path.expanduser('~/maps/my_map_locations.json')
        self.session_locations = {}   # in-memory; reset on each new mapping session

        # Cache the latest /map message so we can write it directly (avoids map_saver QoS issues)
        self.latest_map: OccupancyGrid = None
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, '/map', self._map_callback, map_qos)

        # TF listener for map-frame pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Action client for navigation
        self.nav_to_pose_client = None
        self.nav_goal_handle = None
        
        # Vision control topic — transient-local so vision node gets current state on startup
        vision_ctrl_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.vision_control_pub = self.create_publisher(String, '/vision/control', vision_ctrl_qos)

        # Publishers for Flutter app
        self.state_pub = self.create_publisher(String, '/robot/state', 10)
        self.mapping_status_pub = self.create_publisher(Bool, '/robot/mapping_active', 10)
        self.nav_status_pub = self.create_publisher(Bool, '/robot/navigation_active', 10)
        self.camera_status_pub = self.create_publisher(Bool, '/robot/camera_active', 10)
        self.explore_status_pub = self.create_publisher(Bool, '/robot/explore_active', 10)
        self.vision_status_pub     = self.create_publisher(Bool, '/robot/vision_active',      10)
        self.yolo_enabled_pub      = self.create_publisher(Bool, '/robot/yolo_enabled',       10)
        self.pose_enabled_pub      = self.create_publisher(Bool, '/robot/pose_enabled',       10)
        self.face_enabled_pub      = self.create_publisher(Bool, '/robot/face_enabled',       10)
        self.gesture_enabled_pub   = self.create_publisher(Bool, '/robot/gesture_enabled',    10)
        self.aruco_enabled_pub     = self.create_publisher(Bool, '/robot/aruco_enabled',      10)
        self.precise_move_active_pub    = self.create_publisher(Bool, '/robot/precise_move_active',    10)
        self.follow_person_active_pub   = self.create_publisher(Bool, '/robot/follow_person_active',   10)
        
        # Navigation status publishers
        self.nav_state_pub = self.create_publisher(String, '/robot/nav_status', 10)
        self.distance_to_goal_pub = self.create_publisher(Float32, '/robot/distance_to_goal', 10)
        self.current_goal_pub = self.create_publisher(PoseStamped, '/robot/current_goal', 10)
        
        # Subscribe to odometry for current position
        self.odom_sub = self.create_subscription(
            Odometry,
            '/diff_cont/odom',
            self.odom_callback,
            10
        )
        
        # Services for Flutter app
        self.start_mapping_srv = self.create_service(
            Trigger, '/robot/start_mapping', self.start_mapping_callback)
        self.stop_mapping_srv = self.create_service(
            Trigger, '/robot/stop_mapping', self.stop_mapping_callback)
        self.start_navigation_srv = self.create_service(
            Trigger, '/robot/start_navigation', self.start_navigation_callback)
        self.stop_navigation_srv = self.create_service(
            Trigger, '/robot/stop_navigation', self.stop_navigation_callback)
        self.save_map_srv = self.create_service(
            Trigger, '/robot/save_map', self.save_map_callback)
        self.cancel_nav_srv = self.create_service(
            Trigger, '/robot/cancel_navigation', self.cancel_navigation_callback)
        self.start_camera_srv = self.create_service(
            Trigger, '/robot/start_camera', self.start_camera_callback)
        self.stop_camera_srv = self.create_service(
            Trigger, '/robot/stop_camera', self.stop_camera_callback)
        self.start_explore_srv = self.create_service(
            Trigger, '/robot/start_explore', self.start_explore_callback)
        self.stop_explore_srv = self.create_service(
            Trigger, '/robot/stop_explore', self.stop_explore_callback)

        # Vision pipeline services
        self.create_service(Trigger, '/robot/start_vision',         self.start_vision_callback)
        self.create_service(Trigger, '/robot/stop_vision',          self.stop_vision_callback)
        self.create_service(Trigger, '/robot/enable_yolo',          self.enable_yolo_callback)
        self.create_service(Trigger, '/robot/disable_yolo',         self.disable_yolo_callback)
        self.create_service(Trigger, '/robot/enable_pose',          self.enable_pose_callback)
        self.create_service(Trigger, '/robot/disable_pose',         self.disable_pose_callback)
        self.create_service(Trigger, '/robot/enable_face',          self.enable_face_callback)
        self.create_service(Trigger, '/robot/disable_face',         self.disable_face_callback)
        self.create_service(Trigger, '/robot/enable_gesture',       self.enable_gesture_callback)
        self.create_service(Trigger, '/robot/disable_gesture',      self.disable_gesture_callback)
        self.create_service(Trigger, '/robot/enable_aruco',         self.enable_aruco_callback)
        self.create_service(Trigger, '/robot/disable_aruco',        self.disable_aruco_callback)

        # Precise motion — publish JSON to trigger: {"mode":"move","distance":0.5,"speed":0.15}
        self.create_subscription(String, '/robot/precise_move', self.precise_move_callback, 10)
        self.create_service(Trigger, '/robot/stop_precise_move',     self.stop_precise_move_callback)
        self.create_service(Trigger, '/robot/start_follow_person',   self.start_follow_person_callback)
        self.create_service(Trigger, '/robot/stop_follow_person',    self.stop_follow_person_callback)

        # Location subscriptions (publish name to topic to trigger)
        self.create_subscription(String, '/robot/save_location',   self.save_location_callback,   10)
        self.create_subscription(String, '/robot/goto_location',   self.goto_location_callback,   10)
        self.create_subscription(String, '/robot/delete_location', self.delete_location_callback, 10)

        # Publish list of saved locations (JSON string)
        self.locations_list_pub = self.create_publisher(String, '/robot/locations_list', 10)

        # Publish whether a saved map file exists on disk
        self.map_available_pub = self.create_publisher(Bool, '/robot/map_available', 10)

        # Subscribe to navigation goals
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        # Timer to publish status
        self.create_timer(0.5, self.publish_status)
        
        self.get_logger().info(f'Robot Manager Node Started (use_sim_time: {self.use_sim_time})')
        
    def odom_callback(self, msg):
        """Update current robot position"""
        self.current_pose = msg.pose.pose
        
        # distance_to_goal is updated via nav_feedback_callback (Nav2 path distance)
    
    def goal_callback(self, msg):
        """Handle new navigation goal"""
        if not self.navigation_active:
            self.get_logger().warn('Received goal but navigation is not active')
            return
        
        self.current_goal = msg
        self.nav_status = 'navigating'
        self.get_logger().info(f'New goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        
        # Send goal to Nav2
        if self.nav_to_pose_client is None:
            self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            self.nav_status = 'goal_failed'
            return
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg
        
        # Send goal
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        send_goal_future.add_done_callback(self.nav_goal_response_callback)
    
    def nav_goal_response_callback(self, future):
        """Handle navigation goal acceptance"""
        self.nav_goal_handle = future.result()
        
        if not self.nav_goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            self.nav_status = 'goal_failed'
            return
        
        self.get_logger().info('Navigation goal accepted')
        
        # Get result
        result_future = self.nav_goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)
    
    def nav_feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        self.distance_to_goal = feedback.distance_remaining
    
    def nav_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.nav_status = 'goal_reached'
            self.get_logger().info('Goal reached successfully!')
        elif status == 5:  # CANCELED
            self.nav_status = 'cancelled'
            self.get_logger().info('Navigation cancelled')
        else:  # ABORTED or other
            self.nav_status = 'goal_failed'
            self.get_logger().error('Navigation failed')
        
        self.current_goal = None
        self.distance_to_goal = 0.0
        self.nav_goal_handle = None
        
    def cancel_navigation_callback(self, request, response):
        """Cancel current navigation goal"""
        if self.nav_goal_handle is None:
            response.success = False
            response.message = 'No active navigation goal to cancel'
            return response
        
        try:
            # Cancel the goal
            cancel_future = self.nav_goal_handle.cancel_goal_async()
            self.nav_status = 'cancelled'
            self.current_goal = None
            self.distance_to_goal = 0.0
            
            response.success = True
            response.message = 'Navigation cancelled successfully'
            self.get_logger().info('Navigation cancelled by user')
            
        except Exception as e:
            response.success = False
            response.message = f'Failed to cancel navigation: {str(e)}'
            self.get_logger().error(f'Failed to cancel: {str(e)}')
        
        return response
        
    def publish_status(self):
        """Publish current robot state"""
        # System state
        state_msg = String()
        if self.explore_active:
            state_msg.data = 'exploring'
        elif self.mapping_active and self.navigation_active:
            state_msg.data = 'mapping_and_navigating'
        elif self.mapping_active:
            state_msg.data = 'mapping'
        elif self.navigation_active:
            state_msg.data = 'navigating'
        else:
            state_msg.data = 'idle'
        self.state_pub.publish(state_msg)
        
        # Individual statuses
        mapping_msg = Bool()
        mapping_msg.data = self.mapping_active
        self.mapping_status_pub.publish(mapping_msg)
        
        nav_msg = Bool()
        nav_msg.data = self.navigation_active
        self.nav_status_pub.publish(nav_msg)
        
        camera_msg = Bool()
        camera_msg.data = self.camera_active
        self.camera_status_pub.publish(camera_msg)

        explore_msg = Bool()
        explore_msg.data = self.explore_active
        self.explore_status_pub.publish(explore_msg)

        vision_msg = Bool()
        vision_msg.data = self.vision_active
        self.vision_status_pub.publish(vision_msg)

        # Auto-detect precise_move process exit
        if self.precise_move_active and self.precise_move_process is not None:
            if self.precise_move_process.poll() is not None:
                self.precise_move_process = None
                self.precise_move_active = False
                self.get_logger().info('Precise move completed')

        precise_move_msg = Bool()
        precise_move_msg.data = self.precise_move_active
        self.precise_move_active_pub.publish(precise_move_msg)

        # Auto-detect follow_person process exit
        if self.follow_person_active and self.follow_person_process is not None:
            if self.follow_person_process.poll() is not None:
                self.follow_person_process = None
                self.follow_person_active = False
                self.get_logger().info('Person follower stopped')

        follow_msg = Bool()
        follow_msg.data = self.follow_person_active
        self.follow_person_active_pub.publish(follow_msg)

        for pub, val in [
            (self.yolo_enabled_pub,     self.yolo_enabled),
            (self.pose_enabled_pub,     self.pose_enabled),
            (self.face_enabled_pub,     self.face_enabled),
            (self.gesture_enabled_pub,  self.gesture_enabled),
            (self.aruco_enabled_pub,    self.aruco_enabled),
        ]:
            m = Bool()
            m.data = val
            pub.publish(m)

        # Navigation status
        nav_state_msg = String()
        nav_state_msg.data = self.nav_status
        self.nav_state_pub.publish(nav_state_msg)
        
        # Distance to goal
        distance_msg = Float32()
        distance_msg.data = self.distance_to_goal
        self.distance_to_goal_pub.publish(distance_msg)
        
        # Current goal
        if self.current_goal:
            self.current_goal_pub.publish(self.current_goal)

        # Saved locations list
        locations_msg = String()
        locations_msg.data = json.dumps(list(self._load_locations().keys()))
        self.locations_list_pub.publish(locations_msg)

        # Map available on disk
        map_avail_msg = Bool()
        map_avail_msg.data = os.path.exists(os.path.expanduser('~/maps/my_map.yaml'))
        self.map_available_pub.publish(map_avail_msg)
    
    def start_mapping_callback(self, request, response):
        """Start SLAM Toolbox for mapping"""
        if self.mapping_active:
            response.success = False
            response.message = 'Mapping already active'
            return response
        
        try:
            from ament_index_python.packages import get_package_share_directory
            params_file = os.path.join(
                get_package_share_directory('my_bot'),
                'config', 'mapper_params_online_async.yaml'
            )
            
            self.slam_process = subprocess.Popen([
                'ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
                f'params_file:={params_file}',
                f'use_sim_time:={str(self.use_sim_time).lower()}'
            ])
            
            self.mapping_active = True
            self.session_locations = {}   # clear in-memory locations for new map session
            response.success = True
            response.message = 'Mapping started successfully'
            self.get_logger().info('SLAM Toolbox started')
            
        except Exception as e:
            response.success = False
            response.message = f'Failed to start mapping: {str(e)}'
            self.get_logger().error(f'Failed to start SLAM: {str(e)}')
        
        return response
    
    def stop_mapping_callback(self, request, response):
        """Stop SLAM Toolbox"""
        if not self.mapping_active:
            response.success = False
            response.message = 'Mapping not active'
            return response
        
        try:
            if self.slam_process:
                self.slam_process.send_signal(signal.SIGINT)
                self.slam_process.wait(timeout=5)
                self.slam_process = None
            
            self.mapping_active = False
            response.success = True
            response.message = 'Mapping stopped successfully'
            self.get_logger().info('SLAM Toolbox stopped')
            
        except Exception as e:
            response.success = False
            response.message = f'Failed to stop mapping: {str(e)}'
            self.get_logger().error(f'Failed to stop SLAM: {str(e)}')
        
        return response
    
    def start_navigation_callback(self, request, response):
        """Start Nav2"""
        if self.navigation_active:
            response.success = False
            response.message = 'Navigation already active'
            return response
        
        try:
            from ament_index_python.packages import get_package_share_directory
            
            params_file = os.path.join(
                get_package_share_directory('my_bot'),
                'config', 'nav2_params.yaml'
            )
            
            # If mapping is active, use online SLAM navigation
            if self.mapping_active:
                self.nav2_process = subprocess.Popen([
                    'ros2', 'launch', 'nav2_bringup', 'navigation_launch.py',
                    f'params_file:={params_file}',
                    f'use_sim_time:={str(self.use_sim_time).lower()}'
                ])
            else:
                # Nav2 with saved map
                map_file = os.path.expanduser('~/maps/my_map.yaml')
                if not os.path.exists(map_file):
                    response.success = False
                    response.message = 'No saved map found. Please map first or start mapping.'
                    return response
                
                self.nav2_process = subprocess.Popen([
                    'ros2', 'launch', 'nav2_bringup', 'bringup_launch.py',
                    f'map:={map_file}',
                    f'params_file:={params_file}',
                    f'use_sim_time:={str(self.use_sim_time).lower()}'
                ])
            
            # Wait for Nav2 action server to be ready (costmaps loaded)
            self.get_logger().info('Waiting for Nav2 to become ready...')
            if self.nav_to_pose_client is None:
                self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            if self.nav_to_pose_client.wait_for_server(timeout_sec=60.0):
                self.navigation_active = True
                self.nav_status = 'idle'
                response.success = True
                response.message = 'Navigation started successfully'
                self.get_logger().info('Nav2 ready')
            else:
                self.nav2_process.send_signal(signal.SIGINT)
                self.nav2_process = None
                response.success = False
                response.message = 'Nav2 failed to become ready within 60s'
                self.get_logger().error('Nav2 did not become ready in time')
            
        except Exception as e:
            response.success = False
            response.message = f'Failed to start navigation: {str(e)}'
            self.get_logger().error(f'Failed to start Nav2: {str(e)}')
        
        return response
    
    def stop_navigation_callback(self, request, response):
        """Stop Nav2"""
        if not self.navigation_active:
            response.success = False
            response.message = 'Navigation not active'
            return response
        
        try:
            # Cancel any active goal first
            if self.nav_goal_handle:
                self.nav_goal_handle.cancel_goal_async()
            
            if self.nav2_process:
                self.nav2_process.send_signal(signal.SIGINT)
                self.nav2_process.wait(timeout=10)
                self.nav2_process = None
            
            self.navigation_active = False
            self.nav_status = 'idle'
            self.current_goal = None
            self.distance_to_goal = 0.0
            self.nav_goal_handle = None
            response.success = True
            response.message = 'Navigation stopped successfully'
            self.get_logger().info('Nav2 stopped')
            
        except Exception as e:
            response.success = False
            response.message = f'Failed to stop navigation: {str(e)}'
            self.get_logger().error(f'Failed to stop Nav2: {str(e)}')
        
        return response
    
    def save_map_callback(self, request, response):
        """Save current map via SLAM Toolbox service (reliable when Nav2 also running)"""
        if not self.mapping_active:
            response.success = False
            response.message = 'Mapping not active. Start mapping first.'
            return response

        success, message = self._save_map_internal()
        response.success = success
        response.message = message
        return response
    
    def start_camera_callback(self, request, response):
        """Start camera node"""
        if self.camera_active:
            response.success = False
            response.message = 'Camera already active'
            return response
        
        try:
            # In simulation, camera is always active via camera.xacro in Gazebo
            if self.use_sim_time:
                self.camera_active = True
                response.success = True
                response.message = 'Camera activated (Gazebo camera via camera.xacro)'
                self.get_logger().info('Camera activated in simulation mode (Gazebo camera)')
            else:
                # Real hardware: launch v4l2_camera node
                # Kill any orphaned camera process from a previous session
                # (e.g. Ctrl+C on the launch file without clean shutdown)
                subprocess.run(
                    ['pkill', '-SIGTERM', '-f', 'v4l2_camera_node'],
                    capture_output=True
                )
                time.sleep(0.8)  # wait for device to be released

                from ament_index_python.packages import get_package_share_directory
                camera_launch = os.path.join(
                    get_package_share_directory('my_bot'),
                    'launch', 'camera.launch.py'
                )

                self.camera_process = subprocess.Popen([
                    'ros2', 'launch', camera_launch,
                    'use_sim_time:=false'
                ])
                
                self.camera_active = True
                response.success = True
                response.message = 'Camera started successfully (v4l2_camera hardware)'
                self.get_logger().info('Camera node started (real hardware)')
            
        except Exception as e:
            response.success = False
            response.message = f'Failed to start camera: {str(e)}'
            self.get_logger().error(f'Failed to start camera: {str(e)}')
        
        return response
    
    def stop_camera_callback(self, request, response):
        """Stop camera node"""
        if not self.camera_active:
            response.success = False
            response.message = 'Camera not active'
            return response
        
        try:
            # Auto-stop vision pipeline first (it depends on camera frames)
            if self.vision_active:
                self._stop_vision_internal()
                self.get_logger().info('Vision pipeline auto-stopped (camera stopping)')

            # In simulation, just deactivate flag (can't stop Gazebo camera)
            if self.use_sim_time:
                self.camera_active = False
                response.success = True
                response.message = 'Camera deactivated (Gazebo camera still running)'
                self.get_logger().info('Camera deactivated in simulation mode')
            else:
                # Real hardware: stop v4l2_camera process
                if self.camera_process:
                    self.camera_process.send_signal(signal.SIGINT)
                    self.camera_process.wait(timeout=5)
                    self.camera_process = None

                self.camera_active = False
                response.success = True
                response.message = 'Camera stopped successfully (v4l2_camera hardware)'
                self.get_logger().info('Camera node stopped (real hardware)')

        except Exception as e:
            response.success = False
            response.message = f'Failed to stop camera: {str(e)}'
            self.get_logger().error(f'Failed to stop camera: {str(e)}')

        return response

    def _monitor_explore_output(self):
        """Background thread: watch explore_lite output and auto-stop when done"""
        try:
            for line in self.explore_process.stdout:
                if 'Successfully returned to initial pose' in line:
                    self.get_logger().info('Exploration complete — auto-stopping explore_lite')
                    self.explore_process.send_signal(signal.SIGINT)
                    self.explore_process.wait(timeout=5)
                    self.explore_process = None
                    self.explore_active = False
                    break
        except Exception as e:
            self.get_logger().error(f'explore monitor error: {str(e)}')

    def start_explore_callback(self, request, response):
        """Start explore_lite for autonomous mapping"""
        if self.explore_active:
            response.success = False
            response.message = 'Exploration already active'
            return response

        if not self.mapping_active:
            response.success = False
            response.message = 'Start mapping first before exploring'
            return response

        if not self.navigation_active:
            response.success = False
            response.message = 'Start navigation first before exploring'
            return response

        try:
            self.explore_process = subprocess.Popen(
                [
                    'ros2', 'launch', 'explore_lite', 'explore.launch.py',
                    f'use_sim_time:={str(self.use_sim_time).lower()}',
                    'return_to_init:=true'
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True
            )
            self.explore_active = True
            threading.Thread(target=self._monitor_explore_output, daemon=True).start()
            response.success = True
            response.message = 'Autonomous exploration started'
            self.get_logger().info('explore_lite started')
        except Exception as e:
            response.success = False
            response.message = f'Failed to start exploration: {str(e)}'
            self.get_logger().error(f'Failed to start explore_lite: {str(e)}')

        return response

    def stop_explore_callback(self, request, response):
        """Stop explore_lite"""
        if not self.explore_active:
            response.success = False
            response.message = 'Exploration not active'
            return response

        try:
            if self.explore_process:
                self.explore_process.send_signal(signal.SIGINT)
                self.explore_process.wait(timeout=5)
                self.explore_process = None
            self.explore_active = False
            response.success = True
            response.message = 'Exploration stopped'
            self.get_logger().info('explore_lite stopped')
        except Exception as e:
            response.success = False
            response.message = f'Failed to stop exploration: {str(e)}'
            self.get_logger().error(f'Failed to stop explore_lite: {str(e)}')

        return response

# ── Vision pipeline callbacks ─────────────────────────────────────────────────

    def _publish_vision_control(self):
        """Publish current detector config to /vision/control (transient-local)."""
        msg = String()
        msg.data = json.dumps({
            'yolo':      self.yolo_enabled,
            'pose':      self.pose_enabled,
            'face':      self.face_enabled,
            'gesture':   self.gesture_enabled,
            'aruco':     self.aruco_enabled,
        })
        self.vision_control_pub.publish(msg)

    def start_vision_callback(self, request, response):
        if self.vision_active:
            response.success = False
            response.message = 'Vision pipeline already active'
            return response
        if not self.camera_active:
            response.success = False
            response.message = 'Start camera first before starting vision'
            return response
        try:
            self.vision_process = subprocess.Popen(
                ['ros2', 'run', 'my_bot', 'vision_pipeline.py'])
            self.vision_active = True
            self._publish_vision_control()   # send current (all-off) config on startup
            response.success = True
            response.message = 'Vision pipeline started'
            self.get_logger().info('Vision pipeline started')
        except Exception as e:
            response.success = False
            response.message = f'Failed to start vision: {str(e)}'
            self.get_logger().error(f'Failed to start vision: {str(e)}')
        return response

    def _stop_vision_internal(self):
        """Stop vision process and reset all detector flags. Safe to call even if not active."""
        if self.vision_process:
            self.vision_process.send_signal(signal.SIGINT)
            self.vision_process.wait(timeout=5)
            self.vision_process = None
        self.vision_active   = False
        self.yolo_enabled    = False
        self.pose_enabled    = False
        self.face_enabled    = False
        self.gesture_enabled = False
        self.aruco_enabled   = False

    def stop_vision_callback(self, request, response):
        if not self.vision_active:
            response.success = False
            response.message = 'Vision not active'
            return response
        try:
            self._stop_vision_internal()
            response.success = True
            response.message = 'Vision pipeline stopped'
            self.get_logger().info('Vision pipeline stopped')
        except Exception as e:
            response.success = False
            response.message = f'Failed to stop vision: {str(e)}'
            self.get_logger().error(f'Failed to stop vision: {str(e)}')
        return response

    def enable_yolo_callback(self, request, response):
        if not self.vision_active:
            response.success = False
            response.message = 'Start vision pipeline first'
            return response
        self.yolo_enabled = True
        self._publish_vision_control()
        response.success = True
        response.message = 'YOLO enabled'
        return response

    def disable_yolo_callback(self, request, response):
        self.yolo_enabled = False
        self._publish_vision_control()
        response.success = True
        response.message = 'YOLO disabled'
        return response

    def enable_pose_callback(self, request, response):
        if not self.vision_active:
            response.success = False
            response.message = 'Start vision pipeline first'
            return response
        self.pose_enabled = True
        self._publish_vision_control()
        response.success = True
        response.message = 'Body pose enabled'
        return response

    def disable_pose_callback(self, request, response):
        self.pose_enabled = False
        self._publish_vision_control()
        response.success = True
        response.message = 'Body pose disabled'
        return response

    def enable_face_callback(self, request, response):
        if not self.vision_active:
            response.success = False
            response.message = 'Start vision pipeline first'
            return response
        self.face_enabled = True
        self._publish_vision_control()
        response.success = True
        response.message = 'Face detection enabled'
        return response

    def disable_face_callback(self, request, response):
        self.face_enabled = False
        self._publish_vision_control()
        response.success = True
        response.message = 'Face detection disabled'
        return response

    def enable_gesture_callback(self, request, response):
        if not self.vision_active:
            response.success = False
            response.message = 'Start vision pipeline first'
            return response
        self.gesture_enabled = True
        self._publish_vision_control()
        response.success = True
        response.message = 'Gesture recognition enabled'
        return response

    def disable_gesture_callback(self, request, response):
        self.gesture_enabled = False
        self._publish_vision_control()
        response.success = True
        response.message = 'Gesture recognition disabled'
        return response

    def enable_aruco_callback(self, request, response):
        if not self.vision_active:
            response.success = False
            response.message = 'Start vision pipeline first'
            return response
        self.aruco_enabled = True
        self._publish_vision_control()
        response.success = True
        response.message = 'ArUco enabled'
        return response

    def disable_aruco_callback(self, request, response):
        self.aruco_enabled = False
        self._publish_vision_control()
        response.success = True
        response.message = 'ArUco disabled'
        return response

# ── Precise motion callbacks ─────────────────────────────────────────────────

    def precise_move_callback(self, msg: String):
        """Launch precise_motion.py with params from JSON string.
        Publish to /robot/precise_move: {"mode":"move","distance":0.5,"speed":0.15}
        or {"mode":"rotate","angle":90.0,"speed":30.0,"use_nav2":false}
        """
        if self.precise_move_active:
            self.get_logger().warn('Precise move already active — ignoring')
            return
        try:
            cfg = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'precise_move: bad JSON: {e}')
            return

        mode = cfg.get('mode', 'move')
        speed = cfg.get('speed', 0.0)
        use_nav2 = cfg.get('use_nav2', True)

        cmd = [
            'ros2', 'run', 'my_bot', 'precise_motion.py',
            '--ros-args',
            '-p', f'mode:={mode}',
            '-p', f'speed:={float(speed)}',
            '-p', f'use_nav2:={str(use_nav2).lower()}',
        ]

        if mode == 'move':
            distance = cfg.get('distance', 0.0)
            cmd += ['-p', f'distance:={float(distance)}']
        elif mode == 'rotate':
            angle = cfg.get('angle', 0.0)
            cmd += ['-p', f'angle:={float(angle)}']

        try:
            self.precise_move_process = subprocess.Popen(cmd)
            self.precise_move_active = True
            self.get_logger().info(f'Precise move started: {cfg}')
        except Exception as e:
            self.get_logger().error(f'Failed to start precise_motion: {e}')

    def stop_precise_move_callback(self, request, response):
        """Stop an in-progress precise move."""
        if not self.precise_move_active:
            response.success = False
            response.message = 'No precise move active'
            return response
        try:
            if self.precise_move_process:
                self.precise_move_process.send_signal(signal.SIGINT)
                self.precise_move_process.wait(timeout=3)
                self.precise_move_process = None
            self.precise_move_active = False
            response.success = True
            response.message = 'Precise move stopped'
        except Exception as e:
            response.success = False
            response.message = f'Failed to stop: {str(e)}'
        return response

# ── Person follower callbacks ─────────────────────────────────────────────────

    def start_follow_person_callback(self, request, response):
        if self.follow_person_active:
            response.success = False
            response.message = 'Person follower already active'
            return response
        if not self.camera_active:
            response.success = False
            response.message = 'Start camera first'
            return response
        if not self.vision_active:
            response.success = False
            response.message = 'Start vision pipeline first'
            return response
        if not self.yolo_enabled:
            response.success = False
            response.message = 'Enable YOLO first'
            return response
        try:
            self.follow_person_process = subprocess.Popen(
                ['ros2', 'run', 'my_bot', 'person_follower.py'])
            self.follow_person_active = True
            response.success = True
            response.message = 'Person follower started'
            self.get_logger().info('Person follower started')
        except Exception as e:
            response.success = False
            response.message = f'Failed to start person follower: {str(e)}'
            self.get_logger().error(f'Failed to start person follower: {str(e)}')
        return response

    def stop_follow_person_callback(self, request, response):
        if not self.follow_person_active:
            response.success = False
            response.message = 'Person follower not active'
            return response
        try:
            if self.follow_person_process:
                self.follow_person_process.send_signal(signal.SIGINT)
                self.follow_person_process.wait(timeout=3)
                self.follow_person_process = None
            self.follow_person_active = False
            response.success = True
            response.message = 'Person follower stopped'
            self.get_logger().info('Person follower stopped')
        except Exception as e:
            response.success = False
            response.message = f'Failed to stop: {str(e)}'
        return response

# ── Named location helpers ────────────────────────────────────────────────────

    def _map_callback(self, msg: OccupancyGrid):
        """Cache the latest map for direct file writing."""
        self.latest_map = msg

    def _get_map_pose(self):
        """Return (x, y, yaw) in map frame, or None if TF unavailable."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            t = transform.transform.translation
            q = transform.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return t.x, t.y, yaw
        except Exception as e:
            self.get_logger().error(f'TF map→base_link unavailable: {e}')
            return None

    def _save_map_internal(self):
        """Write the cached OccupancyGrid directly to .pgm + .yaml. Returns (success, message)."""
        if self.latest_map is None:
            return False, 'No map received yet — wait for SLAM to build a map'

        try:
            map_dir = os.path.expanduser('~/maps')
            os.makedirs(map_dir, exist_ok=True)
            map_path = os.path.join(map_dir, 'my_map')
            pgm_path  = map_path + '.pgm'
            yaml_path = map_path + '.yaml'

            info = self.latest_map.info
            width, height = info.width, info.height
            data = self.latest_map.data   # flat row-major, row 0 = bottom

            # Build PGM pixel array (P5 binary, 8-bit grey).
            # ROS convention: free=0→255, occupied=100→0, unknown=-1→205
            pixels = bytearray(width * height)
            for i, v in enumerate(data):
                if v == 0:
                    pixels[i] = 254
                elif v == 100:
                    pixels[i] = 0
                else:
                    pixels[i] = 205   # unknown

            # PGM is stored top-row-first; ROS data is bottom-row-first → flip
            rows = [pixels[r * width:(r + 1) * width] for r in range(height)]
            rows.reverse()

            with open(pgm_path, 'wb') as f:
                header = f'P5\n{width} {height}\n255\n'.encode()
                f.write(header)
                for row in rows:
                    f.write(row)

            origin = info.origin
            ox = origin.position.x
            oy = origin.position.y
            q = origin.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))

            yaml_content = (
                f'image: my_map.pgm\n'
                f'mode: trinary\n'
                f'resolution: {info.resolution}\n'
                f'origin: [{ox}, {oy}, {yaw}]\n'
                f'negate: 0\n'
                f'occupied_thresh: 0.65\n'
                f'free_thresh: 0.25\n'
            )
            with open(yaml_path, 'w') as f:
                f.write(yaml_content)

            self.get_logger().info(f'Map saved: {pgm_path} ({width}x{height})')
            return True, f'Map saved to {map_path}.yaml'

        except Exception as e:
            self.get_logger().error(f'Map save error: {e}')
            return False, f'Failed to save map: {str(e)}'

    def _load_locations(self):
        """Load locations JSON from disk. Returns dict (empty if missing/corrupt)."""
        try:
            if os.path.exists(self.locations_file):
                with open(self.locations_file, 'r') as f:
                    return json.load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to load locations: {e}')
        return {}

    def _write_locations(self, locations: dict):
        """Write locations dict to disk."""
        try:
            os.makedirs(os.path.dirname(self.locations_file), exist_ok=True)
            with open(self.locations_file, 'w') as f:
                json.dump(locations, f, indent=2)
        except Exception as e:
            self.get_logger().error(f'Failed to write locations: {e}')

# ── Named location callbacks ──────────────────────────────────────────────────

    def save_location_callback(self, msg: String):
        """Save current map-frame pose under the given name."""
        name = msg.data.strip()
        if not name:
            self.get_logger().warn('save_location: empty name ignored')
            return

        pose = self._get_map_pose()
        if pose is None:
            return  # error already logged in _get_map_pose

        x, y, yaw = pose

        if self.mapping_active:
            # Use in-memory session dict → overwrites file (clears old map's locations)
            self.session_locations[name] = {'x': x, 'y': y, 'yaw': yaw}
            self._write_locations(self.session_locations)
            # Auto-save the map so location and map stay in sync
            self._save_map_internal()
        else:
            # Navigation-only mode → load existing locations and add/update
            locations = self._load_locations()
            locations[name] = {'x': x, 'y': y, 'yaw': yaw}
            self._write_locations(locations)

        self.get_logger().info(
            f'Location "{name}" saved at ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°)')

    def goto_location_callback(self, msg: String):
        """Navigate to a previously saved named location."""
        name = msg.data.strip()
        locations = self._load_locations()

        if name not in locations:
            self.get_logger().error(
                f'Location "{name}" not found. Saved: {list(locations.keys())}')
            return

        if not self.navigation_active:
            self.get_logger().error('goto_location: navigation is not active')
            return

        loc = locations[name]
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = loc['x']
        goal_pose.pose.position.y = loc['y']
        yaw = loc['yaw']
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'Navigating to "{name}" ({loc["x"]:.2f}, {loc["y"]:.2f})')
        self.goal_callback(goal_pose)

    def delete_location_callback(self, msg: String):
        """Delete a named location."""
        name = msg.data.strip()
        locations = self._load_locations()

        if name not in locations:
            self.get_logger().warn(f'delete_location: "{name}" not found')
            return

        del locations[name]
        self.session_locations.pop(name, None)
        self._write_locations(locations)
        self.get_logger().info(f'Location "{name}" deleted')


def main(args=None):
    rclpy.init(args=args)
    node = RobotManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        if node.slam_process:
            node.slam_process.send_signal(signal.SIGINT)
        if node.nav2_process:
            node.nav2_process.send_signal(signal.SIGINT)
        if node.camera_process:
            node.camera_process.send_signal(signal.SIGINT)
        if node.explore_process:
            node.explore_process.send_signal(signal.SIGINT)
        if node.vision_process:
            node.vision_process.send_signal(signal.SIGINT)
        if node.precise_move_process:
            node.precise_move_process.send_signal(signal.SIGINT)
        if node.follow_person_process:
            node.follow_person_process.send_signal(signal.SIGINT)
        node.destroy_node()
        # Only shutdown if context is still valid
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()