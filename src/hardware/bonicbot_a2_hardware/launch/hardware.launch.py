"""BonicBot A2 hardware bringup — everything that touches /dev/*.

Starts robot_state_publisher, the ros2_control controller_manager bound to the
ESP32-S3 USB CDC interface, all seven controllers, twist_mux, the RPLIDAR and
(optionally) the CSI camera.

Navigation runs separately:
    ros2 launch bonicbot_a2_nav bringup.launch.py

The simulation equivalent of this file is bonicbot_a2_sim/sim.launch.py, which
swaps the ESP interface for gz_ros2_control and is otherwise the same set of
controllers.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_share = get_package_share_directory('bonicbot_a2_hardware')
    description_share = get_package_share_directory('bonicbot_a2_description')
    nav_share = get_package_share_directory('bonicbot_a2_nav')

    use_camera_arg = DeclareLaunchArgument(
        'use_camera', default_value='true',
        description='Start the CSI camera (v4l2_camera on /dev/video0)',
    )
    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar', default_value='true',
        description='Start the RPLIDAR C1M1 (/dev/lidar)',
    )
    use_joystick_arg = DeclareLaunchArgument(
        'use_joystick', default_value='true',
        description='Start joystick teleop on the high-priority twist_mux lane',
    )
    # Forwarded to rplidar.launch.py so the LiDAR's dominant CPU cost can be
    # toggled from the top-level launch without editing files. See the note on
    # the parameter in rplidar.launch.py before setting this false.
    angle_compensate_arg = DeclareLaunchArgument(
        'angle_compensate', default_value='true',
        description='LiDAR scan angular resampling. False reclaims most of '
                    'rplidar_composition\'s CPU at some cost to scan geometry',
    )

    # ── robot description ────────────────────────────────────────
    # sim_mode:=false selects the ESP hardware interface in ros2_control.xacro.
    xacro_file = os.path.join(description_share, 'urdf', 'robot.urdf.xacro')
    # See rsp.launch.py for why ParameterValue(value_type=str) is required here:
    # without it launch_ros YAML-parses the URDF string, and any ": " in the
    # generated XML (comments included) aborts the launch with a message that
    # blames robot_description instead of the comment.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_ros2_control:=true sim_mode:=false']),
        value_type=str)

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(description_share, 'launch', 'rsp.launch.py')]),
        launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items(),
    )

    # ── ros2_control ─────────────────────────────────────────────
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            os.path.join(pkg_share, 'config', 'controllers.yaml'),
        ],
        output='screen',
    )

    def spawner(name):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                name,
                '--controller-manager-timeout', '120',
                '--switch-timeout', '50',
                # All seven spawners fire at once against a controller_manager
                # that is still cold — and on A2 it is colder than most, because
                # on_configure() opens the CDC port and sleeps 500 ms settling it
                # before the manager can serve anything.
                #
                # --service-call-timeout is per-CALL and separate from
                # --controller-manager-timeout (which only covers waiting for the
                # services to appear). Its 10s default can expire on the first
                # load_controller call, and that spawner then dies with exit
                # code 1 leaving its controller LOADED BUT NEVER CONFIGURED:
                # the node exists and the graph looks healthy, but it has no
                # command subscription. Bit M1 on diff_cont, where it silently
                # made the base undrivable.
                '--service-call-timeout', '60',
            ],
            parameters=[{'use_sim_time': False}],
        )

    # ── twist_mux ────────────────────────────────────────────────
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[os.path.join(pkg_share, 'config', 'twist_mux.yaml'),
                    {'use_sim_time': False}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')],
    )

    # ── the other /dev/* owners ──────────────────────────────────
    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch', 'rplidar.launch.py')]),
        launch_arguments={
            'angle_compensate': LaunchConfiguration('angle_compensate'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_lidar')),
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch', 'camera.launch.py')]),
        condition=IfCondition(LaunchConfiguration('use_camera')),
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(nav_share, 'launch', 'joystick.launch.py')]),
        launch_arguments={'use_sim_time': 'false'}.items(),
        condition=IfCondition(LaunchConfiguration('use_joystick')),
    )

    return LaunchDescription([
        use_camera_arg,
        use_lidar_arg,
        use_joystick_arg,
        angle_compensate_arg,
        rsp,
        controller_manager,
        spawner('diff_cont'),
        spawner('joint_broad'),
        spawner('left_arm_controller'),
        spawner('right_arm_controller'),
        spawner('head_controller'),
        spawner('left_gripper_controller'),
        spawner('right_gripper_controller'),
        twist_mux,
        rplidar,
        camera,
        joystick,
    ])
