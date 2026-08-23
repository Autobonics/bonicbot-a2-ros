"""RPLIDAR C1M1 — Pi-direct USB, /dev/lidar (udev symlink)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Was a raw /dev/serial/by-path/... string, which breaks whenever the LiDAR
    # is plugged into a different USB port. config/udev/99-bonicbot.rules pins
    # it to /dev/lidar by vendor/product instead.
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/lidar',
        description='RPLIDAR serial device (udev symlink)',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock',
    )

    rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': 460800,
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_mode': 'Standard',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        serial_port_arg,
        use_sim_time_arg,
        rplidar,
    ])
