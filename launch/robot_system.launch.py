import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_name = 'my_bot'
    
    sim_arg = DeclareLaunchArgument("use_sim_time", default_value="false", description="Set true to launch in simulation mode")
    sim = LaunchConfiguration("use_sim_time")

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'launch_sim.launch.py'
        )]),
        condition=IfCondition(sim)
    )

    # Robot hardware (always running)
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'launch_robot.launch.py'
        )]),
        condition=UnlessCondition(sim)
    )
    
    # RPLidar (always running)
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rplidar.launch.py'
        )]),
        condition=UnlessCondition(sim)
    )
    
    # ROS Bridge Server (always running) - launched as Node instead of XML
    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': 9090,
            'address': '0.0.0.0',
            'use_sim_time': sim
        }]
    )
    
    # Robot Manager Node (always running)
    robot_manager = Node(
        package='my_bot',
        executable='robot_manager.py',
        name='robot_manager',
        output='screen',
        parameters=[{
            'use_sim_time': sim
        }]
    )
    
    return LaunchDescription([
        sim_arg,
        sim_launch,
        robot_launch,
        rplidar_launch,
        rosbridge_server,
        robot_manager
    ])