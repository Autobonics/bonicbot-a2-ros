import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_bot'

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get paths
    pkg_path = get_package_share_directory(package_name)
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(pkg_path, 'config', 'view_bot.rviz')

    # Process URDF
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=false sim_mode:=false'])
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}

    # Node: Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Node: Joint State Publisher GUI
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Node: RViz2
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz
    ])
