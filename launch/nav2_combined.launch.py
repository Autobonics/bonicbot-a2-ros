from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import subprocess

subprocess.run(["pkill", "-f", "slam_toolbox"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    my_bot_dir = get_package_share_directory('my_bot') 

    localization_launch = os.path.join(bringup_dir, 'launch', 'localization_launch.py')
    navigation_launch = os.path.join(bringup_dir, 'launch', 'navigation_launch.py')

    map_file = os.path.abspath(os.path.join(os.getcwd(), 'my_map_save.yaml'))

    rviz_config = os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'true'
        }.items()
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch),
        launch_arguments={
            'use_sim_time': 'true',
            'map_subscribe_transient_local': 'true'
        }.items()
    )

    return LaunchDescription([
        robot_state_publisher, 
        localization,
        navigation,
        rviz_node
    ])
