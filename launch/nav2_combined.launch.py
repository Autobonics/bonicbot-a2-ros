from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os
import subprocess

subprocess.run(["pkill", "-f", "slam_toolbox"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    my_bot_dir = get_package_share_directory('my_bot') 

    use_sim_time = LaunchConfiguration('use_sim_time')

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    localization_launch = os.path.join(bringup_dir, 'launch', 'localization_launch.py')
    navigation_launch = os.path.join(bringup_dir, 'launch', 'navigation_launch.py')

    map_file = os.path.abspath(os.path.join(os.getcwd(), 'my_map_save.yaml'))

    rviz_config = os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz')

    # Logic to select the correct params file
    params_file = PythonExpression([
        "'", os.path.join(my_bot_dir, 'config', 'nav2_params_sim.yaml'), "'",
        " if '", use_sim_time, "' == 'true' else ",
        "'", os.path.join(my_bot_dir, 'config', 'nav2_params.yaml'), "'"
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'map_subscribe_transient_local': 'true'
        }.items()
    )

    return LaunchDescription([
        sim_time_arg,
        robot_state_publisher, 
        localization,
        navigation,
        rviz_node
    ])
