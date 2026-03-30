import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node



def generate_launch_description():

    package_name='my_bot' 
    
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )

    # Gazebo Sim (Ignition Fortress)
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    # Using -r (run) and a default empty world
                    launch_arguments={'gz_args': '-r empty.sdf'}.items()
             )

    # Spawn the robot in Gazebo Sim
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'my_bot',
                                   '-z', '0.1'], # Spawn slightly above ground to avoid clipping
                        output='screen')


    # Bridge for Clock and Sensors
    # /clock -> ROS clock
    # /scan -> sensor_msgs/LaserScan
    # /camera/image_raw -> sensor_msgs/Image
    # /imu/data -> sensor_msgs/Imu
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        output='screen'
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # Humanoid controller spawners
    left_arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller"],
    )

    right_arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller"],
    )

    head_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["head_controller"],
    )

    left_gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_gripper_controller"],
    )

    right_gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_gripper_controller"],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory(package_name), 'config', 'ekf.yaml'), {'use_sim_time': True}]
    )

   
    return LaunchDescription([
        rsp,
        joystick,
        twist_mux,
        gazebo,
        spawn_entity,
        bridge,
        diff_drive_spawner,
        joint_broad_spawner,
        left_arm_spawner,
        right_arm_spawner,
        head_spawner,
        left_gripper_spawner,
        right_gripper_spawner,
        ekf_node
    ])
