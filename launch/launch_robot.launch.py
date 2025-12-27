import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_bot'
    
    # Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), 
        launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )
    
    # Joystick (optional - uncomment if needed)
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'joystick.launch.py'
        )]), 
        launch_arguments={'use_sim_time': 'false'}.items()
    )
    
    # Twist Mux
    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )
    
    # Get robot description dynamically
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    
    # Controller configuration file
    controller_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'my_controllers.yaml')
    
    # Controller Manager Node
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )
    
    # Delay controller manager to ensure robot_state_publisher is ready
    delayed_controller_manager = TimerAction(period=5.0, actions=[controller_manager])
    
    # Diff Drive Controller Spawner
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )
    
    # Wait for controller_manager before spawning diff_drive
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )
    
    # Joint Broadcaster Spawner
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )
    
    # Wait for controller_manager before spawning joint_broad
    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )
    
    # Servo Position Controller Spawners
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
    
    # Wait for controller_manager before spawning servo controllers
    delayed_servo_spawners = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[
                left_arm_spawner,
                right_arm_spawner,
                head_spawner,
                left_gripper_spawner,
                right_gripper_spawner,
            ],
        )
    )
    
    # rplidar = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory(package_name), 'launch', 'rplidar.launch.py'
    #     )])
    # )

    return LaunchDescription([
        rsp,
        joystick,
        twist_mux,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        delayed_servo_spawners,
        # rplidar
    ])