"""BonicBot A2 head camera — RPi CSI module on /dev/video0, via v4l2_camera.

Named `face_camera`, matching bonicbot_m1_hardware_bringup/usb_cameras.launch.py.
A2 fits exactly one camera and it sits in the head (camera_joint parents to
`head`), so it is the same thing M1 calls its face camera. Sharing the topic
name means robot_app addresses both series identically — its per-series
`cameras` map has a "face" entry either way, and a WebRTC client sees the same
track name on both robots.

M1 additionally has `docking_camera` and `depth_camera`; A2 has neither.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    video_device_arg = DeclareLaunchArgument(
        'video_device', default_value='/dev/video0',
        description='v4l2 capture device for the CSI camera',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock',
    )

    face_camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='face_camera',
        namespace='face_camera',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_size': [640, 480],
            # 6 fps: enough for YOLO on an RPi4, and the CPU headroom matters
            # more than frame rate with Nav2 running alongside.
            'time_per_frame': [1, 6],
            'camera_name': 'face_camera',
            'camera_frame_id': 'face_camera_link_optical',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        video_device_arg,
        use_sim_time_arg,
        face_camera,
    ])
