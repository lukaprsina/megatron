from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              choices=['true', 'false']),
    ]

    face_detector = Node(
        package='megatron',
        executable='face_detector',
        name='face_detector',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    ring_detector = Node(
        package='megatron',
        executable='ring_detector',
        name='ring_detector',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    ld = LaunchDescription(args)
    ld.add_action(face_detector)
    ld.add_action(ring_detector)
    return ld
