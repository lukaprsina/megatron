from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('visualization', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('show_debug_window', default_value='false',
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

    visualizer = Node(
        package='megatron',
        executable='perception_visualizer',
        name='perception_visualizer',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'show_window': LaunchConfiguration('show_debug_window'),
        }],
        condition=IfCondition(LaunchConfiguration('visualization')),
    )

    ld = LaunchDescription(args)
    ld.add_action(face_detector)
    ld.add_action(ring_detector)
    ld.add_action(visualizer)
    return ld
