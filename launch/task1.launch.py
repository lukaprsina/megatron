from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dis_tutorial3 = get_package_share_directory('dis_tutorial3')
    pkg_megatron = get_package_share_directory('megatron')

    # Arguments
    args = [
        DeclareLaunchArgument('world', default_value='task1_yellow_demo',
                              description='Gazebo world name'),
        DeclareLaunchArgument('map', default_value=PathJoinSubstitution(
            [pkg_megatron, 'maps', 'task1.yaml']),
            description='Map YAML file'),
        DeclareLaunchArgument('rviz', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              choices=['true', 'false']),
    ]

    # Include the full simulation + navigation stack from dis_tutorial3
    sim_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dis_tutorial3, 'launch', 'sim_turtlebot_nav.launch.py'])),
        launch_arguments=[
            ('world', LaunchConfiguration('world')),
            ('map', LaunchConfiguration('map')),
            ('rviz', LaunchConfiguration('rviz')),
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
        ],
    )

    # Face detector
    face_detector = Node(
        package='megatron',
        executable='face_detector',
        name='face_detector',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    # Ring detector
    ring_detector = Node(
        package='megatron',
        executable='ring_detector',
        name='ring_detector',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    # Mission controller
    controller = Node(
        package='megatron',
        executable='controller',
        name='mission_controller',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    ld = LaunchDescription(args)
    ld.add_action(sim_nav)
    ld.add_action(face_detector)
    ld.add_action(ring_detector)
    ld.add_action(controller)
    return ld
