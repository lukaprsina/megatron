from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Careful, thes are the ./install directory paths, rebuild
    pkg_dis_tutorial3 = get_package_share_directory('dis_tutorial3')
    pkg_megatron = get_package_share_directory('megatron')

    # Arguments
    args = [
        DeclareLaunchArgument('world', default_value='task1_yellow_demo',
                              description='Gazebo world name'),
        DeclareLaunchArgument('map', default_value=PathJoinSubstitution(
            [pkg_megatron, 'maps', 'task1.yaml']),
            description='Map YAML file'),
        DeclareLaunchArgument('launch_rviz', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('visualization', default_value='true',
                      choices=['true', 'false'],
                      description='Start Megatron perception visualizer'),
        DeclareLaunchArgument('show_debug_window', default_value='false',
                      choices=['true', 'false'],
                      description='Show the combined perception panel in an OpenCV window'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('rviz_config', default_value=PathJoinSubstitution(
            [pkg_megatron, 'config', 'production.rviz']),
            description='RViz config file'),
    ]

    # Include the full simulation + navigation stack from dis_tutorial3
    # Use megatron/config/nav2.yaml as the nav2 params file
    sim_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_megatron, 'launch', 'sim_turtlebot_nav.launch.py'])),
        launch_arguments=[
            ('world', LaunchConfiguration('world')),
            ('map', LaunchConfiguration('map')),
            ('rviz', 'false'), # we will configure it manually
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('nav2_config', PathJoinSubstitution([pkg_megatron, 'config', 'nav2.yaml'])),
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution(
            [pkg_megatron, 'config', LaunchConfiguration('rviz_config')])],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
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
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'waypoints_file': PathJoinSubstitution([pkg_megatron, 'waypoints', 'test1.yaml']),

        }],
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
    ld.add_action(sim_nav)
    ld.add_action(rviz)
    ld.add_action(face_detector)
    ld.add_action(ring_detector)
    ld.add_action(controller)
    ld.add_action(visualizer)
    return ld
