from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Careful, these are the ./install directory paths, rebuild
    pkg_megatron = get_package_share_directory('megatron')
    pkg_dis_tutorial3 = get_package_share_directory('dis_tutorial3')

    # Arguments
    args = [
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Robot namespace (leave empty for default Turtlebot topics)',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([pkg_megatron, 'maps', 'task1r.yaml']),
            description='Map YAML file',
        ),
        DeclareLaunchArgument(
            'nav2_config',
            default_value=PathJoinSubstitution([pkg_megatron, 'config', 'nav2.yaml']),
            description='Nav2 parameters file',
        ),
        DeclareLaunchArgument(
            'localization_config',
            default_value=PathJoinSubstitution([pkg_megatron, 'config', 'localization.yaml']),
            description='Localization parameters file (AMCL/map_server)',
        ),
        DeclareLaunchArgument('launch_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'visualization',
            default_value='true',
            choices=['true', 'false'],
            description='Start Megatron perception visualizer',
        ),
        DeclareLaunchArgument(
            'show_debug_window',
            default_value='false',
            choices=['true', 'false'],
            description='Show the combined perception panel in an OpenCV window',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([pkg_megatron, 'config', 'production.rviz']),
            description='RViz config file',
        ),
    ]

    # Real robot: run localization + Nav2 on the workstation,
    # while the robot itself provides /scan, /tf, /odom, etc.
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dis_tutorial3, 'launch', 'localization.launch.py'])
        ),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('map', LaunchConfiguration('map')),
            ('params', LaunchConfiguration('localization_config')),
        ],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dis_tutorial3, 'launch', 'nav2.launch.py'])
        ),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('params_file', LaunchConfiguration('nav2_config')),
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_megatron, 'config', LaunchConfiguration('rviz_config')])],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
    )

    # Camera topic remaps: sim uses OAK-D, real robot uses Gemini (see tutorial Check 4)
    camera_remaps = [
        ('/rgb/image_raw', '/gemini/color/image_raw'),
        ('/depth/points', '/gemini/depth_registered/points'),
    ]

    # Face detector
    face_detector = Node(
        package='megatron',
        executable='face_detector',
        name='face_detector',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=camera_remaps,
    )

    # Ring detector
    ring_detector = Node(
        package='megatron',
        executable='ring_detector',
        name='ring_detector',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=camera_remaps,
    )

    # Mission controller
    controller = Node(
        package='megatron',
        executable='controller',
        name='mission_controller',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'waypoints_file': PathJoinSubstitution([pkg_megatron, 'waypoints', 'test1.yaml']),
            }
        ],
    )

    visualizer = Node(
        package='megatron',
        executable='perception_visualizer',
        name='perception_visualizer',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'show_window': LaunchConfiguration('show_debug_window'),
            }
        ],
        condition=IfCondition(LaunchConfiguration('visualization')),
    )

    ld = LaunchDescription(args)
    ld.add_action(localization)
    ld.add_action(nav2)
    ld.add_action(rviz)
    #ld.add_action(face_detector)
    #ld.add_action(ring_detector)
    #ld.add_action(controller)
    #ld.add_action(visualizer)
    return ld
