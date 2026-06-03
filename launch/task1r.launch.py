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
            'pc2_overlay',
            default_value='true',
            choices=['true', 'false'],
            description='Overlay PointCloud2 points on the RGB image for debugging',
        ),
        DeclareLaunchArgument(
            'show_debug_window',
            default_value='true',
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
        DeclareLaunchArgument(
            'free_will',
            default_value='false',
            choices=['true', 'false'],
            description='freeWill mode: do not autonomously follow waypoints',
        ),
        DeclareLaunchArgument(
            'allowed_to_speak',
            default_value='true',
            choices=['true', 'false'],
            description='Allowed to speak',
        ),

        # If the Gemini driver publishes PointCloud2 with frame_id=gemini_color_optical_frame
        # but that frame is missing in TF, RViz and perception nodes cannot transform it into `map`.
        DeclareLaunchArgument(
            'publish_gemini_tf',
            default_value='true',
            choices=['true', 'false'],
            description='Publish a static TF for Gemini camera optical frame',
        ),
        DeclareLaunchArgument(
            'gemini_tf_parent_frame',
            default_value='base_link',
            description='Parent frame for Gemini static TF (must exist in /tf)',
        ),
        DeclareLaunchArgument(
            'gemini_tf_child_frame',
            default_value='gemini_color_optical_frame',
            description='Child frame for Gemini static TF (matches PointCloud2 header.frame_id)',
        ),
        DeclareLaunchArgument('gemini_tf_x', default_value='-0.03705974'),
        DeclareLaunchArgument('gemini_tf_y', default_value='0.0'),
        DeclareLaunchArgument('gemini_tf_z', default_value='0.12168622'),
        DeclareLaunchArgument('gemini_tf_roll', default_value='-1.57079632679'),
        DeclareLaunchArgument('gemini_tf_pitch', default_value='0.0'),
        DeclareLaunchArgument('gemini_tf_yaw', default_value='-1.57079632679'),
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
        ('/depth/image_raw', '/gemini/depth/image_raw/compressedDepth'),
        ('/rgb/image_raw', '/gemini/color/image_raw/compressed'),
        ('/depth/camera_info', '/gemini/depth/camera_info'),
        ('/rgb/camera_info', '/gemini/color/camera_info'),
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

    pc2_image_overlay = Node(
        package='megatron',
        executable='pc2_image_overlay',
        name='pc2_image_overlay',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=camera_remaps,
        condition=IfCondition(LaunchConfiguration('pc2_overlay')),
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
                'waypoints_file': PathJoinSubstitution([pkg_megatron, 'waypoints', 'task1r_v2.yaml']),
                'free_will': LaunchConfiguration('free_will'),
                'allowed_to_speak': LaunchConfiguration('allowed_to_speak'),

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
        remappings=camera_remaps,
    )

    ld = LaunchDescription(args)
    ld.add_action(localization)
    ld.add_action(nav2)
    ld.add_action(rviz)
    ld.add_action(face_detector)
    ld.add_action(ring_detector)
    ld.add_action(controller)
    ld.add_action(visualizer)
    return ld
