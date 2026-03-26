from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_megatron = get_package_share_directory('megatron')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('rviz_config', default_value=PathJoinSubstitution(
            [pkg_megatron, 'config', 'task1_safe.rviz']),
            description='RViz config file'),
    ]

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        output='screen',
    )

    ld = LaunchDescription(args)
    ld.add_action(rviz)
    return ld