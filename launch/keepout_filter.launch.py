import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_megatron = get_package_share_directory("megatron")

    # Arguments
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock if true"
    )

    declare_mask_yaml = DeclareLaunchArgument(
        "mask_yaml",
        default_value=PathJoinSubstitution([pkg_megatron, "maps", "factory_keepout.yaml"]),
        description="Path to the keepout mask YAML metadata file"
    )

    # 1. Map Server to load and publish the keepout map grid
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="keepout_filter_mask_server",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "yaml_filename": LaunchConfiguration("mask_yaml"),
            "topic_name": "keepout_filter_mask",
            "frame_id": "map"
        }]
    )

    # 2. Costmap Filter Info Server to publish metadata about the filter
    filter_info_server_node = Node(
        package="nav2_map_server",
        executable="costmap_filter_info_server",
        name="costmap_filter_info_server",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "type": 0,  # 0 indicates Keepout Filter
            "filter_info_topic": "/costmap_filter_info",
            "mask_topic": "/keepout_filter_mask",
            "base": 0.0,
            "multiplier": 1.0
        }]
    )

    # 3. Lifecycle Manager (Required to transition Map Server & Filter Info Server to Active)
    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_keepout",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "autostart": True,
            "node_names": ["keepout_filter_mask_server", "costmap_filter_info_server"]
        }]
    )
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_mask_yaml)
    ld.add_action(map_server_node)
    ld.add_action(filter_info_server_node)
    ld.add_action(lifecycle_manager_node)  
    return ld