from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_megatron = get_package_share_directory("megatron")

    args = [
        DeclareLaunchArgument(
            "world", default_value="task2", description="Gazebo world name"
        ),
        DeclareLaunchArgument(
            "map",
            default_value=PathJoinSubstitution([pkg_megatron, "maps", "factory.yaml"]),
            description="Map YAML file",
        ),
        DeclareLaunchArgument(
            "launch_rviz", default_value="true", choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "visualization",
            default_value="true",
            choices=["true", "false"],
            description="Start perception visualizer",
        ),
        DeclareLaunchArgument(
            "show_debug_window",
            default_value="false",
            choices=["true", "false"],
            description="Show combined perception panel in OpenCV window",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true", choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution(
                [pkg_megatron, "config", "production.rviz"]
            ),
            description="RViz config file",
        ),
        DeclareLaunchArgument(
            "nav2_config",
            default_value=PathJoinSubstitution([pkg_megatron, "config", "nav2.yaml"]),
            description="Nav2 params file",
        ),
        DeclareLaunchArgument(
            "manual_mode",
            default_value="false",
            choices=["true", "false"],
            description="Perception-only teleop mode (no autonomous patrol)",
        ),
    ]

    sim_arm_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_megatron, "launch", "sim_arm_nav.launch.py"])
        ),
        launch_arguments=[
            ("world", LaunchConfiguration("world")),
            ("map", LaunchConfiguration("map")),
            ("rviz", "false"),
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
            ("nav2_config", LaunchConfiguration("nav2_config")),
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    face_detector = Node(
        package="megatron",
        executable="face_detector",
        name="face_detector",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    ring_detector = Node(
        package="megatron",
        executable="ring_detector",
        name="ring_detector",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    arm_mover = Node(
        package="dis_tutorial7",
        executable="arm_mover_actions.py",
        name="arm_mover",
        output="screen",
    )

    cylinder_segmentation = Node(
        package="cylinder_segmentation",
        executable="cylinder_segmentation",
        name="cylinder_segmentation",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    controller = Node(
        package="megatron",
        executable="task2_controller",
        name="task2_controller",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "waypoints_file": PathJoinSubstitution(
                    [pkg_megatron, "waypoints", "task2-first.yaml"]
                ),
                "manual_mode": LaunchConfiguration("manual_mode"),
            }
        ],
    )

    visualizer = Node(
        package="megatron",
        executable="perception_visualizer",
        name="perception_visualizer",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "show_window": LaunchConfiguration("show_debug_window"),
            }
        ],
        condition=IfCondition(LaunchConfiguration("visualization")),
    )

    ld = LaunchDescription(args)
    ld.add_action(sim_arm_nav)
    ld.add_action(rviz)
    ld.add_action(cylinder_segmentation)
    ld.add_action(face_detector)
    ld.add_action(ring_detector)
    ld.add_action(arm_mover)
    ld.add_action(controller)
    ld.add_action(visualizer)
    return ld
