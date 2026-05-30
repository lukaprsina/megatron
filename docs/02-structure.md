Done. Here's what was set up:

Files copied:

    factory.pgm + factory.yaml → src/megatron/maps/
    dis_tutorial7 → src/dis_tutorial7 (as buildable ROS 2 package)
    COLCON_IGNORE in src/vendor/dis_tutorial7/ (to avoid duplicate package error)
    Task2 worlds already exist in dis_tutorial3's install, so world=task2 resolves correctly

New launch files:

    launch/sim_arm_nav.launch.py — arm-enabled sim (dis_tutorial7 robot) + dis_tutorial3 gazebo/loc/nav2 with custom params
    launch/task2.launch.py — full task2 stack: arm nav + detectors + controller + arm_mover + RViz

Launch chain:

ros2 launch megatron task2.launch.py
  ├── sim_arm_nav.launch.py
  │     ├── dis_tutorial3 sim.launch.py       (gazebo + task2 world)
  │     ├── dis_tutorial7 turtlebot4_spawn    (arm robot + top_camera bridge)
  │     ├── dis_tutorial3 localization        (AMCL + factory map)
  │     └── dis_tutorial3 nav2                (custom nav2.yaml config)
  ├── RViz2
  ├── face_detector     (megatron)
  ├── ring_detector     (megatron)
  ├── arm_mover         (dis_tutorial7)
  ├── mission_controller (megatron)
  └── perception_visualizer (megatron)

To start: (after . activate.sh)

ros2 launch megatron task2.launch.py

Customize with args: world:=task2_blue_demo, map:=.../factory.yaml, show_debug_window:=true, launch_rviz:=false, etc.