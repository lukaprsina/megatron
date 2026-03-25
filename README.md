# megatron

Task 1 implementation for autonomous face and ring detection, waypoint-based
search, speech feedback, and demo-friendly visualization on TurtleBot4 / ROS 2
Jazzy.

## Nodes

- `face_detector` — YOLO-based face-poster detection with annotated image output
- `ring_detector` — HSV/contour-based ring detection with color labels
- `controller` — waypoint search, approach behavior, and speech output
- `perception_visualizer` — combined side-by-side perception panel for demos

## Visualization workflow

The intended demo stack is:

- **Gazebo** for the live simulated scene
- **RViz** for the map, robot state, waypoints, and confirmed detections
- **Combined perception panel** on `/task1_visualization_image` for 2D detector evidence

The detectors still publish their own per-node debug topics:

- `/face_detections_image`
- `/ring_detections_image`

The `perception_visualizer` node subscribes to those topics and builds a single
side-by-side view for easier demos and recordings.

## Launch

Run the full Task 1 stack:

```bash
ros2 launch megatron task1.launch.py world:=task1_yellow_demo           # currently doesn't work
ros2 launch megatron task1_no_config.launch.py world:=task1_yellow_demo # use this

# runs the visualizer since the RViz config doesn't work
ros2 run megatron perception_visualizer --ros-args -p show_window:=True
```

Useful launch arguments:

- `rviz:=true|false` — start Megatron's RViz preset
- `visualization:=true|false` — start the combined perception visualizer
- `show_debug_window:=true|false` — open the combined perception panel in a local OpenCV window

For detector-only iteration:

```bash
ros2 launch megatron detectors_only.launch.py visualization:=true
```

## RViz preset

Megatron installs a custom RViz config at `share/megatron/config/task1.rviz`.
It shows the map, robot, scan, and the main MarkerArray topics:

- `/face_markers`
- `/ring_markers`
- `/goal_markers`

It also includes the combined image topic `/task1_visualization_image`.
