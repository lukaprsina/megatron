## Structured Summary per Package

---

### 1. **dis_tutorial7** -- Arm Camera Package (HIGH relevance for Task 2)

**Path:** `/home/luka/coding/dis/src/vendor/dis_tutorial7/`

**What it is:** A modified TurtleBot4 simulation that adds a 4-DOF robotic arm with a high-resolution RGBD camera (`top_camera`) mounted on top of the robot. The arm is NOT physically realistic (no collision geometry, no joint limits, negligible mass), making it easy to position. This is the primary package for anomaly inspection and arm camera usage.

**Nodes provided:**

| Node                   | File                           | Description                                                                                                               |
| ---------------------- | ------------------------------ | ------------------------------------------------------------------------------------------------------------------------- |
| `arm_mover_actions.py` | `scripts/arm_mover_actions.py` | Controls the arm via action interface. Subscribes to `/arm_command` (String). Accepts named poses or manual joint angles. |

**Launch files:**

| Launch File                                          | Description                                                                                                     |
| ---------------------------------------------------- | --------------------------------------------------------------------------------------------------------------- |
| `sim_turtlebot_slam.launch.py`                       | Full simulation + SLAM (with arm robot)                                                                         |
| `sim_turtlebot_nav.launch.py`                        | Full simulation + Navigation (with arm robot)                                                                   |
| `turtlebot4_spawn.launch.py`                         | Spawns the arm-equipped TurtleBot4 in Gazebo                                                                    |
| `robot_description.launch.py`                        | Loads the robot URDF (with arm) into `robot_state_publisher`                                                    |
| `ros_gz_bridge.launch.py`                            | Bridges Gazebo sensors to ROS 2 topics. **Adds the `top_camera` bridges** alongside the standard Oak-D bridges. |
| `control.launch.py`                                  | Spawns controllers (joint_state_broadcaster -> diffdrive_controller -> arm_controller in sequence)              |
| `create3_nodes.launch.py`                            | Launches Create3 control (includes control.launch.py)                                                           |
| `view_robot.launch.py` / `view_navigation.launch.py` | RViz visualization configs                                                                                      |

**Topics published by top_camera (arm camera):**

- `top_camera/rgb/preview/image_raw` (sensor_msgs/Image) -- RGB image
- `top_camera/rgb/preview/depth` (sensor_msgs/Image) -- Depth image
- `top_camera/rgb/preview/depth/points` (sensor_msgs/PointCloud2) -- RGB point cloud
- `top_camera/rgb/preview/camera_info` (sensor_msgs/CameraInfo) -- Camera intrinsics

**Arm control topics:**

- `/arm_command` (std_msgs/String) -- **Input** to `arm_mover_actions.py`. Send named poses or manual joint positions.
- `/arm_controller/follow_joint_trajectory` -- Action server for `FollowJointTrajectory` action (used by `arm_mover_actions.py`)

**How the arm mover works:**

1. The `arm_mover_actions.py` node subscribes to `/arm_command` (String).
2. It maps string commands to predefined joint angle lists (4 joints: `arm_base_joint`, `arm_shoulder_joint`, `arm_elbow_joint`, `arm_wrist_joint`).
3. It sends a `FollowJointTrajectory` action goal to `/arm_controller/follow_joint_trajectory` with a single trajectory point (3-second duration).
4. Predefined poses:
   - `look_at_belt_right`: [-1.57, 0.9, 0.3, 1.7] -- suitable for defect/anomaly inspection
   - `look_at_belt_left`: [1.57, 0.9, 0.3, 1.7]
   - `look_for_qr`: [0., 0.6, 0.5, 2.0]
   - `garage`: [0., -0.45, 2.8, -0.8] -- default/stowed position
   - `up`: [0., 0., 0., 0.]
   - `manual`: accepts custom joint angles via `manual:[a,b,c,d]` syntax

**How to reuse in Task 2:**

- **Anomaly inspection:** Position the arm in `look_at_belt_right` or a custom pose, then subscribe to `top_camera/rgb/preview/image_raw` and/or `top_camera/rgb/preview/depth/points` to inspect objects on the conveyor belt or other elevated features.
- **Blue line following:** The `top_camera` could be angled downward to see the ground directly in front of the robot for more precise line following.
- **Yellow line detection:** Same approach -- use the arm camera from an elevated angle.
- Integration: Start `sim_turtlebot_nav.launch.py` from this package (uses `dis_tutorial3` worlds underneath), then run `arm_mover_actions.py`, and publish to `/arm_command` from your own node.

**Configs:**

- `all_controls_jtc.yaml` -- Defines 3 controllers: `joint_state_broadcaster`, `diffdrive_controller`, `arm_controller`
- `arm_controller_position.yaml` -- Standalone arm controller config
- `all_controls.yaml` / `control.yaml` -- Additional controller variants

---

### 2. **dis_tutorial3** -- Simulation/Navigation Utilities (HIGH relevance for Task 2)

**Path:** `/home/luka/coding/dis/src/vendor/dis_tutorial3/`

**What it is:** The main simulation and navigation package for TurtleBot4. Provides worlds, maps, launch files for SLAM/localization/nav2, and utility scripts for robot commanding and person/face detection.

**Nodes provided:**

| Node/Script                        | File                                       | Description                                                                                                                                                                                  |
| ---------------------------------- | ------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `robot_commander.py`               | `scripts/robot_commander.py`               | Full robot commander with Nav2 action clients (`navigate_to_pose`, `spin`, `dock`, `undock`), AMCL pose subscription, initial pose setting. Reusable as a base class for navigation control. |
| `detect_people.py`                 | `scripts/detect_people.py`                 | YOLOv8-based person detector. Subscribes to `/oakd/rgb/preview/image_raw` and `/oakd/rgb/preview/depth/points`, publishes Marker to `/people_marker`.                                        |
| `extract_color_from_pointcloud.py` | `scripts/extract_color_from_pointcloud.py` | Demonstrates reading RGB data from a point cloud (shows how to decode packed float RGB into bytes).                                                                                          |

**Launch files:**

| Launch File                    | Description                                                                |
| ------------------------------ | -------------------------------------------------------------------------- |
| `sim.launch.py`                | Starts Gazebo simulator with clock bridge. Takes `world` and `model` args. |
| `sim_turtlebot_slam.launch.py` | Simulation + robot spawn + SLAM (slam_toolbox).                            |
| `sim_turtlebot_nav.launch.py`  | Simulation + robot spawn + localization (AMCL) + Nav2.                     |
| `dis_sim.launch.py`            | Alternative simulation launch (legacy).                                    |
| `slam.launch.py`               | Standalone SLAM launch (online sync/async).                                |
| `localization.launch.py`       | Standalone localization launch (AMCL).                                     |
| `nav2.launch.py`               | Standalone Nav2 launch.                                                    |
| `fixed_navigation.launch.py`   | Wraps `nav2_bringup/navigation_launch.py` with fixed params.               |
| `robot_description.launch.py`  | Robot state publisher for the standard TurtleBot4.                         |
| `turtlebot4_spawn.launch.py`   | Spawns the standard TurtleBot4 (without arm).                              |

**Maps:**

| Map         | Files                                        |
| ----------- | -------------------------------------------- |
| `factory`   | `maps/factory.yaml` + `maps/factory.pgm`     |
| `task1`     | `maps/task1.yaml` + `maps/task1.pgm`         |
| `bird_demo` | `maps/bird_demo.yaml` + `maps/bird_demo.pgm` |

**Worlds (Task 2 relevant):**

| World                                                                               | File                         |
| ----------------------------------------------------------------------------------- | ---------------------------- |
| `task2`                                                                             | `worlds/task2.sdf`           |
| `task2_blue_demo`                                                                   | `worlds/task2_blue_demo.sdf` |
| Also `task2_green_demo` and `task2_yellow_demo` (referenced but not listed in full) |

**Configs:**

| Config                    | File                             | Key Settings                                                                                                                                                                                                                              |
| ------------------------- | -------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `nav2.yaml`               | `config/nav2.yaml`               | Full Nav2 configuration: RegulatedPurePursuit controller (0.5 m/s desired), NavfnPlanner, local/global costmaps with voxel/obstacle/inflation layers, collision monitor, docking server, behavior server (spin, backup, drive_on_heading) |
| `localization.yaml`       | `config/localization.yaml`       | AMCL config: likelihood_field model, 500-2000 particles, scans `scan_filtered` topic                                                                                                                                                      |
| `slam.yaml`               | `config/slam.yaml`               | SLAM Toolbox config: Ceres solver, 0.05m resolution, uses `scan_filtered`                                                                                                                                                                 |
| `laser_filter_chain.yaml` | `config/laser_filter_chain.yaml` | Laser scan filtering                                                                                                                                                                                                                      |

**Key topics used:**

- Navigation: `navigate_to_pose` (action), `spin` (action), `cmd_vel`, `odom`, `scan_filtered`, `amcl_pose`, `map`, `initialpose`
- Perception: `/oakd/rgb/preview/image_raw`, `/oakd/rgb/preview/depth`, `/oakd/rgb/preview/depth/points`
- Detection output: `/people_marker` (Marker)

**How to reuse in Task 2:**

- `robot_commander.py` is directly reusable as a navigation base class. It handles: `goToPose()`, `spin()`, `isTaskComplete()`, `cancelTask()`, `waitUntilNav2Active()`, `setInitialPose()`, `undock()`. Paste it into your Task 2 code and subclass or instantiate it.
- `detect_people.py` shows the YOLOv8 inference pattern with Ultralytics -- reusable for any object detection (e.g., anomaly detection on the belt).
- Maps and Nav2 config provide a working baseline for autonomous navigation in Task 2 worlds.
- World files for task2_blue_demo can be used directly to test blue line following.

---

### 3. **dis_tutorial5** -- Cylinder Segmentation and Ring Detection (MEDIUM relevance for Task 2)

**Path:** `/home/luka/coding/dis/src/vendor/dis_tutorial5/`

**What it is:** Point cloud processing tutorial for detecting cylinders (using PCL/RANSAC) and rings (using OpenCV). Originally built for last year's task but provided as reference.

**Nodes provided:**

| Node                    | File                            | Language | Description                                                                                                                    |
| ----------------------- | ------------------------------- | -------- | ------------------------------------------------------------------------------------------------------------------------------ |
| `cylinder_segmentation` | `src/cylinder_segmentation.cpp` | C++      | PCL-based cylinder detection. Fits cylinder models to point cloud using RANSAC + SACMODEL_CYLINDER.                            |
| `detect_rings.py`       | `scripts/detect_rings.py`       | Python   | OpenCV-based ring detection using adaptive thresholding, contour extraction, ellipse fitting, and pairing concentric ellipses. |

**Interfaces:**

- **cylinder_segmentation** (C++):
  - **Subscribes:** `/oakd/rgb/preview/depth/points` (PointCloud2) -- configurable via `topic_pointcloud_in` param
  - **Publishes:**
    - `filtered_point_cloud` (PointCloud2) -- filtered point cloud
    - `cylinder_point_cloud` (PointCloud2) -- inliers of detected cylinders
    - `cylinder_markers` (Marker) -- visualization markers for detected cylinders
  - Key parameters: `target_radius=0.11m`, `error_margin=0.04m`, `ransac_distance_threshold=0.005`, `axis=[0,0,1]` (vertical cylinders)

- **detect_rings.py** (Python):
  - **Subscribes:** `/oakd/rgb/preview/image_raw` (Image), `/oakd/rgb/preview/depth` (Image)
  - **No publishers** (only OpenCV imshow windows)
  - Uses adaptive thresholding -> contour extraction -> ellipse fitting -> center-distance pairing

**How to reuse in Task 2:**

- **cylinder_segmentation:** Could be adapted for anomaly inspection -- detecting objects on a conveyor belt by their cylindrical shape. The RANSAC cylinder fit can also be repurposed to detect other geometric primitives (SACMODEL_SPHERE, SACMODEL_PLANE). The C++ node is self-contained and can be pointed at any point cloud topic (including `top_camera/rgb/preview/depth/points`).
- **detect_rings.py:** The ellipse detection and pairing technique could be adapted for detecting circular features on inspected objects (e.g., holes, rings, circular anomalies). The concentric-ellipse pairing logic (`center_thr < 10`) is the key reusable algorithm.
- **Build dependency:** Requires PCL 1.10 and pcl_conversions -- `sudo apt install ros-jazzy-pcl-conversions`.

---

### 4. **dis_tutorial4** -- Perception Utilities (MEDIUM relevance for Task 2)

**Path:** `/home/luka/coding/dis/src/vendor/dis_tutorial4/`

**What it is:** Helper nodes for coordinate transforms, map-to-world conversions, and programmatic navigation. Focused on Task 1 but the transform utilities are universally applicable.

**Nodes provided:**

| Node                 | File                         | Description                                                                                                                                       |
| -------------------- | ---------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| `map_goals.py`       | `scripts/map_goals.py`       | Loads `/map` as an OpenCV image, allows clicking on a pixel to send a Nav2 goal. Shows map_pixel_to_world() and world_to_map_pixel() conversions. |
| `transform_point.py` | `scripts/transform_point.py` | Demonstrates TF2 transform lookups: transforms a point from `base_link` frame to `map` frame, publishes as Marker to `/breadcrumbs`.              |

**Key reusable code patterns:**

- **`transform_point.py`:** Shows how to set up `tf2_ros.Buffer` + `TransformListener`, then call `lookup_transform("map", "base_link", time_now, timeout)` and `do_transform_point()`. This is essential for converting arm-camera detections into map coordinates.
- **`map_goals.py`:** Shows `map_pixel_to_world()` conversion: `world_x = x * resolution + origin[0]`, `world_y = (height - y) * resolution + origin[1]`. Also shows OccupancyGrid reshaping and flipping for OpenCV display.
- **`map_goals.py`:** Shows Nav2 goal sending via ActionClient with `add_done_callback` pattern (same as `robot_commander.py` but simpler).

**How to reuse in Task 2:**

- Transform logic from `transform_point.py` is directly reusable for projecting arm camera detections into the map frame.
- Map coordinate conversion logic is reusable if you need to work with map pixels.
- The `/breadcrumbs` marker-publishing pattern shows how to visualize arbitrary points in RViz.

---

### 5. **ros_navigation** -- Nav2 Tutorials/Demos (LOW relevance for Task 2)

**Path:** `/home/luka/coding/dis/src/vendor/ros_navigation/`

**What it is:** The official Nav2 tutorials repository (submodule). Mostly documentation and niche demo packages.

**Contents:**

| Subdirectory             | Content                                            |
| ------------------------ | -------------------------------------------------- |
| `navigation2_tutorials/` | Working demo packages                              |
| `docs.nav2.org/`         | Documentation sources (RST) for navigation.ros.org |

**Demo packages in `navigation2_tutorials/`:**

1. **`nav2_costmap_filters_demo/`**
   - Provides `keepout_mask.pgm/.yaml` and `speed_mask.pgm/.yaml` -- costmap filter masks
   - Launch file: `costmap_filter_info.launch.py`
   - Params: `keepout_params.yaml`, `speed_params.yaml`
   - **Potential use:** Speed restriction zones or keep-out areas near the conveyor belt/inspection stations

2. **`nav2_gradient_costmap_plugin/`**
   - Custom Nav2 costmap layer plugin: `GradientLayer` (C++)
   - Computes gradient-based cost from a semantic layer
   - **Potential use:** Building custom cost functions for line following

3. **`nav2_semantic_segmentation_demo/`**
   - Semantic segmentation simulation with a custom segmentation node
   - Includes an ONNX model (`model.onnx`), ontology config, and Gazebo world
   - **Potential use:** If you want to build a semantic costmap (e.g., "blue line" as a semantic class)

**How to reuse in Task 2:**

- The costmap filters (keep-out zones) could be used to prevent the robot from driving into inspection areas.
- The gradient costmap plugin could be adapted to generate a cost function that keeps the robot centered on a blue line.
- The semantic segmentation demo provides a pattern for integrating ML models into the Nav2 costmap pipeline.

---

### 6. **Quick-check packages (LOW relevance)**

**dis_tutorial1** (`/home/luka/coding/dis/src/vendor/dis_tutorial1/`):

- Pure ROS 2 basics tutorial (publishers, subscribers, servers, clients, services, actions).
- Nodes: `py_draw_square.py` (turtlesim state machine), various C++ and Python publisher/subscriber/service examples.
- **No reusable components for Task 2** except as learning reference for ROS 2 communication patterns.

**dis_tutorial2** (`/home/luka/coding/dis/src/vendor/dis_tutorial2/`):

- Python package with parameter examples and turtlesim-based go-to-pose node.
- `go_to_position_simple_node.py`: Implements a 3-phase go-to-pose controller for turtlesim (turn -> move forward -> turn to final orientation). Demonstrates dynamic parameter updates.
- `random_velocity_publisher_node.py`: Random velocity publisher.
- **No reusable components for Task 2** -- written for turtlesim, not for real/simulated TurtleBot4.

**dis_tutorial6** (`/home/luka/coding/dis/src/vendor/dis_tutorial6/`):

- Documentation-only package about the real TurtleBot4 hardware.
- No ROS nodes, no launch files.
- Contains `cyclonedds.xml` config for connecting to real robots.
- Documents real robot camera topics under `/gemini/` namespace (real robot uses Gemini 355L, not Oak-D).
- **Useful for:** Understanding real robot camera topics when transitioning from sim to real hardware.

---

## Summary: Most Useful for Task 2

| Package             | Top Reusable Component                                                       | Used For                                   |
| ------------------- | ---------------------------------------------------------------------------- | ------------------------------------------ |
| **dis_tutorial7**   | `arm_mover_actions.py` + `top_camera/*` topics                               | Arm positioning, anomaly inspection camera |
| **dis_tutorial3**   | `robot_commander.py`, Nav2 config, task2 worlds                              | Navigation, blue/yellow line path planning |
| **dis_tutorial5**   | `cylinder_segmentation.cpp`, `detect_rings.py`                               | Object/anomaly detection on the belt       |
| **dis_tutorial4**   | `transform_point.py` (TF2 patterns), `map_goals.py` (coordinate conversions) | Projecting detections into map frame       |
| **ros_navigation**  | `gradient_layer` plugin, costmap filters                                     | Custom costmaps for line following         |
| **dis_tutorial6**   | Camera topic names for real robot                                            | Switching from sim to real hardware        |
| **dis_tutorial1/2** | None for Task 2                                                              | ROS 2 learning reference only              |
