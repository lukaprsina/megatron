# RViz config tips and tricks for Megatron

This project uses RViz as the operator dashboard for Task 1. The goal is not to make RViz look fancy for its own sake; the goal is to make it obvious when the robot is navigating, what it thinks it sees, and whether the detector outputs are sane.

## The presets in this workspace

- `src/megatron/config/task1_safe.rviz` — the default preset now; the most conservative choice
- `src/megatron/config/task1_balanced.rviz` — a balanced 50/50-style layout for map and perception
- `src/megatron/config/task1_perception_focus.rviz` — heavier on camera/debug views
- `src/megatron/config/task1.rviz` — older preset kept for comparison

## How to launch a config

Use the `-d` option when starting RViz:

```bash
ros2 run rviz2 rviz2 -d /path/to/config.rviz
```

In this project, the cleaner way is to let the launch file pass the config from the package share directory.

## What usually matters most

### 1. Fixed frame

Set the fixed frame to the frame that everything else can be transformed into. For Task 1 this is usually:

- `map` for the main demo
- `odom` if you are debugging local motion
- a camera frame only if you are checking the camera stream itself

If the fixed frame is wrong, RViz can look empty even when the robot is publishing everything correctly.

### 2. TF display

Turn on the `TF` display early. It tells you whether the robot frames are actually connected.

Useful signs:

- `map -> odom -> base_link` exists
- the camera frames exist and move with the robot
- static transforms are present for the camera and lidar

### 3. QoS matching

A lot of RViz “it loads but shows nothing” problems are actually QoS mismatches.

Good default pairings:

- `map`: `RELIABLE` + `TRANSIENT_LOCAL`
- `scan`: `BEST_EFFORT`
- `MarkerArray`: usually `BEST_EFFORT` is fine for live demo overlays
- image topics: `RELIABLE` is usually safe for local demo streams

If a display is empty, check the publisher QoS before assuming the config is broken.

### 4. Topic names

RViz does not forgive typos very kindly.

Check that the topic names in the config match the live topics exactly:

- `/face_markers`
- `/ring_markers`
- `/goal_markers`
- `/task1_visualization_image`
- `/scan`
- `/map`

### 5. Use groups to reduce chaos

Grouping related displays keeps the sidebar usable:

- Robot
- Detections
- Camera Debug

That makes it much easier to toggle layers during debugging.

### 6. Save one preset per purpose

A good RViz workspace usually needs more than one view:

- **demo preset**: looks clean and stable
- **debug preset**: shows everything, even noisy streams
- **navigation preset**: map and TF first
- **perception preset**: camera and markers first

Do not try to make one config perfect for every task. That way lies YAML doom.

## A few practical tricks

### Hide the noisy stuff

If a stream is useful only sometimes, keep it disabled by default and enable it when needed.

Good candidates to keep off until debugging:

- raw camera image
- dense point clouds
- extra TF axes or labels

### Use a dark background

A darker background tends to make map overlays, markers, and camera panels easier to read on a projector.

### Keep the view simple

For mobile robots, `TopDownOrtho` is usually the best default. It is the least confusing when you are trying to judge waypoints, paths, and detections.

### Put markers on their own group

Face markers, ring markers, and goal markers should be separated if possible. That makes it obvious which subsystem is failing.

### Show the camera feed only when it helps

The combined perception image is great for demos and validation, but it can clutter the window if you are focused on navigation.

## Troubleshooting checklist

If RViz opens but the scene looks wrong:

1. Check `Fixed Frame`
2. Check TF connectivity
3. Check whether the map is actually being published
4. Check QoS compatibility with `ros2 topic info -v`
5. Check whether the topic exists at all
6. Check whether the display is disabled in the config
7. Check the RViz console for plugin errors

## In this project specifically

The useful visual layers are:

- `map` for the explored world
- `tf` for frame sanity
- `scan` for local sensing
- `face_markers` and `ring_markers` for confirmed detections
- `goal_markers` for controller intent
- `task1_visualization_image` for the combined perception view

If the controller is drawing manual points on the ground or showing vectors from the robot, keep those overlays visible in at least one preset. They are great for explaining what the robot is thinking instead of just where it is.

## My recommended workflow

- Start with `task1_safe.rviz`
- Switch to `task1_balanced.rviz` when you want a clean demo layout
- Switch to `task1_perception_focus.rviz` when you are tuning the detectors
- Keep one terminal dedicated to `ros2 topic list` / `ros2 topic info -v` while tuning

## Small philosophy note

A good RViz config should answer three questions quickly:

- Where am I?
- What did I detect?
- Is the system behaving consistently?

If a config does that, it is doing its job.
