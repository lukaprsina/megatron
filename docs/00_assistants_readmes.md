The following is a merged version of the professor's repository (one through six) README files.

---

# Tutorial 1: Introduction to ROS2

#### Development of Intelligent Systems, FRI, 2025/2026

The focus of the first tutorial is to get familiar with the basics of the [ROS 2](http://www.ros.org) framework. You will learn to write your own programs within the system, how to execute it properly and communicate with other programs. This tutorial will introduce several important concepts that are crucial for further tutorials, so it is recommended that you refresh the topics after the end of the formal laboratory time at home. After you explore the tutorial, you will need to submit two files as **Homework 1** on a link that will become available on Učilnica. The detailed instructions for the homework are at the end of this README.

## Setting up

To set up ROS 2 on your system, read the [official documentation](https://docs.ros.org/en/jazzy/index.html). In this course, we will be using release **Jazzy** Jalisco, which is a 4 year LTS release.

The recommended operating systems are Ubuntu/Kubuntu/Lubuntu/etc. **24.04 LTS** [that support a Tier 1 native installation](https://www.ros.org/reps/rep-2000.html). Dual booting is generally the most hassle-free method if you have the option. We strongly recommend you to use one of the mentioned operating systems. At worst, at-least one of the team members should have it installed natively.

It's also possible to get ROS 2 installed on Windows in several ways:
- as a [pixi install](https://docs.ros.org/en/jazzy/Installation/Windows-Install-Binary.html#)
- by installing [WSL](https://apps.microsoft.com/detail/9P9TQF7MRM4R?hl=en-us&gl=US) and [Ubuntu 24.04](https://apps.microsoft.com/detail/9nz3klhxdjp5?hl=en-US&gl=US) from the Microsoft store
- via [VMWare/Virtualbox Ubuntu 24.04 image](https://www.osboxes.org/ubuntu/)

Note that it is likely that only the native install will be capable of running the Gazebo simulator with GPU acceleration, which is a requirement for real-time simulation. Please note that we might not be able to help you with issues you encounter with a Windows installation of ROS2.

Example code will be available for download as one [metapackage](https://docs.ros.org/en/jazzy/How-To-Guides/Using-Variants.html) (package that only contains other subpackages) per tutorial.

## Concepts and terminology

ROS 2 is a complex distributed system that introduces a few concepts that are good to know under their established expressions:

- [Basic concepts](https://docs.ros.org/en/jazzy/Concepts/Basic.html): nodes, topics, parameters, launch files, cli tools
- [Intermediate concepts](https://docs.ros.org/en/jazzy/Concepts/Intermediate.html): coordinate frames, actions and tasks, message ontology
- [Advanced concepts](https://docs.ros.org/en/jazzy/Concepts/Advanced.html): build system, internal interfaces

More info on the most important concepts:

- [Nodes](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
	- Nodes are standalone programs that perform some function, such as reading from a sensor or processing some data. They communicate with other nodes via topics. Nodes can subscribe to different topics, listen for data or events and react accordingly. They can also set up new topics to which they publish results.

- [Topics](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
	- Topics are data channels for communication between nodes. Each topic has a data type that defines the format of the data published to it. Nodes can subscribe to topics and receive data when it is published using a callback function.

- [Services](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
	- Services are an on-demand method of node communication. A node can create a service that waits for requests and returns the response. The type of request and response message types is defined in a `.srv` file.

- [Actions](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
	- Actions are used for more complex long-running tasks, such as navigation. The client sets a goal, and the action server provides feedback during the execution and notifies of the goal being reached. During execution, the action can also be cancelled if needed.

## Exploring ROS 2

Install the turtlesim package, if it's not already installed:

`sudo apt install ros-jazzy-turtlesim`

All binary ROS packages are all typically available on apt following the ros-<ros_version_name>-<package_name> convention.

Open a new terminal window and run the command:

`ros2 run turtlesim turtlesim_node`

The `ros2 run` command is the simplest way of running nodes. With the previous command we started the turtlesim_node which is located in the turtlesim package. In a third terminal, run the command:
`ros2 run turtlesim turtle_teleop_key`
This will allow you to control the turtle using the keyboard. Note that the terminal running the teleop requires the focus for the teleop to work.

Using the teleop node, messages are being sent to the turtlesim node. Open yet another terminal window and try to find out what's going on in the ROS system with the following commands:

- `ros2 topic`
- `ros2 interface`
- `ros2 service`
- `ros2 param`
- `ros2 doctor --report`
- `ros2 run rqt_graph rqt_graph`

Note that by typing −h or −help after the command verb will print information about the usage of these commands.

Run the node dis_tutorial1/draw_square.py, and observe its effect on the turtle. Print out and analyze the messages being sent to the turtle. Which node is responsible for the turtle movement and what is the structure of the messages?

Answer the following questions:

- Which **nodes** are currently active?
- What **topics** are currently active?
- What is the **message type** for each topic?
- To which topics is each node **publishing**?
- To which topics is each node **subscribed**?
- What are the packages that define different **message types**?
- Which **parameters** can be set on which nodes?

Additionally, try to:
- Get a **visualization** of all the nodes and topics in the system.
- Get a printout of all the **packages** installed in the system.
- Get a printout of all the **messages** installed in the system.
- **Print** out the messages being published on each topic.
- **Publish** a message on each topic.
- **Set** the background color of turtlesim to a color of your choice.

Explore the usage of other commands that are found in the [ROS 2 Cheatsheet](https://www.theconstructsim.com/wp-content/uploads/2021/10/ROS2-Command-Cheat-Sheets-updated.pdf). You can also find the full turtlesim documentation [here](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html#prerequisites).

## Writing a package

Write a package that contains a simple program that you can run using the `ros2 run` command and outputs a string (e.g. "Hello from ROS!") to the terminal every second.

Use the following tutorials as a starting point:

- [Creating a package](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)

Now we can add two more nodes, one that sends a message and another one that retrieves it and prints it to the terminal.

- [Simple publisher and subscriber in C++](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
- [Simple publisher and subscriber in Python](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

## Building a package

In order to build a custom package, you should move to the workspace root directory and run the following: `colcon build`. This will build all the packages within the `src` subdirectory. Since this could take a long time, the flag `--packages-select <package name>` can be used to only build selected packages. The build process will also create setup files in the `install` subdirectory. To make the newly built packages visible to ROS2, you should run the following: `source install/local_setup.bash`. Then, you will be able to run your custom nodes using the following command: `ros2 run <package_name> <node_name>`.

## Services

In the tutorial you have examples of creating a service and a client as well as defining a custom service interface. We define a custom service by specifying the structure of the request that the service will accept and the response that it will return. 

Use the following tutorials as a starting point:

- [Simple service and client in C++](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html)
- [Simple publisher and subscriber in Python](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)

See the [interfaces doc](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Interfaces.html) for a reference of all the possible native data types. The `ros2 interfaces list` command shows all built messages, services, and actions, which are themselves types that can be used in custom interfaces.

Note that only `ament_cmake` packages are capable of building interfaces (i.e. services and messages), so if you have an `ament_python` package you'll need a separate one for message definitions.

------

### 🗎 Tips and tricks

For more useful code snippets, check out the [ROS 2 Cookbook](https://github.com/mikeferguson/ros2_cookbook).

Since running several different nodes in parallel requires multiple terminals, we recommend you to use a terminal emulator that is able to display several terminals in a single window. A good choice is [Terminator](https://gnome-terminator.org/). You can install it with `sudo apt install terminator`. It should also be preinstalled on the classroom computers. You can split the screen horizontally or verically with the shortcuts `ctrl + shift + e` and `ctrl + shift + o`, respectively.

In order for your packages to be visible to ROS2, you will need to run `source install/setup.bash` in your workspace directory after build. As this holds for all terminals, a good idea is to add the following to your `.bashrc` file: `source /home/<user>/ros2_workspace/install/local_setup.bash`

Running `colcon build` is necessary after any source code changes, but only for C++ nodes (or messages/services). If writing python nodes, and you build the package using the flag `--symlink-install`, the links to your python source files were created and changing your code should work without building the package again. If you run into any build errors that don't make any sense, try deleting the `build`, `log` and `install` directories and run the build again.

Here are some other useful colcon parameters:
- `--cmake-args=-DCMAKE_BUILD_TYPE=Release` (disable debugging, enable compile time optimization)
- `--executor sequential` (use single threaded compilation, takes longer but uses less memory which can be useful when compiling large projects on limited hardware)

When in doubt, reset the cache server: `ros2 daemon stop; ros2 daemon start`

Occasionally, it can be easier to debug new topics and services by publishing or calling straight from the command line, instead of from a node. Following are some commands that can help you with this:
```
# send message from terminal
ros2 topic pub <topic name> <message type> <data> -r <publishing rate>

# e.g.:
ros2 topic pub /chat std_msgs/String "data: test" -r 1


# find interface for service
ros2 interface show <service interface name>

e.g.:
ros2 interface show dis_tutorial1/srv/AddTwoInts


# call service from terminal
ros2 service call <service name> <service interface> <data in yaml format>

# e.g.: (note that spaces in data field are important)
ros2 service call add_two_ints dis_tutorial1/srv/AddTwoInts "{a: 1, b: 2}"
```

# Homework 1

## Part 1

Open the file `homework1_answers.txt` from this repository, follow the instructions and include your answers to the questions in the marked slots in the same file. Then, upload the file on the available link on Učilnica.

## Part 2

After you have installed and tested ROS2, as well as set up your own workspace you should:

- Create a new package. In the package you can use C++ or Python.
- Create a custom message type, that has a string fields, an integer field, and a bool field.
- Create a custom service type, where the request contains a string field and an array of integers, and the response contains a string field and an integer field.
- Create a publisher node, that periodically sends a message on a certain topic. You should use the custom message you defined.
- Create a subscriber node, that recieves the message on the same topic and prints out its contents.
- Create a custom service node that accepts an array of integers and responds with the SUM of the recieved integers. Use the custom service you defined.
- Create a custom client node that generates random sequences of 10 integers, calls the service node, and prints out the response that it recieves from your service node.

Compress your package into a single .zip archive named `homework1_package.zip` and upload the file on the available link on Učilnica.

# Tutorial 2: Introduction to ROS2 - some additional concepts

#### Development of Intelligent Systems, FRI, 2025/2026

In this exercise you will get further familiarized with [ROS2](https://twitter.com/OpenRoboticsOrg/status/1629208251563929600). We will explore services,
writing nodes in Python, the usage of `ros2 launch` and `ros2 bag` commands as well as some
additional ROS 2 commands. 

Download the code for Tutorial 2 and extract the files in the `src` directory of your workspace. Build the package using `colcon build`. This will serve as a working example.

## Starting multiple nodes at once

In ROS we usually have multiple nodes running at the same time and it quickly becomes impractical to use `ros2 run` commands for starting each node separately. 

For these purposes we use the `ros2 launch` tool. This tool starts multiple nodes, as they are configured in a `.launch.py` script. Launch scripts can be set up in Python, XML, or YAML, but Python is the widely used standard. 

Run the following command in the terminal: `ros2 launch dis_tutorial2 example_launch.py` Which nodes were just started? (Hint: rqt_graph)

If you want to reset the turtle position, you can use the service /reset using `ros2 service call /reset std_srvs/srv/Empty`

To familiarize yourself with some additional functionalities like setting parameters and remapping topic names see [the launch file documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html).

## Parameters

Sometimes we want to be able to store the values of certain parameters so that they are available to every node and can be reconfigured during operation. 

A parameter is a configuration value of a node. You can think of parameters as node settings. A node can store parameters as integers, floats, booleans, strings, and lists. ROS 2 makes this functionality available though `ros2 param set` and `ros2 param get` services.

Running `ros2 run rqt_reconfigure rqt_reconfigure` will launch a GUI display of all parameters and allow them to be edited at runtime. If you want to set custom initial parameter values, you should include them in the node's launch file since they will reset to their original values upon relaunching it otherwise.

If you want to set up new parameters in your custom node, you can use the function  `node.declare_parameter()` (see the source code `go_to_position_simple_node.py`). This should expose the parameters so other nodes, launch files and rqt are able to change them on the fly.

## Achieving a goal

In order to get your turtle or robot to a specific position in space, you must provide the appropriate movement commands. Launch the example using `ros2 launch dis_tutorial2 go_to_position_launch.py` and set a goal pose by publishing the appropriate message to topic `/goal_pose`. First, observe how the turtle moves to the goal position, then check the code to understand what happens in the background. Which are the phases of moving to the goal position and what happens in each of them?

## Recording and replaying topics

ROS contains the `ros2 bag` node which enables the recording and playback of messages posted to selected topics. This can be extremely useful for debugging purposes and enables us to develop programs without having access to the real robot.

- [ROS 2 bag documentation](https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)

You can start recording the messages using `ros2 bag record <topic name>`. The recordings will be saved in a directory named with the timestamp by default. You can also choose to record all the topics, but this could result in very large files, depending on the sensors used.

You can initiate the data replay using `ros2 bag play <directory name>`. Record some commands on topic `/cmd_vel` (with keyboard or via a node), then reset the turtle and replay the commands from the recording. If you need, the topic names can also be remapped at replay time.

## RQT GUI tools 

You can easily view the topics, services, nodes, etc. by using `rqt`. This command will open up the GUI where you can inspect topics, view the node graph and even call services or visualize topics.

# Homework 2

## Writing a turtle mover

Your task is to create a service node which moves the simulated turtle from the turtlesim package. Check the [custom msgs and srvs documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html) for help.

The service request should contain a string and an integer field. The string should be one of:

- "circle"
- "rectangle"
- "triangle"
- "random"

and the integer field specifies the duration in seconds. 

The node should then move the turtle in the specified trajectory for the given duration in the integer field. After the given duration the turtle should stop moving. The response to the client should contain a string field with the previously issued movement type.



# Tutorial 3: The Turtlebot 4

#### Development of Intelligent Systems, 2026

In this exercise, you will familiarize yourself with the Turtlebot 4 robot platform that you will be using throughout the course for practical work. The robot is composed of the:
1. **iRobot Create 3** platform (based on the Roomba i-series vacuum cleaners)
2. **Luxonis Oak-D Pro** stereo camera depth sensor (only in the sim)
3. **Slamtec RPLIDAR-A1 2D** triangulation lidar sensor
4. **Raspberry Pi 4 SBC** with a Turtlebot HAT board

![turtlebot](figs/turtlebot.png)
*Image source: [Clearpath Robotics](https://turtlebot.github.io/turtlebot4-user-manual/overview/features.html)*

You can use [the official TB4 manual](https://turtlebot.github.io/turtlebot4-user-manual/) as a reference guide.

## Turtlebot 4 Simulator Installation

Here we present the steps needed for installing the Turtlebot 4 packages on a native Ubuntu 24.04 + ROS 2 Jazzy. For WSL/pixi/Docker installation you will need to check the instructions linked above, and possible other sources. Lab PCs are already preinstalled with everything you need.

1. Install Turtlebot related packages and the Zenoh middleware:
```bash
sudo apt update && sudo apt install curl \
ros-jazzy-turtlebot4-simulator \
ros-jazzy-irobot-create-nodes \
ros-jazzy-turtlebot4-description \
ros-jazzy-turtlebot4-msgs \
ros-jazzy-turtlebot4-navigation \
ros-jazzy-turtlebot4-node \
ros-jazzy-rmw-zenoh-cpp \
ros-jazzy-laser-filters \
ros-jazzy-teleop-twist-keyboard
```
2. Install Gazebo Harmonic:
```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update && sudo apt install gz-harmonic
```

3. Clone and build `dis_tutorial3`, i.e. this repo:

```bash
cd ~/colcon_ws/src
git clone https://github.com/vicoslab/dis_tutorial3.git
cd ..
colcon build --symlink-install
```

## Running the simulation

### Setup

For the simulation setup, we'll be using the [Zenoh RMW](https://docs.ros.org/en/jazzy/Installation/RMW-Implementations/Non-DDS-Implementations/Working-with-Zenoh.html) which was installed amongst the packages above. To enable it and configure Gazebo environment variables, add the following lines to your `.bashrc` file:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export GZ_VERSION=harmonic
export IGN_IP=127.0.0.1
```

Then restart any existing terminal windows and start the router:

```bash
pkill -9 -f ros && ros2 daemon stop
ros2 run rmw_zenoh_cpp rmw_zenohd
```

By default, the router will only relay data between nodes on localhost. For more advanced configurations, see the [github documentation](https://github.com/ros2/rmw_zenoh?tab=readme-ov-file#configuration).


### Run SLAM


Our ultimate goal is to have the robot navigate autonomously. The first step towards this capability is to build a map of the working environment of the robot. For building a map we will use the slam_toolbox package which builds a map based on the lidar data and the odometry data from the robot movement. 

Now close all the running nodes and launch the Turtlebot 4 simulation + SLAM + rviz. Open a new terminal and run:

```bash
ros2 launch dis_tutorial3 sim_turtlebot_slam.launch.py
```

This will start the necessary nodes for building a map. You should see the Ignition Gazebo Fortress simulator with a custom GUI for the Turtlebot4, and the RViz tool.

![simulation and slam](figs/sim_slam.png "The simulation and RViz during SLAM")

#### Running the simulation without a GPU

If you do not have a dedicated GPU the simulation might run slow. If the Real Time Factor (RTF) in the bottom right of Ignition Gazebo is more than 30-40% this should be enough to use the simulation normally. The problem is, that Ignition Gazebo (inlike Gazebo classic) only supports the `gpu_laser` plugin to simulate the lidar sensor which in the absence of a dedicated GPU does not generate ranges. You can tell that this is an issue if you start `sim_turtlebot_slam.launch.py` and in RViz you do not see the laser or the map that is being built. Luckily, there is a workaround to force the `gpu_laser` plugin to use CPU for rendering. You need to set up `MESA_GL_VERSION_OVERRIDE=3.3` and `LIBGL_ALWAYS_SOFTWARE=true` in your .bashrc file. For example, the last few lines of my .bashrc look like this:
```bash
export MESA_GL_VERSION_OVERRIDE=3.3 # 1. Hack for laser simulation
export LIBGL_ALWAYS_SOFTWARE=true # 2. Hack for laser simulation

source /opt/ros/jazzy/setup.bash  # Load the ROS installation and packages
source /home/username/colcon_ws/install/setup.bash # Load the packages from my workspace
```

#### DPI Scaling

In case you see Rviz2 or Gazebo flicker, you add the following in your .bashrc file to disable scaling:

```bash
unset QT_SCREEN_SCALE_FACTORS
```

#### Restarting the sim

ROS 2 does not track its own processes and many additional ones like gazebo gui and server will fork and might remain active after the launch file is closed, so we've provided a `kill_ros_processes.sh` script that should clean up anything still running from a previous session.

### Building a map

To build the map we should move the robot around the course using the teleop node:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```
You can also use the the GUI in Gazebo:

![keyboard gui](figs/sim_keyboard.png "Keyboard GUI")

Now move about the course until you get a relatively good map. To build a good map:
- Move the robot slowly. When the robot is moving quickly it can lose the connection between individual scans or build up too much odometry error between map updates.
- Observe the current state that is shown in Rviz. The map is not refreshed in real time but rather in steps therefore make sure that the map has indeed been updated before moving on.

Once you are satisfied with the map you can save it by executing:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/path/to/your/map
```

Do not add any extensions to the name of your map, as the map_saver will create two different files for storing map data. The first one is a `.yaml` file containing the name of the image for the map, the origin of the image, the metric length of a pixel in the map and the thresholds for occupied and free space. The other file is a `.pgm` image file (portable gray map) which you can open in most image editors. This is useful for fixing minor imperfections. 

When you have built a good enough map, close all running nodes.

#### Renaming saved maps

If you want to rename a map you must modify the .yaml file as well, since it contains the .pgm name:

**bird_demo.yaml**
```yaml
image: bird_demo.pgm
mode: trinary
resolution: 0.050
origin: [-4.466, -8.077, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

### Navigation

The Turtlebot 4 uses Nav2 which provides perception, planning, control, localization, visualization, and much more to build autonomous systems. This will compute an environmental model from sensor and semantic data, dynamically path plan, compute velocities for motors, avoid obstacles, and structure higher-level robot behaviors. You can find more info about Nav2 [here](https://navigation.ros.org/).

If you have built a map of the course, we are finally ready to let the robot drive on its own. In one terminal start the simulation:

```bash
ros2 launch dis_tutorial3 sim_turtlebot_nav.launch.py
```

You should see RViz with the already loaded map:

![simulation and nav](figs/sim_nav.png "The simulation and RViz during navigation")

You can send navigation goal to the robot from RViz. You can load your own custom map by modifying the `sim_turtlebot_nav.launch.py` launch file:

```python
DeclareLaunchArgument(
    'map',
    default_value=PathJoinSubstitution([pkg_dis_tutorial3, 'maps', 'map.yaml']), #concats location of the given package and subfolder
    description='Full path to map yaml file to load'
)
```

You can also set parameter in the command line, e.g. `ros2 launch dis_tutorial3 sim_turtlebot_nav.launch.py map:=/home/rins/map.yaml`. 


### Face Detection and Localization

As part of Task 1, you need to detect the faces in the course. The easiest way to run YOLO models is using the [Ultralytics](https://www.ultralytics.com/) python packages. 

On the lab PCs, we've preinstalled a virtual environment with all required packages which you can source using:
```bash
source /opt/ultralytics/bin/activate
```

Then run the person detector node, which sends a marker to RViz at the detected locations:

```bash
ros2 run dis_tutorial3 detect_people.py
```

This node uses a pretrained YOLOv8 model. It can detect many different categories, but we are only using the "person" category.


On your own machines you can install ultralytics and its dependencies using:

```bash
# ROS packages are generally designed to work with apt versions of python packages
sudo apt install python3-pip python3-opencv python3-numpy ros-jazzy-cv-bridge

# we need ultralytics from pip, since it's not on apt, which adds an overlay of some additional pip packages
pip install ultralytics --break-system-packages

# remove pip's numpy 2.4.2, since it breaks opencv ROS compatibility, this reverts back to using numpy 1.26.4 from apt which will work fine
pip uninstall numpy --break-system-packages
```

### Testing with different positions of faces

There are various worlds from previous years in this repository (under dis_tutorial3/world/2024 and 2025) which you can explore to get a feel for what to expect.

The default world is currently set to `bird_demo1` (an example world for last year's Task 2). To load a different world, you can add the `world` argument in the `launch` command without any prefix e.g.:

```bash
ros2 launch dis_tutorial3 sim_turtlebot_slam.launch.py world:=task1_blue_demo
```

You can also change the `default_value` of the `world` in the `.launch` file itself, for example in `sim_turtlebot_slam.launch.py`:

```python
DeclareLaunchArgument('world', default_value='task1_blue_demo', description='Ignition World'),
```

There are three example worlds available for this year's Task1, which use can use to build your map and test run your implementation:
- `task1_blue_demo`
- `task1_green_demo`
- `task1_yellow_demo`

![task_1_sim](figs/task1_worlds.jpg "Three demo worlds")

### Sending movement goals from a node

In the `robot_commander.py` node you have examples of how to programatically send navigation goals to the robot. Explore the code after running it:
```bash
ros2 run dis_tutorial3 robot_commander.py
```

<br>

# Homework 3

Build a map of the course and save it to the disk. Then, load the map and drive the robot around, detect faces and save their positions. Finally, write a script that moves the robot between the positions of the detected faces (you can modify the `robot_commander.py` script for that).

## Resources

Turtlebot4:
- [The official TB4 user manual](https://turtlebot.github.io/turtlebot4-user-manual/)  
- [TB4 navigation tutorial](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/turtlebot4_navigator.html)   
- [TB4 Tutorials](https://github.com/turtlebot/turtlebot4_tutorials)

Slam + Nav:
- [Slam Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2](https://docs.nav2.org/)   

Etc.
- [Kimi](https://www.kimi.com/)
- [Everything you need](google.com)

# Tutorial 4: Help & Info

#### Development of Intelligent Systems, 2026

This week you should be working on Task1. Here are some technical details to help you with the implementation.

## Coordinate Transforms

Establishing the relative positions of the many components of a robot system is one of the most important tasks to ensure reliable positioning. Each of the components, such as map, robot, wheels, and different sensors has its own coordinate system and their relationships must be tracked continuously. The static relationships (such as between different static sensors) are relatively simple, consisting of only sequential rigid transformations that need to be computed only once. The dynamic relationships, such as the relative position of the robot to the world coordinates or the positions of the robot arm joints to the camera, however, must be estimated during operation.
You can install the tools for viewing the tf2 transforms as follows:

```bash
sudo apt install ros-jazzy-tf2-tools
```

Then, while your simulation is running, you can run `ros2 run tf2_tools view_frames` to record the tree of the tf2 transformations. This will create a .pdf representation of the tf2 tree that you can analyze.
You can also request the values of the specific transformation between two coordinate frames using the command:
```
ros2 run tf2_ros tf2_echo base_link map
```
This is important for placing the observed objects (faces, rings etc.) into the map, when they will be firstly detected in the coordinate frame of the robot (or, more specifically, in the coordinate frame of the sensor). You can use the scripts included in this tutorial to get you started. You can also work through the [tutorial for the tf2 transforms](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html): 

### Libraries for Transformations
If you do not want to write your own transformations (e.q. Euler angles to Quaternions or vice versa) you can use the following libraries:

```bash
sudo apt install ros-jazzy-tf-transformations ros-jazzy-turtle-tf2-py
pip install transforms3d --user --break-system-packages
```

Then, you can do stuff like:
```python
import tf_transformations
q = tf_transformations.quaternion_from_euler(r, p, y)
r, p, y = tf_transformations.euler_from_quaternion(quaternion)
```

## Nodes

This week's demo nodes deal with mapping points between different coordinate systems. You can use these as the foundation for your own implementation of the tasks.

### map_goals.py

The node `map_goals.py` reads the pre-constructed map from topic `/map` and displays it in a new window. It then waits for user input. When the user clicks on a valid point in the map, the node sends a navigation goal to the robot. This node illustrates some ideas, like how you can read the map from the topic and convert it to a numpy image, how you can convert from pixel coordinates to real world coordinates and more. Build, run, and explore the code.

### transform_point.py

The node `transform_point.py` demonstrates how you can use the TF2 libraries to do transformations between frames. The node sets up the lookup to the coordinate system transformation tree, specifically the transform between frames `map` and `base_link` (the base of the robot). This allows us to define new points in the coordinate system of the robot, and express them in map coordinates. New markers are created at a distance of 0.5m behind the robot and published to the `/breadcrumbs` topic. If we published these without the appropriate transformation, they would always be located behind the robot. However, if we use the correct transformation, the robot will leave a trail of markers behind itself when driving around the map. You will use transformations such as these in your tasks on to place detected objects and faces into the map.

See [the documentation page](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/RViz/Marker-Display-types/Marker-Display-types.html) for the available marker types and more info.

#### Quaternions

By now, you will probably have observed that the robot orientation in ROS2 is represented with 4 values (instead of the more intuitive 3 values). The reason is that the orientation is expressed with quaternions. Quaternions are one of the ways of representing rotation in 3 dimensions. Some of their advantages are conciseness (4 values are the minimum for 3d rotation), no singularities, and simple interpolation (for graphics). However, they are quite complex and unintuitive. One way of imagining quaternion rotation is to think of it as a rotation about an axis represented in 3 dimensions. 

Further reading regarding quaternions can be found in [ROS2 documentation](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Quaternion-Fundamentals.html). You can also check out this [visualizer](https://quaternions.online/).

For easier development and debugging, you might want to transform the poses to a more intuitive representation, such as Euler angles (pitch, roll and yaw are rotations about the x, y, and z axis, respectively). Since we are dealing with a ground robot, the only applicable rotation is around the z axis (the yaw angle). Transform the robot's orientation to Euler angles and display the current orientation using an arrow marker.

You can read more about ROS2 marker types [here](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/RViz/Marker-Display-types/Marker-Display-types.html).

## Saying Hello

As part of Task1, your robot needs to say something when it approaches a detected face. To do this, you can simply record an audio file, and use a module like `playsound` (that you install with `pip install playsound`). There are also other modules for playing sounds: `pydub`, `simpleaudio`, or using the `os` library and playing the sound with your system player.     

A more interesting approach, and one that will also prove useful in the future, is using a text-to-speech (TTS) generator. This is currently a very active research field, and you have many different options for TTS generators. There are simple ones, complex ones, there are those that run on-device, and those that run in the cloud, you can even try to train a deep model for imitating some voice. 

For our purposes, the quality of the generated voice does not matter, so do as you wish. Some modern include (in roughly increasing order of quality and compute required):
- [espeak-ng](https://github.com/gooofy/py-espeak-ng)
- [Piper TTS](https://github.com/OHF-Voice/piper1-gpl)
- [Kitten TTS](https://github.com/KittenML/KittenTTS)
- [Kokoro TTS](https://github.com/nazdridoy/kokoro-tts)
- [Qwen3-TTS](https://github.com/QwenLM/Qwen3-TTS)

## Using ROS bags

Those of you that can only work on the simulation in the lab, make use of the `ros2 bag` command line tool. It is a tool for recording all or some messages published. For example, you can run the simulation and drive to robot around the polygon, while recording the messages published (like the images from the camera). Then you can copy the `bag` file to another computer, replay it there, and work on face detection and clustering. The tutorial for `ros2 bag` is [here](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html).

## Communication between multiple computers in ROS2

If you want to use multiple computers so they will be able to see each others' ROS2 topics, there are several options, depending on the RMW used. You will have to set `export ROS_LOCALHOST_ONLY=0` and `export ROS_DOMAIN_ID=<id>` (number between 0 and 101) so RMW will only listen on the appropriate [ports](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Domain-ID.html).

More in depth information regarding different RMW implementations is available [here](RMW_notes.md).


# Tutorial 5: Point Cloud Library and ring detection

#### Development of Intelligent Systems, 2026

This exercise will show a few examples of how to use the [Point Cloud Library (PCL)](https://pointclouds.org/) and OpenCV to extract information from the RGBD camera. The PCL project contains a large number of [tutorials](https://pcl.readthedocs.io/projects/tutorials/en/master/) demonstrating how to use the library. From the code in this tutorial, you can extrapolate how to use the PCL library in ROS2. For our purposes, the tutorials on PointCloud [segmentation](https://pcl.readthedocs.io/projects/tutorials/en/master/#segmentation) are the most relevant. The given examples use the RANSAC algorithm to find planes and cylinders, and extract the inliers.

## Ring detection
In the script *detect_rings.py* you can find a rough demonstration for finding the rings in the image. This is one of the simplest possible approaches, for demonstration purposes. You are highly encouraged to develop your own approach. The given code is explained below, which you should at least customize to improve its performance:

First, convert the image to numpy form:
```
cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
```
Convert it to a grayscale image:
```
gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
```

Apply thresholding to get a binary image. There are different possible approaches, here we use the adaptive algotihm:
```
thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 15, 30)
```

Extract contours from the edges in the binary image:
```
contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
```

Then, we fit ellipses to all contours that are longer than some threshold (20) and filter them based on eccentricity (so we ignore ellipses that are too flat):
```
elps = []
        for cnt in contours:
            if cnt.shape[0] >= 20:
                ellipse = cv2.fitEllipse(cnt)

                # filter ellipses that are too eccentric
                e = ellipse[1]
                ecc1 = e[0]
                ecc2 = e[1]
                
                ratio = ecc1/ecc2 if ecc1>ecc2 else ecc2/ecc1
                if ratio<=self.ratio_thr and ecc1<self.ecc_thr and ecc2<self.ecc_thr:
                    elps.append(ellipse)
```

We then evaluate all pairs of ellipses and try to eliminate all that do not form a ring. OpenCV returns ellipses as oriented bounding boxes. Each ellipse is represented as the coordinates of its the center point `e[0]`, the length of the minor and major axis `e[1]` and its rotation `e[2]`. The ellipses that represent the inner and outer circles of a ring have some similar properties. First, their centers should be roughly the same: 
```
e1 = elps[n]
e2 = elps[m]
dist = np.sqrt(((e1[0][0] - e2[0][0]) ** 2 + (e1[0][1] - e2[0][1]) ** 2))

# The centers of the two elipses should be within 5 pixels of each other
if dist >= self.center_thr:
    continue
```

You can do many other filters, including color, point cloud geometry, etc. You will likely need to include different assumptions and filters to correctly detect actual rings consistently and  remove false positives.

## Cylinder segmentation
***NOTE: This is the content from last year's task. It is included here as a point of interest only. You are not required to use it.*** 
You can localize cylinders in the scene by fitting a cylinder model to the geometry of the point cloud using RANSAC. The file `cylinder_segmentation.cpp` implements this using the PCL library. It republishes the filtered point cloud on the `/cylinder` topic, and the markers representing the cylinder locations to `/detected_cylinder` topic

Please note that the given node fits a cylinder to every point cloud it receives. It should be used to find the accurate position of a cylinder, but it is not reliable as a cylinder detector on its own. It can be used, but you need to filter out the false detections.

First we transform the ROS2 message to a PCL point cloud, and then to a type appropriate for processing:
```
// convert ROS msg to PointCloud2
pcl_conversions::toPCL(*msg, *pcl_pc);

// convert PointCloud2 to templated PointCloud
pcl::fromPCLPointCloud2(*pcl_pc, *cloud);
```

Keep only the points that have the x-dimension (view in front of the camera) between 0 and 10.
```
// Build a passthrough filter to remove spurious NaNs
pass.setInputCloud(cloud);
pass.setFilterFieldName("x");
pass.setFilterLimits(0, 10);
pass.filter(*cloud_filtered);
```

Then, we calculate normals to the points. For each point, we take a look at its neighbors and estimate the normal to the surface. This is one of many possible approaches to do this. In this way, we can also create a 3d mesh from 3d points:
```
// Estimate point normals
ne.setSearchMethod(tree);
ne.setInputCloud(cloud_filtered);
ne.setKSearch(50);
ne.compute(*cloud_normals);
```

We find the largest plane in the point cloud. This will usually be the ground plane. It will be simpler for RANSAC to find a good fit for the cylinder if we filter out all the points we are certain do not belong to the cylinder:
```
// Create the segmentation object for the planar model and set all the
// parameters
seg.setOptimizeCoefficients(true);
seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
seg.setNormalDistanceWeight(0.1);
seg.setMethodType(pcl::SAC_RANSAC);
seg.setMaxIterations(100);
seg.setDistanceThreshold(0.03);
seg.setInputCloud(cloud_filtered);
seg.setInputNormals(cloud_normals);

seg.segment(*inliers_plane, *coefficients_plane);
```

Remove all the points that belong to the plane from the point cloud:
```
// Extract the planar inliers from the input cloud
extract.setInputCloud(cloud_filtered);
extract.setIndices(inliers_plane);
extract.setNegative(false);
extract.filter(*cloud_plane);
```

Finally, run RANSAC and fit a cylinder model to the rest of the points:
```
// Create the segmentation object for cylinder segmentation and set all the
// parameters
seg.setOptimizeCoefficients(true);
seg.setModelType(pcl::SACMODEL_CYLINDER);
seg.setMethodType(pcl::SAC_RANSAC);
seg.setNormalDistanceWeight(0.1);
seg.setMaxIterations(100);
seg.setDistanceThreshold(0.05);
seg.setRadiusLimits(0.06, 0.2);
seg.setInputCloud(cloud_filtered2);
seg.setInputNormals(cloud_normals2);

// Obtain the cylinder inliers and coefficients
seg.segment(*inliers_cylinder, *coefficients_cylinder);
```

In the end, extract the points that belong to the cylinder and computer their centroid:
```
// extract cylinder
extract.setInputCloud(cloud_filtered2);
extract.setIndices(inliers_cylinder);
extract.setNegative(false);
pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
extract.filter(*cloud_cylinder);

// calculate marker
pcl::compute3DCentroid(*cloud_cylinder, centroid);
```

## TODO for students:
As part of Task 1, you need to find the faces and the 3D rings in the course. The code in this exercise will NOT perform this task reliably out of the box. You should either develop a completely new approach, or use this code as a starting point.
**General note:** Use markers in RVIZ to help with interpreting your robot's perception and knowledge. It is much easier than looking at terminal output. You can program markers to have different colors, sizes and shapes. Display the hardcoded points for movement, face locations, ring candidates, colors, etc.

### For face detection
Your person detector will process every image and detect the face multiple times when the robot moving around the course. You need to group the detections in the map coordinate space to establish a good estimation of the face's position. Additionally, for programming the robot approach, you might want to also calculate the direction of the wall on which the face is (i.e. the normal of the wall).

### For ring detection
There are two types of rings in the course, flat rings painted on boxes, and actual rings hanging in the air. You should only detect the free-standing rings and ignore the flat rings. You can use either or both 2D and 3D data to solve the task. You can also exploit the color information in the image (using color segmentation). For the 3D rings, there should be a hole in the inside ellipse, which can be verified from the point cloud or the depth image.
# Tutorial 6: The Real Turtlebot 4

#### Development of Intelligent Systems, 2026

![robots](figs/robots.png)
*The Company of iRobot*

## TurtleBot4

The TurtleBot4 platform consists of two main parts:
- The Create3 mobile base that controls the wheels and provides odometry, reacts to bumper switches, and controls docking/undocking. 
- The Raspberry Pi 4 + PCBA HAT that oversees the LIDAR and the stereo camera

For simplicity, we have prepared the necessary system components upfront, so you will not need to access the onboard computers directly. When the TurtleBot turns on, the ROS 2 topics required for control and reading data from sensors should automatically start, and you can access them via ROS 2 from your computer if the network settings are correct.

The main hardware difference between the simulated and real robot is the stereo camera, the sim uses the original Oak-D Pro, while the physical ones now use a Gemini 355L for more reliable operation and lower power consumption.

## STEP 1 - Turning on/restarting the robot 

The TurtleBot can only be turned on by placing it on the charging dock. Please follow the instructions below to ensure safe operation of the robot and avoid possible issues.

If the robot is powered off (lidar not spinning, LED ring is off):
- place the robot on the dock so both computers boot in sync
- wait for the beep (it may take a minute or two)

If the robot is already on the dock but the Raspberry Pi is not turned on (lidar not spinning, LED ring is on):
- remove the robot from the dock
- hold the Create3 power button for 10 seconds so the base powers down
- follow instructions for powering on

If the robot is running but not responsive (lidar is spinning, LED ring is on):
- remove the robot from the dock
- hold the Pi Shutdown button for a few seconds (the lidar should stop spinning)
- hold the Create3 power button for 10 seconds so the base powers down
- follow instructions for powering on

> NOTE: The Create3 cannot be turned off while on the charging station, and the only way to turn it on when it's powered off is to place it on the dock. If the power is cut to the Pi 4 by turning off the Create3 before executing safe shutdown, it might corrupt the SD card. Always power off the Pi 4 first, before cutting power from the Create3.

![poweroff](figs/poweroff.png)

If the buttons on the display do not respond or the display is off, it means that the Pi 4 is powered off, unless the lidar is spinning, in which case the ROS 2 nodes that handle the screen may not be running or the boot process hasn't finished yet.

![oled](figs/oled.png)
*OLED display: The first line shows the IP address. Buttons 3 and 4 select the action, button 1 confirms it, button 2 scrolls back to top. The top bar shows battery level and IP address*


⚠️ To keep robots charged and ready for use, follow these steps when you finish work:
- Place the robot on the charging dock (green LED on the dock should turn on).
- If the robot was completely powered off before, wait until the chime sound.
- Hold the Pi Shutdown button for a few seconds (the lidar should stop spinning).

If the Raspberry Pi remains powered on, the robot will charge extremely slowly, so it's essential to turn it off for faster charging.

### Robot Status

The Create3 base comes with a built-in LED ring around the main power button, which indicates the robot state. The other two buttons around it are unassigned by default and can be used for custom actions.

| State Description | Robot LED State |
|------|-------|
| The Create3 is powered off, the Pi 4 power supply has been cut. In this state the robot can only be activated by placing it back on the dock. | ![off](figs/off.png) |
| The battery is charging, the robot is on the dock. | ![charging](figs/charging.png) |
| Normal operating state | ![normal](figs/normal.png) |
| Emergency stop (ESTOP) has been triggered, either due to hitting a bumper/IR/cliff switch or because there's no communication with the Pi 4. It can be reset by pressing the power button once, or through ROS | ![estop](figs/estop.png) |
| Battery is low, if you don't recharge the robot soon it will automatically power off. Place the robot back on the dock and power off the Pi 4 so it can recharge faster. | ![lowbattery](figs/lowbattery.png) |

More info can be found [in the Create3 documentation](https://iroboteducation.github.io/create3_docs/hw/face/).

The Turtlebot consists of two DDS connected machines, the Pi 4 and the Create3 which are internally connected via USB-C. Both must be powered on for the robot to work properly.

![control](figs/control.png)
*Image source: [Clearpath Robotics](https://turtlebot.github.io/turtlebot4-user-manual/mechanical/turtlebot4.html#removing-the-pcba)*


## STEP 2 - Connecting to the robot from your workstation

As the robot turns on, it will connect to its dedicated color-coded router and network. Robots are split over multiple routers on different channels for more concurrent bandwidth.

If you are using your own laptop, first connect to the same network as the robot (e.g. for the robot `Kili`, connect to `KiliWifi` or `KiliWifi_5GHz`).

Lab workstations are already connected to one of the robot routers via Ethernet, and should be used with the corresponding robot.

![pc](https://github.com/user-attachments/assets/67fe1698-475b-4ff2-ae51-f647a1782ed5)
*The router to which the Gloin Turtlebot connects to, and the workstation that can be used with it*

Next, you need to adjust your environment variables in the `~/.bashrc` file. The `ROS_DOMAIN_ID` should match the one written on the robot, and `RMW_IMPLEMENTATION` needs to be set to `rmw_cyclonedds_cpp`.

![domain](figs/domain.png)
*The sticker with the `ROS_DOMAIN_ID`, different for each robot*

The Cyclone middleware requires [an xml config](cyclonedds.xml), specified as the `CYCLONEDDS_URI` environment variable. For more info about the cyclone config, [see here](https://iroboteducation.github.io/create3_docs/setup/xml-config/), you might need to specify the network interface explicitly if using multiple ones e.g. `<NetworkInterface name="wlan0" />`. Lab workstations already have cyclone configured, if using your own laptop, you need to download and link the provided config.

Make sure your `~/.bashrc` looks as follows:
```
export CYCLONEDDS_URI='/home/<your_user_name>/cyclonedds.xml'
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=<your_robot_domain_id>
export ROS_LOCALHOST_ONLY=0
```
 
Then, `source ~/.bashrc`, or re-open any terminals so the new `.bashrc` is properly sourced. 

Optionally, restart the `ros2 daemon` just to be sure:
```
ros2 daemon stop; ros2 daemon start
```

This should be it!

----------------

To check if everything is working correctly, you can use the following checks:

#### Check 1

To check if the robot is properly connected, you can inspect topics with:

    ros2 topic list

You should see a lot of topics. If not, recheck:
- that the robot is charged and powered on
- your network settings
- try pinging the Raspberry Pi (`192.168.0.<your_robot_domain_id>`)
- your DDS config (`echo $RMW_IMPLEMENTATION`)
- your domain config (`echo $ROS_DOMAIN_ID`)

#### Check 2

See if you are receiving the `/odom` topic:

    ros2 topic echo /odom

You should see a stream of data. If not, the Create3 base has not yet booted, wait for the chime sound. The `/odom` topic is necessary for localization and control of the robot.

#### Check 3

See if you can move the robot with keyboard teleoperation:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard

You should be able to move the robot.

If you have passed all the checks, you can move on with building a map and navigating the robot.

#### Check 4

See if the Gemini stereo camera is sending over the images by running:

```bash
ros2 topic echo --no-arr /gemini/color/image_raw
ros2 topic hz /gemini/color/image_raw
```
    
If you see data being published, you can then use the images in your scripts by changing the subscriber topic name.

The physical robot's camera comes with different topics than the simulated Oak-D, so you'll need to adjust your launch files:

```bash
#RGB Image
/gemini/color/camera_info
/gemini/color/image_raw
/gemini/color/image_raw/compressed
/gemini/color/image_raw/theora

#Depth Image
/gemini/depth/camera_info
/gemini/depth/image_raw
/gemini/depth/image_raw/compressedDepth
/gemini/depth/image_raw/theora

#RGB Point Cloud
/gemini/depth/points
/gemini/depth_registered/points

#Misc.
/gemini/depth/metadata
/gemini/color/metadata
/gemini/depth_filter_status
/gemini/device_status
```


#### TTS

The robots also carry a speaker and run a speech-to-text node which listens to `/speak`. Publish a string message to this topic to have the robot speak your message out loud. 

## Building a map

The procedure for building a map is a little simpler than when using the simulation, we only need to start the SLAM and the keyboard teleoperation.

Please follow the [official map making tutorial](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/generate_map.html).

## Navigating a map

Once you have saved a map, we can have the robot navigate around the map. 

Check the [official navigation tutorial](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/navigation.html).

## Transform frames

![tf_rviz](figs/tf_rviz.png)

The transform system in ROS 2 is a standardized way to define and handle coordinate frames and transforms between them. It consists of two topics: `/tf` and `/tf_static`, as well as a collection of libraries for python and C++.

Every robot is defined by a collection of coordinate frames, with `base_link` being considered the parent frame for the current robot. Other common frames include:

- `base_footprint` - typically below base link, where the robot meets the ground
- `odom` - the transformation between the location where the robot started and where it is now, based on wheel encoder data
- `map` - the correction for odometry drift, usually calculated based on external landmarks
- `world` or `earth` - localizes one or multiple robots on the world WGS84 system based on GNSS or other global data
- sensor frames like `camera`, `imu`, `laser`, `sonar`, etc. which reflect the position and rotation of sensors mounted on the real robot, so their data can be accurately transformed into other frames

When multiple robots are in the same TF graph, the conventional way to separate them is using namespacing, i.e. prepending a robot name to the frames. That way `/robot1/base_link` can be distinct from `/robot2/base_link` while using the same conventions or even be built from the same URDF file.

You can use the following command to collect and view currently available frames:

    ros2 run tf2_tools view_frames

If you're running the TurtleBot simulator, the generated PDF should look something similar to the excerpt below, a full graph of all connected frames:

![tf_frames](figs/tf_frames.png)

Check the [official documentation page](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html) for more info.