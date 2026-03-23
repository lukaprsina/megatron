## Talk with an LLM

---


User:

We are a team of three students in a comp sci uni, we have a semester project involving TurtleBot4 and ROS2. We are starting Task 1, with Task 2 being a continuation after the competition.

```
Setup:
- fenced area
- three printed faces at random places
- two rings at random places

Task:
- build the map of the competition area
- search the space and look for faces and rings
- when a face is detected approach, or at least pass by, (and greet) the face
- when a ring is detected approach, or at least pass by, the ring and say its colour
- when all three faces and both rings are detected, stop
- do not approach a face or ring more than once
- do not have false detections

Goals:
- the robot should detect as many faces and rings as possible
- the robot should not detect something else as a face or ring
- perform the task as fast as possible
```


Evaluation protocol:

```
- The evaluation course will be set up in advance
- The teams will be allowed to build the map in advance
- The faces and rings will be positioned on the day of the evaluation
- The positions of the faces and rings should not be hand coded
- The robot search goal positions can be hand coded
- The robot has to operate completely autonomously
- The teams will be allowed to tune the parameters
```


Evaluation criteria:

```
Measuring:
- number of faces correctly detected, approached (or passed by) and greeted
- number of rings correctly detected, approached (or passed by) and recognised colours
- number of false detections
- the time until the robot stops

But also:
- Robustness of the performance
- Repeatability
- Innovation
- Clarity of demonstration
- Elegance of solution
- Approximately 5 (navigation) + 5 (faces) + 5 (rings) points

Scoring:
- T1s (in simulation): 15 points max
- T1r (on real robot): 15 points max
```


Demonstration:

```
- visualisation of detected locations
- mark locations in Rviz (also goals, detected colour, etc.)
- verbalisation of detections
- simple speech synthesis (greeting, colours)
- (Gazebo), RVIZ, camera view as well as images of detected faces should be shown
```


Task 1 and 2 (\* means that it's for Task 2):

```
System setup
- Running ROS
- Tele-operating TurtleBot

Autonomous navigation
- Autonomous control of the mobile platform
- Acquiring images and 3D information
- Simultaneous mapping and localization (SLAM)
- Path planning, obstacle avoidance
- Advanced fine manoeuvring*
- Basic mobile manipulation*
- Intelligent navigation and exploration of space*

Advanced perception and cognitive capabilities
- Detection of faces, 3D rings, cylinders, and objects (cylinders and objects are Task 2)
- Recognition of faces and ring colours (rings are Task 2)
- Segmentation of the ground*
- Object detection and counting, defect detection*
- Speech synthesis, speech recognition, dialogue processing (reading QR codes) (speech recognition, dialogue processing, QR codes are Task 2)
- Belief maintenance, reasoning, planning (reasoning, planning is Task 2)
```


Task 1 goals:

```
- to learn how to use ROS
- to get familiar with the hardware
- to set up the mobile platform (software and hardware)
- to learn how to build and use a map
- to learn how to set a goal and to instruct the robot to go to the goal position
- to learn how to relate points in different coordinate frames
- to use LIDAR
- to use RGBD camera
- to learn how to process imagess
- to learn to work with 3D data
- to robustly detect faces
- to robustly detect rings
- to search the space
- to learn to work in Gazebo
- to learn to work with a real mobile robot
```

Think about this, talk about the required nodes, topics, services etc. We use python instead of C++. Search the web for whatever you want, but make sure to search for TurtleBot 4.

---


Gemini:

# **Integrated Framework for Autonomous Artifact Detection and Spatial Mapping in the TurtleBot 4 Ecosystem under ROS 2 Jazzy**

The deployment of the TurtleBot 4 platform within the ROS 2 Jazzy Jalisco environment represents a significant evolution in mobile robotics for academic and research applications.1 This report provides a detailed technical analysis of the systems and strategies required to execute a complex search-and-interaction mission involving the mapping of a fenced environment and the semantic identification of printed faces and colored rings. By leveraging the updated capabilities of ROS 2 Jazzy, which provides enhanced support for Gazebo Harmonic and improved CLI tools, this mission architecture prioritizes robustness and repeatability—two critical criteria for evaluation in both simulated and physical robot contexts.2

## **Hardware Integration and Computational Foundations**

The TurtleBot 4 Lite is the primary hardware configuration for this task, utilizing the iRobot Create 3 mobile base and a Raspberry Pi 4B as the central processing unit.4 The selection of the Lite model over the Standard version is a strategic decision tailored to the constraints of a fenced arena, as the Lite model maintains all essential sensing capabilities—specifically the RPLIDAR A1M8 for 2D SLAM and the Luxonis OAK-D Lite for spatial vision—while reducing the mechanical complexity associated with the larger sensor tower of the Standard model.3

## **Core Component Analysis**

The Raspberry Pi 4B, equipped with 4GB of RAM, serves as the host for Ubuntu 24.04 and the ROS 2 Jazzy distribution.6 This hardware environment is optimized for asynchronous processing, which is vital when running simultaneous vision and navigation pipelines. The Create 3 base provides the foundational odometry and motor control, communicating with the Raspberry Pi via an internal USB-C connection that facilitates high-speed data transfer of sensor messages, including infrared, cliff, and bump data.4

| Hardware Component | Specification | Role in Task 1 |
| :---- | :---- | :---- |
| Mobile Base | iRobot Create 3 | Odometry, Motor Control, Bumper Sensing |
| Onboard Computer | Raspberry Pi 4B (4GB) | ROS 2 Jazzy Host, Vision Processing, Logic |
| Primary Vision | OAK-D Lite | Face Detection, Ring Recognition, Depth Data |
| 2D Lidar | RPLIDAR A1M8 | Environment Mapping (SLAM), Obstacle Avoidance |
| Network | 5GHz Wi-Fi | Communication between Robot and User PC |
| Battery | 26Wh Lithium Ion | Sustaining 2.5-4h Mission Duration |

4  
The OAK-D Lite camera is a pivotal asset for artifact detection, combining a 4K color sensor with dual stereo sensors that allow for on-device depth calculation.4 This offloads significant computational burdens from the Raspberry Pi's CPU, as the camera's Myriad X VPU handles neural network inference and disparity mapping internally.8 The RPLIDAR A1M8 provides 360-degree laser scanning at a range of up to 12 meters, which is essential for defining the boundaries of the fenced competition area.4

## **Software and Middleware Requirements**

For ROS 2 Jazzy, the firmware of the Create 3 base must be updated to version I.\*.\*.2 A critical architectural shift in Jazzy is the lack of support for dynamic switching between different ROS Middlewares (RMW) on the Create 3; the firmware is now tied specifically to either FastDDS or CycloneDDS.2 The default and recommended implementation for Jazzy is FastDDS, which provides robust discovery mechanisms for multi-machine setups.2  
The networking configuration involves the turtlebot4-setup tool, which manages the transition between Access Point (AP) mode for initial configuration and Client mode for operational deployment.7 In a university lab setting, the use of Simple Discovery mode is generally preferred unless a complex multi-robot setup requires a dedicated Discovery Server to manage network traffic.11

## **Simultaneous Localization and Mapping (SLAM) Architecture**

The first phase of Task 1 is the generation of a high-fidelity occupancy grid of the fenced arena. This is accomplished using the slam\_toolbox package, which has superseded older mapping tools like Gmapping due to its superior handling of large-scale maps and loop closure.13

## **Implementation of slam\_toolbox**

The slam\_toolbox integrates laser scan data from the /scan topic and odometry from the /odom topic to estimate the robot's trajectory and construct the map.13 For the Raspberry Pi 4, asynchronous mapping is the preferred mode, as it allows the map to be updated without blocking the critical navigation and detection threads.16 The resulting map consists of a .pgm image file representing the occupancy grid and a .yaml file containing metadata such as resolution and origin.13

| SLAM Parameter | Default Value | Recommended for Fenced Area |
| :---- | :---- | :---- |
| resolution | 0.05 m/pixel | 0.05 m/pixel |
| max\_laser\_range | 12.0 m | 5.0 m (Arena boundaries) |
| minimum\_travel\_distance | 0.1 m | 0.05 m (Fine mapping) |
| loop\_closure\_sampling | 1 | 1 (High robustness) |
| map\_update\_interval | 5.0 s | 2.0 s (Real-time updates) |

13  
A mapping resolution of 0.05 meters per pixel is generally sufficient for a fenced arena, providing a balance between spatial detail and computational overhead.13 When mapping, the robot should be teleoperated through the environment, ensuring that the Lidar captures all stationary boundaries and internal obstacles multiple times to facilitate loop closure—a process where the SLAM algorithm recognizes a previously visited location and corrects the accumulated odometric drift.14

## **Coordinate Frames and TF2 Tree**

The integrity of the mapping process depends on the correct configuration of the transform (TF2) tree. In a standard TurtleBot 4 setup, the tree must maintain the relationships between the map frame (global), the odom frame (local), and the base\_link (robot center).18 For artifact detection, the camera frames (camera\_link and camera\_color\_optical\_frame) must be accurately linked to the base\_link to allow for the projection of 2D detections into the 3D map.20  
The relationship between frames is defined by transformation matrices, where a point in the camera frame $P\_c$ is transformed to the map frame $P\_m$ using:

$$P\_m \= T\_{map}^{odom} \\cdot T\_{odom}^{base\\\_link} \\cdot T\_{base\\\_link}^{camera} \\cdot P\_c$$  
The ROS 2 tf2\_ros library handles these calculations through a Buffer and TransformListener, which are essential for determining the global coordinates of detected faces and rings.18

## **Autonomous Exploration and Search Logic**

After the map is finalized and saved, the mission transitions to an autonomous search phase. This requires the Navigation 2 (Nav2) stack, which utilizes the map to plan collision-free paths toward goal poses.13

## **Nav2 Simple Commander and TurtleBot 4 Navigator**

For Python developers, the TurtleBot4Navigator class provides an abstraction layer over the Nav2 Simple Commander.6 It includes specific methods for the TurtleBot 4, such as dock() and undock(), which are necessary for managing the robot's power state before and after the search mission.16  
The search strategy should be designed to maximize coverage of the fenced area. A common approach is to define a sequence of waypoints that the robot visits in order.6 By using the followWaypoints() or goThroughPoses() methods, the robot can traverse the arena while the vision nodes monitor the camera stream for artifacts.22

## **Search Pattern Optimization**

In a fenced competition area, the placement of faces and rings is random. Therefore, a coverage-based search pattern is more effective than a goal-driven one. The robot can be programmed to perform "lawnmower" sweeps across the area or follow the perimeter of the fence.6 The efficiency of the search is enhanced by the 360-degree Lidar, which provides continuous obstacle avoidance, and the OAK-D Lite's wide field of view.4  
The waitUntilNav2Active() method should be called at the beginning of the script to ensure that all navigation servers are ready before the mission starts.22 Robustness in navigation is achieved by tuning the local planner—specifically the Regulated Pure Pursuit (RPP) controller—to handle the tight turns often required in a small fenced area.17

## **Semantic Artifact Detection Mechanisms**

The primary task of Task 1 is the detection and interaction with two types of artifacts: printed faces and colored rings. This is a multi-modal perception task that combines deep learning for face detection and classical computer vision for color and shape recognition.9

## **Face Detection with YOLOv8**

For detecting printed faces, the YOLOv8 (Ultralytics) framework is recommended for its high accuracy and availability of nano models that can run on the Raspberry Pi 4\.9 The vision node subscribes to the /oakd/rgb/preview/image\_raw topic and uses a pre-trained face model to identify the location of faces in the image.9

| YOLOv8 Model | Advantage | Disadvantage | Performance on RPi4 |
| :---- | :---- | :---- | :---- |
| yolov8n.pt (Nano) | Highest Speed | Lowest Accuracy | \~2-5 FPS |
| yolov8s.pt (Small) | Balanced | Moderate CPU usage | \~1-2 FPS |
| yolov8m.pt (Medium) | High Accuracy | Very slow | \<1 FPS |

9  
To improve frame rates, the inference can be performed on a remote PC connected via Wi-Fi, or the model can be exported to OpenVINO or Blob format to run directly on the OAK-D Lite's VPU.9 Once a face is detected, the node extracts the bounding box coordinates $(u\_{min}, v\_{min}, u\_{max}, v\_{max})$.9

## **Color and Ring Recognition**

Ring detection is fundamentally a color-segmentation problem. The OAK-D Lite's RGB stream is converted into the HSV color space, which is more robust to changes in lighting intensity than RGB.27

1. **Color Filtering:** Apply the cv2.inRange() function to create a binary mask based on the hue of the target ring.27  
2. **Noise Reduction:** Use Gaussian blur followed by morphological opening (erosion then dilation) to eliminate small false positives and smooth the contours.24  
3. **Circle Detection:** The Hough Circle Transform (cv2.HoughCircles()) is applied to the filtered mask to confirm the object is indeed a ring and not a random color blob.24

The parameters for cv2.HoughCircles(), such as minDist and param2 (the accumulator threshold), must be tuned to prevent multiple detections of the same ring.24

## **Spatial Coordinate Estimation**

To approach an artifact, the robot must know its position in 3D space. The OAK-D Lite provides depth information synchronized with the color image.4 By sampling the depth value $Z$ at the center of the detected artifact, the robot can calculate the $X$ and $Y$ offsets using the pinhole camera model 31:

$$X \= \\frac{(u \- c\_x) \\cdot Z}{f\_x}$$

$$Y \= \\frac{(v \- c\_y) \\cdot Z}{f\_y}$$  
where $c\_x, c\_y$ are the principal point coordinates and $f\_x, f\_y$ are the focal lengths.32 These values are available in the /oak/stereo/camera\_info topic.26 The calculated $(X, Y, Z)$ point is then transformed into the map frame using TF2.18

## **Mission Logic and State Management**

Task 1 requires a structured approach to state transitions, as the robot must manage mapping, searching, approaching, and interacting in a sequential and non-repetitive manner.33

## **Finite State Machine (FSM) Design**

The mission is best managed through a Finite State Machine implemented using the SMACH library or a dedicated Python class.33 The FSM tracks the number of artifacts detected and ensures the robot stops once the requirement of 3 faces and 2 rings is met.

| State | Transition Trigger | Action Taken |
| :---- | :---- | :---- |
| **Mapping** | Teleop completion | Save map and reboot into Navigation |
| **Searching** | No detections | Follow waypoints via goToPose |
| **Artifact Found** | Detection \> Threshold | Cancel current nav goal and calculate 3D pose |
| **Approaching** | Valid 3D pose | Send new nav goal to a point near the artifact |
| **Interacting** | Proximity reached | Trigger TTS ("Greet" or "Say color") |
| **Recording** | Interaction done | Add artifact location to the "visited" set |
| **Stop** | Count \== 5 | Return to dock and terminate mission |

23  
To satisfy the requirement of "not approaching a face or ring more than once," the robot maintains a set of global coordinates for every successfully approached artifact.16 Before triggering an approach for a new detection, the logic checks the Euclidean distance between the new detection's $(x, y)$ coordinate and all coordinates in the "visited" set.36 If the distance is below a threshold (e.g., 0.5 meters), the detection is ignored as a duplicate.36

## **Handling False Detections and Robustness**

To minimize false detections, a "confirmation counter" is implemented. An artifact is only considered "found" if it is detected in at least 5-10 consecutive frames.9 This prevents transient visual noise from triggering incorrect robot behaviors. Additionally, the robot should perform a 360-degree scan (using the navigator.spin() method) if it reaches a waypoint without finding any artifacts, ensuring that no objects are missed due to the camera's FOV limitations.22

## **Human-Robot Interaction and Audio Output**

The project specifies that the robot must "greet" the face and "say the color" of the ring. Since Task 1 is evaluated on robustness and repeatability, an offline Text-To-Speech (TTS) solution is preferred over cloud-based services to avoid latency or connectivity issues during the competition.40

## **Implementing pyttsx3 in Python**

The pyttsx3 library is a cross-platform, offline TTS engine that works well on the Raspberry Pi 4\.40 It supports multiple voices and adjustable speech rates.42

* **Greeting Logic:** When a face is approached (determined by the robot's proximity to the face's map coordinate), the logic node publishes a string to a dedicated /speech topic.43  
* **Color Announcement:** For rings, the color is passed dynamically to the TTS engine (e.g., engine.say("I have found a green ring")).42

## **Create 3 HMI and Feedback**

The TurtleBot 4 Lite utilizes the Create 3's lightring and interface buttons for supplementary feedback.6 The lightring can be programmed to change colors based on the artifact type or detection status.6 For instance, the ring could pulse white while searching, flash green when a face is greeted, and match the ring's color (Red/Blue/Green) when a ring is detected.6 This provides clear visual confirmation for judges during the evaluation.45

## **Transitioning from Simulation to Real Robot**

A major component of the project is ensuring that the code developed in the simulation environment (Gazebo Harmonic) works on the physical TurtleBot 4 with minimal modification.2

## **Simulation in Gazebo Harmonic**

The simulation provides a controlled environment to test the SLAM and Nav2 configurations.2 The fenced area and artifacts (printed faces and rings) must be modeled as Gazebo entities with appropriate textures.14

* **Sensors:** The turtlebot4\_gz\_bringup package launches the robot in simulation, publishing topics that mirror the hardware's API.6  
* **Parameters:** It is critical to set use\_sim\_time:=True in all launch files when running in Gazebo to ensure the ROS nodes synchronize with the simulator's clock rather than the system clock.13

## **Physical Robot Deployment Challenges**

The transition to hardware introduces challenges such as sensor noise, lighting variability, and networking jitter.47

* **CPU Management:** Running YOLOv8 on the Raspberry Pi 4 can consume significant resources, potentially causing the Nav2 controllers to miss their update deadlines.9 Offloading vision tasks to a remote PC (connected to the same ROS domain) can alleviate this.48  
* **Lidar Reflections:** In a fenced area, the Lidar may experience reflections from glossy surfaces or windows. The laser\_filters package can be used to clip invalid or "ghost" readings outside the expected arena dimensions.15  
* **Communication Latency:** The default RMW (FastDDS) should be tuned for Wi-Fi performance by reducing the frequency of high-bandwidth topics like the point cloud or raw images if they are not strictly needed for the detection logic.9

## **Evaluation Criteria and Mission Success**

The mission is evaluated on a total of 30 points (15 for simulation and 15 for the physical robot), with a focus on robustness and repeatability.

## **Achieving Robustness**

Robustness is defined as the system's ability to handle unexpected conditions without failure. This includes:

* **Obstacle Avoidance:** The robot must not hit the fence or any internal obstacles while searching or approaching artifacts.17  
* **No False Detections:** The use of morphological filters for rings and high confidence thresholds for faces ensures that the robot only interacts with valid targets.25  
* **Recovery Behaviors:** Nav2's built-in recovery behaviors (e.g., clearing costmaps or spinning) allow the robot to "un-stuck" itself if it encounters a difficult path.17

## **Ensuring Repeatability**

Repeatability requires the robot to perform the task identically across multiple runs. This is achieved by:

* **Deterministic State Logic:** The FSM ensures a consistent sequence of actions regardless of the artifact placement.33  
* **Accurate Localization:** Using slam\_toolbox to generate a high-resolution map and Nav2 for precise goal reaching ensures the robot arrives at the correct coordinates every time.13  
* **Persistence:** By tracking visited artifact locations in the map frame, the robot effectively manages its mission progress and stops only when the specific count of 3 faces and 2 rings is met.16

## **Technical Roadmap for Semester Execution**

The project timeline is structured around two tasks, with Task 1 serving as the baseline for all future autonomy on the TurtleBot 4\.

1. **Initial Setup:** Update the Create 3 firmware to I.0.0 and install ROS 2 Jazzy on both the Raspberry Pi and the development PC.2  
2. **Mapping and Lidar Calibration:** Perform SLAM in the fenced area, tuning Lidar ranges to ensure the fence is represented as a solid boundary.13  
3. **Vision Pipeline Development:** Implement YOLOv8 for face detection and HSV thresholding for rings, testing first in simulation then on hardware.9  
4. **Navigation and Approach Logic:** Integrate the TurtleBot4Navigator with the vision nodes, implementing the pixel-to-3D projection for artifact localization.22  
5. **State Machine Implementation:** Build the SMACH FSM to manage the mission flow and coordinate counters for detection.33  
6. **HMI and Feedback:** Integrate the pyttsx3 audio output and Create 3 lightring patterns for artifact interaction.6  
7. **Final Testing and Benchmarking:** Execute full mission runs in simulation and on the real robot, adjusting parameters to maximize the robustness scores.15

By adhering to this structured framework, the team of university students can develop a highly reliable autonomous system that fully utilizes the spatial AI and navigation capabilities of the TurtleBot 4 under the ROS 2 Jazzy distribution.1 The successful completion of Task 1 will provide a robust technical foundation for the more advanced challenges presented in the subsequent Task 2\.6
