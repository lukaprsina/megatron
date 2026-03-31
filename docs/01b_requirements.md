We are a team of three computer science students at university, working on a semester project involving TurtleBot4 and ROS2 Jazzy (Python only). We are starting Task 1, with Task 2 to follow after the competition. Search the web, evaluate options, and create an actionable plan. We have just under two weeks, but we can use LLM assistants. In general, do not blindly choose the quickest solutions, but also avoid unnecessary complexity. I have attached our professor's repository README files, which are mandatory reading; mostly prefer their tools and libraries, as we have already learned to use them.

Requirements from the slides:

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
