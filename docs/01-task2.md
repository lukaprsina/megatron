# 2026 Final task – Industry 5.0

- **Setup:**
  - „A factory plant“ scene. Paths, working cells, conveyor belts, obstacles.
  - Several working cells, containing objects to be inspected, barrels, rings.
  - Several workers, CTO.

- **Goal:**
  - Help human workers by executing visual inspection and counting tasks, while navigating safely and communicating clearly.

- **Task:**
  - Navigate safely through the factory while staying on permitted paths.
  - Detect and approach workers to receive task instructions.
  - Identify and localise the requested working cell.
  - Capture high-resolution images of the working cell and its objects.
  - Execute the prescribed tasks (defect detection, counting, inspection).
  - Generate an inspection report and present it to the CTO.

# Navigation

- Explore the factory plant
  - Avoid all obstacles
- In the first room
  - Use standard (goal-based) navigation
    - by using a predetermined path or
    - preferably implement the autonomous exploration of space
  - Do not cross the yellow lines (!)
  - You can use the range sensor
  - You can set the goal at the exit from this room
- In the second room
  - Follow the blue lines
  - Use visual survailence
  - Use the second camera pointed downwards to see what is there
  - Find CTO at the end of the blue line

Image: Top-down map of a factory plant layout with yellow and blue navigation lines

Image: Close-up of a robotic platform with a camera arm

# Perception

- Line detection
  - To detect forbidden zones
  - For line following
  - To determine the correct working cell

Colored lines: yellow, blue, red, green

Three barrels in a simulated environment, one horizontal with a green spill

- Inspect the barrels
  - Locate all barrels (in the first room only)
  - Recognise barrel colour
  - Determine orientation (horisontal or vertical)
  - Check whether the horisontal barrel is leaking
  - Approach horisontal barrels and issue a warning if a spill is detected

- Count the rings
  - Detect all (3D) rings (discard 2D printed circles)
  - Recognise ring colour

A wooden crate with a green ring and a blue ring in the background

- Find all persons
  - Recognise the person's name and gender
  - Approach and engage in dialogue

- The number of any element present is not known in advance

- Colours: red, green, blue, yellow, purple, orange, brown.

- Anomaly detection

# Perception and dialogue

- Engage in a dialogue with people to determine which task to perform.
  - Approach a person
  - Recognise the person's name
    - The photos and names of all persons will be given in advance
  - Recognise the person's gender
  - Ask the person what task to perform
    - Barrels inspection (in the first room only)
    - Count the rings (in the first room only)
    - Anomaly detection in red|green working cell
    - Nothing
  - Men always tell the task directly
  - Women might reconsider their choice
    - The robot has to ask them twice or more
    - When the same task is named twice, this is the task you are looking for
    - The person can also confirm the selected task with „yes.“

Image: Portrait of a smiling woman with blonde hair

Image: Portrait of a man with glasses and grey hair

# Sample dialogues

- R: „Hi man, which task should I perform?“
- M1: „Detect anomalies in the green cell.“
- R: „OK. I will detect anomalies in the green cell.“

---

- R: „Hi man, which task should I perform?“
- M2: „Inspect the barrels.“
- R: „OK. I will inspect the barrels.“
- R: „Alert! Alert! This barrel is leaking!“

---

- R: „Hi woman, which task should I perform?“
- W1: „Count the rings.“
- R: „Are you sure?“
- W1: „Well, maybe you should inspect the barrels?“
- R: „OK, the barrels then. Are you sure?“
- W1: „Ah, not really. The count the rings.“
- R: „OK. I will count the rings.“

---

- R: „Hi woman, which task should I perform?“
- W2: „Detect anomalies in the red cell.“
- R: „Are you sure?“
- W2: „Yes, I‘m sure.“
- R: „OK. I will inspect anomalies in the red cell.“

---

- R: „Hi woman, which task should I perform?“
- W3: „Inspect anomalies in the red cell.“
- R: „Are you sure?“
- W3: „Well, inspect anomalies in the green cell instead.“
- R: „OK, the green cell then. Are you sure?“
- W3: „Ah, not really. Just count the rings.“.
- R: „Are you sure now?“
- W3: „Yes.“
- R: „OK. I will count the rings.“

# Anomaly detection

- Locate the correct working cell
    - Red or green
- Extend the camera above the objects
- Move along the working cell
- Take images of all objects (tiles)
- Detect anomalies (defects)

Image: 3D simulation of a robotic arm over a conveyor belt with tiles
Image: Top-down map view of the working cell layout
Image: Perspective view of the simulated industrial environment with conveyor belts and boxes

# Anomaly detection

- Create/train a surface anomaly detector
  - Based on PCA, AE, VAE, normalising flow,difusion models, discriminative, reconstructive methods, etc.
  - Supervised or unsupervised learning
  - Training a couple of test images will be given in advance
  - Train an anomaly detection model
  - Two levels of difficulty:
    - Severe degradations (cracks, etc.)
    - Mild degradations (more difficult to detect)
  - Take a number of training images and augment them
    - In Gazebo, in simulation, as you want
    - Taken from the frontal view or from side views as well
    - Vary the illumination conditions etc.
  - Use or design a method for anomaly detection
  - Train the model

Image: Surface texture sample 1

Image: Surface texture sample 2

Image: Surface texture sample 3

Image: Surface texture sample 4

Image: Surface texture with cracks

Image: Segmented anomaly mask

- Evaluate the anomaly detection model
  - Create also test images
  - Evaluate the model under different environments
- The robot shuld detect and segment the anomaly
- Can be done (almost) completely outside ROS and Gazebo

# Shortcuts

- You may not implement all the functionalities (for a lower grade)
- You may not implement the autonomous exploration of space
    - And can use fixed goals instead
    - The first and the exit point in the first room can always be handcoded
- The robot can skip the dialogue with a person
    - And read the task requested in the QR code next to the person
- The robot does not need to classify the faces according to gender
    - You assume that all the persons answer the right answer right away
- The robot does not need to recognise persons
    - It just enumerates the persons
- The robot doe not need to avoid crossing yellow line
    - It can navigate cross the as well
- The robot does not need to follow the blue line
    - You can set a goal in front of the CTO in the second room
- The robot can skip one of the tasks
    - Either ring counting, barrel inspection or anomaly detection
- The robot can skip anomaly segmentation
    - It can report defects o the level of images only
- The robot does not need to create the inspection catalogue
    - It just reports the results during operation

# Demonstration

Map visualization showing robot path and detected objects with numerical labels

- Demonstrate what is going on in the robot
- Visualize in RViz:
    - Locations of detected faces, rings, barrels, tiles
    - Recognised colours, faces
    - Navigation goals, path plans
    - Current sensor readings (images, Lidar)
    - Show live strem from both cameras
- Show the dialogue in a separate window
- Show the reasoning process
- Show also the current environment in Gazebo
- Information in the console does not suffice!
- Use also sound to demonstrate what is going on

> R: „Hi woman, which task should I perform?“
> W2: „Detect anomalies in the red cell.“
> R: „Are you sure?“
> W2: „Yes, I‘m sure.“
> R: „OK. I will inspect anomalies in the red cell.“

Image: Screenshot of RViz interface showing robot model, map, and sensor data

Image: Screenshot of Gazebo simulation environment showing a 3D room with objects

# Evaluation protocol

- The evaluation course will be set up in advance
    - The main setup will not change
- The teams will be allowed to build the map in advance
    - The yellow lines will not be moved, they can be cosidered to alter the map
- The faces, rings, cylinders and tiles will be positioned on the day of the evaluation
    - The size and colours of the rings and cylinders are known in advance
- The robot has to operate completely autonomously
    - only the initial positioning is allowed
    - Also the exit from the first room can be hardcoded
    - (and the optional answering by typing the text)
- The robot starts at the starting position in th first room

- Every team will have allocated 15-20 minutes to show the performance of the robot
- Show the performance of your robot on the lab computer (not on a slow laptop)
- The evaluation will take place in the last week of the May

# Grading

- Must do:
  - Goal-based navigation (1 pt)
  - Face detection (2 pts)
  - Ring detection (2 pts)
  - Cylinder detection (2 pts)
  - Colour recognition (1 pt)
  - Approaching faces (1 pt)
  - Speech synthesis (1 pt)

- Should do:
  - Face recognition (1 pt)
  - Gender recognition (1 pt)
  - Auton. space exploration (1 pt)
  - Not crossing yellow line (2 pts)
  - Follow blue line (2 pts)
  - Correct cell detection (1 pt)
  - Tile detection (1 pt)
  - Defect detection (3 pt)
  - Dialogue with ASR (2 pts)
  - Creating the inspection report (1 pt)

- Performance evaluation
  - Navigation (1 pt)
  - Reasoning (1 pt)
  - Visualisation (1 pt)
  - Robustness (2 pts)
  - Relative speed (2 pts)
  - Overall impression (3 pts)

- Points:
  - Must do: 10
  - Should do: 15
  - Performance: 10
  - Total: 35

## Task 2 goals

- The main goals of the third task and evaluation are:
    - to navigate the robot around
    - to detect faces in 2D
    - to detect and count objects (rings and cylinders) in 3D
    - to learn and recognize colours
    - to classify faces according to gender
    - to recognise faces
    - to detect and follow lines
    - to detect defects on the tiles
    - to do simple reasoning
    - to do simple dialogue processing
    - to plan adequate actions
    - to fine manoeuvre the robot
    - to do simple mobile manipulation
    - **to integrate all functionalities into a coherent system**