**Abstract**
This project presents the design, implementation, and evaluation of a search and rescue swarm robots system using YOLOv8 and ROS2. The aim of this project is to improve search and rescue operations by leveraging the efficiency, coverage, and safety benefits provided by swarm robots and YOLO object detection. The system consists of 11 robots, with one main robot, known as "yolobot," equipped with a camera and image processing capabilities using YOLOv8. The other 10 robots, called "box_bots," navigate to the detected person's location. Through the coordinated efforts of the swarm robots, the system demonstrates the potential for enhancing search and rescue missions using robotics and advanced object detection algorithms.


**Search-and-Rescue-swarm-robots-with-YOLOv8-and-ROS2**
This repository contains the files for a search and rescue swarm robots system using YOLOv8 and ROS2.

**Getting Started**
Download the ZIP file of this repository.

Extract the ZIP file, and copy the src folder.

**Create a new workspace using the following command:**


Copy code
mkdir -p ws/src
Paste the src folder from step 2 into the ws folder you just created.

**Navigate to the ws folder and build the project using colcon build:**


Copy code
cd ws
colcon build
The other two files will be built automatically.

**Source the directory in the terminal:**

Copy code
source install/setup.bash
Launch the project using the appropriate launch file:
Copy code
ros2 launch launchboth yolobot.launch

**Repository Structure**
src: Contains the source code for the project, including the YOLOv8 object detection algorithm, swarm robot navigation, and other necessary components.

launch: Contains the launch files for running the project in the ROS2 and Gazebo simulation environment.

config: Contains configuration files for the YOLOv8 object detection algorithm and other system settings.

**Dependencies**
ROS2 (tested with Foxy Fitzroy)
Gazebo
YOLOv8
Please ensure that you have installed the required dependencies before building and running the project.
