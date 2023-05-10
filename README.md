
Search-and-Rescue-swarm-robots-with-YOLOv8-and-ROS2
This repository contains the files for a search and rescue swarm robots system using YOLOv8 and ROS2.

Getting Started
Download the ZIP file of this repository.

Extract the ZIP file, and copy the src folder.

Create a new workspace using the following command:

sh
Copy code
mkdir -p ws/src
Paste the src folder from step 2 into the ws folder you just created.

Navigate to the ws folder and build the project using colcon build:

sh
Copy code
cd ws
colcon build
The other two files will be built automatically.

Source the directory in the terminal:
sh
Copy code
source install/setup.bash
Launch the project using the appropriate launch file:
sh
Copy code
ros2 launch launchboth yolobot.launch

Repository Structure
src: Contains the source code for the project, including the YOLOv8 object detection algorithm, swarm robot navigation, and other necessary components.

launch: Contains the launch files for running the project in the ROS2 and Gazebo simulation environment.

config: Contains configuration files for the YOLOv8 object detection algorithm and other system settings.

Dependencies
ROS2 (tested with Foxy Fitzroy)
Gazebo
YOLOv8
Please ensure that you have installed the required dependencies before building and running the project.

