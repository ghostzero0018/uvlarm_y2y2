# uvlarm_y2y2 - Robot n°22

## Project Description for the UVLARM Course

Project Robot based on ROS2 env, OS Linux

Robot callsign: Pibot22

Authors: Zhiyan.PIAO, Yin.LI, Yinpu.CHEN

## Developers List

ghostzero0018 -- Laptop of IMT Nord Europe Douai named 'DATA', used by all members of the group

Ghostzero00018 -- Yinpu.CHEN

YinLi-Y2Y2 -- Yin.LI

ZhiyanPiao-Y2Y2 -- Zhiyan.PIAO

## Project video presentation

Click on the link to see the video:

## Project Launch File Structure

- The simulation launch file for Challenge 1 is src\grp_pibot22\launch\simulation_v1_launch.yaml.

- The reality launch file for Challenge 1 is src\grp_pibot22\launch\tbot_v1_launch.yaml.

- The vision launch file for Challenge 2 is src\grp_pibot22\launch\vision_launch.yaml.

- The reality launch file for Challenge 2 is src\grp_pibot22\launch\tbot_v2_launch.yaml.

## Package Contents

This document outlines the structure and contents of this software package. The package is organized into several directories, each containing specific types of files crucial for the operation and visualization within the ROS ecosystem and rviz2.

## Directory Structure

### `config`

Contains configuration files for rviz2 visualizations.

- **`challenge1_rviz_config.rviz`**: Configuration for challenge 1 demonstration.

### `launch`

Includes YAML files for launching various components of the robot system (In the folder only containing the launch files that is validated).

- **`simulation_v1_launch.yaml`**: Launches the simulation setup for challenge 1. (For running this launch file you need to have **`basic_node`** pkg installed)
- **`tbot_v1_launch.yaml`**: Launches the reality test for challenge 1.
- **`vision_launch.yaml`**: Launches the vision detection part for challenge 2.
- **`tbot_v2_launch.yaml`**: Launches the reality test for challenge 2.

### `scripts`

Contains scripts that perform specific tasks within the robot system.

- **`basic_move_test`**: Script for controlling the robot's test-version movement.
- **`robot_camera`**: Handles image capture and analysis.
- **`detect_ghost`**: Detects green objects in the camera feed.
- **`supercamera`**: Detects green objects and giving distance feedback in the camera window.
- **`supermove`**: Better movement.

### Root Files

- **`CMakeLists.txt`**: Specifies the build configuration for the CMake build system.
- **`package.xml`**: Provides meta-information about the package such as version, dependencies, etc.
- **`README.md`**: This file, providing an overview of the package contents and structure.

### Additional Information

For more detailed information on each component and how to use them, refer to the specific configuration files and scripts contained within each directory.

## Requirements & Installation

This project integrates ROS2, Python, OpenCV, and Intel RealSense. To run it on another machine (Linux OS recommanded), the following dependencies are required:

### 1. ROS2 Installation (Iron)

Follow the official ROS installation instructions to install ROS2 Iron:

- [ROS2 Installation Guide (Iron)](https://docs.ros.org/en/iron/Installation.html)

After installing ROS2 Iron, create your ROS workspace and clone this repository into it. Use the commands below to do so.

```bash
mkdir -p ~/ros_space/src
cd ~/ros_space/src
git clone <repository-url> 
cd ~/ros_space
colcon build
source install/setup.bash
```

### 2. Intel RealSense SDK 2.0

Install the necessary RealSense drivers and tools:

- Install `librealsense2-dev` and RealSense tools.
- Verify hardware detection using `realsense-viewer`:

Use the commands below to do so.

```bash
sudo apt install librealsense2-dev
realsense-viewer
```

### 3. Python 3 Dependencies

Make sure you have Python 3 installed. Then, install the following Python packages with commands below:

- Numpy:

  ```bash
  pip install numpy
  ```

- OpenCV:

  ```bash
  pip install opencv-python
  ```

- pyrealsense2:

  ```bash
  pip install pyrealsense2
  ```

- cv_bridge (ROS2 package):

  ```bash
  sudo apt install ros-iron-cv-bridge
  ```

### 4. ROS2 Python Dependencies

Install the necessary ROS2 Python dependencies:

- `rclpy` : ROS2 Python client library for writing ROS nodes.
- `sensor_msgs` : For image data types (`Image`) used in ROS topics.
- `std_msgs` : For standard message types such as `String` and for publishing detection messages.
- `cv_bridge` : ROS2 package for converting between OpenCV images and ROS image messages.

Install these dependencies with the commands below:

```bash
sudo apt install ros-iron-rclpy ros-iron-sensor-msgs ros-iron-std-msgs ros-iron-cv-bridge
```

### 5. Robot-Specific Message Drivers

Install drivers to interpret robot-specific messages (bumper, laser, etc.) with commands below:

```bash
cd $ROS_WORKSPACE
git clone https://github.com/imt-mobisyst/pkg-interfaces.git
colcon build --base-path pkg-interfaces
source ./install/setup.bash
```

### 6. Clone Necessary Repositories

Clone the required repositories into your workspace and build them with commands below:

```bash
cd ~/ros_space
git clone https://github.com/imt-mobisyst/pkg-tsim
colcon build
source ./install/setup.bash
```

### 7. Gazebo Installation

If you plan to use Gazebo for simulation, then you need to install the following Gazebo-related packages with commands below:

- **Gazebo**:

  ```bash
  sudo apt install gazebo=11.10.2+dfsg-1
  sudo apt install gazebo-common=11.10.2+dfsg-1
  sudo apt install gazebo-plugin-base=11.10.2+dfsg-1
  sudo apt install libgazebo-dev=11.10.2+dfsg-1
  sudo apt install libgazebo11:amd64=11.10.2+dfsg-1
  ```

- **ROS2 Gazebo Packages**:

  ```bash
  sudo apt install ros-iron-gazebo-dev=3.7.0-3jammy.20230622.191804
  sudo apt install ros-iron-gazebo-msgs=3.7.0-3jammy.20231117.090251
  sudo apt install ros-iron-gazebo-plugins=3.7.0-3jammy.20231117.111548
  sudo apt install ros-iron-gazebo-ros=3.7.0-3jammy.20231117.104944
  sudo apt install ros-iron-gazebo-ros-pkgs=3.7.0-3jammy.20231117.114324
  sudo apt install ros-iron-turtlebot3-gazebo
  ```

These packages are necessary for integrating Gazebo into ROS2, and for using Gazebo to do robot simulation in the project.

### 8. Compilation and Execution

- Clone this repository into your ROS2 workspace.
- From the root of the workspace, build the project:

```bash
colcon build
source install/setup.bash
```

- Launch a yaml launch file to automatically run several scripts in order to perform the desired funciton, for example:

```bash
ros2 launch tutorial_pkg simulation_launch.yaml
```

---

By following these steps, you should be able to successfully set up and run the project on your own machine.

### 9. Other dependency that might be useful

This work relies on Stage Simulator.
You need to follow [tutorial](https://imt-mobisyst.github.io/lct-mobile-robot/tuto-kick-off/simulation/)
to have it installed.

This work also relies on Pkg-basic.

## Project Structure and Integration Developer's Instructions for Others in the Group

(If you are the beginner of ROS2 in Linux, then it is also recommended to read this)

Important: If you encounter some problems such as "No executable found", please follow the standard procedures in part 2 to solve the errors.

1. To synchronize (do the Push, Pull, and Commit) with GitHub:

```sh
git status # Check whether local codes are up-to-date with the git repo or not
git commit -a # Commit (declare)the changes in the code
git pull # Get the codes from the git repo
git push # Send the local codes to the git repo
```

To correctly manage the structure of a ROS2 package:
For more details, you need to check [ROS Packages](https://imt-mobisyst.github.io/lct-mobile-robot/tuto-kick-off/package/)
and [Nodes and Topics](https://imt-mobisyst.github.io/lct-mobile-robot/tuto-kick-off/first-contact/)
to configure the packages in a proper way.
If you forget some Linux terminal commands, please refer to [OS and Shell](https://imt-mobisyst.github.io/lct-mobile-robot/tuto-kick-off/basics/)

```sh
# (1) If you want to create a package file with CMakeLists.txt and package.xml automatically generated, use these commands:
# Create a ROS2 package
ros2 pkg create --build-type ament_cmake your_pkg  # Replace <your_pkg> with your own package name
colcon list  # For showing the list of existing files
colcon build  # Automatically generate build, log, and install files; remember to always colcon build before running again after modification

# Build: with temporarily generated files by the build process 
# Install: with generated resources of the packages 
# Log: for logs

# (2) After creating the package file, create a folder named `scripts` inside your root directory 
# (for this course it is uvlarm_y2y2) to store the Python scripts. Note that scripts are essential as they serve as Nodes.
# Create a playground folder
mkdir playground  # Playground means that in this folder we just test the functionality of Python 3 scripts not run in ROS2
touch playground/script_1.py  # Create a script file (if it does not exist)
touch playground/script_2.py  # Create another script file (if it does not exist)
# ...existing code...

# Tip1: If you want to copy the previously created scripts 
# and place them into another folder
# (here is the package we are working in), perform like this:

mkdir your_another_pkg/scripts  # Replace <your_another_pkg> with your another package name
cp playground/script_1.py your_another_pkg/scripts/script_1  # Copy and paste the content of previously created script_1.py into script_1 of the project package 
cp playground/script_2.py your_another_pkg/scripts/script_2  # Copy and paste the content of previously created script_2.py into script_2 of the project package

# Tip2: Also, we need to inform the specific interpreter that can process our script (i.e., python3). 
# Add a line

#!/usr/bin/python3 

# at the first line of each script. 

# !
# is a shebang meaning that the rest of the line is used to determine the program to interpret the content of the current file. 
# /usr/bin/python3 is simply the result of the command whereis python3.

# (3) Modify CMakeLists.txt
# Next, we have to modify CMakeLists.txt to state that script_1.py and script_2.py should be installed as programs (i.e., in the appropriate destination to make them reachable by ros2 command).

# Add a python scripts section to your CMakeLists.txt file:

# Python scripts
install(PROGRAMS scripts/script_1.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/script_2.py DESTINATION lib/${PROJECT_NAME})

# Now you can build again your ROS workspace. The install directory contains your_another_pkg with everything inside.

# Build again
colcon build
ls install/
ls install/your_another_pkg/lib/your_another_pkg/

# (4) ROS2 run command to run the scripts
# To use your package with ros2 commands, you have to update your bash environment. Then it would be possible to start your node with ros2 run command.

# Source and then run
source ./install/local_setup.bash  # Always remember to source before running
ros2 run your_another_pkg script_1  # or script_2

# (5) Launch file
# ROS proposes a launch mechanism to start in one command a configuration of several nodes. Launch files can be defined with markup language (XML or YAML) or python3 for more complex launch scenarios.

# YAML provides the simplest syntax to write launch files. The yaml.org (https://yaml.org/) gives an example of YAML resources. Similarly to Python, it relies on indentation to mark the ownership of elements.

# ROS YAML relies on predefined marks, key works: 'launch' as the first element and composed by a list of nodes. Minimal configuration for a node includes pkg and exec attributes to identify the node to start.

# A simple launch file for script_1/script_2 will be:
launch:
    - node:
        pkg: "your_another_pkg"
        exec: "script_1"
    - node:
        pkg: "your_another_pkg"
        exec: "script_2"

# The file has to be set in a launch directory into your package and with a name ending by _launch.yaml, converse_launch.yaml for instance.
# For example:
mkdir your_another_pkg/launch
touch your_another_pkg/launch/launch_1.yaml

# At this point, the launch file can be run using ros2 launch commands
# Launch command
ros2 launch your_another_pkg launch_1.yaml

# By adding launch resources to your package with CMakeLists.txt configuration file, you make launch files easier to find.

# In CMakeLists.txt file:
# Install resource files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/ # No need to change PROJECT_NAME
)

# Then in the terminal:
colcon build
ros2 launch your_another_pkg launch_1.yaml
```
