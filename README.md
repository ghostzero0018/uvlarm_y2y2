# uvlarm_y2y2

## Project description

Project_Robot

Robot number: Pibot22

Authors: Zhiyan.PIAO, Yin.LI, Yinpu.CHEN

The grp_pibot file for Challenge 1 is in the src folder.

## Developers list

```sh
ghostzero0018 -- Laptop of IMT Nord Europe named 'DATA', used by all members of the group
Ghostzero00018 -- Yinpu.CHEN
YinLi-Y2Y2 -- Yin.LI
ZhiyanPiao-Y2Y2 -- Zhiyan.PIAO
```

## Dependencies

This work relies on Stage Simulator.
You need to follow [profs tutorial](https://imt-mobisyst.github.io/lct-mobile-robot/tuto-kick-off/simulation/)
to have it installed.

## ROS2 Structure Developer instructions for others in the group

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
If you forget some Linux terminal commands, please refer to [OS and Shell]([text](https://imt-mobisyst.github.io/lct-mobile-robot/tuto-kick-off/basics/))

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
