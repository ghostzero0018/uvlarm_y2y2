# uvlarm_y2y2

## Project description

Project_Robot

Robot number: Pibot22

Authors: Zhiyan.PIAO, Yin.LI, Yinpu.CHEN

The grp_pibot file for Challenge 1 is in src folder

## Dependencies

This work relies on Stage Simulator.
You need to follow [profs tutorial](https://imt-mobisyst.github.io/lct-mobile-robot/tuto-kick-off/simulation/)
to have it installed.

## ROS2 Structure Developper instructions for others in the group

Important: If you encounter some problems such as "No executable found", please follow the standard procedures in part 2 to solve the errors.

1.To syncronize (do the Push,Pull and Commit) with github...

```sh
git status 
git commit -a
git pull
git push
```

2.To correctly manage the structure of a ros2 pkg...
For more details you need to check [ROS Packages](https://imt-mobisyst.github.io/lct-mobile-robot/tuto-kick-off/package/)
and [Nodes and Topics](https://imt-mobisyst.github.io/lct-mobile-robot/tuto-kick-off/first-contact/)
to configure the packages in a proper way.

```sh
(1) If you want to create a pkg file with CMakeLists.txt and package.xml automatically generated, use

ros2 pkg create --build-type ament_cmake your_pkg  #Replace <your_pkg> with your own pkg name
colcon list  #For showing the list of existing files
colcon build  #Generate automaticlly build, log and install file, which are no need to be committed  and pushed

#Build: with temporaly generated files by the build process 
#Install: with generated ressources of the packages 
#Log: for logs

(2) After creating the pkg file, you need to add a script folder to store the scripts inside)

mkdir your_pkg/scripts  #Replace <your_pkg> with your own pkg name, here a scripts folder is created
touch your_pkg/script_1.py  # Create a script file (if it does not exist)
touch your_pkg/script_2.py  # Create another script file (if it does not exist)
...     
...
Tips: If you want to copy the previously created scripts and paste them in another foleder, do like this:

mkdir your_another_pkg/scripts  #Replace <your_another_pkg> with your another pkg name
cp your_pkg/script_1.py your_another_pkg/scripts/script_1
cp your_pkg/script_2.py your_another_pkg/scripts/script_2

(3) 



```

## Developpers list

```sh
ghostzero0018 -- Laptop of IMT Nord Europe named 'DATA',all operators in the group
Ghostzero00018 -- Yinpu.CHEN
YinLi-Y2Y2 -- Yin.LI
ZhiyanPiao-Y2Y2 -- Zhiyan.PIAO
```
