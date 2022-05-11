# interbotix_descriptions

## Overview
This package contains the URDF and meshes Interbotix wx200 arm. The STL files for the arm are located in a unique folder inside the [meshes](meshes/) directory. Also in the 'meshes' directory is the [interbotix_black.png](meshes/interbotix_black.png) picture. The appearance and texture of the robots come from this picture. Next, the URDF for the robot is located in the [urdf](urdf/) directory. They are written in 'xacro' format so that users have the ability to customize what parts of the URDF get loaded to the parameter server (see the 'Usage' section below for details). Note that all the other ROS packages in the repo reference this package to launch the robot model.


## Usage
This package is called automatically when the launch file of the gazebo launched. This package can be called using the command "roslaunch interbotix_gazebo gazebo.launch" in the standalone mode or using the command executed using matlab "roslaunch interbotix_gazebo gazebo_matlab.launch &" in the GUI mode. This package is responsible for launching the robotic arm model in the gazebo world.

