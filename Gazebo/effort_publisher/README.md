# effort_publisher_node

## Overview
This package contains the controller node. The node publishes the torque to all the five joints. The joints are effort_controller joints. The effort_publisher_node.cpp contains the controller code. The node subscribes to joint_state_controller to get the current position, velocity and effort. It receives the data at 1000Hz frquency which can be change in the 
/Interbotix_src/interbotix_ros_arms/interbotix_gazebo/config/wx200_gazebo_controlllers.yaml file. Based on the required end affector position the node calculates the torque to be applied on the joint using inverse kinematics, Trajectory planning and torque equations.

## Usage

This package is automatically launch when the gazebo model is launched by typing the following command in terminal
```
$ roslaunch interbotix_gazebo gazebo.launch
```
To run this package seperately, type the line below in a terminal(considering the rosmaster is running).
```
$ rosrun effort_publisher effort_publisher_node
```

The reference position for all the joints can be manipulated by entering the following character in the ubuntu terminal.
char 'q' :  waist_ref =  waist_ref + 0.010
char 'a' :  waist_ref =  waist_ref - 0.010
char 'w' :  shoulder_ref =  shoulder_ref + 0.010
char 's' :  shoulder_ref =  shoulder_ref - 0.010
char 'e' :  elbow_ref =  elbow_ref + 0.010
char 'd' :  elbow_ref =  elbow_ref - 0.010
char 'r' :  wrist_angle_ref =  wrist_angle_ref + 0.010
char 'f' :  wrist_angle_ref =  wrist_angle_ref - 0.010
char 't' :  wrist_rotation_ref =  wrist_rotation_ref + 0.010
char 'g' :  wrist_rotation_ref =  wrist_rotation_ref - 0.010

After the gazebo file is terminated the package displays five graphs, related to the reference and current position of the respective joints. 
