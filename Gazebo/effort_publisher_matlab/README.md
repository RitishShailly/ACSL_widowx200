# effort_publisher_matlab_node

## Overview
This package contains the controller node used to launch using the matlab GUI. This package is similar to the effort_publisher package. This package is created specifically for GUI. The end affector positions are updated in the GUI, which writes the X,Y and Z position of the end affector in a text file the node reads from the text file, calculates the required torque to be applied on each joints and publish the message.

## Usage
This package is automatically launch when the gazebo model is launched by executing the following command in matlab GUI

```
$ roslaunch interbotix_gazebo gazebo_matlab.launch &
```

To run this package seperately, type the line below in a terminal.
```
$ rosrun effort_publisher_matlab effort_publisher_matlab_node
```
