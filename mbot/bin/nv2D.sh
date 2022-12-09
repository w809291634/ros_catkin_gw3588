#!/bin/bash

#xfce4-terminal --title="ros" --command="roslaunch mbot demo-gmaping-movebase.launch"
sudo chmod 666 /dev/ttyUSB*
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch mbot nav_demo.launch


