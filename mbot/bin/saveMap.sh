#!/bin/sh
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun map_server map_saver -f /home/zonesion/catkin_ws/src/mbot/maps/map_gmapping
sync
echo "Save Map to mbot/maps/map_gmapping."

sleep 5