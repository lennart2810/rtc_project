#!/bin/bash

# turtlebot3_slam_node und rviz schließen, um navigation zu starten
input="$1"
echo "$input"
gnome-terminal -e "rosnode kill /turtlebot3_slam_gmapping"
gnome-terminal -e "rosnode kill /rviz_slam"
