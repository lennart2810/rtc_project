#!/bin/bash

map_file="$1"
cmd="roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=${map_file}.yaml"
echo "$cmd"
gnome-terminal -e "$cmd"
