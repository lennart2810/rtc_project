#!/bin/bash

cmd="rosrun map_server map_saver -f"
path="$1"
rosrun="${cmd} ${path}"
#echo "$rosrun"
gnome-terminal -e "$rosrun"
