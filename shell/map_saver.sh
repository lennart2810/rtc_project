#!/bin/bash

path="$1"

cmd1="rosrun map_server map_saver -f"
rosrun1="${cmd1} ${path}"

cmd2="rosrun map_server map_server ${path}.yaml"


echo "$rosrun1"
echo "$cmd2"
gnome-terminal -e "$rosrun1"
#gnome-terminal -e "$cmd2"


