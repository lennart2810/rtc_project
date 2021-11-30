#!/bin/bash

map_path="$1"

rosrun="rosrun map_server map_saver -f ${map_path}"

gnome-terminal -e "${rosrun}"
echo "${rosrun}"