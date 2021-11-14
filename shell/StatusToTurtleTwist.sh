#!/bin/bash

gnome-terminal -e "roslaunch rtc_project StatusToTurtleTwist.launch controller_layout:=$1 map_path:=$2 map_saver_path:=$3"
