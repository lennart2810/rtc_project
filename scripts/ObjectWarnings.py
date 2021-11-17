#!/usr/bin/env python3

# -- ObjectWarnings.py --
# Version vom 17.11.2021 by LF
# ----------------------------

import yaml

path = "/home/lennart/catkin_ws/src/rtc_project/config/StatusToTurtleTwist.yaml"

with open(path, "r") as stream:
    try:
        yaml_file = (yaml.safe_load(stream))
        print(yaml_file)
    except yaml.YAMLError as exc:
        print(exc)
