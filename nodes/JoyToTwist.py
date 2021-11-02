#!/usr/bin/env python3
# --- JoyToTwist.py ------
# Version vom 02.11.2021 by LF
# ----------------------------

import rospy
from JoyToTwistClassFile import JoyToTwist


if __name__ == '__main__':
    try:
        joy = JoyToTwist()
    except rospy.ROSInterruptException:
        rospy.loginfo(" Error ")
