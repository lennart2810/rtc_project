#!/usr/bin/env python3
# --- JoyToTwistClassFile.py ------
# Version vom 02.11.2021 by LF
# ----------------------------

# ----------------------------
# $ sudo jstest /dev/input/js0
# $ rosrun joy joy_node dev:=/dev/input/js0
# $ rosrun rtc JoyToTwist.launch
# ----------------------------

# ----------------------------
# Joy.axes[0] --> L3x
# Joy.axes[1] --> L3y
# Joy.axes[2] --> L2
# Joy.axes[3] --> R3x
# Joy.axes[4] --> R3y
# Joy.axes[5] --> R2
#
# Joy.buttons[0] --> X
# Joy.buttons[1] --> O
# ----------------------------

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class JoyToTwist(object):
    def __init__(self):

        self.pub_flag = True

        self.buttons_dict = {"options": 9}

        self.axes_dict = {"L3Y": 1, "R3X": 3}

        rospy.init_node('joy_to_twist_node', anonymous=True)

        rospy.Subscriber('joy', Joy, self.joy_callback)

        self.velocity_publisher = rospy.Publisher('/cmd_vel',
                                                  Twist, queue_size=10)

        self.vel_msg = Twist()

        self.rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            if self.pub_flag:
                self.velocity_publisher.publish(self.vel_msg)
                self.rate.sleep()

    def joy_callback(self, data):

        # "enable bzw. disable" velocity_publisher
        if data.buttons[self.buttons_dict["options"]]:
            self.pub_flag = not self.pub_flag

        self.vel_msg.linear.x = data.axes[self.axes_dict["L3Y"]] * 0.5
        self.vel_msg.angular.z = data.axes[self.axes_dict["R3X"]] * 1.5
