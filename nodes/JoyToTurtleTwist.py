#!/usr/bin/env python3
# --- JoyToTwistClassFile.py ------
# Version vom 02.11.2021 by LF
# ----------------------------

# ----------------------------
# $ sudo jstest /dev/input/js0
# $ rosrun joy joy_node dev:=/dev/input/js0
# ----------------------------

import sys
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class JoyToTwist(object):
    def __init__(self, pub_cmd):

        self.buttons_dict = {"X": 0, "O": 1, "options": 9}
        self.axes_dict = {"L3X": 0, "L3Y": 1, "L2": 2,
                          "R3X": 3, "R3Y": 4, "R2": 5}

        rospy.init_node('joy_to_twist_node', anonymous=True)
        rospy.Subscriber('joy', Joy, self.joy_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Argument aus launch file zuordnen
        # True --> pub von Anfang an
        if pub_cmd == 'True':
            self.pub_flag = True
        elif pub_cmd == 'False':
            self.pub_flag = False

        self.vel_msg = Twist()
        self.rate = rospy.Rate(10)

        # Scales aus JoyToTurtleTwist.yaml
        self.scales = rospy.get_param('~scales')

        while not rospy.is_shutdown():

            if self.pub_flag:
                self.vel_pub.publish(self.vel_msg)
                self.rate.sleep()

    def joy_callback(self, data):

        # "enable bzw. disable" vel_pub
        if data.buttons[self.buttons_dict["options"]]:
            self.pub_flag = not self.pub_flag

        self.vel_msg.linear.x = data.axes[self.axes_dict["L3Y"]] * self.scales["linear"]["x"]
        self.vel_msg.angular.z = data.axes[self.axes_dict["R3X"]] * self.scales["angular"]["z"]


if __name__ == '__main__':

    arg = sys.argv
    del arg[0]  # erste und letzten beiden Argumente sind irrelevant
    del arg[-2:]

    try:
        joy = JoyToTwist(arg[0])  # erste Argument aus launch file übergeben
    except rospy.ROSInterruptException:
        rospy.loginfo(" Error ")
