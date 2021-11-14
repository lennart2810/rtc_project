#!/usr/bin/env python3

# -- StatusToTurtleTwist.py --
# Version vom 13.11.2021 by LF
# ----------------------------

import sys
import rospy
import subprocess
from geometry_msgs.msg import Twist
from ds4_driver.msg import Status, Feedback
from sensor_msgs.msg import LaserScan
from math import isnan


class StatusToTurtleTwist(object):
    def __init__(self, layout, map_path, map_saver_path):

        self.map_path = map_path
        self.map_saver_path = map_saver_path  # map_saver.sh

        self.inputs = rospy.get_param('~inputs')
        self.layouts = rospy.get_param('~layouts')
        self.scales = rospy.get_param('~scales')
        self.rumble = rospy.get_param('~rumble')

        # gewähltes Controller-Layout bestimmen
        rospy.loginfo('controller_layout: %s', layout)
        my_item = {}
        for vel_type in self.inputs:
            for direction in self.inputs[vel_type]:
                my_item[vel_type] = direction+str(layout)

        # alle anderen Layouts rauswerfen
        for vel_type in self.layouts:
            delete_items = []
            for item in self.layouts[vel_type]:
                if item != my_item[vel_type]:
                    delete_items.append(item)
            for item in delete_items:
                self.layouts[vel_type].pop(item)

        # Inputs mit Layout überschreiben
        for vel_type in self.inputs:
            for direction in self.inputs[vel_type]:
                self.inputs[vel_type][direction] = self.layouts[vel_type][direction+str(layout)]
                my_item[vel_type] = direction+str(layout)

        # Lamda-Function für haptischen Distanz-Output
        self.rumble_func = lambda distance: max(min(1, (-1.25 * distance + 1.25)), 0) * self.rumble['distance']
        rospy.loginfo('rumble(x) = (%s * x + %s) * %s', -1.25, 1.25, self.rumble['distance'])
        # rumble(item) = ax + b; mit r(1m) = 100% und r(0.2m) = 0% --> a = -b = -1.25

        # Lamda-Function für visuellen Distanz-Output
        # Laserscan-Range: 0.2m - 3.5m --> delta_range = 3.3m
        self.green_func = lambda distance: max(min(1, (distance/3.3)), 0)
        rospy.loginfo('green(x) = x / %s', 3.3)
        self.red_func = lambda distance: max(min(1, (-(distance/3.3) + 1)), 0)
        rospy.loginfo('red(x) = - x / %s + %s', 3.3, 1)

        self.feedback = Feedback()
        self.feedback.set_led = True
        self.feedback.set_rumble = True
        self.feedback.set_led_flash = True

        # Attribute aus Staus extrahieren
        self.prev_status = Status()
        self.attrs = []
        for attr in Status.__slots__:
            if attr.startswith('axis_') or attr.startswith('button_'):
                self.attrs.append(attr)

        self.vel_msg = Twist
        self.pub_vel_flag = True
        self.pub_vel = rospy.Publisher('cmd_vel', self.vel_msg, queue_size=1)
        self.pub_feedback = rospy.Publisher('set_feedback',Feedback, queue_size=1)
        rospy.Subscriber('status', Status, self.cb_status, queue_size=1)
        rospy.Subscriber('scan', LaserScan, self.cb_scan, queue_size=1)

    def cb_status(self, msg):

        # /cmd_vel - Eingaben
        input_vals = {}
        for attr in self.attrs:
            input_vals[attr] = getattr(msg, attr)

        vel_to_pub = self.vel_msg()
        twist = vel_to_pub

        for vel_type in self.inputs:
            vel_vec = getattr(twist, vel_type)
            for k, expr in self.inputs[vel_type].items():
                scale = self.scales[vel_type].get(k, 1.0)
                val = eval(expr, {}, input_vals)
                setattr(vel_vec, k, scale * val)

        # Buttons
        if self.prev_status != msg:

            # toggle pub_vel mit X und O
            if msg.button_cross and not self.pub_vel_flag:
                self.pub_vel_flag = True
                rospy.loginfo("pub_vel: %s", self.pub_vel_flag)
            elif msg.button_circle and self.pub_vel_flag:
                self.pub_vel_flag = False
                rospy.loginfo("pub_vel: %s", self.pub_vel_flag)

            # Batterie-Ausgabe mit Dreieck
            if msg.button_triangle and not self.prev_status.button_triangle:
                rospy.loginfo("Battery: %s %%", (msg.battery_percentage * 100))
                rospy.loginfo("USB: %s", msg.plug_usb)

            # bash sript starten mit Viereck
            if msg.button_square and not self.prev_status.button_square:
                rospy.loginfo("execute: %s", self.map_saver_path)
                rospy.loginfo("saving map to: %s", self.map_path)
                subprocess.call([self.map_saver_path, self.map_path])

            self.prev_status = msg

        # /set_feedback (battery_percentage)
        if msg.battery_percentage <= 0.375:
            self.feedback.led_flash_on = 0.3
            self.feedback.led_flash_off = 0.3
        elif msg.battery_percentage <= 0.25:
            self.feedback.led_flash_on = 0.1
            self.feedback.led_flash_off = 0.1
        else:
            self.feedback.led_flash_off = 0

        # /set_feedback (rumble_small)
        if self.pub_vel_flag:
            vel_rumble = 0
            if vel_to_pub.linear.x != 0:
                vel_rumble = (abs(vel_to_pub.linear.x) / self.scales['linear'].get('x')) * self.rumble['velocity'].get('linear')
            elif vel_to_pub.angular.z != 0:
                vel_rumble = (abs(vel_to_pub.angular.z) / self.scales['angular'].get('z')) * self.rumble['velocity'].get('angular')

            self.feedback.rumble_small = abs(vel_rumble)

        # /cmd_vel - Publish
        if self.pub_vel_flag:
            self.pub_vel.publish(vel_to_pub)

    def cb_scan(self, scan):

        scan_filter = []
        samples = len(scan.ranges)
        samples_view = 1

        if samples_view > samples:
            samples_view = samples

        if samples_view == 1:
            scan_filter.append(scan.ranges[0])

        else:
            left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
            right_lidar_samples_ranges = samples_view//2

            left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)

        for i in range(samples_view):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 3.5
            elif isnan(scan_filter[i]):
                scan_filter[i] = 0

        self.analyse_distance(scan_filter)

    def analyse_distance(self, front_scan):

        # aktueller Scan; list --> float
        distance = float(front_scan[0])

        # Farbwert in Abhängigkeit der Distanz (s. __init__)
        self.feedback.led_r = self.red_func(distance)
        self.feedback.led_g = self.green_func(distance)

        # Rumblewert in Abhängigkeit der Distanz
        self.feedback.rumble_big = self.rumble_func(distance)

        self.pub_feedback.publish(self.feedback)


def main(layout, map_path, map_saver_path):
    rospy.init_node('status_to_turtle_twist')

    StatusToTurtleTwist(layout, map_path, map_saver_path)

    rospy.spin()


if __name__ == '__main__':
    layout = sys.argv[1]
    map_path = sys.argv[2]
    map_saver_path = sys.argv[3]
    main(layout, map_path, map_saver_path)
