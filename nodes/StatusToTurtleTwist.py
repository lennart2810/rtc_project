#!/usr/bin/env python3

# -- StatusToTurtleTwist.py --
# Version vom 13.11.2021 by LF
# ----------------------------

import sys
import rospy
import subprocess
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from ds4_driver.msg import Status, Feedback
from sensor_msgs.msg import LaserScan
from math import isnan


class StatusToTurtleTwist(object):
    def __init__(self, controller_layout, map_path, map_saver_path):

        # Übergebene Argumente
        self.map_path = map_path  # Pfad zum speichern einer SLAM - Karte
        self.map_saver_path = map_saver_path  # map_saver.sh

        # ROS Parameter einlesen
        self.inputs = rospy.get_param('~inputs')
        self.layouts = rospy.get_param('~layouts')
        self.scales = rospy.get_param('~scales')
        self.rumble = rospy.get_param('~rumble')
        self.distanaces = rospy.get_param('~distance')

        # Controller-Layout einstellen
        self.__init__controller_layout(controller_layout)

        # Lambda-Funktion für Rumblestärke bei Objekterkennung
        self.__init__lambda_func()

        # Attribute aus Staus extrahieren
        self.prev_status = Status()
        self.attrs = []
        for attr in Status.__slots__:
            if attr.startswith('axis_') or attr.startswith('button_'):
                self.attrs.append(attr)

        self.vel_msg = Twist
        self.pub_vel_flag = True
        # self.object_front = False

        self.feedback = Feedback()
        self.feedback.set_led = True
        self.feedback.set_rumble = True
        self.feedback.set_led_flash = True

        self.distance = 0  # im Moment nur Float 

        # Publisher und Subscriber
        self.pub_vel = rospy.Publisher('cmd_vel', self.vel_msg, queue_size=1)
        self.pub_feedback = rospy.Publisher('set_feedback',Feedback, queue_size=1)
        rospy.Subscriber('status', Status, self.cb_status, queue_size=1)
        rospy.Subscriber('scan', LaserScan, self.cb_scan, queue_size=1)

        # Debug Topic 
        self.pub_debug = rospy.Publisher('cmd_vel.linear.x', Float64, queue_size=1)

    def __init__controller_layout(self, layout):

        rospy.loginfo('init controller_layout: %s', layout)
        my_item = {}
        for vel_type in self.inputs:
            for direction in self.inputs[vel_type]:
                my_item[vel_type] = direction + str(layout)

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
                self.inputs[vel_type][direction] = self.layouts[vel_type][direction + str(layout)]
                my_item[vel_type] = direction + str(layout)

    def __init__lambda_func(self):

        # Lineare Funktion über 2 Punkte bestimmen:
        # f(x) = (y_2-y_1)/(x_2-x_1) * (x-x_1) + y_1

        min_dist = self.distanaces.get('min')           # min Laserscan
        crit_dist = self.distanaces.get('critical')     # kritischer Abstand

        self.rumble_func = lambda x: max(min(1, round(-1/(crit_dist-min_dist)*(x-min_dist)+1, 2)), 0)

    def reduce_vel(self):

        x = self.distance
        crit_dist = self.distanaces.get('critical')  # kritischer Abstand

        # Geschwindigkeit nach Vorne begrenzen / blockieren
        # ab 1/3 der kritischen Distanz --> vel = 50%; darunter 0%
        # reduce_vel(x) = (1-0.5)/((cd/3)-cd) * (x-(cd/3)) + 0.5

        if x <= crit_dist/3:
            return 0
        elif x > crit_dist/3 and x <= crit_dist:
            p = (0.5/(crit_dist/3)) * (x-(crit_dist/3)) + 0.5
            p = round(p, 2)
            p = max(min(1, p), 0.5)
            return p
        else:
            return 1

    def cb_status(self, msg):

        # /cmd_vel - Eingaben
        input_vals = {}
        for attr in self.attrs:
            input_vals[attr] = getattr(msg, attr)

        #vel_to_pub = self.vel_msg()
        vel_to_pub = Twist()
        twist = vel_to_pub

        for vel_type in self.inputs:
            vel_vec = getattr(twist, vel_type)
            for k, expr in self.inputs[vel_type].items():
                scale = self.scales[vel_type].get(k, 1.0)
                val = eval(expr, {}, input_vals)
                setattr(vel_vec, k, scale * val)

        self.vel_msg = vel_to_pub

        if self.vel_msg.linear.x > 0:   # Turtle fährt nach vorne
            self.vel_msg.linear.x *= self.reduce_vel()
            # self.pub_debug.publish(self.vel_msg.linear.x) 



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
            if self.vel_msg.linear.x != 0:
                vel_rumble = (abs(self.vel_msg.linear.x) / self.scales['linear'].get('x')) * self.rumble['velocity'].get('linear')
                #vel_rumble = abs(msg.list(self.inputs['linear'].values())[0]) * self.rumble['velocity'].get('linear')
            elif self.vel_msg.angular.z != 0:
                vel_rumble = (abs(self.vel_msg.angular.z) / self.scales['angular'].get('z')) * self.rumble['velocity'].get('angular')

            self.feedback.rumble_small = abs(vel_rumble)

        # /cmd_vel - Publish
        if self.pub_vel_flag:
            # self.pub_vel.publish(vel_to_pub)
            self.pub_vel.publish(self.vel_msg)

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

        self.distance = scan_filter[0]
        self.analyse_distance(scan_filter)

    def analyse_distance(self, front_scan):

        # front scan durch front_m und back_m ersetzen (in cb scan)!!

        # aktueller Scan; list --> float
        distance = float(front_scan[0])

        # Farbwert in Abhängigkeit der Distanz
        self.feedback.led_r = self.colour_distance(distance)[0]
        self.feedback.led_g = self.colour_distance(distance)[1]

        # Distanz zu Objekten bewerten
        self.rumble_distance(distance)

        # Controller Feedback publischen
        self.pub_feedback.publish(self.feedback)

    def rumble_distance(self, x):

        # abstand hinten ebenfalls prüfen!!

        # Abstand zu Objekten vor der Turtle auf kritischen Bereich prüfen
        if x <= self.rumble['distance'].get('max'):

            # Vibration als Warnung in Abhängigkeit der Entfernung
            self.feedback.rumble_big = self.rumble_func(x)

            # weitere Bewegung nach Vorne verhindern
            #self.object_front = True

        else:
            #self.object_front = False
            self.feedback.rumble_big = 0.0  # --> keine Vibration

    def colour_distance(self, x):

        # Laserscan-Range: 0.12m - 3.5m
        # 0.12m --> 100 % rot
        # 3.50m --> 100 % grün

        # Lineare Funktion über 2 Punkte bestimmen:
        # f(x) = (y_2-y_1)/(x_2-x_1) * (x-x_1) + y_1

        # red(x) = (0-1)/(3.5-0.12) * (x-0.12) + 1
        red = -1/(3.38) * (x-0.12) + 1
        red = round(red, 2)
        red = max(min(1, red), 0)

        # green(x) = (1-0)/(3.5-0.12) * (x-0.12) + 0
        green = max(min(1, round(1/(3.38) * (x-0.12), 2)), 0)

        return [red, green]


def main(layout, map_path, map_saver_path):
    rospy.init_node('status_to_turtle_twist')

    StatusToTurtleTwist(layout, map_path, map_saver_path)

    rospy.spin()


if __name__ == '__main__':
    layout = sys.argv[1]
    map_path = sys.argv[2]
    map_saver_path = sys.argv[3]
    main(layout, map_path, map_saver_path)
