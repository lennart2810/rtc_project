#!/usr/bin/env python3

# -- StatusToTurtleTwist.py --
# Version vom 16.11.2021 by LF
# ----------------------------


import sys
import rospy
import subprocess
# from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from ds4_driver.msg import Status, Feedback
from sensor_msgs.msg import LaserScan
from math import isnan  # Not a Number --> return True


class StatusToTurtleTwist(object):
    def __init__(self, filename, controller_layout, map_path):

        # 'relative' Pfade zu shell scipten 
        self.kill_slam_path = filename.replace('nodes/StatusToTurtleTwist.py', 'shell/turtlebot3_slam_kill.sh')
        self.map_saver_path = filename.replace('nodes/StatusToTurtleTwist.py', 'shell/map_saver.sh')
        self.navigation_path = filename.replace('nodes/StatusToTurtleTwist.py', 'shell/turtlebot3_navigation.sh')

        # Übergebene Argumente
        self.map_path = map_path  # Speicherpfad: SLAM-Karte
        self.map_saved = False

        # ROS Parameter einlesen
        self.inputs = rospy.get_param('~inputs')
        self.layouts = rospy.get_param('~layouts')
        self.scales = rospy.get_param('~scales')
        self.rumble = rospy.get_param('~rumble')
        self.distanaces = rospy.get_param('~distance')

        # Controller-Layout einstellen
        self.__init__controller_layout(controller_layout)

        # Lambda-Funktion für visuelle/haptische Objekterkennung
        self.__init__lambda_func()

        # /status
        self.attrs = []
        self.prev_status = Status()
        for attr in Status.__slots__:
            # benötigten Controller-Funktionen aus Status extrahieren
            if attr.startswith('axis_') or attr.startswith('button_'):
                self.attrs.append(attr)
        rospy.Subscriber('status', Status, self.cb_status, queue_size=1)

        # /cmd_vel
        self.vel_msg = Twist
        self.pub_vel_flag = True
        self.pub_vel = rospy.Publisher('cmd_vel', self.vel_msg, queue_size=1)

        # /set_feedback
        self.feedback = Feedback()
        self.feedback.set_led = True
        self.feedback.set_rumble = True
        self.feedback.set_led_flash = True
        self.pub_feedback = \
            rospy.Publisher('set_feedback', Feedback, queue_size=1)

        # /scan
        self.distance = 0  # im Moment nur Float --> Array
        rospy.Subscriber('scan', LaserScan, self.cb_scan, queue_size=1)
        # Modularisieren ?! --> eigene Klasse für schreiben bzw. eigenen Knoten

        # /debug
        # self.pub_debug = rospy.Publisher('debug', Float64, queue_size=1)

    def __init__controller_layout(self, layout):

        # gewähltes Layout ermitteln
        my_item = {}
        for vel_type in self.inputs:  # linear, angular
            for direction in self.inputs[vel_type]:  # linear: x; angular: z
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
            # x, z = x_layout, z_layout
            for direction in self.inputs[vel_type]:
                self.inputs[vel_type][direction] = \
                    self.layouts[vel_type][direction + str(layout)]

        rospy.loginfo('Controller Layout:')
        rospy.loginfo('%s: %s', layout, self.inputs)

    def __init__lambda_func(self):

        # Lineare Funktion über 2 Punkte bestimmen:
        # f(x) = (y_2-y_1)/(x_2-x_1) * (x-x_1) + y_1
        # round(f(x), 2) --> Funktionswert runden
        # max(min(1, f(x)), 0) --> 0 <= f(x) <= 1

        min_dist = self.distanaces.get('min')           # min Laserscan
        max_dist = self.distanaces.get('max')           # max Laserscan
        crit_dist = self.distanaces.get('critical')     # kritischer Abstand

        # rumble(x) = (0-1)/(crit-min) * (x-min) + 1
        self.rumble_func = lambda x: \
            max(min(1, round(-1/(crit_dist-min_dist)*(x-min_dist)+1, 2)), 0)

        # red(x) = (0-1)/(max-min) * (x-min) + 1
        self.red_func = lambda x: \
            max(min(1, round(-1/(max_dist-min_dist)*(x-min_dist)+1, 2)), 0)

        # green(x) = (1-0)/(max-min) * (x-min) + 0
        self.green_func = lambda x: \
            max(min(1, round(1/(max_dist-min_dist)*(x-min_dist)+0, 2)), 0)

    def cb_status(self, msg):

        # Buttons
        self.set_cmd_btn(msg)

        # Axis
        self.set_cmd_vel(msg)

        # /set_feedback <-- Batterieanzeige (Controller)
        if msg.battery_percentage <= 0.375:
            self.feedback.led_flash_on = 0.3
            self.feedback.led_flash_off = 0.3
        elif msg.battery_percentage <= 0.25:
            self.feedback.led_flash_on = 0.1
            self.feedback.led_flash_off = 0.1
        else:
            self.feedback.led_flash_off = 0

    def set_cmd_btn(self, msg):

        # toggle pub_vel mit X und O
        if msg.button_cross and not self.pub_vel_flag:
            self.pub_vel_flag = True
            rospy.loginfo("pub_vel: %s", self.pub_vel_flag)
        elif msg.button_circle and self.pub_vel_flag:
            self.pub_vel_flag = False
            rospy.loginfo("pub_vel: %s", self.pub_vel_flag)

        # Batterie-Ausgabe mit Trackpad-Btn
        if msg.button_trackpad and not self.prev_status.button_trackpad:
            rospy.loginfo("Battery: %s %%", (msg.battery_percentage * 100))
            rospy.loginfo("USB: %s", msg.plug_usb)

        # shell script (map_saver.sh) mit Viereck starten
        if msg.button_square and not self.prev_status.button_square:
            rospy.loginfo("")
            rospy.loginfo("$ rosrun map_server map_saver -f %s", self.map_path)
            subprocess.call([self.map_saver_path, self.map_path])
            self.map_saved = True

        # turtlebot3_slam.sh mit Dreieck beenden
        # und turtlbebot3_navigation.sh starten
        # vorher prüfen, ob node überhaupt aktiv ist !!!
        if msg.button_triangle and not self.prev_status.button_triangle and self.map_saved:
            rospy.loginfo("$ rosnode kill /turtlebot3_slam_gmapping")
            subprocess.call([self.kill_slam_path])
            rospy.loginfo("$ rosrun turtlebot3_navigation turtlebot3_navigation.launch map_file:=%s", self.map_path)
            subprocess.call([self.navigation_path, self.map_path])

        elif msg.button_triangle and not self.prev_status.button_triangle and not self.map_saved:
            rospy.loginfo("$ map not saved !!!")

        self.prev_status = msg

    def set_cmd_vel(self, msg):

        input_vals = {}
        for attr in self.attrs:
            input_vals[attr] = getattr(msg, attr)

        vel_to_pub = Twist()
        twist = vel_to_pub

        for vel_type in self.inputs:
            vel_vec = getattr(twist, vel_type)
            for k, expr in self.inputs[vel_type].items():
                scale = self.scales[vel_type].get(k, 1.0)
                val = eval(expr, {}, input_vals)
                setattr(vel_vec, k, scale * val)

        self.vel_msg = vel_to_pub

        # Kollisionen verhindern, wenn Turtle nach vorne fährt
        if self.vel_msg.linear.x > 0:
            self.vel_msg.linear.x *= self.reduce_vel()

        if self.pub_vel_flag:

            vel_rumble = 0
            if self.vel_msg.linear.x != 0:
                vel_rumble = self.vel_msg.linear.x \
                            / self.scales['linear'].get('x') \
                            * self.rumble['velocity'].get('linear')
            elif self.vel_msg.angular.z != 0:
                vel_rumble = self.vel_msg.angular.z \
                            / self.scales['angular'].get('z') \
                            * self.rumble['velocity'].get('angular')

            # /set_feedback <-- Vibration beim Fahren
            self.feedback.rumble_small = abs(vel_rumble)

            # /cmd_vel publishen
            self.pub_vel.publish(self.vel_msg)

    def reduce_vel(self):

        x = self.distance  # front scan
        crit_dist = self.distanaces.get('critical')  # kritischer Abstand
        n = self.distanaces.get('n')

        # Geschwindigkeit nach Vorne begrenzen / blockieren
        # 1/n der kritischen Distanz --> linear.x = 50%; darunter 0%
        # reduce_vel(x) = (1-0.5)/((crit/n)-crit) * (x-(crit/n)) + 0.5

        if x <= crit_dist/n:
            return 0
        elif x > crit_dist/3 and x <= crit_dist:
            p = (0.5/(crit_dist/n)) * (x-(crit_dist/n)) + 0.5
            p = round(p, 2)
            p = max(min(1, p), 0.5)
            return p
        else:
            return 1

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

        self.distance = scan_filter[0]  # mehrere Werte ergänzen
        self.analyse_scan()

    def analyse_scan(self):

        x = self.distance

        # visuelle Warnung <-- red(x) und green(x)
        self.feedback.led_r = self.red_func(x)
        self.feedback.led_g = self.green_func(x)

        # haptische Warnung <-- rumble(x)
        self.feedback.rumble_big = self.rumble_func(x)

        # /set_feedback publishen
        self.pub_feedback.publish(self.feedback)


def main(filename, layout, map_path):
    rospy.init_node('status_to_turtle_twist')

    StatusToTurtleTwist(filename, layout, map_path)

    rospy.spin()


if __name__ == '__main__':
    filename = sys.argv[0]
    layout = sys.argv[1]
    map_path = sys.argv[2]

    try:
        main(filename, layout, map_path)
    except rospy.ROSInterruptException:
        rospy.loginfo(" Error ")

