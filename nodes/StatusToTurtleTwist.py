#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from ds4_driver.msg import Status
from sensor_msgs.msg import LaserScan
from math import isnan

#test

class StatusToTurtleTwist(object):
    def __init__(self):

        self._cls = Twist
        self._inputs = rospy.get_param('~inputs')
        self._scales = rospy.get_param('~scales')

        self._attrs = []
        for attr in Status.__slots__:
            if attr.startswith('axis_') or attr.startswith('button_'):
                self._attrs.append(attr)

        self._pub = rospy.Publisher('cmd_vel', self._cls, queue_size=1)
        rospy.Subscriber('status', Status, self.cb_status, queue_size=1)
        rospy.Subscriber('scan', LaserScan, self.cb_scan, queue_size=1)

    def cb_status(self, msg):

        input_vals = {}
        for attr in self._attrs:
            input_vals[attr] = getattr(msg, attr)

        to_pub = self._cls()
        twist = to_pub

        for vel_type in self._inputs:
            vel_vec = getattr(twist, vel_type)
            for k, expr in self._inputs[vel_type].items():
                scale = self._scales[vel_type].get(k, 1.0)
                val = eval(expr, {}, input_vals)
                setattr(vel_vec, k, scale * val)

        self._pub.publish(to_pub)

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

        rospy.loginfo(scan_filter)


def main():
    rospy.init_node('status_to_turtle_twist')

    rospy.loginfo('StatusToTurtleTwist started...')

    StatusToTurtleTwist()

    rospy.spin()


if __name__ == '__main__':
    main()
