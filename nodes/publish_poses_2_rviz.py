#!/usr/bin/env python3

import sys
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray


def read_path_from_file(filename):
    rospy.loginfo("Reading Path from path.txt : ")

    global path
    path = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]

    with open(filename, 'r') as fin:
        for line in fin:
            path.append(eval(line))
    del path[0]


def publish_poses():

    read_path_from_file(map_path)

    pose_array = PoseArray()
    pose_array.header.frame_id = 'map'

    for koord in path:

        pose_now = Pose()
        pose_now.position.x = koord[0]
        pose_now.position.y = koord[1]
        pose_now.orientation.x = koord[2]
        pose_now.orientation.y = koord[3]
        pose_now.orientation.z = koord[4]
        pose_now.orientation.w = koord[5]

        pose_array.poses.append(pose_now)

    pose_pub.publish(pose_array)


if __name__ == '__main__':

    map_path = sys.argv[1]
    map_path = map_path.replace('.yaml', '_path.txt')

    rospy.init_node('pose_publisher', anonymous=True)

    pose_pub = rospy.Publisher('poses', PoseArray, queue_size=1)

    while not rospy.is_shutdown():
        publish_poses()
        rospy.sleep(1)
