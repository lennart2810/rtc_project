#!/usr/bin/env python3

# based on the code from
# https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/
# edited WHS, OJ , 12.12.2020

import rospy
import sys
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Initial Koordinaten für Ort x,y und Orientierung x,y,z,w
path = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]


def read_path_from_file(filename):
    rospy.loginfo("Reading Path from path.txt : ")
    # Den vorgegebenen Pfad einlesen, jede Zeile ein Goal
    with open(filename, 'r') as fin:
        for line in fin:
            path.append(eval(line))  # Goal anhaengen
    del path[0]  # [0, 0] entfernen
    rospy.loginfo(str(path))


def movebase_client():
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    client.wait_for_server()
    read_path_from_file(map_path)

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    for koord in path:
        # set posistion (no z)
        goal.target_pose.pose.position.x = koord[0]
        goal.target_pose.pose.position.y = koord[1]
        # set orientation - quaternion
        goal.target_pose.pose.orientation.x = koord[2]
        goal.target_pose.pose.orientation.y = koord[3]
        goal.target_pose.pose.orientation.z = koord[4]
        goal.target_pose.pose.orientation.w = koord[5]

        # Sends the goal to the action server.
        client.send_goal(goal)

        # wait for the action to return, with timeout
        finished_before_timeout = client.wait_for_result(
                                           rospy.Duration(120.0))
        if finished_before_timeout is True:
            rospy.loginfo(" Reached Goal before Timeout ")
        else:
            rospy.loginfo(" Timeout ")

    return client.get_result()


if __name__ == '__main__':

    map_path = sys.argv[1]

    try:
        # let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Path execution done!")

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
