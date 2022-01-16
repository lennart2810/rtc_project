#!/usr/bin/env python3
# original: WHS, OJ, 3.12.2020

import sys
import rospy
from geometry_msgs.msg import PoseStamped


def clickCB(data):
    rospy.loginfo("clicked at " + str(data.pose.position.x)
                  + " " + str(data.pose.position.y))
    fobj = open(map_path, 'a')
    write_str = "[" + str(data.pose.position.x) + ","\
                    + str(data.pose.position.y) + ","\
                    + str(data.pose.orientation.x) + ","\
                    + str(data.pose.orientation.y) + ","\
                    + str(data.pose.orientation.z) + ","\
                    + str(data.pose.orientation.w) \
                    + "] \n"
    fobj.write(write_str)
    fobj.close()


if __name__ == '__main__':

    # filename = sys.argv[0]
    map_file = sys.argv[1]
    map_path = map_file.replace('.yaml', '_path.txt')

    try:
        rospy.init_node('goal_listener', anonymous=True)
        rospy.loginfo("Auf der RVIZ- Karte ein 2D Nav Goal ankllicken,\
                      /move_base_simple/set_goal")
        click_sub = rospy.Subscriber('/move_base_simple/set_goal',
                                     PoseStamped,
                                     clickCB)
        rate = rospy.Rate(2)

        while not rospy.is_shutdown():
            pass

    except rospy.ROSInterruptException:
        print("program close.")
