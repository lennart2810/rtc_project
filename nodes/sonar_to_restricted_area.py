#!/usr/bin/env python3
# sonar_to_costmap.py
# ################################################################################
# edited WHS, OJ , 23.12.2020 #
#
# brings Sonar detected Obstacles into move_base local costmap
# using point_cloud - message
#
# edit
# only Gazebo
# copy content of turtlebot3.burger.gazebo_sonar.xacro
#              to turtlebot3.burger.gazebo_sonar.xacro
# copy content of turtlebot3.burger.urdf_sonar.xacro
#              to turtlebot3.burger.urdf.xacro
#
# real Bot and Gazebo
# edit costmap_common_params_burger.yaml
#    observation_sources: scan sonar
#    scan: ...
#    sonar: {sensor_frame: base_link, data_type: PointCloud,
#             topic: /sonar/point_cloud, marking: true, clearing: true}
#
# edit move_base.launch  => /cmd_vel to /move_base/cmd_vel
#     <arg name="cmd_vel_topic" default="/move_base/cmd_vel" />
#
# usage
#   $1 roslaunch turtlebot3_gazebo turtlebot3_house.launch
#   $2 roslaunch turtlebot3_navigation turtlebot3_navigation.launch
#                map_file:=$HOME/catkin_ws/src/rtc/rtc_maps/gazebo_house_map_2020_12_07.yaml
#   $3 roslaunch rts sonar_twist_mux.launch
#   $4 rosrun rtc sonar_obstacle_avoidance.py
#   $5 rosrun rtc sonar_to_costmap.py
# ------------------------------------------------------------------

import rospy
import std_msgs.msg
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from sensor_msgs.msg import PointCloud  # Message für die Sonar-Hindernisse
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from tf2_msgs.msg import TFMessage
from turtlebot3_msgs.msg import SensorState
from math import atan2, sin, cos, degrees



class Sonar_to_RestrictedArea():
    def __init__(self):
        rospy.loginfo("Publishing PointCloud")

        self.cloud_pub = rospy.Publisher('sonar/point_cloud',
                                         PointCloud,
                                         queue_size=10)

        self.restricted_area_pub = rospy.Publisher('mouse_location',Point,queue_size=10)

        # /cmd_vel
        #self.vel_msg = Twist
        #self.pub_vel_flag = True
        #self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
     
        # tf ist in gazebo deutlich genauer !! am realen turtlebot hoffentlich auch!!
        #self.amcl_pose_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.cb_get_amcl_pose)
        #self.tf_pose_sub = rospy.Subscriber('tf', TFMessage, self.cb_get_tf_pose)
        self.pose = Pose()



        # receiving sonar_left and sonar_right
        self.sonar_sub_left = rospy.Subscriber('sonar_left',
                                               Range,
                                               self.get_sonar_left,
                                               queue_size=10)

        self.sonar_sub_right = rospy.Subscriber('sonar_right',
                                                Range,
                                                self.get_sonar_right,
                                                queue_size=10)

        self.cloud_pub = rospy.Publisher('sonar/point_cloud',
                                         PointCloud,
                                         queue_size=10)

        self.sonar_sub = rospy.Subscriber('sensor_state',
                                           SensorState,
                                           self.get_both_sonar,
                                           queue_size=10)

        self.dist_left = 0.0
        self.dist_right = 0.0
        self.rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.rate.sleep()


    # müssen für den realen Turtlebot angepasst werden!
    # siehe sonar_to_costmap.py (get_both_sonar)



    def get_sonar_left(self, sensor_data_left):
        # rospy.loginfo(" Sonar Data Left received ")
        self.dist_left = sensor_data_left.range
        self.cloud_build()

    def get_sonar_right(self, sensor_data_right):
        # rospy.loginfo(" Sonar Data Right received ")
        self.dist_right = sensor_data_right.range
        self.cloud_build()

    def get_both_sonar(self, sensor_data):
        print = False
        # Left
        dist_left = Range()
        dist_left = sensor_data.sonar / 100
        self.dist_left = dist_left
        if print: rospy.loginfo("Left: " + str(self.dist_left))
        # Right
        dist_right = Range()
        dist_right = sensor_data.cliff / 100
        self.dist_right = dist_right
        if print: rospy.loginfo("Right: " + str(self.dist_right))
        # Cloud Build
        self.cloud_build()

    def cloud_build(self):
        # add sonar readings (robot-local coordinate frame) to cloud
        pl = Point32()  # Punkt von Sonar Left
        pm = Point32()  # Mittelpunkt
        pr = Point32()  # Sonar Right
        # Instanziiere leere PointCloud
        cloud = PointCloud()
        # filling pointcloud header
        header = std_msgs.msg.Header()  # Leere Instanz
        header.stamp = rospy.Time.now()  # Fülle Zeitstempel
        header.frame_id = 'base_link'
        cloud.header = header

        # Linke Seite
        if(self.dist_left < 0.75 and self.dist_left > 0.05):
            pl.x = self.dist_left + 0.04
            # difference frames: base_link to base_sonar_front_left
            pl.y = 0.06
            pl.z = 0.0
            cloud.points.append(pl)

            pm.x = (self.dist_left + self.dist_right)/2 + 0.05
            pm.y = 0.0
            pm.z = 0.0
            cloud.points.append(pm)

        # Rechte Seite  punkt einfügen  (x,y,z)
        if(self.dist_right < 0.75 and self.dist_right > 0.05):
            # difference frames: base_link to base_sonar_front_right
            pr.x = self.dist_right + 0.04
            pr.y = -0.06
            pr.z = 0.0
            cloud.points.append(pr)

        # Senden
        self.cloud_pub.publish(cloud)
        self.pub_mouse_location()

    def pub_mouse_location(self):

        # amcl_pose x und y + x_ und y_ (_ --> inkremental von turtle aus)
        # x_ und y_ aus yaw Winkel und gemittelte Sonar-Distanz berechnen
        # yaw Winkel aus amcl_pose quaternion berechnen

        restricted_area_point = Point()
        restricted_area_point.z = 0.0

        max_dist = 0.75
        min_dist = 0.3


        if(self.dist_left < max_dist and self.dist_left > min_dist
           and self.dist_right < max_dist and self.dist_left < max_dist):

            sonar_dist = (self.dist_left + self.dist_right) / 2 # + 0.1

            yaw = self.quaternion_to_euler()

            x_ = cos(yaw) * sonar_dist
            y_ = sin(yaw) * sonar_dist

            restricted_area_point.x = self.pose.position.x + x_
            restricted_area_point.y = self.pose.position.y + y_
            self.restricted_area_pub.publish(restricted_area_point)

    def cb_get_amcl_pose(self, msg):
        # /amcl_pose
        self.pose.position.x = msg.pose.pose.position.x
        self.pose.position.y = msg.pose.pose.position.y

        self.pose.orientation.x = msg.pose.pose.orientation.x
        self.pose.orientation.y = msg.pose.pose.orientation.y
        self.pose.orientation.z = msg.pose.pose.orientation.z
        self.pose.orientation.w = msg.pose.pose.orientation.w

    def cb_get_tf_pose(self, msg):

        msg_now = msg.transforms[0]

        if msg_now.child_frame_id == 'base_footprint':
            # /tf
            self.pose.position.x = msg_now.transform.translation.x
            self.pose.position.y = msg_now.transform.translation.y

            self.pose.orientation.x = msg_now.transform.rotation.x
            self.pose.orientation.y = msg_now.transform.rotation.y
            self.pose.orientation.z = msg_now.transform.rotation.z
            self.pose.orientation.w = msg_now.transform.rotation.w

    def quaternion_to_euler(self):
        # https://github.com/ProfJust/rtc/blob/master/nodes/ue05_action_server/TurtleBotClassFile.py

        x = self.pose.orientation.x
        y = self.pose.orientation.y
        z = self.pose.orientation.z
        w = self.pose.orientation.w

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = atan2(t3, t4)  # Drehung um Z-Achse in rad

        return yaw


if __name__ == '__main__':
    rospy.init_node('sonar_controller', anonymous=True)
    try:
        sonar = Sonar_to_RestrictedArea()
    except rospy.ROSInterruptException:
        pass
