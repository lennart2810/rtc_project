#!/usr/bin/env python3

# Grundlage
# --- TurtleBotClass.py ------
# Version vom 8.11.2021 by OJ
# https://github.com/ProfJust/rtc/tree/master/nodes/ue04
# ----------------------------

# Obstacle-Erweiterung
# Version vom 10.11.2021 by LF
# ----------------------------

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from math import pow, atan2, sqrt, pi, isnan


class TurtleBotClass:
    # globale Variablen
    # goal = Pose()

    def __init__(self):
        # globale Klassen-Variablen instanzieren
        self.pose = Pose()
        self.goal = Pose()
        self.vel_msg = Twist()

        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/odpm. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('odom',
                                                Odometry, self.update_pose)

        self.rate = rospy.Rate(10)

    def quaternion_to_euler(self, x, y, z, w):
        # https://computergraphics.stackexchange.com/questions/8195/how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr
        """t0 = 2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = atan2(t0, t1) # Drehung um X-Achse

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = asin(t2))  # Drehung um Y-Achse"""

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = atan2(t3, t4)  # Drehung um Z-Achse in rad

        return yaw

    def update_pose(self, data):
        # Callback function which is called when a new message
        # of type Pose is received by the subscriber.
        self.pose.x = round(data.pose.pose.position.x, 4)
        self.pose.y = round(data.pose.pose.position.y, 4)
        # rospy.loginfo(rospy.get_caller_id() + "x %s  y %s ", pose.x, pose.x)
        # orientation als Quaternion
        x = data.pose.pose.orientation.x
        y = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w
        self.pose.theta = self.quaternion_to_euler(x, y, z, w)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def set_linear_vel(self, goal_pose, constant=0.15,
                       lin_max=0.5, lin_min=0.08):
        lin_vel = constant * self.euclidean_distance(goal_pose)
        if lin_vel > lin_max:
            lin_vel = lin_max  # Maximum begrenzen
        if lin_vel < lin_min:
            lin_vel = lin_min  # Minimum begrenzen

        self.vel_msg.linear.x = lin_vel
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def fitPi(self, angle):  # fit to Intervall -Pi ...Pi
        if angle > pi:
            angle = angle - 2*pi
        if angle < -pi:
            angle = angle + 2*pi
        return angle

    def set_angular_vel(self, goal_pose, constant=1.5, ang_max=1.5):
        ang_vel = constant * (self.fitPi(self.steering_angle(goal_pose))
                              - self.fitPi(self.pose.theta))
        if(ang_vel > ang_max):
            ang_vel = ang_max
        if(ang_vel < -ang_max):
            ang_vel = -ang_max
        # Problem, speed theta wechselt von +1.5 auf -1.5 => Turtle torckelt

        self.vel_msg.angular.z = ang_vel
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0

    def getGoalFromUser(self):
        self.stop_robot()
        self.goal.x = eval(input("Set your x goal: "))
        self.goal.y = eval(input("Set your y goal: "))

    def start_info(self):
        # Debug Info
        rospy.loginfo("Start Pose is %f %f", self.pose.x, self.pose.y)
        rospy.loginfo("Goal is       %f %f", self.goal.x, self.goal.y)
        rospy.loginfo("Distannce to Goal is  %f ",
                      self.euclidean_distance(self.goal))
        rospy.loginfo("SteeringAngle to Goal is  %f ",
                      self.steering_angle(self.goal))
        input("Hit any Key to start")

    def pose_speed_info(self):
        rospy.loginfo("Goal is %s %s",
                      round(self.goal.x, 4),
                      round(self.goal.y, 4))
        rospy.loginfo("Pose is %s %s",
                      round(self.pose.x, 4),
                      round(self.pose.y, 4))
        rospy.loginfo("Speed is x: %s  theta: %s",
                      round(self.vel_msg.linear.x, 4),
                      round(self.vel_msg.angular.z, 4))

    def stop_robot(self):
        # Stopping our robot after the movement is over.
        rospy.loginfo(" ######  Stop Robot #######")
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)

    def goal_reached(self, distance_tolerance=0.05):
        if self.euclidean_distance(self.goal) < distance_tolerance:
            rospy.loginfo(" ######  Goal Reached #######")
            return True
        else:
            return False

    def move2goal(self):
        # Moves the turtle to the goal
        if not self.goal_reached():
            # Linear velocity in the x-axis.
            self.set_linear_vel(self.goal)
            # Angular velocity in the z-axis.
            self.set_angular_vel(self.goal)

            # Publishing our vel_msg
            self.velocity_publisher.publish(self.vel_msg)
            # Publish at the desired rate.
            self.rate.sleep()
            # debug Info
            self.pose_speed_info()
            return False

        self.stop_robot()  # when goal is reached
        return True
        # exit()

    def get_scan(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []

        samples = len(scan.ranges)  # The number of samples is defined in 
                                    # turtlebot3_<model>.gazebo.xacro file,
                                    # the default is 360.
        samples_view = 1            # 1 <= samples_view <= samples

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

        return scan_filter
