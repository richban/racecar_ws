#!/usr/bin/env python3

import rospy
import sys
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose
from geometry_msgs.msg import PoseStamped, PointStamped
from math import atan2, sqrt, cos, sin, pi
import numpy as np
from argparse import ArgumentParser
import pdb

PUBLISH_RATE = 20.0  # number of controll commands to publish in one second


class BugController:
    def __init__(self, a, b):
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.lidar_sub = rospy.Subscriber(
            '/scan', LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(
            '/drive', AckermannDriveStamped, queue_size=1)
        self.marker_pub = rospy.Publisher(
            '/visualization_marker', Marker, queue_size=1)
        self.pose_pub = rospy.Publisher('/pose', Pose, queue_size=1)
        # listen for 2D Pose or 2D Nav
        # self.publish_point_sub = rospy.Subscriber('/clicked_point', PointStamped, self.point_callback, queue_size=1)
        # self.nav_goal_sub = rospy.Subscriber('/move_base_simple/goal', PointStamped, self.point_callback, queue_size=1)
        # Controll specific
        self.goal_point = Point()
        self.goal_point.x = a
        self.goal_point.y = b
        self.x = 0
        self.y = 0
        self.theta = 0
        self.angle_min = 0
        self.angle_max = 0
        self.angle_inc = 1
        self.ranges = []

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion(
            [rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        # print(self.x, self.y)

    def lidar_callback(self, msg):
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_inc = msg.angle_increment
        self.ranges = msg.ranges

    def point_callback(self, msg):
        if isinstance(msg, PointStamped):
            self.goal_point = (msg.point.x, msg.point.y)
        else:
            self.goal_point = (msg.pose.position.x,
                               msg.pose.position.y)
        print(self.goal_point)

    def plot_point(self, point):
        vis_marker = Marker()
        vis_marker.header.stamp = rospy.Time.now()
        vis_marker.header.frame_id = 'map'
        vis_marker.id = 0
        vis_marker.type = 2
        vis_marker.action = 0
        vis_marker.pose.position.x = point.x
        vis_marker.pose.position.y = point.y
        vis_marker.pose.position.z = 0.0
        vis_marker.pose.orientation.x = 0.0
        vis_marker.pose.orientation.y = 0.0
        vis_marker.pose.orientation.z = 0.0
        vis_marker.pose.orientation.w = 1.0
        vis_marker.scale.x = 0.1
        vis_marker.scale.y = 0.1
        vis_marker.scale.z = 0.1
        vis_marker.color.a = 1.0
        vis_marker.color.r = 0.0
        vis_marker.color.g = 1.0
        vis_marker.color.b = 0.0

        self.marker_pub.publish(vis_marker)

    def bug_finder(self):
        # Spawning car
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = rospy.Time.now()
        ack_msg.drive.steering_angle = 0
        ack_msg.drive.speed = 0

        p = Pose()
        p.position.x = 0
        p.position.y = -3
        p.position.z = 0
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quaternion_from_euler(
            0, 0, pi/2)

        # Defining Goal
        goal = Point()
        goal.x = self.goal_point.x
        goal.y = self.goal_point.y
        print(goal)

        # Tmp Goal
        tmpgoal = Point()
        tmpgoal.x = self.goal_point.x
        tmpgoal.y = self.goal_point.y

        self.plot_point(goal)
        self.plot_point(tmpgoal)

        self.drive_pub.publish(ack_msg)
        self.pose_pub.publish(p)

        # Sudden point threshold
        sud_d = 1
        a_steps = int((self.angle_max - pi/2)/self.angle_inc)

        ack_msg = AckermannDriveStamped()
        # pdb.set_trace()
        # rospy.sleep(1.0)
        while not rospy.is_shutdown():
            inc_x = tmpgoal.x - self.x
            inc_y = tmpgoal.y - self.y

            dist_to_goal = sqrt(inc_x**2 + inc_y**2)
            angle_to_goal = atan2(inc_y, inc_x)
            rospy.loginfo(
                "Distance to the goal: [%.2f]\nAngle to the goal: [%.2f]", dist_to_goal, angle_to_goal)

            ack_msg.header.stamp = rospy.Time.now()

            adiff, sign = diff(angle_to_goal, self.theta)

            # Checking whether goal can be seen:
            if adiff < pi/2:
                if adiff > 0.1:
                    ack_msg.drive.steering_angle = sign*pi/4
                    ack_msg.drive.speed = 0.2
                elif dist_to_goal < 0.25:
                    ack_msg.drive.steering_angle = 0
                    ack_msg.drive.speed = 0
                else:
                    ack_msg.drive.steering_angle = 0
                    ack_msg.drive.speed = 0.2
            else:
                # Turn to goal
                ack_msg.drive.steering_angle = -sign*pi/4
                ack_msg.drive.speed = -0.2

            self.drive_pub.publish(ack_msg)
            rospy.sleep(1.0/PUBLISH_RATE)

    def sudden_point_finder(self, rngs, steps, threshold):
        # only want 180 vision
        rngs = rngs[steps:-steps]
        sud_points = []

        for i, p in enumerate(rngs):
            if i != 0:
                if (prev_p - p) > threshold:
                    sud_points.append(
                        [self.angle_min + self.angle_inc*(i+steps), min(prev_p, p), 1])
                elif (prev_p - p) < -threshold:
                    sud_points.append(
                        [self.angle_min + self.angle_inc*(i+steps), min(prev_p, p), -1])
                prev_p = p
            else:
                prev_p = p

        return sud_points

    def min_distance_finder(self, sud_points, goal):
        dists = []
        positions = []

        for p in sud_points:
            xpoint_car = p[1]*cos(p[0])
            ypoint_car = p[1]*sin(p[0])

            xpoint = self.x + xpoint_car * \
                cos(self.theta) - ypoint_car*sin(self.theta)
            ypoint = self.y + xpoint_car * \
                sin(self.theta) + ypoint_car*cos(self.theta)

            car2point = sqrt((self.x-xpoint)**2 + (self.y-ypoint)**2)
            point2goal = sqrt((xpoint - goal.x)**2 + (ypoint - goal.y)**2)

            dists.append(car2point + point2goal)

            positions.append(xpoint)

        if len(dists) > 0:
            return np.argmin(dists)
        else:
            return None


def diff(a1, a2):
    phi = abs(a1 - a2) % (2*pi)
    test = ((a1 - a2 >= 0) & (a1 - a2 <= pi)
            ) or ((a1 - a2 <= -pi) & (a1 - a2 >= -2*pi))

    if test:
        sign = 1
    else:
        sign = -1

    if phi > pi:
        return 2*pi-phi, sign
    else:
        return phi, sign


def usage():
    return '{} [a b]'.format(sys.argv[0])


if __name__ == "__main__":
    rospy.init_node('ebug_controller')
    parser = ArgumentParser()

    parser.add_argument('--x', type=float, help='X coordinate')
    parser.add_argument('--y', type=float, help='Y coordinate')
    args = parser.parse_args()

    robot = BugController(args.x, args.y)
    robot.bug_finder()

    rospy.spin()
