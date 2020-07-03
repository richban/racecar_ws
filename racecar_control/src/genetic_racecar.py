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
import pickle


class np_model(object):

    def __init__(self, agent):

        self.deep_1_w = agent.deep_1.weight.detach().numpy()
        self.deep_1_b = agent.deep_1.bias.detach().numpy().reshape(-1,1)
        self.deep_2_w = agent.deep_2.weight.detach().numpy()
        self.deep_2_b = agent.deep_2.bias.detach().numpy().reshape(-1,1)
        self.deep_3_w = agent.deep_3.weight.detach().numpy()
        self.deep_3_b = agent.deep_3.bias.detach().numpy().reshape(-1,1)
        self.deep_4_w = agent.deep_4.weight.detach().numpy()
        self.deep_4_b = agent.deep_4.bias.detach().numpy().reshape(-1,1)
        self.out_w = agent.out.weight.detach().numpy()
        self.out_b = agent.out.bias.detach().numpy().reshape(-1,1)

    def relu(self, x):
        x[x<0] = 0
        return x

    def forward(self, input):
        x = np.dot(self.deep_1_w, input.T)
        x = self.relu(x)
        x = np.dot(self.deep_2_w, x) + self.deep_2_b
        x = self.relu(x)
        x = np.dot(self.deep_3_w, x) + self.deep_3_b
        x = self.relu(x)
        x = np.dot(self.deep_4_w, x) + self.deep_4_b
        x = self.relu(x)
        x = np.dot(self.out_w, x) + self.out_b
        x = np.tanh(x)
        return x


PUBLISH_RATE = 20.0  # number of controll commands to publish in one second

class BugController:
    def __init__(self, a, b):
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.lidar_sub = rospy.Subscriber(
            '/scan', LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(
            '/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=1)
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

        self.indicies = [int(i) for i in np.linspace(0,99,30)]

        with open('final_4_60.pickle', 'rb') as f:
            self.agent = pickle.load(f)

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion(
            [rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        print self.x, self.y

    def lidar_callback(self, msg):
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_inc = msg.angle_increment

        self.ranges = np.array(msg.ranges)[self.indicies]
        self.ranges[self.ranges>15] = 15


    def point_callback(self, msg):
        if isinstance(msg, PointStamped):
            self.goal_point = (msg.point.x, msg.point.y)
        else:
            self.goal_point = (msg.pose.position.x,
                               msg.pose.position.y)
        print(self.goal_point)

    def diff(self, a1, a2):
        '''
        Calculates the difference between two angles as well as the sign from a2 to
        a1.
        '''

        phi = abs(a1 - a2)%(2*pi)
        test = ((a1 - a2 >= 0) & (a1 - a2 <= pi)) or ((a1 - a2 <=-pi) & (a1- a2>= -2*pi))

        if test:
            sign = 1
        else:
            sign = -1

        if phi > pi:
            return 2*pi-phi, sign
        else:
            return phi, sign

    def bug_finder(self):
        # Spawning car
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = rospy.Time.now()
        ack_msg.drive.steering_angle = 0
        ack_msg.drive.speed = 0

        # Defining Goal
        goal = Point()
        goal.x = self.goal_point.x
        goal.y = self.goal_point.y

        self.drive_pub.publish(ack_msg)

        ack_msg = AckermannDriveStamped()
        # pdb.set_trace()
        rospy.sleep(2.0)

        dist = sqrt((goal.x - self.x)**2 + (goal.y - self.y)**2)
        angle_to_goal = atan2((goal.y - self.y), (goal.x - self.x))
        phi, sign = self.diff(self.theta, angle_to_goal)

        action = [[0,0]]
        print self.ranges
        while not rospy.is_shutdown():

            dist = sqrt((goal.x - self.x)**2 + (goal.y - self.y)**2)
            angle_to_goal = atan2((goal.y - self.y), (goal.x - self.x))
            phi, sign = self.diff(self.theta, angle_to_goal)

            if dist < 0.5:
                ack_msg.header.stamp = rospy.Time.now()
                ack_msg.drive.steering_angle = 0
                ack_msg.drive.speed = 0
                self.drive_pub.publish(ack_msg)
            else:
                obs = list(1-np.array(self.ranges)/15)
                act = action[0]
                obs.extend([dist/15, phi/pi, sign])
                obs.extend(act)
                obs = np.array(obs)
                action = self.agent.forward(obs).T
                action[0][1] = action[0][1]*np.pi/4
                rospy.loginfo(
                    "Distance to the goal: [%.2f]\nAngle to the goal: [%.2f]", dist, angle_to_goal)

                ack_msg.header.stamp = rospy.Time.now()
                ack_msg.drive.steering_angle = action[0][1]
                ack_msg.drive.speed = action[0][0]
                self.drive_pub.publish(ack_msg)
            rospy.sleep(1.0/PUBLISH_RATE)


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
