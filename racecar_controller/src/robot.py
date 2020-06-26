#!/usr/bin/env python3

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from threading import Thread  # imsosorrrom sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PointStamped
import utils

PUBLISH_RATE = 20.0  # number of control commands to publish per second


class Robot:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    # SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    # DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    # SIDE = rospy.get_param("wall_follower/side")
    # VELOCITY = rospy.get_param("wall_follower/velocity")
    # DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")

    def __init__(self):
        # Subscribers and Publishers

        self.drive_pub = rospy.Publisher(
            '/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=1)
        self.odom_sub = rospy.Subscriber(
            '/odom', Odometry, self.odom_callback, queue_size=1)
        self.laser_sub = rospy.Subscriber(
            '/scan', LaserScan, self.lidar_callback, queue_size=1)
        # listen for 2D Pose or 2D Nav
        self.publish_point_sub = rospy.Subscriber(
            '/clicked_point', PointStamped, self.point_callback, queue_size=1)
        self.failsafe_sub = rospy.Subscriber(
            'failsafe', String, self.failsafe_callback)
        self.should_stop = False
        print("Initialized. Waiting on messsages...")

    def failsafe_callback(self, msg):
        if msg.data == 'stop_event':
            self.should_stop = True
        else:
            self.should_stop = False

    def odom_callback(self, msg):
        """Extracts robot state information"""
        pose = np.array([msg.pose.pose.position.x,
                         msg.pose.pose.position.y,
                         utils.quaternion_to_angle(msg.pose.pose.orientation)])
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        print(self.x, self.y)

    def point_callback(self, msg):
        if isinstance(msg, PointStamped):
            self.goal_point = (msg.point.x, msg.point.y)
        else:
            self.goal_point = (msg.pose.position.x,
                               msg.pose.position.y)
        print(self.goal_point)

    def lidar_callback(self, msg):
        pass
        # print(msg.ranges)

    def drive_circle(self):
        while not rospy.is_shutdown():
            # if not self.should_stop:
            #     self.apply_control(1.5, 35)
            rospy.sleep(1.0/PUBLISH_RATE)

    def apply_control(self, speed, steering_angle):
        self.actual_speed = speed
        drive_msg_stamped = AckermannDriveStamped()
        drive_msg = AckermannDrive()
        drive_msg.speed = speed
        drive_msg.steering_angle = steering_angle
        drive_msg.acceleration = 0
        drive_msg.jerk = 0
        drive_msg.steering_angle_velocity = 0
        drive_msg_stamped.drive = drive_msg
        self.drive_pub.publish(drive_msg_stamped)
        rospy.sleep(1.0/PUBLISH_RATE)

    def drive_straight(self):
        while not rospy.is_shutdown():
            # self.apply_control(0.2, 0)
            rospy.sleep(1.0/PUBLISH_RATE)

    def stop(self):
        drive_msg_stamped = AckermannDriveStamped()
        drive_msg = AckermannDrive()
        drive_msg.speed = 0
        drive_msg.steering_angle = 0
        drive_msg.acceleration = 0
        drive_msg.jerk = 0
        drive_msg.steering_angle_velocity = 0
        drive_msg_stamped.drive = drive_msg
        self.drive_pub.publish(drive_msg_stamped)


if __name__ == "__main__":
    rospy.init_node('bug_wall_follower')
    robot = Robot()
    robot.drive_straight()
    rospy.spin()
