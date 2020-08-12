#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped

"""
Base class for nodes which publish the AckermannDriveStamped message
"""


class AckermannPublisher(object):

    def __init__(self, node_name):
        rospy.init_node(node_name, log_level=rospy.INFO)
        self.publisher = rospy.Publisher('/ackermann_cmd_mux/input/default',
                                         AckermannDriveStamped, queue_size=5)
        rospy.loginfo("Started node %s" % node_name)

    def publish_ackermann(self, steering, speed):
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.drive.speed = speed
        msg.drive.acceleration = 1
        msg.drive.jerk = 1
        msg.drive.steering_angle = steering
        msg.drive.steering_angle_velocity = 1
        self.publisher.publish(msg)
