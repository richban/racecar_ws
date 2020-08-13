#!/usr/bin/env python

"""
This node subscribes to the control commands sent by teb_local_planner and publishes
 AckermannDriveStamped messages. The commands sent by move_base are of type
 geometry_msgs/Twist
"""

import rospy
from geometry_msgs.msg import Twist
from ackermann_publisher import AckermannPublisher
import math


def convert_ang_vel_to_steering_angle(v, omega, wheelbase):
  if omega == 0 or v == 0:
    return 0

  radius = v / omega
  return math.atan(wheelbase / radius)


class TEBPlannerFollower(AckermannPublisher):

    def __init__(self, node_name):
        super(TEBPlannerFollower, self).__init__(node_name)
        rospy.loginfo("Started TEB Planner Follower")
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback, queue_size=5)
        self.wheel_base = rospy.get_param("~wheel_base")

        # The TEB local planner computes time optimized trajectories and velocity commands
        # It does not take into account any minimum speed
        # Integrating this in the optimization scheme would increase the complexity, according to the authors
        # Source: https://github.com/rst-tu-dortmund/teb_local_planner/issues/10#issuecomment-226218740
        self.minimum_speed = rospy.get_param("~minimum_speed", 0.1)

        # Minimum acceptable velocity command
        # commands below this would be considered zero
        self.velocity_epsilon = rospy.get_param("~velocity_epsilon", 0.05)

        self.velocity = 0.0
        self.steering_angle = 0.0

        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            self.publish_ackermann(self.steering_angle, self.velocity)
            rate.sleep()

    def cmd_callback(self, msg):
        velocity_command = msg.linear.x
        # TO DO: fix if control
        # Should this code go into AckermannPublisher?
        speed = abs(velocity_command)

        if speed > self.minimum_speed:
            velocity = velocity_command
        elif speed > self.velocity_epsilon:
            sign = speed / velocity_command  # this will be +/- 1
            velocity = self.minimum_speed * sign
        else:
            if speed > 0.01:
                rospy.logwarn("[TEB Follower] Velocity command inferred as zero velocity")
            velocity = 0.0

        self.velocity = velocity
        self.steering_angle = convert_ang_vel_to_steering_angle(velocity, msg.angular.z,
                                                           self.wheel_base)


if __name__ == '__main__':
    TEBPlannerFollower("teb_cmd_to_ackermann_drive")
    # rospy.spin()
