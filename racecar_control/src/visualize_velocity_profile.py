#!/usr/bin/env python

import rospy, math
from geometry_msgs.msg import Twist
import numpy as np
import matplotlib.pyplot as plotter

def twist_callback(data):
    global twist_velocity

    if not data:
        return
    twist_velocity = data


def plot_velocity_profile(fig, ax_v, ax_omega, t, v, omega):
    ax_v.cla()
    ax_v.grid()
    ax_v.set_ylabel('Trans. velocity [m/s]')
    ax_v.plot(t, v, '-bx')
    ax_omega.cla()
    ax_omega.grid()
    ax_omega.set_ylabel('Rot. velocity [rad/s]')
    ax_omega.set_xlabel('Time [s]')
    ax_omega.plot(t, omega, '-bx')
    fig.canvas.draw()


def plot_velocity():
    global twist_velocity

    rospy.init_node("visualize_velocity", anonymous=True)

    rospy.Subscriber("/cmd_vel", Twist, twist_callback, queue_size = 1)

    fig, (ax_v, ax_omega) = plotter.subplots(2, sharex=True)
    plotter.ion()
    plotter.show()

    t = 0
    r = rospy.Rate(2) # define rate here
    
    while not rospy.is_shutdown():
        plot_velocity_profile(
                fig, ax_v, ax_omega, np.array(t),
                np.array(twist_velocity.linear.x),
                np.array(twist_velocity.angular.z)
        )
        t += 1
        r.sleep()



if __name__ == '__main__':
    try:
        twist_velocity = Twist()
        plot_velocity()
    except rospy.ROSInterruptException:
        pass
