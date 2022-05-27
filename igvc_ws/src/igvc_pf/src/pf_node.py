#!/usr/bin/env python3

from turtle import left, right
import rospy
from std_msgs.msg import Bool, Int16
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point
from igvc_msgs.msg import gps, velocity, motors, EKFState, imuodom, deltaodom
from math import sin, cos, exp, pi, atan2
import numpy as np
import random
from enum import Enum
import particle_filter as pf

import matplotlib as mpl
from matplotlib import pyplot as plt

class SystemState(Enum):
    DISABLED = 0
    MANUAL = 1
    AUTONOMOUS = 2

def setup_pyplot():
    plt.ion()
    plt.show()

# Draws pure pursuit information to pyplot
def draw_particles(particles):
    plt.figure(1)
    plt.clf()

    for point in particles:
        plt.quiver(point.x, point.y, cos(point.theta), sin(point.theta), scale=10/point.weight, scale_units='width')

    plt.draw()
    plt.pause(0.00000000001)

def gps_point_to_xy_point(lat, lon, base_lat, base_lon):
    pose_stamped = PoseStamped()
    pose_stamped.pose = Pose()

    point = Point()
    point.x = (lat - base_lat) * 110984.8
    point.y = (base_lon - lon) * 90994.1
    pose_stamped.pose.position = point

    return pose_stamped
class Particle:
    def __init__(self, x=0, y=0, theta=0):
        self.x = x
        self.y = y
        self.theta = theta

        self.weight = 1

class ParticleFilterNode:

    def __init__(self):
        # Parameters
        self.PF = pf.ParticleFilter(num_particles=500, gps_noise=[0.6], odom_noise=[0.05, 0.05, 0.1])

        # ROS Setup
        # rospy.Subscriber("/igvc/velocity", velocity, self.velocity_callback)
        rospy.Subscriber("/igvc/deltaodom", deltaodom, self.deltaodom_callback)
        rospy.Subscriber("/igvc/gps", gps, self.gps_callback)
        rospy.Subscriber("/igvc/mobstart", Bool, self.mobi_start_callback)
        rospy.Subscriber("/igvc/system_state", Int16, self.system_state_callback)

        self.state_pub = rospy.Publisher("/igvc/state", EKFState, queue_size=1)

        # rospy.Timer(rospy.Duration(self.dt), self.timer_callback)

        self.global_path_pub = rospy.Publisher("/igvc/global_path", Path, queue_size=1)

        self.first_gps = None
        self.collecting_GPS = True

    def mobi_start_callback(self, data:Bool):
        if data.data:
            # Mobi-started, use final estimate of inital GPS
            self.collecting_GPS = False

    def system_state_callback(self, data:Int16):
        # Reset first_gps estimate on state change (except when going into AUTONOMOUS)
        if SystemState(data.data) == SystemState.DISABLED:
            self.first_gps = None
            self.collecting_GPS = True
            self.PF.init_particles()

    def deltaodom_callback(self, data:deltaodom):

        avg_x, avg_y, avg_theta = self.PF.update_odom(data)

        output_msg = EKFState()
        output_msg.x = avg_x
        output_msg.y = avg_y
        output_msg.yaw = avg_theta

        if self.first_gps is not None:
            output_msg.latitude = self.first_gps[0] + avg_x / 110984.8
            output_msg.longitude = self.first_gps[1] - avg_y / 90994.1

        self.state_pub.publish(output_msg)

    def gps_callback(self, data: gps):

        if self.first_gps is None or self.collecting_GPS:
            if self.first_gps is None:
                self.first_gps = (data.latitude, data.longitude)
            else:
                self.first_gps = (0.8 * self.first_gps[0] + 0.2 * data.latitude,
                                    0.8 * self.first_gps[1] + 0.2 * data.longitude)
            self.PF.set_first_gps(self.first_gps)
            return
        else:
            # Publish some GPS coords as waypoints
            local_path = Path()
            path = [(35.2105295, -97.4419748), (35.2106288, -97.4421037), (35.210634, -97.442325), (35.210477, -97.442324), (35.210477, -97.442117)]
            local_path.poses = [gps_point_to_xy_point(path_point[0], path_point[1], self.first_gps[0], self.first_gps[1]) for path_point in path]

            self.global_path_pub.publish(local_path)

        self.PF.update_gps(data)      


def main():
    global state_pub
    # initalize the node in ROS
    rospy.init_node('pf_node')

    setup_pyplot()
    node = ParticleFilterNode()

    # pump callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
