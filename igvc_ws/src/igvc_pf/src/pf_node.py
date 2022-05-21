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
        self.num_particles = 720

        # ROS Setup
        # rospy.Subscriber("/igvc/velocity", velocity, self.velocity_callback)
        rospy.Subscriber("/igvc/deltaodom", deltaodom, self.deltaodom_callback)
        rospy.Subscriber("/igvc/gps", gps, self.gps_callback)
        rospy.Subscriber("/igvc/mobstart", Bool, self.mobi_start_callback)
        rospy.Subscriber("/igvc/system_state", Int16, self.system_state_callback)

        self.state_pub = rospy.Publisher("/igvc/state", EKFState, queue_size=1)

        # rospy.Timer(rospy.Duration(self.dt), self.timer_callback)

        # Uniformly distritube particles
        self.particles = [Particle(0,0,i/self.num_particles * 2 * pi) for i in range(self.num_particles)]

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
            self.particles = [Particle(0,0,i/self.num_particles * 2 * pi) for i in range(self.num_particles)]

    def deltaodom_callback(self, data:deltaodom):

        sum_x = 0
        sum_y = 0
        sum_theta_x = 0
        sum_theta_y = 0

        for particle in self.particles:
            particle.x += data.delta_x * cos(particle.theta) + data.delta_y * sin(particle.theta)
            particle.y += data.delta_x * sin(particle.theta) + data.delta_y * cos(particle.theta)
            particle.theta += data.delta_theta
            particle.theta = particle.theta % (2 * pi)

            sum_x += particle.x
            sum_y += particle.y
            sum_theta_x += cos(particle.theta)
            sum_theta_y += sin(particle.theta)

        # Report current average of particles
        avg_x = sum_x / self.num_particles
        avg_y = sum_y / self.num_particles
        avg_theta = atan2(sum_theta_y / self.num_particles, sum_theta_x / self.num_particles) % (2 * pi)

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

            return
        else:
            # Publish some GPS coords as waypoints
            local_path = Path()
            path = [(35.210514, -97.441929), (35.210634, -97.442109), (35.210634, -97.442325), (35.210477, -97.442324), (35.210477, -97.442117)]
            local_path.poses = [gps_point_to_xy_point(path_point[0], path_point[1], self.first_gps[0], self.first_gps[1]) for path_point in path]

            self.global_path_pub.publish(local_path)

        gps_x = (data.latitude - self.first_gps[0]) * 110984.8 # Approximations
        gps_y = (self.first_gps[1] - data.longitude) * 90994.1

        for particle in self.particles:
            dist_sqr = (particle.x - gps_x)**2 + (particle.y - gps_y)**2

            particle.weight = exp(-dist_sqr / (2 * 1**2))

        self.resample()

    def timer_callback(self, time):
        self.kinematic_update()

    def resample(self):
        weights = [particle.weight for particle in self.particles]
        weights_sum = sum(weights)
        if weights_sum < 0.00001:
            weights_sum = 0.00001
        weights = [weight / weights_sum for weight in weights]

        print(f"Confidence: {weights_sum}")

        new_particles = random.choices(self.particles, weights, k=self.num_particles)
        self.particles = []

        for particle in new_particles:
            # Sprinkle some random
            x = np.random.normal(particle.x, 0.5)
            y = np.random.normal(particle.y, 0.5)
            theta = np.random.normal(particle.theta, 0.2)

            self.particles.append(Particle(x, y, theta))            


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
