#!/usr/bin/env python3

from turtle import left, right
import rospy
from igvc_msgs.msg import gps, velocity, motors, EKFState, imuodom
from math import sin, cos, exp
import numpy as np
import random

import matplotlib as mpl
from matplotlib import pyplot as plt

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

class Particle:
    def __init__(self, x=0, y=0, theta=0, left_vel=0, right_vel=0):
        self.x = x
        self.y = y
        self.theta = theta
        self.left_vel = left_vel
        self.right_vel = right_vel

        self.weight = 1

class ParticleFilterNode:

    def __init__(self):
        # Parameters
        self.dt = 0.02
        self.num_particles = 200

        # ROS Setup
        rospy.Subscriber("/igvc/velocity", velocity, self.velocity_callback)
        rospy.Subscriber("/igvc/gps", gps, self.gps_callback)

        self.state_pub = rospy.Publisher("/igvc/state", EKFState, queue_size=1)

        rospy.Timer(rospy.Duration(self.dt), self.timer_callback)

        self.particles = [Particle() for i in range(self.num_particles)]

        self.first_gps = (35.19488, -97.43862)

    def velocity_callback(self, data: velocity):
        sum_x = 0
        sum_y = 0

        for particle in self.particles:
            particle.left_vel = np.random.normal(data.leftVel / 10, 0.05)
            particle.right_vel = np.random.normal(data.rightVel / 10, 0.05)
            
            x_dot = (particle.left_vel + particle.right_vel) / 2 * cos(particle.theta)
            y_dot = (particle.left_vel + particle.right_vel) / 2 * sin(particle.theta)

            particle.x += x_dot * self.dt
            particle.y += y_dot * self.dt

            sum_x += particle.x
            sum_y += particle.y

            theta_dot = (particle.right_vel - particle.left_vel) * 0.4 # Constant from ratio of wheleradius to axle length TODO: fix
            particle.theta += theta_dot * self.dt

        draw_particles(self.particles)

        # print(f"Est Pose: {sum_x/self.num_particles:0.01f}, {sum_y/self.num_particles:0.01f}")

    def gps_callback(self, data: gps):

        if self.first_gps is None:
            self.first_gps = (data.latitude, data.longitude)
            return

        gps_x = (data.latitude - self.first_gps[0]) * 110944.12
        gps_y = (data.longitude - self.first_gps[1]) * 91071.17

        for particle in self.particles:
            dist_sqr = (particle.x - gps_x)**2 + (particle.y - gps_y)**2

            particle.weight = exp(-dist_sqr / (2 * 5**2))

        self.resample()

    def timer_callback(self, time):
        self.kinematic_update()

    def kinematic_update(self):
        # for particle in self.particles:
        #     x_dot = (particle.left_vel + particle.right_vel) / 2 * cos(particle.theta) + np.random.normal(0, 0.05, 1)
        #     y_dot = (particle.left_vel + particle.right_vel) / 2 * sin(particle.theta) + np.random.normal(0, 0.05, 1)

        #     particle.x += x_dot * self.dt
        #     particle.y += y_dot * self.dt

        #     theta_dot = (particle.left_vel - particle.right_vel) * 0.1 # Constant from ratio of wheleradius to axle length TODO: fix
        #     theta_dot += + np.random.normal(0, 0.01, 1)
        #     particle.theta += theta_dot * self.dt

        # draw_particles(self.particles)
        pass

    def resample(self):
        weights = [particle.weight for particle in self.particles]
        weights_sum = sum(weights)
        weights = [weight / weights_sum for weight in weights]

        print(f"Confidence: {weights_sum}")

        new_particles = random.choices(self.particles, weights, k=self.num_particles)
        self.particles = []

        for particle in new_particles:
            # Sprinkle some random
            x = np.random.normal(particle.x, 4.0)
            y = np.random.normal(particle.y, 4.0)
            theta = np.random.normal(particle.theta, 0.2)

            self.particles.append(Particle(x, y, theta, particle.left_vel, particle.right_vel))            


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
