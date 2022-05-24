#!/usr/bin/env python3

from math import sin, cos, exp, pi, atan2
import numpy as np
import random
import matplotlib.pyplot as plt

class Particle:
    def __init__(self, x=0, y=0, theta=0):
        self.x = x
        self.y = y
        self.theta = theta

        self.weight = 1

class ParticleFilter:
    def __init__(self, num_particles=720, gps_noise=None, odom_noise=None):
        # Parameters
        self.num_particles = num_particles

        if gps_noise is None:
            self.gps_noise=[1.0]
        else:
            self.gps_noise = gps_noise

        if odom_noise is None:
            self.odom_noise=[0.5, 0.5, 0.2]
        else:
            self.odom_noise = odom_noise

        # Uniformly distritube particles
        self.particles = [Particle(0,0,i/self.num_particles * 2 * pi) for i in range(self.num_particles)]

        self.first_gps = None
        self.last_gps = None


    def update_odom(self, data, display_details):
        i, test_file, display_bool = display_details
        sum_x = 0
        sum_y = 0
        sum_theta_x = 0
        sum_theta_y = 0

        for particle in self.particles:
            particle.x += data.delta_x * cos(particle.theta) + data.delta_y * sin(particle.theta)
            particle.y += data.delta_x * sin(particle.theta) + data.delta_y * cos(particle.theta)
            particle.theta += data.delta_theta
            particle.theta = particle.theta % (2 * pi)

            sum_x += particle.x#*particle.weight
            sum_y += particle.y#*particle.weight
            sum_theta_x += cos(particle.theta)#*particle.weight
            sum_theta_y += sin(particle.theta)#*particle.weight

        # Report current average of particles
        avg_x = sum_x / self.num_particles
        avg_y = sum_y / self.num_particles
        avg_theta = atan2(sum_theta_y / self.num_particles, sum_theta_x / self.num_particles) % (2 * pi)

        if (display_bool):
            display_particles(self,f'plots/{test_file}/{i}_updateodom.png')
        return (avg_x, avg_y, avg_theta)

    def update_gps(self, data, display_details):
        i, test_file, display_bool = display_details
        if self.first_gps is None:
            self.first_gps = (data.latitude, data.longitude)

        gps_x = (data.latitude - self.first_gps[0]) * 110984.8 # Approximations
        gps_y = (self.first_gps[1] - data.longitude) * 90994.1

        if self.last_gps is None:
            self.last_gps = (gps_x, gps_y)

        dif_x = gps_x - self.last_gps[0]
        dif_y = gps_y - self.last_gps[1]
        theta = atan2(dif_x, dif_y)

        for particle in self.particles:
            dist_sqr = np.sqrt((particle.x - gps_x)**2 + (particle.y - gps_y)**2)
            dif_x_part = (particle.x - gps_x)
            dif_y_part = particle.y - gps_y
            theta_part = atan2(dif_x_part, dif_y_part)
            theta_sqr = np.sqrt((theta_part- theta)**2)

            #dist_sqr *= theta_sqr

            particle.weight = exp(-dist_sqr / (2 * self.gps_noise[0]**2))

        if (display_bool):
            display_particles(self,f'plots/{test_file}/{i}_0before_resample.png')
            self.resample()
            display_particles(self,f'plots/{test_file}/{i}_1after_resample.png')
        else:
            self.resample()

        self.last_gps = (gps_x, gps_y)
        return (gps_x, gps_y)

    def resample(self):
        weights = [particle.weight for particle in self.particles]
        weights_sum = sum(weights)
        if weights_sum < 0.00001:
            weights_sum = 0.00001
        weights = [weight / weights_sum for weight in weights]

        new_particles = random.choices(self.particles, weights, k=self.num_particles)
        self.particles = []

        for particle in new_particles:
            # Sprinkle some random
            rand_x = np.random.normal(0, self.odom_noise[0])
            x = particle.x + rand_x * cos(particle.theta)
            y = particle.y + rand_x * sin(particle.theta)
            theta = np.random.normal(particle.theta, self.odom_noise[2])

            self.particles.append(Particle(x, y, theta))                

def display_particles(self, filename):
    xpoints = []
    ypoints = []
    theta = []
    weight = []

    for particle in self.particles:
        xpoints.append(particle.x)
        ypoints.append(particle.y)
        theta.append(particle.theta)
        weight.append(particle.weight)


    u, v = (np.cos(theta), np.sin(theta))

    plt.cla()
    plt.quiver(xpoints, ypoints, u, v, linewidths=1, scale=40, label='PF', color='blue')
    plt.xlim(-5, 15)
    plt.ylim(-5,15)
    plt.savefig(filename)

