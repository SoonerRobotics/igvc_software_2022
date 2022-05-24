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


    def update_odom(self, data):
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

        return (avg_x, avg_y, avg_theta)

    def update_gps(self, data, i, test_file,display_bool):
        if self.first_gps is None:
            self.first_gps = (data.latitude, data.longitude)

        gps_x = (data.latitude - self.first_gps[0]) * 110984.8 # Approximations
        gps_y = (self.first_gps[1] - data.longitude) * 90994.1

        for particle in self.particles:
            dist_sqr = (particle.x - gps_x)**2 + (particle.y - gps_y)**2

            particle.weight = exp(-dist_sqr / (2 * self.gps_noise[0]**2))

        if (display_bool):
            display_particles(self,f'plots/{test_file}/{i}_0before_resample.png')
            self.resample()
            display_particles(self,f'plots/{test_file}/{i}_1after_resample.png')
        else:
            self.resample()
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
            x = np.random.normal(particle.x, self.odom_noise[0])
            y = np.random.normal(particle.y, self.odom_noise[1])
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

    plt.quiver(xpoints, ypoints, u, v, linewidths=1, label='PF', color='blue')
    plt.xlim(-5, 15)
    plt.ylim(-5,15)
    plt.savefig(filename)

