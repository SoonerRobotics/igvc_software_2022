#!/usr/bin/env python3

from math import sin, cos, exp, pi, atan2
import numpy as np
import random

class Particle:
    def __init__(self, x=0, y=0, theta=0, weight=1):
        self.x = x
        self.y = y
        self.theta = theta

        self.weight = weight

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
        self.init_particles()

        self.first_gps = None
        self.last_gps = None

        self.gps_points_x = []
        self.gps_points_y = []

    def init_particles(self):
        self.particles = [Particle(0,0,i/self.num_particles * 2 * pi) for i in range(self.num_particles)]

    def set_first_gps(self, first_gps):
        self.first_gps = first_gps


    def update_odom(self, data):
        sum_x = 0
        sum_y = 0
        sum_theta_x = 0
        sum_theta_y = 0
        sum_weight = 0

        for particle in self.particles:
            particle.x += data.delta_x * 1.2 *cos(particle.theta) + data.delta_y * sin(particle.theta)
            particle.y += data.delta_x * 1.2 * sin(particle.theta) + data.delta_y * cos(particle.theta)
            particle.theta += data.delta_theta
            particle.theta = particle.theta % (2 * pi)

            weight = particle.weight ** 2

            sum_x += particle.x * weight
            sum_y += particle.y * weight
            sum_theta_x += cos(particle.theta) * weight
            sum_theta_y += sin(particle.theta) * weight
            sum_weight += weight

        if sum_weight  < 0.000001:
            sum_weight = 0.000001

        # Report current average of particles
        avg_x = sum_x / sum_weight
        avg_y = sum_y / sum_weight
        avg_theta = atan2(sum_theta_y / sum_weight, sum_theta_x / sum_weight) % (2 * pi)

        # if (display_bool):
        #     display_particles(self,f'plots/{test_file}/{i}_updateodom.png')
        return (avg_x, avg_y, avg_theta)

    def update_gps(self, data):
        if self.first_gps is None:
            self.first_gps = (data.latitude, data.longitude)

        gps_x = (data.latitude - self.first_gps[0]) * 111086.2 # Approximations
        gps_y = (self.first_gps[1] - data.longitude) * 81978.2

        if self.last_gps is None:
            self.last_gps = (gps_x, gps_y)

        dif_x = gps_x - self.last_gps[0]
        dif_y = gps_y - self.last_gps[1]
        theta = atan2(dif_x, dif_y)%(2*pi)

        for particle in self.particles:
            dist_sqr = np.sqrt((particle.x - gps_x)**2 + (particle.y - gps_y)**2)
            particle.weight = exp(-dist_sqr / (2 * self.gps_noise[0]**2))

            # dif_x_part = (particle.x - gps_x)
            # dif_y_part = (particle.y - gps_y)
            # theta_part = atan2(dif_x_part, dif_y_part)%(2*pi)
            # theta_sqr = ((theta_part-theta)**2)%(2*pi)

            # dist_sqr *= (5*theta_sqr) # trying to add in theta heading to weighting, not just gps loc

            # particle.weight = exp(-((dif_x_part**2/(2 * self.gps_noise[0]**2))
            #                         +(dif_y_part**2/(2 * self.gps_noise[0]**2))
            #                         +(theta_sqr/(2 * self.odom_noise[2]**2))))

        self.resample()

        self.last_gps = (gps_x, gps_y)
        self.gps_points_x.append(gps_x)
        self.gps_points_y.append(gps_y)
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
            rand_y = np.random.normal(0, self.odom_noise[1])
            x = particle.x + rand_x * cos(particle.theta) + rand_y * sin(particle.theta)
            y = particle.y + rand_x * sin(particle.theta) + rand_y * cos(particle.theta)
            theta = np.random.normal(particle.theta, self.odom_noise[2]) % (2 * pi)
            self.particles.append(Particle(x, y, theta, particle.weight))                

