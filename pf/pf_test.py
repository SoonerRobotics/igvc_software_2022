import particle_filter as pf
import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt


pf = pf.ParticleFilter(num_particles=400, gps_noise=[0.2], odom_noise=[0.1, 0.2, 0.8])
test_file = "pf_test_20220520-185151.csv"
test_noend = test_file.split(".")[0]

display_particles = True

if not os.path.exists(f'plots/{test_noend}'):
    os.makedirs(f'plots/{test_noend}')

class gps:
    def __init__(self, latitude, longitude):
        self.latitude = float(latitude)
        self.longitude = float(longitude)

class odom:
    def __init__(self, delta_x, delta_y, delta_theta):
        self.delta_x = float(delta_x)
        self.delta_y = float(delta_y)
        self.delta_theta = float(delta_theta)

with open(test_file,"r") as file:
    xpoints = []
    ypoints = []
    theta = []
    gps_xpoints = []
    gps_ypoints = []

    # read in file
    tsv_file = pd.read_csv(file,engine='c', header=0)

    for idx, row in tsv_file.iterrows():
        display_details = (idx, test_noend, display_particles) # info needed to save displaying particles per update
        if not np.isnan(row['gps_lat']):
            # GPS
            gps_data = gps(row['gps_lat'], row['gps_lon'])
            (gps_x, gps_y) = pf.update_gps(gps_data, display_details)
            gps_xpoints.append(gps_x)
            gps_ypoints.append(gps_y)

        if not np.isnan(row['dr_x']):
            # Odom
            odom_data = odom(row['dr_x'], row['dr_y'], row['dr_theta'])
            avg_x, avg_y, avg_theta = pf.update_odom(odom_data, display_details)

            #print(avg_x, avg_y, avg_theta)
            xpoints.append(avg_x)
            ypoints.append(avg_y)
            theta.append(avg_theta)

    # u, v = array*(np.cos(theta), np.sin(theta))
    u, v = (np.cos(theta), np.sin(theta))

    #plt.plot(xpoints, ypoints, plt.quiver(u,v), '.', label='PF', color='blue')
    plt.cla()
    plt.quiver(xpoints, ypoints, u, v, linewidths=1, label='PF', color='blue')
    plt.plot(gps_xpoints, gps_ypoints, '.', label='GPS', color='red')
    plt.legend()
    plt.savefig('plots/particles.png')
    plt.show()
