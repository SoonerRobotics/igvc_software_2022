#!/usr/bin/env python3

import rospy
from enum import Enum
from igvc_msgs.msg import EKFState, velocity, gps, deltaodom
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import Image
from std_msgs.msg import Int16, Bool
from math import sin, cos
from datetime import datetime

import sys
import PyQt5 as Qt
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QLabel, QHBoxLayout, QVBoxLayout
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QFont

import numpy as np
import matplotlib as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
import matplotlib.animation as animation

import time

from pygame import mixer

import cv2
from cv_bridge import CvBridge
bridge = CvBridge()

mixer.init()

# Camera Vertical vision (m)
camera_vertical_distance = 2.75
# Camera Horizontal vision (m)
camera_horizontal_distance = 3

app = None

class SystemState(Enum):
    DISABLED = 0
    MANUAL = 1
    AUTONOMOUS = 2

class IGVCPathCanvas(FigureCanvasQTAgg):
    def __init__(self, parent=None, width=1, height=1, dpi=10):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = self.fig.add_subplot(111)
        # self.axes2 = fig.add_subplot(122)
        super(IGVCPathCanvas, self).__init__(self.fig)

class IGVCWindow(QMainWindow):
    def __init__(self):
        super(IGVCWindow, self).__init__()

        # Setup node
        rospy.init_node("igvc_display_node")

        # Setup window
        self.setWindowTitle("SCR IGVC 21")
        self.showMaximized()

        # Add components to window
        self.centralWidget = QWidget()
        self.hlayout = QHBoxLayout()
        self.centralWidget.setLayout(self.hlayout)

        self.vlayout = QVBoxLayout()
        self.hlayout.addLayout(self.vlayout, stretch=1)

        self.path_canvas = IGVCPathCanvas(self, width=1, height=1, dpi=120)
        self.hlayout.addWidget(self.path_canvas, stretch=1)

        self.system_state_label = QLabel(self)
        self.system_state_label.setText(f"System State: {'DISABLED'}\nMobility: {'Start'}")
        self.system_state_label.setFont(QFont('Arial', 32))

        self.speed_label = QLabel(self)
        self.speed_label.setText(f"Speed: {0:0.01f}mph")
        self.speed_label.setFont(QFont('Arial', 32))

        self.dead_rekt_label = QLabel(self)
        self.dead_rekt_label.setText(f"Dead rekt: Waiting...")
        self.dead_rekt_label.setFont(QFont('Arial', 32))

        self.wheels_label = QLabel(self)
        self.wheels_label.setText(f"Wheels: Waiting...")
        self.wheels_label.setFont(QFont('Arial', 24))

        self.pose_label = QLabel(self)
        self.pose_label.setText(f"Pose: Waiting...")
        self.pose_label.setFont(QFont('Arial', 24))

        self.gps_label = QLabel(self)
        self.gps_label.setText(f"GPS: Waiting...")
        self.gps_label.setFont(QFont('Arial', 24))

        self.vlayout.addWidget(self.system_state_label)
        self.vlayout.addWidget(self.speed_label)
        # self.vlayout.addWidget(self.dead_rekt_label)
        self.vlayout.addWidget(self.wheels_label)
        self.vlayout.addWidget(self.pose_label)
        self.vlayout.addWidget(self.gps_label)

        self.setCentralWidget(self.centralWidget)

        # # Setup draw timer
        self.timer = QTimer()
        self.timer.setInterval(200)
        self.timer.timeout.connect(self.draw_timer)
        self.timer.start()

        # self.ani = animation.FuncAnimation(self.path_canvas.fig, self.draw, interval=1000)

        self.curMap = None
        self.lastEKF = None
        self.ekfAtMap = None
        self.path = None
        self.system_state = SystemState.DISABLED
        self.mobi_start = False
        self.dead_rekt_cum = (0,0,0)
        self.last_image = None
        self.first_draw = True

        # self.cam = cv2.VideoCapture(0)

        # Setup Music
        mixer.music.load('/home/igvc/music/boomersooner.wav')

        # Subscribe to necessary topics
        rospy.Subscriber("/igvc_slam/local_config_space", OccupancyGrid, self.c_space_callback, queue_size=1)
        rospy.Subscriber("/igvc/state", EKFState, self.ekf_callback)
        rospy.Subscriber("/igvc/local_path", Path, self.path_callback)
        rospy.Subscriber("/igvc/velocity", velocity, self.velocity_callback)
        rospy.Subscriber("/igvc/deltaodom", deltaodom, self.deltaodom_callback)
        rospy.Subscriber("/igvc/gps", gps, self.gps_callback)
        rospy.Subscriber("/igvc/system_state", Int16, self.system_state_callback)
        rospy.Subscriber("/igvc/mobstart", Bool, self.mobi_start_callback)
        rospy.Subscriber("/igvc/preview", Image, self.image_callback)

        self.first_gps = None

    def draw_vision(self):
        if self.curMap is not None and self.lastEKF is not None:
            self.path_canvas.axes.cla()

            self.path_canvas.axes.imshow(self.last_image, interpolation = 'none', extent=[-camera_horizontal_distance/2, camera_horizontal_distance/2, 0, camera_vertical_distance])
            map_mat = np.reshape(self.curMap, (80, 80))

            # if self.first_draw:
            self.path_canvas.axes.set_ylabel('x (m)')
            self.path_canvas.axes.set_xlabel('y (m)')

            self.path_canvas.axes.set_xlim(-camera_horizontal_distance/2, camera_horizontal_distance/2)
            self.path_canvas.axes.set_ylim(0, camera_vertical_distance)

            self.norm = plt.colors.Normalize()

            # self.first_draw = False

            self.colors = plt.cm.jet(self.norm(map_mat))
            self.colors[:,:,3] = 0.5
            self.colors[map_mat == 0,3] = 0

            self.path_canvas.axes.imshow(self.colors, interpolation = 'none', extent=[-camera_horizontal_distance/2, camera_horizontal_distance/2, 0, camera_vertical_distance])

            # # Zoom into -5m to 5m
            
            if self.path:
                for pose in self.path.poses:
                    point = (pose.pose.position.x, pose.pose.position.y)
                    self.path_canvas.axes.plot(-point[1], point[0], '.', markersize=8, color="red")

            # robot_pos = (self.lastEKF.x, self.lastEKF.y)
            # self.path_canvas.axes.plot(-robot_pos[1], robot_pos[0], '.', markersize=16, color="black")

            # yes, pic = self.cam.read()

            # if yes:
            #     self.path_canvas.axes2.imshow(pic)

            self.path_canvas.draw_idle()
        # pass

    def draw_path(self):
        if self.curMap is not None and self.lastEKF is not None:
            self.path_canvas.axes.cla()

            #self.path_canvas.axes.imshow(self.last_image, interpolation = 'none', extent=[-camera_horizontal_distance/2, camera_horizontal_distance/2, 0, camera_vertical_distance])
            #map_mat = np.reshape(self.curMap, (100, 200))

            # if self.first_draw:
            self.path_canvas.axes.set_ylabel('x (m)')
            self.path_canvas.axes.set_xlabel('y (m)')

            #self.path_canvas.axes.set_xlim(-camera_horizontal_distance/2, camera_horizontal_distance/2)
            #self.path_canvas.axes.set_ylim(0, camera_vertical_distance)

            #self.norm = plt.colors.Normalize()

            # self.first_draw = False

            #self.colors = plt.cm.jet(self.norm(map_mat))
            #self.colors[:,:,3] = 0.5
            #self.colors[map_mat == 0,3] = 0

            #self.path_canvas.axes.imshow(self.colors, interpolation = 'none', extent=[-camera_horizontal_distance/2, camera_horizontal_distance/2, 0, camera_vertical_distance])

            # # Zoom into -5m to 5m
            
            if self.path:
                for pose in self.path.poses:
                    point = (pose.pose.position.x, pose.pose.position.y)
                    self.path_canvas.axes.plot(-point[1], point[0], '.', markersize=8, color="red")

            robot_pos = (self.lastEKF.x, self.lastEKF.y)
            self.path_canvas.axes.plot(-robot_pos[1], robot_pos[0], '.', markersize=16, color="black")

            # yes, pic = self.cam.read()

            # if yes:
            #     self.path_canvas.axes2.imshow(pic)

            self.path_canvas.draw_idle()
        # pass

    def draw_timer(self):
        if rospy.is_shutdown():
            # Cleanup with ROS is done
            app.quit()
            # self.csvfile.close()

        self.draw_vision()
        # self.draw_path()

    def mobi_start_callback(self, data):
        self.mobi_start = data.data
        self.system_state_label.setText(f"System State: {self.system_state.name}\nMobility: {'Start' if self.mobi_start else 'Stop'}")

        if data.data == True and self.system_state == SystemState.AUTONOMOUS:
            mixer.music.play(-1)

        if data.data == False:
            mixer.music.stop()

    def deltaodom_callback(self, data:deltaodom):
        x = self.dead_rekt_cum[0]
        y = self.dead_rekt_cum[1]
        theta = self.dead_rekt_cum[2]
        self.dead_rekt_cum = (x + data.delta_x * cos(theta) + data.delta_y * sin(theta), y + data.delta_x * sin(theta) + data.delta_y * cos(theta), theta + data.delta_theta)
        self.dead_rekt_label.setText(f"Dead rekt: {self.dead_rekt_cum[0]:0.02f},{self.dead_rekt_cum[1]:0.02f},{self.dead_rekt_cum[2]:0.02f}")

    def system_state_callback(self, data):
        self.system_state = SystemState(data.data)
        self.system_state_label.setText(f"System State: {self.system_state.name}\nMobility: {'Start' if self.mobi_start else 'Stop'}")

        if self.system_state == SystemState.DISABLED:
            self.dead_rekt_cum = (0, 0, 0)

    def velocity_callback(self, data):
        self.wheels_label.setText(f"Wheels: {data.leftVel:0.01f}m/s, {data.rightVel:0.01f}m/s")
        self.speed_label.setText(f"Speedometer: {(data.leftVel + data.rightVel) * 1.119:0.01f}mph")

    def gps_callback(self, data):
        if data.hasSignal:
            self.gps_label.setText(f"GPS:\n\t{data.latitude:0.07f}\n\t{data.longitude:0.07f}")
        else:
            self.gps_label.setText(f"GPS: No Signal")

        # timestamp, gps_lat, gps_lon, pf_lat, pf_lon, pf_x, pf_y, dr_x, dr_y
        if data.hasSignal and self.mobi_start:
            if self.first_gps == None:
                self.first_gps = (data.latitude, data.longitude)

    def ekf_callback(self, data):
        self.lastEKF = data
        self.pose_label.setText(f"Pose: {data.x:0.01f}m, {data.y:0.01f}m\n\t{data.yaw * 180/3.14:0.01f}°\n\t{data.latitude:0.06f}, {data.longitude:0.06f}")

    def c_space_callback(self, data):
        if self.lastEKF:
            self.curMap = data.data
            self.ekfAtMap = (self.lastEKF.x, self.lastEKF.y)

    def path_callback(self, data):
        self.path = data
    
    def image_callback(self, data):
        self.last_image = bridge.imgmsg_to_cv2(data)

    
# Main setup
if __name__ == '__main__':
    try:
        app = QApplication([])
        app.setApplicationName("SCR IGVC 21")
        igvc_window = IGVCWindow()
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass
