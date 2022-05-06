#!/usr/bin/env python3

import rospy
from enum import Enum
from igvc_msgs.msg import EKFState, velocity, gps, deltaodom
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16, Bool
from tf import transformations
from math import sin, cos

import sys
import PyQt5 as Qt
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QLabel, QHBoxLayout, QVBoxLayout
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QFont

import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

from pygame import mixer

mixer.init()


app = None

class SystemState(Enum):
    DISABLED = 0
    MANUAL = 1
    AUTONOMOUS = 2

class IGVCPathCanvas(FigureCanvasQTAgg):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        # self.axes2 = fig.add_subplot(122)
        super(IGVCPathCanvas, self).__init__(fig)

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
        self.hlayout.addLayout(self.vlayout)

        self.path_canvas = IGVCPathCanvas(self, width=4, height=4, dpi=120)
        self.hlayout.addWidget(self.path_canvas)

        self.system_state_label = QLabel(self)
        self.system_state_label.setText(f"System State: {'DISABLED'}, MobilityStart: {True}")
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
        self.vlayout.addWidget(self.dead_rekt_label)
        self.vlayout.addWidget(self.wheels_label)
        self.vlayout.addWidget(self.pose_label)
        self.vlayout.addWidget(self.gps_label)

        self.setCentralWidget(self.centralWidget)

        # Setup draw timer
        self.timer = QTimer()
        self.timer.setInterval(300)
        self.timer.timeout.connect(self.draw_timer)
        self.timer.start()

        self.curMap = None
        self.lastEKF = None
        self.ekfAtMap = None
        self.path = None
        self.system_state = SystemState.DISABLED
        self.mobi_start = False
        self.dead_rekt_cum = (0,0,0)

        # self.cam = cv2.VideoCapture(0)

        # Setup Music
        mixer.music.load('/home/igvc/music/genki_dama.wav')

        # Subscribe to necessary topics
        rospy.Subscriber("/igvc_slam/local_config_space", OccupancyGrid, self.c_space_callback, queue_size=1)
        rospy.Subscriber("/igvc/state", EKFState, self.ekf_callback)
        rospy.Subscriber("/igvc/local_path", Path, self.path_callback)
        rospy.Subscriber("/igvc/velocity", velocity, self.velocity_callback)
        rospy.Subscriber("/igvc/deltaodom", deltaodom, self.deltaodom_callback)
        rospy.Subscriber("/igvc/gps", gps, self.gps_callback)
        rospy.Subscriber("/igvc/system_state", Int16, self.system_state_callback)
        rospy.Subscriber("/igvc/mobstart", Bool, self.mobi_start_callback)

    def draw(self):
        if self.curMap and self.lastEKF and self.path:
            self.path_canvas.axes.clear()

            self.path_canvas.axes.imshow(np.rot90(np.transpose(np.reshape(self.curMap, (200, 200))), 2), interpolation = 'nearest', extent=[-10, 10, -10, 10])

            # Zoom into -5m to 5m
            
            for pose in self.path.poses:
                point = (pose.pose.position.x, pose.pose.position.y)
                self.path_canvas.axes.plot(-point[1], point[0], '.', markersize=8, color="red")

            robot_pos = (self.lastEKF.x - self.ekfAtMap[0], self.lastEKF.y - self.ekfAtMap[1])
            self.path_canvas.axes.plot(robot_pos[1], -robot_pos[0], '.', markersize=16, color="black")

            self.path_canvas.axes.set_xlim(-5, 5)
            self.path_canvas.axes.set_ylim(0, 10)

            # yes, pic = self.cam.read()

            # if yes:
            #     self.path_canvas.axes2.imshow(pic)

            self.path_canvas.draw()

    def draw_timer(self):
        self.draw()
        
        if rospy.is_shutdown():
            # Cleanup with ROS is done
            app.quit()

    def mobi_start_callback(self, data):
        self.mobi_start = data.data
        self.system_state_label.setText(f"System State: {self.system_state.name}, MobilityStart: {self.mobi_start}")

        if data.data == True and self.system_state == SystemState.AUTONOMOUS:
            mixer.music.play()

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
        self.system_state_label.setText(f"System State: {self.system_state.name}, MobilityStart: {self.mobi_start}")

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

    def ekf_callback(self, data):
        self.lastEKF = data
        self.pose_label.setText(f"Pose: {data.x:0.01f}m, {data.y:0.01f}m,{data.left_velocity:0.01f}m/s,{data.right_velocity:0.01f}m/s\n\t{data.yaw * 180/3.14:0.01f}°, {data.yaw_rate * 180 / 3.14:0.01f}°/s\n\t{data.latitude}, {data.longitude}")

    def c_space_callback(self, data):
        self.curMap = data.data
        self.ekfAtMap = (self.lastEKF.x, self.lastEKF.y)

    def path_callback(self, data):
        self.path = data

# Main setup
if __name__ == '__main__':
    try:
        app = QApplication([])
        app.setApplicationName("SCR IGVC 21")
        igvc_window = IGVCWindow()
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass
