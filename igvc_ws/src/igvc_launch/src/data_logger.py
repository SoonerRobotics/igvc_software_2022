#!/usr/bin/env python3

import rospy
import csv
from datetime import datetime
from igvc_msgs.msg import EKFState, gps, deltaodom
from std_msgs.msg import Bool

class DataLogger:
    def __init__(self):
        self.mobi_start = False

        rospy.Subscriber("/igvc/state", EKFState, self.pf_callback)
        rospy.Subscriber("/igvc/deltaodom", deltaodom, self.deltaodom_callback)
        rospy.Subscriber("/igvc/gps", gps, self.gps_callback)
        rospy.Subscriber("/igvc/mobstart", Bool, self.mobi_start_callback)

        self.csvfile = open(f"/home/igvc/igvc_data/datalog_{datetime.now().strftime('%Y%m%d-%H%M%S')}.csv", "w")
        self.csvwriter = csv.writer(self.csvfile)

        # self.csvwriter.writerow("timestamp, gps_lat, gps_lon, pf_lat, pf_lon, pf_x, pf_y, dr_x, dr_y")
        self.csvwriter.writerow(["timestamp","gps_lat","gps_lon","pf_lat","pf_lon","pf_x","pf_y","dr_x","dr_y","dr_theta"])
    
    def pf_callback(self, data):
        if self.mobi_start:
            self.csvwriter.writerow([rospy.Time.now(),None,None,data.latitude,data.longitude,data.x,data.y,None,None,None])

    def deltaodom_callback(self, data):
        if self.mobi_start:
            self.csvwriter.writerow([rospy.Time.now(),None,None,None,None,None,None,data.delta_x,data.delta_y,data.delta_theta])

    def gps_callback(self, data):
        if data.hasSignal and self.mobi_start:
            self.csvwriter.writerow([rospy.Time.now(),data.latitude,data.longitude,None,None,None,None,None,None,None])

    def mobi_start_callback(self, data):
        self.mobi_start = data.data

def data_logger():
    rospy.init_node('data_logger', anonymous=True)

    dl = DataLogger()

    rospy.spin()

    dl.csvfile.close()


if __name__ == '__main__':
    try:
        data_logger()
    except rospy.ROSInterruptException:
        pass

