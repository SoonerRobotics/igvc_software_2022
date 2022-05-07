#!/usr/bin/env python3

import rospy
import serial
import pynmea2
import csv
from igvc_msgs.msg import gps

# Initialize the serial node
# Node handles all serial communication within the robot (motor, GPS)
def init_serial_node():
    
    # Setup serial node
    rospy.init_node("emlid_node", anonymous = False)
    
    emlid = serial.Serial(port='/dev/igvc-emlid-633', baudrate = 38400)

    gps_pub = rospy.Publisher('/igvc/gps', gps, queue_size=1)
    
    while not rospy.is_shutdown():
        line = emlid.readline().decode()
        msg = pynmea2.parse(line)

        # print(f"gps: {line}")
        
        if isinstance(msg, pynmea2.GGA):
            gps_msg = gps()
            gps_msg.latitude = msg.latitude
            gps_msg.longitude = msg.longitude
            gps_msg.hasSignal = msg.is_valid

            gps_pub.publish(gps_msg)

    # with open("/home/zemlin/igvc_software_2021/igvc_ws/gps_out.csv", "w") as f:
        
    #     csvwriter = csv.writer(f)

    #     # Wait for topic updates
    #     while not rospy.is_shutdown():
    #         line = emlid.readline().decode()
    #         msg = pynmea2.parse(line)
            
    #         if isinstance(msg, pynmea2.GGA):
    #             print(f"{msg.latitude}, {msg.longitude}")
    #             csvwriter.writerow([rospy.Time.now(), msg.is_valid, msg.latitude, msg.longitude])

if __name__ == '__main__':
    try:
        init_serial_node()
    except rospy.ROSInterruptException:
        pass