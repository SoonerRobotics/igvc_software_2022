#!/usr/bin/env python3

import rospy
import serial
import pynmea2
import csv

# Initialize the serial node
# Node handles all serial communication within the robot (motor, GPS)
def init_serial_node():
    
    # Setup serial node
    rospy.init_node("emlid_node", anonymous = False)
    
    emlid = serial.Serial(port = '/dev/ttyACM0', baudrate = 38400)
    
    with open("/home/zemlin/igvc_software_2021/igvc_ws/gps_out.csv", "w") as f:
        
        csvwriter = csv.writer(f)

        # Wait for topic updates
        while not rospy.is_shutdown():
            line = emlid.readline().decode()
            msg = pynmea2.parse(line)
            
            if isinstance(msg, pynmea2.GGA):
                print(f"{msg.latitude}, {msg.longitude}")
                csvwriter.writerow([rospy.Time.now(), msg.is_valid, msg.latitude, msg.longitude])

if __name__ == '__main__':
    try:
        init_serial_node()
    except rospy.ROSInterruptException:
        pass