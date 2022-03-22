#!/usr/bin/env python3

## Lane finder reads in the image topic and pushes out to the lane_deviation topic the current deviation from lane center.

import rospy
import json
from igvc_msgs.msg import motors
from sensor_msgs.msg import Joy


manual_control_pub = rospy.Publisher("/igvc/motors_raw", motors, queue_size=10)

max_speed = 0.8

def clamp(n, smallest, largest): return max(smallest, min(n, largest))

def manual_control_callback(data):
    #http://wiki.ros.org/joy#Microsoft_Xbox_360_Wireless_Controller_for_Linux

    axes = data.axes

    drivetrain_msg = motors()

    # # RT
    # if abs(axes[5]) < 0.1:
    #     throttle = 0
    # else:
    #     throttle = (1 - axes[5]) * max_speed

    # # LT
    # if abs(axes[2]) > 0.1:
    #     throttle = -(1 - axes[2]) * max_speed

    # if abs(axes[0]) > 0.15:
    #     turning = axes[0] * max_speed
    # else:
    #     turning = 0
    
    # drivetrain_msg.left = clamp(throttle - turning * 0.6, -max_speed, max_speed)
    # drivetrain_msg.right = clamp(throttle + turning * 0.6, -max_speed, max_speed)

    # # Corrections
    # drivetrain_msg.right = -drivetrain_msg.right
    
    if abs(axes[1]) < 0.1:
        drivetrain_msg.left = 0
    else:
        drivetrain_msg.left = axes[1] * 1.4
        
    if abs(axes[4]) < 0.1:
        drivetrain_msg.right = 0
    else:
        drivetrain_msg.right = axes[4] * 1.4

    # rospy.loginfo("manual_control callback.")
    manual_control_pub.publish(drivetrain_msg)

def manual_node():
    rospy.init_node('manual_node', anonymous=True)

    rospy.Subscriber("/joy", Joy, manual_control_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        manual_node()
    except rospy.ROSInterruptException:
        pass