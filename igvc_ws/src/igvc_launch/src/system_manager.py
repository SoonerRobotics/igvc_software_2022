#!/usr/bin/env python3

import queue
import rospy
from enum import Enum
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16, Bool

class SystemState(Enum):
    DISABLED = 0
    MANUAL = 1
    AUTONOMOUS = 2

class SystemManager:
    def __init__(self):
        self.current_state = SystemState.DISABLED
        self.previous_joy = None

        self.state_publisher = rospy.Publisher("/igvc/system_state", Int16, queue_size=10)
        self.mobi_start_publisher = rospy.Publisher("/igvc/mobstart", Bool, queue_size=1)

        rospy.Subscriber("/joy", Joy, self.joy_callback)

    def publish_state(self):
        self.state_publisher.publish(Int16(self.current_state.value))

        # Mobility stop on any state transition
        self.mobi_start_publisher.publish(Bool(False))

    def joy_callback(self, data):
        if self.previous_joy == None:
            self.previous_joy = data
            return

        # Start button = Prev State
        if self.previous_joy.buttons[7] == 0 and data.buttons[7] == 1:
            self.next_state()
        
        # Back button = Next State
        if self.previous_joy.buttons[6] == 0 and data.buttons[6] == 1:
            self.previous_state()

        # A button = Mobi-start in AUTONOMOUS
        if self.previous_joy.buttons[0] == 0 and data.buttons[0] == 1:
            if self.current_state != SystemState.DISABLED:
                # Send Mobi Start
                self.mobi_start_publisher.publish(Bool(True))

        # B button = Kill ROS
        if self.previous_joy.buttons[1] == 0 and data.buttons[1] == 1:
            if self.current_state == SystemState.DISABLED:
                # Kill ROS by killing this node
                rospy.signal_shutdown(reason="System Manager Shutdown")

        self.previous_joy = data

    def next_state(self):
        if self.current_state == SystemState.DISABLED:
            self.current_state = SystemState.MANUAL
        elif self.current_state == SystemState.MANUAL:
            self.current_state = SystemState.AUTONOMOUS
        elif self.current_state == SystemState.AUTONOMOUS:
            self.current_state = SystemState.DISABLED

        self.publish_state()

    def previous_state(self):
        if self.current_state == SystemState.MANUAL:
            self.current_state = SystemState.DISABLED
        elif self.current_state == SystemState.AUTONOMOUS:
            self.current_state = SystemState.MANUAL

        self.publish_state()


def system_manager():
    rospy.init_node('system_manager', anonymous=True)

    SystemManager()

    rospy.spin()



if __name__ == '__main__':
    try:
        system_manager()
    except rospy.ROSInterruptException:
        pass

