#!/usr/bin/env python3

from turtle import left, right
import rospy
import json
import threading
import can
import struct
import datetime
import sys
from enum import Enum

from std_msgs.msg import String, Bool, Int16
from igvc_msgs.msg import motors, velocity, gps, deltaodom

# ROS node that facilitates all serial communications within the robot
# Subscribes to motor values
# Publishes GPS coordinates

sys.stdout.reconfigure(line_buffering=True)

serials = {}
cans = {}

mob_publisher = rospy.Publisher("/igvc/mobstart", Bool, queue_size=1)

MAX_SPEED = 2.2 # m/s
CAN_ID_ESTOP = 0
CAN_ID_MOBSTOP = 1
CAN_ID_MOBSTART = 9
CAN_ID_SEND_VELOCITY = 10
CAN_ID_RECV_VELOCITY = 11
CAN_ID_SAFETY_LIGHTS_COMMAND = 13
CAN_ID_ODOM_FEEDBACK = 14

class SystemState(Enum):
    DISABLED = 0
    MANUAL = 1
    AUTONOMOUS = 2

system_state = SystemState.DISABLED

class VelocityCANReadThread(threading.Thread):
    def __init__(self, can_obj):
        threading.Thread.__init__(self)

        self.can_obj = can_obj
        
        # self.f = open("/home/zemlin/igvc_software_2021/igvc_ws/encoder_out.csv", "w")
        
        # self.csvwriter = csv.writer(self.f)

        # Allow timeout of up to 1 second on reads. This could be set to None to have infinite timeout,
        # but that would hault the node when it tries to exit. Need to make sure the while loop condition is
        # semi-regularly checked. This is better than rospy.Rate because it will continously wait for new message
        # instead of only checking on a fixed interval.
        # self.can_obj.timeout = 1

        # Assumes String type for now. This class will need to be adapted in the future for different message types.
        self.vel_publisher = rospy.Publisher("/igvc/velocity", velocity, queue_size=1)
        self.odom_publisher = rospy.Publisher("/igvc/deltaodom", deltaodom, queue_size=1)

    def run(self):
        while not rospy.is_shutdown():
            msg = self.can_obj.recv(timeout=1)
            
            if msg is None:
                print("Received None CAN msg")
                continue

            # if msg.arbitration_id != CAN_ID_RECV_VELOCITY and msg.arbitration_id != CAN_ID_ODOM_FEEDBACK:
            #     print(f"Got CAN: {msg.arbitration_id}")

            if msg.arbitration_id == CAN_ID_RECV_VELOCITY:
                left_speed, right_speed, max_speed = struct.unpack("bbB", msg.data)
                
                velPkt = velocity()
                velPkt.leftVel = -right_speed / 127 * max_speed / 10
                velPkt.rightVel = -left_speed / 127 * max_speed / 10
                
                # print(f"vel {velPkt.leftVel}, {velPkt.rightVel}")
                
                # self.csvwriter.writerow([rospy.Time.now(), velPkt.leftVel, velPkt.rightVel])

                self.vel_publisher.publish(velPkt)
            
            if msg.arbitration_id == CAN_ID_ODOM_FEEDBACK:
                delta_theta, delta_y, delta_x = struct.unpack("hhh", msg.data)
                
                # velPkt = velocity()
                # velPkt.leftVel = -right_speed / 127 * max_speed / 10
                # velPkt.rightVel = -left_speed / 127 * max_speed / 10

                odom_pkt = deltaodom()
                odom_pkt.delta_x = -delta_x * 0.00002
                odom_pkt.delta_y = delta_y * 0.00002
                odom_pkt.delta_theta = 2 * delta_theta * 0.00002
                
                # print(f"odom: ({delta_x:0.03f},{delta_y:0.03f},{delta_theta:0.03f})")
                
                # self.csvwriter.writerow([rospy.Time.now(), velPkt.leftVel, velPkt.rightVel])

                self.odom_publisher.publish(odom_pkt)
            
            if msg.arbitration_id == CAN_ID_ESTOP or msg.arbitration_id == CAN_ID_MOBSTOP:
                # Stop blinking
                print(f"Received Stop {msg.arbitration_id}")
                # serials["gps"].write(b'n')
                mob_publisher.publish(Bool(False))

            if msg.arbitration_id == CAN_ID_MOBSTART:
                # Start blinking
                print(f"Received Start {msg.arbitration_id}")
                # serials["gps"].write(b'b')
                mob_publisher.publish(Bool(True))
                
        # self.f.close()


class GPSSerialReadThread(threading.Thread):
    def __init__(self, serial_obj, topic):
        threading.Thread.__init__(self)

        self.serial_obj = serial_obj

        # Allow timeout of up to 1 second on reads. This could be set to None to have infinite timeout,
        # but that would hault the node when it tries to exit. Need to make sure the while loop condition is
        # semi-regularly checked. This is better than rospy.Rate because it will continously wait for new message
        # instead of only checking on a fixed interval.
        self.serial_obj.timeout = 1

        # Assumes String type for now. This class will need to be adapted in the future for different message types.
        self.publisher = rospy.Publisher(topic, gps, queue_size=1)

    def run(self):
        while not rospy.is_shutdown():
            coord = self.serial_obj.readline().decode()[:-1] # Assume all messages end in newline character. This is standard among SCR IGVC serial messages.

            try:
                if coord:
                    coord_json = json.loads(coord)

                    coord_msg = gps()
                    coord_msg.hasSignal = coord_json['hasSignal']

                    if coord_msg.hasSignal:
                        coord_msg.latitude = coord_json['latitude']
                        coord_msg.longitude = coord_json['longitude']

                    self.publisher.publish(coord_msg)
            except ValueError:
                pass

def clamp(val, min, max):
    if val < min:
        return min
    if val > max:
        return max
    return val

# Constructs motor message from given data and sends to serial
def motors_out(data):

    # Soon to be firmware corrections
    

    left_speed = clamp(int(-data.right / MAX_SPEED * 127), -128, 127)
    right_speed = clamp(int(-data.left / MAX_SPEED * 127), -128, 127)

    packed_data = struct.pack('bbB', left_speed, right_speed, int(MAX_SPEED * 10))
    
    # print(f"sending {left_speed}, {right_speed}, {int(MAX_SPEED * 10)}")

    can_msg = can.Message(arbitration_id=CAN_ID_SEND_VELOCITY, data=packed_data)

    try:
        cans["motor"].send(can_msg)
    except:
        print("Could not send CAN message")
    # else:
    #     print("CAN message sent!")

    # print(f"Sent {data.left} {data.right}")

def mobstart_callback(data: Bool):
    can_msg = can.Message(arbitration_id=CAN_ID_MOBSTOP)

    print(f"Sending mobstart with value {data.data}")

    # Mobility start/stop
    if data.data == True:
        can_msg = can.Message(arbitration_id=CAN_ID_MOBSTART)

    try:
        cans["motor"].send(can_msg)
    except:
        print("Could not send CAN message")

    # Safety Lights

    packed_data = struct.pack('bb', 2 if (data.data and system_state == SystemState.AUTONOMOUS) else 1, 0)
    
    # print(f"sending {left_speed}, {right_speed}, {int(MAX_SPEED * 10)}")

    can_msg = can.Message(arbitration_id=CAN_ID_SAFETY_LIGHTS_COMMAND, data=packed_data)

    try:
        cans["motor"].send(can_msg)
    except:
        print("Could not send CAN message")

def system_state_callback(data):
    global system_state
    system_state = SystemState(data.data)

# Initialize the serial node
# Node handles all serial communication within the robot (motor, GPS)
def init_serial_node():
    
    # Setup serial node
    rospy.init_node("serial_node", anonymous = False)

    print("Serial opened!")

    # Setup motor serial and subscriber
    # motor_serial = serials["motor"] = serial.Serial(port = '/dev/igvc-nucleo-120', baudrate = 115200)
    motor_can = cans["motor"] = can.ThreadSafeBus(bustype='slcan', channel='/dev/igvc-can-835', bitrate=100000)
    rospy.Subscriber('/igvc/motors_raw', motors, motors_out)
    
    motor_response_thread = VelocityCANReadThread(can_obj = cans["motor"])
    motor_response_thread.start()

    rospy.Subscriber("/igvc/mobstart", Bool, mobstart_callback)
    rospy.Subscriber("/igvc/system_state", Int16, system_state_callback)

    # Setup GPS serial and publisher
    # gps_serial = serials["gps"] = serial.Serial(port = '/dev/igvc-nucleo-722', baudrate = 9600)

    # gps_response_thread = GPSSerialReadThread(serial_obj = serials["gps"], topic = '/igvc/gps')
    # gps_response_thread.start()
    
    # Wait for topic updates
    rospy.spin()

    # Close the serial ports when program ends
    print("Closing threads")
    motor_can.close()
    # gps_serial.close()

if __name__ == '__main__':
    try:
        init_serial_node()
    except rospy.ROSInterruptException:
        pass
