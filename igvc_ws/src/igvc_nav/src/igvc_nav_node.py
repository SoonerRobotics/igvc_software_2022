#!/usr/bin/env python3
from tkinter import DISABLED
import rospy
import math
import tf
from geometry_msgs.msg import Pose, Point
from pure_pursuit import PurePursuit
from nav_msgs.msg import Path, Odometry
from igvc_msgs.msg import motors, EKFState
from utilities.pp_viwer import setup_pyplot, draw_pp
from enum import Enum
from std_msgs.msg import Int16
import sys

class SystemState(Enum):
    DISABLED = 0
    MANUAL = 1
    AUTONOMOUS = 2

system_state = SystemState.DISABLED

sys.stdout.reconfigure(line_buffering=True)


SHOW_PLOTS = False
USE_SIM_TRUTH = False

pos = None
heading = None
ekf = None
odom = None
publy = rospy.Publisher('/igvc/motors_raw', motors, queue_size=1)

pp = PurePursuit()

def system_state_callback(data):
    global system_state
    system_state = SystemState(data.data)

def ekf_update(ekf_state):
    global pos, heading, ekf

    ekf = ekf_state

    pos = (ekf_state.x, ekf_state.y)
    heading = ekf_state.yaw

def true_pose_callback(data):
    global pos, heading

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
    
    # if pitch > 0 and yaw > 0:
    #     yaw = math.pi - yaw
    # if pitch > 0 and yaw < 0:
    #     yaw = -math.pi - yaw

    pos = (data.position.x, data.position.y)
    heading = math.degrees(roll)

    if heading < 0:
        heading += 360

def global_path_update(data):
    points = [Point(0,0), Point(1,0), Point(1,-1), Point(0,-1), Point(0,0)] # Get points from Path
    pp.set_points([(_point.x, _point.y) for _point in points]) # Give PurePursuit the points

def get_angle_diff(angle1, angle2):
    delta = angle1 - angle2

    # Account for cases when angles are very close or far
    if delta > math.pi:
        delta -= 2 * math.pi
    elif delta < 0:
        delta += 2 * math.pi
    
    return delta

def clamp(val, min, max):
    if val < min:
        return min
    if val > max:
        return max
    return val

def timer_callback(event):
    if pos is None or heading is None:
        return

    cur_pos = (pos[0], pos[1])

    lookahead = None
    radius = 0.3 # Starting radius

    while lookahead is None and radius <= 3: # Look until we hit 3 meters max
        lookahead = pp.get_lookahead_point(cur_pos[0], cur_pos[1], radius)
        radius *= 1.2

    if SHOW_PLOTS:
        draw_pp(cur_pos, lookahead, pp.path)

    motor_pkt = motors()
    motor_pkt.left = 0
    motor_pkt.right = 0

    if lookahead is not None and ((lookahead[1] - cur_pos[1]) ** 2 + (lookahead[0] - cur_pos[0]) ** 2) > 0.001:
        # Get heading to to lookahead from current position
        heading_to_lookahead = math.atan2(lookahead[1] - cur_pos[1], lookahead[0] - cur_pos[0])

        # print(f"h_2_l: {heading_to_lookahead * 180 / (math.pi)}")

        # Get difference in our heading vs heading to lookahead
        # Normalize error to -1 to 1 scale
        error = get_angle_diff(heading_to_lookahead, heading) / math.pi

        # print(f"am at {cur_pos[0]:0.02f},{cur_pos[1]:0.02f}, want to go to {lookahead[0]:0.02f},{lookahead[1]:0.02f}")
        # print(f"angle delta: {error * 180:0.01f}")

        # print(f"error is {error}")
        # if abs(error) < 2.0:
        #     error = 0

        # Base forward velocity for both wheels
        forward_speed = 0.35 * (1 - abs(error))**5

        # Define wheel linear velocities
        # Add proprtional error for turning.
        # TODO: PID instead of just P
        motor_pkt.left = (forward_speed - clamp(1.0 * error, -0.25, 0.25))
        motor_pkt.right = (forward_speed + clamp(1.0 * error, -0.25, 0.25))

    else:
        # We couldn't find a suitable direction to head, stop the robot.
        motor_pkt.left = -0.25
        motor_pkt.right = -0.25

    if system_state == SystemState.AUTONOMOUS:
        publy.publish(motor_pkt)
    elif system_state == SystemState.DISABLED:
        # Nav node will be in charge of stopping the robot during DISABLE
        motor_pkt.left = 0
        motor_pkt.right = 0
        publy.publish(motor_pkt)


def nav():
    rospy.init_node('nav_node', anonymous=True)

    points = [Point(0,0,0), Point(1,0,0), Point(1,-1,0), Point(2,-1,0), Point(2,-2,0), Point(0,-2,0), Point(0,-3,0)] # Get points from Path
    pp.set_points([(_point.x, _point.y) for _point in points]) # Give PurePursuit the points

    if USE_SIM_TRUTH:
        rospy.Subscriber("/sim/true_pose", Pose, true_pose_callback)
    else:
        rospy.Subscriber("/igvc/state", EKFState, ekf_update)

    rospy.Subscriber("/igvc/global_path", Path, global_path_update)
    rospy.Subscriber("/igvc/system_state", Int16, system_state_callback)

    rospy.Timer(rospy.Duration(0.04), timer_callback)

    if SHOW_PLOTS:
        setup_pyplot()

    rospy.spin()

if __name__ == '__main__':
    try:
        nav()
    except rospy.ROSInterruptException:
        pass

