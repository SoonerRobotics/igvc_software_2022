#!/usr/bin/env python3

import rospy
import math
from enum import Enum
from std_msgs.msg import String, Header, Bool, Int16
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Transform, TransformStamped, Vector3
from nav_msgs.msg import OccupancyGrid, Path, MapMetaData
from igvc_msgs.msg import motors, EKFState
from igvc_msgs.srv import EKFService
import tf
import copy
import time
import numpy as np
from path_planner.mt_dstar_lite import mt_dstar_lite
from utilities.dstar_viewer import draw_dstar, setup_pyplot
from heapq import heappush, heappop
from matplotlib import pyplot as plt
import sys

class SystemState(Enum):
    DISABLED = 0
    MANUAL = 1
    AUTONOMOUS = 2

system_state = SystemState.DISABLED


SHOW_PLOTS = False
USE_SIM_TRUTH = False

# Camera Vertical vision (m)
camera_vertical_distance = 2.75
# Camera Horizontal vision (m)
camera_horizontal_distance = 3

global_path_pub = rospy.Publisher("/igvc/global_path", Path, queue_size=1)
local_path_pub = rospy.Publisher("/igvc/local_path", Path, queue_size=1)
mobi_start_publisher = rospy.Publisher("/igvc/mobstart", Bool, queue_size=1)

# Moving Target D* Lite
map_init = False
path_failed = False
planner = mt_dstar_lite()

# Location when map was made
map_reference = (0, 0, 0)

# Localization tracking
prev_state = (0, 0)  # x, y
GRID_SIZE = 0.1      # Map block size in meters

# Path tracking
path_seq = 0

# Cost map
cost_map = None

# Latest EKF update
curEKF = EKFState()

# Practice Waypoints (North-bound)
#orig_waypoints = [(42.66821182,-83.21845873),(42.66808596,-83.21844564),(42.66796006,-83.21843266)]

# Practice Waypoints (South-bound)
#orig_waypoints = [(42.66796006,-83.21843266),(42.66808596,-83.21844564),(42.66821182,-83.21845873)]

# Real Waypoints (North-bound)
#orig_waypoints = [(42.66826972,-83.21934030),(42.66812064,-83.21936061),(42.66792771,-83.21932764)]

# Real Waypoints (South-bound)
orig_waypoints = [(42.66792771,-83.21932764),(42.66807663,-83.21935916),(42.66826972,-83.21934030)]

# Real Waypoints (North-bound with extra)
# orig_waypoints = [(42.66826972, -83.2193403), (42.66819518, -83.21934046), (42.66812064, -83.21936061), (42.66807663, -83.21935916), (42.66800217, -83.2193534), (42.66792771, -83.21932764)]        

# Real Waypoints (South-bound with extra)
# orig_waypoints = [(42.66792771,-83.21932764),(42.66800217,-83.21935340),(42.66807663,-83.21935916),(42.66812064,-83.21936061),(42.66819518,-83.21934046),(42.66826972,-83.21934030)]

#waypoints = [(42.6681017,-83.2184545)]
#waypoints = [(42.6683345, -83.2182354)]

waypoints = []
first_waypoints_time = -1

# Best position to head to on the map (D* goal pos)
best_pos = (0,0)

def system_state_callback(data):
    global system_state, first_waypoints_time, waypoints
    system_state = SystemState(data.data)

    if system_state != SystemState.AUTONOMOUS:
        waypoints = []
        first_waypoints_time = -1

    if system_state == SystemState.AUTONOMOUS and first_waypoints_time == -1:
        first_waypoints_time = time.time() + 40
        # waypoints = [pt for pt in orig_waypoints]
        # print(f"Yaw is {curEKF.yaw/math.pi:0.1f}pi")
        #if math.pi/2 < curEKF.yaw < 3 * math.pi / 2:
            # Facing south, reverse waypoints
            # print("Running South!!")
            #waypoints = waypoints[::-1]
        # else:
            # print("Running North!!")
        
        # sys.stdout.flush()

def ekf_callback(data):
    global curEKF
    curEKF = data

def true_pose_callback(data):
    global curEKF
    curEKF = EKFState()
    curEKF.x = data.position.x
    curEKF.y = data.position.y
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
    curEKF.yaw = roll

def get_angle_diff(to_angle, from_angle):
    delta = to_angle - from_angle
    delta = (delta + math.pi) % (2 * math.pi) - math.pi
    
    return delta

def c_space_callback(c_space):
    global cost_map, map_reference, map_init, best_pos, waypoints, first_waypoints_time

    if curEKF is None:
        return

    grid_data = c_space.data

    # Make a costmap
    temp_cost_map = [0] * 80 * 80

    # Find the best position
    temp_best_pos = (40, 78)
    best_pos_cost = -1000

    # Breath-first look for good points
    # This allows us to find a point within the range of obstacles by not
    # exploring over obstacles. 
    frontier = set()
    frontier.add((40,78))
    explored = set()

    if first_waypoints_time > 0 and time.time() > first_waypoints_time:
        first_waypoints_time = -2
        waypoints = [pt for pt in orig_waypoints]

    if len(waypoints) > 0:
        next_waypoint = waypoints[0]
        north_to_gps = (next_waypoint[0] - curEKF.latitude) * 111086.2
        west_to_gps = (curEKF.longitude - next_waypoint[1]) * 81978.2
        heading_to_gps = math.atan2(west_to_gps,north_to_gps) % (2 * math.pi)

        # print(f"heading_to_gps: {heading_to_gps*180/math.pi:0.01f}")

        if north_to_gps**2 + west_to_gps**2 <= 1:
            # mobi_start_publisher.publish(Bool(False))
            waypoints.pop(0)

    # sys.stdout.flush()
    # best_heading_err = 0

    depth = 0
    while depth < 50 and len(frontier) > 0:
        curfrontier = copy.copy(frontier)
        for pos in curfrontier:
            x = pos[0] # left to right
            y = pos[1] # top to botom
            # Cost at a point is sum of
            # - Negative X value (encourage forward)
            # - Positive Y value (discourage left/right)
            # - Heading
            cost = (80 - y) * 1.3 + depth * 2.2

            if len(waypoints) > 0:
                heading_err_to_gps = abs(get_angle_diff(curEKF.yaw + math.atan2(40-x,80-y), heading_to_gps)) * 180 / math.pi
                cost -= max(heading_err_to_gps, 10)

            if cost > best_pos_cost:
                best_pos_cost = cost
                temp_best_pos = pos
                # best_heading_err = heading_err_to_gps

            frontier.remove(pos)
            explored.add(x + 80 * y)

            # Look left/right for good points
            if y > 1 and grid_data[x + 80 * (y-1)] < 50 and x + 80 * (y-1) not in explored:
                frontier.add((x, y-1))

            # Look forward/back for good points
            if x < 79 and grid_data[x + 1 + 80 * y] < 50 and x + 1 + 80 * y not in explored:
                frontier.add((x+1, y))
            if x > 0 and grid_data[x - 1 + 80 * y] < 50 and x - 1 + 80 * y not in explored:
                frontier.add((x-1, y))

        depth += 1

    # print(f"best_heading_err: {best_heading_err:0.01f}")

    # map_reference = (curEKF.x, curEKF.y, curEKF.yaw)
    map_reference = (0,0,0)
    cost_map = grid_data
    best_pos = temp_best_pos
    map_init = False

def path_point_to_global_pose_stamped(robot_pos, pp0, pp1, header):
    # Local path
    x = (80 - pp1) * camera_vertical_distance / 80
    y = (40 - pp0) * camera_horizontal_distance / 80

    # Translate to global path
    dx = map_reference[0]
    dy = map_reference[1]
    psi = map_reference[2]

    new_x = x * math.cos(psi) + y * math.sin(psi) + dx
    new_y = x * math.sin(psi) + y * math.cos(psi) + dy

    pose_stamped = PoseStamped(header=header)
    pose_stamped.pose = Pose()

    point = Point()
    point.x = new_x
    point.y = new_y
    pose_stamped.pose.position = point

    return pose_stamped

def path_point_to_local_pose_stamped(pp0, pp1, header):
    pose_stamped = PoseStamped(header=header)
    pose_stamped.pose = Pose()

    point = Point()
    point.x = (80 - pp1) * camera_vertical_distance / 80
    point.y = (40 - pp0) * camera_horizontal_distance / 80
    pose_stamped.pose.position = point

    return pose_stamped
        
def reconstruct_path(path, current):
    total_path = [current]

    while current in path:
        current = path[current]
        total_path.append(current)

    return total_path[::-1]

def find_path_to_point(start, goal, map, width, height):

    looked_at = np.zeros((80, 80))
    

    open_set = [start]

    path = {}

    search_dirs = []

    for x in range(-1, 2):
        for y in range(-1, 2):
            if x == 0 and y == 0:
                continue
            search_dirs.append((x,y,math.sqrt(x**2+y**2)))

    # print(search_dirs)

    def h(point):
        return math.sqrt((goal[0] - point[0])**2 + (goal[1] - point[1])**2)

    # assumes adjacent pts
    def d(to_pt, dist):
        return dist + map[to_pt[1] * width + to_pt[0]] / 10

    gScore = {}
    gScore[start] = 0

    def getG(pt):
        if pt in gScore:
            return gScore[pt]
        else:
            gScore[pt] = 1000000000
            return 1000000000 # Infinity

    fScore = {}
    fScore[start] = h(start)

    def getF(pt):
        if pt in fScore:
            return fScore[pt]
        else:
            fScore[pt] = 1000000000
            return 1000000000 # Infinity

    next_current = [(1,start)]
    while len(open_set) != 0:
        current = heappop(next_current)[1]

        looked_at[current[0],current[1]] = 1

        if current == goal:
            # plt.figure(1)
            # plt.clf()
            
            # plt.imshow(looked_at, interpolation = 'nearest')

            # plt.draw()
            # plt.pause(0.00000000001)
            return reconstruct_path(path, current)

        open_set.remove(current)
        for delta_x, delta_y, dist in search_dirs:

            neighbor = (current[0] + delta_x, current[1] + delta_y)
            if neighbor[0] < 0 or neighbor[0] >= width or neighbor[1] < 0 or neighbor[1] >= height:
                continue

            tentGScore = getG(current) + d(neighbor, dist)
            if tentGScore < getG(neighbor):
                path[neighbor] = current
                gScore[neighbor] = tentGScore
                fScore[neighbor] = tentGScore + h(neighbor)
                if neighbor not in open_set:
                    open_set.append(neighbor)
                    heappush(next_current, (fScore[neighbor], neighbor))
                    
def make_map(c_space):
    global planner, map_init, path_failed, prev_state, path_seq

    if cost_map is None or curEKF is None:
        return

    # start_time = time.time()

    # Reset the path
    path = None

    robot_pos = (40, 78)

    # MOVING TARGET D*LITE
    # If this is the first time receiving a map, or if the path failed to be made last time (for robustness),
    # initialize the path planner and plan the first path

    # TODO: Make this not True again lol
    if True:
        # start_time = time.time()
        path = find_path_to_point(robot_pos, best_pos, cost_map, 80, 80)
        # print(path)
        # sys.stdout.flush()
        # print(f"plan time: {(time.time() - start_time) * 1000:02.02f}ms")
        map_init = True

    if path is not None:
        header = Header()
        header.seq = path_seq
        header.stamp = rospy.Time.now()
        header.frame_id = "base_link"

        path_seq += 1

        global_path = Path(header = header)
        global_path.poses = [path_point_to_global_pose_stamped(robot_pos, path_point[0], path_point[1], header) for path_point in path]
        # global_path.poses.reverse() # reverse path becuz its backwards lol

        local_path = Path(header = header)
        local_path.poses = [path_point_to_local_pose_stamped(path_point[0], path_point[1], header) for path_point in path]
        # local_path.poses.reverse() # reverse path becuz its backwards lol

        global_path_pub.publish(global_path)
        local_path_pub.publish(local_path)

    else:
        # Set the path failed flag so we can fully replan
        path_failed = True

        # path_msg = copy.deepcopy(c_space)
        # data = planner.get_search_space_map()
        # data[(best_pos[0]*200) + best_pos[1]] = 100
        # print(str(c_space.info.width) + " x " + str(c_space.info.height))
        # path_msg.data = data
        # path_pub.publish(path_msg)

    if SHOW_PLOTS:
        draw_dstar(robot_pos, best_pos, cost_map, path, fig_num=2)

    # print(f"plan time: {(time.time() - start_time) * 1000:02.02f}ms")

def mt_dstar_node():
    # Setup node
    rospy.init_node("mt_dstar_node")

    # Subscribe to necessary topics
    rospy.Subscriber("/igvc_slam/local_config_space", OccupancyGrid, c_space_callback, queue_size=1)  # Mapping
    if USE_SIM_TRUTH:
        rospy.Subscriber("/sim/true_pose", Pose, true_pose_callback)
    else:
        rospy.Subscriber("/igvc/state", EKFState, ekf_callback)

    rospy.Subscriber("/igvc/system_state", Int16, system_state_callback)

    # Make a timer to publish new paths
    timer = rospy.Timer(rospy.Duration(secs=0.1), make_map, oneshot=False)

    # if SHOW_PLOTS:
    setup_pyplot()

    # Wait for topic updates
    rospy.spin()



# Main setup
if __name__ == '__main__':
    try:
        mt_dstar_node()
    except rospy.ROSInterruptException:
        pass
