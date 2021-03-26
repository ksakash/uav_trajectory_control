#!/usr/bin/env python3

import math
import time
import sys

import rospy

from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, String

from controls_pkg.msg import *
from controls_pkg.srv import *

from uav_trajectory_control.msg import *
from uav_trajectory_control.srv import *

rospy.init_node ('offboard_control_ardupilot')
rate = rospy.Rate (20)
local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10, latch=True)

current_state = State ()
curr_pose = PoseStamped ()

start_x = rospy.get_param ("x", 0)
start_y = rospy.get_param ("y", 0)
height = 10 # for the waypoints not for the takeoff

def state_cb (data):
    global current_state
    current_state = data

def pose_cb (data):
    global curr_pose
    curr_pose = data

def process_input (filename):
    global start_x, start_y, height
    f = open (filename, 'r')
    lines = f.readlines()
    f.close ()
    plan = []
    unit_length = 5

    for line in lines:
        (x, y, z) = (int (line.split(' ')[0]), int (line.split(' ')[1]), height) # int (line.split(' ')[2]))
        wp = Waypoint ()
        wp.point.x = (x - start_x) * unit_length
        wp.point.y = (y - start_y) * unit_length
        wp.point.z = z
        wp.max_forward_speed = 0.5

        plan.append (wp)

    return plan

state_sub = rospy.Subscriber("mavros/state", State, state_cb)
pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, pose_cb)

id = rospy.get_param ("id", 0)
uav_name = "UAV" + str (id)

print (uav_name, ": INTIALIZING UAV ...")
time.sleep (1)

while(not rospy.is_shutdown () and not current_state.connected):
    print (uav_name, ": waiting for the state to be connected")
    rate.sleep()

rospy.wait_for_service ('mavros/set_mode')
print (uav_name, ": mavros/set_mode service is active...")

set_mode_client = rospy.ServiceProxy ("mavros/set_mode", SetMode)
mode_res = set_mode_client (custom_mode="GUIDED")

if mode_res.mode_sent:
    print (uav_name, ": guided mode sent", mode_res.mode_sent)
else:
    print (uav_name, ": failed guided mode!")
    sys.exit (-1)

while (current_state.mode != 'GUIDED' and not rospy.is_shutdown ()):
    print (uav_name, ": current state mode:", current_state.mode)
    time.sleep (0.01)

rospy.wait_for_service ('mavros/cmd/arming')
print (uav_name, ": mavros/cmd/arming service is active...")

arming_client = rospy.ServiceProxy ("mavros/cmd/arming", CommandBool)

arm_res = arming_client (value=True)
if arm_res.success:
    print (uav_name, ": ARM sent", arm_res.success)
else:
    print (uav_name, ": Failed Arming!")
    sys.exit (-1)

takeoff_client = rospy.ServiceProxy ("mavros/cmd/takeoff", CommandTOL)
takeoff_res = takeoff_client (altitude=3)

if takeoff_res.success:
    print (uav_name, ": takeoff sent", takeoff_res.success)
else:
    print (uav_name, ": failed takeoff!")

time.sleep (5)

print (uav_name, ": reading input..")
file = '9x9'
filename = "/home/ksakash/projects/control_ws/src/uav_trajectory_control/cfg/" + file + "_" + str (id)
waypoints = process_input (filename)
print ("length of plan:", len (waypoints))
interpolator = String ()
interpolator.data = "linear"

rospy.wait_for_service ('start_waypoint_list')

print (uav_name, ": starting the mission..")

waypoint_set_client = rospy.ServiceProxy ('start_waypoint_list', InitWaypointSet)
waypoint_set_res = waypoint_set_client (start_now=True, waypoints=waypoints, \
                                        max_forward_speed=0.5, interpolator=interpolator)
if waypoint_set_res.success:
    print (uav_name, ": waypoint list sent")
else:
    print (uav_name, ": error sending the waypoint list")
    sys.exit (-1)

rospy.wait_for_service ('trajectory_complete')

trajectory_complete_client = rospy.ServiceProxy ('trajectory_complete', TrajectoryComplete)

while not rospy.is_shutdown ():
    res = trajectory_complete_client (time_out=0)
    if res.success:
        break
    rate.sleep ()

print (uav_name, ": completed the mission!!")

rtl = True

if rtl:
    print (uav_name, ": RTL mode ...")
    mode_res = set_mode_client (custom_mode="RTL")

    if mode_res.mode_sent:
        print (uav_name, ": RTL mode sent", mode_res.mode_sent)
    else:
        print (uav_name, ": failed RTL mode!")
        sys.exit (-1)
else:
    land_client = rospy.ServiceProxy ("mavros/cmd/land", CommandTOL)
    land_res = land_client ()

    if land_res.success:
        print (uav_name, ": land sent", land_res.success)
    else:
        print (uav_name, ": landing failed!")
        sys.exit (-1)

