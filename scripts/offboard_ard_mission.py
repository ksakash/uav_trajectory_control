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

def state_cb (data):
    global current_state
    current_state = data

def pose_cb (data):
    global curr_pose
    curr_pose = data

def process_input (filename):
    f = open (filename, 'r')
    lines = f.readlines()
    f.close ()
    plan = []

    for line in lines:
        (x, y, z) = (int (line.split(' ')[0]), int (line.split(' ')[1]), int (line.split(' ')[2]))
        wp = Waypoint ()
        wp.point.x = x
        wp.point.y = y
        wp.point.z = z
        wp.max_forward_speed = 1

        plan.append (wp)

    return plan

state_sub = rospy.Subscriber("mavros/state", State, state_cb)
pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, pose_cb)

print ("INTIALIZING...")
time.sleep (1)

while(not rospy.is_shutdown () and not current_state.connected):
    print ("waiting for the state to be connected")
    rate.sleep()

rospy.wait_for_service ('mavros/set_mode')
print ("mavros/set_mode service is active...")

set_mode_client = rospy.ServiceProxy ("mavros/set_mode", SetMode)
mode_res = set_mode_client (custom_mode="GUIDED")

if mode_res.mode_sent:
    print ("guided mode sent", mode_res.mode_sent)
else:
    print ("failed guided mode!")
    sys.exit (-1)

while (current_state.mode != 'GUIDED' and not rospy.is_shutdown ()):
    print ("current state mode:", current_state.mode)
    time.sleep (0.01)

rospy.wait_for_service ('mavros/cmd/arming')
print ("mavros/cmd/arming service is active...")

arming_client = rospy.ServiceProxy ("mavros/cmd/arming", CommandBool)

arm_res = arming_client (value=True)
if arm_res.success:
    print ("ARM sent", arm_res.success)
else:
    print ("Failed Arming!")
    sys.exit (-1)

takeoff_client = rospy.ServiceProxy ("mavros/cmd/takeoff", CommandTOL)
takeoff_res = takeoff_client (altitude=3)

if takeoff_res.success:
    print ("takeoff sent", takeoff_res.success)
else:
    print ("failed takeoff!")

time.sleep (20)

print ("reading input..")
filename = "/home/ksakash/projects/control_ws/src/uav_trajectory_control/scripts/waypoints_2"
waypoints = process_input (filename)
interpolator = String ()
interpolator.data = "cubic"

rospy.wait_for_service ('/start_waypoint_list')

print ("starting the mission..")

waypoint_set_client = rospy.ServiceProxy ('/start_waypoint_list', InitWaypointSet)
waypoint_set_res = waypoint_set_client (start_now=True, waypoints=waypoints, \
                                        max_forward_speed=0.5, interpolator=interpolator)
if waypoint_set_res.success:
    print ("waypoint list sent")
else:
    print ("error sending the waypoint list")
    sys.exit (-1)

while not rospy.is_shutdown ():
    rate.sleep ()

print ("completed the mission!!")

rtl = False

if rtl:
    print ("RTL mode ...")
    mode_res = set_mode_client (custom_mode="RTL")

    if mode_res.mode_sent:
        print ("RTL mode sent", mode_res.mode_sent)
    else:
        print ("failed RTL mode!")
        sys.exit (-1)
else:
    land_client = rospy.ServiceProxy ("mavros/cmd/land", CommandTOL)
    land_res = land_client ()

    if land_res.success:
        print ("land sent", land_res.success)
    else:
        print ("landing failed!")
        sys.exit (-1)

