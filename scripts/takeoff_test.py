#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import math
import time

import sys

current_state = State ()
def state_cb (data):
    global current_state
    current_state = data

rospy.init_node ('takeoff_test')
rate = rospy.Rate (20)
id = rospy.get_param ("id", 0)
uav_name = "UAV" + str (id)

state_sub = rospy.Subscriber("mavros/state", State, state_cb)

print (uav_name, ": TAKEOFF TEST INTIALIZING...")
time.sleep (1)

while(not rospy.is_shutdown () and not current_state.connected):
    print (uav_name, ": waiting for the connection with FCU")
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

height = rospy.get_param ("takeoff_height", 3)

takeoff_client = rospy.ServiceProxy ("mavros/cmd/takeoff", CommandTOL)
takeoff_res = takeoff_client (altitude=height)

if takeoff_res.success:
    print (uav_name, ": takeoff sent", takeoff_res.success)
else:
    print (uav_name, ": failed takeoff!")
    sys.exit (-1)

print (uav_name, ": waiting for 10 seconds till landing")
time.sleep (10)

rospy.wait_for_service ('mavros/cmd/land')
print (uav_name, ": mavros/cmd/land service is active ..")

land_client = rospy.ServiceProxy ("mavros/cmd/land", CommandTOL)
land_res = land_client ()

if land_res.success:
    print (uav_name, ": land sent", land_res.success)
else:
    print (uav_name, ": landing failed!")
    sys.exit (-1)

print (uav_name, ": test completed successfully !!")
